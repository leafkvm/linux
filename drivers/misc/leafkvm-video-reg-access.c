// SPDX-License-Identifier: GPL-2.0
/*
 * LeafKVM I2C register access driver
 *
 * Exposes read_reg / write_reg via ioctl on a /dev/<chip> misc device.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/of.h>

#define VIDEO_CHIP_IOC_MAGIC		'L'
#define VIDEO_CHIP_IOC_READ_REG		_IOWR(VIDEO_CHIP_IOC_MAGIC, 0, struct video_chip_reg_op)
#define VIDEO_CHIP_IOC_WRITE_REG	_IOW(VIDEO_CHIP_IOC_MAGIC, 1, struct video_chip_reg_op)

struct video_chip_reg_op {
	__u16 reg;
	__u8  val;
	__u8  _pad;
};

struct video_chip_variant;

struct video_chip_data {
	struct i2c_client *client;
	struct gpio_desc *reset_gpio;
	struct clk *clk;
	struct miscdevice misc;
	struct mutex lock;
	const struct video_chip_variant *variant;
	char name[32];			/* misc device name, e.g. "leafkvm-video1605-1-0048" */
	bool in_use;
	u8 current_page;
};

/* Low-level I2C helpers -------------------------------------------------- */

static int video_chip_write_reg_direct(struct video_chip_data *chip, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	struct i2c_msg msg = {
		.addr  = chip->client->addr,
		.flags = 0,
		.len   = 2,
		.buf   = buf,
	};

	return i2c_transfer(chip->client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int video_chip_read_reg_direct(struct video_chip_data *chip, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2] = {
		{
			.addr  = chip->client->addr,
			.flags = 0,
			.len   = 1,
			.buf   = &reg,
		},
		{
			.addr  = chip->client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = val,
		},
	};

	return i2c_transfer(chip->client->adapter, msgs, 2) == 2 ? 0 : -EIO;
}

static int video_chip_ensure_page(struct video_chip_data *chip, u8 page)
{
	int ret;

	if (chip->current_page != page) {
		ret = video_chip_write_reg_direct(chip, 0xFF, page);
		if (ret)
			return ret;
		chip->current_page = page;
	}
	return 0;
}

static int video_chip_write_reg(struct video_chip_data *chip, u16 reg, u8 val)
{
	int ret;
	u8 page     = reg >> 8;
	u8 reg_addr = reg & 0xFF;

	ret = video_chip_ensure_page(chip, page);
	if (ret)
		return ret;

	return video_chip_write_reg_direct(chip, reg_addr, val);
}

static int video_chip_read_reg(struct video_chip_data *chip, u16 reg, u8 *val)
{
	int ret;
	u8 page     = reg >> 8;
	u8 reg_addr = reg & 0xFF;

	ret = video_chip_ensure_page(chip, page);
	if (ret)
		return ret;

	return video_chip_read_reg_direct(chip, reg_addr, val);
}

/* Reset ------------------------------------------------------------------ */

static void video_chip_hw_reset(struct video_chip_data *chip)
{
	gpiod_set_value_cansleep(chip->reset_gpio, 1);   /* assert reset (active-low) */
	msleep(10);
	gpiod_set_value_cansleep(chip->reset_gpio, 0);   /* de-assert */
	msleep(100);

	chip->current_page = 0;
}

/* Variant detection ------------------------------------------------------ */

/* Clock rate used while the chip variant is still being decided. */
#define VIDEO_CHIP_DETECT_CLK_RATE	24000000

struct video_chip_variant {
	u16 enable_reg;
	u16 id_reg;
	u16 expected_id;
	unsigned long clk_rate;
};

static const struct video_chip_variant video_chip_variants[] = {
	{ 0x80EE, 0xA000, 0x1605, 27000000 },
	{ 0xE0EE, 0xE100, 0x2102, 24000000 },
	{ 0xE0EE, 0xE100, 0x2003, 0 },
};

/*
 * Probe a single variant: reset the chip, enable register access at the
 * variant-specific location, then read the chip-ID pair. The reset is
 * required because a previous variant's probe sequence may have left the
 * chip in an inconsistent state.
 */
static int video_chip_try_variant(struct video_chip_data *chip,
				   const struct video_chip_variant *v,
				   bool *matched)
{
	int ret;
	u8 id_hi, id_lo;
	u16 chip_id;

	*matched = false;
	video_chip_hw_reset(chip);

	ret = video_chip_write_reg(chip, v->enable_reg, 0x01);
	if (ret)
		return ret;

	ret = video_chip_read_reg(chip, v->id_reg, &id_hi);
	if (ret)
		return ret;
	ret = video_chip_read_reg(chip, v->id_reg + 1, &id_lo);
	if (ret)
		return ret;

	chip_id = ((u16)id_hi << 8) | id_lo;
	dev_dbg(&chip->client->dev, "probing chip ID 0x%04X (expected 0x%04X)\n",
		chip_id, v->expected_id);

	if (chip_id == v->expected_id)
		*matched = true;
	return 0;
}

/* File operations -------------------------------------------------------- */

static struct video_chip_data *file_to_chip(struct file *f)
{
	return container_of(f->private_data, struct video_chip_data, misc);
}

/*
 * The chip is held in reset whenever the device node is not open. Opening it
 * releases reset and enables register access; only a single opener is allowed
 * at a time. Closing it puts the chip back into reset.
 */
static int video_chip_open(struct inode *inode, struct file *f)
{
	struct video_chip_data *chip = file_to_chip(f);
	int ret;

	mutex_lock(&chip->lock);

	if (chip->in_use) {
		mutex_unlock(&chip->lock);
		return -EBUSY;
	}

	video_chip_hw_reset(chip);		/* releases reset, chip starts running */

	ret = video_chip_write_reg(chip, chip->variant->enable_reg, 0x01);
	if (ret) {
		dev_err(&chip->client->dev,
			"failed to enable register access: %d\n", ret);
		gpiod_set_value_cansleep(chip->reset_gpio, 1);	/* hold reset */
		mutex_unlock(&chip->lock);
		return ret;
	}

	chip->in_use = true;
	mutex_unlock(&chip->lock);
	return 0;
}

static int video_chip_release(struct inode *inode, struct file *f)
{
	struct video_chip_data *chip = file_to_chip(f);

	mutex_lock(&chip->lock);
	gpiod_set_value_cansleep(chip->reset_gpio, 1);	/* hold chip in reset */
	chip->in_use = false;
	mutex_unlock(&chip->lock);
	return 0;
}

static long video_chip_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct video_chip_data *chip = file_to_chip(f);
	struct video_chip_reg_op op;
	int ret;

	if (copy_from_user(&op, (void __user *)arg, sizeof(op)))
		return -EFAULT;

	mutex_lock(&chip->lock);

	switch (cmd) {
	case VIDEO_CHIP_IOC_READ_REG:
		ret = video_chip_read_reg(chip, op.reg, &op.val);
		if (ret == 0) {
			if (copy_to_user((void __user *)arg, &op, sizeof(op)))
				ret = -EFAULT;
		}
		break;

	case VIDEO_CHIP_IOC_WRITE_REG:
		ret = video_chip_write_reg(chip, op.reg, op.val);
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&chip->lock);
	return ret;
}

static const struct file_operations video_chip_fops = {
	.owner          = THIS_MODULE,
	.open           = video_chip_open,
	.release        = video_chip_release,
	.unlocked_ioctl = video_chip_ioctl,
	.compat_ioctl   = compat_ptr_ioctl,
};

/* I2C probe / remove ----------------------------------------------------- */

static int video_chip_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct video_chip_data *chip;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	chip->reset_gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->reset_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(chip->reset_gpio),
				     "failed to get reset GPIO\n");

	/*
	 * The clock is optional. If the device tree supplies one, the chip needs
	 * it running before reset is released. The exact operating frequency is
	 * variant-specific, so start at the detection rate and switch to the
	 * variant's rate once it has been identified.
	 * devm_clk_get_optional_enabled() returns NULL (not an error) when no
	 * clock is specified, and disables the clock again on probe failure and
	 * on device removal.
	 */
	chip->clk = devm_clk_get_optional_enabled(&client->dev, NULL);
	if (IS_ERR(chip->clk))
		return dev_err_probe(&client->dev, PTR_ERR(chip->clk),
				     "failed to get clock\n");

	if (chip->clk) {
		ret = clk_set_rate(chip->clk, VIDEO_CHIP_DETECT_CLK_RATE);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "failed to set detection clock rate\n");
	}

	const struct video_chip_variant *detected = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(video_chip_variants); i++) {
		bool matched;

		ret = video_chip_try_variant(chip, &video_chip_variants[i], &matched);
		if (ret) {
			dev_err(&client->dev,
				"I2C error while probing chip ID 0x%04X: %d\n",
				video_chip_variants[i].expected_id, ret);
			return ret;
		}
		if (matched) {
			detected = &video_chip_variants[i];
			break;
		}
	}

	if (!detected) {
		dev_err(&client->dev, "no supported video chip variant detected\n");
		return -ENODEV;
	}

	chip->variant = detected;

	/*
	 * Apply the detected variant's clock requirement. A clk_rate of 0 means
	 * the variant must run without a clock, so supplying one in the device
	 * tree is a configuration error. Otherwise switch to the variant's
	 * operating rate if a clock was supplied. The chip is left in reset from
	 * here on; it is only taken out of reset (and register access enabled)
	 * when the device node is opened.
	 */
	gpiod_set_value_cansleep(chip->reset_gpio, 1);	/* hold chip in reset */

	if (detected->clk_rate == 0) {
		if (chip->clk)
			return dev_err_probe(&client->dev, -EINVAL,
					     "chip ID 0x%04X expects no clock but one was supplied\n",
					     detected->expected_id);
	} else if (chip->clk && detected->clk_rate != VIDEO_CHIP_DETECT_CLK_RATE) {
		ret = clk_set_rate(chip->clk, detected->clk_rate);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "failed to set clock rate for chip ID 0x%04X\n",
					     detected->expected_id);
	}

	/*
	 * Include the I2C bus/address (dev_name() is "<bus>-<addr>") so the node
	 * name is unique per instance; otherwise two chips of the same variant
	 * would compute identical names and the second misc_register() would fail.
	 */
	snprintf(chip->name, sizeof(chip->name), "leafkvm-%s-%04X",
		dev_name(&client->dev), detected->expected_id);

	chip->misc.minor  = MISC_DYNAMIC_MINOR;
	chip->misc.name   = chip->name;
	chip->misc.fops   = &video_chip_fops;
	chip->misc.parent = &client->dev;

	ret = misc_register(&chip->misc);
	if (ret) {
		dev_err(&client->dev, "misc_register failed: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "detected chip ID 0x%04X, registered as /dev/%s\n",
		 detected->expected_id, chip->misc.name);
	return 0;
}

static void video_chip_remove(struct i2c_client *client)
{
	struct video_chip_data *chip = i2c_get_clientdata(client);

	misc_deregister(&chip->misc);
	gpiod_set_value_cansleep(chip->reset_gpio, 1);   /* hold reset */
}

/* Device tree / module glue ---------------------------------------------- */

static const struct of_device_id video_chip_of_match[] = {
	{ .compatible = "leafkvm,video-reg-access" },
	{ }
};
MODULE_DEVICE_TABLE(of, video_chip_of_match);

static const struct i2c_device_id video_chip_id[] = {
	{ "leafkvm-video-reg", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, video_chip_id);

static struct i2c_driver video_chip_driver = {
	.driver = {
		.name           = "leafkvm-video-reg-access",
		.of_match_table = video_chip_of_match,
	},
	.probe    = video_chip_probe,
	.remove   = video_chip_remove,
	.id_table = video_chip_id,
};
module_i2c_driver(video_chip_driver);

MODULE_AUTHOR("LeafKVM");
MODULE_DESCRIPTION("LeafKVM video register access driver");
MODULE_LICENSE("GPL");
