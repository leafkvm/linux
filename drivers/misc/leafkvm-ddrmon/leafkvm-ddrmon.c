// SPDX-License-Identifier: GPL-2.0
/* RV1126B DDRMON v0x40 telemetry, independent of DMC/devfreq.
 * DDR3 BL8: each command transfers bus_width_bits bytes. Compare measured
 * bytes/s to clk_get_rate(SCLK_DDR) * 2 * bus_width_bits/8, NOT the vendor
 * driver's 4*access/count (the DFI counter runs at half the DRAM clock).
 * Calibrated against large DMA-heap transfers; see video-test-tools notes.
 */
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_clk.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <dt-bindings/clock/rockchip,rv1126b-cru.h>

#define CTRL0 0x04
#define WR_COUNT 0x28
#define RD_COUNT 0x2c
#define OS_REG2 0x30208
#define DDR3_MODE 0xffff8000
#define START 0x00020002
#define STOP 0x00020000
#define MAX_WINDOW_NS (3ULL * NSEC_PER_SEC)

static void __iomem *regs;
static struct resource region;
static struct clk *pclk, *ddrclk;
static u32 saved_ctrl, width;
static u64 started_ns, started_rate;
static DEFINE_MUTEX(lock);
static struct {
	u64 read_bytes, write_bytes, elapsed_ns, rate, at_ns;
} sample;
static void measure(struct work_struct *work);
static DECLARE_DELAYED_WORK(poll_work, measure);

static void begin(void)
{
	writel(DDR3_MODE, regs + CTRL0);
	started_rate = clk_get_rate(ddrclk);
	started_ns = ktime_get_ns();
	writel(START, regs + CTRL0);
	readl(regs + CTRL0); /* complete the APB write */
}

static void measure(struct work_struct *work)
{
	u64 now, elapsed, rate;

	mutex_lock(&lock);
	writel(STOP, regs + CTRL0);
	readl(regs + CTRL0);
	now = ktime_get_ns();
	elapsed = now - started_ns;
	rate = clk_get_rate(ddrclk);
	/* Discard suspend/starvation/clock-change windows. At <=1.2GHz these
	 * windows cannot overflow a 32-bit half-rate DFI timer/command count.
	 */
	if (elapsed && elapsed <= MAX_WINDOW_NS && rate == started_rate &&
	    rate && rate <= 1200000000UL) {
		sample.read_bytes = (u64)readl(regs + RD_COUNT) * width;
		sample.write_bytes = (u64)readl(regs + WR_COUNT) * width;
		sample.elapsed_ns = elapsed;
		sample.rate = rate;
		sample.at_ns = now;
	} else {
		sample.at_ns = 0;
	}
	begin();
	mutex_unlock(&lock);
	schedule_delayed_work(&poll_work, HZ);
}

/* One coherent, versioned ASCII snapshot. Reading never resets counters or
 * waits for a new window; multiple clients see the same completed sample.
 */
static int stats_show(struct seq_file *seq, void *unused)
{
	mutex_lock(&lock);
	if (!sample.at_ns || ktime_get_ns() - sample.at_ns > MAX_WINDOW_NS) {
		mutex_unlock(&lock);
		return -EAGAIN;
	}
	seq_printf(seq, "1 %llu %llu %llu %llu %u %llu\n",
		   sample.read_bytes, sample.write_bytes, sample.elapsed_ns,
		   sample.rate, width, sample.at_ns);
	mutex_unlock(&lock);
	return 0;
}

static int stats_open(struct inode *inode, struct file *file)
{
	/* misc_open supplies the miscdevice; seq_open owns this slot next. */
	file->private_data = NULL;
	return single_open(file, stats_show, NULL);
}

static const struct file_operations stats_fops = {
	.owner = THIS_MODULE,
	.open = stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static struct miscdevice stats_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "leafkvm-ddrmon",
	.fops = &stats_fops,
	.mode = 0444,
};

static int sleep_event(struct notifier_block *nb, unsigned long event, void *arg)
{
	if (event == PM_SUSPEND_PREPARE || event == PM_HIBERNATION_PREPARE ||
	    event == PM_RESTORE_PREPARE) {
		cancel_delayed_work_sync(&poll_work);
		mutex_lock(&lock);
		writel(0xffff0000 | saved_ctrl, regs + CTRL0);
		sample.at_ns = 0;
		mutex_unlock(&lock);
	} else if (event == PM_POST_SUSPEND || event == PM_POST_HIBERNATION ||
		   event == PM_POST_RESTORE) {
		mutex_lock(&lock);
		begin();
		mutex_unlock(&lock);
		schedule_delayed_work(&poll_work, HZ);
	}
	return NOTIFY_OK;
}
static struct notifier_block pm_events = { .notifier_call = sleep_event };

static struct clk *cru_clock(struct device_node *cru, unsigned int id)
{
	struct of_phandle_args spec = { .np = cru, .args_count = 1, .args = { id } };

	return of_clk_get_from_provider(&spec);
}

static int __init ddrmon_init(void)
{
	struct device_node *dfi, *dmc, *cru;
	struct regmap *grf;
	u32 config;
	int ret;

	dfi = of_find_compatible_node(NULL, NULL, "rockchip,rv1126b-dfi");
	if (!dfi)
		return -ENODEV;
	dmc = of_find_compatible_node(NULL, NULL, "rockchip,rv1126b-dmc");
	ret = of_device_is_available(dfi) || (dmc && of_device_is_available(dmc));
	of_node_put(dmc);
	if (ret) {
		ret = -EBUSY;
		goto put_dfi;
	}
	grf = syscon_regmap_lookup_by_phandle(dfi, "rockchip,pmugrf");
	if (IS_ERR(grf)) {
		ret = PTR_ERR(grf);
		goto put_dfi;
	}
	ret = regmap_read(grf, OS_REG2, &config);
	if (ret)
		goto put_dfi;
	/* Only calibrated DDR3, with a valid 8/16/32-bit bus width. Do not
	 * probe OS_REG3: that unused handoff register can bus-fault here.
	 */
	if (((config >> 13) & 7) != 3 || ((config >> 2) & 3) == 3) {
		ret = -EOPNOTSUPP;
		goto put_dfi;
	}
	width = 32 >> ((config >> 2) & 3);
	ret = of_address_to_resource(dfi, 0, &region);
	if (ret)
		goto put_dfi;
	if (!request_mem_region(region.start, resource_size(&region), "leafkvm-ddrmon")) {
		ret = -EBUSY;
		goto put_dfi;
	}
	regs = ioremap(region.start, resource_size(&region));
	if (!regs) {
		ret = -ENOMEM;
		goto release_region;
	}
	cru = of_find_compatible_node(NULL, NULL, "rockchip,rv1126b-cru");
	if (!cru) {
		ret = -ENODEV;
		goto unmap;
	}
	pclk = cru_clock(cru, PCLK_DDRMON);
	ddrclk = cru_clock(cru, SCLK_DDR);
	of_node_put(cru);
	if (IS_ERR(pclk) || IS_ERR(ddrclk)) {
		ret = IS_ERR(pclk) ? PTR_ERR(pclk) : PTR_ERR(ddrclk);
		goto put_clocks;
	}
	ret = clk_prepare_enable(pclk);
	if (ret)
		goto put_clocks;
	if (readl(regs) != 0x40 || !clk_get_rate(ddrclk) ||
	    clk_get_rate(ddrclk) > 1200000000UL) {
		ret = -EOPNOTSUPP;
		goto disable_clock;
	}
	saved_ctrl = readl(regs + CTRL0) & 0xffff;
	if (saved_ctrl & 3) {
		ret = -EBUSY;
		goto disable_clock;
	}
	begin();
	ret = register_pm_notifier(&pm_events);
	if (ret)
		goto restore_ctrl;
	ret = misc_register(&stats_device);
	if (ret)
		goto unregister_pm;
	schedule_delayed_work(&poll_work, HZ);
	of_node_put(dfi);
	pr_info("leafkvm-ddrmon: DDR3 x%u, read-only telemetry, no DVFS\n", width);
	return 0;

unregister_pm:
	unregister_pm_notifier(&pm_events);
restore_ctrl:
	writel(0xffff0000 | saved_ctrl, regs + CTRL0);
disable_clock:
	clk_disable_unprepare(pclk);
put_clocks:
	if (!IS_ERR(pclk))
		clk_put(pclk);
	if (!IS_ERR(ddrclk))
		clk_put(ddrclk);
unmap:
	iounmap(regs);
release_region:
	release_mem_region(region.start, resource_size(&region));
put_dfi:
	of_node_put(dfi);
	return ret;
}

static void __exit ddrmon_exit(void)
{
	misc_deregister(&stats_device);
	unregister_pm_notifier(&pm_events);
	cancel_delayed_work_sync(&poll_work);
	writel(0xffff0000 | saved_ctrl, regs + CTRL0);
	clk_disable_unprepare(pclk);
	clk_put(pclk);
	clk_put(ddrclk);
	iounmap(regs);
	release_mem_region(region.start, resource_size(&region));
}
module_init(ddrmon_init);
module_exit(ddrmon_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Read-only RV1126B DDR3 bandwidth telemetry without DMC/DVFS");
