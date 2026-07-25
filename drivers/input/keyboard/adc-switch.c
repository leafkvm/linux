// SPDX-License-Identifier: GPL-2.0-only
/*
 * Input driver for a switch detected via ADC voltage threshold
 *
 * Reports EV_SW events based on whether the ADC reading exceeds
 * a configurable voltage threshold. Designed to expose the same
 * UAPI as gpio-keys does for switches.
 */

#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

struct adc_switch_state {
	struct iio_channel *channel;
	u32 threshold_mv;
	u32 code;
	int last_state;
};

static void adc_switch_poll(struct input_dev *input)
{
	struct adc_switch_state *st = input_get_drvdata(input);
	int value, ret, state;

	ret = iio_read_channel_processed(st->channel, &value);
	if (unlikely(ret < 0))
		return;

	state = value > st->threshold_mv ? 1 : 0;

	if (state != st->last_state) {
		input_report_switch(input, st->code, state);
		input_sync(input);
		st->last_state = state;
	}
}

static int adc_switch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adc_switch_state *st;
	struct input_dev *input;
	enum iio_chan_type type;
	u32 threshold_uv, poll_interval;
	int error;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->channel = devm_iio_channel_get(dev, "switch");
	if (IS_ERR(st->channel))
		return PTR_ERR(st->channel);

	if (!st->channel->indio_dev)
		return -ENXIO;

	error = iio_get_channel_type(st->channel, &type);
	if (error < 0)
		return error;

	if (type != IIO_VOLTAGE) {
		dev_err(dev, "Incompatible channel type %d\n", type);
		return -EINVAL;
	}

	if (device_property_read_u32(dev, "threshold-microvolt",
				     &threshold_uv)) {
		dev_err(dev, "Missing threshold-microvolt\n");
		return -EINVAL;
	}
	st->threshold_mv = threshold_uv / 1000;

	if (device_property_read_u32(dev, "linux,code", &st->code)) {
		dev_err(dev, "Missing linux,code\n");
		return -EINVAL;
	}

	st->last_state = -1;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input_set_drvdata(input, st);

	input->name = pdev->name;
	input->phys = "adc-switch/input0";

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input_set_capability(input, EV_SW, st->code);

	error = input_setup_polling(input, adc_switch_poll);
	if (error) {
		dev_err(dev, "Unable to set up polling: %d\n", error);
		return error;
	}

	if (!device_property_read_u32(dev, "poll-interval", &poll_interval))
		input_set_poll_interval(input, poll_interval);

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device: %d\n", error);
		return error;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id adc_switch_of_match[] = {
	{ .compatible = "adc-switch", },
	{ }
};
MODULE_DEVICE_TABLE(of, adc_switch_of_match);
#endif

static struct platform_driver adc_switch_driver = {
	.driver = {
		.name = "adc_switch",
		.of_match_table = of_match_ptr(adc_switch_of_match),
	},
	.probe = adc_switch_probe,
};
module_platform_driver(adc_switch_driver);

MODULE_DESCRIPTION("Input driver for ADC-based switch detection");
MODULE_LICENSE("GPL v2");
