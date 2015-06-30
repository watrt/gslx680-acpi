/*
 * Silead GSL1680/3680 touchscreen driver
 *
 * Copyright (c) 2015 Gregor Riepl <onitake@gmail.com>
 *
 * This driver is based on gslx680.c and elan_ts.c
 * Copyright (c) 2012 Shanghai Basewin
 *   Guan Yuwei<guanyuwei@basewin.com>
 * Copyright (c) 2013 Joe Burmeister
 *   Joe Burmeister<joe.a.burmeister@googlemail.com>
 * Copyright (C) 2014 Elan Microelectronics Corporation.
 * Scott Liu <scott.liu@emc.com.tw>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/input/mt.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <asm/unaligned.h>
#include <acpi/acpi_bus.h>

/* Device, Driver information */
#define DEVICE_NAME	"gslx680"
#define DRV_VERSION	"1.0.0"

/* Driver state structure */
struct gsl_ts_data {
	struct i2c_client *client;
	struct input_dev *input;

	bool wake_irq_enabled;

	unsigned int x_res;
	unsigned int y_res;
	unsigned int x_max;
	unsigned int y_max;
	unsigned int multi_touches;
};

static int gsl_ts_init(struct gsl_ts_data *ts)
{
	dev_warn(&ts->client->dev, "%s: initializing device\n", __func__);
	
	/* TODO these are almost certainly wrong,
	 * read them from the hardware config or data sheet
	 * or make something up
	 */
	ts->x_max = 1024;
	ts->y_max = 600;
	ts->x_res = 1024 / 155;
	ts->y_res = 600 / 86;
	ts->multi_touches = 8;

    return 0;
}

static irqreturn_t gsl_ts_irq(int irq, void *dev)
{
	struct gsl_ts_data *ts = (struct gsl_ts_data *) dev;
	struct i2c_client *client = ts->client;
	
	dev_warn(&client->dev, "%s: IRQ received\n", __func__);

	return IRQ_HANDLED;
}

static int gsl_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gsl_ts_data *ts;
	struct acpi_device *ac;
	struct device_node *of;
	unsigned long irqflags;
	int error;
	
	dev_warn(&client->dev, "%s: got a device named %s at address 0x%x, IRQ %d, flags 0x%x\n", __func__, client->name, client->addr, client->irq, client->flags);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c check functionality error\n", __func__);
		return -ENXIO;
	}
	
	ac = ACPI_COMPANION(&client->dev);
	if (ac) {
		/* Set up ACPI device descriptor GPIO name mappings, needs a corresponding acpi_dev_remove_driver_gpios(ac) */
		/*acpi_dev_add_driver_gpios(ac, gsl_ts_acpi_gpios);*/
		
		/* Extract the GPIO configuration from the device descriptor */
		
		/* Wake the device up with a power on reset */
		error = acpi_bus_set_power(ACPI_HANDLE(&client->dev), ACPI_STATE_D0);
		if (error) {
			dev_err(&client->dev, "%s: failed to wake up device through ACPI: %d\n", __func__, error);
			return error;
		}
	}
	
	of = of_node_get(client->dev.of_node);
	if (of) {
		
	}

	ts = devm_kzalloc(&client->dev, sizeof(struct gsl_ts_data), GFP_KERNEL);
	if (!ts) {
		return -ENOMEM;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	error = gsl_ts_init(ts);
	if (error) {
		dev_err(&client->dev, "%s: failed to initialize: %d\n", __func__, error);
		return error;
	}

	ts->input = devm_input_allocate_device(&client->dev);
	if (!ts->input) {
		dev_err(&client->dev, "%s: failed to allocate input device\n", __func__);
		return -ENOMEM;
	}

	ts->input->name = "Silead GSLx680 Touchscreen";
	ts->input->id.bustype = BUS_I2C;

	__set_bit(BTN_TOUCH, ts->input->keybit);
	__set_bit(EV_ABS, ts->input->evbit);
	__set_bit(EV_KEY, ts->input->evbit);

	/* Single touch input params setup */
	input_set_abs_params(ts->input, ABS_X, 0, ts->x_max, 0, 0);
	input_set_abs_params(ts->input, ABS_Y, 0, ts->y_max, 0, 0);
	input_set_abs_params(ts->input, ABS_PRESSURE, 0, 255, 0, 0);
	input_abs_set_res(ts->input, ABS_X, ts->x_res);
	input_abs_set_res(ts->input, ABS_Y, ts->y_res);

	/* Multitouch input params setup */
	error = input_mt_init_slots(ts->input, ts->multi_touches, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error) {
		dev_err(&client->dev, "%s: failed to initialize MT slots: %d\n", __func__, error);
		return error;
	}

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_abs_set_res(ts->input, ABS_MT_POSITION_X, ts->x_res);
	input_abs_set_res(ts->input, ABS_MT_POSITION_Y, ts->y_res);

	input_set_drvdata(ts->input, ts);

	error = input_register_device(ts->input);
	if (error) {
		dev_err(&client->dev, "%s: unable to register input device: %d\n", __func__, error);
		return error;
	}

	/*
	 * Systems using device tree should set up interrupt via DTS,
	 * the rest will use the default falling edge interrupts.
	 */
	irqflags = client->dev.of_node ? 0 : IRQF_TRIGGER_FALLING;

	error = devm_request_threaded_irq(
		&client->dev,
		client->irq,
		NULL,
		gsl_ts_irq,
		irqflags | IRQF_ONESHOT,
		client->name,
		ts
	);
	if (error) {
		dev_err(&client->dev, "%s: failed to register interrupt\n", __func__);
		return error;
	}

	/*
	 * Systems using device tree should set up wakeup via DTS,
	 * the rest will configure device as wakeup source by default.
	 */
	if (!client->dev.of_node) {
		device_init_wakeup(&client->dev, true);
	}

	return 0;
}

static int __maybe_unused gsl_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gsl_ts_data *ts = i2c_get_clientdata(client);
	
	disable_irq(client->irq);

	/*int error = gsl_ts_send_sleep_command();
	if (error != 0) {
		dev_err(&client->dev, "%s: suspend command failed: %d\n", __func__, error);
	}*/

	if (device_may_wakeup(dev)) {
		ts->wake_irq_enabled = (enable_irq_wake(client->irq) == 0);
	}

	return 0;
}

static int __maybe_unused gsl_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gsl_ts_data *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(dev) && ts->wake_irq_enabled) {
		disable_irq_wake(client->irq);
	}

	/*int error = gsl_ts_send_wake_command();
	if (error != 0) {
		dev_err(&client->dev, "%s: resume command failed: %d\n", __func__, error);
	}*/

	enable_irq(client->irq);

	return 0;
}

static const struct i2c_device_id gsl_ts_i2c_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gsl_ts_i2c_id);

#ifdef CONFIG_PM
#warning Power management enabled
static SIMPLE_DEV_PM_OPS(gsl_ts_pm_ops, gsl_ts_suspend, gsl_ts_resume);
#endif

#ifdef CONFIG_ACPI
#warning ACPI platform
/* GSL3680 ACPI IDs are untested */
static const struct acpi_device_id gsl_ts_acpi_match[] = {
	{ "MSSL1680", 0 },
	{ "MSSL3680", 0 },
	{ "PNP1680", 0 },
	{ "PNP3680", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, gsl_ts_acpi_match);
#endif

#ifdef CONFIG_OF
#warning OpenFirmware/DeviceTree platform
/* This should take care of OpenFirmware and DeviceTree instantiations,
 * but they're completely untested. Volunteers welcome.
 * Is anyone using DeviceTree with this touch screen at all?
 */
static const struct of_device_id gsl_ts_of_match[] = {
	{ .compatible = "silead,gsl1680" },
	{ .compatible = "silead,gsl3680" },
	{ .compatible = "silead,gslx680" },
	{ }
};
MODULE_DEVICE_TABLE(of, gsl_ts_of_match);
#endif

static struct i2c_driver gslx680_ts_driver = {
	.probe      = gsl_ts_probe,
	.id_table   = gsl_ts_i2c_id,
	.driver = {
		.name   = DEVICE_NAME,
		.owner  = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &gsl_ts_pm_ops,
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(gsl_ts_acpi_match),
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(gsl_ts_of_match),
#endif
	},
};
module_i2c_driver(gslx680_ts_driver);

MODULE_DESCRIPTION("GSLX680 touchscreen controller driver");
MODULE_AUTHOR("Gregor Riepl <onitake@gmail.com>");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
