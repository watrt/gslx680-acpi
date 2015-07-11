/*
 * Silead GSL1680/3680 touchscreen driver
 *
 * Copyright (c) 2015 Gregor Riepl <onitake@gmail.com>
 *
 * This driver is based on gslx680_ts.c and elan_ts.c
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
#include <linux/byteorder/generic.h>
#include <asm/unaligned.h>

/* Device and driver information */
#define DEVICE_NAME	"gslx680"
#define DRV_VERSION	"1.0.0"
#define FIRMWARE_1680 "gsl1680.fw"

/* Hardware API constants */
#define GSL_DATA_REG 0x80
#define GSL_STATUS_REG 0xe0
#define GSL_PAGE_REG 0xf0
#define GSL_TOUCH_STATUS_REG 0xbc
#define GSL_PAGE_SIZE 128
/* Why 126? */
#define GSL_MAX_READ 126
/* Actually 129: 1 command byte and 128 data bytes
 * this was 125 originally - using 128 to allow firmware transfers
 */
#define GSL_MAX_WRITE 128
#define GSL_STATUS_FW 0x80
#define GSL_STATUS_TOUCH 0x00

#define GSL_MAX_CONTACTS 10
#define GSL_PRESSURE 10
#define GSL_FINGER_SIZE 70

#define GSL_PACKET_SIZE (GSL_MAX_CONTACTS * sizeof(struct gsl_ts_packet_touch) + sizeof(struct gsl_ts_packet_header))

#define PRESS_MAX 255
#define MAX_FINGERS 10
#define MAX_CONTACTS 10

#define TOUCH_INDEX 0
#define X_INDEX 6
#define Y_INDEX 4
#define Z_INDEX 5
#define ID_INDEX 7
#define UPDATE_DATA 0x4
#define TOUCH_BYTES 4
#define TOUCH_META_BYTES 4
#define FINGER_SIZE 70

/* Driver state */
enum gsl_ts_state {
	GSL_TS_INIT,
	GSL_TS_SHUTDOWN,
	GSL_TS_GREEN,
};

/* Driver instance data structure */
struct gsl_ts_data {
	struct i2c_client *client;
	struct input_dev *input;

	enum gsl_ts_state state;
	
	bool wake_irq_enabled;

	const char *firmware_name;
	
	unsigned int x_res;
	unsigned int y_res;
	unsigned int x_max;
	unsigned int y_max;
	unsigned int multi_touches;
	bool x_reversed;
	bool y_reversed;
	bool xy_swapped;
	
	u32 id_sign[MAX_CONTACTS + 1];
	u8 id_state_flag[MAX_CONTACTS + 1];
	u8 id_state_old_flag[MAX_CONTACTS + 1];
	u16 x_old[MAX_CONTACTS + 1];
	u16 y_old[MAX_CONTACTS + 1];
	u16 x_new;
	u16 y_new;
	u8 prev_touches;
};

/* Firmware image data chunk */
struct gsl_ts_fw_chunk {
	u32 address;
	u8 data[4];
} __attribute__((packed));

/* TODO use get_unaligned_le16 instead of packed structures */
/* Hardware touch event data header */
struct gsl_ts_packet_header {
	u8 num_fingers;
	u8 reserved;
	u16 time_stamp; /* little-endian */
} __attribute__((packed));
/* Hardware touch event data per finger */
struct gsl_ts_packet_touch {
	u16 y_id; /* little endian, lower 12 bits = y, upper 4 bits = id */
	u16 x; /* little endian */
} __attribute__((packed));


static int gsl_ts_init(struct gsl_ts_data *ts)
{
	dev_dbg(&ts->client->dev, "%s: initialising driver state\n", __func__);
	
	ts->wake_irq_enabled = false;
	ts->state = GSL_TS_INIT;
	
	/* TODO these are almost certainly wrong,
	 * read them from the hardware config or data sheet
	 * or make something up
	 */
	/* experimental range: x = 16..947 y = 13..631 id = 0 or 4 */
	/* theoretical range: x = 0..65535, y = 0..4096 */
	ts->x_max = 1024;
	ts->y_max = 768;
	/* for a 7" device, points/mm */
	ts->x_res = ts->x_max / 155;
	ts->y_res = ts->y_max / 86;
	ts->multi_touches = GSL_MAX_CONTACTS;
	
	/* TODO: Find out what kind of device we have and load the appropriate firmware */
	ts->firmware_name = FIRMWARE_1680;
	
	memset(ts->id_sign, 0, sizeof(ts->id_sign));
	memset(ts->id_state_flag, 0, sizeof(ts->id_state_flag));
	memset(ts->id_state_old_flag, 0, sizeof(ts->id_state_old_flag));
	memset(ts->x_old, 0, sizeof(ts->x_old));
	memset(ts->y_old, 0, sizeof(ts->y_old));
	ts->x_new = 0;
	ts->y_new = 0;
	ts->x_reversed = false;
	ts->y_reversed = false;
	ts->xy_swapped = false;

	ts->prev_touches = 0;

	return 0;
}

static int gsl_ts_write(struct i2c_client *client, u8 reg, u8 *pdata, int datalen)
{
	u8 buf[GSL_PAGE_SIZE + 1];
	unsigned int bytelen = 0;
	
	if (datalen > GSL_MAX_WRITE) {
		dev_err(&client->dev, "%s: data transfer too large (%d > %d)\n", __func__, datalen, GSL_MAX_WRITE);
		return -EINVAL;
	}
	
	buf[0] = reg;
	bytelen++;
	
	if (datalen != 0 && pdata != NULL) {
		memcpy(&buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}
	
	return i2c_master_send(client, buf, bytelen);
}

static int gsl_ts_read(struct i2c_client *client, u8 reg, u8 *pdata, unsigned int datalen)
{
	int ret = 0;
	
	if (datalen > GSL_MAX_READ) {
		dev_err(&client->dev, "%s: data transfer too large (%d > %d)\n", __func__, datalen, GSL_MAX_READ);
		return -EINVAL;
	}
	
	ret = gsl_ts_write(client, reg, NULL, 0);
	if (ret < 0) {
		dev_err(&client->dev, "%s: sending register location failed\n", __func__);
		return ret;
	}
	
	return i2c_master_recv(client, pdata, datalen);
}

static int gsl_ts_startup_chip(struct i2c_client *client)
{
	int rc;
	u8 tmp = 0x00;
	dev_dbg(&client->dev, "%s: starting up\n", __func__);
	rc = gsl_ts_write(client, 0xe0, &tmp, 1);
	msleep(10);
	return rc;
}

static int gsl_ts_reset_chip(struct i2c_client *client)
{
	int rc;
	u8 arg[4] = { 0x00, 0x00, 0x00, 0x00 };

	dev_dbg(&client->dev, "%s: resetting\n", __func__);

	arg[0] = 0x88;
	rc = gsl_ts_write(client, 0xe0, arg, 1);
	if (rc < 0) {
		dev_err(&client->dev, "%s: gsl_ts_write 1 fail!\n", __func__);
		return rc;
	}
	msleep(10);
	
	arg[0] = 0x04;
	rc = gsl_ts_write(client, 0xe4, arg, 1);
	if (rc < 0) {
		dev_err(&client->dev, "%s: gsl_ts_write 2 fail!\n", __func__);
		return rc;
	}
	msleep(10);
	
	arg[0] = 0x00;
	rc = gsl_ts_write(client, 0xbc, arg, 4);
	if (rc < 0) {
		dev_err(&client->dev, "%s: gsl_ts_write 3 fail!\n", __func__);
		return rc;
	}
	msleep(10);
	
	return 0;
}

static int gsl_ts_write_fw(struct gsl_ts_data *ts)
{
	int rc = 0;
	struct i2c_client *client = ts->client;
	const struct firmware *fw;
	size_t index;
	struct gsl_ts_fw_chunk chunk;
	u8 page[GSL_PAGE_SIZE];
	size_t buffered = 0;
	u8 offset = 0;
	
	memset(page, 0, sizeof(page));
	
	dev_dbg(&client->dev, "%s: sending firmware\n", __func__);

	rc = request_firmware(&fw, ts->firmware_name, &client->dev);
	if (rc) {
		dev_err(&client->dev, "%s: failed to load %s, %d.\n", __func__, ts->firmware_name, rc);
		return -EINVAL;
	}
	
	/* Each data transfer chunk is 8 bytes - make sure we don't overread */
	for (index = 0; rc >= 0 && index + sizeof(chunk) <= fw->size; index += sizeof(chunk)) {
		/* Copy the next chunk of data from the firmware image */
		memcpy(&chunk, &fw->data[index], sizeof(chunk));
		
		/* Check if the address offset is the page register */
		if (chunk.address == GSL_PAGE_REG) {
			/* Copy and transfer the data, this sets the current memory page */
			memcpy(page, &chunk.data, sizeof(chunk.data));
			rc = gsl_ts_write(client, GSL_PAGE_REG, page, sizeof(chunk.data));
			if (rc < 0) {
				dev_err(&client->dev, "%s: failed to set page register near offset 0x%zx\n", __func__, index);
			}
			
			/* Reset the data, size and offset */
			memset(page, 0, sizeof(page));
			buffered = 0;
			offset = 0;
		} else {
			/* Check if the address is valid */
			if (chunk.address <= GSL_PAGE_SIZE - sizeof(chunk.data)) {
				/* Verify that the data is consecutive.
				 * It may start at an offset > 0, but holes are not allowed.
				 */
				if (buffered == 0) {
					/* The first chunk may have any offset inside the page boundaries */
					offset = (u8) chunk.address;
				} else {
					/* Check for data consecutivity */
					if (offset + buffered != chunk.address) {
						dev_err(&client->dev, "%s: invalid firmware data, hole detected at offset 0x%zx\n", __func__, index);
						rc = -EINVAL;
					}
				}
				
				if (rc >= 0) {
					/* Store the data chunk into the page buffer.
					 * Note: It seems to be byte-swapped, but should be
					 * transfered to hardware as-is. 
					 */
					memcpy(&page[buffered], &chunk.data, sizeof(chunk.data));
					
					/* Update the data size */
					buffered += sizeof(chunk.data);
					
					/* Check if have a full page yet */
					if (buffered + offset >= GSL_PAGE_SIZE) {
						/* Transfer one page */
						rc = gsl_ts_write(client, offset, page, buffered);
						if (rc < 0) {
							dev_err(&client->dev, "%s: failed to transfer firmware page near offset 0x%zx\n", __func__, index);
						}
						
						/* Reset the data, size and offset */
						memset(page, 0, sizeof(page));
						buffered = 0;
						offset = 0;
					}
				}
			} else {
				dev_err(&client->dev, "%s: invalid firmware data, page address 0x%x out of bounds found at offset 0x%zx\n", __func__, chunk.address, index);
				rc = -EINVAL;
			}
		}
	}
	
	release_firmware(fw);
	
	if (rc < 0) {
		return rc;
	}
	return 0;
}

static int gsl_ts_init_chip(struct gsl_ts_data *ts)
{
	int rc;
	struct i2c_client *client = ts->client;

	dev_dbg(&client->dev, "%s: initialising\n", __func__);

	rc = gsl_ts_reset_chip(client);
	if (rc < 0) {
		dev_err(&client->dev, "%s: chip reset failed: %d\n", __func__, rc);
		return rc;
	}
	rc = gsl_ts_write_fw(ts);
	if (rc < 0) {
		dev_err(&client->dev, "%s: firmware transfer failed: %d\n", __func__, rc);
		return rc;
	}  
	rc = gsl_ts_startup_chip(client);
	if (rc < 0) {
		dev_err(&client->dev, "%s: chip startup failed: %d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static void gsl_ts_record_point(struct gsl_ts_data *ts, u16 x, u16 y, u8 id)
{
	u16 x_err = 0;
	u16 y_err = 0;

	dev_warn(&ts->client->dev, "%s: recording relative point coordinates\n", __func__);
	
	ts->id_sign[id] = ts->id_sign[id] + 1;
	
	if (ts->id_sign[id] == 1) {
		ts->x_old[id] = x;
		ts->y_old[id] = y;
	}
	
	x = (ts->x_old[id] + x) / 2;
	y = (ts->y_old[id] + y) / 2;
	
	if (x > ts->x_old[id]) {
		x_err = x - ts->x_old[id];
	} else {
		x_err = ts->x_old[id] - x;
	}
	
	if (y > ts->y_old[id]) {
		y_err = y - ts->y_old[id];
	} else {
		y_err = ts->y_old[id] - y;
	}
	
	if ((x_err > 6 && y_err > 2) || (x_err > 2 && y_err > 6)) {
		ts->x_new = x;
		ts->x_old[id] = x;
		ts->y_new = y;
		ts->y_old[id] = y;
	} else {
		if (x_err > 6) {
			ts->x_new = x;
			ts->x_old[id] = x;
		} else {
			ts->x_new = ts->x_old[id];
		}
		if (y_err> 6) {
			ts->y_new = y;
			ts->y_old[id] = y;
		} else {
			ts->y_new = ts->y_old[id];
		}
	}
	
	if (ts->id_sign[id] == 1) {
		ts->x_new = ts->x_old[id];
		ts->y_new = ts->y_old[id];
	}
}

static void gsl_ts_report_data(struct gsl_ts_data *ts, u16 x, u16 y, u8 pressure, u8 id)
{
	dev_warn(&ts->client->dev, "%s: reporting input data\n", __func__);

	if (ts->xy_swapped) {
		swap(x, y);
	}
	
	/* This check had x and y reversed in the original code. Why? */
	if (x >= ts->x_max || y >= ts->y_max) {
		dev_err(&ts->client->dev, "%s: coordinates out of bounds, x = %u (max %u), y = %u (max %u)\n", __func__, x, ts->x_max, y, ts->y_max);
		return;
	}
	
	if (ts->x_reversed) {
		x = ts->x_max - x;
	}
	
	if (ts->y_reversed) {
		y = ts->y_max - y;
	}
	
	input_mt_slot(ts->input, id);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 1);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, pressure);
	input_report_abs(ts->input, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 1);
}

static inline u16 join_bytes(u8 a, u8 b)
{
	return (a << 8) | b;
}

static void gsl_ts_process_data(struct gsl_ts_data *ts, u8 *event)
{
	u8 id, touches;
	u16 x, y;
	int i;
	
	touches = event[TOUCH_INDEX];
	for (i = 1; i <= MAX_CONTACTS; i++) {
		if (touches == 0) {
			ts->id_sign[i] = 0;
		}
		ts->id_state_flag[i] = 0;
	}
	
	for (i = 0; i < (touches > MAX_FINGERS ? MAX_FINGERS : touches); i++) {
		x = join_bytes((event[X_INDEX  + 4 * i + 1] & 0xf), event[X_INDEX + 4 * i]);
		y = join_bytes(event[Y_INDEX + 4 * i + 1], event[Y_INDEX + 4 * i ]);
		id = event[ID_INDEX + 4 * i] >> 4;
		
		if (1 <= id && id <= MAX_CONTACTS) {
			gsl_ts_record_point(ts, x, y, id);
			gsl_ts_report_data(ts, ts->x_new, ts->y_new, 10, id);
			ts->id_state_flag[id] = 1;
		}
	}
	for (i = 1; i <= MAX_CONTACTS; i++) {
		if ((0 != ts->id_state_old_flag[i]) && (0 == ts->id_state_flag[i])) {
			input_mt_slot(ts->input, i);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
			ts->id_sign[i] = 0;
		}
		ts->id_state_old_flag[i] = ts->id_state_flag[i];
	}
	
	if (touches || touches != ts->prev_touches) {
		input_mt_report_pointer_emulation(ts->input, true);
		input_sync(ts->input);
	}
	
	ts->prev_touches = touches;
}

static void gsl_ts_mt_event(struct gsl_ts_data *ts, u8 *buf)
{
	struct input_dev *input = ts->input;
	struct device *dev = &ts->client->dev;
	struct gsl_ts_packet_header header;
	struct gsl_ts_packet_touch touch;
	u8 i;
	u16 tseq, x, y, id;
	
	dev_info(dev, "%s: touch event\n", __func__);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 1, buf, GSL_PACKET_SIZE, true);
	
	memcpy(&header, buf, sizeof(header));
	tseq = le16_to_cpu(header.time_stamp);
	/* time_stamp is 0 on zero-touch events, seems to wrap around 21800 */
	dev_vdbg(dev, "%s: got touch events for %u fingers @%u\n", __func__, header.num_fingers, tseq);
	
	if (header.num_fingers > GSL_MAX_CONTACTS) {
		header.num_fingers = GSL_MAX_CONTACTS;
	}
	
	for (i = 0; i < header.num_fingers; i++) {
		memcpy(&touch, &buf[sizeof(header) + i * sizeof(touch)], sizeof(touch));
		x = le16_to_cpu(touch.x);
		y = le16_to_cpu(touch.y_id);
		id = y >> 12;
		y &= 0xfff;

		dev_dbg(dev, "%s: touch event %u: x=%u y=%u id=0x%x\n", __func__, i, x, y, id);

		/* Most events seem to carry 0. Is this really a tracking id? */
		
		/* For now, we're dealing with the Silead as if it were a type A MT device only.
		 * See linux/Documentation/input/multi-touch-protocol.txt for more info on how
		 * to implement multitouch drivers.
		 */
		input_event(input, EV_ABS, ABS_MT_POSITION_X, x > ts->x_max ? ts->x_max : x);
		input_event(input, EV_ABS, ABS_MT_POSITION_Y, y > ts->y_max ? ts->y_max : y);
		input_mt_sync_frame(input);

		/*if (id >= 1 && id < GSL_MAX_CONTACTS) {
			input_mt_slot(input, id - 1);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
			input_event(input, EV_ABS, ABS_MT_POSITION_X, x > ts->x_max ? ts->x_max : x);
			input_event(input, EV_ABS, ABS_MT_POSITION_Y, y > ts->y_max ? ts->y_max : y);
		}*/
	}
	
	/* Insert a single sync report for zero-contact events */
	if (header.num_fingers == 0) {
		input_mt_sync_frame(input);
	}
	
	input_sync(input);
}

static irqreturn_t gsl_ts_irq(int irq, void *arg)
{
	int rc;
	struct gsl_ts_data *ts = (struct gsl_ts_data *) arg;
	struct i2c_client *client = ts->client;
	struct device *dev = &client->dev;
	u8 status[4] = { 0, 0, 0, 0 };
	u8 event[GSL_PACKET_SIZE];
	
	dev_dbg(&client->dev, "%s: IRQ received\n", __func__);

	if (ts->state == GSL_TS_SHUTDOWN) {
		dev_warn(&client->dev, "%s: device supended, not handling interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	rc = gsl_ts_read(client, GSL_STATUS_REG, status, sizeof(status));
	if (rc < 0) {
		dev_err(dev, "%s: error reading chip status\n", __func__);
		return IRQ_HANDLED;
	}
	
	if (status[0] == GSL_STATUS_FW) {
		/* TODO: Send firmware here instead of during init */
		dev_info(dev, "%s: device waiting for firmware\n", __func__);
		
	} else if (status[0] == GSL_STATUS_TOUCH) {
		dev_vdbg(dev, "%s: touch event\n", __func__);

		rc = gsl_ts_read(client, GSL_DATA_REG, event, sizeof(event));
		if (rc < 0) {
			dev_err(dev, "%s: touch data read failed\n", __func__);
			return IRQ_HANDLED;
		}
		if (event[0] == 0xff) {
			dev_warn(dev, "%s: ignoring invalid touch record (0xff)\n", __func__);
			return IRQ_HANDLED;
		}
		
		rc = gsl_ts_read(client, GSL_TOUCH_STATUS_REG, status, sizeof(status));
		if (rc < 0) {
			dev_err(dev, "%s: reading touch status register failed\n", __func__);
			return IRQ_HANDLED;
		}
		
		if ((status[0] | status[1] | status[2] | status[3]) == 0) {
			gsl_ts_mt_event(ts, event);
			/*gsl_ts_process_data(ts, event);*/
			
		} else {
			dev_warn(dev, "%s: device seems to be stuck, resetting\n", __func__);
			
			rc = gsl_ts_reset_chip(ts->client);
			if (rc < 0) {
				dev_err(dev, "%s: reset_chip failed\n", __func__);
				return IRQ_HANDLED;
			}
			rc = gsl_ts_startup_chip(ts->client);
			if (rc < 0) {
				dev_err(dev, "%s: startup_chip failed\n", __func__);
				return IRQ_HANDLED;
			}
		}
	} else {
		dev_warn(&client->dev, "%s: IRQ received, unknown status 0x%02x\n", __func__, status[0]);
	}
	
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
		error = acpi_bus_set_power(ACPI_HANDLE(&client->dev), ACPI_STATE_D3);
		if (error == 0) {
			error = acpi_bus_set_power(ACPI_HANDLE(&client->dev), ACPI_STATE_D0);
		}
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

	/*__set_bit(BTN_TOUCH, ts->input->keybit);*/
	__set_bit(EV_ABS, ts->input->evbit);
	/*__set_bit(EV_KEY, ts->input->evbit);*/

	/* Single touch input params setup */
	/*input_set_abs_params(ts->input, ABS_X, 0, ts->x_max, 0, 0);
	input_set_abs_params(ts->input, ABS_Y, 0, ts->y_max, 0, 0);
	input_set_abs_params(ts->input, ABS_PRESSURE, 0, 255, 0, 0);
	input_abs_set_res(ts->input, ABS_X, ts->x_res);
	input_abs_set_res(ts->input, ABS_Y, ts->y_res);*/

	/* Multitouch input params setup */
	/*error = input_mt_init_slots(ts->input, ts->multi_touches, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error) {
		dev_err(&client->dev, "%s: failed to initialize MT slots: %d\n", __func__, error);
		return error;
	}*/

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
	/*input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);*/
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

	/* Set up interrupt handler - do we still need to account for shared interrupts? */
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

	/* Execute the controller startup sequence */
	error = gsl_ts_init_chip(ts);
	if (error < 0) {
		dev_err(&client->dev, "%s: initialisation failed\n", __func__);
		return error;
	}
	
	/*
	 * Systems using device tree should set up wakeup via DTS,
	 * the rest will configure device as wakeup source by default.
	 */
	if (!client->dev.of_node) {
		device_init_wakeup(&client->dev, true);
	}
	
	ts->state = GSL_TS_GREEN;

	return 0;
}

static int __maybe_unused gsl_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gsl_ts_data *ts = i2c_get_clientdata(client);
	
	dev_warn(&client->dev, "%s: suspending device\n", __func__);
	
	disable_irq(client->irq);
	
	gsl_ts_reset_chip(client);
	msleep(10);

	/* Do we need to do this ourselves? */
	acpi_bus_set_power(ACPI_HANDLE(&client->dev), ACPI_STATE_D3);
	
	if (device_may_wakeup(dev)) {
		ts->wake_irq_enabled = (enable_irq_wake(client->irq) == 0);
	}

	ts->state = GSL_TS_SHUTDOWN;

	return 0;
}

static int __maybe_unused gsl_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gsl_ts_data *ts = i2c_get_clientdata(client);

	dev_warn(&client->dev, "%s: resuming device\n", __func__);

	if (device_may_wakeup(dev) && ts->wake_irq_enabled) {
		disable_irq_wake(client->irq);
	}
	
	/* Do we need to do this ourselves? */
	acpi_bus_set_power(ACPI_HANDLE(&client->dev), ACPI_STATE_D0);
	msleep(20);
	
	gsl_ts_reset_chip(client);
	gsl_ts_startup_chip(client);
	
	enable_irq(client->irq);

	ts->state = GSL_TS_GREEN;
	
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
