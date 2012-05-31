/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
/*
 *   Copyright 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

#include "crtouch_mt.h"

void report_single_touch(void)
{
	input_report_abs(crtouch->input_dev, ABS_MT_POSITION_X, crtouch->x1);
	input_report_abs(crtouch->input_dev, ABS_MT_POSITION_Y, crtouch->y1);
	input_report_abs(crtouch->input_dev, ABS_MT_TOUCH_MAJOR, 1);
	input_mt_sync(crtouch->input_dev);
	input_event(crtouch->input_dev, EV_ABS, ABS_X, crtouch->x1);
	input_event(crtouch->input_dev, EV_ABS, ABS_Y, crtouch->y1);
	input_event(crtouch->input_dev, EV_KEY, BTN_TOUCH, 1);
	input_report_abs(crtouch->input_dev, ABS_PRESSURE, 1);
	input_sync(crtouch->input_dev);

	status_pressed = CRICS_TOUCHED;
}

void report_multi_touch(void)
{

	input_report_key(crtouch->input_dev, ABS_MT_TRACKING_ID, 0);
	input_report_abs(crtouch->input_dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(crtouch->input_dev, ABS_MT_POSITION_X, crtouch->x1);
	input_report_abs(crtouch->input_dev, ABS_MT_POSITION_Y, crtouch->y1);
	input_mt_sync(crtouch->input_dev);

	input_report_key(crtouch->input_dev, ABS_MT_TRACKING_ID, 1);
	input_report_abs(crtouch->input_dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(crtouch->input_dev, ABS_MT_POSITION_X, crtouch->x2);
	input_report_abs(crtouch->input_dev, ABS_MT_POSITION_Y, crtouch->y2);
	input_mt_sync(crtouch->input_dev);

	input_sync(crtouch->input_dev);

	status_pressed = CRICS_TOUCHED;
}

void free_touch(void)
{
	input_event(crtouch->input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(crtouch->input_dev);
	input_event(crtouch->input_dev, EV_KEY, BTN_TOUCH, 0);
	input_report_abs(crtouch->input_dev, ABS_PRESSURE, 0);
	input_sync(crtouch->input_dev);

	status_pressed = CRICS_RELEASED;
}

void free_two_touch(void){

	input_event(crtouch->input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(crtouch->input_dev);
	input_sync(crtouch->input_dev);

}

int read_resolution(void)
{
	char resolution[LEN_RESOLUTION_BYTES];
	int horizontal, vertical, ret;

	ret = i2c_smbus_read_i2c_block_data(client_public,
		HORIZONTAL_RESOLUTION_MBS, LEN_RESOLUTION_BYTES, resolution);

	if (ret < 0)
		return ret;

	horizontal = resolution[1];
	horizontal |= resolution[0] << 8;
	vertical = resolution[3];
	vertical |= resolution[2] << 8;

	xmax = horizontal;
	ymax = vertical;

	printk(KERN_DEBUG "calibration values XMAX:%i YMAX:%i", xmax, ymax);

	return 0;
}

static void report_MT(struct work_struct *work)
{

	struct crtouch_data *crtouch = container_of(work, struct crtouch_data, work);
	struct i2c_client *client = crtouch->client;

	s32 status_register_1 = 0;
	s32 status_register_2 = 0;
	s32 dynamic_status = 0;
	s32 fifo_capacitive = 0;
	s32 rotate_angle_help = 0;
	int result;
	int command = 0;
	int degrees = 0;
	int zoom_value_moved = 0;
	char xy[LEN_XY];

	status_register_1 = i2c_smbus_read_byte_data(client, STATUS_REGISTER_1);

	/*check zoom resistive*/
	if ((status_register_1 & MASK_EVENTS_ZOOM_R) == MASK_EVENTS_ZOOM_R && (status_register_1 & TWO_TOUCH)) {

		status_register_2 = i2c_smbus_read_byte_data(client, STATUS_REGISTER_2);

		if ((status_register_2 & MASK_ZOOM_DIRECTION) == MASK_ZOOM_DIRECTION) {
			command = ZOOM_OUT;
		} else {
			command = ZOOM_IN;
		}

	/*check rotate resistive*/
	} else if ((status_register_1 & MASK_EVENTS_ROTATE_R) == MASK_EVENTS_ROTATE_R  && (status_register_1 & TWO_TOUCH)) {

		status_register_2 = i2c_smbus_read_byte_data(client, STATUS_REGISTER_2);
		rotate_angle = i2c_smbus_read_byte_data(client, ROTATE_ANGLE);
		rotate_angle_help = rotate_angle;

		if ((status_register_2 & MASK_ROTATE_DIRECTION) == MASK_ROTATE_DIRECTION) {

			command = ROTATE_COUNTER_CLK;

			if (rotate_state == ROTATE_CLK)
				last_angle = 0;

			rotate_state = ROTATE_COUNTER_CLK;
			rotate_angle -= last_angle;
			last_angle += rotate_angle;

		} else {

			command = ROTATE_CLK;

			if (rotate_state == ROTATE_COUNTER_CLK)
				last_angle = 0;

			rotate_state = ROTATE_CLK;
			rotate_angle -= last_angle;
			last_angle += rotate_angle;
		}
	}

	/*check slide resistive*/
	if ((status_register_1 & MASK_EVENTS_SLIDE_R) == MASK_EVENTS_SLIDE_R) {

		status_register_2 = i2c_smbus_read_byte_data(client, STATUS_REGISTER_2);

		if ((status_register_2 & MASK_SLIDE_DOWN) == MASK_SLIDE_DOWN) {
			input_report_key(crtouch->input_dev, KEY_H, 1);
			input_report_key(crtouch->input_dev, KEY_H, 0);
			input_sync(crtouch->input_dev);
		} else if ((status_register_2 & MASK_SLIDE_UP) == MASK_SLIDE_UP) {
			input_report_key(crtouch->input_dev, KEY_G, 1);
			input_report_key(crtouch->input_dev, KEY_G, 0);
			input_sync(crtouch->input_dev);
		} else if ((status_register_2 & MASK_SLIDE_LEFT) == MASK_SLIDE_LEFT) {
			input_report_key(crtouch->input_dev, KEY_F, 1);
			input_report_key(crtouch->input_dev, KEY_F, 0);
			input_sync(crtouch->input_dev);
		} else if ((status_register_2 & MASK_SLIDE_RIGHT) == MASK_SLIDE_RIGHT) {
			input_report_key(crtouch->input_dev, KEY_E, 1);
			input_report_key(crtouch->input_dev, KEY_E, 0);
			input_sync(crtouch->input_dev);
		}

	}

	/*check capacitive events*/
	if ((status_register_1 & MASK_EVENTS_CAPACITIVE) == MASK_EVENTS_CAPACITIVE) {

		/*capacitive keypad*/
		if ((data_configuration & MASK_KEYPAD_CONF) == MASK_KEYPAD_CONF) {

			while ((fifo_capacitive = i2c_smbus_read_byte_data(client, CAPACITIVE_ELECTRODES_FIFO)) < 0xFF) {

				if ((fifo_capacitive & KEY_NUMBER) == MASK_DYNAMIC_DISPLACEMENT_BTN3) {

					if ((fifo_capacitive & EV_TYPE) == 0) {
						input_report_key(crtouch->input_dev, KEY_D, 1);
						input_sync(crtouch->input_dev);
					} else{
						input_report_key(crtouch->input_dev, KEY_D, 0);
						input_sync(crtouch->input_dev);
					}

				} else if ((fifo_capacitive & KEY_NUMBER) == MASK_DYNAMIC_DISPLACEMENT_BTN2) {

					if ((fifo_capacitive & EV_TYPE) == 0) {
						input_report_key(crtouch->input_dev, KEY_C, 1);
						input_sync(crtouch->input_dev);
					} else{
						input_report_key(crtouch->input_dev, KEY_C, 0);
						input_sync(crtouch->input_dev);
					}

				} else if ((fifo_capacitive & KEY_NUMBER) == MASK_DYNAMIC_DISPLACEMENT_BTN1) {

					if ((fifo_capacitive & EV_TYPE) == 0) {
						input_report_key(crtouch->input_dev, KEY_B, 1);
						input_sync(crtouch->input_dev);
					} else{
						input_report_key(crtouch->input_dev, KEY_B, 0);
						input_sync(crtouch->input_dev);
					}

				} else if ((fifo_capacitive & KEY_NUMBER) == MASK_DYNAMIC_DISPLACEMENT_BTN0) {

					if ((fifo_capacitive & EV_TYPE) == 0) {
						input_report_key(crtouch->input_dev, KEY_A, 1);
						input_sync(crtouch->input_dev);
					} else {
						input_report_key(crtouch->input_dev, KEY_A, 0);
						input_sync(crtouch->input_dev);
					}
				}

			}


		}
		/*capacitive slide*/
		else if ((data_configuration & MASK_SLIDE_CONF) == MASK_SLIDE_CONF) {
			dynamic_status = i2c_smbus_read_byte_data(client, DYNAMIC_STATUS);

			if (dynamic_status & MASK_DYNAMIC_FLAG) {

				if (dynamic_status & MASK_DYNAMIC_DIRECTION) {
					input_report_key(crtouch->input_dev, KEY_I, 1);
					input_report_key(crtouch->input_dev, KEY_I, 0);
					input_sync(crtouch->input_dev);
				} else {
					input_report_key(crtouch->input_dev, KEY_J, 1);
					input_report_key(crtouch->input_dev, KEY_J, 0);
					input_sync(crtouch->input_dev);
				}
			}
		}
		/*capacitive rotate*/
		else if ((data_configuration & MASK_ROTARY_CONF) == MASK_ROTARY_CONF) {

			dynamic_status = i2c_smbus_read_byte_data(client, DYNAMIC_STATUS);

			if (dynamic_status & MASK_DYNAMIC_FLAG) {

				if (dynamic_status & MASK_DYNAMIC_DIRECTION) {
					input_report_key(crtouch->input_dev, KEY_K, 1);
					input_report_key(crtouch->input_dev, KEY_K, 0);
					input_sync(crtouch->input_dev);
				} else {
					input_report_key(crtouch->input_dev, KEY_L, 1);
					input_report_key(crtouch->input_dev, KEY_L, 0);
					input_sync(crtouch->input_dev);
				}
			}
		}

	}

	/*check xy*/
	if ((status_register_1 & MASK_RESISTIVE_SAMPLE) == MASK_RESISTIVE_SAMPLE && !(status_register_1 & TWO_TOUCH)) {

		/*clean zoom data when release 2touch*/
		last_angle = 0;
		rotate_state = 0;

		if (!(status_register_1 & MASK_PRESSED)) {

			if (status_pressed)
				free_touch();
		} else {
			result = i2c_smbus_read_i2c_block_data(client, X_COORDINATE_MSB, LEN_XY, xy);

			if (result < 0)
				/*Do nothing*/
				printk(KERN_ALERT "Single Touch Error Reading");

			else{
				crtouch->x1 = xy[1];
				crtouch->x1 |= xy[0] << 8;
				crtouch->y1 = xy[3];
				crtouch->y1 |= xy[2] << 8;

				/*if differents points*/
				if (crtouch->x1_old != crtouch->x1 || crtouch->y1_old != crtouch->y1) {

					crtouch->x1_old = crtouch->x1;
					crtouch->y1_old = crtouch->y1;
					report_single_touch();
				}

			}

		}

	}

	/*simulate gestures 2 touch*/
	if (command) {

		if (command == ZOOM_IN || command == ZOOM_OUT) {

			free_two_touch();

			switch (command) {

			case ZOOM_IN:

				msleep(1);
				/*simulate initial points*/
				crtouch->x1 = (xmax/2) - (xmax/20);
				crtouch->x2 = (xmax/2) + (xmax/20);
				crtouch->y1 = ymax/2;
				crtouch->y2 = ymax/2;

				report_multi_touch();

				zoom_value_moved = (xmax/100);

				if (zoom_value_moved > 0) {
					msleep(1);
					crtouch->x1 -= zoom_value_moved;
					crtouch->x2 += zoom_value_moved;
					report_multi_touch();
					msleep(1);
					crtouch->x1 -= zoom_value_moved;
					crtouch->x2 += zoom_value_moved;
					report_multi_touch();

				} else {
					msleep(1);
					crtouch->x1 -= 5;
					crtouch->x2 += 5;
					report_multi_touch();
					msleep(1);
					crtouch->x1 -= 5;
					crtouch->x2 += 5;
					report_multi_touch();
				}
				break;

			case ZOOM_OUT:

				msleep(1);
				crtouch->x1 = ((xmax * 43) / 100);
				crtouch->x2 = ((xmax * 57) / 100);
				crtouch->y1 = ymax/2;
				crtouch->y2 = ymax/2;
				report_multi_touch();

				zoom_value_moved = (xmax / 100);

				if (zoom_value_moved > 0) {
					msleep(1);
					crtouch->x1 += zoom_value_moved;
					crtouch->x2 -= zoom_value_moved;
					report_multi_touch();
					msleep(1);
					crtouch->x1 += zoom_value_moved;
					crtouch->x2 -= zoom_value_moved;
					report_multi_touch();

				} else {
					msleep(1);
					crtouch->x1 += 5;
					crtouch->x2 -= 5;
					report_multi_touch();
					msleep(1);
					crtouch->x1 += 5;
					crtouch->x2 -= 5;
					report_multi_touch();
				}
				break;
			}

		}

		else if (command == ROTATE_CLK || command == ROTATE_COUNTER_CLK) {

			free_two_touch();

			/*get radians from crtouch and change them to degrees*/
			degrees = ((rotate_angle * 360) / (64 * 3));

			if (degrees < 0) {

				/*fix if negative values appear, because of time sincronization*/
				degrees = rotate_angle_help;
				last_angle = rotate_angle_help;
			}

			/*simulate initial points*/
			crtouch->x1 = xmax/2;
			crtouch->y1 = ymax/2;
			crtouch->x2 = (xmax/2) + (xmax);
			crtouch->y2 = (ymax/2);
			report_multi_touch();

			if (command == ROTATE_CLK) {

				if (degrees <= 180) {
					crtouch->x2 = (xmax/2) + (((cos_data[degrees-1]*(xmax))/65536));
					crtouch->y2 = (ymax/2) + (((sin_data[degrees-1]*(ymax))/65536));

				} else if (degrees <= 360) {
					crtouch->x2 = (xmax/2) + (((~cos_data[(degrees-181)])*(xmax))/65536);
					crtouch->y2 = (ymax/2) + ((((~sin_data[(degrees-181)])*(ymax))/65536));

				} else if (degrees <= 540) {
					crtouch->x2 = (xmax/2) + (((cos_data[degrees-361]*(xmax))/65536));
					crtouch->y2 = (ymax/2) + (((sin_data[degrees-361]*(ymax))/65536));
				}
				report_multi_touch();

			} else if (command == ROTATE_COUNTER_CLK) {

				if (degrees <= 180) {
					crtouch->x2 = (xmax/2) - (((cos_data[180 - degrees]*(xmax))/65536));
					crtouch->y2 = (ymax/2) - (((sin_data[180 - degrees]*(ymax))/65536));

				} else if (degrees <= 360) {
					crtouch->x2 = (xmax/2) - (((~cos_data[360 - degrees]*(xmax))/65536));
					crtouch->y2 = (ymax/2) - (((~sin_data[360 - degrees]*(ymax))/65536));

				} else if (degrees <= 540) {
					crtouch->x2 = (xmax/2) - (((cos_data[540 - degrees]*(xmax))/65536));
					crtouch->y2 = (ymax/2) - (((sin_data[540 - degrees]*(ymax))/65536));
				}
				report_multi_touch();
			}

		}

		/*clean command*/
		command = 0;

	}

}

int crtouch_open(struct inode *inode, struct file *filp)
{
	/*Do nothing*/

	return 0;
}

/*read crtouch registers*/
static ssize_t crtouch_read(struct file *filep,
			char *buf, size_t count, loff_t *fpos)
{

	s32 data_to_read;

	if (buf != NULL) {

		switch (*buf) {

		case STATUS_ERROR:
		case STATUS_REGISTER_1:
		case STATUS_REGISTER_2:
		case X_COORDINATE_MSB:
		case X_COORDINATE_LSB:
		case Y_COORDINATE_MSB:
		case Y_COORDINATE_LSB:
		case PRESSURE_VALUE_MSB:
		case PRESSURE_VALUE_LSB:
		case FIFO_STATUS:
		case FIFO_X_COORDINATE_MSB:
		case FIFO_X_COORDINATE_LSB:
		case FIFO_Y_COORDINATE_MSB:
		case FIFO_Y_COORDINATE_LSB:
		case FIFO_PRESSURE_VALUE_COORDINATE_MSB:
		case FIFO_PRESSURE_VALUE_COORDINATE_LSB:
		case UART_BAUDRATE_MSB:
		case UART_BAUDRATE_MID:
		case UART_BAUDRATE_LSB:
		case DEVICE_IDENTIFIER_REGISTER:
		case SLIDE_DISPLACEMENT:
		case ROTATE_ANGLE:
		case ZOOM_SIZE:
		case ELECTRODE_STATUS:
		case FAULTS_NOTE1:
		case E0_BASELINE_MSB:
		case E0_BASELINE_LSB:
		case E1_BASELINE_MSB:
		case E1_BASELINE_LSB:
		case E2_BASELINE_MSB:
		case E2_BASELINE_LSB:
		case E3_BASELINE_MSB:
		case E3_BASELINE_LSB:
		case E0_INSTANT_DELTA:
		case E1_INSTANT_DELTA:
		case E2_INSTANT_DELTA:
		case E3_INSTANT_DELTA:
		case DYNAMIC_STATUS:
		case STATIC_STATUS:
		case CONFIGURATION:
		case TRIGGER_EVENTS:
		case FIFO_SETUP:
		case SAMPLING_X_Y:
		case X_SETTLING_TIME_MBS:
		case X_SETTLING_TIME_LBS:
		case Y_SETTLING_TIME_MBS:
		case Y_SETTLING_TIME_LBS:
		case Z_SETTLING_TIME_MBS:
		case Z_SETTLING_TIME_LBS:
		case HORIZONTAL_RESOLUTION_MBS:
		case HORIZONTAL_RESOLUTION_LBS:
		case VERTICAL_RESOLUTION_MBS:
		case VERTICAL_RESOLUTION_LBS:
		case SLIDE_STEPS:
		case SYSTEM_CONFIG_NOTE2:
		case DC_TRACKER_RATE:
		case RESPONSE_TIME:
		case STUCK_KEY_TIMEOUT:
		case E0_SENSITIVITY:
		case E1_SENSITIVITY:
		case E2_SENSITIVITY:
		case E3_SENSITIVITY:
		case ELECTRODE_ENABLERS:
		case LOW_POWER_SCAN_PERIOD:
		case LOW_POWER_ELECTRODE:
		case LOW_POWER_ELECTRODE_SENSITIVITY:
		case CONTROL_CONFIG:
		case EVENTS:
		case AUTO_REPEAT_RATE:
		case AUTO_REPEAT_START:
		case MAX_TOUCHES:

			data_to_read = i2c_smbus_read_byte_data(client_public, *buf);

			if (data_to_read >= 0 && copy_to_user(buf, &data_to_read, 1))
				printk(KERN_DEBUG "error reading from userspace\n");
			break;

		default:
			printk(KERN_DEBUG "invalid address to read\n");
			return -ENXIO;

		}

	}

	return 1;
}

/*write crtouch register to configure*/
static ssize_t crtouch_write(struct file *filep, const char __user *buf, size_t size, loff_t *fpos)
{

	const unsigned char *data_to_write = NULL;

	data_to_write = buf;

	if (data_to_write == NULL)
		return -ENOMEM;
	if ((data_to_write + 1) == NULL)
		return -EINVAL;

	/*update driver variable*/
	if (*data_to_write == CONFIGURATION)	{
			if ((*(data_to_write + 1) >= 0x00) && (*(data_to_write + 1) <= 0xFF))
				data_configuration = *(data_to_write + 1);
	}

	switch (*data_to_write) {

	case CONFIGURATION:
	case X_SETTLING_TIME_MBS:
	case X_SETTLING_TIME_LBS:
	case Y_SETTLING_TIME_MBS:
	case Y_SETTLING_TIME_LBS:
	case Z_SETTLING_TIME_MBS:
	case Z_SETTLING_TIME_LBS:
	case HORIZONTAL_RESOLUTION_MBS:
	case HORIZONTAL_RESOLUTION_LBS:
	case VERTICAL_RESOLUTION_MBS:
	case VERTICAL_RESOLUTION_LBS:
	case SLIDE_STEPS:
	case SYSTEM_CONFIG_NOTE2:
	case DC_TRACKER_RATE:
	case STUCK_KEY_TIMEOUT:
	case LOW_POWER_ELECTRODE_SENSITIVITY:
	case EVENTS:
	case CONTROL_CONFIG:
	case AUTO_REPEAT_RATE:
	case AUTO_REPEAT_START:
	case TRIGGER_EVENTS:

		if ((*(data_to_write + 1) >= 0x00) && (*(data_to_write + 1) <= 0xFF))
			i2c_smbus_write_byte_data(client_public,
					*data_to_write, *(data_to_write + 1));
		else
			printk(KERN_DEBUG "invalid range of data\n");

		break;


	case RESPONSE_TIME:

		if ((*(data_to_write + 1) >= 0x00) && (*(data_to_write + 1) <= 0x3F))
			i2c_smbus_write_byte_data(client_public,
					*data_to_write, *(data_to_write+1));
		else
			printk(KERN_DEBUG "invalid range of data\n");
		break;

	case FIFO_SETUP:
		if ((*(data_to_write + 1) >= 0x00) && (*(data_to_write + 1) <= 0x1F))
			i2c_smbus_write_byte_data(client_public,
					*data_to_write, *(data_to_write+1));
		else
			printk(KERN_DEBUG "invalid range of data\n");
		break;

	case SAMPLING_X_Y:
		if ((*(data_to_write + 1) >= 0x05) && (*(data_to_write + 1) <= 0x64))
			i2c_smbus_write_byte_data(client_public,
					*data_to_write, *(data_to_write+1));
		else
			printk(KERN_DEBUG "invalid range of data\n");
		break;


	case E0_SENSITIVITY:
	case E1_SENSITIVITY:
	case E2_SENSITIVITY:
	case E3_SENSITIVITY:
		if ((*(data_to_write+1) >= 0x02) && (*(data_to_write+1) <= 0x7F))
			i2c_smbus_write_byte_data(client_public,
					*data_to_write, *(data_to_write+1));
		else
			printk(KERN_DEBUG "invalid range of data\n");
		break;


	case ELECTRODE_ENABLERS:
	case LOW_POWER_SCAN_PERIOD:
	case LOW_POWER_ELECTRODE:
	case MAX_TOUCHES:
		if ((*(data_to_write+1) >= 0x00) && (*(data_to_write+1) <= 0x0F))
			i2c_smbus_write_byte_data(client_public,
					*data_to_write, *(data_to_write+1));
		else
			printk(KERN_DEBUG "invalid range of data\n");
		break;


	default:
		printk(KERN_DEBUG "invalid address to write\n");
		return -ENXIO;

	}


	return 1;
}

struct file_operations file_ops_crtouch = {

open:	crtouch_open,
write :	crtouch_write,
read :	crtouch_read,

};


static irqreturn_t crtouch_irq(int irq, void *dev_id)
{
	queue_work(crtouch->workqueue, &crtouch->work);
	return IRQ_HANDLED;
}

static int crtouch_resume(struct i2c_client *client)
{
	gpio_set_value(PIN_WAKE, GND);
	udelay(10);
	gpio_set_value(PIN_WAKE, VCC);
	return 0;
}

static int crtouch_suspend(struct i2c_client *client, pm_message_t mesg)
{
	s32 data_to_read;

	data_to_read = i2c_smbus_read_byte_data(client_public, CONFIGURATION);
	data_to_read |= SHUTDOWN_CRICS;
	i2c_smbus_write_byte_data(client, CONFIGURATION , data_to_read);
	return 0;
}

static int __devinit crtouch_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{

	int result;
	struct input_dev *input_dev;
	int error = 0;
	s32 mask_trigger = 0;


	/*to be able to communicate by i2c with crtouch (dev)*/
	client_public = client;

	crtouch = kzalloc(sizeof(struct crtouch_data), GFP_KERNEL);

	if (!crtouch)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev) {
		result = -ENOMEM;
		goto err_free_mem;
	}

	crtouch->input_dev = input_dev;
	crtouch->client = client;
	crtouch->workqueue = create_singlethread_workqueue("crtouch");
	INIT_WORK(&crtouch->work, report_MT);

	if (crtouch->workqueue == NULL) {
		printk(KERN_DEBUG "couldn't create workqueue\n");
		result = -ENOMEM;
		goto err_wqueue;
	}

	error = read_resolution();
	if (error < 0) {
		printk(KERN_DEBUG "couldn't read size of screen");
		result = -EIO;
		goto err_free_wq;
	}

	data_configuration = i2c_smbus_read_byte_data(client, CONFIGURATION);
	data_configuration &= CLEAN_SLIDE_EVENTS;
	data_configuration |= SET_MULTITOUCH;
	i2c_smbus_write_byte_data(client, CONFIGURATION, data_configuration);

	mask_trigger = i2c_smbus_read_byte_data(client, TRIGGER_EVENTS);
	mask_trigger |= SET_TRIGGER_RESISTIVE;
	i2c_smbus_write_byte_data(client, TRIGGER_EVENTS, mask_trigger);

	i2c_smbus_write_byte_data(client, TRIGGER_EVENTS, mask_trigger);

	crtouch->input_dev->name = "CRTOUCH Input Device";
	crtouch->input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, crtouch->input_dev->evbit);
	__set_bit(EV_KEY, crtouch->input_dev->evbit);
	__set_bit(BTN_TOUCH, crtouch->input_dev->keybit);
	__set_bit(ABS_X, crtouch->input_dev->absbit);
	__set_bit(ABS_Y, crtouch->input_dev->absbit);
	__set_bit(ABS_PRESSURE, crtouch->input_dev->absbit);
	__set_bit(EV_SYN, crtouch->input_dev->evbit);

	/*register keys that will be reported by crtouch*/
	__set_bit(KEY_A, crtouch->input_dev->keybit);
	__set_bit(KEY_B, crtouch->input_dev->keybit);
	__set_bit(KEY_C, crtouch->input_dev->keybit);
	__set_bit(KEY_D, crtouch->input_dev->keybit);
	__set_bit(KEY_E, crtouch->input_dev->keybit);
	__set_bit(KEY_F, crtouch->input_dev->keybit);
	__set_bit(KEY_G, crtouch->input_dev->keybit);
	__set_bit(KEY_H, crtouch->input_dev->keybit);
	__set_bit(KEY_I, crtouch->input_dev->keybit);
	__set_bit(KEY_J, crtouch->input_dev->keybit);
	__set_bit(KEY_K, crtouch->input_dev->keybit);
	__set_bit(KEY_L, crtouch->input_dev->keybit);

	input_set_abs_params(crtouch->input_dev, ABS_X, XMIN, xmax, 0, 0);
	input_set_abs_params(crtouch->input_dev, ABS_Y, YMIN, ymax, 0, 0);
	input_set_abs_params(crtouch->input_dev, ABS_MT_POSITION_X,
				XMIN, xmax, 0, 0);
	input_set_abs_params(crtouch->input_dev, ABS_MT_POSITION_Y,
				YMIN, ymax, 0, 0);
	input_set_abs_params(crtouch->input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(crtouch->input_dev, ABS_MT_TRACKING_ID, 0, 1, 0, 0);

	input_set_abs_params(crtouch->input_dev, ABS_PRESSURE, 0, 1, 0, 0);
	printk(KERN_DEBUG "CR-TOUCH max values X: %d Y: %d\n", xmax, ymax);

	result = input_register_device(crtouch->input_dev);
	if (result)
		goto err_free_wq;

	if (alloc_chrdev_region(&dev_number, 0, 2, DEV_NAME) < 0) {
		printk(KERN_DEBUG "couldn't allocate cr-touch device with dev");
		goto err_unr_dev;
	}

	cdev_init(&crtouch_cdev , &file_ops_crtouch);

	if (cdev_add(&crtouch_cdev, dev_number, 1)) {
		printk(KERN_DEBUG "couldn't register cr-touch device with dev\n");
		goto err_unr_chrdev;
	}

	crtouch_class = class_create(THIS_MODULE, DEV_NAME);

	if (crtouch_class == NULL) {
		printk(KERN_DEBUG "unable to create a class");
		goto err_unr_createdev;
	}

	if (device_create(crtouch_class, NULL, dev_number,
				NULL, DEV_NAME) == NULL) {
		printk(KERN_DEBUG "unable to create a device");
		goto err_unr_cdev;
	}

	result = gpio_request(PIN_WAKE, "GPIO_WAKE_CRTOUCH");

	if (result != 0) {
		printk(KERN_DEBUG "error requesting GPIO %d", result);
		goto err_unr_class;
	}

	result = gpio_direction_output(PIN_WAKE, GPIOF_OUT_INIT_HIGH);

	if (result != 0) {
		printk(KERN_DEBUG "error config GPIO PIN direction %d", result);
		goto err_free_pin;
	}

	gpio_set_value(PIN_WAKE, VCC);


	/*request gpio to used as interrupt*/
	result = gpio_request(GPIO_IRQ, "GPIO_INTERRUPT_CRTOUCH");

	if (result != 0) {
		printk(KERN_DEBUG "error requesting GPIO for IRQ %d", result);
		goto err_free_pin;
	}

	result = gpio_direction_input(GPIO_IRQ);

	if (result != 0) {
		printk(KERN_DEBUG "error config IRQ PIN direction %d", result);
		goto err_free_pinIrq;
	}

	/* request irq trigger falling */
	result = request_irq(gpio_to_irq(GPIO_IRQ), crtouch_irq,
				IRQF_TRIGGER_FALLING, IRQ_NAME, crtouch_irq);

	if (result < 0) {
		printk(KERN_DEBUG "unable to request IRQ\n");
		goto err_free_pinIrq;
	}

	/*clean interrupt pin*/
	i2c_smbus_read_byte_data(client, STATUS_REGISTER_1);

	return 0;

err_free_pinIrq:
	gpio_free(GPIO_IRQ);
err_free_pin:
	gpio_free(PIN_WAKE);
err_unr_class:
	class_destroy(crtouch_class);
err_unr_createdev:
	device_destroy(crtouch_class, dev_number);
err_unr_cdev:
	cdev_del(&crtouch_cdev);
err_unr_chrdev:
	unregister_chrdev_region(dev_number, 1);
err_unr_dev:
	input_unregister_device(crtouch->input_dev);
err_free_wq:
	destroy_workqueue(crtouch->workqueue);
err_wqueue:
	input_free_device(crtouch->input_dev);
err_free_mem:
	kfree(crtouch);
	return result;
}

static int __devexit crtouch_remove(struct i2c_client *client)
{
	cancel_work_sync(&crtouch->work);
	destroy_workqueue(crtouch->workqueue);
	class_destroy(crtouch_class);
	input_unregister_device(crtouch->input_dev);
	input_free_device(crtouch->input_dev);
	unregister_chrdev_region(dev_number, 1);
	free_irq(gpio_to_irq(GPIO_IRQ), crtouch_irq);
	gpio_free(PIN_WAKE);
	gpio_free(GPIO_IRQ);
	kfree(crtouch);

	return 0;
}

static const struct i2c_device_id crtouch_idtable[] = {
	{"crtouchId", 0},
	{}
};

static struct i2c_driver crtouch_fops = {

	.driver = {
		   .owner = THIS_MODULE,
		   .name = "crtouch_drv",
	},
	.id_table	= 	crtouch_idtable,
	.probe	= 	crtouch_probe,
	.resume	=	crtouch_resume,
	.suspend	= crtouch_suspend,
	.remove 	= __devexit_p(crtouch_remove),

};

MODULE_DEVICE_TABLE(i2c, crtouch_idtable);

static int __init crtouch_init(void)
{
	return i2c_add_driver(&crtouch_fops);
}

static void __exit crtouch_exit(void)
{
	i2c_del_driver(&crtouch_fops);
}


module_init(crtouch_init);
module_exit(crtouch_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CR-TOUCH multitouch driver");
MODULE_LICENSE("GPL");
