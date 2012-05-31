/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/cdev.h>
#include <linux/device.h>

/*keys reported to input device
--capacitive--
keypad: 
	crtouch_btn_0 = KEY_A
	crtouch_btn_1 = KEY_B
	crtouch_btn_2 = KEY_C
	crtouch_btn_3 = KEY_D

slide: 
	incremental: KEY_I
	decremental: KEY_J
rotate:
	incremental: KEY_K
	decremental: KEY_L

--resistive--
slide:
	down:  	KEY_H;
	up:   	KEY_G;
	left: 	KEY_F;
	right:	KEY_E;
*/

/*gestures*/
#define ZOOM_IN 		1
#define ZOOM_OUT		2
#define ROTATE_CLK 		7
#define ROTATE_COUNTER_CLK 	8
#define SINGLE_TOUCH		3

/*status registers*/
#define STATUS_ERROR				0x00
#define STATUS_REGISTER_1 			0x01 
#define STATUS_REGISTER_2 			0x02
#define X_COORDINATE_MSB 			0x03
#define X_COORDINATE_LSB 			0x04
#define Y_COORDINATE_MSB 			0x05
#define Y_COORDINATE_LSB 			0x06
#define PRESSURE_VALUE_MSB 			0x07
#define PRESSURE_VALUE_LSB 			0x08
#define FIFO_STATUS	 			0x09
#define FIFO_X_COORDINATE_MSB 			0x0A
#define FIFO_X_COORDINATE_LSB 			0x0B
#define FIFO_Y_COORDINATE_MSB 			0x0C
#define FIFO_Y_COORDINATE_LSB 			0x0D
#define FIFO_PRESSURE_VALUE_COORDINATE_MSB 	0x0E
#define FIFO_PRESSURE_VALUE_COORDINATE_LSB 	0x0F
#define UART_BAUDRATE_MSB 			0x10
#define UART_BAUDRATE_MID 			0x11
#define UART_BAUDRATE_LSB 			0x12
#define DEVICE_IDENTIFIER_REGISTER 		0x13
#define SLIDE_DISPLACEMENT 			0x14
#define ROTATE_ANGLE	 			0x15
#define ZOOM_SIZE	 			0x16
#define ELECTRODE_STATUS 			0x20
#define FAULTS_NOTE1	 			0x21
#define E0_BASELINE_MSB 			0x22
#define E0_BASELINE_LSB 			0x23
#define E1_BASELINE_MSB 			0x24
#define E1_BASELINE_LSB 			0x25
#define E2_BASELINE_MSB 			0x26
#define E2_BASELINE_LSB 			0x27
#define E3_BASELINE_MSB 			0x28
#define E3_BASELINE_LSB 			0x29
#define E0_INSTANT_DELTA 			0x2A
#define E1_INSTANT_DELTA 			0x2B
#define E2_INSTANT_DELTA 			0x2C
#define E3_INSTANT_DELTA 			0x2D
#define DYNAMIC_STATUS				0x2E 
#define STATIC_STATUS				0x2F  

/*configuration registers*/
#define CAPACITIVE_ELECTRODES_FIFO		0x30
#define CONFIGURATION 				0x40 
#define TRIGGER_EVENTS				0x41
#define FIFO_SETUP				0x42
#define SAMPLING_X_Y				0x43
#define X_SETTLING_TIME_MBS			0x44
#define X_SETTLING_TIME_LBS			0x45
#define Y_SETTLING_TIME_MBS			0x46
#define Y_SETTLING_TIME_LBS			0x47
#define Z_SETTLING_TIME_MBS			0x48
#define Z_SETTLING_TIME_LBS			0x49
#define HORIZONTAL_RESOLUTION_MBS		0x4A
#define HORIZONTAL_RESOLUTION_LBS		0x4B
#define VERTICAL_RESOLUTION_MBS			0x4C
#define VERTICAL_RESOLUTION_LBS			0x4D
#define SLIDE_STEPS				0x4E
#define SYSTEM_CONFIG_NOTE2			0x60
#define DC_TRACKER_RATE				0x61
#define RESPONSE_TIME				0x62
#define STUCK_KEY_TIMEOUT			0x63
#define E0_SENSITIVITY				0x64
#define E1_SENSITIVITY				0x65
#define E2_SENSITIVITY				0x66
#define E3_SENSITIVITY				0x67
#define ELECTRODE_ENABLERS			0x68
#define LOW_POWER_SCAN_PERIOD			0x69
#define LOW_POWER_ELECTRODE			0x6A
#define LOW_POWER_ELECTRODE_SENSITIVITY		0x6B
#define CONTROL_CONFIG				0x6C
#define EVENTS					0x6D
#define AUTO_REPEAT_RATE			0x6E
#define AUTO_REPEAT_START			0x6F
#define MAX_TOUCHES				0x70

/*mask status registers*/
#define MASK_EVENTS				0x7F
#define MASK_PRESSED				0x80
#define MASK_PRESSED_OR_CAPACITIVE_EVT		0x82
#define TWO_TOUCH				0x40
#define MASK_ZOOM_DIRECTION			0x20
#define MASK_ROTATE_DIRECTION			0x10
#define MASK_SLIDE_LEFT				0x04
#define MASK_SLIDE_RIGHT			0x00
#define MASK_SLIDE_DOWN				0x0C
#define MASK_SLIDE_UP				0x08

/*mask resistive*/
#define MASK_RESISTIVE_SAMPLE			0x01
#define MASK_EVENTS_ZOOM_R			0x21
#define MASK_EVENTS_ROTATE_R			0x11
#define MASK_EVENTS_SLIDE_R			0x09

/*mask capacitive*/
#define MASK_EVENTS_CAPACITIVE			0x02
#define MASK_KEYPAD_CONF			0x60
#define MASK_SLIDE_CONF				0x20
#define MASK_ROTARY_CONF			0x10

/*dynamic registers mask*/
#define MASK_DYNAMIC_FLAG			0x80
#define MASK_DYNAMIC_DIRECTION			0x40
#define MASK_DYNAMIC_DISPLACEMENT		0x03
#define MASK_DYNAMIC_DISPLACEMENT_BTN0		0x00 
#define MASK_DYNAMIC_DISPLACEMENT_BTN1		0x01 
#define MASK_DYNAMIC_DISPLACEMENT_BTN2		0x02  
#define MASK_DYNAMIC_DISPLACEMENT_BTN3		0x03  

#define SHUTDOWN_CRICS 				0x01
#define START_CALIBRATION 			0x40
#define READ_AND_WRITE 				0666
#define KEY_NUMBER 				0x03
#define EV_TYPE					0x80
#define LEN_RESOLUTION_BYTES			0x04
#define LEN_XY					0x04
#define MAX_ZOOM_CRICS				0xFF
#define SIZE_OF_SIN_COS				0xB4
#define TRIGGUER_TOUCH_RELEASE			0x83

#define CRICS_TOUCHED				0x01
#define CRICS_RELEASED				0x00
#define SET_TRIGGER_RESISTIVE			0x81
#define CLEAN_SLIDE_EVENTS			0xEF
#define SET_MULTITOUCH				0x0C

#define IRQ_NAME	"CRTOUCH_IRQ"

#define GND		0
#define VCC		1
#define PIN_WAKE	(0 * 32 + 6) /*(bank 1 + 32 bits + pin 6)*/
#define GPIO_IRQ	(6 * 32 + 7) /*(bank 7 + 32 bits + pin 7)*/

#define DEV_NAME 	"crtouch_dev"

#define XMIN	0
#define YMIN	0

/*private struct crtouch*/
struct crtouch_data {
	int x1;
	int x2;
	int y1;
	int y2;
	int x1_old;
	int y1_old;
	struct workqueue_struct *workqueue;
	struct work_struct 	work;
	struct input_dev 	*input_dev;
	struct i2c_client 	*client;
};

	int xmax = 0;
	int ymax = 0;

	u8 data_configuration = 0;

	struct crtouch_data *crtouch;
	struct i2c_client *client_public;
	struct class *crtouch_class;
	dev_t dev_number;
	static struct cdev crtouch_cdev;
	bool status_pressed = 0;
	
	s32 rotate_angle = 0;
	int rotate_state = 0;
	int last_angle = 0;

	int sin_data[SIZE_OF_SIN_COS] = 	{
					1143,  2287,  3429,  4571,  5711,  6850,  7986,  9120,  10252, 11380, 
				    	12504, 13625, 14742, 15854, 16961, 18064, 19160, 20251, 21336, 22414,
				    	23486, 24550, 25606, 26655, 27696, 28729, 29752, 30767, 31772, 32768,
				    	33753, 34728, 35693, 36647, 37589, 38521, 39440, 40347, 41243, 42125,
				    	42995, 43852, 44695, 45525, 46340, 47142, 47929, 48702, 49460, 50203,
				    	50931, 51643, 52339, 53019, 53683, 54331, 54963, 55577, 56175, 56755,
				    	57319, 57864, 58393, 58903, 59395, 59870, 60326, 60763, 61183, 61583,
				    	61965, 62328, 62672, 62997, 63302, 63589, 63856, 64103, 64331, 64540,
				    	64729, 64898, 65047, 65176, 65286, 65376, 65446, 65496, 65526, 65535,
				    	65526, 65496, 65446, 65376, 65286, 65176, 65047, 64898, 64729, 64540,
				    	64331, 64103, 63856, 63589, 63302, 62997, 62672, 62328, 61965, 61583,
				    	61183, 60763, 60326, 59870, 59395, 58903, 58393, 57864, 57319, 56755,
				    	56175, 55577, 54963, 54331, 53683, 53019, 52339, 51643, 50931, 50203,
				    	49460, 48702, 47929, 47142, 46340, 45525, 44695, 43852, 42995, 42125,
				    	41243, 40347, 39440, 38521, 37589, 36647, 35693, 34728, 33753, 32768,
				    	31772, 30767, 29752, 28798, 27696, 26655, 25606, 24550, 23486, 22414,
				    	21336, 20251, 19160, 18064, 16961, 15854, 14742, 13625, 12504, 11380,
				    	10252, 9120,  7986,  6850,  5711,  4571,  3429,  2287,  1143,  65,
					};

	int cos_data[SIZE_OF_SIN_COS] = 	{
					65526, 65496, 65446, 65376, 65286, 65176, 65047, 64898, 64729, 64540,
					64331, 64103, 63856, 63589, 63302, 62997, 62672, 62328, 61965, 61583,
					61183, 60763, 60326, 59870, 59395, 58903, 58393, 57864, 57319, 56755,
					56175, 55577, 54963, 54331, 53683, 53019, 52339, 51643, 50931, 50203,
					49460, 48702, 47929, 47142, 46340, 45525, 44695, 43852, 42995, 42125,
				    	41243, 40347, 39440, 38521, 37589, 36647, 35693, 34728, 33753, 32768,
				    	31772, 30767, 29752, 28798, 27696, 26655, 25606, 24550, 23486, 22414,
				    	21336, 20251, 19160, 18064, 16961, 15854, 14742, 13625, 12504, 11380,
				    	10252, 9120,  7986,  6850,  5711,  4571,  3429,  2287,  1143,  65,
					-1143, -2287, -3429, -4571, -5711, -6850, -7986, -9120, -10252, -11380,
					-12504, -13625, -14742, -15854, -16961, -18064, -19160, -20251, -21336, -22414,
					-23486, -24550, -25606, -26655, -27696, -28729, -29752, -30767, -31772, -32768,
					-33753, -34728, -35693, -36647, -37589, -38521, -39440, -40347, -41243, -42125,
					-42995, -43852, -44695, -45525, -46340, -47142, -47929, -48702, -49460, -50203,
					-50931, -51643, -52339, -53019, -53683, -54331, -54963, -55577, -56175, -56755,
					-57319, -57864, -58393, -58903, -59395, -59870, -60326, -60763, -61183, -61583,
					-61965, -62328, -62672, -62997, -63302, -63589, -63856, -64103, -64331, -64540,
					-64729, -64898, -65047, -65176, -65286, -65376, -65446, -65496, -65526, -65535,  
					};
