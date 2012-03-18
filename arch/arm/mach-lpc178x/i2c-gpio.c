/*
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>

#include <mach/i2c-gpio.h>
#include <mach/platform.h>
#include <mach/gpio.h>

/*
 * GPIO pins connected to the main I2C bus on the EA-LPC1788 board.
 */
static struct i2c_gpio_platform_data ea_lpc1788_i2c_gpio_data = {
	.sda_pin		= LPC178X_GPIO_MKPIN(0, 27),
	.scl_pin		= LPC178X_GPIO_MKPIN(0, 28),
};

static struct platform_device lpc178x_gpio_i2c_device = {
	.name			= "i2c-gpio",
	.id			= 0,
};

void __init lpc178x_i2c_gpio_init(void)
{
	int platform;

	/*
	 * Initialize board-specific data
	 */
	platform = lpc178x_platform_get();
	switch (platform) {
	case PLATFORM_LPC178X_EA_LPC1788:
		lpc178x_gpio_i2c_device.dev.platform_data =
			&ea_lpc1788_i2c_gpio_data;
		break;
	case PLATFORM_LPC178X_LNX_EVB:
		/* Do not use I2C for now on LPC-LNX-EVB */
		break;
	default:
		break;
	}

	/*
	 * Register the platform device
	 */
	if (lpc178x_gpio_i2c_device.dev.platform_data)
		platform_device_register(&lpc178x_gpio_i2c_device);
}
