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
 * GPIO pins used as I2C_0_SDA and I2C_0_SCL signals on the SOM-BSB baseboard.
 *
 * The same pins are used for the I2C bus on TWR-K70F120M. On this board I2C
 * is used particularly to access the resistive touchscreen installed
 * on the TWR-LCD-RGB board.
 */
static struct i2c_gpio_platform_data k70som_i2c_gpio_data = {
	.sda_pin		= KINETIS_GPIO_MKPIN(KINETIS_GPIO_PORT_E, 18),
	.scl_pin		= KINETIS_GPIO_MKPIN(KINETIS_GPIO_PORT_E, 19),
};

static struct platform_device kinetis_gpio_i2c_device = {
	.name			= "i2c-gpio",
	.id			= 0,
};

void __init kinetis_i2c_gpio_init(void)
{
	int platform;

	/*
	 * Initialize board-specific data
	 */
	platform = kinetis_platform_get();
	switch (platform) {
	case PLATFORM_KINETIS_K70_SOM:
	case PLATFORM_KINETIS_K61_SOM:
	case PLATFORM_KINETIS_TWR_K70F120M:
		kinetis_gpio_i2c_device.dev.platform_data =
			&k70som_i2c_gpio_data;
		break;
	default:
		break;
	}

	/*
	 * Register the platform device
	 */
	if (kinetis_gpio_i2c_device.dev.platform_data)
		platform_device_register(&kinetis_gpio_i2c_device);
}
