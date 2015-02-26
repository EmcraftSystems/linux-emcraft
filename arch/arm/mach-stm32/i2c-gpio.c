/*
 * (C) Copyright 2015
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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
 */
static struct i2c_gpio_platform_data stm32f7som_i2c_gpio_data = {
	.sda_pin		= STM32_GPIO_PORTPIN2NUM(1, 7),
	.scl_pin		= STM32_GPIO_PORTPIN2NUM(1, 8),
};

static struct platform_device stm32_gpio_i2c_device = {
	.name			= "i2c-gpio",
	.id			= 0,
};

void __init stm32_i2c_gpio_init(void)
{
	int platform;

	/*
	 * Initialize board-specific data
	 */
	platform = stm32_platform_get();
	switch (platform) {
	case PLATFORM_STM32_STM_STM32F7_SOM:
		stm32_gpio_i2c_device.dev.platform_data =
			&stm32f7som_i2c_gpio_data;
		break;
	default:
		break;
	}

	/*
	 * Register the platform device
	 */
	if (stm32_gpio_i2c_device.dev.platform_data)
		platform_device_register(&stm32_gpio_i2c_device);
}
