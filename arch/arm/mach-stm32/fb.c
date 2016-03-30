/*
 * (C) Copyright 2015
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
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/stm32.h>
#include <mach/fb.h>
#include <mach/platform.h>
#include <mach/gpio.h>

#define STM32F4_LTDC_IRQ	88
#define STM32F4_LTDC_ERR_IRQ	89

/*
 * Framebuffer platform device resources
 */
static struct resource stm32f4_fb_resources[] = {
	{
		.start	= STM32F4_LTDC_BASE,
		.end	= STM32F4_LTDC_BASE + STM32F4_LTDC_LENGTH - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= STM32F4_LTDC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct stm32f4_fb_platform_data stm32f4x9_fb_data = {
	.mode_str	= "480x272",
	.modes		= NULL,
};

static struct platform_device stm32f4_fb_device = {
	.name = "stm32f4-ltdc",
	.id = 0,
	.num_resources = ARRAY_SIZE(stm32f4_fb_resources),
	.resource = stm32f4_fb_resources,
	.dev = {
		.coherent_dma_mask = 0xFFFFFFFF,
		.platform_data = &stm32f4x9_fb_data,
	},
};

#if defined(CONFIG_TOUCHSCREEN_CRTOUCH_MT)
/*
 * I2C-connected Freescale CRTouch touchscreen device.
 * We register it with the driver `crtouch_mt`.
 */
static struct i2c_board_info __initdata emcraft_iot_lcd_crtouch = {
	I2C_BOARD_INFO("crtouchId", 0x49),
};
#endif /* CONFIG_TOUCHSCREEN_CRTOUCH_MT */

static int stm32f4x9_fb_lcd_init(int init)
{
	int p = stm32_platform_get();

	if (p == PLATFORM_STM32_STM32F7_DISCO) {
		gpio_direction_output(STM32_GPIO_PORTPIN2NUM(7, 3), 0);
	}

#if defined(CONFIG_TOUCHSCREEN_CRTOUCH_MT)
	/*
	 * Set wake-up active
	 */
	else if (p == PLATFORM_STM32_STM_STM32F7_SOM)
		gpio_direction_output(STM32_GPIO_PORTPIN2NUM(7, 3), 0);
#endif

	return 0;
}

void __init stm32f4x9_fb_init(void)
{
	int device;
	int ret;

	ret = 0;

	device = stm32_device_get();
	if (device == DEVICE_STM32F439II || device == DEVICE_STM32F746NG) {
#if defined(CONFIG_TOUCHSCREEN_CRTOUCH_MT)
		/*
		 * Register the I2C-connected CRTouch touchscreen
		 */
		i2c_register_board_info(0, &emcraft_iot_lcd_crtouch, 1);
#endif
		stm32f4x9_fb_data.init = stm32f4x9_fb_lcd_init;
		ret = platform_device_register(&stm32f4_fb_device);
	}

	if (ret < 0) {
		printk(KERN_ERR "stm32f4-fb: Could not initialize "
			"the LCD screen.\n");
	}
}
