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
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <mach/stm32.h>
#include <mach/fb.h>
#include <mach/platform.h>

#define STM32F4_LTDC_BASE	0x40016800
#define STM32F4_LTDC_LENGTH	0x400

#define STM32F4_LTDC_IRQ	88
#define STM32F4_LTDC_ERR_IRQ	89

/*
 * Framebuffer platform device resources
 */
static struct resource kinetis_fb_resources[] = {
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
	.num_resources = ARRAY_SIZE(kinetis_fb_resources),
	.resource = kinetis_fb_resources,
	.dev = {
		.coherent_dma_mask = 0xFFFFFFFF,
		.platform_data = &stm32f4x9_fb_data,
	},
};

static int emcraft_iot_lcd_init(int init)
{
	return 0;
}

void __init stm32f4x9_fb_init(void)
{
	int device;
	int ret;

	ret = 0;

	device = stm32_device_get();
	if (device == DEVICE_STM32F439II || device == DEVICE_STM32F746NG) {
		stm32f4x9_fb_data.init = emcraft_iot_lcd_init;
		ret = platform_device_register(&stm32f4_fb_device);
	}

	if (ret < 0) {
		printk(KERN_ERR "stm32f4-fb: Could not initialize "
			"the LCD screen.\n");
	}
}
