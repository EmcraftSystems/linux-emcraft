/*
 * (C) Copyright 2018
 * Emcraft Systems, <www.emcraft.com>
 * Vladimir Skvortsov, Emcraft Systems, <vskvortsov@emcraft.com>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 */

#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/stm32.h>
#include <mach/wdt.h>

#define STM32_IWDG_BASE 0x40003000

static struct resource wdt_resources[] = {
	{
		.start	= STM32_IWDG_BASE,
		.end	= STM32_IWDG_BASE + 0x400 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device wdt_device = {
	.name = "stm32-iwdg",
	.id   = -1,
	.resource	= wdt_resources,
	.num_resources	= ARRAY_SIZE(wdt_resources),
};

void __init stm32_wdt_init(void)
{
	int rv;

	rv = platform_device_register(&wdt_device);
	if (rv != 0)
		pr_err("%s: Failed to register DWT device\n", __func__);
}
