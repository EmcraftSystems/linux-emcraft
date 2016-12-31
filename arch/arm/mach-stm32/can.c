/*
 * (C) Copyright 2016
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 */

#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/platform.h>
#include <mach/can.h>
#include <mach/stm32.h>

#if defined(CONFIG_STM32_CAN1)

#define STM32_RCC_APB1ENR_CAN1		(1 << 25)
#define STM32_CAN1_BASE			0x40006400
#define STM32_CAN1_IRQ			19

/*
 * CAN1 platform device resources
 */
static struct resource		can1_resources[] = {
	{
		.start	= STM32_CAN1_BASE,
		.end	= STM32_CAN1_BASE + 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= STM32_CAN1_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

/*
 * CAN1 platform device instance
 */
static struct platform_device	can1_device = {
	.name		= "stm32_bxcan",
	.id		= -1,
	.resource	= can1_resources,
	.num_resources	= ARRAY_SIZE(can1_resources),
};
#endif /* CONFIG_STM32_CAN1 */

void __init stm32_can_init(void)
{
#if defined(CONFIG_STM32_CAN1)
	/* Enable clocks, and register platform device */
	STM32_RCC->apb1enr |= STM32_RCC_APB1ENR_CAN1;
	platform_device_register(&can1_device);
#endif
}
