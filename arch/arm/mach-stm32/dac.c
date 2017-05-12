/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/platform.h>
#include <mach/dac.h>
#include <mach/stm32.h>
#include <mach/dmainit.h>

#define STM32_RCC_APB1ENR_DAC		(1 << 29)
#define STM32_DAC_BASE			0x40007400

static struct resource dac_resources[] = {
	{
		.start	= STM32_DAC_BASE,
		.end	= STM32_DAC_BASE + 0x400 - 1,
		.flags	= IORESOURCE_MEM,
	},
#if defined(CONFIG_STM32_DAC1_DMA)
	{
		.name	= "dac1_dma_tx_channel",
		.start	= STM32F7_DMACH_DAC1,
		.flags	= IORESOURCE_DMA,
	},
#endif /* CONFIG_STM32_DAC1_DMA */
#if defined(CONFIG_STM32_DAC2_DMA)
	{
		.name	= "dac2_dma_tx_channel",
		.start	= STM32F7_DMACH_DAC2,
		.flags	= IORESOURCE_DMA,
	},
#endif /* CONFIG_STM32_DAC2_DMA */
};

static struct platform_device dac_device = {
	.name		= "stm32_dac",
	.id		= 0,
	.resource	= dac_resources,
	.num_resources	= ARRAY_SIZE(dac_resources),
};

void __init stm32_dac_init(void)
{
#if defined(CONFIG_STM32_DAC1)
	dac_device.id |= STM32_DAC_ID_1;
#endif

#if defined(CONFIG_STM32_DAC2)
	dac_device.id |= STM32_DAC_ID_2;
#endif

#if defined(CONFIG_STM32_DAC_TIMER_ID)
	dac_device.id |= STM32_TIMER_ID(CONFIG_STM32_DAC_TIMER_ID);
#endif

#if defined(CONFIG_STM32_DAC1) || defined(CONFIG_STM32_DAC2)
	/* Enable clocks, and register platform device */
	STM32_RCC->apb1enr |= STM32_RCC_APB1ENR_DAC;
	platform_device_register(&dac_device);
#endif
}
