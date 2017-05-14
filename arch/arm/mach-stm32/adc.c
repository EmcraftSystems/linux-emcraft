/*
 * (C) Copyright 2017 Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * License terms: GNU General Public License (GPL), version 2
 */

#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/stm32_adc.h>

#include <mach/platform.h>
#include <mach/dmainit.h>
#include <mach/stm32.h>
#include <mach/adc.h>

#if defined(CONFIG_STM32_ADC)

#define STM32_RCC_APB2ENR_ADC(x)	(1 << (8 + (x) - 1))
#define STM32_ADC_BASE			0x40012000
#define STM32_ADC_IRQ			18

/*
 * ADC platform device resources
 */
static struct resource		adc_resources[] = {
	{
		.start	= STM32_ADC_BASE,
		.end	= STM32_ADC_BASE + 0x400 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= STM32_ADC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= STM32F7_DMACH_ADC1,
		.flags	= IORESOURCE_DMA,
	},
	{
		.start	= STM32F7_DMACH_ADC2,
		.flags	= IORESOURCE_DMA,
	},
	{
		.start	= STM32F7_DMACH_ADC3,
		.flags	= IORESOURCE_DMA,
	}
};

/*
 * ADC platform device instances
 */
static int adc1_seq[] = STM32_ADC1_SEQ;
static int adc2_seq[] = STM32_ADC2_SEQ;
static int adc3_seq[] = STM32_ADC3_SEQ;
static struct stm32_adc_platform_data	adc_data = {
	.trig_usec	= 1000,	/* Run sequence measurements each 1ms	      */
	.trig_pwm_id	= 4,	/* TIM4 id: timer triggers measurements	      */
	.trig_adc_code	= 0x5,	/* TIM4 code: TIM4_CH4 EXTSEL code	      */
	.smp		= STM32_ADC_SMP112,	/* 112 samples per chan	      */
	.adc		= {
		   {	/* ADC0 measurements */
			.meas	= STM32_ADC1_MEAS,
			.chan	= ARRAY_SIZE(adc1_seq),
			.seq	= STM32_ADC1_SEQ,
		}, {	/* ADC1 measurements */
			.meas	= STM32_ADC2_MEAS,
			.chan	= ARRAY_SIZE(adc2_seq),
			.seq	= STM32_ADC2_SEQ,
		}, {	/* ADC2 measurements */
			.meas	= STM32_ADC3_MEAS,
			.chan	= ARRAY_SIZE(adc3_seq),
			.seq	= STM32_ADC3_SEQ,
		}
	},
};

static struct platform_device	adc_dev = {
	.name		= "stm32_adc",
	.id		= -1,
	.resource	= adc_resources,
	.num_resources	= ARRAY_SIZE(adc_resources),
	.dev		= {
		.platform_data	= &adc_data,
	},
};
#endif /* CONFIG_STM32_ADC */

void __init stm32_adc_init(void)
{
#if defined(CONFIG_STM32_ADC)
	/* Enable clocks, and register platform device */
	if (STM32_ADC1_MEAS)
		STM32_RCC->apb2enr |= STM32_RCC_APB2ENR_ADC(1);
	if (STM32_ADC2_MEAS)
		STM32_RCC->apb2enr |= STM32_RCC_APB2ENR_ADC(2);
	if (STM32_ADC3_MEAS)
		STM32_RCC->apb2enr |= STM32_RCC_APB2ENR_ADC(3);
	platform_device_register(&adc_dev);
#endif
}
