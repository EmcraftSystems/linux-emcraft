/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/platform.h>
#include <mach/pwm.h>
#include <mach/stm32.h>
#include <mach/dmainit.h>

#define STM32_RCC_APB1ENR_TIM5		(1 << 3)
#define STM32_TIM5_BASE			0x40000C00
#define STM32_TIM5_IRQ			50

#define STM32_RCC_APB1ENR_TIM4		(1 << 2)
#define STM32_TIM4_BASE			0x40000800
#define STM32_TIM4_IRQ			30

#if defined(CONFIG_PWM_STM32_TIM5)
static struct resource tim5_resources[] = {
	{
		.start	= STM32_TIM5_BASE,
		.end	= STM32_TIM5_BASE + 0x400 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= STM32_TIM5_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device tim5_device = {
	.name		= "stm32-pwm",
	.id		= 5,
	.resource	= tim5_resources,
	.num_resources	= ARRAY_SIZE(tim5_resources),
};

static struct stm32_pwm_data tim5_data = {
	.tmr = 5,
	.chan = 4,
	.high_on_init = true,
	.trigger_output = true,
	.period = 50000,
	.duty_cycle = 10000,
};
#endif /* CONFIG_PWM_STM32_TIM5 */

#if defined(CONFIG_STM32_ADC)
static struct resource tim4_resources[] = {
	{
		.start	= STM32_TIM4_BASE,
		.end	= STM32_TIM4_BASE + 0x400 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= STM32_TIM4_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device tim4_device = {
	.name		= "stm32-pwm",
	.id		= 4,
	.resource	= tim4_resources,
	.num_resources	= ARRAY_SIZE(tim4_resources),
};

static struct stm32_pwm_data tim4_data = {
	.tmr = 4,
	.chan = 4,
	.high_on_init = true,
	.trigger_output = true,
};
#endif /* CONFIG_STM32_ADC */


void __init stm32_pwm_init(void)
{
#if defined(CONFIG_PWM_STM32_TIM5)
	/* Enable clocks, and register platform device */
	STM32_RCC->apb1enr |= STM32_RCC_APB1ENR_TIM5;
	tim5_device.dev.platform_data = &tim5_data;
	platform_device_register(&tim5_device);
#endif /* CONFIG_PWM_STM32_TIM5 */

#if defined(CONFIG_STM32_ADC)
	/* Enable clocks, and register platform device */
	STM32_RCC->apb1enr |= STM32_RCC_APB1ENR_TIM4;
	tim4_device.dev.platform_data = &tim4_data;
	platform_device_register(&tim4_device);
#endif /* CONFIG_STM32_ADC */
}
