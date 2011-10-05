/*
 * (C) Copyright 2011
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

#include <mach/stm32.h>
#include <mach/uart.h>
#include <mach/iomux.h>

#define STM32_USART3_IRQ	39

/*
 * STM32F2 RCC USART specific definitions
 */
#define STM32_RCC_ENR_USART1	offsetof(struct stm32_rcc_regs, apb2enr)
#define STM32_RCC_MSK_USART1	(1 <<  4)

#define STM32_RCC_ENR_USART2	offsetof(struct stm32_rcc_regs, apb1enr)
#define STM32_RCC_MSK_USART2	(1 << 17)

#define STM32_RCC_ENR_USART3	offsetof(struct stm32_rcc_regs, apb1enr)
#define STM32_RCC_MSK_USART3	(1 << 18)

#define STM32_RCC_ENR_USART4	offsetof(struct stm32_rcc_regs, apb1enr)
#define STM32_RCC_MSK_USART4	(1 << 19)

#define STM32_RCC_ENR_USART5	offsetof(struct stm32_rcc_regs, apb1enr)
#define STM32_RCC_MSK_USART5	(1 << 20)

#define STM32_RCC_ENR_USART6	offsetof(struct stm32_rcc_regs, apb2enr)
#define STM32_RCC_MSK_USART6	(1 <<  5)

static struct resource			stm_uart3_resources[] = {
	{
		.start	= STM32_USART3_BASE,
		.end	= STM32_USART3_BASE +
			  sizeof(struct stm32_usart_regs) - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= STM32_USART3_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device		stm_uart3_device = {
	.name			= STM32_USART_DRV_NAME,
	.id			= 2,
	.resource		= stm_uart3_resources,
	.num_resources		= 2,
};

/*
 * RCC UART register offsets
 */
static const unsigned long rcc_enr_offset[] = {
	STM32_RCC_ENR_USART1, STM32_RCC_ENR_USART2, STM32_RCC_ENR_USART3,
	STM32_RCC_ENR_USART4, STM32_RCC_ENR_USART5, STM32_RCC_ENR_USART6
};

/*
 * RCC UART clocks masks
 */
static const unsigned long rcc_msk[] = {
	STM32_RCC_MSK_USART1, STM32_RCC_MSK_USART2, STM32_RCC_MSK_USART3,
	STM32_RCC_MSK_USART4, STM32_RCC_MSK_USART5, STM32_RCC_MSK_USART6
};

/*
 * Register the STM32 specific UART devices with the kernel
 */
void __init stm32_uart_init(void)
{
	volatile struct stm32_rcc_regs	*rcc_regs;
	volatile u32			*usart_enr;
	struct stm32f2_gpio_dsc		gpio_dsc;

	rcc_regs = (struct stm32_rcc_regs *)STM32_RCC_BASE;

	/*
	 * UART3: configure TX/RX GPIOs, enable clocks, register device
	 */
	gpio_dsc.port = 2;
	gpio_dsc.pin  = 10;
	stm32f2_gpio_config(&gpio_dsc, STM32F2_GPIO_ROLE_USART3);

	gpio_dsc.port = 2;
	gpio_dsc.pin  = 11;
	stm32f2_gpio_config(&gpio_dsc, STM32F2_GPIO_ROLE_USART3);

	usart_enr = (u32 *)(STM32_RCC_BASE + rcc_enr_offset[2]);
	*usart_enr |= rcc_msk[2];

	platform_device_register(&stm_uart3_device);
}
