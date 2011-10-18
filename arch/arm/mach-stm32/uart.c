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

/*
 * STM32 USART Interrupt numbers
 */
#define STM32_USART1_IRQ	37
#define STM32_USART2_IRQ	38
#define STM32_USART3_IRQ	39
#define STM32_USART4_IRQ	52
#define STM32_USART5_IRQ	53
#define STM32_USART6_IRQ	71

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

/*
 * USART platform device resources
 */
#define USART_PLAT_RESOURCES(uid)					       \
static struct resource			stm_usart_## uid ##_resources[] = {    \
	{								       \
		.start	= STM32_USART## uid ##_BASE,			       \
		.end	= STM32_USART## uid ##_BASE + 1,		       \
		.flags	= IORESOURCE_MEM,				       \
	},								       \
	{								       \
		.start	= STM32_USART## uid ##_IRQ,			       \
		.flags	= IORESOURCE_IRQ,				       \
	}								       \
}

/*
 * USART platform device instance
 */
#define USART_PLAT_DEVICE(uid)						       \
static struct platform_device		stm_usart_## uid ##_device = {	       \
	.name			= STM32_USART_DRV_NAME,			       \
	.id			= uid - 1,				       \
	.resource		= stm_usart_## uid ##_resources,	       \
	.num_resources		= 2,					       \
}

/*
 * Enable clocks and register platform device
 */
#define usart_init_clocks_and_register(uid) do {			       \
	volatile u32 *usart_enr;					       \
	usart_enr = (u32 *)(STM32_RCC_BASE+stm32_uart_rcc_enr_ofs[uid - 1]);   \
	*usart_enr |= stm32_uart_rcc_enr_msk[uid - 1];			       \
	platform_device_register(&stm_usart_## uid ##_device);		       \
} while (0)

/*
 * Declare the platform devices for the enabled ports
 */
#if defined(CONFIG_STM32_USART1)
USART_PLAT_RESOURCES(1);
USART_PLAT_DEVICE(1);
#endif

#if defined(CONFIG_STM32_USART2)
USART_PLAT_RESOURCES(2);
USART_PLAT_DEVICE(2);
#endif

#if defined(CONFIG_STM32_USART3)
USART_PLAT_RESOURCES(3);
USART_PLAT_DEVICE(3);
#endif

#if defined(CONFIG_STM32_USART4)
USART_PLAT_RESOURCES(4);
USART_PLAT_DEVICE(4);
#endif

#if defined(CONFIG_STM32_USART5)
USART_PLAT_RESOURCES(5);
USART_PLAT_DEVICE(5);
#endif

#if defined(CONFIG_STM32_USART6)
USART_PLAT_RESOURCES(6);
USART_PLAT_DEVICE(6);
#endif

/*
 * RCC UART register offsets
 */
static const unsigned long stm32_uart_rcc_enr_ofs[] = {
	STM32_RCC_ENR_USART1, STM32_RCC_ENR_USART2, STM32_RCC_ENR_USART3,
	STM32_RCC_ENR_USART4, STM32_RCC_ENR_USART5, STM32_RCC_ENR_USART6
};

/*
 * RCC UART clocks masks
 */
static const unsigned long stm32_uart_rcc_enr_msk[] = {
	STM32_RCC_MSK_USART1, STM32_RCC_MSK_USART2, STM32_RCC_MSK_USART3,
	STM32_RCC_MSK_USART4, STM32_RCC_MSK_USART5, STM32_RCC_MSK_USART6
};

/*
 * Register the STM32 specific UART devices with the kernel
 */
void __init stm32_uart_init(void)
{
	/*
	 * Enable clocks for the enabled ports, and register the platform devs
	 */
#if defined(CONFIG_STM32_USART1)
	usart_init_clocks_and_register(1);
#endif
#if defined(CONFIG_STM32_USART2)
	usart_init_clocks_and_register(2);
#endif
#if defined(CONFIG_STM32_USART3)
	usart_init_clocks_and_register(3);
#endif
#if defined(CONFIG_STM32_USART4)
	usart_init_clocks_and_register(4);
#endif
#if defined(CONFIG_STM32_USART5)
	usart_init_clocks_and_register(5);
#endif
#if defined(CONFIG_STM32_USART6)
	usart_init_clocks_and_register(6);
#endif
}
