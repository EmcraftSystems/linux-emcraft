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
#include <mach/dmaregs.h>

/*
 * USART RX DMAs
 */
#ifdef CONFIG_ARCH_STM32F1
/* STM32F1 */
#define STM32_USART1_DMA_BASE	STM32_DMA1_BASE
#define STM32_USART2_DMA_BASE	STM32_DMA1_BASE
#define STM32_USART3_DMA_BASE	STM32_DMA1_BASE
#define STM32_USART4_DMA_BASE	STM32_DMA2_BASE
#else
/* STM32F2 */
#define STM32_USART1_DMA_BASE	STM32_DMA2_BASE
#define STM32_USART2_DMA_BASE	STM32_DMA1_BASE
#define STM32_USART3_DMA_BASE	STM32_DMA1_BASE
#define STM32_USART4_DMA_BASE	STM32_DMA1_BASE
/* USART5 and USART6 with DMA support are available only on STM32F2 */
#define STM32_USART5_DMA_BASE	STM32_DMA1_BASE
#define STM32_USART6_DMA_BASE	STM32_DMA2_BASE
#endif

/*
 * STM32 USART Interrupt numbers
 */
#define STM32_USART1_IRQ	37
#define STM32_USART2_IRQ	38
#define STM32_USART3_IRQ	39
#define STM32_USART4_IRQ	52
#define STM32_USART5_IRQ	53
/* USART6 is available only on STM32F2 */
#ifndef CONFIG_ARCH_STM32F1
#define STM32_USART6_IRQ	71
#endif

/*
 * USART RX DMAs Interrupt numbers
 */
#ifdef CONFIG_ARCH_STM32F1
/* STM32F1 */
#define STM32_USART1_DMA_IRQ	15	/* DMA1 Channel5 */
#define STM32_USART2_DMA_IRQ	16	/* DMA1 Channel6 */
#define STM32_USART3_DMA_IRQ	13	/* DMA1 Channel3 */
#define STM32_USART4_DMA_IRQ	58	/* DMA2 Channel3 */
#else
/* STM32F2 */
#define STM32_USART1_DMA_IRQ	68
#define STM32_USART2_DMA_IRQ	16
#define STM32_USART3_DMA_IRQ	12
#define STM32_USART4_DMA_IRQ	13
#define STM32_USART5_DMA_IRQ	11
#define STM32_USART6_DMA_IRQ	58
#endif

/*
 * STM32F2 RCC USART specific definitions
 */
#define STM32_RCC_ENR_USART1	offsetof(struct stm32_rcc_regs, apb2enr)
#ifdef CONFIG_ARCH_STM32F1
#define STM32_RCC_MSK_USART1	(1 << 14)	/* STM32F1 */
#else
#define STM32_RCC_MSK_USART1	(1 <<  4)	/* STM32F2 */
#endif

/* Positions of USART_EN bits for USARTs#2..5 are the same on STM32F1 and STM32F2 */
#define STM32_RCC_ENR_USART2	offsetof(struct stm32_rcc_regs, apb1enr)
#define STM32_RCC_MSK_USART2	(1 << 17)

#define STM32_RCC_ENR_USART3	offsetof(struct stm32_rcc_regs, apb1enr)
#define STM32_RCC_MSK_USART3	(1 << 18)

#define STM32_RCC_ENR_USART4	offsetof(struct stm32_rcc_regs, apb1enr)
#define STM32_RCC_MSK_USART4	(1 << 19)

#define STM32_RCC_ENR_USART5	offsetof(struct stm32_rcc_regs, apb1enr)
#define STM32_RCC_MSK_USART5	(1 << 20)

/* USART6 is available only on STM32F2 */
#ifndef CONFIG_ARCH_STM32F1
#define STM32_RCC_ENR_USART6	offsetof(struct stm32_rcc_regs, apb2enr)
#define STM32_RCC_MSK_USART6	(1 <<  5)
#endif

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
		.start	= STM32_USART## uid ##_DMA_BASE,		       \
		.end	= STM32_USART## uid ##_DMA_BASE + 1,		       \
		.flags	= IORESOURCE_MEM,				       \
	},								       \
	{								       \
		.start	= STM32_USART## uid ##_IRQ,			       \
		.flags	= IORESOURCE_IRQ,				       \
	},								       \
	{								       \
		.start	= STM32_USART## uid ##_DMA_IRQ,			       \
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
	.num_resources		= 4,					       \
}

/*
 * Enable clocks for USART & DMA, and register platform device
 */
#define usart_init_clocks_and_register(uid) do {			       \
	volatile u32	*usart_enr;					       \
	usart_enr = (u32 *)(STM32_RCC_BASE + stm32_uart_rcc_enr_ofs[uid - 1]); \
	*usart_enr |= stm32_uart_rcc_enr_msk[uid - 1];			       \
	usart_enr = (u32 *)(STM32_RCC_BASE + offsetof(struct stm32_rcc_regs,   \
						    ahb1enr));		       \
	*usart_enr |= (STM32_USART## uid ##_DMA_BASE ==			       \
		       STM32_DMA1_BASE) ? STM32_RCC_AHB1ENR_DMA1_MSK :	       \
					  STM32_RCC_AHB1ENR_DMA2_MSK;	       \
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
	STM32_RCC_ENR_USART4,
#ifndef CONFIG_ARCH_STM32F1
	STM32_RCC_ENR_USART5, STM32_RCC_ENR_USART6
#endif
};

/*
 * RCC UART clocks masks
 */
static const unsigned long stm32_uart_rcc_enr_msk[] = {
	STM32_RCC_MSK_USART1, STM32_RCC_MSK_USART2, STM32_RCC_MSK_USART3,
	STM32_RCC_MSK_USART4,
#ifndef CONFIG_ARCH_STM32F1
	STM32_RCC_MSK_USART5, STM32_RCC_MSK_USART6
#endif
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
