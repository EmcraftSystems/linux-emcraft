/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * Add support for STM32F1
 * (C) Copyright 2012
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
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

#ifndef _MACH_STM32_UART_H_
#define _MACH_STM32_UART_H_

#include <mach/stm32.h>

/*
 * USART registers bases & offsets
 */
#define STM32_USART1_APBX	STM32_APB2PERITH_BASE
#define STM32_USART2_APBX	STM32_APB1PERITH_BASE
#define STM32_USART3_APBX	STM32_APB1PERITH_BASE
#define STM32_USART4_APBX	STM32_APB1PERITH_BASE
#define STM32_USART5_APBX	STM32_APB1PERITH_BASE
/* USART6 is available only on STM32F2 */
#ifndef CONFIG_ARCH_STM32F1
#define STM32_USART6_APBX	STM32_APB2PERITH_BASE
#endif

/* Different offsets on STM32F1 and STM32F2 */
#ifdef CONFIG_ARCH_STM32F1
#define STM32_USART1_OFFS	0x3800	/* STM32F1 */
#else
#define STM32_USART1_OFFS	0x1000	/* STM32F2 */
#endif
/* Offsets of registers for USARTs#2..5 are the same on STM32F1 and STM32F2 */
#define STM32_USART2_OFFS	0x4400
#define STM32_USART3_OFFS	0x4800
#define STM32_USART4_OFFS	0x4C00
#define STM32_USART5_OFFS	0x5000
/* USART6 is available only on STM32F2 */
#ifndef CONFIG_ARCH_STM32F1
#define STM32_USART6_OFFS	0x1400
#endif

/*
 * USART registers bases
 */
#define STM32_USART1_BASE	(STM32_USART1_APBX + STM32_USART1_OFFS)
#define STM32_USART2_BASE	(STM32_USART2_APBX + STM32_USART2_OFFS)
#define STM32_USART3_BASE	(STM32_USART3_APBX + STM32_USART3_OFFS)
#define STM32_USART4_BASE	(STM32_USART4_APBX + STM32_USART4_OFFS)
#define STM32_USART5_BASE	(STM32_USART5_APBX + STM32_USART5_OFFS)
#define STM32_USART6_BASE	(STM32_USART6_APBX + STM32_USART6_OFFS)

/*
 * USART used for early printks and in uncompress.h. Note, it's supposed
 * that this USART had been previously configured, and inited in boot-
 * loader
 */
#ifdef CONFIG_ARCH_STM32F1
/* STM32F1 hardware reference is the SwissEmbedded Comm board */
#define STM32_DBG_USART_BASE	STM32_USART1_BASE
#else
/* STM32F2 hardware reference is the STM3220G-EVAL board */
#define STM32_DBG_USART_BASE	STM32_USART3_BASE
#endif

/*
 * SR bit masks (these are only those, which used in debug/uncompress code
 * also; see stm32_usart.c for the others)
 */
#define STM32_USART_SR_CTS	(1 << 9)	/* Clear to send	     */
#define STM32_USART_SR_TXE	(1 << 7)	/* Transmit data reg empty   */


/*
 * Register offsets for use in debug-asm, and uncompress prints
 */
#define STM32_UART_SR		0
#define STM32_UART_DR		4

#ifndef __ASSEMBLY__

#include <linux/init.h>

#define STM32_USART_DRV_NAME	"stm32serial"

void __init stm32_uart_init(void);

#endif /* __ASSEMBLY__ */

#endif	/*_MACH_STM32_UART_H_ */
