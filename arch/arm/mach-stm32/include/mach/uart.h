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
#define STM32_USART6_APBX	STM32_APB2PERITH_BASE

#define STM32_USART1_OFFS	0x1000
#define STM32_USART2_OFFS	0x4400
#define STM32_USART3_OFFS	0x4800
#define STM32_USART4_OFFS	0x4C00
#define STM32_USART5_OFFS	0x5000
#define STM32_USART6_OFFS	0x1400

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
#define STM32_DBG_USART_APBX	STM32_USART3_APBX
#define STM32_DBG_USART_OFFS	STM32_USART3_OFFS

/*
 * SR bit masks
 */
#define STM32_USART_SR_CTS	(1 << 9)
#define STM32_USART_SR_TXE	(1 << 7)	/* Transmit data reg empty   */
#define STM32_USART_SR_RXNE	(1 << 5)	/* Read data reg not empty   */
#define STM32_USART_SR_ORE	(1 << 3)	/* Overrun error	     */
#define STM32_USART_SR_FE	(1 << 1)	/* Framing error	     */
#define STM32_USART_SR_PE	(1 << 0)	/* Parity error		     */
#define STM32_USART_SR_ERRORS	(STM32_USART_SR_ORE | STM32_USART_SR_FE |     \
				 STM32_USART_SR_PE)

#ifndef __ASSEMBLY__

#include <linux/init.h>
#include <mach/iomux.h>

#define STM32_USART_DRV_NAME	"stm32serial"

/*
 * USART register map
 */
struct stm32_usart_regs {
	u16	sr;		/* Status				      */
	u16	rsv0;
	u16	dr;		/* Data					      */
	u16	rsv1;
	u16	brr;		/* Baud rate				      */
	u16	rsv2;
	u16	cr1;		/* Control 1				      */
	u16	rsv3;
	u16	cr2;		/* Control 2				      */
	u16	rsv4;
	u16	cr3;		/* Control 3				      */
	u16	rsv5;
	u16	gtpr;		/* Guard time and prescaler		      */
};

void __init stm32_uart_init(void);

#else

/*
 * Register offsets for use in asm
 */
#define STM32_UART_SR		0
#define STM32_UART_DR		4

#endif /* __ASSEMBLY__ */

#endif	/*_MACH_STM32_UART_H_ */
