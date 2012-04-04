/*
 * (C) Copyright 1994-1999
 * Russell King
 *
 * (C) Copyright 2009
 * ARM Ltd.
 *
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

#include <mach/uart.h>

	/*
	 * Use the `ldr` instruction to load an arbitrary 32-bit value to \rx
	 */
	.macro	addruart, rx
	ldr	\rx, =STM32_DBG_USART_BASE
	.endm

	.macro	senduart, rd, rx
	strb	\rd, [\rx, #STM32_UART_DR]
	.endm

	.macro	busyuart, rd, rx
1002:	ldr	\rd, [\rx, #STM32_UART_SR]
	tst	\rd, #STM32_USART_SR_TXE
	beq	1002b
	.endm

	.macro	waituart, rd, rx
#ifdef FLOW_CONTROL
1001:	ldr	\rd, [\rx, #STM32_USART_SR]
	tst	\rd, #STM32_USART_SR_CTS
	beq	1001b
#endif
	.endm
