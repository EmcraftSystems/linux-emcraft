/*
 * (C) Copyright 1994-1999
 * Russell King
 *
 * (C) Copyright 2009
 * ARM Ltd.
 *
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Vladimir Khusainiv, vlad@emcraft.com
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

/*
 * This defines UART port used for early debug messages
 * Change the value of UART_DBG_OFF to switch to UART
 * other than UARR2
 */
#define UART_BASE		0x40000000
#define UART_DBG_OFF		0x0006C000

/*
 * Minimal defintion of UART registers and flags required
 * for earch debug messaging
 */
#define UART_S1			4
#define UART_D			7
#define UART_S1_TDRE_MSK	0x80

/*
 * Kinetis-specific early debug macros
 */
	.macro		addruart,rx
	mov		\rx, #UART_BASE
	add		\rx, \rx, #UART_DBG_OFF
	.endm

	.macro		senduart, rd, rx
	strb		\rd, [\rx, #UART_D]
	.endm

	.macro		busyuart, rd, rx
1002:	ldrb		\rd, [\rx, #UART_S1]
	and		\rd, \rd, #UART_S1_TDRE_MSK
	teq		\rd, #UART_S1_TDRE_MSK
	bne		1002b
	.endm

	.macro		waituart, rd, rx
	.endm

