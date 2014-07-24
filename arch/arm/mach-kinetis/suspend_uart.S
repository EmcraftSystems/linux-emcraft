/*
 * arch/arm/mach-kinetis/suspend_uart.S
 * Kinetis low-level suspend-to-RAM code, UART interface
 *
 * Copyright (C) 2014 Emcraft Systems
 * Vladimir Khusainov, Emcraft Systems, <vlad@emcraft.com>
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

	.macro uart_before_suspend uart_base, uart_bdr

	ldr r0, \uart_base
	ldrb r1, [r0, #3]
	and r1, #0xF3
	strb r1, [r0, #3]

	adr.w r2, \uart_bdr
	ldr r1, [r2, #3]
	strb r1, [r0, #0]
	ldr r1, [r2, #4]
	strb r1, [r0, #1]
	ldr r1, [r2, #5]
	strb r1, [r0, #10]

	ldrb r1, [r0, #3]
	orr r1, #0x0C
	strb r1, [r0, #3]

	.endm

	.macro uart_after_suspend uart_base, uart_cnt, uart_buf, uart_bdr

	ldr r0, \uart_base
	ldrb r1, [r0, #5]
	and r1, #0x40
	cmp r1, #0x40
	bne 3f

	/*
	 * Disable RIE
	 */
	ldrb r1, [r0, #3]
	and r1, #0xDF
	strb r1, [r0, #3]

	/*
	 * Count the number of Rx characters before UART Rx goes IDLE
	 */
	mov r3, #0x0

	/*
	 * Wait for either IDLE or RDRF
	 */
1:
	ldrb r1, [r0, #4]
	tst r1, #0x30
	beq 1b

	/*
	 * If we got IDLE, we are done
	 */
	tst r1, #0x10
	bne 2f

	/*
	 * Next char has been received
	 * Read and store it
	 */
	ldrb r2, [r0, #7]
	adr.w r1, \uart_buf
	strb r2, [r1, r3]
	add r3, #0x1

	b 1b

2:
	/*
	 * Store the number of read characters
	 */
	adr.w r1, \uart_cnt
	str r3, [r1]

	/*
	 * Enable RIE
	 */
	ldrb r1, [r0, #3]
	orr r1, #0x20
	strb r1, [r0, #3]

3:

	/*
	 * Switch UART baud rates back to the normal operational rates.
	 */
	ldrb r1, [r0, #3]
	and r1, #0xF3
	strb r1, [r0, #3]

	adr.w r2, \uart_bdr
	ldr r1, [r2, #0]
	strb r1, [r0, #0]
	ldr r1, [r2, #1]
	strb r1, [r0, #1]
	ldr r1, [r2, #2]
	strb r1, [r0, #10]

	ldrb r1, [r0, #3]
	orr r1, #0x0C
	strb r1, [r0, #3]

	.endm

	.macro uart_declare uart_cnt, uart_buf, uart_bdr

	.global \uart_cnt
	.global \uart_buf
	.global \uart_bdr

	.endm

	.macro uart_defs uart_base, uart_addr, uart_cnt, uart_buf, uart_bdr

\uart_base:
	.word \uart_addr
\uart_cnt:
	.word 0x0
\uart_bdr:
	.space 8
\uart_buf:
	.space 256

	.endm

