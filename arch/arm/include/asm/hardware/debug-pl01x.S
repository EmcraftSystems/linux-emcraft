/* arch/arm/include/asm/hardware/debug-pl01x.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
#include <linux/amba/serial.h>

		.macro	inituart,rd,rx
		mov	\rd, #0x10
		str	\rd, [\rx, #UART011_IBRD]
		mov	\rd, #0xc300
		orr	\rd, #0x0001
		str	\rd, [\rx, #UART011_CR]
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx, #UART01x_DR]
		.endm

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #UART01x_FR]
		tst	\rd, #UART01x_FR_TXFF
		bne	1001b
		.endm

		.macro	busyuart,rd,rx
1001:		ldr	\rd, [\rx, #UART01x_FR]
		tst	\rd, #UART01x_FR_BUSY
		bne	1001b
		.endm
