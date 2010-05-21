/*
 * arch/arm/mach-vexpress/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DEBUG_LL_UART_OFFSET
#define DEBUG_LL_UART_OFFSET	0x00009000
#elif DEBUG_LL_UART_OFFSET != 0x00009000
#warning "DEBUG_LL_UART_OFFSET already defined to a different value"
#endif

#ifndef DEBUG_LL_UART_OFFSET
#error "Unknown Versatile Express platform"
#endif

		.macro	addruart,rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		ite	eq
		moveq	\rx,      #0x10000000
		movne	\rx,      #0xfb000000	@ virtual base
		orr	\rx, \rx, #DEBUG_LL_UART_OFFSET
		.endm

#include <asm/hardware/debug-pl01x.S>
