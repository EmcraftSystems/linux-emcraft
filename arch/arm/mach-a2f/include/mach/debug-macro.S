/* 
 * arch/arm/mach-a2f/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 2010,2011 Vladimir Khusainov, Emcraft Systems
 *  Copyright (C) 2009 ARM Ltd.
 *  Copyright (C) 1994-1999 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/uart.h>

	.macro		addruart,rx
	ldr		\rx, =MSS_UART0_BASE
	.endm

#define UART_SHIFT      2
#include <asm/hardware/debug-8250.S>
