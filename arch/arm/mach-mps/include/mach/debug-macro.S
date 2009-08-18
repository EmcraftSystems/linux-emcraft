/* 
 * arch/arm/mach-mps/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 2009 ARM Ltd.
 *  Copyright (C) 1994-1999 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

	.macro	addruart,rx
#if 0
	mov	\rx,      #0x40000000
	orr	\rx, \rx, #0x00006000
#else
	mov	\rx, #0x1f000000
	orr	\rx, \rx, #0x00005000
#endif
	.endm

#include <asm/hardware/debug-pl01x.S>
