/* linux/include/asm-arm/arch-realview/debug-macro.S
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

#if (defined(CONFIG_MACH_REALVIEW_EB) && defined(CONFIG_MACH_REALVIEW_PB11MP)) \
	|| (defined(CONFIG_MACH_REALVIEW_EB) && defined(CONFIG_MACH_REALVIEW_PB1176)) \
	|| (defined(CONFIG_MACH_REALVIEW_PB11MP) && defined(CONFIG_MACH_REALVIEW_PB1176))
#error "Only one RealView platform can be configured if CONFIG_DEBUG_LL is enabled"
#endif

		.macro	addruart,rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx,      #0x10000000
		movne	\rx,      #0xf0000000	@ virtual base
#if defined(CONFIG_MACH_REALVIEW_EB) || defined(CONFIG_MACH_REALVIEW_PB11MP)
		orr	\rx, \rx, #0x00009000
#elif defined(CONFIG_MACH_REALVIEW_PB1176)
		orr	\rx, \rx, #0x0010C000
#else
#error "Unknown RealView platform"
#endif
		.endm

#include <asm/hardware/debug-pl01x.S>
