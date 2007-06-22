/*
 *  linux/arch/arm/lib/ucmpdi2.S
 *
 *  Author:	Nicolas Pitre
 *  Created:	Oct 19, 2005
 *  Copyright:	Monta Vista Software, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <asm/unified.h>

#include <linux/linkage.h>

#ifdef __ARMEB__
#define xh r0
#define xl r1
#define yh r2
#define yl r3
#else
#define xl r0
#define xh r1
#define yl r2
#define yh r3
#endif

	.type	__ucmpdi2, %function
ENTRY(__ucmpdi2)

	cmp	xh, yh
	it	eq
	cmpeq	xl, yl
	it	lo
	movlo	r0, #0
	it	eq
	moveq	r0, #1
	it	hi
	movhi	r0, #2
	mov	pc, lr

#ifdef CONFIG_AEABI

	.type	__aeabi_ulcmp, %function
ENTRY(__aeabi_ulcmp)

	cmp	xh, yh
	it	eq
	cmpeq	xl, yl
	it	lo
	movlo	r0, #-1
	it	eq
	moveq	r0, #0
	it	hi
	movhi	r0, #1
	mov	pc, lr

#endif

