/*
 *  linux/arch/arm/lib/clear_user.S
 *
 *  Copyright (C) 1995, 1996,1997,1998 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <asm/unified.h>

#include <linux/linkage.h>
#include <asm/assembler.h>

		.text

/* Prototype: int __clear_user(void *addr, size_t sz)
 * Purpose  : clear some user memory
 * Params   : addr - user memory address to clear
 *          : sz   - number of bytes to clear
 * Returns  : number of bytes NOT cleared
 */
ENTRY(__clear_user)
		stmfd	sp!, {r1, lr}
		mov	r2, #0
		cmp	r1, #4
		blt	2f
		ands	ip, r0, #3
		beq	1f
		cmp	ip, #2
ARM(USER(	strbt	r2, [r0], #1	))
THUMB(USER(	strbt	r2, [r0]	))
THUMB(		add	r0, #1		)
ARM(USER(	strlebt	r2, [r0], #1	))
		itt	le
THUMB(USER(	strlebt	r2, [r0]	))
THUMB(		addle	r0, #1		)
ARM(USER(	strltbt	r2, [r0], #1	))
		itt	lt
THUMB(USER(	strltbt	r2, [r0]	))
THUMB(		addlt	r0, #1		)
		rsb	ip, ip, #4
		sub	r1, r1, ip		@  7  6  5  4  3  2  1
1:		subs	r1, r1, #8		@ -1 -2 -3 -4 -5 -6 -7
ARM(USER(	strplt	r2, [r0], #4	))
ARM(USER(	strplt	r2, [r0], #4	))
		itttt	pl
THUMB(USER(	strplt	r2, [r0]	))
THUMB(USER(	strplt	r2, [r0, #4]	))
THUMB(		addpl	r0, #8		)
		bpl	1b
		adds	r1, r1, #4		@  3  2  1  0 -1 -2 -3
ARM(USER(	strplt	r2, [r0], #4	))
		itt	pl
THUMB(USER(	strplt	r2, [r0]	))
THUMB(		addpl	r0, #4		)
2:		tst	r1, #2			@ 1x 1x 0x 0x 1x 1x 0x
ARM(USER(	strnebt	r2, [r0], #1	))
ARM(USER(	strnebt	r2, [r0], #1	))
		ittt	ne
THUMB(USER(	strnebt	r2, [r0]	))
THUMB(USER(	strnebt	r2, [r0, #1]	))
THUMB(		addne	r0, #2		)
		tst	r1, #1			@ x1 x0 x1 x0 x1 x0 x1
		it	ne
USER(		strnebt	r2, [r0]	)
		mov	r0, #0
		ldmfd	sp!, {r1, pc}
ENDPROC(__clear_user)

		.section .fixup,"ax"
		.align	0
9001:		ldmfd	sp!, {r0, pc}
		.previous

