/*
 * include/asm-arm/unified.h - Unified Assembler Syntax helper macros
 *
 * Copyright (C) 2007 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __ASM_UNIFIED_H
#define __ASM_UNIFIED_H

#if defined(__ASSEMBLY__) && defined(CONFIG_ARM_ASM_UNIFIED)
	.syntax unified
#endif

#ifdef CONFIG_THUMB2_KERNEL

#if __GNUC__ < 4
#error Thumb-2 kernel requires gcc >= 4
#endif

#define PSR_ISETSTATE	PSR_T_BIT

#define ARM(x...)
#define THUMB(x...)	x
#define W(instr)	instr.w

#ifdef __ASSEMBLY__
	/* Thumb function address */
	.macro badr, reg, func, cond=al
	adr\cond \reg, \func + 1
	.endm
#endif

#else	/* !CONFIG_THUMB2_KERNEL */

#define PSR_ISETSTATE	0

#define ARM(x...)	x
#define THUMB(x...)
#define W(instr)	instr

#ifdef __ASSEMBLY__
	/* ARM branch address */
	.macro	badr, reg, func, cond=al
	adr\cond \reg, \func
	.endm
#endif

#endif	/* CONFIG_THUMB2_KERNEL */

#ifndef CONFIG_ARM_ASM_UNIFIED

#ifdef __ASSEMBLY__
	.macro	it, cond
	.endm
	.macro	itt, cond
	.endm
	.macro	ite, cond
	.endm
	.macro	ittt, cond
	.endm
	.macro	itte, cond
	.endm
	.macro	itet, cond
	.endm
	.macro	itee, cond
	.endm
	.macro	itttt, cond
	.endm
	.macro	ittte, cond
	.endm
	.macro	ittet, cond
	.endm
	.macro	ittee, cond
	.endm
	.macro	itett, cond
	.endm
	.macro	itete, cond
	.endm
	.macro	iteet, cond
	.endm
	.macro	iteee, cond
	.endm
#else	/* !__ASSEMBLY__ */
__asm__(
"	.macro	it, cond\n"
"	.endm\n"
"	.macro	itt, cond\n"
"	.endm\n"
"	.macro	ite, cond\n"
"	.endm\n"
"	.macro	ittt, cond\n"
"	.endm\n"
"	.macro	itte, cond\n"
"	.endm\n"
"	.macro	itet, cond\n"
"	.endm\n"
"	.macro	itee, cond\n"
"	.endm\n"
"	.macro	itttt, cond\n"
"	.endm\n"
"	.macro	ittte, cond\n"
"	.endm\n"
"	.macro	ittet, cond\n"
"	.endm\n"
"	.macro	ittee, cond\n"
"	.endm\n"
"	.macro	itett, cond\n"
"	.endm\n"
"	.macro	itete, cond\n"
"	.endm\n"
"	.macro	iteet, cond\n"
"	.endm\n"
"	.macro	iteee, cond\n"
"	.endm\n");
#endif	/* __ASSEMBLY__ */

#endif	/* CONFIG_ARM_ASM_UNIFIED */

#endif	/* !CONFIG_ARM_ASM_UNIFIED */
