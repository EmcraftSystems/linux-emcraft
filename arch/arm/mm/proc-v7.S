/*
 *  linux/arch/arm/mm/proc-v7.S
 *
 *  Copyright (C) 2001 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This is the "shell" of the ARMv7 processor support.
 */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/procinfo.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>

#include "proc-macros.S"

#define D_CACHE_LINE_SIZE	32

	.macro	cpsie, flags
	.ifc \flags, f
	.long	0xf1080040
	.exitm
	.endif
	.ifc \flags, i
	.long	0xf1080080
	.exitm
	.endif
	.ifc \flags, if
	.long	0xf10800c0
	.exitm
	.endif
	.err
	.endm

	.macro	cpsid, flags
	.ifc \flags, f
	.long	0xf10c0040
	.exitm
	.endif
	.ifc \flags, i
	.long	0xf10c0080
	.exitm
	.endif
	.ifc \flags, if
	.long	0xf10c00c0
	.exitm
	.endif
	.err
	.endm

ENTRY(cpu_v7_proc_init)
#ifndef CONFIG_CPU_L2CACHE_DISABLE
	@ L2 cache is enabled in the aux control register
	mrc	p15, 0, r0, c1, c0, 1
	orr	r0, r0, #2
	mcr	p15, 0, r0, c1, c0, 1
#endif
	mov	pc, lr

ENTRY(cpu_v7_proc_fin)
	mov	pc, lr

/*
 *	cpu_v7_reset(loc)
 *
 *	Perform a soft reset of the system.  Put the CPU into the
 *	same state as it would be if it had been reset, and branch
 *	to what would be the reset vector.
 *
 *	- loc   - location to jump to for soft reset
 *
 *	It is assumed that:
 */
	.align	5
ENTRY(cpu_v7_reset)
	mov	pc, r0

/*
 *	cpu_v7_do_idle()
 *
 *	Idle the processor (eg, wait for interrupt).
 *
 *	IRQs are already disabled.
 */
ENTRY(cpu_v7_do_idle)
	.long	0xe320f003			@ ARM V7 WFI instruction
	mov	pc, lr

ENTRY(cpu_v7_dcache_clean_area)
#ifndef TLB_CAN_READ_FROM_L1_CACHE
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #D_CACHE_LINE_SIZE
	subs	r1, r1, #D_CACHE_LINE_SIZE
	bhi	1b
#endif
	mov	pc, lr

/*
 *	cpu_arm926_switch_mm(pgd_phys, tsk)
 *
 *	Set the translation table base pointer to be pgd_phys
 *
 *	- pgd_phys - physical address of new TTB
 *
 *	It is assumed that:
 *	- we are not using split page tables
 */
ENTRY(cpu_v7_switch_mm)
	mov	r2, #0
	ldr	r1, [r1, #MM_CONTEXT_ID]	@ get mm->context.id
	mcr	p15, 0, r2, c7, c5, 6		@ flush BTAC/BTB
	mcr	p15, 0, r2, c7, c10, 4		@ drain write buffer
	mcr	p15, 0, r0, c2, c0, 0		@ set TTB 0
	mcr	p15, 0, r1, c13, c0, 1		@ set context ID
	mov	pc, lr

/*
 *	cpu_v7_set_pte(ptep, pte)
 *
 *	Set a level 2 translation table entry.
 *
 *	- ptep  - pointer to level 2 translation table entry
 *		  (hardware version is stored at -1024 bytes)
 *	- pte   - PTE value to store
 *
 *	Permissions:
 *	  YUWD  APX AP1 AP0	SVC	User
 *	  0xxx   0   0   0	no acc	no acc
 *	  100x   1   0   1	r/o	no acc
 *	  10x0   1   0   1	r/o	no acc
 *	  1011   0   0   1	r/w	no acc
 *	  110x   0   1   0	r/w	r/o
 *	  11x0   0   1   0	r/w	r/o
 *	  1111   0   1   1	r/w	r/w
 */
ENTRY(cpu_v7_set_pte)
	str	r1, [r0], #-2048		@ linux version

	bic	r2, r1, #0x000007f0
	bic	r2, r2, #0x00000003
	orr	r2, r2, #PTE_EXT_AP0 | 2

	tst	r1, #L_PTE_WRITE
	tstne	r1, #L_PTE_DIRTY
	orreq	r2, r2, #PTE_EXT_APX

	tst	r1, #L_PTE_USER
	orrne	r2, r2, #PTE_EXT_AP1
	tstne	r2, #PTE_EXT_APX
	bicne	r2, r2, #PTE_EXT_APX | PTE_EXT_AP0

	tst	r1, #L_PTE_YOUNG
	biceq	r2, r2, #PTE_EXT_APX | PTE_EXT_AP1 | PTE_EXT_AP0

	tst	r1, #L_PTE_EXEC
	orreq	r2, r2, #PTE_EXT_XN

	tst	r1, #L_PTE_PRESENT
	moveq	r2, #0

	str	r2, [r0]
	mcr	p15, 0, r0, c7, c10, 1 @ flush_pte
	mov	pc, lr




cpu_v7_name:
	.ascii	"ARMv7 Processor"
	.align

	.section ".text.init", #alloc, #execinstr

/*
 *	__v7_setup
 *
 *	Initialise TLB, Caches, and MMU state ready to switch the MMU
 *	on.  Return in r0 the new CP15 C1 control register setting.
 *
 *	We automatically detect if we have a Harvard cache, and use the
 *	Harvard cache control instructions insead of the unified cache
 *	control instructions.
 *
 *	This should be able to cover all ARMv7 cores.
 *
 *	It is assumed that:
 *	- cache type register is implemented
 */
__v7_setup:
	adr	r12, __v7_setup_stack		@ the local stack
	stmia	r12, {r0-r5, r7, r9, r11, lr}
	bl	v7_flush_dcache_all
	ldmia	r12, {r0-r5, r7, r9, r11, lr}
	mov	r10, #0
#ifdef HARVARD_CACHE
	mcr	p15, 0, r10, c7, c5, 0		@ I+BTB cache invalidate
#endif
	mcr	p15, 0, r10, c7, c10, 4		@ drain write buffer
	mcr	p15, 0, r10, c8, c7, 0		@ invalidate I + D TLBs
	mcr	p15, 0, r10, c2, c0, 2		@ TTB control register
	mcr	p15, 0, r4, c2, c0, 0		@ load TTB0
	mcr	p15, 0, r4, c2, c0, 1		@ load TTB1
	mov	r10, #0x1f			@ domains 0, 1 = manager
	mcr	p15, 0, r10, c3, c0, 0		@ load domain access register
#ifdef CONFIG_VFP
	mrc	p15, 0, r0, c1, c0, 2
	orr	r0, r0, #(0xf << 20)
	mcr	p15, 0, r0, c1, c0, 2		@ Enable full access to VFP
#endif

   	mrc	p15, 0, r0, c1, c0, 0		@ read control register
	ldr	r10, cr1_clear			@ get mask for bits to clear
	bic	r0, r0, r10			@ clear bits them
	ldr	r10, cr1_set			@ get mask for bits to set
	orr	r0, r0, r10			@ set them
	mov	pc, lr				@ return to head.S:__ret

	/*
	 *         V X F   I D LR
	 * .... ...E PUI. .T.T 4RVI ZFRS BLDP WCAM
	 * rrrr rrrx xxx0 0101 xxxx xxxx x111 xxxx < forced
	 *         0 110       0011 1.00 .111 1101 < we want
	 */
	.type	cr1_clear, #object
	.type	cr1_set, #object
cr1_clear:
	.word	0x0120c302
cr1_set:
	.word	0x00c0387d

__v7_setup_stack:
	.space	4 * 11				@ 11 registers

	.type	v7_processor_functions, #object
ENTRY(v7_processor_functions)
	.word	v7_early_abort
	.word	cpu_v7_proc_init
	.word	cpu_v7_proc_fin
	.word	cpu_v7_reset
	.word	cpu_v7_do_idle
	.word	cpu_v7_dcache_clean_area
	.word	cpu_v7_switch_mm
	.word	cpu_v7_set_pte
	.word	pabort_ifar
	.size	v7_processor_functions, . - v7_processor_functions

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv7"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v7"
	.size	cpu_elf_name, . - cpu_elf_name
	.align

	.section ".proc.info.init", #alloc, #execinstr

	/*
	 * Match any ARMv7 processor core.
	 */
	.type	__v7_proc_info, #object
__v7_proc_info:
	.long	0x000f0000		@ Required ID value
	.long	0x00ff0000		@ Mask for ID
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__v7_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP | HWCAP_THUMB | HWCAP_HALF | HWCAP_FAST_MULT | HWCAP_VFP
	.long	cpu_v7_name
	.long	v7_processor_functions
	.long	v6wbi_tlb_fns
	.long	v6_user_fns
	.long	v7_cache_fns
	.size	__v7_proc_info, . - __v7_proc_info
