/*
 *  linux/arch/arm/mm/proc-v7m.S
 *
 *  Copyright (C) 2008 ARM Ltd.
 *  Copyright (C) 2001 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This is the "shell" of the ARMv7-M processor support.
 */
#include <linux/linkage.h>
#include <asm/assembler.h>

ENTRY(cpu_v7m_proc_init)
	mov	pc, lr
ENDPROC(cpu_v7m_proc_init)

ENTRY(cpu_v7m_proc_fin)
	mov	pc, lr
ENDPROC(cpu_v7m_proc_fin)

/*
 *	cpu_v7m_reset(loc)
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
ENTRY(cpu_v7m_reset)
	mov	pc, r0
ENDPROC(cpu_v7m_reset)

/*
 *	cpu_v7m_do_idle()
 *
 *	Idle the processor (eg, wait for interrupt).
 *
 *	IRQs are already disabled.
 */
ENTRY(cpu_v7m_do_idle)
	wfi
	mov	pc, lr
ENDPROC(cpu_v7m_do_idle)

ENTRY(cpu_v7m_dcache_clean_area)
	mov	pc, lr
ENDPROC(cpu_v7m_dcache_clean_area)

/*
 *	cpu_v7m_switch_mm(pgd_phys, tsk)
 *
 *	Set the translation table base pointer to be pgd_phys
 *
 *	- pgd_phys - physical address of new TTB
 *
 *	It is assumed that:
 *	- we are not using split page tables
 */
ENTRY(cpu_v7m_switch_mm)
	mov	pc, lr
ENDPROC(cpu_v7m_switch_mm)

#if 0
ENTRY(cpu_v7m_set_pte_ext)
	mov	pc, lr
ENDPROC(cpu_v7m_set_pte_ext)
#endif

cpu_v7m_name:
	.ascii	"ARMv7-M Processor"
	.align

	.section ".text.init", #alloc, #execinstr

/*
 *	__v7m_setup
 *
 *	Initialise TLB, Caches, and MMU state ready to switch the MMU
 *	on.  Return in r0 the new CP15 C1 control register setting.
 *
 *	We automatically detect if we have a Harvard cache, and use the
 *	Harvard cache control instructions insead of the unified cache
 *	control instructions.
 *
 *	This should be able to cover all ARMv7-M cores.
 *
 *	It is assumed that:
 *	- cache type register is implemented
 */
__v7m_setup:
	@ Configure the vector table base address
	ldr	r0, =0xe000ed08		@ vector table base address
	ldr	r12, =vector_table
	str	r12, [r0]

	@ Lower the priority of the SVC and PendSV exceptions
	ldr	r0, =0xe000ed1c
	mov	r5, #0x80000000
	str	r5, [r0]		@ set SVC priority
	ldr	r0, =0xe000ed20
	mov	r5, #0x00800000
	str	r5, [r0]		@ set PendSV priority

	@ SVC to run the kernel in this mode
	adr	r0, BSYM(1f)
	ldr	r5, [r12, #11 * 4]	@ read the SVC vector entry
	str	r0, [r12, #11 * 4]	@ write the temporary SVC vector entry
	mov	r6, lr			@ save LR
	mov	r7, sp			@ save SP
	ldr	sp, =__v7m_setup_stack_top
	cpsie	i
	svc	#0
1:	cpsid	i
	str	r5, [r12, #11 * 4]	@ restore the original SVC vector entry
	mov	lr, r6			@ restore LR
	mov	sp, r7			@ restore SP

	@ Special-purpose control register
	mov	r0, #1
	msr	control, r0		@ Thread mode has unpriviledged access

	@ Configure the System Control Register
	ldr	r0, =0xe000ed14		@ system control register
	ldr	r12, [r0]
	orr	r12, #1 << 9		@ STKALIGN
	str	r12, [r0]
	mov	pc, lr
ENDPROC(__v7m_setup)

	.align	2
	.type	v7m_processor_functions, #object
ENTRY(v7m_processor_functions)
	.word	nommu_early_abort
	.word	cpu_v7m_proc_init
	.word	cpu_v7m_proc_fin
	.word	cpu_v7m_reset
	.word	cpu_v7m_do_idle
	.word	cpu_v7m_dcache_clean_area
	.word	cpu_v7m_switch_mm
	.word	0			@ cpu_v7m_set_pte_ext
	.word	legacy_pabort
	.size	v7m_processor_functions, . - v7m_processor_functions

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv7m"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v7m"
	.size	cpu_elf_name, . - cpu_elf_name
	.align

	.section ".proc.info.init", #alloc, #execinstr

	/*
	 * Match any ARMv7-M processor core.
	 */
	.type	__v7m_proc_info, #object
__v7m_proc_info:
	.long	0x000f0000		@ Required ID value
	.long	0x000f0000		@ Mask for ID
	.long   0			@ proc_info_list.__cpu_mm_mmu_flags
	.long   0			@ proc_info_list.__cpu_io_mmu_flags
	b	__v7m_setup		@ proc_info_list.__cpu_flush
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_v7m_name
	.long	v7m_processor_functions	@ proc_info_list.proc
	.long	0			@ proc_info_list.tlb
	.long	0			@ proc_info_list.user
	.long	0			@ proc_info_list.cache
	.size	__v7m_proc_info, . - __v7m_proc_info

__v7m_setup_stack:
	.space	4 * 8				@ 8 registers
__v7m_setup_stack_top:
