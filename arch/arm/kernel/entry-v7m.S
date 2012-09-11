/*
 * linux/arch/arm/kernel/entry-v7m.S
 *
 * Copyright (C) 2008 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Low-level vector interface routines for the ARMv7-M architecture
 */
#include <asm/memory.h>
#include <asm/glue.h>
#include <asm/thread_notify.h>

#include <mach/irqs.h>
#include <mach/entry-macro.S>

#include "entry-header.S"

#ifdef CONFIG_PREEMPT
#error "CONFIG_PREEMPT not supported on the current ARMv7M implementation"
#endif
#ifdef CONFIG_TRACE_IRQFLAGS
#error "CONFIG_TRACE_IRQFLAGS not supported on the current ARMv7M implementation"
#endif

__invalid_entry:
	v7m_exception_entry
	adr	r0, strerr
	mrs	r1, ipsr
	mov	r2, lr
#ifdef CONFIG_PRINTK
	bl	printk
#endif
	mov	r0, sp
	bl	show_regs
	mov	r0, 0
	bl	panic
1:	b	1b
ENDPROC(__invalid_entry)

strerr:	.asciz	"\nUnhandled exception: IPSR = %08lx LR = %08lx\n"

	.align	2
__irq_entry:
	v7m_exception_entry

	@
	@ Invoke the IRQ handler
	@
	mrs	r0, ipsr
	and	r0, #0xff
	sub	r0, #16			@ IRQ number
	mov	r1, sp
	@ routine called with r0 = irq number, r1 = struct pt_regs *
	bl	asm_do_IRQ

	@
	@ Check for any pending work if returning to user
	@
	ldr	lr, [sp, #S_EXC_LR]
	cmp	lr, #0xfffffffd		@ check the return stack
	bne	2f			@ returning to kernel
	get_thread_info tsk
	ldr	r1, [tsk, #TI_FLAGS]
	tst	r1, #_TIF_WORK_MASK
	beq	2f			@ no work pending
	ldr	r1, =0xe000ed04		@ ICSR
	mov	r0, #1 << 28		@ ICSR.PENDSVSET
	str	r0, [r1]		@ raise PendSV

2:
	v7m_exception_fast_exit
ENDPROC(__irq_entry)

__pendsv_entry:
	v7m_exception_entry

	ldr	r1, =0xe000ed04		@ ICSR
	mov	r0, #1 << 27		@ ICSR.PENDSVCLR
	str	r0, [r1]		@ clear PendSV

	@ execute the pending work, including reschedule
	get_thread_info tsk
	mov	why, #0
	b	ret_to_user
ENDPROC(__pendsv_entry)

	.align  2
__hardfault_entry:
	v7m_exception_entry

	@
	@ Invoke the Hard Fault handler
	@ routine called with r0 = struct pt_regs *
	mov	r0, sp
	bl	do_hardfault

	@ execute the pending work, including reschedule
	get_thread_info tsk
	mov	why, #0
	b	ret_to_user

        @ v7m_exception_fast_exit
ENDPROC(__hardfault_entry)

#ifdef CONFIG_MPU
	.align	2
__memmanage_entry:
	v7m_exception_entry

	@
	@ Invoke the Memory Fault handler
	@ routine called with r0 = struct pt_regs *
	mov	r0, sp
	bl	do_memmanage

	@ execute the pending work, including reschedule
	get_thread_info tsk
	mov	why, #0
	b	ret_to_user

	@ v7m_exception_fast_exit
ENDPROC(__memmanage_entry)
#endif /* CONFIG_MPU */

	.align  2
__busfault_entry:
	v7m_exception_entry

	@
	@ Invoke the Bus Fault handler
	@ routine called with r0 = struct pt_regs *
	mov	r0, sp
	bl	do_busfault

	@ execute the pending work, including reschedule
	get_thread_info tsk
	mov	why, #0
	b	ret_to_user

	@ v7m_exception_fast_exit
ENDPROC(__busfault_entry)

	.align  2
__usagefault_entry:
	v7m_exception_entry

	@
	@ Invoke the Bus Fault handler
	@ routine called with r0 = struct pt_regs *
	mov	r0, sp
	bl	do_usagefault

	@ execute the pending work, including reschedule
	get_thread_info tsk
	mov	why, #0
	b	ret_to_user

	@ v7m_exception_fast_exit
ENDPROC(__usagefault_entry)

/*
 * Register switch for ARMv7-M processors.
 * r0 = previous task_struct, r1 = previous thread_info, r2 = next thread_info
 * previous and next are guaranteed not to be the same.
 */
ENTRY(__switch_to)
	add	ip, r1, #TI_CPU_SAVE
	stmia	ip!, {r4 - sl, fp}	@ Store most regs on stack
	str	sp, [ip], #4
	str	lr, [ip], #4
	mov	r5, r0
	add	r4, r2, #TI_CPU_SAVE
	ldr	r0, =thread_notify_head
	mov	r1, #THREAD_NOTIFY_SWITCH
	bl	atomic_notifier_call_chain
	mov	ip, r4
	mov	r0, r5
	ldmia	ip!, {r4 - sl, fp}	@ Load all regs saved previously
	ldr	sp, [ip], #4
	ldr	pc, [ip]
ENDPROC(__switch_to)

	.data
	.align	8
/*
 * Vector table
 */
ENTRY(vector_table)
	.long	0			@ 0 - Reset stack pointer
	.long	__invalid_entry		@ 1 - Reset
	.long	__invalid_entry		@ 2 - NMI
	.long	__hardfault_entry	@ 3 - HardFault
#ifdef CONFIG_MPU
	.long	__memmanage_entry	@ 4 - MemManage
#else
	.long	__invalid_entry		@ 4 - MemManage
#endif
	.long	__busfault_entry	@ 5 - BusFault
	.long	__usagefault_entry	@ 6 - UsageFault
	.long	__invalid_entry		@ 7 - Reserved
	.long	__invalid_entry		@ 8 - Reserved
	.long	__invalid_entry		@ 9 - Reserved
	.long	__invalid_entry		@ 10 - Reserved
	.long	vector_swi		@ 11 - SVCall
	.long	__invalid_entry		@ 12 - Debug Monitor
	.long	__invalid_entry		@ 13 - Reserved
	.long	__pendsv_entry		@ 14 - PendSV
	.long	__invalid_entry		@ 15 - SysTick
	.rept	NR_IRQS
	.long	__irq_entry		@ 16.. - NR_IRQS External Interrupts
	.endr
#if defined(CONFIG_ARM_CORTEXM3)
ENTRY(vector_table_end)
#endif
