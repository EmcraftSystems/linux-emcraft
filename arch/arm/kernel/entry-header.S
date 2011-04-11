#include <linux/init.h>
#include <linux/linkage.h>

#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/errno.h>
#include <asm/thread_info.h>

@ Bad Abort numbers
@ -----------------
@
#define BAD_PREFETCH	0
#define BAD_DATA	1
#define BAD_ADDREXCPTN	2
#define BAD_IRQ		3
#define BAD_UNDEFINSTR	4

@
@ Most of the stack format comes from struct pt_regs, but with
@ the addition of 8 bytes for storing syscall args 5 and 6.
@ This _must_ remain a multiple of 8 for EABI.
@
#define S_OFF		8

/* 
 * The SWI code relies on the fact that R0 is at the bottom of the stack
 * (due to slow/fast restore user regs).
 */
#if S_R0 != 0 && !defined(CONFIG_CPU_V7M)
#error "Please fix"
#endif

	.macro	zero_fp
#ifdef CONFIG_FRAME_POINTER
	mov	fp, #0
#endif
	.endm

	.macro	alignment_trap, rtemp
#ifdef CONFIG_ALIGNMENT_TRAP
	ldr	\rtemp, .LCcralign
	ldr	\rtemp, [\rtemp]
	mcr	p15, 0, \rtemp, c1, c0
#endif
	.endm

#ifdef CONFIG_CPU_V7M
/*
 * ARMv7-M exception entry/exit macros.
 *
 * xPSR, ReturnAddress(), LR (R14), R12, R3, R2, R1, and R0 are
 * automatically saved on the current stack (32 words) before
 * switching to the exception stack (SP_main). The order of struct
 * pt_regs members was changed to take advantage of the automatic
 * state saving.
 *
 * If exception is taken while in user mode, SP_main is
 * empty. Otherwise, SP_main is aligned to 64 bit automatically
 * (CCR.STKALIGN set).
 *
 * Linux assumes that the interrupts are disabled when entering an
 * exception handler and it may BUG if this is not the case. Interrupts
 * are disabled during entry and reenabled in the exit macro.
 *
 * The v7m_exception_entry macro preserves the original r0-r5, r7 for
 * the system call arguments.
 *
 * v7_exception_fast_exit is used when returning from interrupts.
 *
 * v7_exception_slow_exit is used when returning from SVC or PendSV.
 * When returning to kernel mode, we don't return from exception.
 */
	.macro	v7m_exception_entry
	cpsid	i
#ifdef CONFIG_VFPM
	get_thread_info r0
	and	r1, lr, #1 << 4		@ VFP clean state
	str	r1, [r0, #TI_VFPSTATE_CLEAN]
#endif
	cmp	lr, #0xfffffffd		@ check the return stack
#ifdef CONFIG_VFPM
	cmpne	lr, #0xffffffed
#endif
	beq	1f			@ exception on process stack
	add	r12, sp, #32		@ MSP before exception
	stmdb	sp!, {r4-r12, lr}	@ push unsaved registers
	b	2f
1:
	mrs	r12, psp		@ get the process stack
	sub	sp, #S_FRAME_SIZE
	stmia	sp, {r4-r12, lr}	@ push unsaved registers
	ldmia	r12, {r0-r3, r6, r8-r10} @ load automatically saved registers
	add	r12, sp, #S_R0
	stmia	r12, {r0-r3, r6, r8-r10} @ fill in the rest of struct pt_regs
2:
	.endm

	.macro	v7m_exception_fast_exit
	ldmia	sp!, {r4-r12, lr}	@ restore previously saved state
	cmp	lr, #0xfffffffd		@ check the return stack
#ifdef CONFIG_VFPM
	cmpne	lr, #0xffffffed
#endif
	addeq	sp, #S_FRAME_SIZE >> 1	@ returning to PSP, just restore MSP
	cpsie	i
	bx	lr
	.endm

	.macro	v7m_exception_slow_exit ret_r0
	cpsid	i
	ldr	lr, [sp, #S_EXC_LR]	@ read exception LR
	cmp	lr, #0xfffffffd		@ check the return stack
#ifdef CONFIG_VFPM
	cmpne	lr, #0xffffffed
#endif
	beq	1f			@ returning to PSP
	@ Prepare the MSP stack
	ldmia	sp, {r4-r11}		@ restore previously saved state
	ldr	lr, [sp, #S_PC]
	add	sp, #S_R0
	ldmia	sp, {r0-r3, r12}	@ restore the rest of registers
	add	sp, #32			@ restore the stack pointer
	cpsie	i
	bx	lr
1:
	@ Prepare the PSP stack
	ldr	r12, [sp, #S_SP]	@ read original PSP
	.if	\ret_r0
	add	r11, sp, #S_R1
	ldmia	r11, {r1-r7}		@ read state saved on MSP (r0 preserved)
	.else
	add	r11, sp, #S_R0
	ldmia	r11, {r0-r7}		@ read state saved on MSP
	.endif
	msr	psp, r12		@ restore PSP
	stmia	r12, {r0-r7}		@ restore saved state to PSP
	ldmia	sp, {r4-r11}		@ restore previously saved state
	add	sp, #S_FRAME_SIZE	@ restore the original MSP
	cpsie	i
	bx	lr
	.endm
#endif	/* CONFIG_CPU_V7M */

	@
	@ Store/load the USER SP and LR registers by switching to the SYS
	@ mode. Useful in Thumb-2 mode where "stm/ldm rd, {sp, lr}^" is not
	@ available. Should only be called from SVC mode
	@
	.macro	store_user_sp_lr, rd, rtemp, offset = 0
	mrs	\rtemp, cpsr
	eor	\rtemp, \rtemp, #(SVC_MODE ^ SYSTEM_MODE)
	msr	cpsr_c, \rtemp			@ switch to the SYS mode

	str	sp, [\rd, #\offset]		@ save sp_usr
	str	lr, [\rd, #\offset + 4]		@ save lr_usr

	eor	\rtemp, \rtemp, #(SVC_MODE ^ SYSTEM_MODE)
	msr	cpsr_c, \rtemp			@ switch back to the SVC mode
	.endm

	.macro	load_user_sp_lr, rd, rtemp, offset = 0
	mrs	\rtemp, cpsr
	eor	\rtemp, \rtemp, #(SVC_MODE ^ SYSTEM_MODE)
	msr	cpsr_c, \rtemp			@ switch to the SYS mode

	ldr	sp, [\rd, #\offset]		@ load sp_usr
	ldr	lr, [\rd, #\offset + 4]		@ load lr_usr

	eor	\rtemp, \rtemp, #(SVC_MODE ^ SYSTEM_MODE)
	msr	cpsr_c, \rtemp			@ switch back to the SVC mode
	.endm

#ifndef CONFIG_THUMB2_KERNEL
	.macro	svc_exit, rpsr
	msr	spsr_cxsf, \rpsr
#if defined(CONFIG_CPU_32v6K)
	clrex					@ clear the exclusive monitor
	ldmia	sp, {r0 - pc}^			@ load r0 - pc, cpsr
#elif defined (CONFIG_CPU_V6)
	ldr	r0, [sp]
	strex	r1, r2, [sp]			@ clear the exclusive monitor
	ldmib	sp, {r1 - pc}^			@ load r1 - pc, cpsr
#else
	ldmia	sp, {r0 - pc}^			@ load r0 - pc, cpsr
#endif
	.endm

	.macro	restore_user_regs, fast = 0, offset = 0
	ldr	r1, [sp, #\offset + S_PSR]	@ get calling cpsr
	ldr	lr, [sp, #\offset + S_PC]!	@ get pc
	msr	spsr_cxsf, r1			@ save in spsr_svc
#if defined(CONFIG_CPU_32v6K)
	clrex					@ clear the exclusive monitor
#elif defined (CONFIG_CPU_V6)
	strex	r1, r2, [sp]			@ clear the exclusive monitor
#endif
	.if	\fast
	ldmdb	sp, {r1 - lr}^			@ get calling r1 - lr
	.else
	ldmdb	sp, {r0 - lr}^			@ get calling r0 - lr
	.endif
#if __LINUX_ARM_ARCH__ < 6
	mov	r0, r0				@ ARMv5T and earlier require a nop
						@ after ldm {}^
#endif
	add	sp, sp, #S_FRAME_SIZE - S_PC
	movs	pc, lr				@ return & move spsr_svc into cpsr
	.endm

	.macro	get_thread_info, rd
	mov	\rd, sp, lsr #13
	mov	\rd, \rd, lsl #13
	.endm

	@
	@ 32-bit wide "mov pc, reg"
	@
	.macro	movw_pc, reg
	mov	pc, \reg
	.endm
#else	/* CONFIG_THUMB2_KERNEL */
	.macro	svc_exit, rpsr
	clrex					@ clear the exclusive monitor
	ldr	r0, [sp, #S_SP]			@ top of the stack
	ldr	r1, [sp, #S_PC]			@ return address
	tst	r0, #4				@ orig stack 8-byte aligned?
	stmdb	r0, {r1, \rpsr}			@ rfe context
	ldmia	sp, {r0 - r12}
	ldr	lr, [sp, #S_LR]
	addeq	sp, sp, #S_FRAME_SIZE - 8	@ aligned
	addne	sp, sp, #S_FRAME_SIZE - 4	@ not aligned
	rfeia	sp!
	.endm

#ifdef CONFIG_CPU_V7M
	.macro	restore_user_regs, fast = 0, offset = 0
	.if	\offset
	add	sp, #\offset
	.endif
	v7m_exception_slow_exit ret_r0 = \fast
	.endm
#else	/* !CONFIG_CPU_V7M */
	.macro	restore_user_regs, fast = 0, offset = 0
	clrex					@ clear the exclusive monitor
	mov	r2, sp
	load_user_sp_lr r2, r3, \offset + S_SP	@ calling sp, lr
	ldr	r1, [sp, #\offset + S_PSR]	@ get calling cpsr
	ldr	lr, [sp, #\offset + S_PC]	@ get pc
	add	sp, sp, #\offset + S_SP
	msr	spsr_cxsf, r1			@ save in spsr_svc
	.if	\fast
	ldmdb	sp, {r1 - r12}			@ get calling r1 - r12
	.else
	ldmdb	sp, {r0 - r12}			@ get calling r0 - r12
	.endif
	add	sp, sp, #S_FRAME_SIZE - S_SP
	movs	pc, lr				@ return & move spsr_svc into cpsr
	.endm
#endif	/* CONFIG_CPU_V7M */

	.macro	get_thread_info, rd
	mov	\rd, sp
	lsr	\rd, \rd, #13
	mov	\rd, \rd, lsl #13
	.endm

	@
	@ 32-bit wide "mov pc, reg"
	@
	.macro	movw_pc, reg
	mov	pc, \reg
	nop
	.endm
#endif	/* !CONFIG_THUMB2_KERNEL */

/*
 * These are the registers used in the syscall handler, and allow us to
 * have in theory up to 7 arguments to a function - r0 to r6.
 *
 * r7 is reserved for the system call number for thumb mode.
 *
 * Note that tbl == why is intentional.
 *
 * We must set at least "tsk" and "why" when calling ret_with_reschedule.
 */
scno	.req	r7		@ syscall number
tbl	.req	r8		@ syscall table pointer
why	.req	r8		@ Linux syscall (!= 0)
tsk	.req	r9		@ current thread_info
