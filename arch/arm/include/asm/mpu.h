/*
 *  arch/arm/include/asm/mpu.h
 *
 *  Copyright (C) 2011 Vladimir Khusainov, Emcraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __ASM_ARM_MPU_H
#define __ASM_ARM_MPU_H

#include <linux/compiler.h>
#include <linux/mm_types.h>

/*
 * Define a mappable region
 */
extern void mpu_region_define(unsigned long b, unsigned long t);

/*
 * Prepare to allocate "the MPU page tables".
 * Enable the hardware MPU.
 */
extern void mpu_init(void);

/*
 * These two functions were defined in the generic kernel code
 * by the Blackfin kernel maintainers (MPU support has been
 * around in Blackfin for a while). I preserve this interface so that
 * I don't have to modify the architecture-neutral code.
 * ...
 * Protect a specified page with specified permissions.
 */
extern void protect_page(struct mm_struct *mm, unsigned long addr,
		   	 unsigned long flags);

/*
 * Update the MPU after permissions have changed for a mm area.
 */
extern void update_protections(struct mm_struct *mm);

#ifdef CONFIG_MPU_USER_ACCESS
/*
 * The MPU version of addr_ok and range_ok used by uaccess.h to check
 * validity of a user-space address
 */
extern int mpu_addr_ok(const void * addr);
#endif

/*
 * Allocate and free a per-process "MPU page table".
 * These are called from asm/mmu_context.h.
 */
extern int mpu_init_new_context(struct task_struct *tsk, struct mm_struct *mm);
extern void mpu_destroy_context(struct mm_struct *mm);

/*
 * MPU-specific part of the context switch 
 */
extern void mpu_switch_mm(struct mm_struct *prev, struct mm_struct *next);

/*
 * MPU-specific part of start_thread
 */
extern void mpu_start_thread(struct pt_regs *regs);

/*
 * Memory Fault (memmanage) exception handler.
 * This is called from the architecture-specific low-level
 * exception handler. It is assumed that this code doesn't
 * get pre-emtped. 
 */
extern asmlinkage void __exception do_memmanage(struct pt_regs *regs);

#endif
