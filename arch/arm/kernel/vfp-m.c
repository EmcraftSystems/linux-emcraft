/*
 * arch/arm/kernel/vfp-m.c
 *
 * Copyright (C) 2010 ARM Limited
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>

#include <asm/thread_notify.h>

static union vfp_state *last_vfp_context;

static void save_vfp_context(union vfp_state *vfp)
{
	/* vstmia %0!, {d8-d15} */
	asm("	stc	p11, cr8, [%0], #16*4\n" : : "r" (vfp) : "cc");
}

static void load_vfp_context(union vfp_state *vfp)
{
	/* vldmia %0!, {d8-d15} */
	asm("	ldc	p11, cr8, [%0], #16*4\n" : : "r" (vfp) : "cc");
}

static int vfpm_notifier(struct notifier_block *self, unsigned long cmd,
			 void *t)
{
	struct thread_info *thread = t;
	union vfp_state *vfp = &thread->vfpstate;
	union vfp_state *old_vfp = &current_thread_info()->vfpstate;
	u32 *fpccr = (u32 *)0xe000ef34;

	switch (cmd) {
	case THREAD_NOTIFY_FLUSH:
		memset(vfp, 0, sizeof(*vfp));
		vfp->hard.clean = 1;
		/* fall through */

	case THREAD_NOTIFY_EXIT:
		if (last_vfp_context == vfp) {
			/* disable lazy state saving */
			*fpccr &= ~1;
			last_vfp_context = NULL;
		}
		break;

	case THREAD_NOTIFY_SWITCH:
		if (!old_vfp->hard.clean) {
			save_vfp_context(old_vfp);
			last_vfp_context = old_vfp;
		}
		if (!vfp->hard.clean && last_vfp_context != vfp) {
			load_vfp_context(vfp);
			last_vfp_context = vfp;
		}
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block vfpm_notifier_block = {
	.notifier_call	= vfpm_notifier,
};

static int __init vfpm_init(void)
{
	u32 *cpacr = (u32 *)0xe000ed88;
	u32 *mvfr0 = (u32 *)0xe000ef40;
	u32 *fpccr = (u32 *)0xe000ef34;

	/* check for single-precision VFP operations */
	if ((*mvfr0 & 0xf0) != 0x20)
		return 0;

	printk(KERN_INFO "ARMv7-M VFP Extension supported\n");

	*cpacr |= 0xf << 20;		/* coprocessor access */
	*fpccr |= 3 << 30;		/* automatic lazy state preservation */

	elf_hwcap |= HWCAP_VFP;
	thread_register_notifier(&vfpm_notifier_block);

	return 0;
}

late_initcall(vfpm_init);
