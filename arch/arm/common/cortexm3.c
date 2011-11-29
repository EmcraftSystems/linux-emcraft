/*
 *  linux/arch/arm/common/cortexm3.c
 *
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/types.h>
#include <asm/hardware/cortexm3.h>

struct cm3_scb {
	u32	cpuid;
	u32	icsr;
	u32	vtor;
	u32	aircr;
};

#define CM3_SCB_BASE	0xE000ED00
#define CM3_SCB		((volatile struct cm3_scb *)(CM3_SCB_BASE))

#define CM3_AIRCR_VECTKEY		0x5fa
#define CM3_AIRCR_VECTKEY_SHIFT		16
#define CM3_AIRCR_PRIGROUP_MSK		0x7
#define CM3_AIRCR_PRIGROUP_SHIFT	8
#define CM3_AIRCR_SYSRESET		(1<<2)

/*
 * Perform the low-level reboot.
 */
void cortex_m3_reboot(void)
{
	/*
	 * Perform reset but keep priority group unchanged.
	 */
	CM3_SCB->aircr = (CM3_AIRCR_VECTKEY << CM3_AIRCR_VECTKEY_SHIFT) |
			 (CM3_SCB->aircr &
			  (CM3_AIRCR_PRIGROUP_MSK << CM3_AIRCR_PRIGROUP_SHIFT))
			 | CM3_AIRCR_SYSRESET;
}
