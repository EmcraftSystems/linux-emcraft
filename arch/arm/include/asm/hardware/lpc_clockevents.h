/*
 * (C) Copyright 2012
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

#ifndef _ASM_ARCH_LPC_CLOCKEVENTS_H_
#define _ASM_ARCH_LPC_CLOCKEVENTS_H_

#if defined(CONFIG_ARCH_LPC178X)

#include <linux/types.h>

extern void lpc_clockevents_tmr_init(u32 timer_regs_base, u32 src_clk, int irq);

#endif /* CONFIG_ARCH_LPC178X */

#endif /* _ASM_ARCH_LPC_CLOCKEVENTS_H_ */
