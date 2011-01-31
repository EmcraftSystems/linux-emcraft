/*
 * linux/arch/arm/mach-a2f/include/mach/a2f.h
 *
 * Copyright (C) 2010,2011 Vladimir Khusainov, Emcraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _MACH_A2F_H_
#define _MACH_A2F_H_

struct a2f_sysreg {
	unsigned int	esram_cr;
	unsigned int	envm_cr;
	unsigned int	envm_remap_syscr;
	unsigned int	envm_remap_fabcr;
	unsigned int	fab_prot_size_cr;
	unsigned int	fab_prot_base_cr;
	unsigned int	ahb_matrix_cr;
	unsigned int	mss_sr;
	unsigned int	clr_mss_sr;
	unsigned int	efrom_cr;
	unsigned int	iap_cr;
	unsigned int	soft_irq_cr;
	unsigned int	soft_rst_cr;
	unsigned int	device_sr;
	unsigned int	systick_cr;
	unsigned int	emc_mux_cr;
	unsigned int	emc_cs_0_cr;
	unsigned int	emc_cs_1_cr;
	unsigned int	mss_clk_cr;
	unsigned int	mss_ccc_div_cr;
	unsigned int	mss_ccc_mux_cr;
	unsigned int	mss_ccc_pll_cr;
	unsigned int	mss_ccc_dly_cr;
	unsigned int	mss_ccc_sr;
	unsigned int	mss_rcosc_cr;
	unsigned int	vrpsm_cr;
	unsigned int	reserved1;
	unsigned int	fab_if_cr;
	unsigned int	fab_apb_hiword_cr;
	unsigned int	loopback_cr;
	unsigned int	mss_io_bank_cr;
	unsigned int	gpin_source_cr;
	unsigned int	test_sr;
	unsigned int	red_rep_addr0;
	unsigned int	red_rep_low_locs0;
	unsigned int	red_rep_high_locs0;
	unsigned int	red_rep_adr1;
	unsigned int	red_rep_low_locs1;
	unsigned int	red_rep_high_locs1;
	unsigned int	fabric_cr;
	unsigned int	reserved2[24];
	unsigned int	iomux_cr[83];
};

#define A2F_SYSREG_BASE	0xE0042000
#define A2F_SYSREG	((volatile struct a2f_sysreg *)(A2F_SYSREG_BASE))

struct a2f_scb {
	unsigned int	cpuid;
	unsigned int	icsr;
	unsigned int	vtor;
	unsigned int	aircr;
};

#define A2F_SCB_BASE	0xE000ED00
#define A2F_SCB		((volatile struct a2f_scb *)(A2F_SCB_BASE))

#define A2F_SCS_BASE	0xE000E000


struct a2f_timer
{
	unsigned int timer1_val;
	unsigned int timer1_loadval;
	unsigned int timer1_bgloadval;
	unsigned int timer1_ctrl;
	unsigned int timer1_ris;
	unsigned int timer1_mis;

	unsigned int timer2_val;
	unsigned int timer2_loadval;
	unsigned int timer2_bgloadval;
	unsigned int timer2_ctrl;
	unsigned int timer2_ris;
	unsigned int timer2_mis;

	unsigned int timer64_val_u;
	unsigned int timer64_val_l;
	unsigned int timer64_loadval_u;
	unsigned int timer64_loadval_l;
	unsigned int timer64_bgloadval_u;
	unsigned int timer64_bgloadval_l;
	unsigned int timer64_ctrl;
	unsigned int timer64_ris;
	unsigned int timer64_mis;
	unsigned int timer64_mode;
};

#define A2F_TIMER_BASE	0x40005000
#define A2F_TIMER	((volatile struct a2f_timer *)(A2F_TIMER_BASE))

static inline void __enable_irq(void)	{ asm volatile ("cpsie i"); }
static inline void __disable_irq(void)	{ asm volatile ("cpsid i"); }

#define ETH_CORE_BASE	0x40003000
#define ETH_CORE_IRQ	5

#endif	/*_MACH_A2F_H_ */
