/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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

/*
 * LPC178x/7x processor definitions
 */
#ifndef _MACH_LPC178X_H_
#define _MACH_LPC178X_H_

#include <asm/byteorder.h>

/*
 * This LPC178X port assumes that the CPU works in little-endian mode.
 * Switching to big-endian will require different bit offsets in peripheral
 * devices' registers. Also, some bit groups may lay on byte edges, so issue
 * with big-endian cannot be fixed only by defining bit offsets differently
 * for the big-endian mode.
 */
#ifndef __LITTLE_ENDIAN
#error This LPC178X port assumes that the CPU works in little-endian mode
#endif

/*
 * Peripheral memory map
 */
#define LPC178X_APB_PERIPH_BASE		0x40000000
#define LPC178X_APB0PERIPH_BASE		(LPC178X_APB_PERIPH_BASE + 0x00000000)
#define LPC178X_APB1PERIPH_BASE		(LPC178X_APB_PERIPH_BASE + 0x00080000)
#define LPC178X_AHB_PERIPH_BASE		0x20080000

#ifndef __ASSEMBLY__

#include <asm/types.h>

/*
 * PLL register map
 * Used for PLL0 at 0x400FC080 and for PLL1 at 0x400FC0A0.
 *
 * This structure is 0x20 bytes long, it is important when it embedding into
 * `struct lpc178x_scc_regs`.
 */
struct lpc178x_pll_regs {
	u32 con;	/* PLL Control register */
	u32 cfg;	/* PLL Configuration register */
	u32 stat;	/* PLL Status register */
	u32 feed;	/* PLL Feed register */
	u32 rsv0[4];
};

/*
 * SCC (System and Clock Control) register map
 * Should be mapped at 0x400FC000.
 *
 * This structure is used by the code in `clock.c` and `power.c`.
 */
struct lpc178x_scc_regs {
	/* 0x400FC000: Flash Accelerator Configuration Register */
	u32 rsv0[32];

	/* 0x400FC080: PLL0 registers */
	struct lpc178x_pll_regs pll0; /* PLL0 registers */

	/* 0x400FC0A0: PLL1 registers */
	struct lpc178x_pll_regs pll1; /* PLL1 registers */

	/* 0x400FC0C0: Power control registers */
	u32 pcon;       /* Power Control register */
	u32 pconp;      /* Power Control for Peripherals */
	/* 0x400FC0C8 */
	u32 rsv1[14];

	/* 0x400FC100: Clock control */
	u32 emcclksel;	/* External Memory Controller
				Clock Selection register */
	u32 cclksel;	/* CPU Clock Selection register */
	u32 usbclksel;	/* USB Clock Selection register */
	u32 clksrcsel;	/* Clock Source Selection register */
	/* 0x400FC110 */
	u32 rsv2[36];

	/* 0x400FC1A0 */
	u32 scs;	/* System Controls and Status register */
	u32 rsv3;
	u32 pclksel;	/* Peripheral Clock Selection register */
	/* 0x400FC1AC */
	u32 rsv4[6];

	/* 0x400FC1C4 */
	u32 dmacreqsel;	/* DMA Request Select register */
	u32 clkoutcfg;
	u32 rstcon0;
	u32 rstcon1;
	/* 0x400FC1D4 */
	u32 rsv5[2];

	/* 0x400FC1DC */
	u32 emcdlyctl;
	u32 emccal;
};

/*
 * SCC registers base
 */
#define LPC178X_SCC_BASE		(LPC178X_APB1PERIPH_BASE + 0x0007C000)
#define LPC178X_SCC			((volatile struct lpc178x_scc_regs *) \
					LPC178X_SCC_BASE)

#endif /* __ASSEMBLY__ */

#endif /* _MACH_LPC178X_H_ */
