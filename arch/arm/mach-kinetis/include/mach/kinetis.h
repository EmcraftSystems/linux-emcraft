/*
 * (C) Copyright 2012
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
 * Freescale Kinetis processor definitions
 */
#ifndef _MACH_KINETIS_H_
#define _MACH_KINETIS_H_

#include <asm/byteorder.h>

/*
 * This Kinetis port assumes that the CPU works in little-endian mode.
 * Switching to big-endian will require different bit offsets in peripheral
 * devices' registers. Also, some bit groups may lay on byte edges, so issue
 * with big-endian cannot be fixed only by defining bit offsets differently
 * for the big-endian mode.
 */
#ifndef __LITTLE_ENDIAN
#error This Kinetis port assumes that the CPU works in little-endian mode
#endif

/*
 * Peripheral memory map
 */
#define KINETIS_AIPS0PERIPH_BASE	0x40000000
#define KINETIS_AIPS1PERIPH_BASE	0x40080000

#ifndef __ASSEMBLY__

#include <asm/types.h>

/*
 * Limits for the `kinetis_periph_enable()` function:
 *     1. The number of SIM_SCGC[] registers
 *     2. The number of bits in those registers
 */
#define KINETIS_SIM_CG_NUMREGS	7
#define KINETIS_SIM_CG_NUMBITS	32

/*
 * System Control Block (SCB) register map
 */
struct kinetis_scb_regs {
	u32 cpuid;	/* CPUID Register */
	u32 icsr;	/* Interrupt Control and State Register */
	u32 rsrv0;
	u32 aircr;	/* Application Interrupt and Reset Control Register */
	u32 scr;	/* System Control Register */
	u32 ccr;	/* Configuration and Control Register */
};

/*
 * SIM registers base
 */
#define KINETIS_SCB_BASE	0xE000ED00
#define KINETIS_SCB		((volatile struct kinetis_scb_regs *) \
				KINETIS_SCB_BASE)

/*
 * System Integration Module (SIM) register map
 *
 * This map actually covers two hardware modules:
 *     1. SIM low-power logic, at 0x40047000
 *     2. System integration module (SIM), at 0x40048000
 */
struct kinetis_sim_regs {
	u32 sopt1;	/* System Options Register 1 */
	u32 sopt1cfg;	/* SOPT1 Configuration Register */
	u32 rsv0[1023];
	u32 sopt2;	/* System Options Register 2 */
	u32 rsv1;
	u32 sopt4;	/* System Options Register 4 */
	u32 sopt5;	/* System Options Register 5 */
	u32 sopt6;	/* System Options Register 6 */
	u32 sopt7;	/* System Options Register 7 */
	u32 rsv2[2];
	u32 sdid;	/* System Device Identification Register */
	u32 scgc[KINETIS_SIM_CG_NUMREGS];	/* Clock Gating Regs 1...7 */
	u32 clkdiv1;	/* System Clock Divider Register 1 */
	u32 clkdiv2;	/* System Clock Divider Register 2 */
	u32 fcfg1;	/* Flash Configuration Register 1 */
	u32 fcfg2;	/* Flash Configuration Register 2 */
	u32 uidh;	/* Unique Identification Register High */
	u32 uidmh;	/* Unique Identification Register Mid-High */
	u32 uidml;	/* Unique Identification Register Mid Low */
	u32 uidl;	/* Unique Identification Register Low */
	u32 clkdiv3;	/* System Clock Divider Register 3 */
	u32 clkdiv4;	/* System Clock Divider Register 4 */
	u32 mcr;	/* Misc Control Register */
};

/*
 * SIM registers base
 */
#define KINETIS_SIM_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00047000)
#define KINETIS_SIM		((volatile struct kinetis_sim_regs *) \
				KINETIS_SIM_BASE)

/*
 * Multipurpose Clock Generator (MCG) register map
 */
struct kinetis_mcg_regs {
	u8 c1;		/* MCG Control 1 Register */
	u8 c2;		/* MCG Control 2 Register */
	u8 c3;		/* MCG Control 3 Register */
	u8 c4;		/* MCG Control 4 Register */
	u8 c5;		/* MCG Control 5 Register */
	u8 c6;		/* MCG Control 6 Register */
	u8 status;	/* MCG Status Register */
	u8 rsv0;
	u8 atc;		/* MCG Status and Control Register */
	u8 rsv1;
	u8 atcvh;	/* MCG Auto Trim Compare Value High Register */
	u8 atcvl;	/* MCG Auto Trim Compare Value Low Register */
	u8 c7;		/* MCG Control 7 Register */
	u8 c8;		/* MCG Control 8 Register */
	u8 rsv2;
	u8 c10;		/* MCG Control 10 Register */
	u8 c11;		/* MCG Control 11 Register */
	u8 c12;		/* MCG Control 12 Register */
	u8 status2;	/* MCG Status 2 Register */
};

/*
 * MCG registers base
 */
#define KINETIS_MCG_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00064000)
#define KINETIS_MCG		((volatile struct kinetis_mcg_regs *) \
				KINETIS_MCG_BASE)

/*
 * Oscillator (OSC) register map
 */
struct kinetis_osc_regs {
	u8 cr;		/* OSC Control Register */
};

/*
 * OSC registers base
 */
#define KINETIS_OSC0_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00065000)
#define KINETIS_OSC0		((volatile struct kinetis_osc_regs *) \
				KINETIS_OSC0_BASE)
#define KINETIS_OSC1_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x000E5000)
#define KINETIS_OSC1		((volatile struct kinetis_osc_regs *) \
				KINETIS_OSC1_BASE)

/*
 * System Mode Controller (SMC) register map
 */
struct kinetis_smc_regs {
	u8 pmprot;	/* Power Mode Protection Register */
	u8 pmctrl;	/* Power Mode Control Register */
	u8 vllsctrl;	/* VLLS Control Register */
	u8 pmstat;	/* Power Mode Status Register */
};

/*
 * SMC registers base
 */
#define KINETIS_SMC_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x0007E000)
#define KINETIS_SMC		((volatile struct kinetis_smc_regs *) \
				KINETIS_SMC_BASE)

/*
 * Watchdog (WDOG) register map
 */
struct kinetis_wdog_regs {
	u16 stctrlh;	/* Watchdog Status & Control Register High */
	u16 stctrll;	/* Watchdog Status & Control Register Low */
};

/*
 * WDOG registers base
 */
#define KINETIS_WDOG_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00052000)
#define KINETIS_WDOG		((volatile struct kinetis_wdog_regs *) \
				KINETIS_WDOG_BASE)
/*
 * FTM registers base
 */
#define KINETIS_FTM0_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00038000)
#define KINETIS_FTM1_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00039000)
#define KINETIS_FTM2_BASE	(KINETIS_AIPS1PERIPH_BASE + 0x00038000)
#define KINETIS_FTM3_BASE	(KINETIS_AIPS1PERIPH_BASE + 0x00039000)

#endif /* __ASSEMBLY__ */

#endif /* _MACH_KINETIS_H_ */
