/*
 * (C) Copyright 2011-2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
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
 * LPC18xx/43xx processor definitions
 */
#ifndef _MACH_LPC18XX_H_
#define _MACH_LPC18XX_H_

#include <asm/byteorder.h>

/*
 * This LPC18XX port assumes that the CPU works in little-endian mode.
 * Switching to big-endian will require different bit offsets in peripheral
 * devices' registers. Also, some bit groups may lay on byte edges, so issue
 * with big-endian cannot be fixed only by defining bit offsets differently
 * for the big-endian mode.
 */
#ifndef __LITTLE_ENDIAN
#error This LPC18XX port assumes that the CPU works in little-endian mode
#endif

/*
 * Peripheral memory map
 */
#define LPC18XX_PERIPH_BASE		0x40000000
#define LPC18XX_APB0PERIPH_BASE		0x40080000

/*
 * RGU (Reset Generation Unit) register map
 */
struct lpc18xx_rgu_regs {
	u32 rsv0[64];

	u32 ctrl0;		/* Reset control register 0 */
	u32 ctrl1;		/* Reset control register 1 */
	u32 rsv1[2];

	u32 status0;		/* Reset status register 0 */
	u32 status1;		/* Reset status register 1 */
	u32 status2;		/* Reset status register 2 */
	u32 status3;		/* Reset status register 3 */
	u32 rsv2[12];

	u32 active_status0;	/* Reset active status register 0 */
	u32 active_status1;	/* Reset active status register 1 */
	u32 rsv3[170];

	u32 ext_stat[64];	/* Reset external status registers */
};

/*
 * RGU registers base
 */
#define LPC18XX_RGU_BASE		0x40053000
#define LPC18XX_RGU			((volatile struct lpc18xx_rgu_regs *) \
					LPC18XX_RGU_BASE)

/*
 * RESET_CTRL0 register
 */
/* ETHERNET_RST */
#define LPC18XX_RGU_CTRL0_ETHERNET	(1 << 22)

/*
 * CREG (Configuration Registers) register map
 */
struct lpc18xx_creg_regs {
	u32 rsv0;
	u32 creg0;		/* Chip configuration register 0 */
	u32 rsv1[62];
	u32 m4memmap;		/* ARM Cortex-M4 memory mapping */
	u32 rsv2;
	u32 creg1;		/* Chip configuration register 1 */
	u32 creg2;		/* Chip configuration register 2 */
	u32 creg3;		/* Chip configuration register 3 */
	u32 creg4;		/* Chip configuration register 4 */
	u32 creg5;		/* Chip configuration register 5 */
	u32 dmamux;		/* DMA muxing control */
	u32 rsv3[2];
	u32 etbcfg;		/* ETB RAM configuration */
	u32 creg6;		/* Chip configuration register 6 */
	u32 m4txevent;		/* Cortex-M4 TXEV event clear */
	u32 rsv4[51];
	u32 chipid;		/* Part ID */
	u32 rsv5[127];
	u32 m0txevent;		/* Cortex-M0 TXEV event clear */
	u32 m0appmemmap;	/* ARM Cortex-M0 memory mapping */
};

/*
 * CREG registers base
 */
#define LPC18XX_CREG_BASE		0x40043000
#define LPC18XX_CREG			((volatile struct lpc18xx_creg_regs *) \
					LPC18XX_CREG_BASE)

/*
 * CREG6 register
 */
/* Selects the Ethernet mode */
#define LPC18XX_CREG_CREG6_ETHMODE_BITS		0
#define LPC18XX_CREG_CREG6_ETHMODE_MSK \
	(7 << LPC18XX_CREG_CREG6_ETHMODE_BITS)
#define LPC18XX_CREG_CREG6_ETHMODE_MII \
	(0 << LPC18XX_CREG_CREG6_ETHMODE_BITS)
#define LPC18XX_CREG_CREG6_ETHMODE_RMII \
	(4 << LPC18XX_CREG_CREG6_ETHMODE_BITS)
/* EMC_CLK divided clock select */
#define LPC18XX_CREG_CREG6_EMCCLKSEL_MSK	(1 << 16)

#endif /* _MACH_LPC18XX_H_ */
