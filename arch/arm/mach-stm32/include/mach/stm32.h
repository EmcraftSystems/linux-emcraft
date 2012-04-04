/*
 * (C) Copyright 2011
 * Yuri Tikhonov, Emcraft Systems, yur@emcraft.com
 *
 * Added support for STM32F1
 * (C) Copyright 2012
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
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
 * STM32 processor definitions
 */
#ifndef _MACH_STM32_H_
#define _MACH_STM32_H_

/******************************************************************************
 * Peripheral memory map
 ******************************************************************************/

#define STM32_PERIPH_BASE	0x40000000
#define STM32_APB1PERITH_BASE	(STM32_PERIPH_BASE + 0x00000000)
#define STM32_APB2PERITH_BASE	(STM32_PERIPH_BASE + 0x00010000)
#define STM32_AHB1PERITH_BASE	(STM32_PERIPH_BASE + 0x00020000)
#define STM32_AHB2PERITH_BASE	(STM32_PERIPH_BASE + 0x10000000)

#ifndef __ASSEMBLY__

#include <asm/types.h>

/******************************************************************************
 * Reset and Clock Control
 ******************************************************************************/

/*
 * RCC register map
 */
struct stm32_rcc_regs {
	u32	cr;		/* RCC clock control			      */
#ifndef CONFIG_ARCH_STM32F1
	u32	pllcfgr;	/* RCC PLL configuration		      */
#endif
	u32	cfgr;		/* RCC clock configuration		      */
	u32	cir;		/* RCC clock interrupt			      */
#ifndef CONFIG_ARCH_STM32F1
	u32	ahb1rstr;	/* RCC AHB1 peripheral reset		      */
	u32	ahb2rstr;	/* RCC AHB2 peripheral reset		      */
	u32	ahb3rstr;	/* RCC AHB3 peripheral reset		      */
	u32	rsv0;
	u32	apb1rstr;	/* RCC APB1 peripheral reset		      */
	u32	apb2rstr;	/* RCC APB2 peripheral reset		      */
	u32	rsv1[2];
#else
	u32	apb2rstr;	/* RCC APB2 peripheral reset		      */
	u32	apb1rstr;	/* RCC APB1 peripheral reset		      */
#endif

	u32	ahb1enr;	/* RCC AHB1 peripheral clock enable	      */
#ifndef CONFIG_ARCH_STM32F1
	u32	ahb2enr;	/* RCC AHB2 peripheral clock enable	      */
	u32	ahb3enr;	/* RCC AHB3 peripheral clock enable	      */
	u32	rsv2;
	u32	apb1enr;	/* RCC APB1 peripheral clock enable	      */
	u32	apb2enr;	/* RCC APB2 peripheral clock enable	      */
#else
	u32	apb2enr;	/* RCC APB2 peripheral clock enable	      */
	u32	apb1enr;	/* RCC APB1 peripheral clock enable	      */
#endif

#ifndef CONFIG_ARCH_STM32F1
	u32	rsv3[2];
	u32	ahb1lpenr;	/* RCC AHB1 periph clk enable in low pwr mode */
	u32	ahb2lpenr;	/* RCC AHB2 periph clk enable in low pwr mode */
	u32	ahb3lpenr;	/* RCC AHB3 periph clk enable in low pwr mode */
	u32	rsv4;
	u32	apb1lpenr;	/* RCC APB1 periph clk enable in low pwr mode */
	u32	apb2lpenr;	/* RCC APB2 periph clk enable in low pwr mode */
	u32	rsv5[2];
#endif
	u32	bdcr;		/* RCC Backup domain control		      */
	u32	csr;		/* RCC clock control & status		      */
#ifndef CONFIG_ARCH_STM32F1
	u32	rsv6[2];
	u32	sscgr;		/* RCC spread spectrum clock generation	      */
	u32	plli2scfgr;	/* RCC PLLI2S configuration		      */
#endif
};

/*
 * RCC registers base
 */
#ifdef CONFIG_ARCH_STM32F1
#define STM32_RCC_BASE		(STM32_AHB1PERITH_BASE + 0x1000) /* STM32F1 */
#else
#define STM32_RCC_BASE		(STM32_AHB1PERITH_BASE + 0x3800) /* STM32F2 */
#endif
#define STM32_RCC	((volatile struct stm32_rcc_regs *)STM32_RCC_BASE)

#endif /* __ASSEMBLY__ */

#endif /* _MACH_STM32_H_ */
