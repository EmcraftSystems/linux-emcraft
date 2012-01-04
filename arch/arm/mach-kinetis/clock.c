/*
 * (C) Copyright 2011, 2012
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
#include <linux/init.h>
#include <linux/io.h>

#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/kinetis.h>

/*
 * Internal oscillator value
 *
 * This frequency should be set to the same value as in U-Boot.
 */
#define CONFIG_KINETIS_EXTAL0_RATE	50000000	/* 50 MHz */

/*
 * MCG Control 5 Register
 */
/* PLL External Reference Divider */
#define KINETIS_MCG_C5_PRDIV_BITS	0
#define KINETIS_MCG_C5_PRDIV_MSK \
	(((1 << 3) - 1) << KINETIS_MCG_C5_PRDIV_BITS)
/* PLL Stop Enable */
#define KINETIS_MCG_C5_PLLSTEN_MSK	(1 << 5)
/* PLL External Reference Select (for K70@120MHz) */
#define KINETIS_MCG_C5_PLLREFSEL_BIT	7
#define KINETIS_MCG_C5_PLLREFSEL_MSK	(1 << KINETIS_MCG_C5_PLLREFSEL_BIT)
/*
 * MCG Control 6 Register
 */
/* VCO Divider */
#define KINETIS_MCG_C6_VDIV_BITS	0
#define KINETIS_MCG_C6_VDIV_MSK \
	(((1 << 5) - 1) << KINETIS_MCG_C6_VDIV_BITS)
/* PLL Select */
#define KINETIS_MCG_C6_PLLS_MSK		(1 << 6)

/*
 * SIM registers
 */
/*
 * System Clock Divider Register 1
 */
/* Clock 1 output divider value (for the core/system clock) */
#define KINETIS_SIM_CLKDIV1_OUTDIV1_BITS	28
#define KINETIS_SIM_CLKDIV1_OUTDIV1_MSK \
	(((1 << 4) - 1) << KINETIS_SIM_CLKDIV1_OUTDIV1_BITS)
/* Clock 2 output divider value (for the peripheral clock) */
#define KINETIS_SIM_CLKDIV1_OUTDIV2_BITS	24
#define KINETIS_SIM_CLKDIV1_OUTDIV2_MSK \
	(((1 << 4) - 1) << KINETIS_SIM_CLKDIV1_OUTDIV2_BITS)

/*
 * Multipurpose Clock Generator (MCG) register map
 *
 * See Chapter 24 of the K60 Reference Manual (page 559)
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
	u8 atc;		/* MCG Auto Trim Control Register */
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
#define KINETIS_MCG_BASE		(KINETIS_AIPS0PERIPH_BASE + 0x00064000)
#define KINETIS_MCG			((volatile struct kinetis_mcg_regs *) \
					KINETIS_MCG_BASE)

/*
 * Clock values
 */
static u32 clock_val[CLOCK_END];

/*
 * Initialize the reference clocks.
 */
void __init kinetis_clock_init(void)
{
	int platform;

	/* MCU-specific parameters */
	int vco_div;
	int vdiv_min;
	/* Frequency at the MCGOUTCLK output of the MCG */
	int mcgout;

	/*
	 * Default values for the MCU-specific parameters
	 */
	vco_div = 1;
	vdiv_min = 24;

	/*
	 * Initialize the MCU-specific parameters
	 */
	platform = kinetis_platform_get();
	switch (platform) {
	case PLATFORM_KINETIS_TWR_K70F120M:
		/*
		 * The PLL0 in a 120 MHz K70 MCU divides its output rate by 2.
		 * This does not apply to 100 MHz K60 MCUs.
		 *
		 * The minimum value for VDIV is also different
		 * for 100 MHz K60 MCUs.
		 */
		vco_div = 2;
		vdiv_min = 16;
		break;
	default:
		/*
		 * Let's assume smth in this case; maybe things
		 * will work...
		 */
		printk(KERN_WARNING "%s: unsupported platform %d\n",
			__func__, platform);
		break;
	}

	/*
	 * The following code assumes that the MCGOUTCLK clock is always the
	 * PLL0 output clock and that the PLL0 input is EXTAL0.
	 */
	/*
	 * Start with the MCG input clock
	 */
	mcgout = CONFIG_KINETIS_EXTAL0_RATE;

	/*
	 * PLL0 internal divider
	 */
	mcgout /= ((KINETIS_MCG->c5 & KINETIS_MCG_C5_PRDIV_MSK) >>
		KINETIS_MCG_C5_PRDIV_BITS) + 1;
	/*
	 * PLL0 multiplication factor
	 */
	mcgout *= ((KINETIS_MCG->c6 & KINETIS_MCG_C6_VDIV_MSK) >>
		KINETIS_MCG_C6_VDIV_BITS) + vdiv_min;

	/*
	 * Apply the PLL0 output divider
	 */
	mcgout /= vco_div;

	/*
	 * CPU clock
	 */
	clock_val[CLOCK_CCLK] = mcgout /
		(((KINETIS_SIM->clkdiv1 & KINETIS_SIM_CLKDIV1_OUTDIV1_MSK) >>
		KINETIS_SIM_CLKDIV1_OUTDIV1_BITS) + 1);

	/*
	 * The SYSTICK clock is always the same as the CPU clock
	 * on the Freescale Kinetis MCUs.
	 */
	clock_val[CLOCK_SYSTICK] = clock_val[CLOCK_CCLK];

	/*
	 * Peripheral clock
	 */
	clock_val[CLOCK_PCLK] = mcgout /
		(((KINETIS_SIM->clkdiv1 & KINETIS_SIM_CLKDIV1_OUTDIV2_MSK) >>
		KINETIS_SIM_CLKDIV1_OUTDIV2_BITS) + 1);
}

/*
 * Return a clock value for the specified clock.
 */
unsigned int kinetis_clock_get(enum kinetis_clock clk)
{
	return clock_val[clk];
}
