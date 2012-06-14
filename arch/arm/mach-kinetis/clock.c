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
#include <linux/module.h>
#include <linux/io.h>

#include <asm/clkdev.h>

#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/kinetis.h>
#include <mach/power.h>

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
 * MCG Control 11 Register
 */
/* PLL1 External Reference Divider */
#define KINETIS_MCG_C11_PRDIV_BITS	0
#define KINETIS_MCG_C11_PRDIV_MSK \
	(((1 << 3) - 1) << KINETIS_MCG_C11_PRDIV_BITS)
/* PLL Clock Select: PLL0 or PLL1 */
#define KINETIS_MCG_C11_PLLCS_MSK	(1 << 4)
/* PLL1 Stop Enable */
#define KINETIS_MCG_C11_PLLSTEN1_MSK	(1 << 5)
/* PLL1 Clock Enable */
#define KINETIS_MCG_C11_PLLCLKEN1_MSK	(1 << 6)
/* PLL1 External Reference Select (for K70@120MHz) */
#define KINETIS_MCG_C11_PLLREFSEL1_BIT	7
#define KINETIS_MCG_C11_PLLREFSEL1_MSK	(1 << KINETIS_MCG_C11_PLLREFSEL1_BIT)
/*
 * MCG Control 12 Register
 */
/* VCO1 Divider */
#define KINETIS_MCG_C12_VDIV1_BITS	0
#define KINETIS_MCG_C12_VDIV1_MSK \
	(((1 << 5) - 1) << KINETIS_MCG_C12_VDIV1_BITS)

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
 * System Clock Divider Register 3
 */
/* LCD Controller clock divider divisor */
#define KINETIS_SIM_CLKDIV3_LCDCDIV_BITS	16
#define KINETIS_SIM_CLKDIV3_LCDCDIV_BITWIDTH	12
#define KINETIS_SIM_CLKDIV3_LCDCDIV_MSK \
	(((1 << KINETIS_SIM_CLKDIV3_LCDCDIV_BITWIDTH) - 1) << \
	KINETIS_SIM_CLKDIV3_LCDCDIV_BITS)
/* LCD Controller clock divider fraction */
#define KINETIS_SIM_CLKDIV3_LCDCFRAC_BITS	8
#define KINETIS_SIM_CLKDIV3_LCDCFRAC_BITWIDTH	8
#define KINETIS_SIM_CLKDIV3_LCDCFRAC_MSK \
	(((1 << KINETIS_SIM_CLKDIV3_LCDCFRAC_BITWIDTH) - 1) << \
	KINETIS_SIM_CLKDIV3_LCDCFRAC_BITS)

/*
 * Misc Control Register
 */
/* Start LCDC display */
#define KINETIS_SIM_MCR_LCDSTART_MSK	(1 << 16)

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
 * The structure that holds the information about a clock
 */
struct clk {
	/* Clock rate, the only possible value of */
	unsigned long rate;
	/* For `kinetis_periph_enable()` */
	kinetis_clock_gate_t gate;

	/* Reference count */
	unsigned int enabled;

	/* Custom `clk_*` functions */
	void (*clk_enable)(struct clk *clk);
	void (*clk_disable)(struct clk *clk);
};

static DEFINE_SPINLOCK(clocks_lock);

/*
 * Enable the clock via the clock gating registers
 */
static void local_clk_enable(struct clk *clk)
{
	kinetis_periph_enable(clk->gate, 1);
}

/*
 * Disable the clock via the clock gating registers
 */
static void local_clk_disable(struct clk *clk)
{
	kinetis_periph_enable(clk->gate, 0);
}

/*
 * Enable the LCDC clock
 */
static void lcdc_clk_enable(struct clk *clk)
{
	KINETIS_SIM->mcr |= KINETIS_SIM_MCR_LCDSTART_MSK;
}

/*
 * Disable the LCDC clock
 */
static void lcdc_clk_disable(struct clk *clk)
{
	KINETIS_SIM->mcr &= ~KINETIS_SIM_MCR_LCDSTART_MSK;
}

/*
 * clk_enable - inform the system when the clock source should be running.
 */
int clk_enable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->enabled++ == 0) {
		if (clk->clk_enable)
			clk->clk_enable(clk);
		else
			local_clk_enable(clk);
	}
	spin_unlock_irqrestore(&clocks_lock, flags);

	return 0;
}
EXPORT_SYMBOL(clk_enable);

/*
 * clk_disable - inform the system when the clock source is no longer required
 */
void clk_disable(struct clk *clk)
{
	unsigned long flags;

	WARN_ON(clk->enabled == 0);

	spin_lock_irqsave(&clocks_lock, flags);
	if (--clk->enabled == 0) {
		if (clk->clk_disable)
			clk->clk_disable(clk);
		else
			local_clk_disable(clk);
	}
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

/*
 * clk_get_rate - obtain the current clock rate (in Hz) for a clock source
 */
unsigned long clk_get_rate(struct clk *clk)
{
	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

/*
 * clk_set_rate - set the clock rate for a clock source
 *
 * We do not support this, because we assume that all clock rates are fixed.
 */
int clk_set_rate(struct clk *clk, unsigned long rate)
{
	return -EINVAL;
}
EXPORT_SYMBOL(clk_set_rate);

/*
 * clk_round_rate - adjust a rate to the exact rate a clock can provide
 *
 * We return the actual clock rate, because we assume that all clock rates
 * are fixed.
 */
long clk_round_rate(struct clk *clk, unsigned long rate)
{
	return clk->rate;
}
EXPORT_SYMBOL(clk_round_rate);

/*
 * Clock for the Ethernet module of the MCU. The clock rate is initialized
 * in `kinetis_clock_init()`.
 */
static struct clk clk_net = {
	.gate = KINETIS_CG_ENET,
};

/*
 * Clock for the LCD Controller module of the MCU. The clock rate is initialized
 * in `kinetis_clock_init()`.
 */
static struct clk clk_lcdc = {
	.gate = KINETIS_CG_LCDC,
	.clk_enable = lcdc_clk_enable,
	.clk_disable = lcdc_clk_disable,
};

/*
 * Array of all clock to register with the `clk_*` infrastructure
 */
#define INIT_CLKREG(_clk,_devname,_conname)		\
	{						\
		.clk		= _clk,			\
		.dev_id		= _devname,		\
		.con_id		= _conname,		\
	}
static struct clk_lookup kinetis_clkregs[] = {
	INIT_CLKREG(&clk_net, NULL, "fec_clk"),
	INIT_CLKREG(&clk_lcdc, "imx-fb.0", NULL),
};

/*
 * Clock values
 */
static u32 clock_val[CLOCK_END];

/*
 * Initialize the reference clocks.
 */
void __init kinetis_clock_init(void)
{
	int i;
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
	case PLATFORM_KINETIS_K70_SOM:
	case PLATFORM_KINETIS_K61_SOM:
		/*
		 * The PLL0 in a PK70FN1M0VMJ12 and PK70FN1M0VMJ15 divides
		 * its output rate by 2.
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
	 * Check whether PLL0 or PLL1 is used for MCGOUTCLK
	 */
	if (KINETIS_MCG->c11 & KINETIS_MCG_C11_PLLCS_MSK) {
		/*
		 * PLL1 internal divider
		 */
		mcgout /= ((KINETIS_MCG->c11 & KINETIS_MCG_C11_PRDIV_MSK) >>
			KINETIS_MCG_C11_PRDIV_BITS) + 1;
		/*
		 * PLL1 multiplication factor
		 */
		mcgout *= ((KINETIS_MCG->c12 & KINETIS_MCG_C12_VDIV1_MSK) >>
			KINETIS_MCG_C12_VDIV1_BITS) + vdiv_min;
	} else {
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
	}

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

	/*
	 * Ethernet clock
	 */
	clock_val[CLOCK_MACCLK] = CONFIG_KINETIS_EXTAL0_RATE;

	/*
	 * LCD Controller clock
	 */
	clock_val[CLOCK_LCDCLK] = mcgout /
		(((KINETIS_SIM->clkdiv3 &
			KINETIS_SIM_CLKDIV3_LCDCDIV_MSK) >>
		KINETIS_SIM_CLKDIV3_LCDCDIV_BITS) + 1) *
		(((KINETIS_SIM->clkdiv3 &
			KINETIS_SIM_CLKDIV3_LCDCFRAC_MSK) >>
		KINETIS_SIM_CLKDIV3_LCDCFRAC_BITS) + 1);

	/*
	 * Initialize the `clk_*` structures
	 */
	clk_net.rate = clock_val[CLOCK_MACCLK];
	clk_lcdc.rate = clock_val[CLOCK_LCDCLK];
	/*
	 * Register clocks with the `clk_*` infrastructure
	 */
	for (i = 0; i < ARRAY_SIZE(kinetis_clkregs); i++)
		clkdev_add(&kinetis_clkregs[i]);
}

/*
 * Return a clock value for the specified clock.
 *
 * This is the legacy way to get a clock rate, independent
 * from `clk_get_rate()`.
 */
unsigned int kinetis_clock_get(enum kinetis_clock clk)
{
	return clock_val[clk];
}
