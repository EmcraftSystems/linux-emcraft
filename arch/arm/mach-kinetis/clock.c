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
#include <linux/delay.h>

#include <asm/clkdev.h>

#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/kinetis.h>
#include <mach/power.h>

/*
 * Frequencies on OSC0 (EXTAL0/XTAL0) and OSC1 (EXTAL1/XTAL1)
 *
 * These frequencies should be set to the same values as in U-Boot.
 * These frequencies happen to be the same on K70-SOM and TWR-K70F120M.
 */
#define CONFIG_KINETIS_OSC0_RATE	50000000	/* 50 MHz */
#define CONFIG_KINETIS_OSC1_RATE	12000000	/* 12 MHz */

/*
 * The USB High Speed ULPI clock rate must be 60MHz
 */
#define KINETIS_USBHS_REQ_RATE		60000000

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
 * System Options Register 2
 */
/* USB HS clock source select */
#define KINETIS_SIM_SOPT2_USBHSRC_BITS	2
#define KINETIS_SIM_SOPT2_USBHSRC_MSK	(3 << KINETIS_SIM_SOPT2_USBHSRC_BITS)
#define KINETIS_SIM_SOPT2_USBHSRC_PLL0	(1 << KINETIS_SIM_SOPT2_USBHSRC_BITS)
#define KINETIS_SIM_SOPT2_USBHSRC_PLL1	(2 << KINETIS_SIM_SOPT2_USBHSRC_BITS)

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
 * System Clock Divider Register 2
 */
/* USB HS clock divider fraction */
#define KINETIS_SIM_CLKDIV2_USBHSFRAC_BIT	8
#define KINETIS_SIM_CLKDIV2_USBHSFRAC_MSK \
	(1 << KINETIS_SIM_CLKDIV2_USBHSFRAC_BIT)
/* USB HS clock divider divisor */
#define KINETIS_SIM_CLKDIV2_USBHSDIV_BIT	9
#define KINETIS_SIM_CLKDIV2_USBHSDIV_MSK \
	(7 << KINETIS_SIM_CLKDIV2_USBHSDIV_BIT)

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
/* 60 MHz ULPI clock (ULPI_CLK) output enable */
#define KINETIS_SIM_MCR_ULPICLKOBE_MSK	(1 << 30)
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
 * Clocks for each of the 6 UARTs supported by the Kinetis K70 MCUs.
 * The clock rates are initialized in `kinetis_clock_init()`.
 */
static struct clk clk_uart[6];

/*
 * Enable the USB-HS module clock
 */
static void usbhs_clk_enable(struct clk *clk)
{
	local_clk_enable(clk);

	/* Wait for the clock to stabilize before accessing the register map */
	mdelay(10);
}


/*
 * USB-HS module clock
 */
static struct clk clk_usbhs = {
	.gate = KINETIS_CG_USBHS,
	.clk_enable = usbhs_clk_enable,
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
	INIT_CLKREG(&clk_uart[0], "kinetis-uart.0", NULL),
	INIT_CLKREG(&clk_uart[1], "kinetis-uart.1", NULL),
	INIT_CLKREG(&clk_uart[2], "kinetis-uart.2", NULL),
	INIT_CLKREG(&clk_uart[3], "kinetis-uart.3", NULL),
	INIT_CLKREG(&clk_uart[4], "kinetis-uart.4", NULL),
	INIT_CLKREG(&clk_uart[5], "kinetis-uart.5", NULL),
	INIT_CLKREG(&clk_usbhs, "mxc-ehci.0", "usb"),
};

/*
 * Clock values
 */
static u32 clock_val[CLOCK_END];

/*
 * Maximum values for the LCDC clock divisor and fraction
 */
/* LCDCDIV: 12 bits, up to 4096 */
#define KINETIS_LCDC_MAX_NUMERATOR	((KINETIS_SIM_CLKDIV3_LCDCDIV_MSK >> \
					KINETIS_SIM_CLKDIV3_LCDCDIV_BITS) + 1)
/* LCDCFRAC: 8 bits, up to 256 */
#define KINETIS_LCDC_MAX_DENOMINATOR	((KINETIS_SIM_CLKDIV3_LCDCFRAC_MSK >> \
					KINETIS_SIM_CLKDIV3_LCDCFRAC_BITS) + 1)

/*
 * Update the "rate" property of the registered LCDC clock
 */
static void kinetis_lcdc_update_clock_rate(void)
{
	clk_lcdc.rate = clock_val[CLOCK_MCGOUTCLK] /
		(((KINETIS_SIM->clkdiv3 &
			KINETIS_SIM_CLKDIV3_LCDCDIV_MSK) >>
		KINETIS_SIM_CLKDIV3_LCDCDIV_BITS) + 1) *
		(((KINETIS_SIM->clkdiv3 &
			KINETIS_SIM_CLKDIV3_LCDCFRAC_MSK) >>
		KINETIS_SIM_CLKDIV3_LCDCFRAC_BITS) + 1);
}

/*
 * Adjust the LCDC clock divider to produce a frequency as close as possible
 * to the requested value.
 */
void kinetis_lcdc_adjust_clock_divider(unsigned long clock, unsigned long base)
{
	/*
	 * Length limit for the continued fraction. In the worst case
	 * of the integer parts in the continued fraction being all ones
	 * (golden ratio), numerator and denominator of the evaluated continued
	 * fraction grow like Fibonacci numbers, or like powers of golden
	 * ratio. This is why for the length limit (N) we choose the logarithm
	 * of 4096 (LCDCDIV maximum value) to base of golden ratio (1.618...),
	 * plus a few extra array elements for safety.
	 */
	const int N = 20;

	/* Integer parts in the continued fraction */
	u32 seq[N];
	/* Indices in the "seq" array */
	int i, j;
	/* Temporary variables for evaluation of continued fraction */
	u32 a, b, c;
	/* Final numerator and denominator */
	u32 last_a = 1, last_b = 1;
	/* Target value, multiplied by 2**16 */
	u32 x;

	/* x = (base << 16) / clock */
	u64 tmp = (u64)base << 16;
	do_div(tmp, clock);
	x = (u32)tmp;

	/*
	 * Build the continued fraction (Diophantine approximation of "x")
	 */
	for (i = 0; i < N; i++) {
		/* floor(x) */
		seq[i] = x >> 16;

		/* Result is a/b, "c" is a helper variable */
		a = 1;
		b = 0;
		for (j = i; j >= 0; j--) {
			/* Partial fraction p_{next} = seq + p^{-1} */
			/* New numerator */
			c = seq[j] * a + b;
			/* New denominator */
			b = a;
			/* Write nominator into the correct variable */
			a = c;

			if (a > KINETIS_LCDC_MAX_NUMERATOR ||
			    b > KINETIS_LCDC_MAX_DENOMINATOR)
				break;
		}

		if (b > KINETIS_LCDC_MAX_DENOMINATOR ||
		    a > KINETIS_LCDC_MAX_NUMERATOR) {
			/* Stop approximation on exceeding of the limits */
			break;
		} else {
			/* Approximation is good, save it */
			last_a = a;
			last_b = b;
		}

		/* Keep fractional part */
		x &= 0xffff;
		/* Exit if we are already very close to the target value */
		if (x < 2)
			break;

		/* x = 1.0/x */
		x = ((u32)-1) / x;
	}

	/*
	 * Write LCDC clock divider values to the SIM_CLKDIV3 register
	 */
	KINETIS_SIM->clkdiv3 =
		((last_a - 1) << KINETIS_SIM_CLKDIV3_LCDCDIV_BITS) |
		((last_b - 1) << KINETIS_SIM_CLKDIV3_LCDCFRAC_BITS);

	/*
	 * Update LCDC clock rate
	 */
	kinetis_lcdc_update_clock_rate();
}

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
	/* PLL0 or PLL1 used to generate MCGOUTCLK ("main" PLL) */
	int pll_sel;
	/* OSC0 or OSC1 used as clock source for the "main" PLL */
	int osc_sel;
	/* Frequency at the MCGOUTCLK output of the MCG */
	int mcgout;
	/* USB High Speed clock divider values */
	int usbhs_div;
	int usbhs_frac;

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
	 * Check whether PLL0 or PLL1 is used for MCGOUTCLK
	 */
	pll_sel = !!(KINETIS_MCG->c11 & KINETIS_MCG_C11_PLLCS_MSK);

	/*
	 * Check whether OSC0 or OSC1 is used to source the main PLL
	 */
	if (pll_sel)
		osc_sel = !!(KINETIS_MCG->c11 & KINETIS_MCG_C11_PLLREFSEL1_MSK);
	else
		osc_sel = !!(KINETIS_MCG->c5 & KINETIS_MCG_C5_PLLREFSEL_MSK);

	/*
	 * Start with the MCG input clock
	 */
	mcgout = osc_sel ? CONFIG_KINETIS_OSC1_RATE : CONFIG_KINETIS_OSC0_RATE;

	/*
	 * Apply dividers and multipliers of the selected PLL
	 */
	if (pll_sel) {
		/*
		 * PLL1 internal divider (PRDIV)
		 */
		mcgout /= ((KINETIS_MCG->c11 & KINETIS_MCG_C11_PRDIV_MSK) >>
			KINETIS_MCG_C11_PRDIV_BITS) + 1;
		/*
		 * PLL1 multiplication factor (VDIV)
		 */
		mcgout *= ((KINETIS_MCG->c12 & KINETIS_MCG_C12_VDIV1_MSK) >>
			KINETIS_MCG_C12_VDIV1_BITS) + vdiv_min;
	} else {
		/*
		 * PLL0 internal divider (PRDIV)
		 */
		mcgout /= ((KINETIS_MCG->c5 & KINETIS_MCG_C5_PRDIV_MSK) >>
			KINETIS_MCG_C5_PRDIV_BITS) + 1;
		/*
		 * PLL0 multiplication factor (VDIV)
		 */
		mcgout *= ((KINETIS_MCG->c6 & KINETIS_MCG_C6_VDIV_MSK) >>
			KINETIS_MCG_C6_VDIV_BITS) + vdiv_min;
	}

	/*
	 * Apply the PLL output divider
	 */
	mcgout /= vco_div;

	/*
	 * Save MCGOUTCLK frequency
	 */
	clock_val[CLOCK_MCGOUTCLK] = mcgout;

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
	clock_val[CLOCK_MACCLK] = CONFIG_KINETIS_OSC0_RATE;

	/*
	 * LCD Controller clock
	 */
	kinetis_lcdc_update_clock_rate();

	/*
	 * USB High Speed controller clock
	 */
	/* Use fractional divider if "msgout" is not a multiple of 60MHz */
	usbhs_frac = (mcgout % KINETIS_USBHS_REQ_RATE) != 0;
	usbhs_div = mcgout * (usbhs_frac + 1) / KINETIS_USBHS_REQ_RATE - 1;
	if (usbhs_div < 0)
		usbhs_div = 0;
	if (usbhs_div > 7)
		usbhs_div = 7;
	clock_val[CLOCK_USBHS] = mcgout * (usbhs_frac + 1) / (usbhs_div + 1);
	/* Write USB-HS clock configuration to registers */
	if (clock_val[CLOCK_USBHS] == KINETIS_USBHS_REQ_RATE) {
		/* Do not output the 60MHz ULPI clock */
		KINETIS_SIM->mcr &= ~KINETIS_SIM_MCR_ULPICLKOBE_MSK;
		/* Clock source: MCGOUT clock (PLL0 or PLL1) */
		KINETIS_SIM->sopt2 =
			(KINETIS_SIM->sopt2 & ~KINETIS_SIM_SOPT2_USBHSRC_MSK) |
			(pll_sel ? KINETIS_SIM_SOPT2_USBHSRC_PLL1 :
				KINETIS_SIM_SOPT2_USBHSRC_PLL0);
		/* Clock divider */
		KINETIS_SIM->clkdiv2 =
			(KINETIS_SIM->clkdiv2 &
				(KINETIS_SIM_CLKDIV2_USBHSDIV_MSK |
				KINETIS_SIM_CLKDIV2_USBHSFRAC_MSK)) |
			(usbhs_frac << KINETIS_SIM_CLKDIV2_USBHSFRAC_BIT) |
			(usbhs_div << KINETIS_SIM_CLKDIV2_USBHSDIV_BIT);
	} else {
		clock_val[CLOCK_USBHS] = 0;
	}

	/*
	 * Initialize the `clk_*` structures
	 */
	clk_net.rate = clock_val[CLOCK_MACCLK];
	/*
	 * UART0 and UART1 are clocked from the core clock, the remaining UARTs
	 * are clocked from the bus clock.
	 */
	for (i = 0; i < ARRAY_SIZE(clk_uart); i++)
		clk_uart[i].rate = clock_val[i <= 1 ? CLOCK_CCLK : CLOCK_PCLK];

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
