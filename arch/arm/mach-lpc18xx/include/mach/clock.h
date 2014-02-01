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

#ifndef _MACH_LPC18XX_CLOCK_H_
#define _MACH_LPC18XX_CLOCK_H_

/*
 * Clocks enumeration
 */
enum lpc18xx_clock {
	CLOCK_CCLK,		/* CPU clock frequency expressed in Hz        */
	CLOCK_SYSTICK,		/* Systimer clock frequency expressed in Hz   */
	CLOCK_PCLK,		/* Peripheral clock frequency expressed in Hz */
	CLOCK_END		/* for internal usage			      */
};

/*
 * Initialize the clock section of the LPC18xx/LPC43xx
 */
void __init lpc18xx_clock_init(void);

/*
 * Return a clock value for the specified clock
 */
unsigned int lpc18xx_clock_get(enum lpc18xx_clock clk);

/*
 * PLL0* register map
 * Used for PLL0USB at 0x4005001C and for PLL0AUDIO at 0x4005002C.
 *
 * This structure is 0x10 bytes long, it is important when it embedding into
 * `struct lpc18xx_cgu_regs`.
 */
struct lpc18xx_pll0_regs {
	u32 stat;	/* PLL status register */
	u32 ctrl;	/* PLL control register */
	u32 mdiv;	/* PLL M-divider register */
	u32 np_div;	/* PLL N/P-divider register */
};

/*
 * CGU (Clock Generation Unit) register map
 * Should be mapped at 0x40050000.
 */
struct lpc18xx_cgu_regs {
	u32 rsv0[5];
	u32 freq_mon;		/* Frequency monitor */
	u32 xtal_osc_ctrl;	/* XTAL oscillator control */
	struct lpc18xx_pll0_regs pll0usb;	/* PLL0USB registers */
	struct lpc18xx_pll0_regs pll0audio;	/* PLL0AUDIO registers */
	u32 pll0audio_frac;	/* PLL0AUDIO fractional divider */
	u32 pll1_stat;		/* PLL1 status register */
	u32 pll1_ctrl;		/* PLL1 control register */
	u32 idiv[5];		/* IDIVA_CTRL .. IDIVE_CTRL */

	/* BASE_* clock configuration registers */
	u32 safe_clk;
	u32 usb0_clk;
	u32 periph_clk;
	u32 usb1_clk;
	u32 m4_clk;
	u32 spifi_clk;
	u32 spi_clk;
	u32 phy_rx_clk;
	u32 phy_tx_clk;
	u32 apb1_clk;
	u32 apb3_clk;
	u32 lcd_clk;
	u32 vadc_clk;
	u32 sdio_clk;
	u32 ssp0_clk;
	u32 ssp1_clk;
	u32 uart0_clk;
	u32 uart1_clk;
	u32 uart2_clk;
	u32 uart3_clk;
	u32 out_clk;
	u32 rsv1[4];
	u32 apll_clk;
	u32 cgu_out0_clk;
	u32 cgu_out1_clk;
};

/*
 * CGU registers base
 */
#define LPC18XX_CGU_BASE		0x40050000
#define LPC18XX_CGU			((volatile struct lpc18xx_cgu_regs *) \
					LPC18XX_CGU_BASE)

/*
 * Bit offsets in Clock Generation Unit (CGU) registers
 */
/*
 * Crystal oscillator control register (XTAL_OSC_CTRL)
 */
/* Oscillator-pad enable */
#define LPC18XX_CGU_XTAL_ENABLE		(1 << 0)
/* Select frequency range */
#define LPC18XX_CGU_XTAL_HF		(1 << 2)
/*
 * For all CGU clock registers
 */
/* CLK_SEL: Clock source selection */
#define LPC18XX_CGU_CLKSEL_BITS		24
#define LPC18XX_CGU_CLKSEL_MSK		(0x1F << LPC18XX_CGU_CLKSEL_BITS)
/* ENET_RX_CLK */
#define LPC18XX_CGU_CLKSEL_ENET_RX	(0x02 << LPC18XX_CGU_CLKSEL_BITS)
/* ENET_TX_CLK */
#define LPC18XX_CGU_CLKSEL_ENET_TX	(0x03 << LPC18XX_CGU_CLKSEL_BITS)
/* Crystal oscillator */
#define LPC18XX_CGU_CLKSEL_XTAL		(0x06 << LPC18XX_CGU_CLKSEL_BITS)
/* PLL1 */
#define LPC18XX_CGU_CLKSEL_PLL1		(0x09 << LPC18XX_CGU_CLKSEL_BITS)
/* Block clock automatically during frequency change */
#define LPC18XX_CGU_AUTOBLOCK_MSK	(1 << 11)
/*
 * PLL1 control register
 */
/* PLL1 power down */
#define LPC18XX_CGU_PLL1CTRL_PD_MSK		(1 << 0)
/* Input clock bypass control */
#define LPC18XX_CGU_PLL1CTRL_BYPASS_MSK		(1 << 1)
/* PLL feedback select */
#define LPC18XX_CGU_PLL1CTRL_FBSEL_MSK		(1 << 6)
/* PLL direct CCO output */
#define LPC18XX_CGU_PLL1CTRL_DIRECT_MSK		(1 << 7)
/* Post-divider division ratio P. The value applied is 2**P. */
#define LPC18XX_CGU_PLL1CTRL_PSEL_MSK		(3 << 8)
/* Block clock automatically during frequency change */
#define LPC18XX_CGU_PLL1CTRL_AUTOBLOCK_MSK	(1 << 11)
/* Pre-divider division ratio */
#define LPC18XX_CGU_PLL1CTRL_NSEL_BITS		12
#define LPC18XX_CGU_PLL1CTRL_NSEL_MSK \
	(3 << LPC18XX_CGU_PLL1CTRL_NSEL_BITS)
/* Feedback-divider division ratio (M) */
#define LPC18XX_CGU_PLL1CTRL_MSEL_BITS		16
#define LPC18XX_CGU_PLL1CTRL_MSEL_MSK \
	(0xFF << LPC18XX_CGU_PLL1CTRL_MSEL_BITS)
/*
 * PLL1 status register
 */
/* PLL1 lock indicator */
#define LPC18XX_CGU_PLL1STAT_LOCK	(1 << 0)

/*
 * The structure that holds the information about a clock
 */
struct clk {
	/* Clock rate, the only possible value of */
	unsigned long rate;

	/* Reference count */
	unsigned int enabled;

	/* Custom `clk_*` functions */
	unsigned long (*clk_get_rate)(struct clk *clk);
	unsigned long (*clk_set_rate)(struct clk *clk, unsigned long rate);
};


#endif	/*_MACH_LPC18XX_CLOCK_H_ */
