/*
 * (C) Copyright 2011-2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusainov, <vlad@emcraft.com>
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
#include <mach/lpc18xx.h>

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
/* Crystal oscillator */
#define LPC18XX_CGU_CLKSEL_XTAL		(0x06 << LPC18XX_CGU_CLKSEL_BITS)
/* PLL1 */
#define LPC18XX_CGU_CLKSEL_PLL1		(0x09 << LPC18XX_CGU_CLKSEL_BITS)
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
};

static DEFINE_SPINLOCK(clocks_lock);

/*
 * In the future, we will enable the respective branch clock in this function
 */
static void local_clk_enable(struct clk *clk)
{
}

/*
 * In the future, we will disable the respective branch clock in this function
 */
static void local_clk_disable(struct clk *clk)
{
}

/*
 * clk_enable - inform the system when the clock source should be running.
 */
int clk_enable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->enabled++ == 0)
		local_clk_enable(clk);
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
	if (--clk->enabled == 0)
		local_clk_disable(clk);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

/*
 * clk_get_rate - obtain the current clock rate (in Hz) for a clock source
 */
unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long rate;

	if (clk->clk_get_rate)
		rate = clk->clk_get_rate(clk);
	else
		rate = clk->rate;

	return rate;
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
 * Clocks for each of the four UARTs supported by the LPC18xx/LPC43xx MCUs.
 * The clock rates are initialized in `lpc18xx_clock_init()`.
 */
static struct clk clk_uart[4];

/*
 * Clock for the Ethernet module of the MCU. The clock rate is initialized
 * in `lpc18xx_clock_init()`.
 */
static struct clk clk_net;

/*
 * Clocks for the two SSP/SPI ports of the LPC18xx/LPC43xx.
 * The clock rates are initialized in `lpc18xx_clock_init()`.
 */
#if defined (CONFIG_LPC18XX_SPI0)
static struct clk clk_ssp0;
#endif
#if defined (CONFIG_LPC18XX_SPI1)
static struct clk clk_ssp1;
#endif

/*
 * Clocks for each of the two I2C interfaces supported by the LPC18xx MCU.
 */
#if defined (CONFIG_LPC18XX_I2C0)
static struct clk clk_i2c0;
#endif
#if defined (CONFIG_LPC18XX_I2C1)
static struct clk clk_i2c1;
#endif

/*
 * Array of all clock to register with the `clk_*` infrastructure
 */
#define INIT_CLKREG(_clk,_devname,_conname)		\
	{						\
		.clk		= _clk,			\
		.dev_id		= _devname,		\
		.con_id		= _conname,		\
	}
static struct clk_lookup lpc18xx_clkregs[] = {
	INIT_CLKREG(&clk_uart[0], NULL, "lpc18xx-uart.0"),
	INIT_CLKREG(&clk_uart[1], NULL, "lpc18xx-uart.1"),
	INIT_CLKREG(&clk_uart[2], NULL, "lpc18xx-uart.2"),
	INIT_CLKREG(&clk_uart[3], NULL, "lpc18xx-uart.3"),
	INIT_CLKREG(&clk_net, "eth0", NULL),
#if defined (CONFIG_LPC18XX_SPI0)
	INIT_CLKREG(&clk_ssp0, "dev:ssp0", NULL),
#endif
#if defined (CONFIG_LPC18XX_SPI1)
	INIT_CLKREG(&clk_ssp1, "dev:ssp1", NULL),
#endif
#if defined (CONFIG_LPC18XX_I2C0)
	INIT_CLKREG(&clk_i2c0, "lpc2k-i2c.0", NULL),
#endif
#if defined (CONFIG_LPC18XX_I2C1)
	INIT_CLKREG(&clk_i2c1, "lpc2k-i2c.1", NULL),
#endif
};

/*
 * Clock rate values
 */
static u32 clock_val[CLOCK_END];

/*
 * Initialize the reference clocks.
 */
void __init lpc18xx_clock_init(void)
{
	int i;
	int platform;

	/* External oscillator frequency */
	int extosc;
	/* Frequency at the output of PLL1 */
	int pll1_out;

	/*
	 * Avoid double initialization of clocks
	 */
	static int init_done;
	if (init_done)
		goto exit;
	init_done = 1;

	/*
	 * Verify the expected clock configuration:
	 *     PLL1 powered-on, no bypass, external osc, etc.
	 */
	if ((LPC18XX_CGU->pll1_ctrl &
		(LPC18XX_CGU_PLL1CTRL_PD_MSK |
		LPC18XX_CGU_PLL1CTRL_BYPASS_MSK |
		LPC18XX_CGU_PLL1CTRL_FBSEL_MSK |
		LPC18XX_CGU_PLL1CTRL_NSEL_MSK |
		LPC18XX_CGU_CLKSEL_MSK)) !=
	    (LPC18XX_CGU_PLL1CTRL_FBSEL_MSK |
		(0 << LPC18XX_CGU_PLL1CTRL_NSEL_BITS) |
		LPC18XX_CGU_CLKSEL_XTAL)) {
		pr_err("%s: Unexpected configuration of PLL1: %08x\n",
			__func__, LPC18XX_CGU->pll1_ctrl);
	}
	/*
	 * Verify that all clocks are sourced from PLL1
	 */
	if ((LPC18XX_CGU->m4_clk & LPC18XX_CGU_CLKSEL_MSK) !=
		LPC18XX_CGU_CLKSEL_PLL1) {
		pr_err("%s: M4 core clock is not sourced from PLL1\n",
			__func__);
	}
	if ((LPC18XX_CGU->uart0_clk & LPC18XX_CGU_CLKSEL_MSK) !=
		LPC18XX_CGU_CLKSEL_PLL1) {
		pr_err("%s: USART0 clock is not sourced from PLL1\n", __func__);
	}
	if ((LPC18XX_CGU->uart1_clk & LPC18XX_CGU_CLKSEL_MSK) !=
		LPC18XX_CGU_CLKSEL_PLL1) {
		pr_err("%s: UART1 clock is not sourced from PLL1\n", __func__);
	}
	if ((LPC18XX_CGU->uart2_clk & LPC18XX_CGU_CLKSEL_MSK) !=
		LPC18XX_CGU_CLKSEL_PLL1) {
		pr_err("%s: USART2 clock is not sourced from PLL1\n", __func__);
	}
	if ((LPC18XX_CGU->uart3_clk & LPC18XX_CGU_CLKSEL_MSK) !=
		LPC18XX_CGU_CLKSEL_PLL1) {
		pr_err("%s: USART3 clock is not sourced from PLL1\n", __func__);
	}

	/*
	 * Initialize `extosc` to the rate of the on-board external oscillator
	 */
	platform = lpc18xx_platform_get();
	switch (platform) {
	case PLATFORM_LPC18XX_HITEX_LPC4350_EVAL:
	case PLATFORM_LPC18XX_HITEX_LPC1850_EVAL:
		extosc = 12000000;	/* 12 MHz */
		break;
	default:
		/*
		 * Let's assume smth in this case; maybe things
		 * will work...
		 */
		extosc = 12000000;	/* 12 MHz */
		printk(KERN_WARNING "%s: unsupported platform %d\n",
			__func__, platform);
		break;
	}

	/*
	 * Initialize PLL1 output rate
	 */
	pll1_out = extosc;
	pll1_out *= 1 +
		((LPC18XX_CGU->pll1_ctrl & LPC18XX_CGU_PLL1CTRL_MSEL_MSK) >>
		LPC18XX_CGU_PLL1CTRL_MSEL_BITS);

	/*
	 * All these clocks are sourced from PLL1 without dividers
	 */
	clock_val[CLOCK_CCLK] = pll1_out;
	clock_val[CLOCK_SYSTICK] = pll1_out;
	clock_val[CLOCK_PCLK] = pll1_out;

	/*
	 * Initialize the `clk_*` structures
	 */
	/* CLK_M4_ETHERNET is branch clock of BASE_M4_CLK, same frequencies */
	clk_net.rate = clock_val[CLOCK_CCLK];

	/* All UART clocks are sourced from PLL1 without dividers */
	for (i = 0; i < ARRAY_SIZE(clk_uart); i++)
		clk_uart[i].rate = pll1_out;

	/* All SSP/SPI clocks are sourced from PLL1 without dividers */
#if defined (CONFIG_LPC18XX_SPI0)
	LPC18XX_CGU->ssp0_clk = LPC18XX_CGU_CLKSEL_PLL1 |
			LPC18XX_CGU_PLL1CTRL_AUTOBLOCK_MSK;
	clk_ssp0.rate = pll1_out;
#endif
#if defined (CONFIG_LPC18XX_SPI1)
	LPC18XX_CGU->ssp1_clk = LPC18XX_CGU_CLKSEL_PLL1 |
			LPC18XX_CGU_PLL1CTRL_AUTOBLOCK_MSK;
	clk_ssp1.rate = pll1_out;
#endif

#if defined (CONFIG_LPC18XX_I2C0)
	clk_i2c0.rate = clock_val[CLOCK_PCLK];
#endif
#if defined (CONFIG_LPC18XX_I2C1)
	clk_i2c1.rate = clock_val[CLOCK_PCLK];
#endif

	/*
	 * Register clocks with the `clk_*` infrastructure
	 */
	for (i = 0; i < ARRAY_SIZE(lpc18xx_clkregs); i++)
		clkdev_add(&lpc18xx_clkregs[i]);

exit:
	;
}

/*
 * Return a clock value for the specified clock.
 *
 * This is the legacy way to get a clock rate, independent
 * from `clk_get_rate()`.
 */
unsigned int lpc18xx_clock_get(enum lpc18xx_clock clk)
{
	return clock_val[clk];
}
