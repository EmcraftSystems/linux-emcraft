/*
 * (C) Copyright 2011-2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusainov, <vlad@emcraft.com>
 * Pavel Boldin, <paboldin@emcraft.com>
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

#if defined (CONFIG_FB_ARMCLCD)
# include <linux/amba/clcd.h>
# include <mach/fb.h>
#endif

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
	int ret = -EINVAL;
	if (clk->clk_set_rate)
		ret = clk->clk_set_rate(clk, rate);

	return ret;
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
 * Clock for the SD/MMC block
 */
#if defined (CONFIG_LPC18XX_MMC)
static struct clk clk_sdio;
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

#if defined (CONFIG_FB_ARMCLCD)
/*
 * The `amba-clcd` driver expects this function to configure the LCD clock
 * divider and set the desired pixel clock rate.
 *
 * The code in this function is based on the code from
 * `linux-2.6.34-lpc32xx/arch/arm/mach-lpc32xx/clock.c` from
 * `http://lpclinux.com/`.
 */
static int lcd_clk_set_rate(struct clk *clk, unsigned long rate)
{
	u32 tmp, prate, div;

	tmp = __raw_readl(LPC18XX_LCD_BASE + CLCD_TIM2) | TIM2_BCD;
	/* CPU clock is the source clock for the LCD controller */
	prate = lpc18xx_clock_get(CLOCK_CCLK);

	if (rate < prate) {
		/* Find closest divider */
		div = prate / rate;
		if (div >= 2) {
			div -= 2;
			tmp &= ~TIM2_BCD;
		}

		tmp &= ~(0xF800001F);
		tmp |= (div & 0x1F);
		tmp |= (((div >> 5) & 0x1F) << 27);
	}

	__raw_writel(tmp, LPC18XX_LCD_BASE + CLCD_TIM2);

	return 0;
}

/*
 * The `amba-clcd` driver does not really call this function, but we implement
 * just in case. Similarly to `lcd_clk_set_rate()`, this function returns
 * the actual pixel clock rate.
 *
 * The code in this function is based on the code from
 * `linux-2.6.34-lpc32xx/arch/arm/mach-lpc32xx/clock.c` from
 * `http://lpclinux.com/`.
 */
unsigned long lcd_clk_get_rate(struct clk *clk)
{
	u32 tmp, div, rate;

	/* The LCD clock must be on when accessing an LCD register */
	tmp = __raw_readl(LPC18XX_LCD_BASE + CLCD_TIM2);

	/* CPU clock is the source clock for the LCD controller */
	rate = lpc18xx_clock_get(CLOCK_CCLK);

	/* Only supports internal clocking */
	if (!(tmp & TIM2_BCD)) {
		div = (tmp & 0x1F) | ((tmp & 0xF8) >> 22);
		rate /= (2 + div);
	}

	return rate;
}

static struct clk clk_lcd = {
	.clk_set_rate	= lcd_clk_set_rate,
	.clk_get_rate	= lcd_clk_get_rate,
};
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
#if defined (CONFIG_LPC18XX_MMC)
	INIT_CLKREG(&clk_sdio, NULL, "sdio"),
#endif
#if defined (CONFIG_LPC18XX_I2C0)
	INIT_CLKREG(&clk_i2c0, "lpc2k-i2c.0", NULL),
#endif
#if defined (CONFIG_LPC18XX_I2C1)
	INIT_CLKREG(&clk_i2c1, "lpc2k-i2c.1", NULL),
#endif
#if defined (CONFIG_FB_ARMCLCD)
	INIT_CLKREG(&clk_lcd, "dev:clcd", NULL),
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

#if defined (CONFIG_FB_ARMCLCD)
	LPC18XX_CGU->lcd_clk = LPC18XX_CGU_CLKSEL_PLL1 |
			LPC18XX_CGU_PLL1CTRL_AUTOBLOCK_MSK;
	clk_lcd.rate = pll1_out;
#endif

#if defined (CONFIG_LPC18XX_I2C0)
	clk_i2c0.rate = clock_val[CLOCK_PCLK];
#endif
#if defined (CONFIG_LPC18XX_I2C1)
	clk_i2c1.rate = clock_val[CLOCK_PCLK];
#endif

#if defined (CONFIG_LPC18XX_MMC)
	LPC18XX_CGU->sdio_clk = LPC18XX_CGU_CLKSEL_PLL1 |
			LPC18XX_CGU_PLL1CTRL_AUTOBLOCK_MSK;
	clk_sdio.rate = pll1_out;
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
