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
#include <linux/amba/clcd.h>

#include <asm/clkdev.h>

#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/lpc178x.h>
#include <mach/power.h>
#include <mach/fb.h>

/*
 * Internal oscillator value
 *
 * This frequency should be set to the same value as in U-Boot.
 */
#define CONFIG_LPC178X_INTOSC_RATE	12000000	/* 12 MHz */

/*
 * Bit offsets in System and Clock Control (SCC) registers
 */
/*
 * Clock Source Selection register bits
 */
#define LPC178X_SCC_CLKSRCSEL_CLKSRC_MSK	(1 << 0)

/*
 * PLL Configuration register bits
 */
#define LPC178X_SCC_PLLCFG_MSEL_BITS		0
#define LPC178X_SCC_PLLCFG_MSEL_MSK		((1 << 5) - 1)	/* bits 4:0 */
#define LPC178X_SCC_PLLCFG_PSEL_BITS		5

/*
 * CPU Clock Selection register bits
 */
#define LPC178X_SCC_CCLKSEL_CCLKDIV_BITS	0
#define LPC178X_SCC_CCLKSEL_CCLKDIV_MSK		((1 << 5) - 1)	/* bits 4:0 */
#define LPC178X_SCC_CCLKSEL_CCLKSEL_MSK		(1 << 8)

/*
 * Peripheral Clock Selection register
 */
#define LPC178X_SCC_PCLKSEL_PCLKDIV_BITS	0
#define LPC178X_SCC_PCLKSEL_PCLKDIV_MSK		((1 << 5) - 1)	/* bits 4:0 */

/*
 * EMC Clock Selection register
 */
#define LPC178X_SCC_EMCCLKSEL_HALFCPU_MSK	(1 << 0)

/*
 * USB Clock Selection register
 */
/* Selects the divide value for creating the USB clock */
#define LPC178X_SCC_USBCLKSEL_USBDIV_BITS	0
/* The mask for all bits of USBCLKSEL[USBDIV] */
#define LPC178X_SCC_USBCLKSEL_USBDIV_MSK \
	(((1 << 5) - 1) << LPC178X_SCC_USBCLKSEL_USBDIV_BITS)
/* Selects the input clock for the USB clock divider */
#define LPC178X_SCC_USBCLKSEL_USBSEL_BITS	8
/* The mask for all bits of USBCLKSEL[USBSEL] */
#define LPC178X_SCC_USBCLKSEL_USBSEL_MSK \
	(3 << LPC178X_SCC_USBCLKSEL_USBSEL_BITS)
/* The output of the Alt PLL is used as the input to the USB clock divider */
#define LPC178X_SCC_USBCLKSEL_USBSEL_PLL1_MSK \
	(2 << LPC178X_SCC_USBCLKSEL_USBSEL_BITS)

/*
 * The structure that holds the information about a clock
 */
struct clk {
	/* Clock rate, the only possible value of */
	unsigned long rate;
	/* For `lpc178x_periph_enable()` */
	u32 pconp_mask;

	/* Reference count */
	unsigned int enabled;

	/* Custom `clk_*` functions */
	int (*clk_set_rate)(struct clk *clk, unsigned long rate);
	unsigned long (*clk_get_rate)(struct clk *clk);
};

static DEFINE_SPINLOCK(clocks_lock);

/*
 * Enable the clock via the Power Control for Peripherals register
 */
static void local_clk_enable(struct clk *clk)
{
	lpc178x_periph_enable(clk->pconp_mask, 1);
}

/*
 * Disable the clock via the Power Control for Peripherals register
 */
static void local_clk_disable(struct clk *clk)
{
	lpc178x_periph_enable(clk->pconp_mask, 0);
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
	int ret;

	if (clk->clk_set_rate)
		ret = clk->clk_set_rate(clk, rate);
	else
		ret = -EINVAL;

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

	tmp = __raw_readl(LPC178X_LCD_BASE + CLCD_TIM2) | TIM2_BCD;
	/* CPU clock is the source clock for the LCD controller */
	prate = lpc178x_clock_get(CLOCK_CCLK);

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

	__raw_writel(tmp, LPC178X_LCD_BASE + CLCD_TIM2);

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
	tmp = __raw_readl(LPC178X_LCD_BASE + CLCD_TIM2);

	/* CPU clock is the source clock for the LCD controller */
	rate = lpc178x_clock_get(CLOCK_CCLK);

	/* Only supports internal clocking */
	if (!(tmp & TIM2_BCD)) {
		div = (tmp & 0x1F) | ((tmp & 0xF8) >> 22);
		rate /= (2 + div);
	}

	return rate;
}

/*
 * Clock for the Ethernet module of the MCU. The clock rate is initialized
 * in `lpc178x_clock_init()`.
 */
static struct clk clk_net = {
	.pconp_mask	= LPC178X_SCC_PCONP_PCENET_MSK,
};

/*
 * Clock for the General Purpose DMA module of the MCU. The clock rate
 * is initialized in `lpc178x_clock_init()`.
 */
static struct clk clk_dma = {
	.pconp_mask	= LPC178X_SCC_PCONP_PCGPDMA_MSK,
};

/*
 * Clock for SSP/SPI port SPI0 of the LPC178x.
 * The clock rate is initialized in `lpc178x_clock_init()`.
 */
#if defined (CONFIG_LPC178X_SPI0)
static struct clk clk_ssp0 = {
	.pconp_mask	= LPC178X_SCC_PCONP_PCSSP0_MSK,
};
#endif

/*
 * Clock for the SD Card Interface module of the MCU. The clock rate
 * is initialized in `lpc178x_clock_init()`.
 */
static struct clk clk_mci = {
	.pconp_mask	= LPC178X_SCC_PCONP_PCSDC_MSK,
};

/*
 * Clocks for each of the three I2C interfaces supported by the LPC178x/7x MCU.
 * The clock rate is initialized in `lpc178x_clock_init()`.
 */
#if defined(CONFIG_LPC178X_I2C0)
static struct clk clk_i2c0 = { .pconp_mask = LPC178X_SCC_PCONP_PCI2C0_MSK };
#endif
#if defined(CONFIG_LPC178X_I2C1)
static struct clk clk_i2c1 = { .pconp_mask = LPC178X_SCC_PCONP_PCI2C1_MSK };
#endif
#if defined(CONFIG_LPC178X_I2C2)
static struct clk clk_i2c2 = { .pconp_mask = LPC178X_SCC_PCONP_PCI2C2_MSK };
#endif

/*
 * Clock for the LCD contoller module of the MCU.
 *
 * The `amba-clcd` framebuffer driver that we use for LPC178x/7x requires the
 * LCD clock driver to be able to control the pixel clock by using
 * `clk_set_rate()`.
 */
static struct clk clk_lcd = {
	.pconp_mask	= LPC178X_SCC_PCONP_PCLCD_MSK,
	.clk_set_rate	= lcd_clk_set_rate,
	.clk_get_rate	= lcd_clk_get_rate,
};

/*
 * Clock for the I2S module of the MCU.
 *
 * The clock rate is initialized in lpc178x_clock_init().
 */
static struct clk clk_i2s = {
	.pconp_mask	= LPC178X_SCC_PCONP_PCI2S_MSK,
};

/*
 * Clock for the WDT module of the MCU.
 * The clock rate is fixed at 500 kHz.
 * The clocks are enabled automatically by enabling WDT.
 */
static struct clk clk_wdt = {
	.pconp_mask	= 0,
	.rate		= 500000
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
static struct clk_lookup lpc178x_clkregs[] = {
	INIT_CLKREG(&clk_net, "lpc-net.0", NULL),
#if defined (CONFIG_LPC178X_SPI0)
	INIT_CLKREG(&clk_ssp0, "dev:ssp0", NULL),
#endif
	INIT_CLKREG(&clk_dma, NULL, "clk_dmac"),
	INIT_CLKREG(&clk_mci, "dev:mmc0", NULL),
#if defined(CONFIG_LPC178X_I2C0)
	INIT_CLKREG(&clk_i2c0, "lpc2k-i2c.0", NULL),
#endif
#if defined(CONFIG_LPC178X_I2C1)
	INIT_CLKREG(&clk_i2c1, "lpc2k-i2c.1", NULL),
#endif
#if defined(CONFIG_LPC178X_I2C2)
	INIT_CLKREG(&clk_i2c2, "lpc2k-i2c.2", NULL),
#endif
	INIT_CLKREG(&clk_lcd, "dev:clcd", NULL),
	INIT_CLKREG(&clk_i2s, NULL, "i2s0_ck"),
	INIT_CLKREG(&clk_wdt, "lpc2k-wdt", NULL),
};

/*
 * Clock rate values
 */
static u32 clock_val[CLOCK_END];

/*
 * Initialize the reference clocks.
 */
void __init lpc178x_clock_init(void)
{
	int i;
	int platform;

	/* External or internal oscillator frequency */
	int sysclk;
	/* Frequency at the output of PLL0 or sysclk (if PLL0 is not used) */
	int cclk_input;

	/*
	 * Initialize `sysclk` to the rate of the currently used oscillator
	 */
	sysclk = CONFIG_LPC178X_INTOSC_RATE;
	if (LPC178X_SCC->clksrcsel & LPC178X_SCC_CLKSRCSEL_CLKSRC_MSK) {
		platform = lpc178x_platform_get();
		switch (platform) {
		case PLATFORM_LPC178X_EA_LPC1788:
			/*
			 * This frequency should be set to the same value as
			 * in U-Boot for this board.
			 *
			 * See CONFIG_LPC178X_EXTOSC_RATE in
			 * u-boot/include/configs/ea-lpc1788.h
			 */
			sysclk = 12000000;	/* 12 MHz */
			break;
		case PLATFORM_LPC178X_LNX_EVB:
			/*
			 * This frequency should be set to the same value as
			 * in U-Boot for this board.
			 *
			 * See CONFIG_LPC178X_EXTOSC_RATE in
			 * u-boot/include/configs/lpc-lnx-evb.h
			 */
			sysclk = 12000000;	/* 12 MHz */
			break;
		default:
			/*
			 * Let's assume smth in this case; maybe things
			 * will work...
			 */
			sysclk = 12000000;	/* 12 MHz */
			printk(KERN_WARNING "%s: unsupported platform %d\n",
				__func__, platform);
			break;
		}
	}

	/*
	 * `cclk_input` is the frequency at the input of the CPU clock
	 * divider, Peripheral clock divider and EMC clock divider.
	 */
	cclk_input = sysclk;
	if (LPC178X_SCC->cclksel & LPC178X_SCC_CCLKSEL_CCLKSEL_MSK) {
		/* If PLL0 is enabled */
		cclk_input *= 1 +
			((LPC178X_SCC->pll0.cfg &
			LPC178X_SCC_PLLCFG_MSEL_MSK) >> LPC178X_SCC_PLLCFG_MSEL_BITS);
	}

	/*
	 * CPU clock
	 */
	clock_val[CLOCK_CCLK] = clock_val[CLOCK_SYSTICK] = cclk_input /
		((LPC178X_SCC->cclksel & LPC178X_SCC_CCLKSEL_CCLKDIV_MSK) >>
		LPC178X_SCC_CCLKSEL_CCLKDIV_BITS);

	/*
	 * Peripheral clock
	 */
	clock_val[CLOCK_PCLK] = cclk_input /
		((LPC178X_SCC->pclksel & LPC178X_SCC_PCLKSEL_PCLKDIV_MSK) >>
		LPC178X_SCC_PCLKSEL_PCLKDIV_BITS);

	/*
	 * EMC clock
	 */
	clock_val[CLOCK_EMCCLK] = cclk_input;
	if (LPC178X_SCC->emcclksel & LPC178X_SCC_EMCCLKSEL_HALFCPU_MSK)
		clock_val[CLOCK_EMCCLK] /= 2;

	/*
	 * USB clock
	 */
	clock_val[CLOCK_USBCLK] = 0;
	if ((LPC178X_SCC->usbclksel & LPC178X_SCC_USBCLKSEL_USBSEL_MSK) ==
	    LPC178X_SCC_USBCLKSEL_USBSEL_PLL1_MSK) {
		/* External oscillator rate */
		clock_val[CLOCK_USBCLK] = sysclk;

		/* Rate multiplication in the PLL1 */
		clock_val[CLOCK_USBCLK] *= 1 +
			((LPC178X_SCC->pll1.cfg &
			LPC178X_SCC_PLLCFG_MSEL_MSK) >> LPC178X_SCC_PLLCFG_MSEL_BITS);

		/* USB clock divider */
		clock_val[CLOCK_USBCLK] /=
			((LPC178X_SCC->usbclksel &
			LPC178X_SCC_USBCLKSEL_USBDIV_MSK) >>
			LPC178X_SCC_USBCLKSEL_USBDIV_BITS);
	}

	/*
	 * Initialize the `clk_*` structures
	 */
	clk_net.rate = clock_val[CLOCK_CCLK];	/* AHB clock = CPU clock */
	clk_dma.rate = clock_val[CLOCK_CCLK];	/* AHB clock = CPU clock */
	clk_mci.rate = clock_val[CLOCK_PCLK];
#if defined (CONFIG_LPC178X_SPI0)
	clk_ssp0.rate = clock_val[CLOCK_PCLK];
#endif
#if defined(CONFIG_LPC178X_I2C0)
	clk_i2c0.rate = clock_val[CLOCK_PCLK];
#endif
#if defined(CONFIG_LPC178X_I2C1)
	clk_i2c1.rate = clock_val[CLOCK_PCLK];
#endif
#if defined(CONFIG_LPC178X_I2C2)
	clk_i2c2.rate = clock_val[CLOCK_PCLK];
#endif
	clk_i2s.rate = clock_val[CLOCK_CCLK];

	/*
	 * Register clocks with the `clk_*` infrastructure
	 */
	for (i = 0; i < ARRAY_SIZE(lpc178x_clkregs); i++)
		clkdev_add(&lpc178x_clkregs[i]);
}

/*
 * Return a clock value for the specified clock.
 *
 * This is the legacy way to get a clock rate, independent
 * from `clk_get_rate()`.
 */
unsigned int lpc178x_clock_get(enum lpc178x_clock clk)
{
	return clock_val[clk];
}
EXPORT_SYMBOL(lpc178x_clock_get);
