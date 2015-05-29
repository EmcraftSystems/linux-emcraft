/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * Add support for STM32F1
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * Implement clock driver based on the clk_*() API
 * (C) Copyright 2012-2015
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
#include <linux/module.h>

#include <asm/clkdev.h>

#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/stm32.h>

/*
 * Internal oscillator value
 */
#ifdef CONFIG_ARCH_STM32F1
#define STM32_HSI_HZ			8000000		/* STM32F1: 8 MHz */
#else
#define STM32_HSI_HZ			16000000	/* STM32F2: 16 MHz */
#endif

/*
 * Offsets and bitmasks of some RCC regs
 */
#define STM32_RCC_CFGR_SWS_BIT		2	/* System clock switch status */
#define STM32_RCC_CFGR_SWS_MSK		0x3

#define STM32_RCC_CFGR_SWS_HSI		0x0
#define STM32_RCC_CFGR_SWS_HSE		0x1
#define STM32_RCC_CFGR_SWS_PLL		0x2

#define STM32_RCC_CFGR_HPRE_BIT		4	/* AHB prescaler	      */
#define STM32_RCC_CFGR_HPRE_MSK		0xF

/* APB Low speed presc (APB1) */
#ifdef CONFIG_ARCH_STM32F1
#define STM32_RCC_CFGR_PPRE1_BIT	8	/* STM32F1 */
#else
#define STM32_RCC_CFGR_PPRE1_BIT	10	/* STM32F2 */
#endif
#define STM32_RCC_CFGR_PPRE1_MSK	0x7
#define STM32_RCC_CFGR_PPRE1_DIVNO	0x0

/* APB high-speed presc (APB2) */
#ifdef CONFIG_ARCH_STM32F1
#define STM32_RCC_CFGR_PPRE2_BIT	11	/* STM32F1 */
#else
#define STM32_RCC_CFGR_PPRE2_BIT	13	/* STM32F2 */
#endif
#define STM32_RCC_CFGR_PPRE2_MSK	0x7
#define STM32_RCC_CFGR_PPRE2_DIVNO	0x0

#define STM32F2_RCC_PLLCFGR_HSESRC	(1 << 22) /* Main PLL entry clock src */

#define STM32F2_RCC_PLLCFGR_PLLM_BIT	0	/* Div factor for input clock */
#define STM32F2_RCC_PLLCFGR_PLLM_MSK	0x3F

#define STM32F2_RCC_PLLCFGR_PLLN_BIT	6	/* Mult factor for VCO	      */
#define STM32F2_RCC_PLLCFGR_PLLN_MSK	0x1FF

#define STM32F2_RCC_PLLCFGR_PLLP_BIT	16	/* Div factor for main sysclk */
#define STM32F2_RCC_PLLCFGR_PLLP_MSK	0x3

#define STM32F2_RCC_PLLCFGR_PLLQ_BIT	24	/* Div factor for USB,SDIO,.. */
#define STM32F2_RCC_PLLCFGR_PLLQ_MSK	0xF

/*
 * STM32F1: RCC_CFGR register
 */
/* PLL multiplication factor */
#define STM32F1_RCC_CFGR_PLLMUL_BITS	18
#define STM32F1_RCC_CFGR_PLLMUL_MSK	0xF
/* PLL entry clock source: HSI/2 or HSE */
#define STM32F1_RCC_CFGR_PLLSRC_MSK	(1 << 16)
/* HSE divider for PLL entry */
#define STM32F1_RCC_CFGR_PLLXTPRE_MSK	(1 << 17)

/*
 * STM32 ENR bit for SD Card Controller
 */
#define STM32_RCC_ENR_SDIOEN		(1 << 11)

#define STM32_RCC_ENR_LTDCEN		(1 << 26)

/*
 * Clock values
 */
static u32 clock_val[CLOCK_END];

/*
 * The structure that holds the information about a clock
 */
struct clk {
	/* Clock rate, the only possible value of */
	unsigned long rate;

	/* Reference count */
	unsigned int enabled;

	/* Custom `clk_*` functions */
	void (*clk_enable)(struct clk *clk);
	void (*clk_disable)(struct clk *clk);
	int (*clk_set_rate)(struct clk *clk, unsigned long rate);
};

static DEFINE_SPINLOCK(clocks_lock);

/*
 * clk_enable - inform the system when the clock source should be running.
 */
int clk_enable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->enabled++ == 0 && clk->clk_enable)
		clk->clk_enable(clk);
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
	if (--clk->enabled == 0 && clk->clk_disable)
		clk->clk_disable(clk);
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
 */
int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (clk->clk_set_rate)
		return clk->clk_set_rate(clk, rate);
	else
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
 * Enable the SD Card Controller clock
 */
static void mci_clk_enable(struct clk *clk)
{
	STM32_RCC->apb2enr |= STM32_RCC_ENR_SDIOEN;
}

/*
 * Disable the SD Card Controller clock
 */
static void mci_clk_disable(struct clk *clk)
{
	STM32_RCC->apb2enr &= ~STM32_RCC_ENR_SDIOEN;
}

/*
 * Clock for the SD Card Interface module of the MCU. The clock rate
 * is initialized in `stm32_clock_init()`.
 */
static struct clk clk_mci = {
	.clk_enable = mci_clk_enable,
	.clk_disable = mci_clk_disable,
};

#define RCC_CR_PLLSAION		(1 << 28)

/*
 * Enable the LCD Controller clock
 */
static void ltdc_clk_enable(struct clk *clk)
{
	STM32_RCC->apb2enr |= STM32_RCC_ENR_LTDCEN;
}

/*
 * Disable the LCD Controller clock
 */
static void ltdc_clk_disable(struct clk *clk)
{
	STM32_RCC->apb2enr &= ~STM32_RCC_ENR_LTDCEN;
}

#define RCC_DCKCFGR_PLLSAIDIVR	(3 << 16)
#define RCC_PLLSAIDivR_Div8	(2 << 16)

/*
 * Disable the LCD pixel clock
 */
static void sai_r_clk_disable(struct clk *clk)
{
	STM32_RCC->cr &= ~RCC_CR_PLLSAION;
}

/*
 * Enable the LCD pixel clock
 */
static void sai_r_clk_enable(struct clk *clk)
{
	u32 parent_rate;
	u32 sai_n;
	u32 sai_q;
	u32 sai_r;
	u32 sai_div_r;
	u32 dckcfgr;

	sai_q = 7;
	sai_r = 3;
	sai_div_r = 8;

	parent_rate = clock_val[CLOCK_DIVM];

	/* Calculate N to match the requested rate */
	sai_n = clk->rate * sai_r * sai_div_r / parent_rate;

	/* Disable PLLSAI */
	sai_r_clk_disable(clk);

	/* Configure PLLSAI */
	STM32_RCC->pllsaicfgr = (sai_n << 6) | (sai_q << 24) | (sai_r << 28);

	/* Configure divider on the "R" output of PLLSAI */
	dckcfgr = STM32_RCC->dckcfgr;

	/* Clear PLLSAIDIVR[2:0] bits */
	dckcfgr &= ~RCC_DCKCFGR_PLLSAIDIVR;

	/* Set PLLSAIDIVR values */
	dckcfgr |= RCC_PLLSAIDivR_Div8;

	/* Store the new value */
	STM32_RCC->dckcfgr = dckcfgr;

	STM32_RCC->cr |= RCC_CR_PLLSAION;
	while ((STM32_RCC->cr & (1 << 29)) == 0);
}

static int sai_r_clk_set_rate(struct clk *clk, unsigned long rate)
{
	clk->rate = rate;
	return 0;
}

static struct clk clk_ltdc = {
	.clk_enable = ltdc_clk_enable,
	.clk_disable = ltdc_clk_disable,
};

static struct clk clk_sai_r = {
	.rate = 9000000,
	.clk_enable = sai_r_clk_enable,
	.clk_disable = sai_r_clk_disable,
	.clk_set_rate = sai_r_clk_set_rate,
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
static struct clk_lookup stm32_clkregs[] = {
	INIT_CLKREG(&clk_mci, "mmci-pl18x", NULL),
	INIT_CLKREG(&clk_ltdc, "stm32f4-ltdc.0", NULL),
	INIT_CLKREG(&clk_sai_r, NULL, "sai_r_clk"),
};

/*
 * Initialize the reference clocks.
 */
void __init stm32_clock_init(void)
{
	static u32 apbahb_presc_tbl[] = {0, 0, 0, 0, 1, 2, 3, 4,
					 1, 2, 3, 4, 6, 7, 8, 9};

	u32	hse_hz, tmp, presc;
#ifdef CONFIG_ARCH_STM32F1
	u32	pllmul;
#else
	u32	pllvco, pllp, pllm;
#endif
	int	platform;
	int	i;

	/*
	 * Select HSE (external oscillator) freq value depending on the
	 * platform we're running on
	 */
	platform = stm32_platform_get();
	switch (platform) {
#ifdef CONFIG_ARCH_STM32F1
	case PLATFORM_STM32_SWISSEMBEDDED_COMM:
		hse_hz = 8000000;
		break;
#else
	case PLATFORM_STM32_STM3220G_EVAL:
	case PLATFORM_STM32_STM3240G_EVAL:
		hse_hz = 25000000;
		break;
	case PLATFORM_STM32_STM_SOM:
	case PLATFORM_STM32_STM_STM32F439_SOM:
	case PLATFORM_STM32_STM_STM32F7_SOM:
		hse_hz = 12000000;
		break;
	case PLATFORM_STM32_STM_DISCO:
		hse_hz = 8000000;
		break;
#endif
	default:
		/*
		 * Let's assume smth in this case; maybe things will work..
		 */
		hse_hz = 25000000;
		printk(KERN_WARNING "%s: unsupported platform %d\n", __func__,
			platform);
		break;
	}

	/*
	 * Get SYSCLK
	 */
#ifndef CONFIG_ARCH_STM32F1
	pllvco = 0;
#endif
	tmp  = STM32_RCC->cfgr >> STM32_RCC_CFGR_SWS_BIT;
	tmp &= STM32_RCC_CFGR_SWS_MSK;
	switch (tmp) {
	case STM32_RCC_CFGR_SWS_HSI:
		/* HSI used as system clock source */
		clock_val[CLOCK_SYSCLK] = STM32_HSI_HZ;
		break;
	case STM32_RCC_CFGR_SWS_HSE:
		/* HSE used as system clock source */
		clock_val[CLOCK_SYSCLK] = hse_hz;
		break;
	case STM32_RCC_CFGR_SWS_PLL:
#ifdef CONFIG_ARCH_STM32F1
		/* PLL used as system clock source */
		/*
		 * SYSCLK = (HSE/PLLXTPRE or HSI/2) * PLLMUL
		 */
		pllmul = ((STM32_RCC->cfgr >> STM32F1_RCC_CFGR_PLLMUL_BITS) &
			STM32F1_RCC_CFGR_PLLMUL_MSK) + 2;

		if (STM32_RCC->cfgr & STM32F1_RCC_CFGR_PLLSRC_MSK) {
			/* HSE used as PLL clock source */
			tmp = hse_hz;
			if (STM32_RCC->cfgr & STM32F1_RCC_CFGR_PLLXTPRE_MSK)
				tmp /= 2;
		} else {
			/* HSI used as PLL clock source */
			tmp = STM32_HSI_HZ / 2;
		}

		clock_val[CLOCK_SYSCLK] = tmp * pllmul;
#else /* Case of STM32F2 */
		/* PLL used as system clock source */
		/*
		 * PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
		 * SYSCLK = PLL_VCO / PLLP
		 */
		pllm  = STM32_RCC->pllcfgr >> STM32F2_RCC_PLLCFGR_PLLM_BIT;
		pllm &= STM32F2_RCC_PLLCFGR_PLLM_MSK;

		if (STM32_RCC->pllcfgr & STM32F2_RCC_PLLCFGR_HSESRC) {
			/* HSE used as PLL clock source */
			tmp = hse_hz;
		} else {
			/* HSI used as PLL clock source */
			tmp = STM32_HSI_HZ;
		}

		/* Input clock for PLL, PLLI2S and PLLSAI */
		clock_val[CLOCK_DIVM] = tmp / pllm;

		pllvco  = STM32_RCC->pllcfgr >> STM32F2_RCC_PLLCFGR_PLLN_BIT;
		pllvco &= STM32F2_RCC_PLLCFGR_PLLN_MSK;
		pllvco *= clock_val[CLOCK_DIVM];

		pllp  = STM32_RCC->pllcfgr >> STM32F2_RCC_PLLCFGR_PLLP_BIT;
		pllp &= STM32F2_RCC_PLLCFGR_PLLP_MSK;
		pllp  = (pllp + 1) * 2;

		clock_val[CLOCK_SYSCLK] = pllvco / pllp;
#endif /* CONFIG_ARCH_STM32F1 */
		break;
	default:
		clock_val[CLOCK_SYSCLK] = STM32_HSI_HZ;
		break;
	}

	/*
	 * Get HCLK
	 */
	tmp  = STM32_RCC->cfgr >> STM32_RCC_CFGR_HPRE_BIT;
	tmp &= STM32_RCC_CFGR_HPRE_MSK;
	presc = apbahb_presc_tbl[tmp];
	clock_val[CLOCK_HCLK] = clock_val[CLOCK_SYSCLK] >> presc;

	/*
	 * Get PCLK1
	 */
	tmp  = STM32_RCC->cfgr >> STM32_RCC_CFGR_PPRE1_BIT;
	tmp &= STM32_RCC_CFGR_PPRE1_MSK;
	presc = apbahb_presc_tbl[tmp];
	clock_val[CLOCK_PCLK1] = clock_val[CLOCK_HCLK] >> presc;

	/*
	 * Get PTMR1: see "Clock tree" in RM, if APB1 is prescaled, then x2
	 */
	clock_val[CLOCK_PTMR1] = clock_val[CLOCK_PCLK1];
	if (tmp != STM32_RCC_CFGR_PPRE1_DIVNO)
		clock_val[CLOCK_PTMR1] *= 2;

	/*
	 * Get PCLK2
	 */
	tmp  = STM32_RCC->cfgr >> STM32_RCC_CFGR_PPRE2_BIT;
	tmp &= STM32_RCC_CFGR_PPRE2_MSK;
	presc = apbahb_presc_tbl[tmp];
	clock_val[CLOCK_PCLK2] = clock_val[CLOCK_HCLK] >> presc;

	/*
	 * Get PTMR2: see "Clock tree" in RM, if APB2 is prescaled, then x2
	 */
	clock_val[CLOCK_PTMR2] = clock_val[CLOCK_PCLK2];
	if (tmp != STM32_RCC_CFGR_PPRE2_DIVNO)
		clock_val[CLOCK_PTMR2] *= 2;

	/*
	 * Initialize the `clk_*` structures
	 */
#ifndef CONFIG_ARCH_STM32F1
	if (pllvco) {
		clk_mci.rate = pllvco /
			((STM32_RCC->pllcfgr >> STM32F2_RCC_PLLCFGR_PLLQ_BIT) &
			STM32F2_RCC_PLLCFGR_PLLQ_MSK);
	}
#endif

	/*
	 * Register clocks with the `clk_*` infrastructure
	 */
	for (i = 0; i < ARRAY_SIZE(stm32_clkregs); i++)
		clkdev_add(&stm32_clkregs[i]);
}

/*
 * Return a clock value for the specified clock.
 *
 * This is the legacy way to get a clock rate, independent
 * from `clk_get_rate()`.
 */
unsigned int stm32_clock_get(enum stm32_clock clk)
{
	return clock_val[clk];
}
EXPORT_SYMBOL(stm32_clock_get);
