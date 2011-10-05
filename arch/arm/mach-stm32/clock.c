/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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
#include <mach/stm32.h>

/*
 * External oscillator value
 */
#define CONFIG_STM32_HSE_HZ		25000000	/* 25 MHz	      */

/*
 * Internal oscillator value
 */
#define STM32_HSI_HZ			16000000	/* 16 MHz	      */

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

#define STM32_RCC_CFGR_PPRE1_BIT	10	/* APB Low speed presc (APB1) */
#define STM32_RCC_CFGR_PPRE1_MSK	0x7
#define STM32_RCC_CFGR_PPRE1_DIVNO	0x0

#define STM32_RCC_CFGR_PPRE2_BIT	13	/* APB high-speed presc (APB2)*/
#define STM32_RCC_CFGR_PPRE2_MSK	0x7
#define STM32_RCC_CFGR_PPRE2_DIVNO	0x0

#define STM32_RCC_PLLCFGR_HSESRC	(1 << 22) /* Main PLL entry clock src */

#define STM32_RCC_PLLCFGR_PLLM_BIT	0	/* Div factor for input clock */
#define STM32_RCC_PLLCFGR_PLLM_MSK	0x3F

#define STM32_RCC_PLLCFGR_PLLN_BIT	6	/* Mult factor for VCO	      */
#define STM32_RCC_PLLCFGR_PLLN_MSK	0x1FF

#define STM32_RCC_PLLCFGR_PLLP_BIT	16	/* Div factor for main sysclk */
#define STM32_RCC_PLLCFGR_PLLP_MSK	0x3

#define STM32_RCC_PLLCFGR_PLLQ_BIT	24	/* Div factor for USB,SDIO,.. */
#define STM32_RCC_PLLCFGR_PLLQ_MSK	0xF

/*
 * Clock values
 */
static u32 clock_val[CLOCK_END];

/*
 * Initialize the reference clocks.
 */
void __init stm32_clock_init(void)
{
	static u32 apbahb_presc_tbl[] = {0, 0, 0, 0, 1, 2, 3, 4,
					 1, 2, 3, 4, 6, 7, 8, 9};

	volatile struct stm32_rcc_regs *rcc_regs =
		(struct stm32_rcc_regs *)STM32_RCC_BASE;
	u32 tmp, presc, pllvco, pllp, pllm;

	/*
	 * Get SYSCLK
	 */
	tmp  = rcc_regs->cfgr >> STM32_RCC_CFGR_SWS_BIT;
	tmp &= STM32_RCC_CFGR_SWS_MSK;
	switch (tmp) {
	case STM32_RCC_CFGR_SWS_HSI:
		/* HSI used as system clock source */
		clock_val[CLOCK_SYSCLK] = STM32_HSI_HZ;
		break;
	case STM32_RCC_CFGR_SWS_HSE:
		/* HSE used as system clock source */
		clock_val[CLOCK_SYSCLK] = CONFIG_STM32_HSE_HZ;
		break;
	case STM32_RCC_CFGR_SWS_PLL:
		/* PLL used as system clock source */
		/*
		 * PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
		 * SYSCLK = PLL_VCO / PLLP
		 */
		pllm  = rcc_regs->pllcfgr >> STM32_RCC_PLLCFGR_PLLM_BIT;
		pllm &= STM32_RCC_PLLCFGR_PLLM_MSK;

		if (rcc_regs->pllcfgr & STM32_RCC_PLLCFGR_HSESRC) {
			/* HSE used as PLL clock source */
			tmp = CONFIG_STM32_HSE_HZ;
		} else {
			/* HSI used as PLL clock source */
			tmp = STM32_HSI_HZ;
		}
		pllvco  = rcc_regs->pllcfgr >> STM32_RCC_PLLCFGR_PLLN_BIT;
		pllvco &= STM32_RCC_PLLCFGR_PLLN_MSK;
		pllvco *= tmp / pllm;

		pllp  = rcc_regs->pllcfgr >> STM32_RCC_PLLCFGR_PLLP_BIT;
		pllp &= STM32_RCC_PLLCFGR_PLLP_MSK;
		pllp  = (pllp + 1) * 2;

		clock_val[CLOCK_SYSCLK] = pllvco / pllp;
		break;
	default:
		clock_val[CLOCK_SYSCLK] = STM32_HSI_HZ;
		break;
	}

	/*
	 * Get HCLK
	 */
	tmp  = rcc_regs->cfgr >> STM32_RCC_CFGR_HPRE_BIT;
	tmp &= STM32_RCC_CFGR_HPRE_MSK;
	presc = apbahb_presc_tbl[tmp];
	clock_val[CLOCK_HCLK] = clock_val[CLOCK_SYSCLK] >> presc;

	/*
	 * Get PCLK1
	 */
	tmp  = rcc_regs->cfgr >> STM32_RCC_CFGR_PPRE1_BIT;
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
	tmp  = rcc_regs->cfgr >> STM32_RCC_CFGR_PPRE2_BIT;
	tmp &= STM32_RCC_CFGR_PPRE2_MSK;
	presc = apbahb_presc_tbl[tmp];
	clock_val[CLOCK_PCLK2] = clock_val[CLOCK_HCLK] >> presc;

	/*
	 * Get PTMR2: see "Clock tree" in RM, if APB2 is prescaled, then x2
	 */
	clock_val[CLOCK_PTMR2] = clock_val[CLOCK_PCLK2];
	if (tmp != STM32_RCC_CFGR_PPRE2_DIVNO)
		clock_val[CLOCK_PTMR2] *= 2;

	return;
}

/*
 * Return a clock value for the specified clock.
 */
unsigned int stm32_clock_get(enum stm32_clock clk)
{
	return clock_val[clk];
}
