/*
 * (C) Copyright 2011
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
#include <mach/lpc178x.h>

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
 * Clock values
 */
static u32 clock_val[CLOCK_END];

/*
 * Initialize the reference clocks.
 */
void __init lpc178x_clock_init(void)
{
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
	clock_val[CLOCK_SYSTICK] = cclk_input /
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
}

/*
 * Return a clock value for the specified clock.
 */
unsigned int lpc178x_clock_get(enum lpc178x_clock clk)
{
	return clock_val[clk];
}
