/*
 * linux/arch/arm/mach-m2s/clock.c
 *
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <mach/platform.h>
#include <mach/m2s.h>
#include <mach/clock.h>

/*
 * Reference clock settings
 */

static unsigned int m2s_clock_pclk0;
static unsigned int m2s_clock_pclk1;

/*
 * Calculate the divisor for a specified FACC1 field
 * @param r		FACC1 value
 * @param s		FACC1 divisor field
 * @returns		divisor
 */
static unsigned int clock_mss_divisor(unsigned int r, unsigned int s)
{
	unsigned int v, ret;

	/*
	 * Get a 3-bit field that defines the divisor
	 */
	v = (r & (0x7<<s)) >> s;

	/*
	 * Translate the bit representation of the divisor to
	 * a value ready to be used in calculation of a clock.
	 */
	switch (v) {
	case 0: ret = 1; break;
	case 1: ret = 2; break;
	case 2: ret = 4; break;
	case 4: ret = 8; break;
	case 5: ret = 16; break;
	case 6: ret = 32; break;
	default: ret = 1; break;
	}

	return ret;
}

/*
 * System frequencies are defined by Libero, with no
 * known way (as of yet) to read them in run time. Hence,
 * we define them as build-time constants
 */
#define CONFIG_SYS_M2S_SYSREF		166000000

/*
 * Perform reference clocks learning
 */
static void clock_mss_learn(void)
{
	unsigned int r1 = M2S_SYSREG->mssddr_facc1_cr;

	m2s_clock_pclk0 = CONFIG_SYS_M2S_SYSREF / clock_mss_divisor(r1, 2);
	m2s_clock_pclk1 = CONFIG_SYS_M2S_SYSREF / clock_mss_divisor(r1, 5);
}

/*
 * Initialize the reference clocks.
 */
void __init m2s_clock_init(void)
{
	clock_mss_learn();
}

/*
 * Return a clock value for the specified clock.
 * @param clck		id of the clock
 * @returns		frequency of the clock
 */
unsigned int m2s_clock_get(enum m2s_clock clck)
{
	unsigned int val = 0;

	switch (clck)  {
	case CLCK_PCLK0:
		val = m2s_clock_pclk0;
		break;
	case CLCK_PCLK1:
		val = m2s_clock_pclk1;
		break;
	case CLCK_SYSREF:
		val = CONFIG_SYS_M2S_SYSREF;
		break;
	}

	return val;
}
EXPORT_SYMBOL(m2s_clock_get);

int clk_enable(struct clk *clk)
{
	/*
	 * TBD
	 */

	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	/*
	 * TBD
	 */
}
EXPORT_SYMBOL(clk_disable);

