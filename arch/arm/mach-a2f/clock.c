/*
 * linux/arch/arm/mach-a2f/clock.c
 *
 * Copyright (C) 2010,2011 Vladimir Khusainov, Emcraft Systems
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
#include <mach/platform.h>
#include <mach/a2f.h>
#include <mach/clock.h>

/*
 * Default value of system frequency (FCLK)
 */
#define A2F_DEF_SYSCLCK_FREQ	80000000

/*
 * System register clock control mask and shift for PCLK dividers.
 */
#define PCLK_DIV_MASK		0x00000003
#define PCLK0_DIV_SHIFT		2
#define PCLK1_DIV_SHIFT		4
#define ACE_DIV_SHIFT		6

/*
 * System register MSS_CCC_DIV_CR mask and shift for GLB (FPGA fabric clock).
 */
#define OBDIV_SHIFT		8
#define OBDIV_MASK		0x0000001F
#define OBDIVHALF_SHIFT		13
#define OBDIVHALF_MASK		0x00000001

/*
 * Actel system boot version defines used to extract the system clock from eNVM
 * spare pages.
 * These defines allow detecting the presence of Actel system boot in eNVM spare
 * pages and the version of that system boot executable and associated
 * configuration data.
 */
#define SYSBOOT_KEY_ADDR	0x6008081C
#define SYSBOOT_KEY_VALUE	0x4C544341
#define SYSBOOT_VERSION_ADDR	0x60080840
#define SYSBOOT_1_3_FCLK_ADDR	0x6008162C
#define SYSBOOT_2_x_FCLK_ADDR	0x60081EAC

/*
 * The system boot version is stored in the least significant 24 bits of a word.
 * The FCLK is stored in eNVM from version 1.3.1 of the system boot. We expect
 * that the major version number of the system boot version will change if the
 * system boot configuration data layout needs to change.
 */
#define SYSBOOT_VERSION_MASK	0x00FFFFFF
#define MIN_SYSBOOT_VERSION	0x00010301
#define SYSBOOT_VERSION_2_X	0x00020000
#define MAX_SYSBOOT_VERSION	0x00030000

/*
 * Reference clock settings
 */

static unsigned int a2f_clock_fclk;
static unsigned int a2f_clock_pclk0;
static unsigned int a2f_clock_pclk1;
static unsigned int a2f_clock_ace;
static unsigned int a2f_clock_fpga;

/*
 * Retrieve the system clock frequency from eNVM spare page if available.
 * Returns the frequency defined through SMARTFUSION_FCLK_FREQ
 * if FCLK cannot be retrieved from eNVM spare pages.
 * The FCLK frequency value selected in the MSS Configurator software tool
 * is stored in eNVM spare pages as part of the Actel system boot
 * configuration data.
 * @returns		system clock frequency
 */
static unsigned int a2f_get_system_clock(void)
{
	unsigned int fclk = 0;
	unsigned int sysboot_version;

	if (SYSBOOT_KEY_VALUE == readl(SYSBOOT_KEY_ADDR)) {
		/*
		 * Actel system boot programmed,
		 * check if it has the FCLK value stored.
		 */
		sysboot_version = readl(SYSBOOT_VERSION_ADDR);
		sysboot_version &= SYSBOOT_VERSION_MASK;

		if (sysboot_version >= MIN_SYSBOOT_VERSION) {
			/*
			 * Read FCLK value from MSS configurator generated
			 * configuration data stored in eNVM spare pages
			 * as part of system boot configuration tables.
			 */
			if (sysboot_version < SYSBOOT_VERSION_2_X) {
				/*
				 * Use version 1.3.x configuration tables.
				 */
				fclk = readl(SYSBOOT_1_3_FCLK_ADDR);
			} else if (sysboot_version < MAX_SYSBOOT_VERSION) {
				/*
				 * Use version 2.x.x configuration tables.
				 */
				fclk = readl(SYSBOOT_2_x_FCLK_ADDR);
			}
		}
	}
	if (0 == fclk) {
		/*
		 * Could not retrieve FCLK from system boot configuration data.
		 * Fall back to using A2F_DEF_SYSFCLK_FREQ.
		 */
		fclk = A2F_DEF_SYSCLCK_FREQ;
	}

	return fclk;
}

/*
 * Initialize the reference clocks.
 * Get the reference clock settings from the hardware.
 * System frequency (FCLK) and the other derivative clocks
 * coming out from firmware. These are defined by the Libero
 * project programmed onto SmartFusion and then, optionally, by firmware.
 */
void __init a2f_clock_init(void)
{
	unsigned int clk_cr;
	unsigned int pclk0_div;
	unsigned int pclk1_div;
	unsigned int ace_div;
	unsigned int fpga_div;
	unsigned int fpga_div_half;

	const unsigned int pclk_div_lut[4] = {1, 2, 4, 1};

	/*
	 * Read PCLK dividers from system registers.
	 */
	clk_cr = readl(&A2F_SYSREG->mss_clk_cr);
	pclk0_div = pclk_div_lut[(clk_cr >> PCLK0_DIV_SHIFT) & PCLK_DIV_MASK];
	pclk1_div = pclk_div_lut[(clk_cr >> PCLK1_DIV_SHIFT) & PCLK_DIV_MASK];
	ace_div	 = pclk_div_lut[(clk_cr >> ACE_DIV_SHIFT) & PCLK_DIV_MASK];

	/*
	 * Compute the FPGA fabric frequency divider.
	 */
	{
		unsigned int div_cr = readl(&A2F_SYSREG->mss_ccc_div_cr);
		unsigned int obdiv = (div_cr >> OBDIV_SHIFT) & OBDIV_MASK;
		unsigned int obdivhalf = (div_cr >> OBDIVHALF_SHIFT)
				& OBDIVHALF_MASK;
		fpga_div = obdiv + 1;
		/*
		 * Dmitry Cherukhin (dima_ch@emcraft.com):
		 * At this point, the file CMSIS/system_a2fxxxm3.c does not
		 * match Actel MSS User's Guide. According to the MSS UG,
		 * the divisor FabDiv should be divided by 2, but in the file
		 * CMSIS/system_a2fxxxm3.k divisor is multiplied by 2.
		 * We follow the MSS UG, because there is not only a formula,
		 * but also a detailed table of frequencies.
		 */
		if (obdivhalf && obdiv) {
			fpga_div_half = 2;
		} else {
			fpga_div_half = 1;
		}
	}

	/*
	 * Retrieve FCLK from eNVM spare pages
	 * if Actel system boot programmed as part of the system.
	 */
	a2f_clock_fclk	= a2f_get_system_clock();

	a2f_clock_pclk0	= a2f_clock_fclk / pclk0_div;
	a2f_clock_pclk1	= a2f_clock_fclk / pclk1_div;
	a2f_clock_ace	= a2f_clock_fclk / ace_div;
	a2f_clock_fpga	= (a2f_clock_fclk * fpga_div_half) / fpga_div;
}

/*
 * Return a clock value for the specified clock.
 * @param clck		id of the clock
 * @returns		frequency of the clock
 */
EXPORT_SYMBOL(a2f_clock_get);
unsigned int a2f_clock_get(enum a2f_clock clck)
{
	unsigned int val = 0;

	switch (clck)  {
	case CLCK_SYSTEMCORE:
		val = a2f_clock_fclk;
		break;
	case CLCK_PCLK0:
		val = a2f_clock_pclk0;
		break;
	case CLCK_PCLK1:
		val = a2f_clock_pclk1;
		break;
	case CLCK_ACE:
		val = a2f_clock_ace;
		break;
	case CLCK_FPGA:
		val = a2f_clock_fpga;
		break;
	}

	return val;
}
