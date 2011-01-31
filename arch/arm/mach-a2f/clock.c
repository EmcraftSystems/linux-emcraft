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
#include <linux/init.h>
#include <mach/a2f.h>
#include <mach/clock.h>

/*
 * System frequency (FCLK) and the other derivative clocks
 * coming out from reset. These are defined by the Libero
 * project programmed onto SmartFusion.
 * It is possible to read these frequencies from SmartFusion
 * at run-time, however for simplicity of configuration we define these
 * clocks at build-time.
 */
#define SF_CLK_FREQ			80000000uL
#define SF_CLK_PCLK0			(SF_CLK_FREQ / 4)
#define SF_CLK_PCLK1			(SF_CLK_FREQ / 4)
#define SF_ACE_PCLK1			(SF_CLK_FREQ / 2)
#define SF_FPGA_PCLK1			(SF_CLK_FREQ / 2)

/*
 * Initialize the clock section of the A2F.
 */
void __init a2f_clock_init(void)
{
}

/*
 * Return a clock value for the specified clock.
 */
unsigned int a2f_clock_get(enum a2f_clock clck)
{
	unsigned int val = 0;

	switch (clck)  {
	case CLCK_SYSTEMCORE:
		val = SF_CLK_FREQ;
		break;
	case CLCK_PCLK0:
		val = SF_CLK_PCLK0;
		break;
	case CLCK_PCLK1:
		val = SF_CLK_PCLK1;
		break;
	case CLCK_ACE:
		val = SF_ACE_PCLK1;
		break;
	case CLCK_FPGA:
		val = SF_FPGA_PCLK1;
		break;
	}

	return val;
}
