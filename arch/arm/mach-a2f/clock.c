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
#include <mach/platform.h>
#include <mach/a2f.h>
#include <mach/clock.h>

/*
 * Reference clock settings
 */

static unsigned int clock_fclk;
static unsigned int clock_pclk0;
static unsigned int clock_pclk1;
static unsigned int clock_ace;
static unsigned int clock_fpga;

/*
 * Initialize the reference clocks.
 * TO-DO: Eventually, this needs to be changed to get
 * the reference clock settings from the hardware, rather
 * than to have the user to configure it here. 
 * System frequency (FCLK) and the other derivative clocks
 * coming out from firmware. These are defined by the Libero
 * project programmed onto SmartFusion and then, optionally, by firmware.
 * It is possible to read these frequencies from SmartFusion
 * at run-time, however for simplicity of configuration we define these
 * clocks as constants.
 */
void __init a2f_clock_init(void)
{
	if (a2f_platform == PLATFORM_A2F_LNX_EVB) {
		clock_fclk	= 80000000;
		clock_pclk0	= clock_fclk / 4;
		clock_pclk1	= clock_fclk / 4;
		clock_ace	= clock_fclk / 2;
		clock_fpga	= clock_fclk / 2;
	}
	else if (a2f_platform == PLATFORM_A2F_ACTEL_DEV_BRD) {
		clock_fclk	= 80000000;
		clock_pclk0	= clock_fclk / 4;
		clock_pclk1	= clock_fclk / 4;
		clock_ace	= clock_fclk / 2;
		clock_fpga	= clock_fclk / 2;
	}
	else if (a2f_platform == PLATFORM_A2F_HOERMANN_BRD) {
		clock_fclk	= 80000000;
		clock_pclk0	= clock_fclk / 4;
		clock_pclk1	= clock_fclk / 4;
		clock_ace	= clock_fclk / 2;
		clock_fpga	= clock_fclk / 2;
	}
}

/*
 * Return a clock value for the specified clock.
 */
unsigned int a2f_clock_get(enum a2f_clock clck)
{
	unsigned int val = 0;

	switch (clck)  {
	case CLCK_SYSTEMCORE:
		val = clock_fclk;
		break;
	case CLCK_PCLK0:
		val = clock_pclk0;
		break;
	case CLCK_PCLK1:
		val = clock_pclk1;
		break;
	case CLCK_ACE:
		val = clock_ace;
		break;
	case CLCK_FPGA:
		val = clock_fpga;
		break;
	}

	return val;
}
