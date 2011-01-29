/*
 * linux/arch/arm/mach-a2f/include/mach/clock.h
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

#ifndef _MACH_A2F_CLOCK_H_
#define _MACH_A2F_CLOCK_H_

/* 
 * Initialize the clock section of the A2F.
 */
extern void __init a2f_clock_init(void);

/*
 * Clocks enumeration.
 */
enum a2f_clock  {
	CLCK_SYSTEMCORE,
	CLCK_PCLK0,
	CLCK_PCLK1,
	CLCK_ACE,
	CLCK_FPGA
};

/*
 * Return a clock value for the specified clock.
 */
extern unsigned int a2f_clock_get(enum a2f_clock);

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

#endif	/*_MACH_A2F_CLOCK_H_ */

/*
 * End of File
 */


