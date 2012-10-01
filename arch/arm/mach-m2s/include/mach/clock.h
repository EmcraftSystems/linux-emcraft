/*
 * linux/arch/arm/mach-m2s/include/mach/clock.h
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

#ifndef _MACH_M2S_CLOCK_H_
#define _MACH_M2S_CLOCK_H_

/*
 * Initialize the clock section of the M2S.
 */
extern void __init m2s_clock_init(void);

/*
 * Clocks enumeration.
 */
enum m2s_clock  {
	CLCK_PCLK0,
	CLCK_PCLK1,
};

/*
 * Return a clock value for the specified clock.
 */
extern unsigned int m2s_clock_get(enum m2s_clock);

#endif	/*_MACH_M2S_CLOCK_H_ */
