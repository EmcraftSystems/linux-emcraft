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

#ifndef _MACH_KINETIS_CLOCK_H_
#define _MACH_KINETIS_CLOCK_H_

/*
 * Clocks enumeration
 */
enum kinetis_clock {
	CLOCK_SYSTICK,		/* Systimer clock frequency expressed in Hz   */
	CLOCK_MCGOUTCLK,	/* MCGOUTCLK frequency expressed in Hz        */
	CLOCK_CCLK,		/* Core clock frequency expressed in Hz       */
	CLOCK_PCLK,		/* Bus clock frequency expressed in Hz        */
	CLOCK_MACCLK,		/* MAC module clock frequency expressed in Hz */
	CLOCK_USBHS,		/* USB-HS clock frequency expressed in Hz     */
	CLOCK_END		/* for internal usage			      */
};

/*
 * Initialize the MCG (Multipurpose Clock Generator) of the Kinetis MCU
 */
void __init kinetis_clock_init(void);

/*
 * Return a clock value for the specified clock
 */
unsigned int kinetis_clock_get(enum kinetis_clock clk);

/*
 * Configure the LCDC clock fractional divider to produce a frequency as close
 * as possible to the requested value.
 */
void kinetis_lcdc_adjust_clock_divider(
	unsigned long clock, unsigned long base);

#endif /* _MACH_KINETIS_CLOCK_H_ */
