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

#ifndef _MACH_LPC178X_CLOCK_H_
#define _MACH_LPC178X_CLOCK_H_

/*
 * Clocks enumeration
 */
enum lpc178x_clock {
	CLOCK_CCLK,		/* CPU clock frequency expressed in Hz        */
	CLOCK_SYSTICK,		/* Systimer clock frequency expressed in Hz   */
	CLOCK_EMCCLK,		/* EMC clock frequency expressed in Hz        */
	CLOCK_PCLK,		/* Peripheral clock frequency expressed in Hz */
	CLOCK_USBCLK,		/* USB clock frequency expressed in Hz        */
	CLOCK_END		/* for internal usage			      */
};

/*
 * Initialize the clock section of the LPC178x/7x
 */
void __init lpc178x_clock_init(void);

/*
 * Return a clock value for the specified clock
 */
unsigned int lpc178x_clock_get(enum lpc178x_clock clk);

#endif	/*_MACH_LPC178X_CLOCK_H_ */
