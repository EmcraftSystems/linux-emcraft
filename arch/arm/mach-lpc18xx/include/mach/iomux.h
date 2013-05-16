/*
 * (C) Copyright 2011-2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
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

#ifndef _MACH_LPC18XX_IOMUX_H_
#define _MACH_LPC18XX_IOMUX_H_

#include <linux/init.h>

/*
 * Set up direction of a GPIO: 1-> out; 0-> in
 */
void lpc18xx_gpio_dir(int port, int pin, int d);

/*
 * Define the value of a general-purpose output
 */
void lpc18xx_gpio_out(int port, int pin, int v);

/*
 * Set up IOMUX configuration of various processor chips
 */
void __init lpc18xx_iomux_init(void);

#endif	/*_MACH_LPC18XX_IOMUX_H_ */
