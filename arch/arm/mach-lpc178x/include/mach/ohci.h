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

#ifndef _MACH_LPC178X_OHCI_H_
#define _MACH_LPC178X_OHCI_H_

#include <linux/init.h>

#include <mach/lpc178x.h>

/*
 * LPC178x/7x platform OHCI driver name
 */
#define LPC178X_OHCI_DRV_NAME		"lpc178x-ohci"

/*
 * LPC178x/8x USB registers base
 */
#define LPC178X_USB_BASE		(LPC178X_AHB_PERIPH_BASE + 0x0000C000)

void __init lpc178x_ohci_init(void);

#endif /* _MACH_LPC178X_OHCI_H_ */
