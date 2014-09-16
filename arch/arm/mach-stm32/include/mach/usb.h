/*
 * (C) Copyright 2014
 * Emcraft Systems, <www.emcraft.com>
 * Pavel Boldin <paboldin@emcraft.com>
 * Dmitry Konyshev <probables@emcraft.com>
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

#ifndef _MACH_STM32_USB_H_
#define _MACH_STM32_USB_H_

#include <mach/stm32.h>

/*
 * STM32 USB register base
 */
#define STM32_USB_OTG_HS_BASE		(STM32_AHB1PERITH_BASE + 0x20000)
#define STM32_USB_OTG_FS_BASE		(STM32_AHB2PERITH_BASE + 0x0000)

void __init stm32_usb_otg_hs_init(void);
void __init stm32_usb_otg_fs_init(void);

#endif /* _MACH_STM32_USB_H_ */
