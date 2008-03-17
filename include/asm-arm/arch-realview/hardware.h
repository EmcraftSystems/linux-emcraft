/*
 *  linux/include/asm-arm/arch-realview/hardware.h
 *
 *  This file contains the hardware definitions of the RealView boards.
 *
 *  Copyright (C) 2003 ARM Limited.
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>

/*
 * PCI space virtual addresses
 */
#define REALVIEW_EB_PCI_VIRT_BASE	0xe8000000
#define REALVIEW_EB_PCI_CFG_VIRT_BASE	0xe9000000
#define REALVIEW_EB_PCI_IO_VIRT_BASE	0xea000000

/* CIK guesswork */
#define PCIBIOS_MIN_IO			0x62000000
#define PCIBIOS_MIN_MEM			0x63000000

#define pcibios_assign_all_busses()     1

/* macro to get at IO space when running virtually */
#ifdef CONFIG_MMU
#define IO_ADDRESS(x)		((((x) & 0x0effffff) | (((x) >> 4) & 0x0f000000)) + 0xf0000000)
#else
#define IO_ADDRESS(x)		(x)
#endif
#define __io_address(n)		((void __iomem *)IO_ADDRESS(n))

#endif
