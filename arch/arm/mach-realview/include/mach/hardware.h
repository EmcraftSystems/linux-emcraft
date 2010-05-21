/*
 *  arch/arm/mach-realview/include/mach/hardware.h
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
#include <asm/mach-types.h>

/*
 * PCI space virtual addresses
 */
#define REALVIEW_PCI_VIRT_BASE		0xF8000000
#define REALVIEW_PCI_CFG_VIRT_BASE	0xF9000000
#define PCIX_UNIT_BASE			0xF8000000
#define REALVIEW_PCI_IO_VBASE		0xFA000000
/*
 * PCI space physical addresses and sizes
 */
#define REALVIEW_PB_PCI_BASE		0x90040000	/* PCI-X Unit base */
#define REALVIEW_PB_PCI_BASE_SIZE	0x00010000	/* 4 Kb + 60Kb reserved */
#define REALVIEW_PB_PCI_IO_BASE		0x90050000	/* IO Region on AHB */
#define REALVIEW_PB_PCI_IO_SIZE		0x00010000	/* 64 Kb */
#define REALVIEW_PB_PCI_IO_LIMIT       (REALVIEW_PB_PCI_IO_BASE + REALVIEW_PB_PCI_IO_SIZE - 1)
#define REALVIEW_PB_PCI_MEM_BASE	0xA0000000	/* MEM Region on AHB */
#define REALVIEW_PB_PCI_MEM_SIZE	0x20000000	/* 512 MB */

#define REALVIEW_ISSP_REG_BASE		0x100E3000

#ifdef CONFIG_PCI
#if !defined(__ASSEMBLY__)
static inline unsigned int pcibios_min_io(void)
{
	if (machine_is_realview_pb11mp() || machine_is_realview_pba8() ||
	    machine_is_realview_pbx())
		return REALVIEW_PB_PCI_IO_BASE;
	else
		return 0;
}

static inline unsigned int pcibios_min_mem(void)
{
	if (machine_is_realview_pb11mp() || machine_is_realview_pba8() ||
	    machine_is_realview_pbx())
		return REALVIEW_PB_PCI_MEM_BASE;
	else
		return 0;
}
#endif

/*
 * These are needed so that generic pci code doesn't know about our
 * machine specific details.
 */
#define PCIBIOS_MIN_IO		pcibios_min_io()
#define PCIBIOS_MIN_MEM		pcibios_min_mem()

#define pcibios_assign_all_busses()     1

#endif

/* macro to get at IO space when running virtually */
#ifdef CONFIG_MMU
/*
 * Statically mapped addresses:
 *
 * 10xx xxxx -> fbxx xxxx
 * 1exx xxxx -> fdxx xxxx
 * 1fxx xxxx -> fexx xxxx
 */
#define IO_ADDRESS(x)		(((x) & 0x03ffffff) + 0xfb000000)
#else
#define IO_ADDRESS(x)		(x)
#endif
#define __io_address(n)		__io(IO_ADDRESS(n))

#endif
