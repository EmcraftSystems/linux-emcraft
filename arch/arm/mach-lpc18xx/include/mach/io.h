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
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <linux/mm.h>

#define IO_SPACE_LIMIT 0xffffffff

static inline void __iomem *__io(unsigned long addr)
{
        return (void __iomem *)addr;
}

#define __io(a)                 __io(a)
#define __mem_pci(a)            (a)

#define ARCH_HAS_VALID_PHYS_ADDR_RANGE

#ifndef __ASSEMBLY__

static inline int valid_phys_addr_range(unsigned long addr, size_t size)
{
	if (addr + size <= __pa(high_memory))
		return 1;

	/*
	 * TBD
	 */

	return 0;
}

static inline int valid_mmap_phys_addr_range(unsigned long pfn, size_t size)
{
	return 1;
}

#endif

#endif
