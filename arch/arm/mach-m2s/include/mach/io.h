/*
 * arch/arm/mach-m2s/include/mach/io.h
 *
 * Copyright (C) 2010-2012 EmCraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <mach/m2s.h>
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

	/* allow user programs to read/write SYSREG */
	if (addr >= M2S_SYSREG_BASE &&
		addr + size < M2S_SYSREG_BASE + sizeof(struct m2s_sysreg))
		return 1;

	return 0;
}

static inline int valid_mmap_phys_addr_range(unsigned long pfn, size_t size)
{
	return 1;
}

#endif

#endif
