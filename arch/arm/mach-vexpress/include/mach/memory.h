/*
 *  arch/arm/mach-vexpress/include/mach/memory.h
 *
 *  Copyright (C) 2003 ARM Limited
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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
#ifdef CONFIG_VEXPRESS_HIGH_PHYS_OFFSET
#define PHYS_OFFSET		UL(0x60000000)
#else
#define PHYS_OFFSET		UL(0x00000000)
#endif

#if !defined(__ASSEMBLY__) && defined(CONFIG_ZONE_DMA)
extern void vexpress_adjust_zones(int node, unsigned long *size,
				  unsigned long *hole);
#define arch_adjust_zones(node, size, hole) \
	vexpress_adjust_zones(node, size, hole)
#endif

/*
 * Sparsemem definitions, only valid for high PHYS_OFFSET.
 *
 * The Versatile Express boards have 512MB of RAM at 0x60000000 and 512MB of RAM at 0x80000000.
 * Unfortunately the upper block is mapped over the vector window rather
 * than the lower block which causes problems.
 * By using SPARSEMEM we can work around the problem area.
 *
 * The macros below define a section size of 256MB and a non-linear virtual to
 * physical mapping:
 *
 * 0x80000000 -> PAGE_OFFSET
 * 0x60000000 -> PAGE_OFFSET + 0x20000000
 * 0xA0000000 -> PAGE_OFFSET + 0x40000000 (required for high_memory)
 */
#ifdef CONFIG_SPARSEMEM

#ifndef CONFIG_VEXPRESS_HIGH_PHYS_OFFSET
#error "SPARSEMEM only available with VEXPRESS_HIGH_PHYS_OFFSET"
#endif

#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	28

#define __phys_to_virt(phys) ({                                 \
        unsigned long virt = 0;                                 \
        if ((phys) >= 0xA0000000UL)                             \
                virt = (phys) - 0x60000000UL + PAGE_OFFSET;     \
        else if ((phys) >= 0x80000000UL)                        \
                virt = (phys) - 0x80000000UL + PAGE_OFFSET;     \
        else if ((phys) >= 0x60000000UL)                        \
                virt = (phys) - 0x40000000UL + PAGE_OFFSET;     \
        virt;                                                   \
})

#define __virt_to_phys(virt) ({                                 \
        unsigned long phys = 0;                                 \
        if ((virt) >= PAGE_OFFSET + 0x40000000UL)               \
                phys = (virt) - PAGE_OFFSET + 0x60000000UL;     \
        else if ((virt) >= PAGE_OFFSET + 0x20000000UL)          \
                phys = (virt) - PAGE_OFFSET + 0x40000000UL;     \
        else if ((virt) >= PAGE_OFFSET)                         \
                phys = (virt) - PAGE_OFFSET + 0x80000000UL;     \
        phys;                                                   \
})

#endif	/* CONFIG_SPARSEMEM */

#endif
