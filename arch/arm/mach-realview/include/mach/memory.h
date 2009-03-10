/*
 *  arch/arm/mach-realview/include/mach/memory.h
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
#ifdef CONFIG_REALVIEW_HIGH_PHYS_OFFSET
#define PHYS_OFFSET		UL(0x70000000)
#else
#define PHYS_OFFSET		UL(0x00000000)
#endif

#if !defined(__ASSEMBLY__) && defined(CONFIG_ZONE_DMA)
extern void realview_adjust_zones(int node, unsigned long *size,
				  unsigned long *hole);
#define arch_adjust_zones(node, size, hole) \
	realview_adjust_zones(node, size, hole)
#endif

/*
 * Sparsemem definitions, only valid for high PHYS_OFFSET.
 *
 * Most RealView boards (except PB1176) have 512MB of RAM at 0x70000000. The
 * PBX board has another block of 512MB of RAM at 0x20000000, however only the
 * block at 0x70000000 may be used for DMA.
 *
 * The macros below define a section size of 256MB and a non-linear virtual to
 * physical mapping:
 *
 * 0x70000000 -> PAGE_OFFSET
 * 0x20000000 -> PAGE_OFFSET + 0x20000000
 * 0x90000000 -> PAGE_OFFSET + 0x40000000 (required for high_memory)
 */
#ifdef CONFIG_SPARSEMEM

#ifndef CONFIG_REALVIEW_HIGH_PHYS_OFFSET
#error "SPARSEMEM only available with REALVIEW_HIGH_PHYS_OFFSET"
#endif

#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	28

#define __phys_to_virt(phys) ({					\
	unsigned long virt = 0;					\
	if ((phys) >= 0x90000000UL)				\
		virt = (phys) - 0x50000000UL + PAGE_OFFSET;	\
	else if ((phys) >= 0x70000000UL)			\
		virt = (phys) - 0x70000000UL + PAGE_OFFSET;	\
	else if ((phys) >= 0x20000000UL)			\
		virt = (phys) + PAGE_OFFSET;			\
	virt;							\
})

#define __virt_to_phys(virt) ({					\
	unsigned long phys = 0;					\
	if ((virt) >= PAGE_OFFSET + 0x40000000UL)		\
		phys = (virt) - PAGE_OFFSET + 0x50000000UL;	\
	else if ((virt) >= PAGE_OFFSET + 0x20000000UL)		\
		phys = (virt) - PAGE_OFFSET;			\
	else if ((virt) >= PAGE_OFFSET)				\
		phys = (virt) - PAGE_OFFSET + 0x70000000UL;	\
	phys;							\
})

#endif

/*
 * Virtual view <-> DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *              address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *              to an address that the kernel can use.
 */
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)

#endif
