/*
 * (C) Copyright 2009
 * ARM Ltd.
 *
 * (C) Copyright 2012
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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Write-back cached DDRAM-alias region.
 * This is used for run-time kernel memory.
 */
#define PHYS_OFFSET		UL(0x70000000)

/*
 * Write-through cached DDRAM-alias region.
 * This is used to run kernel core from.
 */
#define PHYS_ALIAS_OFFSET	UL(0x08000000)

/*
 * Non-cacheble DDRAM-alias region. 
 * This is used to allocate DMA buffers and data structures
 */
#define PHYS_DMA_OFFSET		UL(0x80000000)

/*
 * Mask of the field used to distinguish DDRAM aliases
 */
#define DRAM_ALIAS_MASK		UL(0xF8000000)

/*
 * This macro converts an address in the kernel code or
 * in the non-cacheable DMA region to an alias in
 * the run-time kernel memory region
 */
#define PHYS_ALIAS_ADDR(a)						\
	(((unsigned int)(a) & ~DRAM_ALIAS_MASK) | 			\
	(PHYS_OFFSET & DRAM_ALIAS_MASK))

/*
 * This macro converts an address in the kernel run-time memory
 * to an alias in the non-cacheable memory region
 */
#define DMA_ALIAS_ADDR(a)						\
	(((unsigned int)(a) & ~DRAM_ALIAS_MASK) |			\
	(PHYS_DMA_OFFSET & DRAM_ALIAS_MASK))

/*
 * Architecture-specific low-level DMA macros
 */
#define __arch_dma_to_virt(dev, a)					\
	((void *) PHYS_ALIAS_ADDR(a))

#define __arch_virt_to_dma(dev, a)					\
	((dma_addr_t) DMA_ALIAS_ADDR(a))

#define __arch_dma_to_page(dev, addr)					\
	((void *) phys_to_page(PHYS_ALIAS_ADDR(addr)))

#define __arch_page_to_dma(dev, page)					\
	((dma_addr_t) DMA_ALIAS_ADDR(page_to_phys(page)))
/*
 * Base of the on-chip Flash
 */
#define ENVM_PHYS_OFFSET	UL(0x00000000)

#endif

