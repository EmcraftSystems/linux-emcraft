/*
 * arch/arm/mach-m2s/include/mach/memory.h
 *
 * Copyright (C) 2012 Alexander Potashev, Emcraft Systems.
 * Copyright (C) 2010,2011 Vladimir Khusainov, Emcraft Systems.
 * Copyright (C) 2009 ARM Ltd.
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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Base of the external RAM. In SmartFusion2, accessing RAM
 * using this address region bypasses the on-chip cache
 */
#define PHYS_OFFSET		UL(0xA0000000)

#if defined(CONFIG_M2S_CACHE)

/*
 * Same external RAM is mirrored into the address region below.
 * When accessing the RAM using this address region, accesses are cached.
 * Note that only code can be cached. Anything that is not code
 * (stack, data, bss, heap) must be accessed using the non-cached address
 * region (or the kernel will crash on you).
 */
#define PHYS_OFFSET_CACHED	UL(0x10000000)

/*
 * Convert a non-cached address into its cached equivalent
 */
#define m2s_phys_to_cached(a)	((a) - (PHYS_OFFSET - PHYS_OFFSET_CACHED))

/*
 * Convert a cached address into its non-cached equivalent
 */
#define m2s_cached_to_phys(a)	((a) + (PHYS_OFFSET - PHYS_OFFSET_CACHED))

/*
 * Is address cached?
 */
#define m2s_addr_is_cached(a)	(((a) & 0xF0000000) == PHYS_OFFSET_CACHED)

/*
 * Flush the entire cache
 */
#define m2s_flush_cache()					\
	* ((volatile unsigned int *) 0x400381A8) |= 0x1FF;	\
	* ((volatile unsigned int *) 0x400381A8) &= ~0x1FF

#endif /* CONFIG_M2S_CACHE */

/*
 * Base of the on-chip Flash (eNVM)
 */
#define ENVM_PHYS_OFFSET	UL(0x60000000)

#endif
