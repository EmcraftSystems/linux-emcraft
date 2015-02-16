/*
 * arch/arm/mm/cache-v7m.c
 *
 * ARM-V7M inner L1 cache handling routines.
 *
 * Copyright (C) 2015 Yuri Tikhonov (yur@emcraft.com), EmCraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <asm/cache.h>
#include <asm/cacheflush.h>

/* Instruction cache invalidate by address to Point of Unification (PoU) */
#define ICIMVAU(v)	*(volatile u32 *)0xE000EF58 = v
/* Data cache invalidate by address to Point of Coherency (PoC) */
#define DCIMVAC(v)	*(volatile u32 *)0xE000EF5C = v
/* Data cache clean by address to PoC */
#define DCCMVAC(v)	*(volatile u32 *)0xE000EF68 = v
/* Data cache clean and invalidate by address to PoC */
#define DCCIMVAC(v)	*(volatile u32 *)0xE000EF70 = v

/*
 * Ensure that the I and D caches are coherent within specified
 * region. This is typically used when code has been written to
 * a memory region, and will be executed.
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
void v7m_coherent_kern_range(unsigned long start, unsigned long end)
{
	while (start < end) {
		/* Clean D-line */
		DCCMVAC(start);
		asm("dsb");
		/* Invalidate I-line */
		ICIMVAU(start);

		start += L1_CACHE_BYTES;
	}

	asm("dsb");
	asm("isb");
}

void v7m_coherent_user_range(unsigned long start, unsigned long end)
{
	v7m_coherent_kern_range(start, end);
}

/*
 * Ensure that the data held in the page kaddr is written back
 * to the page in question.
 * - addr  - kernel address
 * - size  - region size
 */
void v7m_flush_kern_dcache_area(void *addr, size_t size)
{
	u32	s, e;

	/* Clean & invalidate */
	for (s = (u32)addr, e = s + size; s < e; s += L1_CACHE_BYTES)
		DCCIMVAC(s);

	asm("dsb");
}

/*
 * Invalidate the data cache within the specified region.
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
void v7m_dma_inv_range(const void *start, const void *end)
{
	u32	s = (u32)start, e = (u32)end;

	/* Flush cache-unaligned head & tail */
	if (s & (L1_CACHE_BYTES - 1)) {
		DCCIMVAC(s);
		s = (s + L1_CACHE_BYTES) & ~(L1_CACHE_BYTES - 1);
	}
	if (e & (L1_CACHE_BYTES - 1)) {
		DCCIMVAC(e);
		e = (e - L1_CACHE_BYTES) & ~(L1_CACHE_BYTES - 1);
	}

	/* Invalidate aligned part */
	for ( ; s < e; s += L1_CACHE_BYTES)
		DCIMVAC(s);

	asm("dsb");
}

/*
 * Clean the data cache within the specified region.
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
void v7m_dma_clean_range(const void *start, const void *end)
{
	u32	s, e;

	/* Clean */
	for (s = (u32)start, e = (u32)end; s < e; s += L1_CACHE_BYTES)
		DCCMVAC(s);

	asm("dsb");
}


/*
 * Clean & invalidate the data cache within the specified region.
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
void v7m_dma_flush_range(const void *start, const void *end)
{
	u32	s, e;

	/* Clean & invalidate */
	for (s = (u32)start, e = (u32)end; s < e; s += L1_CACHE_BYTES)
		DCCIMVAC(s);

	asm("dsb");
}

/*
 * Prepare region for DMAing
 * - start - kernel virtual start address
 * - size  - size of region
 * - dir   - DMA direction
 */
void v7m_dma_map_area(const void *start, size_t size, int dir)
{
	void	*end = (void *)((u32)start + size);

	if (dir == DMA_FROM_DEVICE)
		v7m_dma_inv_range(start, end);
	v7m_dma_clean_range(start, end);
}

/*
 * DMA to region complete, unmap
 * - start - kernel virtual start address
 * - size  - size of region
 * - dir   - DMA direction
 */
void v7m_dma_unmap_area(const void *start, size_t size, int dir)
{
	void	*end = (void *)((u32)start + size);

	if (dir == DMA_TO_DEVICE)
		v7m_dma_inv_range(start, end);
}
