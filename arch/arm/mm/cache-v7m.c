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

/* Configuration and MPU Registers */
#define CCR()		(*(volatile u32 *)0xE000ED14)
#define MPU_TYPE()	(*(volatile u32 *)0xE000ED90)
#define MPU_RNR(v)	*(volatile u32 *)0xE000ED98 = v
#define MPU_RASR()	(*(volatile u32 *)0xE000EDA0)

/*
 * The caching policy is set per MPU region. This func walks through all Valid
 * MPU regions, and detect the best caching policy we have
 */
static char *v7m_best_cache(void)
{
	static struct {
		char		*nm;
		struct {
			u32	msk;
			u32	val;
		} r[2];
	} m[] = {
		{ "OFF", { /* Non-cacheable */
		  { 4 << 19 | 1 << 17 | 1 << 16, 4 << 19 | 0 << 17 | 0 << 16 },
		  { 7 << 19 | 1 << 17 | 1 << 16, 1 << 19 | 0 << 17 | 0 << 16 },
		} },
		{ "WT", { /* Write-through */
		  { 4 << 19 | 1 << 17 | 1 << 16, 4 << 19 | 1 << 17 | 0 << 16 },
		  { 7 << 19 | 1 << 17 | 1 << 16, 0 << 19 | 1 << 17 | 0 << 16 },
		} },
		{ "WB", { /* Write-back */
		  { 4 << 19 | 1 << 17 | 1 << 16, 4 << 19 | 1 << 17 | 1 << 16 },
		  { 7 << 19 | 1 << 17 | 1 << 16, 0 << 19 | 1 << 17 | 1 << 16 },
		} },
		{ "WBA", { /* Write-back, read-write allocate */
		  { 4 << 19 | 1 << 17 | 1 << 16, 4 << 19 | 0 << 17 | 1 << 16 },
		  { 7 << 19 | 1 << 17 | 1 << 16, 1 << 19 | 1 << 17 | 1 << 16 },
		} },
	};

	u32	rasr;
	int	i, k;
	int	regions, best = -1;

	regions = (MPU_TYPE() >> 8) & 0xFF;

	for (i = 0; i < regions; i++) {
		/* Check if this region is enabled */
		MPU_RNR(i);
		rasr = MPU_RASR();
		if (!(rasr & 1))
			continue;
		for (k = 0; k < sizeof(m) / sizeof(m[0]); k++) {
			if ((rasr & m[k].r[0].msk) != m[k].r[0].val &&
			    (rasr & m[k].r[1].msk) != m[k].r[1].val)
				continue;
			if (k > best)
				best = k;
		}
	}

	return best < 0 ? "OFF" : m[best].nm;
}

/*
 * Get DCache current mode description
 */
char *v7m_dcache_mode(void)
{
	return !(CCR() & (1 << 16)) ? "OFF" : v7m_best_cache();
}

/*
 * Get ICache current mode description
 */
char *v7m_icache_mode(void)
{
	return !(CCR() & (1 << 17)) ? "OFF" : v7m_best_cache();
}

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
