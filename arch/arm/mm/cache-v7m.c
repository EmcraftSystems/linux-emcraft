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

/* Instruction cache invalidate all to Point of Unification (PoU) */
#define ICIALLU(v)	*(volatile u32 *)0xE000EF50 = v
/* Instruction cache invalidate by address to PoU */
#define ICIMVAU(v)	*(volatile u32 *)0xE000EF58 = v
/* Data cache invalidate by address to Point of Coherency (PoC) */
#define DCIMVAC(v)	*(volatile u32 *)0xE000EF5C = v
/* Data cache clean by address to PoC */
#define DCCMVAC(v)	*(volatile u32 *)0xE000EF68 = v
/* Data cache clean and invalidate by address to PoC */
#define DCCIMVAC(v)	*(volatile u32 *)0xE000EF70 = v
/* Data cache clean and invalidate by set/way */
#define DCCISW(v)	*(volatile u32 *)0xE000EF74 = v
/* Branch predictor invalidate all */
#define BPIALL(v)	*(volatile u32 *)0xE000EF78 = v

/* Configuration and MPU Registers */
#define CCR()		(*(volatile u32 *)0xE000ED14)
#define CCSIDR()	(*(volatile u32 *)0xE000ED80)
#define CSSELR(v)	*(volatile u32 *)0xE000ED84 = v

#define MPU_TYPE()	(*(volatile u32 *)0xE000ED90)
#define MPU_RNR(v)	*(volatile u32 *)0xE000ED98 = v
#define MPU_RASR()	(*(volatile u32 *)0xE000EDA0)

#define CCSIDR_NSETS(x)	(((x) >> 13) & 0x7fff)	/* Number of sets (0-based) */
#define CCSIDR_ASC(x)	(((x) >> 3) & 0x3ff)	/* Associativity */
#define CCSIDR_LSZ(x)	(((x) >> 0) & 0x7)	/* Line Size */

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
 * Flush the entire cache
 */
void v7m_flush_kern_cache_all(void)
{
	u32	ccsidr;
	u32	nsets, asc, linesz;
	u32	set, way;
	int	wshift, sshift;

	/* Get cache configuration */
	CSSELR(0);
	asm("dsb");
	ccsidr = CCSIDR();

	nsets = CCSIDR_NSETS(ccsidr);
	asc = CCSIDR_ASC(ccsidr);
	linesz = CCSIDR_LSZ(ccsidr);

	sshift = linesz + 4;
	asm("clz %0, %1" : "=r" (wshift) : "r" (asc));

	/* Flush (clean & invalidate) all Data Cache set/ways */
	for (set = 0; set <= nsets; set++) {
		for (way = 0; way <= asc; way++) {
			u32 sw = (way << wshift) | (set << sshift) | (0 << 1);
			DCCISW(sw);
		}
	}

	/* Invalidate all Instruction Cache */
	asm("isb");
	ICIALLU(0);

	/* Invalidate BPU */
	BPIALL(0);

	asm("dsb");
	asm("isb");
}

/*
 * Flush all TLB entries
 */
void v7m_flush_user_cache_all(void)
{
	/* Do nothing */
}

/*
 * Flush a range of TLB entries in the specified address space.
 * - start - start address (may not be aligned)
 * - end   - end address (exclusive, may not be aligned)
 * - flags - vm_area_struct flags describing address space
 */
void v7m_flush_user_cache_range(unsigned long start, unsigned long end,
				unsigned int flags)
{
	/* Do nothing */
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
	asm("dsb");
	asm("isb");

	/* Synchronize unaligned head & tail */
	if (start & (L1_CACHE_BYTES - 1)) {
		/* Clean D-line */
		DCCMVAC(start);
		asm("dsb");
		/* Invalidate I-line */
		ICIMVAU(start);
		start = (start + L1_CACHE_BYTES) & ~(L1_CACHE_BYTES - 1);
	}
	if (end & (L1_CACHE_BYTES - 1)) {
		DCCMVAC(start);
		asm("dsb");
		ICIMVAU(start);
		end &= ~(L1_CACHE_BYTES - 1);
	}

	/* Synchronize aligned part */
	for ( ; start < end; start += L1_CACHE_BYTES) {
		DCCMVAC(start);
		asm("dsb");
		ICIMVAU(start);
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
	u32	s = (u32)addr, e = (u32)addr + size;

	asm("dsb");

	/* Flush cache-unaligned head & tail */
	if (s & (L1_CACHE_BYTES - 1)) {
		DCCIMVAC(s);
		s = (s + L1_CACHE_BYTES) & ~(L1_CACHE_BYTES - 1);
	}
	if (e & (L1_CACHE_BYTES - 1)) {
		DCCIMVAC(e);
		e &= ~(L1_CACHE_BYTES - 1);
	}

	/* Flush aligned part */
	for ( ; s < e; s += L1_CACHE_BYTES)
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

	asm("dsb");

	/* Flush cache-unaligned head & tail */
	if (s & (L1_CACHE_BYTES - 1)) {
		DCCIMVAC(s);
		s = (s + L1_CACHE_BYTES) & ~(L1_CACHE_BYTES - 1);
	}
	if (e & (L1_CACHE_BYTES - 1)) {
		DCCIMVAC(e);
		e &= ~(L1_CACHE_BYTES - 1);
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
	u32	s = (u32)start, e = (u32)end;

	asm("dsb");

	/* Clean cache-unaligned head & tail */
	if (s & (L1_CACHE_BYTES - 1)) {
		DCCMVAC(s);
		s = (s + L1_CACHE_BYTES) & ~(L1_CACHE_BYTES - 1);
	}
	if (e & (L1_CACHE_BYTES - 1)) {
		DCCMVAC(e);
		e &= ~(L1_CACHE_BYTES - 1);
	}

	/* Clean aligned part */
	for ( ; s < e; s += L1_CACHE_BYTES)
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
	u32	s = (u32)start, e = (u32)end;

	asm("dsb");

	/* Flush cache-unaligned head & tail */
	if (s & (L1_CACHE_BYTES - 1)) {
		DCCIMVAC(s);
		s = (s + L1_CACHE_BYTES) & ~(L1_CACHE_BYTES - 1);
	}
	if (e & (L1_CACHE_BYTES - 1)) {
		DCCIMVAC(e);
		e &= ~(L1_CACHE_BYTES - 1);
	}

	/* Flush aligned part */
	for ( ; s < e; s += L1_CACHE_BYTES)
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
