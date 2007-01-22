/*
 * arch/arm/mm/cache-l2x0.c - L210/L220 cache controller support
 *
 * Copyright (C) 2007 ARM Limited
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
#include <linux/init.h>

#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/hardware/cache-l2x0.h>

#define log2(x)			ffz(~(x))
#define CACHE_LINE_SIZE		32

static void __iomem *l2x0_base;
static unsigned long way_size;

static inline void sync_writel(unsigned long val, unsigned long reg,
			       unsigned long complete_mask)
{
	writel(val, l2x0_base + reg);
	/* wait for the operation to complete */
	while (readl(l2x0_base + reg) & complete_mask)
		;
}

static inline void cache_sync(void)
{
	sync_writel(0, L2X0_CACHE_SYNC, 1);
}

static inline void cacheline_index_op(unsigned long addr, unsigned long reg)
{
	unsigned long way, index;

	for (way = 0; way < 8; way++)
		for (index = 0; index < way_size; index += PAGE_SIZE) {
			unsigned long val = (way << 29) | index
				| (addr & (PAGE_SIZE - 1));
			sync_writel(val, reg, 1);
		}
}

static inline void l2x0_inv_all(void)
{
	/* invalidate all ways */
	sync_writel(0xff, L2X0_INV_WAY, 0xff);
	cache_sync();
}

static inline void l2x0_clean_all(void)
{
	/* clean all ways */
	sync_writel(0xff, L2X0_CLEAN_WAY, 0xff);
	cache_sync();
}

static inline void l2x0_flush_all(void)
{
	/* clean and invalidate all ways */
	sync_writel(0xff, L2X0_CLEAN_INV_WAY, 0xff);
	cache_sync();
}

#if 0
static void l2x0_inv_range(unsigned long start, unsigned long end)
{
	l2x0_inv_all();
}
#endif

static void l2x0_clean_range(unsigned long start, unsigned long end)
{
	unsigned long size = end - start;
	unsigned long addr;

	if (size >= PAGE_SIZE) {
		l2x0_clean_all();
		return;
	}

	/* no physical address information, flush by index/way */
	for (addr = start & ~(CACHE_LINE_SIZE - 1); addr < end;
	     addr += CACHE_LINE_SIZE)
		cacheline_index_op(addr, L2X0_CLEAN_LINE_IDX);
	cache_sync();
}

static void l2x0_flush_range(unsigned long start, unsigned long end)
{
	unsigned long size = end - start;
	unsigned long addr;

	if (size >= PAGE_SIZE) {
		l2x0_flush_all();
		return;
	}

	/* no physical address information, flush by index/way */
	for (addr = start & ~(CACHE_LINE_SIZE - 1); addr < end;
	     addr += CACHE_LINE_SIZE)
		cacheline_index_op(addr, L2X0_CLEAN_INV_LINE_IDX);
	cache_sync();
}

void __init l2x0_init(void __iomem *base, unsigned long cache_size)
{
	unsigned long aux, way_size_enc;

	l2x0_base = base;
	way_size = cache_size >> 3;
	way_size_enc = log2(way_size) - 13;

	/* disable L2X0 */
	writel(0, l2x0_base + L2X0_CTRL);

	/* 8-way associativity, evmon/parity/share enabled
	 * Bits:  .... ...0 0111 ???1 0000 .... .... .... */
	aux = readl(l2x0_base + L2X0_AUX_CTRL);
	aux &= 0xfe000fff;
	aux |= 0x00710000 | (way_size_enc << 17);
	writel(aux, l2x0_base + L2X0_AUX_CTRL);

	l2x0_inv_all();

	/* enable L2X0 */
	writel(1, l2x0_base + L2X0_CTRL);

#if 0
	/* the invalidate range operation must be restricted only to
	 * the given address range, otherwise it would invalidate
	 * dirty cache lines outside the range. Since the virtual to
	 * physical translation is not implemented yet, it is safer to
	 * flush the cache here (this is only an intermediate
	 * solution) */
	outer_cache.inv_range = l2x0_inv_range;
#else
	outer_cache.inv_range = l2x0_flush_range;
#endif
	outer_cache.clean_range = l2x0_clean_range;
	outer_cache.flush_range = l2x0_flush_range;

	printk(KERN_INFO "L2X0 cache controller enabled\n");
}
