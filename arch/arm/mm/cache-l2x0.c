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
#include <linux/spinlock.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>

#define CACHE_LINE_SIZE		32

#ifdef CONFIG_PL310_ASSOC_16_WAY
#define CACHE_ASSOC		16
#else
#define CACHE_ASSOC		8
#endif

static void __iomem *l2x0_base;

#ifdef CONFIG_CACHE_PL310
static inline void cache_wait(void __iomem *reg, unsigned long mask)
{
	/* cache operations are atomic */
}

#define _l2x0_lock(lock, flags)		((void)(flags))
#define _l2x0_unlock(lock, flags)	((void)(flags))

#define block_end(start, end)		(end)

#define L2CC_TYPE			"PL310/L2C-310"
#else
static inline void cache_wait(void __iomem *reg, unsigned long mask)
{
	/* wait for the operation to complete */
	while (readl(reg) & mask)
		;
}

static DEFINE_SPINLOCK(l2x0_lock);
#define _l2x0_lock(lock, flags)		spin_lock_irqsave(lock, flags)
#define _l2x0_unlock(lock, flags)	spin_unlock_irqrestore(lock, flags)

#define block_end(start, end)		((start) + min((end) - (start), 4096UL))

#define L2CC_TYPE			"L2x0"
#endif

static inline void cache_wait_always(void __iomem *reg, unsigned long mask)
{
	/* wait for the operation to complete */
	while (readl(reg) & mask)
		;
}

static inline void cache_sync(void)
{
	void __iomem *base = l2x0_base;
#ifdef CONFIG_ARM_ERRATA_484863
	unsigned long val = 0;
	asm volatile("swp %0, %0, [%1]\n"
		     : "+r" (val) : "r" (base + L2X0_CACHE_SYNC));
#else
	writel(0, base + L2X0_CACHE_SYNC);
#endif
	cache_wait(base + L2X0_CACHE_SYNC, 1);
}

static void l2x0_cache_sync(void)
{
	unsigned long flags;

	_l2x0_lock(&l2x0_lock, flags);
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

static inline void l2x0_inv_all(void)
{
	unsigned long flags, ways = (1 << CACHE_ASSOC) - 1;

	/* invalidate all ways */
	_l2x0_lock(&l2x0_lock, flags);
	writel(ways, l2x0_base + L2X0_INV_WAY);
	cache_wait_always(l2x0_base + L2X0_INV_WAY, ways);
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

static void l2x0_inv_range(unsigned long start, unsigned long end)
{
	void __iomem *base = l2x0_base;
	unsigned long flags;

	_l2x0_lock(&l2x0_lock, flags);
	if (start & (CACHE_LINE_SIZE - 1)) {
		start &= ~(CACHE_LINE_SIZE - 1);
		cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
		writel(start, base + L2X0_CLEAN_INV_LINE_PA);
		start += CACHE_LINE_SIZE;
	}

	if (end & (CACHE_LINE_SIZE - 1)) {
		end &= ~(CACHE_LINE_SIZE - 1);
		cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
		writel(end, base + L2X0_CLEAN_INV_LINE_PA);
	}

	while (start < end) {
		unsigned long blk_end = block_end(start, end);

		while (start < blk_end) {
			cache_wait(base + L2X0_INV_LINE_PA, 1);
			writel(start, base + L2X0_INV_LINE_PA);
			start += CACHE_LINE_SIZE;
		}

		if (blk_end < end) {
			_l2x0_unlock(&l2x0_lock, flags);
			_l2x0_lock(&l2x0_lock, flags);
		}
	}
	cache_wait(base + L2X0_INV_LINE_PA, 1);
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

static void l2x0_clean_range(unsigned long start, unsigned long end)
{
	void __iomem *base = l2x0_base;
	unsigned long flags;

	_l2x0_lock(&l2x0_lock, flags);
	start &= ~(CACHE_LINE_SIZE - 1);
	while (start < end) {
		unsigned long blk_end = block_end(start, end);

		while (start < blk_end) {
			cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
			writel(start, base + L2X0_CLEAN_LINE_PA);
			start += CACHE_LINE_SIZE;
		}

		if (blk_end < end) {
			_l2x0_unlock(&l2x0_lock, flags);
			_l2x0_lock(&l2x0_lock, flags);
		}
	}
	cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

static void l2x0_flush_range(unsigned long start, unsigned long end)
{
	void __iomem *base = l2x0_base;
	unsigned long flags;

	_l2x0_lock(&l2x0_lock, flags);
	start &= ~(CACHE_LINE_SIZE - 1);
	while (start < end) {
		unsigned long blk_end = block_end(start, end);

		while (start < blk_end) {
			cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
			writel(start, base + L2X0_CLEAN_INV_LINE_PA);
			start += CACHE_LINE_SIZE;
		}

		if (blk_end < end) {
			_l2x0_unlock(&l2x0_lock, flags);
			_l2x0_lock(&l2x0_lock, flags);
		}
	}
	cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

void __init l2x0_init(void __iomem *base, __u32 aux_val, __u32 aux_mask)
{
	__u32 aux;

	l2x0_base = base;

	/*
	 * Check if l2x0 controller is already enabled.
	 * If you are booting from non-secure mode
	 * accessing the below registers will fault.
	 */
	if (!(readl(l2x0_base + L2X0_CTRL) & 1)) {

		/* l2x0 controller is disabled */

		aux = readl(l2x0_base + L2X0_AUX_CTRL);
		aux &= aux_mask;
		aux |= aux_val;
		writel(aux, l2x0_base + L2X0_AUX_CTRL);

		l2x0_inv_all();

		/* enable L2X0 */
		writel(1, l2x0_base + L2X0_CTRL);
	}

	outer_cache.inv_range = l2x0_inv_range;
	outer_cache.clean_range = l2x0_clean_range;
	outer_cache.flush_range = l2x0_flush_range;
	outer_cache.sync = l2x0_cache_sync;

	pr_info(L2CC_TYPE " cache controller enabled (%08x)\n",
		readl(l2x0_base + L2X0_CACHE_ID));
}
