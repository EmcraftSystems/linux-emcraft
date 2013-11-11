/*
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
#include <linux/types.h>
#include <linux/module.h>

/*
 * Local Memory Controller: Cache control register
 */
#define KINETIS_LMEM_PSCCR		(*(volatile u32 *)0xe0082800)
#define KINETIS_LMEM_PSCLCR		(*(volatile u32 *)0xe0082804)
#define KINETIS_LMEM_PSCSAR		(*(volatile u32 *)0xe0082808)

/* Initiate Cache Command */
#define KINETIS_LMEM_CCR_GO_MSK		(1 << 31)
/* Push Way 1 */
#define KINETIS_LMEM_CCR_PUSHW1_MSK	(1 << 27)
/* Invalidate Way 1 */
#define KINETIS_LMEM_CCR_INVW1_MSK	(1 << 26)
/* Push Way 0 */
#define KINETIS_LMEM_CCR_PUSHW0_MSK	(1 << 25)
/* Invalidate Way 0 */
#define KINETIS_LMEM_CCR_INVW0_MSK	(1 << 24)
/* Enable Write Buffer */
#define KINETIS_LMEM_CCR_ENWRBUF_MSK	(1 << 1)
/* Cache enable */
#define KINETIS_LMEM_CCR_ENCACHE_MSK	(1 << 0)

#define KINETIS_LMEM_LCR_LADSEL_MSK	(1 << 26)
#define KINETIS_LMEM_LCR_LCMD		24

#define KINETIS_LMEM_SAR_LGO_MSK	(1 << 0)

/* Cache line size */
#define KINETIS_CACHE_LINE_SIZE		0x10

/*
 * Disable Kinetis Processor System (PS) cache and save previous state
 */
void kinetis_ps_cache_save(unsigned long *flags)
{
	/* Save previous state of cache */
	*flags = KINETIS_LMEM_PSCCR &
		(KINETIS_LMEM_CCR_ENWRBUF_MSK | KINETIS_LMEM_CCR_ENCACHE_MSK);

	/* Clear and disable cache */
	KINETIS_LMEM_PSCCR =
		KINETIS_LMEM_CCR_GO_MSK |
		KINETIS_LMEM_CCR_PUSHW1_MSK | KINETIS_LMEM_CCR_INVW1_MSK |
		KINETIS_LMEM_CCR_PUSHW0_MSK | KINETIS_LMEM_CCR_INVW0_MSK;
	/* Wait for the cache operation to finish */
	while (KINETIS_LMEM_PSCCR & KINETIS_LMEM_CCR_GO_MSK);
}
EXPORT_SYMBOL(kinetis_ps_cache_save);

/*
 * Restore a previous state of Kinetis Processor System (PS) cache
 */
void kinetis_ps_cache_restore(unsigned long *flags)
{
	/* Invalidate and optionally enable cache */
	KINETIS_LMEM_PSCCR =
		KINETIS_LMEM_CCR_GO_MSK |
		KINETIS_LMEM_CCR_INVW1_MSK | KINETIS_LMEM_CCR_INVW0_MSK |
		(*flags);
	/* Wait for the cache operation to finish */
	while (KINETIS_LMEM_PSCCR & KINETIS_LMEM_CCR_GO_MSK);
}
EXPORT_SYMBOL(kinetis_ps_cache_restore);

/*
 * Flush Kinetis Processor System (PS) cache
 */
void kinetis_ps_cache_flush(void)
{
	unsigned long	flags;

	kinetis_ps_cache_save(&flags);
	kinetis_ps_cache_restore(&flags);
}
EXPORT_SYMBOL(kinetis_ps_cache_flush);

/*
 * Flush data cache lines
 */
void kinetis_ps_cache_flush_mlines(void *adr, unsigned long len)
{
	void	*end_adr = (void *)((u32)adr + len);

	adr = (void *)((u32)adr & ~(KINETIS_CACHE_LINE_SIZE - 1));
	do {
		KINETIS_LMEM_PSCLCR = KINETIS_LMEM_LCR_LADSEL_MSK |
				      (0x2 << KINETIS_LMEM_LCR_LCMD);
		KINETIS_LMEM_PSCSAR = ((u32)adr & ~0x3) |
				      KINETIS_LMEM_SAR_LGO_MSK;
		while (KINETIS_LMEM_PSCSAR & KINETIS_LMEM_SAR_LGO_MSK);

		adr = (void *)((u32)adr + KINETIS_CACHE_LINE_SIZE);
	} while (adr < end_adr);
}
EXPORT_SYMBOL(kinetis_ps_cache_flush_mlines);

/*
 * Invalidate data cache lines
 */
void kinetis_ps_cache_inval_mlines(void *adr, unsigned long len)
{
	void	*end_adr = (void *)((u32)adr + len);

	adr = (void *)((u32)adr & ~(KINETIS_CACHE_LINE_SIZE - 1));
	do {
		KINETIS_LMEM_PSCLCR = KINETIS_LMEM_LCR_LADSEL_MSK |
				      (0x1 << KINETIS_LMEM_LCR_LCMD);
		KINETIS_LMEM_PSCSAR = ((u32)adr & ~0x3) |
				      KINETIS_LMEM_SAR_LGO_MSK;
		while (KINETIS_LMEM_PSCSAR & KINETIS_LMEM_SAR_LGO_MSK);

		adr = (void *)((u32)adr + KINETIS_CACHE_LINE_SIZE);
	} while (adr < end_adr);
}
EXPORT_SYMBOL(kinetis_ps_cache_inval_mlines);

