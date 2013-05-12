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
/* Code bus cache */
#define KINETIS_LMEM_PCCCR		(*(volatile u32 *)0xe0082000)
/* System bus cache */
#define KINETIS_LMEM_PSCCR		(*(volatile u32 *)0xe0082800)
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

