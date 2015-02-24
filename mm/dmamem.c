/*
 * linux/mm/dmamem.c
 *
 * Copyright (C) 2015 Yuri Tikhonov (yur@emcraft.com), EmCraft Systems
 *
 * Based on M. Welsh (mdw@cs.cornell.edu) bigphysarea.c driver, extended
 * and adapted for linux-2.6.x by J. Joe Feise <jfeise@feise.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/bootmem.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/dmamem.h>

#include <asm/setup.h>

#define DM_NAME		"dmamem"

/*
 * dmamem block descriptor
 */
struct dm_dsc {
	struct dm_dsc	*next;
	caddr_t		base;		/* base of allocated block */
	size_t		size;		/* size in bytes */
};

static int dm_proc(char *page, char **start, off_t off,
		   int count, int *eof, void *data);

static unsigned long	dm_base;
static unsigned long	dm_sz_all;
static unsigned long	dm_sz_fb;

static struct dm_dsc	*dm_lst_free;
static struct dm_dsc	*dm_lst_used;

static struct resource	dm_res = {
	.name	= DM_NAME " area",
	.flags	= IORESOURCE_MEM | IORESOURCE_BUSY,
};

static int __init parse_tag_dmamem(const struct tag *tag)
{
	/* Check if fb size isn't more than whole size */
	if (tag->u.dmamem.sz_all < tag->u.dmamem.sz_fb)
		return 0;

	/* Check if base & sizes are page aligned */
	if (tag->u.dmamem.base & (PAGE_SIZE - 1) ||
	    tag->u.dmamem.sz_fb & (PAGE_SIZE - 1) ||
	    tag->u.dmamem.sz_all & (PAGE_SIZE - 1)) {
		return 0;
	}

	dm_base = tag->u.dmamem.base;
	dm_sz_fb = tag->u.dmamem.sz_fb;
	dm_sz_all = tag->u.dmamem.sz_all;

	return 0;
}
__tagtable(ATAG_DMAMEM, parse_tag_dmamem);

static int __init dm_init(void)
{
	struct proc_dir_entry	*res;
	int			rv;

	if (!dm_sz_all) {
		printk(KERN_CRIT "%s: no correct ATAG found, "
			"no coherent mem will be available.\n", DM_NAME);
		rv = 0;
		goto out;
	}

	/* Register the resource for mem allocation */
	dm_res.start = dm_base;
	dm_res.end = dm_base + dm_sz_all;
	request_resource(&iomem_resource, &dm_res);

	/* Create free list */
	dm_lst_free = kmalloc(sizeof(struct dm_dsc), GFP_KERNEL);
	if (!dm_lst_free) {
		rv = -ENOMEM;
		goto out;
	}
	dm_lst_free->next = NULL;
	dm_lst_free->base = (void *)(dm_base + dm_sz_fb);
	dm_lst_free->size = dm_sz_all - dm_sz_fb;

	/*
	 * Create /proc entry for it
	 */
	res = create_proc_entry(DM_NAME, 0444, NULL);
	if (!res) {
		/*
		 * continue without proc support, it is not fatal in itself
		 */
		printk(KERN_INFO "%s: failed create proc entry\n", DM_NAME);
	} else {
		res->read_proc = dm_proc;
	}

	rv = 0;
out:
	return rv;
}
__initcall(dm_init);

/*
 * Reserve memory for dmamem
 */
int dmamem_init(int memnode)
{
	pg_data_t *pgdat;
	int rv;

	pgdat = NODE_DATA(memnode);
	rv = reserve_bootmem_node(pgdat, dm_base, dm_sz_all,
				BOOTMEM_EXCLUSIVE);
	if (rv) {
		 printk(KERN_ERR "%s: 0x%08lx+%08lx overlaps in-use "
			"memory region\n", DM_NAME, dm_base, dm_sz_all);
	}

	return rv;
}

/*
 * Allocate mem from dmamem region
 */
caddr_t dmamem_alloc(size_t size, size_t align, int gfp)
{
	struct dm_dsc	*dm, *ndm, *adm, **pdm;
	caddr_t		abase = 0;
	int		count, a;

	count = (size + PAGE_SIZE - 1) / PAGE_SIZE;

	ndm = NULL;
	adm = NULL;

	a = align;
	if (!align)
		align = PAGE_SIZE;
	if (align % PAGE_SIZE)
		align = (align + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

	/*
	 * Search a free block which is large enough, even with alignment.
	 */
	pdm = &dm_lst_free;
	while (*pdm != NULL) {
		dm= *pdm;
		abase = (caddr_t)((((unsigned long)dm->base +
				    align - 1) / align) * align);

		if (abase + count * PAGE_SIZE <= dm->base + dm->size)
			break;

		pdm = &dm->next;
	}

	dm = *pdm;
	if (!dm)
		return NULL;

	/*
	 * When we have to align, the pages needed for alignment can
	 * be put back to the free pool.
	 * We check here if we need a second dmamem data structure later
	 * and allocate it now, so that we don't have to check for a
	 * failed kmalloc later.
	 */
	if (abase - dm->base + count * PAGE_SIZE < dm->size) {
		ndm = kmalloc(sizeof(struct dm_dsc), gfp);
		if (!ndm)
			return NULL;
	}

	if (abase != dm->base) {
		adm = kmalloc(sizeof(struct dm_dsc), gfp);
		if (!adm) {
			if (ndm)
				kfree(ndm);
			return NULL;
		}
		adm->base = dm->base;
		adm->size = abase - dm->base;
		dm->base = abase;
		dm->size -= adm->size;
		adm->next = dm;
		*pdm = adm;
		pdm = &adm->next;
	}

	if (ndm) {
		/*
		 * Range is larger than needed, create a new list element for
		 * the used list and shrink the element in the free list.
		 */
		ndm->base = dm->base;
		ndm->size = count * PAGE_SIZE;
		dm->base = ndm->base + ndm->size;
		dm->size = dm->size - ndm->size;
	} else {
		/*
		 * Range fits perfectly, remove it from free list.
		 */
		*pdm = dm->next;
		ndm = dm;
	}

	/*
	 * Insert block into used list
	 */
	ndm->next = dm_lst_used;
	dm_lst_used = ndm;

	return ndm->base;
}

/*
 * Free pages previously allocated with dmamem_xxx()
 */
void dmamem_free(caddr_t base)
{
	struct dm_dsc *prev, *next, *dm, **pdm;

	/*
	 * Search the block in the used list.
	 */
	for (pdm = &dm_lst_used; *pdm; pdm = &(*pdm)->next) {
		if ((*pdm)->base == base)
			break;
	}

	if (*pdm == NULL) {
		printk("%s: 0x%08x not allocated!\n", DM_NAME, (unsigned)base);
		return;
	}

	/*
	 * Remove range from the used list:
	 */
	dm = *pdm;
	*pdm = (*pdm)->next;

	/*
	 * The free-list is sorted by address, search insertion point
	 * and insert block in free list.
	 */
	for (pdm = &dm_lst_free, prev = NULL; *pdm;
	     prev = *pdm, pdm = &(*pdm)->next) {
		if ((*pdm)->base >= base)
			break;
	}

	/*
	 * Concatenate free range with neighbors, if possible.
	 * Try for upper neighbor (next in list) first, then
	 * for lower neighbor (predecessor in list).
	 */
	dm->next = *pdm;
	*pdm = dm;

	if (dm->next &&
	    dm->base + dm->size == dm->next->base) {
		next = dm->next;
		dm->size += dm->next->size;
		dm->next = next->next;
		kfree(next);
	}

	if (prev &&
	    prev->base + prev->size == dm->base) {
		prev->size += prev->next->size;
		prev->next = dm->next;
		kfree(dm);
	}
}

/*
 * Get 'fb' area reserved in dmamem
 */
int dmamem_fb_get(dma_addr_t *base, unsigned long *size)
{
	if (!dm_sz_all)
		return -ENODEV;

	if (base)
		*base = dm_base;

	if (size)
		*size = dm_sz_fb;

	return 0;
}

/*
 * Called on /proc read
 */
static int dm_proc(char *page, char **start, off_t off,
		   int count, int *eof, void *data)
{
	struct dm_dsc	*dm;
	int		len;
	int		free_count, free_total, free_max;
	int		used_count, used_total, used_max;

	free_count = 0;
	free_total = 0;
	free_max   = 0;
	for (dm = dm_lst_free; dm; dm = dm->next) {
		free_count++;
		free_total += dm->size;
		if (dm->size > free_max)
			free_max = dm->size;
	}

	used_count = 0;
	used_total = 0;
	used_max   = 0;
	for (dm = dm_lst_used; dm; dm = dm->next) {
		used_count++;
		used_total += dm->size;
		if (dm->size > used_max)
			used_max = dm->size;
	}

	if (!dm_sz_all) {
		len = sprintf(page, "No %s area allocated!\n", DM_NAME);
		goto out;
	}

	len = sprintf(page,
		"%s area, fb mem size %ld kB, gen dma mem size %ld kB\n"
		"                       free list:             used list:\n"
		"number of blocks:      %8d               %8d\n"
		"size of largest block: %8d kB            %8d kB\n"
		"total:                 %8d kB            %8d kB\n",
		DM_NAME, dm_sz_fb / 1024, (dm_sz_all - dm_sz_fb) / 1024,
		free_count, used_count,
		free_max / 1024, used_max / 1024,
		free_total / 1024, used_total /1024);
out:
	/*
	 * Calculate /proc metrics
	 */
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;
}

