/*
 * arch/arm/mm/cache-v7m-usr.c
 *
 * This driver allows to control (flush/invalidate) ARM-V7M inner L1
 * cache handling from user-space.
 *
 * Copyright (C) 2016 Yuri Tikhonov (yur@emcraft.com), EmCraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/fs.h>

#include <asm/cacheflush.h>
#include <asm/uaccess.h>

#include "cache-v7m-usr.h"

/*
 * Define this to enable checking if the requested area is fully fit into
 * some non-cacheable region, and thus its synchronization may be skipped
 */
#undef V7M_CACHE_CHECK

/*
 * Driver name
 */
#define V7M_CACHE_NAME		"v7m-cache-usr"

#if defined(V7M_CACHE_CHECK)
/*
 * MPU constants
 */
# define V7M_MPU_BASE		0xE000ED90
# define V7M_MPU_AREA_NUM	8

/*
 * MPU registers
 */
struct v7m_mpu {
	unsigned long		type;	/* Type Register */
	unsigned long		ctrl;	/* Control Register */
	unsigned long		rnr;	/* Region Number Register */
	unsigned long		rbar;	/* Region Base Address Register */
	unsigned long		rasr;	/* Region Attribute and Size Register */
};

/*
 * Area descriptor
 */
struct area_dsc {
	struct list_head	node;
	unsigned long		adr;
	unsigned long		len;
};

/*
 * Non-cached regions
 */
static LIST_HEAD(non_cached_area_lst);
#endif /* V7M_CACHE_CHECK */

static long cache_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
#if defined(V7M_CACHE_CHECK)
	struct area_dsc		*p;
#endif
	struct cache_v7m_reg	reg;
	int			rv;

	/*
	 * Get region params
	 */
	rv = copy_from_user(&reg, (void __user *)arg, sizeof(reg));
	if (rv) {
		printk(KERN_ERR "%s: failed copy from user (%d)\n",
			V7M_CACHE_NAME, rv);
		goto out;
	}

	if (!reg.len) {
		printk(KERN_ERR "%s: bad 0x%08lx region len\n",
			V7M_CACHE_NAME, reg.start);
		rv = -EINVAL;
		goto out;
	}

#if defined(V7M_CACHE_CHECK)
	/*
	 * If the region requested is fully in the non-cacheable mem, then
	 * do nothing
	 */
	list_for_each_entry(p, &non_cached_area_lst, node) {
		if (reg.start < p->adr || reg.start > p->adr + p->len - 1)
			continue;
		if (reg.start + reg.len > p->adr + p->len)
			continue;

		rv = 0;
		goto out;
	}
#endif

	/*
	 * Execute command
	 */
	switch (cmd) {
	case CACHE_V7M_IOC_CLEAN:
		v7m_dma_clean_range((void *)reg.start,
				    (void *)(reg.start + reg.len));
		break;
	case CACHE_V7M_IOC_INV:
		v7m_dma_inv_range((void *)reg.start,
				  (void *)(reg.start + reg.len));
		break;
	case CACHE_V7M_IOC_FLUSH:
		v7m_dma_flush_range((void *)reg.start,
				    (void *)(reg.start + reg.len));
		break;
	default:
		rv = -EINVAL;
		goto out;
	}

	rv = 0;
out:
	return rv;
}

static struct file_operations cache_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= cache_ioctl,
};

static int __init cache_init_module(void)
{
#if defined(V7M_CACHE_CHECK)
	volatile struct v7m_mpu	*mpu = (void *)V7M_MPU_BASE;
	struct area_dsc		*dsc;
	unsigned int		tex, c, b, i;
#endif
	int			rv;

#if defined(V7M_CACHE_CHECK)
	/*
	 * Get the list of uncached areas
	 */
	for (i = 0; i < V7M_MPU_AREA_NUM; i++) {
		/*
		 * Check if region is enabled (valid)
		 */
		mpu->rnr = i;
		if (!(mpu->rasr & 1))
			continue;

		/*
		 * Check if region is cached, see the tables
		 * 'Table B3-50 MPU_RASR bit assignments' and
		 * 'Table B3-51 TEX, C, B, and S Encoding' in
		 * ARMv7-M Architecture Reference Manual
		 */
		tex = (mpu->rasr >> 19) & 0x7;
		c   = (mpu->rasr >> 17) & 0x1;
		b   = (mpu->rasr >> 16) & 0x1;
		if (!(tex == 1 && c == 0 && b == 0) &&
		    !(tex == 4 && c == 0 && b == 0))
			continue;

		/*
		 * The region is non-cacheable, add it to the list
		 */
		dsc = kmalloc(sizeof(*dsc), GFP_KERNEL);
		if (!dsc) {
			printk(KERN_WARNING "%s: kmalloc(%d) failure\n",
				__func__, sizeof(*dsc));
			continue;
		}

		dsc->adr = mpu->rbar & ~0x1F;
		dsc->len = 2 << (1 + ((mpu->rasr >> 1) & 0x1F));
		list_add_tail(&dsc->node, &non_cached_area_lst);
	}
#endif

	/*
	 * Register char device
	 */
	rv = register_chrdev(V7M_CACHE_MAJOR, V7M_CACHE_NAME, &cache_fops);
	if (rv < 0) {
		printk(KERN_ERR "%s: register_chrdev(%d) failed %d\n",
			__func__, V7M_CACHE_MAJOR, rv);
		goto out;
	}

	rv = 0;
out:
	return rv;
}

static void cache_cleanup_module(void)
{
#if defined(V7M_CACHE_CHECK)
	struct area_dsc	*p, *tmp;
#endif

	/*
	 * Unregister device
	 */
	unregister_chrdev(V7M_CACHE_MAJOR, V7M_CACHE_NAME);

#if defined(V7M_CACHE_CHECK)
	/*
	 * Free mem
	 */
	list_for_each_entry_safe(p, tmp, &non_cached_area_lst, node)
		kfree(p);
#endif
}

module_init(cache_init_module);
module_exit(cache_cleanup_module);

MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_DESCRIPTION("V7M Cache user-level control driver");
MODULE_LICENSE("GPL");

