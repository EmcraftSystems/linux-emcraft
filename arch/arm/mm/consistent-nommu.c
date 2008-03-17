/*
 *  linux/arch/arm/mm/consistent.c
 *
 *  Copyright (C) 2000-2004 Russell King
 *  Modified by Catalin Marinas for MMU-less support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  DMA uncached mapping support.
 */
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/memory.h>
#include <asm/cacheflush.h>
#include <asm/sizes.h>

static void *
__dma_alloc(struct device *dev, size_t size, dma_addr_t *handle, gfp_t gfp)
{
	struct page *page, *end;
	unsigned long order;
	u64 mask = ISA_DMA_THRESHOLD;
	void *ptr;

	if (dev) {
		mask = dev->coherent_dma_mask;

		/*
		 * Sanity check the DMA mask - it must be non-zero, and
		 * must be able to be satisfied by a DMA allocation.
		 */
		if (mask == 0) {
			dev_warn(dev, "coherent DMA mask is unset\n");
			goto no_page;
		}

		if ((~mask) & ISA_DMA_THRESHOLD) {
			dev_warn(dev, "coherent DMA mask %#llx is smaller "
				 "than system GFP_DMA mask %#llx\n",
				 mask, (unsigned long long)ISA_DMA_THRESHOLD);
			goto no_page;
		}
	}

	size = PAGE_ALIGN(size);
	order = get_order(size);

	if (mask != 0xffffffff)
		gfp |= GFP_DMA;

	page = alloc_pages(gfp, order);
	if (!page)
		goto no_page;

	/*
	 * Invalidate any data that might be lurking in the
	 * kernel direct-mapped region for device DMA.
	 */
	ptr = page_address(page);
	memset(ptr, 0, size);
	dmac_flush_range(ptr, ptr + size);
	outer_flush_range(__pa(ptr), __pa(ptr) + size);

	end = page + (1 << order);

	split_page(page, order);

	/*
	 * Set the "dma handle"
	 */
	*handle = page_to_dma(dev, page);

	do {
		/*
		 * x86 does not mark the pages reserved...
		 */
		SetPageReserved(page);
		page++;
	} while (size -= PAGE_SIZE);

	/*
	 * Free the otherwise unused pages.
	 */
	while (page < end) {
		__free_page(page);
		page++;
	}

 	return ptr;

 no_page:
	*handle = ~0;
	return NULL;
}

/*
 * Allocate DMA-coherent memory space and return both the kernel and
 * bus address for that space.
 */
void *
dma_alloc_coherent(struct device *dev, size_t size, dma_addr_t *handle, gfp_t gfp)
{
	return __dma_alloc(dev, size, handle, gfp);
}
EXPORT_SYMBOL(dma_alloc_coherent);

/*
 * Allocate a writecombining region, in the same way as
 * dma_alloc_coherent above.
 */
void *
dma_alloc_writecombine(struct device *dev, size_t size, dma_addr_t *handle, gfp_t gfp)
{
	return __dma_alloc(dev, size, handle, gfp);
}
EXPORT_SYMBOL(dma_alloc_writecombine);

static int dma_mmap(struct device *dev, struct vm_area_struct *vma,
		    void *cpu_addr, dma_addr_t dma_addr, size_t size)
{
	unsigned long user_size;
	int ret = -ENXIO;

	user_size = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;

	vma->vm_flags |= VM_RESERVED;
	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			      user_size << PAGE_SHIFT, vma->vm_page_prot);

	return ret;
}

int dma_mmap_coherent(struct device *dev, struct vm_area_struct *vma,
		      void *cpu_addr, dma_addr_t dma_addr, size_t size)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return dma_mmap(dev, vma, cpu_addr, dma_addr, size);
}
EXPORT_SYMBOL(dma_mmap_coherent);

int dma_mmap_writecombine(struct device *dev, struct vm_area_struct *vma,
			  void *cpu_addr, dma_addr_t dma_addr, size_t size)
{
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return dma_mmap(dev, vma, cpu_addr, dma_addr, size);
}
EXPORT_SYMBOL(dma_mmap_writecombine);

/*
 * free a page as defined by the above mapping.
 * Must not be called with IRQs disabled.
 */
void dma_free_coherent(struct device *dev, size_t size, void *cpu_addr, dma_addr_t handle)
{
	struct page *page;

	WARN_ON(irqs_disabled());

	size = PAGE_ALIGN(size);
	page = virt_to_page(cpu_addr);

	do {
		/*
		 * x86 does not mark the pages reserved...
		 */
		ClearPageReserved(page);

		__free_page(page);
		page++;
	} while (size -= PAGE_SIZE);
}
EXPORT_SYMBOL(dma_free_coherent);

/*
 * Make an area consistent for devices.
 * Note: Drivers should NOT use this function directly, as it will break
 * platforms with CONFIG_DMABOUNCE.
 * Use the driver DMA support - see dma-mapping.h (dma_sync_*)
 */
void consistent_sync(const void *start, size_t size, int direction)
{
	const void *end = start + size;

	switch (direction) {
	case DMA_FROM_DEVICE:		/* invalidate only */
		dmac_inv_range(start, end);
		outer_inv_range(__pa(start), __pa(end));
		break;
	case DMA_TO_DEVICE:		/* writeback only */
		dmac_clean_range(start, end);
		outer_clean_range(__pa(start), __pa(end));
		break;
	case DMA_BIDIRECTIONAL:		/* writeback and invalidate */
		dmac_flush_range(start, end);
		outer_flush_range(__pa(start), __pa(end));
		break;
	default:
		BUG();
	}
}
EXPORT_SYMBOL(consistent_sync);
