
#ifndef _DMA_MEM_H_
#define _DMA_MEM_H_

/*
 * Init dmamem driver
 */
int dmamem_init(int memnode);

/*
 * Allocate pages from dmamem region
 */
caddr_t dmamem_alloc(size_t size, size_t align, int gfp);

/*
 * Free pages previously allocated with dmamem_xxx()
 */
void dmamem_free(caddr_t adr);

/*
 * Get 'fb' area reserved in dmamem
 */
int dmamem_fb_get(dma_addr_t *base, unsigned long *size);

#endif /* _DMA_MEM_H_ */

