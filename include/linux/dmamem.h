
#ifndef _DMA_MEM_H_
#define _DMA_MEM_H_

/*
 * Allocate pages from dmamem region
 */
caddr_t dmamem_alloc(size_t size, size_t align, int gfp);

/*
 * Free pages previously allocated with dmamem_xxx()
 */
void dmamem_free(caddr_t adr);

/*
 * Get 'start' and 'end' of dmamem region
 */
void dmamem_area(dma_addr_t *start, dma_addr_t *end);

#endif /* _DMA_MEM_H_ */

