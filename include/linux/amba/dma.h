/*
 *  linux/include/linux/amba/dma.h
 *
 * Copyright (C) 2006 ARM Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *	DMA API for any AMBA system supporting DMA
 *      i.e. there may or may not be an AMBA DMA controller in the system
 */
#ifndef LINUX_AMBA_DMA_H
#define LINUX_AMBA_DMA_H

/*
 *  Data from the peripheral using DMA, required by the AMBA DMA manager
 */
struct amba_dma_data {
	/* DMAC interrupt data as known by the board */
	int  (*irq_ignore)(dmach_t chan); /* Requesting device should ignore this interrupt */
	void (*irq_pre)(dmach_t chan);    /* Any necessary DMA operations e.g error checking */
	void (*irq_post)(dmach_t chan);   /* Clear the interrupt and carry out any necessary DMA operations e.g re-enable DMAC channel */
	unsigned int packet_size;         /* Size, in bytes, of each DMA transfer packet */
	void * dmac_data;                 /* DMA controller specific data e.g. for pl080/pl041 TX transfers, the pl040 TX FIFO offset */ 
};

extern void amba_set_ops(struct dma_ops * board_ops);

#endif

