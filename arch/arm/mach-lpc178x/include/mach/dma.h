/*
 * arch/arm/mach-lpc178x/include/mach/dma.h
 *
 * (was named asm-arm/arch-lpc32xx/dma.h)
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
 *
 * Customized for LPC178x/7x by:
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _MACH_LPC178X_DMA_H_
#define _MACH_LPC178X_DMA_H_

#include <mach/platform.h>

#define MAX_DMA_CHANNELS 8

#define DMA_CH_SDCARD_TX	0
#define DMA_CH_SDCARD_RX	1
#define DMA_CH_I2S_TX		2
#define DMA_CH_I2S_RX		3
#define DMA_CH_SLCNAND		4

enum {
	DMA_NO_INT = 0,
	DMA_ERR_INT = 1,
	DMA_TC_INT = 2,
};

/*
 * DMA channel control structure
 */
struct dma_config {
	int ch;		/* Channel # to use */
	int tc_inten;	/* !0 = Enable TC interrupts for this channel */
	int err_inten;	/* !0 = Enable error interrupts for this channel */
	int src_size;	/* Source xfer size - must be 1, 2, or 4 */
	int src_inc;	/* !0 = Enable source address increment */
	int src_bsize;	/* Source burst size (ie, DMAC_CHAN_SRC_BURST_xxx) */
	u32 src_prph;	/* Source peripheral (ie, DMA_PERID_xxxx) */
	int dst_size;	/* Destination xfer size - must be 1, 2, or 4 */
	int dst_inc;	/* !0 = Enable destination address increment */
	int dst_bsize;	/* Destination burst size (DMAC_CHAN_DEST_BURST_xxx) */
	u32 dst_prph;	/* Destination peripheral (ie, DMA_PERID_xxxx) */
	u32 flowctrl;	/* Flow control (ie, DMAC_CHAN_FLOW_xxxxxx) */
};

void __init lpc178x_dma_init(void);

/*
 * Channel enable and disable functions
 */
extern int lpc178x_dma_ch_enable(int ch);
extern int lpc178x_dma_ch_disable(int ch);

/*
 * Channel allocation and deallocation functions
 */
extern int lpc178x_dma_ch_get(struct dma_config *dmachcfg,
				char *name,
				void *irq_handler,
				void *data);
extern int lpc178x_dma_ch_put(int ch);
extern int lpc178x_dma_ch_pause_unpause(int ch, int pause);

/*
 * Setup or start an unbound DMA transfer
 */
extern int lpc178x_dma_start_pflow_xfer(int ch,
					void *src,
					void *dst,
					int enable);

/*
 * DMA channel status
 */
extern int lpc178x_dma_is_active(int ch);

/*
 * DMA linked list support
 */
extern u32 lpc178x_dma_alloc_llist(int ch,
				   int entries);
extern void lpc178x_dma_dealloc_llist(int ch);
extern u32 lpc178x_dma_llist_v_to_p(int ch,
				    u32 vlist);
extern u32 lpc178x_dma_llist_p_to_v(int ch,
				    u32 plist);
extern u32 lpc178x_dma_get_llist_head(int ch);
extern void lpc178x_dma_flush_llist(int ch);
extern u32 lpc178x_dma_queue_llist_entry(int ch,
					 void *src,
					 void *dst,
					 int size);
extern u32 lpc178x_get_free_llist_entry(int ch);
extern u32 lpc178x_dma_queue_llist(int ch,
				   void *src,
				   void *dst,
				   int size,
				   u32 ctrl);
extern int lpc178x_dma_start_xfer(int chan, u32 config);
extern void lpc178x_dma_force_burst(int ch, int src);

/*
 * Function aliases to be used by drivers initially written for LPC32xx
 */
#define lpc32xx_dma_ch_enable		lpc178x_dma_ch_enable
#define lpc32xx_dma_ch_disable		lpc178x_dma_ch_disable
#define lpc32xx_dma_ch_get		lpc178x_dma_ch_get
#define lpc32xx_dma_ch_put		lpc178x_dma_ch_put
#define lpc32xx_dma_ch_pause_unpause	lpc178x_dma_ch_pause_unpause
#define lpc32xx_dma_start_pflow_xfer	lpc178x_dma_start_pflow_xfer
#define lpc32xx_dma_is_active		lpc178x_dma_is_active
#define lpc32xx_dma_alloc_llist		lpc178x_dma_alloc_llist
#define lpc32xx_dma_dealloc_llist	lpc178x_dma_dealloc_llist
#define lpc32xx_dma_llist_v_to_p	lpc178x_dma_llist_v_to_p
#define lpc32xx_dma_llist_p_to_v	lpc178x_dma_llist_p_to_v
#define lpc32xx_dma_get_llist_head	lpc178x_dma_get_llist_head
#define lpc32xx_dma_flush_llist		lpc178x_dma_flush_llist
#define lpc32xx_dma_queue_llist_entry	lpc178x_dma_queue_llist_entry
#define lpc32xx_get_free_llist_entry	lpc178x_get_free_llist_entry
#define lpc32xx_dma_queue_llist		lpc178x_dma_queue_llist
#define lpc32xx_dma_start_xfer		lpc178x_dma_start_xfer
#define lpc32xx_dma_force_burst		lpc178x_dma_force_burst

#endif /* _MACH_LPC178X_DMA_H_ */
