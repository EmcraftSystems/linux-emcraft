/*
 *	linux/amba/pl080.h - ARM PrimeCell AACI DMA Controller driver
 *
 *	Copyright (C) 2005 ARM Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Please credit ARM.com
 * Documentation: ARM DDI 0196D
 */

#ifndef AMBA_PL080_H
#define AMBA_PL080_H 


/*
 * Memory to peripheral transfer may be visualized as
 * 	Source burst data from memory to DMAC
 *	Until no data left
 *		On burst request from peripheral 
 *			Destination burst from DMAC to peripheral
 *			Clear burst request
 *	Raise terminal count interrupt
 *
 * For peripherals with a FIFO:
 * Source      burst size == half the depth of the peripheral FIFO
 * Destination burst size == width of the peripheral FIFO
 *
 *
 */

/*
 * Header for pl080 drivers
 */


/*
 *  dma_pool defines
 */

#define PL080_MAX_LLIS_SIZE	0x2000	/* Size (bytes) of each buffer allocated */
#define PL080_ALIGN		8	/* Alignment each buffer allocated */
#define PL080_ALLOC		0	/* Boundary not to cross 0 == N/A */
 
#define	PL080_OS_ISR		0x00
#define	PL080_OS_ISR_TC		0x04
#define	PL080_OS_ICLR_TC	0x08
#define	PL080_OS_ISR_ERR	0x0C
#define	PL080_OS_ICLR_ERR	0x10
#define	PL080_OS_ENCHNS		0x1C
#define	PL080_MASK_ENCHNS	0x000000FF
#define	PL080_OS_CFG		0x30
#define	PL080_MASK_CFG		0xFFFFFFF1
#define	PL080_MASK_EN		0x00000001
#define	PL080_OS_CHAN_BASE	0x100
#define	PL080_OS_CHAN		0x20
#define	PL080_OS_CSRC		0x00
#define	PL080_OS_CDST		0x04
#define	PL080_OS_CLLI		0x08
#define	PL080_MASK_CLLI		0x00000002
#define	PL080_OS_CCTL		0x0C
/*
 *	The DMA channel control register entries (see pl080 TRM) have the following units
 *	Width 			- bits
 *	Burst size		- number of transfers
 *	Transfer (packet) size	- number of (destination width) transfers
 *	See e.g include/asm-arm/arch-versatile/hardware.h
 */
#define	PL080_MASK_TSFR_SIZE	0x00000FFF
#define	PL080_OS_CCFG		0x10
#define	PL080_MASK_CEN		0x00000001
#define	PL080_MASK_INTTC	0x00008000
#define	PL080_MASK_INTERR	0x00004000
#define	PL080_MASK_HALT		0x00040000
#define	PL080_MASK_ACTIVE	0x00020000
#define	PL080_MASK_CCFG		0x00000000
/*
 * Flow control bit masks
 */
/*
 * Bit values for  Transfer type Controller
 * [13 - 11]
 */
#define PL080_FCMASK_M2M_DMA	0x00000000	/* Memory-to-memory DMA */
#define PL080_FCMASK_M2P_DMA	0x00000800	/* Memory-to-peripheral DMA */
#define PL080_FCMASK_P2M_DMA	0x00001000	/* Peripheral-to-memory DMA */
#define PL080_FCMASK_P2P_DMA	0x00001800	/* Source peripheral-to-destination peripheral DMA */
#define PL080_FCMASK_P2P_DST	0x00002000	/* Source peripheral-to-destination peripheral Destination peripheral */
#define PL080_FCMASK_M2P_PER	0x00002800	/* Memory-to-peripheral Peripheral */
#define PL080_FCMASK_P2P_PER	0x00003000	/* Peripheral-to-memory Peripheral */
#define PL080_FCMASK_P2P_SRC	0x00003800	/* Source peripheral-to-destination peripheral Source peripheral */

typedef struct _chan_lli {
	dma_addr_t bus_list;	/* the linked lli list bus     address for use in the LLI */
	void	*  va_list;	/* the linked lli list virtual address for use by the driver */
	unsigned int num_entries; /* number of entries - might not be circular */
	unsigned int index_next; /* Index of next lli to load */
} chanlli;

/*
 * An LLI struct - see pl080 TRM
 */
typedef struct _lli{
	dma_addr_t start;
	dma_addr_t dest;
	dma_addr_t next;
	unsigned int cword;
} pl080_lli;

/*
 *  Channel register settings
 */
typedef struct _periph_id_dma_channel_settings {
	unsigned int periphid;
	unsigned int ctl;
	unsigned int cfg;
} setting;

/*
 *	One structure for each DMA channel
 */
extern chanlli * chanllis;

int		pl080_init_dma		(dma_t * dma_chan, struct dma_ops * ops);
void		pl080_reset_cycle	(dmach_t chan_num);
dma_addr_t	pl080_make_llis		(dmach_t chan_num, unsigned int address, unsigned int length, unsigned int packet_size, pl080_lli * setting);
void		pl080_configure_chan	(dmach_t chan_num, struct amba_dma_data * data);
void		pl080_transfer_configure(dmach_t chan_num, pl080_lli * setting, unsigned int ccfg);

#endif	/* AMBA_PL080_H */

