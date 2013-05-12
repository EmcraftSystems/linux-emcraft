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

#ifndef _MACH_KINETIS_DMAC_H_
#define _MACH_KINETIS_DMAC_H_

#include <linux/types.h>

/* Number of DMA channels */
#define KINETIS_DMA_CH_NUM		32

/*
 * DMA data widths, for writing to TCD_ATTR[SSIZE,DSIZE] or
 * for passing to kinetis_dma_ch_set_src() and kinetis_dma_ch_set_dest()
 * in the "bitwidth" argument.
 */
#define KINETIS_DMA_WIDTH_8BIT		0
#define KINETIS_DMA_WIDTH_16BIT		1
#define KINETIS_DMA_WIDTH_32BIT		2
#define KINETIS_DMA_WIDTH_16BYTE	4

/*
 * Flags to pass as "flags" argument to kinetis_dma_ch_request_irq().
 * Combinations of these flags are allowed.
 */
#define KINETIS_DMA_INTMAJOR		(1 << 0)
#define KINETIS_DMA_INTHALF		(1 << 1)

/*
 * API functions of the Kinetis eDMA controller driver
 *
 * See arch/arm/mach-kinetis/dmac.c for details on each of these functions.
 */
int kinetis_dma_ch_get(int ch);
int kinetis_dma_ch_put(int ch);
int kinetis_dma_ch_enable(int ch, int single);
int kinetis_dma_ch_disable(int ch);
int kinetis_dma_ch_request_irq(
	int ch, void (*handler)(int ch, unsigned long flags, void *data),
	unsigned long flags, void *data);
int kinetis_dma_ch_free_irq(int ch, void *data);
int kinetis_dma_ch_init(int ch);
int kinetis_dma_ch_set_src(
	int ch, u32 saddr, s16 soff, u8 bitwidth, s32 slast);
int kinetis_dma_ch_set_dest(
	int ch, u32 daddr, s16 doff, u8 bitwidth, s32 dlast);
int kinetis_dma_ch_set_nbytes(int ch, u32 nbytes);
int kinetis_dma_ch_set_iter_num(int ch, u16 iter);
int kinetis_dma_ch_iter_done(int ch);
int kinetis_dma_ch_is_active(int ch);

#endif /* _MACH_KINETIS_DMAC_H_ */
