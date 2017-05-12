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

#ifndef _MACH_STM32_DMAC_H_
#define _MACH_STM32_DMAC_H_

#include <linux/types.h>

/*
 * Flags to pass as "flags" argument to stm32_dma_ch_request_irq().
 * Combinations of these flags are allowed.
 */
#define STM32_DMA_INTCOMPLETE		(1 << 0)
#define STM32_DMA_INTHALF		(1 << 1)

enum stm32_dma_direction {
	DMA_DIR_DEV_TO_MEM=0,
	DMA_DIR_MEM_TO_DEV,
	DMA_DIR_MEM_TO_MEM
};

enum stm32_dma_flow_control {
	DMA_FLOW_CONTROL_DISABLE=0,
	DMA_FLOW_CONTROL_ENABLE
};

enum stm32_dma_priority {
	DMA_PRIORITY_LOW=0,
	DMA_PRIORITY_MEDIUM,
	DMA_PRIORITY_HIGH,
	DMA_PRIORITY_VERY_HIGH
};

enum stm32_dma_double_buffer {
	DMA_DOUBLE_BUFFER_DISABLE=0,
	DMA_DOUBLE_BUFFER_ENABLE
};

enum stm32_dma_circular_mode {
	DMA_CIRCULAR_MODE_DISABLE=0,
	DMA_CIRCULAR_MODE_ENABLE
};

enum stm32_dma_mode {
	DMA_DIRECT_MODE_ENABLE=0,
	DMA_DIRECT_MODE_DISABLE
};

enum stm32_dma_threshold {
	DMA_THRESH_1_4_FIFO=0,
	DMA_THRESH_HALF_FIFO,
	DMA_THRESH_3_4_FIFO,
	DMA_THRESH_FULL_FIFO
};

enum stm32_dma_increment {
	DMA_INCREMENT_DISABLE=0,
	DMA_INCREMENT_ENABLE
};

enum stm32_dma_bitwidth {
	DMA_BITWIDTH_8=0,
	DMA_BITWIDTH_16,
	DMA_BITWIDTH_32
};

enum stm32_dma_burst {
	DMA_BURST_SINGLE=0,
	DMA_BURST_4,
	DMA_BURST_8,
	DMA_BURST_16
};

/*
 * API functions of the STM32 DMA controller driver
 *
 * See arch/arm/mach-stm32/dmac.c for details on each of these functions.
 */
int stm32_dma_ch_get(int ch);
int stm32_dma_ch_put(int ch);
int stm32_dma_ch_enable(int ch);
int stm32_dma_ch_disable(int ch);
int stm32_dma_ch_request_irq(
	int ch, void (*handler)(int ch, unsigned long flags, void *data),
	unsigned long flags, void *data);
int stm32_dma_ch_free_irq(int ch, void *data);
int stm32_dma_ch_init(int ch,
		      enum stm32_dma_direction dir,
		      enum stm32_dma_flow_control fpctrl,
		      enum stm32_dma_priority pl,
		      enum stm32_dma_double_buffer dbm,
		      enum stm32_dma_circular_mode circ);
int stm32_dma_ch_init_fifo(int ch,
			   enum stm32_dma_mode burst,
			   enum stm32_dma_threshold threshold);
int stm32_dma_ch_set_periph(int ch, u32 addr,
			    enum stm32_dma_increment inc,
			    enum stm32_dma_bitwidth bitwidth,
			    enum stm32_dma_burst burst);
int stm32_dma_ch_set_memory(int ch, u32 addr,
			    enum stm32_dma_increment inc,
			    enum stm32_dma_bitwidth bitwidth,
			    enum stm32_dma_burst burst);
int stm32_dma_ch_set_nitems(int ch, u16 nitems);

#endif /* _MACH_STM32_DMAC_H_ */
