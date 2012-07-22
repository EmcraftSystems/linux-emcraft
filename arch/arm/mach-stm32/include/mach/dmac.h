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
int stm32_dma_ch_init(int ch, u8 dir, u8 fpctrl, u8 pl, u8 dbm, u8 circ);
int stm32_dma_ch_init_fifo(int ch, u8 burst, u8 threshold);
int stm32_dma_ch_set_periph(int ch, u32 addr, u8 inc, u8 bitwidth, u8 burst);
int stm32_dma_ch_set_memory(int ch, u32 addr, u8 inc, u8 bitwidth, u8 burst);
int stm32_dma_ch_set_nitems(int ch, u16 nitems);

#endif /* _MACH_STM32_DMAC_H_ */
