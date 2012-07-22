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

#ifndef _MACH_STM32_DMAINIT_H_
#define _MACH_STM32_DMAINIT_H_

#include <linux/init.h>

/*
 * DMA channels ("streams" in terms of STM32) used for particular DMA requests
 */
#ifndef CONFIG_ARCH_STM32F1
/*
 * STM32F2
 */
/* SDIO: Use one channel for both Tx and Rx - DMA2, stream3 */
#define STM32F2_DMACH_SDIO	11
#define DMA_CH_SDCARD_TX	STM32F2_DMACH_SDIO
#define DMA_CH_SDCARD_RX	STM32F2_DMACH_SDIO
#endif

void __init stm32_dma_init(void);
void __init stm32_dmac_init(void);

#endif /* _MACH_STM32_DMAINIT_H_ */
