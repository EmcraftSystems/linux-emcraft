/*
 * linux/arch/arm/mach-m2s/include/mach/spi.h
 *
 * Copyright (C) 2012 Yuri Tikhonov, Emcraft Systems
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

#ifndef _MACH_M2S_SPI_H_
#define _MACH_M2S_SPI_H_

extern void __init m2s_spi_init(void);

struct spi_m2s_platform_data {
	unsigned int	ref_clk;        /* Reference clock	*/
	unsigned char	dma_rx;		/* Rx DMA channel	*/
	unsigned char	dma_tx;		/* Tx DMA channel	*/
};

#endif	/*_MACH_M2S_SPI_H_ */
