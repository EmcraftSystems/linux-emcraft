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

#ifndef _MACH_KINETIS_DMAINIT_H_
#define _MACH_KINETIS_DMAINIT_H_

#include <linux/init.h>

/*
 * DMA channels used for particular DMA requests
 */
#define KINETIS_DMACH_UART0_RX	0
#define KINETIS_DMACH_UART1_RX	1
#define KINETIS_DMACH_UART2_RX	2
#define KINETIS_DMACH_UART3_RX	3
#define KINETIS_DMACH_UART4_RX	4
#define KINETIS_DMACH_UART5_RX	5
#define KINETIS_DMACH_ESDHC	6

void __init kinetis_dma_init(void);
void __init kinetis_dmac_init(void);

#endif /* _MACH_KINETIS_DMAINIT_H_ */
