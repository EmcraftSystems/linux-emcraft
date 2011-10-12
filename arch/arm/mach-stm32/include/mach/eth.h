/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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

#ifndef _MACH_STM32_ETH_H_
#define _MACH_STM32_ETH_H_

#include <mach/stm32.h>

/*
 * STM32 platform Ethernet driver name
 */
#define STM32_ETH_DRV_NAME		"stm32-eth"

/*
 * STM32 MAC register base
 */
#define STM32_MAC_BASE			(STM32_AHB1PERITH_BASE + 0x8000)

/*
 * MAC, MMC, PTP, DMA register map
 */
struct stm32_mac_regs {
	u32	maccr;		/* MAC configuration			      */
	u32	macffr;		/* MAC frame filter			      */
	u32	machthr;	/* MAC hash table high			      */
	u32	machtlr;	/* MAC hash table low			      */
	u32	macmiiar;	/* MAC MII address			      */
	u32	macmiidr;	/* MAC MII data				      */
	u32	macfcr;		/* MAC flow control			      */
	u32	macvlantr;	/* MAC VLAN tag				      */
	u32	rsv0[2];
	u32	macrwuffr;	/* MAC remote wakeup frame filter	      */
	u32	macpmtcsr;	/* MAC PMT control and status		      */
	u32	rsv1;
	u32	macdbgr;	/* MAC debug				      */
	u32	macsr;		/* MAC interrupt status			      */
	u32	macimr;		/* MAC interrupt mask			      */
	u32	maca0hr;	/* MAC address 0 high			      */
	u32	maca0lr;	/* MAC address 0 low			      */
	u32	maca1hr;	/* MAC address 1 high			      */
	u32	maca1lr;	/* MAC address 1 low			      */
	u32	maca2hr;	/* MAC address 2 high			      */
	u32	maca2lr;	/* MAC address 2 low			      */
	u32	maca3hr;	/* MAC address 3 high			      */
	u32	maca3lr;	/* MAC address 3 low			      */
	u32	rsv2[40];
	u32	mmccr;		/* MMC control				      */
	u32	mmcrir;		/* MMC receive interrupt		      */
	u32	mmctir;		/* MMC transmit interrupt		      */
	u32	mmcrimr;	/* MMC receive interrupt mask		      */
	u32	mmctimr;	/* MMC transmit interrupt mask		      */
	u32	rsv3[14];
	u32	mmctgfsccr;	/* MMC transmitted good frms after single col */
	u32	mmctgfmsccr;	/* MMC transmitted good frms after more col   */
	u32	rsv4[5];
	u32	mmctgfcr;	/* MMC transmitted good frames counter	      */
	u32	rsv5[10];
	u32	mmcrfcecr;	/* MMC received frames with CRC error counter */
	u32	mmcrfaecr;	/* MMC received frames with alignment error   */
	u32	rsv6[10];
	u32	mmcrgufcr;	/* MMC received good unicast frames counter   */
	u32	rsv7[334];
	u32	ptptscr;	/* PTP time stamp control		      */
	u32	ptpssir;	/* PTP subsecond increment		      */
	u32	ptptshr;	/* PTP time stamp high			      */
	u32	ptptslr;	/* PTP time stamp low			      */
	u32	ptptshur;	/* PTP time stamp high update		      */
	u32	ptptslur;	/* PTP time stamp low update		      */
	u32	ptptsar;	/* PTP time stamp addend		      */
	u32	ptptthr;	/* PTP target time high			      */
	u32	ptpttlr;	/* PTP target time low			      */
	u32	rsv8;
	u32	ptptssr;	/* PTP time stamp status		      */
	u32	ptpppscr;	/* PTP PPS control			      */
	u32	rsv9[564];
	u32	dmabmr;		/* DMA bus mode				      */
	u32	dmatpdr;	/* DMA transmit poll demand		      */
	u32	dmarpdr;	/* DMA receive poll demand		      */
	u32	dmardlar;	/* DMA receive descriptor list address	      */
	u32	dmatdlar;	/* DMA transmit descriptor list address	      */
	u32	dmasr;		/* DMA status				      */
	u32	dmaomr;		/* DMA operation mode			      */
	u32	dmaier;		/* DMA interrupt enable			      */
	u32	dmamfbocr;	/* DMA missed frame and buffer overflow	      */
	u32	dmarswtr;	/* DMA receive status watchdog timer	      */
	u32	rsv10[8];
	u32	dmachtdr;	/* DMA current host transmit descriptor	      */
	u32	dmachrdr;	/* DMA current host receive descriptor	      */
	u32	dmachtbar;	/* DMA current host transmit buffer address   */
	u32	dmachrbar;	/* DMA current host receive buffer address    */
};

/*
 * Ethernet platform data
 */
struct stm32_eth_data {
	unsigned int	frame_max_size;	/* Max eth frame size (up to 0x3FFC)  */
	unsigned int	rx_buf_num;	/* Max frames num simulateously rxed  */
	unsigned int	tx_buf_num;	/* Max frames num simulateously txed  */
	unsigned char	mac_addr[6];	/* MAC address to use by default      */
	unsigned char	phy_id;		/* PHY address (identifier)	      */
};

void __init stm32_eth_init(void);

#endif /* _MACH_STM32_ETH_H_ */
