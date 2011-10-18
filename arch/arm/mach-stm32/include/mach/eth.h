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
