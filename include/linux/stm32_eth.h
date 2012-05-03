/*
 * (C) Copyright 2011, 2012
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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

#ifndef _STM32_ETH_H_
#define _STM32_ETH_H_

/*
 * STM32 platform Ethernet driver name
 */
#define STM32_ETH_DRV_NAME		"blackfin-eth"

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

#endif /* _STM32_ETH_H_ */
