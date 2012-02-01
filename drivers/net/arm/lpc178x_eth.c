/*
 * drivers/net/lpc-eth.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2010 NXP Semiconductors
 *
 * Customized for LPC178x/7x by:
 * (C) Copyright 2011, 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/crc32.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/phy.h>

#include <asm/delay.h>
#include <asm/io.h>

#include <mach/power.h>
#include <mach/eth.h>

/*
 * Ethernet MAC controller Register offsets
 */
#define LPC_ENET_MAC1(x)			(x + 0x000)
#define LPC_ENET_MAC2(x)			(x + 0x004)
#define LPC_ENET_IPGT(x)			(x + 0x008)
#define LPC_ENET_IPGR(x)			(x + 0x00C)
#define LPC_ENET_CLRT(x)			(x + 0x010)
#define LPC_ENET_MAXF(x)			(x + 0x014)
#define LPC_ENET_SUPP(x)			(x + 0x018)
#define LPC_ENET_TEST(x)			(x + 0x01C)
#define LPC_ENET_MCFG(x)			(x + 0x020)
#define LPC_ENET_MCMD(x)			(x + 0x024)
#define LPC_ENET_MADR(x)			(x + 0x028)
#define LPC_ENET_MWTD(x)			(x + 0x02C)
#define LPC_ENET_MRDD(x)			(x + 0x030)
#define LPC_ENET_MIND(x)			(x + 0x034)
#define LPC_ENET_SA0(x)				(x + 0x040)
#define LPC_ENET_SA1(x)				(x + 0x044)
#define LPC_ENET_SA2(x)				(x + 0x048)
#define LPC_ENET_COMMAND(x)			(x + 0x100)
#define LPC_ENET_STATUS(x)			(x + 0x104)
#define LPC_ENET_RXDESCRIPTOR(x)		(x + 0x108)
#define LPC_ENET_RXSTATUS(x)			(x + 0x10C)
#define LPC_ENET_RXDESCRIPTORNUMBER(x)		(x + 0x110)
#define LPC_ENET_RXPRODUCEINDEX(x)		(x + 0x114)
#define LPC_ENET_RXCONSUMEINDEX(x)		(x + 0x118)
#define LPC_ENET_TXDESCRIPTOR(x)		(x + 0x11C)
#define LPC_ENET_TXSTATUS(x)			(x + 0x120)
#define LPC_ENET_TXDESCRIPTORNUMBER(x)		(x + 0x124)
#define LPC_ENET_TXPRODUCEINDEX(x)		(x + 0x128)
#define LPC_ENET_TXCONSUMEINDEX(x)		(x + 0x12C)
#define LPC_ENET_TSV0(x)			(x + 0x158)
#define LPC_ENET_TSV1(x)			(x + 0x15C)
#define LPC_ENET_RSV(x)				(x + 0x160)
#define LPC_ENET_FLOWCONTROLCOUNTER(x)		(x + 0x170)
#define LPC_ENET_FLOWCONTROLSTATUS(x)		(x + 0x174)
#define LPC_ENET_RXFILTER_CTRL(x)		(x + 0x200)
#define LPC_ENET_RXFILTERWOLSTATUS(x)		(x + 0x204)
#define LPC_ENET_RXFILTERWOLCLEAR(x)		(x + 0x208)
#define LPC_ENET_HASHFILTERL(x)			(x + 0x210)
#define LPC_ENET_HASHFILTERH(x)			(x + 0x214)
#define LPC_ENET_INTSTATUS(x)			(x + 0xFE0)
#define LPC_ENET_INTENABLE(x)			(x + 0xFE4)
#define LPC_ENET_INTCLEAR(x)			(x + 0xFE8)
#define LPC_ENET_INTSET(x)			(x + 0xFEC)
#define LPC_ENET_POWERDOWN(x)			(x + 0xFF4)

/*
 * Structure of a TX/RX descriptors and RX status
 */
struct txrx_desc_t {
	volatile u32 packet;
	volatile u32 control;
};
struct rx_status_t {
	volatile u32 statusinfo;
	volatile u32 statushashcrc;
};

/*
 * mac1 register definitions
 */
#define LPC_MAC1_RECV_ENABLE			(1 << 0)
#define LPC_MAC1_PASS_ALL_RX_FRAMES		(1 << 1)
#define LPC_MAC1_RX_FLOW_CONTROL		(1 << 2)
#define LPC_MAC1_TX_FLOW_CONTROL		(1 << 3)
#define LPC_MAC1_LOOPBACK			(1 << 4)
#define LPC_MAC1_RESET_TX			(1 << 8)
#define LPC_MAC1_RESET_MCS_TX			(1 << 9)
#define LPC_MAC1_RESET_RX			(1 << 10)
#define LPC_MAC1_RESET_MCS_RX			(1 << 11)
#define LPC_MAC1_SIMULATION_RESET		(1 << 14)
#define LPC_MAC1_SOFT_RESET			(1 << 15)

/*
 * mac2 register definitions
 */
#define LPC_MAC2_FULL_DUPLEX			(1 << 0)
#define LPC_MAC2_FRAME_LENGTH_CHECKING		(1 << 1)
#define LPC_MAC2_HUGH_LENGTH_CHECKING		(1 << 2)
#define LPC_MAC2_DELAYED_CRC			(1 << 3)
#define LPC_MAC2_CRC_ENABLE			(1 << 4)
#define LPC_MAC2_PAD_CRC_ENABLE			(1 << 5)
#define LPC_MAC2_VLAN_PAD_ENABLE		(1 << 6)
#define LPC_MAC2_AUTO_DETECT_PAD_ENABLE		(1 << 7)
#define LPC_MAC2_PURE_PREAMBLE_ENFORCEMENT	(1 << 8)
#define LPC_MAC2_LONG_PREAMBLE_ENFORCEMENT	(1 << 9)
#define LPC_MAC2_NO_BACKOFF			(1 << 12)
#define LPC_MAC2_BACK_PRESSURE			(1 << 13)
#define LPC_MAC2_EXCESS_DEFER			(1 << 14)

/*
 * ipgt register definitions
 */
#define LPC_IPGT_LOAD(n)			((n) & 0x7F)

/*
 * ipgr register definitions
 */
#define LPC_IPGR_LOAD_PART2(n)			((n) & 0x7F)
#define LPC_IPGR_LOAD_PART1(n)			(((n) & 0x7F) << 8)

/*
 * clrt register definitions
 */
#define LPC_CLRT_LOAD_RETRY_MAX(n)		((n) & 0xF)
#define LPC_CLRT_LOAD_COLLISION_WINDOW(n)	(((n) & 0x3F) << 8)

/*
 * maxf register definitions
 */
#define LPC_MAXF_LOAD_MAX_FRAME_LEN(n)		((n) & 0xFFFF)

/*
 * supp register definitions
 */
#define LPC_SUPP_SPEED				(1 << 8)
#define LPC_SUPP_RESET_RMII			(1 << 11)

/*
 * test register definitions
 */
#define LPC_TEST_SHORTCUT_PAUSE_QUANTA		(1 << 0)
#define LPC_TEST_PAUSE				(1 << 1)
#define LPC_TEST_BACKPRESSURE			(1 << 2)

/*
 * mcfg register definitions
 */
#define LPC_MCFG_SCAN_INCREMENT			(1 << 0)
#define LPC_MCFG_SUPPRESS_PREAMBLE		(1 << 1)
#define LPC_MCFG_CLOCK_SELECT(n)		(((n) & 0x7) << 2)
#define LPC_MCFG_CLOCK_HOST_DIV_4		0
#define LPC_MCFG_CLOCK_HOST_DIV_6		2
#define LPC_MCFG_CLOCK_HOST_DIV_8		3
#define LPC_MCFG_CLOCK_HOST_DIV_10		4
#define LPC_MCFG_CLOCK_HOST_DIV_14		5
#define LPC_MCFG_CLOCK_HOST_DIV_20		6
#define LPC_MCFG_CLOCK_HOST_DIV_28		7
#define LPC_MCFG_RESET_MII_MGMT			(1 << 15)

/*
 * mcmd register definitions
 */
#define LPC_MCMD_READ				(1 << 0)
#define LPC_MCMD_SCAN				(1 << 1)

/*
 * madr register definitions
 */
#define LPC_MADR_REGISTER_ADDRESS(n)		((n) & 0x1F)
#define LPC_MADR_PHY_0ADDRESS(n)		(((n) & 0x1F) << 8)

/*
 * mwtd register definitions
 */
#define LPC_MWDT_WRITE(n)			((n) & 0xFFFF)

/*
 * mrdd register definitions
 */
#define LPC_MRDD_READ_MASK			0xFFFF

/*
 * mind register definitions
 */
#define LPC_MIND_BUSY				(1 << 0)
#define LPC_MIND_SCANNING			(1 << 1)
#define LPC_MIND_NOT_VALID			(1 << 2)
#define LPC_MIND_MII_LINK_FAIL			(1 << 3)

/*
 * command register definitions
 */
#define LPC_COMMAND_RXENABLE			(1 << 0)
#define LPC_COMMAND_TXENABLE			(1 << 1)
#define LPC_COMMAND_REG_RESET			(1 << 3)
#define LPC_COMMAND_TXRESET			(1 << 4)
#define LPC_COMMAND_RXRESET			(1 << 5)
#define LPC_COMMAND_PASSRUNTFRAME		(1 << 6)
#define LPC_COMMAND_PASSRXFILTER		(1 << 7)
#define LPC_COMMAND_TXFLOWCONTROL		(1 << 8)
#define LPC_COMMAND_RMII			(1 << 9)
#define LPC_COMMAND_FULLDUPLEX			(1 << 10)

/*
 * status register definitions
 */
#define LPC_STATUS_RXACTIVE			(1 << 0)
#define LPC_STATUS_TXACTIVE			(1 << 1)

/*
 * tsv0 register definitions
 */
#define LPC_TSV0_CRC_ERROR			(1 << 0)
#define LPC_TSV0_LENGTH_CHECK_ERROR		(1 << 1)
#define LPC_TSV0_LENGTH_OUT_OF_RANGE		(1 << 2)
#define LPC_TSV0_DONE				(1 << 3)
#define LPC_TSV0_MULTICAST			(1 << 4)
#define LPC_TSV0_BROADCAST			(1 << 5)
#define LPC_TSV0_PACKET_DEFER			(1 << 6)
#define LPC_TSV0_ESCESSIVE_DEFER		(1 << 7)
#define LPC_TSV0_ESCESSIVE_COLLISION		(1 << 8)
#define LPC_TSV0_LATE_COLLISION			(1 << 9)
#define LPC_TSV0_GIANT				(1 << 10)
#define LPC_TSV0_UNDERRUN			(1 << 11)
#define LPC_TSV0_TOTAL_BYTES(n)			(((n) >> 12) & 0xFFFF)
#define LPC_TSV0_CONTROL_FRAME			(1 << 28)
#define LPC_TSV0_PAUSE				(1 << 29)
#define LPC_TSV0_BACKPRESSURE			(1 << 30)
#define LPC_TSV0_VLAN				(1 << 31)

/*
 * tsv1 register definitions
 */
#define LPC_TSV1_TRANSMIT_BYTE_COUNT(n)		((n) & 0xFFFF)
#define LPC_TSV1_COLLISION_COUNT(n)		(((n) >> 16) & 0xF)

/*
 * rsv register definitions
 */
#define LPC_RSV_RECEIVED_BYTE_COUNT(n)		((n) & 0xFFFF)
#define LPC_RSV_RXDV_EVENT_IGNORED		(1 << 16)
#define LPC_RSV_RXDV_EVENT_PREVIOUSLY_SEEN	(1 << 17)
#define LPC_RSV_CARRIER_EVNT_PREVIOUS_SEEN	(1 << 18)
#define LPC_RSV_RECEIVE_CODE_VIOLATION		(1 << 19)
#define LPC_RSV_CRC_ERROR			(1 << 20)
#define LPC_RSV_LENGTH_CHECK_ERROR		(1 << 21)
#define LPC_RSV_LENGTH_OUT_OF_RANGE		(1 << 22)
#define LPC_RSV_RECEIVE_OK			(1 << 23)
#define LPC_RSV_MULTICAST			(1 << 24)
#define LPC_RSV_BROADCAST			(1 << 25)
#define LPC_RSV_DRIBBLE_NIBBLE			(1 << 26)
#define LPC_RSV_CONTROL_FRAME			(1 << 27)
#define LPC_RSV_PAUSE				(1 << 28)
#define LPC_RSV_UNSUPPORTED_OPCODE		(1 << 29)
#define LPC_RSV_VLAN				(1 << 30)

/*
 * flowcontrolcounter register definitions
 */
#define LPC_FCCR_MIRRORCOUNTER(n)		((n) & 0xFFFF)
#define LPC_FCCR_PAUSETIMER(n)			(((n) >> 16) & 0xFFFF)

/*
 * flowcontrolstatus register definitions
 */
#define LPC_FCCR_MIRRORCOUNTERCURRENT(n)	((n) & 0xFFFF)

/*
 * rxfliterctrl, rxfilterwolstatus, and rxfilterwolclear shared
 * register definitions
 */
#define LPC_RXFLTRW_ACCEPTUNICAST		(1 << 0)
#define LPC_RXFLTRW_ACCEPTUBROADCAST		(1 << 1)
#define LPC_RXFLTRW_ACCEPTUMULTICAST		(1 << 2)
#define LPC_RXFLTRW_ACCEPTUNICASTHASH		(1 << 3)
#define LPC_RXFLTRW_ACCEPTUMULTICASTHASH	(1 << 4)
#define LPC_RXFLTRW_ACCEPTPERFECT		(1 << 5)

/*
 * rxfliterctrl register definitions
 */
#define LPC_RXFLTRWSTS_MAGICPACKETENWOL		(1 << 12)
#define LPC_RXFLTRWSTS_RXFILTERENWOL		(1 << 13)

/*
 * rxfilterwolstatus/rxfilterwolclear register definitions
 */
#define LPC_RXFLTRWSTS_RXFILTERWOL		(1 << 7)
#define LPC_RXFLTRWSTS_MAGICPACKETWOL		(1 << 8)

/*
 * intstatus, intenable, intclear, and Intset shared register
 * definitions
 */
#define LPC_MACINT_RXOVERRUNINTEN		(1 << 0)
#define LPC_MACINT_RXERRORONINT			(1 << 1)
#define LPC_MACINT_RXFINISHEDINTEN		(1 << 2)
#define LPC_MACINT_RXDONEINTEN			(1 << 3)
#define LPC_MACINT_TXUNDERRUNINTEN		(1 << 4)
#define LPC_MACINT_TXERRORINTEN			(1 << 5)
#define LPC_MACINT_TXFINISHEDINTEN		(1 << 6)
#define LPC_MACINT_TXDONEINTEN			(1 << 7)
#define LPC_MACINT_SOFTINTEN			(1 << 12)
#define LPC_MACINT_WAKEUPINTEN			(1 << 13)

/*
 * powerdown register definitions
 */
#define LPC_POWERDOWN_MACAHB			(1 << 31)

#define MODNAME "lpc-net"
#define DRV_VERSION "$Revision: 1.00 $"

#define ENET_MAXF_SIZE 1536	/* up to 2k */
#define ENET_RX_DESC 48
#define ENET_TX_DESC 16

/*
 * FIXME
 * Dynamic buffer allocation as needed
 * Check/fix ethtool support
 * Better MAC address support
 * Better DMA allocation support (dma pool)
 * MII/RMII support (only supports RMII as of now)
 */

static int lpc_net_hard_start_xmit(struct sk_buff *skb,
	struct net_device *ndev);

/*
 * Transmit timeout, default 2.5 seconds.
 */
static int watchdog = 2500;
module_param(watchdog, int, 0400);
MODULE_PARM_DESC(watchdog, "transmit timeout in milliseconds");

/*
 * Default local config if board config is not defined
 */
static struct lpc178x_eth_data __lpc_local_net_config = {
	.phy_irq = -1,
	.phy_mask = 0xFFFFFFF0,
};

/*
 * Device driver data structure
 */
struct netdata_local {
	struct platform_device	*pdev;
	struct net_device	*ndev;
	spinlock_t		lock;
	void __iomem		*net_base;
	unsigned long		net_region_start;
	unsigned long		net_region_size;
	u32			msg_enable;
	struct sk_buff		*skb[ENET_TX_DESC];
	unsigned int		last_tx_idx;
	unsigned int		num_used_tx_buffs;
	struct mii_bus		*mii_bus;
	struct phy_device	*phy_dev;
	struct clk		*clk;
	u32			dma_buff_base_p;
	u32			dma_buff_base_v;
	u32			dma_buff_size;
	u32			tx_desc_v [ENET_TX_DESC];
	u32			tx_stat_v [ENET_TX_DESC];
	u32			tx_buff_v [ENET_TX_DESC];
	u32			rx_desc_v [ENET_RX_DESC];
	u32			rx_stat_v [ENET_RX_DESC];
	u32			rx_buff_v [ENET_RX_DESC];
	struct lpc178x_eth_data	*ncfg;
	int			link;
	int			speed;
	int			duplex;
};

/*
 * MAC address is provided as a boot paramter (ethaddr) via u-boot
 */
static u8 mac_address[6] = {0};

static int __init ethaddr(char *str)
{
        char *s, *e;
        int i;

        s = str;
        for (i = 0; i < 6; ++i) {
                mac_address[i] = s ? simple_strtoul (s, &e, 16) : 0;
                if (s)
                        s = (*e) ? e + 1 : e;
        }
        return 1;
}
__setup("ethaddr=", ethaddr);

static int get_mac_addr(u8 *mac)
{
        int i;

        for (i = 0; i < 6; i++) {
                mac[i] = mac_address[i];
        }
        return 0;
}

/*
 * MAC support functions
 */
static void __lpc_set_mac(struct netdata_local *pldat, u8 *mac)
{
	u32 tmp;

	/* Set station address */
	tmp = (u32) mac[0] | ((u32) mac[1] << 8);
	writel(tmp, LPC_ENET_SA2(pldat->net_base));
	tmp = (u32) mac[2] | ((u32) mac[3] << 8);
	writel(tmp, LPC_ENET_SA1(pldat->net_base));
	tmp = (u32) mac[4] | ((u32) mac[5] << 8);
	writel(tmp, LPC_ENET_SA0(pldat->net_base));

	pr_debug("Ethernet MAC address %02x:%02x:%02x:%02x:%02x:%02x\n",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void __lpc_net_clock_enable(struct netdata_local *pldat,
	int enable)
{
	if (enable)
		clk_enable(pldat->clk);
	else
		clk_disable(pldat->clk);
}

static void __lpc_params_setup(struct netdata_local *pldat)
{
	u32 tmp;

	if (pldat->duplex == DUPLEX_FULL) {
		tmp = readl(LPC_ENET_MAC2(pldat->net_base));
		tmp |= LPC_MAC2_FULL_DUPLEX;
		writel(tmp, LPC_ENET_MAC2(pldat->net_base));
		tmp = readl(LPC_ENET_COMMAND(pldat->net_base));
		tmp |= LPC_COMMAND_FULLDUPLEX;
		writel(tmp, LPC_ENET_COMMAND(pldat->net_base));
		writel(LPC_IPGT_LOAD(0x15), LPC_ENET_IPGT(pldat->net_base));
	} else {
		tmp = readl(LPC_ENET_MAC2(pldat->net_base));
		tmp &= ~LPC_MAC2_FULL_DUPLEX;
		writel(tmp, LPC_ENET_MAC2(pldat->net_base));
		tmp = readl(LPC_ENET_COMMAND(pldat->net_base));
		tmp &= ~LPC_COMMAND_FULLDUPLEX;
		writel(tmp, LPC_ENET_COMMAND(pldat->net_base));
		writel(LPC_IPGT_LOAD(0x12), LPC_ENET_IPGT(pldat->net_base));
	}

	if (pldat->speed == SPEED_100)
		writel(LPC_SUPP_SPEED, LPC_ENET_SUPP(pldat->net_base));
	else
		writel(0, LPC_ENET_SUPP(pldat->net_base));
}

static void __lpc_eth_reset(struct netdata_local *pldat)
{
	/* Reset all MAC logic */
	writel((LPC_MAC1_RESET_TX | LPC_MAC1_RESET_MCS_TX | LPC_MAC1_RESET_RX |
		LPC_MAC1_RESET_MCS_RX | LPC_MAC1_SIMULATION_RESET |
		LPC_MAC1_SOFT_RESET), LPC_ENET_MAC1(pldat->net_base));
	writel((LPC_COMMAND_REG_RESET | LPC_COMMAND_TXRESET |
		LPC_COMMAND_RXRESET), LPC_ENET_COMMAND(pldat->net_base));
}

static int __lpc_mii_mngt_reset(struct netdata_local *pldat)
{
	/* Reset MII management hardware */
	writel(LPC_MCFG_RESET_MII_MGMT, LPC_ENET_MCFG(pldat->net_base));

	/* Setup MII clock to slowest rate with a /28 divider */
	writel(LPC_MCFG_CLOCK_SELECT(LPC_MCFG_CLOCK_HOST_DIV_28),
		LPC_ENET_MCFG(pldat->net_base));

	return 0;
}

static u32 __ptr_align(u32 pbuff)
{
	pbuff &= 0xFFFFFFF0;
	pbuff += 0x10;

	return pbuff;
}

/* Setup TX/RX descriptors */
static void __lpc_txrx_desc_setup(struct netdata_local *pldat)
{
	u32 tbuff, *ptxstat;
	int i;
	struct txrx_desc_t *ptxrxdesc;
	struct rx_status_t *prxstat;

	tbuff = __ptr_align(pldat->dma_buff_base_v);

	/* Setup TX descriptors, status, and buffers */
	for (i = 0; i < ENET_TX_DESC; i++) {
		pldat->tx_desc_v [i] = tbuff;
		tbuff += sizeof(struct txrx_desc_t);
	}
	for (i = 0; i < ENET_TX_DESC; i++) {
		pldat->tx_stat_v [i] = tbuff;
		tbuff += sizeof(u32);
	}
	tbuff = __ptr_align(tbuff);
	for (i = 0; i < ENET_TX_DESC; i++) {
		pldat->tx_buff_v [i] = tbuff;
		tbuff += ENET_MAXF_SIZE;
	}

	/* Setup RX descriptors, status, and buffers */
	for (i = 0; i < ENET_RX_DESC; i++) {
		pldat->rx_desc_v [i] = tbuff;
		tbuff += sizeof(struct txrx_desc_t);
	}
	tbuff = __ptr_align(tbuff);
	for (i = 0; i < ENET_RX_DESC; i++) {
		pldat->rx_stat_v [i] = tbuff;
		tbuff += sizeof(struct rx_status_t);
	}
	tbuff = __ptr_align(tbuff);
	for (i = 0; i < ENET_RX_DESC; i++) {
		pldat->rx_buff_v [i] = tbuff;
		tbuff += ENET_MAXF_SIZE;
	}

	/* Map the TX descriptors to the TX buffers in hardware */
	for (i = 0; i < ENET_TX_DESC; i++) {
		ptxstat = (u32 *) pldat->tx_stat_v [i];
		ptxrxdesc = (struct txrx_desc_t *) pldat->tx_desc_v [i];

		ptxrxdesc->packet = pldat->tx_buff_v [i];
		ptxrxdesc->control = 0;
		*ptxstat = 0;
	}

	/* Map the RX descriptors to the RX buffers in hardware */
	for (i = 0; i < ENET_RX_DESC; i++) {
		prxstat = (struct rx_status_t *) pldat->rx_stat_v [i];
		ptxrxdesc = (struct txrx_desc_t *) pldat->rx_desc_v [i];

		ptxrxdesc->packet = pldat->rx_buff_v [i];
		ptxrxdesc->control = 0x80000000 | (ENET_MAXF_SIZE - 1);
		prxstat->statusinfo = 0;
		prxstat->statushashcrc = 0;
	}

	/* Setup base addresses in hardware to point to buffers and
	   descriptors */
	writel((ENET_TX_DESC - 1),
		LPC_ENET_TXDESCRIPTORNUMBER(pldat->net_base));
	writel(pldat->tx_desc_v [0], LPC_ENET_TXDESCRIPTOR(pldat->net_base));
	writel(pldat->tx_stat_v [0], LPC_ENET_TXSTATUS(pldat->net_base));
	writel((ENET_RX_DESC - 1),
		LPC_ENET_RXDESCRIPTORNUMBER(pldat->net_base));
	writel(pldat->rx_desc_v [0], LPC_ENET_RXDESCRIPTOR(pldat->net_base));
	writel(pldat->rx_stat_v [0], LPC_ENET_RXSTATUS(pldat->net_base));
}

static void __lpc_eth_init(struct netdata_local *pldat)
{
	u32 tmp;

	/* Disable controller and reset */
	tmp = readl(LPC_ENET_COMMAND(pldat->net_base));
	tmp &= ~LPC_COMMAND_RXENABLE | LPC_COMMAND_TXENABLE;
	writel(tmp, LPC_ENET_COMMAND(pldat->net_base));
	tmp = readl(LPC_ENET_MAC1(pldat->net_base));
	tmp &= ~LPC_MAC1_RECV_ENABLE;
	writel(tmp, LPC_ENET_MAC1(pldat->net_base));

	/* Initial MAC setup */
	writel(LPC_MAC1_PASS_ALL_RX_FRAMES, LPC_ENET_MAC1(pldat->net_base));
	writel((LPC_MAC2_PAD_CRC_ENABLE | LPC_MAC2_CRC_ENABLE),
		LPC_ENET_MAC2(pldat->net_base));
	writel(ENET_MAXF_SIZE, LPC_ENET_MAXF(pldat->net_base));

	/* Collision window, gap */
	writel((LPC_CLRT_LOAD_RETRY_MAX(0xF) |
		LPC_CLRT_LOAD_COLLISION_WINDOW(0x37)),
		LPC_ENET_CLRT(pldat->net_base));
	writel(LPC_IPGR_LOAD_PART2(0x12), LPC_ENET_IPGR(pldat->net_base));

#if defined (CONFIG_ARCH_LPC32XX_MII_SUPPORT)
	writel(LPC_COMMAND_PASSRUNTFRAME, LPC_ENET_COMMAND(pldat->net_base));
#else
	writel((LPC_COMMAND_PASSRUNTFRAME | LPC_COMMAND_RMII),
		LPC_ENET_COMMAND(pldat->net_base));
	writel(LPC_SUPP_RESET_RMII, LPC_ENET_SUPP(pldat->net_base));
#endif

	__lpc_params_setup(pldat);

	/* Setup TX and RX descriptors */
	__lpc_txrx_desc_setup(pldat);

	/* Setup packet filtering */
	writel((LPC_RXFLTRW_ACCEPTUBROADCAST | LPC_RXFLTRW_ACCEPTPERFECT),
		LPC_ENET_RXFILTER_CTRL(pldat->net_base));

	/* Clear and enable interrupts */
	writel(0xFFFF, LPC_ENET_INTCLEAR(pldat->net_base));
	writel((LPC_MACINT_RXDONEINTEN | LPC_MACINT_TXDONEINTEN),
		LPC_ENET_INTENABLE(pldat->net_base));

	/* Get the next TX buffer output index */
	pldat->num_used_tx_buffs = 0;
	pldat->last_tx_idx =
		readl(LPC_ENET_TXCONSUMEINDEX(pldat->net_base));

	/* Enable controller */
	tmp = readl(LPC_ENET_COMMAND(pldat->net_base));
	tmp |= LPC_COMMAND_RXENABLE | LPC_COMMAND_TXENABLE;
	writel(tmp, LPC_ENET_COMMAND(pldat->net_base));
	tmp = readl(LPC_ENET_MAC1(pldat->net_base));
	tmp |= LPC_MAC1_RECV_ENABLE;
	writel(tmp, LPC_ENET_MAC1(pldat->net_base));
}

static void __lpc_net_shutdown(struct netdata_local *pldat)
{
	/* Reset ethernet and power down PHY */
	__lpc_eth_reset(pldat);
	writel(0, LPC_ENET_MAC1(pldat->net_base));
	writel(0, LPC_ENET_MAC2(pldat->net_base));
}

/*
 * MAC<--->PHY support functions
 */
static int lpc_mdio_read(struct mii_bus *bus, int phy_id, int phyreg)
{
	struct netdata_local *pldat = bus->priv;
	unsigned long timeout = jiffies + ((HZ * 100) / 1000); /* 100mS */
	int lps;

	writel(((phy_id << 8) | phyreg), LPC_ENET_MADR(pldat->net_base));
	writel(LPC_MCMD_READ, LPC_ENET_MCMD(pldat->net_base));

	/* Wait for unbusy status */
	while (readl(LPC_ENET_MIND(pldat->net_base)) & LPC_MIND_BUSY) {
		if (jiffies > timeout)
			return -EIO;
		cpu_relax();
	}

	lps = (int) readl(LPC_ENET_MRDD(pldat->net_base));
	writel(0, LPC_ENET_MCMD(pldat->net_base));

	return lps;
}

static int lpc_mdio_write(struct mii_bus *bus, int phy_id, int phyreg,
			u16 phydata)
{
	struct netdata_local *pldat = bus->priv;
	unsigned long timeout = jiffies + ((HZ * 100) / 1000); /* 100mS */

	writel(((phy_id << 8) | phyreg), LPC_ENET_MADR(pldat->net_base));
	writel(phydata, LPC_ENET_MWTD(pldat->net_base));

	/* Wait for completion */
	while (readl(LPC_ENET_MIND(pldat->net_base)) & LPC_MIND_BUSY) {
		if (jiffies > timeout)
			return -EIO;
		cpu_relax();
	}

	return 0;
}

static int lpc_mdio_reset(struct mii_bus *bus)
{
	return __lpc_mii_mngt_reset((struct netdata_local *) bus->priv);
}

static void lpc_handle_link_change(struct net_device *ndev)
{
	struct netdata_local *pldat = netdev_priv(ndev);
	struct phy_device *phydev = pldat->phy_dev;
	unsigned long flags;

	int status_change = 0;

	spin_lock_irqsave(&pldat->lock, flags);

	if (phydev->link) {
		if ((pldat->speed != phydev->speed) ||
		    (pldat->duplex != phydev->duplex)) {
			pldat->speed = phydev->speed;
			pldat->duplex = phydev->duplex;
			status_change = 1;
		}
	}

	if (phydev->link != pldat->link) {
		if (!phydev->link) {
			pldat->speed = 0;
			pldat->duplex = -1;
		}
		pldat->link = phydev->link;

		status_change = 1;
	}

	spin_unlock_irqrestore(&pldat->lock, flags);

	if (status_change)
		__lpc_params_setup(pldat);
}

static int lpc_mii_probe(struct net_device *ndev)
{
	struct netdata_local *pldat = netdev_priv(ndev);
	struct phy_device *phydev = NULL;
	int phy_addr;

	/* find the first phy */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (pldat->mii_bus->phy_map[phy_addr]) {
			phydev = pldat->mii_bus->phy_map[phy_addr];
			break;
		}
	}

	if (!phydev) {
		pr_err("%s: no PHY found\n", ndev->name);
		return -ENODEV;
	}

	/* Attach to the PHY */
#if defined (CONFIG_ARCH_LPC32XX_MII_SUPPORT)
	pr_info("%s: using MII interface\n", ndev->name);
	phydev = phy_connect(ndev, dev_name(&phydev->dev),
		&lpc_handle_link_change, 0, PHY_INTERFACE_MODE_MII);
#else
	pr_info("%s: using RMII interface\n", ndev->name);
	phydev = phy_connect(ndev, dev_name(&phydev->dev),
		&lpc_handle_link_change, 0, PHY_INTERFACE_MODE_RMII);
#endif

	if (IS_ERR(phydev)) {
		pr_err("%s: Could not attach to PHY\n", ndev->name);
		return PTR_ERR(phydev);
	}

	/* mask with MAC supported features */
	phydev->supported &= PHY_BASIC_FEATURES;

	phydev->advertising = phydev->supported;

	pldat->link = 0;
	pldat->speed = 0;
	pldat->duplex = -1;
	pldat->phy_dev = phydev;

	return 0;
}

static int lpc_mii_init(struct netdata_local *pldat)
{
	int err = -ENXIO, i;

	pldat->mii_bus = mdiobus_alloc();
	if (!pldat->mii_bus) {
		err = -ENOMEM;
		goto err_out;
	}

	/* Setup MII mode */
#if defined (CONFIG_ARCH_LPC32XX_MII_SUPPORT)
	writel(LPC_COMMAND_PASSRUNTFRAME, LPC_ENET_COMMAND(pldat->net_base));
#else
	writel((LPC_COMMAND_PASSRUNTFRAME | LPC_COMMAND_RMII),
		LPC_ENET_COMMAND(pldat->net_base));
	writel(LPC_SUPP_RESET_RMII, LPC_ENET_SUPP(pldat->net_base));
#endif

	pldat->mii_bus->name = "lpc_mii_bus";
	pldat->mii_bus->read = &lpc_mdio_read;
	pldat->mii_bus->write = &lpc_mdio_write;
	pldat->mii_bus->reset = &lpc_mdio_reset;
	snprintf(pldat->mii_bus->id, MII_BUS_ID_SIZE, "%x", pldat->pdev->id);
	pldat->mii_bus->priv = pldat;
	pldat->mii_bus->parent = &pldat->pdev->dev;
	pldat->mii_bus->phy_mask = 0xFFFFFFF0;

	if (pldat->ncfg)
		pldat->mii_bus->phy_mask = pldat->ncfg->phy_mask;

	pldat->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!pldat->mii_bus->irq) {
		err = -ENOMEM;
		goto err_out_1;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		pldat->mii_bus->irq[i] = PHY_POLL;

	//platform_set_drvdata(pldat->ndev, pldat->mii_bus);

	if (mdiobus_register(pldat->mii_bus)) {
		goto err_out_free_mdio_irq;
	}

	if (lpc_mii_probe(pldat->ndev) != 0) {
		goto err_out_unregister_bus;
	}

	return 0;

err_out_unregister_bus:
	mdiobus_unregister(pldat->mii_bus);
err_out_free_mdio_irq:
	kfree(pldat->mii_bus->irq);
err_out_1:
	mdiobus_free(pldat->mii_bus);
err_out:
	return err;
}

static void __lpc_handle_xmit(struct net_device *ndev)
{
	struct netdata_local *pldat = netdev_priv(ndev);
	struct sk_buff *skb;
	unsigned int txcidx, *ptxstat, txstat;

	txcidx = readl(LPC_ENET_TXCONSUMEINDEX(pldat->net_base));
	while (pldat->last_tx_idx != txcidx)
	{
		skb = (struct sk_buff *) pldat->skb[pldat->last_tx_idx];

		/* A buffer is available, get buffer status */
		ptxstat = (unsigned int *) pldat->tx_stat_v[pldat->last_tx_idx];
		txstat = *ptxstat;

		/* Next buffer and decrement used buffer counter */
		pldat->num_used_tx_buffs--;
		pldat->last_tx_idx++;
		if (pldat->last_tx_idx >= ENET_TX_DESC)
			pldat->last_tx_idx = 0;

		/* Update collision counter */
		ndev->stats.collisions += ((txstat >> 21) & 0xF);

		/* Any errors occurred? */
		if (txstat & 0x80000000) {
			if (txstat & 0x20000000) {
				/* FIFO underrun */
				ndev->stats.tx_fifo_errors++;
				ndev->stats.tx_errors++;
			}
			if (txstat & 0x10000000) {
				/* Late collision */
				ndev->stats.tx_aborted_errors++;
				ndev->stats.tx_errors++;
			}
			if (txstat & 0x08000000) {
				/* Excessive collision */
				ndev->stats.tx_aborted_errors++;
				ndev->stats.tx_errors++;
			}
			if (txstat & 0x04000000) {
				/* Defer limit */
				ndev->stats.tx_aborted_errors++;
				ndev->stats.tx_errors++;
			}

			/* Buffer transmit failed, requeue it */
			lpc_net_hard_start_xmit(skb, ndev);
		} else {
			/* Update stats */
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += skb->len;

			/* Free buffer */
			dev_kfree_skb_irq(skb);
		}

		txcidx = readl(LPC_ENET_TXCONSUMEINDEX(pldat->net_base));
	}

	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

static void __lpc_handle_recv(struct net_device *ndev)
{
	struct netdata_local *pldat = netdev_priv(ndev);
	struct sk_buff *skb;
	int rxconsidx, len, ethst;
	struct rx_status_t *prxstat;
	u8 *prdbuf;

	/* Get the current RX buffer indexes */
	rxconsidx = (int) readl(LPC_ENET_RXCONSUMEINDEX(pldat->net_base));
	while (rxconsidx != (int) readl(LPC_ENET_RXPRODUCEINDEX(pldat->net_base)))
	{
		/* Get pointer to receive status */
		prxstat = (struct rx_status_t *) pldat->rx_stat_v [rxconsidx];
		len = (prxstat->statusinfo & 0x7FF) + 1;

		/* Status error? */
		ethst = prxstat->statusinfo;
		if ((ethst & 0xBF800000) == 0x84000000)
			ethst &= ~0x80000000;

		if (ethst & 0x80000000) {
			/* Check statuses */
			if (prxstat->statusinfo & (1 << 28)) {
				/* Overrun error */
				ndev->stats.rx_fifo_errors++;
			} else if (prxstat->statusinfo & (1 << 23)) {
				/* CRC error */
				ndev->stats.rx_crc_errors++;
			} else if (prxstat->statusinfo & (1 << 25)) {
				/* Length error */
				ndev->stats.rx_length_errors++;
			} else if (prxstat->statusinfo & 0x80000000) {
				/* Other error */
				ndev->stats.rx_length_errors++;
			}
			ndev->stats.rx_errors++;
		} else {
			/* Packet is good */
			skb = dev_alloc_skb(len + NET_IP_ALIGN);
			if (!skb)
				ndev->stats.rx_dropped++;
			else {
				/*
				 * We cannot access unaligned 32-bit data in
				 * SDRAM, therefore we have to align the IP
				 * header. `skb_reserve()` is used here to
				 * perform this alignment.
				 *
				 * See also:
				 * Documentation/unaligned-memory-access.txt
				 */
				skb_reserve(skb, NET_IP_ALIGN);
				prdbuf = skb_put(skb, (len - 0));

				/* Copy packer from buffer */
				memcpy(prdbuf, (void *) pldat->rx_buff_v [rxconsidx], len);

				/* Pass to upper layer */
				skb->protocol = eth_type_trans(skb, ndev);
				netif_rx(skb);
				ndev->last_rx = jiffies;
				ndev->stats.rx_packets++;
				ndev->stats.rx_bytes += len;
			}
		}

		/* Increment consume index */
		rxconsidx = rxconsidx + 1;
		if (rxconsidx >= ENET_RX_DESC)
			rxconsidx = 0;
		writel((u32) rxconsidx, LPC_ENET_RXCONSUMEINDEX(pldat->net_base));
	}
}

static irqreturn_t __lpc_eth_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct netdata_local *pldat = netdev_priv(ndev);
	u32 tmp;
	unsigned long flags;

	//spin_lock(&pldat->lock);
	spin_lock_irqsave(&pldat->lock, flags);

	/* Get the interrupt status */
	tmp = readl(LPC_ENET_INTSTATUS(pldat->net_base));

	while (tmp) {
		/* Clear interrupts */
		writel(tmp, LPC_ENET_INTCLEAR(pldat->net_base));

		/* Transmit complete? */
		if (tmp & (LPC_MACINT_TXUNDERRUNINTEN | LPC_MACINT_TXERRORINTEN |
			LPC_MACINT_TXFINISHEDINTEN | LPC_MACINT_TXDONEINTEN))
			__lpc_handle_xmit(ndev);

		/* Receive buffer available */
		if (tmp & (LPC_MACINT_RXOVERRUNINTEN | LPC_MACINT_RXERRORONINT |
			LPC_MACINT_RXFINISHEDINTEN | LPC_MACINT_RXDONEINTEN))
			__lpc_handle_recv(ndev);

		/* Recheck the interrupt status */
		tmp = readl(LPC_ENET_INTSTATUS(pldat->net_base));
	}

	//spin_unlock(&pldat->lock);
	spin_unlock_irqrestore(&pldat->lock, flags);

	return IRQ_HANDLED;
}

static int lpc_net_close(struct net_device *ndev)
{
	unsigned long flags;
	struct netdata_local *pldat = netdev_priv(ndev);

	if (netif_msg_ifdown(pldat))
	{
		dev_dbg(&pldat->pdev->dev, "shutting down %s\n", ndev->name);
	}

	netif_stop_queue(ndev);

	if (pldat->phy_dev)
	{
		phy_stop(pldat->phy_dev);
	}

	spin_lock_irqsave(&pldat->lock, flags);
	__lpc_eth_reset(pldat);
	netif_carrier_off(ndev);
	writel(0, LPC_ENET_MAC1(pldat->net_base));
	writel(0, LPC_ENET_MAC2(pldat->net_base));
	spin_unlock_irqrestore(&pldat->lock, flags);

	__lpc_net_clock_enable(pldat, 0);

	return 0;
}

static int lpc_net_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct netdata_local *pldat = netdev_priv(ndev);
	unsigned int len, txidx;
	u32 *ptxstat;
	struct txrx_desc_t *ptxrxdesc;

	len = skb->len;

	spin_lock_irq(&pldat->lock);

	if (pldat->num_used_tx_buffs >= (ENET_TX_DESC - 1)) {
		/* This function should never be called when there are no
		   buffers, log the error */
		netif_stop_queue(ndev);
		spin_unlock_irq(&pldat->lock);
		dev_err(&pldat->pdev->dev,
			"BUG! TX request when no free TX buffers!\n");
		return 1;
	}

	/* Get the next TX descriptor index */
	txidx = readl(LPC_ENET_TXPRODUCEINDEX(pldat->net_base));

	/* Setup control for the transfer */
	ptxstat = (u32 *) pldat->tx_stat_v [txidx];
	*ptxstat = 0;
	ptxrxdesc = (struct txrx_desc_t *) pldat->tx_desc_v [txidx];
	ptxrxdesc->control = (len - 1) | 0xC0000000;

	/* Copy data to the DMA buffer */
	memcpy((void *) pldat->tx_buff_v [txidx], skb->data, len);

	/* Save the buffer and increment the buffer counter */
	pldat->skb[txidx] = skb;
	pldat->num_used_tx_buffs++;

	/* Start transmit */
	txidx++;
	if (txidx >= ENET_TX_DESC)
		txidx = 0;
	writel((u32) txidx, LPC_ENET_TXPRODUCEINDEX(pldat->net_base));

	/* Stop queue if no more TX buffers */
	if (pldat->num_used_tx_buffs >= (ENET_TX_DESC - 1))
		netif_stop_queue(ndev);

	spin_unlock_irq(&pldat->lock);
	ndev->trans_start = jiffies;

	return 0;
}

static void lpc_net_timeout(struct net_device *ndev)
{
	struct netdata_local *pldat = netdev_priv(ndev);

	/* This should never happen and indicates a problem */
	dev_err(&pldat->pdev->dev, "BUG! TX timeout occurred!\n");
}

static int lpc_set_mac_address(struct net_device *ndev, void *p)
{
        struct sockaddr *addr = p;
	struct netdata_local *pldat = netdev_priv(ndev);
	unsigned long flags;

	if (!is_valid_ether_addr(addr->sa_data))
                return -EADDRNOTAVAIL;
        memcpy(ndev->dev_addr, addr->sa_data, ETH_ALEN);

	spin_lock_irqsave(&pldat->lock, flags);

	/* Set station address */
	__lpc_set_mac(pldat, ndev->dev_addr);

	spin_unlock_irqrestore(&pldat->lock, flags);

	return 0;
}

static void lpc_net_set_multicast_list(struct net_device *ndev)
{
	struct netdata_local *pldat = netdev_priv(ndev);
	struct dev_mc_list *mclist;
	int i;
	u32 tmp32, hash_val, hashlo, hashhi;
	unsigned long flags;

	spin_lock_irqsave(&pldat->lock, flags);

	/* Set station address */
	__lpc_set_mac(pldat, ndev->dev_addr);

	tmp32 =  LPC_RXFLTRW_ACCEPTUBROADCAST | LPC_RXFLTRW_ACCEPTPERFECT;

	if (ndev->flags & IFF_PROMISC)
		tmp32 |= LPC_RXFLTRW_ACCEPTUNICAST | LPC_RXFLTRW_ACCEPTUMULTICAST;
	if (ndev->flags & IFF_ALLMULTI)
		tmp32 |= LPC_RXFLTRW_ACCEPTUMULTICAST;

	if(ndev->mc_count != 0)
		tmp32 |= LPC_RXFLTRW_ACCEPTUMULTICASTHASH;

	writel(tmp32, LPC_ENET_RXFILTER_CTRL(pldat->net_base));


	/* Set initial hash table */
	hashlo = 0x0;
	hashhi = 0x0;

	/* 64 bits : multicast address in hash table */
	for (i = 0, mclist = ndev->mc_list; mclist && i < ndev->mc_count;
		i++, mclist = mclist->next) {
		hash_val = (ether_crc(6, mclist->dmi_addr) >> 23) & 0x3F;

		if (hash_val >= 32)
			hashhi |= 1 << (hash_val - 32);
		else
			hashlo |= 1 << hash_val;
	}

	writel(hashlo, LPC_ENET_HASHFILTERL(pldat->net_base));
	writel(hashhi, LPC_ENET_HASHFILTERH(pldat->net_base));

	spin_unlock_irqrestore(&pldat->lock, flags);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void lpc_net_poll_controller(struct net_device *ndev)
{
	disable_irq(ndev->irq);
	__lpc_eth_interrupt(dev->irq, ndev);
	enable_irq(ndev->irq);
}
#endif

static int lpc_net_ioctl(struct net_device *ndev, struct ifreq *req, int cmd)
{
	struct netdata_local *pldat = netdev_priv(ndev);
	struct phy_device *phydev = pldat->phy_dev;

	if (!netif_running(ndev))
	{
		return -EINVAL;
	}

	if (!phydev)
	{
		return -ENODEV;
	}

	return phy_mii_ioctl(phydev, if_mii(req), cmd);
}

static int lpc_net_open(struct net_device *ndev)
{
	struct netdata_local *pldat = netdev_priv(ndev);

	/* if the phy is not yet registered, retry later*/
	if (!pldat->phy_dev)
	{
		return -EAGAIN;
	}

	if (netif_msg_ifup(pldat))
	{
		dev_dbg(&pldat->pdev->dev, "enabling %s\n", ndev->name);
	}

	if (!is_valid_ether_addr(ndev->dev_addr))
	{
		return -EADDRNOTAVAIL;
	}

	__lpc_net_clock_enable(pldat, 1);

	/* Reset and initialize */
	__lpc_eth_reset(pldat);
	__lpc_eth_init(pldat);

	/* schedule a link state check */
	phy_start(pldat->phy_dev);
	netif_start_queue(ndev);

	return 0;
}

/*
 * Ethtool ops
 */
static void lpc_net_ethtool_getdrvinfo(struct net_device *ndev,
	struct ethtool_drvinfo *info)
{
	strcpy(info->driver, MODNAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->bus_info, dev_name(ndev->dev.parent));
}

static u32 lpc_net_ethtool_getmsglevel(struct net_device *ndev)
{
	struct netdata_local *pldat = netdev_priv(ndev);

	return pldat->msg_enable;
}

static void lpc_net_ethtool_setmsglevel(struct net_device *ndev, u32 level)
{
	struct netdata_local *pldat = netdev_priv(ndev);

	pldat->msg_enable = level;
}

static int lpc_net_ethtool_getsettings(struct net_device *ndev,
	struct ethtool_cmd *cmd)
{
	struct netdata_local *pldat = netdev_priv(ndev);
	struct phy_device *phydev = pldat->phy_dev;

	if (!phydev)
	{
		return -ENODEV;
	}

	return phy_ethtool_gset(phydev, cmd);
}

static int lpc_net_ethtool_setsettings(struct net_device *ndev,
	struct ethtool_cmd *cmd)
{
	struct netdata_local *pldat = netdev_priv(ndev);
	struct phy_device *phydev = pldat->phy_dev;

	if (!phydev)
	{
		return -ENODEV;
	}

	return phy_ethtool_sset(phydev, cmd);
}

static const struct ethtool_ops lpc_net_ethtool_ops = {
	.get_drvinfo	= lpc_net_ethtool_getdrvinfo,
	.get_settings	= lpc_net_ethtool_getsettings,
	.set_settings	= lpc_net_ethtool_setsettings,
	.get_msglevel	= lpc_net_ethtool_getmsglevel,
	.set_msglevel	= lpc_net_ethtool_setmsglevel,
	.get_link	= ethtool_op_get_link,
};

static const struct net_device_ops lpc_netdev_ops = {
	.ndo_open		= lpc_net_open,
	.ndo_stop		= lpc_net_close,
	.ndo_start_xmit		= lpc_net_hard_start_xmit,
	.ndo_set_multicast_list	= lpc_net_set_multicast_list,
//	.ndo_get_stats		= tsi108_get_stats,
	.ndo_do_ioctl		= lpc_net_ioctl,
	.ndo_tx_timeout		= lpc_net_timeout,
	.ndo_set_mac_address	= lpc_set_mac_address,
//	.ndo_validate_addr	= eth_validate_addr,
//	.ndo_change_mtu		= eth_change_mtu,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.poll_controller	= lpc_net_poll_controller;
#endif
};

/*
 * Pointer to the private data for the only Ethernet block in our SoC
 *
 * Is is only used in `lpc178x_phy_final_reset()`.
 */
static struct netdata_local *lpc178x_pldat;

static int lpc_net_drv_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct net_device *ndev;
	struct netdata_local *pldat;
	struct phy_device *phydev;
	dma_addr_t dma_handle;
	int irq, ret;

	/* Get platform resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if ((!res) || (irq < 0) || (irq >= NR_IRQS)) {
		dev_err(&pdev->dev, "error getting resources.\n");
		ret = -ENXIO;
		goto err_exit;
	}

	/* Allocate net driver data structure */
	ndev = alloc_etherdev(sizeof(struct netdata_local));
	if (!ndev) {
		dev_err(&pdev->dev, "could not allocate device.\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	pldat = netdev_priv(ndev);
	lpc178x_pldat = pldat;
	pldat->pdev = pdev;
	pldat->ndev = ndev;

	spin_lock_init(&pldat->lock);

	/* Save resources */
	pldat->net_region_start = res->start;
	pldat->net_region_size = res->end - res->start + 1;
	ndev->irq = irq;

	/* Get clock for the device */
	pldat->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(pldat->clk)) {
		dev_err(&pdev->dev, "error getting clock.\n");
		ret = PTR_ERR(pldat->clk);
		goto err_out_free_dev;
	}

	/* Enable network clock */
	__lpc_net_clock_enable(pldat, 1);

	/* Map IO space */
	pldat->net_base = ioremap(pldat->net_region_start, pldat->net_region_size);
	if (!pldat->net_base) {
		dev_err(&pdev->dev, "failed to map registers\n");
		ret = -ENOMEM;
		goto err_out_disable_clocks;
	}
	ret = request_irq(ndev->irq, __lpc_eth_interrupt, 0,
			  ndev->name, ndev);
	if (ret) {
		dev_err(&pdev->dev, "error requesting interrupt.\n");
		goto err_out_iounmap;
	}

	/* Fill in the fields of the device structure with ethernet values. */
	ether_setup(ndev);

	/* Setup driver functions */
	ndev->netdev_ops = &lpc_netdev_ops;
	ndev->ethtool_ops = &lpc_net_ethtool_ops;
	ndev->base_addr = pldat->net_region_start;
	ndev->watchdog_timeo = msecs_to_jiffies(watchdog);

	/* Save board specific configuration */
	pldat->ncfg = (struct lpc178x_eth_data *) pdev->dev.platform_data;
	if (pldat->ncfg == NULL) {
		dev_err(&pdev->dev, "error requesting interrupt.\n");
		pldat->ncfg = &__lpc_local_net_config;
	}

	/* Get size of DMA buffers/descriptors region */
	pldat->dma_buff_size = (ENET_TX_DESC + ENET_RX_DESC) * (ENET_MAXF_SIZE +
		sizeof(struct txrx_desc_t) + sizeof(struct rx_status_t));
	pldat->dma_buff_base_v = 0;

	pldat->dma_buff_size += 4096; /* Allows room for alignment */

	/* Align on the next highest page entry size */
	pldat->dma_buff_size &= 0Xfffff000;
	pldat->dma_buff_size += 0X00001000;

	/* Allocate a chunk of memory for the DMA ethernet buffers and descriptors */
	pldat->dma_buff_base_v = (u32) dma_alloc_coherent(&pldat->pdev->dev,
		pldat->dma_buff_size, &dma_handle, GFP_KERNEL);

	if (pldat->dma_buff_base_v == (u32) NULL)
	{
		dev_err(&pdev->dev, "error getting DMA region.\n");
		ret = -ENOMEM;
		goto err_out_free_irq;
	}

	pldat->dma_buff_base_p = (u32) dma_handle;

	pr_debug("IO address start     :0x%08x\n", (u32) pldat->net_region_start);
	pr_debug("IO address size      :%d\n", (u32) pldat->net_region_size);
	pr_debug("IO address (mapped)  :0x%08x\n", (u32) pldat->net_base);
	pr_debug("IRQ number           :%d\n", ndev->irq);
	pr_debug("DMA buffer size      :%d\n", pldat->dma_buff_size);
	pr_debug("DMA buffer P address :0x%08x\n", pldat->dma_buff_base_p);
	pr_debug("DMA buffer V address :0x%08x\n", pldat->dma_buff_base_v);

	/* Get the board MAC address */
	ret = get_mac_addr(ndev->dev_addr);
	if (ret) {
		/* Mac address load error */
		goto err_out_dma_unmap;
	}
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		pr_info("%s: ethernet address is not set or invalid, "
		       "using random.\n", __func__);
		random_ether_addr(ndev->dev_addr);
	}

	/* Reset the ethernet controller */
	__lpc_eth_reset(pldat);

	/* then shut everything down to save power */
	__lpc_net_shutdown(pldat);

	/* Set default parameters */
	pldat->msg_enable = NETIF_MSG_LINK;

	/* Force an MII interface reset and clock setup */
	__lpc_mii_mngt_reset(pldat);

	/* Force default PHY interface setup in chip, this will probably be
	   changed by the PHY driver */
	pldat->link = 0;
	pldat->speed = 100;
	pldat->duplex = DUPLEX_FULL;
	__lpc_params_setup(pldat);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot register net device, aborting.\n");
		goto err_out_dma_unmap;
	}
	platform_set_drvdata(pdev, ndev);

	if (lpc_mii_init(pldat) != 0) {
		goto err_out_unregister_netdev;
	}

	pr_info("%s: LPC mac at 0x%08lx irq %d\n",
	       ndev->name, ndev->base_addr, ndev->irq);

	phydev = pldat->phy_dev;
	pr_info("%s: attached PHY driver [%s] "
		"(mii_bus:phy_addr=%s, irq=%d)\n",
		ndev->name, phydev->drv->name, dev_name(&phydev->dev),
		phydev->irq);

	device_init_wakeup(&pdev->dev, 1);
	device_set_wakeup_enable(&pdev->dev, 0);

	return 0;

err_out_unregister_netdev:
	platform_set_drvdata(pdev, NULL);
	unregister_netdev(ndev);
err_out_dma_unmap:
	dma_free_coherent(&pldat->pdev->dev, pldat->dma_buff_size,
		(void *) pldat->dma_buff_base_v, (dma_addr_t) pldat->dma_buff_base_p);
err_out_free_irq:
	free_irq(ndev->irq, ndev);
err_out_iounmap:
	iounmap(pldat->net_base);
err_out_disable_clocks:
	clk_disable(pldat->clk);
	clk_put(pldat->clk);
err_out_free_dev:
	free_netdev(ndev);
err_exit:
	pr_err("%s: not found (%d).\n", MODNAME, ret);
	return ret;
}

#if defined(MODULE)
static int lpc_net_drv_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct netdata_local *pldat = netdev_priv(ndev);

	/*
	 * `lpc178x_pldat` is not valid anymore
	 */
	lpc178x_pldat = NULL;

	unregister_netdev(ndev);
	platform_set_drvdata(pdev, NULL);

	dma_free_coherent(&pldat->pdev->dev, pldat->dma_buff_size,
		(void *) pldat->dma_buff_base_v,
		(dma_addr_t) pldat->dma_buff_base_p);
	free_irq(ndev->irq, ndev);
	iounmap(pldat->net_base);
	clk_disable(pldat->clk);
	clk_put(pldat->clk);
	free_netdev(ndev);

	return 0;
}
#endif

static int lpc_net_drv_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct netdata_local *pldat = netdev_priv(ndev);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(ndev->irq);

	if (ndev) {
		if (netif_running(ndev)) {
			netif_device_detach(ndev);
			__lpc_net_shutdown(pldat);
			clk_disable(pldat->clk);

			/*
			 * Reset again now clock is disable to be sure
			 * EMC_MDC is down
			 */
			__lpc_eth_reset(pldat);
		}
	}

	return 0;
}

static int lpc_net_drv_resume(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct netdata_local *pldat;

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(ndev->irq);

	if (ndev) {
		if (netif_running(ndev)) {
			pldat = netdev_priv(ndev);

			/* Enable interface clock */
			clk_enable(pldat->clk);

			/* Reset and initialize */
			__lpc_eth_reset(pldat);
			__lpc_eth_init(pldat);

			netif_device_attach(ndev);
		}
	}

	return 0;
}

static struct platform_driver lpc_net_driver = {
	.probe		= lpc_net_drv_probe,
	.remove		= __devexit_p(lpc_net_drv_remove),
	.suspend	= lpc_net_drv_suspend,
	.resume		= lpc_net_drv_resume,
	.driver		= {
		.name	= MODNAME,
	},
};

static int __init lpc_net_init(void)
{
	return platform_driver_register(&lpc_net_driver);
}

static void __exit lpc_net_cleanup(void)
{
	platform_driver_unregister(&lpc_net_driver);
}

/*
 * Final PHY reset before performing SYSRESET of SoC
 *
 * We have to reset the PHY immediately before doing a software SoC
 * reset, because otherwise the Ethernet block of the SoC will hang
 * after reset.
 */
void lpc178x_phy_final_reset(void)
{
	/*
	 * Enable power on the Ethernet block
	 */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCENET_MSK, 1);

	/*
	 * Minimal MAC initialization
	 */
	__lpc_mii_mngt_reset(lpc178x_pldat);
	writel(0, LPC_ENET_MAC1(lpc178x_pldat->net_base));
	writel(0, LPC_ENET_MAC2(lpc178x_pldat->net_base));

	/*
	 * Reset PHY
	 */
	phy_write(lpc178x_pldat->phy_dev, MII_BMCR, BMCR_RESET);

	/*
	 * Disable power on the Ethernet block
	 */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCENET_MSK, 0);
}

module_init(lpc_net_init);
module_exit(lpc_net_cleanup);

MODULE_AUTHOR("Alexander Potashev <aspotashev@emcraft.com>");
MODULE_DESCRIPTION("LPC178x/7x Ethernet Driver");
MODULE_LICENSE("GPL");

