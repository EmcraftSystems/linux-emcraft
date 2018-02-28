/*
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

#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

#include <asm/setup.h>
#include <mach/eth.h>

/*
 * Configs
 */
#define M2S_RX_NUM		CONFIG_M2S_ETH_RX_NUM
#define M2S_TX_NUM		CONFIG_M2S_ETH_TX_NUM
#if defined(CONFIG_M2S_ETH_BUF_IN_ESRAM)
# define M2S_ESRAM		(0x20000000 + CONFIG_M2S_ETH_BUF_IN_ESRAM_BASE)
#else
# undef M2S_ESRAM
#endif

/*
 * Driver name string
 */
#define M2S_MAC_NAME		"m2s_eth"

/*
 * Maximum frame size
 */
#define M2S_MAC_FRM_SIZE	0x600		/* 1536 bytes		      */

/*
 * Debug control
 */
#undef DEBUG

/*
 * Different timeouts, in msec
 */
#define M2S_FIFO_TOUT		100		/* FIFO initialization	      */
#define M2S_MII_TOUT		250		/* MII read/write cycles      */

/*
 * CFG1 register fields
 */
#define M2S_MAC_CFG1_RST	(1 << 31)	/* PE-MCXMAC full reset	      */
#define M2S_MAC_CFG1_RX_ENA	(1 << 2)	/* MAC receive enable	      */
#define M2S_MAC_CFG1_TX_ENA	(1 << 0)	/* MAC transmit enable	      */

/*
 * CFG2 register fields
 */
#define M2S_MAC_CFG2_PREAM_LEN_BIT	12
#define M2S_MAC_CFG2_PREAM_LEN_MSK	0xf
#define M2S_MAC_CFG2_MODE_BIT		8	/* MAC interface mode	      */
#define M2S_MAC_CFG2_MODE_MSK		0x3
#define M2S_MAC_CFG2_MODE_BYTE		0x2	/* Byte mode		      */
#define M2S_MAC_CFG2_MODE_MII		0x1	/* Nibble mode		      */
#define M2S_MAC_CFG2_HUGE_FRAME_EN	(1 << 5)
#define M2S_MAC_CFG2_LEN_CHECK		(1 << 4)
#define M2S_MAC_CFG2_PAD_CRC		(1 << 2) /* PAD&CRC appending enable   */
#define M2S_MAC_CFG2_CRC_EN		(1 << 1)
#define M2S_MAC_CFG2_FULL_DUP		(1 << 0) /* PE-MCXMAC Full duplex      */

/*
 * MII_COMMAND register fields
 */
#define M2S_MAC_MII_CMD_READ	(1 << 0)	/* Do single Read cycle	      */

/*
 * MII_ADDRESS register fields
 */
#define M2S_MAC_MII_ADR_PHY_BIT	8		/* 5-bit PHY address	      */
#define M2S_MAC_MII_ADR_REG_BIT	0		/* 5-bit register address     */

/*
 * MII_INDICATORS register fields
 */
#define M2S_MAC_MII_IND_NVAL	(1 << 2)	/* Read Data not yet validated*/
#define M2S_MAC_MII_IND_BUSY	(1 << 0)	/* MII is performing cycle    */

/*
 * DMA_RX_CTRL/DMA_TX_CTRL register fields
 */
#define M2S_MAC_DMA_CTRL_ENA	(1 << 0)	/* Enable Tx/Rx DMA xfers     */

/*
 * DMA_RX_STAT/DMA_TX_STAT register fields
 */
#define M2S_MAC_DMA_STAT_RX_BE	(1 << 3)	/* Bus error		      */
#define M2S_MAC_DMA_STAT_RX_OF	(1 << 2)	/* Overflow		      */
#define M2S_MAC_DMA_STAT_RX_PR	(1 << 0)	/* Packet received	      */
#define M2S_MAC_DMA_STAT_TX_BE	(1 << 3)	/* Bus error		      */
#define M2S_MAC_DMA_STAT_TX_UN	(1 << 1)	/* Underrun		      */
#define M2S_MAC_DMA_STAT_TX_PS	(1 << 0)	/* Packet sent		      */
#define M2S_MAC_DMA_STAT_RX	(M2S_MAC_DMA_STAT_RX_BE|M2S_MAC_DMA_STAT_RX_OF|\
				 M2S_MAC_DMA_STAT_RX_PR)
#define M2S_MAC_DMA_STAT_TX	(M2S_MAC_DMA_STAT_TX_BE|M2S_MAC_DMA_STAT_RX_PS)

/*
 * DMA_IRQ register fields
 */
#define M2S_MAC_DMA_IRQ_RBUS	(1 << 7)	/* Rx Bus Error		      */
#define M2S_MAC_DMA_IRQ_ROVF	(1 << 6)	/* Rx Overflow		      */
#define M2S_MAC_DMA_IRQ_RPKT	(1 << 4)	/* Rx Packet Received	      */
#define M2S_MAC_DMA_IRQ_TBUS	(1 << 3)	/* Tx Bus Error		      */
#define M2S_MAC_DMA_IRQ_TPKT	(1 << 0)	/* Tx Packet Sent	      */
#define M2S_MAC_DMA_IRQ_MSK	(M2S_MAC_DMA_IRQ_RBUS | M2S_MAC_DMA_IRQ_ROVF | \
				 M2S_MAC_DMA_IRQ_RPKT |			       \
				 M2S_MAC_DMA_IRQ_TBUS | M2S_MAC_DMA_IRQ_TPKT)
#define M2S_MAC_DMA_IRQ_XMIT	(M2S_MAC_DMA_IRQ_RPKT | M2S_MAC_DMA_IRQ_TPKT | \
				 M2S_MAC_DMA_IRQ_ROVF)
#define M2S_MAC_DMA_IRQ_ERR	(M2S_MAC_DMA_IRQ_RBUS | M2S_MAC_DMA_IRQ_TBUS)

/*
 * Interface Control register fields
 */
#define M2S_MAC_INTF_RESET		(1 << 31) /* Reset interface module */
#define M2S_MAC_INTF_SPEED_100		(1 << 16)  /* MII PHY speed 100Mbit */

/*
 * FIFO_CFG0 register fields
 */
#define M2S_MAC_FIFO_CFG0_FTFENRPLY	(1 << 20) /* Fabric tx iface ena ack  */
#define M2S_MAC_FIFO_CFG0_STFENRPLY	(1 << 19) /* PE-MCXMAC tx iface	      */
#define M2S_MAC_FIFO_CFG0_FRFENRPLY	(1 << 18) /* Fabric rx iface	      */
#define M2S_MAC_FIFO_CFG0_SRFENRPLY	(1 << 17) /* PE-MCXMAC rx iface	      */
#define M2S_MAC_FIFO_CFG0_WTMENRPLY	(1 << 16) /* PE-MCXMAC watermark modu */

/*
 * Note, PE-MCXMAC rx becomes 'enabled' only after smth received from line,
 * so don't care about this at initialization: miss M2S_MAC_FIFO_CFG0_SRFENRPLY
 * in the following mask
 */
#define M2S_MAC_FIFO_CFG0_ALL_RPLY	(M2S_MAC_FIFO_CFG0_FTFENRPLY      | \
		M2S_MAC_FIFO_CFG0_STFENRPLY | M2S_MAC_FIFO_CFG0_FRFENRPLY | \
		M2S_MAC_FIFO_CFG0_WTMENRPLY)

#define M2S_MAC_FIFO_CFG0_FTFENREQ	(1 << 12) /* Fabric tx iface ena req  */
#define M2S_MAC_FIFO_CFG0_STFENREQ	(1 << 11) /* PE-MCXMAC tx iface	      */
#define M2S_MAC_FIFO_CFG0_FRFENREQ	(1 << 10) /* Fabric rx iface	      */
#define M2S_MAC_FIFO_CFG0_SRFENREQ	(1 << 9)  /* PE-MCXMAC rx iface	      */
#define M2S_MAC_FIFO_CFG0_WTMENREQ	(1 << 8)  /* PE-MCXMAC watermark modu */
#define M2S_MAC_FIFO_CFG0_ALL_REQ	(M2S_MAC_FIFO_CFG0_FTFENREQ     | \
		M2S_MAC_FIFO_CFG0_STFENREQ | M2S_MAC_FIFO_CFG0_FRFENREQ | \
		M2S_MAC_FIFO_CFG0_SRFENREQ | M2S_MAC_FIFO_CFG0_WTMENREQ)

#define M2S_MAC_FIFO_CFG0_HSTRSTFT	(1 << 4)  /* Fabric tx iface reset    */
#define M2S_MAC_FIFO_CFG0_HSTRSTST	(1 << 3)  /* PE-MCXMAC tx iface	      */
#define M2S_MAC_FIFO_CFG0_HSTRSTFR	(1 << 2)  /* Fabric rx iface	      */
#define M2S_MAC_FIFO_CFG0_HSTRSTSR	(1 << 1)  /* PE-MCXMAC rx iface	      */
#define M2S_MAC_FIFO_CFG0_HSTRSTWT	(1 << 0)  /* PE-MCXMAC watermark modu */
#define M2S_MAC_FIFO_CFG0_ALL_RST	(M2S_MAC_FIFO_CFG0_HSTRSTFT     | \
		M2S_MAC_FIFO_CFG0_HSTRSTST | M2S_MAC_FIFO_CFG0_HSTRSTFR | \
		M2S_MAC_FIFO_CFG0_HSTRSTSR | M2S_MAC_FIFO_CFG0_HSTRSTWT)

/*
 * FIFO_CFG5 register fields
 */
#define M2S_MAC_FIFO_CFG5_CFGHDPLX	(1 << 22) /* Half-duplex flow ctrl */

/*
 * MAC Configuration Register in Sysreg block fields
 */
#define M2S_SYS_MAC_CR_PM_BIT	2		/* PHY mode		      */
#define M2S_SYS_MAC_CR_PM_MSK	0x7
#define M2S_SYS_MAC_CR_PM_TBI	0x2		/* Use TBI mode		      */
#define M2S_SYS_MAC_CR_PM_MII	0x3		/* Use MII mode		      */
#define M2S_SYS_MAC_CR_LS_BIT	0		/* Line speed		      */
#define M2S_SYS_MAC_CR_LS_MSK	0x3
#define M2S_SYS_MAC_CR_LS_10	0x0		/* 10 Mbps		      */
#define M2S_SYS_MAC_CR_LS_100	0x1		/* 100 Mbps		      */
#define M2S_SYS_MAC_CR_LS_1000	0x2		/* 1000 Mbps		      */

/*
 * Software Reset Control Register fields
 */
#define M2S_SYS_SOFT_RST_CR_MAC	(1 << 4)	/* MAC_SOFTRESET	      */

/*
 * BD constants
 */
#define M2S_BD_EMPTY		(1 << 31)	/* Empty flag		      */
#define M2S_BD_SIZE_MSK		0xFFF		/* Frame size mask	      */

/*
 * Register base offsets
 */
#define M2S_MAC_OFS		0x000		/* MAC register block base    */
#define M2S_DMA_OFS		0x180		/* DMA register block base    */

/*
 * MAC register access macros
 */
#define M2S_MAC_CFG(d)	((volatile struct m2s_mac_cfg_regs *)d->reg[0])
#define M2S_MAC_DMA(d)	((volatile struct m2s_mac_dma_regs *)d->reg[1])

#ifdef DEBUG
#define debug(fmt,args...)	printk(fmt ,##args)
#else
#define debug(fmt,args...)
#endif

/*
 * M2S MAC Device descriptor
 */
struct m2s_mac_dev {
	void __iomem			*reg[2];	/* MAC registers      */

	struct platform_device		*ptf_dev;	/* Platform device    */
	struct net_device		*net_dev;
	struct napi_struct		napi;

	struct phy_device		*phy_dev;
	struct mii_bus			*mii_bus;

	struct {
		volatile struct m2s_mac_dma_bd	*bd;	/* RxBD ring CPU adr  */
		dma_addr_t			bd_adr;	/* RxBD ring DMA adr  */
		u32				idx;	/* RxBD id	      */

		char				*buf[M2S_RX_NUM];
	} rx;

	struct {
		volatile struct m2s_mac_dma_bd	*bd;	/* TxBD ring CPU adr  */
		dma_addr_t			bd_adr;	/* TxBD ring DMA adr  */
		u32				idx_nxt;/* Next TxBD id	      */
		u32				idx_cur;/* Sent TxBD id	      */

		char				*buf[M2S_TX_NUM];
		struct sk_buff			*skb[M2S_TX_NUM];
	} tx;

	spinlock_t			lock;		/* Exclusive access   */

	u32				freq_src;	/* Source freq	      */
	u32				freq_mdc;	/* MDC freq	      */
	u32				flags;		/* Various flags      */

	u32				link;		/* Link status	      */
	u32				speed;		/* Speed value	      */
	u32				duplex;		/* Duplex mode	      */

	u8				phy_id;		/* Phy address	      */
};

/*
 * Ethernet MAC PE-MCXMAC Register Map, accessed via M2S_MAC_CFG macro
 */
struct m2s_mac_cfg_regs {
	u32	cfg1;			/* MAC Configuration register 1	      */
	u32	cfg2;			/* MAC Configuration register 2	      */
	u32	ifg;			/* Inter Packet/Frame gaps	      */
	u32	half_duplex;		/* Definition of half duplex	      */
	u32	max_frame_length;	/* Maximum frame size		      */
	u32	rsv[2];
	u32	test;			/* For testing purposes		      */
	u32	mii_config;		/* MII configuration		      */
	u32	mii_command;		/* MII command			      */
	u32	mii_address;		/* 5-bit PHY addr / 5 bit reg addr    */
	u32	mii_ctrl;		/* MII Mgmt write cycle control	      */
	u32	mii_status;		/* MII Mgmt read cycle status	      */
	u32	mii_ind;		/* MII Mgmt indication		      */
	u32	if_ctrl;		/* Interface controls		      */
	u32	if_stat;		/* Interface status		      */
	u32	station_addr[2];	/* Station MAC address		      */
	u32	fifo_cfg[6];		/* A-MCXFIFO configuration registers  */
	u32	fifo_ram_access[8];	/* FIFO RAM access registers	      */
	u32	stat[(0x140 - 0x80) / 4];/* Statistic registers		      */
};

/*
 * Ethernet MAC M-AHB Register Map, accessed via M2S_MAC_DMA macro
 */
struct m2s_mac_dma_regs {
	u32	tx_ctrl;		/* Transmit control register	      */
	u32	tx_desc;		/* Pointer to Transmit Descriptor     */
	u32	tx_stat;		/* Transmit Status register	      */
	u32	rx_ctrl;		/* Receive Control register	      */
	u32	rx_desc;		/* Pointer to Receive Descriptor      */
	u32	rx_stat;		/* Receive Status register	      */
	u32	irq_msk;		/* Interrupt Mask register	      */
	u32	irq;			/* Interrupts register		      */
};

/*
 * M2S MAC DMA Receive/Transmit descriptor
 */
struct m2s_mac_dma_bd {
	u32	adr_buf;		/* Pointer to frame buffer	      */
	u32	cfg_size;		/* Cfg flags and frame size	      */
	u32	adr_bd_next;		/* Pointer to next BD in chain	      */
};

static void m2s_mac_dump_regs(char *who, struct m2s_mac_dev *d);
static int msgmii_phy_autonegotiate(struct mii_bus *bus);

/******************************************************************************
 * miiphy routines
 ******************************************************************************/

/*
 * Update link status info
 */
static void m2s_phy_adjust_link(struct net_device *dev)
{
	struct m2s_mac_dev	*d = netdev_priv(dev);
	struct phy_device	*phy_dev = d->phy_dev;
	unsigned long		f, msk;
	char			change = 0;

	spin_lock_irqsave(&d->lock, f);
	if (!phy_dev->link)
		goto link;

	if (d->duplex == phy_dev->duplex)
		goto speed;
	/*
	 * Duplex changed
	 */
	M2S_MAC_CFG(d)->cfg2 &= ~M2S_MAC_CFG2_FULL_DUP;
	M2S_MAC_CFG(d)->fifo_cfg[5] |= M2S_MAC_FIFO_CFG5_CFGHDPLX;
	if (phy_dev->duplex) {
		M2S_MAC_CFG(d)->cfg2 |= M2S_MAC_CFG2_FULL_DUP;
		M2S_MAC_CFG(d)->fifo_cfg[5] &= ~M2S_MAC_FIFO_CFG5_CFGHDPLX;
	}
	d->duplex = phy_dev->duplex;
	change = 1;
speed:
	if (d->speed == phy_dev->speed)
		goto link;
	/*
	 * Speed changed
	 */

	M2S_SYSREG->mac_cr &= ~(M2S_SYS_MAC_CR_LS_MSK << M2S_SYS_MAC_CR_LS_BIT);
	M2S_MAC_CFG(d)->cfg2 &= ~(M2S_MAC_CFG2_MODE_MSK << M2S_MAC_CFG2_MODE_BIT);
	switch (phy_dev->speed) {
	case 10:
		M2S_MAC_CFG(d)->if_ctrl &= ~M2S_MAC_INTF_SPEED_100;
		/* A note from the Microsemi TSEMAC driver states: avoid extra
		 * divide by 10 of clock for 10Mbps links. */
		msk = M2S_SYS_MAC_CR_LS_100 << M2S_SYS_MAC_CR_LS_BIT;
		M2S_MAC_CFG(d)->cfg2 |= (M2S_MAC_CFG2_MODE_MII << M2S_MAC_CFG2_MODE_BIT);
		break;
	case 100:
		M2S_MAC_CFG(d)->if_ctrl |= M2S_MAC_INTF_SPEED_100;
		msk = M2S_SYS_MAC_CR_LS_100 << M2S_SYS_MAC_CR_LS_BIT;
		M2S_MAC_CFG(d)->cfg2 |= (M2S_MAC_CFG2_MODE_MII << M2S_MAC_CFG2_MODE_BIT);
		break;
	case 1000:
		M2S_MAC_CFG(d)->if_ctrl &= ~M2S_MAC_INTF_SPEED_100;
		msk = M2S_SYS_MAC_CR_LS_1000 << M2S_SYS_MAC_CR_LS_BIT;
		M2S_MAC_CFG(d)->cfg2 |= (M2S_MAC_CFG2_MODE_BYTE << M2S_MAC_CFG2_MODE_BIT);
		break;
	default:
		printk(KERN_WARNING "%s: Bad speed(%d)\n", dev->name, d->speed);
		goto link;
	}
	M2S_SYSREG->mac_cr |= msk;
	d->speed = phy_dev->speed;
	change = 1;
link:
	if (d->link == phy_dev->link)
		goto done;
	/*
	 * Link changed
	 */
	if (!phy_dev->link) {
		d->speed = 0;
		d->duplex = -1;
	}
	d->link = phy_dev->link;
	change = 1;
done:
	spin_unlock_irqrestore(&d->lock, f);

	if (!change)
		goto out;

	if (d->link && (!(d->flags & ETH_M2S_FLAGS_MODE_SGMII)
			|| msgmii_phy_autonegotiate(d->mii_bus) >= 0)) {
		printk(KERN_INFO "%s: link up (%d/%s)\n", dev->name,
			d->speed, d->duplex == DUPLEX_FULL ? "full" : "half");
	} else {
		printk(KERN_INFO "%s: link down\n", dev->name);
	}
out:
	return;
}

/*
 * Read register value via MII
 */
static int m2s_mii_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct m2s_mac_dev	*d = bus->priv;
	int			timeout;

	/*
	 * Set PHY & REG addresses, and issue read command.
	 * Note, we MUST reset 'READ' bit in cmd, otherwise, on each
	 * next read - the same data will be returned in mii_status
	 */
	M2S_MAC_CFG(d)->mii_address = (phy_id << M2S_MAC_MII_ADR_PHY_BIT) |
				      (reg << M2S_MAC_MII_ADR_REG_BIT);
	M2S_MAC_CFG(d)->mii_command = M2S_MAC_MII_CMD_READ;
	M2S_MAC_CFG(d)->mii_command = 0;

	timeout = M2S_MII_TOUT;
	while ((M2S_MAC_CFG(d)->mii_ind & (M2S_MAC_MII_IND_NVAL |
					   M2S_MAC_MII_IND_BUSY)) && timeout) {
		timeout--;
		udelay(1000);
	}

	timeout = timeout ? M2S_MAC_CFG(d)->mii_status : -ETIMEDOUT;

	return timeout;
}

/*
 * Write register value to MII
 */
static int m2s_mii_write(struct mii_bus *bus, int phy_id, int reg, u16 data)
{
	struct m2s_mac_dev	*d = bus->priv;
	int			timeout;

	/*
	 * Set PHY & REG addresses, then write data
	 */
	M2S_MAC_CFG(d)->mii_address = (phy_id << M2S_MAC_MII_ADR_PHY_BIT) |
				      (reg << M2S_MAC_MII_ADR_REG_BIT);
	M2S_MAC_CFG(d)->mii_ctrl = data;

	timeout = M2S_MII_TOUT;
	while ((M2S_MAC_CFG(d)->mii_ind & M2S_MAC_MII_IND_BUSY) && timeout) {
		timeout--;
		udelay(1000);
	}

	return timeout ? 0 : -ETIMEDOUT;
}

/*
 * Init MII
 */
static int m2s_mii_init(struct net_device *dev)
{
	struct m2s_mac_dev	*d = netdev_priv(dev);
	struct phy_device	*phy_dev;
	int			rv, i;

	d->mii_bus = mdiobus_alloc();
	if (!d->mii_bus) {
		printk(KERN_ERR "%s: mii bus alloc error\n", __func__);
		rv = -ENOMEM;
		goto err_free_none;
	}

	d->mii_bus->name = "m2f MII bus";
	snprintf(d->mii_bus->id, MII_BUS_ID_SIZE, "%02x", d->ptf_dev->id);

	d->mii_bus->read = m2s_mii_read;
	d->mii_bus->write = m2s_mii_write;
	d->mii_bus->priv = d;
	d->mii_bus->parent = &d->ptf_dev->dev;
	d->mii_bus->phy_mask = 0xFFFFFFF0;

	rv = mdiobus_register(d->mii_bus);
	if (rv) {
		printk(KERN_ERR "%s: mii bus registration error %d\n",
			__func__, rv);
		goto err_free_alloc;
	}

	phy_dev = NULL;
	for (i = 0; i < PHY_MAX_ADDR; i++) {
		if (!d->mii_bus->phy_map[i])
			continue;
		phy_dev = d->mii_bus->phy_map[i];
		printk(KERN_INFO "found PHY id 0x%x addr %d\n",
			phy_dev->phy_id, phy_dev->addr);
		break;
	}
	if (!phy_dev) {
		printk(KERN_ERR "no PHY found\n");
		goto err_free_reg;
	}

	phy_dev = phy_connect(dev, dev_name(&phy_dev->dev),
			      &m2s_phy_adjust_link, 0, PHY_INTERFACE_MODE_MII);
	if (IS_ERR(phy_dev)) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		rv = -ENODEV;
		goto err_free_reg;
	}

	phy_dev->supported &= PHY_BASIC_FEATURES;

	d->phy_id = phy_dev->phy_id;
	d->phy_dev = phy_dev;
	d->link = 0;
	d->speed = 0;
	d->duplex = -1;

	rv = 0;
	goto done;
err_free_reg:
	mdiobus_unregister(d->mii_bus);
err_free_alloc:
	mdiobus_free(d->mii_bus);
err_free_none:
	d->mii_bus = NULL;
done:
	return rv;
}

#define SF2_MSGMII_PHY_ADDR	0x1e
#define M2S_AUTONEG_TOUT	10000

static int msgmii_phy_init(struct mii_bus *bus)
{
	int rv;

	/* Reset M-SGMII. */
	rv = m2s_mii_write(bus, SF2_MSGMII_PHY_ADDR, 0x00, 0x9000u);
	if (rv != 0) {
		return -1;
	}
	/* Register 0x04 of M-SGMII must be always be set to 0x0001. */
	rv = m2s_mii_write(bus, SF2_MSGMII_PHY_ADDR, 0x04, 0x0001);
	if (rv != 0) {
		return -1;
	}
	/*
	 * Enable auto-negotiation inside SmartFusion2 SGMII block.
	 */
	rv = m2s_mii_read(bus, SF2_MSGMII_PHY_ADDR, 0);
	if (rv < 0) {
		return -1;
	}
	rv |= 0x1000;
	rv = m2s_mii_write(bus, SF2_MSGMII_PHY_ADDR, 0x0, rv);
	if (rv != 0) {
		return -1;
	}

	return 0;
}

static int msgmii_phy_autonegotiate(struct mii_bus *bus)
{
	int rv;
	int timeout;

	rv = m2s_mii_read(bus, SF2_MSGMII_PHY_ADDR, MII_BMSR);
	if (rv < 0) {
		goto out;
	}

	if (rv & BMSR_ANEGCOMPLETE) {
		/* no need to start auto-negotiation if it is already done */
		goto out;
	}

	rv = m2s_mii_write(bus, SF2_MSGMII_PHY_ADDR, MII_BMCR,
			BMCR_ANRESTART | BMCR_ANENABLE);
	if (rv < 0) {
		goto out;
	}

	/*
	 * Wait until auto-negotiation completes
	 */
	timeout = M2S_AUTONEG_TOUT/10;
	while (timeout--) {
		rv = m2s_mii_read(bus, SF2_MSGMII_PHY_ADDR, MII_BMSR);

		if (rv < 0) {
			printk("mii err %d.\n", rv);
			goto out;
		}
		if (!(rv & BMSR_ANEGCOMPLETE)) {
			if (timeout % 100 != 0) {
				mdelay(10);
				continue;
			}

			/* restart auto-negotiation if it is not complete
			   in a second */
			rv = m2s_mii_write(bus, SF2_MSGMII_PHY_ADDR, MII_BMCR,
					BMCR_ANRESTART | BMCR_ANENABLE);
			if (rv != 0) {
				goto out;
			}

			continue;
		}
		break;
	}
	if (timeout <= 0)
		debug("MSGMII PHY auto-negotiaiton timed out!\n");

 out:
	return rv;


}

/*
 * MAC Hardware initialization
 */
static __init int m2s_mac_hw_init(struct m2s_mac_dev *d)
{
	static u8	div[] = {2, 4, 6, 8, 10, 14, 20, 28};
	int		rv, i, timeout;

	/*
	 * Release the Ethernet MAC from reset
	 */
	M2S_SYSREG->soft_reset_cr &= ~M2S_SYS_SOFT_RST_CR_MAC;

	/*
	 * Set-up CR
	 */
	M2S_SYSREG->mac_cr &= ~(M2S_SYS_MAC_CR_PM_MSK << M2S_SYS_MAC_CR_PM_BIT);
	if (!(d->flags & ETH_M2S_FLAGS_MODE_SGMII))
		M2S_SYSREG->mac_cr |=
			M2S_SYS_MAC_CR_PM_MII << M2S_SYS_MAC_CR_PM_BIT;
	else
		M2S_SYSREG->mac_cr |=
			M2S_SYS_MAC_CR_PM_TBI << M2S_SYS_MAC_CR_PM_BIT;

	/*
	 * Reset all PE-MCXMAC modules, and configure
	 */
	M2S_MAC_CFG(d)->cfg1 |= M2S_MAC_CFG1_RST;
	M2S_MAC_CFG(d)->cfg1 &= ~M2S_MAC_CFG1_RST;

	M2S_MAC_CFG(d)->if_ctrl &= ~M2S_MAC_INTF_RESET;

	M2S_MAC_CFG(d)->cfg2 &= ~(M2S_MAC_CFG2_MODE_MSK<<M2S_MAC_CFG2_MODE_BIT);
	if (!(d->flags & ETH_M2S_FLAGS_MODE_SGMII))
		M2S_MAC_CFG(d)->cfg2 |=
			(M2S_MAC_CFG2_MODE_MII<<M2S_MAC_CFG2_MODE_BIT) |
			M2S_MAC_CFG2_PAD_CRC;
	else
		M2S_MAC_CFG(d)->cfg2 =
			M2S_MAC_CFG2_FULL_DUP | M2S_MAC_CFG2_CRC_EN
			| M2S_MAC_CFG2_PAD_CRC | M2S_MAC_CFG2_LEN_CHECK
			| (M2S_MAC_CFG2_MODE_BYTE << M2S_MAC_CFG2_MODE_BIT)
			| (0x7 << M2S_MAC_CFG2_PREAM_LEN_BIT);

	M2S_MAC_CFG(d)->max_frame_length = M2S_MAC_FRM_SIZE;

	for (i = 0; i < ARRAY_SIZE(div); i++) {
		if (d->freq_src / div[i] > d->freq_mdc)
			continue;
		break;
	}
	if (i == ARRAY_SIZE(div)) {
		i--;
		printk(KERN_WARNING "%s: MDC set to %dHz (min) instead %dHz\n",
			__func__, d->freq_src / div[i], d->freq_mdc);
	}
	M2S_MAC_CFG(d)->mii_config = i;

	/*
	 * Set BDs
	 */
	M2S_MAC_DMA(d)->rx_desc = d->rx.bd_adr;
	M2S_MAC_DMA(d)->rx_ctrl = M2S_MAC_DMA_CTRL_ENA;

	M2S_MAC_DMA(d)->tx_desc = d->tx.bd_adr;
	M2S_MAC_DMA(d)->tx_ctrl = M2S_MAC_DMA_CTRL_ENA;

	/*
	 * Reset and enable FIFOs
	 */
	M2S_MAC_CFG(d)->fifo_cfg[0] |= M2S_MAC_FIFO_CFG0_ALL_RST;
	M2S_MAC_CFG(d)->fifo_cfg[0] &= ~M2S_MAC_FIFO_CFG0_ALL_RST;

	M2S_MAC_CFG(d)->fifo_cfg[0] = M2S_MAC_FIFO_CFG0_ALL_REQ | 0xFF00;
	timeout = M2S_FIFO_TOUT;
	while (((M2S_MAC_CFG(d)->fifo_cfg[0] & M2S_MAC_FIFO_CFG0_ALL_RPLY) !=
		M2S_MAC_FIFO_CFG0_ALL_RPLY) && timeout) {
		timeout--;
		udelay(1000);
	}
	if (!timeout) {
		printk(KERN_ERR "%s: FIFO initialization timeout\n", __func__);
		m2s_mac_dump_regs("FIFO init", d);
		rv = -ETIMEDOUT;
		goto out;
	}

	/*
	 * Use same FIFO settings as in AR71xx driver
	 */
	M2S_MAC_CFG(d)->fifo_cfg[1] = 0x0fff0000;
	M2S_MAC_CFG(d)->fifo_cfg[2] = 0x00001fff;
	M2S_MAC_CFG(d)->fifo_cfg[3] = 0x008001ff;

	/* Filter out bad packets */
	M2S_MAC_CFG(d)->fifo_cfg[4] = 0x0000ffff;
	M2S_MAC_CFG(d)->fifo_cfg[5] = 0x0007ffff;

	M2S_MAC_DMA(d)->irq_msk = M2S_MAC_DMA_IRQ_MSK;
	M2S_MAC_DMA(d)->irq = M2S_MAC_DMA_IRQ_MSK;

	rv = 0;
out:
	if (rv)
		M2S_SYSREG->soft_reset_cr |= M2S_SYS_SOFT_RST_CR_MAC;

	return rv;
}

/*
 * Common MAC interrupt handler
 */
static irqreturn_t m2s_mac_irq_cb(int irq, void *dev_id)
{
	struct net_device	*dev = dev_id;
	struct m2s_mac_dev	*d = netdev_priv(dev);
	u32			status, handled = 0;

	status = M2S_MAC_DMA(d)->irq;

	if (unlikely(status & M2S_MAC_DMA_IRQ_ERR)) {
		handled++;
		if (status & M2S_MAC_DMA_IRQ_RBUS)
			M2S_MAC_DMA(d)->rx_stat = M2S_MAC_DMA_STAT_RX_BE;
		if (status & M2S_MAC_DMA_IRQ_TBUS)
			M2S_MAC_DMA(d)->tx_stat = M2S_MAC_DMA_STAT_TX_BE;
	}

	if (likely(status & M2S_MAC_DMA_IRQ_XMIT)) {
		handled++;
		M2S_MAC_DMA(d)->irq_msk &= ~M2S_MAC_DMA_IRQ_XMIT;
		napi_schedule(&d->napi);
	}

	if (unlikely(!handled))
		m2s_mac_dump_regs("Unhandled M2S MAC IRQ", d);

	return IRQ_RETVAL(handled);
}

static int m2s_mac_tx_packets(struct m2s_mac_dev *d)
{
	volatile struct m2s_mac_dma_bd	*bd;
	struct net_device		*dev = d->net_dev;
	u32				idx;
	int				sent = 0;

	/*
	 * Process frames
	 */
	while (d->tx.idx_cur != d->tx.idx_nxt) {
		idx = d->tx.idx_cur % M2S_TX_NUM;
		bd = &d->tx.bd[idx];

		if (!(bd->cfg_size & M2S_BD_EMPTY))
			break;
		if (!(d->tx.skb[idx]))
			break;

		dev->stats.tx_bytes += d->tx.skb[idx]->len;
		dev->stats.tx_packets++;

		dev_kfree_skb_any(d->tx.skb[idx]);
		d->tx.skb[idx] = NULL;

		M2S_MAC_DMA(d)->tx_stat = M2S_MAC_DMA_STAT_TX_PS;
		d->tx.idx_cur++;
		sent++;
	}

	netif_wake_queue(dev);

	return sent;
}

int m2s_mac_rx_packets(struct m2s_mac_dev *d, int limit)
{
	volatile struct m2s_mac_dma_bd	*bd;
	struct sk_buff			*skb;
	struct net_device		*dev = d->net_dev;
	u32				size, idx;
	int				done = 0;

	/*
	 * Process rx frames
	 */
	while (done < limit) {
		idx = d->rx.idx % M2S_RX_NUM;
		bd = &d->rx.bd[idx];
		if (bd->cfg_size & M2S_BD_EMPTY)
			break;

		size = bd->cfg_size & M2S_BD_SIZE_MSK;
		if (size < sizeof(struct ethhdr)) {
			printk("%s: packet too small: %d\n", __func__, size);
			dev->stats.rx_dropped++;
			goto next;
		}

		skb = dev_alloc_skb(size - 4 + NET_IP_ALIGN);
		if (unlikely(!skb)) {
			printk("%s: no mem, drop packet\n", __func__);
			dev->stats.rx_dropped++;
			goto next;
		}

		size -= 4;
		dev->stats.rx_bytes += size;
		dev->stats.rx_packets++;

		skb_reserve(skb, NET_IP_ALIGN);
		skb_put(skb, size);
		skb_copy_to_linear_data(skb, d->rx.buf[idx], size);
		skb->protocol = eth_type_trans(skb, dev);
		netif_rx(skb);
next:
		bd->cfg_size = M2S_BD_EMPTY;
		M2S_MAC_DMA(d)->rx_stat = M2S_MAC_DMA_STAT_RX_PR;

		d->rx.idx++;
		done++;
	}

	return done;
}

/*
 * NAPI poll
 */
static int m2s_mac_poll(struct napi_struct *napi, int limit)
{
	struct m2s_mac_dev	*d = container_of(napi, struct m2s_mac_dev,
						  napi);
	int			tx_done, rx_done;
	u32			status;
	int			rv = 1;

	tx_done = m2s_mac_tx_packets(d);
	rx_done = m2s_mac_rx_packets(d, limit);

	status = M2S_MAC_DMA(d)->rx_stat;
	if (unlikely(status & M2S_MAC_DMA_STAT_RX_OF)) {
		d->net_dev->stats.rx_fifo_errors++;
		M2S_MAC_DMA(d)->rx_stat = M2S_MAC_DMA_STAT_RX_OF;
		M2S_MAC_DMA(d)->rx_ctrl = M2S_MAC_DMA_CTRL_ENA;
	}

	if (rx_done < limit) {
		if (status & M2S_MAC_DMA_STAT_RX_PR)
			goto more;

		status = M2S_MAC_DMA(d)->tx_stat;
		if (status & M2S_MAC_DMA_STAT_TX_PS)
			goto more;

		rv = 0;
		napi_complete(napi);
		M2S_MAC_DMA(d)->irq_msk |= M2S_MAC_DMA_IRQ_XMIT;
	}
more:
	/*
	 * Return 1: there are still packets waiting, 0: stop polling
	 */
	return rv;
}

/*
 * Dump MAC registers
 */
static void m2s_mac_dump_regs(char *who, struct m2s_mac_dev *d)
{
	volatile struct m2s_mac_dma_bd	*bd;
	int				i;

	printk("*** %s %s:\n", __func__, who ? who : "<unknown>");

	printk(" DMA TX CTRL=%08x;DESC=%08x;STAT=%08x\n",
		M2S_MAC_DMA(d)->tx_ctrl, M2S_MAC_DMA(d)->tx_desc,
		M2S_MAC_DMA(d)->tx_stat);
	printk(" DMA RX CTRL=%08x;DESC=%08x;STAT=%08x\n",
		M2S_MAC_DMA(d)->rx_ctrl, M2S_MAC_DMA(d)->rx_desc,
		M2S_MAC_DMA(d)->rx_stat);
	printk(" DMA IRQ %08x/%08x\n",
		M2S_MAC_DMA(d)->irq, M2S_MAC_DMA(d)->irq_msk);
	printk(" CFG1=%08x;CFG2=%08x;IFG=%08x;HD=%08x;MFL=%08x\n",
		M2S_MAC_CFG(d)->cfg1, M2S_MAC_CFG(d)->cfg2,
		M2S_MAC_CFG(d)->ifg, M2S_MAC_CFG(d)->half_duplex,
		M2S_MAC_CFG(d)->max_frame_length);
	printk(" IFCTRL=%08x;IFSTAT=%08x;ADR1=%08x;ADR2=%08x\n",
		M2S_MAC_CFG(d)->if_ctrl, M2S_MAC_CFG(d)->if_stat,
		M2S_MAC_CFG(d)->station_addr[0],
		M2S_MAC_CFG(d)->station_addr[1]);
	printk(" FIFO CFG ");
	for (i = 0; i < 6; i++)
		printk("%08x/", M2S_MAC_CFG(d)->fifo_cfg[i]);
	printk("\n");

	printk(" FIFO ACC ");
	for (i = 0; i < 8; i++)
		printk("%08x/", M2S_MAC_CFG(d)->fifo_ram_access[i]);
	printk("\n");
	printk(" TX BDs (idx_nxt=%d,idx_cur=%d.drop=%ld,err=%ld,done=%ld):\n",
		d->tx.idx_nxt, d->tx.idx_cur,
		d->net_dev->stats.tx_dropped, d->net_dev->stats.tx_fifo_errors,
		d->net_dev->stats.tx_packets);
	bd = (void *)M2S_MAC_DMA(d)->tx_desc;
	do {
		printk("  at %p: buf=0x%08x.cfg=0x%08x.nxt=0x%08x\n", bd,
			bd ? bd->adr_buf : 0,
			bd ? bd->cfg_size : 0,
			bd ? bd->adr_bd_next : 0);
		bd = (void *)bd->adr_bd_next;
	} while (bd && bd != (void *)M2S_MAC_DMA(d)->tx_desc);

	printk(" RX BDs (idx=%d.drop=%ld,err=%ld,over=%ld,done=%ld):\n",
		d->rx.idx,
		d->net_dev->stats.rx_dropped, d->net_dev->stats.rx_fifo_errors,
		d->net_dev->stats.rx_over_errors, d->net_dev->stats.rx_packets);
	bd = (void *)M2S_MAC_DMA(d)->rx_desc;
	do {
		printk("  at %p: buf=0x%08x.cfg=0x%08x.nxt=0x%08x\n", bd,
			bd ? bd->adr_buf : 0,
			bd ? bd->cfg_size : 0,
			bd ? bd->adr_bd_next : 0);
		bd = (void *)bd->adr_bd_next;
	} while (bd && bd != (void *)M2S_MAC_DMA(d)->rx_desc);
}

/******************************************************************************
 * net_device_ops functions
 ******************************************************************************/

static int m2s_mac_net_open(struct net_device *dev)
{
	struct m2s_mac_dev	*d = netdev_priv(dev);
	int			rv;

	debug("%s\n", __func__);

	napi_enable(&d->napi);

	/*
	 * Enable MAC Rx and Tx
	 */
	M2S_MAC_CFG(d)->cfg1 = M2S_MAC_CFG1_RX_ENA | M2S_MAC_CFG1_TX_ENA;

	d->link = 0;
	d->duplex = d->speed = -1;

	/*
	 * Schedule a link state check
	 */
	rv = phy_start_aneg(d->phy_dev);
	if (rv) {
		printk(KERN_ERR "%s: start auto-negotiation error %d\n",
			__func__, rv);
		goto out;
	}

	phy_start(d->phy_dev);
	netif_start_queue(dev);
out:
	return rv;
}

static int m2s_mac_net_stop(struct net_device *dev)
{
	struct m2s_mac_dev	*d = netdev_priv(dev);

	debug("%s\n", __func__);

	netif_stop_queue(dev);
	netif_carrier_off(dev);

	M2S_MAC_CFG(d)->cfg1 = 0;

	napi_disable(&d->napi);

	return 0;
}

static netdev_tx_t m2s_mac_net_start_xmit(struct sk_buff *skb,
					  struct net_device *dev)
{
	struct m2s_mac_dev	*d = netdev_priv(dev);
	unsigned char		*p = skb->data;
	u32			idx;
	int			rv;

	idx = d->tx.idx_nxt % M2S_TX_NUM;
	if (!(d->tx.bd[idx].cfg_size & M2S_BD_EMPTY)) {
		dev->stats.tx_dropped++;
		rv = NETDEV_TX_BUSY;
		goto out;
	}

	/*
	 * Check alignment; actually - should check dma mapped addr..
	 */
	if ((u32)p & 0x3) {
		skb_copy_from_linear_data(skb, d->tx.buf[idx], skb->len);
		p = d->tx.buf[idx];
	}


	d->tx.skb[idx] = skb;
	d->tx.bd[idx].adr_buf = (dma_addr_t)p;
	d->tx.bd[idx].cfg_size = skb->len;

	d->tx.idx_nxt++;
	if (d->tx.idx_nxt == (d->tx.idx_cur + M2S_TX_NUM))
		netif_stop_queue(dev);

	debug("%s: %dB; dsc[%d]=%08x.%08x.%08x / %08x.%08x\n", __func__,
		skb->len, idx, d->tx.bd[idx].adr_buf, d->tx.bd[idx].cfg_size,
		d->tx.bd[idx].adr_bd_next, M2S_MAC_DMA(d)->tx_ctrl,
		M2S_MAC_DMA(d)->tx_stat);

	M2S_MAC_DMA(d)->tx_ctrl = M2S_MAC_DMA_CTRL_ENA;

	rv = NETDEV_TX_OK;
out:
	return rv;
}

static int m2s_mac_net_set_mac_address(struct net_device *dev, void *p)
{
	struct m2s_mac_dev	*d = netdev_priv(dev);
	struct sockaddr *address = (struct sockaddr *)p;
	u8			*adr;
	int			rv = 0;

	if (!d || !address) {
		printk(KERN_ERR "%s: bad params %p/%p\n", __func__, d, address);
		rv = -EINVAL;
		goto out;
	}
	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	adr = (u8 *)address->sa_data;
	debug("%s %02x:%02x:%02x:%02x:%02x:%02x\n", __func__, adr[0], adr[1],
		adr[2], adr[3], adr[4], adr[5]);

	M2S_MAC_CFG(d)->station_addr[0] = (adr[0] << 24) | (adr[1] << 16) |
					  (adr[2] <<  8) | (adr[3] <<  0);
	M2S_MAC_CFG(d)->station_addr[1] = (adr[4] << 24) | (adr[5] << 16);
out:
	return rv;
}

/*
 * Net device descriptor
 */
static const struct net_device_ops m2s_mac_netdev_ops = {
	.ndo_open		= m2s_mac_net_open,
	.ndo_stop		= m2s_mac_net_stop,
	.ndo_start_xmit		= m2s_mac_net_start_xmit,
	.ndo_set_mac_address	= m2s_mac_net_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
};

/******************************************************************************
 * Platform driver functions
 ******************************************************************************/

static int __init m2s_mac_probe(struct platform_device *pd)
{
	struct eth_m2s_platform_data	*pdat;
	struct m2s_mac_dev		*d;
	struct net_device		*dev;
	struct resource			*res;
	char				*p;
	u32				reg_base;
	int				rv, irq, i;
	u8				mac[6];

	res = platform_get_resource(pd, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pd->dev, "no mmio resource defined\n");
		rv = -EINVAL;
		goto err_free_none;
	}
	reg_base = res->start;

	irq = platform_get_irq(pd, 0);
	if (irq < 0) {
		dev_err(&pd->dev, "no irq resource defined\n");
		rv = -EINVAL;
		goto err_free_none;
	}

	pdat = platform_get_drvdata(pd);
	if (!pdat) {
		dev_err(&pd->dev, "no platform data specified\n");
		rv = -EINVAL;
		goto err_free_none;
	}

	dev = alloc_etherdev(sizeof(struct m2s_mac_dev));
	if (!dev) {
		dev_err(&pd->dev, "etherdev alloc failed\n");
		rv = -ENOMEM;
		goto err_free_none;
	}
	platform_set_drvdata(pd, dev);
	dev->netdev_ops = &m2s_mac_netdev_ops;

	d = netdev_priv(dev);
	memset(d, 0, sizeof(*d));

	d->net_dev = dev;
	d->ptf_dev = pd;

	SET_NETDEV_DEV(dev, &pd->dev);
	spin_lock_init(&d->lock);

	d->freq_src = pdat->freq_src;
	d->freq_mdc = pdat->freq_mdc;
	d->flags = pdat->flags;

	if (!M2S_RX_NUM || !M2S_TX_NUM) {
		dev_err(&pd->dev, "bad rx/tx %d/%d buf number",
			M2S_RX_NUM, M2S_TX_NUM);
		rv = -EINVAL;
		goto err_free_dev;
	}

	dev->irq = irq;
	rv = request_irq(irq, m2s_mac_irq_cb, 0, M2S_MAC_NAME, dev);
	if (rv) {
		dev_err(&pd->dev, "request IRQ%d error %d", irq, rv);
		rv = -EFAULT;
		goto err_free_dev;
	}

	d->reg[0] = ioremap(reg_base + M2S_MAC_OFS,
			    sizeof(struct m2s_mac_cfg_regs));
	d->reg[1] = ioremap(reg_base + M2S_DMA_OFS,
			    sizeof(struct m2s_mac_dma_regs));
	if (!d->reg[0] || !d->reg[1]) {
		dev_err(&pd->dev, "ioremap 0x%08x %p/%p error",
			reg_base, d->reg[0], d->reg[1]);
		rv = -ENOMEM;
		goto err_free_remap;
	}

	printk(KERN_INFO "Found M2S MAC at 0x%08x, irq %d\n", reg_base,
		dev->irq);

	memset(mac, 0, sizeof(mac));
	p = strnstr(boot_command_line, "ethaddr=", COMMAND_LINE_SIZE);
	if (p) {
		int	i;
		u8	ethaddr[18];

		memcpy(ethaddr, p + strlen("ethaddr="), sizeof(ethaddr));
		p = ethaddr;
		for (i = 0; i <= 5; i++) {
			mac[i] = simple_strtoul(p, &p, 16) |
				 (simple_strtoul(p, &p, 16) << 4);
			p++;
		}
	}

	if (!is_valid_ether_addr(mac)) {
		printk(KERN_ERR "MAC addr isn't set or invalid, use random\n");
		random_ether_addr(mac);
	}
	memcpy(dev->dev_addr, mac, sizeof(mac));

	/*
	 * Use same NAPI weight as in AR71xx driver
	 */
	netif_napi_add(dev, &d->napi, m2s_mac_poll, 64);

	rv = register_netdev(dev);
	if (rv) {
		dev_err(&pd->dev, "can't register net device\n");
		goto err_free_remap;
	}

	/*
	 * Allocate descriptors
	 */
	pd->dev.coherent_dma_mask = DMA_BIT_MASK(32);
#if !defined(M2S_ESRAM)
	d->rx.bd = dma_alloc_coherent(&pd->dev, sizeof(*d->rx.bd) * M2S_RX_NUM,
				      &d->rx.bd_adr, GFP_DMA);
	d->tx.bd = dma_alloc_coherent(&pd->dev, sizeof(*d->tx.bd) * M2S_TX_NUM,
				      &d->tx.bd_adr, GFP_DMA);
	if (!d->rx.bd || !d->tx.bd) {
		dev_err(&pd->dev, "dma_alloc BD failure %p/%p\n",
			d->rx.bd, d->tx.bd);
		rv = -ENOMEM;
		goto err_free_bd;
	}
#else
	p = (void *)M2S_ESRAM;

	d->rx.bd = (void *)p;
	d->rx.bd_adr = (dma_addr_t)p;
	p += sizeof(*d->rx.bd) * M2S_RX_NUM;

	d->tx.bd = (void *)p;
	d->tx.bd_adr = (dma_addr_t)p;
	p += sizeof(*d->tx.bd) * M2S_TX_NUM;
#endif
	memset((void *)d->rx.bd, 0, sizeof(*d->rx.bd) * M2S_RX_NUM);
	memset((void *)d->tx.bd, 0, sizeof(*d->tx.bd) * M2S_TX_NUM);

	/*
	 * Allocate bufs for incoming frames, link BDs to list, and mark all
	 * as empty. We don't specify buf sizes in cfg_size, 'cause these
	 * aren't used according to doc; guess MAC assumes size of buffer
	 * basing on max_frame_length register
	 */
	for (i = 0; i < M2S_RX_NUM; i++) {
#if !defined(M2S_ESRAM)
		d->rx.buf[i] = dma_alloc_coherent(&pd->dev, M2S_MAC_FRM_SIZE,
					(void *)&d->rx.bd[i].adr_buf, GFP_DMA);
		if (!d->rx.buf[i]) {
			dev_err(&pd->dev, "dma_alloc rx buf[%d] failure\n", i);
			rv = -ENOMEM;
			goto err_free_buf;
		}
#else
		d->rx.buf[i] = p;
		d->rx.bd[i].adr_buf = (dma_addr_t)p;
		p += M2S_MAC_FRM_SIZE;
#endif
		d->rx.bd[i].cfg_size = M2S_BD_EMPTY;
		d->rx.bd[i].adr_bd_next = d->rx.bd_adr +
			(((i + 1) % M2S_RX_NUM) * sizeof(d->rx.bd[0]));
	}
	for (i = 0; i < M2S_TX_NUM; i++) {
#if !defined(M2S_ESRAM)
		d->tx.buf[i] = dma_alloc_coherent(&pd->dev, M2S_MAC_FRM_SIZE,
					(void *)&d->tx.bd[i].adr_buf, GFP_DMA);
		if (!d->tx.buf[i]) {
			dev_err(&pd->dev, "dma_alloc tx buf[%d] failure", i);
			rv = -ENOMEM;
			goto err_free_buf;
		}
#else
		d->tx.buf[i] = p;
		p += M2S_MAC_FRM_SIZE;
#endif
		d->tx.skb[i] = NULL;
		d->tx.bd[i].cfg_size = M2S_BD_EMPTY;
		d->tx.bd[i].adr_bd_next = d->tx.bd_adr +
			(((i + 1) % M2S_TX_NUM) * sizeof(d->tx.bd[0]));
	}

	d->rx.idx = d->tx.idx_cur = d->tx.idx_nxt = 0;

	/*
	 * Init hw
	 */
	rv = m2s_mac_hw_init(d);
	if (rv) {
		dev_err(&pd->dev, "hw init error %d", rv);
		goto err_free_hw;
	}

	rv = m2s_mii_init(dev);
	if (rv) {
		dev_err(&pd->dev, "mii init error %d", rv);
		goto err_free_hw;
	}

	if ((d->flags & ETH_M2S_FLAGS_MODE_SGMII)
			&& msgmii_phy_init(d->mii_bus) < 0)
		goto out;

	rv = 0;
	goto out;

err_free_hw:
	m2s_mac_net_stop(dev);
	if (d->mii_bus) {
		mdiobus_unregister(d->mii_bus);
		mdiobus_free(d->mii_bus);
	}
#if !defined(M2S_ESRAM)
err_free_buf:
	for (i = 0; i < M2S_TX_NUM; i++) {
		if (!d->tx.buf[i])
			break;
		dma_free_coherent(&pd->dev, M2S_MAC_FRM_SIZE,
				  d->tx.buf[i], d->tx.bd[i].adr_buf);
	}
	for (i = 0; i < M2S_RX_NUM; i++) {
		if (!d->rx.buf[i])
			break;
		dma_free_coherent(&pd->dev, M2S_MAC_FRM_SIZE,
				  d->rx.buf[i], d->rx.bd[i].adr_buf);
	}
err_free_bd:
	if (d->tx.bd) {
		dma_free_coherent(&pd->dev, sizeof(d->tx.bd[0]) * M2S_TX_NUM,
				  (void *)d->tx.bd, d->tx.bd_adr);
	}
	if (d->rx.bd) {
		dma_free_coherent(&pd->dev, sizeof(d->rx.bd[0]) * M2S_RX_NUM,
				  (void *)d->rx.bd, d->rx.bd_adr);
	}
#endif /* !M2S_ESRAM */
	unregister_netdev(dev);
err_free_remap:
	if (M2S_MAC_CFG(d))
		iounmap(M2S_MAC_CFG(d));
	if (M2S_MAC_DMA(d))
		iounmap(M2S_MAC_DMA(d));
	free_irq(dev->irq, dev);
err_free_dev:
	free_netdev(dev);
err_free_none:
out:
	return rv;
}

static int __devexit m2s_mac_remove(struct platform_device *pd)
{
	struct net_device	*dev = platform_get_drvdata(pd);
	struct m2s_mac_dev	*d = netdev_priv(dev);
#if !defined(M2S_ESRAM)
	int			i;
#endif

	m2s_mac_net_stop(dev);

	mdiobus_unregister(d->mii_bus);
	mdiobus_free(d->mii_bus);

#if !defined(M2S_ESRAM)
	for (i = 0; i < M2S_TX_NUM; i++) {
		dma_free_coherent(&pd->dev, M2S_MAC_FRM_SIZE,
				  d->tx.buf[i], d->tx.bd[i].adr_buf);
	}
	for (i = 0; i < M2S_RX_NUM; i++) {
		dma_free_coherent(&pd->dev, M2S_MAC_FRM_SIZE,
				  d->rx.buf[i], d->rx.bd[i].adr_buf);
	}
	dma_free_coherent(&pd->dev, sizeof(d->tx.bd[0]) * M2S_TX_NUM,
			  (void *)d->tx.bd, d->tx.bd_adr);
	dma_free_coherent(&pd->dev, sizeof(d->rx.bd[0]) * M2S_RX_NUM,
			  (void *)d->rx.bd, d->rx.bd_adr);
#endif
	unregister_netdev(dev);
	iounmap(M2S_MAC_CFG(d));
	iounmap(M2S_MAC_DMA(d));
	free_irq(dev->irq, dev);
	free_netdev(dev);

	return 0;
}

/*
 * Platform driver instance
 */
static struct platform_driver m2s_mac_platform_driver = {
	.probe	= m2s_mac_probe,
	.remove	= __devexit_p(m2s_mac_remove),
	.driver	= {
		.name	= M2S_MAC_NAME,
		.owner	= THIS_MODULE,
	}
};

static int __init m2s_mac_modinit(void)
{
	return platform_driver_register(&m2s_mac_platform_driver);
}

static void __exit m2s_mac_modexit(void)
{
	platform_driver_unregister(&m2s_mac_platform_driver);
}

module_init(m2s_mac_modinit);
module_exit(m2s_mac_modexit);

MODULE_AUTHOR("Yuri Tikhonov, <yur@emcraft.com>");
MODULE_DESCRIPTION("Device driver for MAC controller of SmartFusion2");
MODULE_LICENSE("GPL");
