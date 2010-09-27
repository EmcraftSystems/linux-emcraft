/*
 *
 * Copyright (C) 2010 Dmitry Cherkassov, Emcraft Systems
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
 *
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/mdio-bitbang.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>


#define DRV_NAME "core10100"

#define PFX DRV_NAME ": "

MODULE_LICENSE("GPL");



/* ethernet mac reset flag */
#define MAC_SR (4 << 1)

/* Soft reset controller address */
#define SOFT_RST_CR 0xE0042030


/* Register offsets */
#define CSR0			0x00
#define CSR1			0x08
#define CSR2			0x10
#define CSR3			0x18
#define CSR4			0x20
#define CSR5			0x28
#define CSR6			0x30
#define CSR7			0x38
#define CSR8			0x40
#define CSR9			0x48
#define CSR10			0x50
#define CSR11			0x58

/* Register fields */
#define CSR0_SWR		1
#define CSR0_TAP		(7 << 17)
#define CSR0_DBO		(1 << 20)

#define CSR1_TPD		1

#define CSR5_TPS		(1 << 1)
#define CSR5_RPS		(1 << 8)
#define CSR5_RS_SHIFT		17
#define CSR5_RS_MASK		0x07
#define CSR5_RS_STOP		0x00
#define CSR5_TS_SHIFT		20
#define CSR5_TS_MASK		0x07
#define CSR5_TS_STOP		0x00
#define CSR5_TS_SUSP		0x06

#define CSR5_TI_OFFSET   0x28
#define CSR5_TI_MASK     0x00000001UL
#define CSR5_TI_SHIFT    0

#define CSR5_NIS_OFFSET   0x28
#define CSR5_NIS_MASK     0x00010000uL
#define CSR5_NIS_SHIFT    16

#define CSR5_RI_OFFSET   0x28
#define CSR5_RI_MASK     0x00000040UL
#define CSR5_RI_SHIFT    6

#define CSR5_AIS_OFFSET   0x28
#define CSR5_AIS_MASK     0x00008000UL
#define CSR5_AIS_SHIFT    15

/* Early receive interrupt */
#define CSR5_ERI_OFFSET   0x28
#define CSR5_ERI_MASK     0x00004000UL
#define CSR5_ERI_SHIFT    14

 /* Transmit underflow */
#define CSR5_UNF_OFFSET   0x28
#define CSR5_UNF_MASK     0x00000020UL
#define CSR5_UNF_SHIFT    5

/* Transmit buffer unavailable */
#define CSR5_TU_OFFSET   0x28
#define CSR5_TU_MASK     0x00000004UL
#define CSR5_TU_SHIFT    2

 /* Receive process stopped */
#define CSR5_RPS_OFFSET   0x28
#define CSR5_RPS_MASK     0x00000100UL
#define CSR5_RPS_SHIFT    8

 /* General-purpose timer expiration */
#define CSR5_GTE_OFFSET   0x28
#define CSR5_GTE_MASK     0x00000800UL
#define CSR5_GTE_SHIFT    11

 /*Early transmit interrupt*/
#define CSR5_ETI_OFFSET   0x28
#define CSR5_ETI_MASK     0x00000400UL
#define CSR5_ETI_SHIFT    10

 /* Receive buffer unavailable */

#define CSR5_RU_OFFSET   0x28
#define CSR5_RU_MASK     0x00000080UL
#define CSR5_RU_SHIFT    7

 /* Transmit process stopped */
#define CSR5_TPS_OFFSET   0x28
#define CSR5_TPS_MASK     0x00000002UL
#define CSR5_TPS_SHIFT    1



/* Abnormal interrupt summary */
#define CSR5_INT_BITS	(CSR5_NIS_MASK | CSR5_AIS_MASK | CSR5_ERI_MASK | \
	CSR5_GTE_MASK | CSR5_ETI_MASK | CSR5_RPS_MASK | CSR5_RU_MASK | \
	CSR5_RI_MASK | CSR5_UNF_MASK | CSR5_TU_MASK | CSR5_TPS_MASK | CSR5_TI_MASK)



#define CSR6_SR			(1 << 1)
#define CSR6_PR			(1 << 6)
#define CSR6_PM			(1 << 7)
#define CSR6_FD			(1 << 9)
#define CSR6_ST			(1 << 13)
#define CSR6_SF			(1 << 21)
#define CSR6_TTM		(1 << 22)


 /* Receive all */
#define CSR6_RA_OFFSET   0x30
#define CSR6_RA_MASK     0x40000000UL
#define CSR6_RA_SHIFT    30


/* Interrupt enable register */
#define CSR7_TIE  (1)
#define CSR7_TSE  (1 << 1)
#define CSR7_RIE  (1 << 6)
#define CSR7_RUE  (1 << 7)
#define CSR7_RSE  (1 << 8)
#define CSR7_AIE  (1 << 15)
#define CSR7_NIE  (1 << 16)


#define CSR9_MDC		(1 << 16)
#define CSR9_MDO		(1 << 17)
#define CSR9_MDEN		(1 << 18)
#define CSR9_MDI		(1 << 19)


/* Descriptor flags */
#define DESC_OWN		(1 << 31)
#define DESC_TES		(1 << 15)
#define DESC_TLO		(1 << 11)
#define DESC_TNC		(1 << 10)
#define DESC_TLC		(1 << 9)
#define DESC_TEC		(1 << 8)
#define DESC_TUF		(1 << 1)
#define DESC_TLS		(1 << 30)
#define DESC_TFS		(1 << 29)
#define DESC_SET		(1 << 27)
#define DESC_TCH		(1 << 24)
#define DESC_RES		(1 << 15)
#define DESC_RFS		(1 << 9)
#define DESC_RLS		(1 << 8)


/* Link status */
#define LINK_UP			0x01
#define LINK_FD			0x02
#define LINK_100		0x04
#define TX_RX_ENABLED		0x10

/* Timeouts */

#define TIMEOUT_UDELAY		500
#define TIMEOUT_LOOPS		10000

#define LINK_STAT_TIMEOUT	5   /* seconds */


/*Register access functions*/

#define read_reg(reg) (readl(bp->base + reg))
#define write_reg(reg, val) (writel(val, bp->base + reg))

/* PHY (Micrel KS8721) definitions */

/* MDIO command bits */
#define MDIO_ST			(1 << 14)
#define MDIO_READ		(1 << 13)
#define MDIO_WRITE		(1 << 12) | (1 << 1)
#define MDIO_PHYADR_SHIFT	7
#define MDIO_REG_SHIFT		2

/* Compose an MDIO command */
#define mdio_cmd(op, reg) \
	(MDIO_ST | op | (pd->phy_id << MDIO_PHYADR_SHIFT) | \
	(reg << MDIO_REG_SHIFT))

/* Wait a half of MDC period (200 ns). Maximum possible frequency is 2.5 MHz. */
#define mdio_wait ndelay(200)

/* Get/Set MDC and MDIO pins */
#define mii_set_mdc(val) { \
    if (val) { \
	write_reg(CSR9, read_reg(CSR9) | CSR9_MDC); \
    } else { \
	write_reg( CSR9, read_reg(CSR9) & ~CSR9_MDC); \
    } \
}

#define mii_set_mdio(val) { \
    if (val) { \
	write_reg(CSR9, read_reg(CSR9) | CSR9_MDO); \
    } else { \
	write_reg(CSR9, read_reg(CSR9) & ~CSR9_MDO); \
    } \
}


#define mii_get_mdio() (read_reg(CSR9) & CSR9_MDI)

/* PHY registers */
#define PHY_BCR			0
#define PHY_BSR			1
#define PHY_ID1			2

/* PHY register bits */
#define BCR_SR			(1 << 15)
#define BCR_SS			(1 << 13)
#define BCR_ANE			(1 << 12)
#define BCR_RAN			(1 << 9)
#define BCR_DM			(1 << 8)
#define BSR_LS			(1 << 2)
#define BSR_ANC			(1 << 5)


#define RX_RING_SIZE 4
#define TX_RING_SIZE 2

/**
 * Default MAC address
 */

#define DEFAULT_MAC_ADDRESS             0xC0u,0xB1u,0x3Cu,0x88u,0x88u,0x88u


/*MORE REGS*/

/*------------------------------------------------------------------------------
 * CSR0_DSL:
 *   DSL field of register CSR0.
 *------------------------------------------------------------------------------
 * Descriptor skip length
 */
#define CSR0_DSL_OFFSET   0x00
#define CSR0_DSL_MASK     0x0000007CuL
#define CSR0_DSL_SHIFT    2

/*------------------------------------------------------------------------------
 * CSR0_TAP:
 *   TAP field of register CSR0.
 *------------------------------------------------------------------------------
 * Transmit automatic polling
 */
#define CSR0_TAP_OFFSET   0x00
#define CSR0_TAP_MASK     0x000E0000UL
#define CSR0_TAP_SHIFT    17


/*--------------------------------------------------------------*/
/*
 * Allowed values for CSR0_TAP:
 *------------------------------------------------------------------------------
 * TAP_DISABLED:   TAP disabled
 * TAP_819US:      TAP 819/81.9us
 * TAP_2450US:     TAP 2450/245us
 * TAP_5730US:     TAP 5730/573us
 * TAP_51_2US:     TAP 51.2/5.12us
 * TAP_102_4US:    TAP 102.4/10.24us
 * TAP_153_6US:    TAP 156.6/15.26us
 * TAP_358_4US:    TAP 358.4/35.84us
 */
#define TAP_DISABLED    0x0
#define TAP_819US       0x1
#define TAP_2450US      0x2
#define TAP_5730US      0x3
#define TAP_51_2US      0x4
#define TAP_102_4US     0x5
#define TAP_153_6US     0x6
#define TAP_358_4US     0x7
/*------------------------------------------------------------------------------*/


/**
 * Size of the max packet that can be received/transmited.
 */
#define MSS_MAX_PACKET_SIZE  1514uL

/**
 * Size of a receive/transmit buffer.
 * Buffer size must be enough big to hold a full frame and must be multiple of
 * four. For rx buffer +4 bytes allocated for crc values. These bytes doesn't
 * copied to the user buffer.
 */
#define MSS_TX_BUFF_SIZE  ((MSS_MAX_PACKET_SIZE + 3u) & (~(uint32_t)3))
#define MSS_RX_BUFF_SIZE  ((MSS_MAX_PACKET_SIZE + 7u) & (~(uint32_t)3))

/***************************************************************************//**
 * Buffer 2 size.
 * Indicates the size, in bytes, of memory space used by the second data buffer. This number must be a
 * multiple of four. If it is 0, Core10/100 ignores the second data buffer and fetches the next data descriptor.
 * This number is valid only when RDES1.24 (second address chained) is cleared.
 */
#define RDES1_RBS2_MASK		0x7FF
#define RDES1_RBS2_OFFSET	11

/***************************************************************************//**
 * Buffer 1 size
 * Indicates the size, in bytes, of memory space used by the first data buffer. This number must be a multiple of
 * four. If it is 0, Core10/100 ignores the first data buffer and uses the second data buffer.
 */
#define RDES1_RBS1_MASK		0x7FF
#define RDES1_RBS1_OFFSET	0

/***************************************************************************//**
 * Receive end of ring.
 * When set, indicates that this is the last descriptor in the receive descriptor ring. Core10/100 returns to the
 * first descriptor in the ring, as specified by CSR3 (start of receive list address).
 */
#define RDES1_RER   0x02000000UL

/***************************************************************************//**
 * Transmit end of ring.
 * When set, indicates the last descriptor in the descriptor ring.
 */
#define TDES1_TER     ((uint32_t)1 << 25)


#define RX_MSG_NUM 1

/* Driver functions */
static int core10100_probe(struct platform_device *);
static int core10100_remove(struct platform_device *);

/* netdev functions */
static int core10100_open(struct net_device *dev);
static int core10100_close(struct net_device *dev);
static struct net_device_stats *core10100_get_stats(struct net_device *dev);
static netdev_tx_t core10100_start_xmit(struct sk_buff *skb,
					struct net_device *dev);

static int core10100_ioctl(struct net_device *dev, struct ifreq *rq,
			   int cmd);

/* Receive/transmit descriptor */
struct rxtx_desc {
	unsigned int own_stat;
	unsigned int cntl_size;
	void *buf1;
	void *buf2;
};

/* struct core10100_ring { */
	
/* }; */

struct core10100_stat {
	u32 rx_interrupts;
	u32 tx_interrupts;
};

struct core10100_dev {
	void __iomem			*base;
	spinlock_t		        lock;
	struct platform_device		*pdev;
	struct net_device		*dev;
	unsigned int			capabilities; /* read from FPGA */
	/* struct napi_struct		napi; */
	uint8_t		flags;                  /**< Configuration of the driver*/

	/*device mac-address*/
	u16 mac[6];

	/* RX/TX descriptors */
	volatile struct rxtx_desc *rx_descs;
	volatile struct rxtx_desc *tx_descs;
	volatile struct rxtx_desc *tx_mac;

	/* Current transmit/receieve descriptors */
	u8 tx_cur;
	u8 rx_cur; 
	
	/* mac filter buffer */
	void *mac_filter;

	/* RX buffer pointer */
	void *rx_buf;

	/* RX/TX dma handles */
	dma_addr_t rx_dma_handle;
	dma_addr_t tx_dma_handle;
	dma_addr_t tx_mac_dma_handle;
	dma_addr_t rx_buf_dma_handle;

	struct completion rx_mac_completion;

	/* previous transmit skb, must be freed in ISR */
	struct sk_buff *tx_skb;

	/* received  skb */
	struct sk_buff *rx_skb;
	
	/*special mac filter buffer dma handle*/
	dma_addr_t mac_filter_dma_handle;
	
	/* PHY stuff */
	struct mii_bus			*mii_bus;
	struct mdiobb_ctrl core10100_mdio_ctrl;
	struct phy_device *phy_dev;

	unsigned char phy_id;	/* ID of the PHY */
	unsigned int			link;
	unsigned int			speed;
	unsigned int			duplex;

	/* statistics */
	struct core10100_stat statistics;

	
};


#define MAX_ETH_MSG_SIZE 1500
#define CORE_MAC_RX_BUF_SIZE 3000
#define MAX_NUM_ETH_RX_MSG 3000


/* MII access callbacks */
static void set_mdc(struct mdiobb_ctrl *ctrl, int level);
static void set_mdio_dir(struct mdiobb_ctrl *ctrl, int output);
static void set_mdio_data(struct mdiobb_ctrl *ctrl, int value);
static int get_mdio_data(struct mdiobb_ctrl *ctrl);


/* Set the Management Data Clock high if level is one,
 * low if level is zero.
 */
void set_mdc(struct mdiobb_ctrl *ctrl, int level)
{
	struct core10100_dev *bp = container_of(ctrl, struct core10100_dev,
						core10100_mdio_ctrl);
	
	mii_set_mdc(level);
}

/* Configure the Management Data I/O pin as an input if
 * "output" is zero, or an output if "output" is one.
 */
void set_mdio_dir(struct mdiobb_ctrl *ctrl, int output)
{
	struct core10100_dev *bp = container_of(ctrl, struct core10100_dev,
						core10100_mdio_ctrl);

	/* if (!output)  */
	if (output) 
		write_reg(CSR9, read_reg(CSR9) | CSR9_MDEN);
	else 
		write_reg(CSR9, read_reg(CSR9) & ~CSR9_MDEN);
}

/* Set the Management Data I/O pin high if value is one,
 * low if "value" is zero.  This may only be called
 * when the MDIO pin is configured as an output.
 */
void set_mdio_data(struct mdiobb_ctrl *ctrl, int value)
{
	struct core10100_dev *bp = container_of(ctrl, struct core10100_dev,
						core10100_mdio_ctrl);
	
	mii_set_mdio(value);
}

/* Retrieve the state Management Data I/O pin. */
int get_mdio_data(struct mdiobb_ctrl *ctrl)
{
	struct core10100_dev *bp = container_of(ctrl, struct core10100_dev,
						core10100_mdio_ctrl);

	return (mii_get_mdio() != 0);
}

/*
  Adapter initialization
*/

/* static void reset_eth(void) */
/* { */
/* 	unsigned int sfrst; */
	
/* 	sfrst = readl(SOFT_RST_CR); */
	
/* 	printk(KERN_INFO "read SOFT_RST_CR = 0x%x", sfrst); */
/* 	writel(sfrst & ~MAC_SR, SOFT_RST_CR); */

/* 	printk(KERN_INFO "wrote 0x%x to SOFT_RST_CR", sfrst & ~MAC_SR); */
/* 	printk(KERN_INFO "read SOFT_RST_CR = 0x%x", sfrst); */
/* } */

struct mdiobb_ops  core10100_mdio_ops = {
	.owner = THIS_MODULE,
	.set_mdc       = set_mdc,
	.set_mdio_dir  = set_mdio_dir,
	.set_mdio_data = set_mdio_data,
	.get_mdio_data = get_mdio_data
};


/* static int dnet_mdio_read(struct mii_bus *bus, int mii_id, int regnum); */
/* static int dnet_mdio_write(struct mii_bus *bus, int mii_id, int regnum, */
/* 			   u16 value); */

/* bp->mii_bus->read = &dnet_mdio_read; */
/* bp->mii_bus->write = &dnet_mdio_write; */
/* bp->mii_bus->reset = &dnet_mdio_reset; */


/* Stop transmission and receiving */
static void stop_tx_rx(	struct core10100_dev *bp)
{
	int i;

	/* Stop transmission and receiving */
	write_reg(CSR6, read_reg(CSR6) & ~(CSR6_ST | CSR6_SR));

	/* Wait for the transmission and receiving processes stopped */
	for (i = 0; i < TIMEOUT_LOOPS; i++) {
		if (((read_reg(CSR5) >> CSR5_TS_SHIFT) & CSR5_TS_MASK) ==
		    CSR5_TS_STOP &&
		    ((read_reg(CSR5) >> CSR5_RS_SHIFT) & CSR5_RS_MASK) ==
		    CSR5_RS_STOP) {
			break;
		}
		udelay(TIMEOUT_UDELAY);
		/* WDT_RESET; */
	}
	write_reg(CSR5, read_reg(CSR5) | CSR5_TPS);
	write_reg(CSR5, read_reg(CSR5) | CSR5_RPS);
}



static void core10100_adjust_link(struct net_device *dev)
{
	struct core10100_dev *bp = netdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;

	u8  tx_rx_stopped = 0;
	u32 status_change = 0;
	u32 flags;
	/* u8  link_stat; */

	printk(KERN_INFO "in adjust_link!");

	spin_lock_irqsave(&bp->lock, flags);
	
	if (phydev->link) {

		/* Check whether link duplex has been changed */
		if (bp->duplex != phydev->duplex) {

			/* TX/RX have to be stopped for updating */
			if (bp->flags & TX_RX_ENABLED) {
				stop_tx_rx(bp);
				tx_rx_stopped = 1;
			}
			
			if (phydev->duplex) {
				bp->flags |= LINK_FD;
				write_reg(CSR6, read_reg(CSR6) | CSR6_FD);
			}
			else {
				bp->flags &= ~LINK_FD;
				write_reg(CSR6, read_reg(CSR6) & ~CSR6_FD);
			}
			
			bp->duplex = phydev->duplex;
			status_change = 1;
		}
		
		/* Check whether link speed has been changed */
		if (bp->speed != phydev->speed) {
			/* TX/RX have to be stopped for updating */
			if (tx_rx_stopped != 1 && bp->flags & TX_RX_ENABLED){
				stop_tx_rx(bp);
				tx_rx_stopped = 1;
			}

			/* Update MAC register */
			if (phydev->speed == 100) {
				bp->flags |= LINK_100;
				write_reg(CSR6, read_reg(CSR6) | CSR6_TTM);
			} else if (phydev->speed == 10) {
				bp->flags &= ~LINK_100;
				write_reg(CSR6, read_reg(CSR6) & ~CSR6_TTM);
			}
			else
				printk(KERN_WARNING
				       "%s: Ack!  Speed (%d) is not "
				       "10/100/1000!\n", dev->name,
				       phydev->speed);

			status_change = 1;
			bp->speed = phydev->speed;
		}
	}

	/* Check whether link up/down has been changed */
	if (phydev->link != bp->link) {
		if (phydev->link) {
			bp->flags |= LINK_UP;
			/* Perform auto-negotiation */
			/* phy_auto_negotiation(pd); */
		} else {
			bp->flags &= ~LINK_UP;
			bp->speed = 0;
			bp->duplex = -1;
		}
		
		bp->link = phydev->link;

		status_change = 1;
	}

	spin_unlock_irqrestore(&bp->lock, flags);

	if (status_change) {
		if (phydev->link)
			printk(KERN_INFO "%s: link up (%d/%s)\n",
			       dev->name, phydev->speed,
			       DUPLEX_FULL == phydev->duplex ? "Full" : "Half");
		else
			printk(KERN_INFO "%s: link down\n", dev->name);
	}
	
	/* If TX/RX has been stopped, start them */
	if (tx_rx_stopped)
		write_reg(CSR6, read_reg(CSR6) | CSR6_ST | CSR6_SR);

}

static int core10100_mii_init(struct net_device *dev)
{
	struct core10100_dev *bp = netdev_priv(dev);
	int ret;
	short phy_addr;
	struct phy_device *phydev = NULL;

	bp->mii_bus = alloc_mdio_bitbang(&bp->core10100_mdio_ctrl);

	if (!bp->mii_bus) {
		printk(KERN_INFO "alloc_mdio_bitbang failed!");
	}

	
	bp->mii_bus->name = "eth_mii_bus";	
	snprintf(bp->mii_bus->id, MII_BUS_ID_SIZE, "%x", 0);

	ret = mdiobus_register(bp->mii_bus);
	
	if (ret) {
		printk(KERN_INFO "mdiobus_register failed!");
	}

	
	/* find the first phy */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (bp->mii_bus->phy_map[phy_addr]) {
			phydev = bp->mii_bus->phy_map[phy_addr];
			printk(KERN_INFO "found PHY: id: 0x%x addr %d",
			       phydev->phy_id, phydev->addr);
			 break;
		}
	}
	
	if (!phydev) {
		printk(KERN_ERR "no PHY found\n");
		 return -ENODEV; 
		/* bp->phy_id = MSS_PHY_ADDRESS_AUTO_DETECT; */
	}


	phydev = phy_connect(dev, dev_name(&phydev->dev),
			     &core10100_adjust_link, 0,
			     PHY_INTERFACE_MODE_RMII);


	if (IS_ERR(phydev)) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		return PTR_ERR(phydev);
	}

	
	phydev->supported &= PHY_BASIC_FEATURES;


	bp->phy_id = phydev->phy_id;
	bp->link = 0;
	bp->speed = 0;
	bp->duplex = -1;
	bp->phy_dev = phydev;

	return 0;

 	/* printk(KERN_INFO "found PHY: id: %d", phydev->phy_id); */
}

/* TODO: get it */
#define FRAME_LEN 1500

static inline u8 find_next_desc(unsigned char cur, unsigned char size)
{
	cur++;
	
	if(cur >= size) {
		return 0;
	}
	
	return cur;
}

/* handle (not) received frame */
static short rx_handler(struct core10100_dev *bp)
{
	u32 i;

	u8 size = 0;
	
	/*
	  Check whether Core10/100 returns the descriptor to the host
	  i.e. a packet is received.
	*/
	for (i = 0; i < RX_MSG_NUM; i++) {
		bp->rx_cur = find_next_desc(bp->rx_cur, RX_MSG_NUM);
		
		if(!(bp->rx_descs[bp->rx_cur].own_stat & DESC_OWN)) {
			break;
		}
	}
	
	if (i == RX_MSG_NUM) {
		printk(KERN_INFO "Bad RX num!\n");
		return 0;

	}
	
	/*
	  Check that the descriptor contains the whole packet,
	  i.e. the fist and last descriptor flags are set.
	*/
	if (!(bp->rx_descs[bp->rx_cur].own_stat & DESC_RFS) ||
	    !(bp->rx_descs[bp->rx_cur].own_stat & DESC_RLS)) {
		printk(KERN_INFO "receive_frame error: not whole packet");

		goto end;
	}

	/* The DESC_RES bit is valid only when the DESC_RLS is set */
	if(bp->rx_descs[bp->rx_cur].own_stat & DESC_RES) {
		/* Chesk status: may be status cache is out of sync */
		/* link_stat(pd); */
		printk(KERN_INFO "receive_frame error: DESC_RES flag is set");
		goto end;
	}

	/* Check the received packet size */
	size = (bp->rx_descs[bp->rx_cur].own_stat >> 16) & 0x3fff;
	if (size > FRAME_LEN) {
		/* Drop the packet */
		/* size = 0; */
		printk(KERN_INFO "frame length > %d", FRAME_LEN);
		goto end;
	}

end:

		
	netif_receive_skb(bp->rx_skb);

	/* alloc skb for future rx-ed frame */
	bp->rx_skb = dev_alloc_skb(FRAME_LEN);
	
	bp->rx_descs[bp->rx_cur].buf1 = bp->rx_skb->data; 

	/* Prepare the packet for the following receiving */
	bp->rx_descs[bp->rx_cur].cntl_size = DESC_TCH | FRAME_LEN;

	/* Give the descriptor ownership to Core */
	bp->rx_descs[bp->rx_cur].own_stat = DESC_OWN;

	/* Receive poll demand */
	write_reg(CSR2, 1);
	


	return 0;
	
}

static irqreturn_t core10100_interrupt (int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct core10100_dev *bp = netdev_priv(dev);
	unsigned int handled = 0;
	u32 intr_status;
	u8 rx_next;

	intr_status = read_reg(CSR5);
	
	if ( (intr_status & CSR5_NIS_MASK) != 0u ) {

		/* Transmit */
		if ( (intr_status & CSR5_TI_MASK) != 0u ) {
			
			bp->statistics.tx_interrupts++;
			/* events |= MSS_MAC_EVENT_PACKET_SEND; */

			printk(KERN_NOTICE "received TX irq");
			
			
			/* TODO: Этого достаточно ? */
			dev_kfree_skb(bp->tx_skb);
		}

		/* Receive  */
		if( (intr_status & CSR5_RI_MASK) != 0u ) {
			
			printk(KERN_NOTICE "received RX irq");
			
			/* skb = dev_alloc_skb(FRAME_LEN);  */
			rx_next = (bp->rx_cur + 1) % 2;
			
			/* bp->rx_descs[rx_next].buf1 = skb->data;  */
			
			/* netif_receive_skb(skb) and friends */
			bp->statistics.rx_interrupts++;
			/* events |= MSS_MAC_EVENT_PACKET_RECEIVED; */

			rx_handler(bp);
			
			if (bp->rx_skb != NULL) {
				netif_receive_skb(bp->rx_skb);
			} else {
				printk(KERN_NOTICE
				       "%s: No memory to allocate a sk_buff of "
				       "size %u.\n", dev->name, FRAME_LEN);
				
				return IRQ_RETVAL(handled);
			}


		}
	}


	printk(KERN_ERR "core10100: in irq");
	
	write_reg(CSR5, CSR5_INT_BITS);

	return IRQ_RETVAL(handled);
}

static int core10100_open(struct net_device *dev)
{
	struct core10100_dev *bp = netdev_priv(dev); 

	printk(KERN_NOTICE "in open");
	       
	
	/* if the phy is not yet register, retry later */
	/* if (!bp->phy_dev) */
	/* 	return -EAGAIN; */

	/* if (!is_valid_ether_addr(dev->dev_addr)) */
	/* 	return -EADDRNOTAVAIL; */

	/*
	napi_enable(&bp->napi);
	dnet_init_hw(bp);
	*/
	bp->link = 0;
	bp->duplex -1;
	bp->speed = -1;
	
	phy_start_aneg(bp->phy_dev);

	/* schedule a link state check */
	phy_start(bp->phy_dev);

	netif_start_queue(dev);


	return 0;
}

static int core10100_close(struct net_device *dev)
{
	/* struct core10100 *bp = netdev_priv(dev); */

	netif_stop_queue(dev);
	/*
	netif_stop_queue(dev);
	napi_disable(&bp->napi);

	if (bp->phy_dev)
		phy_stop(bp->phy_dev);

	dnet_reset_hw(bp);
	netif_carrier_off(dev);
	*/
	netif_carrier_off(dev);

	return 0;
}

static struct net_device_stats *core10100_get_stats(struct net_device *dev)
{
	return NULL;
}

static inline void core10100_print_skb(struct sk_buff *skb)
{
	int k;
	printk(KERN_DEBUG PFX "data:");
	for (k = 0; k < skb->len; k++)
		printk(" %02x", (unsigned int)skb->data[k]);
	printk("\n");
}

static netdev_tx_t core10100_start_xmit(struct sk_buff *skb,
					struct net_device *dev)

{
	u32 tx_status;
	/* u32 irq_enable; */
	u32 len, i; 
	unsigned long flags;
	u8 tx_next;
	
	struct core10100_dev *bp = netdev_priv(dev);

	pr_debug("start_xmit: len %u head %p data %p\n",
		 skb->len, skb->head, skb->data);
	
	core10100_print_skb(skb);


	/* <TODO>: core10100_init, intitial setup */
	/* frame size (words) */

	/*TODO: это именно длина сообщения? */
	len = skb->len;

	spin_lock_irqsave(&bp->lock, flags);

	tx_status = read_reg(CSR5);


	/* 
	Wait for the previous packet transmission end 
	pd->tx_cur - free descriptor
	tx_next - used in previous transmit operation
	*/
	tx_next = (bp->tx_cur + 1) % 2;

	for (i=0; i < TIMEOUT_LOOPS; i++) {
		if (!(bp->tx_descs[tx_next].own_stat & DESC_OWN)) {
			break;
		}
		udelay(TIMEOUT_UDELAY);
//		WDT_RESET;
	}

	if ( i == TIMEOUT_LOOPS ) {
		/* Chesk status: may be status cache is out of sync */
		/* link_stat(pd); */
		printk (KERN_ERR "transmit_frame error: timeout");
		return !0;
	    }
	
	/*
	Prepare the descriptors as follows:
	 - set the last descriptor flag
	 - set the first descriptor flag
	 - set the packet length
	*/
	
	bp->tx_descs[bp->tx_cur].cntl_size = DESC_TCH | DESC_TLS | DESC_TFS | len;

	/* make descriptor pointer to point to skb buf */
	bp->tx_descs[bp->tx_cur].buf1 = skb->data;
	
	/*
	  Give the current descriptor ownership to Core10/100 and
	  the following descriptor ownership to the host.
	*/

	bp->tx_descs[tx_next].own_stat = 0;
	bp->tx_descs[bp->tx_cur].own_stat = DESC_OWN;



	
	/* Start transmission */
	write_reg(CSR6, read_reg(CSR6) | CSR6_ST);

	/* Transmit poll demand */
	write_reg(CSR1, CSR1_TPD);
    

	/* save the buffer */
	bp->tx_skb = skb;
	

	spin_unlock_irqrestore(&bp->lock, flags);

	dev->trans_start = jiffies;

	bp->tx_cur = tx_next;
	
	return NETDEV_TX_OK;
}


static int core10100_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	/* struct core10100 *bp = netdev_priv(dev); */
	/* struct phy_device *phydev = bp->phy_dev; */

	/* if (!netif_running(dev)) */
	/* 	return -EINVAL; */

	/* if (!phydev) */
	/* 	return -ENODEV; */
	return 0;
	
}


int core10100_mac_addr(struct net_device *dev, void *p)
{
	int i;
	struct core10100_dev *bp = netdev_priv(dev);
	
	memcpy(bp->mac, p, sizeof(bp->mac));

	/* Fill all the entries of the mac filter */
	
	for (i = 0; i < 192; i += 12) {
		memcpy(bp->mac_filter + i, bp->mac, 12);
	}

	/*
	  Setup TX descriptor for the setup frame as follows:
	  - owned by Core
	  - chained
	  - buffer1 points to tx_buf
	  - buffer2 points to the descriptor itself
	  - perfect filtering
	  - length is 192
	*/
	
	bp->tx_mac->own_stat = DESC_OWN;
	bp->tx_mac->cntl_size = DESC_TCH | DESC_SET | 192;
	bp->tx_mac->buf1 = bp->mac_filter;
	bp->tx_mac->buf2 = (struct rxtx_desc *) bp->tx_mac;
	write_reg(CSR4, (unsigned long) bp->tx_mac);
	
	
	/* <TODO>: fix nasty busy loops to proper waits */
	
	/* Start transmission */
	write_reg(CSR6, read_reg(CSR6) | CSR6_ST);
	
	/* Wait for the packet transmission end */
	for (i = 0; i < TIMEOUT_LOOPS; i++) {
		/* Transmit poll demand */
		write_reg(CSR1, CSR1_TPD);
		/* Wait until Core10/100 returns the descriptor ownership */
		if (!(bp->tx_mac->own_stat & DESC_OWN)) {
			break;
		}
		udelay(TIMEOUT_UDELAY);
		/* WDT_RESET; */
	}
	
	if (i == TIMEOUT_LOOPS) {
		printk(KERN_ERR "MAC addr setup TX timeout");
		return !0;
	}

	/* Stop transmission */
	write_reg(CSR6, read_reg(CSR6) & ~CSR6_ST);
	/* Wait for the transmission process stopped */
	for (i = 0; i < TIMEOUT_LOOPS; i++) {
		if (((read_reg(CSR5) >> CSR5_TS_SHIFT) & CSR5_TS_MASK) ==
		    CSR5_TS_STOP) {
			break;
		}
		udelay(TIMEOUT_UDELAY);
		/* WDT_RESET; */
	}
    
    if (i == TIMEOUT_LOOPS) {
	    printk(KERN_ERR "Can not stop TX!");
	    return !0;
    }
    write_reg(CSR5, read_reg(CSR5) | CSR5_TPS);

    /* Restore the real TX descriptors pointers */
    write_reg(CSR4, (unsigned long)&bp->tx_descs[0]);
    /* pd->tx_cur = 0; */


    printk(KERN_INFO "MAC address is set up. (maybe)");
    return 0;
    
}

/*Init the adapter*/
static int core10100_init(struct core10100_dev *bp)
{
	int i;
	int ra_mask;
	/* unsigned long rd; */
	
	printk(KERN_INFO "-->core10100_init");
	
	 /* Reset the controller  */
	write_reg(CSR0,  read_reg(CSR0) | CSR0_SWR);
	
	/* Wait for reset  */
	for (i = 0; i < TIMEOUT_LOOPS; i++) {
		if (!(read_reg(CSR0) & CSR0_SWR)) {
			break;
		}
		udelay(TIMEOUT_UDELAY);
		/* WDT_RESET; */
	}
	
	if (i == TIMEOUT_LOOPS) {
		printk(KERN_INFO "core10100: SWR timeout");
		return !0;
	}
	
	/* reset_eth(); */
	
	 /* Setup the little endian mode for the data descriptors  */
	write_reg(CSR0, read_reg(CSR0) & ~CSR0_DBO);
	
	/*
	  Disable the promiscuous mode
	  Pass all multicast
	  Store and forward
	*/
	/* write_reg(CSR6, (read_reg(CSR6) & ~CSR6_PR) | CSR6_PM | CSR6_SF); */


	/* receive all (just for test) */

	ra_mask = read_reg(CSR6);

	/* if (ra_mask & CSR6_RA_MASK) */
	/* 	printk( KERN_INFO "Receive all is set!"); */
	/* else { */
	/* 	write_reg(CSR6, CSR6_RA_MASK); */


	/* 	ra_mask = read_reg(CSR6); */
		
	/* 	if (ra_mask & CSR6_RA_MASK) */
	/* 		printk( KERN_INFO "Can enable RA!!"); */
	/* } */

//	core10100_mac_addr(struct net_device *dev, void *p)

	bp->tx_cur = 0;
	
	printk(KERN_INFO "<--core10100_init");
	return 0;
}

static const struct net_device_ops core10100_netdev_ops = {
	.ndo_open		= core10100_open,
	.ndo_stop		= core10100_close,
	.ndo_get_stats		= core10100_get_stats,
	.ndo_start_xmit		= core10100_start_xmit,
	.ndo_do_ioctl		= core10100_ioctl,
	/* .ndo_set_mac_address	= eth_mac_addr, */
	.ndo_set_mac_address    = core10100_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
};


static int core10100_probe(struct platform_device *pd)
{
	/* unsigned int sz; */
	/* unsigned int rd; */
	struct net_device *dev;
	struct core10100_dev *bp;
	u32 mem_base, mem_size, a;
	u16 irq;
	const u8 mac_address[6] = { DEFAULT_MAC_ADDRESS };
	
	int err = -ENXIO;
	struct resource *res;
	
	printk(KERN_INFO "In probe!");

	res = platform_get_resource(pd, IORESOURCE_MEM, 0);
	if (!res) {
	     dev_err(&pd->dev, "no mmio resource defined\n");
	     goto err_out;
	}

	mem_base = res->start;
	mem_size = resource_size(res);
	
	irq = platform_get_irq(pd, 0);

	printk(KERN_INFO "Device irq: %d", irq);
	
	dev = alloc_etherdev(sizeof(*bp));
	
	if (!dev) {
	     dev_err(&pd->dev, "etherdev alloc failed, aborting.\n");
	     goto err_out;
	}

	dev->netdev_ops = &core10100_netdev_ops;

	bp = netdev_priv(dev);
	bp->dev = dev;
	
	
	SET_NETDEV_DEV(dev, &pd->dev);
	spin_lock_init(&bp->lock); 
	
	dev->irq = irq;
	
	err = request_irq(dev->irq, core10100_interrupt, 0, DRV_NAME, dev);

	if (err) {
		dev_err(&dev->dev, "Unable to request IRQ %d (error %d)\n",
			irq, err);
		goto err_out;
	}

	bp->base = ioremap(mem_base, mem_size);
	
	printk(KERN_INFO "bp base = 0x%x", (unsigned int) bp->base);

	if ( !(read_reg(CSR0) == 0xFE000000 &&
	       read_reg(CSR5) == 0xF0000000 &&
	       read_reg(CSR6) == 0x32000040))
	{
		return -ENODEV;
	}
	
	printk(KERN_INFO "CSR[0,5,6] reset values are OK.");

	bp->core10100_mdio_ctrl.ops = &core10100_mdio_ops;

	core10100_mii_init(dev);

	core10100_init(bp);

	random_ether_addr(dev->dev_addr);

	memcpy(dev->dev_addr, mac_address, sizeof(mac_address));
	
	err = register_netdev(dev);
	
	if (err) {
		dev_err(&pd->dev, "Cannot register net device, aborting.\n");
		goto err_out;
	}

	/* <TODO> верно ли это? */
	pd->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	

	/*alloc rx/tx descriptors in DMA-coherent memory*/
	
	bp->rx_descs =
		dma_alloc_coherent(&pd->dev,
				   sizeof(struct rxtx_desc) * RX_RING_SIZE,
				   &bp->rx_dma_handle,
				   GFP_DMA);

	bp->tx_descs =
		dma_alloc_coherent(&pd->dev,
				   sizeof(struct rxtx_desc) * TX_RING_SIZE,
				   &bp->tx_dma_handle,
				   GFP_DMA);

	/*alloc mac tx descriptor and buffer*/
	
	bp->tx_mac =
		dma_alloc_coherent(&pd->dev,
				   sizeof(struct rxtx_desc),
				   &bp->tx_mac_dma_handle,
				   GFP_DMA);

	
	bp->mac_filter =
		dma_alloc_coherent(&pd->dev,
				   192,
				   &bp->mac_filter_dma_handle,
				   GFP_DMA);

	/* bp->rx_buf =  */
	/* 	dma_alloc_coherent(&pd->dev, */
	/* 			   1500, */
	/* 			   &bp->rx_buf_dma_handle, */
	/* 			   GFP_DMA); */

	
	if (!(bp->rx_descs && bp->tx_descs
	      && bp->mac_filter && bp->tx_mac)) {
		dev_err(&pd->dev, "unable to alloc dma memory!\n");
		goto err_out;
	}

	/* alloc skb for first received frame */
	bp->rx_skb = dev_alloc_skb(FRAME_LEN);

	/* No automatic polling */
	write_reg(CSR0, read_reg(CSR0) &~ CSR0_TAP_MASK);
	
	/* No space between descriptors */
	/* write_reg(CSR0, read_reg(CSR0) &~ CSR0_DSL_MASK); */
	
#define	CFG_MAX_ETH_MSG_SIZE 1500

	/*
	  Setup RX descriptor as follows (the only descriptor is used):
	  - owned by Core
	  - chained
	  - buffer size is CFG_MAX_ETH_MSG_SIZE_ALIGNED
	  - buffer1 points to rx_buf
	  - buffer2 points to the descriptor itself
	*/
	
	bp->rx_cur = 0;

	for(a = 0; a < RX_MSG_NUM; a++ )
	{
		/* Give the ownership to the MAC */
		bp->rx_descs[a].own_stat = DESC_OWN;

		/*
		  The size field of the descriptor is 10 bits in size,
		  so lets check that the
		  CFG_MAX_ETH_MSG_SIZE is not bigger than 2047
		*/
 
		bp->rx_descs[a].cntl_size =
			DESC_TCH |
			(CFG_MAX_ETH_MSG_SIZE > 0x7FF ?
			 0x7FF : CFG_MAX_ETH_MSG_SIZE);
		
		bp->rx_descs[a].buf1 = (struct rxtx_desc *) bp->rx_skb->data;
		
		bp->rx_descs[a].buf2 =	(struct rxtx_desc *) bp->rx_descs;

	}

	
	write_reg(CSR3, (u32) bp->rx_descs);
	
	/*
	  Setup TX descriptors as follows (two descriptor are used,
	  refer to the Core10/100 header file (core_mac.h) for details):
	  - chained
	  - buffer1  will be set later to skb->data
	  - buffer2 points to the following itself
	*/

	/* bp->tx_desc[0].buf1 = bp->tx_buf; */
	bp->tx_descs[0].buf2 = (struct rxtx_desc *) &bp->tx_descs[1];
	/* bp->tx_desc[1].buf1 = bp->tx_buf; */
	bp->tx_descs[1].buf2 = (struct rxtx_desc *) &bp->tx_descs[0];
	bp->tx_cur = 0;
	
	write_reg(CSR4, (u32) bp->tx_descs);

	/* enable normal interrupts */
	write_reg(CSR7, CSR7_NIE | read_reg(CSR7));

	/* enable tx and rx interrupts */
	write_reg(CSR7, CSR7_RIE | read_reg(CSR7));
	write_reg(CSR7, CSR7_TIE | read_reg(CSR7));
	
	
	/* setup mac address */
	
	core10100_mac_addr(dev, (void *) mac_address);

	/* receive all packets */
	/* write_reg(CSR6, CSR6_RA_MASK); */
	
	
	/* Start transmission and receiving */
	write_reg(CSR6, read_reg(CSR6) | CSR6_ST | CSR6_SR);
	bp->flags |= TX_RX_ENABLED;

	
	return 0;
	
err_out:
	return err;
}

static int core10100_remove(struct platform_device *pd)
{
	return 0;
}


static struct platform_driver core10100_platform_driver = {
	.probe = core10100_probe,
	.remove = core10100_remove,
	/* .init = core10100_init, */
	/* .exit = core10100_exit */
	.driver = {
		.name = "core10100",
		.owner = THIS_MODULE
	}
};



/* Receive/transmit descriptor */

/* static struct desc { */
/* 	volatile unsigned int own_stat; */
/* 	volatile unsigned int cntl_size; */
/* 	volatile void *buf1; */
/* 	volatile void *buf2; */
/*  }; */


static int __init core10100_modinit(void) {
	printk(KERN_INFO "core10100 entry\n");

	platform_driver_register(&core10100_platform_driver);
	
	return 0;
}

static void __exit core10100_modexit(void) {
	printk(KERN_INFO "core10100 unload\n");
	platform_driver_unregister(&core10100_platform_driver);
}

module_init(core10100_modinit);
module_exit(core10100_modexit);
