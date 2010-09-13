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
#include <asm/io.h>
//#include <arm/io.h>


MODULE_LICENSE("GPL");

//#define ETH_BASE 0x40003000

//#define ADDR_CSR5 0x40003028


//ethernet mac reset flag
#define MAC_SR (4 << 1)

//Soft reset controller address
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

#define CSR6_SR			(1 << 1)
#define CSR6_PR			(1 << 6)
#define CSR6_PM			(1 << 7)
#define CSR6_FD			(1 << 9)
#define CSR6_ST			(1 << 13)
#define CSR6_SF			(1 << 21)
#define CSR6_TTM		(1 << 22)

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
//static u32 read_reg(u16);
//static void write_reg(u16, u32);

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


//Driver functions
static int core10100_probe(struct platform_device *);
static int core10100_remove(struct platform_device *);

//netdev functions
static int core10100_open(struct net_device *dev);
static int core10100_close(struct net_device *dev);
static struct net_device_stats *core10100_get_stats(struct net_device *dev);
static netdev_tx_t core10100_start_xmit(struct sk_buff *skb, struct net_device *dev);
static int core10100_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);

struct core10100_dev {
	void __iomem			*base;
	spinlock_t			lock;
	struct platform_device		*pdev;
	struct net_device		*dev;
//	struct dnet_stats		hw_stats;
	unsigned int			capabilities; /* read from FPGA */
	struct napi_struct		napi;
	char mac[12];

	/* PHY stuff */
	struct mii_bus			*mii_bus;
	unsigned char phy_id;	/* ID of the PHY */
	unsigned int			link;
	unsigned int			speed;
	unsigned int			duplex;
};


/*TODO: removeit Core10/100 memory base */
void *core10100_base;


#define MAX_ETH_MSG_SIZE 1500
#define CORE_MAC_RX_BUF_SIZE 3000
#define MAX_NUM_ETH_RX_MSG 3000


/* MII access callbacks */
static void set_mdc(struct mdiobb_ctrl *ctrl, int level);
static void set_mdio_dir(struct mdiobb_ctrl *ctrl, int output);
static void set_mdio_data(struct mdiobb_ctrl *ctrl, int value);
static int get_mdio_data(struct mdiobb_ctrl *ctrl);



//struct mii_bus *eth_mii_bus;

/*Register access routines*/

/* static u32 read_reg(u16 reg) */
/* { */
/* 	u32 rd; */

/* 	rd =  readl(core10100_base + reg); */

/* 	return rd; */
/* } */

/* static void write_reg(u16 reg, u32 val) */
/* { */
/* 	writel(val, core10100_base + reg); */
/* } */


/* Set the Management Data Clock high if level is one,
 * low if level is zero.
 */
void set_mdc(struct mdiobb_ctrl *ctrl, int level)
{
/* TODO: remove it */
	struct core10100_dev *bp = core10100_base;
	
	mii_set_mdc(level);
}

/* Configure the Management Data I/O pin as an input if
 * "output" is zero, or an output if "output" is one.
 */
void set_mdio_dir(struct mdiobb_ctrl *ctrl, int output)
{
	/* TODO: remove it */
	struct core10100_dev *bp = core10100_base;

	if (!output) 
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
	/* TODO: remove it */
	struct core10100_dev *bp = core10100_base;
	
	mii_set_mdio(value);
}

/* Retrieve the state Management Data I/O pin. */
int get_mdio_data(struct mdiobb_ctrl *ctrl)
{
	/* TODO: remove it */
	struct core10100_dev *bp = core10100_base;
	
	return mii_get_mdio();
}


#ifdef TROLOLO
static inline void enable_mdo(int enable)
{
    /* MSS MAC has MDEN bit inverted compared to CORE_10100 */

    if (!enable) {
	 write_reg(CSR9, read_register(pd, CSR9) | CSR9_MDEN);
    } else {
	 write_reg(CSR9, read_register(pd, CSR9) & ~CSR9_MDEN);
    }
}


/* Read a register value via MII */
static unsigned short mii_read(core10100_mac_desc_t *pd, unsigned short reg)
{
    unsigned short cmd;		/* MII command */
    unsigned short data;	/* Read data */
    unsigned short mask;	/* Auxiliary mask */
    int i;

    if (pd->phy_id == 0xff) {
	return 0;
    }

    /* Compose and send the command */
    cmd = mdio_cmd(MDIO_READ, reg);

    enable_mdo(pd, 1); /* enable transmit */

    mii_set_mdio(1);

    for (i = 0; i < 32; i++) {
	mii_set_mdc(0);
	mdio_wait;
	mii_set_mdc(1);
	mdio_wait;
    }

    for (mask = 0x8000; mask > 0; mask >>= 1) {
	mii_set_mdc(0);
	mdio_wait;
	mii_set_mdio((mask & cmd) ? 1 : 0);
	mii_set_mdc(1);
	mdio_wait;
    }

    /* Read a register value */
    enable_mdo(pd, 0); /* receive */
    for (data = 0, mask = 0x8000; mask > 0; mask >>= 1) {
	mii_set_mdc(0);
	mdio_wait;
	if (mii_get_mdio()) {
	    data |= mask;
	}
	mii_set_mdc(1);
	mdio_wait;
    }
    mii_set_mdc(0);

    return data;
}



/* Write a register value via MII */
static void mii_write(core10100_mac_desc_t *pd, unsigned short reg,
	unsigned short data)
{
     unsigned long cmd;		/* MII command */
     unsigned long mask;		/* Auxiliary mask */
     int i;

     if (pd->phy_id == 0xff) {
	  return;
     }

     /* Send the command and the new register value */
     cmd = ((unsigned long)mdio_cmd(MDIO_WRITE, reg)) << 16 | data;

     enable_mdo(pd, 1); /* enable transmit */

     mii_set_mdio(1);

     for (i = 0; i < 32; i++) {
	  mii_set_mdc(0);
	  mdio_wait;
	  mii_set_mdc(1);
	  mdio_wait;
     }

     for (mask = 0x80000000; mask > 0; mask >>= 1) {
	  mii_set_mdc(0);
	  mdio_wait;
	  mii_set_mdio((mask & cmd) ? 1 : 0);
	  mii_set_mdc(1);
	  mdio_wait;
     }
     mii_set_mdc(0);
}


/* Initialize PHY */
static unsigned char phy_init(core10100_mac_desc_t *pd)
{
     int i;
     unsigned short val;

     /* Probe (find) a PHY */
     for (i = 0; i < 32; i++) {
	  pd->phy_id = i;
	  val = mii_read(pd, PHY_ID1);
	  if (val != 0 && val != 0xffff) {
	       break;
	  }
	  DBG_STR("PHY ");
	  DBG_HEX(i);
	  DBG_STR(" read ");
	  DBG_HEX(val>>8);
	  DBG_HEX(val);
	  DBG_STR("\n");
	  WDT_RESET;
     }
     if (i == 32) {
	  /* Have not found a PHY */
	  pd->phy_id = 0xff;
	  return !0;
     }

     /* Software reset */
     mii_write(pd, PHY_BCR, BCR_SR);

     return 0;
}
#endif
/*
  Adapter initialization
*/

static void reset_eth()
{
	unsigned int sfrst;
	
	sfrst = readl(SOFT_RST_CR);

	printk(KERN_INFO "read SOFT_RST_CR = 0x%x", sfrst);
	writel(sfrst & ~MAC_SR, SOFT_RST_CR);

	printk(KERN_INFO "wrote 0x%x to SOFT_RST_CR", sfrst & ~MAC_SR);
	printk(KERN_INFO "read SOFT_RST_CR = 0x%x", sfrst);
}

struct mdiobb_ops  core10100_mdio_ops = {
	THIS_MODULE,
	set_mdc,
	set_mdio_dir,
	set_mdio_data,
	get_mdio_data
};


struct mdiobb_ctrl core10100_mdio_ctrl = {
	&core10100_mdio_ops
};


static int mdio_init(struct core10100_dev *bp)
{
	int ret;
	int phy_addr;
	struct phy_device *phydev = NULL;
	
	bp->mii_bus = alloc_mdio_bitbang(&core10100_mdio_ctrl);
	
	bp->mii_bus->name = "eth_mii_bus";
	
	
	snprintf(bp->mii_bus->id, MII_BUS_ID_SIZE, "%x", 0);
	
	ret = mdiobus_register(bp->mii_bus);
	
	if (ret) {
		printk(KERN_INFO "mdiobus_register failed!");
	}
	
	return 1;
	
	/* find the first phy */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (bp->mii_bus->phy_map[phy_addr]) {
			phydev = bp->mii_bus->phy_map[phy_addr];
			printk(KERN_INFO "found PHY: id: %d addr %d", phydev->phy_id, phydev->addr);
			//	break;
		}
	}
	
	if (!phydev) {
		printk(KERN_ERR "no PHY found\n");
		return -ENODEV;
	}

// 	printk(KERN_INFO "found PHY: id: %d", phydev->phy_id);
}

static int core10100_open(struct net_device *dev)
{
	struct core10100_dev *bp = netdev_priv(dev);

	/* if the phy is not yet register, retry later */
//	if (!bp->phy_dev)
//		return -EAGAIN;

	if (!is_valid_ether_addr(dev->dev_addr))
		return -EADDRNOTAVAIL;

	/*
	napi_enable(&bp->napi);
	dnet_init_hw(bp);

	phy_start_aneg(bp->phy_dev);


	phy_start(bp->phy_dev);

	netif_start_queue(dev);
	*/

	return 0;
}

static int core10100_close(struct net_device *dev)
{
	struct dnet *bp = netdev_priv(dev);

	/*
	netif_stop_queue(dev);
	napi_disable(&bp->napi);

	if (bp->phy_dev)
		phy_stop(bp->phy_dev);

	dnet_reset_hw(bp);
	netif_carrier_off(dev);
	*/


	return 0;
}

static struct net_device_stats *core10100_get_stats(struct net_device *dev)
{
	return NULL;
}

static netdev_tx_t core10100_start_xmit(struct sk_buff *skb, struct net_device *dev)

{
	return NETDEV_TX_OK;
}

static int core10100_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct dnet *bp = netdev_priv(dev);
//	struct phy_device *phydev = bp->phy_dev;

	if (!netif_running(dev))
		return -EINVAL;

//	if (!phydev)
//		return -ENODEV;
	return 0;
	
}




static int core10100_init(struct core10100_dev *bp)
{
	int i;
	unsigned long rd;
	
	printk(KERN_INFO "-->core10100_init");

	// Reset the controller 
	write_reg(CSR0,  read_reg(CSR0) | CSR0_SWR);
	
	// Wait for reset 
	for (i = 0; i < TIMEOUT_LOOPS; i++) {
		if (!(read_reg(CSR0) & CSR0_SWR)) {
			break;
		}
		udelay(TIMEOUT_UDELAY);
//		WDT_RESET;
	}
	if (i == TIMEOUT_LOOPS) {
		printk(KERN_INFO "core10100: SWR timeout");
		return !0;
	}

	
	mdio_init(bp);
	
//	reset_eth();
	
	// Setup the little endian mode for the data descriptors 
	write_reg(CSR0, read_reg(CSR0) & ~CSR0_DBO);


	/*
	  Disable the promiscuous mode
	  Pass all multicast
	  Store and forward
	*/
	write_reg(CSR6, (read_reg(CSR6) & ~CSR6_PR) | CSR6_PM | CSR6_SF);


	
	printk(KERN_INFO "<--core10100_init");
	return 0;
}

static const struct net_device_ops core10100_netdev_ops = {
	.ndo_open		= core10100_open,
	.ndo_stop		= core10100_close,
	.ndo_get_stats		= core10100_get_stats,
	.ndo_start_xmit		= core10100_start_xmit,
	.ndo_do_ioctl		= core10100_ioctl,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
};


static int core10100_probe(struct platform_device *pd)
{
	/* unsigned int sz; */
	unsigned int rd;
	struct net_device *dev;
	struct core10100_dev *bp;
	u32 mem_base, mem_size;
	
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

	
	/*<TODO> заменить на platform_get_resource*/	
	/* sz = pd->resource[0].end - pd->resource[0].start; */
	/* core10100_base = ioremap(pd->resource[0].start, sz);  */
	
	/* printk(KERN_INFO "trying to access CSR5 (0x%x+0x%x = 0x%x)...", core_base, CSR5, core_base+CSR5); */
	

	dev = alloc_etherdev(sizeof(*bp));
	
	if (!dev) {
		dev_err(&pd->dev, "etherdev alloc failed, aborting.\n");
		goto err_out;
	}

	dev->netdev_ops = &core10100_netdev_ops;

	bp = netdev_priv(dev);
	bp->dev = dev;

	/* bp->base = core10100_base; */

	
	SET_NETDEV_DEV(dev, &pd->dev);
	//spin_lock_init(&bp->lock);

	bp->base = ioremap(mem_base, mem_size);

	printk(KERN_INFO "bp base = 0x%x", (unsigned int) bp->base);

/* TODO: remove */
	core10100_base = bp->base;


	if ( (rd = read_reg(CSR5)) != 0xF0000000){
		return -ENODEV;
	}
	
	printk(KERN_INFO "read csr5 defval ==> device match");

	core10100_init(bp);

	err = register_netdev(dev);
	if (err) {
		dev_err(&pd->dev, "Cannot register net device, aborting.\n");
		goto err_out;
	}

err_out:
	return err;

	return 0;
		
}

static int core10100_remove(struct platform_device *pd)
{
	return 0;
}



static struct platform_driver core10100_platform_driver = {
	.probe = core10100_probe,
	.remove = core10100_remove,
//	.init = core10100_init,
//	.exit = core10100_exit
	.driver = {
		.name = "core10100",
		.owner = THIS_MODULE
	}
};



/* Receive/transmit descriptor */
static struct desc {
	volatile unsigned int own_stat;
	volatile unsigned int cntl_size;
	volatile void *buf1;
	volatile void *buf2;
 };


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
