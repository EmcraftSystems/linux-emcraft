/*
 * linux/drivers/net/core10100.c
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


//Register access functions
static unsigned long read_reg(unsigned short);
static void write_reg(unsigned short, unsigned long);

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


//Core10/100 memory base
void *core_base;

struct platform_driver core10100_platform_driver = {
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
typedef struct desc {
	volatile unsigned int own_stat;
	volatile unsigned int cntl_size;
	volatile void *buf1;
	volatile void *buf2;
} desc_t;

#define MAX_ETH_MSG_SIZE 1500
#define CORE_MAC_RX_BUF_SIZE 3000
#define MAX_NUM_ETH_RX_MSG 3000

typedef struct {
	unsigned char *base;	/* Base address of the Core10/100 IP block */
	unsigned char flags;	/* Generic and link status flags */
	unsigned char phy_id;	/* ID of the PHY */
	time_t link_stat_timer;	/* For less frequent PHY line status polling */
	//mac_addr_t mac[CFG_NUMBER_OF_LAN_CHANNELS];	/* MAC address(es) */
	char mac[12];
	desc_t tx_desc[2] __attribute ((__aligned__(4))); //ALIGN(4);		/* Two transmit descriptors */
//	unsigned char tx_cur;	/* Current transmit descriptor */
//	desc_t rx_desc[MAX_NUM_ETH_RX_MSG] __attribute ((__aligned__(4))) ; //ALIGN(4);		/* Receive descriptors */
//	unsigned char rx_cur;
//	unsigned char tx_buf[MAX_ETH_MSG_SIZE] __attribute((__aligned__(4)));//ALIGN(4); /* Transmit buffer */
//	unsigned char rx_buf[CORE_MAC_RX_BUF_SIZE] __attribute ((__aligned__(4))); //ALIGN(4); /* Receive buffers */
} core_mac_desc_t;

core_mac_desc_t core_desc = {
	.phy_id = 0xff
};


//MII access callbacks
void set_mdc(struct mdiobb_ctrl *ctrl, int level);
void set_mdio_dir(struct mdiobb_ctrl *ctrl, int output);
void set_mdio_data(struct mdiobb_ctrl *ctrl, int value);
int get_mdio_data(struct mdiobb_ctrl *ctrl);


struct mdiobb_ops  core_mdio_ops = {
	THIS_MODULE,
	set_mdc,
	set_mdio_dir,
	set_mdio_data,
	get_mdio_data
};


struct mdiobb_ctrl core_mdio_ctrl = {
	&core_mdio_ops
};

struct mii_bus *eth_mii_bus;
//= {
//	.name = "eth_mii",
//	.parent = ndev->dev
//	.irq = 
//};

/*
  Register access routines
*/

static unsigned long read_reg(unsigned short reg)
{
	unsigned long rd;

	rd =  readl(core_base + reg);

//	printk(KERN_INFO "read 0x%x from *(0x%x)", (unsigned int) rd, (unsigned int) (core_base + reg));
	return rd;
}

static void write_reg(unsigned short reg, unsigned long val)
{
//	printk(KERN_INFO "wrote 0x%x to *(0x%x)",(unsigned int) val,(unsigned int) (core_base + reg));
	
	writel(val, core_base + reg);

	//outl(core_base + reg, val);
}


/* Set the Management Data Clock high if level is one,
 * low if level is zero.
 */
void set_mdc(struct mdiobb_ctrl *ctrl, int level)
{
	mii_set_mdc(level);
}

/* Configure the Management Data I/O pin as an input if
 * "output" is zero, or an output if "output" is one.
 */
void set_mdio_dir(struct mdiobb_ctrl *ctrl, int output)
{
	//<TODO> check inverted
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
	mii_set_mdio(value);
}

/* Retrieve the state Management Data I/O pin. */
int get_mdio_data(struct mdiobb_ctrl *ctrl)
{
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
static unsigned short mii_read(core_mac_desc_t *pd, unsigned short reg)
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
static void mii_write(core_mac_desc_t *pd, unsigned short reg,
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
static unsigned char phy_init(core_mac_desc_t *pd)
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

static int mdio_init()
{
	int ret;
	int phy_addr;
	struct phy_device *phydev = NULL;
	
	eth_mii_bus = alloc_mdio_bitbang(&core_mdio_ctrl);
	
	eth_mii_bus->name = "eth_mii_bus";
	snprintf(eth_mii_bus->id, MII_BUS_ID_SIZE, "%x", 0);
	ret = mdiobus_register(eth_mii_bus);

	if (ret) {
		printk(KERN_INFO "mdiobus_register failed!");
	}

	/* find the first phy */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (eth_mii_bus->phy_map[phy_addr]) {
			phydev = eth_mii_bus->phy_map[phy_addr];
			break;
		}
	}
	
	if (!phydev) {
		printk(KERN_ERR "no PHY found\n");
		return -ENODEV;
	}

	printk(KERN_INFO "found PHY: id: %d", phydev->phy_id);
}

static int core10100_init()
{
	int i;
	unsigned long rd;
	
	printk(KERN_INFO "-->core10100_init");

	// Reset the controller 
	write_reg(CSR0, read_reg(CSR0) | CSR0_SWR);

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

	mdio_init();
	
//	reset_eth();
	
	// Setup the little endian mode for the data descriptors 
	write_reg(CSR0, read_reg(CSR0) & ~CSR0_DBO);


	/*
	  Disable the promiscuous mode
	  Pass all multicast
	  Store and forward
	*/
	write_reg(CSR6, (read_reg(CSR6) & ~CSR6_PR) | CSR6_PM | CSR6_SF);

	return 0;
	
	printk(KERN_INFO "<--core10100_init");
	return 0;
}


static int core10100_probe(struct platform_device *pd)
{
	unsigned int sz;
	unsigned int rd;

	
	printk(KERN_INFO "In probe!");

	
	sz = pd->resource[0].end - pd->resource[0].start;
	core_base = ioremap(pd->resource[0].start, sz); //<TODO> заменить на platform_get_resource

	core_desc.base = core_base;
	
//	printk(KERN_INFO "trying to access CSR5 (0x%x+0x%x = 0x%x)...", core_base, CSR5, core_base+CSR5);
	
	if ( (rd = read_reg(CSR5)) != 0xF0000000){
		return -ENODEV;
	}
	
	printk(KERN_INFO "read csr5 defval ==> device match");
	core10100_init();


//	printk(KERN_INFO "read 0x%x. BAD", rd);
	
	/*
	  unsigned int *pcsr5 = (unsigned int *) ADDR_CSR5;

	  if (*pcsr5 == 0xF0000000) {
	  printk(KERN_INFO "read csr5 defval!");
	  return 0;
	  }
	*/

	return 0;
		
}

static int core10100_remove(struct platform_device *pd)
{
	return 0;
}

static int core10100_modinit(void) {
	printk(KERN_INFO "core10100 entry\n");

	platform_driver_register(&core10100_platform_driver);
	
	return 0;
}

static void core10100_modexit(void) {
	printk(KERN_INFO "core10100 unload\n");
	platform_driver_unregister(&core10100_platform_driver);
}

module_init(core10100_modinit);
module_exit(core10100_modexit);
