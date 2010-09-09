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
#include <asm/io.h>
//#include <arm/io.h>


MODULE_LICENSE("GPL");

//#define ETH_BASE 0x40003000

//#define ADDR_CSR5 0x40003028

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
	unsigned char tx_cur;	/* Current transmit descriptor */
	desc_t rx_desc[MAX_NUM_ETH_RX_MSG] __attribute ((__aligned__(4))) ; //ALIGN(4);		/* Receive descriptors */
	unsigned char rx_cur;
	unsigned char tx_buf[MAX_ETH_MSG_SIZE] __attribute((__aligned__(4)));//ALIGN(4); /* Transmit buffer */
	unsigned char rx_buf[CORE_MAC_RX_BUF_SIZE] __attribute ((__aligned__(4))); //ALIGN(4); /* Receive buffers */
} core_mac_desc_t;

core_mac_desc_t core_desc = {
	.phy_id = 0xff
};

/*
  Register access routines
*/

static unsigned long read_reg(unsigned short reg)
{
	unsigned long rd;

	printk(KERN_INFO "trying to read *(0x%x + 0x%x = 0x%x)", core_base, reg, core_base + reg);
	rd =  readl(core_base + reg);
	return rd;
}

static void write_reg(unsigned short reg, unsigned long val)
{
	printk(KERN_INFO "trying to write 0x%x to  *(0x%x + 0x%x = 0x%x)", val, core_base, reg, core_base + reg);
	
	writel(val, core_base + reg);

	//outl(core_base + reg, val);
}

/*
  Adapter initialization
*/

static int core10100_init()
{
	int i;
	printk(KERN_INFO "-->core10100_init");

	/* Reset the controller */
	write_reg(CSR0, read_reg(CSR0) | CSR0_SWR);

	/* Wait for reset */
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



	/* Setup the little endian mode for the data descriptors */
	write_reg(CSR0, read_reg(CSR0) & ~CSR0_DBO);


	/*
	  Disable the promiscuous mode
	  Pass all multicast
	  Store and forward
	*/
	write_reg(CSR6, (read_reg(CSR6) & ~CSR6_PR) | CSR6_PM | CSR6_SF);


	return 0;
	
#ifdef TROLOLO
	
	/*
	  Setup RX descriptor as follows (the only descriptor is used):
	  - owned by Core
	  - chained
	  - buffer size is CFG_MAX_ETH_MSG_SIZE_ALIGNED
	  - buffer1 points to rx_buf
	  - buffer2 points to the descriptor itself
	*/
	pd->rx_cur = 0;
	for( i = 0; i < CFG_MAX_NUM_ETH_RX_MSG; i++ ) {
		pd->rx_desc[i].own_stat = DESC_OWN;
		/* The size field of the descriptor is 10 bits in size, so 
		   lets check that the CFG_MAX_ETH_MSG_SIZE is not bigger than
		   2047 */
		pd->rx_desc[i].cntl_size = DESC_TCH | (CFG_MAX_ETH_MSG_SIZE > 0x7FF ?
						       0x7FF : CFG_MAX_ETH_MSG_SIZE);
		pd->rx_desc[i].buf1 = &pd->rx_buf[i*CFG_MAX_ETH_MSG_SIZE_ALIGNED];
		pd->rx_desc[i].buf2 = &pd->rx_desc[core_mac_find_next_desc(i, CFG_MAX_NUM_ETH_RX_MSG)];
	}
	write_register(pd, CSR3, (unsigned long)&pd->rx_desc);

	/*
	  Setup TX descriptors as follows (two descriptor are used,
	  refer to the Core10/100 header file (core_mac.h) for details):
	  - chained
	  - buffer1 points to tx_buf
	  - buffer2 points to the following itself
	*/
	pd->tx_desc[0].buf1 = pd->tx_buf;
	pd->tx_desc[0].buf2 = &pd->tx_desc[1];
	pd->tx_desc[1].buf1 = pd->tx_buf;
	pd->tx_desc[1].buf2 = &pd->tx_desc[0];
	pd->tx_cur = 0;
	write_register(pd, CSR4, (unsigned long)&pd->tx_desc[0]);

	/* Setup the controller mac filter */
	if (setup_mac_filter(p, instance, mac)) {
		DBG_STR(PSTR(LOG_PREFIX " filter failure\n"));
		return !0;
	}

#ifndef CFG_NET_ETH_COREMAC_IGNORE_PHY
	/* Update link status */
	link_stat(pd);
#else
	/* Assume 100 MBit FD */
	write_register(pd, CSR6, read_register(pd, CSR6) | CSR6_FD | CSR6_TTM);
#endif

	/* Start transmission and receiving */
	write_register(pd, CSR6, read_register(pd, CSR6) | CSR6_ST | CSR6_SR);
	pd->flags |= TX_RX_ENABLED;

#endif

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
