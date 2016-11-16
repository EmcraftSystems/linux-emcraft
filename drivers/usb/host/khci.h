/*
 * Freescale USB-FS Host Controller (Kirin Host Controller) Driver header
 *
 * Copyright 2013 EmCraft Systems www.emcraft.com
 * Author: Yuri Tikhonov <yur@emcraft.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _KHCI_H_
#define _KHCI_H_

/*****************************************************************************
 * Include files
 *****************************************************************************/

#include <linux/usb.h>
#include <linux/hrtimer.h>
#include "../core/hcd.h"

/*****************************************************************************
 * Constants and macros:
 *****************************************************************************/

#undef DEBUG
#undef dbg

/*
 * Debug level here
 */
#define DEBUG			0
#if defined(DEBUG)
# define dbg(lvl, fmt...)	if (lvl <= DEBUG) printk("KHCI " fmt)
#else
# define dbg(lvl, fmt...)
#endif

/*
 * Buffer descriptor flags
 */
#define KHCI_BD_OWN		(1 << 7)
#define KHCI_BD_DATA_OFS	6
#define KHCI_BD_DATA_MSK	0x1
#define KHCI_BD_BC_OFS		16
#define KHCI_BD_BC_MSK		0x3FF

#define KHCI_BD_TOK_DATA0	0x3
#define KHCI_BD_TOK_DATA1	0xB
#define KHCI_BD_TOK_ACK		0x2
#define KHCI_BD_TOK_STALL	0xE
#define KHCI_BD_TOK_NAK		0xA
#define KHCI_BD_TOK_BUS_TOUT	0x0
#define KHCI_BD_TOK_DATA_ERR	0xF

#define KHCI_BD_DATA_SET(x)	(((x) & KHCI_BD_DATA_MSK) << KHCI_BD_DATA_OFS)
#define KHCI_BD_BC_SET(x)	(((x) & KHCI_BD_BC_MSK) << KHCI_BD_BC_OFS)

#define KHCI_BD_BC_GET(x)	(((x) >> KHCI_BD_BC_OFS) & KHCI_BD_BC_MSK)
#define KHCI_BD_TOK_GET(x)	(((x) >> 2) & 0xF)
#define KHCI_BD_FLG_SET(ln,dt)	(KHCI_BD_DATA_SET(dt) |			       \
				 KHCI_BD_BC_SET(ln) |			       \
				 KHCI_BD_OWN)

#define KHCI_BD_PTR(hcd, ep, rxtx, odd)	(struct khci_bd *)		       \
	((u32)(hcd)->bdt | (((ep) & 0x0f) << 5) | (((rxtx) & 1) << 4) |	       \
	 (((odd) & 1) << 3))

/*
 * Register fields
 */
#define KHCI_INT_ATTACH		(1 << 6)
#define KHCI_INT_TOKDNE		(1 << 3)
#define KHCI_INT_SOFTOK		(1 << 2)
#define KHCI_INT_ERROR		(1 << 1)
#define KHCI_INT_RST		(1 << 0)

#define KHCI_STAT_EP(x)		((x) >> 4)
#define KHCI_STAT_TX(x)		((x) & (1 << 3) ? 1 : 0)
#define KHCI_STAT_ODD(x)	((x) & (1 << 2) ? 1 : 0)

#define KHCI_ERR_BTSERR		(1 << 7)
#define KHCI_ERR_DMAERR		(1 << 5)
#define KHCI_ERR_DFN8		(1 << 3)
#define KHCI_ERR_CRC16		(1 << 2)
#define KHCI_ERR_PIDERR		(1 << 0)

/*
 * Don't process the following (like in MQX driver):
 * - BTOERR: happen if e.g. connect mouse
 * - CRC5EOF: these shouldn't happen with our SOFTHLD scheduling
 */
#define KHCI_ERR_MSK		(KHCI_ERR_BTSERR | KHCI_ERR_DMAERR |	       \
				 KHCI_ERR_DFN8 | KHCI_ERR_CRC16 |	       \
				 KHCI_ERR_PIDERR)

#define KHCI_CTL_JSTATE		(1 << 7)
#define KHCI_CTL_SE0		(1 << 6)
#define KHCI_CTL_TOKENBUSY	(1 << 5)
#define KHCI_CTL_RESET		(1 << 4)
#define KHCI_CTL_HOSTMODEEN	(1 << 3)
#define KHCI_CTL_ODDRST		(1 << 1)
#define KHCI_CTL_USBENSOFEN	(1 << 0)

#define KHCI_ADDR_LSEN		(1 << 7)

#define KHCI_TOKEN_OUT		(0x1 << 4)
#define KHCI_TOKEN_IN		(0x9 << 4)
#define KHCI_TOKEN_SETUP	(0xD << 4)
#define KHCI_TOKEN_MSK		(0xF << 4)

#define KHCI_EP_HOSTWOHUB	(1 << 7)
#define KHCI_EP_RETRYDIS	(1 << 6)
#define KHCI_EP_EPCTLDIS	(1 << 4)
#define KHCI_EP_EPRXEN		(1 << 3)
#define KHCI_EP_EPTXEN		(1 << 2)
#define KHCI_EP_EPHSHK		(1 << 0)

#define KHCI_USBCTRL_SUSP	(1 << 7)
#define KHCI_USBCTRL_PDE	(1 << 6)

/*****************************************************************************
 * C-types:
 *****************************************************************************/

/*
 * Register map
 */
struct khci_reg {
	u8	perid;		/* Peripheral ID Register */
	u8			rsv0[3];
	u8	idcomp;		/* Peripheral ID Complement Register */
	u8			rsv1[3];
	u8	rev;		/* Peripheral Revision Register */
	u8			rsv2[3];
	u8	addinfo;	/* Peripheral Additional Info Register */
	u8			rsv3[3];
	u8	otgistat;	/* OTG Interrupt Status Register */
	u8			rsv4[3];
	u8	otgicr;		/* OTG Interrupt Control Register */
	u8			rsv5[3];
	u8	otgstat;	/* OTG Status Register */
	u8			rsv6[3];
	u8	otgctl;		/* OTG Control Register */
	u8			rsv7[99];
	u8	istat;		/* Interrupt Status Register */
	u8			rsv8[3];
	u8	inten;		/* Interrupt Enable Register */
	u8			rsv9[3];
	u8	errstat;	/* Error Interrupt Status Register */
	u8			rsv10[3];
	u8	erren;		/* Error Interrupt Enable Register */
	u8			rsv11[3];
	u8	stat;		/* Status Register */
	u8			rsv12[3];
	u8	ctl;		/* Control Register */
	u8			rsv13[3];
	u8	addr;		/* Address Register */
	u8			rsv14[3];
	u8	bdtpage1;	/* BDT Page Register 1 */
	u8			rsv15[3];
	u8	frmnuml;	/* Frame Number Register Low */
	u8			rsv16[3];
	u8	frmnumh;	/* Frame Number Register High */
	u8			rsv17[3];
	u8	token;		/* Token Register */
	u8			rsv18[3];
	u8	softhld;	/* SOF Threshold Register */
	u8			rsv19[3];
	u8	bdtpage2;	/* BDT Page Register 2 */
	u8			rsv20[3];
	u8	bdtpage3;	/* BDT Page Register 3 */
	u8			rsv21[11];
	struct {
		u8	endpt;	/* Endpoint Control Register */
		u8		rsv0[3];
	} ep[16];
	u8	usbctrl;	/* USB Control Register */
	u8			rsv22[3];
	u8	observe;	/* USB OTG Observe Register */
	u8			rsv23[3];
	u8	control;	/* USB OTG Control Register */
	u8			rsv24[3];
	u8	usbtrc0;	/* USB Transceiver Control Register 0 */
	u8			rsv25[7];
	u8	usbfrmadjust;	/* Frame Adjust Register */
} __attribute__((packed));

/*
 * Buffer descriptor
 */
struct khci_bd {
	u32	flg;		/* Buffer descriptor flags KHCI_BD_xxx */
	u32	adr;		/* Buffer address */
} __attribute__((packed));

/*
 * KHCI device descriptor
 */
struct khci_hcd {
	spinlock_t		lock;
	struct device		*dev;

	volatile struct khci_reg *reg;	/* Register map */

	struct hrtimer		tmr;	/* INT timer */

	struct khci_bd		*bdt;	/* BD table (from DMA pool) */
	dma_addr_t		bdt_hdl;

	struct clk		*clk;	/* USB clock */

	unsigned long		cache_flags;

	struct {
		struct usb_hub_status	hub;
		struct usb_port_status	port;
	} vrh;				/* Virtual root hub descriptor */

	struct {
		int		setup;
		int		in;
		int		out;

		int		own;
		int		nak;
		int		bus;
		int		err;
		int		len;
	} stat;

	u8			bd[2];	/* 'odd' BD index (tx/rx) */

	struct list_head	ctrl_lst;	/* CONTROL EPs */
	struct list_head	intr_lst;	/* INTERRUPT EPs */
	struct list_head	bulk_lst;	/* BULK EPs */

	struct workqueue_struct	*wq;
	struct work_struct	wrk;

	struct khci_td		*td;		/* Currently xfering TD */
	struct list_head	td_done_lst;	/* Completed TD */
	int			td_proc;
};

/*
 * KHCI EP descriptor
 */
struct khci_ep {
	struct list_head	node;	/* Note in HCD's EP list */

	struct usb_host_endpoint *hep;
	struct khci_hcd		*khci;

	int			type;	/* EP type */
	u32			plen;	/* Max packet length */

	enum {
		KHCI_EP_IDLE	= 0,
		KHCI_EP_DEL
	}			state;

	struct list_head	urb_lst;/* Scheduled URBs */
};

/*
 * Private per-URB data
 */
struct khci_urb {
	struct list_head	node;	/* Node in EP's URB list */

	struct urb		*urb;
	struct khci_ep		*kep;	/* EP for this URB */
	u64			nxt_msec;

	u8			dev_addr;
	u8			ep_addr;
	u8			ep_cfg;
	int			status;

	enum {
		KHCI_URB_INIT	= 0,
		KHCI_URB_IDLE,
		KHCI_URB_RUN,
		KHCI_URB_DEL
	}			state;

	struct {
		int		own;
		int		nak;
		int		bus;
		int		err;
		int		len;
		int		ok;
        } stat;

	u32			td_todo;
	u32			td_done;

	struct list_head	td_sched_lst; /* Scheduled TDs */
};

/*
 * Transfer descriptor
 */
struct khci_td {
	struct list_head	node;	/* Node in td_*_lst lists */
	struct khci_urb		*kurb;
	struct khci_hcd		*khci;

	int			tries;
	int			tries_max;

	volatile struct khci_bd	*bd;

	u8			softhld;

	int			status;
	u8			istat;
	u8			stat;

	u32			org_flg;/* Original BD flags */

	u32			bd_flg;	/* BD flags (updated in IRQ) */
	u32			bd_adr;	/* Buffer address */
	u8			token;	/* Token reg val */

	u8			tx;	/* Tx/Rx direction */
	u8			err;
	u8			skip;

	struct {
		int		own;
		int		nak;
		int		bus;
		int		err;
		int		len;
	} retry;
} __attribute__((packed));

/*****************************************************************************
 * Inlines:
 *****************************************************************************/

static inline struct khci_hcd *hcd_to_khci(struct usb_hcd *hcd)
{
	return (struct khci_hcd *)(hcd->hcd_priv);
}

static inline struct usb_hcd *khci_to_hcd(struct khci_hcd *ufs)
{
	return container_of((void *)ufs, struct usb_hcd, hcd_priv);
}

/*****************************************************************************
 * Prototypes:
 *****************************************************************************/
/*
 * khci-sched.c
 */
irqreturn_t khci_hc_irq(struct usb_hcd *);
int khci_hc_urb_enqueue(struct usb_hcd *, struct urb *, gfp_t);
int khci_hc_urb_dequeue(struct usb_hcd *, struct urb *, int);
void khci_worker(struct work_struct *wrk);

/*
 * khci-hub.c
 */
int khci_hc_hub_status_data(struct usb_hcd *, char *);
int khci_hc_hub_control(struct usb_hcd *, u16, u16, u16, char *, u16);

/*
 * khci-mem.c
 */
struct khci_ep *khci_ep_alloc(struct khci_hcd *khci, struct urb *kurb);
struct khci_urb *khci_urb_alloc(struct khci_hcd *khci, struct urb *kurb);
struct khci_td *khci_td_alloc(struct khci_urb *kurb);
int khci_ep_free(struct khci_ep *kep);
int khci_urb_free(struct khci_urb *kurb);
void khci_td_free(struct khci_td *td);

/*
 * khci-dbg.c
 */
void khci_dbg_dump_reg(struct khci_hcd *khci);
void khci_dbg_dump_buf(u8 *buf, u32 len);
char *khci_dbg_flg2tock(u32 bd_flg);

#endif /* _KHCI_H_ */
