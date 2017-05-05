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
#ifndef _KHCI_HCD_H_
#define _KHCI_HCD_H_

/*****************************************************************************
 * Include files
 *****************************************************************************/

#include <linux/usb.h>
#include <linux/usb/khci.h>
#include "../core/hcd.h"

/*
 * KHCI device descriptor
 */
struct khci_hcd {
	spinlock_t		lock;
	struct device		*dev;

	volatile struct khci_reg *reg;	/* Register map */

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

	struct list_head	ep_lst;

	struct workqueue_struct	*wq;
	struct work_struct	wrk;

	struct khci_td		*td;		/* Currently xfering TD */
	struct list_head	td_done_lst;	/* Completed TD */
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

	struct timeval		tm_run;
	struct timeval		tm_done;

	u8			dev_addr;
	u8			ep_addr;
	u8			ep_cfg;
	int			status;

	enum {
		KHCI_URB_IDLE	= 0,
		KHCI_URB_RUN,
		KHCI_URB_DEL,
		KHCI_URB_DONE,
		KHCI_URB_CLEAN
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

	int			tries;
	int			tries_max;

	struct timeval		tm_run;
	struct timeval		tm_done;
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

#endif /* _KHCI_HCD_H_ */
