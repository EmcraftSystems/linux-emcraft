/*
 * Freescale USB-FS OTG Host Controller Driver (Kirin Host Controller).
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

/*****************************************************************************
 * Include files:
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/list.h>

#include "khci.h"

/*****************************************************************************
 * Exported API:
 *****************************************************************************/

/*
 * Allocate KHCI EP descriptor
 */
struct khci_ep *khci_ep_alloc(struct khci_hcd *khci, struct urb *urb)
{
	struct khci_ep	*kep;

	kep = kmalloc(sizeof(struct khci_ep), GFP_ATOMIC);
	if (!kep)
		goto out;
	memset(kep, 0, sizeof(struct khci_ep));

	INIT_LIST_HEAD(&kep->urb_lst);

	kep->state = KHCI_EP_IDLE;
	kep->hep = urb->ep;
	kep->khci = khci;
	kep->type = usb_endpoint_type(&urb->ep->desc);

	kep->plen = usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe));

	kep->hep->hcpriv = kep;

	list_add_tail(&kep->node, &khci->ep_lst);
out:
	return kep;
}

/*
 * Allocate KHCI URB descriptor
 */
struct khci_urb *khci_urb_alloc(struct khci_hcd *khci, struct urb *urb)
{
	struct khci_ep	*kep = urb->ep->hcpriv;
	struct khci_urb	*kurb;

	kurb = kmalloc(sizeof(struct khci_urb), GFP_KERNEL);
	if (!kurb)
		goto out;

	urb->hcpriv = kurb;

	do_gettimeofday(&kurb->tm_run);
	memset(&kurb->tm_done, 0, sizeof(kurb->tm_done));

	memset(&kurb->stat, 0, sizeof(kurb->stat));

	INIT_LIST_HEAD(&kurb->td_sched_lst);
	kurb->td_todo = kurb->td_done = 0;

	kurb->state = KHCI_URB_IDLE;
	kurb->status = -ERANGE;

	kurb->urb = urb;
	kurb->kep = kep;

	kurb->dev_addr = usb_pipedevice(urb->pipe);
	kurb->ep_cfg = KHCI_EP_RETRYDIS | KHCI_EP_EPRXEN | KHCI_EP_EPTXEN |
		       KHCI_EP_EPHSHK;
	if (urb->dev->speed == USB_SPEED_LOW) {
		kurb->dev_addr |= KHCI_ADDR_LSEN;
		kurb->ep_cfg |= KHCI_EP_HOSTWOHUB;
	}
	if (kep->type != USB_ENDPOINT_XFER_CONTROL)
		kurb->ep_cfg |= KHCI_EP_EPCTLDIS;
	kurb->ep_addr = usb_pipeendpoint(urb->pipe);

	list_add_tail(&kurb->node, &kep->urb_lst);
out:
	return kurb;
}

/*
 * Allocate KHCI Transfer descriptor
 */
struct khci_td *khci_td_alloc(struct khci_urb *kurb)
{
	struct khci_td	*td;

	td = kmalloc(sizeof(struct khci_td), GFP_KERNEL);
	if (!td)
		goto out;

	td->kurb = kurb;

	td->tries = 0;
	switch (usb_endpoint_type(&kurb->urb->ep->desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
	case USB_ENDPOINT_XFER_BULK:
		td->tries_max = 32;
		break;
	case USB_ENDPOINT_XFER_INT:
	default:
		td->tries_max = 1;
		break;
	}

	td->skip = 0;

	td->retry.own = td->retry.nak = td->retry.bus = 0;
	td->retry.err = td->retry.len = 0;

	list_add_tail(&td->node, &kurb->td_sched_lst);
	kurb->td_todo++;
out:
	return td;
}

/*
 * Free KHCI EP descriptor
 */
int khci_ep_free(struct khci_ep *kep)
{
	struct khci_urb	*kurb, *tmp;
	int		busy = 0;

	/*
	 * Clean-up queued URBs
	 */
	list_for_each_entry_safe(kurb, tmp, &kep->urb_lst, node) {
		if (khci_urb_free(kurb))
			busy = 1;
	}

	/*
	 * Clean-up EP
	 */
	if (!busy)
		kfree(kep);
	else
		kep->state = KHCI_EP_DEL;

	return busy;
}

/*
 * Free KHCI URB
 */
int khci_urb_free(struct khci_urb *kurb)
{
	struct khci_td	*td, *tmp;
	int		busy = 0;

	/*
	 * Delete URB from EP's URB list if necessary
	 */
	switch (kurb->state) {
	case KHCI_URB_DEL:
		break;
	default:
		list_del(&kurb->node);
		break;
	}

	kurb->state = KHCI_URB_DEL;
	kurb->urb->hcpriv = NULL;

	/*
	 * Clean-up queued TDs
	 */
	list_for_each_entry_safe(td, tmp, &kurb->td_sched_lst, node) {
		kurb->td_done++;
		list_del(&td->node);
		khci_td_free(td);
	}

	/*
	 * Check if some TDs are executed right now, so URB can't be
	 * deleted
	 */
	if (kurb->td_done < kurb->td_todo)
		busy = 1;

	if (!busy)
		kfree(kurb);

	return busy;
}

/*
 * Free USB-FS TD
 */
void khci_td_free(struct khci_td *td)
{
	kfree(td);
}
