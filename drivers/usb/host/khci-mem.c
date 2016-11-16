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

	/*
	 * Add EP to the appropriate list
	 */
	kep->type = usb_endpoint_type(&urb->ep->desc);
	switch (kep->type) {
	case USB_ENDPOINT_XFER_CONTROL:
		list_add_tail(&kep->node, &khci->ctrl_lst);
		break;
	case USB_ENDPOINT_XFER_INT:
		list_add_tail(&kep->node, &khci->intr_lst);
		break;
	case USB_ENDPOINT_XFER_BULK:
		list_add_tail(&kep->node, &khci->bulk_lst);
		break;
	default:
		printk("%s: EP type (%d) not supported\n", __func__,
			kep->type);
		kfree(kep);
		kep = NULL;
		goto out;
	}

	kep->plen = usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe));
	kep->hep->hcpriv = kep;
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

	memset(&kurb->stat, 0, sizeof(kurb->stat));

	INIT_LIST_HEAD(&kurb->td_sched_lst);
	kurb->td_todo = kurb->td_done = 0;

	kurb->state = KHCI_URB_INIT;
	kurb->status = -ERANGE;

	kurb->urb = urb;
	kurb->kep = kep;
	kurb->nxt_msec = 0;

	kurb->dev_addr = usb_pipedevice(urb->pipe);
	kurb->ep_cfg = KHCI_EP_RETRYDIS | KHCI_EP_EPRXEN | KHCI_EP_EPTXEN |
		       KHCI_EP_EPHSHK;
	if (urb->dev->speed == USB_SPEED_LOW) {
		kurb->dev_addr |= KHCI_ADDR_LSEN;

		/*
		 * If the LS device is connected to the root hub directly
		 * then don't produce PRE_PID
		 */
		if (urb->dev->parent && !urb->dev->parent->parent)
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

	td->khci = kurb->kep->khci;
	td->kurb = kurb;

	td->tries = 0;
	switch (kurb->kep->type) {
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
	if (busy) {
		kep->state = KHCI_EP_DEL;
		goto out;
	}

	kep->hep->hcpriv = NULL;
	list_del(&kep->node);
	kfree(kep);

out:
	return busy;
}

/*
 * Free KHCI URB
 */
int khci_urb_free(struct khci_urb *kurb)
{
	struct khci_td	*td, *tmp;
	struct urb	*urb = kurb->urb;
	int		busy = 0;

	/*
	 * Delete URB from EP's URB list if necessary
	 */
	if (kurb->state != KHCI_URB_DEL)
		list_del(&kurb->node);

	/*
	 * Clean-up queued TDs
	 */
	list_for_each_entry_safe(td, tmp, &kurb->td_sched_lst, node) {
		kurb->td_done++;
		list_del_init(&td->node);
		khci_td_free(td);
	}

	/*
	 * Check if some TDs are executed right now, so URB can't be
	 * deleted
	 */
	if (kurb->td_done < kurb->td_todo) {
		busy = 1;
		kurb->state = KHCI_URB_DEL;
		goto out;
	}

	/*
	 * Callback may re-submit this URB, so update it before giveback()
	 */
	urb->hcpriv = NULL;

	/*
	 * If we returned from enqueue(), then we must giveback() URB
	 */
	if (kurb->state != KHCI_URB_INIT) {
		struct khci_hcd	*khci = kurb->kep->khci;
		struct usb_hcd	*hcd = khci_to_hcd(khci);

		usb_hcd_unlink_urb_from_ep(hcd, urb);
		spin_unlock(&khci->lock);
		usb_hcd_giveback_urb(hcd, urb, kurb->status);
		spin_lock(&khci->lock);
	}
	kfree(kurb);
out:
	return busy;
}

/*
 * Free USB-FS TD
 */
void khci_td_free(struct khci_td *td)
{
	struct khci_hcd	*khci = td->khci;
	unsigned long	flags;

	spin_lock_irqsave(&khci->lock, flags);
	if (!list_empty(&td->node))
		dbg(0, "%s remove linked td(%p)\n", __func__, td);
	if (khci->td == td)
		khci->td = NULL;
	kfree(td);

	spin_unlock_irqrestore(&khci->lock, flags);
}
