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

#if defined(CONFIG_ARCH_KINETIS)
#include <mach/cache.h>
#endif

#include "khci.h"

/*****************************************************************************
 * Static routines prototypes:
 *****************************************************************************/

static void khci_irq_attach(struct khci_hcd *khci, u8 istat);
static void khci_irq_tokdne(struct khci_hcd *khci, u8 istat);
static void khci_irq_error(struct khci_hcd *khci, u8 istat);
static void khci_irq_rst(struct khci_hcd *khci, u8 istat);
static void khci_irq_sof(struct khci_hcd *khci, u8 istat);

static int khci_queue_xfer_control(struct khci_hcd *khci, struct khci_urb *kurb);
static int khci_queue_xfer_bulk_int(struct khci_hcd *khci, struct khci_urb *kurb);

static void khci_ep_process(struct khci_hcd *khci);
static int khci_urb_process(struct khci_urb *kurb);

static void khci_td_complete(struct khci_hcd *khci, struct khci_td *td);

/*****************************************************************************
 * USB framework API:
 *****************************************************************************/

/*
 * HCD interrupt handler
 */
irqreturn_t khci_hc_irq(struct usb_hcd *hcd)
{
	struct khci_hcd		*khci = hcd_to_khci(hcd);
	u8			istat, inten;

	istat = khci->reg->istat;
	inten = khci->reg->inten;

	dbg(4, "IRQ ISTAT: %02x\n", istat);

	if (istat & KHCI_INT_ATTACH)
		khci_irq_attach(khci, istat);

	if (istat & KHCI_INT_TOKDNE)
		khci_irq_tokdne(khci, istat);

	if (istat & KHCI_INT_ERROR)
		khci_irq_error(khci, istat);

	if (istat & KHCI_INT_RST)
		khci_irq_rst(khci, istat);

	if ((istat & inten) & KHCI_INT_SOFTOK)
		khci_irq_sof(khci, istat);

	if (khci->td && khci->td->status != -EBUSY && !khci->td_proc) {
		khci->td_proc = 1;
		queue_work(khci->wq, &khci->wrk);
	} else if (khci->sof >= khci->sof_wrk) {
		khci->sof_wrk = (unsigned int)-1;
		queue_work(khci->wq, &khci->wrk);
	}

	return IRQ_HANDLED;
}

/*
 * Add new URB to the device queue
 */
int khci_hc_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct khci_hcd		*khci = hcd_to_khci(hcd);
	struct khci_urb		*kurb;
	struct khci_ep		*kep;
	unsigned long		flags;
	int			type = usb_endpoint_type(&urb->ep->desc);
	int			rv;

	dbg(1, "%s URB:%p,EP:%p.%p,Type:%d,Flags:%x\n", __func__,
	    urb, urb->ep, urb->ep->hcpriv, type, urb->transfer_flags);

	/*
	 * Don't support ISO for now
	 */
	if (type == USB_ENDPOINT_XFER_ISOC) {
		dbg(0, "ISOCHRONOUS transfers not supported\n");
		rv = -EINVAL;
		goto out;
	}

	spin_lock_irqsave(&khci->lock, flags);

	rv = usb_hcd_link_urb_to_ep(hcd, urb);
	if (rv) {
		dbg(0, "link URB fail %d\n", rv);
		goto out_spin;
	}

	kep = urb->ep->hcpriv ? urb->ep->hcpriv : khci_ep_alloc(khci, urb);
	if (!kep) {
		dbg(0, "failed get EP\n");
		rv = -ENOMEM;
		goto out_unlink;
	}

	kurb = khci_urb_alloc(khci, urb);
	if (!kurb) {
		dbg(0, "failed alloc URB\n");
		rv = -ENOMEM;
		goto out_unlink;
	}

	switch (kep->type) {
	case USB_ENDPOINT_XFER_CONTROL:
		rv = khci_queue_xfer_control(khci, kurb);
		break;
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		rv = khci_queue_xfer_bulk_int(khci, kurb);
		break;
	default:
		BUG();
		rv = -EINVAL;
		break;
	}

	if (rv) {
		khci_urb_free(kurb);
		goto out_spin;
	}

	kurb->state = KHCI_URB_IDLE;
	khci_ep_process(khci);
out_unlink:
	if (rv)
		usb_hcd_unlink_urb_from_ep(hcd, urb);
out_spin:
	spin_unlock_irqrestore(&khci->lock, flags);
out:
	return rv;
}

/*
 * Delete URB (previously enqueued) from the device queue
 */
int khci_hc_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct khci_hcd	*khci = hcd_to_khci(hcd);
	struct khci_urb	*kurb;
	unsigned long	flags;
	int		rv;

	dbg(1, "%s URB:%p(%d),EP:%p,Type:%d\n", __func__, urb, status, urb->ep,
	    usb_endpoint_type(&urb->ep->desc));

	spin_lock_irqsave(&khci->lock, flags);

	rv = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (rv)
		goto out;

	kurb = urb->hcpriv;
	if (kurb) {
		kurb->status = status;
		khci_urb_free(kurb);
	}
out:
	spin_unlock_irqrestore(&khci->lock, flags);

	return rv;
}

/*****************************************************************************
 * Exported helpers
 *****************************************************************************/

/*
 * Work-thread for processing complete TDs, and scheduling next xfers
 */
void khci_worker(struct work_struct *wrk)
{
	struct khci_hcd	*khci = container_of(wrk, struct khci_hcd, wrk);
	struct khci_urb	*kurb;
	struct khci_td	*td;
	unsigned long	flags;
	int		msec;
	u8		tok;

	spin_lock_irqsave(&khci->lock, flags);
	if (!khci->td) {
		/*
		 * SOF condition triggers this call
		 */
		goto retry;
	}

	td = khci->td;
	kurb = td->kurb;

	switch (td->status) {
	case 0:
		kurb->urb->actual_length += KHCI_BD_BC_GET(td->bd_flg);
		break;
	case -ENOMSG:
		/*
		 * NAK on INT xfer
		 */
		goto retry;
	case -EPIPE:
	case -ESHUTDOWN:
		/*
		 * Some unreconverable error
		 */
		break;
	case -EFAULT:
		/*
		 * Some error detected in ERRSTAT
		 */
		if (td->tries >= td->tries_max) {
			td->status = -EPROTO;
			break;
		}
		goto retry;
	case -EAGAIN:
		/*
		 * Err in BD: NAK, bus timeout ...
		 */
		tok = KHCI_BD_TOK_GET(td->bd_flg);
		msec = 5 * td->tries;

		if (tok == KHCI_BD_TOK_NAK) {
			td->retry.nak++;
			khci->stat.nak++;
			kurb->stat.nak++;
		} else if (tok == KHCI_BD_TOK_BUS_TOUT) {
			td->retry.bus++;
			khci->stat.bus++;
			kurb->stat.bus++;
			/*
			 * FIXME: don't retry if ISO pipe
			 */
		} else if (tok == KHCI_BD_TOK_DATA_ERR) {
			td->retry.err++;
			khci->stat.err++;
			kurb->stat.err++;
		} else {
			dbg(0, "%s: bad err tok %02x\n", __func__, td->bd_flg);
			break;
		}

		if (td->tries >= td->tries_max) {
			td->status = -EPROTO;
			break;
		}
		if (!msec)
			break;

		/*
		 * We keep khci->td set, so even if someone decide to do
		 * new xfer while we out of spin-lock - it will fail
		 */
		spin_unlock_irqrestore(&khci->lock, flags);
		msleep(msec);
		spin_lock_irqsave(&khci->lock, flags);
		goto retry;
	default:
		dbg(0, "%s: bad TD status %d\n", __func__, td->status);
		khci_dbg_dump_reg(khci);
		break;
	}

	/*
	 * TD complete.
	 * Free TD, complete current URB if necessary, and initiate
	 * next xfer
	 */
	list_move_tail(&td->node, &khci->td_done_lst);
	khci_td_complete(khci, td);
retry:
	khci->td = NULL;
	khci->td_proc = 0;
	khci_ep_process(khci);
	spin_unlock_irqrestore(&khci->lock, flags);
}

/*****************************************************************************
 * Functions local to this module:
 *****************************************************************************/

/*
 * Submit CONTROL transfer
 */
static int khci_queue_xfer_control(struct khci_hcd *khci, struct khci_urb *kurb)
{
	struct urb	*urb = kurb->urb;
	struct khci_ep	*kep = urb->ep->hcpriv;
	struct khci_td	*td;
	u8		*dbuf;
	u32		dlen;
	u8		tock;
	u8		ep_addr = kurb->ep_addr;
	int		rv, tgl, tx;

	dbg(3, "%s '%s' %d\n", __func__, usb_pipeout(urb->pipe) ? "out" : "in",
	    urb->transfer_buffer_length);

	/*
	 * Build Setup phase TD (always DATA0)
	 */
	td = khci_td_alloc(kurb);
	if (!td) {
		rv = -ENOMEM;
		goto out;
	}

	dlen = 8;
	dbuf = urb->setup_packet;

	td->tx = 1;
	td->token = KHCI_TOKEN_SETUP | ep_addr;

	tgl = 0;
	td->bd_adr = (u32)dbuf;
	td->bd_flg = KHCI_BD_FLG_SET(dlen, tgl);

	/*
	 * Build Data phase TDs (DATA1/0/1/...)
	 */
	if (usb_pipeout(urb->pipe)) {
		tx = 1;
		tock = KHCI_TOKEN_OUT;
	} else {
		tx = 0;
		tock = KHCI_TOKEN_IN;
	}

	dlen = urb->transfer_buffer_length;
	dbuf = urb->transfer_buffer;

	while (dlen) {
		u32	pkt;

		td = khci_td_alloc(kurb);
		if (!td) {
			rv = -ENOMEM;
			goto out;
		}

		pkt = dlen < kep->plen ? dlen : kep->plen;

		td->tx = tx;
		td->token = tock | ep_addr;

		tgl ^= 1;
		td->bd_adr = (u32)dbuf;
		td->bd_flg = KHCI_BD_FLG_SET(pkt, tgl);

		dlen -= pkt;
		dbuf += pkt;
	}

	/*
	 * Build Status phase TD (always DATA1)
	 */
	td = khci_td_alloc(kurb);
	if (!td) {
		rv = -ENOMEM;
		goto out;
	}

	if (tx) {
		td->tx = 0;
		td->token = KHCI_TOKEN_IN | ep_addr;
	} else {
		td->tx = 1;
		td->token = KHCI_TOKEN_OUT | ep_addr;
	}

	tgl = 1;
	td->bd_adr = 0;
	td->bd_flg = KHCI_BD_FLG_SET(0, tgl);

	rv = 0;
out:
	return rv;
}

/*
 * Submit BULK transfers
 */
static int khci_queue_xfer_bulk_int(struct khci_hcd *khci, struct khci_urb *kurb)
{
	struct urb	*urb = kurb->urb;
	struct khci_ep	*kep = urb->ep->hcpriv;
	struct khci_td	*td;
	u8		*dbuf = urb->transfer_buffer;
	u32		dlen = urb->transfer_buffer_length;
	u8		tock;
	u8		ep_addr = kurb->ep_addr;
	int		rv, tgl, num = 0, zlp, tx;

	dbg(3, "%s '%s' L:%d B:%p/0x%x\n",
	    __func__, usb_pipeout(urb->pipe) ? "out" : "in",
	    dlen, urb->transfer_buffer, urb->transfer_dma);

	/*
	 * Build Data phase TDs
	 */
	if (usb_pipeout(urb->pipe)) {
		tx = 1;
		tock = KHCI_TOKEN_OUT;
	} else {
		tx = 0;
		tock = KHCI_TOKEN_IN;
	}

	tgl = usb_gettoggle(urb->dev, ep_addr, tx);
	zlp = (urb->transfer_flags & URB_ZERO_PACKET) ? 1 : 0;

	do {
		u32	pkt;

		td = khci_td_alloc(kurb);
		if (!td) {
			rv = -ENOMEM;
			goto out;
		}

		pkt = dlen < kep->plen ? dlen : kep->plen;
		if (pkt == 0)
			zlp = 0;

		td->tx = tx;
		td->token = tock | ep_addr;
		td->bd_adr = (u32)dbuf;
		td->bd_flg = KHCI_BD_FLG_SET(pkt, tgl);

		tgl ^= 1;

		dlen -= pkt;
		dbuf += pkt;
		num++;
	} while (dlen || zlp);

	rv = 0;
out:
	return rv;
}

/*
 * Process EP transaction
 */
static int khci_xfer_process(struct khci_hcd *khci,
			     struct list_head *lst, int type)
{
	struct khci_ep	*kep, *tmp;
	struct khci_urb	*kurb;
	unsigned int sof, sof_wrk;
	LIST_HEAD(done_lst);
	unsigned long flags;
	int num = 0;

	/*
	 * Walk through the EP list, and process URBs scheduled.
	 * Upon flushing all URBs scheduled - move this EP to the end of list
	 */
	spin_lock_irqsave(&khci->lock, flags);
	sof = khci->sof;
	sof_wrk = khci->sof_wrk;
	spin_unlock_irqrestore(&khci->lock, flags);

	list_for_each_entry_safe(kep, tmp, lst, node) {
		if (type != USB_ENDPOINT_XFER_INT) {
			/*
			 * Don't process next EP entry until there are some
			 * unprocessed URBs in it.
			 */
			while (!list_empty(&kep->urb_lst)) {
				kurb = list_first_entry(&kep->urb_lst,
							struct khci_urb, node);
				if (khci_urb_process(kurb))
					break;
				num++;
			}

			if (!list_empty(&kep->urb_lst))
				break;

			list_move_tail(&kep->node, &done_lst);
		} else {
			/*
			 * Check intervals
			 */
			list_for_each_entry(kurb, &kep->urb_lst, node) {
				if (sof < kurb->sof_nxt) {
					if (sof_wrk > kurb->sof_nxt)
						sof_wrk = kurb->sof_nxt;
					continue;
				}

				khci_urb_process(kurb);
				num++;
				kurb->sof_nxt = sof + kurb->urb->interval;
				if (sof_wrk > kurb->sof_nxt)
					sof_wrk = kurb->sof_nxt;
			}
		}
	}

	if (!list_empty(&done_lst))
		list_splice_tail(&done_lst, lst);

	spin_lock_irqsave(&khci->lock, flags);
	khci->sof_wrk = sof_wrk;
	spin_unlock_irqrestore(&khci->lock, flags);

	return num;
}

/*
 * Process EP list
 */
static void khci_ep_process(struct khci_hcd *khci)
{
	int	num = 0;

	dbg(3, "%s in\n", __func__);

	/*
	 * Very dummy scheduling: don't schedule BULK transfers in the time
	 * frames with the CONTROL/INT transactions. To implement more intel-
	 * legent schemas we should count bytes sent on the bus in this frame,
	 * and estimate if the time remaining in the current frame will be
	 * enough to process BULK transfer.
	 */
	num += khci_xfer_process(khci, &khci->ctrl_lst,
				 USB_ENDPOINT_XFER_CONTROL);
	num += khci_xfer_process(khci, &khci->intr_lst,
				 USB_ENDPOINT_XFER_INT);

	if (!num) {
		khci_xfer_process(khci, &khci->bulk_lst,
				  USB_ENDPOINT_XFER_BULK);
	}
}

/*
 * Transfer next TD of URB
 */
static int khci_urb_process(struct khci_urb *kurb)
{
	struct khci_hcd		*khci = kurb->kep->khci;
	volatile struct khci_bd	*bd;
	volatile struct khci_reg *reg = khci->reg;
	struct khci_td		*td;
	int			busy, usec;
	u32			len;
	u8			mask;

	dbg(3, "%s %d,%x,%x %s.%s.%x\n", __func__,
	    kurb->state, kurb->dev_addr, kurb->ep_cfg,
	    list_empty(&kurb->td_sched_lst) ? "no sched" : "have sched",
	    !khci->td ? "no cur" : "have cur", reg->ctl);

	/*
	 * Check if we have some TDs for this URB, if there's nothing
	 * transmitting right now, and if it's OK to xfer next token
	 */
	if (list_empty(&kurb->td_sched_lst) || khci->td) {
		busy = 1;
		goto out;
	}

	/*
	 * Program USB-FS with next TD. We don't use double-buffering
	 * scheme - since don't allow to program next xfer if some xfer
	 * is already active. This is to simplify processing errors (we
	 * do re-send in this case)
	 */
	td = list_first_entry(&kurb->td_sched_lst, struct khci_td, node);
	bd = KHCI_BD_PTR(khci, 0, td->tx, khci->bd[td->tx]);

	if (bd->flg & KHCI_BD_OWN) {
		dbg(0, "BD [%d][%d] BUSY\n", td->tx, khci->bd[td->tx]);
		khci_dbg_dump_reg(khci);
		busy = 1;
		goto out;
	}

	/*
	 * Wait until it's OK to send next token, max 1ms.
	 * Note, such busy-wait, as well as regs programming is taken
	 * from MQX driver
	 */
	reg->addr = kurb->dev_addr;
	reg->ep[0].endpt = kurb->ep_cfg;
	usec = 1000;
	while ((reg->ctl & KHCI_CTL_TOKENBUSY) && usec > 0) {
		udelay(1);
		usec--;
	}
	if (usec <= 0) {
		busy = 1;
		goto out;
	}

	if (kurb->state == KHCI_URB_IDLE)
		kurb->state = KHCI_URB_RUN;

	/*
	 * If this is first xfer of this TD, then save the original BD flags;
	 * these will be updated then in IRQ handler, and may be necessary if
	 * we'll decide to retry the xfer
	 */
	if (!td->tries)
		td->org_flg = td->bd_flg;

	td->tries++;
	td->bd_flg = td->org_flg;
	td->bd = bd;
	td->status = -EBUSY;

	/*
	 * Syncronize cache with mem to be sure that USB DMA is working
	 * with a valid buffer. In case of RX we invalidate buffer, and
	 * to keep not our data which may sit in these cache lines we
	 * do flushing
	 */
	len = KHCI_BD_BC_GET(td->bd_flg);

#if defined(CONFIG_ARCH_KINETIS)
	kinetis_ps_cache_flush_mlines((void *)td->bd_adr, len);
	if (!td->tx)
		kinetis_ps_cache_inval_mlines((void *)td->bd_adr, len);
#endif

	khci->td = td;

	/*
	 * Program hw.
	 * SOFs are generated each 1ms; setting SOFTHLD reg we allow to suspend
	 * sending the current frame if there's no enough time for next SOF
	 */
	if (kurb->urb->dev->speed == USB_SPEED_LOW)
		reg->softhld = len * 12 * 7 / 6 + 0x65;
	else
		reg->softhld = len * 7 / 6 + 0x65;
	reg->istat = KHCI_INT_SOFTOK;
	reg->errstat = 0xFF;

	bd->adr = td->bd_adr;
	bd->flg = td->bd_flg;
	reg->token = td->token;

	khci->bd[td->tx] ^= 1;

	dbg(3, "xfer[%d][%d] %dof%d: URB:%p.TOK:%02x.BD:%08x.%08x\n",
	    td->tx, khci->bd[td->tx], td->tries, td->tries_max,
	    td->kurb->urb, td->token, td->bd_flg, td->bd_adr);

	/*
	 * Wait for transaction completes, max 100ms.
	 * Note, if we just return here, then sometimes we get TOK_DNE IRQ, but
	 * BD's OWN flag remains set. This is observed if cache enabled, even
	 * if we use SRAM, or non-cacheable DRAM slot in UFS DMA BD's addr field
	 */
	mask = KHCI_INT_TOKDNE | KHCI_INT_ERROR | KHCI_INT_RST;
	usec = 100000;
	while (!(reg->istat & mask) && usec > 0) {
		udelay(1);
		usec--;
	}

	busy = 0;
out:
	return busy;
}

/*
 * Process ATTACH interrupt
 */
static void khci_irq_attach(struct khci_hcd *khci, u8 istat)
{
	volatile struct khci_reg	*reg = khci->reg;

	reg->istat = KHCI_INT_ATTACH;
	if (!(reg->inten & KHCI_INT_ATTACH))
		goto out;

	/*
	 * Check if the connecting device is low-speed
	 */
	if (!(reg->ctl & KHCI_CTL_JSTATE))
		khci->vrh.port.wPortStatus |= USB_PORT_STAT_LOW_SPEED;
	else
		khci->vrh.port.wPortStatus &= ~USB_PORT_STAT_LOW_SPEED;

	khci->vrh.port.wPortStatus |= USB_PORT_STAT_CONNECTION |
				      USB_PORT_STAT_ENABLE;
	khci->vrh.port.wPortChange |= USB_PORT_STAT_C_CONNECTION |
				      USB_PORT_STAT_C_ENABLE;

	/*
	 * Enable SOF generation, and update interrupt mask
	 */
	reg->ctl |= KHCI_CTL_USBENSOFEN;

	reg->inten &= ~KHCI_INT_ATTACH;
	reg->inten |= KHCI_INT_TOKDNE | KHCI_INT_ERROR | KHCI_INT_RST;
out:
	return;
}

/*
 * Process TOKDNE interrupt
 */
static void khci_irq_tokdne(struct khci_hcd *khci, u8 istat)
{
	volatile struct khci_bd	*bd;
	struct khci_td		*td;
	u8			stat, tok;

	stat = khci->reg->stat;
	khci->reg->istat = KHCI_INT_TOKDNE;

	if (!khci->td) {
		dbg(0, "%s: no active TD (stat=%02x)\n", __func__, stat);
		goto out;
	}

	/*
	 * If TD status is smth not -EBUSY, then this was updated because
	 * of some other (RST, ERR) IRQ; so don't process it
	 */
	td = khci->td;
	if (td->status != -EBUSY)
		goto out;

	td->istat = istat;
	td->stat = stat;
	td->err = khci->reg->errstat;

	bd = td->bd;

	td->bd_flg = bd->flg;
	tok = KHCI_BD_TOK_GET(bd->flg);

	if (bd->flg & KHCI_BD_OWN) {
		/*
		 * Bad BD state
		 */
		dbg(0,"%s: BD%d.%d.%d %08x(%08x).%08x is still busy (try %d)\n",
		    __func__,
		    KHCI_STAT_EP(stat), KHCI_STAT_TX(stat), KHCI_STAT_ODD(stat),
		    bd->flg, td->bd_flg, bd->adr, td->tries);
		khci_dbg_dump_reg(khci);

		td->retry.own++;
		khci->stat.own++;

		td->status = -EIO;
	} else if (tok == KHCI_BD_TOK_DATA0 || tok == KHCI_BD_TOK_DATA1 ||
		   tok == KHCI_BD_TOK_ACK) {
		/*
		 * Xfer passed OK
		 */
		td->status = 0;
	} else if (tok == KHCI_BD_TOK_STALL) {
		/*
		 * STALL can't be recovered
		 */
		td->status = -EPIPE;
	} else if (tok == KHCI_BD_TOK_NAK &&
		   usb_endpoint_xfer_int(&td->kurb->urb->ep->desc)) {
		/*
		 * INT NAK
		 */
		td->status = -ENOMSG;
	} else {
		/*
		 * Some error
		 */
		td->status = -EAGAIN;
	}
out:
	return;
}

/*
 * Process ERROR interrupt
 */
static void khci_irq_error(struct khci_hcd *khci, u8 istat)
{
	u8	err = khci->reg->errstat;

	dbg(0, "ERR IRQ %02x\n", err);
	khci_dbg_dump_reg(khci);

	if (khci->td)
		khci->td->status = -EFAULT;

	khci->reg->errstat = err;
	khci->reg->istat = KHCI_INT_ERROR;
}

/*
 * Process RESET interrupt
 */
static void khci_irq_rst(struct khci_hcd *khci, u8 istat)
{
	volatile struct khci_reg	*reg = khci->reg;
	struct usb_hcd			*hcd = khci_to_hcd(khci);

	if (khci->td)
		khci->td->status = -ESHUTDOWN;
	reg->istat = KHCI_INT_RST;

	/*
	 * Reset control registers to defaults, and restart HCD
	 */
	reg->usbctrl = KHCI_USBCTRL_SUSP | KHCI_USBCTRL_PDE;
	reg->addr = 0;
	reg->ctl = 0;

	/*
	 * Call our khci_hc_start()
	 */
	hcd->driver->start(hcd);

	/*
	 * Device disconnected; update our virtual hub port status
	 */
	khci->vrh.port.wPortStatus &= ~(USB_PORT_STAT_CONNECTION |
				        USB_PORT_STAT_ENABLE);
	khci->vrh.port.wPortChange |= USB_PORT_STAT_C_CONNECTION |
				      USB_PORT_STAT_C_ENABLE;
}

/*
 * Process SOF interrupt
 */
static void khci_irq_sof(struct khci_hcd *khci, u8 istat)
{
	khci->reg->istat = KHCI_INT_SOFTOK;
	khci->sof++;
}

/*
 * Called upon completion xfering TD
 */
static void khci_td_complete(struct khci_hcd *khci, struct khci_td *td)
{
	struct khci_urb	*kurb = td->kurb;
	struct urb	*urb = kurb->urb;
	struct khci_ep	*kep = urb->ep->hcpriv;
	struct khci_td	*nxt;
	int		i, status = 0;

	kurb->td_done++;
	dbg(3, "%s URB:%p %dof%d %d.%d.%x\n", __func__, urb, kurb->td_done,
	    kurb->td_todo, td->tx, KHCI_BD_BC_GET(td->org_flg), td->bd_adr);

	if (!td->status) {
		switch (td->token & KHCI_TOKEN_MSK) {
		case KHCI_TOKEN_OUT:
			khci->stat.out++;
			break;
		case KHCI_TOKEN_IN:
			khci->stat.in++;
			break;
		case KHCI_TOKEN_SETUP:
			khci->stat.setup++;
			break;
		default:
			break;
		}

		kurb->stat.ok++;
	}

	/*
	 * If we've received less than requested, or if xfer failed, then
	 * skip all the TDs remaining in this URB. If this isn't a err, then
	 * keep last ZLP though
	 */
	status = td->status;
	if (KHCI_BD_BC_GET(td->bd_flg) < KHCI_BD_BC_GET(td->org_flg) ||
	    status || kurb->state == KHCI_URB_DEL) {
		list_for_each_entry_safe(td, nxt, &kurb->td_sched_lst, node) {
			if (!KHCI_BD_BC_GET(td->bd_flg) && !status &&
			    kurb->state != KHCI_URB_DEL) {
				break;
			}
			td->skip = 1;
			kurb->td_done++;
			list_move_tail(&td->node, &khci->td_done_lst);
		}
	}

	if (kurb->td_done < kurb->td_todo)
		goto out;

	/*
	 * Check if URB was rejected (free) while we processed it
	 */
	if (kurb->state == KHCI_URB_DEL)
		status = -EINTR;

	/*
	 * If this is a CONTROL transfer, then substract the length
	 * of the SETUP packet
	 */
	if (kep->type == USB_ENDPOINT_XFER_CONTROL)
		urb->actual_length -= 8;

	i = 0;
	dbg(3, "DONE EP%d.%p.%p,URB:%p.%d done (A:%d.E:%02x.L:%d.S:%d)\n",
	    kep->type, urb->ep, urb->ep->hcpriv,
	    urb, kurb->td_todo,
	    khci->reg->addr, khci->reg->ep[0].endpt,
	    urb->actual_length, status);
	list_for_each_entry_safe(td, nxt, &khci->td_done_lst, node) {
		list_del_init(&td->node);
		dbg(3, "     TD[%d.%3d.%d]: %s L:%2d/%2d BD:%08x.%08x "
		    "TOK:%02x ER:%02x ST:%02x "
		    "I:%02x RTR:%d.%d.%d.%d.%d "
		    "RES:%s\n",
		    i++, td->tries, td->skip, td->tx ? "TX" : "RX",
		    KHCI_BD_BC_GET(td->bd_flg), KHCI_BD_BC_GET(td->org_flg),
		    td->bd_flg, td->bd_adr,
		    td->token, td->err, td->stat,
		    td->istat,
		    td->retry.own, td->retry.nak, td->retry.bus,
		    td->retry.err, td->retry.len,
		    khci_dbg_flg2tock(td->bd_flg));

#if defined(DEBUG) && DEBUG >= 3
		if (!td->skip && KHCI_BD_BC_GET(td->bd_flg)) {
			khci_dbg_dump_buf((u8 *)td->bd_adr,
					 KHCI_BD_BC_GET(td->bd_flg));
		}
#endif

		if (td->skip)
			goto free;

		if (!status && td->status) {
			status = td->status;
			goto free;
		}

#if defined(CONFIG_ARCH_KINETIS)
		if (!td->tx) {
			kinetis_ps_cache_inval_mlines((void *)td->bd_adr,
						       KHCI_BD_BC_GET(td->bd_flg));
		}
#endif
		usb_dotoggle(urb->dev, td->token & 0xF, td->tx);
free:
		khci_td_free(td);
	}
	dbg(3, "----------------------------------------------------\n");

	dbg(1, "%s URB:%p,EP:%p.%p,Stat:%d,Len:%d/%d\n", __func__,
	    urb, urb->ep, urb->ep->hcpriv,
	    status, urb->actual_length, urb->transfer_buffer_length);

	kurb->status = status;
	khci_urb_free(kurb);

	if (kep->state == KHCI_EP_DEL)
		khci_ep_free(kep);
out:
	return;
}
