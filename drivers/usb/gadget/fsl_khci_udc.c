/*
 * Copyright (C) 2013. All rights reserved.
 *
 * Author: Axel Utech axel.utech@gmail.com
 *
 * Description:
 * Device driver for Freescale USB Full Speed controller available in Kinetis
 * controllers (K70,K60,...) and some Coldfire controllers (MCF52259,...).
 * Based on bare board code delivered by Freescale Semicondutor, Inc.
 * Based on structure of fsl_udc_core.c
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/delay.h>

#include <linux/clk.h>

#include <mach/memory.h>
#include <mach/cache.h>
#include "fsl_khci_udc.h"

#define DRIVER_DESC     "Freescale KHCI (USB-FS) Device controller"
#define DRIVER_AUTHOR   "Axel Utech"
#define DRIVER_VERSION  "0.2"


static const char driver_name[] = "fsl_khci_udc";
static const char driver_desc[] = DRIVER_DESC;

/* it is initialized in probe()  */
struct fsl_udc *udc = NULL;

volatile struct k70_usb0_regs *regs;

static const struct usb_endpoint_descriptor
fsl_ep0_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	0,
	.bmAttributes =		USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize =	USB_MAX_CTRL_PAYLOAD,
};

/*
 * USB DMA is 4 byte aligned (and writes 0x00 to other places else)
 * e. g. buffer for g_ether are 2 byte aligned and can not be used directly
 */
unsigned use_own_buffer = 1;
module_param(use_own_buffer, bool, S_IRUGO);
MODULE_PARM_DESC(use_own_buffer, "use extra buffer for OUT requests");


/*-------------------------------------------------------------------------*/
static void fsl_ep_fifo_flush(struct usb_ep *_ep);
static void callback_dr_ep0_setup(struct usb_ep *_ep, struct usb_request *_req);
static void callback_ep0_in_status(struct usb_ep *_ep, struct usb_request *_req);
static void callback_ep0_out_status(struct usb_ep *_ep, struct usb_request *_req);

/********************************************************************
	Internal helper functions
********************************************************************/
static struct k70_bd_entry* get_bd_entry(u8 endpoint_num, u8 dir, u8 odd)
{
	u32 num;

	if(endpoint_num >= USB_NUM_ENDPOINTS){
		ERR("get_bd_entry: endpoint_num not valid");
		return NULL;
	}
	if(dir > 1){
		ERR("get_bd_entry: dir not valid");
		return NULL;
	}
	if(odd > 1){
		ERR("get_bd_entry: odd not valid");
		return NULL;
	}

	num = endpoint_num << 2 | dir << 1 | odd;
	return &(udc->bd_table[num]);
}

/* get ODD Buffer for EVEN Buffer and EVEN for ODD */
static struct k70_bd_entry* get_bd_entry_alt(struct k70_bd_entry* bd)
{
	return (void*)((u32)bd ^ (1<<3));
}

u8 bd_entry_is_odd(struct k70_bd_entry* bd)
{
	return !!((u32)bd & (1<<3));
}


volatile struct endpoint_register* endpt_reg(u8 num){
	return &(regs->endpoints[num * 4]);
}

/********************************************************************
	Internal debugging and testing functions
********************************************************************/

void print_data(void *data, int length)
{
#ifdef VERBOSE
	int i;
	printk(KERN_INFO "Data (%u byte): ",length);
	for(i=0;i<length;i++){
		printk(" %02X",((u8*)data)[i]);
	}
	printk("\n");
#endif
}

static void check_bd_adress(void)
{
	u32 bd_entry_addr = (u32)udc->bd_table_dma;
	u32 bd_adr=0;

	bd_adr |= (u32)regs->bdtpage3 << 24;
	bd_adr |= (u32)regs->bdtpage2 << 16;
	bd_adr |= (u32)regs->bdtpage1 << 8;

	if(bd_entry_addr != bd_adr){
		ERR("BD Address mismatch! (u32)bd = %X  !=  bd_adr = %X",bd_entry_addr,bd_adr);
	}

	if((bd_entry_addr & 0x1FF) != 0)
		DBG("BD Table not on 512 byte boundary!");
}

static void check(void)
{
#ifdef DEBUG
	/* Test some addresses */
	if((u32)&(regs->perid) != 0x40072000)
		ERR("address of regs->perid != 0x40072000 (0x%X)",(u32)&(regs->perid) );
	if((u32)&(regs->usbfrmadjust) != 0x40072114)
		ERR("address of regs->usbfrmadjust != 0x40072114 (0x%X)",(u32)&(regs->usbfrmadjust) );
	if((u32)&(regs->ctl) != 0x40072094)
		ERR("address of regs->ctl != 0x40072094 (0x%X)",(u32)&(regs->ctl) );

	if((u32)&(endpt_reg(0)->reg_value) != 0x400720C0)
		ERR("address of regs->endpt0 != 0x400720C0 (0x%X)",(u32)&(endpt_reg(0)->reg_value) );
	if((u32)&(endpt_reg(15)->reg_value) != 0x400720FC)
		ERR("address of regs->endpt15 != 0x400720FC (0x%X)",(u32)&(endpt_reg(15)->reg_value) );

	if((u32)&(regs->usbctrl) != 0x40072100)
		ERR("address of regs->usbctrl != 0x40072100 (0x%X)",(u32)&(regs->usbctrl) );

	if(sizeof(struct k70_bd_entry) * USB_NUM_ENDPOINTS_BUFFERS != 512)
		ERR("total size of buffer descriptor table mismatch!");
#endif
}

/* check and free unused buffers */
static void dr_unbuffer(struct fsl_req *req)
{
	struct fsl_ep *ep = req->ep;
	struct fsl_buffer_descriptor *buf;
	int i;

	buf = ep->next_free_bd;

	if(!buf){
		/* all buffers are used */
		buf = ep->next_use_bd;
	}
	buf = buf->alt_buffer;

	for(i=0;i<=1;i++){
		if(!buf->in_use) continue;
		if(buf->req != req) continue;

		if(buf->bd_entry->own){
			/* buffer was not used for a transmission */
			buf->bd_entry->own = 0;
			buf->bd_entry->stall = 0;

			ep->next_free_bd = buf;
		}

		buf->in_use = 0;
		buf->req = NULL;
		req->ep->stat.buffers_freed ++;

		buf = buf->alt_buffer;
	}
}

/*-----------------------------------------------------------------
 * done() - retire a request; caller blocked irqs
 * @status : request status to be set
 *--------------------------------------------------------------*/
static void dr_done(struct fsl_ep *ep, struct fsl_req *req, int status)
{
	/* Removed the req from fsl_ep->queue */
	list_del_init(&req->queue);

	dr_unbuffer(req);
	ep->stat.requests_complete ++;

	if(req->req.length > 0){
		if(req->ep->dir == EP_DIR_IN){
			dma_unmap_single(NULL, req->req.dma, req->req.length,
				req->ep->dir == EP_DIR_IN ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		}
		mb();
		kinetis_ps_cache_flush();
	}

	if(udc->ep0_dir == USB_DIR_IN && ep == &udc->eps[1])
		ep = &udc->eps[0]; /* switch back to EP0.0 */

	req->req.status = status;

	spin_unlock(&udc->lock);
	/* complete() is from gadget layer,
	 * eg fsg->bulk_in_complete() */
	if (req->req.complete)
		req->req.complete(&ep->ep, &req->req);

	spin_lock(&udc->lock);
}

/*-----------------------------------------------------------------
 * nuke(): delete all requests related to this ep
 *--------------------------------------------------------------*/
static void dr_nuke(struct fsl_ep *ep, int status)
{
	int c=0;

	while (!list_empty(&ep->queue)) {
		struct fsl_req *req = NULL;

		req = list_entry(ep->queue.next, struct fsl_req, queue);

		dr_done(ep, req, status);
		c++;
	}

	/* Flush fifo */
	spin_unlock(&udc->lock);
	fsl_ep_fifo_flush(&ep->ep);
	spin_lock(&udc->lock);

	if(c) VDBG("entries %s: %u",ep->name,c);
}

/*
 * clear one pipe
 */
static void dr_reset_pipe(u8 pipe)
{
	struct fsl_ep *ep = &udc->eps[pipe];

	if (ep->name)
		dr_nuke(ep, -ESHUTDOWN);
}

/*
 * Clear up all queues
 * */
static int dr_reset_queues(void)
{
	u8 pipe;

	VDBG("reset queues");
	for (pipe = 0; pipe < USB_NUM_PIPES; pipe++)
		dr_reset_pipe(pipe);

	/* report disconnect */
	spin_unlock(&udc->lock);
	udc->driver->disconnect(&udc->gadget);
	spin_lock(&udc->lock);

	return 0;
}

static void dr_reset_data0_1(struct fsl_ep *ep)
{
	ep->next_use_bd->bd_entry->data0_1 = 0;
	ep->next_use_bd->alt_buffer->bd_entry->data0_1 = 1;
}

static int dr_ep_set_buffer_descriptor(struct fsl_req *req,
		struct fsl_buffer_descriptor *bd)
{
	int remaining_bytes =req->req.length - req->planned;
	int datalength = min(remaining_bytes, (int)req->ep->ep.maxpacket);
	u8 *curr_buf = (u8*)req->req.dma + (u32)req->planned;
	struct k70_bd_entry *bd_ptr = bd->bd_entry;

	VDBG("%s %s %s: %u bytes  %u/%u (DATA%u) %s",req->ep->ep.name,
			req->ep->dir ? "IN ":"OUT",
			bd_entry_is_odd(bd_ptr) ? "ODD ":"EVEN",
			datalength,req->planned, req->req.length, req->data0_1,
			(req->stall)?"STALL":"");

	if(req->ep->dir == EP_DIR_IN || !use_own_buffer)
		bd_ptr->addr = (u32)curr_buf;

	bd_ptr->bytecount = datalength;

	/*
	 * Data0_1 change only for EP0
	 * all other endpoints keep one buffer for data0 and the other for data 1
	 */
	if(ep_index(req->ep)==0){
		bd_ptr->data0_1 = req->data0_1;
		req->data0_1 = !req->data0_1;
	}

	bd_ptr->stall = req->stall;
	bd_ptr->no_adress_increment = 0;
	bd_ptr->keep = 0;
	bd_ptr->dts = 1;

	bd_ptr->own = 1;

	bd->in_use = 1;
	bd->req = req;

	req->ep->stat.buffers_set ++;

	return datalength;
}

static void dr_fill_buffer(struct fsl_req *req)
{
	struct fsl_ep *ep = req->ep;
	int datalength;

	datalength = dr_ep_set_buffer_descriptor(req, ep->next_free_bd);

	/* update byte counter */
	req->planned += datalength;

	/* zero length packet already queued */
	if(datalength == 0){
		req->state = REQ_READY;
		return;
	}

	/* data completely queued but zero length packet needed */
	if(req->planned == req->req.length && req->send_zero_packet){
		req->state = REQ_PACKET_PENDING;
		return;
	}
	/* last queued packet was last data packet and not full */
	if(req->planned == req->req.length && datalength > 0){
		req->state = REQ_READY;
		return;
	}

	/* for OUT requests only use one buffer at a time */
	if(req->ep->dir == EP_DIR_OUT && datalength!=0){
		req->state = REQ_DATA_BUFFER;
		return;
	}

	/* data pending */
	if(req->planned < req->req.length){
		req->state = REQ_PACKET_PENDING;
		return;
	}
}

/* check if buffers can be prepared for transmission */
static void dr_check_dequeue(struct fsl_ep *ep)
{
	struct fsl_req *req, *temp_req;
	u8 num_free_buffers = 0;

	if(list_empty(&ep->queue)){
		return;
	}

	if(ep->ep_bd[0]->in_use == 0)
		num_free_buffers++;
	if(ep->ep_bd[1]->in_use == 0)
		num_free_buffers++;


	/* all buffers currently used */
	if(ep->next_free_bd == NULL || num_free_buffers == 0)
		return;

	req = list_first_entry(&ep->queue, struct fsl_req, queue);
	list_for_each_entry_safe_from(req, temp_req, &ep->queue, queue) {
		while(req->state == REQ_PACKET_PENDING){

			dr_fill_buffer(req);

			num_free_buffers--;

			if(num_free_buffers == 0){
				ep->next_free_bd = NULL;
				return;
			}

			ep->next_free_bd = ep->next_free_bd->alt_buffer;
		}

		/* OUT traffic does not know how much data will be send */
		if(req->state == REQ_DATA_BUFFER)
			return;
	}
}

/* internal function to queue a request */
static int dr_queue_request(struct fsl_req *req)
{
	/* DMA map the data buffer */
	if(req->req.length > 0){
		kinetis_ps_cache_flush();
		mb();

		if(req->ep->dir == EP_DIR_IN || !use_own_buffer){
			req->req.dma = dma_map_single(NULL, req->req.buf, req->req.length,
				req->ep->dir == EP_DIR_IN ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

			if(dma_mapping_error(NULL, req->req.dma)){
				ERR("dma mapping error %s",req->ep->name);
				return -ENOMEM;
			}
		}
	}

	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->planned = 0;

	req->ep->stat.requests_set ++;

	list_add_tail(&req->queue, &req->ep->queue);

	req->state = REQ_PACKET_PENDING;

	dr_check_dequeue(req->ep);
	return 0;
}

static void dr_ep0_out_status(u8 stall)
{
	udc->ep0_req_out->req.length = 0;
	udc->ep0_req_out->stall = stall;
	udc->ep0_req_out->data0_1 = 1;
	udc->ep0_req_out->req.complete = callback_ep0_out_status;
	udc->ep0_req_out->send_zero_packet = 0;

	dr_queue_request(udc->ep0_req_out);
}

static void dr_ep0_setup_token(void)
{
	udc->ep0_req_setup->req.length = USB_MAX_CTRL_PAYLOAD;
	udc->ep0_req_setup->stall = 0;
	udc->ep0_req_setup->data0_1 = 0;
	udc->ep0_req_setup->req.complete = callback_dr_ep0_setup;
	udc->ep0_req_setup->send_zero_packet = 0;

	dr_queue_request(udc->ep0_req_setup);
}
static void dr_ep0_in_status(u8 stall)
{
	udc->ep0_req_in->req.length = 0;
	udc->ep0_req_in->stall = stall;
	udc->ep0_req_in->data0_1 = 1;
	udc->ep0_req_in->req.complete = callback_ep0_in_status;
	udc->ep0_req_in->send_zero_packet = 0;

	dr_queue_request(udc->ep0_req_in);

	udc->ep0_state = WAIT_FOR_IN_STATUS;
}

static void dr_ep0_reset(void)
{
	VDBG("reseting EP0");

	dr_reset_pipe(0);
	dr_reset_pipe(1);

	dr_ep0_setup_token();

	endpt_reg(0)->stall = 0;

	udc->ep0_dir = 0;
	udc->ep0_state = WAIT_FOR_SETUP;

	regs->ctl.suspend_busy = 0;
}


/*
 * Setup Endpoint register
 * possible types:
 * 		USB_ENDPOINT_XFER_CONTROL
 * 		USB_ENDPOINT_XFER_ISOC
 * 		USB_ENDPOINT_XFER_BULK
 * 		USB_ENDPOINT_XFER_INT
 *
 * 	internal function; called with spinlock
*/
static void dr_ep_setup(u8 ep_num, u8 dir, u8 ep_type, u8 enable)
{
	volatile struct endpoint_register *ep = endpt_reg(ep_num);

	VDBG("setting endpoint %u: dir = %u  type = %u enable = %u",
			ep_num, dir, ep_type, enable);

	ep->stall = 0;
	if(dir == USB_SEND){
		ep->txen = enable;
	}else{
		ep->rxen = enable;
	}

	switch(ep_type){
	case USB_ENDPOINT_XFER_CONTROL:
		ep->handshake = 1;
		ep->disablecontrol = 0;
		break;

	case USB_ENDPOINT_XFER_ISOC:
		ep->handshake = 0;
		ep->disablecontrol = 1;
		break;

	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		ep->handshake = 1;
		ep->disablecontrol = 1;
		break;
	}
}

/*
 * set an endpoint stall
 * note: stall on e. g. EP1in also stalls EP1out
 */
static void dr_ep_change_stall(struct fsl_ep *ep, u8 value)
{
	VDBG("%s stall %u => %u",ep->name, ep->endpt_reg->stall, value);

	/* EP0 */
	if(ep_index(ep)==0){
		endpt_reg(0)->stall = (value) ? 1: 0;
		regs->inten.stall = (value) ? 1: 0;
		return;
	}

	if(ep->endpt_reg->stall == 0 && value == 1){
		ep->endpt_reg->stall = 1;
		dr_nuke(ep, -ESHUTDOWN);
	}

	if(ep->endpt_reg->stall == 1 && value == 0){
		endpt_reg(1)->stall = 0;
		dr_reset_data0_1(ep);
	}
}

/* Get stall status of a specific ep
   Return: 0: not stalled; 1:stalled */
static volatile int dr_ep_get_stall(struct fsl_ep *ep)
{
	return ep->endpt_reg->stall;
}

static void clear_all_err_flags(void)
{
	regs->errstat.regvalue = 0xFF;
}
static void clear_all_int_flags(void)
{
	regs->istat.regvalue = 0xFF;
}


/* Enable DR irq and set controller to run state */
static void dr_controller_run(void)
{
	DBG("Activating controller");

	clear_all_err_flags();
	clear_all_int_flags();

	/* Disable all interrupt sources */
	regs->inten.regvalue = 0;

	/* Enable USB Reset Interrupt */
	regs->inten.usbrst = 1;

	/*Enable DP Pull up in non-OTG mode */
	regs->control.dppluuup = 1;

	udc->stopped = 0;
}

static void dr_controller_stop(void)
{
	DBG("Stopping controller");

	/* disable all INTR */
	regs->inten.regvalue = 0;
	regs->erren.regvalue = 0;

	/* Set stopped bit for isr */
	udc->stopped = 1;

	/* set controller to Stop */
	/*disable DP Pull up in non-OTG mode */
	regs->control.dppluuup = 0;
}

/*-----------------------------------------------------------------------------
 * endpoint-specific parts of the api to the usb controller hardware
 *-----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------
 * allocate a request object used by this endpoint
 * the main operation is to insert the req->queue to the eq->queue
 * Returns the request, or null if one could not be allocated
*---------------------------------------------------------------------*/
static struct usb_request *
fsl_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	struct fsl_req *req = NULL;

	req = kzalloc(sizeof(struct fsl_req), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);

	req->ep = ep;
	req->stall = 0;
	req->data0_1 = 0;

	DBG("alloc request for %s addr=%X b %X",_ep->name, (u32)req, (u32)req->req.buf);
	return &req->req;
}

static void fsl_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct fsl_req *req = NULL;

	DBG("free request for %s",ep->name);
	req = container_of(_req, struct fsl_req, req);


	if (_req)
		kfree(req);
}

/* Release udc structures */
static void fsl_udc_release(struct device *dev)
{
	complete(udc->done);
	kfree(udc);
}

/*
 * enable one endpoint
 */
static int fsl_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	int ret = 0;
	unsigned long flags;

	VDBG("enabling %s",_ep->name);

	/* catch various bogus parameters */
	if (!_ep || !desc || ep->desc
			|| (desc->bDescriptorType != USB_DT_ENDPOINT))
		return -EINVAL;

	if (!udc->driver || (udc->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	if((desc->bmAttributes & 0x03) == USB_ENDPOINT_XFER_ISOC){
		ERR("Isochronous endpoints not supported!");
		return -EINVAL;
	}

	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);;
	ep->desc = desc;

	spin_lock_irqsave(&udc->lock, flags);

	dr_reset_data0_1(ep);

	/* Init endpoint ctrl register */
	dr_ep_setup((u8) ep_index(ep),
			(u8) ((desc->bEndpointAddress & USB_DIR_IN)	? USB_SEND : USB_RECV),
			(u8) (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK),
			1);

	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

/*---------------------------------------------------------------------
 * @ep : the ep being unconfigured. May not be ep0
 * Any pending and uncomplete req will complete with status (-ESHUTDOWN)
*---------------------------------------------------------------------*/
static int fsl_ep_disable(struct usb_ep *_ep)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	unsigned long flags;
	VDBG("disabling %s",_ep->name);

	/* catch various bogus parameters */
	if (!_ep || !ep->desc || ep_index(ep) == 0)
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);

	/* Init endpoint ctrl register */
	dr_ep_setup((u8) ep_index(ep),
			(u8) ((ep->desc->bEndpointAddress & USB_DIR_IN)	? USB_SEND : USB_RECV),
			(u8) (ep->desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK),
			0);


	/* nuke all pending requests (does flush) */
	dr_nuke(ep, -ESHUTDOWN);

	ep->desc = NULL;

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

/* queues (submits) an I/O request to an endpoint */
static int
fsl_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	struct fsl_req *req = container_of(_req, struct fsl_req, req);
	unsigned long flags;
	int ret = 0;

	/* catch various bogus parameters */
	if (!_req || !req->req.complete || !req->req.buf
			|| !list_empty(&req->queue)) {
		DBG("%s, bad params", __func__);
		return -EINVAL;
	}
	if (unlikely(!_ep || !ep->desc)) {
		DBG("%s, bad ep", __func__);
		return -EINVAL;
	}
	if (ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		if (req->req.length > ep->ep.maxpacket){
			ERR("req->req.length = %u > ep->ep.maxpacket = %u",req->req.length,
					ep->ep.maxpacket);
			return -EMSGSIZE;
		}
		return -EINVAL;
	}

	if (!udc->driver){
		ERR("ep_queues: no driver: error shutdown");
		return -ESHUTDOWN;
	}
	if (udc->gadget.speed == USB_SPEED_UNKNOWN){
		ERR("ep_queues: error shutdown (udc->gadget.speed == USB_SPEED_UNKNOWN)");
		return -ESHUTDOWN;
	}

	req->stall = 0;
	req->data0_1 = 0;


	if(ep_index(ep) == 0){
		req->data0_1 = 1;
		if(udc->ep0_dir == 123){
			DBG("no EP0 DIR!");
			return -EINVAL;
		}

		if(udc->ep0_dir == USB_DIR_IN){
			req->ep = &udc->eps[1];
		}else{
			req->ep = &udc->eps[0];
		}
	}

	req->send_zero_packet = 0;
	/* check if zero packet is needed */
	if(req->req.zero && req->ep->dir == EP_DIR_IN){
		if((req->req.length % req->ep->ep.maxpacket) == 0)
			req->send_zero_packet = 1;
	}

	VDBG("gadget have request in %s! %d byte %X", req->ep->ep.name, req->req.length,
			(u32)req->req.buf);

	spin_lock_irqsave(&udc->lock, flags);
	ret = dr_queue_request(req);
	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

/* dequeues (cancels, unlinks) an I/O request from an endpoint */
static int fsl_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	struct fsl_req *req = container_of(_req, struct fsl_req, req);
	unsigned long flags;

	VDBG("dequeue request for %s", ep->name);

	if(req->ep != ep){
		return -EINVAL;
	}

	spin_lock_irqsave(&udc->lock, flags);
	dr_done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

/*-----------------------------------------------------------------
 * modify the endpoint halt feature
 * @ep: the non-isochronous endpoint being stalled
 * @value: 1--set halt  0--clear halt
 * Returns zero, or a negative error code.
*----------------------------------------------------------------*/
static int fsl_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);
	dr_ep_change_stall(ep, value);

	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

/*
 * hard reset for endpointbuffers
 * called without spinlock
 */
static void fsl_ep_fifo_flush(struct usb_ep *_ep)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	unsigned long flags;
	VDBG("%s fifo flush", _ep->name);

	spin_lock_irqsave(&udc->lock, flags);
	ep->ep_bd[0]->bd_entry->own = 0;
	ep->ep_bd[0]->in_use = 0;
	ep->ep_bd[0]->req = NULL;

	ep->ep_bd[1]->bd_entry->own = 0;
	ep->ep_bd[1]->in_use = 0;
	ep->ep_bd[1]->req = NULL;

	spin_unlock_irqrestore(&udc->lock, flags);
	return;
}
static struct usb_ep_ops fsl_ep_ops = {
	.enable = fsl_ep_enable,
	.disable = fsl_ep_disable,

	.alloc_request = fsl_alloc_request,
	.free_request = fsl_free_request,

	.queue = fsl_ep_queue,
	.dequeue = fsl_ep_dequeue,

	.set_halt = fsl_ep_set_halt,
	.fifo_flush = fsl_ep_fifo_flush,
};

/*-------------------------------------------------------------------------
		Gadget Driver Layer Operations
-------------------------------------------------------------------------*/

/*
 * Get the current frame number
 * */
static int fsl_get_frame(struct usb_gadget *gadget)
{
	return ((int)regs->frmnumh)<<8 | ((int)regs->frmnuml);
}

/* Tries to wake up the host connected to this gadget */
static int fsl_wakeup(struct usb_gadget *gadget)
{
	return -ENOTSUPP;
}

/* Notify controller that VBUS is powered, Called by whatever
   detects VBUS sessions */
static int fsl_vbus_session(struct usb_gadget *gadget, int is_active)
{
	return -ENOTSUPP;
}

/* constrain controller's VBUS power usage
 * This call is used by gadget drivers during SET_CONFIGURATION calls,
 * reporting how much power the device may consume.  For example, this
 * could affect how quickly batteries are recharged.
 *
 * Returns zero on success, else negative errno.
 */
static int fsl_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	return -ENOTSUPP;
}

/* Change Data+ pullup status
 * this func is used by usb_gadget_connect/disconnet
 */
static int fsl_pullup(struct usb_gadget *gadget, int is_on)
{
	DBG("Pull UP = %u",is_on);
	regs->control.dppluuup = (is_on) ? 1 : 0;
	return 0;
}

/* defined in gadget.h */
static struct usb_gadget_ops fsl_gadget_ops = {
	.get_frame = fsl_get_frame,
	.wakeup = fsl_wakeup,
/*	.set_selfpowered = fsl_set_selfpowered,	*/ /* Always selfpowered */
	.vbus_session = fsl_vbus_session,
	.vbus_draw = fsl_vbus_draw,
	.pullup = fsl_pullup,
};


/*----------------------------------------------------------------*
 * Hook to gadget drivers
 * Called by initialization code of gadget drivers
*----------------------------------------------------------------*/
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	int retval = -ENODEV;
	unsigned long flags = 0;

	printk(KERN_INFO "%s: bind to driver %s\n",
			udc->gadget.name, driver->driver.name);
	if (!udc){
		retval = -ENODEV;
		goto out;
	}

	if (!driver
			|| (driver->speed != USB_SPEED_FULL && driver->speed != USB_SPEED_HIGH)
			|| !driver->bind || !driver->disconnect
			|| !driver->setup){
		retval = -EINVAL;
		goto out;
	}

	if (udc->driver){
		retval = -EBUSY;
		goto out;
	}

	spin_lock_irqsave(&udc->lock, flags);

	driver->driver.bus = NULL;
	/* hook up the driver */
	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	spin_unlock_irqrestore(&udc->lock, flags);

	/* bind udc driver to gadget driver */
	retval = driver->bind(&udc->gadget);
	if (retval) {
		DBG("bind to %s --> %d", driver->driver.name, retval);
		udc->gadget.dev.driver = NULL;
		udc->driver = NULL;
		goto out;
	}

	spin_lock_irqsave(&udc->lock, flags);
	/* Enable DR IRQ reg and Set usbcmd reg  Run bit */
	dr_controller_run();
	udc->usb_state = USB_STATE_ATTACHED;
	udc->ep0_state = WAIT_FOR_SETUP ;
	udc->ep0_dir = 0;
	udc->gadget.speed = USB_SPEED_FULL;
	spin_unlock_irqrestore(&udc->lock, flags);

	printk(KERN_INFO "%s: binding successful", udc->gadget.name);

out:
	if (retval)
		printk(KERN_WARNING "gadget driver register failed %d\n",
		       retval);
	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

/* Disconnect from gadget driver */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct fsl_ep *loop_ep;
	unsigned long flags;

	if (!udc)
		return -ENODEV;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);
	dr_controller_stop();

	/* in fact, no needed */
	udc->usb_state = USB_STATE_ATTACHED;
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;

	/* stand operation */
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	dr_nuke(&udc->eps[0], -ESHUTDOWN);
	list_for_each_entry(loop_ep, &udc->gadget.ep_list,
			ep.ep_list)
		dr_nuke(loop_ep, -ESHUTDOWN);
	spin_unlock_irqrestore(&udc->lock, flags);

	/* report disconnect; the controller is already quiesced */
	driver->disconnect(&udc->gadget);

	/* unbind gadget and unhook driver. */
	driver->unbind(&udc->gadget);
	udc->gadget.dev.driver = NULL;
	udc->driver = NULL;

	printk(KERN_WARNING "unregistered gadget driver '%s'\n",
	       driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*------------------------------------------------------------------
	Internal Hardware related function
 ------------------------------------------------------------------*/


static void __init dr_controller_setup(void)
{
	u32 dma_adr = (u32)udc->bd_table_dma;

	DBG("controller setup");
	regs->usbtrc0.reset = 1;
	while(regs->usbtrc0.reset == 1){;}


	regs->bdtpage1=(u8)(dma_adr >> 8);
	regs->bdtpage2=(u8)(dma_adr >> 16);
	regs->bdtpage3=(u8)(dma_adr >> 24);

	check_bd_adress();

	regs->ctl.hostmode=0;

	/* Clear Interrupt Flags */
	clear_all_int_flags();
	clear_all_err_flags();

	regs->usbtrc0.strange=1;

	/* Activate USB transceiver  */
	regs->usbctrl.susp=0;
	regs->usbctrl.pde=0;

	/* Enable USB module */
	regs->ctl.usben=1;
}

/*
 * Resets all buffers and endpoints to their default states
 */
void dr_reset_handler(void)
{
	int i;

	regs->usbtrc0.strange=1;

	dr_reset_queues();

    /* Disable all EP registers */
    for(i=0;i<USB_NUM_ENDPOINTS;i++){
    	endpt_reg(i)->reg_value=0;
    }

    for(i=0;i<USB_NUM_PIPES;i++){
    	udc->eps[i].next_free_bd = udc->eps[i].ep_bd[0];
    	udc->eps[i].next_use_bd = udc->eps[i].ep_bd[0];

    	udc->eps[i].ep_bd[0]->in_use = 0;
    	udc->eps[i].ep_bd[0]->req = NULL;
    	udc->eps[i].ep_bd[1]->in_use = 0;
    	udc->eps[i].ep_bd[1]->req = NULL;
    }
	regs->ctl.oddrst = 1;

	clear_all_err_flags();
	clear_all_int_flags();

	check_bd_adress();

	/* Clear usb state */
	udc->resume_state = 0;
	udc->remote_wakeup = 0;	/* default to 0 on reset */
	udc->gadget.b_hnp_enable = 0;
	udc->gadget.a_hnp_support = 0;
	udc->gadget.a_alt_hnp_support = 0;

	udc->usb_state = USB_STATE_DEFAULT;

	regs->addr.addr = 0;
	udc->device_address = 0;

	/* enable all interrupt error sources */
	regs->erren.regvalue = 0xFF;

	regs->inten.regvalue = 0;
	regs->inten.tokdne = 1;
	regs->inten.softok = 0;
	regs->inten.error = 1;
	regs->inten.usbrst = 1;
	regs->inten.sleep = 1;

	regs->ctl.suspend_busy = 0;
	regs->ctl.oddrst = 0;

	regs->usbctrl.susp=0;
	regs->usbctrl.pde=0;

	/* Enable EP0 */
	dr_ep_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL, 1);
	dr_ep_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL, 1);
	dr_ep0_reset();

	regs->ctl.usben = 1;
}

/*
 * ch9 Set address
 */
static void dr_ch9setaddress(u16 value, u16 index, u16 length)
{
	/* Save the new address to device struct */
	udc->device_address = (u8) value;
	/* Update usb state */
	udc->usb_state = USB_STATE_ADDRESS;

	/* Status phase */
	dr_ep0_in_status(0);
}

/*
 * ch9 Get status
 */
static void dr_ch9getstatus(u8 request_type, u16 value, u16 index, u16 length)
{
	u16 tmp = 0;		/* Status, cpu endian */
	struct fsl_req *req;

	if ((request_type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		/* Get device status */
		tmp = 1 << USB_DEVICE_SELF_POWERED;
		tmp |= udc->remote_wakeup << USB_DEVICE_REMOTE_WAKEUP;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
		/* Get interface status */
		/* We don't have interface information in udc driver */
		tmp = 0;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
		/* Get endpoint status */
		struct fsl_ep *ep = &udc->eps[get_pipe_by_windex(index)];

		/* stall if endpoint is not enabled */
		if (ep->ep.maxpacket == 0xFFFF)
			goto stall;

		tmp = dr_ep_get_stall(ep) << USB_ENDPOINT_HALT;
	}

	udc->ep0_dir = USB_DIR_IN;

	req = udc->ep0_req_in;
	/* Fill in the reqest structure */
	*((u16 *) req->req.buf) = cpu_to_le16(tmp);
	req->req.length = 2;
	req->data0_1 = 1;
	req->stall = 0;
	req->req.complete = callback_ep0_in_status;

	dr_queue_request(req);

	dr_ep0_out_status(0);
	dr_ep0_setup_token();

	udc->ep0_state = DATA_IN;

	return;
stall:
	dr_ep0_setup_token();
	dr_ep_change_stall(&udc->eps[0], 1);
}

static void callback_dr_ep0_setup(struct usb_ep *_ep, struct usb_request *_req)
{
	struct fsl_req *req = container_of(_req, struct fsl_req, req);
	struct usb_ctrlrequest *setup = req->req.buf;
	int ret;

	u16 wValue = le16_to_cpu(setup->wValue);
	u16 wIndex = le16_to_cpu(setup->wIndex);
	u16 wLength = le16_to_cpu(setup->wLength);

	if(req->req.status < 0) return;

	spin_lock(&udc->lock);

	udc->ep0_state = WAIT_FOR_SETUP;

	/* We process some standard setup requests here */
	switch (setup->bRequest) {
	case USB_REQ_GET_STATUS:
		/* Data+Status phase from udc */
		if ((setup->bRequestType & (USB_DIR_IN | USB_TYPE_MASK))
					!= (USB_DIR_IN | USB_TYPE_STANDARD))
			break;
		dr_ch9getstatus(setup->bRequestType, wValue, wIndex, wLength);
		goto out;

	case USB_REQ_SET_ADDRESS:
		/* Status phase from udc */
		if (setup->bRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD
						| USB_RECIP_DEVICE))
			break;
		dr_ch9setaddress(wValue, wIndex, wLength);
		goto out;

	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		/* Status phase from udc */
	{

		if ((setup->bRequestType & (USB_RECIP_MASK | USB_TYPE_MASK))
				== (USB_RECIP_ENDPOINT | USB_TYPE_STANDARD)) {
			struct fsl_ep *target_ep = &udc->eps[get_pipe_by_windex(wIndex)];

			if (wValue != 0 || wLength != 0 ||
					target_ep->ep.maxpacket == 0xFFFF){
				DBG("SET/Clear Feature with illegal params.");
				break;
			}

			dr_ep_change_stall(target_ep,
					setup->bRequest == USB_REQ_SET_FEATURE ? 1 : 0);

		} else
			break;

		dr_ep0_in_status(0);
		dr_ep0_setup_token();
		goto out;
	}
	case USB_REQ_SET_INTERFACE:
	{
		VDBG("set interface = %u alt setting = %u", wIndex, wValue);
		break;
	}
	default:
		break;
	}

	/* Requests handled by gadget driver */

	if (wLength)
		udc->ep0_dir = (setup->bRequestType & USB_DIR_IN)
				?  USB_DIR_IN : USB_DIR_OUT;
	else
		udc->ep0_dir = USB_DIR_IN;

	spin_unlock(&udc->lock);
	ret = udc->driver->setup(&udc->gadget, setup);
	spin_lock(&udc->lock);

	/* error returned: stall ep0 */
	if(ret < 0){
		dr_ep0_setup_token();
		dr_ep_change_stall(&udc->eps[0],1);
		goto out;
	}

	if (wLength) {
		/* Data phase from gadget, status phase from udc */
		if(udc->ep0_dir == USB_DIR_IN)
			dr_ep0_out_status(0);
		else
			dr_ep0_in_status(0);

		udc->ep0_state = DATA_IN;

	} else {
		/* No data phase, IN status from gadget */
		udc->ep0_state = WAIT_FOR_IN_STATUS;
	}

	dr_ep0_setup_token();
	goto out;

out:
	regs->ctl.suspend_busy=0;
	spin_unlock(&udc->lock);
}

static void callback_ep0_in_status(struct usb_ep *_ep, struct usb_request *_req)
{
	udc->ep0_state = WAIT_FOR_SETUP;
	if(_req->status < 0) return;

	spin_lock(&udc->lock);
	if(udc->usb_state == USB_STATE_ADDRESS && regs->addr.addr == 0){
		INFO("address %u",udc->device_address);
		regs->addr.addr = udc->device_address;
		dr_ep0_reset();
	}

	spin_unlock(&udc->lock);
}
static void callback_ep0_out_status(struct usb_ep *_ep, struct usb_request *_req)
{
	if(udc->ep0_state == DATA_IN){
		VDBG("Reset IN queue");
		spin_lock(&udc->lock);
		dr_reset_pipe(1);
		udc->ep0_state = WAIT_FOR_SETUP;
		udc->ep0_dir = 123;
		spin_unlock(&udc->lock);
	}
}

char* get_pid_name(u8 pid){
	return pid == USB_PID_SETUP ? "SETUP" :
			pid == USB_PID_OUT ? "OUT" :
					pid == USB_PID_IN ? "IN" : "ERR";
}

static struct fsl_buffer_descriptor* buffer_for_entry(struct k70_bd_entry *bd,
		struct fsl_ep *ep){
	return ep->ep_bd[0]->bd_entry == bd ? ep->ep_bd[0] : ep->ep_bd[1];
}

static void dr_token_handler(struct status_register stat)
{
	struct fsl_req *req;
	int ep_num = stat.endp<<1 | stat.tx;
	struct fsl_ep *ep = &udc->eps[ep_num];
	struct k70_bd_entry *bd, *bd_alt;
	struct fsl_buffer_descriptor *buffer, *buffer_alt;
	u8 *target, *source;
	u32 count;

	bd = get_bd_entry(stat.endp, stat.tx, stat.odd);
	bd_alt = get_bd_entry_alt(bd);
	buffer = buffer_for_entry(bd, ep);
	buffer_alt = buffer->alt_buffer;

	ep->next_use_bd = buffer_alt;

	VDBG("EP%u %s %s\t%u byte %s (%u) Data%u",stat.endp,stat.tx ? "IN":"OUT",
		stat.odd ? "ODD" : "EVEN", bd->bytecount,
		get_pid_name(bd->tok_pid),
		bd->tok_pid,
		bd->data0_1);

	if(dr_ep_get_stall(ep)){
		DBG("%s stalled", ep->name);
		return;
	}

	if(list_empty(&ep->queue)){
		ERR("No requests in %s queue!",ep->ep.name);
		return;
	}
	req = list_first_entry(&ep->queue, struct fsl_req, queue);

	if(req != buffer->req){
		ERR("Requests do not match!");
	}

	if(bd->own){
		ERR("Buffer descriptor is owned by SIE!");
		return;
	}

	target = req->req.buf + req->req.actual;
	source = (u8*)bd->addr;
	count = bd->bytecount;

	req->req.actual += buffer->bd_entry->bytecount;
	ep->stat.bytes_transfered += buffer->bd_entry->bytecount;

	if(req->ep->dir == EP_DIR_OUT){
		req->state = REQ_PACKET_PENDING;
		if(bd->bytecount < req->ep->ep.maxpacket){
			req->state = REQ_READY;
		}

		if(req->req.actual == req->req.length)
			req->state = REQ_READY;
	}

	buffer->in_use = 0;
	buffer->req = NULL;
	ep->stat.buffers_freed ++;

	if(buffer_alt->in_use)
		ep->next_free_bd = buffer;
	else
		ep->next_free_bd = buffer_alt;

	if(req->state == REQ_READY){

		if(req->ep->dir == EP_DIR_OUT && use_own_buffer){
			memcpy(target, source, count);
			count = 0;
		}

		if(!(req->ep->dir == EP_DIR_IN && buffer_alt->req == req))
			dr_done(ep, req, 0);
	}

	dr_check_dequeue(ep);

	if(req->ep->dir == EP_DIR_OUT && count && use_own_buffer){
		memcpy(target, source, count);
	}
	return;
}

static void dr_clear_int(u8 bit){
	regs->istat.regvalue = (1 << bit);
}

/*
 * USB device controller interrupt handler
 * on entry: spinlock not held
 */
volatile u8 t1;
static irqreturn_t callback_udc_irq(int irq, void *_udc)
{
	union status_register_union stat;
	union int_status istat;

	spin_lock(&udc->lock);
	t1++;

	istat.regvalue = regs->istat.regvalue;
	istat.regvalue &= regs->inten.regvalue;

	/* stay in interrupt until no new interrupt is asserted */
	while(istat.regvalue != 0){
		if(istat.error){
			ERR("USB Error: %u",regs->errstat.regvalue);

			if(regs->errstat.btoerr) ERR("Timeout error");
			if(regs->errstat.btserr)ERR("bit stuff error");
			if(regs->errstat.crc16)ERR("CRC16 error");
			if(regs->errstat.crc5)ERR("CRC5 error");
			if(regs->errstat.dfn8)ERR("not 8 bits");
			if(regs->errstat.dmaerr)ERR("DMA error");
			if(regs->errstat.piderr)ERR("PID error");

			udc->error_count ++;

			clear_all_err_flags();
			dr_clear_int(INT_ERROR);
		}

		if(istat.resume){
			if(regs->inten.resume)
				DBG("RESUME");
			regs->inten.resume = 0;
			dr_clear_int(INT_RESUME);
		}
		if(istat.sleep){
			DBG("SLEEP");
			regs->inten.resume = 1;
			dr_clear_int(INT_SLEEP);
		}

		if(istat.usbrst){
			DBG("USB Reset Int");
			dr_reset_handler();
			goto out;
		}

		/* stall int only enabled then ep0 is stalled */
		if(istat.stall){
			VDBG("STALL send.");

			/* short delay before clearing the stall on EP0
			 * otherwise the host sometimes waits for an data phase */
			udelay(100);

			wmb();
			dr_ep_change_stall(&udc->eps[0],0);

			wmb();
			dr_clear_int(INT_STALL);
		}


		if(istat.tokdne){
			stat.regvalue = regs->stat.regvalue;
			dr_clear_int(INT_TOKDNE);
			dr_token_handler(stat.stat);
		}

		istat.regvalue = regs->istat.regvalue;
		istat.regvalue &= regs->inten.regvalue;
	}

out:
	spin_unlock(&udc->lock);
	t1--;
	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------
		PROC File System Support
-------------------------------------------------------------------------*/
#ifdef CONFIG_USB_GADGET_DEBUG_FILES

#include <linux/seq_file.h>
#include <linux/proc_fs.h>

static const char proc_filename[] = "driver/fsl_khci_udc";

static int fsl_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char *buf = page;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t, i;
	struct fsl_ep *ep = NULL;
	struct fsl_req *req;

	if (off != 0)
		return 0;

	spin_lock_irqsave(&udc->lock, flags);

	/* ------basic driver information ---- */
	t = scnprintf(next, size,
				DRIVER_DESC "\n"
				"%s version: %s\n"
				"Gadget driver: %s\n"
				"Recovered Bytes: %u\n"
				"Error count: %u\n\n",
				driver_name, DRIVER_VERSION,
				udc->driver ? udc->driver->driver.name : "(none)",
						udc->mark_count,
				udc->error_count);
		size -= t;
		next += t;


	/* ------fsl_udc, fsl_ep, fsl_request structure information ----- */
	for(i=0;i<USB_NUM_PIPES;i++){
		ep = &udc->eps[i];
		if(ep_maxpacket(ep) == 0xffff)
			continue;

		t = scnprintf(next, size, "For %s Maxpkt is 0x%x index is 0x%x\n",
				ep->ep.name, ep_maxpacket(ep), ep_index(ep));
		size -= t;
		next += t;


		if (list_empty(&ep->queue)) {
			t = scnprintf(next, size, "its req queue is empty\n");
			size -= t;
			next += t;
		} else {
			list_for_each_entry(req, &ep->queue, queue) {
				t = scnprintf(next, size,
					"req %p actual 0x%x length 0x%x buf %p\n",
					&req->req, req->req.actual,
					req->req.length, req->req.buf);
				size -= t;
				next += t;
			}
		}
		t = scnprintf(next, size, "Statistic: buffers: %u set %u freed  "
				"requests: %u queued %u complete %u bytes transfered\n\n",
				ep->stat.buffers_set, ep->stat.buffers_freed,
				ep->stat.requests_set, ep->stat.requests_complete,
				ep->stat.bytes_transfered);
					size -= t;
					next += t;
	}
	spin_unlock_irqrestore(&udc->lock, flags);

	*eof = 1;
	return count - size;
}

#define create_proc_file()	create_proc_read_entry(proc_filename, \
				0, NULL, fsl_proc_read, NULL)

#define remove_proc_file()	remove_proc_entry(proc_filename, NULL)

#else				/* !CONFIG_USB_GADGET_DEBUG_FILES */

#define create_proc_file()	do {} while (0)
#define remove_proc_file()	do {} while (0)

#endif				/* CONFIG_USB_GADGET_DEBUG_FILES */

/*----------------------------------------------------------------
 * Setup the fsl_ep struct for eps
 * Link fsl_ep->ep to gadget->ep_list
 *--------------------------------------------------------------*/
static int __init struct_ep_setup(u8 index, char *name, u8 link)
{
	struct fsl_ep *ep = &udc->eps[index];
	int endpt_reg_num = index / 2;
	int endpt_dir = index % 2;
	dma_addr_t buffer_dma;

	strcpy(ep->name, name);
	ep->ep.name = ep->name;

	ep->ep.ops = &fsl_ep_ops;
	ep->dir = endpt_dir;

	ep->endpt_reg = endpt_reg(endpt_reg_num);

	ep->ep_bd[0] = kzalloc(sizeof(struct fsl_buffer_descriptor), GFP_KERNEL);
	if(!ep->ep_bd[0])
		goto out;

	ep->ep_bd[1] = kzalloc(sizeof(struct fsl_buffer_descriptor), GFP_KERNEL);
	if(!ep->ep_bd[1])
		goto free_bd0;

	ep->ep_bd[0]->bd_entry = get_bd_entry(endpt_reg_num, endpt_dir, 0);
	ep->ep_bd[1]->bd_entry = get_bd_entry(endpt_reg_num, endpt_dir, 1);

	ep->ep_bd[0]->alt_buffer = ep->ep_bd[1];
	ep->ep_bd[1]->alt_buffer = ep->ep_bd[0];

	/* 128 Byte Buffer for each OUT EP */
	if(ep->dir == EP_DIR_OUT){
		buffer_dma = udc->bd_table_dma + 512 + 64 * index;
		ep->ep_bd[0]->bd_entry->addr = buffer_dma;
		ep->ep_bd[1]->bd_entry->addr = buffer_dma + 64;
	}

	/* for ep0: maxP defined in desc
	 * for other eps, maxP is set by epautoconfig() called by gadget layer
	 */
	ep->ep.maxpacket = (unsigned short) ~0;

	/* the queue lists any req for this ep */
	INIT_LIST_HEAD(&ep->queue);

	/* gagdet.ep_list used for ep_autoconfig so no ep0 */
	if (link)
		list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
	ep->gadget = &udc->gadget;

	return 0;

	free_bd0:
	kfree(ep->ep_bd[0]);
	out:
	return -ENOMEM;
}

static int __init ep0_requests_setup(void){
	struct usb_request *tmpreq;

	/* Initialize ep0 status request structure */
	tmpreq = fsl_alloc_request(&udc->eps[0].ep, GFP_KERNEL);
	if(tmpreq == NULL){
		ERR("malloc ep0 status request failed\n");
		return -ENOMEM;
	}
	udc->ep0_req_out = container_of(tmpreq, struct fsl_req, req);

	tmpreq->buf = kmalloc(USB_MAX_CTRL_PAYLOAD, GFP_KERNEL);
	if(!tmpreq->buf){
		ERR("malloc status_req->req.buf failed\n");
		goto err_free_ep0_req_out;
	}

	tmpreq = fsl_alloc_request(&udc->eps[1].ep, GFP_KERNEL);
	if(tmpreq == NULL){
		ERR("malloc ep0 status request failed\n");
		goto err_free_ep0_out_buf;
	}
	udc->ep0_req_in = container_of(tmpreq, struct fsl_req, req);

	tmpreq->buf = kmalloc(USB_MAX_CTRL_PAYLOAD, GFP_KERNEL);
	if(!tmpreq->buf){
		ERR("malloc status_req->req.buf failed\n");
		goto err_free_ep0_req_in;
	}

	tmpreq = fsl_alloc_request(&udc->eps[0].ep, GFP_KERNEL);
	if(tmpreq == NULL){
		ERR("malloc ep0 status request failed\n");
		goto err_free_ep0_in_buf;
	}
	udc->ep0_req_setup = container_of(tmpreq, struct fsl_req, req);

	tmpreq->buf = kmalloc(USB_MAX_CTRL_PAYLOAD, GFP_KERNEL);
	if(!tmpreq->buf){
		ERR("malloc status_req->req.buf failed\n");
		goto err_free_ep0_req_status;
	}

	return 0;
/*	err_free_ep0_status_buf:
		kfree(udc->ep0_req_setup->req.buf); */
	err_free_ep0_req_status:
		fsl_free_request(&udc->eps[0].ep, &udc->ep0_req_setup->req);
	err_free_ep0_in_buf:
		kfree(udc->ep0_req_in->req.buf);
	err_free_ep0_req_in:
		fsl_free_request(&udc->eps[0].ep, &udc->ep0_req_in->req);
	err_free_ep0_out_buf:
		kfree(udc->ep0_req_out->req.buf);
	err_free_ep0_req_out:
		fsl_free_request(&udc->eps[1].ep, &udc->ep0_req_out->req);
	return -ENOMEM;
}

/*
 * Driver probe function
 * board setup should have been done in the platform code, including clock setup
 */
static int __devinit fsl_udc_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = -ENODEV;
	int i;

	DBG("Starting Probe");

	udc = kzalloc(sizeof(struct fsl_udc), GFP_KERNEL);
	if (udc == NULL) {
		ERR("malloc udc failed\n");
		return -ENOMEM;
	}

	spin_lock_init(&udc->lock);
	spin_lock(&udc->lock);

	udc->stopped = 1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENXIO;
		goto err_kfree;
	}

	if (!request_mem_region(res->start, resource_size(res),
				driver_name)) {
		ERR("request mem region for %s failed\n", pdev->name);
		ret = -EBUSY;
		goto err_kfree;
	}

	regs = ioremap(res->start, resource_size(res));
	if (!regs) {
		ret = -ENOMEM;
		goto err_release_mem_region;
	}

	udc->irq = platform_get_irq(pdev, 0);
	if (!udc->irq) {
		ret = -ENODEV;
		goto err_iounmap;
	}

	ret = request_irq(udc->irq, callback_udc_irq, 0,	driver_name, udc);
	if (ret != 0) {
		ERR("cannot request irq %d err %d\n",
				udc->irq, ret);
		goto err_iounmap;
	}

	/*
	 * Enable clocks
	 */
	udc->clk = clk_get(&pdev->dev, "khci");
	if (IS_ERR(udc->clk)) {
		ret = -EFAULT;
		udc->clk = 0;
		dev_err(&pdev->dev, "failed get khci clock\n");
		goto err_free_irq;
	}
	clk_enable(udc->clk);

	udc->bd_table = dma_alloc_coherent(NULL, PAGE_SIZE, &udc->bd_table_dma,
			GFP_KERNEL);
	if(!udc->bd_table){
		ERR("no memory");
		ret = -ENOMEM;
		goto err_free_clk;
	}

	VDBG("bd_table = %X dma = %X",(u32)udc->bd_table, (u32)udc->bd_table_dma);

	/* Setup gadget structure */
	udc->gadget.ops = &fsl_gadget_ops;
	udc->gadget.is_dualspeed = 0;
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.name = driver_name;

	udc->eps = kzalloc(sizeof(struct fsl_ep) * USB_NUM_PIPES, GFP_KERNEL);
	if (!udc->eps) {
		ERR("malloc fsl_ep failed\n");
		ret = -ENOMEM;
		goto err_free_bd_table;
	}

	/* setup udc->eps[] for ep0 */
	struct_ep_setup(0, "ep0", 0);
	struct_ep_setup(1, "ep0", 0);
	/* for ep0: the desc defined here;
	 * for other eps, gadget layer called ep_enable with defined desc
	 */
	udc->eps[0].desc = &fsl_ep0_desc;
	udc->eps[0].ep.maxpacket = USB_MAX_CTRL_PAYLOAD;

	udc->eps[1].desc = &fsl_ep0_desc;
	udc->eps[1].ep.maxpacket = USB_MAX_CTRL_PAYLOAD;

	if(ep0_requests_setup() != 0){
		goto err_free_eps;
	}
	/* setup the udc->eps[] for non-control endpoints and link
	 * to gadget.ep_list */
	for (i = 1; i < USB_NUM_ENDPOINTS; i++) {
		char name[14];
		sprintf(name, "ep%dout", i);
		struct_ep_setup(i * 2, name, 1);
		sprintf(name, "ep%din", i);
		struct_ep_setup(i * 2 + 1, name, 1);
	}


	udc->resume_state = USB_STATE_NOTATTACHED;
	udc->usb_state = USB_STATE_POWERED;
	udc->ep0_dir = 0;
	udc->remote_wakeup = 0;	/* default to 0 on reset */

	dr_controller_setup();

	udc->gadget.ep0 = &udc->eps[0].ep;
	/* Setup gadget.dev and register with kernel */
	dev_set_name(&udc->gadget.dev, "gadget");
	udc->gadget.dev.release = fsl_udc_release;
	udc->gadget.dev.parent = &pdev->dev;
	ret = device_register(&udc->gadget.dev);
	if (ret < 0)
		goto err_free_eps;

	create_proc_file();

	check();
	DBG("Probe successful");

	spin_unlock(&udc->lock);
	return 0;

/*
err_unregister:
	device_unregister(&udc->gadget.dev);*/
err_free_eps:
	kfree(udc->eps);
err_free_bd_table:
	dma_free_coherent(NULL, PAGE_SIZE, udc->bd_table, udc->bd_table_dma);
err_free_clk:
	clk_disable(udc->clk);
err_free_irq:
	free_irq(udc->irq, udc);
err_iounmap:
	iounmap(regs);
err_release_mem_region:
	release_mem_region(res->start, res->end - res->start + 1);
err_kfree:
	spin_unlock(&udc->lock);
	kfree(udc);
	udc = NULL;
	return ret;
}

/*
 * Driver removal function
 * Free resources and finish pending transactions
 */
static int __exit fsl_udc_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	DECLARE_COMPLETION(done);

	VDBG("Removing driver");

	if (!udc)
		return -ENODEV;
	udc->done = &done;

	/* DR has been stopped in usb_gadget_unregister_driver() */
	remove_proc_file();

	/* Clear Interrupt Flags */
	clear_all_int_flags();
	clear_all_err_flags();

	/* Deactivate USB transceiver  */
	regs->usbctrl.susp=1;

	/* Disable USB module */
	regs->ctl.usben=0;

	/* Free memory */
	dma_free_coherent(NULL, PAGE_SIZE, udc->bd_table, udc->bd_table_dma);

	kfree(udc->ep0_req_setup->req.buf);
	fsl_free_request(&udc->eps[0].ep, &udc->ep0_req_setup->req);
	kfree(udc->ep0_req_in->req.buf);
	fsl_free_request(&udc->eps[0].ep, &udc->ep0_req_in->req);
	kfree(udc->ep0_req_out->req.buf);
	fsl_free_request(&udc->eps[1].ep, &udc->ep0_req_out->req);

	kfree(udc->eps);
	clk_disable(udc->clk);
	free_irq(udc->irq, udc);
	iounmap(regs);
	release_mem_region(res->start, res->end - res->start + 1);

	device_unregister(&udc->gadget.dev);
	/* free udc --wait for the release() finished */
	wait_for_completion(&done);

	return 0;
}
/*-------------------------------------------------------------------------
	Register entry point for the peripheral controller driver
--------------------------------------------------------------------------*/

static struct platform_driver udc_driver = {
	.probe   = fsl_udc_probe,
	.remove  = __exit_p(fsl_udc_remove),
	.driver  = {
		.name = "khci-hcd",
		.owner = THIS_MODULE,
	},
};

static int __init udc_init(void)
{
	printk(KERN_INFO "%s (%s)\n", driver_desc, DRIVER_VERSION);
	return platform_driver_register(&udc_driver);
}

module_init(udc_init);

static void __exit udc_exit(void)
{
	platform_driver_unregister(&udc_driver);
	printk(KERN_WARNING "%s unregistered\n", driver_desc);
}

module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
