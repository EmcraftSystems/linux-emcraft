/*
 * Freescale USB-FS OTG Device Controller Driver (Kirin USB Controller).
 * Implements support for USB Full-Speed controller available in Kinetis
 * controllers (K70,K60,...) and some Coldfire controllers (MCF52259,...).
 *
 * Copyright 2017 EmCraft Systems www.emcraft.com
 * Author: Dmitry Konyshev <probables@emcraft.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/dmapool.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/khci.h>

#define	DRIVER_VERSION	"0.9"

struct khci_request {
	struct usb_request		req;
	struct list_head		queue;
	unsigned			tx:1; /* for ep0 */
	unsigned			started:1;
	unsigned			datan:1;
};

struct khci_ep {
	struct usb_ep			ep;
	struct list_head		queue;
	unsigned long			irqs;
	const struct usb_endpoint_descriptor	*desc;
	char				name[14];
	u16				maxpacket;
	u8				bEndpointAddress;
	u8				bmAttributes;
	unsigned			datan:1;
	unsigned			stopped:1;
	struct khci_udc			*udc;
	u8				bd[2];	/* 'odd' BD index (tx/rx) */
};

struct khci_udc {
	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;
	struct device			*dev;
	struct khci_ep			ep[32];
	volatile struct khci_reg	*reg;	/* Register map */
	struct khci_bd			*bdt;	/* BD table (from DMA pool) */
	dma_addr_t			dma_addr;
	void				*dma_bufs; /* DMA buffers, 4 per EP
						      (rx/tx odd/even) */
	struct clk			*clk;	/* USB clock */
	int				irq;
	u16				devstat;
	unsigned			ep0_in:1;
	u8				usb_addr;
	struct completion		*done;
};

static const char driver_name[] = "khci-udc";
static struct khci_udc *the_controller;
static struct dma_pool *khci_bdt_pool;
static const char ep0name[] = "ep0";

static void nuke(struct khci_ep *ep, int status);
static int prepare_for_setup_packet(struct khci_udc *dev, gfp_t gfp_flags);

static int khci_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct khci_ep *ep = container_of(_ep, struct khci_ep, ep);
	struct khci_udc	*udc;
	volatile struct khci_reg *reg;
	unsigned long	flags;
	u16		maxp;

	dbg(1, "%s:%i %s\n", __FUNCTION__, __LINE__, _ep ? _ep->name : "NULL");

	maxp = desc ? le16_to_cpu(desc->wMaxPacketSize) : 0;

	/* catch various bogus parameters */
	if (!_ep || !desc
			|| desc->bDescriptorType != USB_DT_ENDPOINT
			|| ep->bEndpointAddress != desc->bEndpointAddress
			|| ep->maxpacket < maxp) {
		dbg(1, "%s, bad ep or descriptor\n", __func__);
		return -EINVAL;
	}

	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
				&& maxp != ep->maxpacket)
			|| maxp > ep->maxpacket
			|| !desc->wMaxPacketSize) {
		dbg(1, "%s, bad %s maxpacket\n", __func__, _ep->name);
		return -ERANGE;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
			&& ep->bmAttributes != USB_ENDPOINT_XFER_BULK
			&& desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		dbg(1, "%s, %s type mismatch\n", __func__, _ep->name);
		return -EINVAL;
	}

	if (desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		dbg(1, "%s, ISO nyet\n", _ep->name);
		return -EDOM;
	}

	udc = ep->udc;
	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		dbg(1, "%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	reg = udc->reg;

	local_irq_save(flags);

	ep->desc = desc;
	ep->irqs = 0;
	ep->stopped = 0;
	ep->ep.maxpacket = maxp;
	ep->datan = 0;

	/* enable endpoint communications */
	reg->ep[ep->bEndpointAddress & 0x0f].endpt =
		KHCI_EP_RETRYDIS | KHCI_EP_EPRXEN |
		KHCI_EP_EPTXEN | KHCI_EP_EPHSHK;

	local_irq_restore(flags);

	dbg(5, "%s enabled\n", _ep->name);
	return 0;
}

static int khci_ep_disable(struct usb_ep *_ep)
{
	struct khci_ep *ep;
	struct khci_udc	*udc;
	volatile struct khci_reg *reg;
	unsigned long flags;

	dbg(1, "%s:%i\n", __FUNCTION__, __LINE__);

	ep = container_of (_ep, struct khci_ep, ep);
	if (!_ep || !ep->desc) {
		dbg(1, "%s, %s not enabled\n", __func__,
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	udc = ep->udc;
	reg = udc->reg;

	local_irq_save(flags);

	nuke(ep, -ESHUTDOWN);

	ep->desc = NULL;
	ep->stopped = 1;

	reg->ep[ep->bEndpointAddress & 0x0f].endpt = 0;

	local_irq_restore(flags);

	dbg(5, "%s disabled\n", _ep->name);

	return 0;
}

static struct usb_request *
khci_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct khci_request *req;

	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);

	req = kzalloc(sizeof(*req), gfp_flags);
	if (req)
		INIT_LIST_HEAD(&req->queue);

	return req ? &req->req : NULL;
}

static void
khci_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct khci_request *req = container_of(_req, struct khci_request, req);

	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);

	if (_req)
		kfree(req);
}

/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct khci_ep *ep, struct khci_request *req, int status)
{
	unsigned		stopped = ep->stopped;

	dbg(1, "%s:%i %p %p %i %i\n", __FUNCTION__, __LINE__, req, req->req.complete, status, req->req.status);

	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN)
		dbg(8, "complete %s req %p stat %d len %u/%u\n",
			ep->ep.name, &req->req, status,
			req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;
	req->req.complete(&ep->ep, &req->req);
	ep->stopped = stopped;
}

static void khci_start_bd(struct khci_udc *dev, u8 ep, u8 tx,
		void *addr, u32 len, u8 data1)
{
	struct khci_bd *bd;
	struct khci_ep *kep;
	void *buf;

	dbg(1, "%s:%i ep%i %s addr %p len %i %s\n", __FUNCTION__, __LINE__,
		ep, tx ? "tx" : "rx", addr, len, data1 ? "data1" : "data0");

	BUG_ON((!addr && tx && len) || len > 64);

	kep = &dev->ep[ep ? ep + 16 * tx : 0];

	bd = KHCI_BD_PTR(dev, ep, tx, kep->bd[tx]);
	buf = KHCI_BD_BUF(dev, ep, tx, kep->bd[tx]);

	BUG_ON(bd->flg & KHCI_BD_OWN);

	if (tx && len)
		memcpy(buf, addr, len);

	bd->flg = KHCI_BD_FLG_SET(len, (!ep ? data1 : kep->datan));
	if (len)
		bd->adr = (u32)buf;
	else
		bd->adr = 0;

	dbg(1, "%s:%i bd %p adr %x flg %x\n", __FUNCTION__, __LINE__, bd, bd->adr, bd->flg);

	kep->bd[tx] ^= 1;
	kep->datan ^= 1;
}

static int
khci_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct khci_request	*req;
	struct khci_ep	*ep;
	struct khci_udc	*dev;
	unsigned long flags;
	int epn, tx;

	dbg(1, "%s:%i %s %p %i %p %i\n", __FUNCTION__, __LINE__,
			_ep->name, _req->buf,
			_req->length, _req->complete, _req->zero);

	req = container_of(_req, struct khci_request, req);
	if (unlikely(!_req || !_req->complete || !_req->buf
			|| !list_empty(&req->queue))) {
		dbg(0, "%s, bad params\n", __func__);
		return -EINVAL;
	}

	ep = container_of(_ep, struct khci_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		dbg(0, "%s, bad ep\n", __func__);
		return -EINVAL;
	}

	dev = ep->udc;
	if (unlikely(!dev->driver
			|| dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		dbg(0, "%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	local_irq_save(flags);

	epn = ep->bEndpointAddress & 0xf;
	tx = (ep->bEndpointAddress & USB_DIR_IN) >> 7;

	/* Silently ignore zero packet on ep0? We do status phase ourselves */
	if (!epn && !_req->length) {
		local_irq_restore(flags);
		return 0;
	}

	if (!epn)
		req->tx = dev->ep0_in;

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* kickstart this i/o queue? */
	dbg(8, "%s:%i %i %i\n", __FUNCTION__, __LINE__, list_empty(&ep->queue), ep->stopped);
	req->started = 0;
	if (list_empty(&ep->queue) && !ep->stopped) {
		khci_start_bd(dev, epn,
			(epn ? tx : req->tx), _req->buf,
			min((u32)ep->maxpacket, (u32)_req->length),
			(epn ? 0 : 1));
		req->started = 1;
	}

	/* irq handler advances the queue. */
	if (likely(req != NULL))
		list_add_tail(&req->queue, &ep->queue);

	local_irq_restore(flags);

	return 0;
}

/*
 *	nuke - dequeue ALL requests
 */
static void nuke(struct khci_ep *ep, int status)
{
	struct khci_request *req;

	dbg(8, "%s:%i addr %x\n", __FUNCTION__, __LINE__, ep->bEndpointAddress);

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next,
				struct khci_request,
				queue);
		done(ep, req, status);
	}
}

static int khci_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct khci_ep *ep;
	struct khci_request *req;
	unsigned long flags;

	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);

	ep = container_of(_ep, struct khci_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	local_irq_save(flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		local_irq_restore(flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);

	local_irq_restore(flags);

	return 0;
}

static int khci_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct khci_ep *ep;
	struct khci_udc	*dev;
	unsigned long flags;
	u8 epn;

	dbg(1, "%s:%i\n", __FUNCTION__, __LINE__);

	ep = container_of(_ep, struct khci_ep, ep);
	dev = ep->udc;

	epn = ep->bEndpointAddress & 0xf;

	local_irq_save(flags);

	if (value)
		dev->reg->ep[epn].endpt = KHCI_EP_STALL;
	else
		dev->reg->ep[epn].endpt =
			KHCI_EP_RETRYDIS | KHCI_EP_EPRXEN |
			KHCI_EP_EPTXEN | KHCI_EP_EPHSHK;

	local_irq_restore(flags);

	return 0;
}

static int khci_ep_fifo_status(struct usb_ep *_ep)
{
	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);
	return -EOPNOTSUPP;
}

static void khci_ep_fifo_flush(struct usb_ep *_ep)
{
	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);
}

static struct usb_ep_ops khci_ep_ops = {
	.enable		= khci_ep_enable,
	.disable	= khci_ep_disable,

	.alloc_request	= khci_ep_alloc_request,
	.free_request	= khci_ep_free_request,

	.queue		= khci_ep_queue,
	.dequeue	= khci_ep_dequeue,

	.set_halt	= khci_ep_set_halt,
	.fifo_status	= khci_ep_fifo_status,
	.fifo_flush	= khci_ep_fifo_flush,
};

static int __init
khci_ep_setup(char *name, u8 addr, u8 type, u32 maxp)
{
	struct khci_ep	*ep;
	struct khci_udc *udc = the_controller;

	/* OUT endpoints first, then IN */
	ep = &udc->ep[addr & 0xf];
	if (addr & USB_DIR_IN)
		ep += 16;

	if (type == USB_ENDPOINT_XFER_ISOC) {
		return -ENODEV;
	}

	dbg(8, "%s addr %02x maxp %d\n", name, addr, maxp);

	/* set up driver data structures */
	BUG_ON(strlen(name) >= sizeof(ep->name));
	strlcpy(ep->name, name, sizeof(ep->name));
	INIT_LIST_HEAD(&ep->queue);
	ep->bEndpointAddress = addr;
	ep->bmAttributes = type;
	ep->udc = udc;

	ep->ep.name = addr ? ep->name : ep0name;
	ep->ep.ops = &khci_ep_ops;
	ep->ep.maxpacket = ep->maxpacket = maxp;
	list_add_tail (&ep->ep.ep_list, &udc->gadget.ep_list);

	return 0;
}

static int khci_udc_get_frame(struct usb_gadget *_gadget)
{
	struct khci_udc	*udc;

	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);

	udc = container_of(_gadget, struct khci_udc, gadget);

	return udc->reg->frmnumh << 8 | udc->reg->frmnuml;
}

static int khci_udc_wakeup(struct usb_gadget *_gadget)
{
	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);
	return 0;
}

static int khci_udc_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);
	return 0;
}

static int khci_udc_pullup(struct usb_gadget *_gadget, int is_active)
{
	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);
	return 0;
}

static int khci_udc_vbus_draw(struct usb_gadget *_gadget, unsigned mA)
{
	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);
	return 0;
}

static const struct usb_gadget_ops khci_udc_ops = {
	.get_frame	= khci_udc_get_frame,
	.wakeup		= khci_udc_wakeup,
	.vbus_session	= khci_udc_vbus_session,
	.pullup		= khci_udc_pullup,
	.vbus_draw	= khci_udc_vbus_draw,
};

static void khci_udc_release(struct device *dev)
{
	dbg(8, "%s %s\n", __func__, dev_name(dev));
}

/*
 *	udc_reinit - initialize software state
 */
static void udc_reinit(struct khci_udc *dev)
{
	u32	i;

	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);

	/* device/ep0 records init */
	INIT_LIST_HEAD (&dev->gadget.ep_list);
	INIT_LIST_HEAD (&dev->gadget.ep0->ep_list);

	/* basic endpoint records init */
	for (i = 0; i < ARRAY_SIZE(dev->ep); i++) {
		struct khci_ep *ep = &dev->ep[i];

		if (i != 0)
			list_add_tail (&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->desc = NULL;
		ep->stopped = 0;
		INIT_LIST_HEAD (&ep->queue);
	}
}

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct khci_udc	*dev = the_controller;
	volatile struct khci_reg *reg = dev->reg;
	int			retval;

	dbg(8, "%s:%i %s\n", __FUNCTION__, __LINE__,
		driver ? driver->driver.name : "NULL");

	if (!driver
			|| driver->speed < USB_SPEED_FULL
			|| !driver->bind
			|| !driver->disconnect
			|| !driver->setup)
		return -EINVAL;
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	/* first hook up the driver ... */
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;

	dbg(8, "%s:%i %s %p\n", __FUNCTION__, __LINE__, driver->driver.name, dev);
	retval = device_add(&dev->gadget.dev);
	if (retval) {
		dbg(1, "device_add fail --> error %d\n", retval);
		goto add_fail;
	}
	dbg(8, "%s:%i %p\n", __FUNCTION__, __LINE__, driver->bind);
	retval = driver->bind(&dev->gadget);
	if (retval) {
		dbg(1, "bind to driver %s --> error %d\n",
				driver->driver.name, retval);
		goto bind_fail;
	}
	dbg(8, "%s:%i %s %p\n", __FUNCTION__, __LINE__, driver->driver.name, dev);

	/* ... then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 */
	dbg(1, "registered gadget driver '%s'\n", driver->driver.name);

	/*
	 * Clear interrupt statuses
	 */
	reg->istat = 0xFF;
	reg->erren = KHCI_ERR_MSK;

	/*
	 * Disable weak pull-downs; remove suspend state
	 */
	reg->usbctrl &= ~(KHCI_USBCTRL_SUSP | KHCI_USBCTRL_PDE);

	/*
	 * Enable D+ pull-up
	 */
	reg->otgctl = KHCI_OTGCTL_DPHIGH | KHCI_OTGCTL_OTGEN;

	/*
	 * Disable weak pull-downs; remove suspend state
	 */
	reg->usbctrl &= ~(KHCI_USBCTRL_SUSP | KHCI_USBCTRL_PDE);

	reg->bdtpage1 = (u32)dev->bdt >> 8;
	reg->bdtpage2 = (u32)dev->bdt >> 16;
	reg->bdtpage3 = (u32)dev->bdt >> 24;

	reg->ctl |= KHCI_CTL_ODDRST;
	memset(dev->bdt, 0, sizeof(struct khci_bd) * 4 * 16);

	/*
	 * Disable host mode
	 */
	reg->ctl &= ~KHCI_CTL_HOSTMODEEN;
	reg->inten = 0xff;
	reg->ctl |= KHCI_CTL_USBENSOFEN;

	return 0;

bind_fail:
	device_del (&dev->gadget.dev);

add_fail:
	dev->driver = NULL;
	dev->gadget.dev.driver = NULL;
	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

static void
stop_activity(struct khci_udc *dev, struct usb_gadget_driver *driver)
{
	int i;

	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);

	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < ARRAY_SIZE(dev->ep); i++) {
		struct khci_ep *ep = &dev->ep[i];

		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}

	/* report disconnect; the driver is already quiesced */
	if (driver)
		driver->disconnect(&dev->gadget);

	/* re-init driver-visible data structures */
	udc_reinit(dev);
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct khci_udc	*dev = the_controller;

	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver || !driver->unbind)
		return -EINVAL;

	local_irq_disable();
	stop_activity(dev, driver);
	local_irq_enable();

	driver->unbind(&dev->gadget);
	dev->gadget.dev.driver = NULL;
	dev->driver = NULL;

	device_del(&dev->gadget.dev);

	dbg(1, "unregistered gadget driver '%s'\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

static int queue_ep0_request(struct khci_udc *dev, gfp_t gfp_flags,
	int tx, int length, int data1,
	void (*complete)(struct usb_ep *ep, struct usb_request *req))
{
	struct usb_ep *ep = &dev->ep[0].ep;
	struct usb_request *rq;
	struct khci_ep *kep = container_of(ep, struct khci_ep, ep);
	struct khci_request *krq;

	dbg(1, "%s:%i %s %i %s %p\n", __FUNCTION__, __LINE__,
		tx ? "tx" : "rx", length, data1 ? "data1" : "data0", complete);

	rq = khci_ep_alloc_request(ep, gfp_flags);
	if (!rq)
		return -ENOMEM;

	rq->length = length;
	if (rq->length) {
		rq->buf = kzalloc(rq->length, gfp_flags);
		if (!rq->buf) {
			khci_ep_free_request(ep, rq);
			return -ENOMEM;
		}
	}

	rq->complete = complete;
	rq->context = dev;

	krq = container_of(rq, struct khci_request, req);
	krq->tx = !!tx;

	if (list_empty(&kep->queue)) {
		khci_start_bd(dev, 0, krq->tx, rq->buf, rq->length, data1);
		krq->started = 1;
		dbg(1, "start ep0 in %s %p\n", __func__, krq);
	}

	krq->req.status = -EINPROGRESS;
	list_add_tail(&krq->queue, &kep->queue);

	return 0;
}

static void on_set_address(struct usb_ep *ep, struct usb_request *req)
{
	struct khci_udc *dev;
	int status, rv;

	dev = req->context;
	status = req->status;

	khci_ep_free_request(ep, req);

	if (status == -ESHUTDOWN)
		return;

	if (status) {
		pr_err("%s: error %i confirming set_address; stall\n",
			driver_name, status);
		dev->reg->ep[0].endpt = KHCI_EP_STALL;
		return;
	}

	dev->reg->addr = dev->usb_addr;

	if ((rv = prepare_for_setup_packet(dev, GFP_ATOMIC)) < 0) {
		pr_err("%s: stall ep0 after set addr %i\n", __func__, rv);
		dev->reg->ep[0].endpt = KHCI_EP_STALL;
	}
}

static void on_gadget_rq(struct usb_ep *ep, struct usb_request *req)
{
	struct khci_udc *dev;
	int status, rv;

	dev = req->context;
	status = req->status;

	dbg(8, "%s:%i status %i\n", __FUNCTION__, __LINE__, status);

	khci_ep_free_request(ep, req);

	if (status == -ESHUTDOWN)
		return;

	if (status) {
		pr_err("%s: error %i confirming gadget rq; stall\n",
			driver_name, status);
		dev->reg->ep[0].endpt = KHCI_EP_STALL;
		return;
	}

	if ((rv = prepare_for_setup_packet(dev, GFP_ATOMIC)) < 0) {
		pr_err("%s: stall ep0 after set addr %i\n", __func__, rv);
		dev->reg->ep[0].endpt = KHCI_EP_STALL;
	}
}

static void on_setup_packet(struct usb_ep *ep, struct usb_request *req)
{
	struct usb_ctrlrequest ctrl_rq;
	struct khci_udc *dev;
	int rv, len, status;

	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);

	dev = req->context;
	memcpy(&ctrl_rq, req->buf, sizeof(ctrl_rq));
	len = req->actual;
	status = req->status;

	khci_ep_free_request(ep, req);

	if (status == -ESHUTDOWN)
		return;

	if (status || len != 8) {
		pr_err("%s: error %i receiving setup packet (len %i); stall\n",
				driver_name, status, len);
		dev->reg->ep[0].endpt = KHCI_EP_STALL;
		return;
	}

	dbg(1, "SETUP %02x.%02x v%04x i%04x l%04x\n",
		ctrl_rq.bRequestType, ctrl_rq.bRequest,
		le16_to_cpu(ctrl_rq.wValue),
		le16_to_cpu(ctrl_rq.wIndex),
		le16_to_cpu(ctrl_rq.wLength));

	switch (ctrl_rq.bRequest) {
	case USB_REQ_SET_ADDRESS:
		dev->usb_addr = le16_to_cpu(ctrl_rq.wValue);
		queue_ep0_request(dev, GFP_ATOMIC, 1, 0, 1, on_set_address);
		break;

	default:
		dev->ep0_in = (ctrl_rq.bRequestType & USB_DIR_IN) >> 7;
		rv = dev->driver->setup(&dev->gadget, &ctrl_rq);
		dbg(8, "dev->driver->setup ret %i\n", rv);
		if (rv < 0) {
			dev->reg->ep[0].endpt = KHCI_EP_STALL;
			return;
		}
		queue_ep0_request(dev, GFP_ATOMIC, !dev->ep0_in,
			0, 1, on_gadget_rq);
		break;
	}
}

static int prepare_for_setup_packet(struct khci_udc *dev, gfp_t gfp_flags)
{
	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);
	nuke(&dev->ep[0], -ESHUTDOWN);
	return queue_ep0_request(dev, gfp_flags, 0, 64, 0, on_setup_packet);
}

static void khci_usb_reset(struct khci_udc *dev)
{
	volatile struct khci_reg *reg = dev->reg;
	int i, rv;

	dbg(1, "%s:%i\n", __FUNCTION__, __LINE__);

	reg->istat = KHCI_INT_RST;

	reg->errstat = 0xff;
	reg->ctl |= KHCI_CTL_ODDRST;
	reg->usbctrl = 0;

	memset(dev->bdt, 0, 2 * 2 * 16 * sizeof(struct khci_bd));

	for (i = 0; i < ARRAY_SIZE(dev->ep); i++) {
		struct khci_ep *ep = &dev->ep[i];

		ep->stopped = 0;
		nuke(ep, -ESHUTDOWN);
		memset(ep->bd, 0, sizeof(ep->bd));
	}

	reg->ctl &= ~KHCI_CTL_ODDRST;
	reg->usbtrc0 = 0x40;
	reg->erren = KHCI_ERR_BTSERR | KHCI_ERR_DMAERR | KHCI_ERR_BTOERR |
		KHCI_ERR_DFN8 | KHCI_ERR_CRC16 | KHCI_ERR_CRC5EOF |
		KHCI_ERR_PIDERR;
	reg->inten = KHCI_INT_STALL | KHCI_INT_SLEEP | KHCI_INT_TOKDNE |
		KHCI_INT_SOFTOK | KHCI_INT_ERROR | KHCI_INT_RST;
	reg->ctl &= ~KHCI_CTL_TOKENBUSY;

	dev->gadget.speed = USB_SPEED_FULL;

	reg->ep[0].endpt = KHCI_EP_RETRYDIS | KHCI_EP_EPRXEN |
		KHCI_EP_EPTXEN | KHCI_EP_EPHSHK;
	reg->addr = 0;

	/* Start ep0 rx */
	if ((rv = prepare_for_setup_packet(dev, GFP_ATOMIC)) < 0) {
		pr_err("%s: stall ep0 due to internal error %i\n", __func__, rv);
		reg->ep[0].endpt = KHCI_EP_STALL;
	}
}

static void khci_start_next_transfer(struct khci_udc *dev)
{
	struct khci_ep *ep;
	struct khci_request *rq;
	int epn, tx;

	dbg(1, "%s:%i\n", __FUNCTION__, __LINE__);

	for (epn = 0; epn < 32; epn++) {
		ep = &dev->ep[epn];
		tx = epn >= 16;
		dbg(8, "%s:%i ep%i %sempty and %sstopped\n", __FUNCTION__, __LINE__, epn,
				list_empty(&ep->queue) ? "" : "not ",
				ep->stopped ? "" : "not ");
		if (!list_empty(&ep->queue) && !ep->stopped) {
			rq = list_first_entry(&ep->queue,
					struct khci_request, queue);
			if (!rq->started) {
				dbg(8, "start ep%i in %s %p %i\n", epn, __func__, rq, rq->started);
				khci_start_bd(dev, epn % 16,
					epn ? tx : rq->tx, rq->req.buf,
					min((u32)ep->maxpacket, (u32)rq->req.length),
					epn ? 0 : 1);
				rq->started = 1;
			}
			continue;
		}
	}
}

static void khci_usb_transfer_done(struct khci_udc *dev)
{
	volatile struct khci_reg *reg = dev->reg;
	u8 stat = reg->stat;
	u8 epn, tx, odd, len;
	volatile struct khci_bd	*bd;
	struct khci_ep *ep;
	struct khci_request *rq;
	char *p;

	dbg(1, "%s:%i %x\n", __FUNCTION__, __LINE__, stat);

	dev->reg->istat = KHCI_INT_TOKDNE;

	epn = KHCI_STAT_EP(stat);
	tx = KHCI_STAT_TX(stat);
	odd = KHCI_STAT_ODD(stat);

	bd = KHCI_BD_PTR(dev, epn, tx, odd);
	len = KHCI_BD_BC_GET(bd->flg);
	dbg(1, "bd done %p %p %x\n", bd, (void *)bd->adr, bd->flg);

	p = (char *)bd->adr;

	dbg(8, "%p %02x%02x%02x%02x%02x%02x%02x%02x\n", dev->driver->setup, p[0], p[1], p[2],
		p[3], p[4], p[5], p[6], p[7]);

	ep = &dev->ep[epn + (epn ? 16 * tx : 0)];

	if (list_empty(&ep->queue)) {
		pr_err("%s: rx/tx on empty queue of %s?", __func__, ep->ep.name);
	} else {
		rq = list_first_entry(&ep->queue, struct khci_request, queue);

		if (rq->req.actual + len > rq->req.length) {
			done(ep, rq, -EOVERFLOW);
			khci_start_next_transfer(dev);
			reg->ctl &= ~KHCI_CTL_TOKENBUSY;
			return;
		}

		if (!tx && len) {
			memcpy(rq->req.buf + rq->req.actual, p, len);
		}
		rq->req.actual += len;
		if (rq->req.actual == rq->req.length &&
				len == ep->ep.maxpacket && rq->req.zero) {
			khci_start_bd(dev, epn,
				epn ? tx : rq->tx, NULL, 0,
				KHCI_BD_DATA_GET(bd->flg));
		}
		else if (rq->req.actual == rq->req.length ||
				(!tx && len < ep->ep.maxpacket)) {
			done(ep, rq, 0);
			if (DEBUG >= 8)
				print_hex_dump_bytes(ep->ep.name,
					DUMP_PREFIX_NONE, rq->req.buf,
					rq->req.actual);
			khci_start_next_transfer(dev);
		} else {
			khci_start_bd(dev, epn, tx,
				tx ? rq->req.buf + rq->req.actual : NULL,
				min((u32)ep->maxpacket,
					(u32)(rq->req.length - rq->req.actual)),
				KHCI_BD_DATA_GET(bd->flg));
		}
	}

	reg->ctl &= ~KHCI_CTL_TOKENBUSY;
}

static irqreturn_t khci_udc_irq(int irq, void *_dev)
{
	struct khci_udc	*dev = _dev;
	volatile struct khci_reg *reg = dev->reg;
	u8 istat;

	istat = reg->istat;

	if (istat != KHCI_INT_SOFTOK && istat != 0)
		dbg(1, "%s:%i %x\n", __FUNCTION__, __LINE__, istat);

	if (istat & KHCI_INT_TOKDNE)
		khci_usb_transfer_done(dev);

	if (istat & KHCI_INT_RST)
		khci_usb_reset(dev);

	if (istat & KHCI_INT_ERROR) {
		dbg(1, "error irq %x\n", reg->errstat);
		dev->reg->ep[0].endpt = KHCI_EP_STALL;
	}

	reg->istat = 0xff & ~(KHCI_INT_RST | KHCI_INT_TOKDNE);
	if (reg->istat != 0) {
		dbg(1, "%s:%i %x\n", __FUNCTION__, __LINE__, reg->istat);
	}

	return IRQ_HANDLED;
}

/*
 * Probe device
 */
static int __devinit khci_udc_probe(struct platform_device *pdev)
{
	struct resource	*mres, *ires;
	void __iomem	*reg = NULL;
	struct khci_udc	*udc = NULL;
	unsigned long	msk;
	int		rv, irq, i;

	dbg(8, "%s:%i\n", __FUNCTION__, __LINE__);

	/*
	 * Verify and get resources
	 */
	mres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mres) {
		rv = -ENODEV;
		dev_err(&pdev->dev, "platform_get_resource MEM error.\n");
		goto out;
	}

	ires = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!ires) {
		rv = -ENODEV;
		dev_err(&pdev->dev, "platform_get_resource IRQ error.\n");
		goto out;
	}

	irq = ires->start;
	msk = ires->flags & IRQF_TRIGGER_MASK;

	reg = ioremap(mres->start, resource_size(mres));
	if (!reg) {
		rv = -ENOMEM;
		dev_err(&pdev->dev, "ioremap error.\n");
		goto out;
	}

	udc = kzalloc(sizeof(*udc), GFP_KERNEL);
	if (!udc) {
		rv = -ENOMEM;
		dev_err(&pdev->dev, "ioremap error.\n");
		goto out;
	}

	rv = request_irq(irq, khci_udc_irq, IRQF_DISABLED, driver_name, udc);
	if (rv < 0) {
		rv = -ENODEV;
		dev_err(&pdev->dev, "can't get irq.\n");
		goto out;
	}

	udc->irq = irq;

	dev_set_drvdata(&pdev->dev, udc);
	udc->dev = &pdev->dev;

	/*
	 * Enable clocks
	 */
	udc->clk = clk_get(&pdev->dev, "khci");
	if (IS_ERR(udc->clk)) {
		rv = -EFAULT;
		dev_err(&pdev->dev, "failed get khci clock %li\n",
			PTR_ERR(udc->clk));
		udc->clk = 0;
		goto out;
	}
	clk_enable(udc->clk);

	udc->reg = reg;

	udc->bdt = dma_pool_alloc(khci_bdt_pool, GFP_KERNEL, &udc->dma_addr);
	if (!udc->bdt) {
		rv = -ENOMEM;
		dev_err(&pdev->dev, "failed to alloc bdt\n");
		goto out;
	}
	udc->dma_bufs = (void *)udc->bdt + 16 * 2 * 2 * sizeof(struct khci_bd);

	udc->gadget.ops = &khci_udc_ops;
	udc->gadget.ep0 = &udc->ep[0].ep;
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.name = driver_name;

	device_initialize(&udc->gadget.dev);
	dev_set_name(&udc->gadget.dev, "gadget");
	udc->gadget.dev.release = khci_udc_release;
	udc->gadget.dev.parent = &pdev->dev;

	the_controller = udc;

	khci_ep_setup("ep0", 0, USB_ENDPOINT_XFER_CONTROL, 64);
	khci_ep_setup("unused", 0 | USB_DIR_IN, 0, 8);
	list_del_init(&udc->ep[0].ep.ep_list);

	for (i = 1; i < ARRAY_SIZE(udc->ep) / 2; i++) {
		char ep_name[32];

		sprintf(ep_name, "ep%i%s", i, "out");
		khci_ep_setup(ep_name, i | USB_DIR_OUT,
			USB_ENDPOINT_XFER_BULK, 64);
		sprintf(ep_name, "ep%i%s", i, "in");
		khci_ep_setup(ep_name, i | USB_DIR_IN,
			USB_ENDPOINT_XFER_BULK, 64);
	}

out:
	if (rv) {
		if (udc->irq)
			free_irq(udc->irq, udc);

		if (udc) {
			if (udc->bdt) {
				dma_pool_free(khci_bdt_pool, udc->bdt,
					      udc->dma_addr);
			}
			if (udc->clk)
				clk_disable(udc->clk);
			kfree(udc);
		}

		if (reg)
			iounmap(reg);
	}

	return rv;
}

static void khci_udc_shutdown(struct platform_device *_dev)
{
}

static int __exit khci_udc_remove(struct platform_device *pdev)
{
	struct khci_udc *dev = platform_get_drvdata(pdev);

	if (dev->driver)
		return -EBUSY;

	if (dev->irq)
		free_irq(dev->irq, dev);

	clk_put(dev->clk);

	platform_set_drvdata(pdev, NULL);
	the_controller = NULL;
	return 0;
}

static struct platform_driver udc_driver = {
	.shutdown	= khci_udc_shutdown,
	.remove		= __exit_p(khci_udc_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "khci-udc",
	},
};

static int __init udc_init(void)
{
	pr_info("%s: version %s\n", driver_name, DRIVER_VERSION);

	/*
	 * Allocate pool for BDT tables (2 BDs per dir (x2) per EP (x16))
	 * and 64 byte buffer for each BD
	 */
	khci_bdt_pool = dma_pool_create("khci_bdt", NULL,
			16 * 2 * 2 * (sizeof(struct khci_bd) + 64), 512, 0);

	return platform_driver_probe(&udc_driver, khci_udc_probe);
}
module_init(udc_init);

static void __exit udc_exit(void)
{
	platform_driver_unregister(&udc_driver);
}
module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Dmitry Konyshev");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:khci-udc");
