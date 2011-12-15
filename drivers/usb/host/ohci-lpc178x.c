/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 *
 * Bus Glue for NXP LPC178x
 *
 * Based on the LH7A404 driver.
 *
 * Modified for LPC24xx from ohci-lh7a404.c
 * by Embedded Artists AB
 *
 * Modified for LPC178x from ohci-lpc24xx.c by:
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * This file is licenced under the GPL.
 */

#include <linux/platform_device.h>

#include <mach/power.h>
#include <mach/ohci.h>

/*
 * USB Clock Control register
 */
/* Host clock enable */
#define LPC178X_USB_CLKCTRL_HOSTCKLEN_MSK	(1 << 0)
/* AHB clock enable */
#define LPC178X_USB_CLKCTRL_AHBCKLEN_MSK	(1 << 4)

#define USB_CLKCTRL_REQUIRED_MSK	(LPC178X_USB_CLKCTRL_AHBCKLEN_MSK | \
					LPC178X_USB_CLKCTRL_HOSTCKLEN_MSK)

/*
 * USB controller register map
 *
 * This structure should be mapped at 0x2008C000
 */
struct lpc178x_usb_regs {
	u32 rsv0[1021];

	/* 0x2008CFF4 */
	u32 clkctrl;	/* USB Clock Control */
	u32 clkst;	/* USB Clock Status */
};

/*
 * USB device registers base
 */
#define LPC178X_USB			((volatile struct lpc178x_usb_regs *) \
					LPC178X_USB_BASE)

static void lpc178x_hc_start(void)
{
	/*
	 * Enable power on the USB module
	 */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCUSB_MSK, 1);

	/*
	 * Enable the necessary clocks in the USB controller
	 */
	LPC178X_USB->clkctrl = USB_CLKCTRL_REQUIRED_MSK;
	/*
	 * Wait for availability of all necessary USB clocks
	 */
	while ((LPC178X_USB->clkst & USB_CLKCTRL_REQUIRED_MSK) !=
		USB_CLKCTRL_REQUIRED_MSK);
}

static void lpc178x_hc_stop(void)
{
	/* Disable power on the USB module */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCUSB_MSK, 0);
}

/**
 * usb_hcd_lpc178x_probe - initialize the HCD
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
int usb_hcd_lpc178x_probe(const struct hc_driver *driver,
			  struct platform_device *dev)
{
	int retval;
	struct usb_hcd *hcd;

	if (dev->resource[1].flags != IORESOURCE_IRQ) {
		pr_debug("resource[1] is not IORESOURCE_IRQ");
		retval = -ENOMEM;
		goto out;
	}

	hcd = usb_create_hcd(driver, &dev->dev, LPC178X_OHCI_DRV_NAME);
	if (!hcd) {
		retval = -ENOMEM;
		goto out;
	}
	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len = dev->resource[0].end - dev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("request_mem_region failed");
		retval = -EBUSY;
		goto err_res_mem;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_debug("ioremap failed");
		retval = -ENOMEM;
		goto err_iomap;
	}

	lpc178x_hc_start();
	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = usb_add_hcd(hcd, dev->resource[1].start, IRQF_DISABLED);
	if (retval != 0)
		goto err_hcd;
	/* Initialization was successful */
	goto out;

err_hcd:
	lpc178x_hc_stop();
	iounmap(hcd->regs);
err_iomap:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err_res_mem:
	usb_put_hcd(hcd);
out:
	return retval;
}

/**
 * usb_hcd_lpc178x_remove - shutdown processing
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_lh7a404_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
void usb_hcd_lpc178x_remove(struct usb_hcd *hcd, struct platform_device *dev)
{
	usb_remove_hcd(hcd);
	lpc178x_hc_stop();
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}

static int __devinit ohci_lpc178x_start(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	int ret;

	if ((ret = ohci_init(ohci)) < 0)
		goto out;

	if ((ret = ohci_run(ohci)) < 0) {
		err("can't start %s", hcd->self.bus_name);
		ohci_stop(hcd);
		goto out;
	}

	ret = 0;
out:
	return ret;
}

static const struct hc_driver ohci_lpc178x_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"LPC178x OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_lpc178x_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*
 * Platform driver initialization functions
 */
static int ohci_hcd_lpc178x_drv_probe(struct platform_device *pdev)
{
	int ret;

	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_lpc178x_probe(&ohci_lpc178x_hc_driver, pdev);
	return ret;
}

static int ohci_hcd_lpc178x_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_lpc178x_remove(hcd, pdev);
	return 0;
}

static struct platform_driver ohci_hcd_lpc178x_driver = {
	.probe		= ohci_hcd_lpc178x_drv_probe,
	.remove		= ohci_hcd_lpc178x_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= LPC178X_OHCI_DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

