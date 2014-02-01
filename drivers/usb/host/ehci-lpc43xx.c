/*
 * (c) 2014 EmCraft Systems
 * Pavel Boldin <paboldin@emcraft.com>

 * EHCI HCD (Host Controller Driver) for USB.
 *
 * Bus Glue for NXP LPC43XX
 *
 * Based on "ohci-au1xxx.c" by Matt Porter <mporter@kernel.crashing.org>
 *
 * Modified for AMD Alchemy Au1200 EHC
 *  by K.Boge <karsten.boge@amd.com>
 *
 * This file is licenced under the GPL.
 */

#include <linux/platform_device.h>

#include <mach/lpc18xx.h>
#include <mach/clock.h>

extern int usb_disabled(void);

#define	LPC43XX_CAP_REG_SHIFT	0x100
#define	LPC43XX_USBMODE_SHIFT	0xA8

static int lpc43xx_start_ehc(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int ret;

	/* Start and configure PLL0USB clock */

	/* Stop first */
	LPC18XX_CGU->pll0usb.ctrl |= 1; /* Power down PLL */

	/* Configure */
	LPC18XX_CGU->pll0usb.np_div = (98 << 0) | (514 << 12);
	LPC18XX_CGU->pll0usb.mdiv = (0xB<<17)|(0x10<<22)|(0<<28)|(0x7FFA<<0);

	/* Run from here */
	LPC18XX_CGU->pll0usb.ctrl = LPC18XX_CGU_CLKSEL_XTAL | (0x3<<2) | (1<<4);

	while(!(LPC18XX_CGU->pll0usb.stat & 1));

	/* Enable PHY */
	LPC18XX_CREG->creg0 &= ~(1<<5);

	hcd->has_tt = 1;

	ret = ehci_reset(ehci);
	if (ret)
		ehci_err(ehci, "lpc43xx EHCI init");
	return ret;
}

static void lpc43xx_stop_ehc(struct usb_hcd *hcd)
{
	LPC18XX_CGU->pll0usb.ctrl |= 1; /* Power down PLL */

	/* Disable PHY */
	LPC18XX_CREG->creg0 |= 1<<5;
}

static const struct hc_driver ehci_lpc43xx_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "LPC43XX EHCI",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset			= ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	.clear_tt_buffer_complete	= ehci_clear_tt_buffer_complete,
};

static int ehci_hcd_lpc43xx_drv_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	int ret;

	if (usb_disabled())
		return -ENODEV;

	if (pdev->resource[1].flags != IORESOURCE_IRQ) {
		pr_err("resource[1] is not IORESOURCE_IRQ");
		return -ENOMEM;
	}
	hcd = usb_create_hcd(&ehci_lpc43xx_hc_driver, &pdev->dev, "LPC43XX");
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_err("request_mem_region failed");
		ret = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_err("ioremap failed");
		ret = -ENOMEM;
		goto err2;
	}

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs + LPC43XX_CAP_REG_SHIFT;
	ehci->regs = hcd->regs + LPC43XX_CAP_REG_SHIFT + HC_LENGTH(readl(&ehci->caps->hc_capbase));

	ret = lpc43xx_start_ehc(hcd);
	if (ret)
		goto err3;

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	ret = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);
	if (ret == 0) {
		platform_set_drvdata(pdev, hcd);
		return ret;
	}

err3:
	lpc43xx_stop_ehc(hcd);
	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	usb_put_hcd(hcd);
	return ret;
}

static int ehci_hcd_lpc43xx_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	lpc43xx_stop_ehc(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver ehci_hcd_lpc43xx_driver = {
	.probe		= ehci_hcd_lpc43xx_drv_probe,
	.remove		= ehci_hcd_lpc43xx_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver = {
		.name	= "lpc43xx-ehci",
		.owner	= THIS_MODULE,
		.pm	= NULL,
	}
};

MODULE_ALIAS("platform:lpc43xx-ehci");
