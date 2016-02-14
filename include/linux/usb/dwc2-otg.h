/* include/linux/usb/dwc2-otg.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S3C USB2.0 High-speed / OtG platform information
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __LINUX_USB_DWC2_OTG_H
#define __LINUX_USB_DWC2_OTG_H

struct platform_device;

/**
 * struct dwc2_otg_plat - platform data for high-speed otg/udc
 * rx_fifo_sz:	size of all OUT EPs FIFO, in words
 * tx_fifo_sz:	sizes of IN EPs, in words
 * ggpio:	GGPIO register value
 */
struct dwc2_otg_plat {
	u32		rx_fifo_sz;
	u32		tx_fifo_sz[15];
	u32		ggpio;

	int		phy_type;

	int (*phy_init)(struct platform_device *pdev, int type);
	int (*phy_exit)(struct platform_device *pdev, int type);
};

#endif /* __iLINUX_USB_DWC2_OTG_H */
