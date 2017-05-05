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

#include "khci-hcd.h"

/*****************************************************************************
 * Variables local to this module:
 *****************************************************************************/

/*
 * Virtual root hub specific descriptor
 */
static u8			khci_root_hub_dsc[] = {
	0x09,	/* blength */
	0x29,	/* bDescriptorType;hub-descriptor */
	0x01,	/* bNbrPorts */
	0x11,	/* wHubCharacteristics */
	0x00,
	0x01,	/* bPwrOn2pwrGood;2ms */
	0x00,	/* bHubContrCurrent;0mA */
	0x00,	/* DeviceRemoveable */
	0xff,	/* PortPwrCtrlMask */
};

/*****************************************************************************
 * USB framework API:
 *****************************************************************************/

/*
 * Get connected devices status
 */
int khci_hc_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct khci_hcd	*khci = hcd_to_khci(hcd);
	int		rv = 0;

	dbg(2, "%s: (%x)\n", __func__, khci->vrh.port.wPortChange);

	if (khci->vrh.port.wPortChange &
	    (USB_PORT_STAT_C_CONNECTION | USB_PORT_STAT_C_ENABLE |
	     USB_PORT_STAT_C_SUSPEND | USB_PORT_STAT_C_RESET |
	     USB_PORT_STAT_C_OVERCURRENT)) {
		*buf = 1 << 1;
		rv = 1;
	}

	return rv;
}

/*
 * Hub control command handler
 */
int khci_hc_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
			u16 wIndex, char *buf, u16 wLength)
{
	struct khci_hcd		*khci = hcd_to_khci(hcd);
	struct usb_hub_status	*hs;
	struct usb_port_status	*ps;
	unsigned long		flags;
	int			rv = 0;

	dbg(1, "%s(0x%x,%d,%d)\n", __func__, typeReq, wValue, wIndex);

	spin_lock_irqsave(&khci->lock, flags);

	switch (typeReq) {
	case GetHubDescriptor:
		memcpy(buf, khci_root_hub_dsc, sizeof(khci_root_hub_dsc));
		break;
	case GetHubStatus:
		hs = (struct usb_hub_status *)buf;
		hs->wHubStatus = cpu_to_le16(khci->vrh.hub.wHubStatus);
		hs->wHubChange = cpu_to_le16(khci->vrh.hub.wHubChange);
		break;
	case GetPortStatus:
		ps = (struct usb_port_status *)buf;
		ps->wPortStatus = cpu_to_le16(khci->vrh.port.wPortStatus);
		ps->wPortChange = cpu_to_le16(khci->vrh.port.wPortChange);
		break;
	case SetPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_RESET:
			spin_unlock_irqrestore(&khci->lock, flags);

			khci->reg->ctl |= KHCI_CTL_RESET;
			msleep(30);
			khci->reg->ctl &= ~KHCI_CTL_RESET;

			spin_lock_irqsave(&khci->lock, flags);
			break;
		case USB_PORT_FEAT_POWER:
			khci->vrh.port.wPortStatus |= USB_PORT_STAT_POWER;
		default:
			break;
		}
		break;
	case ClearPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_C_ENABLE:
			khci->vrh.port.wPortChange &=
				~USB_PORT_STAT_C_ENABLE;
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			khci->vrh.port.wPortChange &=
				~USB_PORT_STAT_C_CONNECTION;
			break;
		}
		break;
	default:
		break;
	}

	spin_unlock_irqrestore(&khci->lock, flags);

	return rv;
}
