/*
 * Based on arch/arm/plat-mxc/include/mach/mxc_ehci.h
 *
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _MACH_KINETIS_MXC_EHCI_H_
#define _MACH_KINETIS_MXC_EHCI_H_

#include <linux/init.h>

/* values for portsc field */
#define MXC_EHCI_PHY_LOW_POWER_SUSPEND	(1 << 23)
#define MXC_EHCI_FORCE_FS		(1 << 24)
#define MXC_EHCI_UTMI_8BIT		(0 << 28)
#define MXC_EHCI_UTMI_16BIT		(1 << 28)
#define MXC_EHCI_SERIAL			(1 << 29)
#define MXC_EHCI_MODE_UTMI		(0 << 30)
#define MXC_EHCI_MODE_PHILIPS		(1 << 30)
#define MXC_EHCI_MODE_ULPI		(2 << 30)
#define MXC_EHCI_MODE_SERIAL		(3 << 30)

/* values for flags field */
#define MXC_EHCI_INTERFACE_DIFF_UNI	(0 << 0)
#define MXC_EHCI_INTERFACE_DIFF_BI	(1 << 0)
#define MXC_EHCI_INTERFACE_SINGLE_UNI	(2 << 0)
#define MXC_EHCI_INTERFACE_SINGLE_BI	(3 << 0)
#define MXC_EHCI_INTERFACE_MASK		(0xf)

#define MXC_EHCI_POWER_PINS_ENABLED	(1 << 5)
#define MXC_EHCI_TTL_ENABLED		(1 << 6)

struct platform_device;
struct otg_transceiver;

struct mxc_usbh_platform_data {
	int (*init)(struct platform_device *pdev);
	int (*exit)(struct platform_device *pdev);

	unsigned int portsc;
	unsigned int flags;
	struct otg_transceiver *otg;
};

int mxc_set_usbcontrol(int port, unsigned int flags);

void __init kinetis_ehci_init(void);

#endif /* _MACH_KINETIS_MXC_EHCI_H_ */
