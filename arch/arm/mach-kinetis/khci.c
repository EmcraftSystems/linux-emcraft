/*
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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

#include <linux/module.h>
#include <linux/platform_device.h>

#include <mach/kinetis.h>
#include <mach/clock.h>
#include <mach/khci.h>

/*
 * Freescale Kinetis USB Full Speed controller register base
 */
#define KINETIS_KHCI_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00072000)

/*
 * Freescale Kinetis USB Full Speed controller interrupt
 */
#define KINETIS_KHCI_IRQ	73

/*
 * Data structures for the USB Host Controller platform device
 */
static struct resource khci_resources[] = {
	{
		.start	= KINETIS_KHCI_BASE,
		.end	= KINETIS_KHCI_BASE + 0x1ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= KINETIS_KHCI_IRQ,
		.end	= KINETIS_KHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device khci_dev = {
	.name = "khci-hcd",
	.id = 0,
	.resource = khci_resources,
	.num_resources = ARRAY_SIZE(khci_resources),
};

struct platform_device khci_udc = {
	.name = "khci-udc",
	.id = 0,
	.resource = khci_resources,
	.num_resources = ARRAY_SIZE(khci_resources),
};

void __init kinetis_khci_init(void)
{
	int	rv;

	/*
	 * Do not register USB Host if USB-FS clock was not set up
	 */
	if (!kinetis_clock_get(CLOCK_USBFS)) {
		pr_err("%s: KHCI clock is not configured.\n", __func__);
		goto out;
	}

	/*
	 * Register the USB Host controller
	 */
	rv = platform_device_register(&khci_dev);
	if (rv)
		pr_err("%s: register device failed.\n", __func__);

	/*
	 * Register the USB Device controller
	 */
	rv = platform_device_register(&khci_udc);
	if (rv)
		pr_err("%s: register device failed.\n", __func__);

out:
	return;
}
