/*
 * (C) Copyright 2011
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
#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/ohci.h>
#include <mach/platform.h>
#include <mach/clock.h>

/*
 * LPC178x/7x USB interrupt
 */
#define LPC178X_USB_IRQ		24

/*
 * The USB block works only with the clock rate of 48 MHz
 */
#define REQUIRED_USB_CLK_RATE	48000000

static struct resource ohci_resources[] = {
	{
		.start	= LPC178X_USB_BASE,
		.end	= LPC178X_USB_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LPC178X_USB_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/*
 * OHCI platform device instance
 */
static u64 lpc178x_ohci_dma_mask = 0xffffffffUL;
static struct platform_device lpc178x_ohci_device = {
	.name = LPC178X_OHCI_DRV_NAME,
	.id = -1,
	.dev = {
		.dma_mask = &lpc178x_ohci_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource	= ohci_resources,
	.num_resources	= ARRAY_SIZE(ohci_resources),
};

void __init lpc178x_ohci_init(void)
{
	if (lpc178x_clock_get(CLOCK_USBCLK) == REQUIRED_USB_CLK_RATE) {
		platform_device_register(&lpc178x_ohci_device);
	} else {
		pr_err(LPC178X_OHCI_DRV_NAME ": cannot initialize "
			"the OHCI USB Host driver since the USB clock "
			"was not set to 48 MHz by the bootloader\n");
	}
}
