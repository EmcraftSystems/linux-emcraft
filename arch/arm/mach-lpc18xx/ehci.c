/*
 * (C) Copyright 2014
 * Emcraft Systems, <www.emcraft.com>
 * Pavel Boldin <paboldin@emcraft.com>
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

#include <mach/platform.h>
#include <mach/clock.h>

/*
 * LPC43XX USB0 interrupt
 */
#define LPC43XX_USB0_IRQ		8

/*
 * LPC43XX USB0 base
 */
#define LPC43XX_USB0_BASE		0x40006000

static struct resource ehci_resources[] = {
	{
		.start	= LPC43XX_USB0_BASE,
		.end	= LPC43XX_USB0_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LPC43XX_USB0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/*
 * EHCI platform device instance
 */
static u64 lpc43xx_ehci_dma_mask = 0xffffffffUL;
static struct platform_device lpc43xx_ehci_device = {
	.name = "lpc43xx-ehci",
	.id = -1,
	.dev = {
		.dma_mask = &lpc43xx_ehci_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource	= ehci_resources,
	.num_resources	= ARRAY_SIZE(ehci_resources),
};

void __init lpc43xx_ehci_init(void)
{
	platform_device_register(&lpc43xx_ehci_device);
}
