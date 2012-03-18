/*
 * (C) Copyright 2011, 2012
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

#include <mach/eth.h>
#include <mach/platform.h>

#if defined(CONFIG_LPC178X_MAC)

/*
 * LPC178x/7x MAC interrupt
 */
#define LPC178X_MAC_IRQ		28

/*
 * Ethernet platform device resources
 */
static struct resource		eth_resources[] = {
	{
		.start	= LPC178X_MAC_BASE,
		.end	= LPC178X_MAC_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LPC178X_MAC_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

/*
 * This platform data structure supports the following boards:
 *   1. Embedded Artists LPC1788-32
 *   2. Emcraft LPC-LNX-EVB
 */
static struct lpc178x_eth_data lpc178x_common_eth_data = {
	.phy_irq        = -1,
	.phy_mask       = 0xFFFFFFF0,
};

/*
 * Ethernet platform device instance
 */
static u64 lpc178x_mac_dma_mask = 0xffffffffUL;
static struct platform_device lpc178x_net_device = {
	.name = LPC178X_ETH_DRV_NAME,
	.id = 0,
	.dev = {
		.dma_mask = &lpc178x_mac_dma_mask,
		.coherent_dma_mask = 0xffffffffUL,
	},
	.num_resources = ARRAY_SIZE(eth_resources),
	.resource = eth_resources,
};

#endif /* CONFIG_LPC178X_MAC */

void __init lpc178x_eth_init(void)
{
#if defined(CONFIG_LPC178X_MAC)
	int	platform;

	/*
	 * Fix-up and register platform device
	 */
	platform = lpc178x_platform_get();
	switch (platform) {
	case PLATFORM_LPC178X_EA_LPC1788:
	case PLATFORM_LPC178X_LNX_EVB:
		lpc178x_net_device.dev.platform_data = &lpc178x_common_eth_data;
		break;
	default:
		break;
	}

	platform_device_register(&lpc178x_net_device);
#endif
}
