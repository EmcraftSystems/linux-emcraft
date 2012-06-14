/*
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/fec.h>

#include <mach/platform.h>
#include <mach/kinetis.h>
#include <mach/eth.h>

/*
 * Freescale Kinetis MAC register base
 */
#define KINETIS_MAC_BASE		(KINETIS_AIPS1PERIPH_BASE + 0x00040000)

/*
 * Freescale Kinetis MAC interrupt
 */
#define KINETIS_MAC_TX_IRQ	76	/* Transmit interrupt */
#define KINETIS_MAC_RX_IRQ	77	/* Receive interrupt */
#define KINETIS_MAC_ERR_IRQ	78	/* Error/misc interrupt */

/*
 * We limit the MDC rate to 800 kHz, because the rate of 2.5 MHz leads to data
 * corruption when reading the PHY registers (we experienced data corruptions
 * on TWR-K60N512.)
 */
#define KINETIS_MII_SPEED_LIMIT		800000

/*
 * Ethernet platform device resources
 */
static struct resource kinetis_eth_resources[] = {
	{
		.start	= KINETIS_MAC_BASE,
		.end	= KINETIS_MAC_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= KINETIS_MAC_TX_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= KINETIS_MAC_RX_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= KINETIS_MAC_ERR_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/*
 * The FEC driver platform-specific configuration for the Freescale TWR-K60N512
 * and TWR-K70F120M boards.
 *
 * This configuration is compatible with the Emcraft K70-SOM board.
 */
static struct fec_platform_data kinetis_twr_fec_platform_data = {
	/* Use RMII to communicate with the PHY */
	.flags = FEC_FLAGS_RMII,
	/* MDIO clock rate limit */
	.mii_clk_limit = KINETIS_MII_SPEED_LIMIT,
};

/*
 * Ethernet platform device instance
 */
static struct platform_device kinetis_net_device = {
	.name = "fec",
	.id = -1,
	.num_resources = ARRAY_SIZE(kinetis_eth_resources),
	.resource = kinetis_eth_resources,
};

void __init kinetis_eth_init(void)
{
	int	platform;

	/*
	 * Fix-up and register platform device
	 */
	platform = kinetis_platform_get();
	switch (platform) {
	case PLATFORM_KINETIS_TWR_K70F120M:
	case PLATFORM_KINETIS_K70_SOM:
	case PLATFORM_KINETIS_K61_SOM:
		kinetis_net_device.dev.platform_data =
			&kinetis_twr_fec_platform_data;
		break;
	default:
		break;
	}

	platform_device_register(&kinetis_net_device);
}
