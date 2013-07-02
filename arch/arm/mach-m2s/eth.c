/*
 * (C) Copyright 2012
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
#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/clock.h>
#include <mach/eth.h>
#include <mach/platform.h>

#if defined(CONFIG_M2S_MSS_MAC)

/*
 * M2S MAC resources
 */
#define M2S_MAC_IRQ			12
#define M2S_MAC_BASE			0x40041000

/*
 * Ethernet platform device resources
 */
static struct resource			eth_m2s_dev_resources[] = {
	{
		.start	= M2S_MAC_BASE,
		.end	= M2S_MAC_BASE + 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= M2S_MAC_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

/*
 * Ethernet platform data.
 */
static struct eth_m2s_platform_data	eth_m2s_dev_data = {
	.freq_mdc	= 2500000,
};

/*
 * Ethernet platform device instance
 */
static struct platform_device		eth_m2s_dev = {
	.name		= "m2s_eth",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(eth_m2s_dev_resources),
	.resource	= eth_m2s_dev_resources,
};
#endif /* CONFIG_M2S_MSS_MAC */

void __init m2s_eth_init(void)
{
#if defined(CONFIG_M2S_MSS_MAC)
	int	platform;

	/*
	 * Fix-up and register platform device
	 */
	platform = m2s_platform_get();
	switch (platform) {
	case PLATFORM_M2S_SOM:
	case PLATFORM_M2S_FG484_SOM:
		eth_m2s_dev_data.freq_src = m2s_clock_get(CLCK_SYSREF);
		break;
	case PLATFORM_SF2_DEV_KIT:
		eth_m2s_dev_data.freq_src = m2s_clock_get(CLCK_SYSREF);
		eth_m2s_dev_data.flags = ETH_M2S_FLAGS_MODE_SGMII;
		break;
	default:
		break;
	}

	platform_set_drvdata(&eth_m2s_dev, &eth_m2s_dev_data);
	platform_device_register(&eth_m2s_dev);
#endif
}
