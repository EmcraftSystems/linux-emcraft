/*
 * Copyright (C) 2010,2011 Sergei Poselenov, Emcraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <mach/a2f.h>
#include <mach/eth.h>

/*
 * The MSS subsystem of SmartFusion contains one Ethernet MAC device.
 */
#if defined(CONFIG_A2F_MSS_ETHER)

#define ETH_CORE_SIZE		0xFFF
#define MAC_RST_CLR		(1<<4)

static struct resource eth_core_resources[] = {
	{
		.start          = ETH_CORE_BASE,
		.end            = ETH_CORE_BASE + ETH_CORE_SIZE,
		.flags          = IORESOURCE_MEM,
	}, {
		.start          = ETH_CORE_IRQ,
		.end            = ETH_CORE_IRQ,
		.flags          = IORESOURCE_IRQ,
	}
	
};

static struct platform_device eth_device = {
	.name           = "core10100",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(eth_core_resources),
	.resource       = eth_core_resources,
};

#endif	/* CONFIG_A2F_MSS_ETHER */

void __init a2f_eth_init()
{
#if defined(CONFIG_A2F_MSS_ETHER)
	/*
	 * Bring Ethernet MAC out of the power-up reset.
	 */
	A2F_SYSREG->soft_rst_cr &= ~MAC_RST_CLR;
	platform_device_register(&eth_device);
#endif
}
