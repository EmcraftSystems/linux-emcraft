/*
 * (C) Copyright 2011
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
#include <linux/stm32_eth.h>

#include <mach/eth.h>
#include <mach/platform.h>

#if defined(CONFIG_STM32_MAC)

/*
 * STM32 MAC interrupt
 */
#define STM32_MAC_IRQ			61

/*
 * STM32 ENR bit
 */
#define STM32_RCC_ENR_ETHMACEN		(1 << 25)
#define STM32_RCC_ENR_ETHMACTXEN	(1 << 26)
#define STM32_RCC_ENR_ETHMACRXEN	(1 << 27)

/*
 * Ethernet platform device resources
 */
static struct resource		eth_resources[] = {
	{
		.start	= STM32_MAC_BASE,
		.end	= STM32_MAC_BASE + 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= STM32_MAC_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

/*
 * STM3220G-EVAL board Ethernet platform data.
 *
 * Use the same platform data also for the STM3240G-EVAL board since these two
 * boards are compatible.
 */
static struct stm32_eth_data	stm3220g_eval_eth_data = {
	.frame_max_size	= 2044,
	.tx_buf_num	= 24,
	.rx_buf_num	= 32,
	.phy_id		= 1,
};

/*
 * Ethernet platform device instance
 */
static struct platform_device	eth_device = {
	.name		= STM32_ETH_DRV_NAME,
	.resource	= eth_resources,
	.num_resources	= 2,
/*
 * The .id should be set to -1 to indicate the only instance,
 * but it results in the "PHY: ffffffff:01 - Link is Down"
 * ugly printout from the eth driver, so we set it to 0.
 */
	.id		= 0,
};
#endif /* CONFIG_STM32_MAC */

void __init stm32_eth_init(void)
{
#if defined(CONFIG_STM32_MAC)
	int	platform;

	/*
	 * Enable clocks
	 */
	STM32_RCC->ahb1enr |= STM32_RCC_ENR_ETHMACEN   |
			      STM32_RCC_ENR_ETHMACTXEN |
			      STM32_RCC_ENR_ETHMACRXEN;

	/*
	 * Fix-up and register platform device
	 */
	platform = stm32_platform_get();
	switch (platform) {
	case PLATFORM_STM32_STM3220G_EVAL:
	case PLATFORM_STM32_STM3240G_EVAL:
	case PLATFORM_STM32_STM_SOM:
		eth_device.dev.platform_data = &stm3220g_eval_eth_data;
		break;
	default:
		break;
	}

	platform_device_register(&eth_device);
#endif
}
