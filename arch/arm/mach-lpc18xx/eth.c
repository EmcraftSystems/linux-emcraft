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
#include <linux/delay.h>
#include <linux/stm32_eth.h>

#include <mach/eth.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/lpc18xx.h>

/*
 * MCU-specific parameters of the Ethernet MAC module
 */
#define LPC18XX_MAC_IRQ		5
#define LPC18XX_MAC_BASE	0x40010000

/*
 * Ethernet platform device resources
 */
static struct resource eth_resources[] = {
	{
		.start	= LPC18XX_MAC_BASE,
		.end	= LPC18XX_MAC_BASE + SZ_8K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LPC18XX_MAC_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

/*
 * Hitex LPC4350 Eval board Ethernet platform data.
 *
 * Use the same platform data also for the Hitex LPC1850 Eval board
 * since these two boards are compatible.
 */
static struct stm32_eth_data hitex_lpc4350_eth_data = {
	.frame_max_size	= 2044,
	.tx_buf_num	= 24,
	.rx_buf_num	= 32,
	.phy_id		= 1,
};

/*
 * Ethernet platform device instance
 */
static struct platform_device eth_device = {
	.name		= STM32_ETH_DRV_NAME,
	.num_resources	= ARRAY_SIZE(eth_resources),
	.resource	= eth_resources,
/*
 * The .id should be set to -1 to indicate the only instance,
 * but it results in the "PHY: ffffffff:01 - Link is Down"
 * ugly printout from the eth driver, so we set it to 0.
 */
	.id		= 0,
};

/*
 * Initialize the Ethernet controller
 */
static int __init eth_setup(int rmii)
{
	int timeout;
	int rv;

	/*
	 * Initialize clock driver to make Ethernet clock available
	 * to the driver.
	 */
	lpc18xx_clock_init();

	if (!rmii) {
		/*
		 * This clock configuration is valid only for MII
		 */
		LPC18XX_CGU->phy_rx_clk =
			LPC18XX_CGU_CLKSEL_ENET_RX | LPC18XX_CGU_AUTOBLOCK_MSK;
		LPC18XX_CGU->phy_tx_clk =
			LPC18XX_CGU_CLKSEL_ENET_TX | LPC18XX_CGU_AUTOBLOCK_MSK;
	}

	/*
	 * Choose the MII Ethernet mode
	 */
	LPC18XX_CREG->creg6 =
		(LPC18XX_CREG->creg6 & ~LPC18XX_CREG_CREG6_ETHMODE_MSK) |
		(rmii ? LPC18XX_CREG_CREG6_ETHMODE_RMII :
			LPC18XX_CREG_CREG6_ETHMODE_MII);

	/*
	 * Reset the Ethernet module of the MCU
	 */
	LPC18XX_RGU->ctrl0 = LPC18XX_RGU_CTRL0_ETHERNET;

	/*
	 * Wait for the Ethernet module to exit the reset state
	 */
	timeout = 10;
	rv = -ETIMEDOUT;
	while (timeout-- > 0) {
		if (!(LPC18XX_RGU->active_status0 &
		    LPC18XX_RGU_CTRL0_ETHERNET)) {
			mdelay(1);
		} else {
			timeout = 0;
			rv = 0;
		}
	}

	if (rv < 0) {
		pr_err("%s: Reset of the Ethernet module timed out.\n",
			__func__);
	}

	return rv;
}

void __init lpc18xx_eth_init(void)
{
	int	platform, rmii = 0;

	/*
	 * Fix-up and register platform device
	 */
	platform = lpc18xx_platform_get();
	switch (platform) {
	case PLATFORM_LPC18XX_EA_LPC4357_EVAL:
	case PLATFORM_LPC18XX_HITEX_LPC4350_EVAL:
		rmii = 1;
		/* Pass thru */
	case PLATFORM_LPC18XX_HITEX_LPC1850_EVAL:
		eth_device.dev.platform_data = &hitex_lpc4350_eth_data;
		break;
	default:
		break;
	}

	/*
	 * Initialize the Ethernet controller
	 */
	if (eth_setup(rmii) < 0)
		goto out;


	platform_device_register(&eth_device);
out:
	;
}
