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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/mxc_ehci.h>

/*
 * Freescale Kinetis USB High Speed controller register base
 */
#define KINETIS_USBHS_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00034000)

/*
 * Freescale Kinetis USB High Speed controller interrupt
 */
#define KINETIS_USBHS_IRQ	96

/*
 * This function is called by the "ehci-mxc" driver on USB host initialization
 */
int mxc_set_usbcontrol(int port, unsigned int flags)
{
	return 0;
}
EXPORT_SYMBOL(mxc_set_usbcontrol);

/*
 * Data structures for the USB Host Controller platform device
 */

static u64 usbh1_dmamask = ~(u32)0;

static struct resource usbhs_resources[] = {
	{
		.start	= KINETIS_USBHS_BASE,
		.end	= KINETIS_USBHS_BASE + 0x1ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= KINETIS_USBHS_IRQ,
		.end	= KINETIS_USBHS_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct mxc_usbh_platform_data usbhs_pdata = {
	.portsc	= MXC_EHCI_MODE_ULPI,
	.flags	= MXC_EHCI_POWER_PINS_ENABLED | MXC_EHCI_INTERFACE_SINGLE_UNI,
};

struct platform_device usbhs_dev = {
	.name = "mxc-ehci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
		.dma_mask = &usbh1_dmamask,
		.platform_data = &usbhs_pdata,
	},
	.resource = usbhs_resources,
	.num_resources = ARRAY_SIZE(usbhs_resources),
};

/*
 * When using a TWR-K70-SOM-BSB or TWR-K70F120M board as an MCU board
 * in a Tower System, the MCU's PTB.8 pin is connected to the RSTOUT_B signal
 * on TWR-SER2 and can be used to reset the USB PHY and Ethernet PHY
 * on the TWR-SER2 board.
 */
#define TWRSER2_RESET_GPIO	KINETIS_GPIO_MKPIN(KINETIS_GPIO_PORT_B, 8)

void __init kinetis_ehci_init(void)
{
	int platform;
	int rv;

	/*
	 * Reset the USB High Speed ULPI PHY installed on the TWR-SER2 board
	 */
	platform = kinetis_platform_get();
	if (platform == PLATFORM_KINETIS_TWR_K70F120M ||
	    platform == PLATFORM_KINETIS_K70_SOM ||
	    platform == PLATFORM_KINETIS_K61_SOM) {
		gpio_direction_output(TWRSER2_RESET_GPIO, 1);

		/* Activate USB PHY reset */
		gpio_set_value(TWRSER2_RESET_GPIO, 0);
		mdelay(100);

		/* Exit from reset */
		gpio_set_value(TWRSER2_RESET_GPIO, 1);
		mdelay(100);
	}

	/* Do not register USB Host if USB-HS clock was not set up */
	if (!kinetis_clock_get(CLOCK_USBHS)) {
		pr_err("%s: USB-HS clock is not configured.\n", __func__);
		goto out;
	}

	/*
	 * Register the USB Host controller
	 */
	rv = platform_device_register(&usbhs_dev);
	if (rv)
		pr_err("%s: mxc_register_device failed.\n", __func__);

out:
	;
}
