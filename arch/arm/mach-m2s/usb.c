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
#include <linux/dma-mapping.h>

#include <linux/usb/musb.h>
#include <linux/gpio.h>

#include <mach/m2s.h>
#include <mach/clock.h>
#include <mach/usb.h>
#include <mach/platform.h>

#define M2S_USB_BASE		0x40043000
#define M2S_USB_IRQ		20
#define M2S_USB_DMA_IRQ		21

/* MSS GPIO number to control the ULPI USB PHY reset on SF2-DEV-KIT. */
#define ULPI_RST_GPIO	29

#ifdef CONFIG_USB_MUSB_SOC

static struct resource musb_resources[] = {
	{
		.start	= M2S_USB_BASE,
		.end	= M2S_USB_BASE + 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= M2S_USB_IRQ,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= M2S_USB_DMA_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct musb_hdrc_eps_bits musb_eps[] = {
	{	"ep1_tx", 10,	},
	{	"ep1_rx", 10,	},
	{	"ep2_tx", 9,	},
	{	"ep2_rx", 9,	},
	{	"ep3_tx", 3,	},
	{	"ep3_rx", 3,	},
	{	"ep4_tx", 3,	},
	{	"ep4_rx", 3,	},
	{	"ep5_tx", 3,	},
	{	"ep5_rx", 3,	},
	{	"ep6_tx", 3,	},
	{	"ep6_rx", 3,	},
	{	"ep7_tx", 3,	},
	{	"ep7_rx", 3,	},
	{	"ep8_tx", 2,	},
	{	"ep8_rx", 2,	},
	{	"ep9_tx", 2,	},
	{	"ep9_rx", 2,	},
	{	"ep10_tx", 2,	},
	{	"ep10_rx", 2,	},
	{	"ep11_tx", 2,	},
	{	"ep11_rx", 2,	},
	{	"ep12_tx", 2,	},
	{	"ep12_rx", 2,	},
	{	"ep13_tx", 2,	},
	{	"ep13_rx", 2,	},
	{	"ep14_tx", 2,	},
	{	"ep14_rx", 2,	},
	{	"ep15_tx", 2,	},
	{	"ep15_rx", 2,	},
};

static struct musb_hdrc_config musb_config = {
	.multipoint	= 1,
	.dyn_fifo	= 1,
	.soft_con	= 1,
	.dma		= 1,
	.num_eps	= 16,
	.dma_channels	= 7,
	.dma_req_chan	= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
	.ram_bits	= 12,
	.eps_bits	= musb_eps,
};

static struct musb_hdrc_platform_data musb_plat = {
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode		= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode		= MUSB_PERIPHERAL,
#endif
	.config		= &musb_config,
	.power		= 250,			/* up to 500 mA */
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

static struct platform_device musb_device = {
	.name		= "musb_hdrc",
	.id		= -1,
	.dev = {
		.dma_mask		= &musb_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &musb_plat,
	},
	.num_resources	= ARRAY_SIZE(musb_resources),
	.resource	= musb_resources,
};

static void __init usb_musb_init(void)
{
	if (platform_device_register(&musb_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (MUSB) device\n");
		return;
	}
}
#endif /* CONFIG_USB_MUSB_SOC */

/*
 * Register the M2S specific USB devices with the kernel
 */
void __init m2s_usb_init(void)
{
#if defined(CONFIG_USB_MUSB_SOC)
	int p = m2s_platform_get();
	/*
	 * Bring USB out of reset
	 */
	M2S_SYSREG->soft_reset_cr &= ~(1 << 14);
	if (p == PLATFORM_SF2_DEV_KIT) {
		/*
		 * Bring USB PHY out of reset
		 */
		if (gpio_request(ULPI_RST_GPIO, "ULPI_RST") != 0 ||
			gpio_direction_output(ULPI_RST_GPIO, 0) != 0) {
			printk(KERN_ERR "Unable to bring USB PHY out of reset\n");
			return;
		}
	}

	usb_musb_init();
#endif
}
