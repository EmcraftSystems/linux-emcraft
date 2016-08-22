/*
 * (C) Copyright 2014-2016
 * Emcraft Systems, <www.emcraft.com>
 * Pavel Boldin <paboldin@emcraft.com>
 * Dmitry Konyshev <probables@emcraft.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/platform.h>
#include <mach/usb.h>
#include <mach/iomux.h>

#include <linux/usb/dwc2-otg.h>

/*
 * USB HS interrupt
 */
#define STM32_USB_OTG_HS_IRQ		77

/*
 * USB HS ENR bit
 */
#define STM32_RCC_ENR_OTGHSEN		(1 << 29)
#define STM32_RCC_ENR_OTGHSULPIEN	(1 << 30)

#ifdef CONFIG_STM32_USB_OTG_HS_DEVICE
/*
 * USB platform device data
 */
static struct dwc2_otg_plat	usb_otg_hs_data = {
	/*
	 * Overall FIFO size is ~4KB (F4: 1012 words, F7: 1006 words).
	 * High-speed maximum packet sizes:
	 * - CTL (64), BLK (512), INT (1024), ISO (1024)
	 * High-speed ACM Serial gadget configuration:
	 * - CTL IN/OUT (64), BLK IN/OUT (512), INT IN (10)
	 * High-speed Mass-Storage gadget configuration:
	 * - CTL IN/OUT (64), BLK IN/OUT (512)
	 * High-speed Ethernet gadget configuration:
	 * - CTL IN/OUT (64), BLK IN/OUT (512), INT IN (8)
	 * Optimize FIFO distribution accordingly
	 */
#if defined(CONFIG_ARCH_STM32F7)
	.fifo = {
		   {	.name	= "default",
			.rx	= 238,
			.tx	= { 128, 128, 128, 128, 128, 128 },
		}, {	.name	= "g_serial",
			.rx	= 512,
			.tx	= { 64, 46, 384 },
		}, {	.name	= "g_file_storage",
			.rx	= 558,
			.tx	= { 64, 384 },
		}, {	.name	= "g_ether",
			.rx	= 512,
			.tx	= { 64, 46, 384 },
		},
	},
#else
	.fifo = {
		   {	.name	= "default",
			.rx	= 244,
			.tx	= { 128, 128, 128, 128, 128, 128 },
		}, {	.name	= "g_serial",
			.rx	= 512,
			.tx	= { 64, 52, 384 },
		}, {	.name	= "g_file_storage",
			.rx	= 564,
			.tx	= { 64, 384 },
		}, {	.name	= "g_ether",
			.rx	= 512,
			.tx	= { 64, 52, 384 },
		},
	},
#endif

	/* GCCFG: NOVBUSSENS | PWRDWN */
	.ggpio		= (1 << 21) | (1 << 16),
#if defined(CONFIG_ARCH_STM32F7)
	/* GOTGCTL: BVALOVAL | BVALOEN */
	.gotgctl	= (1 << 7) | (1 << 6),
#endif
};
#endif /* CONFIG_STM32_USB_OTG_HS_DEVICE */

/*
 * USB platform device resources
 */
static struct resource		usb_otg_hs_resources[] = {
	{
		.start	= STM32_USB_OTG_HS_BASE,
		.end	= STM32_USB_OTG_HS_BASE + 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= STM32_USB_OTG_HS_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

/*
 * USB platform device instance
 */
static struct platform_device	usb_otg_hs_device = {
#ifdef CONFIG_STM32_USB_OTG_HS_HOST
	.name		= DWC2_DRIVER_HOST,
#else
	.name		= DWC2_DRIVER_DEVICE,
#endif
	.resource	= usb_otg_hs_resources,
	.num_resources	= ARRAY_SIZE(usb_otg_hs_resources),
	.id		= 1,
#ifdef CONFIG_STM32_USB_OTG_HS_DEVICE
	.dev		= {
		.platform_data = &usb_otg_hs_data,
	},
#endif
};

int __init stm32_usb_otg_hs_init(void)
{
#if defined(CONFIG_STM32_USB_OTG_HS_DEVICE)
	if (stm32_udc_num++ > 0) {
		printk(KERN_ERR "USB Device Controller (full-speed) is already "
			"registered in the system.\nLinux USB Gadget sybsystem "
			"doesn't support more than one UDC so far!\n");
		return -EBUSY;
	}
#endif

	/*
	 * Init IO mux, and enable clocks
	 */
	stm32_iomux_usb_hs_init();
	STM32_RCC->ahb1enr |= STM32_RCC_ENR_OTGHSEN | STM32_RCC_ENR_OTGHSULPIEN;

	platform_device_register(&usb_otg_hs_device);

	return 0;
}
module_init(stm32_usb_otg_hs_init);

MODULE_DESCRIPTION("STM32 USB High-speed device");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:usb-hs");
