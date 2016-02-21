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
 * USB FS interrupt
 */
#define STM32_USB_OTG_FS_IRQ		67

/*
 * USB FS ENR bit
 */
#define STM32_RCC_ENR_OTGFSEN		(1 << 7)

#ifdef CONFIG_STM32_USB_OTG_FS_DEVICE
/*
 * USB platform device data
 */
static struct dwc2_otg_plat	usb_otg_fs_data = {
	/*
	 * Overall FIFO size is 1280 bytes (320 words).
	 * Full-speed maximum packet sizes:
	 * - CTL (64), BLK (64), INT (64), ISO (1023)
	 * Full-speed ACM Serial gadget configuration:
	 * - CTL IN/OUT (64), BLK IN/OUT (64), INT IN (10)
	 * Full-speed Mass-Storage gadget configuration:
	 * - CTL IN/OUT (64), BLK IN/OUT (64)
	 * Optimize FIFO distribution accordingly
	 */
	.fifo = {
		   {	.name	= "default",
			.rx	= 128,
			.tx	= { 48, 48, 48, 48 },
		}, {	.name	= "g_serial",
			.rx	= 160,
			.tx	= { 64, 32, 64 },
		}, {	.name	= "g_file_storage",
			.rx	= 160,
			.tx	= { 64, 96 },
		},
	},

#if defined(CONFIG_ARCH_STM32F7)
	/* GCCFG: PWRDWN */
	.ggpio		= (1 << 16),
	/* GOTGCTL: BVALOVAL | BVALOEN */
	.gotgctl	= (1 << 7) | (1 << 6),
#else
	/* GCCFG: NOVBUSSENS | PWRDWN */
	.ggpio		= (1 << 21) | (1 << 16),
#endif
};
#endif /* CONFIG_STM32_USB_OTG_FS_DEVICE */

/*
 * USB platform device resources
 */
static struct resource		usb_otg_fs_resources[] = {
	{
		.start	= STM32_USB_OTG_FS_BASE,
		.end	= STM32_USB_OTG_FS_BASE + 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= STM32_USB_OTG_FS_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

/*
 * USB platform device instance
 */
static struct platform_device	usb_otg_fs_device = {
#ifdef CONFIG_STM32_USB_OTG_FS_HOST
	.name		= DWC2_DRIVER_HOST,
#else
	.name		= DWC2_DRIVER_DEVICE,
#endif
	.resource	= usb_otg_fs_resources,
	.num_resources	= ARRAY_SIZE(usb_otg_fs_resources),
	.id		= 0,
#ifdef CONFIG_STM32_USB_OTG_FS_DEVICE
	.dev		= {
		.platform_data = &usb_otg_fs_data,
	},
#endif
};

int __init stm32_usb_otg_fs_init(void)
{
	if (stm32_udc_num++ > 0) {
		printk(KERN_ERR "USB Device Controller (high-speed) is already "
			"registered in the system.\nLinux USB Gadget sybsystem "
			"doesn't support more than one UDC so far!\n");
		return -EBUSY;
	}

	/*
	 * Init IO mux, and enable clocks
	 */
	stm32_iomux_usb_fs_init();
	STM32_RCC->ahb2enr |= STM32_RCC_ENR_OTGFSEN;

	platform_device_register(&usb_otg_fs_device);

	return 0;
}
module_init(stm32_usb_otg_fs_init);

MODULE_DESCRIPTION("STM32 USB Full-speed device");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:usb-hs");
