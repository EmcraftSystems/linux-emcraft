/*
 * (C) Copyright 2014
 * Emcraft Systems, <www.emcraft.com>
 * Pavel Boldin <paboldin@emcraft.com>
 * Dmitry Konyshev <probables@emcraft.com>
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

#include <mach/platform.h>
#include <mach/usb.h>

/*
 * STM32 USB interrupt
 */
#define STM32_USB_OTG_HS_IRQ		77
#define STM32_USB_OTG_FS_IRQ		67

/*
 * STM32 ENR bit
 */
#define STM32_RCC_ENR_OTGHSEN		(1 << 29)
#define STM32_RCC_ENR_OTGHSULPIEN	(1 << 30)

#define STM32_RCC_ENR_OTGFSEN		(1 << 7)

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
 * Ethernet platform device instance
 */
static struct platform_device	usb_otg_fs_device = {
	.name		= "dwc2",
	.resource	= usb_otg_fs_resources,
	.num_resources	= ARRAY_SIZE(usb_otg_fs_resources),
	.id		= 0,
};

void __init stm32_usb_otg_fs_init(void)
{
	/*
	 * Enable clocks
	 */
	STM32_RCC->ahb2enr |= STM32_RCC_ENR_OTGFSEN;

	platform_device_register(&usb_otg_fs_device);
}
