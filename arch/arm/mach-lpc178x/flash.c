/*
 * Flash resource initialization for LPC178x/7x-based boards
 *
 * Copyright (C) 2011, 2012
 * Vladimir Khusainov, Emcraft Systems, vlad@emcraft.com
 * Sergei Poselenov, Emcraft Systems, sposelenov@emcraft.com
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
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
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/serial_8250.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>
#include <mach/platform.h>
#include <mach/lpc178x.h>
#include <mach/flash.h>

/*
 * Provide support for the external Flash.
 * This is board specific:
 *   1. EA-LPC1788 board has a 4MBytes NOR Flash (located on the Base Board).
 *   2. Emcraft LPC-LNX-EVB has a 16 MBytes NOR Flash.
 */

/*
 * Where the NOR Flash resides in the physical map
 */
#define FLASH_BASE		0x80000000
static struct resource flash_resources[] = {
	{
		.start	= FLASH_BASE,
		.flags	= IORESOURCE_MEM,
	},
};

/*
 * Flash partitioning. Generally speaking, this is application
 * specific. However, the default U-boot does make some assumptions
 * about Flash partitioning, specifically:
 *
 * 0-1ffff:		U-boot environment
 * 20000-end of Flash:	Linux bootable image + whatever else.
 *
 * Based on these assumptions, we define the following Flash partitions:
 *
 * 0-1ffff:		U-boot environment
 * 20000-2fffff:	Linux bootable image
 * 300000-end of Flash:	JFFS2 filesystem
 */
#define FLASH_IMAGE_OFFSET	0x20000
#define FLASH_JFFS2_OFFSET	(3*1024*1024)
static struct mtd_partition flash_partitions[] = {
	{
		.name	= "flash_uboot_env",
		.offset = 0,
		.size	= FLASH_IMAGE_OFFSET,
	},
	{
		.name	= "flash_linux_image",
		.offset = FLASH_IMAGE_OFFSET,
		.size	= (FLASH_JFFS2_OFFSET - FLASH_IMAGE_OFFSET),
	},
	{
		.name	= "flash_jffs2",
		.offset = FLASH_JFFS2_OFFSET,
	},
};
static struct physmap_flash_data flash_data = {
	.width		= 2,
	.nr_parts	= ARRAY_SIZE(flash_partitions),
	.parts		= flash_partitions,
};

/*
 * Platform device for the external Flash
 */
static struct platform_device flash_dev = {
	.name           = "physmap-flash",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(flash_resources),
	.resource       = flash_resources,
	.dev		= {
	.platform_data = &flash_data,
	},
};

/*
 * Register the Flash platform device with the kernel.
 */
void __init lpc178x_flash_init(void)
{
	unsigned int size = 0;

	/*
	 * Calculate Flash and partition sizes at run time
	 */
	switch (lpc178x_platform_get()) {
	case PLATFORM_LPC178X_EA_LPC1788:
		size = 4*1024*1024;
		break;
	case PLATFORM_LPC178X_LNX_EVB:
		size = 16*1024*1024;
		break;
	default:
		printk(KERN_ERR "%s: Unknown platform %#x, exit\n", __func__,
			lpc178x_platform_get());
		goto xit;
	}

	flash_resources[0].end = flash_resources[0].start + size - 1;
	flash_partitions[2].size = size - FLASH_JFFS2_OFFSET;

	/*
	 * Register a platform device for the external Flash.
	 * If there is no external Flash in your design, or
	 * you don't want to use it in your application, just
	 * comment out the line below.
	 */
	platform_device_register(&flash_dev);
xit:
	return;
}
