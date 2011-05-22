/*
 * linux/arch/arm/mach-a2f/flash.c
 *
 * Copyright (C) 2011 Vladimir Khusainov, Emcraft Systems
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
#include <mach/a2f.h>
#include <mach/flash.h>

/*
 * Provide support for the external Flash.
 * This is board specific; A2F-LNX-EVB has a 8MBytes NOR Flash.
 */

/*
 * Where the NOR Flash resides in the physical map
 */
#define FLASH_BASE		0x74000000
#define FLASH_SIZE		(8 * 1024 * 1024)
static struct resource flash_resources[] = {
	{
		.start	= FLASH_BASE,
		.end	= FLASH_BASE + FLASH_SIZE-1,
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
 * 0-1ffff: 		U-boot environment
 * 20000-2fffff: 	Linux bootable image
 * 300000-7fffff:	JFFS2 filesystem
 */
static struct mtd_partition flash_partitions[] = {
	{
		.name	= "flash_uboot_env",
		.offset = 0,
		.size	= 0x20000,
	},
	{
		.name	= "flash_linux_image",
		.offset = 0x20000,
		.size	= (3 * 1024 * 1024 - 0x20000),
	},
	{
		.name	= "flash_jffs2",
		.offset = (3 * 1024 * 1024),
		.size	= (5 * 1024 * 1024),
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
void __init a2f_flash_init(void)
{

	/*
	 * Register a platform device for the external Flash.
	 * If there is no external Flash in your design, or 
	 * you don't want to use it in your application, just
	 * comment out the line below.
	 */
	platform_device_register(&flash_dev);		
}
