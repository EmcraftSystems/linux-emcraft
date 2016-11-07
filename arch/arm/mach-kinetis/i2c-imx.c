/*
 * (C) Copyright 2015
 * Emcraft Systems, <www.emcraft.com>
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
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
#include <linux/i2c.h>

#include <mach/kinetis.h>
#include <mach/i2c.h>
#include <mach/platform.h>

#define KINETIS_I2C0_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00066000)
#define KINETIS_I2C1_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00067000)

#define KINETIS_I2C0_IRQ	24
#define KINETIS_I2C1_IRQ	25

static struct imxi2c_platform_data kinetis_imx_i2c_data = {
	.bitrate = 100000,
};

static struct resource imx_i2c_resources[] = {
	{
		.start  = KINETIS_I2C0_BASE,
		.end    = KINETIS_I2C0_BASE + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	}, {
		.start  = KINETIS_I2C0_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device kinetis_imx_i2c_device = {
	.name           = "imx-i2c",
	.id             = 0,
	.resource       = imx_i2c_resources,
	.num_resources  = ARRAY_SIZE(imx_i2c_resources),
};

void __init kinetis_i2c_imx_init(void)
{
	kinetis_imx_i2c_device.dev.platform_data = &kinetis_imx_i2c_data;

	platform_device_register(&kinetis_imx_i2c_device);
}
