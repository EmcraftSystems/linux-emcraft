/*
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
 * Anton Protopopov <antonp@emcraft.com>
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
#include <mach/kinetis.h>
#include <mach/spi.h>
#include <mach/spi-mvf.h>

#define SPI_PLAT_DEVICE(uid)						\
static struct resource kinetis_spi ## uid ##_resources[] = {		\
        {								\
                .start	= KINETIS_SPI ## uid ## _BASE,			\
                .end	= KINETIS_SPI ## uid ## _BASE + SZ_4K - 1,	\
                .flags	= IORESOURCE_MEM,				\
        },								\
	{								\
                .start	= KINETIS_SPI ## uid ## _IRQ,			\
                .flags	= IORESOURCE_IRQ,				\
        },								\
};									\
struct platform_device kinetis_spi ## uid ##_device = {			\
	.name           = "kinetis-dspi",				\
	.id             = uid,						\
	.num_resources  = ARRAY_SIZE(kinetis_spi ## uid ## _resources),	\
	.resource       = kinetis_spi ## uid ## _resources,		\
};									\
static const struct spi_mvf_master kinetis_spi ## uid ## _pdata = {	\
        .bus_num = uid,							\
        .chipselect = NULL,						\
        .num_chipselect = 1,						\
        .cs_control = NULL,						\
}

#ifdef CONFIG_KINETIS_SPI0
#define KINETIS_SPI0_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x0002C000)
#define KINETIS_SPI0_IRQ	26
SPI_PLAT_DEVICE(0);
#endif

#ifdef CONFIG_KINETIS_SPI1
#define KINETIS_SPI1_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x0002D000)
#define KINETIS_SPI1_IRQ	27
SPI_PLAT_DEVICE(1);
#endif

#ifdef CONFIG_KINETIS_SPI2
#define KINETIS_SPI2_BASE	(KINETIS_AIPS1PERIPH_BASE + 0x0002C000)
#define KINETIS_SPI2_IRQ	28
SPI_PLAT_DEVICE(2);
#endif

void __init kinetis_spi_init(void)
{
	int	platform;

	platform = kinetis_platform_get();
	switch (platform) {
	case PLATFORM_KINETIS_K70_SOM:
	case PLATFORM_KINETIS_K61_SOM:
#ifdef CONFIG_KINETIS_SPI0
		platform_device_add_data(&kinetis_spi0_device, &kinetis_spi0_pdata, sizeof(kinetis_spi0_pdata));
		platform_device_register(&kinetis_spi0_device);
#endif
#ifdef CONFIG_KINETIS_SPI1
		platform_device_add_data(&kinetis_spi1_device, &kinetis_spi1_pdata, sizeof(kinetis_spi1_pdata));
		platform_device_register(&kinetis_spi1_device);
#endif
#ifdef CONFIG_KINETIS_SPI2
		platform_device_add_data(&kinetis_spi2_device, &kinetis_spi2_pdata, sizeof(kinetis_spi2_pdata));
		platform_device_register(&kinetis_spi2_device);
#endif
		break;
	default:
		break;
	}
}
