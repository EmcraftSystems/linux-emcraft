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
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#include <mach/gpio.h>

#if defined(CONFIG_KINETIS_SPI2_GPIO)
/*
 * GPIO pins used as SPI2 bus signals on the TWR-K70-SOM-BSB baseboard.
 *
 * The same pins are used for the SPI2 bus on TWR-K70F120M. On these boards
 * SPI is used particularly to configure some type of LCDs connected
 * the Secondary Elevator board, e.g. FutureElectronics TWR-PIM-41WVGA.
 */
static struct spi_gpio_platform_data kinetis_spi2_gpio_pdata = {
	.sck		= KINETIS_GPIO_MKPIN(KINETIS_GPIO_PORT_D, 12),
	.mosi		= KINETIS_GPIO_MKPIN(KINETIS_GPIO_PORT_D, 13),
	.miso		= KINETIS_GPIO_MKPIN(KINETIS_GPIO_PORT_D, 14),
	.num_chipselect	= 1,
};

static struct platform_device kinetis_spi2_gpio = {
	.name			= "spi_gpio",
	.id			= 0,
	.dev.platform_data	= &kinetis_spi2_gpio_pdata,
};
#endif /* CONFIG_KINETIS_SPI2_GPIO */

void __init kinetis_spi_gpio_init(void)
{
#if defined(CONFIG_KINETIS_SPI2_GPIO)
	platform_device_register(&kinetis_spi2_gpio);
#endif /* CONFIG_KINETIS_SPI2_GPIO */
}
