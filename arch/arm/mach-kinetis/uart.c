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

#include <mach/kinetis.h>
#include <mach/power.h>
#include <mach/uart.h>

/*
 * Kinetis UART interrupt numbers (for status sources and error sources)
 */
#define KINETIS_UART0_STAT_IRQ	45
#define KINETIS_UART0_ERR_IRQ	46
#define KINETIS_UART1_STAT_IRQ	47
#define KINETIS_UART1_ERR_IRQ	48
#define KINETIS_UART2_STAT_IRQ	49
#define KINETIS_UART2_ERR_IRQ	50
#define KINETIS_UART3_STAT_IRQ	51
#define KINETIS_UART3_ERR_IRQ	52
#define KINETIS_UART4_STAT_IRQ	53
#define KINETIS_UART4_ERR_IRQ	54
#define KINETIS_UART5_STAT_IRQ	55
#define KINETIS_UART5_ERR_IRQ	56

/*
 * UART platform device resources
 */
#define UART_PLAT_RESOURCES(uid)					       \
static struct resource kinetis_uart_## uid ##_resources[] = {		       \
	{								       \
		.start	= KINETIS_UART## uid ##_BASE,			       \
		.end	= KINETIS_UART## uid ##_BASE + 1,		       \
		.flags	= IORESOURCE_MEM,				       \
	},								       \
	{								       \
		.start	= KINETIS_UART## uid ##_STAT_IRQ,		       \
		.flags	= IORESOURCE_IRQ,				       \
	},								       \
	{								       \
		.start	= KINETIS_UART## uid ##_ERR_IRQ,		       \
		.flags	= IORESOURCE_IRQ,				       \
	},								       \
}

/*
 * UART platform device instance
 */
#define UART_PLAT_DEVICE(uid)						       \
static struct platform_device kinetis_uart_## uid ##_device = {		       \
	.name			= "kinetis-uart",			       \
	.id			= uid,					       \
	.resource		= kinetis_uart_## uid ##_resources,	       \
	.num_resources		= ARRAY_SIZE(kinetis_uart_## uid ##_resources),\
}

/*
 * Enable clocks for USART & DMA, and register platform device
 */
#define uart_init_clocks_and_register(uid) do {			        \
	kinetis_periph_enable(KINETIS_CG_UART## uid, 1);	        \
	platform_device_register(&kinetis_uart_## uid ##_device);       \
} while (0)

/*
 * Declare the platform devices for the enabled ports
 */
#if defined(CONFIG_KINETIS_UART0)
UART_PLAT_RESOURCES(0);
UART_PLAT_DEVICE(0);
#endif

#if defined(CONFIG_KINETIS_UART1)
UART_PLAT_RESOURCES(1);
UART_PLAT_DEVICE(1);
#endif

#if defined(CONFIG_KINETIS_UART2)
UART_PLAT_RESOURCES(2);
UART_PLAT_DEVICE(2);
#endif

#if defined(CONFIG_KINETIS_UART3)
UART_PLAT_RESOURCES(3);
UART_PLAT_DEVICE(3);
#endif

#if defined(CONFIG_KINETIS_UART4)
UART_PLAT_RESOURCES(4);
UART_PLAT_DEVICE(4);
#endif

#if defined(CONFIG_KINETIS_UART5)
UART_PLAT_RESOURCES(5);
UART_PLAT_DEVICE(5);
#endif

/*
 * Register the Kinetis-specific UART devices with the kernel
 */
void __init kinetis_uart_init(void)
{
	/*
	 * Enable clocks for the enabled ports, and register the platform devs
	 */
#if defined(CONFIG_KINETIS_UART0)
	uart_init_clocks_and_register(0);
#endif
#if defined(CONFIG_KINETIS_UART1)
	uart_init_clocks_and_register(1);
#endif
#if defined(CONFIG_KINETIS_UART2)
	uart_init_clocks_and_register(2);
#endif
#if defined(CONFIG_KINETIS_UART3)
	uart_init_clocks_and_register(3);
#endif
#if defined(CONFIG_KINETIS_UART4)
	uart_init_clocks_and_register(4);
#endif
#if defined(CONFIG_KINETIS_UART5)
	uart_init_clocks_and_register(5);
#endif
}
