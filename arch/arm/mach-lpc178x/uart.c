/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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
#include <mach/lpc178x.h>
#include <mach/clock.h>
#include <mach/uart.h>
#include <mach/power.h>

/*
 * UARTs are compatible with the 16550 device.
 */

#define	LPC178X_UART_RGSZ	(0x60 - 1)

/*
 * "Interrupt ID" in Table 43 in the LPC178x/7x User Manual (page 70).
 */
#define LPC178X_UART0_IRQ	5
#define LPC178X_UART1_IRQ	6
#define LPC178X_UART2_IRQ	7
#define LPC178X_UART3_IRQ	8
#define LPC178X_UART4_IRQ	35

/*
 * UART platform device resources
 */
#define UART_PLAT_RESOURCES(uid)					       \
static struct resource uart## uid ##_resources[] = {			       \
	{								       \
		.start	= LPC178X_UART## uid ##_BASE,			       \
		.end	= LPC178X_UART## uid ##_BASE + LPC178X_UART_RGSZ,      \
		.flags	= IORESOURCE_MEM,				       \
	},								       \
	{								       \
		.start	= LPC178X_UART## uid ##_IRQ,			       \
		.flags	= IORESOURCE_IRQ,				       \
	}								       \
}

const char serial8250_drv_name[] = "serial8250";

/*
 * UART platform device instance
 */
#define UART_PLAT_DEVICE(uid)						       \
static struct plat_serial8250_port uart## uid ##_data[] = {		       \
	{								       \
		.membase	= (char *) LPC178X_UART## uid ##_BASE,	       \
		.mapbase	= LPC178X_UART## uid ##_BASE,		       \
		.irq		= LPC178X_UART## uid ##_IRQ,		       \
		.regshift	= 2,					       \
		.iotype		= UPIO_MEM,				       \
		/* About UPF_MANUAL_INT_CLEAR:				    */ \
		/* The UARTs' controller in the LPC178x/7x SoC requires	    */ \
		/* manual cleanup of pending interrupts, otherwise we	    */ \
		/* will be flooded with interrupts.			    */ \
		.flags		= UPF_SKIP_TEST | UPF_MANUAL_INT_CLEAR,	       \
	},								       \
	{  },								       \
};									       \
									       \
static struct platform_device uart## uid ##_device = {			       \
	.name			= serial8250_drv_name,			       \
	.id			= uid,					       \
	.dev.platform_data	= uart## uid ##_data,			       \
	.num_resources		= 2,					       \
	.resource		= uart## uid ##_resources,		       \
}

/*
 * Enable power on UART, initialize clock rate and register platform device
 */
#define uart_init_and_register(uid) do {				       \
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCUART## uid ##_MSK, 1);       \
	uart## uid ##_data[0].uartclk = lpc178x_clock_get(CLOCK_PCLK);	       \
	(void) platform_device_register(&uart## uid ##_device);		       \
} while (0)


/*
 * Declare the platform devices for the enabled ports
 */
#if defined(CONFIG_LPC178X_UART0)
UART_PLAT_RESOURCES(0);
UART_PLAT_DEVICE(0);
#endif

#if defined(CONFIG_LPC178X_UART1)
UART_PLAT_RESOURCES(1);
UART_PLAT_DEVICE(1);
#endif

#if defined(CONFIG_LPC178X_UART2)
UART_PLAT_RESOURCES(2);
UART_PLAT_DEVICE(2);
#endif

#if defined(CONFIG_LPC178X_UART3)
UART_PLAT_RESOURCES(3);
UART_PLAT_DEVICE(3);
#endif

#if defined(CONFIG_LPC178X_UART4)
UART_PLAT_RESOURCES(4);
UART_PLAT_DEVICE(4);
#endif

/*
 * Register the LPC178x/7x-specific UART devices with the kernel.
 */
void __init lpc178x_uart_init(void)
{
	/*
	 * Enable power on UARTs, initialize clock rate
	 * and register platform devices
	 */
#if defined(CONFIG_LPC178X_UART0)
	uart_init_and_register(0);
#endif

#if defined(CONFIG_LPC178X_UART1)
	uart_init_and_register(1);
#endif

#if defined(CONFIG_LPC178X_UART2)
	uart_init_and_register(2);
#endif

#if defined(CONFIG_LPC178X_UART3)
	uart_init_and_register(3);
#endif

#if defined(CONFIG_LPC178X_UART4)
	uart_init_and_register(4);
#endif
}
