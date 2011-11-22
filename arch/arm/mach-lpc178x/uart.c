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

/*
 * UART0
 */
#ifdef CONFIG_LPC178X_UART0

#define	LPC178X_UART_RGSZ	(0x60 - 1)

/* "Interrupt ID" in Table 43 in the LPC178x/7x User Manual (page 70). */
#define LPC178X_UART0_IRQ	5

#define LPC178X_UART0_BASE	0x4000C000

static struct resource uart0_resources[] = {
	{
		.start          = LPC178X_UART0_BASE,
		.end            = LPC178X_UART0_BASE + LPC178X_UART_RGSZ,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = LPC178X_UART0_IRQ,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct plat_serial8250_port uart0_data[] = {
	{
		.membase	= (char *) LPC178X_UART0_BASE,
		.mapbase	= LPC178X_UART0_BASE,
		.irq		= LPC178X_UART0_IRQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
	{  },
};

static struct platform_device uart0_device = {
	.name           = "serial8250",
	.id             = 0,
	.dev.platform_data = uart0_data,
	.num_resources  = 2,
	.resource       = uart0_resources,
};
#endif /* CONFIG_LPC178X_UART0 */

/*
 * Register the LPC178x/7x-specific UART devices with the kernel.
 */
void __init lpc178x_uart_init(void)
{
#if defined(CONFIG_LPC178X_UART0)
	/*
	 * Enable power on UART0
	 */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCUART0_MSK, 1);

	/*
	 * Get the reference clock for this UART port
	 */
	uart0_data[0].uartclk = lpc178x_clock_get(CLOCK_PCLK);

	/*
	 * Register device for UART0
	 */
	(void) platform_device_register(&uart0_device);
#endif
}
