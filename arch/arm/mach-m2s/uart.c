/*
 * linux/arch/arm/mach-m2s/uart.c
 *
 * Copyright (C) 2010,2011 Vladimir Khusainov, Emcraft Systems
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
#include <linux/console.h>
#include <mach/m2s.h>
#include <mach/clock.h>
#include <mach/uart.h>
#include <mach/platform.h>

/*
 * The MSS subsystem of SmartFusion contains two UART ports that
 * provide the s/w compatibility with the 16550 device.
 */
#define	MSS_UART_RGSZ		(0x20 - 1)

#define UART0_RST_CLR		(1 << 7)
#define UART1_RST_CLR		(1 << 8)

/*
 * MSS UART_0
 */
#if defined(CONFIG_M2S_MSS_UART0)

#define MSS_UART0_IRQ		10

static struct resource mss_uart0_resources[] = {
	{
		.start		= MSS_UART0_BASE,
		.end            = MSS_UART0_BASE + MSS_UART_RGSZ,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = MSS_UART0_IRQ,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct plat_serial8250_port mss_uart0_data[] = {
	{
		.membase     	= (char *) MSS_UART0_BASE,
		.mapbase     	= MSS_UART0_BASE,
		.irq         	= MSS_UART0_IRQ,
		.regshift    	= 2,
		.iotype      	= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
	{  },
};

static struct platform_device mss_uart0_device = {
	.name           = "serial8250",
	.id             = 0,
	.dev.platform_data = mss_uart0_data,
	.num_resources  = 2,
	.resource       = mss_uart0_resources,
};
#endif	/* CONFIG_M2S_MSS_UART0 */

/*
 * MSS UART_1
 */
#if defined(CONFIG_M2S_MSS_UART1)

#define MSS_UART1_IRQ	11

static struct resource mss_uart1_resources[] = {
	{
		.start          = MSS_UART1_BASE,
		.end            = MSS_UART1_BASE + MSS_UART_RGSZ,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = MSS_UART1_IRQ,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct plat_serial8250_port mss_uart1_data[] = {
	{
		.membase        = (char *) MSS_UART1_BASE,
		.mapbase        = MSS_UART1_BASE,
		.irq            = MSS_UART1_IRQ,
		.regshift       = 2,
		.iotype         = UPIO_MEM,
		.flags          = UPF_SKIP_TEST,
	},
	{  },
};

static struct platform_device mss_uart1_device = {
	.name           = "serial8250",
	.id             = 1,
	.dev.platform_data = mss_uart1_data,
	.num_resources  = 2,
	.resource       = mss_uart1_resources,
};
#endif	/* CONFIG_M2S_MSS_UART1 */

/*
 * Register the M2S specific UART devices with the kernel.
 */
void __init m2s_uart_init(void)
{
#if defined(CONFIG_M2S_MSS_UART0)

	/*
	 * Clear the reset bit.
	 */
	writel(readl(&M2S_SYSREG->soft_reset_cr) | UART0_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);
	writel(readl(&M2S_SYSREG->soft_reset_cr) & ~UART0_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);

	/*
	 * Get the reference clock for this UART port
	 */
	mss_uart0_data[0].uartclk = m2s_clock_get(CLCK_PCLK0);

	/*
	 * Register device for UART_0.
	 */
	(void) platform_device_register(&mss_uart0_device);
#else
	/*
	 * Put UART0 into reset if it is not required.
	 */
	writel(readl(&M2S_SYSREG->soft_reset_cr) | UART0_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);
#endif
#if defined(CONFIG_M2S_MSS_UART1)

	/*
	 * Clear the reset bit.
	 */
	writel(readl(&M2S_SYSREG->soft_reset_cr) | UART1_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);
	writel(readl(&M2S_SYSREG->soft_reset_cr) & ~UART1_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);

	/*
	 * Get the reference clock for this UART port
	 */
	mss_uart1_data[0].uartclk = m2s_clock_get(CLCK_PCLK1);

	/*
	 * Register device for UART_1.
	 */
	(void) platform_device_register(&mss_uart1_device);
#else
	/*
	 * Put UART1 into reset if it is not required.
	 */
	writel(readl(&M2S_SYSREG->soft_reset_cr) | UART1_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);
#endif
}

static int __init m2s_add_console(void) {
	int	p = m2s_platform_get();

	if (p == PLATFORM_SF2_DEV_KIT) {
		/*
		 * Set UART1 as a preferred console. If the only UART1 port
		 * is enabled, then it corresponds to the ttyS0 console name,
		 * otherwise, if UART0 is enabled as well, then UART1
		 * corresponds to ttyS1.
		 *
		 * This is default setting and can be overridden by user via the
		 * "console=" boot argument.
		 */
#if defined(CONFIG_M2S_MSS_UART0) && defined(CONFIG_M2S_MSS_UART1)
		add_preferred_console("ttyS", 1, "115200");
#elif defined(CONFIG_M2S_MSS_UART1)
		add_preferred_console("ttyS", 0, "115200");
#endif
	}
	return 0;
}
console_initcall(m2s_add_console);
