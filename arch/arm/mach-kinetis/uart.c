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
#include <linux/kinetis_uart.h>

#include <mach/kinetis.h>
#include <mach/power.h>
#include <mach/uart.h>
#include <mach/dmainit.h>

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
 * Clock gates for all UARTs
 *
 * These values will be passed into the `kinetis_periph_enable()` function.
 */
static const kinetis_clock_gate_t uart_clock_gate[] = {
	KINETIS_CG_UART0, KINETIS_CG_UART1, KINETIS_CG_UART2,
	KINETIS_CG_UART3, KINETIS_CG_UART4, KINETIS_CG_UART5,
};

/*
 * Register map bases
 */
static const resource_size_t uart_base[] = {
	KINETIS_UART0_BASE, KINETIS_UART1_BASE, KINETIS_UART2_BASE,
	KINETIS_UART3_BASE, KINETIS_UART4_BASE, KINETIS_UART5_BASE,
};

/*
 * Status IRQs
 */
static const resource_size_t uart_stat_irq[] = {
	KINETIS_UART0_STAT_IRQ, KINETIS_UART1_STAT_IRQ, KINETIS_UART2_STAT_IRQ,
	KINETIS_UART3_STAT_IRQ, KINETIS_UART4_STAT_IRQ, KINETIS_UART5_STAT_IRQ,
};

/*
 * Error IRQs
 */
static const resource_size_t uart_err_irq[] = {
	KINETIS_UART0_ERR_IRQ, KINETIS_UART1_ERR_IRQ, KINETIS_UART2_ERR_IRQ,
	KINETIS_UART3_ERR_IRQ, KINETIS_UART4_ERR_IRQ, KINETIS_UART5_ERR_IRQ,
};

#if defined(CONFIG_KINETIS_EDMA)
/*
 * UART Rx DMA channels
 */
static const resource_size_t uart_rx_dma[] = {
	KINETIS_DMACH_UART0_RX, KINETIS_DMACH_UART1_RX, KINETIS_DMACH_UART2_RX,
	KINETIS_DMACH_UART3_RX, KINETIS_DMACH_UART4_RX, KINETIS_DMACH_UART5_RX,
};
#endif /* CONFIG_KINETIS_EDMA */

/*
 * Platform data for Kinetis UART driver to enable CTS/RTS handshaking
 * (hardware flow control).
 */
static struct kinetis_uart_data platform_data_ctsrts = {
	.flags = KINETIS_UART_FLAG_CTSRTS,
};

/*
 * We use this per-UART structure to simplify memory allocation
 */
struct uart_data_structures {
#if defined(CONFIG_KINETIS_EDMA)
	struct resource res[4];
#else
	struct resource res[3];
#endif

	struct platform_device pdev;
};

/*
 * Enable clocks for USART & DMA, and register platform device
 */
static void __init kinetis_uart_register(int uid, int ctsrts)
{
	struct uart_data_structures *uart;

	uart = kzalloc(sizeof(struct uart_data_structures), GFP_KERNEL);
	if (!uart) {
		pr_err("kinetis uart: No enough memory for data structures\n");
		goto out;
	}

	/*
	 * Initialize resources
	 */
	uart->res[0].start = uart_base[uid];
	uart->res[0].end = uart_base[uid] + 1;
	uart->res[0].flags = IORESOURCE_MEM;

	uart->res[1].start = uart_stat_irq[uid];
	uart->res[1].flags = IORESOURCE_IRQ;

	uart->res[2].start = uart_err_irq[uid];
	uart->res[2].flags = IORESOURCE_IRQ;

#if defined(CONFIG_KINETIS_EDMA)
	uart->res[3].start = uart_rx_dma[uid];
	uart->res[3].flags = IORESOURCE_DMA;
#endif

	/*
	 * Initialize platform device
	 */
	uart->pdev.name = "kinetis-uart";
	uart->pdev.id = uid;
	uart->pdev.resource = uart->res;
	uart->pdev.num_resources = ARRAY_SIZE(uart->res);
	if (ctsrts)
		uart->pdev.dev.platform_data = &platform_data_ctsrts;

	/*
	 * Enable UART module clock
	 */
	kinetis_periph_enable(uart_clock_gate[uid], 1);

	if (platform_device_register(&uart->pdev) < 0)
		goto err_periph_disable;

	goto out;

err_periph_disable:
	kinetis_periph_enable(uart_clock_gate[uid], 0);
	kfree(uart);
out:
	;
}

#if defined(CONFIG_KINETIS_UART0_CTSRTS)
#define UART0_CTSRTS	1
#else
#define UART0_CTSRTS	0
#endif

#if defined(CONFIG_KINETIS_UART1_CTSRTS)
#define UART1_CTSRTS	1
#else
#define UART1_CTSRTS	0
#endif

#if defined(CONFIG_KINETIS_UART2_CTSRTS)
#define UART2_CTSRTS	1
#else
#define UART2_CTSRTS	0
#endif

#if defined(CONFIG_KINETIS_UART3_CTSRTS)
#define UART3_CTSRTS	1
#else
#define UART3_CTSRTS	0
#endif

#if defined(CONFIG_KINETIS_UART4_CTSRTS)
#define UART4_CTSRTS	1
#else
#define UART4_CTSRTS	0
#endif

#if defined(CONFIG_KINETIS_UART5_CTSRTS)
#define UART5_CTSRTS	1
#else
#define UART5_CTSRTS	0
#endif

/*
 * Register the Kinetis-specific UART devices with the kernel
 */
void __init kinetis_uart_init(void)
{
#if defined(CONFIG_KINETIS_UART0)
	kinetis_uart_register(0, UART0_CTSRTS);
#endif
#if defined(CONFIG_KINETIS_UART1)
	kinetis_uart_register(1, UART1_CTSRTS);
#endif
#if defined(CONFIG_KINETIS_UART2)
	kinetis_uart_register(2, UART2_CTSRTS);
#endif
#if defined(CONFIG_KINETIS_UART3)
	kinetis_uart_register(3, UART3_CTSRTS);
#endif
#if defined(CONFIG_KINETIS_UART4)
	kinetis_uart_register(4, UART4_CTSRTS);
#endif
#if defined(CONFIG_KINETIS_UART5)
	kinetis_uart_register(5, UART5_CTSRTS);
#endif
}
