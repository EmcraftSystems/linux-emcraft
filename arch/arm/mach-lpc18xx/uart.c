/*
 * (C) Copyright 2011, 2012
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
#include <linux/clk.h>

#include <mach/lpc18xx.h>
#include <mach/clock.h>
#include <mach/uart.h>

/*
 * UARTs are compatible with the 16550 device.
 */

#define LPC18XX_NR_UARTS	4

#define	LPC18XX_UART_RGSZ	(0x5C - 1)

/*
 * IRQ numbers for UARTs
 */
#define LPC18XX_UART0_IRQ	24
#define LPC18XX_UART1_IRQ	25
#define LPC18XX_UART2_IRQ	26
#define LPC18XX_UART3_IRQ	27

int lpc18xx_uart_irq[] = {
	LPC18XX_UART0_IRQ, LPC18XX_UART1_IRQ,
	LPC18XX_UART2_IRQ, LPC18XX_UART3_IRQ,
};

/*
 * Base addresses of UART registers
 */
u32 lpc18xx_uart_base[] = {
	LPC18XX_UART0_BASE, LPC18XX_UART1_BASE,
	LPC18XX_UART2_BASE, LPC18XX_UART3_BASE,
};

/*
 * All data structures required be an UART port, to simplify memory allocation
 */
struct lpc18xx_uart {
	/*
	 * UART platform device resources
	 */
	struct resource res[2];
	/*
	 * UART platform device instance
	 *
	 * The second element in the array must be zeroed to let the driver know
	 * where this array ends.
	 */
	struct plat_serial8250_port plat_data[2];
	/*
	 * UART platform device
	 */
	struct platform_device plat_dev;
};

const char serial8250_drv_name[] = "serial8250";

static void uart_init_and_register(int id)
{
	struct lpc18xx_uart *uart;
	char clk_name[20];
	struct clk *clk;

	if (id < 0 || id >= LPC18XX_NR_UARTS)
		pr_err("%s: UART ID is out of range\n", __func__);

	uart = kzalloc(sizeof(struct lpc18xx_uart), GFP_KERNEL);

	/*
	 * Initialize resources
	 */
	/* Register map */
	uart->res[0].start = lpc18xx_uart_base[id];
	uart->res[0].end = lpc18xx_uart_base[id] + LPC18XX_UART_RGSZ;
	uart->res[0].flags = IORESOURCE_MEM;
	/* Interrupt */
	uart->res[1].start = lpc18xx_uart_irq[id];
	uart->res[1].flags = IORESOURCE_IRQ;

	/*
	 * Initialize platform data
	 */
	uart->plat_data[0].membase = (char *)lpc18xx_uart_base[id];
	uart->plat_data[0].mapbase = lpc18xx_uart_base[id];
	uart->plat_data[0].irq = lpc18xx_uart_irq[id];
	/* 32-bit registers */
	uart->plat_data[0].regshift = 2;
	uart->plat_data[0].iotype = UPIO_MEM;
	/*
	 * The UART/USART controller on LPC18xx/43xx requires manual cleanup
	 * of pending interrupts (UPF_MANUAL_INT_CLEAR), otherwise we will be
	 * flooded with interrupts.
	 */
	uart->plat_data[0].flags = UPF_SKIP_TEST | UPF_MANUAL_INT_CLEAR;

	/*
	 * Initialize platform device
	 */
	uart->plat_dev.name = serial8250_drv_name;
	uart->plat_dev.id = id;
	uart->plat_dev.dev.platform_data = uart->plat_data;
	uart->plat_dev.num_resources = 2;
	uart->plat_dev.resource = uart->res;

	/*
	 * Get UART clock
	 */
	sprintf(clk_name, "lpc18xx-uart.%d", id);
	clk = clk_get(NULL, clk_name);
	if (IS_ERR(clk)) {
		pr_err("%s: Error getting clock\n", __func__);
		goto err_clk;
	}
	clk_enable(clk);

	/*
	 * Initialize clock rate and register platform device
	 */
	uart->plat_data[0].uartclk = clk_get_rate(clk);
	if (platform_device_register(&uart->plat_dev) < 0) {
		pr_err("%s: Could not register platform device\n", __func__);
		goto err_register;
	}

	goto exit;

err_register:
	clk_disable(clk);
	clk_put(clk);

err_clk:
	kfree(uart);

exit:
	;
}

/*
 * Register the LPC18xx-specific UART/USART devices with the kernel
 */
void __init lpc18xx_uart_init(void)
{
	/*
	 * Initialize the clock driver to make the UART clocks
	 * available in `uart_init_and_register()`.
	 */
	lpc18xx_clock_init();

	/*
	 * Register UART platform devices
	 */
#if defined(CONFIG_LPC18XX_UART0)
	uart_init_and_register(0);
#endif

#if defined(CONFIG_LPC18XX_UART1)
	uart_init_and_register(1);
#endif

#if defined(CONFIG_LPC18XX_UART2)
	uart_init_and_register(2);
#endif

#if defined(CONFIG_LPC18XX_UART3)
	uart_init_and_register(3);
#endif
}
