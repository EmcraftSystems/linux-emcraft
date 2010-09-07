/*
 * linux/drivers/net/core10100.c
 *
 * Copyright (C) Dmitry Cherkassov, Emcraft Systems
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
 *
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/module.h>
#include <linux/kernel.h>

MODULE_LICENSE("GPL");

static struct resource mss_uart0_resources[] = {
        {
        .start          = MSS_UART0_BASE,
        .end            = MSS_UART0_BASE + MSS_UART0_SIZE,
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
       .uartclk     	= 25000000,
       .regshift    	= 2,
       .iotype      	= UPIO_MEM,
       .flags		= UPF_SKIP_TEST,
        },
        {  },
};

static struct platform_device core10100_device = {
        .name           = "core10100",
        .id             = 0,
	.dev.platform_data = mss_uart0_data,
        .num_resources  = 2,
        .resource       = mss_uart0_resources,
};

static int core10100_init(void) {
     printk("<1> Hello world!\n");
  return 0;
}

static void core10100_exit(void) {
     printk("<1> Bye, cruel world\n");
}

module_init(core10100_init);
module_exit(core10100_exit);
