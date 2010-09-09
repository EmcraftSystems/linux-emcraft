/*
 * linux/arch/arm/mach-a2f/eth.c
 *
 * Copyright (C) 2010 Dmitry Cherkassov, Emcraft Systems
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

#include <mach/a2f.h>
#include <mach/eth.h>

MODULE_LICENSE("GPL");

#define ETH_CORE_BASE 0x40003000
#define ETH_CORE_SIZE 0xFFF

//IRQ label is EMAC_IRQ, Cortex M3 NVIC Inuput is INTISR[5]
#define ETH_CORE_IRQ 5

static struct resource eth_core_resources[] = {
        {
        .start          = ETH_CORE_BASE,
        .end            = ETH_CORE_BASE + ETH_CORE_SIZE,
        .flags          = IORESOURCE_MEM,
//	.irq            = ETH_CORE_IRQ
        }
};

/*
static struct plat_serial8250_port mss_uart0_data[] = {
        {
       .membase     	= (char *) MSS_CORE_BASE,
       .mapbase     	= MSS_CORE_BASE,
       .irq         	= MSS_CORE_IRQ,
       .uartclk     	= 25000000,
       .regshift    	= 2,
       .iotype      	= UPIO_MEM,
       .flags		= UPF_SKIP_TEST,
        },
        {  },
};
*/


static struct platform_device eth_device = {
        .name           = "core10100",
        .id             = -1,
	//.dev.platform_data = mss_uart0_data,
        .num_resources  = 1,
        .resource       = eth_core_resources,
};

void __init a2f_eth_init()
{
	printk(KERN_INFO "in a2f_eth_init");
	platform_device_register(&eth_device);		
}
