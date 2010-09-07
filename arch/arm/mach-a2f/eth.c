/*
 * linux/arch/arm/mach-a2f/eth.c
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

#define MSS_CORE_BASE 0
#define MSS_CORE_SIZE 0
#define MSS_CORE_IRQ A

static struct resource mss_core_resources[] = {
        {
        .start          = MSS_CORE_BASE,
        .end            = MSS_CORE_BASE + MSS_CORE_SIZE,
        .flags          = IORESOURCE_MEM,
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

static int eth_probe(struct platform_device *pd)
{
	return 0;
}

static int eth_remove(struct platform_device *pd)
{
	return 0;
}

static void eth_shutdown(struct platform_device *pd)
{
	
}




static struct platform_device eth_device = {
        .name           = "eth",
        .id             = 0,
	//.dev.platform_data = mss_uart0_data,
        .num_resources  = 1,
        .resource       = mss_core_resources,
};


static int eth_init(void) {
	printk(KERN_INFO "eth entry\n");

	//platform_driver_register(&eth_platform_driver);
	
	platform_device_register(&eth_device);
	
	return 0;
}

static void eth_exit(void) {
	printk(KERN_INFO "eth unload\n");
	//platform_driver_unregister(&eth_platform_driver);
}

module_init(eth_init);
module_exit(eth_exit);
 
