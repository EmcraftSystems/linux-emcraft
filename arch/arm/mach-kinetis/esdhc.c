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

#define KINETIS_ESDHC_BASE	(KINETIS_AIPS1PERIPH_BASE + 0x00031000)
#define KINETIS_ESDHC_IRQ	80


/*
 * We use this per-UART structure to simplify memory allocation
 */
struct esdhc_data_structures {
	struct resource res[2];

	struct platform_device pdev;
};

/*
 * Enable clocks for USART & DMA, and register platform device
 */
static void __init kinetis_esdhc_register(void)
{
	struct esdhc_data_structures *esdhc;

	esdhc = kzalloc(sizeof(struct esdhc_data_structures), GFP_KERNEL);
	if (!esdhc) {
		pr_err("kinetis esdhc: No enough memory for data structures\n");
		goto out;
	}

	/*
	 * Initialize resources
	 */
	esdhc->res[0].start = KINETIS_ESDHC_BASE;
	esdhc->res[0].end = KINETIS_ESDHC_BASE + 0x100;
	esdhc->res[0].flags = IORESOURCE_MEM;

	esdhc->res[1].start = KINETIS_ESDHC_IRQ;
	esdhc->res[1].flags = IORESOURCE_IRQ;

	/*
	 * Initialize platform device
	 */
	esdhc->pdev.name = "esdhc";
	esdhc->pdev.id = 0;
	esdhc->pdev.resource = esdhc->res;
	esdhc->pdev.num_resources = ARRAY_SIZE(esdhc->res);

	/*
	 * Enable ESDHC module clock
	 */
	kinetis_periph_enable(KINETIS_CG_ESDHC, 1);

	if (platform_device_register(&esdhc->pdev) < 0)
		goto err_periph_disable;

	goto out;

err_periph_disable:
	kinetis_periph_enable(KINETIS_CG_ESDHC, 0);
	kfree(esdhc);
out:
	;
}

/*
 * Register the Kinetis-specific ESDHC devices with the kernel
 */
void __init kinetis_esdhc_init(void)
{
	kinetis_esdhc_register();
}
