/*
 * (C) Copyright 2017
 * Emcraft Systems, <www.emcraft.com>
 * Dmitry Konyshev <probables@emcraft.com>
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
#include <linux/flexcan_platform.h>

#include <mach/kinetis.h>
#include <mach/power.h>
#include <mach/clock.h>

#define KINETIS_FLEXCAN0_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00024000)
#define KINETIS_FLEXCAN0_IRQ	29
#define KINETIS_FLEXCAN1_BASE	(KINETIS_AIPS1PERIPH_BASE + 0x00024000)
#define KINETIS_FLEXCAN1_IRQ	37

static struct flexcan_platform_data flexcan_data;

/*
 * We use this per-FLEXCAN structure to simplify memory allocation
 */
struct flexcan_data_structures {
	struct resource res[7];

	struct platform_device pdev;
};

/*
 * Enable clocks for flexcan and register platform device
 */
static void __init kinetis_flexcan_register(u32 iomem, int irq, u32 gate, int id)
{
	struct flexcan_data_structures *flexcan;
	int i;

	flexcan = kzalloc(sizeof(struct flexcan_data_structures), GFP_KERNEL);
	if (!flexcan) {
		pr_err("kinetis flexcan: No enough memory for data structures\n");
		return;
	}

	/*
	 * Initialize resources
	 */
	flexcan->res[0].start = iomem;
	flexcan->res[0].end = iomem + 0x3fff;
	flexcan->res[0].flags = IORESOURCE_MEM;

	for (i = 1; i < 7; i++) {
		flexcan->res[i].start = irq + i - 1;
		flexcan->res[i].flags = IORESOURCE_IRQ;
	}

	/*
	 * Initialize platform device
	 */
	flexcan->pdev.name = "kinetis-flexcan";
	flexcan->pdev.id = id;
	flexcan->pdev.resource = flexcan->res;
	flexcan->pdev.num_resources = ARRAY_SIZE(flexcan->res);
	flexcan->pdev.dev.platform_data = &flexcan_data;

	/*
	 * Enable FLEXCAN module clock
	 */
	kinetis_periph_enable(gate, 1);

	if (platform_device_register(&flexcan->pdev) < 0)
		goto err_periph_disable;

	return;

err_periph_disable:
	kinetis_periph_enable(gate, 0);
	kfree(flexcan);
}

/*
 * Register the Kinetis-specific FLEXCAN devices with the kernel
 */
void __init kinetis_flexcan_init(void)
{
	flexcan_data.clock_frequency = kinetis_clock_get(CLOCK_PCLK);

	kinetis_flexcan_register(KINETIS_FLEXCAN0_BASE,
				 KINETIS_FLEXCAN0_IRQ, KINETIS_CG_FLEXCAN0, 0);
	kinetis_flexcan_register(KINETIS_FLEXCAN1_BASE,
				 KINETIS_FLEXCAN1_IRQ, KINETIS_CG_FLEXCAN1, 1);
}
