/*
 * (C) Copyright 2017
 * Emcraft Systems, <www.emcraft.com>
 * Reinhard Mötzel <rmoetzel@z-laser.de>
 * Sergei Poselenov <sposelenov@emcraft.com>
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
#include <mach/a2f.h>
#include <mach/clock.h>

#if defined(CONFIG_A2F_WDT)

#define IRQ_A2F_WDINT   0

static struct resource a2f_wdt_resources[] = {
        {
                .start = A2F_WDT_BASE,
                .end = A2F_WDT_BASE + SZ_4K - 1,
                .flags = IORESOURCE_MEM,
        }, {
                .start = IRQ_A2F_WDINT,
                .flags = IORESOURCE_IRQ,
        },
};
struct platform_device a2f_wdt_device = {
        .name           = "a2f-wdt", /* we are using a2f driver */
        .id             = -1,
        .num_resources  = ARRAY_SIZE(a2f_wdt_resources),
        .resource       = a2f_wdt_resources,
};
#endif

void __init a2f_wdt_init(void)
{
#if defined(CONFIG_A2F_WDT)
    int rv;

    /*
     * Register the A2F WDT
     */
    rv = platform_device_register(&a2f_wdt_device);
    if (rv != 0){
        pr_err("%s: Failed to register WDT device\n", __func__);
    }
#endif
}
