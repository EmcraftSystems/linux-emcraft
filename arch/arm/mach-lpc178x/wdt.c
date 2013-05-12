/*
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
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
#include <mach/lpc178x.h>
#include <mach/clock.h>
#include <mach/power.h>

#if defined(CONFIG_LPC178X_WATCHDOG)

#define LPC17XX_WDT_BASE	0x40000000
#define IRQ_LPC178X_WDINT	0

static struct resource lpc178x_wdt_resources[] = {
        {
                .start = LPC17XX_WDT_BASE,
                .end = LPC17XX_WDT_BASE + SZ_4K - 1,
                .flags = IORESOURCE_MEM,
        }, {
                .start = IRQ_LPC178X_WDINT,
                .flags = IORESOURCE_IRQ,
        },
};
struct platform_device lpc178x_wdt_device = {
        .name           = "lpc2k-wdt", /* we are using lpc2k driver */
        .id             = -1,
        .num_resources  = ARRAY_SIZE(lpc178x_wdt_resources),
        .resource       = lpc178x_wdt_resources,
};
#endif

void __init lpc178x_wdt_init(void)
{
#if defined(CONFIG_LPC178X_WATCHDOG)
	int rv;

	/*
	 * Register the LPC178x WDT
	 */
	rv = platform_device_register(&lpc178x_wdt_device);
	if (rv != 0){
		pr_err("%s: Failed to register WDT device\n", __func__);
	}
#endif
}
