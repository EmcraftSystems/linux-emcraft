/*
 * (C) Copyright 2014
 * Emcraft Systems, <www.emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
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
#include <mach/rtc.h>

/*
 * Platform specific RTC parameters
 */
#define KINETIS_RTC_BASE	0x4003D000
#define KINETIS_RTC_IRQ		66

/*
 * RTC platform device resources
 */
static struct resource kinetis_rtc_resources[] = {
	{
		.start	= KINETIS_RTC_BASE,
		.end	= KINETIS_RTC_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= KINETIS_RTC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/*
 * RTC platform device data structure
 */
static struct platform_device rtc_device = {
	.name = "rtc-kinetis",
	.id   = -1,
	.num_resources = ARRAY_SIZE(kinetis_rtc_resources),
	.resource = kinetis_rtc_resources,
};

/*
 * Instantiate a platform device for the Kinetis on-chip RTC
 * @returns		0->success, <0->error code
 */
void __init kinetis_rtc_init(void)
{

	/*
	 * Register a platform device for the on-chip RTC
	 */
	if (platform_device_register(&rtc_device)) {
		pr_err("%s: Failed to register RTC device\n", __func__);
	}
}
