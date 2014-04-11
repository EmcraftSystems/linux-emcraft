/*
 * (C) Copyright 2014
 * Emcraft Systems, <www.emcraft.com>
 * Pavel Boldin <paboldin@emcraft.com>
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
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <mach/rtc.h>
#include <mach/lpc18xx.h>
#include <mach/iomux.h>
#include <mach/platform.h>

/*
 * LPC18XX/43XX RTC interrupt
 */
#define LPC43XX_RTC_IRQ		47

/*
 * LPC18XX/43XX RTC base
 */
#define LPC43XX_RTC_BASE	0x40046000

static struct resource rtc_resources[] = {
	{
		.start	= LPC43XX_RTC_BASE,
		.end	= LPC43XX_RTC_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LPC43XX_RTC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static void lpc18xx_rtc_platform_init(void)
{
	LPC18XX_CREG->creg0 &= ~LPC18XX_CREG_CREG0_PD32KHZ;
	LPC18XX_CREG->creg0 &= ~LPC18XX_CREG_CREG0_RESET32KHZ;
	LPC18XX_CREG->creg0 |= LPC18XX_CREG_CREG0_EN1KHZ;

	/* sleep for 2 seconds :-( */
	msleep(2000);
}

static struct platform_device lpc18xx_rtc_device = {
	.name = "rtc-lpc178x",
	.id = -1,
	.dev = {
		.platform_data = lpc18xx_rtc_platform_init
	},
	.resource	= rtc_resources,
	.num_resources	= ARRAY_SIZE(rtc_resources),
};

void __init lpc18xx_rtc_init(void)
{
	platform_device_register(&lpc18xx_rtc_device);
}
