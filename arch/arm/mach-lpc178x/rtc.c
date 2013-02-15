/*
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * (C) Copyright 2013
 * Intmash, <www.intmash.ru>
 * Gilmanov Ildar <gil_ildar@mail.ru>
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

static struct platform_device rtc_device = {
	.name = "rtc-lpc178x",
	.id   = -1,
};

void __init lpc178x_rtc_init(void)
{
	int rv;

	/*
	 * Register the LPC178x Real-Time Clock device
	 */
	rv = platform_device_register(&rtc_device);
	if (rv != 0){
		pr_err("%s: Failed to register RTC device\n", __func__);
	}
}
