/*
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
 * Anton Protopopov <antonp@emcraft.com>
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
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_ts.h>

#include <mach/lpc18xx.h>
#include <mach/i2s.h>
#include <mach/iomux.h>
#include <mach/platform.h>
#include <mach/dma.h>

static struct i2c_board_info board_lpc4357_i2c0_nau8822 = {
	.type = "nau8822",
	.addr = 0x1a,
};


void __init lpc18xx_i2s_init(void)
{
	int platform;

	platform = lpc18xx_platform_get();

	if (platform == PLATFORM_LPC18XX_BOARD_LPC4357) {
#define LPC18XX_CREG_DMAMUXPER9		(1 << 18)
#define LPC18XX_CREG_DMAMUXPER10	(1 << 20)
		LPC18XX_CREG->dmamux |= \
			LPC18XX_CREG_DMAMUXPER9 | LPC18XX_CREG_DMAMUXPER10;

		lpc18xx_dma_init();

		i2c_register_board_info(0, &board_lpc4357_i2c0_nau8822, 1);
	}
}
