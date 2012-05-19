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
#include <linux/types.h>

#include <mach/wdt.h>

/*
 * WWDT (Windowed Watchdog timer) register map
 */
struct lpc18xx_wwdt_regs {
	u32 mod;		/* Mode register */
	u32 tc;			/* Timer constant register */
	u32 feed;		/* Feed register */
	u32 tv;			/* Timer value register */
	u32 rsv0;
	u32 warnint;		/* Timer warning interrupt register */
	u32 window;		/* Timer window register */
};

/*
 * WWDT registers base
 */
#define LPC18XX_WWDT_BASE		0x40080000
#define LPC18XX_WWDT			((volatile struct lpc18xx_wwdt_regs *) \
					LPC18XX_WWDT_BASE)

/*
 * Mode register
 */
/* Watchdog enable */
#define LPC18XX_WWDT_MOD_EN_MSK		(1 << 0)
/* Watchdog reset enable */
#define LPC18XX_WWDT_MOD_RESET_MSK	(1 << 1)
/*
 * Feed sequence
 */
#define LPC18XX_WWDT_FEED1		0xAA
#define LPC18XX_WWDT_FEED2		0x55

/*
 * Trigger watchdog reset on LPC18xx/43xx
 */
void lpc18xx_wdt_reset(void)
{
	/*
	 * Enable watchdog timer
	 */
	LPC18XX_WWDT->mod = LPC18XX_WWDT_MOD_EN_MSK;
	LPC18XX_WWDT->feed = LPC18XX_WWDT_FEED1;
	LPC18XX_WWDT->feed = LPC18XX_WWDT_FEED2;

	/*
	 * Enable watchdog reset
	 */
	LPC18XX_WWDT->mod |= LPC18XX_WWDT_MOD_RESET_MSK;

	/*
	 * Satisfy one of the watchdog reset conditions:
	 *    Write wrong feed sequence to the Feed register.
	 */
	LPC18XX_WWDT->feed = LPC18XX_WWDT_FEED1;
	LPC18XX_WWDT->feed = ~LPC18XX_WWDT_FEED2;
}
