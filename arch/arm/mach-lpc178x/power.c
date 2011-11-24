/*
 * (C) Copyright 2011
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

#include <mach/lpc178x.h>
#include <mach/power.h>

/*
 * Enable or disable power on a peripheral device (timers, UARTs, USB, etc)
 */
void lpc178x_periph_enable(u32 pconp_mask, int enable)
{
	if (enable)
		LPC178X_SCC->pconp |= pconp_mask;
	else
		LPC178X_SCC->pconp &= ~pconp_mask;
}
