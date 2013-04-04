/*
 * (C) Copyright 2011, 2012
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
#include <linux/errno.h>
#include <linux/module.h>

#include <mach/kinetis.h>
#include <mach/power.h>

/*
 * Enable or disable the clock on a peripheral device (timers, UARTs, USB, etc)
 */
int kinetis_periph_enable(kinetis_clock_gate_t gate, int enable)
{
	volatile u32 *scgc;
	u32 mask;
	int rv;

	/*
	 * Verify the function arguments
	 */
	if (KINETIS_CG_REG(gate) >= KINETIS_SIM_CG_NUMREGS ||
	    KINETIS_CG_IDX(gate) >= KINETIS_SIM_CG_NUMBITS) {
		rv = -EINVAL;
		goto out;
	}

	scgc = &KINETIS_SIM->scgc[KINETIS_CG_REG(gate)];
	mask = 1 << KINETIS_CG_IDX(gate);

	if (gate == KINETIS_CG_PORTF && enable) { /* K70 Errata #5234 */
		mask |= (1 << KINETIS_CG_IDX(KINETIS_CG_PORTE));
	}

	if (enable)
		*scgc |= mask;
	else
		*scgc &= ~mask;

	rv = 0;
out:
	return rv;
}
EXPORT_SYMBOL(kinetis_periph_enable);
