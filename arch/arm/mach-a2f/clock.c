/*
 *  linux/arch/arm/mach-a2f/clock.c
 *
 *  Copyright (C) 2010 Vladimir Khusainov, Emcraft Systems
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
#include <linux/serial_8250.h>

#include <mach/a2f.h>
#include <mach/clock.h>
#include <mach/a2fxxxm3.h>

/*
 * Initialize the clock section of the A2F.
 */

void __init a2f_clock_init(void)
{
	SystemCoreClockUpdate();
}

/*
 * Return a clock value for the specified clock.
 */

unsigned int a2f_clock_get(enum a2f_clock clck)
{
	unsigned int val = 0;

	switch (clck)  {
	case CLCK_SYSTEMCORE:
		val = SystemCoreClock;
		break;
	case CLCK_PCLK0:
		val = g_FrequencyPCLK0;
		break;
	case CLCK_PCLK1:
		val = g_FrequencyPCLK1;
		break;
	case CLCK_ACE:
		val = g_FrequencyACE;
		break;
	case CLCK_FPGA:
		val = g_FrequencyFPGA;
		break;
	}

	return val;
}

/*
 * End of File
 */


