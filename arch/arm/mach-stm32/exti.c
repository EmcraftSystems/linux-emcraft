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
#include <linux/module.h>

#include <mach/exti.h>

struct kinetis_exti_regs {
	u32 imr;	/* Interrupt mask register */
	u32 emr;	/* Event mask register */
	u32 rtsr;	/* Rising trigger selection register */
	u32 ftsr;	/* Falling trigger selection register */
	u32 swier;	/* Software interrupt event register */
	u32 pr;		/* Pending register */
};

/*
 * EXTI register map base
 */
#define KINETIS_EXTI_BASE	0x40013c00
#define KINETIS_EXTI		((volatile struct kinetis_exti_regs *) \
				KINETIS_EXTI_BASE)

/*
 * Enable or disable interrupt on the rising edge of a event line
 */
void stm32_exti_enable_int(unsigned int line, int enable)
{
	if (line >= STM32F2_EXTI_NUM_LINES)
		goto out;

	if (enable) {
		stm32_exti_clear_pending(line);

		/* Enable trigger on rising edge */
		KINETIS_EXTI->rtsr |= (1 << line);
		/* Disable trigger on falling edge */
		KINETIS_EXTI->ftsr &= ~(1 << line);
		/* Enable interrupt for the event */
		KINETIS_EXTI->imr |= (1 << line);
	} else {
		/* Disable interrupt for the event */
		KINETIS_EXTI->imr &= ~(1 << line);
		/* Disable trigger on rising edge */
		KINETIS_EXTI->rtsr &= ~(1 << line);
		/* Disable trigger on falling edge */
		KINETIS_EXTI->ftsr &= ~(1 << line);

		stm32_exti_clear_pending(line);
	}

out:
	;
}
EXPORT_SYMBOL(stm32_exti_enable_int);

/*
 * Clear the pending state of a given event
 */
void stm32_exti_clear_pending(unsigned int line)
{
	if (line < STM32F2_EXTI_NUM_LINES)
		KINETIS_EXTI->pr = (1 << line);
}
EXPORT_SYMBOL(stm32_exti_clear_pending);
