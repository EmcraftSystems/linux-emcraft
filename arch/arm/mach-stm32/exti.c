/*
 * (C) Copyright 2012-2015
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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
#include <linux/module.h>
#include <linux/io.h>
#include <mach/exti.h>

#if 0
#define DEBUG
#endif

/*
 * EXTI control and status registers
 */
struct stm32_exti_regs {
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
#define STM32_EXTI_BASE		0x40013c00
#define STM32_EXTI		((volatile struct stm32_exti_regs *) \
				STM32_EXTI_BASE)

/*
 * EXTI-related SYSCFG control and status registers
 */
struct stm32_exti_syscfg_regs {
	u32 rsvd[2];	/* Reserved */
	u32 exticr[4];	/* External interrupt config registers */
};

/*
 * EXTI register map base
 */
#define STM32_EXTI_SYSCFG_BASE	0x40013800
#define STM32_EXTI_SYSCFG	((volatile struct stm32_exti_syscfg_regs *) \
				STM32_EXTI_SYSCFG_BASE)

/*
 * connect an exti line to a specified STM32 pin
 * @line	EXTI line
 * @pin		pin within the range for this line (0..15)
 */
void stm32_exti_connect(unsigned int line, int pin)
{
	uint v;
	int offset = line / 4;
	int shft = (line % 4) * 4;

	v = readl(&STM32_EXTI_SYSCFG->exticr[offset]);
	v &= ~(0xF << shft);
	v |= pin << shft;
	writel(v, &STM32_EXTI_SYSCFG->exticr[offset]);

#if defined(DEBUG)
	printk("%s:%d,%d=%d,%d,%x\n", __func__, line, pin, offset, shft,
		readl(&STM32_EXTI_SYSCFG->exticr[offset]));
#endif
}
EXPORT_SYMBOL(stm32_exti_connect);

/*
 * enable or disable interrupt
 * @line	EXTI line
 * @enable	enable(1) / disable(0)
 */
void stm32_exti_enable_int(unsigned int line, int enable)
{
	if (enable) {
		stm32_exti_clear_pending(line);

		/* Enable interrupt for the event */
		writel(readl(&STM32_EXTI->imr) | (1 << line),
			&STM32_EXTI->imr);
	} else {
		/* Disable interrupt for the event */
		writel(readl(&STM32_EXTI->imr) & ~(1 << line),
			&STM32_EXTI->imr);
		/* Disable trigger on rising edge */
		writel(readl(&STM32_EXTI->rtsr) & ~(1 << line),
			&STM32_EXTI->rtsr);
		/* Disable trigger on falling edge */
		writel(readl(&STM32_EXTI->ftsr) & ~(1 << line),
			&STM32_EXTI->ftsr);
		/* Clear pending events if any */
		stm32_exti_clear_pending(line);
	}

#if defined(DEBUG)
	printk("%s:%d,%d=%x\n",
		__func__, line, enable, readl(&STM32_EXTI->imr));
#endif
}
EXPORT_SYMBOL(stm32_exti_enable_int);

/*
 * set rising trigger
 * @line	EXTI line
 */
void stm32_exti_set_rising(unsigned int line)
{
	/* Enable trigger on rising edge */
	writel(readl(&STM32_EXTI->rtsr) | (1 << line), &STM32_EXTI->rtsr);

#if defined(DEBUG)
	printk("%s:%d=%x\n", __func__, line, readl(&STM32_EXTI->rtsr));
#endif
}
EXPORT_SYMBOL(stm32_exti_set_rising);

/*
 * set falling trigger
 * @line	EXTI line
 */
void stm32_exti_set_falling(unsigned int line)
{
	/* Enable trigger on falling edge */
	writel(readl(&STM32_EXTI->ftsr) | (1 << line), &STM32_EXTI->ftsr);

#if defined(DEBUG)
	printk("%s:%d=%x\n", __func__, line, readl(&STM32_EXTI->ftsr));
#endif
}
EXPORT_SYMBOL(stm32_exti_set_falling);

/*
 * get the bit mask of pending EXTI interrupts
 * @ret		current value of the PR register
 */
unsigned long stm32_exti_get_pending(void)
{
	unsigned long ret = readl(&STM32_EXTI->pr);

#if defined(DEBUG)
	printk("%s:%lx\n", __func__, ret);
#endif
	return ret;
}
EXPORT_SYMBOL(stm32_exti_get_pending);

/*
 * clear the pending state of an EXTI event
 * @line	EXTI line
 */
void stm32_exti_clear_pending(unsigned int line)
{
	writel(1 << line, &STM32_EXTI->pr);

#if defined(DEBUG)
	printk("%s:%d=%x\n", __func__, line, readl(&STM32_EXTI->pr));
#endif
}
EXPORT_SYMBOL(stm32_exti_clear_pending);
