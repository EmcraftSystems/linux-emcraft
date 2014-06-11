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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <asm/io.h>

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/iomux.h>

/*
 * GPIO interrupts map
 */
struct lpc18xx_gpio_pint_regs {
	u32 pintsel[2];
};

/*
 * GPIO interruption access handlers
 */
#define LPC18XX_GPIO_PINT_BASE	0x40086E00
#define LPC18XX_GPIO_PINT	((volatile struct lpc18xx_gpio_pint_regs *) \
					LPC18XX_GPIO_PINT_BASE)

/*
 * GPIO interruption settings
 */
struct lpc18xx_gpio_int_regs {
	u32 pmode;
	u32 enrl;
	u32 setenrl;
	u32 cenrl;
	u32 enaf;
	u32 setenaf;
	u32 cenaf;
	u32 rdet;
	u32 fdet;
	u32 pstat;
};

/*
 * GPIO interruption settings handlers
 */
#define LPC18XX_GPIO_INT_BASE	0x40087000
#define LPC18XX_GPIO_INT	((volatile struct lpc18xx_gpio_int_regs *) \
					LPC18XX_GPIO_INT_BASE)


/*
 * GPIO ports registers map
 */
struct lpc18xx_gpio_regs {
	u8 pbyte[256];		/* GPIO port byte pin registers */
	u32 rsv0[960];
	u32 pword[256];		/* GPIO port word pin registers */
	u32 rsv1[768];
	u32 dir[8];		/* GPIO port direction registers */
	u32 rsv2[24];
	u32 mask[8];		/* GPIO port mask registers */
	u32 rsv3[24];
	u32 pin[8];		/* GPIO port pin registers */
	u32 rsv4[24];
	u32 mpin[8];		/* GPIO masked port pin registers */
	u32 rsv5[24];
	u32 set[8];		/* GPIO port set registers */
	u32 rsv6[24];
	u32 clr[8];		/* GPIO port clear registers */
	u32 rsv7[24];
	u32 not[8];		/* GPIO port toggle registers */
};

/*
 * GPIO registers access handlers
 */
#define LPC18XX_GPIO_BASE	0x400F4000
#define LPC18XX_GPIO		((volatile struct lpc18xx_gpio_regs *) \
					LPC18XX_GPIO_BASE)

static int lpc18xx_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	LPC18XX_GPIO->dir[LPC18XX_GPIO_GETPORT(gpio)] &= ~(1 << LPC18XX_GPIO_GETPIN(gpio));
	return 0;
}

static int lpc18xx_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int level)
{
	LPC18XX_GPIO->dir[LPC18XX_GPIO_GETPORT(gpio)] |= (1 << LPC18XX_GPIO_GETPIN(gpio));

	gpio_set_value(gpio, level);

	return 0;
}

/*
 * Define the value of a general-purpose output
 */
static void lpc18xx_gpio_set_value(struct gpio_chip* chip, unsigned gpio, int value)
{
	if (value) {
		LPC18XX_GPIO->set[LPC18XX_GPIO_GETPORT(gpio)] |= (1 << LPC18XX_GPIO_GETPIN(gpio));
	}
	else {
		LPC18XX_GPIO->clr[LPC18XX_GPIO_GETPORT(gpio)] |= (1 << LPC18XX_GPIO_GETPIN(gpio));
	}
}

static int lpc18xx_gpio_get_value(struct gpio_chip* chip, unsigned gpio)
{
	return (LPC18XX_GPIO->pin[LPC18XX_GPIO_GETPORT(gpio)] >> LPC18XX_GPIO_GETPIN(gpio)) & 1;
}

#define LPC18XX_GPIO_PIN_BASE_INT	32
#define LPC18XX_GPIO_PINT_COUNT		8

static DEFINE_SPINLOCK(gpio_interrupt_lock);
static unsigned long lpc18xx_gpio_irq_mask = 0;

static unsigned int _lpc18xx_irq_alloc(int port, int pin)
{
	int rv = 0, int_num, val;

	if (port < 0 || port > 7 || pin < 0 || pin > 31) {
		/* Arguments invalid */
		rv = -EINVAL;
		goto out;
	}

	spin_lock(&gpio_interrupt_lock);

	/* Look for interrupt */
	int_num = find_first_zero_bit(&lpc18xx_gpio_irq_mask, LPC18XX_GPIO_PINT_COUNT);
	if (int_num == LPC18XX_GPIO_PINT_COUNT) {
		rv = -ENOSPC;
		goto out_unlock;
	}
	set_bit(int_num, &lpc18xx_gpio_irq_mask);

	val = pin | (port << 5);

	LPC18XX_GPIO_PINT->pintsel[int_num / 4] |=  val << (8 * (int_num % 4));

	rv = int_num + LPC18XX_GPIO_PIN_BASE_INT;

out_unlock:
	spin_unlock(&gpio_interrupt_lock);
out:
	return rv;
}

void lpc18xx_irq_free(int irq)
{
	spin_lock(&gpio_interrupt_lock);
	clear_bit(irq - LPC18XX_GPIO_PIN_BASE_INT, &lpc18xx_gpio_irq_mask);
	spin_unlock(&gpio_interrupt_lock);
}
EXPORT_SYMBOL(lpc18xx_gpio_irq_free);

static void _lpc18xx_gpio_int_ack(unsigned int irq)
{
	LPC18XX_GPIO_INT->pstat |= 1 << (irq - LPC18XX_GPIO_PIN_BASE_INT);
}

static struct irq_chip *nvic_irq_chip = NULL;
static struct irq_chip gpio_irq_chip;

void lpc18xx_gpio_ack(unsigned int irq)
{
	_lpc18xx_gpio_int_ack(irq);

	nvic_irq_chip->ack(irq);
}

int lpc18xx_gpio_set_type(unsigned int irq, unsigned int flow_type)
{
	unsigned int int_mask = 1 << (irq - LPC18XX_GPIO_PIN_BASE_INT);

	if (irq >= LPC18XX_GPIO_PIN_BASE_INT + LPC18XX_GPIO_PINT_COUNT ||
		irq < LPC18XX_GPIO_PINT_COUNT)
		return -EINVAL;

	if (flow_type & IRQ_TYPE_EDGE_BOTH == IRQ_TYPE_EDGE_BOTH)
		return -EINVAL;

	if (flow_type & IRQ_TYPE_EDGE_BOTH) {
		LPC18XX_GPIO_INT->pmode &= ~int_mask;

		/* ENRL is for rising edge interrupt */
		if (flow_type & IRQ_TYPE_EDGE_RISING)
			LPC18XX_GPIO_INT->setenrl = int_mask;
		else
			LPC18XX_GPIO_INT->cenrl = int_mask;

		/* ENAF is for falling edge interrupt */
		if (flow_type & IRQ_TYPE_EDGE_FALLING)
			LPC18XX_GPIO_INT->setenaf = int_mask;
		else
			LPC18XX_GPIO_INT->cenaf = int_mask;
	}
	else {
		LPC18XX_GPIO_INT->pmode |= int_mask;

		/* Set required level interrupt */
		if (flow_type & IRQ_TYPE_LEVEL_HIGH)
			LPC18XX_GPIO_INT->setenaf = int_mask;
		else
			LPC18XX_GPIO_INT->cenaf = int_mask;

		/* Enable level interrupt */
		LPC18XX_GPIO_INT->setenrl = int_mask;
	}

	return 0;
}

/*
 * Request GPIO interrupt at given GPIO port and pin
 */
unsigned int lpc18xx_gpio_irq_request(int port, int pin)
{
	int irq;
	struct irq_desc *desc;

	irq = _lpc18xx_irq_alloc(port, pin);
	if (irq < 0)
		return irq;

	desc = irq_to_desc(irq);

	if (nvic_irq_chip == NULL) {
		nvic_irq_chip = desc->chip;
		gpio_irq_chip = *nvic_irq_chip;
		gpio_irq_chip.ack = lpc18xx_gpio_ack;
		gpio_irq_chip.set_type = lpc18xx_gpio_set_type;
	}
	else
		BUG_ON(nvic_irq_chip != desc->chip && &gpio_irq_chip != desc->chip);

	set_irq_chip(irq, &gpio_irq_chip);

	return irq;
}
EXPORT_SYMBOL(lpc18xx_gpio_irq_request);

static struct gpio_chip lpc18xx_chip = {
	.label			= "lpc18xx",
	.direction_input	= lpc18xx_gpio_direction_input,
	.get			= lpc18xx_gpio_get_value,
	.direction_output	= lpc18xx_gpio_direction_output,
	.set			= lpc18xx_gpio_set_value,
	.base			= LPC18XX_GPIO_OFF_MCU,
	.ngpio			= LPC18XX_GPIO_LEN_MCU,
	.can_sleep		= 1,
};

void __init lpc18xx_gpio_init(void)
{
	if (gpiochip_add(&lpc18xx_chip) < 0)
		pr_err("%s: gpiochip_add failed.\n", __func__);
	else
		pr_info("%s GPIO chip added\n", lpc18xx_chip.label);
}
