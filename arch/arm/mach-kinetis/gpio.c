/*
 * (C) Copyright 2012,2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/iomux.h>

/*
 * GPIO register map
 * Should be mapped at (0x400ff000 + port * 0x40).
 */
struct kinetis_gpio_regs {
	u32 pdor;	/* Port Data Output Register */
	u32 psor;	/* Port Set Output Register */
	u32 pcor;	/* Port Clear Output Register */
	u32 ptor;	/* Port Toggle Output Register */
	u32 pdir;	/* Port Data Input Register */
	u32 pddr;	/* Port Data Direction Register */
};

/*
 * GPIO registers base
 */
#define KINETIS_GPIO_BASE		0x400ff000
#define KINETIS_GPIO_PORT_ADDR(port)	((KINETIS_GPIO_BASE) + (port) * 0x40)
#define KINETIS_GPIO(port) \
	((volatile struct kinetis_gpio_regs *)KINETIS_GPIO_PORT_ADDR(port))

/*
 * IRQC mask (19-16 bits) for PORTx_PCRn
 */
#define IRQC_INTERRUPT_RISING	(0b1001 << 16)
#define IRQC_INTERRUPT_FALLING	(0b1010 << 16)
#define IRQC_INTERRUPT_BOTH	(0b1011 << 16)

/*
 * Get the current state of a GPIO input pin
 */
static int kinetis_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	return (KINETIS_GPIO(KINETIS_GPIO_GETPORT(gpio))->pdir >>
		KINETIS_GPIO_GETPIN(gpio)) & 1;
}

/*
 * Change the direction of a GPIO pin to input
 */
static int kinetis_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	KINETIS_GPIO(KINETIS_GPIO_GETPORT(gpio))->pddr &=
		~(1 << KINETIS_GPIO_GETPIN(gpio));

	return 0;
}

/*
 * Set the state of a GPIO output pin
 */
static void kinetis_gpio_set_value(
	struct gpio_chip *chip, unsigned gpio, int value)
{
	if (value) {
		KINETIS_GPIO(KINETIS_GPIO_GETPORT(gpio))->psor =
			(1 << KINETIS_GPIO_GETPIN(gpio));
	} else {
		KINETIS_GPIO(KINETIS_GPIO_GETPORT(gpio))->pcor =
			(1 << KINETIS_GPIO_GETPIN(gpio));
	}
}

/*
 * Change the direction of a GPIO pin to output and
 * set the level on this pin.
 */
static int kinetis_gpio_direction_output(
	struct gpio_chip *chip, unsigned gpio, int level)
{
	KINETIS_GPIO(KINETIS_GPIO_GETPORT(gpio))->pddr |=
		(1 << KINETIS_GPIO_GETPIN(gpio));

	gpio_set_value(gpio, level);

	return 0;
}

#if defined (CONFIG_KINETIS_GPIO_INT)

/*
 * In K70 ports have assigned IRQs listed here.
 * See Table 3-4 in the K70 Sub-Family Reference Manual.
 */
static int kinetis_port_irqs[] = {
	87, /* PORT_A */
	88, /* PORT_B */
	89, /* PORT_C */
	90, /* PORT_D */
	91, /* PORT_E */
	92, /* PORT_F */
};

/*
 * Custom chip structure that contains data needed for interrupts handling
 */
struct kinetis_gpio_chip {
	spinlock_t irq_lock;
	unsigned irq_base;
	struct gpio_chip gc;		/* standard chip descriptor */
	u8 inttype[KINETIS_GPIO_IRQS];	/* type of interrupt for each pin */
};

static int kinetis_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	int port;
	struct kinetis_gpio_chip *chip;

	chip = container_of(gc, struct kinetis_gpio_chip, gc);
	if (chip->irq_base == (unsigned) -1)
		return -EINVAL;

	port = offset/KINETIS_GPIO_PORT_PINS;
	if (port < KINETIS_GPIO_PORT_A || port > KINETIS_GPIO_PORT_F)
		return -EINVAL;

	return chip->irq_base + offset;
}

static void kinetis_gpio_irq_handler(unsigned irq, struct irq_desc *desc)
{
	int offset, port, pin;
	unsigned long isfr;
	struct kinetis_gpio_chip *chip = get_irq_chip_data(irq);

	for (port = 0; port < KINETIS_GPIO_PORTS; port++) {
		if (irq != kinetis_port_irqs[port])
			continue;

		desc->chip->ack(irq);

		isfr = KINETIS_PORT(port)->isfr;
		for_each_bit(offset, &isfr, KINETIS_GPIO_PORT_PINS) {
			pin = KINETIS_GPIO_MKPIN(port, offset);
			generic_handle_irq(chip->irq_base + pin);
		}

		KINETIS_PORT(port)->isfr = ~0U; /* ISFR is W1C */

		desc->chip->unmask(irq);

		break;
	}
}

static void kinetis_irq_enable(unsigned irq)
{
	unsigned long flags;
	struct kinetis_gpio_chip *chip = get_irq_chip_data(irq);
	int offset = irq - chip->irq_base;
	struct kinetis_gpio_dsc pin_dsc = {
		.port = KINETIS_GPIO_GETPORT(offset),
		.pin = KINETIS_GPIO_GETPIN(offset),
	};
	u32 mask = 0;

	spin_lock_irqsave(&chip->irq_lock, flags);

	kinetis_gpio_config_unmask(&pin_dsc, IRQC_INTERRUPT_BOTH);

	if (chip->inttype[offset] & IRQ_TYPE_EDGE_RISING)
		mask |= IRQC_INTERRUPT_RISING;

	if (chip->inttype[offset] & IRQ_TYPE_EDGE_FALLING)
		mask |= IRQC_INTERRUPT_FALLING;

	if (mask)
		kinetis_gpio_config_mask(&pin_dsc, mask);

	spin_unlock_irqrestore(&chip->irq_lock, flags);
}

static void kinetis_irq_disable(unsigned irq)
{
	unsigned long flags;
	struct kinetis_gpio_chip *chip = get_irq_chip_data(irq);
	int offset = irq - chip->irq_base;
	struct kinetis_gpio_dsc pin_dsc = {
		.port = KINETIS_GPIO_GETPORT(offset),
		.pin = KINETIS_GPIO_GETPIN(offset),
	};

	spin_lock_irqsave(&chip->irq_lock, flags);
	kinetis_gpio_config_unmask(&pin_dsc, IRQC_INTERRUPT_BOTH);
	spin_unlock_irqrestore(&chip->irq_lock, flags);
}

static int kinetis_irq_set_type(unsigned irq, unsigned trigger)
{
	unsigned long flags;
	struct kinetis_gpio_chip *chip = get_irq_chip_data(irq);
	int offset = irq - chip->irq_base;

	/* Level interrupts are not supported */
	if (trigger & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW))
		return -EINVAL;

	spin_lock_irqsave(&chip->irq_lock, flags);
	chip->inttype[offset] = trigger & IRQ_TYPE_SENSE_MASK;
	spin_unlock_irqrestore(&chip->irq_lock, flags);

	return 0;
}

/*
 * Structure to represent chip for each particular pin interrupt
 */
static struct irq_chip kinetis_irq_chip = {
		.name		= "GPIO",
		.enable		= kinetis_irq_enable,
		.disable	= kinetis_irq_disable,
		.set_type	= kinetis_irq_set_type,
};

static struct kinetis_gpio_chip kinetis_chip = {
	.irq_base = NVIC_IRQS,
	.gc = {
		.label			= "kinetis",
		.direction_input	= kinetis_gpio_direction_input,
		.get			= kinetis_gpio_get_value,
		.direction_output	= kinetis_gpio_direction_output,
		.set			= kinetis_gpio_set_value,
		.to_irq                 = kinetis_gpio_to_irq,
		.base			= KINETIS_GPIO_OFF_MCU,
		.ngpio			= KINETIS_GPIO_LEN_MCU,
		.can_sleep		= 1,
	}
};

void __init kinetis_gpio_init(void)
{
	int i;
	struct kinetis_gpio_chip *chip = &kinetis_chip;

	/* Assigning handler for all ports PORT_A - PORT_F */
	for (i = 0; i < KINETIS_GPIO_PORTS; i++) {
		set_irq_chained_handler(kinetis_port_irqs[i],
					kinetis_gpio_irq_handler);
		set_irq_chip_data(kinetis_port_irqs[i], chip);
	}

	/* Allowing all interrupts for pins */
	for (i = 0; i < KINETIS_GPIO_IRQS; i++) {
		chip->inttype[i] = 0;
		set_irq_chip(i+chip->irq_base, &kinetis_irq_chip);
		set_irq_handler(i+chip->irq_base, handle_simple_irq);
		set_irq_flags(i+chip->irq_base, IRQF_VALID);
		set_irq_chip_data(i+chip->irq_base, chip);
	}

	if (gpiochip_add(&chip->gc) < 0)
		pr_err("%s: gpiochip_add failed.\n", __func__);
}

#else

static struct gpio_chip kinetis_chip = {
	.label			= "kinetis",
	.direction_input	= kinetis_gpio_direction_input,
	.get			= kinetis_gpio_get_value,
	.direction_output	= kinetis_gpio_direction_output,
	.set			= kinetis_gpio_set_value,
	.base			= KINETIS_GPIO_OFF_MCU,
	.ngpio			= KINETIS_GPIO_LEN_MCU,
	.can_sleep		= 1,
};

void __init kinetis_gpio_init(void)
{
	if (gpiochip_add(&kinetis_chip) < 0)
		pr_err("%s: gpiochip_add failed.\n", __func__);
}

#endif
