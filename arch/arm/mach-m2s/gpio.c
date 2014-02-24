/*
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
 * Vladimir Skvortsov <vskvortsov@emcraft.com>
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
#include <linux/spinlock.h>
#include <linux/irq.h>

#include <asm/io.h>

#include <mach/gpio.h>

/*
 * GPIO registers map
 */
#define MSS_GPIO_CFG		0x40013000
#define MSS_GPIO_IRQ		0x40013080
#define MSS_GPIO_IN		0x40013084
#define MSS_GPIO_OUT		0x40013088

#define MSS_GPIO_IRQS		32

#define FPGA_GPIO_CFG		CONFIG_M2S_FPGA_GPIO_ADDR
#define FPGA_GPIO_IRQ		(FPGA_GPIO_CFG + 0x80)
#define FPGA_GPIO_IN		(FPGA_GPIO_CFG + 0x90)
#define FPGA_GPIO_OUT		(FPGA_GPIO_CFG + 0xa0)

#define FPGA_GPIO_BASE	32 /* Base number of the FPGA GPIOs in Linux */

#define M2S_GPIO_CFG(base, port) ((base) + ((port) << 2))

/*
 * GPIO configuration register bits
 */
#define GPIO_OUT_REG_EN		(1 << 0)
#define GPIO_IN_REG_EN		(1 << 1)
#define GPIO_OUT_BUF_EN		(1 << 2)
#define GPIO_INT_EN		(1 << 3)

struct m2s_gpio_chip {
	struct gpio_chip chip;
	u32 gpio_cfg;
	u32 gpio_irq;
	u32 gpio_in;
	u32 gpio_out;
	spinlock_t lock;

#if defined (CONFIG_M2S_MSS_GPIO_INT)
	spinlock_t irq_lock;
	unsigned irq_base;
#endif
};

/*
 * Get the current state of a GPIO input pin
 */
static int m2s_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	struct m2s_gpio_chip *m2s_chip
		= container_of(chip, struct m2s_gpio_chip, chip);
	return (readl(m2s_chip->gpio_in) >> gpio) & 1;
}

/*
 * Change the direction of a GPIO pin to input
 */
static int m2s_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	struct m2s_gpio_chip *m2s_chip
		= container_of(chip, struct m2s_gpio_chip, chip);
	u32 gpio_cfg;
	unsigned long f;

	spin_lock_irqsave(&m2s_chip->lock, f);

	gpio_cfg = readl(M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));
	gpio_cfg |= GPIO_IN_REG_EN;
	gpio_cfg &= ~(GPIO_OUT_REG_EN | GPIO_OUT_BUF_EN);
	writel(gpio_cfg, M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));

	spin_unlock_irqrestore(&m2s_chip->lock, f);

	return 0;
}

/*
 * Set the state of a GPIO output pin
 */
static void m2s_gpio_set_value_locked(struct m2s_gpio_chip *m2s_chip,
				      unsigned gpio, int value)
{
	u32 gpio_out = readl(m2s_chip->gpio_out);
	if (value) {
		gpio_out |=  1 << gpio;
	} else {
		gpio_out &=  ~(1 << gpio);
	}
	writel(gpio_out, m2s_chip->gpio_out);
}

static void m2s_gpio_set_value(struct gpio_chip *chip,
			       unsigned gpio, int value)
{
	struct m2s_gpio_chip *m2s_chip
		= container_of(chip, struct m2s_gpio_chip, chip);
	unsigned long f;

	spin_lock_irqsave(&m2s_chip->lock, f);

	m2s_gpio_set_value_locked(m2s_chip, gpio, value);

	spin_unlock_irqrestore(&m2s_chip->lock, f);
}

/*
 * Change the direction of a GPIO pin to output and
 * set the level on this pin.
 */
static int m2s_gpio_direction_output(struct gpio_chip *chip,
				     unsigned gpio, int level)
{
	struct m2s_gpio_chip *m2s_chip
		= container_of(chip, struct m2s_gpio_chip, chip);
	u32 gpio_cfg;
	unsigned long f;

	spin_lock_irqsave(&m2s_chip->lock, f);

	gpio_cfg = readl(M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));
	gpio_cfg |= GPIO_OUT_REG_EN | GPIO_OUT_BUF_EN;
	gpio_cfg &= ~(GPIO_IN_REG_EN);
	writel(0, M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));
	m2s_gpio_set_value_locked(m2s_chip, gpio, level);
	writel(gpio_cfg, M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));

	spin_unlock_irqrestore(&m2s_chip->lock, f);

	return 0;
}

#if defined (CONFIG_M2S_MSS_GPIO)

# if defined (CONFIG_M2S_MSS_GPIO_INT)
static int m2s_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct m2s_gpio_chip *chip;

	chip = container_of(gc, struct m2s_gpio_chip, chip);
	if (chip->irq_base == (unsigned) -1)
		return -EINVAL;

	if (offset >= 32)
		return -EINVAL;

	return chip->irq_base + offset;
}

static void m2s_irq_enable(unsigned irq)
{
	struct m2s_gpio_chip *m2s_chip = get_irq_chip_data(irq);
	struct irq_desc *desc = irq_to_desc(irq);
	int gpio = irq - m2s_chip->irq_base;
	unsigned long flags, gpio_cfg;

	spin_lock_irqsave(&m2s_chip->irq_lock, flags);

	gpio_cfg = readl(M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));
	gpio_cfg |= GPIO_INT_EN;
	writel(gpio_cfg, M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));

	desc->chip->unmask(irq);
	desc->status &= ~IRQ_MASKED;

	spin_unlock_irqrestore(&m2s_chip->irq_lock, flags);
}

static void m2s_irq_ack(unsigned irq)
{
	struct m2s_gpio_chip *m2s_chip = get_irq_chip_data(irq);
	int gpio = irq - m2s_chip->irq_base;
	unsigned long flags;

	spin_lock_irqsave(&m2s_chip->irq_lock, flags);
	writel(1 << gpio, m2s_chip->gpio_irq);
	spin_unlock_irqrestore(&m2s_chip->irq_lock, flags);
}


static void m2s_irq_disable(unsigned irq)
{
	struct m2s_gpio_chip *m2s_chip = get_irq_chip_data(irq);
	struct irq_desc *desc = irq_to_desc(irq);
	int gpio = irq - m2s_chip->irq_base;
	unsigned long flags, gpio_cfg;

	spin_lock_irqsave(&m2s_chip->irq_lock, flags);

	gpio_cfg = readl(M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));
	gpio_cfg &= ~GPIO_INT_EN;
	writel(gpio_cfg, M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));

	desc->chip->mask(irq);
	desc->status |= IRQ_MASKED;

	spin_unlock_irqrestore(&m2s_chip->irq_lock, flags);
}

#define IRQC_INTERRUPT_HIGH	(0b000 << 5)
#define IRQC_INTERRUPT_LOW	(0b001 << 5)
#define IRQC_INTERRUPT_RISING	(0b010 << 5)
#define IRQC_INTERRUPT_FALLING	(0b011 << 5)
#define IRQC_INTERRUPT_BOTH	(0b100 << 5)
#define IRQC_INTERRUPT_MASK	(0b111 << 5)

static int m2s_irq_set_type(unsigned irq, unsigned trigger)
{
	unsigned long flags;
	struct m2s_gpio_chip *m2s_chip = get_irq_chip_data(irq);
	int gpio = irq - m2s_chip->irq_base;
	unsigned long intflag, gpio_cfg;

	if (gpio >= 32)
		return -EINVAL;

	switch (trigger & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_NONE:
		intflag = 0;
		break;
	case IRQ_TYPE_EDGE_RISING:
		intflag = IRQC_INTERRUPT_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		intflag = IRQC_INTERRUPT_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		intflag = IRQC_INTERRUPT_BOTH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		intflag = IRQC_INTERRUPT_LOW;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		intflag = IRQC_INTERRUPT_HIGH;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&m2s_chip->irq_lock, flags);

	gpio_cfg = readl(M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));
	gpio_cfg &= ~IRQC_INTERRUPT_MASK;
	gpio_cfg |= intflag | GPIO_IN_REG_EN;

	writel(gpio_cfg, M2S_GPIO_CFG(m2s_chip->gpio_cfg, gpio));

	spin_unlock_irqrestore(&m2s_chip->irq_lock, flags);

	return 0;
}

/*
 * Structure to represent chip for each particular pint interrupt
 */
static struct irq_chip m2s_irq_chip = {
		.name		= "GPIO",
		.enable		= m2s_irq_enable,
		.disable	= m2s_irq_disable,
		.set_type	= m2s_irq_set_type,
		.ack		= m2s_irq_ack,
};
# endif /* CONFIG_M2S_MSS_GPIO_INT */

static struct m2s_gpio_chip mss_gpio_chip = {
# if defined (CONFIG_M2S_MSS_GPIO_INT)
	.irq_base = 50,
# endif /* CONFIG_M2S_MSS_GPIO_INT */
	.chip = {
		.label			= "mss_gpio",
		.direction_input	= m2s_gpio_direction_input,
		.get			= m2s_gpio_get_value,
		.direction_output	= m2s_gpio_direction_output,
		.set			= m2s_gpio_set_value,
# if defined (CONFIG_M2S_MSS_GPIO_INT)
		.to_irq			= m2s_gpio_to_irq,
# endif /* CONFIG_M2S_MSS_GPIO_INT */
		.base			= 0,
		.ngpio			= 32,
		.can_sleep		= 0,
	},
	.gpio_cfg = MSS_GPIO_CFG,
	.gpio_irq = MSS_GPIO_IRQ,
	.gpio_in = MSS_GPIO_IN,
	.gpio_out = MSS_GPIO_OUT,
	.lock = SPIN_LOCK_UNLOCKED,
};
#endif

#if defined (CONFIG_M2S_FPGA_GPIO)
static struct m2s_gpio_chip fpga_gpio_chip = {
	.chip = {
		.label			= "fpga_gpio",
		.direction_input	= m2s_gpio_direction_input,
		.get			= m2s_gpio_get_value,
		.direction_output	= m2s_gpio_direction_output,
		.set			= m2s_gpio_set_value,
		.base			= FPGA_GPIO_BASE,
		.ngpio			= 32,
		.can_sleep		= 0,
	},
	.gpio_cfg = FPGA_GPIO_CFG,
	.gpio_irq = FPGA_GPIO_IRQ,
	.gpio_in = FPGA_GPIO_IN,
	.gpio_out = FPGA_GPIO_OUT,
	.lock = SPIN_LOCK_UNLOCKED,
};
#endif

void __init m2s_gpio_init(void)
{
#if defined (CONFIG_M2S_MSS_GPIO)
# if defined (CONFIG_M2S_MSS_GPIO_INT)
	/* Allowing all interrupts for pins */
	int irq_base, i;

	irq_base = mss_gpio_chip.irq_base;

	m2s_irq_chip.mask = get_irq_chip(irq_base)->mask;
	m2s_irq_chip.unmask = get_irq_chip(irq_base)->unmask;

	for (i = 0; i < MSS_GPIO_IRQS; i++) {
		set_irq_chip(i + irq_base, &m2s_irq_chip);
		set_irq_handler(i + irq_base, handle_edge_irq);
		set_irq_flags(i + irq_base, IRQF_VALID);
		set_irq_chip_data(i + irq_base, &mss_gpio_chip);
	}
# endif /* CONFIG_M2S_MSS_GPIO_INT */

	if (gpiochip_add(&mss_gpio_chip.chip) < 0) {
		pr_err("%s: gpiochip_add failed.\n", __func__);
	}
#endif

#if defined (CONFIG_M2S_FPGA_GPIO)
	if (gpiochip_add(&fpga_gpio_chip.chip) < 0) {
		pr_err("%s: gpiochip_add failed.\n", __func__);
	}
#endif
}
