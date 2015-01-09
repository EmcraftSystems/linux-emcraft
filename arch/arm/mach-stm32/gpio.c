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
#include <linux/init.h>
#include <mach/iomux.h>
#include <mach/gpio.h>
#include <mach/exti.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/irq.h>

#if 0
#define DEBUG
#endif

#if defined(CONFIG_STM32_GPIO)

/*
 * Register map bases
 */
static const unsigned long stm32_gpio_base[] = {
	STM32F2_GPIOA_BASE, STM32F2_GPIOB_BASE, STM32F2_GPIOC_BASE,
	STM32F2_GPIOD_BASE, STM32F2_GPIOE_BASE, STM32F2_GPIOF_BASE,
	STM32F2_GPIOG_BASE, STM32F2_GPIOH_BASE, STM32F2_GPIOI_BASE
};

/*
 * Convert a single GPIO port number to a {port, pin} pair 
 */
#define STM32_GPIO_GETPIN(number)	(number % STM32_GPIO_PORT_PINS)
#define STM32_GPIO_GETPORT(number)	(number / STM32_GPIO_PORT_PINS)

/*
 * Number of EXTI IRQ lines -> these are used to implement GPIO interrupts
 */
#define	STM32_EXTI_IRQS_LEN		7

/*
 * Number of EXTI lines -> there are more of those than EXTI IRQ lines
 */
#define	STM32_EXTI_LINES_LEN		15

/*
 * STM32-specific GPIO chip data structure definition
 */
struct stm32_gpio_chip {
	struct gpio_chip chip;			/* GPIO chip */
	spinlock_t lock;			/* GPIO chip lock */
#if defined (CONFIG_STM32_GPIO_INT)
	struct irq_chip irq_chip;		/* GPIO IRQ chip */
	spinlock_t irq_lock;			/* GPIO interrupt lock */
	unsigned irq_base;			/* GPIO IRQ base number */
	int exti_irqs[STM32_EXTI_IRQS_LEN];	/* NVIC EXTI IRQ numbers */
	int exti_2_gpio[STM32_EXTI_LINES_LEN];	/* GPI0 assigned to EXTI */
#endif
};

/*
 * Set the state of a GPIO output pin
 * @chip	GPIO chip
 * @gpio	GPIO
 * @v		New state of the GPIO signal
 */
static void stm32_gpio_set_value(
	struct gpio_chip *chip, unsigned gpio, int v)
{
	struct stm32_gpio_chip *stm32_chip
		= container_of(chip, struct stm32_gpio_chip, chip);
	int port = STM32_GPIO_GETPORT(gpio);
	int pin = STM32_GPIO_GETPIN(gpio);
	volatile struct stm32f2_gpio_regs *gpio_regs =
		(struct stm32f2_gpio_regs *) stm32_gpio_base[port];
	unsigned long f, l;

	spin_lock_irqsave(&stm32_chip->lock, f);

	l = readl(&gpio_regs->odr);
	l &=  ~(1 << pin);
	l |= (v << pin);
	writel(l, &gpio_regs->odr);

	spin_unlock_irqrestore(&stm32_chip->lock, f);

#if defined(DEBUG)
	printk("%s:%d[%d,%d]=%d\n", __func__, gpio, port, pin, v);
#endif
}

/*
 * Get the current state of a GPIO input pin
 * @chip	GPIO chip
 * @gpio	GPIO
 * @ret		Current value of the GPIO signal
 */
static int stm32_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	struct stm32_gpio_chip *stm32_chip
		= container_of(chip, struct stm32_gpio_chip, chip);
	int port = STM32_GPIO_GETPORT(gpio);
	int pin = STM32_GPIO_GETPIN(gpio);
	volatile struct stm32f2_gpio_regs *gpio_regs =
		(struct stm32f2_gpio_regs *) stm32_gpio_base[port];
	unsigned long f;
	int ret;

	spin_lock_irqsave(&stm32_chip->lock, f);

	ret = (readl(&gpio_regs->idr) & (1 << pin)) ? 1 : 0;

	spin_unlock_irqrestore(&stm32_chip->lock, f);

#if defined(DEBUG)
	printk("%s:%d[%d,%d]=%d\n", __func__, gpio, port, pin, ret);
#endif
	return ret;
}

/*
 * Change the direction of a GPIO pin to input
 * @chip	GPIO chip
 * @gpio	GPIO
 * @ret		0 -> success; error code otherwise
 */
static int stm32_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	struct stm32_gpio_chip *stm32_chip
		= container_of(chip, struct stm32_gpio_chip, chip);
	int port = STM32_GPIO_GETPORT(gpio);
	int pin = STM32_GPIO_GETPIN(gpio);
	int shf = pin * 2;
	volatile struct stm32f2_gpio_regs *gpio_regs =
		(struct stm32f2_gpio_regs *) stm32_gpio_base[port];
	unsigned long f;

	spin_lock_irqsave(&stm32_chip->lock, f);

	writel(readl(&gpio_regs->moder) & ~(3 << shf), &gpio_regs->moder);

	spin_unlock_irqrestore(&stm32_chip->lock, f);

#if defined(DEBUG)
	printk("%s:%d[%d,%d]\n", __func__, gpio, port, pin);
#endif
	return 0;
}

/*
 * Change the direction of a GPIO pin to output and
 * set the level on this pin
 * @chip	GPIO chip
 * @gpio	GPIO
 * @level	Level of the GPIO signal
 * @ret		0 -> success; error code otherwise
 */
static int stm32_gpio_direction_output(
	struct gpio_chip *chip, unsigned gpio, int level)
{
	struct stm32_gpio_chip *stm32_chip
		= container_of(chip, struct stm32_gpio_chip, chip);
	int port = STM32_GPIO_GETPORT(gpio);
	int pin = STM32_GPIO_GETPIN(gpio);
	int shf = pin * 2;
	volatile struct stm32f2_gpio_regs *gpio_regs =
		(struct stm32f2_gpio_regs *) stm32_gpio_base[port];
	unsigned long f, l;

	spin_lock_irqsave(&stm32_chip->lock, f);

	l = readl(&gpio_regs->moder);
	l &= ~(3 << shf);
	l |= (1 << shf);
	writel(l, &gpio_regs->moder);

	gpio_set_value(gpio, level);

	spin_unlock_irqrestore(&stm32_chip->lock, f);

#if defined(DEBUG)
	printk("%s:%d[%d,%d]=%d\n", __func__, gpio, port, pin, level);
#endif
	return 0;
}

#if defined(CONFIG_STM32_GPIO_INT)

/*
 * EXTI line interrupt handler
 * @irq		EXTI IRQ number
 * @irq_desc	IRQ data structure
 */
static void stm32_exti_irq_handler(unsigned irq, struct irq_desc *desc)
{
	int line, pin;
	unsigned long pending;
	struct stm32_gpio_chip *stm32_chip = get_irq_chip_data(irq);

	desc->chip->ack(irq);

	/*
	 * Call handler for all pending EXTI interrupts
	 */
	pending = stm32_exti_get_pending();
	for_each_bit(line, &pending, STM32_EXTI_LINES_LEN) {
		pin = stm32_chip->exti_2_gpio[line];
		generic_handle_irq(stm32_chip->irq_base + pin);
		stm32_exti_clear_pending(line);
#if defined(DEBUG)
		printk("%s:%d=%d,%d\n", __func__, irq, line, pin);
#endif
	}

	desc->chip->unmask(irq);
}

/*
 * Convert GPIO number to logical GPIO IRQ number
 * @chip	GPIO chip
 * @offset	GPIO number
 * @ret		logical GPIO IRQ number
 */
static int stm32_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct stm32_gpio_chip *stm32_chip
		= container_of(chip, struct stm32_gpio_chip, chip);
	int ret = -EINVAL;

	if (stm32_chip->irq_base == (unsigned) -1 ||
		offset >= STM32_GPIO_LEN) {

		goto Done;
	}

	ret = stm32_chip->irq_base + offset;
Done:
#if defined(DEBUG)
	printk("%s:%d=%d\n", __func__, offset, ret);
#endif
	return ret;
}

/*
 * Enable interrupts for the specified GPIO IRQ
 * @irq		Logical GPIO IRQ number
 */
static void stm32_gpio_irq_enable(unsigned irq)
{
	struct stm32_gpio_chip *stm32_chip = get_irq_chip_data(irq);
	struct irq_desc *desc = irq_to_desc(irq);
	int gpio = irq - stm32_chip->irq_base;
	int line = STM32_GPIO_GETPIN(gpio);
	int pin = STM32_GPIO_GETPORT(gpio);
	unsigned long flags;

	spin_lock_irqsave(&stm32_chip->irq_lock, flags);

	/*
	 * On the STM32, multiple GPIO signals share a single EXTI line
	 */
	if (stm32_chip->exti_2_gpio[line] != -1 &&
		stm32_chip->exti_2_gpio[line] != gpio) {
		pr_err("STM32 GPIO: had to disable irq for GPIO %d "
			"to enable irq for GPIO %d\n",
			stm32_chip->exti_2_gpio[line], gpio);
	}

	/*
	 * Set up the new connection between EXTI line and STM32 pin
	 */
	stm32_chip->exti_2_gpio[line] = gpio;
	stm32_exti_connect(line, pin);

	/*
	 * Enable EXTI interrupt
	 */
	stm32_exti_enable_int(line, 1);

	/*
	 * Allow interrupts on this GPIO IRQ
	 */
	desc->status &= ~IRQ_MASKED;

	spin_unlock_irqrestore(&stm32_chip->irq_lock, flags);

#if defined(DEBUG)
	printk("%s:%d[%d,%d]=%d\n", __func__, gpio, line, pin, irq);
#endif
}

/*
 * Disable interrupts for the specified GPIO IRQ
 * @irq		Logical GPIO IRQ number
 */
static void stm32_gpio_irq_disable(unsigned irq)
{
	struct stm32_gpio_chip *stm32_chip = get_irq_chip_data(irq);
	struct irq_desc *desc = irq_to_desc(irq);
	int gpio = irq - stm32_chip->irq_base;
	int line = STM32_GPIO_GETPIN(gpio);
	int pin = STM32_GPIO_GETPORT(gpio);
	unsigned long flags;

	spin_lock_irqsave(&stm32_chip->irq_lock, flags);

	stm32_exti_enable_int(line, 0);
	desc->status |= IRQ_MASKED;

	spin_unlock_irqrestore(&stm32_chip->irq_lock, flags);

#if defined(DEBUG)
	printk("%s:%d[%d,%d]=%d\n", __func__, gpio, line, pin, irq);
#endif
}

/*
 * Set trigger type for the specified logical GPIO IRG
 * @irq		Logical GPIO IRQ number
 * @trigger	Trigger type
 * @ret		0 -> success; error code otherwise
 */
static int stm32_gpio_irq_set_type(unsigned irq, unsigned trigger)
{
	unsigned long flags;
	struct stm32_gpio_chip *stm32_chip = get_irq_chip_data(irq);
	int gpio = irq - stm32_chip->irq_base;
	int line = STM32_GPIO_GETPIN(gpio);
	int pin = STM32_GPIO_GETPORT(gpio);
	int ret = -EINVAL;

	if ((gpio >= STM32_GPIO_LEN) ||

		/*
		 * Only edge interrupts are supported
		 */
		(trigger & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW))) {

		goto Done;
	}

	spin_lock_irqsave(&stm32_chip->irq_lock, flags);

	if (trigger & IRQ_TYPE_EDGE_RISING) {
		stm32_exti_set_rising(line);
	}
	if (trigger & IRQ_TYPE_EDGE_FALLING) {
		stm32_exti_set_falling(line);
	}

	spin_unlock_irqrestore(&stm32_chip->irq_lock, flags);

	ret = 0;

Done:
#if defined(DEBUG)
	printk("%s:%d[%d,%d]=%d%d\n", __func__, gpio, line, pin, trigger, ret);
#endif
	return ret;
}

#endif /* CONFIG_STM32_GPIO_INT */

/*
 * STM32-specific GPIO chip data structure
 */
static struct stm32_gpio_chip stm32_gpio_chip = {
	.chip			= {
		.label			= "stm32_gpio",
		.direction_input	= stm32_gpio_direction_input,
		.get			= stm32_gpio_get_value,
		.direction_output	= stm32_gpio_direction_output,
		.set			= stm32_gpio_set_value,
		.base			= STM32_GPIO_OFF,
		.ngpio			= STM32_GPIO_LEN,
		.can_sleep		= 1,
#if defined (CONFIG_STM32_GPIO_INT)
		.to_irq			= stm32_gpio_to_irq,
#endif
	},
	.lock			= SPIN_LOCK_UNLOCKED,
#if defined (CONFIG_STM32_GPIO_INT)
	.irq_chip		= {
		.name			= "GPIO",
		.enable			= stm32_gpio_irq_enable,
		.disable		= stm32_gpio_irq_disable,
		.set_type		= stm32_gpio_irq_set_type,
	},
	.irq_lock		= SPIN_LOCK_UNLOCKED,
	.irq_base		= NVIC_IRQS,
	.exti_irqs		=  {
		6,			/* EXTI0 */
		7,			/* EXTI1 */
		8,			/* EXTI2 */
		9,			/* EXTI3 */
		10,			/* EXTI4 */
		23,			/* EXTI[9:5] */
		40,			/* EXTI[15:10] */
	},
#endif
};

#endif /* CONFIG_STM32_GPIO */

/*
 * Initialize the platform specific GPIO subsystem
 */
void __init stm32_gpio_init(void)
{
#if defined(CONFIG_STM32_GPIO)

	struct stm32_gpio_chip *stm32_chip = &stm32_gpio_chip;

	int ret;
#if defined (CONFIG_STM32_GPIO_INT)
	int i;

	/*
	 * Indicate that intially there is no GPIO connected to any EXTI line
	 */
	for (i = 0; i < STM32_EXTI_LINES_LEN; i++) {
		stm32_chip->exti_2_gpio[i] = -1;
	}

	/*
	 * Register an IRQ handler for each EXTI IRQ line
	 */
	for (i = 0; i < STM32_EXTI_IRQS_LEN; i++) {
		set_irq_chained_handler(stm32_chip->exti_irqs[i],
					stm32_exti_irq_handler);
		set_irq_chip_data(stm32_chip->exti_irqs[i], stm32_chip);
	}

	/*
	 * Register a generic handler for each logical GPIO IRQ line
	 */
	for (i = 0; i < STM32_GPIO_LEN; i++) {
		set_irq_chip(i + stm32_chip->irq_base, &stm32_chip->irq_chip);
		set_irq_handler(i + stm32_chip->irq_base, handle_simple_irq);
		set_irq_flags(i + stm32_chip->irq_base, IRQF_VALID);
		set_irq_chip_data(i + stm32_chip->irq_base, stm32_chip);
	}
#endif

	/*
	 * Register GPIO chip
	 */
	if ((ret = gpiochip_add(&stm32_chip->chip)) < 0) {
		pr_err("%s: gpio_chip add failed: %d\n", __func__, ret);
	}

#endif
}
