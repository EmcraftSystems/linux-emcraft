/*
 * (C) Copyright 2012-2013
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

#if 0
#define DEBUG
#endif

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
 * Set the state of a GPIO output pin
 */
static void stm32_gpio_set_value(
	struct gpio_chip *chip, unsigned gpio, int v)
{
	int port = STM32_GPIO_GETPORT(gpio);
	int pin = STM32_GPIO_GETPIN(gpio);
	volatile struct stm32f2_gpio_regs *gpio_regs =
		(struct stm32f2_gpio_regs *) stm32_gpio_base[port];

	/*
	 * Set the output
	 */
	gpio_regs->odr &= ~(1 << pin);
	gpio_regs->odr |= (v << pin);

#if defined(DEBUG)
	printk("%s:%d[%d,%d]=%d\n", __func__, gpio, port, pin, v);
#endif
}

/*
 * Get the current state of a GPIO input pin
 */
static int stm32_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	int port = STM32_GPIO_GETPORT(gpio);
	int pin = STM32_GPIO_GETPIN(gpio);
	volatile struct stm32f2_gpio_regs *gpio_regs =
		(struct stm32f2_gpio_regs *) stm32_gpio_base[port];
	int ret = (gpio_regs->idr & (1 << pin)) ? 1 : 0;

#if defined(DEBUG)
	printk("%s:%d[%d,%d]=%d\n", __func__, gpio, port, pin, ret);
#endif
	return ret;
}

/*
 * Change the direction of a GPIO pin to input
 */
static int stm32_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	int port = STM32_GPIO_GETPORT(gpio);
	int pin = STM32_GPIO_GETPIN(gpio);
	int shf = pin * 2;
	volatile struct stm32f2_gpio_regs *gpio_regs =
		(struct stm32f2_gpio_regs *) stm32_gpio_base[port];

	/*
	 * Set the direction to input
	 */
	gpio_regs->moder &= ~(3 << shf);
#if defined(DEBUG)
	printk("%s:%d[%d,%d]\n", __func__, gpio, port, pin);
#endif
	return 0;
}

/*
 * Change the direction of a GPIO pin to output and
 * set the level on this pin.
 */
static int stm32_gpio_direction_output(
	struct gpio_chip *chip, unsigned gpio, int level)
{
	int port = STM32_GPIO_GETPORT(gpio);
	int pin = STM32_GPIO_GETPIN(gpio);
	int shf = pin * 2;
	volatile struct stm32f2_gpio_regs *gpio_regs =
		(struct stm32f2_gpio_regs *) stm32_gpio_base[port];

	/*
	 * Set the direction to output
	 */
	gpio_regs->moder &= ~(3 << shf);
	gpio_regs->moder |= (1 << shf);

	/*
	 * Define the level
	 */
	gpio_set_value(gpio, level);

#if defined(DEBUG)
	printk("%s:%d[%d,%d]=%d\n", __func__, gpio, port, pin, level);
#endif
	return 0;
}

static struct gpio_chip stm32_chip = {
	.label			= "stm32",
	.direction_input	= stm32_gpio_direction_input,
	.get			= stm32_gpio_get_value,
	.direction_output	= stm32_gpio_direction_output,
	.set			= stm32_gpio_set_value,
	.base			= STM32_GPIO_OFF,
	.ngpio			= STM32_GPIO_LEN,
	.can_sleep		= 1,
};

void __init stm32_gpio_init(void)
{
	if (gpiochip_add(&stm32_chip) < 0)
		pr_err("%s: gpiochip_add failed.\n", __func__);
}
