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
#include <linux/init.h>

#include <mach/gpio.h>

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
