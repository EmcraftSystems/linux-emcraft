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

#include <mach/lpc178x.h>
#include <mach/gpio.h>

/*
 * GPIO register map
 * Should be mapped at (0x20098000 + port * 0x20).
 */
struct lpc178x_gpio_regs {
	u32 fiodir;	/* Fast GPIO Port Direction control register */
	u32 rsv0[3];
	u32 fiomask;	/* Fast Mask register for port */
	u32 fiopin;	/* Fast Port Pin value register using FIOMASK */
	u32 fioset;	/* Fast Port Output Set register using FIOMASK */
	u32 fioclr;	/* Fast Port Output Clear register using FIOMASK */
};

/*
 * GPIO registers base
 */
#define LPC178X_GPIO_BASE		(LPC178X_AHB_PERIPH_BASE + 0x00018000)
#define LPC178X_GPIO_PORT_ADDR(port)	(LPC178X_GPIO_BASE + (port) * 0x20)
#define LPC178X_GPIO(port) \
	((volatile struct lpc178x_gpio_regs *)LPC178X_GPIO_PORT_ADDR(port))

/*
 * Get the current state of a GPIO input pin
 */
static int lpc178x_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	return (LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fiopin >>
		LPC178X_GPIO_GETPIN(gpio)) & 1;
}

/*
 * Change the direction of a GPIO pin to input
 */
static int lpc178x_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fiodir &=
		~(1 << LPC178X_GPIO_GETPIN(gpio));

	return 0;
}

/*
 * Set the state of a GPIO output pin
 */
static void lpc178x_gpio_set_value(
	struct gpio_chip *chip, unsigned gpio, int value)
{
	if (value) {
		LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fioset =
			(1 << LPC178X_GPIO_GETPIN(gpio));
	} else {
		LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fioclr =
			(1 << LPC178X_GPIO_GETPIN(gpio));
	}
}

/*
 * Change the direction of a GPIO pin to output and
 * set the level on this pin.
 */
static int lpc178x_gpio_direction_output(
	struct gpio_chip *chip, unsigned gpio, int level)
{
	LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fiodir |=
		(1 << LPC178X_GPIO_GETPIN(gpio));

	lpc178x_gpio_set_value(chip, gpio, level);

	return 0;
}

static struct gpio_chip lpc178x_chip = {
	.label			= "lpc178x",
	.direction_input	= lpc178x_gpio_direction_input,
	.get			= lpc178x_gpio_get_value,
	.direction_output	= lpc178x_gpio_direction_output,
	.set			= lpc178x_gpio_set_value,
	.base			= LPC178X_GPIO_OFF_MCU,
	.ngpio			= LPC178X_GPIO_LEN_MCU,
	.can_sleep		= 1,
};

void __init lpc178x_gpio_init(void)
{
	if (gpiochip_add(&lpc178x_chip) < 0)
		pr_err("%s: gpiochip_add failed.\n", __func__);
}
