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

#ifndef _MACH_LPC178X_GPIO_H_
#define _MACH_LPC178X_GPIO_H_

#include <mach/lpc178x.h>

/*
 * List of available GPIO pins:
 *    P0[31:0]; P1[31:0]; P2[31:0]; P3[31:0]; P4[31:0]; P5[4:0]
 */
/* Number of GPIO ports */
#define LPC178X_GPIO_PORTS		6
/* Number of pins in all ports except the last one */
#define LPC178X_GPIO_NORMAL_PORT_PINS	32
/* Number of pins in the last port */
#define LPC178X_GPIO_LAST_PORT_PINS	5

/*
 * Convert a {port, pin} pair to an single integer
 */
#define LPC178X_GPIO_MKPIN(port,pin) \
	((LPC178X_GPIO_NORMAL_PORT_PINS) * (port) + (pin))

/*
 * Backward conversion
 */
#define LPC178X_GPIO_GETPIN(number) \
	(number % LPC178X_GPIO_NORMAL_PORT_PINS)
#define LPC178X_GPIO_GETPORT(number) \
	(number / LPC178X_GPIO_NORMAL_PORT_PINS)

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
static inline int gpio_get_value(unsigned gpio)
{
	return (LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fiopin >>
		LPC178X_GPIO_GETPIN(gpio)) & 1;
}

/*
 * Change the direction of a GPIO pin to input
 */
static inline int gpio_direction_input(unsigned gpio)
{
	LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fiodir &=
		~(1 << LPC178X_GPIO_GETPIN(gpio));

	return 0;
}

/*
 * Set the state of a GPIO output pin
 */
static inline void gpio_set_value(unsigned gpio, int value)
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
static inline int gpio_direction_output(unsigned gpio, int level)
{
	LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fiodir |=
		(1 << LPC178X_GPIO_GETPIN(gpio));

	gpio_set_value(gpio, level);

	return 0;
}

/*
 * We do not support GPIO configuration for now
 */
static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

/*
 * We do not support GPIO configuration for now
 */
static inline void gpio_free(unsigned gpio)
{
	might_sleep();
}

/*
 * Verify that the pin number identifies an existing pin
 */
static inline int gpio_is_valid(int number)
{
	return number >= 0 && number <= LPC178X_GPIO_MKPIN(
		LPC178X_GPIO_PORTS - 1, LPC178X_GPIO_LAST_PORT_PINS - 1);
}

/*
 * We do not support GPIO interrupts
 */
static inline int gpio_to_irq(int gpio)
{
	return -EINVAL;
}

#endif /* _MACH_LPC178X_GPIO_H_ */
