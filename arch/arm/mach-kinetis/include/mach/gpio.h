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

#ifndef _MACH_KINETIS_GPIO_H_
#define _MACH_KINETIS_GPIO_H_

#include <mach/kinetis.h>

/* Number of GPIO ports (5 ports on K60; 6 ports on K70) */
#define KINETIS_GPIO_PORTS		6
/* Number of pins in each port */
#define KINETIS_GPIO_PORT_PINS		32

/*
 * GPIO ports
 */
#define KINETIS_GPIO_PORT_A	0
#define KINETIS_GPIO_PORT_B	1
#define KINETIS_GPIO_PORT_C	2
#define KINETIS_GPIO_PORT_D	3
#define KINETIS_GPIO_PORT_E	4
#define KINETIS_GPIO_PORT_F	5

/*
 * Convert a {port, pin} pair to an single integer
 */
#define KINETIS_GPIO_MKPIN(port,pin) \
	((KINETIS_GPIO_PORT_PINS) * (port) + (pin))

/*
 * Backward conversion
 */
#define KINETIS_GPIO_GETPIN(number) \
	(number % KINETIS_GPIO_PORT_PINS)
#define KINETIS_GPIO_GETPORT(number) \
	(number / KINETIS_GPIO_PORT_PINS)

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
static inline int gpio_get_value(unsigned gpio)
{
	return (KINETIS_GPIO(KINETIS_GPIO_GETPORT(gpio))->pdir >>
		KINETIS_GPIO_GETPIN(gpio)) & 1;
}

/*
 * Change the direction of a GPIO pin to input
 */
static inline int gpio_direction_input(unsigned gpio)
{
	KINETIS_GPIO(KINETIS_GPIO_GETPORT(gpio))->pddr &=
		~(1 << KINETIS_GPIO_GETPIN(gpio));

	return 0;
}

/*
 * Set the state of a GPIO output pin
 */
static inline void gpio_set_value(unsigned gpio, int value)
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
static inline int gpio_direction_output(unsigned gpio, int level)
{
	KINETIS_GPIO(KINETIS_GPIO_GETPORT(gpio))->pddr |=
		(1 << KINETIS_GPIO_GETPIN(gpio));

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
	return number >= 0 && number <= KINETIS_GPIO_MKPIN(
		KINETIS_GPIO_PORTS - 1, KINETIS_GPIO_PORT_PINS - 1);
}

/*
 * We do not support GPIO interrupts
 */
static inline int gpio_to_irq(int gpio)
{
	return -EINVAL;
}

#endif /* _MACH_KINETIS_GPIO_H_ */
