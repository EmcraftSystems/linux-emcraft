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
 * GPIO index map
 */
/* Kinetis GPIOs */
#define KINETIS_GPIO_OFF_MCU	0
#define KINETIS_GPIO_LEN_MCU \
	KINETIS_GPIO_MKPIN(KINETIS_GPIO_PORTS - 1, KINETIS_GPIO_PORT_PINS)

#define ARCH_NR_GPIOS		KINETIS_GPIO_LEN_MCU

#if defined(CONFIG_GPIOLIB)
#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_to_irq	__gpio_to_irq
#define gpio_cansleep	__gpio_cansleep

#include <asm-generic/gpio.h>
#endif /* CONFIG_GPIOLIB */

void __init kinetis_gpio_init(void);

#endif /* _MACH_KINETIS_GPIO_H_ */
