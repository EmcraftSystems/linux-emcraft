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

#ifndef _MACH_LPC18XX_GPIO_H_
#define _MACH_LPC18XX_GPIO_H_

#include <mach/lpc18xx.h>

/* Number of GPIO ports */
#define LPC18XX_GPIO_PORTS		8
/* Maximum number of pins in each group */
#define LPC18XX_GPIO_PORT_PINS		32

/*
 * Convert a {port, pin} pair to an single integer
 */
#define LPC18XX_GPIO_MKPIN(port,pin) \
	((LPC18XX_GPIO_PORT_PINS) * (port) + (pin))

/*
 * Backward conversion
 */
#define LPC18XX_GPIO_GETPIN(number) \
	(number % LPC18XX_GPIO_PORT_PINS)
#define LPC18XX_GPIO_GETPORT(number) \
	(number / LPC18XX_GPIO_PORT_PINS)

/*
 * System Control Unit (SCU) registers base
 */
#define LPC18XX_SCU_BASE	(LPC18XX_APB0PERIPH_BASE + 0x00006000)

/*
 * GPIO index map
 */
/* Kinetis GPIOs */
#define LPC18XX_GPIO_OFF_MCU	0
#define LPC18XX_GPIO_LEN_MCU \
	LPC18XX_GPIO_MKPIN(LPC18XX_GPIO_PORTS - 1, LPC18XX_GPIO_PORT_PINS)

#define ARCH_NR_GPIOS		LPC18XX_GPIO_LEN_MCU

#if defined(CONFIG_GPIOLIB)
#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_to_irq	__gpio_to_irq
#define gpio_cansleep	__gpio_cansleep

#include <asm-generic/gpio.h>
#endif /* CONFIG_GPIOLIB */

void __init lpc18xx_gpio_init(void);

/* FIXME this should be done via standard GPIO IRQ mechanism */
/*
 * GPIO interrupt types
 */
enum GPIO_INTERRUPT_TYPE {
	GPIO_INTERRUPT_FREE = 0,
	GPIO_INTERRUPT_EDGE_RISING = 0x1,
	GPIO_INTERRUPT_EDGE_FALLING = 0x2,
	GPIO_INTERRUPT_EDGE_MASK = 0x3,

	GPIO_INTERRUPT_LEVEL_HIGH = 0x4,
	GPIO_INTERRUPT_LEVEL_LOW = 0x8,
	GPIO_INTERRUPT_LEVEL_MASK = 0xC,
};

/*
 * Request GPIO for specific port/pin
 */
unsigned int lpc18xx_gpio_irq_request(int port, int pin);

/* Free GPIO */
void lpc18xx_gpio_irq_free(unsigned int irq);

#endif /* _MACH_LPC18XX_GPIO_H_ */
