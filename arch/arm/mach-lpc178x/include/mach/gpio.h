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

#include <linux/errno.h>

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
 * GPIO index map
 */
/* LPC178x GPIOs */
#define LPC178X_GPIO_OFF_MCU	0
#define LPC178X_GPIO_LEN_MCU \
	LPC178X_GPIO_MKPIN(LPC178X_GPIO_PORTS - 1, LPC178X_GPIO_LAST_PORT_PINS)
/* GPIOs on the PCA9532 chip on Embedded Artists OEM Base Board */
#define LPC178X_GPIO_OFF_EA_LPC1788_PCA9532	LPC178X_GPIO_LEN_MCU
#define LPC178X_GPIO_LEN_EA_LPC1788_PCA9532	16

#define ARCH_NR_GPIOS \
	(LPC178X_GPIO_OFF_EA_LPC1788_PCA9532 + \
	LPC178X_GPIO_LEN_EA_LPC1788_PCA9532)

#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_to_irq	__gpio_to_irq
#define gpio_cansleep	__gpio_cansleep

#include <asm-generic/gpio.h>

void __init lpc178x_gpio_init(void);

#endif /* _MACH_LPC178X_GPIO_H_ */
