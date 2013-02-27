/*
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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

#ifndef _MACH_M2S_GPIO_H_
#define _MACH_M2S_GPIO_H_

/* 32 MSS and 32 CORE GPIOs */
#define ARCH_NR_GPIOS		(32 + 32)

#if defined(CONFIG_GPIOLIB)
#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_to_irq	__gpio_to_irq
#define gpio_cansleep	__gpio_cansleep

#include <asm-generic/gpio.h>
#endif /* CONFIG_GPIOLIB */

void __init m2s_gpio_init(void);

#endif /* _MACH_M2S_GPIO_H_ */
