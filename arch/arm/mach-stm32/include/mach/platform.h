/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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

#ifndef __ASM_ARCH_PLATFORM_H
#define __ASM_ARCH_PLATFORM_H

/*
 * STM32 Device (microcontroller) identifiers
 */
#ifndef CONFIG_ARCH_STM32F1
/* STM32F2-based and STM32F4-based */
#define DEVICE_STM32F207IG		0
#define DEVICE_STM32F407IG		1
#define DEVICE_STM32F437II		2
#define DEVICE_STM32F439II		3
#else
/* STM32F1-based */
#define DEVICE_STM32F103ZE		1000
#endif

/*
 * STM32 platform identifiers
 */
#ifndef CONFIG_ARCH_STM32F1
/* STM32F2-based */
#define PLATFORM_STM32_STM3220G_EVAL		0
#define PLATFORM_STM32_STM3240G_EVAL		1
#define PLATFORM_STM32_STM_SOM			2
#define PLATFORM_STM32_STM_STM32F439_SOM	3
#define PLATFORM_STM32_STM_DISCO		4
#else
/* STM32F1-based */
#define PLATFORM_STM32_SWISSEMBEDDED_COMM	1000
#endif

/*
 * Get device DEVICE_STM32xxx ID we're running on
 */
int stm32_device_get(void);

/*
 * Get platform PLATFORM_STM32_xxx ID we're running om
 */
int stm32_platform_get(void);

#endif /* __ASM_ARCH_PLATFORM_H */
