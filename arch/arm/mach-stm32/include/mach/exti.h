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

#ifndef _MACH_STM32_EXTI_H_
#define _MACH_STM32_EXTI_H_

/* RTC Alarm event */
#define STM32F2_EXTI_LINE_RTC_ALARM	17
/* RTC Tamper and TimeStamp events */
#define STM32F2_EXTI_LINE_RTC_TIMESTAMP	21
/* RTC Wakeup event */
#define STM32F2_EXTI_LINE_RTC_WAKEUP	22
/* Total number of EXTI event lines */
#define STM32F2_EXTI_NUM_LINES		23

/*
 * API functions of the STM32 EXTI controller driver
 *
 * See arch/arm/mach-stm32/exti.c for details on each of these functions.
 */
void stm32_exti_enable_int(unsigned int line, int enable);
void stm32_exti_clear_pending(unsigned int line);

#endif /* _MACH_STM32_EXTI_H_ */
