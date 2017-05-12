/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef _MACH_STM32_DAC_H_
#define _MACH_STM32_DAC_H_

#include <linux/init.h>

#define STM32_DAC_ID_1		(1 << 0)
#define STM32_DAC_ID_2		(1 << 1)
#define STM32_TIMER_ID(x)	((x) << 2)
#define STM32_GET_TIMER_ID(x)	(((x) >> 2) & 0xf)

void __init stm32_dac_init(void);

#endif /* _MACH_STM32_DAC_H_ */
