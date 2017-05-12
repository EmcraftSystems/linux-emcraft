/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#ifndef __STM32_DAC_H
#define __STM32_DAC_H

#include <linux/types.h>

#define STM32_DAC_FORMAT_BIT		12
#define STM32_DAC_MAX_VALUE		((1 << STM32_DAC_FORMAT_BIT) - 1)
#define STM32_DAC_MAX_BUF_LEN		(32 * 1024)
#define STM32_DAC_ELEMENT_SIZE		sizeof(unsigned short)

#define STM32_DAC_IOC_MAGIC		'Z'
#define STM32_DAC_SINGLE_SEQ		_IOW(STM32_DAC_IOC_MAGIC, 0, __s32)
#define STM32_DAC_LOOP_SEQ		_IOW(STM32_DAC_IOC_MAGIC, 1, __s32)

struct stm32_dac_mmap {
	unsigned short data[STM32_DAC_MAX_BUF_LEN];
};

#endif /* __STM32_DAC_H */
