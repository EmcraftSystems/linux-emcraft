/*
 * Copyright (C) 2017 Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * STM32 ADCx definitions shared with user-space apps
 *
 * License terms: GNU General Public License (GPL), version 2
 */
#ifndef _STM32_ADC_H_
#define _STM32_ADC_H_

/*
 * Maximum number of channels in sequence per measurement
 */
#define STM32_ADC_SEQ_MAX	16

/*
 * ADC measurements configuration
 */
#define STM32_ADC1_MEAS		5
#define STM32_ADC1_SEQ		{ 17, 18 }

#define STM32_ADC2_MEAS		0
#define STM32_ADC2_SEQ		{ 0 }

#define STM32_ADC3_MEAS		10
#define STM32_ADC3_SEQ		{ 0, 8, 7, 6, 5, 4 }

/*
 * IOCTLs
 */
#define STM32_ADC_IOC_MAGIC	'A'
#define STM32_ADC_SINGLE_SEQ	_IO(STM32_ADC_IOC_MAGIC, 0)
#define STM32_ADC_LOOP_SEQ	_IO(STM32_ADC_IOC_MAGIC, 1)
#define STM32_ADC_STOP_SEQ	_IO(STM32_ADC_IOC_MAGIC, 2)

/*
 * This structure is available via mmap():
 * - data[STM32_ADCx_MEAS][STM32_ADCx_CHAN]: array of measured values on ADCx
 */
struct stm32_adc_mmap {
	unsigned int		data[1][1];
};

#endif /* _STM32_ADC_H_ */
