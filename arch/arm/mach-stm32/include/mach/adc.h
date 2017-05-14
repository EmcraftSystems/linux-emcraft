/*
 * (C) Copyright 2017 Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * License terms: GNU General Public License (GPL), version 2
 */

#ifndef _MACH_STM32_ADC_H_
#define _MACH_STM32_ADC_H_

#include <linux/init.h>

/*
 * Maximum size of conversion sequence
 */
#define STM32_ADC_CHAN_NUM	16

/*
 * STM32 ADCs
 */
enum stm32_adc {
	STM32_ADC0		= 0,
	STM32_ADC1,
	STM32_ADC2,

	STM32_ADC_NUM
};

/*
 * Sampling times
 */
enum stm32_adc_smp {
	STM32_ADC_SMP3  = 0,
	STM32_ADC_SMP15,
	STM32_ADC_SMP28,
	STM32_ADC_SMP56,
	STM32_ADC_SMP84,
	STM32_ADC_SMP112,
	STM32_ADC_SMP144,
	STM32_ADC_SMP480,

	STM32_ADC_SMP_NUM
};

/*
 * Info about ADCx channels used in conversion:
 * - trig_usec: sequence measurement period, usec
 * - trig_pwm_id: trigger PWM id
 * - trig_adc_code: EXTSEL code
 * - smp: sampling time
 * - adc[]: per ADCx device
 * -- dma: DMA channel used
 * -- meas: number of measurements to do
 * -- chan: number of ADCx channels used in conversions
 * -- seq[]: ADCx channels themselves
 */
struct stm32_adc_platform_data {
	int			trig_usec;
	int			trig_pwm_id;
	int			trig_adc_code;
	enum stm32_adc_smp	smp;
	struct {
		int		dma;
		int		meas;
		int		chan;
		int		seq[STM32_ADC_CHAN_NUM];
	} adc[STM32_ADC_NUM];
};

void __init stm32_adc_init(void);

#endif /* _MACH_STM32_ADC_H_ */
