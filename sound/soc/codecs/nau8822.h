/*
 * nau8822.h  --  Nuvoton NAU8822 ALSA Soc codec driver
 *
 * Copyright (c) 2012 Nuvoton Technology Corporation
 * Written by Ashish Chavan
 *
 * Tested on Samsung SMDK6410 + NAU8822 EVB using I2C and I2S
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __NAU8822_H__
#define __NAU8822_H__

/*
 * Register values.
 */
#define NAU8822_RESET				0x00
#define NAU8822_POWER_MANAGEMENT_1		0x01
#define NAU8822_POWER_MANAGEMENT_2		0x02
#define NAU8822_POWER_MANAGEMENT_3		0x03
#define NAU8822_AUDIO_INTERFACE			0x04
#define NAU8822_COMPANDING			0x05
#define NAU8822_CLOCK_CONTROL_1			0x06
#define NAU8822_CLOCK_CONTROL_2			0x07
#define NAU8822_GPIO				0x08
#define NAU8822_JACK_DETECT_1			0x09
#define NAU8822_DAC_CONTROL			0x0A
#define NAU8822_LEFT_DAC_VOLUME			0x0B
#define NAU8822_RIGHT_DAC_VOLUME		0x0C
#define NAU8822_JACK_DETECT_2			0x0D
#define NAU8822_ADC_CONTROL			0x0E
#define NAU8822_LEFT_ADC_VOLUME			0x0F
#define NAU8822_RIGHT_ADC_VOLUME		0x10
#define NAU8822_EQ1_LOW_CUTOFF			0x12
#define NAU8822_EQ2_PEAK_1			0x13
#define NAU8822_EQ3_PEAK_2			0x14
#define NAU8822_EQ4_PEAK_3			0x15
#define NAU8822_EQ5_HIGH_CUTOFF			0x16
#define NAU8822_DAC_LIMITER_1			0x18
#define NAU8822_DAC_LIMITER_2			0x19
#define NAU8822_NOTCH_FILTER_1			0x1b
#define NAU8822_NOTCH_FILTER_2			0x1c
#define NAU8822_NOTCH_FILTER_3			0x1d
#define NAU8822_NOTCH_FILTER_4			0x1e
#define NAU8822_ALC_CONTROL_1			0x20
#define NAU8822_ALC_CONTROL_2			0x21
#define NAU8822_ALC_CONTROL_3			0x22
#define NAU8822_NOISE_GATE			0x23
#define NAU8822_PLL_N				0x24
#define NAU8822_PLL_K1				0x25
#define NAU8822_PLL_K2				0x26
#define NAU8822_PLL_K3				0x27
#define NAU8822_3D_CONTROL			0x29
#define NAU8822_RIGHT_SPEAKER_SUBMIXER		0x2b
#define NAU8822_INPUT_CONTROL			0x2c
#define NAU8822_LEFT_INP_PGA_GAIN		0x2d
#define NAU8822_RIGHT_INP_PGA_GAIN		0x2e
#define NAU8822_LEFT_ADC_BOOST			0x2f
#define NAU8822_RIGHT_ADC_BOOST			0x30
#define NAU8822_OUTPUT_CONTROL			0x31
#define NAU8822_LEFT_MIXER			0x32
#define NAU8822_RIGHT_MIXER			0x33
#define NAU8822_LHP_VOLUME			0x34
#define NAU8822_RHP_VOLUME			0x35
#define NAU8822_LSPKOUT_VOLUME			0x36
#define NAU8822_RSPKOUT_VOLUME			0x37
#define NAU8822_AUX2_MIXER			0x38
#define NAU8822_AUX1_MIXER			0x39

#define NAU8822_MAX_REGISTER			0x39

/* Clock divider Id's */
enum nau8822_clk_id {
	NAU8822_IMCLKRATE,
	NAU8822_BCLKDIV,
};

/* Sys clk source Id's */
enum nau8822_sysclk_src {
	NAU8822_PLL,
	NAU8822_MCLK,
};

extern struct snd_soc_codec_device soc_codec_dev_nau8822;
extern struct snd_soc_dai nau8822_dai;

#endif	/* __NAU8822_H__ */
