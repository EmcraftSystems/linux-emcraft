/*
 * nau8822.c  --  Nuvoton NAU8822 ALSA Soc codec driver
 *
 * Copyright (c) 2012 Nuvoton Technology Corporation
 * Written by Ashish Chavan
 *
 * Tested on Samsung SMDK6410 + NAU8822 EVB using I2C and I2S
 *
 * Copyright (c) 2014 Emcraft Inc.
 * Backported to linux-2.6 by Pavel Boldin <paboldin@emcraft.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "nau8822.h"

static inline void *snd_soc_codec_get_drvdata(struct snd_soc_codec *codec)
{
	return codec->private_data;
}

u16 nau8822_reg_defaults[NAU8822_MAX_REGISTER + 1] = {
	[0x01] = 0x000,
	[0x02] = 0x000,
	[0x03] = 0x000,
	[0x04] = 0x050,
	[0x05] = 0x000,
	[0x06] = 0x140,
	[0x07] = 0x000,
	[0x08] = 0x000,
	[0x09] = 0x000,
	[0x0a] = 0x000,
	[0x0b] = 0x0ff,
	[0x0c] = 0x0ff,
	[0x0d] = 0x000,
	[0x0e] = 0x100,
	[0x0f] = 0x0ff,
	[0x10] = 0x0ff,
	[0x12] = 0x12c,
	[0x13] = 0x02c,
	[0x14] = 0x02c,
	[0x15] = 0x02c,
	[0x16] = 0x02c,
	[0x18] = 0x032,
	[0x19] = 0x000,
	[0x1b] = 0x000,
	[0x1c] = 0x000,
	[0x1d] = 0x000,
	[0x1e] = 0x000,
	[0x20] = 0x038,
	[0x21] = 0x00b,
	[0x22] = 0x032,
	[0x23] = 0x010,
	[0x24] = 0x008,
	[0x25] = 0x00c,
	[0x26] = 0x093,
	[0x27] = 0x0e9,
	[0x29] = 0x000,
	[0x2b] = 0x000,
	[0x2c] = 0x033,
	[0x2d] = 0x010,
	[0x2e] = 0x010,
	[0x2f] = 0x100,
	[0x30] = 0x100,
	[0x31] = 0x002,
	[0x32] = 0x001,
	[0x33] = 0x001,
	[0x34] = 0x039,
	[0x35] = 0x039,
	[0x36] = 0x039,
	[0x37] = 0x039,
	[0x38] = 0x001,
	[0x39] = 0x001,
};

/*
 * Following registers contain an "update" bit i.e. bit 8. This means that
 * when this "update" bit is set then only corresponding values come into
 * effect. e.g.,
 *
 * We can write new DAC volume for both channels, but this new volume will be
 * effective only when the update bit is set. Thus if we keep this "update"
 * bit enabled, all changes to volume will come into effect immediately.
 */
static const int update_reg[] = {
	NAU8822_LEFT_DAC_VOLUME,
	NAU8822_RIGHT_DAC_VOLUME,
	NAU8822_LEFT_ADC_VOLUME,
	NAU8822_RIGHT_ADC_VOLUME,
	NAU8822_LEFT_INP_PGA_GAIN,
	NAU8822_RIGHT_INP_PGA_GAIN,
	NAU8822_LHP_VOLUME,
	NAU8822_RHP_VOLUME,
	NAU8822_LSPKOUT_VOLUME,
	NAU8822_RSPKOUT_VOLUME,
};

/* codec private data */
struct nau8822_priv {
	struct snd_soc_codec codec;
	unsigned int f_mclk;
	unsigned int f_imclk;
	enum nau8822_sysclk_src sysclk;
};

struct pll_div {
	int fref;
	int fout;
	u16 K1;
	u16 K2;
	u16 K3;
	u8 N;
};

/* PLL divisors table */
static const struct pll_div nau8822_pll_div[] = {
	/* fs = 44.1KHz and harmonics */
	{ 12000000, 1128960, 0x021, 0x161, 0x026, 0x07},    /* MCLK=12Mhz */
	{ 14400000, 1128960, 0x011, 0x0D0, 0x1CA, 0x06},    /* MCLK=14.4Mhz */
	{ 19200000, 1128960, 0x01A, 0x039, 0x0B0, 0x09},    /* MCLK=19.2Mhz */
	{ 19800000, 1128960, 0x007, 0x1BB, 0x0F8, 0x09},    /* MCLK=19.8Mhz */
	{ 24000000, 1128960, 0x021, 0x161, 0x026, 0x07},    /* MCLK=24Mhz */
	{ 26000000, 1128960, 0x03C, 0x145, 0x1D4, 0x06},    /* MCLK=26Mhz */
	/* fs = 48KHz and harmonics */
	{ 12000000, 1228800, 0x00C, 0x093, 0x0E9, 0x08},    /* MCLK=12Mhz */
	{ 14400000, 1228800, 0x034, 0x1D0, 0x06D, 0x06},    /* MCLK=14.4Mhz */
	{ 19200000, 1228800, 0x00F, 0x0B8, 0x0A3, 0x0A},    /* MCLK=19.2Mhz */
	{ 19800000, 1228800, 0x03B, 0x100, 0x09E, 0x09},    /* MCLK=19.8Mhz */
	{ 24000000, 1228800, 0x00C, 0x093, 0x0E9, 0x08},    /* MCLK=24Mhz */
	{ 26000000, 1228800, 0x023, 0x1EA, 0x126, 0x07},    /* MCLK=26Mhz */
};

static const char * const nau8822_companding[] = {"Off", "NC", "u-law",
						  "A-law"};
static const char * const nau8822_micbias_volt[] = {"0.9x", "0.65x", "0.75x",
						    "0.50x"};
static const char * const nau8822_eqmode[] = {"Capture", "Playback"};
static const char * const nau8822_bw[] = {"Narrow", "Wide"};
static const char * const nau8822_eq1[] = {"80Hz", "105Hz", "135Hz", "175Hz"};
static const char * const nau8822_eq2[] = {"230Hz", "300Hz", "385Hz", "500Hz"};
static const char * const nau8822_eq3[] = {"650Hz", "850Hz", "1.1kHz",
					   "1.4kHz"};
static const char * const nau8822_eq4[] = {"1.8kHz", "2.4kHz", "3.2kHz",
					   "4.1kHz"};
static const char * const nau8822_eq5[] = {"5.3kHz", "6.9kHz", "9kHz",
					   "11.7kHz"};
static const char * const nau8822_mode[] = {"ALC", "Limiter"};
static const char * const nau8822_chn_select[] = {"Off", "Right", "Left",
						  "Both"};
static const char * const nau8822_rspk_mode[] = {"Normal", "Inverted"};

#define SOC_ENUM_SINGLE_DECL(name, xreg, xshift, xtexts)	\
	struct soc_enum name = SOC_ENUM_SINGLE(xreg, xshift, ARRAY_SIZE(xtexts), (const char **)xtexts)


static const SOC_ENUM_SINGLE_DECL(adc_compand, NAU8822_COMPANDING, 1,
				  nau8822_companding);
static const SOC_ENUM_SINGLE_DECL(dac_compand, NAU8822_COMPANDING, 3,
				  nau8822_companding);
static const SOC_ENUM_SINGLE_DECL(micbias, NAU8822_INPUT_CONTROL, 7,
				  nau8822_micbias_volt);
static const SOC_ENUM_SINGLE_DECL(eqmode, NAU8822_EQ1_LOW_CUTOFF, 8,
				  nau8822_eqmode);
static const SOC_ENUM_SINGLE_DECL(eq1, NAU8822_EQ1_LOW_CUTOFF, 5, nau8822_eq1);
static const SOC_ENUM_SINGLE_DECL(eq2bw, NAU8822_EQ2_PEAK_1, 8, nau8822_bw);
static const SOC_ENUM_SINGLE_DECL(eq2, NAU8822_EQ2_PEAK_1, 5, nau8822_eq2);
static const SOC_ENUM_SINGLE_DECL(eq3bw, NAU8822_EQ3_PEAK_2, 8, nau8822_bw);
static const SOC_ENUM_SINGLE_DECL(eq3, NAU8822_EQ3_PEAK_2, 5, nau8822_eq3);
static const SOC_ENUM_SINGLE_DECL(eq4bw, NAU8822_EQ4_PEAK_3, 8, nau8822_bw);
static const SOC_ENUM_SINGLE_DECL(eq4, NAU8822_EQ4_PEAK_3, 5, nau8822_eq4);
static const SOC_ENUM_SINGLE_DECL(eq5, NAU8822_EQ5_HIGH_CUTOFF, 5, nau8822_eq5);
static const SOC_ENUM_SINGLE_DECL(alc_mode, NAU8822_ALC_CONTROL_3, 8,
				  nau8822_mode);
static const SOC_ENUM_SINGLE_DECL(alc_chn_select, NAU8822_ALC_CONTROL_1, 7,
				  nau8822_chn_select);
static const SOC_ENUM_SINGLE_DECL(rspk_mode, NAU8822_RIGHT_SPEAKER_SUBMIXER, 4,
				  nau8822_rspk_mode);

static const DECLARE_TLV_DB_SCALE(digital_tlv, -12750, 50, 1);
static const DECLARE_TLV_DB_SCALE(eq_tlv, -1200, 100, 0);
static const DECLARE_TLV_DB_SCALE(inpga_tlv, -1200, 75, 0);
static const DECLARE_TLV_DB_SCALE(spk_tlv, -5700, 100, 0);
static const DECLARE_TLV_DB_SCALE(boost_tlv, -1500, 300, 1);
static const DECLARE_TLV_DB_SCALE(limiter_tlv, 0, 100, 0);
static const DECLARE_TLV_DB_SCALE(mixer_tlv, -1500, 300, 0);
static const DECLARE_TLV_DB_SCALE(alc_min_tlv, -1200, 600, 0);
static const DECLARE_TLV_DB_SCALE(alc_max_tlv, -675, 600, 0);
static const DECLARE_TLV_DB_SCALE(ng_tlv, -8100, 600, 0);

static const struct snd_kcontrol_new nau8822_snd_controls[] = {

	SOC_DOUBLE_R_TLV("DAC Volume",
			 NAU8822_LEFT_DAC_VOLUME, NAU8822_RIGHT_DAC_VOLUME,
			 0, 255, 0, digital_tlv),

	SOC_DOUBLE_R_TLV("ADC Volume",
			 NAU8822_LEFT_ADC_VOLUME, NAU8822_RIGHT_ADC_VOLUME,
			 0, 255, 0, digital_tlv),

	SOC_SINGLE("High Pass Filter Enable", NAU8822_ADC_CONTROL, 8, 1, 0),
	SOC_SINGLE("High Pass Cut Off", NAU8822_ADC_CONTROL, 4, 7, 0),

	SOC_ENUM("Equaliser Mode", eqmode),
	SOC_ENUM("EQ1 Cut Off", eq1),
	SOC_SINGLE_TLV("EQ1 Volume", NAU8822_EQ1_LOW_CUTOFF,  0, 24, 1, eq_tlv),

	SOC_ENUM("Equaliser EQ2 Bandwidth", eq2bw),
	SOC_ENUM("EQ2 Cut Off", eq2),
	SOC_SINGLE_TLV("EQ2 Volume", NAU8822_EQ2_PEAK_1,  0, 24, 1, eq_tlv),

	SOC_ENUM("Equaliser EQ3 Bandwidth", eq3bw),
	SOC_ENUM("EQ3 Cut Off", eq3),
	SOC_SINGLE_TLV("EQ3 Volume", NAU8822_EQ3_PEAK_2,  0, 24, 1, eq_tlv),

	SOC_ENUM("Equaliser EQ4 Bandwidth", eq4bw),
	SOC_ENUM("EQ4 Cut Off", eq4),
	SOC_SINGLE_TLV("EQ4 Volume", NAU8822_EQ4_PEAK_3,  0, 24, 1, eq_tlv),

	SOC_ENUM("EQ5 Cut Off", eq5),
	SOC_SINGLE_TLV("EQ5 Volume", NAU8822_EQ5_HIGH_CUTOFF, 0, 24, 1, eq_tlv),

	SOC_SINGLE("Digital Limiter Playback Enable Switch",
		   NAU8822_DAC_LIMITER_1, 8, 1, 0),
	SOC_SINGLE("Digital Limiter Playback Decay",
		   NAU8822_DAC_LIMITER_1, 4, 15, 0),
	SOC_SINGLE("Digital Limiter Playback Attack",
		   NAU8822_DAC_LIMITER_1, 0, 15, 0),

	SOC_SINGLE("Digital Limiter Playback Threshold",
		   NAU8822_DAC_LIMITER_2, 4, 7, 0),
	SOC_SINGLE_TLV("Digital Limiter Playback Volume",
		       NAU8822_DAC_LIMITER_2, 0, 12, 0, limiter_tlv),

	/* ALC */
	SOC_ENUM("ALC Enable Switch", alc_chn_select),
	SOC_SINGLE_TLV("ALC Capture Min Gain Volume",
		       NAU8822_ALC_CONTROL_1, 0, 7, 0, alc_min_tlv),
	SOC_SINGLE_TLV("ALC Capture Max Gain Volume",
		       NAU8822_ALC_CONTROL_1, 3, 7, 0, alc_max_tlv),

	SOC_SINGLE("ALC Capture Hold", NAU8822_ALC_CONTROL_2, 4, 10, 0),
	SOC_SINGLE("ALC Capture Target", NAU8822_ALC_CONTROL_2, 0, 15, 0),

	SOC_ENUM("ALC Capture Mode", alc_mode),
	SOC_SINGLE("ALC Capture Decay", NAU8822_ALC_CONTROL_3, 4, 10, 0),
	SOC_SINGLE("ALC Capture Attack", NAU8822_ALC_CONTROL_3, 0, 10, 0),

	SOC_SINGLE("ALC Capture Noise Gate Enable",
		   NAU8822_NOISE_GATE, 3, 1, 0),
	SOC_SINGLE_TLV("ALC Capture Noise Gate Threshold Volume",
		       NAU8822_NOISE_GATE, 0, 7, 1, ng_tlv),

	/* Headphones */
	SOC_DOUBLE_R_TLV("Headphone Playback Volume",
			 NAU8822_LHP_VOLUME, NAU8822_RHP_VOLUME,
			 0, 63, 0, spk_tlv),
	SOC_DOUBLE_R("Headphone Playback ZC Switch",
		     NAU8822_LHP_VOLUME, NAU8822_RHP_VOLUME, 7, 1, 0),

	/* Speakers */
	SOC_DOUBLE_R_TLV("Speaker Playback Volume",
			 NAU8822_LSPKOUT_VOLUME, NAU8822_RSPKOUT_VOLUME,
			 0, 63, 0, spk_tlv),
	SOC_DOUBLE_R("Speaker Playback ZC Switch",
		     NAU8822_LSPKOUT_VOLUME, NAU8822_RSPKOUT_VOLUME, 7, 1, 0),

	/* Mixer : Mixer (Input) Boost */
	SOC_DOUBLE_R("PGA Boost",
		     NAU8822_LEFT_ADC_BOOST, NAU8822_RIGHT_ADC_BOOST,
		     8, 1, 0),
	SOC_DOUBLE_R_TLV("LLIN/RLIN Boost Volume",
			 NAU8822_LEFT_ADC_BOOST, NAU8822_RIGHT_ADC_BOOST,
			 4, 7, 0, boost_tlv),
	SOC_DOUBLE_R_TLV("Aux Boost Volume",
			 NAU8822_LEFT_ADC_BOOST, NAU8822_RIGHT_ADC_BOOST,
			 0, 7, 0, boost_tlv),

	/*Auxout Boost */
	SOC_SINGLE("AUXOUT1 Gain Boost", NAU8822_OUTPUT_CONTROL, 4, 1, 0),
	SOC_SINGLE("AUXOUT2 Gain Boost", NAU8822_OUTPUT_CONTROL, 3, 1, 0),

	/*Speaker Boost */
	SOC_SINGLE("SPKOUT Gain Boost", NAU8822_OUTPUT_CONTROL, 2, 1, 0),

	/* Main Mixers : Gain for Auxin, Bypass paths */
	SOC_DOUBLE_R_TLV("Line Bypass Gain Volume",
			 NAU8822_LEFT_MIXER, NAU8822_RIGHT_MIXER,
			 2, 7, 0, mixer_tlv),
	SOC_DOUBLE_R_TLV("Auxin Playback Gain Volume",
			 NAU8822_LEFT_MIXER, NAU8822_RIGHT_MIXER,
			 6, 7, 0, mixer_tlv),

	/* Right Speaker Submixer */
	SOC_ENUM("Right Speaker Submixer Mode", rspk_mode),
	SOC_SINGLE_TLV("RAUXIN Input Gain Volume",
		       NAU8822_RIGHT_SPEAKER_SUBMIXER, 1, 7, 0, mixer_tlv),

	/* Input PGA volume */
	SOC_DOUBLE_R_TLV("Input PGA Volume",
			 NAU8822_LEFT_INP_PGA_GAIN, NAU8822_RIGHT_INP_PGA_GAIN,
			 0, 63, 0, inpga_tlv),
	SOC_DOUBLE_R("Input PGA ZC Switch",
		     NAU8822_LEFT_INP_PGA_GAIN, NAU8822_RIGHT_INP_PGA_GAIN,
		     7, 1, 0),

	/* Mute Controls */
	SOC_DOUBLE_R("Headphone Output Enable",
		     NAU8822_LHP_VOLUME, NAU8822_RHP_VOLUME, 6, 1, 1),
	SOC_DOUBLE_R("Speaker Output Enable",
		     NAU8822_LSPKOUT_VOLUME, NAU8822_RSPKOUT_VOLUME, 6, 1, 1),
	SOC_DOUBLE_R("AUX Output Enable",
		     NAU8822_AUX2_MIXER, NAU8822_AUX1_MIXER, 6, 1, 1),

	/* DAC / ADC oversampling */
	SOC_SINGLE("DAC 128x Oversampling Switch",
		   NAU8822_DAC_CONTROL, 3, 1, 0),
	SOC_SINGLE("ADC 128x Oversampling Switch",
		   NAU8822_ADC_CONTROL, 3, 1, 0),

	/* ADC to DAC loop back */
	SOC_SINGLE("ADDAP Switch", NAU8822_COMPANDING, 0, 1, 0),

	SOC_ENUM("ADC Companding", adc_compand),
	SOC_ENUM("DAC Companding", dac_compand),

	SOC_ENUM("Micbias Voltage", micbias),

	SOC_DOUBLE("ADC Polarity Switch", NAU8822_ADC_CONTROL, 0, 1, 1, 0),
	SOC_DOUBLE("DAC Polarity Switch", NAU8822_DAC_CONTROL, 0, 1, 1, 0),
};

/* Mixer #1: Output Main Mixers: mix AUX, Input mixer output and DAC */
static const struct snd_kcontrol_new nau8822_left_main_mixer[] = {
	SOC_DAPM_SINGLE("Line Bypass Switch", NAU8822_LEFT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("Auxin Playback Switch", NAU8822_LEFT_MIXER, 5, 1, 0),
	SOC_DAPM_SINGLE("DAC Playback Switch", NAU8822_LEFT_MIXER, 0, 1, 0),
};

static const struct snd_kcontrol_new nau8822_right_main_mixer[] = {
	SOC_DAPM_SINGLE("Line Bypass Switch", NAU8822_RIGHT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("Auxin Playback Switch", NAU8822_RIGHT_MIXER, 5, 1, 0),
	SOC_DAPM_SINGLE("DAC Playback Switch", NAU8822_RIGHT_MIXER, 0, 1, 0),
};

/* Mixer #2: Right Speaker Submixer : mix RAUX and RMIX and invert
   the output */
static const struct snd_kcontrol_new nau8822_right_speaker_submixer[] = {
	SOC_DAPM_SINGLE("RMIX Input Switch",
			NAU8822_RIGHT_SPEAKER_SUBMIXER, 5, 1, 1),
	SOC_DAPM_SINGLE("RAUXIN Input Switch",
			NAU8822_RIGHT_SPEAKER_SUBMIXER, 0, 1, 0),
};

/* Mixer # 3: AUX (AUX1, AUX2) Mixer: */
static const struct snd_kcontrol_new nau8822_aux1_mixer[] = {
	SOC_DAPM_SINGLE("RINMIX Switch", NAU8822_AUX1_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("LDAC Switch", NAU8822_AUX1_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("RDAC Switch", NAU8822_AUX1_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("LMIX Switch", NAU8822_AUX1_MIXER, 4, 1, 0),
	SOC_DAPM_SINGLE("RMIX Switch", NAU8822_AUX1_MIXER, 1, 1, 0),
};
static const struct snd_kcontrol_new nau8822_aux2_mixer[] = {
	SOC_DAPM_SINGLE("LINMIX Switch", NAU8822_AUX2_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("LDAC Switch", NAU8822_AUX2_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("LMIX Switch", NAU8822_AUX2_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("AUX1 Switch", NAU8822_AUX2_MIXER, 3, 1, 0),
};

/* Mixer #4: Input Mixer */
static const struct snd_kcontrol_new nau8822_left_input_mixer[] = {
	SOC_DAPM_SINGLE("LLIN Switch", NAU8822_INPUT_CONTROL, 2, 1, 0),
	SOC_DAPM_SINGLE("MICN Switch", NAU8822_INPUT_CONTROL, 1, 1, 0),
	SOC_DAPM_SINGLE("MICP Switch", NAU8822_INPUT_CONTROL, 0, 1, 0),
};
static const struct snd_kcontrol_new nau8822_right_input_mixer[] = {
	SOC_DAPM_SINGLE("RLIN Switch", NAU8822_INPUT_CONTROL, 6, 1, 0),
	SOC_DAPM_SINGLE("MICN Switch", NAU8822_INPUT_CONTROL, 5, 1, 0),
	SOC_DAPM_SINGLE("MICP Switch", NAU8822_INPUT_CONTROL, 4, 1, 0),
};

static const struct snd_soc_dapm_widget nau8822_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback",
			 NAU8822_POWER_MANAGEMENT_3, 0, 0),
	SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback",
			 NAU8822_POWER_MANAGEMENT_3, 1, 0),
	SND_SOC_DAPM_ADC("Left ADC", "Left HiFi Capture",
			 NAU8822_POWER_MANAGEMENT_2, 0, 0),
	SND_SOC_DAPM_ADC("Right ADC", "Right HiFi Capture",
			 NAU8822_POWER_MANAGEMENT_2, 1, 0),

	SND_SOC_DAPM_MIXER("Left Main Mixer", NAU8822_POWER_MANAGEMENT_3,
			   2, 0, &nau8822_left_main_mixer[0],
			   ARRAY_SIZE(nau8822_left_main_mixer)),
	SND_SOC_DAPM_MIXER("Right Main Mixer", NAU8822_POWER_MANAGEMENT_3,
			   3, 0, &nau8822_right_main_mixer[0],
			   ARRAY_SIZE(nau8822_right_main_mixer)),
	SND_SOC_DAPM_MIXER("AUX1 Mixer", NAU8822_POWER_MANAGEMENT_1,
			   7, 0, &nau8822_aux1_mixer[0],
			   ARRAY_SIZE(nau8822_aux1_mixer)),
	SND_SOC_DAPM_MIXER("AUX2 Mixer", NAU8822_POWER_MANAGEMENT_1,
			   6, 0, &nau8822_aux2_mixer[0],
			   ARRAY_SIZE(nau8822_aux2_mixer)),
	SND_SOC_DAPM_MIXER("Right Speaker Submixer", SND_SOC_NOPM, 0, 0,
			   &nau8822_right_speaker_submixer[0],
			   ARRAY_SIZE(nau8822_right_speaker_submixer)),
	SND_SOC_DAPM_MIXER("Left Input Mixer", NAU8822_POWER_MANAGEMENT_2,
			   2, 0, &nau8822_left_input_mixer[0],
			   ARRAY_SIZE(nau8822_left_input_mixer)),
	SND_SOC_DAPM_MIXER("Right Input Mixer", NAU8822_POWER_MANAGEMENT_2,
			   3, 0, &nau8822_right_input_mixer[0],
			   ARRAY_SIZE(nau8822_right_input_mixer)),

	SND_SOC_DAPM_PGA("Left Boost Mixer", NAU8822_POWER_MANAGEMENT_2,
			4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Boost Mixer", NAU8822_POWER_MANAGEMENT_2,
			 5, 0, NULL, 0),

	SND_SOC_DAPM_PGA("Left Capture PGA", NAU8822_LEFT_INP_PGA_GAIN,
			 6, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right Capture PGA", NAU8822_RIGHT_INP_PGA_GAIN,
			 6, 1, NULL, 0),

	SND_SOC_DAPM_PGA("Left Headphone Out", NAU8822_POWER_MANAGEMENT_2,
			 7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Headphone Out", NAU8822_POWER_MANAGEMENT_2,
			 8, 0, NULL, 0),

	SND_SOC_DAPM_PGA("Left Speaker Out", NAU8822_POWER_MANAGEMENT_3,
			 6, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Speaker Out", NAU8822_POWER_MANAGEMENT_3,
			 5, 0, NULL, 0),

	SND_SOC_DAPM_PGA("Aux Out1", NAU8822_POWER_MANAGEMENT_3,
			 8, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Aux Out2", NAU8822_POWER_MANAGEMENT_3,
			 7, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("Mic Bias", NAU8822_POWER_MANAGEMENT_1, 4, 0,
			    NULL, 0),

	SND_SOC_DAPM_INPUT("LMICN"),
	SND_SOC_DAPM_INPUT("LMICP"),
	SND_SOC_DAPM_INPUT("RMICN"),
	SND_SOC_DAPM_INPUT("RMICP"),
	SND_SOC_DAPM_INPUT("LAUXIN"),
	SND_SOC_DAPM_INPUT("RAUXIN"),
	SND_SOC_DAPM_INPUT("LLIN"),
	SND_SOC_DAPM_INPUT("RLIN"),
	SND_SOC_DAPM_OUTPUT("LHP"),
	SND_SOC_DAPM_OUTPUT("RHP"),
	SND_SOC_DAPM_OUTPUT("LSPK"),
	SND_SOC_DAPM_OUTPUT("RSPK"),
	SND_SOC_DAPM_OUTPUT("AUX1"),
	SND_SOC_DAPM_OUTPUT("AUX2"),
};

static const struct snd_soc_dapm_route nau8822_dapm_routes[] = {

	/* Output side */

	/* Output Main mixers */
	{"Left Main Mixer", "DAC Playback Switch", "Left DAC"},
	{"Left Main Mixer", "Auxin Playback Switch", "LAUXIN"},
	{"Left Main Mixer", "Line Bypass Switch", "Left Boost Mixer"},

	{"Right Main Mixer", "DAC Playback Switch", "Right DAC"},
	{"Right Main Mixer", "Auxin Playback Switch", "RAUXIN"},
	{"Right Main Mixer", "Line Bypass Switch", "Right Boost Mixer"},

	/* Output Auxiliary mixers */
	{"AUX1 Mixer", "RINMIX Switch", "Right Boost Mixer"},
	{"AUX1 Mixer", "LDAC Switch", "Left DAC"},
	{"AUX1 Mixer", "RDAC Switch", "Right DAC"},
	{"AUX1 Mixer", "LMIX Switch", "Left Main Mixer"},
	{"AUX1 Mixer", "RMIX Switch", "Right Main Mixer"},

	{"AUX2 Mixer", "LINMIX Switch", "Left Boost Mixer"},
	{"AUX2 Mixer", "LDAC Switch", "Left DAC"},
	{"AUX2 Mixer", "LMIX Switch", "Left Main Mixer"},
	{"AUX2 Mixer", "AUX1 Switch", "AUX1 Mixer"},

	/* Right Speaker submixer */
	{"Right Speaker Submixer", "RMIX Input Switch", "Right Main Mixer"},
	{"Right Speaker Submixer", "RAUXIN Input Switch", "RAUXIN"},

	/* Head Phone Outputs */
	{"Right Headphone Out", NULL, "Right Main Mixer"},
	{"RHP", NULL, "Right Headphone Out"},

	{"Left Headphone Out", NULL, "Left Main Mixer"},
	{"LHP", NULL, "Left Headphone Out"},

	/* Speaker Outputs */
	{"Left Speaker Out", NULL, "Left Main Mixer"},
	{"LSPK", NULL, "Left Speaker Out"},

	{"Right Speaker Out", NULL, "Right Speaker Submixer"},
	{"RSPK", NULL, "Right Speaker Out"},

	/* Auxiliary Outputs */
	{"Aux Out1", NULL, "AUX1 Mixer"},
	{"AUX1", NULL, "Aux Out1"},

	{"Aux Out2", NULL, "AUX2 Mixer"},
	{"AUX2", NULL, "Aux Out2"},

	/* Input side */

	/* Auxiliary inputs */
	{"Left Boost Mixer", NULL, "LAUXIN"},
	{"Right Boost Mixer", NULL, "RAUXIN"},

	/* Line inputs */
	{"Left Boost Mixer", NULL, "LLIN"},
	{"Right Boost Mixer", NULL, "RLIN"},

	/* MIC inputs and Mixers */
	{"Left Input Mixer", "LLIN Switch", "LLIN"},
	{"Left Input Mixer", "MICN Switch", "LMICN"},
	{"Left Input Mixer", "MICP Switch", "LMICP"},

	{"Right Input Mixer", "RLIN Switch", "RLIN"},
	{"Right Input Mixer", "MICN Switch", "RMICN"},
	{"Right Input Mixer", "MICP Switch", "RMICP"},

	/* PGAs */
	{"Left Capture PGA", NULL, "Left Input Mixer"},
	{"Right Capture PGA", NULL, "Right Input Mixer"},

	/* Mixer Boost */
	{"Left Boost Mixer", NULL, "Left Capture PGA"},
	{"Right Boost Mixer", NULL, "Right Capture PGA"},

	/* ADCs */
	{"Left ADC", NULL, "Left Boost Mixer"},
	{"Right ADC", NULL, "Right Boost Mixer"},
};

/* MCLK divisors */
static const int mclk_numerator[]	= {1, 3, 2, 3, 4, 6, 8, 12};
static const int mclk_denominator[]	= {1, 2, 1, 1, 1, 1, 1, 1};

static int nau8822_configure_pll(struct snd_soc_dai *codec_dai,
				 unsigned int fout)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	u16 pll_k1, pll_k2, pll_k3;
	u8 pll_n, cnt;
	int fref = nau8822->f_mclk;

	/* Search pll div array for correct divisors */
	for (cnt = 0; cnt < ARRAY_SIZE(nau8822_pll_div); cnt++) {
		/* check fref and fout */
		if ((fref == nau8822_pll_div[cnt].fref) &&
		    (fout == nau8822_pll_div[cnt].fout)) {
			/* all match, pick up divisors */
			pll_k1 = nau8822_pll_div[cnt].K1;
			pll_k2 = nau8822_pll_div[cnt].K2;
			pll_k3 = nau8822_pll_div[cnt].K3;
			pll_n = nau8822_pll_div[cnt].N;
			break;
		}
	}
	if (cnt >= ARRAY_SIZE(nau8822_pll_div))
		goto err;

	/* Turn OFF PLL before configuration... */
	snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1, 0x20, 0);

	snd_soc_update_bits(codec, NAU8822_PLL_N, 0x0F, pll_n);
	snd_soc_update_bits(codec, NAU8822_PLL_K1, 0x3F, pll_k1);
	snd_soc_write(codec, NAU8822_PLL_K2, pll_k2);
	snd_soc_write(codec, NAU8822_PLL_K3, pll_k3);

	/* Configuration done, Turn ON PLL */
	snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1, 0x20, 0x20);

	/* Route output from PLL (fPLL) to GPIO1 */
	snd_soc_update_bits(codec, NAU8822_GPIO, 7, 4);

	return 0;
err:
	dev_err(codec_dai->dev, "Can't find suitable PLL divisors\n");
	dev_err(codec_dai->dev, "fref = %d, fout = %d\n", fref, fout);
	return -EINVAL;
}

static int nau8822_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
				  unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);

	switch (clk_id) {
	case NAU8822_MCLK:
		/* Codec imclk directly derived from MCLK, PLL not needed */
		snd_soc_update_bits(codec, NAU8822_CLOCK_CONTROL_1, 0x100, 0);

		/* Turn OFF PLL */
		snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1, 0x20, 0);
		nau8822->sysclk = NAU8822_MCLK;
		nau8822->f_mclk = freq;
		return 0;

	case NAU8822_PLL:
		/* Codec imclk derived from PLL */
		snd_soc_update_bits(codec, NAU8822_CLOCK_CONTROL_1, 0x100,
				    0x100);
		nau8822->sysclk = NAU8822_PLL;

		switch (freq) {
		case 12000000:
		case 14400000:
		case 19200000:
		case 19800000:
		case 24000000:
		case 26000000:
			nau8822->f_mclk = freq;
			return 0;
		default:
			dev_err(codec_dai->dev,
				"Unsupported PLL input frequency %d\n", freq);
			return -EINVAL;
		}
	default:
		dev_err(codec_dai->dev, "Unknown clock source %d\n", clk_id);
		return -EINVAL;
	}
}

static int nau8822_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
				 int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	switch (div_id) {
	case NAU8822_IMCLKRATE:
		nau8822->f_imclk = div;

		if (nau8822->sysclk == NAU8822_PLL) {
			/* Codec imclk derived from PLL */
			ret = nau8822_configure_pll(codec_dai, div);
			if (ret) {
				dev_err(codec_dai->dev,
					"Can't generate required fout %d\n",
					div);
				return -EINVAL;
			}
		}
		break;
	case NAU8822_BCLKDIV:
		/* This needs to be done only if we are in MASTER mode */
		if (div & ~0x1c)
			return -EINVAL;
		snd_soc_update_bits(codec, NAU8822_CLOCK_CONTROL_1, 0x1c, div);
		/*
		snd_soc_update_bits(codec, NAU8822_CLOCK_CONTROL_1, 0x1c,0x0c);
		*/
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(codec->dev, "%s: ID %d, value %u\n", __func__, div_id, div);

	return ret;
}

static int nau8822_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	/*
	 * BCLK polarity mask = 0x100, LRC clock polarity mask = 0x80,
	 * Data Format mask = 0x18
	 */
	u16 iface = snd_soc_read(codec, NAU8822_AUDIO_INTERFACE) & ~0x198;
	u16 clk = snd_soc_read(codec, NAU8822_CLOCK_CONTROL_1);

	dev_dbg(codec->dev, "%s\n", __func__);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		clk |= 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		clk &= ~1;
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x10;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x8;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x18;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x180;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x100;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x80;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, NAU8822_AUDIO_INTERFACE, iface);
	snd_soc_write(codec, NAU8822_CLOCK_CONTROL_1, clk);

	return 0;
}

static int nau8822_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	/* Word length mask = 0x60 */
	u16 iface_ctl = snd_soc_read(codec, NAU8822_AUDIO_INTERFACE) & ~0x60;
	/* Sampling rate mask = 0xe */
	u16 add_ctl = snd_soc_read(codec, NAU8822_CLOCK_CONTROL_2) & ~0xe;
	u16 clking = snd_soc_read(codec, NAU8822_CLOCK_CONTROL_1);
	u32 f_256fs = 0;

	unsigned int f_sel, diff, diff_best = INT_MAX;
	int i, best = 0;

	if (!nau8822->f_mclk)
		return -EINVAL;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface_ctl |= 0x20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface_ctl |= 0x40;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface_ctl |= 0x60;
		break;
	}

	/* sampling rate */

	/*
	 * The logic to find out value of MCLK prescaler (best) is as below
	 *
	 *    IMCLK      Divisor	  FS
	 * (1228800*2  /   12)	=	(8000	* 256)
	 * (1228800*2  /   6)	=	(16000	* 256)
	 * (1228800*2  /   3)	=	(32000	* 256)
	 * (1228800*2  /   2)	=	(48000	* 256)
	 *
	 * (1128960*2  /   8)	=	(11025	* 256)
	 * (1128960*2  /   4)	=	(22050	* 256)
	 * (1128960*2  /   2)	=	(44100	* 256)
	 */
	switch (params_rate(params)) {
	case 8000:
		add_ctl |= 0x5 << 1;
		best = 0x7;
		break;
	case 11025:
		add_ctl |= 0x4 << 1;
		best = 0x6;
		break;
	case 16000:
		add_ctl |= 0x3 << 1;
		best = 0x5;
		break;
	case 22050:
		add_ctl |= 0x2 << 1;
		best = 0x4;
		break;
	case 32000:
		add_ctl |= 0x1 << 1;
		best = 0x3;
		break;
	case 44100:
		best = 0x2;
		break;
	case 48000:
		best = 0x2;
		break;
	}

	snd_soc_write(codec, NAU8822_AUDIO_INTERFACE, iface_ctl);
	snd_soc_write(codec, NAU8822_CLOCK_CONTROL_2, add_ctl);

	if (nau8822->sysclk == NAU8822_MCLK) {
		/* Non PLL Mode - MCLK is used directly */

		/* Sampling rate is known, find out required iMCLK */
		f_256fs = params_rate(params) * 256;
		f_sel = nau8822->f_mclk;

		if (f_sel < f_256fs || f_sel > 12 * f_256fs)
			return -EINVAL;

		for (i = 0; i < ARRAY_SIZE(mclk_numerator); i++) {
			diff = abs((f_256fs * 3) - (f_sel * 3 *
			(mclk_denominator[i] / mclk_numerator[i])));

			if (diff < diff_best) {
				diff_best = diff;
				best = i;
			}

			if (!diff)
				break;
		}

		if (diff) {
			dev_warn(codec->dev, "Imprecise sampling rate: %uHz%s\n",
				f_sel * mclk_denominator[best] /
				mclk_numerator[best] / 256,
				nau8822->sysclk == NAU8822_MCLK ?
				", consider using PLL" : "");
		}
	} else {
		/* PLL Mode - MCLK is used through PLL */
		/* Check if we are in MASTER mode */
		if (!(clking & 0x1)) {
			/* We are in I2S SLAVE mode */
			/* MCLK divisor must be set to 1 */
			best = 0;
		}
		/* In I2S MASTER mode, use divisor value (best) as per fs */
	}
	dev_dbg(codec->dev, "%s: fmt %d, rate %u, MCLK divisor #%d\n",
		__func__, params_format(params), params_rate(params), best);

	/* MCLK divisor mask = 0xe0 */
	snd_soc_update_bits(codec, NAU8822_CLOCK_CONTROL_1, 0xe0, best << 5);

	return 0;
}

static int nau8822_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev, "%s: %d\n", __func__, mute);

	if (mute)
		snd_soc_update_bits(codec, NAU8822_DAC_CONTROL, 0x40, 0x40);
	else
		snd_soc_update_bits(codec, NAU8822_DAC_CONTROL, 0x40, 0);

	return 0;
}

static int nau8822_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u16 power1 = snd_soc_read(codec, NAU8822_POWER_MANAGEMENT_1) & ~3;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		power1 |= 1;  /* VREF 80k */
		snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_1, power1);
		break;
	case SND_SOC_BIAS_STANDBY:
		/* bit 3: enable bias, bit 2: enable I/O tie off buffer */
		power1 |= 0xc;

		if (codec->bias_level == SND_SOC_BIAS_OFF) {
			/* Initial cap charge at VREF 3k */
			snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_1,
				      power1 | 0x3);
			mdelay(100);
		}

		power1 |= 0x2;  /* VREF 300k */
		snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_1, power1);
		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1,
				    ~0x20, 0);
		snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_2, 0);
		snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_3, 0);
		break;
	}

	dev_dbg(codec->dev, "%s: %d, %x\n", __func__, level, power1);

	codec->bias_level = level;
	return 0;
}

#define NAU8822_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops nau8822_dai_ops = {
	.hw_params	= nau8822_hw_params,
	.digital_mute	= nau8822_mute,
	.set_fmt	= nau8822_set_dai_fmt,
	.set_sysclk	= nau8822_set_dai_sysclk,
	.set_clkdiv	= nau8822_set_dai_clkdiv,
};

/* Also supports 12kHz */
struct snd_soc_dai nau8822_dai = {
	.name = "nau8822-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = NAU8822_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = NAU8822_FORMATS,
	},
	.ops = &nau8822_dai_ops,
};
EXPORT_SYMBOL_GPL(nau8822_dai);

static struct snd_soc_codec *nau8822_codec = NULL;

#if 0
static int nau8822_suspend(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = nau8822_codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);

	nau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);
	if (nau8822->sysclk == NAU8822_PLL) {
		/* Codec imclk derived from PLL, so PLL is ON. Switch it OFF */
		snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_1, 0);
	}

	return 0;
}

static int nau8822_resume(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = nau8822_codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);

	nau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	if (nau8822->sysclk == NAU8822_PLL) {
		/* Codec imclk was derived from PLL, so switch ON PLL */
		snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1,
				    0x20, 0x20);
	}

	return 0;
}
#endif

static int nau8822_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = nau8822_codec;
	struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
	int ret = 0, i;

	socdev->card->codec = codec;

	/* Register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register new PCMs\n");
		goto pcm_err;
	}

	snd_soc_add_controls(codec, nau8822_snd_controls,
				ARRAY_SIZE(nau8822_snd_controls));
	snd_soc_dapm_new_controls(codec, nau8822_dapm_widgets,
				  ARRAY_SIZE(nau8822_dapm_widgets));

	snd_soc_dapm_add_routes(codec, nau8822_dapm_routes, ARRAY_SIZE(nau8822_dapm_routes));

	/* Set default system clock to PLL */
	nau8822->sysclk = NAU8822_PLL;
	nau8822->f_mclk = 0;    /* This will be set in set_sysclk() */
	nau8822->f_imclk = 0;	/* This will be set in set_clkdiv() */

	/* Set the update bit in all registers, that have it. */
	for (i = 0; i < ARRAY_SIZE(update_reg); i++)
		snd_soc_update_bits(codec, update_reg[i], 0x100, 0x100);

	nau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
pcm_err:
	return ret;
}

/* power down chip */
static int nau8822_remove(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = nau8822_codec;
	nau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_nau8822 = {
	.probe = nau8822_probe,
	.remove = nau8822_remove,
#if 0
	.suspend = nau8822_suspend,
	.resume = nau8822_resume,
#endif
};
EXPORT_SYMBOL_GPL(soc_codec_dev_nau8822);

int (*original_codec_write)(struct snd_soc_codec *codec,
				unsigned int reg, unsigned int val);

int nau8822_codec_write(struct snd_soc_codec *codec,
				unsigned int reg, unsigned int val)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(update_reg); ++i) {
		if (reg == update_reg[i]) {
			val |= 0x100;
			break;
		}
	}

	return original_codec_write(codec, reg, val);
}

static __devinit int nau8822_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct nau8822_priv *nau8822;
	struct snd_soc_codec *codec;
	int ret;

	nau8822 = devm_kzalloc(&i2c->dev, sizeof(struct nau8822_priv),
			      GFP_KERNEL);
	if (nau8822 == NULL)
		return -ENOMEM;

	codec = &nau8822->codec;
	nau8822_codec = codec;

	i2c_set_clientdata(i2c, nau8822);

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->dev = &i2c->dev;
	codec->name = "NAU8822";
	codec->owner = THIS_MODULE;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = nau8822_set_bias_level;
	codec->dai = &nau8822_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = NAU8822_MAX_REGISTER + 1;
	codec->reg_cache = &nau8822_reg_defaults;
	codec->private_data = nau8822;
	codec->control_data = i2c;

	snd_soc_codec_set_cache_io(codec, 7, 9, SND_SOC_I2C);

	/* Set update bits to necessary registers */
	original_codec_write = codec->write;
	codec->write = nau8822_codec_write;


	ret = snd_soc_register_dai(&nau8822_dai);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to register CODEC DAI: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to register CODEC: %d\n", ret);
		goto err;
	}

	snd_soc_write(codec, NAU8822_RESET, 0);

	return 0;

err:
	kfree(nau8822);
	return ret;
}

static __devexit int nau8822_i2c_remove(struct i2c_client *client)
{
	struct nau8822_priv *nau8822 = i2c_get_clientdata(client);

	snd_soc_unregister_dai(&nau8822_dai);
	snd_soc_unregister_codec(&nau8822->codec);

	kfree(nau8822);
	return 0;
}

static const struct i2c_device_id nau8822_i2c_id[] = {
	{ "nau8822", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nau8822_i2c_id);

static struct i2c_driver nau8822_i2c_driver = {
	.driver = {
		.name = "nau8822-codec",
		.owner = THIS_MODULE,
	},
	.probe = nau8822_i2c_probe,
	.remove = __devexit_p(nau8822_i2c_remove),
	.id_table = nau8822_i2c_id,
};

static int __init nau8822_modinit(void)
{
	int ret = 0;
	ret = i2c_add_driver(&nau8822_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register NAU8822 I2C driver: %d\n",
		       ret);
	}
	return ret;
}
module_init(nau8822_modinit);

static void __exit nau8822_exit(void)
{
	i2c_del_driver(&nau8822_i2c_driver);
}
module_exit(nau8822_exit);

MODULE_DESCRIPTION("ASoC NAU8822 codec driver");
MODULE_AUTHOR("Ashish Chavan");
MODULE_LICENSE("GPL");
