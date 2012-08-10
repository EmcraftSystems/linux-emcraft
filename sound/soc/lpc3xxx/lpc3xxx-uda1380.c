/*
 * sound/soc/lpc3xxx/lpc3xxx-uda1380.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/uda1380.h>

#include <mach/gpio.h>

#include "../codecs/uda1380.h"
#include "lpc3xxx-pcm.h"
#include "lpc3xxx-i2s.h"

#define SND_MODNAME "lpc3xxx_uda1380"

static int phy3250_uda1380_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int fmt = SND_SOC_DAIFMT_I2S;
	int ret;

	/* The perspective is from the CODEC side, so slave mode means that
	    the i2s interface is the master. For the UDA1380 and playback,
	    the CODEC is always a slave and i2s is always a master. */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		fmt |= SND_SOC_DAIFMT_CBS_CFS;
	}
	else {

#if defined(CONFIG_SND_LPC32XX_USEI2S_SLAVE_MODERX)
		fmt |= SND_SOC_DAIFMT_CBM_CFM;
#else
		fmt |= SND_SOC_DAIFMT_CBS_CFS;
#endif
	}

	/* Set the CPU I2S rate clock (first) */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, params_rate(params),
					    SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		pr_warning("%s: "
			   "Failed to set I2S clock (%d)\n",
			   SND_MODNAME, ret);
		return ret;
	}

	/* Set CPU and CODEC DAI format */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		pr_warning("%s: "
			   "Failed to set CPU DAI format (%d)\n",
			   SND_MODNAME, ret);
		return ret;
	}
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_warning("%s: "
			   "Failed to set CODEC DAI format (%d)\n",
			   SND_MODNAME, ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops phy3250_uda1380_ops = {
	.hw_params = phy3250_uda1380_hw_params,
};

static const struct snd_soc_dapm_widget phy3250_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_route intercon[] = {
	/* Headphone connected to VOUTRHP, VOUTLHP */
	{"Headphone Jack", NULL, "VOUTRHP"},
	{"Headphone Jack", NULL, "VOUTLHP"},

	/* Line Out connected to VOUTR, VOUTL */
	{"Line Out", NULL, "VOUTR"},
	{"Line Out", NULL, "VOUTL"},

	/* Mic connected to VINM */
	{"VINM", NULL, "Mic Jack"},

	/* Line In connected to VINR, VINL */
	{"VINL", NULL, "Line In"},
	{"VINR", NULL, "Line In"},
};

static int phy3250_uda1380_init(struct snd_soc_codec *codec)
{
	/* Add widgets */
	snd_soc_dapm_new_controls(codec, phy3250_dapm_widgets,
				  ARRAY_SIZE(phy3250_dapm_widgets));

	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	/* Always connected pins */
	snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Line Out");
	snd_soc_dapm_enable_pin(codec, "Mic Jack");
	snd_soc_dapm_enable_pin(codec, "Line In");

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link phy3250_uda1380_dai[] = {
{
	.name = "UDA1380",
	.stream_name = "UDA1380",
#if defined(CONFIG_SND_LPC32XX_USEI2S1)
	.cpu_dai = &lpc3xxx_i2s_dai[1],
#else
	.cpu_dai = &lpc3xxx_i2s_dai[0],
#endif
	.codec_dai = &uda1380_dai[UDA1380_DAI_DUPLEX],
	.init = phy3250_uda1380_init,
	.ops = &phy3250_uda1380_ops,
},
};

static struct snd_soc_card phy3250_snd_soc_card = {
        .name = "LPC32XX",
	.platform = &lpc3xxx_soc_platform,
	.dai_link = &phy3250_uda1380_dai[0],
	.num_links = ARRAY_SIZE(phy3250_uda1380_dai),
};

static struct snd_soc_device phy3250_uda1380_snd_devdata = {
	.card = &phy3250_snd_soc_card,
	.codec_dev = &soc_codec_dev_uda1380,
};

static struct platform_device *phy3250_snd_device;
static int __init phy3250_asoc_init(void)
{
	int ret = 0;

	/*
	 * Create and register platform device
	 */
	phy3250_snd_device = platform_device_alloc("soc-audio", 0);
	if (phy3250_snd_device == NULL) {
		return -ENOMEM;
	}

	platform_set_drvdata(phy3250_snd_device, &phy3250_uda1380_snd_devdata);
	phy3250_uda1380_snd_devdata.dev = &phy3250_snd_device->dev;

	ret = platform_device_add(phy3250_snd_device);
	if (ret) {
		pr_warning("%s: platform_device_add failed (%d)\n",
			   SND_MODNAME, ret);
		goto err_device_add;
	}

	return 0;

err_device_add:
	if (phy3250_snd_device != NULL) {
		platform_device_put(phy3250_snd_device);
		phy3250_snd_device = NULL;
	}

	return ret;
}

static void __exit phy3250_asoc_exit(void)
{
	platform_device_unregister(phy3250_snd_device);
	phy3250_snd_device = NULL;
}

module_init(phy3250_asoc_init);
module_exit(phy3250_asoc_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("ASoC machine driver for LPC3XXX/UDA1380");
MODULE_LICENSE("GPL");
