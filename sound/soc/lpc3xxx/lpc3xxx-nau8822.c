/*
 * sound/soc/lpc3xxx/lpc3xxx-nau8822.c
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

#include <mach/iomux.h>
#include <mach/clock.h>
#include <mach/lpc18xx.h>

#include "../codecs/nau8822.h"
#include "lpc3xxx-pcm.h"
#include "lpc3xxx-i2s.h"

#define SND_MODNAME "lpc3xxx_nau8822"

static int lpc4357_nau8822_hw_params(struct snd_pcm_substream *substream,
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
	ret = snd_soc_dai_set_sysclk(cpu_dai,
			LPC3XXX_I2S_CLK_BASE_AUDIO_CLK,
			256 * params_rate(params),
			SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		pr_warning("%s: "
			   "Failed to set I2S clock (%d)\n",
			   SND_MODNAME, ret);
		return ret;
	}

	/* Set BCLK divider */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 0, params_channels(params) == 1 ? 15 : 7);
	if (ret < 0) {
		pr_warning("%s: "
			   "Failed to set I2S clock (%d)\n",
			   SND_MODNAME, ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, NAU8822_MCLK,
					256 * params_rate(params),
					SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		pr_warning("%s: "
			   "Failed to set CODEC I2S clock (%d)\n",
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

int lpc4357_nau8822_startup(struct snd_pcm_substream *substream)
{
//	lpc18xx_gpio_out(0x7, 2, 1);

	return 0;
}

int lpc4357_nau8822_shutdown(struct snd_pcm_substream *substream)
{
//	lpc18xx_gpio_out(0x7, 2, 0);

	return 0;
}

static struct snd_soc_ops lpc4357_nau8822_ops = {
	.hw_params = lpc4357_nau8822_hw_params,
	.startup = lpc4357_nau8822_startup,
	.shutdown = lpc4357_nau8822_shutdown,
};

static struct snd_soc_dai_link lpc4357_nau8822_dai[] = {
{
	.name = "NAU8822",
	.stream_name = "NAU8822",
	.cpu_dai = &lpc3xxx_i2s_dai[0],
	.codec_dai = &nau8822_dai,
	.ops = &lpc4357_nau8822_ops,
},
};

static struct snd_soc_card lpc4357_snd_soc_card = {
        .name = "LPC4357",
	.platform = &lpc3xxx_soc_platform,
	.dai_link = &lpc4357_nau8822_dai[0],
	.num_links = ARRAY_SIZE(lpc4357_nau8822_dai),
};

static struct snd_soc_device lpc4357_nau8822_snd_devdata = {
	.card = &lpc4357_snd_soc_card,
	.codec_dev = &soc_codec_dev_nau8822,
};

static struct platform_device *lpc4357_snd_device;
static int __init lpc4357_asoc_init(void)
{
	int ret = 0;

	/*
	 * Create and register platform device
	 */
	lpc4357_snd_device = platform_device_alloc("soc-audio", -1);
	if (lpc4357_snd_device == NULL) {
		return -ENOMEM;
	}

	lpc18xx_gpio_out(0x7, 2, 1);

	/* Set BASE_AUDIO_CLOCK as MCLK and clocking source for I2S */
	LPC18XX_CREG->creg6 |= LPC18XX_CREG_CREG6_I2S0_TX_SCK_BASE_AUDIO_CLOCK;

	platform_set_drvdata(lpc4357_snd_device, &lpc4357_nau8822_snd_devdata);
	lpc4357_nau8822_snd_devdata.dev = &lpc4357_snd_device->dev;

	ret = platform_device_add(lpc4357_snd_device);
	if (ret) {
		pr_warning("%s: platform_device_add failed (%d)\n",
			   SND_MODNAME, ret);
		goto err_device_add;
	}

	return 0;

err_device_add:
	if (lpc4357_snd_device != NULL) {
		platform_device_put(lpc4357_snd_device);
		lpc4357_snd_device = NULL;
	}

	return ret;
}

static void __exit lpc4357_asoc_exit(void)
{
	platform_device_unregister(lpc4357_snd_device);
	lpc4357_snd_device = NULL;
}

module_init(lpc4357_asoc_init);
module_exit(lpc4357_asoc_exit);

MODULE_AUTHOR("Pavel Boldin <paboldin@emcraft.com>");
MODULE_DESCRIPTION("ASoC machine driver for LPC4357/NAU8822");
MODULE_LICENSE("GPL");
