/*
 * sound/soc/lpc3xxx/lpc3xxx-pcm.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
 *
 * Add support for NXP LPC178x/7x
 * Copyright (c) 2012
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/dma.h>
#include <mach/dmac.h>
#include <mach/i2s.h>
#include "lpc3xxx-pcm.h"

#define SND_NAME "lpc3xxx-audio"
static u64 lpc3xxx_pcm_dmamask = 0xffffffff;

#define NUMLINKS (3) /* 3 DMA buffers */

static const struct snd_pcm_hardware lpc3xxx_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_PAUSE |
		 SNDRV_PCM_INFO_RESUME),
	.formats = (SND_SOC_DAIFMT_I2S),
	.period_bytes_min = 128,
	.period_bytes_max = 2048,
	.periods_min = 2,
	.periods_max = 1024,
	.buffer_bytes_max = 128 * 1024
};

struct lpc3xxx_dma_data {
	dma_addr_t dma_buffer;	/* physical address of DMA buffer */
	dma_addr_t dma_buffer_end; /* first address beyond DMA buffer */
	size_t period_size;

	/* DMA configuration and support */
	int dmach;
	struct dma_config dmacfg;
	volatile dma_addr_t period_ptr;	/* physical address of next period */
	volatile dma_addr_t dma_cur;
	u32 llptr;		/* Saved for debug only, not used */
};

static int lpc3xxx_pcm_allocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *dmabuf = &substream->dma_buffer;
	size_t size = lpc3xxx_pcm_hardware.buffer_bytes_max;

	dmabuf->dev.type = SNDRV_DMA_TYPE_DEV;
	dmabuf->dev.dev = pcm->card->dev;
	dmabuf->private_data = NULL;
	dmabuf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &dmabuf->addr, GFP_KERNEL);

	if (!dmabuf->area)
		return -ENOMEM;

	dmabuf->bytes = size;
	return 0;
}

/*
 * DMA ISR - occurs when a new DMA buffer is needed
 */
static void lpc3xxx_pcm_dma_irq(int channel, int cause,
				struct snd_pcm_substream *substream) {
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct lpc3xxx_dma_data *prtd = rtd->private_data;
	static int count = 0;

	count++;

	/* A DMA interrupt occurred - for most cases, this will be the end
	   of a transmitted buffer in the DMA link list, but errors are also
	   handled. */
	if (cause & DMA_ERR_INT) {
		/* DMA error - this should never happen, but you just never
		   know. If it does happen, the driver will continue without
		   any problems except for maybe an audio glitch or pop. */
		pr_debug("%s: DMA error %s (count=%d)\n", SND_NAME,
			   substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			   "underrun" : "overrun", count);
	}
	/* Dequeue buffer from linked list */
	lpc32xx_get_free_llist_entry(channel);
	prtd->dma_cur += prtd->period_size;
	if (prtd->dma_cur >= prtd->dma_buffer_end) {
		prtd->dma_cur = prtd->dma_buffer;
	}

	/* Re-queue buffer another buffer */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		lpc32xx_dma_queue_llist_entry(prtd->dmach, (void *) prtd->period_ptr,
#if defined(CONFIG_SND_LPC32XX_USEI2S1)
			(void *) I2S_TX_FIFO(LPC32XX_I2S1_BASE),
#else
			(void *) I2S_TX_FIFO(LPC32XX_I2S0_BASE),
#endif
			prtd->period_size);
	}
	else {
		lpc32xx_dma_queue_llist_entry(prtd->dmach,
#if defined(CONFIG_SND_LPC32XX_USEI2S1)
			(void *) I2S_RX_FIFO(LPC32XX_I2S1_BASE),
#else
			(void *) I2S_RX_FIFO(LPC32XX_I2S0_BASE),
#endif
			(void *) prtd->period_ptr, prtd->period_size);
	}
	prtd->period_ptr += prtd->period_size;
	if (prtd->period_ptr >= prtd->dma_buffer_end)
		prtd->period_ptr = prtd->dma_buffer;

	/* This only needs to be called once, even if more than 1 period has passed */
	snd_pcm_period_elapsed(substream);
}

/*
 * PCM operations
 */
static int lpc3xxx_pcm_hw_params(struct snd_pcm_substream *substream,
			         struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpc3xxx_dma_data *prtd = runtime->private_data;

	/* this may get called several times by oss emulation
	 * with different params
	 */
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->dma_buffer = runtime->dma_addr;
	prtd->dma_buffer_end = runtime->dma_addr + runtime->dma_bytes;
	prtd->period_size = params_period_bytes(params);

	return 0;
}

static int lpc3xxx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct lpc3xxx_dma_data *prtd = substream->runtime->private_data;

	/* Return the DMA channel */
	if (prtd->dmach != -1) {
		lpc32xx_dma_ch_disable(prtd->dmach);
		lpc32xx_dma_dealloc_llist(prtd->dmach);
		lpc32xx_dma_ch_put(prtd->dmach);
		prtd->dmach = -1;
	}

	return 0;
}

static int lpc3xxx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct lpc3xxx_dma_data *prtd = substream->runtime->private_data;

	/* Setup DMA channel */
	if (prtd->dmach == -1) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			prtd->dmach = DMA_CH_I2S_TX;
			prtd->dmacfg.ch = DMA_CH_I2S_TX;
			prtd->dmacfg.tc_inten = 1;
			prtd->dmacfg.err_inten = 1;
			prtd->dmacfg.src_size = 4;
			prtd->dmacfg.src_inc = 1;
#if defined(CONFIG_ARCH_LPC32XX) || defined(CONFIG_ARCH_LPC18XX)
			prtd->dmacfg.src_ahb1 = 0;
#endif
			prtd->dmacfg.src_bsize = DMAC_CHAN_SRC_BURST_4;
			prtd->dmacfg.src_prph = 0;
			prtd->dmacfg.dst_size = 4;
			prtd->dmacfg.dst_inc = 0;
			prtd->dmacfg.dst_bsize = DMAC_CHAN_DEST_BURST_4;
#if defined(CONFIG_ARCH_LPC32XX) || defined(CONFIG_ARCH_LPC18XX)
			prtd->dmacfg.dst_ahb1 = 1;
#endif

#if defined(CONFIG_SND_LPC32XX_USEI2S1)
			prtd->dmacfg.dst_prph = DMAC_DEST_PERIP(DMA_PERID_I2S1_DMA1);
#else
			prtd->dmacfg.dst_prph = DMAC_DEST_PERIP(DMA_PERID_I2S0_DMA1);
#endif
			prtd->dmacfg.flowctrl = DMAC_CHAN_FLOW_D_M2P;
			if (lpc32xx_dma_ch_get(&prtd->dmacfg, "dma_i2s_tx",
				&lpc3xxx_pcm_dma_irq, substream) < 0) {
				pr_debug(KERN_ERR "Error setting up I2S TX DMA channel\n");
				return -ENODEV;
			}

			/* Allocate a linked list for audio buffers */
			prtd->llptr = lpc32xx_dma_alloc_llist(prtd->dmach, NUMLINKS);
			if (prtd->llptr == 0) {
				lpc32xx_dma_ch_put(prtd->dmach);
				prtd->dmach = -1;
				pr_debug(KERN_ERR "Error allocating list buffer (I2S TX)\n");
				return -ENOMEM;
			}
		}
		else {
			prtd->dmach = DMA_CH_I2S_RX;
			prtd->dmacfg.ch = DMA_CH_I2S_RX;
			prtd->dmacfg.tc_inten = 1;
			prtd->dmacfg.err_inten = 1;
			prtd->dmacfg.src_size = 4;
			prtd->dmacfg.src_inc = 0;
#if defined(CONFIG_ARCH_LPC32XX) || defined(CONFIG_ARCH_LPC18XX)
			prtd->dmacfg.src_ahb1 = 1;
#endif
			prtd->dmacfg.src_bsize = DMAC_CHAN_SRC_BURST_4;
#if defined(CONFIG_SND_LPC32XX_USEI2S1)
			prtd->dmacfg.src_prph = DMAC_SRC_PERIP(DMA_PERID_I2S1_DMA0);
#else
			prtd->dmacfg.src_prph = DMAC_SRC_PERIP(DMA_PERID_I2S0_DMA0);
#endif
			prtd->dmacfg.dst_size = 4;
			prtd->dmacfg.dst_inc = 1;
#if defined(CONFIG_ARCH_LPC32XX) || defined(CONFIG_ARCH_LPC18XX)
			prtd->dmacfg.dst_ahb1 = 0;
#endif
			prtd->dmacfg.dst_bsize = DMAC_CHAN_DEST_BURST_4;
			prtd->dmacfg.dst_prph = 0;
			prtd->dmacfg.flowctrl = DMAC_CHAN_FLOW_D_P2M;
			if (lpc32xx_dma_ch_get(&prtd->dmacfg, "dma_i2s_rx",
				&lpc3xxx_pcm_dma_irq, substream) < 0) {
				pr_debug(KERN_ERR "Error setting up I2S RX DMA channel\n");
				return -ENODEV;
			}

			/* Allocate a linked list for audio buffers */
			prtd->llptr = lpc32xx_dma_alloc_llist(prtd->dmach, NUMLINKS);
			if (prtd->llptr == 0) {
				lpc32xx_dma_ch_put(prtd->dmach);
				prtd->dmach = -1;
				pr_debug(KERN_ERR "Error allocating list buffer (I2S RX)\n");
				return -ENOMEM;
			}
		}
	}

	return 0;
}

static int lpc3xxx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct lpc3xxx_dma_data *prtd = rtd->private_data;
	int i, ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		prtd->period_ptr = prtd->dma_cur = prtd->dma_buffer;
		lpc32xx_dma_flush_llist(prtd->dmach);

		/* Queue a few buffers to start DMA */
		for (i = 0; i < NUMLINKS; i++) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				lpc32xx_dma_queue_llist_entry(prtd->dmach, (void *) prtd->period_ptr,
#if defined(CONFIG_SND_LPC32XX_USEI2S1)
					(void *) I2S_TX_FIFO(LPC32XX_I2S1_BASE),
#else
					(void *) I2S_TX_FIFO(LPC32XX_I2S0_BASE),
#endif
					prtd->period_size);
			}
			else {
				lpc32xx_dma_queue_llist_entry(prtd->dmach,
#if defined(CONFIG_SND_LPC32XX_USEI2S1)
				(void *) I2S_RX_FIFO(LPC32XX_I2S1_BASE),
#else
				(void *) I2S_RX_FIFO(LPC32XX_I2S0_BASE),
#endif
				(void *) prtd->period_ptr, prtd->period_size);

			}

			prtd->period_ptr += prtd->period_size;
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		lpc32xx_dma_flush_llist(prtd->dmach);
		lpc32xx_dma_ch_disable(prtd->dmach);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		lpc32xx_dma_ch_pause_unpause(prtd->dmach, 1);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		lpc32xx_dma_ch_pause_unpause(prtd->dmach, 0);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t lpc3xxx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpc3xxx_dma_data *prtd = runtime->private_data;
	snd_pcm_uframes_t x;

	/* Return an offset into the DMA buffer for the next data */
	x = bytes_to_frames(runtime, (prtd->dma_cur - runtime->dma_addr));
	if (x >= runtime->buffer_size)
		x = 0;

	return x;
}

static int lpc3xxx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpc3xxx_dma_data *prtd;
	int ret = 0;

	snd_soc_set_runtime_hwparams(substream, &lpc3xxx_pcm_hardware);

	/* ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	runtime->private_data = prtd;
	prtd->dmach = -1;

out:
	return ret;
}

static int lpc3xxx_pcm_close(struct snd_pcm_substream *substream)
{
	struct lpc3xxx_dma_data *prtd = substream->runtime->private_data;

	kfree(prtd);
	return 0;
}

static int lpc3xxx_pcm_mmap(struct snd_pcm_substream *substream,
			    struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

static struct snd_pcm_ops lpc3xxx_pcm_ops = {
	.open = lpc3xxx_pcm_open,
	.close = lpc3xxx_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = lpc3xxx_pcm_hw_params,
	.hw_free = lpc3xxx_pcm_hw_free,
	.prepare = lpc3xxx_pcm_prepare,
	.trigger = lpc3xxx_pcm_trigger,
	.pointer = lpc3xxx_pcm_pointer,
	.mmap = lpc3xxx_pcm_mmap,
};

/*
 * ASoC platform driver
 */
static int lpc3xxx_pcm_new(struct snd_card *card,
			   struct snd_soc_dai *dai,
			   struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &lpc3xxx_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (dai->playback.channels_min) {
		ret = lpc3xxx_pcm_allocate_dma_buffer(
			  pcm, SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		pr_debug("%s: Allocating PCM capture DMA buffer\n", SND_NAME);
		ret = lpc3xxx_pcm_allocate_dma_buffer(
			  pcm, SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}

out:
	return ret;
}

static void lpc3xxx_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (substream == NULL)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);

		buf->area = NULL;
	}
}

#if defined(CONFIG_PM)
static int lpc3xxx_pcm_suspend(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct lpc3xxx_dma_data *prtd;

	if (runtime == NULL)
		return 0;

	prtd = runtime->private_data;

	/* Disable the DMA channel */
	lpc32xx_dma_ch_disable(prtd->dmach);

	return 0;
}

static int lpc3xxx_pcm_resume(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct lpc3xxx_dma_data *prtd;

	if (runtime == NULL)
		return 0;

	prtd = runtime->private_data;

	/* Enable the DMA channel */
	lpc32xx_dma_ch_enable(prtd->dmach);

	return 0;
}

#else
#define lpc3xxx_pcm_suspend	NULL
#define lpc3xxx_pcm_resume	NULL
#endif

struct snd_soc_platform lpc3xxx_soc_platform = {
	.name = SND_NAME,
	.pcm_ops = &lpc3xxx_pcm_ops,
	.pcm_new = lpc3xxx_pcm_new,
	.pcm_free = lpc3xxx_pcm_free_dma_buffers,
	.suspend = lpc3xxx_pcm_suspend,
	.resume = lpc3xxx_pcm_resume,
};
EXPORT_SYMBOL_GPL(lpc3xxx_soc_platform);

static int __init lpc3xxx_soc_platform_init(void)
{
        return snd_soc_register_platform(&lpc3xxx_soc_platform);
}
module_init(lpc3xxx_soc_platform_init);

static void __exit lpc3xxx_soc_platform_exit(void)
{
        snd_soc_unregister_platform(&lpc3xxx_soc_platform);
}
module_exit(lpc3xxx_soc_platform_exit)

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("NXP LPC3XXX PCM module");
MODULE_LICENSE("GPL");
