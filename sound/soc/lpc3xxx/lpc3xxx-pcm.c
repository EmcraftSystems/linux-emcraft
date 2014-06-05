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
#include <linux/dmaengine.h>
#include <linux/amba/pl08x.h>

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

struct lpc3xxx_pcm_priv {
	void				*regs;

	/* DMA configuration and support */
	struct dma_chan			*dma_channel;
	struct sg_table			*sgt;
	unsigned int			sgt_i, sgt_l, sgt_n;
	size_t				period_size;

	enum dma_data_direction		map_type;
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

static void setup_dma_scatter(void *buffer,
			      unsigned int length,
			      struct sg_table *sgtab)
{
	struct scatterlist *sg;
	int bytesleft = length;
	void *bufp = buffer;
	int mapbytes;
	int i;

	if (buffer) {
		for_each_sg(sgtab->sgl, sg, sgtab->nents, i) {
			/*
			 * If there are less bytes left than what fits
			 * in the current page (plus page alignment offset)
			 * we just feed in this, else we stuff in as much
			 * as we can.
			 */
			if (bytesleft < (PAGE_SIZE - offset_in_page(bufp)))
				mapbytes = bytesleft;
			else
				mapbytes = PAGE_SIZE - offset_in_page(bufp);
			sg_set_page(sg, virt_to_page(bufp),
				    mapbytes, offset_in_page(bufp));
			bufp += mapbytes;
			bytesleft -= mapbytes;
		}
	}

	BUG_ON(bytesleft);
}

/*
 * PCM operations
 */
static int lpc3xxx_pcm_hw_params(struct snd_pcm_substream *substream,
			         struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpc3xxx_pcm_priv *prtd = runtime->private_data;
	unsigned int pages;
	int ret = 0, i;

	/* this may get called several times by oss emulation
	 * with different params
	 */
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->period_size = params_period_bytes(params);

	pages = (prtd->period_size >> PAGE_SHIFT) + 1;

	prtd->sgt_n = DIV_ROUND_UP(runtime->dma_bytes, prtd->period_size);
	prtd->sgt = kcalloc(prtd->sgt_n, sizeof(*prtd->sgt), GFP_KERNEL);
	if (!prtd->sgt) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < prtd->sgt_n; ++i) {
		ret = sg_alloc_table(&prtd->sgt[i], pages, GFP_KERNEL);
		if (ret)
			goto out_free_table;
		setup_dma_scatter(
			(void *)runtime->dma_addr + i * prtd->period_size,
			prtd->period_size, &prtd->sgt[i]);
	}

out:
	return ret;

out_free_table:
	while(i--) {
		sg_free_table(&prtd->sgt[i]);
	}
	kfree(prtd->sgt);
	return ret;
}

static int lpc3xxx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct lpc3xxx_pcm_priv *prtd = substream->runtime->private_data;
	int i;

	for (i = 0; i < prtd->sgt_n; ++i) {
		struct sg_table *sgt = &prtd->sgt[i];
		sg_free_table(sgt);
	}

	/* Return the DMA channel */
	if (prtd->dma_channel) {
		dma_release_channel(prtd->dma_channel);
		prtd->dma_channel = NULL;
	}
	return 0;
}

static int lpc3xxx_pcm_queue_dma(struct snd_pcm_substream *substream);

static void lpc3xxx_pcm_dma_callback(void *data)
{
	struct snd_pcm_substream *substream = data;
	struct lpc3xxx_pcm_priv *prtd = substream->runtime->private_data;

	struct dma_chan *chan = prtd->dma_channel;
	struct sg_table *sgt = &prtd->sgt[prtd->sgt_l];

	dma_unmap_sg(chan->device->dev, sgt->sgl, sgt->nents, prtd->map_type);

	lpc3xxx_pcm_queue_dma(substream);
	snd_pcm_period_elapsed(substream);
}

static int lpc3xxx_pcm_queue_dma(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct lpc3xxx_pcm_priv *prtd = rtd->private_data;

	struct dma_chan *chan = prtd->dma_channel;
	struct sg_table *sgt = &prtd->sgt[prtd->sgt_i];
	unsigned int sglen;
	struct dma_async_tx_descriptor *desc;

	prtd->sgt_l = prtd->sgt_i;
	prtd->sgt_i = (prtd->sgt_i + 1) % prtd->sgt_n;

	sglen = dma_map_sg(chan->device->dev,
			sgt->sgl, sgt->nents, prtd->map_type);
	if (!sglen)
		return -ENOMEM;

	desc = chan->device->device_prep_slave_sg(
			chan,
			sgt->sgl,
			sglen,
			prtd->map_type,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK,
			NULL);

	desc->callback = lpc3xxx_pcm_dma_callback;
	desc->callback_param = substream;

	dmaengine_submit(desc);
	dma_async_issue_pending(chan);

	return 0;
}


static int lpc3xxx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct lpc3xxx_pcm_priv *prtd = substream->runtime->private_data;
	struct dma_chan *chan = prtd->dma_channel;
	struct dma_slave_config ch_config;

	ch_config.dst_addr_width = 4;
	ch_config.dst_maxburst = 4;
	ch_config.src_addr_width = 4;
	ch_config.src_maxburst = 4;
	ch_config.device_fc = false;

	/* Setup DMA channel */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ch_config.dst_addr = (dma_addr_t)I2S_TX_FIFO(prtd->regs);
		prtd->map_type = ch_config.direction = DMA_TO_DEVICE;
	}
	else {
		ch_config.src_addr = (dma_addr_t)I2S_RX_FIFO(prtd->regs);
		prtd->map_type = ch_config.direction = DMA_FROM_DEVICE;
	}

	dmaengine_slave_config(chan, &ch_config);

	return 0;
}

static int lpc3xxx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct lpc3xxx_pcm_priv *prtd = rtd->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		lpc3xxx_pcm_queue_dma(substream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		dmaengine_terminate_all(prtd->dma_channel);

		dma_unmap_sg(prtd->dma_channel->device->dev,
				prtd->sgt[prtd->sgt_l].sgl,
				prtd->sgt[prtd->sgt_l].nents,
				prtd->map_type);
		prtd->sgt_l = 0;
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dmaengine_pause(prtd->dma_channel);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dmaengine_resume(prtd->dma_channel);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t lpc3xxx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpc3xxx_pcm_priv *prtd = runtime->private_data;

	return bytes_to_frames(runtime, prtd->sgt_l * prtd->period_size);
}

static int lpc3xxx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpc3xxx_pcm_priv *prtd;
	int ret = 0;

	dma_cap_mask_t dma_cap;

	/* Try to acquire a generic DMA engine slave channel */
	dma_cap_zero(dma_cap);
	dma_cap_set(DMA_SLAVE, dma_cap);

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

	prtd->regs = LPC32XX_I2S0_BASE;

	prtd->dma_channel = dma_request_channel(dma_cap,
			pl08x_filter_id, "i2s0_tx");
	if (prtd->dma_channel == NULL) {
		ret = -ENODEV;
		goto out_free;
	}

	return 0;

out_free:
	kfree(prtd);
out:
	return ret;
}

static int lpc3xxx_pcm_close(struct snd_pcm_substream *substream)
{
	struct lpc3xxx_pcm_priv *prtd = substream->runtime->private_data;

	if (prtd->dma_channel) {
		dma_release_channel(prtd->dma_channel);
		prtd->dma_channel = NULL;
	}

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
	struct lpc3xxx_pcm_priv *prtd;

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
	struct lpc3xxx_pcm_priv *prtd;

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
