/*
 *  linux/drivers/mmc/host/mmci.c - ARM PrimeCell MMCI PL180/1 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *  Copyright (C) 2010 ST-Ericsson AB.
 *  Copyright (C) 2010 NXP Semiconductors (LPC32xx DMA modifications)
 *
 * Customization of the DMA support for LPC178x/7x:
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * Add DMA support for STM32F2
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/amba/bus.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/gpio.h>
#include <linux/amba/mmci.h>
#include <linux/regulator/consumer.h>

#include <asm/div64.h>
#include <asm/io.h>
#include <asm/sizes.h>

#ifdef CONFIG_LPC178X_SD_DMA
#include <linux/dma-mapping.h>
#include <mach/clkdev.h>
#include <mach/dmac.h>
#include <mach/dma.h>
#endif /* CONFIG_LPC178X_SD_DMA */

#ifdef CONFIG_STM32_SD_DMA
#include <linux/dma-mapping.h>
#include <mach/dmainit.h>
#include <mach/dmac.h>
#endif /* CONFIG_STM32_SD_DMA */

#include "mmci.h"

#define DRIVER_NAME "mmci-pl18x"

static unsigned int fmax = 515633;

#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)

#define SD_FIFO(x)			(x + 0x80)

#define DMA_BUFF_SIZE SZ_64K

struct sddrv_dmac_data {
	struct device *dev;
	int lastch;

#if defined(CONFIG_LPC178X_SD_DMA)
	struct dma_config dmacfgtx;
	struct dma_config dmacfgrx;
	dma_addr_t dma_handle_tx;
	void *dma_v_base;
	int mapped;
	int preallocated_tx_buf;
#endif /* CONFIG_LPC178X_SD_DMA */
};
static struct sddrv_dmac_data dmac_drvdat;

#undef MCI_IRQENABLE
#define MCI_IRQENABLE \
	(MCI_CMDCRCFAILMASK | MCI_DATACRCFAILMASK | MCI_CMDTIMEOUTMASK | \
	MCI_DATATIMEOUTMASK | MCI_TXUNDERRUNMASK | MCI_RXOVERRUNMASK| \
	MCI_CMDRESPENDMASK | MCI_CMDSENTMASK | MCI_DATAENDMASK)

#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */

#if defined(CONFIG_LPC178X_SD_DMA)
static int mmc_dma_setup(struct mmci_platform_data *plat)
{
	u32 llptrrx, llptrtx;
	int ret = 0;

	/*
	 * There is a quirk with the LPC32XX and SD burst DMA. DMA sg
	 * transfers where DMA is the flow controller will not transfer
	 * the last few bytes to or from the SD card controller and
	 * memory. For RX, the last few bytes in the SD transfer can be
	 * forced out with a software DMA burst request. For TX, this
	 * can't be done, so TX sg support cannot be supported. For TX,
	 * a temporary bouncing buffer is used if more than 1 sg segment
	 * is passed in the data request. The bouncing buffer will get a
	 * contiguous copy of the TX data and it will be used instead.
	 */

	if (plat->dma_tx_size) {
		/* Use pre-allocated memory for the DMA Tx buffer */
		dmac_drvdat.dma_handle_tx = (dma_addr_t)plat->dma_tx_v_base;
		dmac_drvdat.dma_v_base = plat->dma_tx_v_base;
		dmac_drvdat.preallocated_tx_buf = 1;
	} else {
		/* Allocate a chunk of memory for the DMA TX buffers */
		dmac_drvdat.dma_v_base = dma_alloc_coherent(dmac_drvdat.dev,
			DMA_BUFF_SIZE, &dmac_drvdat.dma_handle_tx, GFP_KERNEL);
		dmac_drvdat.preallocated_tx_buf = 0;
	}

	if (dmac_drvdat.dma_v_base == NULL) {
		dev_err(dmac_drvdat.dev, "error getting DMA region\n");
		ret = -ENOMEM;
		goto dma_no_tx_buff;
	}
	dev_info(dmac_drvdat.dev, "DMA buffer: phy:%p, virt:%p\n",
		(void *) dmac_drvdat.dma_handle_tx,
		dmac_drvdat.dma_v_base);

	/* Setup TX DMA channel */
	dmac_drvdat.dmacfgtx.ch = DMA_CH_SDCARD_TX;
	dmac_drvdat.dmacfgtx.tc_inten = 0;
	dmac_drvdat.dmacfgtx.err_inten = 0;
	dmac_drvdat.dmacfgtx.src_size = 4;
	dmac_drvdat.dmacfgtx.src_inc = 1;
	dmac_drvdat.dmacfgtx.src_bsize = DMAC_CHAN_SRC_BURST_8;
	dmac_drvdat.dmacfgtx.src_prph = DMAC_SRC_PERIP(DMA_PERID_SDCARD);
	dmac_drvdat.dmacfgtx.dst_size = 4;
	dmac_drvdat.dmacfgtx.dst_inc = 0;
	dmac_drvdat.dmacfgtx.dst_bsize = DMAC_CHAN_DEST_BURST_8;
	dmac_drvdat.dmacfgtx.dst_prph = DMAC_DEST_PERIP(DMA_PERID_SDCARD);
	dmac_drvdat.dmacfgtx.flowctrl = DMAC_CHAN_FLOW_P_M2P;
	if (lpc178x_dma_ch_get(
		&dmac_drvdat.dmacfgtx, "dma_sd_tx", NULL, NULL) < 0)
	{
		dev_err(dmac_drvdat.dev,
			"Error setting up SD card TX DMA channel\n");
		ret = -ENODEV;
		goto dma_no_txch;
	}

	/* Allocate a linked list for DMA support */
	llptrtx = lpc178x_dma_alloc_llist(
		dmac_drvdat.dmacfgtx.ch, NR_SG * 2);
	if (llptrtx == 0) {
		dev_err(dmac_drvdat.dev,
			"Error allocating list buffer (MMC TX)\n");
		ret = -ENOMEM;
		goto dma_no_txlist;
	}

	/* Setup RX DMA channel */
	dmac_drvdat.dmacfgrx.ch = DMA_CH_SDCARD_RX;
	dmac_drvdat.dmacfgrx.tc_inten = 0;
	dmac_drvdat.dmacfgrx.err_inten = 0;
	dmac_drvdat.dmacfgrx.src_size = 4;
	dmac_drvdat.dmacfgrx.src_inc = 0;
	dmac_drvdat.dmacfgrx.src_bsize = DMAC_CHAN_SRC_BURST_8;
	dmac_drvdat.dmacfgrx.src_prph = DMAC_SRC_PERIP(DMA_PERID_SDCARD);
	dmac_drvdat.dmacfgrx.dst_size = 4;
	dmac_drvdat.dmacfgrx.dst_inc = 1;
	dmac_drvdat.dmacfgrx.dst_bsize = DMAC_CHAN_DEST_BURST_8;
	dmac_drvdat.dmacfgrx.dst_prph = DMAC_DEST_PERIP(DMA_PERID_SDCARD);
	dmac_drvdat.dmacfgrx.flowctrl = DMAC_CHAN_FLOW_D_P2M;
	if (lpc178x_dma_ch_get(
		&dmac_drvdat.dmacfgrx, "dma_sd_rx", NULL, NULL) < 0)
	{
		dev_err(dmac_drvdat.dev,
			"Error setting up SD card RX DMA channel\n");
		ret = -ENODEV;
		goto dma_no_rxch;
	}

	/* Allocate a linked list for DMA support */
	llptrrx = lpc178x_dma_alloc_llist(
		dmac_drvdat.dmacfgrx.ch, NR_SG * 2);
	if (llptrrx == 0) {
		dev_err(dmac_drvdat.dev,
			"Error allocating list buffer (MMC RX)\n");
		ret = -ENOMEM;
		goto dma_no_rxlist;
	}

	return 0;

dma_no_rxlist:
	lpc178x_dma_ch_put(dmac_drvdat.dmacfgrx.ch);
	dmac_drvdat.dmacfgrx.ch = -1;
dma_no_rxch:
	lpc178x_dma_dealloc_llist(dmac_drvdat.dmacfgtx.ch);
dma_no_txlist:
	lpc178x_dma_ch_put(dmac_drvdat.dmacfgtx.ch);
	dmac_drvdat.dmacfgtx.ch = -1;
dma_no_txch:
	if (!dmac_drvdat.preallocated_tx_buf) {
		dma_free_coherent(dmac_drvdat.dev, DMA_BUFF_SIZE,
			dmac_drvdat.dma_v_base,
			dmac_drvdat.dma_handle_tx);
	}
dma_no_tx_buff:
	return ret;
}

static void mmc_dma_dealloc(void)
{
	lpc178x_dma_dealloc_llist(dmac_drvdat.dmacfgrx.ch);
	lpc178x_dma_ch_put(dmac_drvdat.dmacfgrx.ch);
	dmac_drvdat.dmacfgrx.ch = -1;
	lpc178x_dma_dealloc_llist(dmac_drvdat.dmacfgtx.ch);
	lpc178x_dma_ch_put(dmac_drvdat.dmacfgtx.ch);
	dmac_drvdat.dmacfgtx.ch = -1;
	if (!dmac_drvdat.preallocated_tx_buf) {
		dma_free_coherent(dmac_drvdat.dev, DMA_BUFF_SIZE,
			dmac_drvdat.dma_v_base,
			dmac_drvdat.dma_handle_tx);
	}
}

/* Supports scatter/gather */
static void mmc_dma_rx_start(struct mmci_host *host)
{
	unsigned int len;
	int i, dma_len;
	struct scatterlist *sg;
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *reqdata = mrq->data;
	void *dmaaddr;
	u32 dmalen, dmaxferlen;

	sg = reqdata->sg;
	len = reqdata->sg_len;

	dma_len = dma_map_sg(
		mmc_dev(host->mmc), reqdata->sg, reqdata->sg_len,
		DMA_FROM_DEVICE);
	if (dma_len == 0)
		return;

	/* Setup transfer */
	for (i = 0; i < len; i++) {
		dmalen = (u32) sg_dma_len(&sg[i]);
		dmaaddr = (void *) sg_dma_address(&sg[i]);

		/* Build a list with a max size if 15872 bytes per seg */
		while (dmalen > 0) {
			dmaxferlen = dmalen;
			if (dmaxferlen > 15872)
				dmaxferlen = 15872;

			lpc178x_dma_queue_llist_entry(dmac_drvdat.lastch,
				(void *) SD_FIFO((u32)host->base),
				dmaaddr, dmaxferlen);

				dmaaddr += dmaxferlen;
				dmalen -= dmaxferlen;
		}
	}
}

/* May need to reorganize buffer for scatter/gather */
static void mmc_dma_tx_start(struct mmci_host *host)
{
	unsigned int len;
	int dma_len;
	struct scatterlist *sg;
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *reqdata = mrq->data;
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	void *dmaaddr;
	char *src_buffer, *dst_buffer;
	unsigned long flags;

	local_irq_save(flags);

	sg = reqdata->sg;
	len = reqdata->sg_len;

	/* Only 1 segment and no need to copy? */
	if (len == 1 && !dmac_drvdat.preallocated_tx_buf) {
		dma_len = dma_map_sg(mmc_dev(host->mmc), reqdata->sg,
			reqdata->sg_len, DMA_TO_DEVICE);
		if (dma_len == 0)
			return;

		dmaaddr = (void *) sg_dma_address(&sg[0]);
		dmac_drvdat.mapped = 1;
	} else {
		/* Move data to contiguous buffer first, then transfer it */
		dst_buffer = (char *) dmac_drvdat.dma_v_base;
		do
		{
			if (!sg_miter_next(sg_miter))
				break;

			/*
			 * Map the current scatter buffer, copy data, and unmap
			 */
			src_buffer = sg_miter->addr;
			memcpy(dst_buffer, src_buffer, sg_miter->length);
			dst_buffer += sg_miter->length;
		} while (1);

		sg_miter_stop(sg_miter);

		dmac_drvdat.mapped = 0;
		dmaaddr = (void *) dmac_drvdat.dma_handle_tx;
	}

	lpc178x_dma_start_pflow_xfer(DMA_CH_SDCARD_TX, dmaaddr,
		(void *) SD_FIFO((u32)host->base), 1);

	local_irq_restore(flags);
}

#elif defined(CONFIG_STM32_SD_DMA)

static int mmc_dma_setup(struct mmci_platform_data *plat)
{
	return stm32_dma_ch_get(STM32F2_DMACH_SDIO);
}

static void mmc_dma_dealloc(void)
{
	int rv;

	rv = stm32_dma_ch_put(STM32F2_DMACH_SDIO);
	if (rv < 0)
		pr_err("%s: stm32_dma_ch_put() failed (%d)\n", __func__, rv);
}

/*
 * Prepare and enable DMA Rx channel (on STM32)
 */
static void mmc_dma_rx_start(struct mmci_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *reqdata = mrq->data;
	int dma_len;
	int rv;

	/* Scatter/gather DMA is not supported */
	BUG_ON(reqdata->sg_len > 1);

	dma_len = dma_map_sg(
		mmc_dev(host->mmc), reqdata->sg, reqdata->sg_len,
		DMA_FROM_DEVICE);
	if (dma_len == 0) {
		dev_err(mmc_dev(host->mmc), "could not map DMA Rx buffer\n");
		goto out;
	}

	/*
	 * Direction: peripheral-to-memory
	 * Flow controller: peripheral
	 * Priority: very high (3)
	 * Double buffer mode: disabled
	 * Circular mode: disabled
	 */
	rv = stm32_dma_ch_init(STM32F2_DMACH_SDIO, 0, 1, 3, 0, 0);
	if (rv < 0)
		goto err;

	/*
	 * Enable burst mode; set FIFO threshold to "full FIFO"
	 */
	rv = stm32_dma_ch_init_fifo(STM32F2_DMACH_SDIO, 1, 3);
	if (rv < 0)
		goto err;

	/*
	 * Peripheral address: SDIO controller FIFO data register
	 * Peripheral increment: disabled
	 * Peripheral data size: 32-bit
	 * Burst transfer configuration: incremental burst of 4 beats
	 */
	rv = stm32_dma_ch_set_periph(STM32F2_DMACH_SDIO,
		SD_FIFO((u32)host->base), 0, 2, 1);
	if (rv < 0)
		goto err;

	/*
	 * Memory address: DMA buffer address
	 * Memory incremental: enabled
	 * Memory data size: 32-bit
	 * Burst transfer configuration: incremental burst of 4 beats
	 */
	rv = stm32_dma_ch_set_memory(STM32F2_DMACH_SDIO,
		sg_dma_address(&reqdata->sg[0]), 1, 2, 1);
	if (rv < 0)
		goto err;

	/*
	 * Set number of items to transfer to zero, because we use peripheral
	 * flow controller, and therefore the SDIO controller will stop
	 * the transfer when the whole block data has been transferred.
	 */
	rv = stm32_dma_ch_set_nitems(STM32F2_DMACH_SDIO, 0);
	if (rv < 0)
		goto err;

	/*
	 * Enable the DMA channel. After this point, the DMA transfer will
	 * be able to start.
	 */
	rv = stm32_dma_ch_enable(STM32F2_DMACH_SDIO);
	if (rv < 0)
		goto err;

	goto out;

err:
	dev_err(mmc_dev(host->mmc), "Rx DMA channel initialization failed\n");
out:
	;
}

/*
 * Prepare and enable DMA Tx channel (on STM32)
 */
static void mmc_dma_tx_start(struct mmci_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *reqdata = mrq->data;
	int dma_len;
	int rv;

	/* Scatter/gather DMA is not supported */
	BUG_ON(reqdata->sg_len > 1);

	dma_len = dma_map_sg(
		mmc_dev(host->mmc), reqdata->sg, reqdata->sg_len,
		DMA_TO_DEVICE);
	if (dma_len == 0) {
		dev_err(mmc_dev(host->mmc), "could not map DMA Tx buffer\n");
		goto out;
	}

	/*
	 * Direction: memory-to-peripheral
	 * Flow controller: peripheral
	 * Priority: very high (3)
	 * Double buffer mode: disabled
	 * Circular mode: disabled
	 */
	rv = stm32_dma_ch_init(STM32F2_DMACH_SDIO, 1, 1, 3, 0, 0);
	if (rv < 0)
		goto err;

	/*
	 * Enable burst mode; set FIFO threshold to "full FIFO"
	 */
	rv = stm32_dma_ch_init_fifo(STM32F2_DMACH_SDIO, 1, 3);
	if (rv < 0)
		goto err;

	/*
	 * Peripheral address: SDIO controller FIFO data register
	 * Peripheral increment: disabled
	 * Peripheral data size: 32-bit
	 * Burst transfer configuration: incremental burst of 4 beats
	 */
	rv = stm32_dma_ch_set_periph(STM32F2_DMACH_SDIO,
		SD_FIFO((u32)host->base), 0, 2, 1);
	if (rv < 0)
		goto err;

	/*
	 * Memory address: DMA buffer address
	 * Memory incremental: enabled
	 * Memory data size: 32-bit
	 * Burst transfer configuration: incremental burst of 4 beats
	 */
	rv = stm32_dma_ch_set_memory(STM32F2_DMACH_SDIO,
		sg_dma_address(&reqdata->sg[0]), 1, 2, 1);
	if (rv < 0)
		goto err;

	/*
	 * Set number of items to transfer to zero, because we use peripheral
	 * flow controller, and therefore the SDIO controller will stop
	 * the transfer when the whole block data has been transferred.
	 */
	rv = stm32_dma_ch_set_nitems(STM32F2_DMACH_SDIO, 0);
	if (rv < 0)
		goto err;

	/*
	 * Enable the DMA channel. After this point, the DMA transfer will
	 * be able to start.
	 */
	rv = stm32_dma_ch_enable(STM32F2_DMACH_SDIO);
	if (rv < 0)
		goto err;

	goto out;

err:
	dev_err(mmc_dev(host->mmc), "Tx DMA channel initialization failed\n");
out:
	;
}

#endif /* CONFIG_LPC178X_SD_DMA; CONFIG_STM32_SD_DMA */

/**
 * struct variant_data - MMCI variant-specific quirks
 * @clkreg: default value for MCICLOCK register
 * @clkreg_enable: enable value for MMCICLOCK register
 * @datalength_bits: number of bits in the MMCIDATALENGTH register
 * @fifosize: number of bytes that can be written when MMCI_TXFIFOEMPTY
 *	      is asserted (likewise for RX)
 * @fifohalfsize: number of bytes that can be written when MCI_TXFIFOHALFEMPTY
 *		  is asserted (likewise for RX)
 * @sdio: variant supports SDIO
 * @st_clkdiv: true if using a ST-specific clock divider algorithm
 * @pwrreg_powerup: power up value for MMCIPOWER register
 */
struct variant_data {
	unsigned int		clkreg;
	unsigned int		clkreg_enable;
	unsigned int		datalength_bits;
	unsigned int		fifosize;
	unsigned int		fifohalfsize;
	bool			sdio;
	bool			st_clkdiv;
	u32			pwrreg_powerup;
};

static struct variant_data variant_arm = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.datalength_bits	= 16,
	.pwrreg_powerup		= MCI_PWR_UP,
};

static struct variant_data variant_u300 = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg_enable		= MCI_ST_U300_HWFCEN,
	.datalength_bits	= 16,
	.sdio			= true,
	.pwrreg_powerup		= MCI_PWR_ON,
};

static struct variant_data variant_ux500 = {
	.fifosize		= 30 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg			= MCI_CLK_ENABLE,
#if defined(CONFIG_STM32_SD_DMA)
	/*
	 * On STM32, do not use HW flow control as recommended
	 * in the STM32F20x/STM32F21x errata sheet.
	 */
	.clkreg_enable		= 0,
#else
	.clkreg_enable		= MCI_ST_UX500_HWFCEN,
#endif
	.datalength_bits	= 24,
	.sdio			= true,
	.st_clkdiv		= true,
	.pwrreg_powerup		= MCI_PWR_ON,
};

/*
 * This must be called with host->lock held
 */
static void mmci_set_clkreg(struct mmci_host *host, unsigned int desired)
{
	struct variant_data *variant = host->variant;
	u32 clk = variant->clkreg;

	if (desired) {
		if (desired >= host->mclk) {
			/*
			 * The ST clock divider does not like the bypass bit,
			 * even though it's available. Instead the datasheet
			 * recommends setting the divider to zero.
			 */
			if (!variant->st_clkdiv)
				clk = MCI_CLK_BYPASS;
			host->cclk = host->mclk;
		} else if (variant->st_clkdiv) {
			/*
			 * DB8500 TRM says f = mclk / (clkdiv + 2)
			 * => clkdiv = (mclk / f) - 2
			 * Round the divider up so we don't exceed the max
			 * frequency
			 */
			clk = DIV_ROUND_UP(host->mclk, desired) - 2;
			if (clk >= 256)
				clk = 255;
			host->cclk = host->mclk / (clk + 2);
		} else {
			/*
			 * PL180 TRM says f = mclk / (2 * (clkdiv + 1))
			 * => clkdiv = mclk / (2 * f) - 1
			 */
			clk = DIV_ROUND_UP(host->mclk, 2 * desired) - 1;
			if (clk >= 256)
				clk = 255;
			host->cclk = host->mclk / (2 * (clk + 1));
		}

		clk |= variant->clkreg_enable;
		clk |= MCI_CLK_ENABLE;
		/* This hasn't proven to be worthwhile */
		/* clk |= MCI_CLK_PWRSAVE; */
	}

	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_4)
		clk |= MCI_4BIT_BUS;
	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_8)
		clk |= MCI_ST_8BIT_BUS;

	writel(clk, host->base + MMCICLOCK);
}

static void
mmci_request_end(struct mmci_host *host, struct mmc_request *mrq)
{
	writel(0, host->base + MMCICOMMAND);

	BUG_ON(host->data);

	host->mrq = NULL;
	host->cmd = NULL;

	if (mrq->data)
		mrq->data->bytes_xfered = host->data_xfered;

	/*
	 * Need to drop the host lock here; mmc_request_done may call
	 * back into the driver...
	 */
	spin_unlock(&host->lock);
	mmc_request_done(host->mmc, mrq);
	spin_lock(&host->lock);
}

static void mmci_set_mask1(struct mmci_host *host, unsigned int mask)
{
	void __iomem *base = host->base;

	if (host->singleirq) {
		unsigned int mask0 = readl(base + MMCIMASK0);

		mask0 &= ~MCI_IRQ1MASK;
		mask0 |= mask;

		writel(mask0, base + MMCIMASK0);
	}

	writel(mask, base + MMCIMASK1);
}

static void mmci_stop_data(struct mmci_host *host)
{
	writel(0, host->base + MMCIDATACTRL);
	mmci_set_mask1(host, 0);
	host->data = NULL;
}

static void mmci_init_sg(struct mmci_host *host, struct mmc_data *data)
{
	unsigned int flags = SG_MITER_ATOMIC;

	if (data->flags & MMC_DATA_READ)
		flags |= SG_MITER_TO_SG;
	else
		flags |= SG_MITER_FROM_SG;

	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
}

static void mmci_start_data(struct mmci_host *host, struct mmc_data *data)
{
	struct variant_data *variant = host->variant;
	unsigned int datactrl, timeout, irqmask = 0;
	unsigned long long clks;
	void __iomem *base;
	int blksz_bits;

	dev_dbg(mmc_dev(host->mmc), "blksz %04x blks %04x flags %08x\n",
		data->blksz, data->blocks, data->flags);

	host->data = data;
	host->size = data->blksz;
	host->data_xfered = 0;

	mmci_init_sg(host, data);

	clks = (unsigned long long)data->timeout_ns * host->cclk;
	do_div(clks, 1000000000UL);

	timeout = data->timeout_clks + (unsigned int)clks;

	base = host->base;
	writel(timeout, base + MMCIDATATIMER);
#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)
	writel((host->size * data->blocks), base + MMCIDATALENGTH);
#else
	writel(host->size, base + MMCIDATALENGTH);
#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */

	blksz_bits = ffs(data->blksz) - 1;
	BUG_ON(1 << blksz_bits != data->blksz);

#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)
	datactrl = MCI_DPSM_ENABLE | MCI_DPSM_DMAENABLE | blksz_bits << 4;
	if (data->flags & MMC_DATA_READ) {
		datactrl |= MCI_DPSM_DIRECTION;
		dmac_drvdat.lastch = DMA_CH_SDCARD_RX;
		mmc_dma_rx_start(host);
	} else {
		dmac_drvdat.lastch = DMA_CH_SDCARD_TX;
		mmc_dma_tx_start(host);
	}
#else
	datactrl = MCI_DPSM_ENABLE | blksz_bits << 4;
	if (data->flags & MMC_DATA_READ) {
		datactrl |= MCI_DPSM_DIRECTION;
		irqmask = MCI_RXFIFOHALFFULLMASK;

		/*
		 * If we have less than a FIFOSIZE of bytes to transfer,
		 * trigger a PIO interrupt as soon as any data is available.
		 */
		if (host->size < variant->fifosize)
			irqmask |= MCI_RXDATAAVLBLMASK;
	} else {
		/*
		 * We don't actually need to include "FIFO empty" here
		 * since its implicit in "FIFO half empty".
		 */
		irqmask = MCI_TXFIFOHALFEMPTYMASK;
	}
#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */

	/* The ST Micro variants has a special bit to enable SDIO */
	if (variant->sdio && host->mmc->card)
		if (mmc_card_sdio(host->mmc->card))
			datactrl |= MCI_ST_DPSM_SDIOEN;

	writel(datactrl, base + MMCIDATACTRL);
#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)
	datactrl = readl(base + MMCIMASK0) & ~MCI_DATABLOCKENDMASK;
	writel(datactrl | MCI_DATAENDMASK, base + MMCIMASK0);
#else
	writel(readl(base + MMCIMASK0) & ~MCI_DATAENDMASK, base + MMCIMASK0);
#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */

	mmci_set_mask1(host, irqmask);
}

static void
mmci_start_command(struct mmci_host *host, struct mmc_command *cmd, u32 c)
{
	void __iomem *base = host->base;

	dev_dbg(mmc_dev(host->mmc), "op %02x arg %08x flags %08x\n",
	    cmd->opcode, cmd->arg, cmd->flags);

	if (readl(base + MMCICOMMAND) & MCI_CPSM_ENABLE) {
		writel(0, base + MMCICOMMAND);
		udelay(1);
	}

	c |= cmd->opcode | MCI_CPSM_ENABLE;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			c |= MCI_CPSM_LONGRSP;
		c |= MCI_CPSM_RESPONSE;
	}
	if (/*interrupt*/0)
		c |= MCI_CPSM_INTERRUPT;

	host->cmd = cmd;

	writel(cmd->arg, base + MMCIARGUMENT);
	writel(c, base + MMCICOMMAND);
}

static void
mmci_data_irq(struct mmci_host *host, struct mmc_data *data,
	      unsigned int status)
{
#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)
	if (status & MCI_DATAEND) {
		host->data_xfered += data->blksz * data->blocks;
	}
#else
	if (status & MCI_DATABLOCKEND) {
		host->data_xfered += data->blksz;
	}
#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */

	if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_TXUNDERRUN|MCI_RXOVERRUN)) {
		dev_dbg(mmc_dev(host->mmc), "MCI ERROR IRQ (status %08x)\n", status);
		if (status & MCI_DATACRCFAIL)
			data->error = -EILSEQ;
		else if (status & MCI_DATATIMEOUT)
			data->error = -ETIMEDOUT;
		else if (status & (MCI_TXUNDERRUN|MCI_RXOVERRUN))
			data->error = -EIO;
		status |= MCI_DATAEND;

		/*
		 * We hit an error condition.  Ensure that any data
		 * partially written to a page is properly coherent.
		 */
		if (data->flags & MMC_DATA_READ) {
			struct sg_mapping_iter *sg_miter = &host->sg_miter;
			unsigned long flags;

			local_irq_save(flags);
			if (sg_miter_next(sg_miter)) {
				flush_dcache_page(sg_miter->page);
				sg_miter_stop(sg_miter);
			}
			local_irq_restore(flags);
		}
	}

	if (status & MCI_DATAEND) {
#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)
		if (data->flags & MMC_DATA_READ) {
#if defined(CONFIG_LPC178X_SD_DMA)
			lpc178x_dma_force_burst(dmac_drvdat.lastch,
				DMA_PERID_SDCARD);
			lpc178x_dma_flush_llist(dmac_drvdat.lastch);
#else /* CONFIG_STM32_SD_DMA */
			stm32_dma_ch_disable(STM32F2_DMACH_SDIO);
#endif
			dma_unmap_sg(mmc_dev(host->mmc),
				data->sg, data->sg_len, DMA_FROM_DEVICE);
		} else {
#if defined(CONFIG_LPC178X_SD_DMA)
			lpc178x_dma_ch_disable(dmac_drvdat.lastch);
			if (dmac_drvdat.mapped)
				dma_unmap_sg(mmc_dev(host->mmc), data->sg,
					data->sg_len, DMA_TO_DEVICE);
#else /* CONFIG_STM32_SD_DMA */
			stm32_dma_ch_disable(STM32F2_DMACH_SDIO);
			dma_unmap_sg(mmc_dev(host->mmc), data->sg,
				data->sg_len, DMA_TO_DEVICE);
#endif
		}
#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */

		mmci_stop_data(host);

		if (!data->stop) {
			mmci_request_end(host, data->mrq);
		} else {
			mmci_start_command(host, data->stop, 0);
		}
	}
}

static void
mmci_cmd_irq(struct mmci_host *host, struct mmc_command *cmd,
	     unsigned int status)
{
	void __iomem *base = host->base;

	host->cmd = NULL;

	cmd->resp[0] = readl(base + MMCIRESPONSE0);
	cmd->resp[1] = readl(base + MMCIRESPONSE1);
	cmd->resp[2] = readl(base + MMCIRESPONSE2);
	cmd->resp[3] = readl(base + MMCIRESPONSE3);

	if (status & MCI_CMDTIMEOUT) {
		cmd->error = -ETIMEDOUT;
	} else if (status & MCI_CMDCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		cmd->error = -EILSEQ;
	}

	if (!cmd->data || cmd->error) {
		if (host->data)
			mmci_stop_data(host);
		mmci_request_end(host, cmd->mrq);
	} else if (!(cmd->data->flags & MMC_DATA_READ)) {
		mmci_start_data(host, cmd->data);
	}
}

static int mmci_pio_read(struct mmci_host *host, char *buffer, unsigned int remain)
{
	void __iomem *base = host->base;
	char *ptr = buffer;
	u32 status;
	int host_remain = host->size;

	do {
		int count = host_remain - (readl(base + MMCIFIFOCNT) << 2);

		if (count > remain)
			count = remain;

		if (count <= 0)
			break;

		readsl(base + MMCIFIFO, ptr, count >> 2);

		ptr += count;
		remain -= count;
		host_remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_RXDATAAVLBL);

	return ptr - buffer;
}

static int mmci_pio_write(struct mmci_host *host, char *buffer, unsigned int remain, u32 status)
{
	struct variant_data *variant = host->variant;
	void __iomem *base = host->base;
	char *ptr = buffer;

	do {
		unsigned int count, maxcnt;

		maxcnt = status & MCI_TXFIFOEMPTY ?
			 variant->fifosize : variant->fifohalfsize;
		count = min(remain, maxcnt);

		/*
		 * The ST Micro variant for SDIO transfer sizes
		 * less then 8 bytes should have clock H/W flow
		 * control disabled.
		 */
		if (variant->sdio &&
		    mmc_card_sdio(host->mmc->card)) {
			if (count < 8)
				writel(readl(host->base + MMCICLOCK) &
					~variant->clkreg_enable,
					host->base + MMCICLOCK);
			else
				writel(readl(host->base + MMCICLOCK) |
					variant->clkreg_enable,
					host->base + MMCICLOCK);
		}

		/*
		 * SDIO especially may want to send something that is
		 * not divisible by 4 (as opposed to card sectors
		 * etc), and the FIFO only accept full 32-bit writes.
		 * So compensate by adding +3 on the count, a single
		 * byte become a 32bit write, 7 bytes will be two
		 * 32bit writes etc.
		 */
		writesl(base + MMCIFIFO, ptr, (count + 3) >> 2);

		ptr += count;
		remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_TXFIFOHALFEMPTY);

	return ptr - buffer;
}

/*
 * PIO data transfer IRQ handler.
 */
static irqreturn_t mmci_pio_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	struct variant_data *variant = host->variant;
	void __iomem *base = host->base;
	unsigned long flags;
	u32 status;

	status = readl(base + MMCISTATUS);

	dev_dbg(mmc_dev(host->mmc), "irq1 (pio) %08x\n", status);

	local_irq_save(flags);

	do {
		unsigned int remain, len;
		char *buffer;

		/*
		 * For write, we only need to test the half-empty flag
		 * here - if the FIFO is completely empty, then by
		 * definition it is more than half empty.
		 *
		 * For read, check for data available.
		 */
		if (!(status & (MCI_TXFIFOHALFEMPTY|MCI_RXDATAAVLBL)))
			break;

		if (!sg_miter_next(sg_miter))
			break;

		buffer = sg_miter->addr;
		remain = sg_miter->length;

		len = 0;
		if (status & MCI_RXACTIVE)
			len = mmci_pio_read(host, buffer, remain);
		if (status & MCI_TXACTIVE)
			len = mmci_pio_write(host, buffer, remain, status);

		sg_miter->consumed = len;

		host->size -= len;
		remain -= len;

		if (remain)
			break;

		if (status & MCI_RXACTIVE)
			flush_dcache_page(sg_miter->page);

		status = readl(base + MMCISTATUS);
	} while (1);

	sg_miter_stop(sg_miter);

	local_irq_restore(flags);

	/*
	 * If we're nearing the end of the read, switch to
	 * "any data available" mode.
	 */
	if (status & MCI_RXACTIVE && host->size < variant->fifosize)
		mmci_set_mask1(host, MCI_RXDATAAVLBLMASK);

	/*
	 * If we run out of data, disable the data IRQs; this
	 * prevents a race where the FIFO becomes empty before
	 * the chip itself has disabled the data path, and
	 * stops us racing with our data end IRQ.
	 */
	if (host->size == 0) {
		mmci_set_mask1(host, 0);
		writel(readl(base + MMCIMASK0) | MCI_DATAENDMASK, base + MMCIMASK0);
	}

	return IRQ_HANDLED;
}

/*
 * Handle completion of command and data transfers.
 */
static irqreturn_t mmci_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;
	u32 status;
	int ret = 0;

	spin_lock(&host->lock);

	do {
		struct mmc_command *cmd;
		struct mmc_data *data;

		status = readl(host->base + MMCISTATUS);

		if (host->singleirq) {
			if (status & readl(host->base + MMCIMASK1))
				mmci_pio_irq(irq, dev_id);

			status &= ~MCI_IRQ1MASK;
		}

		status &= readl(host->base + MMCIMASK0);
#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)
		writel((status | MCI_DATABLOCKEND), host->base + MMCICLEAR);
#else
		writel(status, host->base + MMCICLEAR);
#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */

		dev_dbg(mmc_dev(host->mmc), "irq0 (data+cmd) %08x\n", status);

		data = host->data;
		if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_TXUNDERRUN|
			      MCI_RXOVERRUN|MCI_DATAEND|MCI_DATABLOCKEND) && data)
			mmci_data_irq(host, data, status);

		cmd = host->cmd;
		if (status & (MCI_CMDCRCFAIL|MCI_CMDTIMEOUT|MCI_CMDSENT|MCI_CMDRESPEND) && cmd)
			mmci_cmd_irq(host, cmd, status);

		ret = 1;
	} while (status);

	spin_unlock(&host->lock);

	return IRQ_RETVAL(ret);
}

static void mmci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmci_host *host = mmc_priv(mmc);
	unsigned long flags;

	WARN_ON(host->mrq != NULL);

	if (mrq->data && !is_power_of_2(mrq->data->blksz)) {
		dev_err(mmc_dev(mmc), "unsupported block size (%d bytes)\n",
			mrq->data->blksz);
		mrq->cmd->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	host->mrq = mrq;

	if (mrq->data && mrq->data->flags & MMC_DATA_READ)
		mmci_start_data(host, mrq->data);

	mmci_start_command(host, mrq->cmd, 0);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void mmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmci_host *host = mmc_priv(mmc);
	struct variant_data *variant = host->variant;
	u32 pwr = 0;
	unsigned long flags;
	int ret;

	if (host->plat->ios_handler &&
		host->plat->ios_handler(mmc_dev(mmc), ios))
			dev_err(mmc_dev(mmc), "platform ios_handler failed\n");

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		if (host->vcc)
			ret = mmc_regulator_set_ocr(mmc, host->vcc, 0);
		break;
	case MMC_POWER_UP:
		if (host->vcc) {
			ret = mmc_regulator_set_ocr(mmc, host->vcc, ios->vdd);
			if (ret) {
				dev_err(mmc_dev(mmc), "unable to set OCR\n");
				/*
				 * The .set_ios() function in the mmc_host_ops
				 * struct return void, and failing to set the
				 * power should be rare so we print an error
				 * and return here.
				 */
				return;
			}
		}
		/*
		 * The ST Micro variant doesn't have the PL180s MCI_PWR_UP
		 * and instead uses MCI_PWR_ON so apply whatever value is
		 * configured in the variant data.
		 */
		pwr |= variant->pwrreg_powerup;

		break;
	case MMC_POWER_ON:
		pwr |= MCI_PWR_ON;
		break;
	}

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN) {
		if (host->hw_designer != AMBA_VENDOR_ST)
			pwr |= MCI_ROD;
		else {
			/*
			 * The ST Micro variant use the ROD bit for something
			 * else and only has OD (Open Drain).
			 */
			pwr |= MCI_OD;
		}
	}

	spin_lock_irqsave(&host->lock, flags);

	mmci_set_clkreg(host, ios->clock);

	if (host->pwr != pwr) {
		host->pwr = pwr;
		writel(pwr, host->base + MMCIPOWER);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static int mmci_get_ro(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);

	if (host->gpio_wp == -ENOSYS)
		return -ENOSYS;

	return gpio_get_value(host->gpio_wp);
}

static int mmci_get_cd(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);
	unsigned int status;

	if (host->gpio_cd == -ENOSYS)
		status = host->plat->status(mmc_dev(host->mmc));
	else
		status = gpio_get_value(host->gpio_cd);

	return !status;
}

static irqreturn_t mmci_cd_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;

	mmc_detect_change(host->mmc, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

static const struct mmc_host_ops mmci_ops = {
	.request	= mmci_request,
	.set_ios	= mmci_set_ios,
	.get_ro		= mmci_get_ro,
	.get_cd		= mmci_get_cd,
};

static void mmci_check_status(unsigned long data)
{
	struct mmci_host *host = (struct mmci_host *)data;
	unsigned int status = mmci_get_cd(host->mmc);

	if (status ^ host->oldstat)
		mmc_detect_change(host->mmc, 0);

	host->oldstat = status;
	mod_timer(&host->timer, jiffies + HZ);
}

static int __devinit mmci_probe(struct amba_device *dev, struct amba_id *id)
{
	struct mmci_platform_data *plat = dev->dev.platform_data;
	struct variant_data *variant = id->data;
	struct mmci_host *host;
	struct mmc_host *mmc;
	unsigned int mask;
	int ret;

	/* must have platform data */
	if (!plat) {
		ret = -EINVAL;
		goto out;
	}

	ret = amba_request_regions(dev, DRIVER_NAME);
	if (ret)
		goto out;

	mmc = mmc_alloc_host(sizeof(struct mmci_host), &dev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto rel_regions;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;

	host->gpio_wp = -ENOSYS;
	host->gpio_cd = -ENOSYS;
	host->gpio_cd_irq = -1;

	host->hw_designer = amba_manf(dev);
	host->hw_revision = amba_rev(dev);
	dev_dbg(mmc_dev(mmc), "designer ID = 0x%02x\n", host->hw_designer);
	dev_dbg(mmc_dev(mmc), "revision = 0x%01x\n", host->hw_revision);

	host->clk = clk_get(&dev->dev, NULL);
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto host_free;
	}

	ret = clk_enable(host->clk);
	if (ret)
		goto clk_free;

	host->plat = plat;
	host->variant = variant;
	host->mclk = clk_get_rate(host->clk);
	/*
	 * According to the spec, mclk is max 100 MHz,
	 * so we try to adjust the clock down to this,
	 * (if possible).
	 */
	if (host->mclk > 100000000) {
		ret = clk_set_rate(host->clk, 100000000);
		if (ret < 0)
			goto clk_disable;
		host->mclk = clk_get_rate(host->clk);
		dev_dbg(mmc_dev(mmc), "eventual mclk rate: %u Hz\n",
			host->mclk);
	}
	host->base = ioremap(dev->res.start, resource_size(&dev->res));
	if (!host->base) {
		ret = -ENOMEM;
		goto clk_disable;
	}

	mmc->ops = &mmci_ops;
	mmc->f_min = (host->mclk + 511) / 512;
	/*
	 * If the platform data supplies a maximum operating
	 * frequency, this takes precedence. Else, we fall back
	 * to using the module parameter, which has a (low)
	 * default value in case it is not specified. Either
	 * value must not exceed the clock rate into the block,
	 * of course.
	 */
	if (plat->f_max)
		mmc->f_max = min(host->mclk, plat->f_max);
	else
		mmc->f_max = min(host->mclk, fmax);
	dev_dbg(mmc_dev(mmc), "clocking block at %u Hz\n", mmc->f_max);

#ifdef CONFIG_REGULATOR
	/* If we're using the regulator framework, try to fetch a regulator */
	host->vcc = regulator_get(&dev->dev, "vmmc");
	if (IS_ERR(host->vcc))
		host->vcc = NULL;
	else {
		int mask = mmc_regulator_get_ocrmask(host->vcc);

		if (mask < 0)
			dev_err(&dev->dev, "error getting OCR mask (%d)\n",
				mask);
		else {
			host->mmc->ocr_avail = (u32) mask;
			if (plat->ocr_mask)
				dev_warn(&dev->dev,
				 "Provided ocr_mask/setpower will not be used "
				 "(using regulator instead)\n");
		}
	}
#endif
	/* Fall back to platform data if no regulator is found */
	if (host->vcc == NULL)
		mmc->ocr_avail = plat->ocr_mask;
	mmc->caps = plat->capabilities;

#ifdef CONFIG_STM32_SD_DMA
	/*
	 * Always use a single bounce buffer on STM32. On STM32F2 this
	 * could be set to 2, because this MCU supports Double Buffer Mode,
	 * but it would be harder to implement than single buffer DMA.
	 */
	mmc->max_hw_segs = 1;
#else
	/*
	 * We can do SGIO
	 */
	mmc->max_hw_segs = 16;
#endif /* CONFIG_STM32_SD_DMA */

	mmc->max_phys_segs = NR_SG;

#ifdef CONFIG_STM32_SD_DMA
	/*
	 * The Number Of Data register on STM32 is 16-bit, but note that it
	 * denotes the number of 32-bit words in a data transfer. This is why
	 * we multiply the maximum value of this register by 4.
	 */
	mmc->max_req_size = ((1 << 16) - 1) * 4;
#else
	/*
	 * Since only a certain number of bits are valid in the data length
	 * register, we must ensure that we don't exceed 2^num-1 bytes in a
	 * single request.
	 */
	mmc->max_req_size = (1 << variant->datalength_bits) - 1;
#endif /* CONFIG_STM32_SD_DMA */

#ifdef CONFIG_LPC178X_SD_DMA
	/*
	 * The LPC32x0 DMA controller can handle up to a 16383 byte DMA
	 * transfer. We'll rely on the mmc core to make sure the passed
	 * size for a request is block aligned.
	 *
	 * We configure the peripheral (i.e. the SD card controller)
	 * as the flow controller for the DMA transmit requests, therefore
	 * the transfer size value in the DMA channel control register
	 * is ignored and the DMA transmit buffer may have unlimited size.
	 */
	if (plat->dma_tx_size)
		mmc->max_seg_size = plat->dma_tx_size;
	else
		mmc->max_seg_size = DMA_BUFF_SIZE;
#else
	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */
	mmc->max_seg_size = mmc->max_req_size;
#endif /* CONFIG_LPC178X_SD_DMA */

	/*
	 * Block size can be up to 2048 bytes, but must be a power of two.
	 */
	mmc->max_blk_size = 2048;

	/*
	 * No limit on the number of blocks transferred.
	 */
	mmc->max_blk_count = mmc->max_req_size;

#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)
	/*
	 * Setup DMA for the interface
	 */
	dmac_drvdat.dev = &dev->dev;
	if (mmc_dma_setup(plat))
		goto err_dma_setup;
#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */

	spin_lock_init(&host->lock);

	writel(0, host->base + MMCIMASK0);
	writel(0, host->base + MMCIMASK1);
	writel(0xfff, host->base + MMCICLEAR);

	if (gpio_is_valid(plat->gpio_cd)) {
		ret = gpio_request(plat->gpio_cd, DRIVER_NAME " (cd)");
		if (ret == 0)
			ret = gpio_direction_input(plat->gpio_cd);
		if (ret == 0)
			host->gpio_cd = plat->gpio_cd;
		else if (ret != -ENOSYS)
			goto err_gpio_cd;

		ret = request_any_context_irq(gpio_to_irq(plat->gpio_cd),
					      mmci_cd_irq, 0,
					      DRIVER_NAME " (cd)", host);
		if (ret >= 0)
			host->gpio_cd_irq = gpio_to_irq(plat->gpio_cd);
	}
	if (gpio_is_valid(plat->gpio_wp)) {
		ret = gpio_request(plat->gpio_wp, DRIVER_NAME " (wp)");
		if (ret == 0)
			ret = gpio_direction_input(plat->gpio_wp);
		if (ret == 0)
			host->gpio_wp = plat->gpio_wp;
		else if (ret != -ENOSYS)
			goto err_gpio_wp;
	}

	if (host->gpio_cd_irq < 0)
		mmc->caps |= MMC_CAP_NEEDS_POLL;

	ret = request_irq(dev->irq[0], mmci_irq, IRQF_SHARED, DRIVER_NAME " (cmd)", host);
	if (ret)
		goto unmap;

	if (dev->irq[1] == NO_IRQ)
		host->singleirq = true;
	else {
		ret = request_irq(dev->irq[1], mmci_pio_irq, IRQF_SHARED,
				  DRIVER_NAME " (pio)", host);
		if (ret)
			goto irq0_free;
	}

	mask = MCI_IRQENABLE;
	writel(mask, host->base + MMCIMASK0);

	amba_set_drvdata(dev, mmc);
	host->oldstat = mmci_get_cd(host->mmc);

	mmc_add_host(mmc);

	dev_info(&dev->dev, "%s: MMCI rev %x cfg %02x at 0x%016llx irq %d,%d\n",
		mmc_hostname(mmc), amba_rev(dev), amba_config(dev),
		(unsigned long long)dev->res.start, dev->irq[0], dev->irq[1]);

	init_timer(&host->timer);
	host->timer.data = (unsigned long)host;
	host->timer.function = mmci_check_status;
	host->timer.expires = jiffies + HZ;
	add_timer(&host->timer);

	return 0;

 irq0_free:
	free_irq(dev->irq[0], host);
 unmap:
	if (host->gpio_wp != -ENOSYS)
		gpio_free(host->gpio_wp);
 err_gpio_wp:
	if (host->gpio_cd_irq >= 0)
		free_irq(host->gpio_cd_irq, host);
	if (host->gpio_cd != -ENOSYS)
		gpio_free(host->gpio_cd);
 err_gpio_cd:
#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)
	mmc_dma_dealloc();
 err_dma_setup:
#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */
	iounmap(host->base);
 clk_disable:
	clk_disable(host->clk);
 clk_free:
	clk_put(host->clk);
 host_free:
	mmc_free_host(mmc);
 rel_regions:
	amba_release_regions(dev);
 out:
	return ret;
}

static int __devexit mmci_remove(struct amba_device *dev)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);

	amba_set_drvdata(dev, NULL);

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);

		del_timer_sync(&host->timer);

		mmc_remove_host(mmc);

		writel(0, host->base + MMCIMASK0);
		writel(0, host->base + MMCIMASK1);

		writel(0, host->base + MMCICOMMAND);
		writel(0, host->base + MMCIDATACTRL);

		free_irq(dev->irq[0], host);
		if (!host->singleirq)
			free_irq(dev->irq[1], host);

		if (host->gpio_wp != -ENOSYS)
			gpio_free(host->gpio_wp);
		if (host->gpio_cd_irq >= 0)
			free_irq(host->gpio_cd_irq, host);
		if (host->gpio_cd != -ENOSYS)
			gpio_free(host->gpio_cd);

#if defined(CONFIG_LPC178X_SD_DMA) || defined(CONFIG_STM32_SD_DMA)
		mmc_dma_dealloc();
#endif /* CONFIG_LPC178X_SD_DMA || CONFIG_STM32_SD_DMA */

		iounmap(host->base);
		clk_disable(host->clk);
		clk_put(host->clk);

		if (host->vcc)
			mmc_regulator_set_ocr(mmc, host->vcc, 0);
		regulator_put(host->vcc);

		mmc_free_host(mmc);

		amba_release_regions(dev);
	}

	return 0;
}

#ifdef CONFIG_PM
static int mmci_suspend(struct amba_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);
	int ret = 0;

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);

		ret = mmc_suspend_host(mmc, state);
		if (ret == 0)
			writel(0, host->base + MMCIMASK0);
	}

	return ret;
}

static int mmci_resume(struct amba_device *dev)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);
	int ret = 0;

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);

		writel(MCI_IRQENABLE, host->base + MMCIMASK0);

		ret = mmc_resume_host(mmc);
	}

	return ret;
}
#else
#define mmci_suspend	NULL
#define mmci_resume	NULL
#endif

static struct amba_id mmci_ids[] = {
	{
		.id	= 0x00041180,
		.mask	= 0x000fffff,
		.data	= &variant_arm,
	},
	{
		.id	= 0x00041181,
		.mask	= 0x000fffff,
		.data	= &variant_arm,
	},
	/* ST Micro variants */
	{
		.id     = 0x00180180,
		.mask   = 0x00ffffff,
		.data	= &variant_u300,
	},
	{
		.id     = 0x00280180,
		.mask   = 0x00ffffff,
		.data	= &variant_u300,
	},
	{
		.id     = 0x00480180,
		.mask   = 0x00ffffff,
		.data	= &variant_ux500,
	},
	{ 0, 0 },
};

static struct amba_driver mmci_driver = {
	.drv		= {
		.name	= DRIVER_NAME,
	},
	.probe		= mmci_probe,
	.remove		= __devexit_p(mmci_remove),
	.suspend	= mmci_suspend,
	.resume		= mmci_resume,
	.id_table	= mmci_ids,
};

static int __init mmci_init(void)
{
	return amba_driver_register(&mmci_driver);
}

static void __exit mmci_exit(void)
{
	amba_driver_unregister(&mmci_driver);
}

module_init(mmci_init);
module_exit(mmci_exit);
module_param(fmax, uint, 0444);

MODULE_DESCRIPTION("ARM PrimeCell PL180/181 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
