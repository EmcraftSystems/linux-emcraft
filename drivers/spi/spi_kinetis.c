/*
 * spi_mvf_dspi.c - DSPI controller for Faraday processors
 *
 * Copyright 2012 Freescale Semiconductor, Inc.
 *
 * Based on dspi_mcf.c
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ***************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/time.h>
#include <mach/spi-mvf.h>

#define DRIVER_NAME			"kinetis-dspi"

#define START_STATE			((void *)0)
#define RUNNING_STATE			((void *)1)
#define DONE_STATE			((void *)2)
#define ERROR_STATE			((void *)-1)

#define QUEUE_RUNNING			0
#define QUEUE_STOPPED			1
#define TRAN_STATE_RX_VOID		0x01
#define TRAN_STATE_TX_VOID		0x02
#define TRAN_STATE_WORD_ODD_NUM		0x04

#define DSPI_FIFO_SIZE			4

#if defined(CONFIG_SPI_MVF_DSPI_EDMA)
#define SPI_DSPI_EDMA
#define EDMA_BUFSIZE_KMALLOC	(DSPI_FIFO_SIZE * 4)
#define DSPI_DMA_RX_TCD		DMA_MUX_DSPI0_RX
#define DSPI_DMA_TX_TCD		DMA_MUX_DSPI0_TX
#endif

struct DSPI_MCR {
	unsigned halt:1;
	unsigned reserved71:7;
	unsigned smpl_pt:2;
	unsigned clr_rxf:1;
	unsigned clr_tx:1;
	unsigned dis_rxf:1;
	unsigned dis_tx:1;
	unsigned mdis:1;
	unsigned reserved15:1;
	unsigned pcsis:8;
	unsigned rooe:1;
	unsigned pcsse:1;
	unsigned mtfe:1;
	unsigned frz:1;
	unsigned dconf:2;
	unsigned cont_scke:1;
	unsigned master:1;
};

struct DSPI_CTAR {
	unsigned br:4;
	unsigned dt:4;
	unsigned asc:4;
	unsigned cssck:4;
	unsigned pbr:2;
	unsigned pdt:2;
	unsigned pasc:2;
	unsigned pcssck:2;
	unsigned lsbfe:1;
	unsigned cpha:1;
	unsigned cpol:1;
	unsigned fmsz:4;
	unsigned dbr:1;
};

struct chip_data {
	/* dspi data */
	union {
		u32 mcr_val;
		struct DSPI_MCR mcr;
	};
	union {
		u32 ctar_val;
		struct DSPI_CTAR ctar;
	};
	u16 void_write_data;
};


struct spi_mvf_data {
	/* Driver model hookup */
	struct platform_device *pdev;

	/* SPI framework hookup */
	struct spi_master *master;

	void *base;
	int irq;
	struct clk *clk;

	/* Driver message queue */
	struct workqueue_struct	*workqueue;
	struct work_struct pump_messages;
	spinlock_t lock;
	struct list_head queue;
	int busy;
	int run;

	/* Message Transfer pump */
	struct tasklet_struct pump_transfers;

	/* Current message transfer state info */
	struct spi_message *cur_msg;
	struct spi_transfer *cur_transfer;
	struct chip_data *cur_chip;
	size_t len;
	void *tx;
	void *tx_end;
	void *rx;
	void *rx_end;
	char flags;
	u8 cs;
	u16 void_write_data;
	unsigned cs_change:1;

	u32 trans_cnt;
	u32 wce_cnt;
	u32 abrt_cnt;

#if defined(SPI_DSPI_EDMA)
	void *edma_tx_buf;
	void *edma_rx_buf;
	dma_addr_t edma_tx_buf_pa;
	dma_addr_t edma_rx_buf_pa;
	int tx_chan;
	int rx_chan;
#endif

	void (*cs_control)(u8 cs, u8 command);
};


/* SPI local functions */
static void *next_transfer(struct spi_mvf_data *spi_mvf)
{
	struct spi_message *msg = spi_mvf->cur_msg;
	struct spi_transfer *trans = spi_mvf->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		spi_mvf->cur_transfer = list_entry(trans->transfer_list.next,
				struct spi_transfer, transfer_list);

		if (spi_mvf->cur_transfer->transfer_list.next
			== &msg->transfers) /* last transfer */
			spi_mvf->cur_transfer->cs_change = 1;

		return RUNNING_STATE;
	} else
		return DONE_STATE;
}

static inline int is_word_transfer(struct spi_mvf_data *spi_mvf)
{
	return ((readl(spi_mvf->base + SPI_CTAR(spi_mvf->cs)) & SPI_FRAME_BITS)
			== SPI_FRAME_BITS_8) ? 0 : 1;
}

static inline void set_8bit_transfer_mode(struct spi_mvf_data *spi_mvf)
{
	u32 temp;

	temp = readl(spi_mvf->base + SPI_CTAR(spi_mvf->cs));
	temp &= ~SPI_FRAME_BITS;
	temp |= SPI_FRAME_BITS_8;
	writel(temp, spi_mvf->base + SPI_CTAR(spi_mvf->cs));
}

static inline void set_16bit_transfer_mode(struct spi_mvf_data *spi_mvf)
{
	u32 temp;

	temp = readl(spi_mvf->base + SPI_CTAR(spi_mvf->cs));
	temp &= ~SPI_FRAME_BITS;
	temp |= SPI_FRAME_BITS_16;
	writel(temp, spi_mvf->base + SPI_CTAR(spi_mvf->cs));
}

static unsigned char hz_to_spi_baud(int pbr, int dbr, int speed_hz)
{
	 /* Valid baud rate pre-scaler values */
	int pbr_tbl[4] = {2, 3, 5, 7};
	int brs[16] = {	2,	4,	6,	8,
			16,	32,	64,	128,
			256,	512,	1024,	2048,
			4096,	8192,	16384,	32768 };
	int temp, index = 0;

	 /* table indexes out of range, go slow */
	if ((pbr < 0) || (pbr > 3) || (dbr < 0) || (dbr > 1))
		return 15;

	/* cpu core clk need to check */
	temp = ((((66000000 / 2) / pbr_tbl[pbr]) * (1 + dbr)) / speed_hz);

	while (temp > brs[index])
		if (index++ >= 15)
			break;

	return index;
}

static int write(struct spi_mvf_data *spi_mvf)
{
	int tx_count = 0;
	int tx_word = is_word_transfer(spi_mvf);
	u16 d16;
	u8  d8;
	u32 dspi_pushr = 0;
	int first = 1;
#if defined(SPI_DSPI_EDMA)
	u32 *edma_wr = (u32 *)(spi_mvf->edma_tx_buf);
#endif

	/* If we are in word mode, but only have a single byte to transfer
	 * then switch to byte mode temporarily.  Will switch back at the
	 * end of the transfer. */
	if (tx_word && ((spi_mvf->tx_end - spi_mvf->tx) == 1)) {
		spi_mvf->flags |= TRAN_STATE_WORD_ODD_NUM;
		set_8bit_transfer_mode(spi_mvf);
		tx_word = 0;
	}
	while ((spi_mvf->tx < spi_mvf->tx_end)
			&& (tx_count < DSPI_FIFO_SIZE)) {
		if (tx_word) {
			if ((spi_mvf->tx_end - spi_mvf->tx) == 1)
				break;

			if (!(spi_mvf->flags & TRAN_STATE_TX_VOID))
				d16 = *(u16 *)spi_mvf->tx;
			else
				d16 = spi_mvf->void_write_data;

			dspi_pushr = SPI_PUSHR_TXDATA(d16) |
				     SPI_PUSHR_PCS(spi_mvf->cs) |
				     SPI_PUSHR_CTAS(spi_mvf->cs) |
				     SPI_PUSHR_CONT;

			spi_mvf->tx += 2;
		} else {
			if (!(spi_mvf->flags & TRAN_STATE_TX_VOID))
				d8 = *(u8 *)spi_mvf->tx;
			else
				d8 = (u8)spi_mvf->void_write_data;

			dspi_pushr = SPI_PUSHR_TXDATA(d8) |
				     SPI_PUSHR_PCS(spi_mvf->cs) |
				     SPI_PUSHR_CTAS(spi_mvf->cs) |
				     SPI_PUSHR_CONT;

			spi_mvf->tx++;
		}

		if (spi_mvf->tx == spi_mvf->tx_end
			|| tx_count == DSPI_FIFO_SIZE - 1) {
			/* last transfer in the queue */
			dspi_pushr |= SPI_PUSHR_EOQ;
			if ((spi_mvf->cs_change)
			 && (spi_mvf->tx == spi_mvf->tx_end))
				dspi_pushr &= ~SPI_PUSHR_CONT;
		} else if (tx_word && ((spi_mvf->tx_end - spi_mvf->tx) == 1))
			dspi_pushr |= SPI_PUSHR_EOQ;

		if (first) {
			first = 0;
			dspi_pushr |= SPI_PUSHR_CTCNT; /* clear counter */
		}

#if defined(SPI_DSPI_EDMA)
		*(u32 *)edma_wr = dspi_pushr;
		edma_wr++;
#else
		writel(dspi_pushr, spi_mvf->base + SPI_PUSHR);
#endif
		tx_count++;
	}
#if defined(SPI_DSPI_EDMA)
	if (tx_count > 0) {
		mcf_edma_set_tcd_params(spi_mvf->tx_chan,
			spi_mvf->edma_tx_buf_pa,
			0x4002c034,
			MCF_EDMA_TCD_ATTR_SSIZE_32BIT
			| MCF_EDMA_TCD_ATTR_DSIZE_32BIT,
			4,		/* soff */
			4 * tx_count,	/* nbytes */
			0,		/* slast */
			1,		/* citer */
			1,		/* biter */
			0,		/* doff */
			0,		/* dlastsga */
			1,		/* major_int */
			0,		/* disable_req */
			0);		/* enable sg */

		mcf_edma_set_tcd_params(spi_mvf->rx_chan,
			0x4002c038,
			spi_mvf->edma_rx_buf_pa,
			MCF_EDMA_TCD_ATTR_SSIZE_32BIT
			| MCF_EDMA_TCD_ATTR_DSIZE_32BIT,
			0,		/* soff */
			4 * tx_count,	/* nbytes */
			0,		/* slast */
			1,		/* citer */
			1,		/* biter */
			4,		/* doff */
			0,		/* dlastsga */
			1,		/* major_int */
			0,		/* disable_req */
			0);		/* enable sg */

		mcf_edma_start_transfer(spi_mvf->tx_chan);
	}
#endif
	return tx_count * (tx_word + 1);
}

static int read(struct spi_mvf_data *spi_mvf)
{
	int rx_count = 0;
	int rx_word = is_word_transfer(spi_mvf);
	u16 d;
#if defined(SPI_DSPI_EDMA)
	u32 *rx_edma = (u32 *) spi_mvf->edma_rx_buf;

	/* receive SPI data */
	udelay(10);
	mcf_edma_start_transfer(spi_mvf->rx_chan);
	udelay(10);
#endif
	while ((spi_mvf->rx < spi_mvf->rx_end)
		&& (rx_count < DSPI_FIFO_SIZE)) {

		if (rx_word) {
			if ((spi_mvf->rx_end - spi_mvf->rx) == 1)
				break;
#if defined(SPI_DSPI_EDMA)
			d = SPI_POPR_RXDATA(*rx_edma);
			rx_edma++;
#else
			d = SPI_POPR_RXDATA(readl(spi_mvf->base + SPI_POPR));
#endif
			if (!(spi_mvf->flags & TRAN_STATE_RX_VOID))
				*(u16 *)spi_mvf->rx = d;
			spi_mvf->rx += 2;

		} else {
#if defined(SPI_DSPI_EDMA)
			d = SPI_POPR_RXDATA(*rx_edma);
			rx_edma++;
#else
			d = SPI_POPR_RXDATA(readl(spi_mvf->base + SPI_POPR));
#endif
			if (!(spi_mvf->flags & TRAN_STATE_RX_VOID))
				*(u8 *)spi_mvf->rx = d;
			spi_mvf->rx++;
		}
		rx_count++;
	}
	return rx_count;
}


static inline void dspi_setup_chip(struct spi_mvf_data *spi_mvf)
{
	struct chip_data *chip = spi_mvf->cur_chip;

	writel(chip->mcr_val, spi_mvf->base + SPI_MCR);
	writel(chip->ctar_val, spi_mvf->base + SPI_CTAR(spi_mvf->cs));

	writel(SPI_RSER_EOQFE, spi_mvf->base + SPI_RSER);
}

#if defined(SPI_DSPI_EDMA)
static struct spi_mvf_data *dspi_drv_data;

static irqreturn_t edma_tx_handler(int channel, void *dev)
{
	struct spi_mvf_data *spi_mvf = dspi_drv_data;

	if (channel == spi_mvf->tx_chan)
		mcf_edma_stop_transfer(spi_mvf->tx_chan);
	return IRQ_HANDLED;
}

static irqreturn_t edma_rx_handler(int channel, void *dev)
{
	struct spi_mvf_data *spi_mvf = dspi_drv_data;

	if (channel == spi_mvf->rx_chan) {
		mcf_edma_stop_transfer(spi_mvf->tx_chan);
		mcf_edma_stop_transfer(spi_mvf->rx_chan);
	}

	return IRQ_HANDLED;
}
#endif

static irqreturn_t dspi_interrupt(int irq, void *dev_id)
{
	struct spi_mvf_data *spi_mvf = (struct spi_mvf_data *)dev_id;
	struct spi_message *msg = spi_mvf->cur_msg;

	/* Clear all flags immediately */
	writel(SPI_SR_EOQF, spi_mvf->base + SPI_SR);

	if (!spi_mvf->cur_msg || !spi_mvf->cur_msg->state) {
#if !defined(SPI_DSPI_EDMA)
		u32 irq_status = readl(spi_mvf->base + SPI_SR);
		/* if eDMA is used it happens some time (at least once)*/
		printk(KERN_ERR "Bad message or transfer state handler. "
				"IRQ status = %x\n", irq_status);
#endif
		return IRQ_NONE;
	}

	/*
	 * Read the data into the buffer and reload and start
	 * queue with new data if not finished.  If finished
	 * then setup the next transfer
	 */
#if defined(SPI_DSPI_EDMA)
	 mcf_edma_start_transfer(spi_mvf->rx_chan);
#endif
	read(spi_mvf);

	if (spi_mvf->rx == spi_mvf->rx_end) {
		/*
		 * Finished now - fall through and schedule next
		 * transfer tasklet
		 */
		if (spi_mvf->flags & TRAN_STATE_WORD_ODD_NUM)
			set_16bit_transfer_mode(spi_mvf);

		msg->state = next_transfer(spi_mvf);
	} else {
		/* not finished yet - keep going */
		msg->actual_length += write(spi_mvf);
		return IRQ_HANDLED;
	}

	tasklet_schedule(&spi_mvf->pump_transfers);

	return IRQ_HANDLED;
}

/* caller already set message->status; dma and pio irqs are blocked */
static void giveback(struct spi_mvf_data *spi_mvf)
{
	struct spi_transfer *last_transfer;
	unsigned long flags;
	struct spi_message *msg;

	spin_lock_irqsave(&spi_mvf->lock, flags);
	msg = spi_mvf->cur_msg;
	spi_mvf->cur_msg = NULL;
	spi_mvf->cur_transfer = NULL;
	spi_mvf->cur_chip = NULL;
	queue_work(spi_mvf->workqueue, &spi_mvf->pump_messages);
	spin_unlock_irqrestore(&spi_mvf->lock, flags);

	last_transfer = list_entry(msg->transfers.prev,
				   struct spi_transfer,	transfer_list);

	if (!last_transfer->cs_change && spi_mvf->cs_control)
		spi_mvf->cs_control(spi_mvf->cs, SPI_CS_DROP);

	msg->state = NULL;
	if (msg->complete)
		msg->complete(msg->context);
}


static void pump_transfers(unsigned long data)
{
	struct spi_mvf_data *spi_mvf = (struct spi_mvf_data *)data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct chip_data *chip = NULL;
	unsigned long flags;

	/* Get current state information */
	message = spi_mvf->cur_msg;
	transfer = spi_mvf->cur_transfer;
	chip = spi_mvf->cur_chip;

	/* Handle for abort */
	if (message->state == ERROR_STATE) {
		message->status = -EIO;
		giveback(spi_mvf);
		return;
	}

	/* Handle end of message */
	if (message->state == DONE_STATE) {
		message->status = 0;
		giveback(spi_mvf);
		return;
	}

	spi_mvf->cs = message->spi->chip_select;
	spi_mvf->cs_change = transfer->cs_change;
	spi_mvf->void_write_data = chip->void_write_data;

	if (message->state == START_STATE) {
		dspi_setup_chip(spi_mvf);

		if (spi_mvf->cs_control)
			spi_mvf->cs_control(message->spi->chip_select,
				SPI_CS_ASSERT);
	}

	/* Delay if requested at end of transfer*/
	if (message->state == RUNNING_STATE) {
		previous = list_entry(transfer->transfer_list.prev,
				struct spi_transfer, transfer_list);

		if (spi_mvf->cs_control && transfer->cs_change)
			spi_mvf->cs_control(message->spi->chip_select,
				SPI_CS_DROP);

		if (previous->delay_usecs)
			udelay(previous->delay_usecs);

		if (spi_mvf->cs_control && transfer->cs_change)
			spi_mvf->cs_control(message->spi->chip_select,
				SPI_CS_ASSERT);
	}

	spi_mvf->flags = 0;
	spi_mvf->tx = (void *)transfer->tx_buf;
	spi_mvf->tx_end = spi_mvf->tx + transfer->len;
	spi_mvf->rx = transfer->rx_buf;
	spi_mvf->rx_end = spi_mvf->rx + transfer->len;

	if (!spi_mvf->rx)
		spi_mvf->flags |= TRAN_STATE_RX_VOID;

	if (!spi_mvf->tx)
		spi_mvf->flags |= TRAN_STATE_TX_VOID;

	if (transfer->speed_hz)
		writel((chip->ctar_val & ~0xf) |
			hz_to_spi_baud(chip->ctar.pbr, chip->ctar.dbr,
			transfer->speed_hz),
			spi_mvf->base + SPI_CTAR(spi_mvf->cs));

	message->state = RUNNING_STATE;

	local_irq_save(flags);
	message->actual_length += write(spi_mvf);
	local_irq_restore(flags);
}

static void pump_messages(struct work_struct *work)
{
	struct spi_mvf_data *spi_mvf;
	unsigned long flags;

	spi_mvf = container_of(work, struct spi_mvf_data, pump_messages);

	/* Lock queue and check for queue work */
	spin_lock_irqsave(&spi_mvf->lock, flags);
	if (list_empty(&spi_mvf->queue)
		|| spi_mvf->run == QUEUE_STOPPED) {
		spi_mvf->busy = 0;
		spin_unlock_irqrestore(&spi_mvf->lock, flags);
		return;
	}

	/* Make sure we are not already running a message */
	if (spi_mvf->cur_msg) {
		spin_unlock_irqrestore(&spi_mvf->lock, flags);
		return;
	}

	/* Extract head of queue */
	spi_mvf->cur_msg = list_entry(spi_mvf->queue.next,
					struct spi_message, queue);
	list_del_init(&spi_mvf->cur_msg->queue);

	/* Initial message state*/
	spi_mvf->cur_msg->state = START_STATE;
	spi_mvf->cur_transfer = list_entry(spi_mvf->cur_msg->transfers.next,
			struct spi_transfer, transfer_list);

	if (spi_mvf->cur_transfer->transfer_list.next
		== &spi_mvf->cur_msg->transfers)
		spi_mvf->cur_transfer->cs_change = 1; /* last */

	/* Setup the SPI Registers using the per chip configuration */
	spi_mvf->cur_chip = spi_get_ctldata(spi_mvf->cur_msg->spi);

	/* Mark as busy and launch transfers */
	tasklet_schedule(&spi_mvf->pump_transfers);

	spi_mvf->busy = 1;
	spin_unlock_irqrestore(&spi_mvf->lock, flags);
}

/* SPI master implementation */
static int transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct spi_mvf_data *spi_mvf = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&spi_mvf->lock, flags);

	if (spi_mvf->run == QUEUE_STOPPED) {
		spin_unlock_irqrestore(&spi_mvf->lock, flags);
		return -ESHUTDOWN;
	}

	msg->actual_length = 0;
	msg->status = -EINPROGRESS;
	msg->state = START_STATE;

	list_add_tail(&msg->queue, &spi_mvf->queue);

	if (spi_mvf->run == QUEUE_RUNNING && !spi_mvf->busy)
		queue_work(spi_mvf->workqueue, &spi_mvf->pump_messages);

	spin_unlock_irqrestore(&spi_mvf->lock, flags);

	return 0;
}


static int setup(struct spi_device *spi)
{
	struct chip_data *chip;
	struct spi_mvf_chip *chip_info
		= (struct spi_mvf_chip *)spi->controller_data;

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (chip == NULL) {
		chip = kcalloc(1, sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		spi->mode = chip_info->mode;
		spi->bits_per_word = chip_info->bits_per_word;
	}

	chip->mcr.master = 1;
	chip->mcr.cont_scke = 0;
	chip->mcr.dconf = 0;
	chip->mcr.frz = 0;
	chip->mcr.mtfe = 0;
	chip->mcr.pcsse = 0;
	chip->mcr.rooe = 0;
	chip->mcr.pcsis = 0xFF;
	chip->mcr.reserved15 = 0;
	chip->mcr.mdis = 0;
	chip->mcr.dis_tx = 0;
	chip->mcr.dis_rxf = 0;
	chip->mcr.clr_tx = 1;
	chip->mcr.clr_rxf = 1;
	chip->mcr.smpl_pt = 0;
	chip->mcr.reserved71 = 0;
	chip->mcr.halt = 0;

	if ((spi->bits_per_word >= 4) && (spi->bits_per_word <= 16)) {
		chip->ctar.fmsz = spi->bits_per_word - 1;
	} else {
		printk(KERN_ERR "DSPI: Invalid wordsize %d\n",
			spi->bits_per_word);
		kfree(chip);
		return -ENODEV;
	}

	chip->void_write_data = chip_info->void_write_data;

	if (spi->max_speed_hz != 0)
		chip_info->br = hz_to_spi_baud(chip_info->pbr, chip_info->dbr,
				spi->max_speed_hz);

	chip->ctar.cpha = (spi->mode & SPI_CPHA) ? 1 : 0;
	chip->ctar.cpol = (spi->mode & SPI_CPOL) ? 1 : 0;
	chip->ctar.lsbfe = (spi->mode & SPI_LSB_FIRST) ? 1 : 0;
	chip->ctar.dbr = chip_info->dbr;
	chip->ctar.pbr = chip_info->pbr;
	chip->ctar.br = chip_info->br;
	chip->ctar.pcssck = chip_info->pcssck;
	chip->ctar.pasc = chip_info->pasc;
	chip->ctar.pdt = chip_info->pdt;
	chip->ctar.cssck = chip_info->cssck;
	chip->ctar.asc = chip_info->asc;
	chip->ctar.dt = chip_info->dt;

	spi_set_ctldata(spi, chip);

	return 0;
}

static int init_queue(struct spi_mvf_data *spi_mvf)
{
	INIT_LIST_HEAD(&spi_mvf->queue);
	spin_lock_init(&spi_mvf->lock);

	spi_mvf->run = QUEUE_STOPPED;
	spi_mvf->busy = 0;

	tasklet_init(&spi_mvf->pump_transfers,
			pump_transfers,	(unsigned long)spi_mvf);

	INIT_WORK(&spi_mvf->pump_messages, pump_messages);

	spi_mvf->workqueue = create_singlethread_workqueue(
			dev_name(spi_mvf->master->dev.parent));
	if (spi_mvf->workqueue == NULL)
		return -EBUSY;

	return 0;
}

static int start_queue(struct spi_mvf_data *spi_mvf)
{
	unsigned long flags;

	spin_lock_irqsave(&spi_mvf->lock, flags);

	if (spi_mvf->run == QUEUE_RUNNING || spi_mvf->busy) {
		spin_unlock_irqrestore(&spi_mvf->lock, flags);
		return -EBUSY;
	}

	spi_mvf->run = QUEUE_RUNNING;
	spi_mvf->cur_msg = NULL;
	spi_mvf->cur_transfer = NULL;
	spi_mvf->cur_chip = NULL;
	spin_unlock_irqrestore(&spi_mvf->lock, flags);

	queue_work(spi_mvf->workqueue, &spi_mvf->pump_messages);

	return 0;
}

static int stop_queue(struct spi_mvf_data *spi_mvf)
{
	unsigned long flags;
	unsigned limit = 500;
	int status = 0;

	spin_lock_irqsave(&spi_mvf->lock, flags);

	/* This is a bit lame, but is optimized for the common execution path.
	 * A wait_queue on the spi_mvf->busy could be used, but then the common
	 * execution path (pump_messages) would be required to call wake_up or
	 * friends on every SPI message. Do this instead */
	spi_mvf->run = QUEUE_STOPPED;
	while (!list_empty(&spi_mvf->queue) && spi_mvf->busy && limit--) {
		spin_unlock_irqrestore(&spi_mvf->lock, flags);
		msleep(20);
		spin_lock_irqsave(&spi_mvf->lock, flags);
	}

	if (!list_empty(&spi_mvf->queue) || spi_mvf->busy)
		status = -EBUSY;

	spin_unlock_irqrestore(&spi_mvf->lock, flags);

	return status;
}

static int destroy_queue(struct spi_mvf_data *spi_mvf)
{
	int status;

	status = stop_queue(spi_mvf);
	if (status != 0)
		return status;

	destroy_workqueue(spi_mvf->workqueue);

	return 0;
}


static void cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata((struct spi_device *)spi);

	dev_dbg(&spi->dev, "spi_device %u.%u cleanup\n",
		spi->master->bus_num, spi->chip_select);

	kfree(chip);
}

/* Generic Device driver routines and interface implementation */
static int spi_mvf_probe(struct platform_device *pdev)
{
	struct spi_mvf_master *platform_info;
	struct spi_master *master;
	struct spi_mvf_data *spi_mvf;
	struct resource *res;
	int ret = 0;
	int i;

	platform_info = dev_get_platdata(&pdev->dev);
	if (!platform_info) {
		dev_err(&pdev->dev, "can't get the platform data\n");
		return -EINVAL;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_mvf_data));
	if (!master)
		return -ENOMEM;

	spi_mvf = spi_master_get_devdata(master);
	spi_mvf->master = master;

	INIT_LIST_HEAD(&spi_mvf->queue);
	spin_lock_init(&spi_mvf->lock);

	master->bus_num = platform_info->bus_num;
	master->num_chipselect = platform_info->num_chipselect;
	master->cleanup = cleanup;
	master->setup = setup;
	master->transfer = transfer;
	master->mode_bits = SPI_MODE_3;

	spi_mvf->cs_control = platform_info->cs_control;
	if (spi_mvf->cs_control)
		for (i = 0; i < master->num_chipselect; i++)
			spi_mvf->cs_control(i, SPI_CS_INIT | SPI_CS_DROP);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get platform resource\n");
		ret = -ENOMEM;
		goto out_error_master_alloc;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto out_error_master_alloc;
	}

	spi_mvf->base = ioremap(res->start, resource_size(res));
	if (!spi_mvf->base) {
		ret = EINVAL;
		goto out_error_release_mem;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get platform irq\n");
		ret = -ENOMEM;
		goto out_error_iounmap;
	}
	spi_mvf->irq = res->start;

	ret = request_irq(spi_mvf->irq, dspi_interrupt, IRQF_DISABLED,
			pdev->name, spi_mvf);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Unable to attach ColdFire DSPI interrupt\n");
		goto out_error_iounmap;
	}

	/* Initial and start queue */
	ret = init_queue(spi_mvf);
	if (ret != 0) {
		dev_err(&pdev->dev, "Problem initializing DSPI queue\n");
		goto out_error_irq_alloc;
	}
	ret = start_queue(spi_mvf);
	if (ret != 0) {
		dev_err(&pdev->dev, "Problem starting DSPI queue\n");
		goto out_error_irq_alloc;
	}

	spi_mvf->clk = clk_get(&pdev->dev, "kinetis-dspi");
	if (IS_ERR(spi_mvf->clk)) {
		dev_err(&pdev->dev, "unable to get clock\n");
		goto out_error_irq_alloc;
	}
	clk_enable(spi_mvf->clk);

#if defined(SPI_DSPI_EDMA)
	spi_mvf->edma_tx_buf = dma_alloc_coherent(NULL, EDMA_BUFSIZE_KMALLOC,
			&spi_mvf->edma_tx_buf_pa, GFP_DMA);
	if (!spi_mvf->edma_tx_buf) {
		dev_dbg(&pdev->dev, "cannot allocate eDMA TX memory\n");
		goto out_error_master_alloc;
	}
	spi_mvf->edma_rx_buf = dma_alloc_coherent(NULL, EDMA_BUFSIZE_KMALLOC,
			&spi_mvf->edma_rx_buf_pa, GFP_DMA);
	if (!spi_mvf->edma_rx_buf) {
		dma_free_coherent(NULL, EDMA_BUFSIZE_KMALLOC, \
				(void *)spi_mvf->edma_tx_buf,
				spi_mvf->edma_tx_buf_pa);
		dev_dbg(&pdev->dev, "cannot allocate eDMA RX memory\n");
		goto out_error_master_alloc;
	}
	printk(KERN_INFO "Faraday DSPI DMA addr: Tx-0x%p[0x%x],"
			" Rx-0x%p[0x%x]\n",
			spi_mvf->edma_tx_buf, spi_mvf->edma_tx_buf_pa,
			spi_mvf->edma_rx_buf, spi_mvf->edma_rx_buf_pa);

	spi_mvf->tx_chan = mcf_edma_request_channel(DSPI_DMA_TX_TCD,
		edma_tx_handler, NULL, 0x00, pdev, NULL, DRIVER_NAME);
	if (spi_mvf->tx_chan < 0) {
		dev_err(&pdev->dev, "eDMA transmit channel request\n");
		ret = -EINVAL;
		goto out_error_queue_alloc;
	}
/*
 * we only need RX eDMA interrupt to sync a spi transfer,
 * the Tx eDMA interrupt can be ignored, this is determined
 * by SPI communicate machnisim, i.e, is half duplex mode, that is
 * whether read or write, we need write data out to get we wanted.
 */
	spi_mvf->rx_chan = mcf_edma_request_channel(DSPI_DMA_RX_TCD,
		edma_rx_handler, NULL, 0x06, pdev, NULL, DRIVER_NAME);
	if (spi_mvf->rx_chan < 0) {
		dev_err(&pdev->dev, "eDAM receive channel request\n");
		ret = -EINVAL;
		mcf_edma_free_channel(spi_mvf->tx_chan, pdev);
		goto out_error_queue_alloc;
	}

	dspi_drv_data = spi_mvf;
#endif

	/* Register with the SPI framework */
	platform_set_drvdata(pdev, spi_mvf);
	ret = spi_register_master(master);
	if (ret != 0) {
		dev_err(&pdev->dev, "Problem registering DSPI master [ret=%d]\n", ret);
		ret = -EINVAL;
		goto out_error_queue_alloc;
	}

	printk(KERN_INFO "DSPI: controller %d at hz=%ld,irq=%d\n",
		master->bus_num, clk_get_rate(spi_mvf->clk), spi_mvf->irq);
	return ret;

out_error_queue_alloc:
	destroy_queue(spi_mvf);
out_error_irq_alloc:
	free_irq(spi_mvf->irq, spi_mvf);
out_error_iounmap:
	iounmap(spi_mvf->base);
out_error_release_mem:
	release_mem_region(res->start, resource_size(res));
out_error_master_alloc:
	spi_master_put(master);
	return ret;
}

static int spi_mvf_remove(struct platform_device *pdev)
{
	struct spi_mvf_data *spi_mvf = platform_get_drvdata(pdev);
	struct resource *res;
	int irq;
	int ret = 0;

	clk_disable(spi_mvf->clk);
	clk_put(spi_mvf->clk);

	if (!spi_mvf)
		return 0;

#if defined(SPI_DSPI_EDMA)
	mcf_edma_free_channel(spi_mvf->tx_chan, pdev);
	mcf_edma_free_channel(spi_mvf->rx_chan, pdev);
#endif

	/* Remove the queue */
	ret = destroy_queue(spi_mvf);
	if (ret != 0)
		return ret;

	/* Release IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq >= 0)
		free_irq(irq, spi_mvf);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	release_mem_region(res->start, resource_size(res));
	iounmap(spi_mvf->base);

	/* Disconnect from the SPI framework */
	spi_unregister_master(spi_mvf->master);
	spi_master_put(spi_mvf->master);

	/* Prevent double remove */
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void spi_mvf_shutdown(struct platform_device *pdev)
{
	int ret = spi_mvf_remove(pdev);

	if (ret != 0)
		dev_err(&pdev->dev, "shutdown failed with %d\n", ret);
}


#ifdef CONFIG_PM
static int spi_mvf_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct spi_mvf_data *spi_mvf = platform_get_drvdata(pdev);

	clk_disable(spi_mvf->clk);

	return 0;
}

static int spi_mvf_resume(struct platform_device *pdev)
{
	struct spi_mvf_data *spi_mvf = platform_get_drvdata(pdev);

	clk_enable(spi_mvf->clk);

	return 0;
}
#else
#define spi_mvf_suspend		NULL
#define spi_mvf_resume		NULL
#endif /* CONFIG_PM */

static struct platform_driver driver = {
	.driver = {
		.name = DRIVER_NAME,
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = spi_mvf_probe,
	.remove = __devexit_p(spi_mvf_remove),
	.shutdown = spi_mvf_shutdown,
	.suspend = spi_mvf_suspend,
	.resume = spi_mvf_resume,
};

static int __init spi_mvf_init(void)
{
	platform_driver_register(&driver);

	return 0;
}
module_init(spi_mvf_init);

static void __exit spi_mvf_exit(void)
{
	platform_driver_unregister(&driver);
}
module_exit(spi_mvf_exit);

MODULE_AUTHOR("Alison Wang");
MODULE_DESCRIPTION("Faraday DSPI Contoller");
MODULE_LICENSE("GPL");
