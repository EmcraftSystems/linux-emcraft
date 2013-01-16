/*
 * Device driver for the SPI controller of the SmartFusion2 SoC.
 * Author: Yuri Tikhonov, Emcraft Systems, yur@emcraft.com
 * Copyright 2012-2013 Emcraft Systems
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/m2s.h>
#include <mach/platform.h>
#include <mach/spi.h>

/*
 * Maximum size we allow per xfer (limited with txrxdf_size register)
 */
#define M2S_SPI_MAX_LEN			65535

/*
 * Some bits in various regs
 */
#define M2S_SYS_SOFT_RST_CR_SPI1	(1 << 10)
#define M2S_SYS_SOFT_RST_CR_SPI0	(1 << 9)
#define M2S_SYS_SOFT_RST_CR_PDMA	(1 << 5)

#define SPI_CONTROL_ENABLE		(1 << 0)
#define SPI_CONTROL_MASTER		(1 << 1)
#define SPI_CONTROL_PROTO_MSK		(3 << 2)
#define SPI_CONTROL_PROTO_MOTO		(0 << 2)
#define SPI_CONTROL_CNT_MSK		(0xffff << 8)
#define SPI_CONTROL_CNT_SHF		(8)
#define SPI_CONTROL_SPO			(1 << 24)
#define SPI_CONTROL_SPH			(1 << 25)
#define SPI_CONTROL_SPS			(1 << 26)
#define SPI_CONTROL_BIGFIFO		(1 << 29)
#define SPI_CONTROL_CLKMODE		(1 << 28)
#define SPI_CONTROL_RESET		(1 << 31)

/*
 * PDMA register bits
 */
#define PDMA_CONTROL_PER_SEL_SPI0_RX	(0x4 << 23)
#define PDMA_CONTROL_PER_SEL_SPI0_TX	(0x5 << 23)
#define PDMA_CONTROL_PER_SEL_SPI1_RX	(0x6 << 23)
#define PDMA_CONTROL_PER_SEL_SPI1_TX	(0x7 << 23)
/*
 * TBD: calculte ADJ value dynamically, basing on SPI clk value?
 * With smaller values we just hang-up
 */
#define PDMA_CONTROL_WRITE_ADJ		(0xF << 14)

#define PDMA_CONTROL_DST_ADDR_INC_MSK	(0x3 << 12)
#define PDMA_CONTROL_DST_ADDR_INC_0	(0x0 << 12)
#define PDMA_CONTROL_DST_ADDR_INC_1	(0x1 << 12)
#define PDMA_CONTROL_SRC_ADDR_INC_MSK	(0x3 << 10)
#define PDMA_CONTROL_SRC_ADDR_INC_0	(0x0 << 10)
#define PDMA_CONTROL_SRC_ADDR_INC_1	(0x1 << 10)
#define PDMA_CONTROL_CLR_B		(1 << 8)
#define PDMA_CONTROL_CLR_A		(1 << 7)
#define PDMA_CONTROL_INTEN		(1 << 6)
#define PDMA_CONTROL_RESET		(1 << 5)
#define PDMA_CONTROL_PAUSE		(1 << 4)
#define PDMA_CONTROL_XFER_SIZE_1B	(0x0 << 2)
#define PDMA_CONTROL_DIR		(1 << 1)
#define PDMA_CONTROL_PERIPH		(1 << 0)

#define PDMA_STATUS_BUF_SEL		(1 << 2)
#define PDMA_STATUS_CH_COMP_B	(1 << 1)
#define PDMA_STATUS_CH_COMP_A	(1 << 0)

/*
 * Access handle for the control registers
 */
#define M2S_SPI(s)	((volatile struct m2s_spi_regs *)(s->spi_regs))
#define M2S_PDMA(s)	((volatile struct m2s_pdma_regs *)(s->pdma_regs))

/*
 * Private data structure for an SPI slave
 */
struct m2s_spi_dsc {
	void			*spi_regs;	/* SPI registers base	*/
	void			*pdma_regs;	/* DMA registers base	*/
	struct spi_device	*slave;		/* Generic slave	*/
	int				irq;		/* DMA IRQ		*/

	int			bus;		/* SPI bus ID		*/
	struct list_head	queue;		/* Message queue	*/
	spinlock_t		lock;		/* Exclusive access	*/
	struct work_struct	work;		/* Worker		*/

	struct spi_message	*msg;		/* Current msg		*/
	struct spi_transfer	*xf;		/* Current transfer	*/

	u32			rst_clr;	/* Reset clear mask	*/
	u32			drx_sel;	/* Rx PDMA peripheral	*/
	u32			dtx_sel;	/* Tx PDMA peripheral	*/

	u32			ref_clk;	/* SPI controller clock	*/
	u8			drx;		/* Rx PDMA channel	*/
	u8			dtx;		/* Tx PDMA channel	*/

	u8			stopping;	/* Stopping status	*/
	u8			dummy;		/* Dummy buffer		*/
};

/*
 * Description of the the SmartFusion SPI hardware interfaces.
 * This is a 1-to-1 mapping of Actel's documentation onto a C structure.
 * Refer to SmartFusion Data Sheet for details.
 */
struct m2s_spi_regs {
	u32	control;
	u32	txrxdf_size;
	u32	status;
	u32	int_clear;
	u32	rx_data;
	u32	tx_data;
	u32	clk_gen;
	u32	slave_select;
	u32	mis;
	u32	ris;
};

 /*
  * Peripheral DMA registers
  */
struct m2s_pdma_regs {
	u32	ratio;
	u32	status;
	u32	reserved[(0x20 - 0x08) >> 2];
	struct m2s_pdma_chan {
		u32	control;
		u32	status;
		struct {
			u32	src;
			u32	dst;
			u32	cnt;
		} buf[2];			/* Buffers A-B	*/
	} chan[8];				/* Channels 0-7	*/
};

/*
 * Hardware initialization of the SPI controller
 * @s		controller data structure
 * @returns	0->success, <0->error code
 */
static int spi_m2s_hw_init(struct m2s_spi_dsc *s)
{
	unsigned int ret = 0;

	/*
	 * Reset the MSS SPI controller and then bring it out of reset
	 */
	M2S_SYSREG->soft_reset_cr |= s->rst_clr;
	M2S_SYSREG->soft_reset_cr &= ~s->rst_clr;

	/*
	 * Set the master mode
	 */
	M2S_SPI(s)->control |= SPI_CONTROL_MASTER;

	/*
	 * Set the transfer protocol. We are using the Motorola
	 * SPI mode, with no user interface to configure it to
	 * some other mode.
	 */
	M2S_SPI(s)->control &= ~SPI_CONTROL_PROTO_MSK;
	M2S_SPI(s)->control |= SPI_CONTROL_PROTO_MOTO;

	/*
	 * Set-up the controller in such a way that it doesn't remove
	 * Chip Select until the entire message has been transferred,
	 * even if at some points TX FIFO becomes empty.
	 */
	M2S_SPI(s)->control |= SPI_CONTROL_SPS | SPI_CONTROL_BIGFIFO |
			       SPI_CONTROL_CLKMODE;

	/*
	 * Enable the SPI controller
	 * It is critical to clear RESET in the control bit.
	 */
	M2S_SPI(s)->control &= ~SPI_CONTROL_RESET;
	M2S_SPI(s)->control |= SPI_CONTROL_ENABLE;

	/*
	 * Configure DMA (preliminary)
	 */
	M2S_SYSREG->soft_reset_cr &= ~M2S_SYS_SOFT_RST_CR_PDMA;

	M2S_PDMA(s)->chan[s->drx].control = PDMA_CONTROL_RESET |
					    PDMA_CONTROL_CLR_B |
					    PDMA_CONTROL_CLR_A;
	M2S_PDMA(s)->chan[s->drx].control |= PDMA_CONTROL_WRITE_ADJ |
					     PDMA_CONTROL_SRC_ADDR_INC_0 |
					     PDMA_CONTROL_XFER_SIZE_1B;
	M2S_PDMA(s)->chan[s->drx].control |= s->drx_sel |
					     PDMA_CONTROL_PERIPH;
	M2S_PDMA(s)->chan[s->drx].control |= PDMA_CONTROL_INTEN;

	M2S_PDMA(s)->chan[s->dtx].control = PDMA_CONTROL_RESET |
					    PDMA_CONTROL_CLR_B |
					    PDMA_CONTROL_CLR_A;
	M2S_PDMA(s)->chan[s->dtx].control |= PDMA_CONTROL_WRITE_ADJ |
					     PDMA_CONTROL_DST_ADDR_INC_0 |
					     PDMA_CONTROL_XFER_SIZE_1B;
	M2S_PDMA(s)->chan[s->dtx].control |= s->dtx_sel |
					     PDMA_CONTROL_DIR |
					     PDMA_CONTROL_PERIPH;
	/* Do not enable interrupts on the TX channel.
	   Control completion of the transfer by the RX interrupt only. */
	M2S_PDMA(s)->chan[s->dtx].control &= ~PDMA_CONTROL_INTEN;

	return ret;
}

/*
 * Get the number of chip selects supported by this controller
 * @s		controller data structure
 * @returns	max chip select
 */
static int spi_m2s_hw_cs_max(struct m2s_spi_dsc *s)
{
	return 8;
}

/*
 * Set chip select
 * @s		slave
 * @cs		chip select: [0..7]->slave, otherwise->deselect all
 * @returns	0->good,!=0->bad
 */
static inline int spi_m2s_hw_cs_set(struct m2s_spi_dsc *s, int cs)
{
	unsigned int v = (0 <= cs && cs <= 7) ? (1 << cs) : 0;

	M2S_SPI(s)->slave_select = v;

	return 0;
}

/*
 * Set controller clock rate
 * @s		slave
 * @spd		clock rate in Hz
 * @returns	0->good,!=0->bad
 */
static inline int spi_m2s_hw_clk_set(struct m2s_spi_dsc *s, unsigned int spd)
{
	int i;
	unsigned int h;
	int ret = 0;

	/*
	 * Calculate the clock rate that works for this slave
	 */
	h = s->ref_clk;
	for (i = 0; i <= 255; i++) {
		if (h / ((i + 1) << 1) <= spd)
			break;
	}

	/*
	 * Can't provide a rate that is slow enough for the slave
	 */
	if (i > 255) {
		ret = -EINVAL;
		goto done;
	}

	/*
	 * Set the clock rate
	 */
	M2S_SPI(s)->clk_gen = i;
done:
	return ret;
}

/*
 * Check frame size
 * @s		controller data structure
 * @bt		frame size
 * @returns	0->good,!=0->bad
 */
static inline int spi_m2s_hw_bt_check(struct m2s_spi_dsc *s, int bt)
{
	/*
	 * TO-DO: add support for frames that are not 8 bits
	 */
	return (8 <= bt && bt <= 16) ? 0 : 1;
}

/*
 * Set frame size (making an assumption that the supplied size is
 * supported by this controller)
 * @s		slave
 * @bt		frame size
 * @returns	0->good,!=0->bad
 */
static inline int spi_m2s_hw_bt_set(struct m2s_spi_dsc *s, int bt)
{
	int ret = 0;

	/*
	 * Disable the SPI controller. Writes to data frame size have
	 * no effect when the controller is enabled.
	 */
	M2S_SPI(s)->control &= ~SPI_CONTROL_ENABLE;

	/*
	 * Set the new data frame size.
	 */
	M2S_SPI(s)->txrxdf_size = bt;

	/*
	 * Re-enable the SPI controller
	 */
	M2S_SPI(s)->control |= SPI_CONTROL_ENABLE;

	return ret;
}

/*
 * Set transfer length
 * @s		slave
 * @len		transfer size
 */
static inline void spi_m2s_hw_tfsz_set(struct m2s_spi_dsc *s, int len)
{
	/*
	 * Disable the SPI controller. Writes to transfer length have
	 * no effect when the controller is enabled.
	 */
	M2S_SPI(s)->control &= ~SPI_CONTROL_ENABLE;

	/*
	 * Set the new data frame size.
	 */
	M2S_SPI(s)->control &= ~SPI_CONTROL_CNT_MSK;
	M2S_SPI(s)->control |= len << SPI_CONTROL_CNT_SHF;

	/*
	 * Re-enable the SPI controller
	 */
	M2S_SPI(s)->control |= SPI_CONTROL_ENABLE;
}

/*
 * Set SPI mode
 * @s		slave
 * @mode	mode
 * @returns	0->good;!=0->bad
 */
static inline int spi_m2s_hw_mode_set(struct m2s_spi_dsc *s, unsigned int mode)
{
	/*
	 * Set the mode
	 */
	if (mode & SPI_CPHA)
		M2S_SPI(s)->control |= SPI_CONTROL_SPH;
	else
		M2S_SPI(s)->control &= ~SPI_CONTROL_SPH;

	if (mode & SPI_CPOL)
		M2S_SPI(s)->control |= SPI_CONTROL_SPO;
	else
		M2S_SPI(s)->control &= ~SPI_CONTROL_SPO;

	return 0;
}

/*
 * Shut down the SPI controller
 * @s		SPI slave
 */
static void spi_m2s_hw_release(struct m2s_spi_dsc *s)
{
	/*
	 * Reset DMAs
	 */
	M2S_PDMA(s)->chan[s->drx].control = PDMA_CONTROL_RESET;
	M2S_PDMA(s)->chan[s->dtx].control = PDMA_CONTROL_RESET;

	/*
	 * Disable the SPI controller
	 */
	M2S_SPI(s)->control &= ~SPI_CONTROL_ENABLE;

	/*
	 * Put the SPI controller into reset
	 */
	M2S_SYSREG->soft_reset_cr |= s->rst_clr;
}

/*
 * Prepare to transfer to a slave
 * @s		controller data structure
 * @dev		slave data structure
 * @returns	0->success, <0->error code
 */
static int spi_m2s_prepare_for_slave(struct m2s_spi_dsc *s,
				     struct spi_device *dev)
{
	unsigned int spd;
	int ret = 0;

	/*
	 * Set for this slave: frame size, clock, slave select, mode
	 */
	if (spi_m2s_hw_bt_set(s, dev->bits_per_word)) {
		dev_err(&s->slave->dev, "unsupported frame size: %d\n",
			dev->bits_per_word);
		ret = -EINVAL;
		goto done;
	}
	if (spi_m2s_hw_clk_set(s, spd = min(dev->max_speed_hz, s->ref_clk))) {
		dev_err(&s->slave->dev, "slave rate too low: %d\n", spd);
		ret = -EINVAL;
		goto done;
	}
	if (spi_m2s_hw_cs_set(s, dev->chip_select)) {
		dev_err(&s->slave->dev, "incorrect chip select: %d\n",
			dev->chip_select);
		ret = -EINVAL;
		goto done;
	}
	if (spi_m2s_hw_mode_set(s, dev->mode)) {
		dev_err(&s->slave->dev, "unsupported mode: %x\n", dev->mode);
		ret = -EINVAL;
		goto done;
	}

done:
	return ret;
}

/*
 * Initiate an SPI transfer
 * @s          controller data structure
 */
static void spi_m2s_xfer_start(struct m2s_spi_dsc *s)
{
	volatile struct m2s_pdma_chan	*chan;
	int				brx, btx;
	void				*p;
	struct spi_device *dev = s->msg->spi;
	int wb = (dev->bits_per_word + 7) / 8;
	int len;
	int ret = 0;

	if (s->xf == NULL) {
		/* Should never come here. */
		goto done;
	}

	/*
	 * We don't use double buffering scheme, because we should be able to
	 * change ADDR_INC value in each msg->transfer transaction (to set it to
	 * zero in cases of dummy tx/rx (null tx_buf/rx_buf value).
	 * Note, bad address (null) can't be used as src/dst; in this case PDMA
	 * just terminate execution.
	 * Below we use different vars for indexing in A/B bufs of TX & RX DMAs
	 * (brx and btx), though actually these are always the same; so do such
	 * way just for more clearance
	 */

	/*
	 * Set-up RX
	 */
	chan = &M2S_PDMA(s)->chan[s->drx];
	brx = !!(chan->status & PDMA_STATUS_BUF_SEL);
	if (s->xf->rx_buf) {
		p = s->xf->rx_buf;
		chan->control &= ~PDMA_CONTROL_DST_ADDR_INC_MSK;
		chan->control |= PDMA_CONTROL_DST_ADDR_INC_1;
	} else {
		p = &s->dummy;
		chan->control &= ~PDMA_CONTROL_DST_ADDR_INC_MSK;
		chan->control |= PDMA_CONTROL_DST_ADDR_INC_0;
	}
	chan->buf[brx].src = (u32)&M2S_SPI(s)->rx_data;
	chan->buf[brx].dst = (u32)p;

	/*
	 * Set-up TX
	 */
	chan = &M2S_PDMA(s)->chan[s->dtx];
	btx = !!(chan->status & PDMA_STATUS_BUF_SEL);
	if (s->xf->tx_buf) {
		p = (void *)s->xf->tx_buf;
		chan->control &= ~PDMA_CONTROL_SRC_ADDR_INC_MSK;
		chan->control |= PDMA_CONTROL_SRC_ADDR_INC_1;
	} else {
		p = &s->dummy;
		chan->control &= ~PDMA_CONTROL_SRC_ADDR_INC_MSK;
		chan->control |= PDMA_CONTROL_SRC_ADDR_INC_0;
	}
	chan->buf[btx].src = (u32)p;
	chan->buf[btx].dst = (u32)&M2S_SPI(s)->tx_data;

	if ((s->msg->actual_length + s->xf->len) / wb > M2S_SPI_MAX_LEN) {
		len = M2S_SPI_MAX_LEN - (s->msg->actual_length / wb);
	} else {
		len = s->xf->len / wb;
	}

	/* Start TX and then RX. */
	M2S_PDMA(s)->chan[s->dtx].buf[btx].cnt = len;
	M2S_PDMA(s)->chan[s->drx].buf[brx].cnt = len;
 done:
	return;
}

/*
 * Transfer a message
 * @s		controller data structure
 * @msg		message
 * @returns	0 -> success, negative error code -> error
 */
static int spi_m2s_handle_message(struct m2s_spi_dsc *s)
{
	struct spi_message *msg = s->msg;
	struct spi_device *dev = msg->spi;
	int ret = 0;
	struct spi_transfer *t;
	int xfer_ttl = 0;
	int wb = (dev->bits_per_word + 7) / 8;

	/*
	 * Check if the current slave of this controller is
	 * the message's SPI device, and if not, re-set
	 * the speed and chip select for the message's SPI device.
	 */
	if (s->slave != dev) {
		s->slave = dev;
		ret = spi_m2s_prepare_for_slave(s, dev);
		if (ret) {
			s->slave = NULL;
			goto done;
		}
	}

	/*
	 * We can't provide persistent TxFIFO data flow even with PDMA, so to
	 * avoid resetting #SS - set up frame counter
	 */
	list_for_each_entry(t, &msg->transfers, transfer_list)
		xfer_ttl += t->len;
	xfer_ttl /= wb;

	if (xfer_ttl > M2S_SPI_MAX_LEN) {
		xfer_ttl = M2S_SPI_MAX_LEN;
	}

	spi_m2s_hw_tfsz_set(s, xfer_ttl);

	/*
	 * Initiate first transfer.
	 */
	s->xf = list_entry(msg->transfers.next,
					   struct spi_transfer, transfer_list);

	spi_m2s_xfer_start(s);
 done:
	return ret;
}

/*
 * Handle a ping to the workqueue
 * @w	work data structure
 */
static void spi_m2s_handle_queue(struct work_struct *w)
{
	struct m2s_spi_dsc *s = container_of(w, struct m2s_spi_dsc, work);
	unsigned long f;
	int ret;

	spin_lock_irqsave(&s->lock, f); /* lock s->msg and s->queue */
	if (s->msg) {
		/* Current message is not completely transferred yet. */
		spin_unlock_irqrestore(&s->lock, f);
		goto out;
	}

	while (!list_empty(&s->queue)) {
		/*
		 * Extract the next message from the queue.
		 */
		s->msg = list_entry(s->queue.next, struct spi_message, queue);
		list_del_init(&s->msg->queue);
		spin_unlock_irqrestore(&s->lock, f);
		/*
		 * Start message transfer over SPI wires.
		 */
		s->msg->actual_length = 0;
		ret = spi_m2s_handle_message(s);
		if (ret == 0) {
			goto out;
		}
		/*
		 * Transfer failed (has not been started).
		 */
		s->msg->status = ret;
		s->msg->complete(s->msg->context);
		s->msg = NULL;
		spin_lock_irqsave(&s->lock, f); /* lock s->queue */
	}
	spin_unlock_irqrestore(&s->lock, f);
 out:
	return;
}

/*
 * DMA interrupt handler
 */
static irqreturn_t spi_m2s_irq_cb(int irq, void *ptr)
{
	struct m2s_spi_dsc				*s = ptr;
	volatile struct m2s_pdma_chan	*chan;
	struct spi_message *msg = s->msg;
	struct spi_transfer *xf = s->xf;
	int wb = (((struct spi_device *)s->msg->spi)->bits_per_word + 7) / 8;

	/*
	 * Clear statuses
	 */
	chan = &M2S_PDMA(s)->chan[s->drx];
	if (chan->status & PDMA_STATUS_CH_COMP_A) {
		chan->control |= PDMA_CONTROL_CLR_A;
	} else if (chan->status & PDMA_STATUS_CH_COMP_B) {
		chan->control |= PDMA_CONTROL_CLR_B;
	}

	chan = &M2S_PDMA(s)->chan[s->dtx];
	if (chan->status & PDMA_STATUS_CH_COMP_A) {
		chan->control |= PDMA_CONTROL_CLR_A;
	} else if (chan->status & PDMA_STATUS_CH_COMP_B) {
		chan->control |= PDMA_CONTROL_CLR_B;
	}

	/* Complete trasfer. */
	msg->actual_length += xf->len;
	if ((msg->actual_length / wb) >= M2S_SPI_MAX_LEN) {
		/* Can't send more, so truncate the message. */
		msg->actual_length = M2S_SPI_MAX_LEN * wb;
		xf = NULL;
	} else if (xf->transfer_list.next != NULL &&
			   xf->transfer_list.next != &msg->transfers) {
		xf = list_entry(xf->transfer_list.next, struct spi_transfer, transfer_list);
	} else {
		xf = NULL;
	}
	spin_lock(&s->lock); /* lock s->xf and s->msg */
	s->xf = xf;
	if (s->xf == NULL) {
		/* Current message is completely transferred. */
		s->msg = NULL;
		spin_unlock(&s->lock);
		msg->status = 0;
		msg->complete(msg->context);
		/* Current transfer is complete. Schedule the next one. */
		schedule_work(&s->work);
		goto done;
	} else {
		spin_unlock(&s->lock);
		/* Start next transfer from the current message. */
		spi_m2s_xfer_start(s);
		goto done;
	}

	spin_unlock(&s->lock);

 done:
	return IRQ_HANDLED;
}

/*
 * Set up the SPI controller for a specified SPI slave device
 * @dev		SPI slave
 * @returns	0->success, <0->error code
 */
static int spi_m2s_setup(struct spi_device *dev)
{
	struct m2s_spi_dsc *s = spi_master_get_devdata(dev->master);
	int ret = 0;

	/*
	 * Check that the controller is still running
	 */
	if (s->stopping) {
		ret = -ESHUTDOWN;
		goto done;
	}

	/*
	 * Check the width of transfer for this device
	 */
	if (spi_m2s_hw_bt_check(s, dev->bits_per_word)) {
		dev_err(&dev->dev, "unsupported bits per word %d\n",
			dev->bits_per_word);
		ret = -EINVAL;
		goto done;
	}

	/*
	 * Don't remember the current slave. When transferring
	 * a message, we will check if the current slave is
	 * the message's SPI device, and if not, will re-set
	 * the speed and chip select for the message's SPI device.
	 */
	s->slave = NULL;
done:
	return ret;
}

/*
 * Clean up the SPI controller after a specified SPI slave device
 * @dev		SPI slave
 */
static void spi_m2s_cleanup(struct spi_device *dev)
{
	/*
	 * TBD
	 */
}

/*
 * Transfer a message to a specified SPI slave
 * @dev		SPI slave
 * @msg		message to transfer
 * @returns	0->success; <0->error code
 */
static int spi_m2s_transfer(struct spi_device *dev, struct spi_message *msg)
{
	struct m2s_spi_dsc *s = spi_master_get_devdata(dev->master);
	unsigned long f;
	int ret = 0;

	/*
	 * Check that the controller is still running
	 */
	if (s->stopping) {
		ret = -ESHUTDOWN;
		goto done;
	}

	/*
	 * Message content sanity check
	 */
	if (unlikely(list_empty(&msg->transfers))) {
		ret = -EINVAL;
		goto done;
	}

	/*
	 * Prepare it he message for transfer
	 */
	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	/*
	 * Add the message to the message queue
	 */
	spin_lock_irqsave(&s->lock, f);
	list_add_tail(&msg->queue, &s->queue);
	spin_unlock_irqrestore(&s->lock, f);

	schedule_work(&s->work);
done:
	return ret;
}

/*
 * Instantiate a new instance of the SPI controller
 * @dev		SPI controller platform device
 * @returns	0->success, <0->error code
 */
static int __devinit spi_m2s_probe(struct platform_device *pdev)
{
	struct spi_m2s_platform_data *pd;
	struct spi_master *m = NULL;
	struct m2s_spi_dsc *s = NULL;
	struct resource *spi_regs, *pdma_regs;
	int bus, irq;
	int ret = 0;

	/*
	 * Get the bus # from the platform device:
	 * [0,1]->hard-core SPI controller of SmartFusion;
	 * [2-9]->soft-IP SPI controller specific to a custom design.
	 */
	bus = pdev->id;
	if (! (0 <= bus && bus <= 10)) {
		dev_err(&pdev->dev, "invalid SPI controller %d\n", bus);
		ret = -ENXIO;
		goto error_release_nothing;
	}

	/*
	 * Get the register bases, and IRQ from the platform device
	 */
	spi_regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!spi_regs) {
		dev_err(&pdev->dev, "no register base for SPI controller %d\n",
			bus);
		ret = -ENXIO;
		goto error_release_nothing;
	}

	pdma_regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pdma_regs) {
		dev_err(&pdev->dev, "no register base for PDMA controller %d\n",
			bus);
		ret = -ENXIO;
		goto error_release_nothing;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "invalid IRQ %d for SPI controller %d\n", irq, bus);
		ret = -ENXIO;
		goto error_release_nothing;
	}

	/*
	 * Allocate an SPI master
	 */
	m = spi_alloc_master(&pdev->dev, sizeof *s);
	if (!m) {
		dev_err(&pdev->dev, "unable to allocate master for "
			"SPI controller %d\n", bus);
		ret = -ENOMEM;
		goto error_release_nothing;
	}

	/*
	 * Pointer the controller-specific data structure
	 */
	s = spi_master_get_devdata(m);

	/*
	 * Set up the bus number so that platform
	 * can set up SPI slave devices on this bus
	 */
	m->bus_num = bus;
	s->bus = bus;

	/*
	 * Get platform data
	 */
	pd = platform_get_drvdata(pdev);
	s->ref_clk = pd->ref_clk;
	s->drx = pd->dma_rx;
	s->dtx = pd->dma_tx;

	/*
	 * Set-up SPI controller index specific masks
	 */
	switch (bus) {
	case 0:
		s->drx_sel = PDMA_CONTROL_PER_SEL_SPI0_RX;
		s->dtx_sel = PDMA_CONTROL_PER_SEL_SPI0_TX;
		s->rst_clr = M2S_SYS_SOFT_RST_CR_SPI0;
		break;
	case 1:
		s->drx_sel = PDMA_CONTROL_PER_SEL_SPI1_RX;
		s->dtx_sel = PDMA_CONTROL_PER_SEL_SPI1_TX;
		s->rst_clr = M2S_SYS_SOFT_RST_CR_SPI1;
		break;
	}

	/*
	 * Map in the controller registers
	 */
	s->spi_regs = ioremap(spi_regs->start, resource_size(spi_regs));
	if (!s->spi_regs) {
		dev_err(&pdev->dev, "unable to map registers for "
			"SPI controller %d, base=%08x\n", bus, spi_regs->start);
		ret = -EINVAL;
		goto error_release_master;
	}

	s->pdma_regs = ioremap(pdma_regs->start, resource_size(pdma_regs));
	if (!s->pdma_regs) {
		dev_err(&pdev->dev, "unable to map registers for "
			"PDMA controller %d, base=%08x\n", bus,
			pdma_regs->start);
		ret = -EINVAL;
		goto error_release_spi_regs;
	}

	s->irq = irq;
	ret = request_irq(irq, spi_m2s_irq_cb, 0, dev_name(&pdev->dev), s);
	if (ret) {
		dev_err(&pdev->dev, "request irq %d failed for "
			"SPI controller %d\n", irq, bus);
		ret = -EINVAL;
		goto error_release_pdma_regs;
	}

	/*
	 * Init work queue
	 */
	spin_lock_init(&s->lock);
	INIT_LIST_HEAD(&s->queue);
	INIT_WORK(&s->work, spi_m2s_handle_queue);

	/*
	 * Initialize the controller hardware
	 */
	if (spi_m2s_hw_init(s)) {
		dev_err(&pdev->dev, "unable to initialize hardware for "
			"SPI controller %d\n", bus);
		ret = -ENXIO;
		goto error_release_irq;
	}

	/*
	 * SPI mode understood by this driver
	 */
	m->mode_bits = SPI_CPOL | SPI_CPHA;

	/*
	 * Number of chip selects supported by the controller
	 */
	m->num_chipselect = spi_m2s_hw_cs_max(s);

	/*
	 * Set-up SPI slave action callbacks
	 */
	m->setup = spi_m2s_setup;
	m->cleanup = spi_m2s_cleanup;
	m->transfer = spi_m2s_transfer;

	/*
	 * We will be running soon
	 */
	s->stopping = 0;

	/*
	 * Register the SPI controller
	 */
	ret = spi_register_master(m);
	if (ret) {
		dev_err(&pdev->dev, "unable to register master "
			"for SPI controller %d\n", bus);
		goto error_release_hardware;
	}

	/*
	 * Remember the master in the platform device.
	 * We are going to need that pointer when we
	 * are doing removal on the platform device.
	 */
	platform_set_drvdata(pdev, m);

	/*
	 * If we are here, we are successful
	 */
	dev_info(&pdev->dev, "SPI Controller %d at %p,clk=%d\n",
		m->bus_num, s->spi_regs, s->ref_clk);
	goto done;

	/*
	 * Error processing
	 */
error_release_hardware:
	spi_m2s_hw_release(s);
error_release_irq:
	free_irq(s->irq, s);
error_release_pdma_regs:
	iounmap(s->pdma_regs);
error_release_spi_regs:
	iounmap(s->spi_regs);
error_release_master:
	spi_master_put(m);
	platform_set_drvdata(pdev, NULL);
error_release_nothing:
done:
	return ret;
}

/*
 * Shutdown of an instance of the controller device
 * @dev		SPI controller platform device
 * @returns	0->success, <0->error code
 */
static int __devexit spi_m2s_remove(struct platform_device *pdev)
{
	struct spi_master *m  = platform_get_drvdata(pdev);
	struct m2s_spi_dsc *s = spi_master_get_devdata(m);
	struct spi_message *msg;
	unsigned long f;

	/*
	 * Block the queue progress and terminate queued transfers
	 */
	spin_lock_irqsave(&s->lock, f);
	s->stopping = 1;
	list_for_each_entry(msg, &s->queue, queue) {
		msg->status = -ESHUTDOWN;
		msg->complete(msg->context);
	}
	spin_unlock_irqrestore(&s->lock, f);

	/*
	 * Release kernel resources.
	 */
	spi_unregister_master(m);
	free_irq(s->irq, s);
	iounmap(s->spi_regs);
	iounmap(s->pdma_regs);
	spi_master_put(m);
	platform_set_drvdata(pdev, NULL);

	/*
	 * Shut the hardware down
	 */
	spi_m2s_hw_release(s);

	return 0;
}

/*
 * Platform driver data structure
 */
static struct platform_driver spi_m2s_driver = {
	.probe  = spi_m2s_probe,
	.remove = __devexit_p(spi_m2s_remove),
	.driver = {
		.name = "spi_m2s",
		.owner = THIS_MODULE,
	},
};

/*
 * Driver init
 */
static int __init spi_m2s_module_init(void)
{
	return platform_driver_register(&spi_m2s_driver);
}

/*
 * Driver clean-up
 */
static void __exit spi_m2s_module_exit(void)
{
	platform_driver_unregister(&spi_m2s_driver);
}

module_init(spi_m2s_module_init);
module_exit(spi_m2s_module_exit);
MODULE_AUTHOR("Yuri Tikhonov, <yur@emcraft.com>");
MODULE_DESCRIPTION("Device driver for SPI controller of SmartFusion2");
MODULE_LICENSE("GPL");
