/*
 * Device driver for the SPI controller of the SmartFusion SoC.
 * Author: Vladimir Khusainov, vlad@emcraft.com
 * Copyright 2011,2012 Emcraft Systems
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
#include <mach/a2f.h>
#include <mach/platform.h>

/*
 * Debug output control. While debugging, have SPI_A2F_DEBUG defined.
 * In deployment, make sure that SPI_A2F_DEBUG is undefined
 * to avoid performance and size overhead of debug messages.
 */
#define SPI_A2F_DEBUG
#if 1
#undef SPI_A2F_DEBUG
#endif

#if defined(SPI_A2F_DEBUG)

/*
 * Driver verbosity level: 0->silent; >0->verbose (1 to 4, growing verbosity)
 */
static int spi_a2f_debug = 0;

/*
 * User can change verbosity of the driver (when loading the module,
 * or the kernel, in case the driver is linked in statically,
 * but not through a /sysfs parameter file)
 */
module_param(spi_a2f_debug, int, S_IRUGO);
MODULE_PARM_DESC(spi_a2f_debug, "SPI controller driver verbosity level");

#if !defined(MODULE)

/*
 * Parser for the boot time driver verbosity parameter
 * @param str		user defintion of the parameter
 * @returns		1->success
 */
static int __init spi_a2f_debug_setup(char * str)
{
	get_option(&str, &spi_a2f_debug);
	return 1;
}
__setup("spi_a2f_debug=", spi_a2f_debug_setup);

#endif /* !defined(MODULE) */

/*
 * Service to print debug messages
 */
#define d_printk(level, fmt, args...)					\
	if (spi_a2f_debug >= level) printk(KERN_INFO "%s: " fmt,	\
				       	   __func__, ## args)
#else

#define d_printk(level, fmt, args...)

#endif /* defined(SPI_A2F_DEBUG) */

/*
 * Private data structure for an instance of the SPI controller
 */
struct spi_a2f {
	int				bus;		/* Bus (ID) */
	void * __iomem 			regs;		/* Registers base */
	int				irq;		/* IRQ # */
	unsigned int			speed_hz;	/* Max clock rate */
	unsigned char			stopping;	/* Is being stopped? */
	spinlock_t 			lock;		/* Exclusive access */
	struct list_head 		queue;		/* Message Q */
	struct work_struct		work;		/* Work Q */
	struct workqueue_struct *	workqueue;	/* Work Q */
	struct spi_device *		slave;		/* Current SPI slave */
	int				a2f_dev;	/* SmartFusion chip */
};

/* 
 * Description of the the SmartFusion SPI hardware interfaces.
 * This is a 1-to-1 mapping of Actel's documenation onto a C structure.
 * Refer to SmartFusion Data Sheet for details.
 */
struct mss_spi {
	unsigned int			spi_control;
	unsigned int			spi_txrxdf_size;
	unsigned int			spi_status;
	unsigned int			spi_int_clear;
	unsigned int			spi_rx_data;
	unsigned int			spi_tx_data;
	unsigned int			spi_clk_gen;
	unsigned int			spi_slave_select;
	unsigned int			spi_mis;
	unsigned int			spi_ris;
};

/*
 * Access handle for the control registers
 */
#define MSS_SPI_REGS(regs)		((volatile struct mss_spi *)(regs))
#define MSS_SPI(c)			(MSS_SPI_REGS(c->regs))

/*
 * Some bits in various CSRs 
 */
#define SPI0_RST_CLR			(1<<9)
#define SPI1_RST_CLR			(1<<10)
#define SPI_CONTROL_ENABLE		(1<<0)
#define SPI_CONTROL_MASTER		(1<<1)
#define SPI_CONTROL_PROTO_MSK		(3<<2)
#define SPI_CONTROL_PROTO_MOTO		(0<<2)
#define SPI_CONTROL_CNT_MSK		(0xffff<<8)
#define SPI_CONTROL_CNT_SHF		(8)
#define SPI_CONTROL_SPO			(1<<24)
#define SPI_CONTROL_SPH			(1<<25)
#define SPI_CONTROL_SPS			(1<<26)
#define SPI_CONTROL_BIGFIFO		(1<<28)
#define SPI_CONTROL_RESET		(1<<31)
#define SPI_STATUS_RXFIFOOVR		(1<<2)
#define SPI_STATUS_RXFIFOEMP		(1<<6)
#define SPI_STATUS_TXFIFOFUL		(1<<8)
#define SPI_INTCLR_RXFIFOOVR		(1<<2)

/*
 * Hardware initialization of the SPI controller
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static int spi_a2f_hw_init(struct spi_a2f *c)
{
	/*
 	 * TO-DO: here and everywhere, add appropriate code
 	 * for handling soft-IP SPI controllers
 	 * The assumption regarding the bus number (ID) is:
 	 * 0 - MSS SPI0
 	 * 1 - MSS SPI1
 	 * 2-9 - soft-IP SPI controllers (these are not supported as of yet)
 	 */
	unsigned int v = c->bus==0 ? SPI0_RST_CLR : 
			 c->bus==1 ? SPI1_RST_CLR : 0; 
	int ret = 0;

	/*
 	 * Reset the SPI controller and then bring it out of reset
 	 */
	writel(v | readl(&A2F_SYSREG->soft_rst_cr), &A2F_SYSREG->soft_rst_cr);
	writel(~v & readl(&A2F_SYSREG->soft_rst_cr), &A2F_SYSREG->soft_rst_cr);

	/*
 	 * Set the master mode
 	 */
	writel(SPI_CONTROL_MASTER | readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);

	/*
 	 * Set the transfer protocol. We are using the Motorola
 	 * SPI mode, with no user interface to configure it to 
 	 * some other mode.
 	 */
	writel(~SPI_CONTROL_PROTO_MSK & readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);
	writel(SPI_CONTROL_PROTO_MOTO | readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);

	/*
	 * If we are running on the A2F500 device, we have an option
	 * to set-up the controller in such a way that it doesn't remove
	 * Chip Select until the entire message has been transferred,
	 * even if at some points TX FIFO becomes empty.
	 * ...
	 * Similarly on A2F400, we have an option to extend FIFO to
	 * 32 8-bit FIFO frames.
 	 */
	if (c->a2f_dev == DEVICE_A2F_500) {
		writel(SPI_CONTROL_SPS | SPI_CONTROL_BIGFIFO |
			readl(&MSS_SPI(c)->spi_control),
			&MSS_SPI(c)->spi_control);
	}

	/*
 	 * Enable the SPI contoller
	 * On the A2F500 it is critical to clear RESET in
	 * the control bit. This bit is not defined for A2F200.
 	 */
	if (c->a2f_dev == DEVICE_A2F_500) {
		writel(~SPI_CONTROL_RESET & readl(&MSS_SPI(c)->spi_control),
			&MSS_SPI(c)->spi_control);
	}
	writel(SPI_CONTROL_ENABLE | readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);

	d_printk(2, "bus=%d,soft_rst_cr=0x%x,spi_control=0x%x,ret=%d\n", 
		 c->bus, readl(&A2F_SYSREG->soft_rst_cr),
		 readl(&MSS_SPI(c)->spi_control), ret);
	return ret;
}

/*
 * Get the number of chip selects supported by this controller
 * @param c		controller data structure
 * @returns		max chip select
 */
static int spi_a2f_hw_cs_max(struct spi_a2f *c)
{
	int ret = 8;

	d_printk(2, "bus=%d,ret=%d\n", c->bus, ret);
	return ret;
}

/*
 * Set chip select
 * @param c		controller data structure
 * @param cs		chip select: [0..7]->slave, otherwise->deselect all
 * @returns		0->good,!=0->bad
 */
static inline int spi_a2f_hw_cs_set(struct spi_a2f *c, int cs)
{
	unsigned int v = 0<=cs && cs<=7 ? 1<<cs : 0;
	int ret = 0;

	writel(v, &MSS_SPI(c)->spi_slave_select);

	d_printk(2, "bus=%d,cs=%d,slave_select=0x%x,ret=%d\n", 
		 c->bus, cs, readl(&MSS_SPI(c)->spi_slave_select), ret);
	return ret;
}

/*
 * Set controller clock rate
 * @param c		controller data structure
 * @param spd		clock rate in Hz
 * @returns		0->good,!=0->bad
 */
static inline int spi_a2f_hw_clk_set(struct spi_a2f *c, unsigned int spd)
{
	int i;
	unsigned int h = c->speed_hz;
	int ret = 0;

	/*
 	 * Calculate the clock rate that works for this slave
 	 */
	for (i = 1; i <= 8; i ++) {
		if (h / (1 << i) <= spd) break;
	}

	/*
 	 * Can't provide a rate that is slow enough for the slave
 	 */
	if (i == 9) {
		ret = -EIO;
		goto Done;
	}

	/*
 	 * Set the clock rate
 	 */
	writel(i - 1, &MSS_SPI(c)->spi_clk_gen);

Done:
	d_printk(1, "bus=%d,cnt_hz=%d,slv_hz=%d,rsl_hz=%d,clk_gen=%d,ret=%d\n",
		c->bus, h, spd, h / (1 << i),
		readl(&MSS_SPI(c)->spi_clk_gen), ret);
	return ret;
}

/*
 * Check frame size
 * @param c		controller data structure
 * @param bt		frame size
 * @returns		0->good,!=0->bad
 */
static inline int spi_a2f_hw_bt_check(struct spi_a2f *c, int bt)
{
	/*
 	 * TO-DO: add support for frames that are not 8 bits
 	 */
	int ret = 8 <= bt && bt <= 16 ? 0 : 1;

	d_printk(2, "bus=%d,bt=%d,ret=%d\n", c->bus, bt, ret);
	return ret;
}

/*
 * Set frame size (making an assumption that the supplied size is
 * supported by this controller)
 * @param c		controller data structure
 * @param bt		frame size
 * @returns		0->good,!=0->bad
 */
static inline int spi_a2f_hw_bt_set(struct spi_a2f *c, int bt)
{
	int ret = 0;

	/*
 	 * Disable the SPI contoller. Writes to data frame size have
 	 * no effect when the controller is enabled.
 	 */
	writel(~SPI_CONTROL_ENABLE & readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);

	/*
 	 * Set the new data frame size.
 	 */
	writel(bt, &MSS_SPI(c)->spi_txrxdf_size);

	/*
 	 * Re-enable the SPI contoller 
 	 */
	writel(SPI_CONTROL_ENABLE | readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);

	d_printk(2, "bus=%d,bt=%d,spi_txrxdf_size=%d,ret=%d\n", 
		 c->bus, bt, readl(&MSS_SPI(c)->spi_txrxdf_size), ret);
	return ret;
}

/*
 * Set transfer length
 * @param c		controller data structure
 * @param len		transfer size
 */
static inline void spi_a2f_hw_tfsz_set(struct spi_a2f *c, int len)
{
	/*
 	 * Disable the SPI contoller. Writes to transfer length have
 	 * no effect when the controller is enabled.
 	 */
	writel(~SPI_CONTROL_ENABLE & readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);

	/*
 	 * Set the new data frame size.
 	 */
	writel(~SPI_CONTROL_CNT_MSK & readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);
	writel((len << SPI_CONTROL_CNT_SHF) | readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);

	/*
 	 * Re-enable the SPI contoller 
 	 */
	writel(SPI_CONTROL_ENABLE | readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);
}

/*
 * Set SPI mode
 * @param c		controller data structure
 * @param mode		mode
 * @returns		0->good;!=0->bad
 */
static inline int spi_a2f_hw_mode_set(struct spi_a2f *c, unsigned int mode)
{
	int ret = 0;

	/*
 	 * Set the mode
 	 */
	if (mode & SPI_CPHA) {
		writel(SPI_CONTROL_SPH | readl(&MSS_SPI(c)->spi_control),
			&MSS_SPI(c)->spi_control);
	}
	else {
		writel(~SPI_CONTROL_SPH & readl(&MSS_SPI(c)->spi_control),
			&MSS_SPI(c)->spi_control);
	}
	if (mode & SPI_CPOL) {
		writel(SPI_CONTROL_SPO | readl(&MSS_SPI(c)->spi_control),
			&MSS_SPI(c)->spi_control);
	}
	else {
		writel(~SPI_CONTROL_SPO & readl(&MSS_SPI(c)->spi_control),
			&MSS_SPI(c)->spi_control);
	}

	d_printk(2, "bus=%d,mode=%x,spi_control=%x,ret=%d\n", 
		 c->bus, mode, readl(&MSS_SPI(c)->spi_control), ret);
	return ret;
}

/*
 * Is transmit FIFO full?
 * @param c		controller data structure
 * @returns		!0->full;0->not full
 */
static inline int spi_a2f_hw_txfifo_full(struct spi_a2f *c)
{
	return readl(&MSS_SPI(c)->spi_status) & SPI_STATUS_TXFIFOFUL;
}

/*
 * Put a frame into the transmit FIFO
 * TO-DO: support frames of size != 8
 * @param c		controller data structure
 * @param wb		frame size in full bytes
 * @param tx		transmit buf (can be NULL)
 * @param i		index of frame in buf
 */
static inline void spi_a2f_hw_txfifo_put(
	struct spi_a2f *c, int wb, const void *tx, int i)
{
	int j;
	unsigned int d = 0;
	unsigned char *p = (unsigned char *)tx;

	if (p) {
		for (j = 0; j < wb; j++) {
			d <<= 8;
			d |= p[i*wb + j];
		}
	}
	writel(d, &MSS_SPI(c)->spi_tx_data);
}

/*
 * Is receive FIFO empty?
 * @param c		controller data structure
 * @returns		!0->empty,0->not empty
 */
static inline int spi_a2f_hw_rxfifo_empty(struct spi_a2f *c)
{
	return readl(&MSS_SPI(c)->spi_status) & SPI_STATUS_RXFIFOEMP;
}

/*
 * Is receive FIFO overflown?
 * @param c		controller data structure
 * @returns		!0->error,0->no error
 */
static inline int spi_a2f_hw_rxfifo_error(struct spi_a2f *c)
{
	return readl(&MSS_SPI(c)->spi_status) & SPI_STATUS_RXFIFOOVR;
}

/*
 * Retrieve a frame from the receive FIFO
 * TO-DO: support frames of size != 8
 * @param c		controller data structure
 * @param wb		frame size in full bytes
 * @param rx		receive buf (can be NULL)
 * @param i		index of frame in buf
 */
static inline void spi_a2f_hw_rxfifo_get(
	struct spi_a2f *c, unsigned int wb, void *rx, int i)
{
	int j;
	unsigned int d = readl(&MSS_SPI(c)->spi_rx_data);
	unsigned char *p = (unsigned char *)rx;

	if (p) {
		for (j = wb-1; j >= 0; j--) {
			p[i*wb + j] = d & 0xFF;
			d >>= 8;
		}
	}
}

/*
 * Receive FIFO overflown; clean-up
 * @param c		controller data structure
 * @param rx		receive buf (can be NULL)
 * @param i		index of frame in buf
 */
static inline void spi_a2f_hw_rxfifo_purge(struct spi_a2f *c) 
{
	while (!spi_a2f_hw_rxfifo_empty(c)) {
		spi_a2f_hw_rxfifo_get(c, 1, NULL, 0);
	}
	writel(SPI_INTCLR_RXFIFOOVR | readl(&MSS_SPI(c)->spi_int_clear),
		&MSS_SPI(c)->spi_int_clear);
}

/*
 * Hardware shutdown of the SPI controller
 * @param c		controller data structure
 */
static void spi_a2f_hw_release(struct spi_a2f *c)
{
	unsigned int v = c->bus==0 ? SPI0_RST_CLR : 
			 c->bus==1 ? SPI1_RST_CLR : 0; 

	/*
 	 * Disable the SPI contoller
 	 */
	writel(~SPI_CONTROL_ENABLE & readl(&MSS_SPI(c)->spi_control),
		&MSS_SPI(c)->spi_control);

	/*
 	 * Put the SPI controller into reset
 	 */
	writel(v | readl(&A2F_SYSREG->soft_rst_cr), &A2F_SYSREG->soft_rst_cr);
	
	d_printk(2, "bus=%d,soft_rst_cr=0x%x,spi_control=0x%x\n", 
		c->bus, readl(&A2F_SYSREG->soft_rst_cr),
		readl(&MSS_SPI(c)->spi_control));
}

/*
 * Prepare to transfer to a slave
 * @param c		controller data structure
 * @param s		slave data structure
 * @returns		0->success, <0->error code
 */
static int spi_a2f_prepare_for_slave(
	struct spi_a2f *c, struct spi_device *s)
{
	unsigned int spd;
	int ret = 0;

	/*
 	 * Set for this slave: frame size, clock, slave select, mode
 	 */
	if (spi_a2f_hw_bt_set(c, s->bits_per_word)) {
		dev_err(&c->slave->dev, "unsupported frame size: %d\n",
			s->bits_per_word);
		ret = -EINVAL;
		goto Done;
	}
	if (spi_a2f_hw_clk_set(c, spd = min(s->max_speed_hz, c->speed_hz))) {
		dev_err(&c->slave->dev, "slave rate too low: %d\n", spd);
		ret = -EINVAL;
		goto Done;
	}
	if (spi_a2f_hw_cs_set(c, s->chip_select)) {
		dev_err(&c->slave->dev, "incorrect chip select: %d\n", 
			s->chip_select);
		ret = -EINVAL;
		goto Done;
	}
	if (spi_a2f_hw_mode_set(c, s->mode)) {
		dev_err(&c->slave->dev, "unsupported mode: %x\n", s->mode);
		ret = -EINVAL;
		goto Done;
	}

Done:
	d_printk(2, "slv=%s,ret=%d\n", dev_name(&c->slave->dev), ret);
	return ret;
}

/*
 * Transfer a message in PIO mode
 * @param c		controller data structure
 * @param s		slave data structure
 * @param msg		message
 * @param		pointer to actual transfer length (set here)
 * @returns		0->success, <0->error code
 */
static int spi_a2f_pio(
	struct spi_a2f *c, struct spi_device *s, 
	struct spi_message *msg, int *rlen)
{
	struct spi_transfer *t, *tx_t, *rx_t;
	int ti, ri, tx_l, tx_i, rx_l, rx_i;
	int wb = (s->bits_per_word + 7) / 8;
	int len = 0;
	int ret = 0;
	int i;

	/*
 	 * Count the total length of the message
 	 */
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		len += t->len;
	}
	len /= wb;

	/*
 	 * Set the size of the transfer in the SPI controller
 	 */
	spi_a2f_hw_tfsz_set(c, len);

	/*
 	 * Prepare to traverse the message list.
 	 * We will need to advance separately over 
 	 * transmit and receive data
 	 */
	tx_t = list_entry((&msg->transfers)->next,
		   struct spi_transfer, transfer_list);
	tx_l = tx_t->len / wb;
	tx_i = 0;
	rx_t = list_entry((&msg->transfers)->next,
		   struct spi_transfer, transfer_list);
	rx_l = rx_t->len / wb;
	rx_i = 0;

	/*
 	 * Perform the transfer. Transfer is done when all frames
 	 * have been received (i.e. ri == len). Each time we
 	 * iterate in this loop, we have received a next frame.
 	 */
	for (ti = 0, ri = 0; ri < len; ri++) {

		/* It is important to keep transmit fifo not empty,
		 * while there are frames to be transmitted. If this
		 * is not done, the SPI controller asserts slave
		 * select as soon as transmit fifo has been emptied
		 * regardless of the value in transfer count (which
		 * cancels a transaction at the slave). On the other
		 * hand, it is important to let the code retrieving
		 * incoming data (below) run every so frequenly or
		 * otherwise an RX overflow will happen.
		 */
	        for (i = 0; 
		     i < 2 && ti < len && !spi_a2f_hw_txfifo_full(c);
		     i++) {

			/*
 			 * If the trasmit in the current transfer
 			 * has been finished, go to the next one.
 			 */
			while (tx_i == tx_l) {
				tx_t = list_entry(tx_t->transfer_list.next, 
		                          struct spi_transfer, transfer_list);
				tx_l = tx_t->len / wb;
				tx_i = 0;
			}

			/*
 			 * Put a frame (or a dummy value) to the transmit fifo
 			 */
			spi_a2f_hw_txfifo_put(c, wb, tx_t->tx_buf, tx_i);
			tx_i++;
			ti++;
		}

		/*
 		 * Wait for a frame to come in (but not indefinitely)
 		 * but check for error conditions first
 		 */
		if (spi_a2f_hw_rxfifo_error(c)) {
			/*
			 * If the receive fifo overflown, this transfer
			 * needs to be finished with an error.
			 */
			spi_a2f_hw_rxfifo_purge(c);
			ret = -EIO;
			goto Done;
		}
		for (i = 0; i < 100 && spi_a2f_hw_rxfifo_empty(c); i++);

		/*
		 * Process as many incoming frames as there is in the fifo
		 */
		while (!spi_a2f_hw_rxfifo_empty(c)) {

			/*
 			 * If the receive in the current transfer
 	 	 	 * has been finished, go to the next one.
 		 	 */
			while (rx_i == rx_l) {
				/*
		 	 	 * Wait, if a wait is needed
		 	 	 */
				if (rx_t->delay_usecs) {
					udelay(rx_t->delay_usecs);
				}

				/*
 			 	 * Advance to the next transfer
 			 	 */
				rx_t = list_entry(rx_t->transfer_list.next, 
	                        	  struct spi_transfer, transfer_list);
				rx_l = rx_t->len / wb;
				rx_i = 0;
			}

			/* 
 		 	 * Read in the frame (or a dummy frame).
 		 	 */
			spi_a2f_hw_rxfifo_get(c, wb, rx_t->rx_buf, rx_i);
			rx_i++;
		}
	}

	/*
 	 * Return the number of bytes actully transferred
 	 */
	*rlen = ri;
Done:
	d_printk(3, "msg=%p,len=%d,rlen=%d,ret=%d\n", msg, len, *rlen, ret);
	return ret;
}

/*
 * Transfer a message 
 * @param c		controller data structure
 * @param m		message
 * @returns		0 -> success, negative error code -> error
 */
static int spi_a2f_handle_message(struct spi_a2f *c, struct spi_message *msg)
{
#if defined(SPI_A2F_DEBUG)
	struct spi_transfer *t;
#endif
	struct spi_device *s = msg->spi;
	int rlen = 0;
	int ret = 0;

	/*
 	 * Check if the current slave of this controller is
 	 * the message's SPI device, and if not, re-set 
 	 * the speed and chip select for the message's SPI device.
 	 */
	if (c->slave != s) {
		c->slave = s;
		ret = spi_a2f_prepare_for_slave(c, s);
		if (ret) {
			c->slave = NULL;
			goto Done;
		}
	}

	/*
 	 * Transfer the message over the wire
 	 */
	ret = spi_a2f_pio(c, s, msg, &rlen);
	if (ret) {
		goto Done;
	}

	msg->actual_length = rlen;

Done:
#if defined(SPI_A2F_DEBUG)
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		int i; 
		char * p;

		d_printk(3, "t=%p,tx=%p,rx=%p,len=%d\n",
		         t, t->tx_buf, t->rx_buf, t->len);
		if (rlen < 10) {
			p = (char *) (t->tx_buf ? t->tx_buf : t->rx_buf);
			for (i = 0; i < t->len; i++) {
				d_printk(4, "%02x ", p[i]);
			}
		}
	}
#endif
	return ret;
}

/*
 * Handle a ping to the workqueue
 * @param w		work data structure
 */
static void spi_a2f_handle(struct work_struct *w)
{
	struct spi_message *msg;
	struct spi_a2f *c = container_of(w, struct spi_a2f, work);
	unsigned long f = 0;

	/*
	 * In atomic manner 
	 */
	if (c->a2f_dev == DEVICE_A2F_500) {
		spin_lock_irqsave(&c->lock, f);
	}

	/*
 	 * Check if the message queue has messages
 	 */
	while (!list_empty(&c->queue)) {

		/*
		 * Extract the next message from the queue
		 */
		msg = list_entry(c->queue.next, struct spi_message, queue);
		list_del_init(&msg->queue);

		/*
		 * Transfer the message over SPI wires
		 */
		msg->status = spi_a2f_handle_message(c, msg);
		d_printk(2, "slv=%s,msg=%p,sta=%d\n",
		         dev_name(&msg->spi->dev), msg, msg->status);

		/*
		 * Release the lock after transferring the message
		 * ...
		 * TO-DO: this disables interrupts for quite a long
		 * time. This is not good clearly, however this is
		 * the only mode that works reliably in a sense that
		 * it allows to successfully tranfer long messages
		 * without breaking a transfer. Even that works only
		 * at relatively slow clock rates (<1MHz).
		 *
		 * Conceptually, the message transfer routine is written
		 * to handle incomplete transfers and return the actual
		 * number of transmitted frames to upper layers. This doesn't
		 * quite work however (suspect a defect in upper layers;
		 * in all likelihood, they haven't been tested heavily with
		 * a driver that returns incomplete messages often).
		 *
		 * The only resort I can think of is switching to
		 * the DMA mode. This is some work though so we will
		 * do it when there is clear need for that. 
		 *
		 * Another option is using an I/O pin as a slave select,
		 * instead of using the SPI controller's dedicated SS
		 * signal. Using that option, we would be able to control
		 * the slave select signal (I/O based) explicitly, and
		 * an empty TX FIFO wouldn't interrupt a transaction 
		 * at the slave. This option assumes however a deviation
		 * from the standard design to an SPI slave (GPIO vs
		 * dedicated SS used as a slave select).
		 * ...
		 * The above only concerns A2F200. A2F500 defines a special
		 * control that allows to configure the SPI control in
		 * such a way that Chip Select remains asserted until
		 * an antire message has been sent out even if TX FIFO
		 * gets empty at some points. This allows not to disable
		 * interrupts in this function, on the A2F500. Also,
		 * it allows running SPI devices are high frequencies.
		 */
		if (c->a2f_dev == DEVICE_A2F_500) {
			spin_unlock_irqrestore(&c->lock, f);
		}

		/*
 		 * Let the upper layers complete processing of the message
 		 */
		msg->complete(msg->context);

		/*
 		 * Re-acquire the lock and go check the message list again
 		 */
		if (c->a2f_dev == DEVICE_A2F_500) {
			spin_lock_irqsave(&c->lock, f);
		}
	}

	/*
	 * Release the lock and return
	 */
	if (c->a2f_dev == DEVICE_A2F_500) {
		spin_unlock_irqrestore(&c->lock, f);
	}
}

/*
 * Set up the SPI controller for a specified SPI slave device
 * @param s		SPI slave
 * @returns		0->success, <0->error code
 */
static int spi_a2f_setup(struct spi_device *s)
{
	struct spi_a2f *c = spi_master_get_devdata(s->master);
	int ret = 0;

	/*
 	 * Check that the controller is still running
 	 */
	if (c->stopping) {
		ret = -ESHUTDOWN;
		goto Done;
	}

	/*
 	 * Check the width of transfer for this device
 	 */
	if (spi_a2f_hw_bt_check(c, s->bits_per_word)) {
		dev_err(&s->dev, "unsupported bits per word %d\n", 
			s->bits_per_word);
		ret = -EINVAL;
		goto Done;
	}

	/*
 	 * Don't remember the current slave. When transferring
 	 * a message, we will check if the current slave is
 	 * the message's SPI device, and if not, will re-set 
 	 * the speed and chip select for the message's SPI device.
 	 */
	c->slave = NULL;

Done:
	d_printk(1, "slv=%s,spd=%d,cs=%d,bt=%d,md=0x%x,ret=%d\n",
		 dev_name(&s->dev), s->max_speed_hz,
		 s->chip_select, s->bits_per_word, s->mode, ret);
	return ret;
}
/*
 * Clean up the SPI controller after a specified SPI slave device
 * @param s		SPI slave
 */
static void spi_a2f_cleanup(struct spi_device *s)
{
	d_printk(1, "slv=%s\n", dev_name(&s->dev));
}

/*
 * Transfer a message to a specified SPI slave
 * @s			SPI slave
 * @m			message to transfer
 * @returns		0->success; <0->error code
 */
static int spi_a2f_transfer(struct spi_device *s, struct spi_message *msg)
{
	struct spi_a2f *c = spi_master_get_devdata(s->master);
	unsigned long f;
	int ret = 0;

	/*
 	 * Check that the controller is still running
 	 */
	if (c->stopping) {
		ret = -ESHUTDOWN;
		goto Done;
	}

	/*
 	 * Message content sanity check
 	 */
	if (unlikely(list_empty(&msg->transfers))) {
		ret = -EINVAL;
		goto Done;
	}

	/*
	 * Prepare ithe message for transfer
	 */
	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	/*
	 * In atomic manner: add the message to the message queue
	 * and ping the workqueue thread letting it process the message
	 */
	spin_lock_irqsave(&c->lock, f);
	list_add_tail(&msg->queue, &c->queue);
	queue_work(c->workqueue, &c->work);
	spin_unlock_irqrestore(&c->lock, f);

Done:
	d_printk(3, "msg=%p,ret=%d\n", msg, ret);
	return ret;
}

/*
 * Interrupt handler routine
 * This is not used in this version of the driver
 */
static irqreturn_t spi_a2f_irq(int irq, void *dev_id)
{
	d_printk(1, "ok\n");
	return IRQ_HANDLED;
}

/*
 * Instantiate a new instance of the SPI controller
 * @dev			SPI controller platform device
 * @returns		0->success, <0->error code
 */
static int __devinit spi_a2f_probe(struct platform_device *dev)
{
	struct spi_master *m = NULL;
	struct spi_a2f *c = NULL;
	struct resource *regs;
	int bus;
	int irq;
	int ret = 0;

	/*
 	 * Get the bus # from the platform device: 
 	 * [0,1]->hard-core SPI contorller of SmartFusion; 
 	 * [2-9]->soft-IP SPI controller specific to a custom design.
 	 */
	bus = dev->id;
	if (! (0 <= bus && bus <= 10)) {
		dev_err(&dev->dev, "invalid SPI controller %d\n", bus);
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	/*
	 * Get the IRQ number from the platform device
	 */
	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		dev_err(&dev->dev, "invalid IRQ %d for SPI contoller %d\n",
			irq, bus);
		ret = irq;
		goto Error_release_nothing;
	}

	/*
	 * Get the register base from the platform device
	 */
	regs = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (! regs) {
		dev_err(&dev->dev, "no register base for SPI controller %d\n",
                        bus);
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	/*
	 * Allocate an SPI master
	 */
	m = spi_alloc_master(&dev->dev, sizeof *c);
	if (!m) {	
		dev_err(&dev->dev, "unable to allocate master for "
			"SPI controller %d\n", bus);
		ret = -ENOMEM;
		goto Error_release_nothing;
	}


	/*
	 * Pointer the controller-specific data structure
 	 */
	c = spi_master_get_devdata(m);

	/*
	 * Find out which SmartFusion chip we are running on
	 */
	c->a2f_dev = a2f_device_get();

	/*
 	 * Set up the bus number so that platform
 	 * can set up SPI slave devices on this bus
 	 */
	m->bus_num = bus;
	c->bus = bus;

	/*
 	 * Map in the controller registers
 	 */
	c->regs = ioremap(regs->start, resource_size(regs));
	if (!c->regs) {
		dev_err(&dev->dev, "unable to map registers for "
			"SPI controller %d, base=%08x\n", bus, regs->start);
		ret = -EINVAL;
		goto Error_release_master;
	}

	/*
 	 * Register interrupt handler
 	 */
	ret = request_irq(irq, spi_a2f_irq, 0, dev_name(&dev->dev), c);
	if (ret) {
		dev_err(&dev->dev, "request irq %d failed for "
			"SPI controller %d\n", irq, bus);
		goto Error_release_regs;
	}
	c->irq = irq;

	/*
 	 * Create a workqueue and register the handler
 	 */
	c->workqueue = create_singlethread_workqueue(dev_name(&dev->dev));
	if (!c->workqueue) {
		dev_err(&dev->dev, "unable to create workqueue for "
			"SPI controller %d\n", bus);
		ret = -ENXIO;
		goto Error_release_irq;
	}
	INIT_WORK(&c->work, spi_a2f_handle);

	/*
 	 * Set up queue of messages
 	 */
	INIT_LIST_HEAD(&c->queue);

	/*
 	 * Set up the lock
 	 */
	spin_lock_init(&c->lock);

	/* 
 	 * Initialize the controller hardware
 	 */
	if (spi_a2f_hw_init(c)) {
		dev_err(&dev->dev, "unable to initialize hardware for "
			"SPI controller %d\n", bus);
		ret = -ENXIO;
		goto Error_release_workqueue;
	}

	/*
 	 * Figure the clock rate for this controller.
 	 * This is passed to us by the platform.
 	 */
	c->speed_hz = (unsigned int) platform_get_drvdata(dev);

	/*
 	 * SPI mode understood by this driver
 	 */
	m->mode_bits = SPI_CPOL | SPI_CPHA;

	/*
 	 * Number of chip selects supported by the controller
 	 */
	m->num_chipselect = spi_a2f_hw_cs_max(c);

	/*
 	 * Set-up SPI slave action callbacks
 	 */
	m->setup = spi_a2f_setup;
	m->cleanup = spi_a2f_cleanup;
	m->transfer = spi_a2f_transfer;

	/*
 	 * We will be running soon
 	 */
	c->stopping = 0;

	/*
 	 * Register the SPI controller
 	 */
	ret = spi_register_master(m);
	if (ret) {	
		dev_err(&dev->dev, "unable to register master "
			"for SPI controller %d\n", bus);
		goto Error_release_hardware;
	}

	/*
 	 * Remember the master in the platform device.
 	 * We are going to need that pointer when we
 	 * are doing removal on the platform device.
 	 */
	platform_set_drvdata(dev, m);

	/*
	 * If we are here, we are successful
	 */
	dev_info(&dev->dev, "SPI Controller %d at %p,irq=%d,hz=%d\n",
		 m->bus_num, c->regs, c->irq, c->speed_hz);
	goto Done;

	/*
	 * Error processing
	 */
Error_release_hardware: 
	spi_a2f_hw_release(c);
Error_release_workqueue: 
	destroy_workqueue(c->workqueue);
Error_release_irq: 
	free_irq(c->irq, c);
Error_release_regs: 
	iounmap(c->regs);
Error_release_master: 
	spi_master_put(m);
	platform_set_drvdata(dev, NULL);
Error_release_nothing: 
	
	/*
	 * Exit point
	 */
Done:
	d_printk(1, "dev=%s,regs=%p,irq=%d,hz=%d,ret=%d\n", 
		 dev_name(&dev->dev), 
		 c? c->regs : NULL, c ? c->irq : 0, c ? c->speed_hz : 0, 
		 ret);
	return ret;
}

/*
 * Shutdown of an instance of the controller device
 * @dev			SPI controller platform device
 * @returns		0->success, <0->error code
 */
static int __devexit spi_a2f_remove(struct platform_device *dev)
{
	struct spi_master *m  = platform_get_drvdata(dev);
	struct spi_a2f *c = spi_master_get_devdata(m);
	struct spi_message *msg;
	unsigned long f;
	int ret = 0;

	/*
 	 * Block the queue progress and terminate queued transfers
 	 */
	spin_lock_irqsave(&c->lock, f);
	c->stopping = 1;
	list_for_each_entry(msg, &c->queue, queue) {
		msg->status = -ESHUTDOWN;
		msg->complete(msg->context);
	}
	spin_unlock_irqrestore(&c->lock, f);

	/*
	 * Release kernel resources.
	 */
	spi_unregister_master(m);
	free_irq(c->irq, c);
	destroy_workqueue(c->workqueue);
	iounmap(c->regs);
	spi_master_put(m);
	platform_set_drvdata(dev, NULL);

	/*
 	 * Shut the hardware down
 	 */
	spi_a2f_hw_release(c);

	/*
	 * Exit point
	 */
	d_printk(1, "dev=%s,ret=%d\n", dev_name(&dev->dev), ret);
	return ret;
}

/*
 * Platform driver data structure
 */
static struct platform_driver spi_a2f_drv = {
	.probe	= spi_a2f_probe,
	.remove	= __devexit_p(spi_a2f_remove),
	.driver = {
		.name = "spi_a2f",
		.owner = THIS_MODULE,
	},
};

/*
 * Driver init
 */
static int __init spi_a2f_module_init(void)
{
	int ret;
	
	ret = platform_driver_register(&spi_a2f_drv);

	d_printk(1, "drv=%s,ret=%d\n", spi_a2f_drv.driver.name, ret);
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit spi_a2f_module_exit(void)
{
	platform_driver_unregister(&spi_a2f_drv);

	d_printk(1, "drv=%s\n", spi_a2f_drv.driver.name);
}

module_init(spi_a2f_module_init);
module_exit(spi_a2f_module_exit);
MODULE_AUTHOR("Vladimir Khusainov, <vlad@emcraft.com>");
MODULE_DESCRIPTION("Device driver for SPI controller of SmartFusion");
MODULE_LICENSE("GPL");
