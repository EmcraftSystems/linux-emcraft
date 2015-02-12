/*
 * Device driver for the SPI controller of the STM32F2/F4/F7
 * Author: Vladimir Khusainov, vlad@emcraft.com
 * Copyright 2013-2015 Emcraft Systems
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_stm32.h>
#include <mach/platform.h>
#include <mach/stm32.h>
#include <mach/iomux.h>

/*
 * Debug output control. While debugging, have SPI_STM32_DEBUG defined.
 * In deployment, make sure that SPI_STM32_DEBUG is undefined
 * to avoid the performance and size overhead of debug messages.
 */
#define SPI_STM32_DEBUG
#if 1
#undef SPI_STM32_DEBUG
#endif

#if defined(SPI_STM32_DEBUG)

/*
 * Driver verbosity level: 0->silent; >0->verbose (1 to 4, growing verbosity)
 */
static int spi_stm32_debug = 0;

/*
 * User can change verbosity of the driver (when loading the module,
 * or the kernel, in case the driver is linked in statically,
 * but not through a /sysfs parameter file)
 */
module_param(spi_stm32_debug, int, S_IRUGO);
MODULE_PARM_DESC(spi_stm32_debug, "SPI controller driver verbosity level");

#if !defined(MODULE)

/*
 * Parser for the boot time driver verbosity parameter
 * @param str		user defintion of the parameter
 * @returns		1->success
 */
static int __init spi_stm32_debug_setup(char * str)
{
	get_option(&str, &spi_stm32_debug);
	return 1;
}
__setup("spi_stm32_debug=", spi_stm32_debug_setup);

#endif /* !defined(MODULE) */

/*
 * Service to print debug messages
 */
#define d_printk(level, fmt, args...)					\
	if (spi_stm32_debug >= level) printk(KERN_INFO "%s: " fmt,	\
				       	   __func__, ## args)
#else

#define d_printk(level, fmt, args...)

#endif /* defined(SPI_STM32_DEBUG) */

/*
 * Polled or interrupt driven PIO mode.
 */
#if 0
#define	CONFIG_SPI_STM32_POLLED
#endif

/*
 * Private data structure for an SPI controller instance
 */
struct spi_stm32 {
	int				bus;		/* Bus (ID) */
	void * __iomem 			regs;		/* Registers base */
	unsigned int			speed_hz;	/* Max clock rate */
	unsigned char			stopping;	/* Is being stopped? */
	spinlock_t 			lock;		/* Exclusive access */
	struct list_head 		queue;		/* Message Q */
	struct work_struct		work;		/* Work Q */
	struct workqueue_struct *	workqueue;	/* Work Q */
#if !defined(CONFIG_SPI_STM32_POLLED)
	wait_queue_head_t		wait;		/* Wait queue */
	int				irq;		/* IRQ # */
#endif
	struct spi_device *		slave;		/* Current SPI slave */
	struct spi_message *		msg;		/* SPI message */
	volatile int			xfer_status;	/* Xfer status */
	int				len;		/* Xfer len */
	int				wb;		/* Xfer width */
	struct spi_transfer *		tx_t;		/* Cur Tx xfer */
	struct spi_transfer *		rx_t;		/* Cur Rx xfer */
	int				tx_l;		/* Tx len */
	int				rx_l;		/* Rx len */
	int				tx_i;		/* Cur Tx index */
	int				rx_i;		/* Cur Rx index */
	int				ti;		/* Tx count */
	int				ri;		/* Rx count */
};

/* 
 * Description of the the STM32 SPI hardware registers
 */
struct reg_spi {
	unsigned int			spi_cr1;
	unsigned int			spi_cr2;
	unsigned int			spi_sr;
	unsigned int			spi_dr;
	unsigned int			spi_crcpr;
	unsigned int			spi_rxcrcr;
	unsigned int			spi_txcrcr;
	unsigned int			spi_i2scfgr;
	unsigned int			spi_i2spr;
};

/*
 * Access handle for the control registers
 */
#define SPI_REGS(regs)		((volatile struct reg_spi *)(regs))
#define SPI(c)			(SPI_REGS(c->regs))

/*
 * Some bits in various CSRs 
 */
#define RCC_APB1RSTR_SPI2		(1<<14)
#define RCC_APB1RSTR_SPI3		(1<<15)
#define RCC_APB1ENR_SPI2		(1<<14)
#define RCC_APB1ENR_SPI3		(1<<15)
#define RCC_APB2RSTR_SPI1		(1<<12)
#define RCC_APB2RSTR_SPI4		(1<<13)
#define RCC_APB2RSTR_SPI5		(1<<20)
#define RCC_APB2RSTR_SPI6		(1<<21)
#define RCC_APB2ENR_SPI1		(1<<12)
#define RCC_APB2ENR_SPI4		(1<<13)
#define RCC_APB2ENR_SPI5		(1<<20)
#define RCC_APB2ENR_SPI6		(1<<21)
#define SPI_CR1_DFF			(1<<11)
#define SPI_CR1_SSM			(1<<9)
#define SPI_CR1_SSI			(1<<8)
#define SPI_CR1_SPE			(1<<6)
#define SPI_CR1_BR(x)			((x)<<3)
#define SPI_CR1_MSTR			(1<<2)
#define SPI_CR1_CPOL			(1<<1)
#define SPI_CR1_CPHA			(1<<0)
#define SPI_CR2_FRXTH			(1<<12)
#define SPI_CR2_DS(x)			((x)<<8)
#define SPI_CR2_TXEIE			(1<<7)
#define SPI_CR2_RXNEIE			(1<<6)
#define SPI_CR2_ERRIE			(1<<5)
#define SPI_SR_FRE			(1<<8)
#define SPI_SR_BSY			(1<<7)
#define SPI_SR_OVR			(1<<6)
#define SPI_SR_UDR			(1<<3)
#define SPI_SR_TXE			(1<<1)
#define SPI_SR_RXNE			(1<<0)

/*
 * Hardware initialization of the SPI controller
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static int spi_stm32_hw_init(struct spi_stm32 *c)
{
	unsigned int v;
	int ret = 0;

	/*
 	 * Reset the SPI controller and then bring it out of reset.
 	 * Enable the SPI controller clock.
 	 */
	if (c->bus == 0) {		/* SPI1 */
		v = readl(&STM32_RCC->apb2rstr);
		writel(v | RCC_APB2RSTR_SPI1, &STM32_RCC->apb2rstr);
		writel(v & ~RCC_APB2RSTR_SPI1, &STM32_RCC->apb2rstr);
		v = readl(&STM32_RCC->apb2enr);
		writel(v | RCC_APB2ENR_SPI1, &STM32_RCC->apb2enr);
	}
	else if (c->bus == 1) {		/* SPI2 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_SPI2, &STM32_RCC->apb1rstr);
		writel(v & ~RCC_APB1RSTR_SPI2, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v | RCC_APB1ENR_SPI2, &STM32_RCC->apb1enr);
	}
	else if (c->bus == 2) {		/* SPI3 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_SPI3, &STM32_RCC->apb1rstr);
		writel(v & ~RCC_APB1RSTR_SPI3, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v | RCC_APB1ENR_SPI3, &STM32_RCC->apb1enr);
	}
	else if (c->bus == 3) {		/* SPI4 */
		v = readl(&STM32_RCC->apb2rstr);
		writel(v | RCC_APB2RSTR_SPI4, &STM32_RCC->apb2rstr);
		writel(v & ~RCC_APB2RSTR_SPI4, &STM32_RCC->apb2rstr);
		v = readl(&STM32_RCC->apb2enr);
		writel(v | RCC_APB2ENR_SPI4, &STM32_RCC->apb2enr);
	}
	else if (c->bus == 4) {		/* SPI5 */
		v = readl(&STM32_RCC->apb2rstr);
		writel(v | RCC_APB2RSTR_SPI5, &STM32_RCC->apb2rstr);
		writel(v & ~RCC_APB2RSTR_SPI5, &STM32_RCC->apb2rstr);
		v = readl(&STM32_RCC->apb2enr);
		writel(v | RCC_APB2ENR_SPI5, &STM32_RCC->apb2enr);
	}
	else if (c->bus == 5) {		/* SPI6 */
		v = readl(&STM32_RCC->apb2rstr);
		writel(v | RCC_APB2RSTR_SPI6, &STM32_RCC->apb2rstr);
		writel(v & ~RCC_APB2RSTR_SPI6, &STM32_RCC->apb2rstr);
		v = readl(&STM32_RCC->apb2enr);
		writel(v | RCC_APB2ENR_SPI6, &STM32_RCC->apb2enr);
	}

	/*
 	 * Set the master mode
 	 */
	writel(SPI_CR1_MSTR, &SPI(c)->spi_cr1);

	/*
	 * Software chip select management, NSS pin can be re-used as a GPIO.
	 * Software is responsible for driving CS (which can be NSS or any
	 * other GPIO) appropriately.
	 */
	writel(SPI_CR1_SSM | SPI_CR1_SSI | readl(&SPI(c)->spi_cr1), 
		&SPI(c)->spi_cr1);

	/*
 	 * Set the transfer protocol. We are using the Motorola
 	 * SPI mode, with no API to configure it to 
 	 * some other mode. The Motorola mode is default, so
 	 * no explicit update of the registers.
 	 */

	/*
 	 * If PIO interrupt-driven, enable interrupts
 	 */
#if !defined(CONFIG_SPI_STM32_POLLED)
	writel(SPI_CR2_RXNEIE | SPI_CR2_ERRIE | readl(&SPI(c)->spi_cr2),
		&SPI(c)->spi_cr2);
#endif

	/*
 	 * Enable the SPI contoller
 	 */
	writel(SPI_CR1_SPE | readl(&SPI(c)->spi_cr1), &SPI(c)->spi_cr1);

	d_printk(2, "bus=%d,spi_cr1=0x%x,ret=%d\n", 
		 c->bus, readl(&SPI(c)->spi_cr1), ret);
	return ret;
}

/*
 * Get the number of chip selects supported by this controller
 * @param c		controller data structure
 * @returns		max chip select
 */
static int spi_stm32_hw_cs_max(struct spi_stm32 *c)
{
	/*
 	 * With the software chip select management, we
 	 * can actually support as many CS as there is GPIO
 	 * but let's limit this to some reasonable value
 	 */
	int ret = 8;

	d_printk(2, "bus=%d,ret=%d\n", c->bus, ret);
	return ret;
}

/*
 * Set chip select
 * @param c		controller data structure
 * @param n		GPIO number for CS
 * @param activate	1->CS=low (activated); 0->CS=high (deactivated)
 * @returns		0->good,!=0->bad
 */
static inline int spi_stm32_hw_cs_set(
	struct spi_stm32 *c, int n, int activate)
{
	int ret = 0;

	/*
 	 * Drive the CS GPIO manually
 	 */
	gpio_set_value(n, !activate);

	d_printk(4, "bus=%d,cs=%d,b=%d,ret=%d\n", 
		c->bus, n, activate, ret);
	return ret;
}

/*
 * Set controller clock rate
 * @param c		controller data structure
 * @param spd		clock rate in Hz
 * @returns		0->good,!=0->bad
 */
static inline int spi_stm32_hw_clk_set(struct spi_stm32 *c, unsigned int spd)
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
	writel((SPI_CR1_BR(i-1)) | readl(&SPI(c)->spi_cr1), &SPI(c)->spi_cr1);

Done:
	d_printk(1, "bus=%d,cnt_hz=%d,slv_hz=%d,rsl_hz=%d,"
		"i=%d,cr1=%x,ret=%d\n",
		c->bus, h, spd, h / (1 << i), i-1, 
		readl(&SPI(c)->spi_cr1), ret);
	return ret;
}

/*
 * Check frame size
 * @param c		controller data structure
 * @param bt		frame size
 * @returns		0->good,!=0->bad
 */
static inline int spi_stm32_hw_bt_check(struct spi_stm32 *c, int bt)
{
	int ret = 8 == bt || bt == 16 ? 0 : 1;

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
static inline int spi_stm32_hw_bt_set(struct spi_stm32 *c, int bt)
{
#if defined(CONFIG_ARCH_STM32F7)
	unsigned int v = readl(&SPI(c)->spi_cr2);
	int ret = 0;

	v &= ~SPI_CR2_DS(0xF);
	v |= SPI_CR2_DS(bt - 1) | (bt == 8 ? SPI_CR2_FRXTH : 0);
	writel(v, &SPI(c)->spi_cr2);

	d_printk(2, "bus=%d,bt=%d,spi_cr2=%x,ret=%d\n",
		 c->bus, bt, readl(&SPI(c)->spi_cr2), ret);
	return ret;
#else
	unsigned int v = readl(&SPI(c)->spi_cr1);
	int ret = 0;

	if (bt == 8) {
		v &= ~SPI_CR1_DFF;
	}
	else {
		v |= SPI_CR1_DFF;
	}
	writel(v, &SPI(c)->spi_cr1);

	d_printk(2, "bus=%d,bt=%d,spi_cr1=%x,ret=%d\n", 
		 c->bus, bt, readl(&SPI(c)->spi_cr1), ret);
	return ret;
#endif
}

/*
 * Set transfer length
 * @param c		controller data structure
 * @param len		transfer size
 */
static inline void spi_stm32_hw_tfsz_set(struct spi_stm32 *c, int len)
{
	/*
	 * This is a dummy for PIO mode
	 */
}

/*
 * Set SPI mode
 * @param c		controller data structure
 * @param mode		mode
 * @returns		0->good;!=0->bad
 */
static inline int spi_stm32_hw_mode_set(struct spi_stm32 *c, unsigned int mode)
{
	int ret = 0;

	/*
 	 * Disable the SPI contoller 
 	 */
	writel(~SPI_CR1_SPE & readl(&SPI(c)->spi_cr1), &SPI(c)->spi_cr1);

	/*
 	 * Set the mode
 	 */
	if (mode & SPI_CPHA) {
		writel(SPI_CR1_CPHA | readl(&SPI(c)->spi_cr1),
			&SPI(c)->spi_cr1);
	}
	else {
		writel(~SPI_CR1_CPHA & readl(&SPI(c)->spi_cr1),
			&SPI(c)->spi_cr1);
	}
	if (mode & SPI_CPOL) {
		writel(SPI_CR1_CPOL | readl(&SPI(c)->spi_cr1),
			&SPI(c)->spi_cr1);
	}
	else {
		writel(~SPI_CR1_CPOL & readl(&SPI(c)->spi_cr1),
			&SPI(c)->spi_cr1);
	}

	/*
 	 * Re-enable the SPI contoller 
 	 */
	writel(SPI_CR1_SPE | readl(&SPI(c)->spi_cr1), &SPI(c)->spi_cr1);

	d_printk(2, "bus=%d,mode=%x,spi_cr1=%x,ret=%d\n", 
		 c->bus, mode, readl(&SPI(c)->spi_cr1), ret);
	return ret;
}

/*
 * Is transmit FIFO full?
 * @param c		controller data structure
 * @returns		!0->full;0->not full
 */
static inline int spi_stm32_hw_txfifo_full(struct spi_stm32 *c)
{
	return !(readl(&SPI(c)->spi_sr) & SPI_SR_TXE);
}

/*
 * Put a frame into the transmit FIFO
 * @param c		controller data structure
 * @param wb		frame size in full bytes
 * @param tx		transmit buf (can be NULL)
 * @param i		index of frame in buf
 */
static inline void spi_stm32_hw_txfifo_put(
	struct spi_stm32 *c, int wb, const void *tx, int i)
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
	writel(d, &SPI(c)->spi_dr);
}

/*
 * Is receive FIFO empty?
 * @param c		controller data structure
 * @returns		!0->empty,0->not empty
 */
static inline int spi_stm32_hw_rxfifo_empty(struct spi_stm32 *c)
{
	return !(readl(&SPI(c)->spi_sr) & SPI_SR_RXNE);
}

/*
 * Is receive FIFO overflown?
 * @param c		controller data structure
 * @returns		!0->error,0->no error
 */
static inline int spi_stm32_hw_rxfifo_error(struct spi_stm32 *c)
{
	return readl(&SPI(c)->spi_sr) & 
		(SPI_SR_FRE | SPI_SR_UDR);
}

/*
 * Retrieve a frame from the receive FIFO
 * @param c		controller data structure
 * @param wb		frame size in full bytes
 * @param rx		receive buf (can be NULL)
 * @param i		index of frame in buf
 */
static inline void spi_stm32_hw_rxfifo_get(
	struct spi_stm32 *c, unsigned int wb, void *rx, int i)
{
	int j;
	unsigned long d = readl(&SPI(c)->spi_dr);
	unsigned char *p = (unsigned char *)rx;

	if (p) {
		for (j = wb-1; j >= 0; j--) {
			p[i*wb + j] = d & 0xFF;
			d >>= 8;
		}
	}
}

/*
 * Clean-up receive FIFO
 * @param c		controller data structure
 * @param rx		receive buf (can be NULL)
 * @param i		index of frame in buf
 */
static inline void spi_stm32_hw_rxfifo_purge(struct spi_stm32 *c) 
{
	unsigned long d;

	while (readl(&SPI(c)->spi_sr) & SPI_SR_RXNE) {
		d = readl(&SPI(c)->spi_dr);
	}
}

/*
 * Hardware shutdown of the SPI controller
 * @param c		controller data structure
 */
static void spi_stm32_hw_release(struct spi_stm32 *c)
{
	unsigned int v;

	/*
 	 * Disable the SPI contoller
 	 */
	writel(~SPI_CR1_SPE & readl(&SPI(c)->spi_cr1), &SPI(c)->spi_cr1);

	/*
 	 * Put the SPI controller into reset.
 	 * Disable clock to the SPI controller.
 	 */
	if (c->bus == 0) {		/* SPI1 */
		v = readl(&STM32_RCC->apb2rstr);
		writel(v | RCC_APB2RSTR_SPI1, &STM32_RCC->apb2rstr);
		v = readl(&STM32_RCC->apb2enr);
		writel(v & ~RCC_APB2ENR_SPI1, &STM32_RCC->apb2enr);
	}
	else if (c->bus == 1) {		/* SPI2 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_SPI2, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v & ~RCC_APB1ENR_SPI2, &STM32_RCC->apb1enr);
	}
	else if (c->bus == 2) {		/* SPI3 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_SPI3, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v & ~RCC_APB1ENR_SPI3, &STM32_RCC->apb1enr);
	}
	else if (c->bus == 3) {		/* SPI4 */
		v = readl(&STM32_RCC->apb2rstr);
		writel(v | RCC_APB2RSTR_SPI4, &STM32_RCC->apb2rstr);
		v = readl(&STM32_RCC->apb2enr);
		writel(v & ~RCC_APB2ENR_SPI4, &STM32_RCC->apb2enr);
	}
	else if (c->bus == 4) {		/* SPI5 */
		v = readl(&STM32_RCC->apb2rstr);
		writel(v | RCC_APB2RSTR_SPI5, &STM32_RCC->apb2rstr);
		v = readl(&STM32_RCC->apb2enr);
		writel(v & ~RCC_APB2ENR_SPI5, &STM32_RCC->apb2enr);
	}
	else if (c->bus == 5) {		/* SPI6 */
		v = readl(&STM32_RCC->apb2rstr);
		writel(v | RCC_APB2RSTR_SPI6, &STM32_RCC->apb2rstr);
		v = readl(&STM32_RCC->apb2enr);
		writel(v & ~RCC_APB2ENR_SPI6, &STM32_RCC->apb2enr);
	}

	d_printk(2, "bus=%d\n", c->bus);
}

/*
 * Prepare to transfer to a slave
 * @param c		controller data structure
 * @param s		slave data structure
 * @returns		0->success, <0->error code
 */
static int spi_stm32_prepare_for_slave(
	struct spi_stm32 *c, struct spi_device *s)
{
	unsigned int spd;
	int ret = 0;

	/*
 	 * Set for this slave: frame size, clock, mode 
 	 */
	if (spi_stm32_hw_bt_set(c, s->bits_per_word)) {
		dev_err(&c->slave->dev, "unsupported frame size: %d\n",
			s->bits_per_word);
		ret = -EINVAL;
		goto Done;
	}
	if (spi_stm32_hw_clk_set(c, spd = min(s->max_speed_hz, c->speed_hz))) {
		dev_err(&c->slave->dev, "slave rate too low: %d\n", spd);
		ret = -EINVAL;
		goto Done;
	}
	if (spi_stm32_hw_mode_set(c, s->mode)) {
		dev_err(&c->slave->dev, "unsupported mode: %x\n", s->mode);
		ret = -EINVAL;
		goto Done;
	}

Done:
	d_printk(2, "slv=%s,ret=%d\n", dev_name(&c->slave->dev), ret);
	return ret;
}

/*
 * Capture a slave
 * @param c		controller data structure
 * @param s		slave data structure
 */
static void spi_stm32_capture_slave(struct spi_stm32 *c, struct spi_device *s)
{
	struct spi_stm32_slv *v = s->controller_data;

	/*
 	 * Activate CS for this slave
 	 */
	if (spi_stm32_hw_cs_set(c, v->cs_gpio, 1)) {
		dev_err(&c->slave->dev, "incorrect chip select: %d\n", 
			s->chip_select);
		goto Done;
	}

Done:
	d_printk(3, "slv=%s\n", dev_name(&c->slave->dev));
}

/*
 * Release a slave
 * @param c		controller data structure
 * @param s		slave data structure
 */
static void spi_stm32_release_slave(struct spi_stm32 *c, struct spi_device *s)
{
	struct spi_stm32_slv *v = s->controller_data;

	/*
 	 * Release CS for this slave
 	 */
	if (spi_stm32_hw_cs_set(c, v->cs_gpio, 0)) {
		dev_err(&c->slave->dev, "incorrect chip select: %d\n", 
			s->chip_select);
		goto Done;
	}

Done:
	d_printk(3, "slv=%s\n",
		c->slave ? dev_name(&c->slave->dev) : "");
}

/*
 * Prepare for a transfer
 * @param c		controller data structure
 * @param s		slave data structure
 */
static void inline spi_stm32_xfer_init(
	struct spi_stm32 *c, struct spi_device *s)
{
	struct spi_transfer *t;

	/*
 	 * Count the total length of the message.
 	 */
	c->len = 0;
	c->wb = (s->bits_per_word + 7) / 8;
	list_for_each_entry(t, &c->msg->transfers, transfer_list) {
		c->len += t->len;
	}
	c->len /= c->wb;

	/*
 	 * Set the size of the transfer in the SPI controller
 	 */
	spi_stm32_hw_tfsz_set(c, c->len);

	/*
 	 * Prepare to traverse the message list.
 	 * We will need to advance separately over 
 	 * transmit and receive data
 	 */
	c->tx_t = list_entry((&c->msg->transfers)->next,
		   struct spi_transfer, transfer_list);
	c->tx_l = c->tx_t->len / c->wb;
	c->tx_i = 0;
	c->ti = 0;
	c->rx_t = list_entry((&c->msg->transfers)->next,
		   struct spi_transfer, transfer_list);
	c->rx_l = c->rx_t->len / c->wb;
	c->rx_i = 0;
	c->ri = 0;
}

/*
 * Advance to next Tx frame
 * @param c		controller data structure
 * @param x		xfer to advance to
 */
static void inline spi_stm32_xfer_tx_next(struct spi_stm32 *c)
{
	unsigned long f;

	/*
 	 * If the trasmit in the current transfer
 	 * has been finished, go to the next one.
 	 */
	while (c->tx_i == c->tx_l) {
		c->tx_t = list_entry(c->tx_t->transfer_list.next, 
	                         struct spi_transfer, transfer_list);
		c->tx_l = c->tx_t->len / c->wb;
		c->tx_i = 0;
	}

	/*
 	 * Put a frame to the transmit fifo
 	 */
	spin_lock_irqsave(&c->lock, f);
	spi_stm32_hw_txfifo_put(c, c->wb, c->tx_t->tx_buf, c->tx_i);
	c->tx_i++;
	c->ti++;
	spin_unlock_irqrestore(&c->lock, f);
}

/*
 * Advance to next Rx frame
 * @param c		controller data structure
 * @param x		xfer to advance to
 */
static void inline spi_stm32_xfer_rx_next(struct spi_stm32 *c)
{
	unsigned long f;

	/*
 	 * If the receive in the current transfer
   	 * has been finished, go to the next one.
  	 */
	while (c->rx_i == c->rx_l) {

		/*
 	 	 * Advance to the next transfer
 	 	 */
		c->rx_t = list_entry(c->rx_t->transfer_list.next, 
	                     	  struct spi_transfer, transfer_list);
		c->rx_l = c->rx_t->len / c->wb;
		c->rx_i = 0;
	}

	/* 
  	 * Read in a frame 
  	 */
	spin_lock_irqsave(&c->lock, f);
	spi_stm32_hw_rxfifo_get(c, c->wb, c->rx_t->rx_buf, c->rx_i);
	c->rx_i++;
	c->ri++;
	spin_unlock_irqrestore(&c->lock, f);
}

#if defined(CONFIG_SPI_STM32_POLLED)

/*
 * Transfer a message in PIO, polled mode
 * @param c		controller data structure
 * @param s		slave data structure
 * @param		pointer to actual transfer length (set here)
 * @returns		0->success, <0->error code
 */
static int spi_stm32_pio_polled(
	struct spi_stm32 *c, struct spi_device *s, int *rlen)
{
	int i;
	int ret = 0;

	/*
 	 * Prepare to run a transfer
 	 */
	spi_stm32_xfer_init(c, s);

	/*
 	 * Perform the transfer. Transfer is done when all frames
 	 * have been received (i.e. ri == len). Each time we
 	 * iterate in this loop, we have received a next frame.
 	 */
	while (c->ri < c->len) {

		/*
 		 * Transfer a frame
 		 */
	        for (i = 0; 
		     i < 1 && c->ti < c->len && !spi_stm32_hw_txfifo_full(c);
		     i++) {
			spi_stm32_xfer_tx_next(c);
		}

		/*
 		 * Wait for a frame to come in
 		 * but check for error conditions first
 		 */
		if (spi_stm32_hw_rxfifo_error(c)) {

			/*
			 * If there is an error, this transfer
			 * needs to be finished with an error.
			 */
			ret = -EIO;
			goto Done;
		}

		/*
  	 	 * Receive a frame
 	 	 */
		while (spi_stm32_hw_rxfifo_empty(c));
		spi_stm32_xfer_rx_next(c);
	}

	/*
 	 * Return the number of bytes actully transferred
 	 */
	*rlen = c->ri;
Done:
	spi_stm32_hw_rxfifo_purge(c);
	d_printk(3, "msg=%p,len=%d,rlen=%d,ret=%d\n", 
		c->msg, c->len, *rlen, ret);
	return ret;
}

#else

/*
 * Interrupt handler routine
 */
static irqreturn_t spi_stm32_irq(int irq, void *dev_id)
{
	struct spi_stm32 *c = dev_id;
#if defined(SPI_STM32_DEBUG)
	int sr = readl(&SPI(c)->spi_sr);
#endif

	if (! spi_stm32_hw_rxfifo_empty(c)) {

		/*
		 * Read in a frame
		 */
		spi_stm32_xfer_rx_next(c);

		/*
		 * If the entire transfer has been received, that's it.
		 */
		if (c->ri == c->len) {
			spi_stm32_hw_rxfifo_purge(c);
			c->xfer_status = 0;
			wake_up(&c->wait);
		}
	}

	/*
	 * Push a next frame out
	 */
	if (c->ti < c->len) {

		/*
		 * Write a frame
		 */
		while (spi_stm32_hw_txfifo_full(c));
		spi_stm32_xfer_tx_next(c);
	}

	d_printk(4, "ok: sr=%x\n", sr);
	return IRQ_HANDLED;
}

/*
 * Transfer a message in PIO, interrupted mode
 * @param c		controller data structure
 * @param s		slave data structure
 * @param		pointer to actual transfer length (set here)
 * @returns		0->success, <0->error code
 */
static int spi_stm32_pio_interrupted(
	struct spi_stm32 *c, struct spi_device *s, int *rlen)
{
	struct spi_stm32_slv *v = s->controller_data;
	int ret = 0;

	/*
 	 * Prepare to run a transfer
 	 */
	spi_stm32_xfer_init(c, s);

	/*
 	 * Start the transfer
 	 */
	c->xfer_status = -EBUSY;
	*rlen = 0;
	spi_stm32_xfer_tx_next(c);

	/*
	 * Wait for the transfer to complete, one way or another
	 */
	if (wait_event_interruptible_timeout(c->wait, 
		c->xfer_status != -EBUSY, 
		(v->timeout ? v->timeout : 1) * HZ) == 0) {
		ret = -ETIMEDOUT;
	} else {
		ret = c->xfer_status;
		if (!ret) {
			/*
			 * Success ->
 	 		 * Return the number of bytes actully transferred
 	 		 */
			*rlen = c->ri;
		}
	}
	c->xfer_status = 0;

	d_printk(3, "msg=%p,len=%d,rlen=%d,ret=%d\n", 
		c->msg, c->len, *rlen, ret);
	return ret;
}

#endif

/*
 * Transfer a message 
 * @param c		controller data structure
 * @param m		message
 * @returns		0 -> success, negative error code -> error
 */
static int spi_stm32_handle_message(
	struct spi_stm32 *c, struct spi_message *msg)
{
#if defined(SPI_STM32_DEBUG)
	struct spi_transfer *t;
#endif
	struct spi_device *s = msg->spi;
	int rlen = 0;
	int ret = 0;


	/*
 	 * Check if the current slave of this controller is
 	 * the message's SPI device, and if not, set 
 	 * the speed and mode select for the slave
 	 */
	if (c->slave != s) {
		c->slave = s;
		ret = spi_stm32_prepare_for_slave(c, s);
		if (ret) {
			c->slave = NULL;
			goto Done;
		}
	}

	/*
	 * Activate chip select for the slave
	 */
	spi_stm32_capture_slave(c, s);

	/*
 	 * Transfer the message over the wire
 	 */
	c->msg = msg;
#if defined(CONFIG_SPI_STM32_POLLED)
	ret = spi_stm32_pio_polled(c, s, &rlen);
#else
	ret = spi_stm32_pio_interrupted(c, s, &rlen);
#endif
	if (ret) {
		goto Done;
	}

	msg->actual_length = rlen;

Done:
	/*
	 * Release chip select for the slave
	 */
	spi_stm32_release_slave(c, s);

#if defined(SPI_STM32_DEBUG)
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		int i; 
		char * p;

		d_printk(3, "t=%p,tx=%p,rx=%p,len=%d,rlen=%d\n",
		         t, t->tx_buf, t->rx_buf, t->len, msg->actual_length);
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
static void spi_stm32_handle(struct work_struct *w)
{
	struct spi_message *msg;
	struct spi_stm32 *c = container_of(w, struct spi_stm32, work);
	unsigned long f = 0;

	/*
	 * In atomic manner 
	 */
	spin_lock_irqsave(&c->lock, f);

	/*
 	 * Check if the message queue has messages
 	 */
	while (!c->xfer_status && !list_empty(&c->queue)) {

		/*
		 * Extract the next message from the queue
		 */
		msg = list_entry(c->queue.next, struct spi_message, queue);
		list_del_init(&msg->queue);

		spin_unlock_irqrestore(&c->lock, f);

		/*
		 * Transfer the message over SPI wires
		 */
		msg->status = spi_stm32_handle_message(c, msg);

		/*
 		 * Let the upper layers complete processing of the message
 		 */
		msg->complete(msg->context);

		/*
 		 * Re-acquire the lock and go check the message list again
 		 */
		spin_lock_irqsave(&c->lock, f);
	}

	/*
	 * Release the lock and return
	 */
	spin_unlock_irqrestore(&c->lock, f);
}

/*
 * Set up the SPI controller for a specified SPI slave device
 * @param s		SPI slave
 * @returns		0->success, <0->error code
 */
static int spi_stm32_setup(struct spi_device *s)
{
	struct spi_stm32 *c = spi_master_get_devdata(s->master);
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
	if (spi_stm32_hw_bt_check(c, s->bits_per_word)) {
		dev_err(&s->dev, "unsupported bits per word %d\n", 
			s->bits_per_word);
		ret = -EINVAL;
		goto Done;
	}

	/*
	 * Make sure Chip Select is inactive for this slave
	 */
	spi_stm32_release_slave(c, s);

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
static void spi_stm32_cleanup(struct spi_device *s)
{
	d_printk(1, "slv=%s\n", dev_name(&s->dev));
}

/*
 * Transfer a message to a specified SPI slave
 * @s			SPI slave
 * @m			message to transfer
 * @returns		0->success; <0->error code
 */
static int spi_stm32_transfer(struct spi_device *s, struct spi_message *msg)
{
	struct spi_stm32 *c = spi_master_get_devdata(s->master);
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
	 * Prepare the message for transfer
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
 * Instantiate an SPI controller
 * @dev			SPI controller platform device
 * @returns		0->success, <0->error code
 */
static int __devinit spi_stm32_probe(struct platform_device *dev)
{
	struct spi_master *m = NULL;
	struct spi_stm32 *c = NULL;
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
#if !defined(CONFIG_SPI_STM32_POLLED)
	ret = request_irq(irq, spi_stm32_irq, 0, dev_name(&dev->dev), c);
	if (ret) {
		dev_err(&dev->dev, "request irq %d failed for "
			"SPI controller %d\n", irq, bus);
		goto Error_release_regs;
	}
	c->irq = irq;

	/* 
 	 * Set up the wait queue
 	 */
	init_waitqueue_head(&c->wait);
#endif

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
	INIT_WORK(&c->work, spi_stm32_handle);

	/*
 	 * Set up queue of messages
 	 */
	INIT_LIST_HEAD(&c->queue);

	/*
 	 * Set up the lock
 	 */
	spin_lock_init(&c->lock);
	c->xfer_status = 0;

	/* 
 	 * Initialize the controller hardware
 	 */
	if (spi_stm32_hw_init(c)) {
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
	m->num_chipselect = spi_stm32_hw_cs_max(c);

	/*
 	 * Set-up SPI slave action callbacks
 	 */
	m->setup = spi_stm32_setup;
	m->cleanup = spi_stm32_cleanup;
	m->transfer = spi_stm32_transfer;
	c->slave = NULL;

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
#if !defined(CONFIG_SPI_STM32_POLLED)
	dev_info(&dev->dev, "SPI Controller %d at %p,irq=%d,hz=%d\n",
		 m->bus_num, c->regs, c->irq, c->speed_hz);
#else
	dev_info(&dev->dev, "SPI Controller %d at %p,hz=%d\n",
		 m->bus_num, c->regs, c->speed_hz);
#endif
	goto Done;

	/*
	 * Error processing
	 */
Error_release_hardware: 
	spi_stm32_hw_release(c);
Error_release_workqueue: 
	destroy_workqueue(c->workqueue);
Error_release_irq: 
#if !defined(CONFIG_SPI_STM32_POLLED)
	free_irq(c->irq, c);
Error_release_regs: 
#endif
	iounmap(c->regs);
Error_release_master: 
	spi_master_put(m);
	platform_set_drvdata(dev, NULL);
Error_release_nothing: 
	
	/*
	 * Exit point
	 */
Done:
#if !defined(CONFIG_SPI_STM32_POLLED)
	d_printk(1, "dev=%s,regs=%p,irq=%d,hz=%d,ret=%d\n", 
		 dev_name(&dev->dev), 
		 c? c->regs : NULL, c ? c->irq : 0, c ? c->speed_hz : 0, 
		 ret);
#else
	d_printk(1, "dev=%s,regs=%p,hz=%d,ret=%d\n", 
		 dev_name(&dev->dev), 
		 c? c->regs : NULL, c ? c->speed_hz : 0, ret);
#endif
	return ret;
}

/*
 * Shutdown of an instance of the controller device
 * @dev			SPI controller platform device
 * @returns		0->success, <0->error code
 */
static int __devexit spi_stm32_remove(struct platform_device *dev)
{
	struct spi_master *m  = platform_get_drvdata(dev);
	struct spi_stm32 *c = spi_master_get_devdata(m);
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
#if !defined(CONFIG_SPI_STM32_POLLED)
	free_irq(c->irq, c);
#endif
	destroy_workqueue(c->workqueue);
	iounmap(c->regs);
	spi_master_put(m);
	platform_set_drvdata(dev, NULL);

	/*
 	 * Shut the hardware down
 	 */
	spi_stm32_hw_release(c);

	/*
	 * Exit point
	 */
	d_printk(1, "dev=%s,ret=%d\n", dev_name(&dev->dev), ret);
	return ret;
}

/*
 * Platform driver data structure
 */
static struct platform_driver spi_stm32_drv = {
	.probe	= spi_stm32_probe,
	.remove	= __devexit_p(spi_stm32_remove),
	.driver = {
		.name = "spi_stm32",
		.owner = THIS_MODULE,
	},
};

/*
 * Driver init
 */
static int __init spi_stm32_module_init(void)
{
	int ret;
	
	ret = platform_driver_register(&spi_stm32_drv);

	d_printk(1, "drv=%s,ret=%d\n", spi_stm32_drv.driver.name, ret);
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit spi_stm32_module_exit(void)
{
	platform_driver_unregister(&spi_stm32_drv);

	d_printk(1, "drv=%s\n", spi_stm32_drv.driver.name);
}

module_init(spi_stm32_module_init);
module_exit(spi_stm32_module_exit);
MODULE_AUTHOR("Vladimir Khusainov, <vlad@emcraft.com>");
MODULE_DESCRIPTION("Device driver for the STM32 SPI controller");
MODULE_LICENSE("GPL");
