/*
 * Device driver for the I2C controller of the STM32F7 microcontroller.
 * Authors: Vladimir Khusainov, vlad@emcraft.com
 *          Vladimir Skvortsov, vskvortsov@emcraft.com
 * Copyright 2013-2015 Emcraft Systems
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/i2c.h>
#include <mach/stm32.h>

#include <mach/dmac.h>
#include <linux/dmamem.h>

/*
 * Debug output control. While debugging, have I2C_STM32_DEBUG defined.
 * In deployment, make sure that I2C_STM32_DEBUG is undefined
 * to avoid performance and size overhead of debug messages.
 */
#define I2C_STM32_DEBUG
#if 1
#undef I2C_STM32_DEBUG
#endif

#if defined(I2C_STM32_DEBUG)

/*
 * Driver verbosity level: 0->silent; >0->verbose (1 to 4, growing verbosity)
 */
static int i2c_stm32_debug = 4;

/*
 * User can change verbosity of the driver (when loading the module,
 * or the kernel, in case the driver is linked in statically,
 * but not through a /sysfs parameter file)
 */
module_param(i2c_stm32_debug, int, S_IRUGO);
MODULE_PARM_DESC(i2c_stm32_debug, "I2C controller driver verbosity level");

#if !defined(MODULE)

/*
 * Parser for the boot time driver verbosity parameter
 * @param str		user definition of the parameter
 * @returns		1->success
 */
static int __init i2c_stm32_debug_setup(char * str)
{
	get_option(&str, &i2c_stm32_debug);
	return 1;
}
__setup("i2c_stm32_debug=", i2c_stm32_debug_setup);

#endif /* !defined(MODULE) */

/*
 * Service to print debug messages
 */
#define d_printk(level, fmt, args...)					\
	if (i2c_stm32_debug >= level) printk(KERN_INFO "%s: " fmt,	\
					   __func__, ## args)

#else

#define d_printk(level, fmt, args...)

#endif /* defined(I2C_STM32_DEBUG) */

/*
 * I2C controller private data structure
 */
struct i2c_stm32f7 {
	struct platform_device *	dev;		/* Platform device */
	int				bus;		/* Bus (ID) */
	unsigned int			regs_base;	/* Regs base (phys) */
	unsigned int			regs_size;	/* Regs size */
	void * __iomem			regs;		/* Regs base (virt) */
	int				irq;		/* IRQ # */
	unsigned int			ref_clk;	/* Ref clock */
	unsigned int			i2c_clk;	/* Bus clock */
	volatile int			msg_status;	/* Message status */
	struct i2c_adapter		adap;		/* I2C adapter data */
	wait_queue_head_t		wait;		/* Wait queue */
	int				nbytes;		/* Count of the remained bytes after reload */
	caddr_t				dmabuf;		/* Buffer in non-cached region to perform DMA */
	int				dma_ch_tx;	/* DMA channel configured for I2C TX */
	int				dma_ch_rx;	/* DMA channel configured for I2C RX */
};

/*
 * Description of the the STM32F7 I2C hardware interfaces.
 */
struct i2c_stm32f7_regs {
	unsigned int			cr1;
	unsigned int			cr2;
	unsigned int			oar1;
	unsigned int			oar2;
	unsigned int			timingr;
	unsigned int			timeoutr;
	unsigned int			isr;
	unsigned int			icr;
	unsigned int			pecr;
	unsigned int			rxdr;
	unsigned int			txdr;
};
/*
 * Access handle for the control registers
 */
#define I2C_STM32F7_REGS(r)		((volatile struct i2c_stm32f7_regs *)(r))
#define I2C_STM32F7(c)			(I2C_STM32F7_REGS(c->regs))

/*
 * Convert to MHz
 */
#define MHZ(v)				((v) * 1000000)

#define I2C_TIMEOUT_BUSY	25	/* 25 ms */

/* DMA buffer is allocated from non-cached region in order
   to guarantee validity of the DMA transactions when cache
   is enabled. */
#define DMABUF_SIZE		1024

/*
 * Some bits in various CSRs
 */
#define RCC_APB1RSTR_I2C1		(1<<21)
#define RCC_APB1RSTR_I2C2		(1<<22)
#define RCC_APB1RSTR_I2C3		(1<<23)
#define RCC_APB1ENR_I2C1		(1<<21)
#define RCC_APB1ENR_I2C2		(1<<22)
#define RCC_APB1ENR_I2C3		(1<<23)

/* CR1 register bits */
#define	_CR1_PECEN		(1<<23)
#define	_CR1_ALERTEN		(1<<22)
#define	_CR1_SMBDEN		(1<<21)
#define	_CR1_SMBHEN		(1<<20)
#define	_CR1_GCEN		(1<<19)
#define	_CR1_NOSTRETCH		(1<<17)
#define	_CR1_SBC		(1<<16)
#define	_CR1_RXDMAEN		(1<<15)
#define	_CR1_TXDMAEN		(1<<14)
#define	_CR1_ANFOFF		(1<<12)
#define	_CR1_DNF(x)		(((x) & 0xf) << 8)
#define	_CR1_ERRIE		(1<<7)
#define	_CR1_TCIE		(1<<6)
#define	_CR1_STOPIE		(1<<5)
#define	_CR1_NACKIE		(1<<4)
#define	_CR1_ADDRIE		(1<<3)
#define	_CR1_RXIE		(1<<2)
#define	_CR1_TXIE		(1<<1)
#define	_CR1_PE			(1<<0)

/* CR2 register bits */
#define	_CR2_PECBYTE		(1<<26)
#define	_CR2_AUTOEND		(1<<25)
#define	_CR2_RELOAD		(1<<24)
#define _CR2_NBYTES(x)		(((x) & 0xff)<<16)
#define	_CR2_NACK		(1<<15)
#define	_CR2_STOP		(1<<14)
#define	_CR2_START		(1<<13)
#define	_CR2_HEAD10R		(1<<12)
#define	_CR2_ADD10		(1<<11)
#define	_CR2_RD_WRN		(1<<10)
#define	_CR2_SADDR7(x)		(((x) & 0x7f)<<1)
#define	_CR2_SADDR10(x)		(((x) & 0x3ff) | (1 << 11))

/* Timing register values */
#define EVAL_I2Cx_TIMING	0x40912732	/* from STM alpha program */
#define STM32F7_I2Cx_TIMING	0x10806190	/* calculated using a program from AN4235 for 100KHz of the I2C clock and 50MHz of the source clock */

/* ISR register bits */
#define	_ISR_DIR		(1<<16)
#define	_ISR_BUSY		(1<<15)
#define	_ISR_ALERT		(1<<13)
#define	_ISR_TIMEOUT		(1<<12)
#define	_ISR_PECERR		(1<<11)
#define	_ISR_OVR		(1<<10)
#define	_ISR_ARLO		(1<<9)
#define	_ISR_BERR		(1<<8)
#define	_ISR_TCR		(1<<7)
#define	_ISR_TC			(1<<6)
#define	_ISR_STOPF		(1<<5)
#define	_ISR_NACKF		(1<<4)
#define	_ISR_ADDR		(1<<3)
#define	_ISR_RXNE		(1<<2)
#define	_ISR_TXIS		(1<<1)
#define	_ISR_TXE		(1<<0)

/* ICR register bits */
#define	_ICR_STOPCF		(1<<5)
#define	_ICR_NACKCF		(1<<4)

static inline void _cr1_set_val(struct i2c_stm32f7 *c, uint32_t val)
{
	uint32_t cr1 = readl(&I2C_STM32F7(c)->cr1);

	/* clear cr1 specific bits (do not clear reserved bits) */
	cr1 &= (uint32_t)~((uint32_t)(
				      _CR1_PECEN
				      | _CR1_ALERTEN
				      | _CR1_SMBDEN
				      | _CR1_SMBHEN
				      | _CR1_GCEN
				      | _CR1_NOSTRETCH
				      | _CR1_SBC
				      | _CR1_RXDMAEN
				      | _CR1_TXDMAEN
				      | _CR1_ANFOFF
				      | _CR1_DNF(0xf)
				      | _CR1_ERRIE
				      | _CR1_TCIE
				      | _CR1_STOPIE
				      | _CR1_NACKIE
				      | _CR1_ADDRIE
				      | _CR1_RXIE
				      | _CR1_TXIE
				      | _CR1_PE
				      ));

	/* set the desired bits */
	cr1 |= val;
	writel(cr1, &I2C_STM32F7(c)->cr1);
}

static inline void _cr1_set_bits(struct i2c_stm32f7 *c, uint32_t mask)
{
	uint32_t cr1 = readl(&I2C_STM32F7(c)->cr1);

	/* set the desired bits */
	cr1 |= mask;
	writel(cr1, &I2C_STM32F7(c)->cr1);
}

static inline void _cr1_clear_bits(struct i2c_stm32f7 *c, uint32_t mask)
{
	uint32_t cr1 = readl(&I2C_STM32F7(c)->cr1);

	/* clear the desired bits */
	cr1 &= ~mask;
	writel(cr1, &I2C_STM32F7(c)->cr1);
}

static inline void _cr2_set_val(struct i2c_stm32f7 *c, uint32_t val)
{
	uint32_t cr2 = readl(&I2C_STM32F7(c)->cr2);

	/* clear cr2 specific bits (do not clear reserved bits) */
	cr2 &= (uint32_t)~((uint32_t)(_CR2_PECBYTE
				      | _CR2_AUTOEND
				      | _CR2_RELOAD
				      | _CR2_NBYTES(0xff)
				      | _CR2_NACK
				      | _CR2_STOP
				      | _CR2_START
				      | _CR2_HEAD10R
				      | _CR2_ADD10
				      | _CR2_RD_WRN
				      |_CR2_SADDR10(0x3ff)
				      ));

	/* set the desired bits */
	cr2 |= val;
	writel(cr2, &I2C_STM32F7(c)->cr2);
}

/*
 * Hardware initialization of the I2C controller
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static int i2c_stm32_hw_init(struct i2c_stm32f7 *c)
{
	unsigned int v;
	int ret = 0;

	/*
	 * First, figure out if we are able to configure the clocks
	 * If not, we want to bail out without enabling anything.
	 */
	if (c->i2c_clk == 0 || c->i2c_clk > 400000) {
		dev_err(&c->dev->dev, "bus clock %d not supported\n",
			c->i2c_clk);
		ret = -ENXIO;
		goto Done;
	}

	/*
	 * Reset the I2C controller and then bring it out of reset.
	 * Enable the I2C controller clock.
	 */
	if (c->bus == 0) {		/* I2C1 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_I2C1, &STM32_RCC->apb1rstr);
		writel(v & ~RCC_APB1RSTR_I2C1, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v | RCC_APB1ENR_I2C1, &STM32_RCC->apb1enr);
	}
	else if (c->bus == 1) {		/* I2C2 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_I2C2, &STM32_RCC->apb1rstr);
		writel(v & ~RCC_APB1RSTR_I2C2, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v | RCC_APB1ENR_I2C2, &STM32_RCC->apb1enr);
	}
	else if (c->bus == 2) {		/* I2C3 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_I2C3, &STM32_RCC->apb1rstr);
		writel(v & ~RCC_APB1RSTR_I2C3, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v | RCC_APB1ENR_I2C3, &STM32_RCC->apb1enr);
	}

	/*
	 * Reset the controller to clear possible errors or locks
	 */
	_cr2_set_val(c, 0);

	_cr1_clear_bits(c, _CR1_PE);

	/* wait the reset */
	do {
		v = readl(&I2C_STM32F7(c)->cr1);
	} while ((v & _CR1_PE) != 0);

	v = readl(&I2C_STM32F7(c)->timingr);

	v &= 0x0f000000;
	v |= STM32F7_I2Cx_TIMING;

	writel(v, &I2C_STM32F7(c)->timingr);

	/*
	 * Enable the I2C controller
	 */
	_cr1_set_bits(c, _CR1_PE);

	/* Init DMA */

	/*
	 * Direction: peripheral-to-memory
	 * Flow controller: peripheral
	 * Priority: very high (3)
	 * Double buffer mode: disabled
	 * Circular mode: disabled
	 */
	if (stm32_dma_ch_init(c->dma_ch_rx, 0, 1, 3, 0, 0) < 0) {
		goto err_dma;
	}

	/*
	 * Disable burst mode; set FIFO threshold to "full FIFO"
	 */
	if (stm32_dma_ch_init_fifo(c->dma_ch_rx, 0, 3) < 0) {
		goto err_dma;
	}

	/*
	 * Peripheral address: I2C controller receive data register
	 * Peripheral increment: disabled
	 * Peripheral data size: 8-bit
	 * Burst transfer configuration: incremental burst of 0 beats
	 */
	if (stm32_dma_ch_set_periph(c->dma_ch_rx,
				    (uint32_t)&I2C_STM32F7(c)->rxdr, 0, 0, 0)) {
		goto err_dma;
	}

	/*
	 * Direction: memory-to-peripheral
	 * Flow controller: peripheral
	 * Priority: very high (3)
	 * Double buffer mode: disabled
	 * Circular mode: disabled
	 */
	if (stm32_dma_ch_init(c->dma_ch_tx, 1, 1, 3, 0, 0) < 0) {
		goto err_dma;
	}

	/*
	 * Enable burst mode; set FIFO threshold to "full FIFO"
	 */
	if (stm32_dma_ch_init_fifo(c->dma_ch_tx, 0, 3) < 0) {
		goto err_dma;
	}

	/*
	 * Peripheral address: I2C controller transmit data register
	 * Peripheral increment: disabled
	 * Peripheral data size: 8-bit
	 * Burst transfer configuration: incremental burst of 0 beats
	 */
	if (stm32_dma_ch_set_periph(c->dma_ch_tx,
				    (uint32_t)&I2C_STM32F7(c)->txdr, 0, 0, 0) < 0) {
		goto err_dma;
	}

Done:
	d_printk(2, "bus=%d,"
		"cr1=0x%x,cr2=0x%x,isr=0x%x,ret=%d\n",
		c->bus,
		!ret ? readl(&I2C_STM32(c)->cr1) : 0,
		!ret ? readl(&I2C_STM32(c)->cr2) : 0,
		!ret ? readl(&I2C_STM32(c)->isr) : 0, ret);
	return ret;

 err_dma:
	return -1;
}

/*
 * Hardware shutdown of the I2C controller
 * @param c		controller data structure
 */
static void i2c_stm32_hw_release(struct i2c_stm32f7 *c)
{
	unsigned int v;

	/*
	 * Disable the controller
	 */
	_cr1_clear_bits(c, _CR1_PE);

	/*
	 * Put the I2C controller into reset.
	 * Disable clock to the I2C controller.
	 */
	if (c->bus == 0) {		/* I2C1 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_I2C1, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v & ~RCC_APB1ENR_I2C1, &STM32_RCC->apb1enr);
	}
	else if (c->bus == 1) {		/* I2C2 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_I2C2, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v & ~RCC_APB1ENR_I2C2, &STM32_RCC->apb1enr);
	}
	else if (c->bus == 2) {		/* I2C3 */
		v = readl(&STM32_RCC->apb1rstr);
		writel(v | RCC_APB1RSTR_I2C3, &STM32_RCC->apb1rstr);
		v = readl(&STM32_RCC->apb1enr);
		writel(v & ~RCC_APB1ENR_I2C3, &STM32_RCC->apb1enr);
	}

	d_printk(2, "bus=%d\n", c->bus);
}

/*
 * Interrupt handler routine
 * @param irq		IRQ number
 * @param d		controller data structure
 * @returns		IRQ handler exit code
 */
static irqreturn_t i2c_stm32_irq(int irq, void *d)
{
	struct i2c_stm32f7 *c = (struct i2c_stm32f7 *) d;
	uint32_t isr = readl(&I2C_STM32F7(c)->isr);

	if (isr & _ISR_STOPF) {
		uint32_t icr = _ICR_STOPCF;

		/* disable interrupts */
		_cr1_clear_bits(c, _CR1_ERRIE | _CR1_STOPIE);

		if (isr & _ISR_NACKF) {
			/* handle NACK event */
			icr |= _ICR_NACKCF;
			c->msg_status = -ENODEV;
		}

		if (c->msg_status == -EBUSY) {
			c->msg_status = 0;
		}

		/* clear the event */
		writel(icr, &I2C_STM32F7(c)->icr);

		/* cleanup the finished transfer */
		_cr2_set_val(c, 0);

		wake_up(&c->wait);
	} else if (isr & _ISR_TCR) {
		uint32_t cr2 = readl(&I2C_STM32F7(c)->cr2);

		if (c->nbytes > 255) {
			c->nbytes -= 255;
			cr2 |= _CR2_NBYTES(255);
		} else {
			c->nbytes = 0;
			cr2 |= _CR2_NBYTES(c->nbytes);
			cr2 &= ~_CR2_RELOAD;
			_cr1_clear_bits(c, _CR1_TCIE);
		}

		_cr2_set_val(c, cr2);
	} else {
		/*
		 * Some error condition -> let's stop and report a failure
		 */
		_cr2_set_val(c, _CR2_STOP);

		c->msg_status = -EIO;

		dev_dbg(&c->dev->dev,
			"error condition in irq handler: isr=%x\n", isr);
	}

	return 0;
}

/*
 * Adapter transfer callback
 * @param a		I2C adapter
 * @param m		array of messages
 * @param n		number of messages
 * @returns		message segments transferred or error code (<1)
 */
static int i2c_stm32_transfer(struct i2c_adapter *a, struct i2c_msg *m, int n)
{
#if defined(I2C_STM32_DEBUG)
	int j;
#endif
	struct i2c_stm32f7 *c = a->algo_data;
	int ret = 0;
	int i;

	for (i = 0; i < n; i++) {
		uint32_t cr2 = 0;
		int dma_ch;

		/* Setup transfer */

		/* Transfer length */

		/* Maximum size of a single transfer
		   is limited by the DMA buffer size. */
		if (m[i].len > DMABUF_SIZE) {
			if (ret == 0) {
				ret = -EINVAL;
			}
			break;
		}
		if (m[i].len > 255) {
			cr2 |= _CR2_NBYTES(255);
			c->nbytes = m[i].len - 255;
			cr2 |= _CR2_RELOAD;
			_cr1_set_bits(c, _CR1_TCIE);
		} else {
			c->nbytes = 0;
			cr2 |= _CR2_NBYTES(m[i].len);
		}

		/* Addressing mode */
		if (m[i].flags & I2C_M_TEN) {
			cr2 |= _CR2_SADDR10(m[i].addr);
		} else {
			cr2 |= _CR2_SADDR7(m[i].addr);
		}

		/* Direction */
		if (m[i].flags & I2C_M_RD) {
			cr2 |= _CR2_RD_WRN;
			dma_ch = c->dma_ch_rx;
		} else {
			memcpy(c->dmabuf, m[i].buf, m[i].len);
			dma_ch = c->dma_ch_tx;
		}

		/* Autogenerate STOP when finished */
		cr2 |= _CR2_AUTOEND;

		c->msg_status = -EBUSY;

		/*
		 * Setup and enable the DMA channel. After this point, the DMA transfer will
		 * be able to start.
		 */

		/*
		 * Memory address: DMA buffer address
		 * Memory incremental: enabled
		 * Memory data size: 8-bit
		 * Burst transfer configuration: no burst
		 */
		if (stm32_dma_ch_set_memory(dma_ch, (u32)c->dmabuf, 1, 0, 1) < 0) {
			goto err_dma;
		}

		if (stm32_dma_ch_set_nitems(dma_ch, m[i].len) < 0) {
			goto err_dma;
		}

		if (stm32_dma_ch_enable(dma_ch) < 0) {
			goto err_dma;
		}

		/* Enable interrupts */
		_cr1_set_bits(c, _CR1_ERRIE | _CR1_STOPIE);

		/* Put config and generate START */
		_cr2_set_val(c, cr2 | _CR2_START);

		if (m[i].len > 0) {
			/* Busy loop to wait the I2C controller is ready for DMA */
			unsigned long start = jiffies;
			unsigned long tmout_jiffies = (I2C_TIMEOUT_BUSY * HZ) / 1000;
			unsigned long tmout = 0;

			while (1) {
				uint32_t isr = readl(&I2C_STM32F7(c)->isr);
				if (m[i].flags & I2C_M_RD) {
					if (isr & _ISR_RXNE) {
						/* Start DMA */
						_cr1_set_bits(c, _CR1_RXDMAEN);
						break;
					}
				} else {
					if (isr & _ISR_TXIS) {
						/* Start DMA */
						_cr1_set_bits(c, _CR1_TXDMAEN);
						break;
					}
				}

				tmout = jiffies - start;

				if (tmout > tmout_jiffies) {
					break;
				}
			}

			if (tmout > tmout_jiffies) {
				/* Disable interrupts */
				_cr1_clear_bits(c, _CR1_ERRIE | _CR1_STOPIE | _CR1_TCIE);

				/* Clean up the transfer */
				_cr2_set_val(c, 0);
				if (ret == 0) {
					ret = -ETIMEDOUT;
				}
				stm32_dma_ch_disable(dma_ch);
				break;
			}
		}

		/*
		 * Wait for the transfer to complete, one way or another
		 */
		if (wait_event_timeout(c->wait, c->msg_status != -EBUSY, 5*HZ) == 0) {
			/* Disable interrupts */
			_cr1_clear_bits(c, _CR1_ERRIE | _CR1_STOPIE | _CR1_TCIE);

			/* Clean up the transfer */
			_cr2_set_val(c, 0);
			if (ret == 0) {
				ret = -ETIMEDOUT;
			}
		} else {
			if (c->msg_status != 0) {
				if (ret == 0) {
					ret = c->msg_status;
				}
			} else {
				ret ++;
			}
		}

		/* Cleanup DMA */
		_cr1_clear_bits(c, _CR1_TXDMAEN | _CR1_RXDMAEN);
		stm32_dma_ch_disable(dma_ch);

		if (ret != (i + 1)) {
			/* Error */
			break;
		}

		if (m[i].flags & I2C_M_RD) {
			memcpy(m[i].buf, c->dmabuf, m[i].len);
		}
	}

	return ret;

 err_dma:
	return -1;
}

/*
 * Adapter functionality callback
 * @param a		I2C adapter
 * @returns		OR-ed functionality flags
 */
static unsigned int i2c_stm32_functionality(struct i2c_adapter *a)
{
	d_printk(3, "ok\n" );
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/*
 * Algorithm data structure
 */
static const struct i2c_algorithm i2c_stm32_algorithm = {
	.functionality	= i2c_stm32_functionality,
	.master_xfer	= i2c_stm32_transfer,
};

/*
 * Instantiate a new instance of the I2C controller
 * @dev			I2C controller platform device
 * @returns		0->success, <0->error code
 */
static int __devinit i2c_stm32_probe(struct platform_device *dev)
{
	struct i2c_stm32f7 *c = NULL;
	struct i2c_stm32_data *d;
	struct resource *regs;
	struct resource	*dma_res;
	int bus;
	int irq;
	int ret = 0;

	/*
	 * Get the bus # from the platform device:
	 */
	bus = dev->id;
	if (! (0 <= bus && bus <= 3)) {
		dev_err(&dev->dev, "invalid bus number %d\n", bus);
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	/*
	 * Get the IRQ number from the platform device
	 */
	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		dev_err(&dev->dev, "invalid IRQ number %d\n", irq);
		ret = irq;
		goto Error_release_nothing;
	}

	/*
	 * Get the register base from the platform device
	 */
	regs = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&dev->dev, "no register base provided\n");
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	/*
	 * Allocate the controller-private data structure
	 */
	c = kzalloc(sizeof(struct i2c_stm32f7), GFP_KERNEL);
	if (!c) {
		dev_err(&dev->dev, "unable to allocate memory\n");
		ret = -ENOMEM;
		goto Error_release_nothing;
	}
	c->dev = dev;
	c->bus = bus;

	/*
	 * Request a memory region for the CSR block
	 */
	if (!request_mem_region(regs->start, resource_size(regs),
		regs->name)) {
		dev_err(&dev->dev, "registers already in use\n");
		ret = -ENOMEM;
		goto Error_release_memory;
	}
	c->regs_base = regs->start;
	c->regs_size = resource_size(regs);

	/*
	 * Map in the CSR block
	 */
	c->regs = ioremap(regs->start, resource_size(regs));
	if (!c->regs) {
		dev_err(&dev->dev, "unable to map registers\n");
		ret = -EINVAL;
		goto Error_release_mem_region;
	}

	/*
	 * Register interrupt handler for events
	 */
	ret = request_irq(irq, i2c_stm32_irq, 0, dev_name(&dev->dev), c);
	if (ret) {
		dev_err(&dev->dev, "request for IRQ %d failed\n", irq);
		goto Error_release_regs;
	}
	c->irq = irq;

	/*
	 * Register interrupt handler for errors
	 */
	ret = request_irq(irq + 1, i2c_stm32_irq, 0, dev_name(&dev->dev), c);
	if (ret) {
		dev_err(&dev->dev, "request for IRQ %d failed\n", irq + 1);
		goto Error_release_irq1;
	}

	/*
	 * Retrieve the private parameters
	 */
	d = (struct i2c_stm32_data *) platform_get_drvdata(dev);
	c->ref_clk = d->ref_clk;
	c->i2c_clk = d->i2c_clk;

	/*
	 * Link the private data to dev
	 */
	platform_set_drvdata(dev, c);

	/*
	 * Initialize the I2C adapter data structure
	 */
	c->adap.owner = THIS_MODULE;
	c->adap.nr = bus;
	snprintf(c->adap.name, sizeof(c->adap.name), "i2c_stm32.%u", bus);
	c->adap.algo = &i2c_stm32_algorithm;
	c->adap.algo_data = c;
	c->adap.dev.parent = &dev->dev;

	/*
	 * Initialize DMA
	 */
	dma_res = platform_get_resource_byname(dev, IORESOURCE_DMA, "dma_tx_channel");
	if (!dma_res) {
		dev_err(&dev->dev, "no DMA TX channel provided\n");
		ret = -ENXIO;
		goto Error_release_irq2;
	}

	c->dma_ch_tx = dma_res->start;

	if (stm32_dma_ch_get(c->dma_ch_tx) != 0) {
		dev_err(&dev->dev, "can't acquire DMA TX channel\n");
		ret = -ENXIO;
		goto Error_release_irq2;
	}

	dma_res = platform_get_resource_byname(dev, IORESOURCE_DMA, "dma_rx_channel");
	if (!dma_res) {
		dev_err(&dev->dev, "no DMA RX channel provided\n");
		ret = -ENXIO;
		goto Error_release_dma_tx;
	}

	c->dma_ch_rx = dma_res->start;

	if (stm32_dma_ch_get(c->dma_ch_rx) != 0) {
		dev_err(&dev->dev, "can't acquire DMA RX channel\n");
		ret = -ENXIO;
		goto Error_release_dma_tx;
	}

	c->dmabuf = dmamem_alloc(DMABUF_SIZE, 0, GFP_KERNEL | GFP_DMA);
	if (c->dmabuf == NULL) {
		dev_err(&dev->dev, "unable to allocate DMA buffer\n");
		goto Error_release_dma_rx;
	}

	/*
	 * Initialize the controller hardware
	 */
	ret = i2c_stm32_hw_init(c);
	if (ret) {
		goto Error_release_dmabuf;
	}

	/*
	 * Set up the wait queue
	 */
	init_waitqueue_head(&c->wait);

	/*
	 * Register the I2C adapter
	 */
	if (i2c_add_numbered_adapter(&c->adap)) {
		dev_err(&dev->dev, "unable to add adapter\n");
		ret = -ENXIO;
		goto Error_release_hw;
	}

	/*
	 * If we are here, we are successful
	 */
	dev_info(&dev->dev, "I2C Controller %s at %p,irq=%d\n",
		 dev_name(&c->adap.dev), c->regs, c->irq);
	goto Done;

	/*
	 * Error processing
	 */
Error_release_hw:
	i2c_stm32_hw_release(c);
Error_release_dmabuf:
	dmamem_free(c->dmabuf);
Error_release_dma_rx:
	stm32_dma_ch_put(c->dma_ch_rx);
Error_release_dma_tx:
	stm32_dma_ch_put(c->dma_ch_tx);
Error_release_irq2:
	free_irq(c->irq + 1, c);
Error_release_irq1:
	free_irq(c->irq, c);
Error_release_regs:
	iounmap(c->regs);
Error_release_mem_region:
	release_mem_region(regs->start, resource_size(regs));
Error_release_memory:
	kfree(c);
	platform_set_drvdata(dev, NULL);
Error_release_nothing:

Done:
	d_printk(1, "dev=%s,regs=%p,irq=%d,ref_clk=%d,i2c_clk=%d,ret=%d\n",
		 dev_name(&dev->dev),
		 c ? c->regs : NULL, c ? c->irq : 0,
		 c ? c->ref_clk : 0, c ? c->i2c_clk : 0, ret);
	return ret;
}

/*
 * Shutdown of an instance of the controller device
 * @dev			I2C controller platform device
 * @returns		0->success, <0->error code
 */
static int __devexit i2c_stm32_remove(struct platform_device *dev)
{
	struct i2c_stm32f7 *c  = platform_get_drvdata(dev);
	int ret = 0;

	dmamem_free(c->dmabuf);

	/*
	 * Shut the hardware down
	 */
	i2c_stm32_hw_release(c);

	stm32_dma_ch_put(c->dma_ch_rx);
	stm32_dma_ch_put(c->dma_ch_tx);
	/*
	 * Release kernel resources.
	 */
	platform_set_drvdata(dev, NULL);
	i2c_del_adapter(&c->adap);
	free_irq(c->irq, c);
	free_irq(c->irq + 1, c);
	iounmap(c->regs);
	release_mem_region(c->regs_base, c->regs_size);
	kfree(c);

	d_printk(1, "dev=%s,ret=%d\n", dev_name(&dev->dev), ret);
	return ret;
}

/*
 * Driver data structure
 */
static struct platform_driver i2c_stm32f7_drv = {
	.probe	= i2c_stm32_probe,
	.remove	= __devexit_p(i2c_stm32_remove),
	.driver = {
		.name = "i2c_stm32f7",
		.owner = THIS_MODULE,
	},
};

/*
 * Driver init
 */
static int __init i2c_stm32_module_init(void)
{
	int ret;

	ret = platform_driver_register(&i2c_stm32f7_drv);

	d_printk(1, "drv=%s,ret=%d\n", i2c_stm32f7_drv.driver.name, ret);
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit i2c_stm32_module_exit(void)
{
	platform_driver_unregister(&i2c_stm32f7_drv);

	d_printk(1, "drv=%s\n", i2c_stm32f7_drv.driver.name);
}

module_init(i2c_stm32_module_init);
module_exit(i2c_stm32_module_exit);
MODULE_AUTHOR("Vladimir Skvortsov, <vskvortsov@emcraft.com>");
MODULE_DESCRIPTION("Device driver for the I2C controller of STM32F7");
MODULE_LICENSE("GPL");
