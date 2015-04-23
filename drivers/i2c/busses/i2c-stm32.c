/*
 * Device driver for the I2C controller of the STM32F microcontrollers.
 * Author: Vladimir Khusainov, vlad@emcraft.com
 * Copyright 2013 Emcraft Systems
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
 * @param str		user defintion of the parameter
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
struct i2c_stm32 {
	struct platform_device *	dev;		/* Platform device */
	int				bus;		/* Bus (ID) */
	unsigned int			regs_base;	/* Regs base (phys) */
	unsigned int			regs_size;	/* Regs size */
	void * __iomem 			regs;		/* Regs base (virt) */
	int				irq;		/* IRQ # */
	unsigned int			ref_clk;	/* Ref clock */
	unsigned int			i2c_clk;	/* Bus clock */
	struct i2c_msg*			msg;		/* Current message */
	int				msg_n;		/* Segments in msg */
	int				msg_i;		/* Idx in a segment */
	volatile int			msg_status;	/* Message status */
	struct i2c_adapter		adap;		/* I2C adapter data */
	wait_queue_head_t		wait;		/* Wait queue */
};

/*
 * Description of the the STM32 I2C hardware interfaces.
 */
struct i2c_stm32_regs {
	unsigned int			cr1;
	unsigned int			cr2;
	unsigned int			oar1;
	unsigned int			oar2;
	unsigned int			dr;
	unsigned int			sr1;
	unsigned int			sr2;
	unsigned int			ccr;
	unsigned int			trise;
	unsigned int			fltr;
};

/*
 * Access handle for the control registers
 */
#define I2C_STM32_REGS(r)		((volatile struct i2c_stm32_regs *)(r))
#define I2C_STM32(c)			(I2C_STM32_REGS(c->regs))

/*
 * Convert to MHz
 */
#define MHZ(v)				((v) * 1000000)

/*
 * Some bits in various CSRs
 */
#define RCC_APB1RSTR_I2C1		(1<<21)
#define RCC_APB1RSTR_I2C2		(1<<22)
#define RCC_APB1RSTR_I2C3		(1<<23)
#define RCC_APB1ENR_I2C1		(1<<21)
#define RCC_APB1ENR_I2C2		(1<<22)
#define RCC_APB1ENR_I2C3		(1<<23)
#define	I2C_STM32_CR1_SWRST		(1<<15)
#define	I2C_STM32_CR1_ACK		(1<<10)
#define	I2C_STM32_CR1_STO		(1<<9)
#define	I2C_STM32_CR1_STA		(1<<8)
#define	I2C_STM32_CR1_PE		(1<<0)
#define	I2C_STM32_CR1_MASK		(I2C_STM32_CR1_ACK | \
					 I2C_STM32_CR1_STO | I2C_STM32_CR1_STA | \
					 I2C_STM32_CR1_PE  | I2C_STM32_CR1_SWRST)
#define	I2C_STM32_CR2_ITBUFEN		(1<<10)
#define	I2C_STM32_CR2_ITEVTEN		(1<<9)
#define	I2C_STM32_CR2_ITERREN		(1<<8)
#define	I2C_STM32_CR2_FREQ(v)		((v)<<0)
#define	I2C_STM32_SR1_AF		(1<<10)
#define	I2C_STM32_SR1_TXE		(1<<7)
#define	I2C_STM32_SR1_RXNE		(1<<6)
#define	I2C_STM32_SR1_BTF		(1<<2)
#define	I2C_STM32_SR1_ADDR		(1<<1)
#define	I2C_STM32_SR2_TRA		(1<<2)
#define	I2C_STM32_SR1_SB		(1<<0)
#define	I2C_STM32_CCR_CCR(v)		((v)<<0)
#define	I2C_STM32_CCR_FS		(1<<15)
#define	I2C_STM32_CCR_DUTY		(1<<14)
#define	I2C_STM32_TRISE_TRISE(v)	((v)<<0)

static inline void i2c_stm32_disable_irqs(struct i2c_stm32 *c)
{
	disable_irq_nosync(c->irq);
	disable_irq_nosync(c->irq + 1);
}

static inline void i2c_stm32_enable_irqs(struct i2c_stm32 *c)
{
	enable_irq(c->irq);
	enable_irq(c->irq + 1);
}

/*
 * Reset the I2C controller to known state
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static void i2c_stm32_hw_clear(struct i2c_stm32 *c)
{
	int v;

	/*
	 * Reset the controller to clear possible errors or locks
	 */
	v = readl(&I2C_STM32(c)->cr1) & ~I2C_STM32_CR1_MASK;
	writel(v | I2C_STM32_CR1_SWRST, &I2C_STM32(c)->cr1);
	udelay(20);
	writel(v & ~I2C_STM32_CR1_SWRST, &I2C_STM32(c)->cr1);
	/*
	 * Set the clocks.
	 */
	if (c->i2c_clk <= 100000) {
		v = c->ref_clk / MHZ(1);
		writel(I2C_STM32_CR2_FREQ(v), &I2C_STM32(c)->cr2);
		writel(I2C_STM32_TRISE_TRISE(v + 1), &I2C_STM32(c)->trise);
		v = c->ref_clk / (c->i2c_clk << 1);
		v = v < 0x04 ? 0x04 : v;
		writel(I2C_STM32_CCR_CCR(v), &I2C_STM32(c)->ccr);
	}
	else {
		v = c->ref_clk / MHZ(1);
		v = ((v * 300) / 1000) + 1;
		writel(I2C_STM32_TRISE_TRISE(v), &I2C_STM32(c)->trise);
		v = c->ref_clk / (c->i2c_clk * 25);
		v |= ((v & 0x0FFF) == 0) ? 0x0001 : 0;
		v |= I2C_STM32_CCR_FS | I2C_STM32_CCR_DUTY;
		writel(I2C_STM32_CCR_CCR(v), &I2C_STM32(c)->ccr);
	}

	/*
	 * Enable hardware interrupts, including for error conditions
	 */
	v = readl(&I2C_STM32(c)->cr2);
	writel(v |
		I2C_STM32_CR2_ITBUFEN |
		I2C_STM32_CR2_ITEVTEN |
		I2C_STM32_CR2_ITERREN, &I2C_STM32(c)->cr2);

	/*
	 * Enable the I2C controller
	 */
	v = readl(&I2C_STM32(c)->cr1);
	writel(v | I2C_STM32_CR1_PE, &I2C_STM32(c)->cr1);
}

/*
 * Hardware initialization of the I2C controller
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static int i2c_stm32_hw_init(struct i2c_stm32 *c)
{

	unsigned int v;
	int ret = 0;

	/*
	 * First, figure out if we are able to configure the clocks
	 * If not, we want to bail out without enabling enything.
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

	i2c_stm32_hw_clear(c);
Done:
	d_printk(2, "bus=%d,"
		"cr1=0x%x,cr2=0x%x,sr1=0x%x,ccr=0x%x,trise=0x%x,ret=%d\n",
		c->bus,
		!ret ? readl(&I2C_STM32(c)->cr1) : 0,
		!ret ? readl(&I2C_STM32(c)->cr2) : 0,
		!ret ? readl(&I2C_STM32(c)->sr1) : 0,
		!ret ? readl(&I2C_STM32(c)->ccr) : 0,
		!ret ? readl(&I2C_STM32(c)->trise) : 0, ret);
	return ret;
}

/*
 * Hardware shutdown of the I2C controller
 * @param c		controller data structure
 */
static void i2c_stm32_hw_release(struct i2c_stm32 *c)
{
	unsigned int v;

	/*
	 * Disable the controller
	 */
	writel(readl(&I2C_STM32(c)->cr1) & ~I2C_STM32_CR1_PE,
		&I2C_STM32(c)->cr1);

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
	struct i2c_stm32 *c = (struct i2c_stm32 *) d;
	unsigned int cr1 = readl(&I2C_STM32(c)->cr1);
	unsigned int sr1 = readl(&I2C_STM32(c)->sr1);
	unsigned int sr2 = 0;
	int disable_intr = 0;
	irqreturn_t ret = IRQ_HANDLED;

	/*
	 * Check if there is an interrupt event
	 * pending at the controller. Bail out if there is none.
	 * It does happen sometimes for some reason.
	 */
	if (!sr1) {
		ret = IRQ_NONE;
		goto Done;
	}

	/*
	 * Implement the state machine defined by the I2C controller
	 */
	if (sr1 & I2C_STM32_SR1_SB) {

		/*
		 * Remove the start condition
		 */
		writel(cr1 & ~I2C_STM32_CR1_STA, &I2C_STM32(c)->cr1);

		/*
		 * Start sent -> send out addr and direction
		 */
		writel((c->msg->addr << 1) |
                        (c->msg->flags & I2C_M_RD ? 1 : 0), &I2C_STM32(c)->dr);

	}

	else if (sr1 & I2C_STM32_SR1_AF) {

		/*
		 * No ack -> let's stop and report a failure
		 */
		writel(sr1 & ~I2C_STM32_SR1_AF, &I2C_STM32(c)->sr1);
		c->msg_status = -ENODEV;
		disable_intr = 1;
	}

	else if ((sr1 & I2C_STM32_SR1_ADDR) ||
		 (sr1 & I2C_STM32_SR1_TXE) ||
		 (sr1 & I2C_STM32_SR1_BTF) ||
		 (sr1 & I2C_STM32_SR1_RXNE)) {

		/*
		 * There is a pecularity in the hardware controller that
		 * TXE can be set while BTF is not. Wait until BTF sets.
		 * TO-DO: This is terribly bad of course. Waiting in an ISR
		 * is out of question. Need to fix that somehow.
		 */
		while (sr1 == I2C_STM32_SR1_TXE) {
			sr1 = readl(&I2C_STM32(c)->sr1);
		}

		/*
		 * Slave has acknowledged the address
		 * or byte transfer has finished.
		 *
		 * If this is the master receiver mode, it is important
		 * to set/reset ACK before clearing the interrupt condition:
		 *
	 	 * Will be receiving the last byte from the slave.
	 	 * Return NACK to tell the slave to stop sending.
	 	 */
		if (c->msg_i + 1 == c->msg->len) {
			writel(cr1 & ~I2C_STM32_CR1_ACK,
				&I2C_STM32(c)->cr1);
		}

		/*
	 	 * Will be receiving more data from the slave.
	 	 * Return ACK to tell the slave to send more.
	 	 */
		else {
			writel(cr1 | I2C_STM32_CR1_ACK,
				&I2C_STM32(c)->cr1);
		}

		/*
		 * Clear the interrupt condition
		 */
		sr2 = readl(&I2C_STM32(c)->sr2);

		/*
		 * Depending on the direction, enter
		 * transmitter or receiver mode.
		 */
		if (sr2 & I2C_STM32_SR2_TRA) {

			/*
			 * This is the master transmiter mode:
			 * If there is more data to send, send it.
			 */
			if (c->msg_i < c->msg->len) {
				writel(c->msg->buf[(c->msg_i)++],
					&I2C_STM32(c)->dr);
			}

			/*
			 * If this is last transfer in the message,
			 * report success.
			 */
			else if (--(c->msg_n) == 0) {
				c->msg_status = 0;
				disable_intr = 1;
			}

			/*
			 * This is not the last transfer in the message.
			 * Advance to the next segment and
			 * initate a repeated start.
			 */
			else {
				c->msg++;
				c->msg_i = 0;
				writel(cr1 | I2C_STM32_CR1_STA,
					&I2C_STM32(c)->cr1);
			}
		}
		else if (sr1 & I2C_STM32_SR1_RXNE) {

			/*
			 * This is the master receiver mode:
			 * Retrieve the data.
			 */
			c->msg->buf[c->msg_i++] = readl(&I2C_STM32(c)->dr);

			/*
			 * More data to get. Get out of IRQ and wait
			 * for a next interrupt.
			 */
			if (c->msg_i < c->msg->len) {
			}

			/*
			 * If this is last transfer in the message,
			 * report success.
			 */
			else if (--(c->msg_n) == 0) {
				c->msg_status = 0;
				disable_intr = 1;
			}

			/*
			 * This is not the last transfer in the message.
			 * Advance to the next segment
			 * and initate a repeated start.
			 */
			else {
				c->msg++;
				c->msg_i = 0;
				writel(cr1 | I2C_STM32_CR1_STA,
					&I2C_STM32(c)->cr1);
			}
		}
	}

	else {

		/*
		 * Some error condition -> let's stop and report a failure
		 */
		writel(0x0, &I2C_STM32(c)->sr1);
		dev_dbg(&c->dev->dev,
			"error condition in irq handler: sr1=%x\n", sr1);
		c->msg_status = -EIO;
		disable_intr = 1;
	}

	/*
	 * If the current transfer is done, disable interrupts
	 */
	if (disable_intr) {
		i2c_stm32_disable_irqs(c);
	}

	/*
	 * Exit on failure or all bytes have been transferred
	 */
	if (c->msg_status != -EBUSY) {

		/*
		 * Clear the interrupt condition
		 */
		wake_up(&c->wait);
	}

Done:
	d_printk(4, "sr1=0x%x,sr2=0x%x,ret=%d\n", sr1, sr2, ret);
	return ret;
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
	int i, j;
#endif
	struct i2c_stm32 *c = a->algo_data;
	int ret = 0;

	/*
	 * Store the software parameters of the message.
	 * These will be used by the IRQ handler.
	 */
	c->msg = &m[0];
	c->msg_i = 0;
	c->msg_n = n;
	c->msg_status = -EBUSY;

	/*
	 * A transfer is kicked off by initiating a start condition.
	 * Actual transfer is handled by the state machine implemented
	 * in the IRQ routine.
	 */
	writel(readl(&I2C_STM32(c)->cr1) | I2C_STM32_CR1_STA,
		&I2C_STM32(c)->cr1);

	/*
	 * Let interrupts happen
	 */
	i2c_stm32_enable_irqs(c);

	/*
	 * Wait for the transfer to complete, one way or another
	 */
	if (wait_event_timeout(c->wait, c->msg_status != -EBUSY, 5*HZ) == 0) {
		i2c_stm32_disable_irqs(c);
		ret = -ETIMEDOUT;
	} else {
		ret = c->msg_status;
		if (!ret) {
			ret = n;
		}
	}

	/*
	 * Stop activity on the bus
	 */
	writel(readl(&I2C_STM32(c)->cr1) | I2C_STM32_CR1_STO,
		&I2C_STM32(c)->cr1);


	/*
	 * Reset the bus to a known state in case of error.
	 */
	if (ret < 0 && ret != -ENODEV) {
		i2c_stm32_hw_clear(c);
	}

#if defined(I2C_STM32_DEBUG)
	for (i = 0; i < n; i++) {
		d_printk(4, "%d:flags=0x%x,addr=0x%x,len=%d\n",
			 i, m[i].flags, m[i].addr, m[i].len);
		for (j = 0; j < m[i].len; j++) {
			d_printk(4, "%d=%x\n", j, m[i].buf[j]);
		}
	}
#endif
	d_printk(3, "n=%d,ret=%d\n", n, ret);

	return ret;
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
	struct i2c_stm32 *c = NULL;
	struct i2c_stm32_data *d;
	struct resource *regs;
	int bus;
	int irq;
	int ret = 0;

	/*
	 * Get the bus # from the platform device:
	 */
	bus = dev->id;
	if (! (0 <= bus && bus <= 2)) {
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
	c = kzalloc(sizeof(struct i2c_stm32), GFP_KERNEL);
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
	disable_irq_nosync(irq);
	c->irq = irq;

	/*
	 * Register interrupt handler for errors
	 */
	ret = request_irq(irq + 1, i2c_stm32_irq, 0, dev_name(&dev->dev), c);
	if (ret) {
		dev_err(&dev->dev, "request for IRQ %d failed\n", irq + 1);
		goto Error_release_irq1;
	}
	disable_irq_nosync(irq + 1);

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
	 * Initialize the controller hardware
	 */
	ret = i2c_stm32_hw_init(c);
	if (ret) {
		goto Error_release_irq2;
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
	struct i2c_stm32 *c  = platform_get_drvdata(dev);
	int ret = 0;

	/*
	 * Shut the hardware down
	 */
	i2c_stm32_hw_release(c);

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
static struct platform_driver i2c_stm32_drv = {
	.probe	= i2c_stm32_probe,
	.remove	= __devexit_p(i2c_stm32_remove),
	.driver = {
		.name = "i2c_stm32",
		.owner = THIS_MODULE,
	},
};

/*
 * Driver init
 */
static int __init i2c_stm32_module_init(void)
{
	int ret;

	ret = platform_driver_register(&i2c_stm32_drv);

	d_printk(1, "drv=%s,ret=%d\n", i2c_stm32_drv.driver.name, ret);
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit i2c_stm32_module_exit(void)
{
	platform_driver_unregister(&i2c_stm32_drv);

	d_printk(1, "drv=%s\n", i2c_stm32_drv.driver.name);
}

module_init(i2c_stm32_module_init);
module_exit(i2c_stm32_module_exit);
MODULE_AUTHOR("Vladimir Khusainov, <vlad@emcraft.com>");
MODULE_DESCRIPTION("Device driver for the I2C controller of STM32");
MODULE_LICENSE("GPL");

