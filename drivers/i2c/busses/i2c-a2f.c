/*
 * Device driver for the I2C controller of the SmartFusion SoC.
 * Author: Vladimir Khusainov, vlad@emcraft.com
 * Copyright 2012 Emcraft Systems
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

/*
 * Debug output control. While debugging, have I2C_A2F_DEBUG defined.
 * In deployment, make sure that I2C_A2F_DEBUG is undefined 
 * to avoid performance and size overhead of debug messages.
 */
#define I2C_A2F_DEBUG
#if 1
#undef I2C_A2F_DEBUG
#endif

#if defined(I2C_A2F_DEBUG)

/*
 * Driver verbosity level: 0->silent; >0->verbose (1 to 4, growing verbosity)
 */
static int i2c_a2f_debug = 4;

/*
 * User can change verbosity of the driver (when loading the module,
 * or the kernel, in case the driver is linked in statically,
 * but not through a /sysfs parameter file)
 */
module_param(i2c_a2f_debug, int, S_IRUGO);
MODULE_PARM_DESC(i2c_a2f_debug, "I2C controller driver verbosity level");

#if !defined(MODULE)

/*
 * Parser for the boot time driver verbosity parameter
 * @param str		user defintion of the parameter
 * @returns		1->success
 */
static int __init i2c_a2f_debug_setup(char * str)
{
	get_option(&str, &i2c_a2f_debug);
	return 1;
}
__setup("i2c_a2f_debug=", i2c_a2f_debug_setup);

#endif /* !defined(MODULE) */

/*
 * Service to print debug messages
 */
#define d_printk(level, fmt, args...)					\
	if (i2c_a2f_debug >= level) printk(KERN_INFO "%s: " fmt,	\
				       	   __func__, ## args)

#else

#define d_printk(level, fmt, args...)

#endif /* defined(I2C_A2F_DEBUG) */

/*
 * I2C controller private data structure
 */
struct i2c_a2f {
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
 * Description of the the SmartFusion I2C hardware interfaces.
 * This is a 1-to-1 mapping of Actel's documentation onto a C structure.
 * Refer to SmartFusion Data Sheet for details.
 */
struct i2c_a2f_regs {
	unsigned int			ctrl;
	unsigned int			status;
	unsigned int			data;
	unsigned int			addr;
	unsigned int			smbus;
	unsigned int			freq;
	unsigned int			glitchreg;
};

/*
 * Access handle for the control registers
 */
#define I2C_A2F_REGS(r)			((volatile struct i2c_a2f_regs *)(r))
#define I2C_A2F(c)			(I2C_A2F_REGS(c->regs))

/*
 * Some bits in various CSRs 
 */
#define	I2C_A2F_CTRL_ENS1		(1<<6)
#define	I2C_A2F_CTRL_STA		(1<<5)
#define	I2C_A2F_CTRL_STO		(1<<4)
#define	I2C_A2F_CTRL_SI 		(1<<3)
#define	I2C_A2F_CTRL_AA 		(1<<2)
#define	I2C_A2F_CTRL_CR0_SHFT		0
#define	I2C_A2F_CTRL_CR1_SHFT		1
#define	I2C_A2F_CTRL_CR2_SHFT		7

/*
 * I2C status codes applicable to master mode operation
 */
#define I2C_A2F_STATUS_START		0x08
#define I2C_A2F_STATUS_REPSTART		0x10
#define I2C_A2F_STATUS_ADDR_W_ACK	0x18
#define I2C_A2F_STATUS_ADDR_W_NACK	0x20
#define I2C_A2F_STATUS_ADDR_R_ACK	0x40
#define I2C_A2F_STATUS_ADDR_R_NACK	0x48
#define I2C_A2F_STATUS_DATA_W_ACK	0x28
#define I2C_A2F_STATUS_DATA_W_NACK	0x30
#define I2C_A2F_STATUS_DATA_R_ACK	0x50
#define I2C_A2F_STATUS_DATA_R_NACK	0x58
#define I2C_A2F_STATUS_BUS_ERROR	0x00
#define I2C_A2F_STATUS_ARB_LOST		0x38

/*
 * Hardware initialization of the I2C controller
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static int i2c_a2f_hw_init(struct i2c_a2f *c)
{
	/*
	 * Array of supported clock divisors
	 */
	static struct {
		unsigned int	d;
		unsigned int	b;
	} div[] = 
	{{60,6},{120,5},{160,3},{192,2},{224,1},{256,0},{960,4},{0,0}};

	unsigned int v, b;
	int i;
	int ret = 0;

	/*
 	 * Calculate the serial clock rate.
 	 * We do it first since if the requested rate can't be supported,
 	 * we want to bail out without enabling the hardware.
 	 */
	for (i = 0; div[i].d && c->ref_clk/div[i].d > c->i2c_clk; i++);
	if (!div[i].d) {
		dev_err(&c->dev->dev, "ref clock %d too high "
			"for requested rate %d\n", c->ref_clk, c->i2c_clk);
		ret = -ENXIO;
		goto Done;
	}
	b = div[i].b;

	/*
 	 * Enable the controller
 	 */
	v = readl(&I2C_A2F(c)->ctrl);
	writel(v | I2C_A2F_CTRL_ENS1, &I2C_A2F(c)->ctrl);

	/*
 	 * Set up the clock rate
 	 */
	v = readl(&I2C_A2F(c)->ctrl);
	writel(v 
		| (v & ~(1 << I2C_A2F_CTRL_CR0_SHFT))
		| (((b >> 0) & 0x1) << I2C_A2F_CTRL_CR0_SHFT)
		| (v & ~(1 << I2C_A2F_CTRL_CR1_SHFT))
		| (((b >> 1) & 0x1) << I2C_A2F_CTRL_CR2_SHFT)
		| (v & ~(1 << I2C_A2F_CTRL_CR2_SHFT))
		| (((b >> 2) & 0x1) << I2C_A2F_CTRL_CR2_SHFT),
		&I2C_A2F(c)->ctrl);

	/*
	 * Set the address for master-only operation
	 */
	writel(0x0, &I2C_A2F(c)->addr);

Done:
	d_printk(2, "bus=%d,clk=%d,ctrl=0x%x,ret=%d\n",
		 c->bus, !ret ? c->ref_clk/div[i].d : 0,
		 !ret ? readl(&I2C_A2F(c)->ctrl) : 0, ret);
	return ret;
}

/*
 * Hardware shutdown of the I2C controller
 * @param c		controller data structure
 */
static void i2c_a2f_hw_release(struct i2c_a2f *c)
{
	/*
 	 * Disable the controller
 	 */
	writel(readl(&I2C_A2F(c)->ctrl) & ~I2C_A2F_CTRL_ENS1, 
		&I2C_A2F(c)->ctrl);

	d_printk(2, "bus=%d\n", c->bus);
}

/*
 * Reset the I2C controller to known state
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static void i2c_a2f_hw_clear(struct i2c_a2f *c)
{
	/*
 	 * Clear various conditions that affect the controler and the bus
 	 */
	writel(readl(&I2C_A2F(c)->ctrl) & 
		~(I2C_A2F_CTRL_SI | I2C_A2F_CTRL_STA | 
		  I2C_A2F_CTRL_STO | I2C_A2F_CTRL_AA),
		&I2C_A2F(c)->ctrl);

	d_printk(3, "ok\n"); 
}

/*
 * Interrupt handler routine
 * @param irq		IRQ number
 * @param d		controller data structure
 * @returns		IRQ handler exit code
 */
static irqreturn_t i2c_a2f_irq(int irq, void *d)
{
	struct i2c_a2f *c = (struct i2c_a2f *) d;
	unsigned int ctrl = readl(&I2C_A2F(c)->ctrl);
	unsigned int sta = readl(&I2C_A2F(c)->status);
	int disable_intr = 0;
	irqreturn_t ret = IRQ_HANDLED;

	/*
 	 * Check if there is a serial interrupt event 
 	 * pending at the controller. Bail out if there is none
 	 */
	if (!(ctrl & I2C_A2F_CTRL_SI)) {
		ret = IRQ_NONE;
		goto Done;
	}	

	/*
	 * Implement the state machine defined by the I2C controller
	 */
	switch (sta) {

	case I2C_A2F_STATUS_START:
	case I2C_A2F_STATUS_REPSTART:

		/*
		 * Remove the start condition
		 */
		writel(ctrl & ~I2C_A2F_CTRL_STA, &I2C_A2F(c)->ctrl);

		/*
 		 * Send out addr and direction
 		 */
		writel((c->msg->addr << 1) |
                        (c->msg->flags & I2C_M_RD ? 1 : 0), 
			&I2C_A2F(c)->data);

		break;

	case I2C_A2F_STATUS_ADDR_W_NACK:
	case I2C_A2F_STATUS_ADDR_R_NACK:
	case I2C_A2F_STATUS_DATA_W_NACK:

		/* 
		 * No ack -> let's stop and report a failure
		 */	
		writel(ctrl | I2C_A2F_CTRL_STO, &I2C_A2F(c)->ctrl);
		c->msg_status = -ENODEV;
		disable_intr = 1;

		break;

	case I2C_A2F_STATUS_ADDR_W_ACK:
	case I2C_A2F_STATUS_DATA_W_ACK:

		/*
 		 * If there is more data to send, send it
 		 */
		if (c->msg_i < c->msg->len) {
			writel(c->msg->buf[(c->msg_i)++],
				&I2C_A2F(c)->data);
		}

		/*
 		 * If this is last transfer in the message,
 		 * send stop and report success
 		 */
		else if (--(c->msg_n) == 0) {
			writel(ctrl | I2C_A2F_CTRL_STO, &I2C_A2F(c)->ctrl);
			c->msg_status = 0;
			disable_intr = 1;
		}

		/*
		 * This is not the last transfer in the message.
		 * Advance to the next segment and initate a repeated start.
		 */
		else {
			c->msg++;
			c->msg_i = 0;
			writel(ctrl | I2C_A2F_CTRL_STA, &I2C_A2F(c)->ctrl);
		}

		break;

	case I2C_A2F_STATUS_ADDR_R_ACK:

		/*
		 * Will be receiving the last byte from the slave.
		 * Return NACK to tell the slave to stop sending.
		 */
		if (c->msg_i + 1 == c->msg->len) {
			writel(ctrl & ~I2C_A2F_CTRL_AA, &I2C_A2F(c)->ctrl);
		}
	
		/*
		 * Will be receiving more data from the slave.
		 * Return ACK to tell the slave to send more.
		 */
		else {
			writel(ctrl | I2C_A2F_CTRL_AA, &I2C_A2F(c)->ctrl);
		}

		break;

	case I2C_A2F_STATUS_DATA_R_ACK:

		/*
 		 * Retrieve the data but
		 * there is more data to receive in this transfer
 		 */
		c->msg->buf[c->msg_i++] = readl(&I2C_A2F(c)->data);

		/*
	 	 * Will be receiving the last byte from the slave.
	 	 * Return NACK to tell the slave to stop sending.
	 	 */
		if (c->msg_i + 1 == c->msg->len) {
			writel(ctrl & ~I2C_A2F_CTRL_AA, &I2C_A2F(c)->ctrl);
		}
	
		/*
	 	 * Will be receiving more data from the slave.
	 	 * Return ACK to tell the slave to send more.
	 	 */
		else {
			writel(ctrl | I2C_A2F_CTRL_AA, &I2C_A2F(c)->ctrl);
		}

		break;

	case I2C_A2F_STATUS_DATA_R_NACK:

		/*
 		 * Retrieve the data but
		 * this segment is done with
 		 */
		c->msg->buf[c->msg_i++] = readl(&I2C_A2F(c)->data);

		/*
 		 * If this is last transfer in the message,
 		 * send stop and report success
 		 */
		if (--(c->msg_n) == 0) {
			writel(ctrl | I2C_A2F_CTRL_STO, &I2C_A2F(c)->ctrl);
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
			writel(ctrl | I2C_A2F_CTRL_STA, &I2C_A2F(c)->ctrl);
		}

		break;

	default:

		/* 
		 * Some error condition -> let's stop and report a failure
		 */	
		i2c_a2f_hw_clear(c);
		c->msg_status = -EIO;
		disable_intr = 1;

		break;
	}

	/*
 	 * Clear the serial interrupt condition
 	 */
	ctrl = readl(&I2C_A2F(c)->ctrl);
	writel(ctrl & ~I2C_A2F_CTRL_SI, &I2C_A2F(c)->ctrl);

	/*
 	 * If the current transfer is done, disable interrupts
 	 */
	if (disable_intr) {
		disable_irq_nosync(c->irq);
	}

	/* 
	 * Exit on failure or all bytes have been transferred
	 */
	if (c->msg_status != -EBUSY) {
		wake_up(&c->wait);
	}

Done:
	d_printk(4, "ctrl=0x%x,sta=0x%x,ret=%d\n", ctrl, sta, ret);
	return ret;
}

/*
 * Adapter transfer callback
 * @param a		I2C adapter
 * @param m		array of messages
 * @param n		number of messages
 * @returns		message segments transferred or error code (<1)
 */
static int i2c_a2f_transfer(struct i2c_adapter *a, struct i2c_msg *m, int n)
{
#if defined(I2C_A2F_DEBUG)
	int i, j;
#endif
	unsigned int ctrl;
	struct i2c_a2f *c = a->algo_data;
	int ret = 0;

	/*
	 * Store the software parameters of the message.
	 * There will be used by the IRQ handler.
	 */
	c->msg = &m[0];
	c->msg_i = 0;
	c->msg_n = n;
	c->msg_status = -EBUSY;

	/*
 	 * Reset the bus to a known state
 	 */
	i2c_a2f_hw_clear(c);

	/*
	 * A transfer is kicked off by initiating a start condition
	 */
	ctrl = readl(&I2C_A2F(c)->ctrl);
	writel(ctrl | I2C_A2F_CTRL_STA, &I2C_A2F(c)->ctrl);

	/*
	 * Let interrupts happen
	 */
	enable_irq(c->irq);

	/*
	 * Wait for the transfer to complete, one way or another
	 */
	if (wait_event_timeout(c->wait, c->msg_status != -EBUSY, HZ) == 0) {
		ret = -ETIMEDOUT;
	} else {
		ret = c->msg_status;
		if (!ret) {
			ret = n;
		}
	}

#if defined(I2C_A2F_DEBUG)
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
static unsigned int i2c_a2f_functionality(struct i2c_adapter *a)
{
	d_printk(3, "ok\n" );
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/*
 * Algorithm data structure
 */
static const struct i2c_algorithm i2c_a2f_algorithm = {
	.functionality	= i2c_a2f_functionality,
	.master_xfer	= i2c_a2f_transfer,
};

/*
 * Instantiate a new instance of the I2C controller
 * @dev			I2C controller platform device
 * @returns		0->success, <0->error code
 */
static int __devinit i2c_a2f_probe(struct platform_device *dev)
{
	struct i2c_a2f *c = NULL;
	struct i2c_a2f_data *d;
	struct resource *regs;
	int bus;
	int irq;
	int ret = 0;

	/*
 	 * Get the bus # from the platform device: 
 	 * [0,1]->hard-core I2C contorller of SmartFusion; 
 	 * [2-9]->soft-IP I2C controller specific to a custom design.
 	 */
	bus = dev->id;
	if (! (0 <= bus && bus <= 10)) {
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
	c = kzalloc(sizeof(struct i2c_a2f), GFP_KERNEL);
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
 	 * Register interrupt handler
 	 */
	ret = request_irq(irq, i2c_a2f_irq, 0, dev_name(&dev->dev), c);
	if (ret) {
		dev_err(&dev->dev, "request for IRQ %d failed\n", irq);
		goto Error_release_regs;
	}
	disable_irq_nosync(irq);
	c->irq = irq;

	/*
 	 * Retrieve the private parameters
 	 */
	d = (struct i2c_a2f_data *) platform_get_drvdata(dev);
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
	snprintf(c->adap.name, sizeof(c->adap.name), "i2c_a2f.%u", bus);
	c->adap.algo = &i2c_a2f_algorithm;
	c->adap.algo_data = c;
	c->adap.dev.parent = &dev->dev;

	/* 
 	 * Set up the wait queue
 	 */
	init_waitqueue_head(&c->wait);

	/* 
 	 * Initialize the controller hardware
 	 */
	ret = i2c_a2f_hw_init(c);
	if (ret) {
		goto Error_release_irq;
	}

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
	i2c_a2f_hw_release(c);
Error_release_irq: 
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
static int __devexit i2c_a2f_remove(struct platform_device *dev)
{
	struct i2c_a2f *c  = platform_get_drvdata(dev);
	int ret = 0;

	/*
 	 * Shut the hardware down
 	 */
	i2c_a2f_hw_release(c);

	/*
	 * Release kernel resources.
	 */
	platform_set_drvdata(dev, NULL);
	i2c_del_adapter(&c->adap);
	free_irq(c->irq, c);
	iounmap(c->regs);
	release_mem_region(c->regs_base, c->regs_size);
	kfree(c);

	d_printk(1, "dev=%s,ret=%d\n", dev_name(&dev->dev), ret);
	return ret;
}

/*
 * Driver data structure
 */
static struct platform_driver i2c_a2f_drv = {
	.probe	= i2c_a2f_probe,
	.remove	= __devexit_p(i2c_a2f_remove),
	.driver = {
		.name = "i2c_a2f",
		.owner = THIS_MODULE,
	},
};

/*
 * Driver init
 */
static int __init i2c_a2f_module_init(void)
{
	int ret;
	
	ret = platform_driver_register(&i2c_a2f_drv);

	d_printk(1, "drv=%s,ret=%d\n", i2c_a2f_drv.driver.name, ret);
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit i2c_a2f_module_exit(void)
{
	platform_driver_unregister(&i2c_a2f_drv);

	d_printk(1, "drv=%s\n", i2c_a2f_drv.driver.name);
}

module_init(i2c_a2f_module_init);
module_exit(i2c_a2f_module_exit);
MODULE_AUTHOR("Vladimir Khusainov, <vlad@emcraft.com>");
MODULE_DESCRIPTION("Device driver for the I2C controller of SmartFusion");
MODULE_LICENSE("GPL");

