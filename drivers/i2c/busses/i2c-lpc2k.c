/*
 * Copyright (C) 2011 NXP Semiconductors
 *
 * Code portions referenced from the i2x-pxa and i2c-pnx drivers
 *
 * Make SMBus byte and word transactions work on LPC178x/7x
 * Copyright (c) 2012
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
 * Anton Protopopov, Emcraft Systems, antonp@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/io.h>


#define LPC24XX_I2CONSET	0X00
#define LPC24XX_I2STAT		0X04
#define LPC24XX_I2DAT		0X08
#define LPC24XX_I2ADDR		0X0C
#define LPC24XX_I2SCLH		0X10
#define LPC24XX_I2SCLL		0X14
#define LPC24XX_I2CONCLR	0X18

#define LPC24XX_AA		0x04
#define LPC24XX_SI		0x08
#define LPC24XX_STO		0x10
#define LPC24XX_STA		0x20
#define LPC24XX_I2EN		0x40

/*
 * 26 possible I2C status codes, but codes applicable onlt to master
 * are listed here and used in this driver
 */
enum {
	m_bus_error		= 0x00,
	m_start			= 0x08,
	m_repstart		= 0x10,
	mx_addr_w_ack		= 0x18,
	mx_addr_w_nack		= 0x20,
	mx_data_w_ack		= 0x28,
	mx_data_w_nack		= 0x30,
	m_data_arb_lost		= 0x38,
	mr_addr_r_ack		= 0x40,
	mr_addr_r_nack		= 0x48,
	mr_data_r_ack		= 0x50,
	mr_data_r_nack		= 0x58,
	m_i2c_idle		= 0xF8,
};

#define MODULE_NAME		"lpc2k-i2c"
#define LPC24XX_MAX_ADAPTERS	3

/* I2C clock speed, 0 - 400000+Hz */
static unsigned int scl_frequency = 100000;
module_param(scl_frequency, uint,  0644);

struct lpc2k_i2c {
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	int			msg_idx;
	volatile int		msg_status;
	int			is_last;

	struct i2c_adapter	adap;
	struct clk		*clk;

	void __iomem		*reg_base;

	unsigned long		iobase;
	unsigned long		iosize;
	int			irq;
};

static inline unsigned long i2c_readl(void __iomem *reg)
{
	return __raw_readl(reg);
}

static inline void i2c_writel(unsigned long val, void __iomem *reg)
{
	__raw_writel(val, reg);
}

static void i2c_lpc2k_reset(struct lpc2k_i2c *i2c)
{
	/* Will force clear all statuses */
	i2c_writel(0x7C, i2c->reg_base + LPC24XX_I2CONCLR);
	i2c_writel(0, i2c->reg_base + LPC24XX_I2ADDR);
	i2c_writel(LPC24XX_I2EN, i2c->reg_base + LPC24XX_I2CONSET);
}

static int i2c_lpc2k_clear_arb(struct lpc2k_i2c *i2c)
{
	long timeout = jiffies + HZ;
	int ret = 0;

	/*
	 * If the transfer needs to abort for some reason, we'll try to
	 * force a stop condition to clear any pending bus conditions
	 */
	i2c_writel(LPC24XX_STO, i2c->reg_base + LPC24XX_I2CONSET);

	/* Wait for status change */
	while (jiffies < timeout &&
		(i2c_readl(i2c->reg_base + LPC24XX_I2STAT) != m_i2c_idle))
		cpu_relax();

	if (i2c_readl(i2c->reg_base + LPC24XX_I2STAT) != m_i2c_idle) {
		/* Bus was not idle, try to reset adapter */
		i2c_lpc2k_reset(i2c);
		ret = -EBUSY;
	}

	return ret;
}

static void i2c_lpc2k_pump_msg(struct lpc2k_i2c *i2c)
{
	unsigned long status;
	unsigned char data;

	/*
	 * I2C in the LPC2xxx series is basically a state machine.
	 * Just run through the steps based on the current status.
	 */
	status = i2c_readl(i2c->reg_base + LPC24XX_I2STAT);

	switch (status) {
	case m_start:
	case m_repstart:
		/* Start bit was just sent out, send out addr and dir */
		data = (i2c->msg->addr << 1);
		if (i2c->msg->flags & I2C_M_RD)
			data |= 1;

		i2c_writel((unsigned long) data,
			i2c->reg_base + LPC24XX_I2DAT);
		i2c_writel(LPC24XX_STA,
			i2c->reg_base + LPC24XX_I2CONCLR);

		dev_dbg(&i2c->adap.dev, "Start sent, sending address "
			"0x%02x\n", data);
		break;

	case mx_addr_w_ack:
	case mx_data_w_ack:
		/*
		 * Address or data was sent out with an ACK. If there is more
		 * data to send, send it now
		 */
		if (i2c->msg_idx < i2c->msg->len) {
			i2c_writel((unsigned long)
				i2c->msg->buf[i2c->msg_idx],
				i2c->reg_base + LPC24XX_I2DAT);
			dev_dbg(&i2c->adap.dev, "ACK ok, sending "
				"(0x%02x)\n", i2c->msg->buf[i2c->msg_idx]);
		} else if (i2c->is_last) {
			/* Last message, send stop */
			i2c_writel(LPC24XX_STO | LPC24XX_AA,
				i2c->reg_base + LPC24XX_I2CONSET);
			i2c_writel(LPC24XX_SI, i2c->reg_base + LPC24XX_I2CONCLR);
			i2c->msg_status = 0;
			dev_dbg(&i2c->adap.dev, "ACK ok, sending stop\n");
			disable_irq_nosync(i2c->irq);
		} else {
			i2c->msg_status = 0;
			dev_dbg(&i2c->adap.dev, "ACK ok, idling until "
				"next message start\n");
			disable_irq_nosync(i2c->irq);
		}

		i2c->msg_idx++;
		break;

	case mr_addr_r_ack:
		/*
		 * Receive first byte from slave
		 */
		if (i2c->msg->len == 1) {
			/* Last byte, return NACK */
			i2c_writel(LPC24XX_AA,
				i2c->reg_base + LPC24XX_I2CONCLR);
		} else {
			/* Not last byte, return ACK */
			i2c_writel(LPC24XX_AA,
				i2c->reg_base + LPC24XX_I2CONSET);
		}

		i2c_writel(LPC24XX_STA, i2c->reg_base + LPC24XX_I2CONCLR);
		break;

	case mr_data_r_nack:
		/*
		 * The I2C shows NACK status on reads, so we need to accept
		 * the NACK as an ACK here. This should be ok, as the real
		 * BACK would of been caught on the address write.
		 */
	case mr_data_r_ack:
		/*
		 * Data was received
		 */
		if (i2c->msg_idx < i2c->msg->len) {
			i2c->msg->buf[i2c->msg_idx] =
				i2c_readl(i2c->reg_base + LPC24XX_I2DAT);
			dev_dbg(&i2c->adap.dev, "ACK ok, received "
				"(0x%02x)\n", i2c->msg->buf[i2c->msg_idx]);
		}

		/*
		 * If transfer is done, send STOP
		 */
		if (i2c->msg_idx >= i2c->msg->len - 1 && i2c->is_last) {
			i2c_writel(LPC24XX_STO | LPC24XX_AA,
				i2c->reg_base + LPC24XX_I2CONSET);
			i2c_writel(LPC24XX_SI, i2c->reg_base + LPC24XX_I2CONCLR);
			i2c->msg_status = 0;
			dev_dbg(&i2c->adap.dev, "ACK ok, sending stop\n");
		}

		/*
		 * Message is done
		 */
		if (i2c->msg_idx >= i2c->msg->len - 1) {
			i2c->msg_status = 0;
			dev_dbg(&i2c->adap.dev, "ACK ok, idling until "
				"next message start\n");
			disable_irq_nosync(i2c->irq);
		}

		/*
		 * One pre-last data input, send NACK to tell the slave that
		 * this is going to be the last data byte to be transferred.
		 */
		if (i2c->msg_idx >= i2c->msg->len - 2) {
			/* One byte left to receive - NACK */
			i2c_writel(LPC24XX_AA,
				i2c->reg_base + LPC24XX_I2CONCLR);
		} else {
			/* More than one byte left to receive - ACK */
			i2c_writel(LPC24XX_AA,
				i2c->reg_base + LPC24XX_I2CONSET);
		}

		i2c_writel(LPC24XX_STA,
			i2c->reg_base + LPC24XX_I2CONCLR);
		i2c->msg_idx++;
		break;

	case mx_addr_w_nack:
	case mx_data_w_nack:
	case mr_addr_r_nack:
		/*
		 * NACK! Processing is done
		 */
		i2c_writel(LPC24XX_STO | LPC24XX_AA,
			i2c->reg_base + LPC24XX_I2CONSET);
		i2c->msg_status = -ENODEV;
		dev_dbg(&i2c->adap.dev, "Device NACKed, error\n");
		disable_irq_nosync(i2c->irq);
		break;

	case m_data_arb_lost:
		/*
		 * Arbitration lost
		 */
		i2c->msg_status = -EIO;
		dev_dbg(&i2c->adap.dev, "Arbitration lost, error\n");

		/*
		 * Release the I2C bus
		 */
		i2c_writel(LPC24XX_STA | LPC24XX_STO,
			i2c->reg_base + LPC24XX_I2CONCLR);
		disable_irq_nosync(i2c->irq);
		break;

	default:
		/* Unexpected statuses */
		i2c->msg_status = -EIO;
		dev_err(&i2c->adap.dev, "Unexpected status, error (%x)\n",
			(unsigned int) status);
		disable_irq_nosync(i2c->irq);
		break;
	}

	/* Exit on failure or all bytes transferred */
	if (i2c->msg_status != -EBUSY)
		wake_up(&i2c->wait);

	/*
	 * If `msg_status` is zero, then `lpc2k_process_msg()` is responsible
	 * for clearing the SI flag.
	 */
	if (i2c->msg_status != 0)
		i2c_writel(LPC24XX_SI, i2c->reg_base + LPC24XX_I2CONCLR);
}

static int lpc2k_process_msg(struct lpc2k_i2c *i2c, int msgidx)
{
	int ret;

	dev_dbg(&i2c->adap.dev, "Processing message %d (len=%d) (flags=%x)\n",
		msgidx, i2c->msg->len, i2c->msg->flags);

	/*
	 * A new transfer is kicked off by initiating a start condition
	 */
	if (!msgidx) {
		dev_dbg(&i2c->adap.dev, "Start sent\n");
		i2c_writel(LPC24XX_STA, i2c->reg_base + LPC24XX_I2CONSET);
	} else {
		/*
		 * A multi-message I2C transfer continues where the previous
		 * I2C transfer left off and uses the current condition of the
		 * I2C adapter.
		 */
		if (unlikely(i2c->msg->flags & I2C_M_NOSTART)) {
			WARN_ON(i2c->msg->len == 0);

			if (!(i2c->msg->flags & I2C_M_RD)) {
				/* Start transmit of data */
				i2c_writel((unsigned long) i2c->msg->buf[0],
					i2c->reg_base + LPC24XX_I2DAT);
				i2c->msg_idx++;
				dev_dbg(&i2c->adap.dev, "New data sent\n");
			}
			else
				dev_dbg(&i2c->adap.dev, "New data incoming\n");
		} else {
			/* Start or repeated start */
			dev_dbg(&i2c->adap.dev, "Repeated start sent\n");
			i2c_writel(LPC24XX_STA,
				i2c->reg_base + LPC24XX_I2CONSET);
		}

		i2c_writel(LPC24XX_SI,
			i2c->reg_base + LPC24XX_I2CONCLR);
	}

	enable_irq(i2c->irq);

	/* Wait for transfer completion */
	if (wait_event_timeout(i2c->wait, i2c->msg_status != -EBUSY,
		HZ) == 0) {
		disable_irq_nosync(i2c->irq);
		dev_dbg(&i2c->adap.dev, "Transfer timed out!\n");
		ret = -ETIMEDOUT;
	} else {
		ret = i2c->msg_status;
		if (ret == 0)
			dev_dbg(&i2c->adap.dev, "Transfer successful\n");
		else
			dev_dbg(&i2c->adap.dev, "Transfer failed (%d)\n",
				ret);
	}

	return ret;
}

static int i2c_lpc2k_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
	int msg_num)
{
	int ret, i;
	unsigned long stat;
	struct lpc2k_i2c *i2c = adap->algo_data;

	/* Check for bus idle condition */
	stat = i2c_readl(i2c->reg_base + LPC24XX_I2STAT);
	if (stat != m_i2c_idle) {
		/* Something is holding the bus, try to clear it */
		ret = i2c_lpc2k_clear_arb(i2c);
		if (ret) {
			dev_err(&i2c->adap.dev, "Bus is not idle\n");
			return ret;
		}
	}

	dev_dbg(&i2c->adap.dev, "Processing total messages = %d\n", msg_num);

	/* Process a single message at a time */
	for (i = 0; i < msg_num; i++) {
		/* Save message pointer and current message data index */
		i2c->msg = &msgs[i];
		i2c->msg_idx = 0;
		i2c->msg_status = -EBUSY;
		i2c->is_last = (i == (msg_num - 1));

		ret = lpc2k_process_msg(i2c, i);
		if (ret)
			return ret;
	}

	return msg_num;
}

static irqreturn_t i2c_lpc2k_handler(int this_irq, void *dev_id)
{
	struct lpc2k_i2c *i2c = (struct lpc2k_i2c *) dev_id;

	if (i2c_readl(i2c->reg_base + LPC24XX_I2CONSET) & LPC24XX_SI) {
		i2c_lpc2k_pump_msg(i2c);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static u32 i2c_lpc2k_functionality(struct i2c_adapter *adap)
{
	/* Only emulated SMBus for now */
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_lpc2k_algorithm = {
	.master_xfer	= i2c_lpc2k_xfer,
	.functionality	= i2c_lpc2k_functionality,
};

static int i2c_lpc2k_probe(struct platform_device *dev)
{
	struct lpc2k_i2c *i2c;
	struct resource *res;
	int ret, irq;
	unsigned long clkrate;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(dev, 0);
	if (res == NULL || irq < 0) {
		dev_err(&dev->dev, "No resource data!\n");
		return -ENODEV;
	}

	if (dev->id < 0 || dev->id >= LPC24XX_MAX_ADAPTERS) {
		dev_err(&dev->dev, "I2C bus number invalid (%d)\n", dev->id);
		return -ENODEV;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&dev->dev, "Memory region already used!\n");
		return -ENOMEM;
	}

	i2c = kzalloc(sizeof(struct lpc2k_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&dev->dev, "Error allocating memory!\n");
		ret = -ENOMEM;
		goto emalloc;
	}

	i2c->adap.owner = THIS_MODULE;

	init_waitqueue_head(&i2c->wait);

	i2c->adap.nr = dev->id;
	snprintf(i2c->adap.name, sizeof(i2c->adap.name), MODULE_NAME ".%u",
		 i2c->adap.nr);

	i2c->clk = clk_get(&dev->dev, NULL);
	if (IS_ERR(i2c->clk)) {
		dev_err(&dev->dev, "Error getting clock!\n");
		ret = PTR_ERR(i2c->clk);
		goto eclk;
	}

	i2c->reg_base = ioremap(res->start, resource_size(res));
	if (!i2c->reg_base) {
		dev_err(&dev->dev, "Error mapping memory!\n");
		ret = -EIO;
		goto eremap;
	}
	i2c->iobase = res->start;
	i2c->iosize = resource_size(res);
	i2c->irq = irq;

	clk_enable(i2c->clk);

	i2c->adap.algo = &i2c_lpc2k_algorithm;
	ret = request_irq(irq, i2c_lpc2k_handler, IRQF_DISABLED,
		i2c->adap.name, i2c);
	if (ret)
		goto ereqirq;

	disable_irq_nosync(irq);

	i2c_lpc2k_reset(i2c);

	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &dev->dev;

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&dev->dev, "Failed to add bus!\n");
		goto eadapt;
	}

	platform_set_drvdata(dev, i2c);

	printk(KERN_INFO "I2C: %s: LPC2K I2C adapter\n",
	       dev_name(&i2c->adap.dev));

	/* Place controller is a known state */
	i2c_lpc2k_reset(i2c);

	/* Get I2C base clock rate */
	clkrate = clk_get_rate(i2c->clk);
	if (!clkrate) {
		dev_warn(&dev->dev, "Can't get I2C base clock, using "
			"12MHz!\n");
		clkrate = 12000000;
	}

	/* Setup I2C dividers to generate clock rate with 50% duty cycle */
	clkrate = (clkrate / scl_frequency) / 2;
	i2c_writel(clkrate, i2c->reg_base + LPC24XX_I2SCLL);
	i2c_writel(clkrate, i2c->reg_base + LPC24XX_I2SCLH);

	return 0;

eadapt:
	free_irq(irq, i2c);
ereqirq:
	clk_disable(i2c->clk);
	iounmap(i2c->reg_base);
eremap:
	clk_put(i2c->clk);
eclk:
	kfree(i2c);
emalloc:
	release_mem_region(res->start, resource_size(res));
	return ret;
}

static int __exit i2c_lpc2k_remove(struct platform_device *dev)
{
	struct lpc2k_i2c *i2c = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	free_irq(i2c->irq, i2c);

	i2c_del_adapter(&i2c->adap);

	clk_disable(i2c->clk);
	clk_put(i2c->clk);

	iounmap(i2c->reg_base);
	release_mem_region(i2c->iobase, i2c->iosize);
	kfree(i2c);

	return 0;
}

#ifdef CONFIG_PM
static int i2c_lpc2k_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc2k_i2c *i2c = platform_get_drvdata(pdev);

	clk_disable(i2c->clk);

	return 0;
}

static int i2c_lpc2k_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc2k_i2c *i2c = platform_get_drvdata(pdev);

	clk_enable(i2c->clk);
	i2c_lpc2k_reset(i2c);

	return 0;
}

static const struct dev_pm_ops i2c_lpc2k_dev_pm_ops = {
	.suspend_noirq = i2c_lpc2k_suspend,
	.resume_noirq = i2c_lpc2k_resume,
};

#define I2C_LPC2K_DEV_PM_OPS (&i2c_lpc2k_dev_pm_ops)
#else
#define I2C_LPC2K_DEV_PM_OPS NULL
#endif

static struct platform_driver i2c_lpc2k_driver = {
	.probe		= i2c_lpc2k_probe,
	.remove		= __exit_p(i2c_lpc2k_remove),
	.driver		= {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= I2C_LPC2K_DEV_PM_OPS,
	},
};

static int __init i2c_lpc2k_init(void)
{
	return platform_driver_register(&i2c_lpc2k_driver);
}

static void __exit i2c_lpc2k_exit(void)
{
	platform_driver_unregister(&i2c_lpc2k_driver);
}

MODULE_AUTHOR("kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("I2C driver for LPC2xxx devices");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lpc2k-i2c");

/* We need to make sure I2C is initialized before USB */
subsys_initcall(i2c_lpc2k_init);
module_exit(i2c_lpc2k_exit);
