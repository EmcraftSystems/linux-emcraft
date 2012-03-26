/*
 * (C) Copyright 2011, 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Yurtsev <alex@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * SmartFusion CoreUART platform driver
 */

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty.h>

#include <mach/uart.h>


#define A2F_CUART_DRV_NAME  "a2f_cuart"
#define A2F_CUART_NAME	    "ttyCS"


/*
 * CoreUART registers
 */
struct a2f_cuart_regs {
    u32	    tx_data; /* tx data reg */
    u32	    rx_data; /* rx data reg */
    u32	    ctrl1;   /* control 1 reg */
    u32	    ctrl2;   /* control 2 reg */
    u32	    status;  /* status reg */
};

struct a2f_cuart_priv {
    struct a2f_cuart_regs *regs;
    unsigned int           clk;
    int                    tx_irq;
    int                    rx_irq;
};

/*
 * CoreUART Status bits
 */
#define A2F_CORE_UART_ST_TXRDY (1 << 0)
#define A2F_CORE_UART_ST_RXRDY (1 << 1)
#define A2F_CORE_UART_ST_PERR  (1 << 2)
#define A2F_CORE_UART_ST_OFLOW (1 << 3)
#define A2F_CORE_UART_ST_FERR  (1 << 4)

/*
 * CoreUART Ctrl flags
 */
#define A2F_CORE_UART_CTRL2_BIT8	(1 << 0)
#define A2F_CORE_UART_CTRL2_PAR_EN      (1 << 1)
#define A2F_CORE_UART_CTRL2_PAR_ODD     (1 << 2)
#define A2F_CORE_UART_CTRL2_BAUDR_MASK	0xF8

#define A2F_NR_CUARTS 8

/*
 * UART ports and privates
 */
static struct uart_port		a2f_ports[A2F_NR_CUARTS];
static struct a2f_cuart_priv	a2f_cuart_priv[A2F_NR_CUARTS];

#define a2f_cuart_priv_ptr(port)    (struct a2f_cuart_priv *)((port)->private_data)
#define a2f_cuart_regs_ptr(port)    (struct a2f_cuart_regs *)(a2f_cuart_priv_ptr(port))->regs

/*
 * UART driver instance
 */
static struct uart_driver	a2f_cuart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= A2F_CUART_DRV_NAME,
	.dev_name	= A2F_CUART_NAME,
	.major		= TTY_MAJOR,
	.minor		= 92,
	.nr		= A2F_NR_CUARTS,
};

static inline int tx_enabled(struct uart_port *port)
{
	return port->unused[0] & 1;
}

static inline int rx_enabled(struct uart_port *port)
{
	return port->unused[0] & 2;
}

static inline void rx_enable(struct uart_port *port, int enabled)
{
	if(enabled)
		port->unused[0] |= 2;
	else
		port->unused[0] &= ~2;
}

static inline void tx_enable(struct uart_port *port, int enabled)
{
	if(enabled)
		port->unused[0] |= 1;
	else
		port->unused[0] &= ~1;
}
/*
 * Check if transmitter in idle (tx reg empty)
 */
static unsigned int a2f_port_tx_empty(struct uart_port *port)
{
	volatile struct a2f_cuart_regs	*regs = a2f_cuart_regs_ptr(port);
	unsigned long flags;
	u32 ret;

	spin_lock_irqsave(&port->lock, flags);
	ret = (regs->status & A2F_CORE_UART_ST_TXRDY) ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}

/*
 * Stop transmitter
 */
static void a2f_port_stop_tx(struct uart_port *port)
{
	if (tx_enabled(port)) {
		/* why not just disable_irq ? */
		/*disable_irq_nosync(priv->tx_irq);*/
		tx_enable(port, 0);
	}
}

static void a2f_cuart_transmit(void *dev_id);

/*
 * Start transmitter
 */
static void a2f_port_start_tx(struct uart_port *port)
{
	if (!tx_enabled(port)) {
		tx_enable(port, 1);
	}
	a2f_cuart_transmit(port);
}

/*
 * Stop receiver
 */
static void a2f_port_stop_rx(struct uart_port *port)
{
	struct a2f_cuart_priv *priv = a2f_cuart_priv_ptr(port);
	if (rx_enabled(port)) {
		disable_irq(priv->rx_irq);
		rx_enable(port, 0);
	}
}

/*
 * Set modem state
 */
static void a2f_port_set_mctrl(struct uart_port *port, u32 mctrl)
{
	/*
	 * N/A
	 */
}

/*
 * Get current modem state
 */
static unsigned int a2f_port_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

/*
 * Enable modem status interrups
 */
static void a2f_port_enable_ms(struct uart_port *port)
{
	/*
	 * N/A
	 */
}

/*
 * Control the transmission of a break signal
 */
static void a2f_port_break_ctl(struct uart_port *port, int ctl)
{
	/*
	 * N/A
	 */
}

static void a2f_cuart_set_baudrate(struct uart_port *port, unsigned int baud_val)
{
	volatile struct a2f_cuart_regs	*regs = a2f_cuart_regs_ptr(port);

	regs->ctrl1 = baud_val & 0xFF;
	regs->ctrl2 &= ~A2F_CORE_UART_CTRL2_BAUDR_MASK;
	regs->ctrl2 |= (baud_val >> 5) & A2F_CORE_UART_CTRL2_BAUDR_MASK;
}

/*
 * Send a char
 */
static void a2f_xmit_char(volatile struct a2f_cuart_regs *uart,
			    unsigned char c)
{
	int count = 10000;
	while (count-- > 0 && !(uart->status & A2F_CORE_UART_ST_TXRDY));

	uart->tx_data = c;
}
/*
 * Process receive interrupt event
 */
static void a2f_cuart_receive(void *dev_id)
{
	struct uart_port			*port = dev_id;
	volatile struct a2f_cuart_regs	*regs = a2f_cuart_regs_ptr(port);
	struct tty_struct			*tty = port->state->port.tty;
	unsigned char c;
	int count = 256;
	int flag;

	while (count-- > 0 && regs->status & A2F_CORE_UART_ST_RXRDY) {
	    c = regs->rx_data;
	    port->icount.rx++;

		flag = TTY_NORMAL;

	    if (regs->status & A2F_CORE_UART_ST_FERR) {
		port->icount.frame++;
		flag = TTY_FRAME;
	    }

	    tty_insert_flip_char(tty, c, flag);
	}

	tty_flip_buffer_push(tty);
}

/*
 * Process receive interrupt event
 */
static void a2f_cuart_transmit(void *dev_id)
{
	struct uart_port			*port = dev_id;
	volatile struct a2f_cuart_regs	*regs = a2f_cuart_regs_ptr(port);
	struct circ_buf				*xmit;
	int count = 64;

	if (port->x_char) {
		a2f_xmit_char(regs, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	xmit = &port->state->xmit;
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		a2f_port_stop_tx(port);
		return;
	}

	while (count-- > 0 && !uart_circ_empty(xmit)){
	    a2f_xmit_char(regs, xmit->buf[xmit->tail]);
	    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	    port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		a2f_port_stop_tx(port);
}

/*
 * Core UART irq handler
 */
static irqreturn_t a2f_cuart_isr(int irq, void *dev_id)
{
	struct uart_port			*port = dev_id;
	volatile struct a2f_cuart_regs	*regs = a2f_cuart_regs_ptr(port);

	if (regs->status & A2F_CORE_UART_ST_RXRDY) {
	    a2f_cuart_receive(dev_id);
	}

	return IRQ_HANDLED;
}

/*
 * Open the port
 */
static int a2f_port_startup(struct uart_port *port)
{
    volatile struct a2f_cuart_regs	*regs = a2f_cuart_regs_ptr(port);
    struct a2f_cuart_priv		*priv = a2f_cuart_priv_ptr(port);
    int ret;
    int baud_val;

    ret = request_irq(priv->rx_irq, a2f_cuart_isr, 0, A2F_CUART_NAME, port);
    if (ret) {
	printk(KERN_ERR "%s: request_irq (%d)failed (%d)\n",
		__func__, priv->rx_irq, ret);
	free_irq(priv->rx_irq, port);
	return ret;
    }

    /*
     * Configure UART
     */

    baud_val = uart_get_divisor(port, 9600) - 1;
    a2f_cuart_set_baudrate(port, baud_val);

    /* 8 bits */
    regs->ctrl2 |= A2F_CORE_UART_CTRL2_BIT8;

    tx_enable(port, 0);
    rx_enable(port, 1);

    return ret;
}

/*
 * Disable the port
 */
static void a2f_port_shutdown(struct uart_port *port)
{
    struct a2f_cuart_priv		*priv = a2f_cuart_priv_ptr(port);
    free_irq(priv->rx_irq, port);
}

/*
 * Set termios
 */
static void a2f_port_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, baud_val;

	spin_lock_irqsave(&port->lock, flags);

	baud = uart_get_baud_rate(port, termios, old, 9600, 115200);
	baud_val = uart_get_divisor(port, baud) - 1;
	a2f_cuart_set_baudrate(port, baud_val);

	spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * Port description
 */
static const char *a2f_port_type(struct uart_port *port)
{
	return A2F_CUART_NAME;
}

/*
 * Request resources required
 */
static int a2f_port_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Autoconfiguration steps required for the port
 */
static void a2f_port_config_port(struct uart_port *port, int flags)
{
	volatile struct a2f_cuart_regs	*regs = a2f_cuart_regs_ptr(port);

	if (!a2f_port_request_port(port))
		port->type = PORT_A2FCUART;
}

/*
 * Verify if serial_struct is suitable for this port type
 */
static int a2f_port_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/*
	 * We don't want the core code to modify any port params
	 */

	return -EINVAL;
}

/*
 * Remove A2F cuart device
 */
static int a2f_port_release(struct device *dev)
{
	struct uart_port	*port = dev_get_drvdata(dev);
	struct a2f_cuart_priv	*priv;
	int			ret = 0;

	if (port) {
		priv = a2f_cuart_priv_ptr(port);
		ret = uart_remove_one_port(&a2f_cuart_driver, port);
		dev_set_drvdata(dev, NULL);
		if (priv) {
			priv->regs = NULL;
		}
	}

	return ret;
}

/*
 * Release resource used
 */
static void a2f_port_release_port(struct uart_port *port)
{
	/*
	 * N/A
	 */
}

/*
 * UART driver operations
 */
static struct uart_ops a2f_cuart_ops = {
	.tx_empty	= a2f_port_tx_empty,
	.set_mctrl	= a2f_port_set_mctrl,
	.get_mctrl	= a2f_port_get_mctrl,
	.stop_tx	= a2f_port_stop_tx,
	.start_tx	= a2f_port_start_tx,
	.stop_rx	= a2f_port_stop_rx,
	.enable_ms	= a2f_port_enable_ms,
	.break_ctl	= a2f_port_break_ctl,
	.startup	= a2f_port_startup,
	.shutdown	= a2f_port_shutdown,
	.set_termios	= a2f_port_set_termios,
	.type		= a2f_port_type,
	.release_port	= a2f_port_release_port,
	.request_port	= a2f_port_request_port,
	.config_port	= a2f_port_config_port,
	.verify_port	= a2f_port_verify_port
};

/*
 * Platform bus binding
 */
static int __devinit a2f_cuart_probe(struct platform_device *pdev)
{
	struct uart_port	*port;
	int			id = pdev->id, ret;
	struct device		*dev = &pdev->dev;

	if (id >= A2F_NR_CUARTS) {
		dev_err(dev, "%s: bad port id %d\n", __func__, id);
		ret = -EINVAL;
		return ret;
	}

	/*
	 * If id is negative, assign the first free port
	 */
	if (id < 0) {
		for (id = 0; id < A2F_NR_CUARTS; id++) {
			if (a2f_ports[id].ops == NULL)
				break;
		}
	}
	if (id == A2F_NR_CUARTS) {
		dev_err(dev, "%s: no free port\n", __func__);
		ret = -EBUSY;
		return ret;
	}

	a2f_cuart_priv[id].regs = (struct a2f_cuart_regs *)(platform_get_resource(pdev, IORESOURCE_MEM, 0))->start;
	a2f_cuart_priv[id].rx_irq = (platform_get_resource(pdev, IORESOURCE_IRQ, 0))->start;
	a2f_cuart_priv[id].clk = *(unsigned int *)pdev->dev.platform_data;

	port = &a2f_ports[id];
	spin_lock_init(&port->lock);
	port->private_data = &a2f_cuart_priv[id];
	port->iotype  = SERIAL_IO_MEM,
	port->line    = id;
	port->flags   = UPF_BOOT_AUTOCONF;
	port->ops     = &a2f_cuart_ops;
	port->dev     = dev;
	port->irq     = a2f_cuart_priv[id].rx_irq;
	port->mapbase = a2f_cuart_priv[id].regs;
	port->uartclk = a2f_cuart_priv[id].clk;
	dev_set_drvdata(dev, port);

	/*
	 * Register the port
	 */
	ret = uart_add_one_port(&a2f_cuart_driver, port);
	if (ret) {
		dev_err(dev, "%s: uart_add_one_port failed (%d)\n",
			__func__, ret);
		dev_set_drvdata(dev, NULL);
	}
	return ret;
}

/*
 * Platform bus unbinding
 */
static int __devexit a2f_cuart_remove(struct platform_device *pdev)
{
	return a2f_port_release(&pdev->dev);
}

/*
 * Platform driver instance
 */
static struct platform_driver a2f_platform_driver = {
	.remove		= __devexit_p(a2f_cuart_remove),
	.driver		= {
			.owner	= THIS_MODULE,
			.name	= A2F_CUART_DRV_NAME,
			},
};

/*
 * Module init
 */
static int __init a2f_cuart_init(void)
{
	int	ret;

	printk(KERN_INFO "Serial: SmartFusion CUART driver\n");

	ret = uart_register_driver(&a2f_cuart_driver);
	if (ret) {
		printk(KERN_ERR "%s: uart_register_driver failed (%d)\n",
			__func__, ret);
		return ret;
	}

	ret = platform_driver_probe(&a2f_platform_driver, a2f_cuart_probe);
	if (ret) {
		printk(KERN_ERR "%s: platform_driver_register failed (%d)\n",
			__func__, ret);
		uart_unregister_driver(&a2f_cuart_driver);
		return ret;
	}

	return ret;
}

/*
 * Module exit
 */
static void __exit a2f_cuart_exit(void)
{
	platform_driver_unregister(&a2f_platform_driver);
	uart_unregister_driver(&a2f_cuart_driver);
}

module_init(a2f_cuart_init);
module_exit(a2f_cuart_exit);

MODULE_AUTHOR("TBD");
MODULE_DESCRIPTION("SmartFusion CoreUART serial driver");
MODULE_LICENSE("GPL");
