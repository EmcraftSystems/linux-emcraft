/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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
 * STM32 USART platform driver
 */

#include <linux/console.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty.h>

#include <mach/uart.h>

#define STM32_NR_UARTS		6
#define STM32_USART_NAME	"ttyS"

#define STM32_USART_PORT	"STM32 USART Port"

/*
 * CR1 bits
 */
#define STM32_USART_CR1_UE	(1 << 13)	/* USART enable		     */
#define STM32_USART_CR1_TXEIE	(1 <<  7)	/* TXE interrupt enable	     */
#define STM32_USART_CR1_RXNEIE	(1 <<  5)	/* RXNE interrupt enable     */
#define STM32_USART_CR1_TE	(1 <<  3)	/* Transmitter enable	     */
#define STM32_USART_CR1_RE	(1 <<  2)	/* Receiver enable	     */

/*
 * SR bits (this is not the full list, see the others in uart.h header)
 */
#define STM32_USART_SR_RXNE	(1 << 5)	/* Read data reg not empty   */
#define STM32_USART_SR_ORE	(1 << 3)	/* Overrun error	     */
#define STM32_USART_SR_FE	(1 << 1)	/* Framing error	     */
#define STM32_USART_SR_PE	(1 << 0)	/* Parity error		     */
#define STM32_USART_SR_ERRORS	(STM32_USART_SR_ORE | STM32_USART_SR_FE |     \
				 STM32_USART_SR_PE)
#define STM32_USART_SR_RX_FLAGS	(STM32_USART_SR_RXNE | STM32_USART_SR_ORE |   \
				 STM32_USART_SR_PE   | STM32_USART_SR_FE)

/*
 * USART register map
 */
struct stm32_usart_regs {
	u16	sr;		/* Status				      */
	u16	rsv0;
	u16	dr;		/* Data					      */
	u16	rsv1;
	u16	brr;		/* Baud rate				      */
	u16	rsv2;
	u16	cr1;		/* Control 1				      */
	u16	rsv3;
	u16	cr2;		/* Control 2				      */
	u16	rsv4;
	u16	cr3;		/* Control 3				      */
	u16	rsv5;
	u16	gtpr;		/* Guard time and prescaler		      */
};

/*
 * Prototypes
 */
static void stm32_xmit_char(volatile struct stm32_usart_regs *uart,
			    unsigned char c);
static int stm32_transmit(struct uart_port *port);
static irqreturn_t stm32_isr(int irq, void *dev_id);

/*
 * UART ports
 */
static struct uart_port		stm32_ports[STM32_NR_UARTS];

/*
 * Check if transmitter in idle (tx reg empty)
 */
static u32 stm_port_tx_empty(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart;
	unsigned long				flags;
	u32					rv;

	uart = (struct stm32_usart_regs *)port->mapbase;
	spin_lock_irqsave(&port->lock, flags);
	rv = (uart->sr & STM32_USART_SR_TXE) ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&port->lock, flags);

	return rv;
}

/*
 * Stop transmitter
 */
static void stm_port_stop_tx(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart;
	unsigned long				flags;

	spin_lock_irqsave(&port->lock, flags);
	uart = (struct stm32_usart_regs *)port->mapbase;
	uart->cr1 &= ~STM32_USART_CR1_TXEIE;
	spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * Start transmitter
 */
static void stm_port_start_tx(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart;
	unsigned long				flags;

	spin_lock_irqsave(&port->lock, flags);
	uart = (struct stm32_usart_regs *)port->mapbase;
	uart->cr1 |= STM32_USART_CR1_TXEIE;
	spin_unlock_irqrestore(&port->lock, flags);

	stm32_transmit(port);
}

/*
 * Stop receiver
 */
static void stm_port_stop_rx(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart;
	unsigned long				flags;

	spin_lock_irqsave(&port->lock, flags);
	uart = (struct stm32_usart_regs *)port->mapbase;
	uart->cr1 &= ~STM32_USART_CR1_RXNEIE;
	spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * Open the port
 */
static int stm_port_startup(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart;
	int					rv;

	rv = request_irq(port->irq, stm32_isr,
			 IRQF_DISABLED | IRQF_SAMPLE_RANDOM,
			 STM32_USART_PORT, port);
	if (rv) {
		printk(KERN_ERR "%s: request_irq (%d)failed (%d)\n",
			__func__, port->irq, rv);
		goto out;
	}

	uart = (struct stm32_usart_regs *)port->mapbase;

	/*
	 * Clear status
	 */
	uart->sr = 0;

	/*
	 * Enable tx, rx, and UART itself
	 */
	uart->cr1  = STM32_USART_CR1_TE | STM32_USART_CR1_RE |
		     STM32_USART_CR1_UE;

	/*
	 * Enable RX-not-empty & TX-empty interrupts
	 */
	uart->cr1 |= STM32_USART_CR1_RXNEIE | STM32_USART_CR1_TXEIE;

	rv = 0;
out:
	return rv;
}

/*
 * Disable the port
 */
static void stm_port_shutdown(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart;

	uart = (struct stm32_usart_regs *)port->mapbase;

	uart->cr1 &= ~(STM32_USART_CR1_TE | STM32_USART_CR1_RE |
		       STM32_USART_CR1_UE);
	uart->cr1 &= ~(STM32_USART_CR1_RXNEIE | STM32_USART_CR1_TXEIE);

	uart->sr &= ~(u32)(-1);

	free_irq(port->irq, port);
}

/*
 * Port description
 */
static const char *stm_port_type(struct uart_port *port)
{
	return STM32_USART_PORT;
}

/*
 * Set termios
 */
static void stm_port_sert_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	/*
	 * TBD
	 */
}

/*
 * Verify if serial_struct is suitable for this port type
 */
static int stm_port_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/*
	 * We don't want the core code to modify any port params
	 */

	return -EINVAL;
}

/*
 * Release resource used
 */
static void stm_port_release_port(struct uart_port *port)
{
	/*
	 * N/A
	 */
}

/*
 * Request resources required
 */
static int stm_port_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Autoconfiguration steps required for the port
 */
static void stm_port_config_port(struct uart_port *port, int flags)
{
	if (!stm_port_request_port(port))
		port->type = PORT_STM32USART;
}

/*
 * Get current modem state
 */
static u32 stm_port_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

/*
 * Set modem state
 */
static void stm_port_set_mctrl(struct uart_port *port, u32 mctrl)
{
	/*
	 * N/A
	 */
}


/*
 * Enable modem status interrups
 */
static void stm_port_enable_ms(struct uart_port *port)
{
	/*
	 * N/A
	 */
}

/*
 * Control the transmission of a break signal
 */
static void stm_port_break_ctl(struct uart_port *port, int ctl)
{
	/*
	 * N/A
	 */
}

/*
 * UART driver operations
 */
static struct uart_ops stm32_uart_ops = {
	.tx_empty	= stm_port_tx_empty,
	.set_mctrl	= stm_port_set_mctrl,
	.get_mctrl	= stm_port_get_mctrl,
	.stop_tx	= stm_port_stop_tx,
	.start_tx	= stm_port_start_tx,
	.stop_rx	= stm_port_stop_rx,
	.enable_ms	= stm_port_enable_ms,
	.break_ctl	= stm_port_break_ctl,
	.startup	= stm_port_startup,
	.shutdown	= stm_port_shutdown,
	.set_termios	= stm_port_sert_termios,
	.type		= stm_port_type,
	.release_port	= stm_port_release_port,
	.request_port	= stm_port_request_port,
	.config_port	= stm_port_config_port,
	.verify_port	= stm_port_verify_port
};

#ifdef CONFIG_SERIAL_STM32_CONSOLE

/*
 * Send char to console
 */
static void stm_console_putchar(struct uart_port *port, int ch)
{
	stm32_xmit_char((struct stm32_usart_regs *)port->mapbase, ch);
}

/*
 * Send string to console
 */
static void stm_console_write(struct console *co, const char *s,
				unsigned int count)
{
	volatile struct stm32_usart_regs	*uart;
	struct uart_port			*port;
	unsigned long				flags;
	unsigned char				irxne, itxe;
	int					locked;

	port = &stm32_ports[co->index];
	uart = (struct stm32_usart_regs *)port->mapbase;

	if (oops_in_progress) {
		locked = spin_trylock_irqsave(&port->lock, flags);
	} else {
		locked = 1;
		spin_lock_irqsave(&port->lock, flags);
	}

	/*
	 * Save and disable interrupts
	 */
	irxne = uart->cr1 & STM32_USART_CR1_RXNEIE;
	itxe  = uart->cr1 & STM32_USART_CR1_TXEIE;

	if (irxne)
		uart->cr1 &= ~STM32_USART_CR1_RXNEIE;
	if (itxe)
		uart->cr1 &= ~STM32_USART_CR1_TXEIE;

	uart_console_write(port, s, count, stm_console_putchar);

	/*
	 * Restore interrupt state
	 */
	if (itxe)
		uart->cr1 |= STM32_USART_CR1_TXEIE;

	if (irxne)
		uart->cr1 |= STM32_USART_CR1_RXNEIE;

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * Setup console
 */
static int __init stm_console_setup(struct console *co, char *options)
{
	struct uart_port	*port;
	int			rv;

	if (co->index < 0 || co->index >= STM32_NR_UARTS) {
		rv = -EINVAL;
		goto out;
	}

	port = &stm32_ports[co->index];
	if (!port->mapbase) {
		pr_debug("console on %s%i not present\n", STM32_USART_NAME,
			 co->index);
		rv = -ENODEV;
		goto out;
	}

	/*
	 * TBD: depending on the options configure device (baud, parity, ...)
	 */
	rv = 0;
out:
	return rv;
}

/*
 * Console instance. Use '-1' as the index to get console from the cmdline
 */
static struct uart_driver	stm32_uart_driver;
static struct console		stm32_console = {
	.name	= STM32_USART_NAME,
	.device	= uart_console_device,
	.write	= stm_console_write,
	.setup	= stm_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &stm32_uart_driver,
};

#endif /* CONFIG_SERIAL_STM32_CONSOLE */

/*
 * UART driver instance
 */
static struct uart_driver	stm32_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= STM32_USART_DRV_NAME,
	.dev_name	= STM32_USART_NAME,
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= STM32_NR_UARTS,
#ifdef CONFIG_SERIAL_STM32_CONSOLE
	.cons		= &stm32_console,
#endif
};

/*
 * Send a char
 */
static void stm32_xmit_char(volatile struct stm32_usart_regs *uart,
			    unsigned char c)
{
	while (!(uart->sr & STM32_USART_SR_TXE));

	uart->dr = c;
}

/*
 * Process transmit interrupt event
 */
static int stm32_transmit(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart;
	struct circ_buf				*xmit;
	int					rv;

	uart = (struct stm32_usart_regs *)port->mapbase;

	if (port->x_char) {
		stm32_xmit_char(uart, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
		rv = 1;
		goto out;
	}

	xmit = &port->state->xmit;
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		stm_port_stop_tx(port);
		rv = 0;
		goto out;
	}

	stm32_xmit_char(uart, xmit->buf[xmit->tail]);
	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	port->icount.tx++;

	/*
	 * Wake-up
	 */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	rv = 1;
out:
	return rv;
}

/*
 * Process receive interrupt event
 */
static int stm32_receive(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart;
	struct tty_struct			*tty;
	int					rv, max_count;
	u16					status;

	uart = (struct stm32_usart_regs *)port->mapbase;

	status = uart->sr;
	if (!(status & STM32_USART_SR_RX_FLAGS)) {
		rv = 0;
		goto out;
	}

	tty = port->state->port.tty;
	max_count = 256;
	do {
		unsigned char	ch;
		char		flag;

		ch = uart->dr;
		port->icount.rx++;
		flag = TTY_NORMAL;

		if (unlikely(status & STM32_USART_SR_ERRORS)) {
/*
 * Don't say kernel about overflow to avoid "input overrun(s)" message.
 * TBD: implement rx/tx with DMA
 */
#if 0
			if (status & STM32_USART_SR_ORE) {
				tty_insert_flip_char(tty, ch, flag);
				port->icount.overrun++;
				flag = TTY_OVERRUN;
			}
#endif
			if (status & STM32_USART_SR_PE) {
				port->icount.parity++;
				flag = TTY_PARITY;
			}
			if (status & STM32_USART_SR_FE) {
				port->icount.frame++;
				flag = TTY_FRAME;
			}
		}

		tty_insert_flip_char(tty, (flag == TTY_NORMAL) ? ch : 0, flag);

		if (max_count-- <= 0)
			break;

		status = uart->sr;
	} while (status & STM32_USART_SR_RX_FLAGS);

	tty_flip_buffer_push(tty);

	rv = 1;
out:
	return rv;
}

/*
 * STM32 USART irq handler
 */
static irqreturn_t stm32_isr(int irq, void *dev_id)
{
	stm32_receive(dev_id);
	stm32_transmit(dev_id);

	return IRQ_HANDLED;
}

/*
 * Register stm32 uart device with the driver
 */
static int __devinit stm32_assign(struct device *dev, int id,
				  u32 base, int irq)
{
	struct uart_port	*port;
	int			rv;

	if (id >= STM32_NR_UARTS) {
		dev_err(dev, "%s: bad port id %d\n", __func__, id);
		rv = -EINVAL;
		goto out;
	}

	/*
	 * If id is negative, assign the first free port
	 */
	if (id < 0) {
		for (id = 0; id < STM32_NR_UARTS; id++) {
			if (!stm32_ports[id].mapbase)
				break;
		}
	}
	if (id == STM32_NR_UARTS) {
		dev_err(dev, "%s: no free port\n", __func__);
		rv = -EBUSY;
		goto out;
	}

	if (stm32_ports[id].mapbase && stm32_ports[id].mapbase != base) {
		dev_err(dev, "%s: port %d already in use\n", __func__, id);
		rv = -EBUSY;
		goto out;
	}

	port = &stm32_ports[id];

	spin_lock_init(&port->lock);
	port->iotype  = SERIAL_IO_MEM,
	port->irq     = irq;
	port->flags   = UPF_BOOT_AUTOCONF;
	port->line    = id;
	port->ops     = &stm32_uart_ops;
	port->dev     = dev;
	port->mapbase = base;

	dev_set_drvdata(dev, port);

	/*
	 * Register the port
	 */
	rv = uart_add_one_port(&stm32_uart_driver, port);
	if (rv) {
		dev_err(dev, "%s: uart_add_one_port failed (%d)\n",
			__func__, rv);
		dev_set_drvdata(dev, NULL);
		port->mapbase = 0;
		goto out;
	}

	rv = 0;
out:
	return rv;
}

/*
 * Remove stm32 uart device
 */
static int __devexit stm32_release(struct device *dev)
{
	struct uart_port	*port = dev_get_drvdata(dev);
	int			rv = 0;

	if (port) {
		rv = uart_remove_one_port(&stm32_uart_driver, port);
		dev_set_drvdata(dev, NULL);
		port->mapbase = 0;
	}

	return rv;
}

/*
 * Platform bus binding
 */
static int __devinit stm32_probe(struct platform_device *pdev)
{
	struct resource	*reg_res, *irq_res;
	int		rv = -ENODEV;

	reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!reg_res)
		goto out;

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq_res)
		goto out;

	rv = stm32_assign(&pdev->dev, pdev->id, reg_res->start, irq_res->start);
out:
	return rv;
}

/*
 * Platform bus unbinding
 */
static int __devexit stm32_remove(struct platform_device *pdev)
{
	return stm32_release(&pdev->dev);
}

/*
 * Platform driver instance
 */
static struct platform_driver stm32_platform_driver = {
	.probe		= stm32_probe,
	.remove		= __devexit_p(stm32_remove),
	.driver		= {
			.owner	= THIS_MODULE,
			.name	= STM32_USART_DRV_NAME,
			},
};

/*
 * Module init
 */
static int __init stm32_usart_init(void)
{
	int	rv;

	printk(KERN_INFO "Serial: STM32 USART driver\n");

	rv = uart_register_driver(&stm32_uart_driver);
	if (rv) {
		printk(KERN_ERR "%s: uart_register_driver failed (%d)\n",
			__func__, rv);
		goto out;
	}

	rv = platform_driver_register(&stm32_platform_driver);
	if (rv) {
		printk(KERN_ERR "%s: platform_driver_register failed (%d)\n",
			__func__, rv);
		uart_unregister_driver(&stm32_uart_driver);
		goto out;
	}

out:
	return rv;
}

/*
 * Module exit
 */
static void __exit stm32_usart_exit(void)
{
	platform_driver_unregister(&stm32_platform_driver);
	uart_unregister_driver(&stm32_uart_driver);
}

module_init(stm32_usart_init);
module_exit(stm32_usart_exit);

MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_DESCRIPTION("STM32 USART serial driver");
MODULE_LICENSE("GPL");
