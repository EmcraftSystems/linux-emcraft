/*
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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
 * Freescale Kinetis UART platform driver
 */

#include <linux/console.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/clk.h>
#include <linux/kinetis_uart.h>

#include <mach/uart.h>

/*
 * Driver settings
 */
#define KINETIS_NR_UARTS	6
#define KINETIS_UART_NAME	"ttyS"
#define KINETIS_UART_PORT	"Kinetis UART Port"
#define KINETIS_DRIVER_NAME	"kinetis-uart"

/*
 * Distrubution of the baudrate divisor value over the BDH/BDL registers and
 * the C4[BRFA] bit field.
 */
/* C4[BRFA] (Baud Rate Fine Adjust) */
#define KINETIS_UART_BRFA_BITWIDTH	5
#define KINETIS_UART_BRFA_BITWIDTH_MSK	((1 << KINETIS_UART_BRFA_BITWIDTH) - 1)
/* BDL (Baud Rate Registers: Low) */
#define KINETIS_UART_BDL_BITWIDTH	8
#define KINETIS_UART_BDL_BITWIDTH_MSK	((1 << KINETIS_UART_BDL_BITWIDTH) - 1)
/* BDH (Baud Rate Registers: High) */
#define KINETIS_UART_BDH_BITWIDTH	5
#define KINETIS_UART_BDH_BITWIDTH_MSK	((1 << KINETIS_UART_BDH_BITWIDTH) - 1)

/*
 * UART registers
 */
/*
 * UART Baud Rate Registers: High
 */
#define KINETIS_UART_BDH_SBR_BITS	0
#define KINETIS_UART_BDH_SBR_MSK	(KINETIS_UART_BDH_BITWIDTH_MSK << \
					KINETIS_UART_BDH_SBR_BITS)

/*
 * UART Baud Rate Registers: Low
 */
#define KINETIS_UART_BDL_SBR_BITS	0

/*
 * UART Status Register 1
 */
/* Receive Data Register Full Flag */
#define KINETIS_UART_S1_RDRF_MSK	(1 << 5)
/* Transmit Data Register Empty Flag */
#define KINETIS_UART_S1_TDRE_MSK	(1 << 7)

/*
 * UART Control Register 1
 */
/* 9-bit or 8-bit Mode Select */
#define KINETIS_UART_C1_M_MSK		(1 << 4)
/* Parity Enable */
#define KINETIS_UART_C1_PE_MSK		(1 << 1)
/* Parity Type: 0=even parity, 1=odd parity */
#define KINETIS_UART_C1_PT_MSK		(1 << 0)

/*
 * UART Control Register 2
 */
/* Receiver Enable */
#define KINETIS_UART_C2_RE_MSK	(1 << 2)
/* Transmitter Enable */
#define KINETIS_UART_C2_TE_MSK	(1 << 3)
/* Receiver Full Interrupt or DMA Transfer Enable */
#define KINETIS_UART_C2_RIE_MSK	(1 << 5)
/* Transmitter Interrupt or DMA Transfer Enable */
#define KINETIS_UART_C2_TIE_MSK	(1 << 7)

/*
 * UART Control Register 4
 */
/* Baud Rate Fine Adjust */
#define KINETIS_UART_C4_BRFA_BITS	0
#define KINETIS_UART_C4_BRFA_MSK	(KINETIS_UART_BRFA_BITWIDTH_MSK << \
					KINETIS_UART_C4_BRFA_BITS)

/*
 * UART Control Register 5
 */
/* Receiver Full DMA Select */
#define KINETIS_UART_C5_RDMAS_MSK	(1 << 5)
/* Transmitter DMA Select */
#define KINETIS_UART_C5_TDMAS_MSK	(1 << 7)

/*
 * UART Modem Register
 */
/* Receiver request-to-send enable */
#define KINETIS_UART_MODEM_RXRTSE_MSK	(1 << 3)
/* Transmitter clear-to-send enable */
#define KINETIS_UART_MODEM_TXCTSE_MSK	(1 << 0)


static struct console kinetis_console;
static void kinetis_console_write(
	struct console *co, const char *s, unsigned int count);

/*
 * UART register map
 */
struct kinetis_uart_regs {
	u8 bdh;		/* Baud Rate Registers: High */
	u8 bdl;		/* Baud Rate Registers: Low */
	u8 c1;		/* Control Register 1 */
	u8 c2;		/* Control Register 2 */
	u8 s1;		/* Status Register 1 */
	u8 s2;		/* Status Register 2 */
	u8 c3;		/* Control Register 3 */
	u8 d;		/* Data Register */
	u8 m1;		/* Match Address Registers 1 */
	u8 m2;		/* Match Address Registers 2 */
	u8 c4;		/* Control Register 4 */
	u8 c5;		/* Control Register 5 */
	u8 ed;		/* Extended Data Register */
	u8 modem;	/* Modem Register */
	u8 ir;		/* Infrared Register */
	u8 rsv0;
	u8 pfifo;	/* FIFO Parameters */
	u8 cfifo;	/* FIFO Control Register */
	u8 sfifo;	/* FIFO Status Register */
	u8 twfifo;	/* FIFO Transmit Watermark */
	u8 tcfifo;	/* FIFO Transmit Count */
	u8 rwfifo;	/* FIFO Receive Watermark */
	u8 rcfifo;	/* FIFO Receive Count */
	u8 rsv1;
	u8 c7816;	/* 7816 Control Register */
	u8 ie7816;	/* 7816 Interrupt Enable Register */
	u8 is7816;	/* 7816 Interrupt Status Register */
	u8 wp7816t;	/* 7816 Wait Parameter Register */
	u8 wn7816;	/* 7816 Wait N Register */
	u8 wf7816;	/* 7816 Wait FD Register */
	u8 et7816;	/* 7816 Error Threshold Register */
	u8 tl7816;	/* 7816 Transmit Length Register */
};

/*
 * Driver private
 */
struct kinetis_uart_priv {
	struct uart_port port;

	volatile struct kinetis_uart_regs *regs;
	int stat_irq;
	int err_irq;
	struct clk *clk;

	int have_ctsrts;
};
#define kinetis_up(p)	(container_of((p), struct kinetis_uart_priv, port))
#define kinetis_regs(p)	(kinetis_up(p)->regs)

/*
 * UART ports and privates
 */
static struct kinetis_uart_priv kinetis_uart_priv[KINETIS_NR_UARTS];

/*
 * Prototypes
 */
static void kinetis_uart_tx_chars(struct kinetis_uart_priv *up);
static void kinetis_stop_tx(struct uart_port *port);

static void kinetis_uart_rx_irq(struct kinetis_uart_priv *up)
{
	struct tty_struct *tty = up->port.state->port.tty;
	volatile struct kinetis_uart_regs *regs = up->regs;
	unsigned char ch;

	/*
	 * Since this function was called, there is at least one
	 * character in the receive FIFO.
	 */
	do {
		ch = regs->d;
		up->port.icount.rx++;

		if (uart_handle_sysrq_char(&up->port, ch))
			;
		else
			tty_insert_flip_char(tty, ch, TTY_NORMAL);
	} while (regs->s1 & KINETIS_UART_S1_RDRF_MSK);
	tty_flip_buffer_push(tty);
}

static void kinetis_uart_tx_irq(struct kinetis_uart_priv *up)
{
	kinetis_uart_tx_chars(up);
}

static irqreturn_t kinetis_uart_status_irq(int irq, void *dev_id)
{
	struct kinetis_uart_priv *up = dev_id;
	volatile struct kinetis_uart_regs *regs = up->regs;
	u8 status;

	status = regs->s1;
	if (status & KINETIS_UART_S1_RDRF_MSK)
		kinetis_uart_rx_irq(up);
	if (status & KINETIS_UART_S1_TDRE_MSK)
		kinetis_uart_tx_irq(up);

	return IRQ_HANDLED;
}

static irqreturn_t kinetis_uart_error_irq(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

/*
 * Request IRQs, enable interrupts
 */
static int kinetis_startup(struct uart_port *port)
{
	struct kinetis_uart_priv *up = kinetis_up(port);
	volatile struct kinetis_uart_regs *regs = up->regs;
	int rv;

	/*
	 * Set up the status interrupt handler
	 */
	rv = request_irq(up->stat_irq, kinetis_uart_status_irq,
			 IRQF_DISABLED | IRQF_SAMPLE_RANDOM,
			 KINETIS_UART_PORT, up);
	if (rv) {
		printk(KERN_ERR "%s: request_irq (%d)failed (%d)\n",
			__func__, up->stat_irq, rv);
		goto out;
	}

	/*
	 * Set up the error interrupt handler
	 */
	rv = request_irq(up->err_irq, kinetis_uart_error_irq,
			 IRQF_DISABLED,
			 KINETIS_UART_PORT, up);
	if (rv) {
		printk(KERN_ERR "%s: request_irq (%d)failed (%d)\n",
			__func__, up->err_irq, rv);
		free_irq(up->stat_irq, up);
		goto out;
	}

	/*
	 * Enable UART receive interrupts
	 */
	regs->c5 &= ~(KINETIS_UART_C5_TDMAS_MSK | KINETIS_UART_C5_RDMAS_MSK);
	regs->c2 |= KINETIS_UART_C2_RIE_MSK;

	/*
	 * Enable receiver and transmitter
	 */
	regs->c2 |= KINETIS_UART_C2_RE_MSK | KINETIS_UART_C2_TE_MSK;

	rv = 0;
out:
	return rv;
}

static void kinetis_uart_tx_chars(struct kinetis_uart_priv *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	volatile struct kinetis_uart_regs *regs = up->regs;

	/*
	 * Exit if the transmit FIFO is full
	 */
	if (!(regs->s1 & KINETIS_UART_S1_TDRE_MSK))
		goto out;

	if (up->port.x_char) {
		/*
		 * Send one character to the transmit FIFO
		 */
		regs->d = up->port.x_char;

		up->port.icount.tx++;
		up->port.x_char = 0;
		goto out;
	}

	/*
	 * If there are no more characters to transmit, disable
	 * the transmit interrupt.
	 */
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		kinetis_stop_tx(&up->port);
		goto out;
	}

	while ((regs->s1 & KINETIS_UART_S1_TDRE_MSK) &&
	       !uart_circ_empty(xmit)) {
		/*
		 * Send one character to the transmit FIFO
		 */
		regs->d = xmit->buf[xmit->tail];

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

out:
	return;
}

static unsigned int kinetis_tx_empty(struct uart_port *port)
{
	unsigned int rv;
	unsigned long flags;
	volatile struct kinetis_uart_regs *regs = kinetis_regs(port);

	spin_lock_irqsave(&port->lock, flags);
	rv = (regs->s1 & KINETIS_UART_S1_TDRE_MSK) ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&port->lock, flags);

	return rv;
}

static unsigned int kinetis_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_CD | TIOCM_DSR;
}

static void kinetis_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void kinetis_stop_tx(struct uart_port *port)
{
	unsigned long flags;

	/*
	 * Disable the transmit interrupt, because otherwise we will get
	 * flooded with interrupts.
	 */
	spin_lock_irqsave(&port->lock, flags);
	kinetis_regs(port)->c2 &= ~KINETIS_UART_C2_TIE_MSK;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void kinetis_start_tx(struct uart_port *port)
{
	unsigned long flags;

	/*
	 * Enable the transmit interrupt in order to send the rest of the data
	 */
	spin_lock_irqsave(&port->lock, flags);
	kinetis_regs(port)->c2 |= KINETIS_UART_C2_TIE_MSK;
	spin_unlock_irqrestore(&port->lock, flags);

	kinetis_uart_tx_chars(kinetis_up(port));
}

static void kinetis_stop_rx(struct uart_port *port)
{
}

static void kinetis_enable_ms(struct uart_port *port)
{
}

static void kinetis_break_ctl(struct uart_port *port, int break_state)
{
}

static void kinetis_shutdown(struct uart_port *port)
{
	struct kinetis_uart_priv *up = kinetis_up(port);

	free_irq(up->stat_irq, up);
	free_irq(up->err_irq, up);
}

static void kinetis_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old)
{
	struct kinetis_uart_priv *up = kinetis_up(port);
	volatile struct kinetis_uart_regs *regs = up->regs;
	unsigned long flags;
	/* Requested baudrate in bits/sec */
	unsigned int baud;
	/* If 7-bit with parity requested */
	int want_7bit;
	/* If parity bit requested */
	int want_parity;
	/* UART's base clock rate in Hz */
	unsigned int base_clk;
	/* Baudrate generator divider value */
	u32 br_div;

	/*
	 * UARTs on Kinetis support 8-bit and 9-bit modes, but there is no
	 * standard API for 9-bit mode in Linux, this is why 8-bit mode will
	 * always be used.
	 *
	 * When parity bit is enabled, 7-bit data is also supported.
	 *
	 * If 7-bit with parity is requested, then set it up. In all other
	 * cases, use 8-bit mode.
	 */
	want_7bit = (termios->c_cflag & CSIZE) == CS7 &&
		(termios->c_cflag & PARENB);
	want_parity = termios->c_cflag & PARENB;

	/*
	 * Get base clock for this particular UART
	 */
	base_clk = clk_get_rate(up->clk);
	/*
	 * If base clock is OK, determine the requested baud rate.
	 * The maximum baud rate is 1/16 of the base clock frequency.
	 */
	baud = 0;
	if (base_clk)
		baud = uart_get_baud_rate(port, termios, old, 0, base_clk / 16);

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Disable Rx/Tx while changing UART parameters
	 */
	regs->c2 &= ~(KINETIS_UART_C2_RE_MSK | KINETIS_UART_C2_TE_MSK);

	/*
	 * Change baudrate
	 */
	if (baud) {
		/*
		 * Prepare baudrate divisor.
		 *
		 * 32*SBR + BRFD = 2 * base_clk / baudrate
		 */
		br_div = 2 * base_clk / baud;

		/*
		 * Changes in BDH will not take effect until we write into BDL.
		 * Therefore we have to write into BDH first, and after that
		 * into BDL.
		 */
		regs->bdh =
			(regs->bdh & ~KINETIS_UART_BDH_SBR_MSK) |
			(((br_div >>
				(KINETIS_UART_BRFA_BITWIDTH +
				KINETIS_UART_BDL_BITWIDTH)) &
			KINETIS_UART_BDH_BITWIDTH_MSK) <<
			KINETIS_UART_BDH_SBR_BITS);
		regs->bdl =
			((br_div >> KINETIS_UART_BRFA_BITWIDTH) &
			KINETIS_UART_BDL_BITWIDTH_MSK) <<
			KINETIS_UART_BDL_SBR_BITS;

		/*
		 * Baudrate fine adjust
		 */
		regs->c4 =
			(regs->c4 & ~KINETIS_UART_C4_BRFA_MSK) |
			((br_div & KINETIS_UART_BRFA_BITWIDTH_MSK) <<
			KINETIS_UART_C4_BRFA_BITS);

		/*
		 * Update the per-port timeout.
		 */
		uart_update_timeout(port, termios->c_cflag, baud);
	}

	/*
	 * 8-bit character + parity bit = 9 bits of data.
	 */
	if (!want_7bit && want_parity)
		regs->c1 |= KINETIS_UART_C1_M_MSK;
	else
		regs->c1 &= ~KINETIS_UART_C1_M_MSK;

	/*
	 * Enable or disable parity
	 */
	if (want_parity) {
		regs->c1 |= KINETIS_UART_C1_PE_MSK;

		/*
		 * Set parity type
		 */
		if (termios->c_cflag & PARODD)
			regs->c1 |= KINETIS_UART_C1_PT_MSK;
		else
			regs->c1 &= ~KINETIS_UART_C1_PT_MSK;
	} else {
		regs->c1 &= ~KINETIS_UART_C1_PE_MSK;
	}

	/*
	 * Enable symmetric hardware flow control (CTS/RTS handshaking),
	 * if available and requested.
	 */
	if (up->have_ctsrts && (termios->c_cflag & CRTSCTS)) {
		regs->modem = KINETIS_UART_MODEM_RXRTSE_MSK |
			KINETIS_UART_MODEM_TXCTSE_MSK;
	} else {
		regs->modem = 0;
	}

	/*
	 * Enable Rx/Tx again
	 */
	regs->c2 |= KINETIS_UART_C2_RE_MSK | KINETIS_UART_C2_TE_MSK;

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *kinetis_type(struct uart_port *port)
{
	return KINETIS_UART_PORT;
}

static void kinetis_release_port(struct uart_port *port)
{
}

static int kinetis_request_port(struct uart_port *port)
{
	return 0;
}

static void kinetis_config_port(struct uart_port *port, int flags)
{
	if (!kinetis_request_port(port))
		port->type = PORT_KINETIS;
}

static int kinetis_verify_port(
	struct uart_port *port, struct serial_struct *ser)
{
	return 0;
}

struct uart_ops kinetis_uart_ops = {
	.tx_empty	= kinetis_tx_empty,
	.set_mctrl	= kinetis_set_mctrl,
	.get_mctrl	= kinetis_get_mctrl,
	.stop_tx	= kinetis_stop_tx,
	.start_tx	= kinetis_start_tx,
	.stop_rx	= kinetis_stop_rx,
	.enable_ms	= kinetis_enable_ms,
	.break_ctl	= kinetis_break_ctl,
	.startup	= kinetis_startup,
	.shutdown	= kinetis_shutdown,
	.set_termios	= kinetis_set_termios,
	.type		= kinetis_type,
	.release_port	= kinetis_release_port,
	.request_port	= kinetis_request_port,
	.config_port	= kinetis_config_port,
	.verify_port	= kinetis_verify_port,
};

#ifdef CONFIG_SERIAL_KINETIS_CONSOLE

/*
 * Send char to console
 */
static void kinetis_console_putchar(struct uart_port *port, int ch)
{
	volatile struct kinetis_uart_regs *regs =
		(volatile struct kinetis_uart_regs *)KINETIS_UART2_BASE;

	while (!(regs->s1 & KINETIS_UART_S1_TDRE_MSK));
	regs->d = ch;
}

/*
 * Send string to console
 */
static void kinetis_console_write(
	struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port = &kinetis_uart_priv[co->index].port;
	volatile struct kinetis_uart_regs *regs =
		(volatile struct kinetis_uart_regs *)KINETIS_UART2_BASE;
	u8 uart_ie;
	unsigned long flags;
	int locked;

	if (oops_in_progress) {
		locked = spin_trylock_irqsave(&port->lock, flags);
	} else {
		locked = 1;
		spin_lock_irqsave(&port->lock, flags);
	}

	/*
	 * Save and clear the interrupt enable flags
	 */
	uart_ie =
		regs->c2 & (KINETIS_UART_C2_TIE_MSK | KINETIS_UART_C2_RIE_MSK);
	if (uart_ie)
		regs->c2 &= ~uart_ie;

	uart_console_write(port, s, count, kinetis_console_putchar);

	/*
	 * Restore the interrupt enable flags
	 */
	if (uart_ie)
		regs->c2 |= uart_ie;

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * Setup console
 */
static int __init kinetis_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int rv;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= KINETIS_NR_UARTS) {
		rv = -EINVAL;
		goto out;
	}

	port = &kinetis_uart_priv[co->index].port;
	if (!port->private_data) {
		pr_debug("console on %s%i not present\n", KINETIS_UART_NAME,
			 co->index);
		rv = -ENODEV;
		goto out;
	}

	/*
	 * Configure UART (baud, parity, ...) depending on the options passed
	 * in the kernel parameters line.
	 */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	rv = uart_set_options(port, co, baud, parity, bits, flow);

out:
	return rv;
}

/*
 * Console instance. Use '-1' as the index to get console from the cmdline
 */
static struct uart_driver kinetis_uart_driver;
static struct console kinetis_console = {
	.name	= KINETIS_UART_NAME,
	.device	= uart_console_device,
	.write	= kinetis_console_write,
	.setup	= kinetis_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &kinetis_uart_driver,
};

static int __init kinetis_console_init(void)
{
	register_console(&kinetis_console);
	return 0;
}
console_initcall(kinetis_console_init);

#endif /* CONFIG_SERIAL_KINETIS_CONSOLE */

static struct uart_driver kinetis_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= KINETIS_DRIVER_NAME,
	.dev_name	= KINETIS_UART_NAME,
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= KINETIS_NR_UARTS,
#ifdef CONFIG_SERIAL_KINETIS_CONSOLE
	.cons		= &kinetis_console,
#endif
};

static int kinetis_uart_probe(struct platform_device *pdev)
{
	struct kinetis_uart_priv *up;
	struct uart_port *port;
	struct resource *uart_reg_res;
	struct resource *uart_stat_irq_res, *uart_err_irq_res;
	struct device *dev = &pdev->dev;
	struct kinetis_uart_data *pdata = dev->platform_data;
	int rv;
	int id = pdev->id;

	if (id >= KINETIS_NR_UARTS) {
		dev_err(dev, "%s: bad port id %d\n", __func__, id);
		rv = -EINVAL;
		goto out;
	}

	/*
	 * If id is negative, assign the first free port
	 */
	if (id < 0) {
		for (id = 0; id < KINETIS_NR_UARTS; id++) {
			if (!kinetis_uart_priv[id].port.mapbase)
				break;
		}
	}
	if (id == KINETIS_NR_UARTS) {
		dev_err(dev, "%s: no free port\n", __func__);
		rv = -EBUSY;
		goto out;
	}

	up = &kinetis_uart_priv[id];

	/*
	 * Handle platform data
	 */
	up->have_ctsrts = pdata && (pdata->flags & KINETIS_UART_FLAG_CTSRTS);

	/*
	 * Acquire resources
	 */
	uart_reg_res      = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	uart_stat_irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	uart_err_irq_res  = platform_get_resource(pdev, IORESOURCE_IRQ, 1);

	if (!uart_reg_res || !uart_stat_irq_res || !uart_err_irq_res) {
		rv = -ENODEV;
		goto out;
	}

	if (kinetis_uart_priv[id].port.mapbase &&
	    kinetis_uart_priv[id].port.mapbase != uart_reg_res->start) {
		dev_err(dev, "%s: port %d already in use\n", __func__, id);
		rv = -EBUSY;
		goto out;
	}

	port = &kinetis_uart_priv[id].port;

	up->regs = (volatile struct kinetis_uart_regs *)uart_reg_res->start;
	up->stat_irq  = uart_stat_irq_res->start;
	up->err_irq   = uart_err_irq_res->start;

	spin_lock_init(&port->lock);
	port->iotype  = SERIAL_IO_MEM,
	port->membase = (void __iomem *)up->regs;
	port->mapbase = (u32)up->regs;
	port->irq     = up->stat_irq;
	port->flags   = UPF_BOOT_AUTOCONF;
	port->ops     = &kinetis_uart_ops;
	port->line    = id;
	port->dev     = dev;

	port->private_data = &kinetis_uart_priv[id];
	dev_set_drvdata(dev, port);

	/*
	 * Acquire and enable clock
	 */
	up->clk = clk_get(dev, NULL);
	if (IS_ERR(up->clk)) {
		rv = PTR_ERR(up->clk);
		goto err_cleanup_priv;
	}
	clk_enable(up->clk);

	/*
	 * Register the port
	 */
	rv = uart_add_one_port(&kinetis_uart_driver, port);
	if (rv) {
		dev_err(dev, "%s: uart_add_one_port failed (%d)\n",
			__func__, rv);
		goto err_clk_put;
	}

	goto out;

err_clk_put:
	clk_disable(up->clk);
	clk_put(up->clk);
err_cleanup_priv:
	dev_set_drvdata(dev, NULL);
	up->regs = NULL;
out:
	return rv;
}

static int __devexit kinetis_uart_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct uart_port *port = dev_get_drvdata(dev);
	struct kinetis_uart_priv *up;
	int rv = 0;

	if (!port)
		goto out;

	up = kinetis_up(port);

	/* Unregister the port */
	rv = uart_remove_one_port(&kinetis_uart_driver, port);

	/* Release clock */
	clk_disable(up->clk);
	clk_put(up->clk);

	/* Reset pointers */
	dev_set_drvdata(dev, NULL);
	up->regs = NULL;

out:
	return rv;
}

static struct platform_driver kinetis_platform_driver = {
	.probe		= kinetis_uart_probe,
	.remove		= __devexit_p(kinetis_uart_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= KINETIS_DRIVER_NAME,
	},
};

static int __init kinetis_uart_driver_init(void)
{
	int rv;

	printk(KERN_INFO "Serial: Freescale Kinetis UART driver\n");

	rv = uart_register_driver(&kinetis_uart_driver);
	if (rv) {
		printk(KERN_ERR "%s: uart_register_driver failed (%d)\n",
			__func__, rv);
		goto out;
	}

	rv = platform_driver_register(&kinetis_platform_driver);
	if (rv) {
		printk(KERN_ERR "%s: platform_driver_register failed (%d)\n",
			__func__, rv);
		uart_unregister_driver(&kinetis_uart_driver);
		goto out;
	}

out:
	return rv;
}

static void __exit kinetis_uart_driver_exit(void)
{
	platform_driver_unregister(&kinetis_platform_driver);
	uart_unregister_driver(&kinetis_uart_driver);
}

module_init(kinetis_uart_driver_init);
module_exit(kinetis_uart_driver_exit);

MODULE_AUTHOR("Alexander Potashev <aspotashev@emcraft.com>");
MODULE_DESCRIPTION("Freescale Kinetis UART serial driver");
MODULE_LICENSE("GPL");
