/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * Contributions
 * Jordi López <jordi.lopg@gmail.com>
 * 06-20-2012 Added termios support and console settings parsing
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
#include <linux/serial_reg.h>
#include <linux/tty.h>

#include <mach/uart.h>
#include <mach/clock.h>
#include <mach/dmaregs.h>

/*
 * Driver settings
 */
#ifdef CONFIG_ARCH_STM32F1
#define STM32_NR_UARTS		4	/* STM32F1 */
#else
#define STM32_NR_UARTS		6	/* STM32F2 */
#endif

#define STM32_USART_NAME	"ttyS"

#define STM32_USART_PORT	"STM32 USART Port"

/*
 * Size, and number of DMAed RX buffers.
 * Notes:
 * - the number of buffers may be either 2, or 1;
 * - in case of strong flow on serial line, i.e. without IDLEs, rx
 * interrupts (from DMA) will be generated at half, and full levels of these
 * buffers.
 */
/* 512 chars */
#define STM32_DMA_RX_BUF_LEN	512

#ifdef CONFIG_ARCH_STM32F1
/* STM32F1: Double buffer mode is not supported */
#define STM32_DMA_RX_BUF_NUM	1
#else
/* STM32F2: Use double buffer mode */
#define STM32_DMA_RX_BUF_NUM	2
#endif

#if (STM32_DMA_RX_BUF_NUM != 2) && (STM32_DMA_RX_BUF_NUM != 1)
# error "Incorrect RX BUF Number configuration."
#endif

/*
 * Define this to enable debugging msgs
 */
#undef DEBUG

#ifdef DEBUG
# define debug(fmt,args...)	printk(fmt, ##args)
#else
# define debug(fmt,args...)
#endif /* DEBUG */

/*
 * USART CR bits
 */
#define STM32_USART_CR1_UE	(1 << 13)	/* USART enable		      */
#define STM32_USART_CR1_M	(1 << 12)	/* USART 8/9 bits word length */
#define STM32_USART_CR1_PCE	(1 << 10)	/* USART parity enable        */
#define STM32_USART_CR1_PS	(1 << 9)	/* USART parity	selection     */
#define STM32_USART_CR1_TXEIE	(1 << 7)	/* TXE interrupt enable	      */
#define STM32_USART_CR1_IDLIE	(1 << 4)	/* IDLE interrupt enable      */
#define STM32_USART_CR1_TE	(1 << 3)	/* Transmitter enable	      */
#define STM32_USART_CR1_RE	(1 << 2)	/* Receiver enable	      */

#define STM32_USART_CR2_STOP_MASK	(3 << 12)
#define STM32_USART_CR2_1STOP		(0 << 12)
#define STM32_USART_CR2_HSTOP		(1 << 12)
#define STM32_USART_CR2_2STOP		(2 << 12)
#define STM32_USART_CR2_1HSTOP		(3 << 12)

#define STM32_USART_CR3_CTSE	(1 << 9)
#define STM32_USART_CR3_RTSE	(1 << 8)
#define STM32_USART_CR3_DMAR	(1 << 6)	/* DMA enable receiver	      */

/*
 * USART SR bits (this is not the full list, see the others in uart.h header)
 */
#define STM32_USART_SR_IDLE	(1 << 4)
#define STM32_USART_SR_ORE	(1 << 3)	/* Overrun error	      */
#define STM32_USART_SR_FE	(1 << 1)	/* Framing error	      */
#define STM32_USART_SR_PE	(1 << 0)	/* Parity error		      */
#define STM32_USART_SR_ERRORS	(STM32_USART_SR_ORE | STM32_USART_SR_FE |      \
				 STM32_USART_SR_PE)
#define STM32_USART_SR_RX_FLAGS	(STM32_USART_SR_ORE | STM32_USART_SR_PE |      \
				 STM32_USART_SR_FE)

/*
 * BRR reg fields
 */
#define STM32_USART_BRR_F_BIT	0		/* fraction of USARTDIV */
#define STM32_USART_BRR_F_MSK	0x0F

#define STM32_USART_BRR_M_BIT	4		/* mantissa of USARTDIV */
#define STM32_USART_BRR_M_MSK	0xFFF

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

#ifndef CONFIG_ARCH_STM32F1
/*
 * DMA streams and channels
 */
enum stm32_dma_stream {
	STM32_DMA_STREAM_0		= 0,
	STM32_DMA_STREAM_1,
	STM32_DMA_STREAM_2,
	STM32_DMA_STREAM_3,
	STM32_DMA_STREAM_4,
	STM32_DMA_STREAM_5,
	STM32_DMA_STREAM_6,
	STM32_DMA_STREAM_7,

	STM32_DMA_STREAM_LAST
};
#endif /* !CONFIG_ARCH_STM32F1 */

enum stm32_dma_chan {
#ifdef CONFIG_ARCH_STM32F1
	STM32_DMA_CHAN_1		= 0,
#else
	STM32_DMA_CHAN_0		= 0,
	STM32_DMA_CHAN_1,
#endif
	STM32_DMA_CHAN_2,
	STM32_DMA_CHAN_3,
	STM32_DMA_CHAN_4,
	STM32_DMA_CHAN_5,
	STM32_DMA_CHAN_6,
	STM32_DMA_CHAN_7,

	STM32_DMA_CHAN_LAST
};

/*
 * DMA initiator
 */
struct stm32_dma_ini {
#ifdef CONFIG_ARCH_STM32F1
	/*
	 * We call DMA channels on STM32F1 `streams` to minimize differences
	 * in the code.
	 */
	enum stm32_dma_chan		stream;
#else
	enum stm32_dma_stream		stream;
	enum stm32_dma_chan		chan;
#endif
};

/*
 * Driver private
 */
struct stm32_usart_priv {
	volatile struct stm32_usart_regs	*reg_usart_base;
	volatile struct stm32_dma_regs		*reg_dma_base;

	int					usart_irq;
	int					dma_irq;

	struct stm32_dma_ini			ini;
	volatile u32				*dma_isr;
	volatile u32				*dma_ifcr;

	/*
	 * TBD: actually, should allocate this in some coherent area
	 */
	u8					rx_buf[2][STM32_DMA_RX_BUF_LEN];
	u32					wbuf;
	u32					wpos;
	u32					rbuf;
	u32					rpos;
};
#define stm32_drv_priv(port)	    (struct stm32_usart_priv *)		       \
				    ((port)->private_data)
#define stm32_drv_regs(port, name)  (struct stm32_##name##_regs *)	       \
				    ((stm32_drv_priv(port))->reg_##name##_base)

#define stm32_usart(port)	(stm32_drv_regs(port, usart))
#define stm32_dma(port)		(stm32_drv_regs(port, dma))

/*
 * Prototypes
 */
static void stm32_xmit_char(volatile struct stm32_usart_regs *uart,
			    unsigned char c);
static void stm32_transmit(struct uart_port *port);

static irqreturn_t stm32_usart_isr(int irq, void *dev_id);
static irqreturn_t stm32_dma_isr(int irq, void *dev_id);

/*
 * UART ports and privates
 */
static struct uart_port		stm32_ports[STM32_NR_UARTS];
static struct stm32_usart_priv	stm32_usart_priv[STM32_NR_UARTS];

#ifdef CONFIG_ARCH_STM32F1
/*
 * STM32F1: DMA peripheral channels (USART1,2,3 - DMA1, UART4 - DMA2)
 */
static struct stm32_dma_ini	stm32_usart_rx_dma[STM32_NR_UARTS] = {
	/* USART1 */
	{STM32_DMA_CHAN_5},
	/* USART2 */
	{STM32_DMA_CHAN_6},
	/* USART3 */
	{STM32_DMA_CHAN_3},
	/* UART4 */
	{STM32_DMA_CHAN_3},
};
#else
/*
 * STM32F2: DMA peripheral streams & channels (USART1/6 - DMA2, others - DMA1)
 */
static struct stm32_dma_ini	stm32_usart_rx_dma[STM32_NR_UARTS] = {
	/* USART1 */
	{STM32_DMA_STREAM_5, STM32_DMA_CHAN_4},
	/* USART2 */
	{STM32_DMA_STREAM_5, STM32_DMA_CHAN_4},
	/* USART3 */
	{STM32_DMA_STREAM_1, STM32_DMA_CHAN_4},
	/* USART4 */
	{STM32_DMA_STREAM_2, STM32_DMA_CHAN_4},
	/* USART5 */
	{STM32_DMA_STREAM_0, STM32_DMA_CHAN_4},
	/* USART6 */
	{STM32_DMA_STREAM_2, STM32_DMA_CHAN_5}
};
#endif

#ifdef CONFIG_ARCH_STM32F1
/*
 * STM32F1: Channel ISR bits for half[0]/complete[1] transfers
 */
static u32 stm32_dma_isr_bit[STM32_DMA_CHAN_LAST] = {
	(1 <<  1) | (1 <<  2), /* Channel 1 */
	(1 <<  5) | (1 <<  6),
	(1 <<  9) | (1 << 10),
	(1 << 13) | (1 << 14),
	(1 << 17) | (1 << 18),
	(1 << 21) | (1 << 22),
	(1 << 25) | (1 << 26), /* Channel 7 */
};
#else
/*
 * STM32F2: Stream ISR bits for half[0]/complete[1] transfers
 */
static u32 stm32_dma_isr_bit[STM32_DMA_STREAM_LAST] = {
	(1 <<  4) | (1 <<  5),
	(1 << 10) | (1 << 11),
	(1 << 20) | (1 << 21),
	(1 << 26) | (1 << 27),
	(1 <<  4) | (1 <<  5),
	(1 << 10) | (1 << 11),
	(1 << 20) | (1 << 21),
	(1 << 26) | (1 << 27)
};
#endif

/*
 * Check if transmitter in idle (tx reg empty)
 */
static u32 stm_port_tx_empty(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);
	unsigned long				flags;
	u32					rv;

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
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);
	unsigned long				flags;

	spin_lock_irqsave(&port->lock, flags);
	uart->cr1 &= ~STM32_USART_CR1_TXEIE;
	spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * Start transmitter
 */
static void stm_port_start_tx(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);
	unsigned long				flags;

	spin_lock_irqsave(&port->lock, flags);
	uart->cr1 |= STM32_USART_CR1_TXEIE;
	spin_unlock_irqrestore(&port->lock, flags);

	stm32_transmit(port);
}

/*
 * Stop receiver
 */
static void stm_port_stop_rx(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);
	volatile struct stm32_dma_regs		*dma = stm32_dma(port);
	struct stm32_usart_priv			*priv = stm32_drv_priv(port);
	unsigned long				flags;

	spin_lock_irqsave(&port->lock, flags);
	/*
	 * Stop DMA
	 */
	dma->s[priv->ini.stream].cr &= ~STM32_DMA_CR_EN;
	while (dma->s[priv->ini.stream].cr & STM32_DMA_CR_EN);

	/*
	 * Stop USART receiver
	 */
	uart->cr1 &= ~STM32_USART_CR1_RE;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void stm_set_baud_rate(struct uart_port *port, int baudrate)
{
	u32 apb_clock, int_div, frac_div;
	u16 tmp;

	if (port->line == 0 || port->line == 5)
		apb_clock = stm32_clock_get(CLOCK_PCLK2);
	else
		apb_clock = stm32_clock_get(CLOCK_PCLK1);

	/*
	 * Assume oversampling mode of 16 Samples
	 */
	int_div = (25 * apb_clock) / (4 * baudrate);

	tmp = (int_div / 100) << STM32_USART_BRR_M_BIT;
	frac_div = int_div - (100 * (tmp >> 4));
	tmp |= (((frac_div * 16) + 50) / 100) & STM32_USART_BRR_F_MSK;

	stm32_usart(port)->brr = tmp;
}

/*
 * Open the port
 */
static int stm_port_startup(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);
	volatile struct stm32_dma_regs		*dma = stm32_dma(port);
	struct stm32_usart_priv			*priv = stm32_drv_priv(port);
	u32					tmp;
	int					rv;

	/*
	 * Reinitialize offsets in DMA buffers, otherwise wrong data will be
	 * received after second `open()` system call operation.
	 */
	priv->wbuf = 0;
	priv->wpos = 0;
	priv->rbuf = 0;
	priv->rpos = 0;

	rv = request_irq(priv->usart_irq, stm32_usart_isr,
			 IRQF_DISABLED | IRQF_SAMPLE_RANDOM,
			 STM32_USART_PORT, port);
	if (rv) {
		printk(KERN_ERR "%s: request_irq (%d)failed (%d)\n",
			__func__, priv->usart_irq, rv);
		goto out;
	}
	rv = request_irq(priv->dma_irq, stm32_dma_isr,
			 IRQF_DISABLED | IRQF_SAMPLE_RANDOM,
			 STM32_USART_PORT, port);
	if (rv) {
		printk(KERN_ERR "%s: request_irq (%d)failed (%d)\n",
			__func__, priv->dma_irq, rv);
		free_irq(priv->usart_irq, port);
		goto out;
	}

	/*
	 * Configure DMA to receive from USART:
	 * - circular mode,
	 * - high priority,
	 * - full/half interrupts enable
	 */
	tmp =
#ifndef CONFIG_ARCH_STM32F1
		(priv->ini.chan << STM32_DMA_CR_CHSEL_BIT) |
#endif
		(STM32_DMA_CR_PL_HIGH << STM32_DMA_CR_PL_BIT) |
		STM32_DMA_CR_CIRC | STM32_DMA_CR_MINC |
		STM32_DMA_CR_TCIE | STM32_DMA_CR_HTIE;
#if (STM32_DMA_RX_BUF_NUM == 2)
	/*
	 * Double buffers
	 */
	tmp |= STM32_DMA_CR_DBM;
#endif

	dma->s[priv->ini.stream].cr &= ~STM32_DMA_CR_EN;
	while (dma->s[priv->ini.stream].cr & STM32_DMA_CR_EN);

	dma->s[priv->ini.stream].cr   = tmp;
	dma->s[priv->ini.stream].ndtr = STM32_DMA_RX_BUF_LEN;
	dma->s[priv->ini.stream].par  = &uart->dr;
	dma->s[priv->ini.stream].m0ar = &priv->rx_buf[0][0];
#if (STM32_DMA_RX_BUF_NUM == 2)
	dma->s[priv->ini.stream].m1ar = &priv->rx_buf[1][0];
#endif
	dma->s[priv->ini.stream].cr  |= STM32_DMA_CR_EN;

	/*
	 * Configure USART
	 */
	uart->sr = 0;

	/*
	 * Enable Tx and Rx
	 */
	uart->cr1 = STM32_USART_CR1_TE | STM32_USART_CR1_RE;

	/*
	 * CR2:
	 * - 1 Stop bit
	 */
	uart->cr2 = 0;

	/*
	 * Read SR & DR to clear IDLE
	 */
	rv = uart->sr;
	rv = uart->dr;

	/*
	 * Enable DMA access to USART
	 */
	uart->cr3 = STM32_USART_CR3_DMAR;

	/*
	 * Enable RX-idle & TX-empty interrupts
	 */
	uart->cr1 |= STM32_USART_CR1_IDLIE | STM32_USART_CR1_TXEIE;

	/*
	 * Set baudrate to the default value of 115200 if the baudrate
	 * was not set yet.
	 */
	if ((stm32_usart(port)->brr &
	     ((STM32_USART_BRR_F_MSK << STM32_USART_BRR_F_BIT) |
	      (STM32_USART_BRR_M_MSK << STM32_USART_BRR_M_BIT))) == 0)
		stm_set_baud_rate(port, 115200);

	/*
	 * Enable USART
	 */
	uart->cr1 |= STM32_USART_CR1_UE;

	rv = 0;
out:
	return rv;
}

/*
 * Disable the port
 */
static void stm_port_shutdown(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);
	volatile struct stm32_dma_regs		*dma = stm32_dma(port);
	struct stm32_usart_priv			*priv = stm32_drv_priv(port);

	dma->s[priv->ini.stream].cr &= ~STM32_DMA_CR_EN;
	while (dma->s[priv->ini.stream].cr & STM32_DMA_CR_EN);

	uart->cr1 &= ~(STM32_USART_CR1_TE | STM32_USART_CR1_RE |
		       STM32_USART_CR1_UE);
	uart->cr1 &= ~(STM32_USART_CR1_IDLIE | STM32_USART_CR1_TXEIE);
	uart->sr   = 0;

	free_irq(priv->dma_irq, port);
	free_irq(priv->usart_irq, port);
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
static void stm_port_set_termios(struct uart_port *port,
				 struct ktermios *termios, struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, max_baud;
	volatile struct stm32_usart_regs *uart = stm32_usart(port);
	u32 newcr1, newcr2, newcr3;

	newcr1 = uart->cr1;
	newcr2 = uart->cr2;
	newcr3 = uart->cr3;

	/* Calculate new word length and parity */
	newcr1 &= ~(STM32_USART_CR1_M | STM32_USART_CR1_PCE | STM32_USART_CR1_PS);
	if (termios->c_cflag & PARENB) {
		newcr1 |= STM32_USART_CR1_PCE;
		if (termios->c_cflag & PARODD)
			newcr1 |= STM32_USART_CR1_PS;

		if ((termios->c_cflag & CSIZE) == CS8)
			newcr1 |= STM32_USART_CR1_M;
	}

	/* Calculate stop bits */
	newcr2 &= ~STM32_USART_CR2_STOP_MASK;
	if (termios->c_cflag & CSTOPB)
		newcr2 |= STM32_USART_CR2_2STOP;

	/* Calculate new flow control */
	newcr3 &= ~(STM32_USART_CR3_CTSE | STM32_USART_CR3_RTSE);
	if (termios->c_cflag & CRTSCTS)
		newcr3 |= (STM32_USART_CR3_CTSE | STM32_USART_CR3_RTSE);

	/* Determine maximum baudrate from USART clock */
	if (port->line == 0 || port->line == 5)
		max_baud = stm32_clock_get(CLOCK_PCLK2) >> 4;
	else
		max_baud = stm32_clock_get(CLOCK_PCLK1) >> 4;

	/* Calculate new baud rate */
	baud = uart_get_baud_rate(port, termios, old, 0, max_baud);

	spin_lock_irqsave(&port->lock, flags);

	/* update timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	/* set up USART registers */
	uart->cr1 = newcr1;
	uart->cr2 = newcr2;
	uart->cr3 = newcr3;
	stm_set_baud_rate(port, baud);

	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);

	spin_unlock_irqrestore(&port->lock, flags);
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
	.set_termios	= stm_port_set_termios,
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
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);

	stm32_xmit_char(uart, ch);
}

/*
 * Send string to console
 */
static void stm_console_write(struct console *co, const char *s,
				unsigned int count)
{
	struct uart_port			*port = &stm32_ports[co->index];
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);
	volatile struct stm32_dma_regs		*dma = stm32_dma(port);
	struct stm32_usart_priv			*priv = stm32_drv_priv(port);
	unsigned long				flags;
	u32					iuarte, idmae;
	int					locked;

	if (oops_in_progress) {
		locked = spin_trylock_irqsave(&port->lock, flags);
	} else {
		locked = 1;
		spin_lock_irqsave(&port->lock, flags);
	}

	/*
	 * Save and disable interrupts
	 */
	iuarte = uart->cr1 & (STM32_USART_CR1_IDLIE | STM32_USART_CR1_TXEIE);
	idmae  = dma->s[priv->ini.stream].cr & (STM32_DMA_CR_TCIE |
						STM32_DMA_CR_HTIE);

	if (iuarte)
		uart->cr1 &= ~iuarte;
	if (idmae)
		dma->s[priv->ini.stream].cr &= ~idmae;

	uart_console_write(port, s, count, stm_console_putchar);

	/*
	 * Restore interrupt state
	 */
	if (idmae)
		dma->s[priv->ini.stream].cr |= idmae;

	if (iuarte)
		uart->cr1 |= iuarte;

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
	/*
	 * Set of console default values. They will be used only if
	 * no options are given, may be changed at will.
	 */
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= STM32_NR_UARTS) {
		rv = -EINVAL;
		goto out;
	}

	port = &stm32_ports[co->index];
	if (!port->private_data) {
		pr_debug("console on %s%i not present\n", STM32_USART_NAME,
			 co->index);
		rv = -ENODEV;
		goto out;
	}

	/* If options are present parse them and overide defaults */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	/* Set options to console and port termios */
	rv = uart_set_options(port, co, baud, parity, bits, flow);
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
static void stm32_transmit(struct uart_port *port)
{
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);
	struct circ_buf				*xmit;

	if (port->x_char) {
		stm32_xmit_char(uart, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
		goto out;
	}

	xmit = &port->state->xmit;
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		stm_port_stop_tx(port);
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
out:
	return;
}

/*
 * Process receive interrupt event
 */
static void stm32_receive(struct uart_port *port)
{
	volatile struct stm32_dma_regs		*dma = stm32_dma(port);
	struct stm32_usart_priv			*priv = stm32_drv_priv(port);
	struct tty_struct			*tty = port->state->port.tty;
	u32					new_wbuf, new_wpos, tmp, wrap;

	/*
	 * Read DMA current buf, counter, and make sure we done this atomic
	 */
#ifdef CONFIG_ARCH_STM32F1
	/* Double Buffer Mode is not supported on STM32F1 */
	new_wbuf = 0;
#else
	new_wbuf = !!(dma->s[priv->ini.stream].cr & STM32_DMA_CR_CT);
#endif
	new_wpos = dma->s[priv->ini.stream].ndtr;
#if (STM32_DMA_RX_BUF_NUM == 2)
	tmp = !!(dma->s[priv->ini.stream].cr & STM32_DMA_CR_CT);
	if (new_wbuf != tmp) {
		/*
		 * DMA changed the buffer (maybe between our reads from CR
		 * and NDTR)
		 */
		new_wpos = dma->s[priv->ini.stream].ndtr;
		new_wbuf = tmp;
	}
#endif

	/*
	 * Convert 'remained space' to 'written bytes'
	 */
	new_wpos = STM32_DMA_RX_BUF_LEN - new_wpos;

#if (STM32_DMA_RX_BUF_NUM == 2)
	/*
	 * Wrap criteria: DMA switched to another buffer, than we saw
	 * last time. Otherwise, assume DMA is still ahead the reader.
	 * Note, actually may use the same criteria as for 1-buf case.
	 */
	wrap = likely(new_wbuf == priv->wbuf) ? 0 : 1;
#else
	/*
	 * Wrap criteria: DMA pointer becomes less, that we saw last
	 * time. Otherwise, assume DMA is still ahead the reader.
	 */
	wrap = likely(priv->wpos <= new_wpos) ? 0 : 1;
#endif

	if (unlikely(wrap)) {
		if (priv->rpos == STM32_DMA_RX_BUF_LEN) {
			/*
			 * Reader completed this buf, switch to the next one
			 * (or to the beginning of the same, if have 1 buf)
			 */
			priv->rpos = 0;
			priv->wpos = new_wpos;
			priv->wbuf = priv->rbuf = new_wbuf;

			/*
			 * We've just wrapped, clear the pending DMA interrupt
			 * (full buffer complete)
			 */
			wrap = 0;
		} else {
			/*
			 * There are unread chars at the end of the buffer
			 * completed by DMA, read these chars to the end
			 */
			priv->wpos = STM32_DMA_RX_BUF_LEN;
		}
	} else {
		/*
		 * No wrap, just update the writer position
		 */
		priv->wpos = new_wpos;
	}

	debug("%s: w:[%d;%d]; r[%d;%d] -> ", __func__, priv->wbuf, priv->wpos,
		priv->rbuf, priv->rpos);

	/*
	 * Fetch rxed data from buffer. Note, we do not control overflows
	 * here. That is, DMA may catch us, and overwrite data ahead of
	 * our 'rpos'.
	 * Do read until catch the writer
	 */
	while (priv->rpos < priv->wpos) {
		tty_insert_flip_char(tty, priv->rx_buf[priv->rbuf][priv->rpos],
				     TTY_NORMAL);
		priv->rpos++;
	}

	debug("w:[%d;%d]; r[%d;%d] [%s,%d,%d,%x/%x,%d/%d]\n", priv->wbuf, priv->wpos,
		priv->rbuf, priv->rpos);

	/*
	 * Let's DMA interrupt pending to come here again asap, and
	 * to read data in new buffer (or behind us if have 1 buf) if wrap
	 * has a place. Otherwise, just ACK DMA
	 */
	if (likely(!wrap)) {
		tmp = *priv->dma_isr & stm32_dma_isr_bit[priv->ini.stream];
		if (tmp)
			*priv->dma_ifcr = tmp;
	}

	tty_flip_buffer_push(tty);
}

/*
 * STM32 USART irq handler
 */
static irqreturn_t stm32_usart_isr(int irq, void *dev_id)
{
	struct uart_port			*port = dev_id;
	volatile struct stm32_usart_regs	*uart = stm32_usart(port);

	/*
	 * TBD: without reading DR IDLE interrupt continue pending; check
	 * if this read don't lead to missing the read char, and it's DMAed
	 * to buf
	 */
	if (uart->sr & STM32_USART_SR_IDLE) {
		u8	tmp;

		tmp = uart->dr;
		stm32_receive(dev_id);
	}

	stm32_transmit(dev_id);

	return IRQ_HANDLED;
}

/*
 * STM32 DMA irq handler
 */
static irqreturn_t stm32_dma_isr(int irq, void *dev_id)
{
	stm32_receive(dev_id);

	return IRQ_HANDLED;
}

/*
 * Remove stm32 uart device
 */
static int __devexit stm32_release(struct device *dev)
{
	struct uart_port	*port = dev_get_drvdata(dev);
	struct stm32_usart_priv	*priv;
	int			rv = 0;

	if (port) {
		priv = stm32_drv_priv(port);
		rv = uart_remove_one_port(&stm32_uart_driver, port);
		dev_set_drvdata(dev, NULL);
		if (priv) {
			priv->reg_usart_base = NULL;
			priv->reg_dma_base = NULL;
		}
	}

	return rv;
}

/*
 * Platform bus binding
 */
static int __devinit stm32_probe(struct platform_device *pdev)
{
	struct stm32_usart_priv	*priv;
	struct uart_port	*port;
	struct resource		*usart_reg_res, *usart_irq_res;
	struct resource		*dma_reg_res, *dma_irq_res;
	struct device		*dev = &pdev->dev;
	int			id = pdev->id, rv;

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

	priv = &stm32_usart_priv[id];

	usart_reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dma_reg_res   = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	usart_irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	dma_irq_res   = platform_get_resource(pdev, IORESOURCE_IRQ, 1);

	if (!usart_reg_res || !dma_reg_res || !usart_irq_res || !dma_irq_res) {
		rv = -ENODEV;
		goto out;
	}

	if (stm32_ports[id].mapbase &&
	    stm32_ports[id].mapbase != usart_reg_res->start) {
		dev_err(dev, "%s: port %d already in use\n", __func__, id);
		rv = -EBUSY;
		goto out;
	}

	port = &stm32_ports[id];

	priv->reg_usart_base = (struct stm32_usart_regs *)usart_reg_res->start;
	priv->usart_irq  = usart_irq_res->start;

	priv->reg_dma_base = (struct stm32_dma_regs *)dma_reg_res->start;
	priv->dma_irq  = dma_irq_res->start;

	spin_lock_init(&port->lock);
	port->iotype  = SERIAL_IO_MEM,
	port->irq     = priv->usart_irq;
	port->flags   = UPF_BOOT_AUTOCONF;
	port->line    = id;
	port->ops     = &stm32_uart_ops;
	port->dev     = dev;
	port->mapbase = (u32)priv->reg_usart_base;

	priv->ini = stm32_usart_rx_dma[id];

	/*
	 * STM32F1: Use the only available pair of ISR and IFCR registers
	 *
	 * STM32F2: Choose the pair of ISR and IFCR register relevent
	 * to the DMA stream.
	 */
#ifndef CONFIG_ARCH_STM32F1
	if (priv->ini.stream <= STM32_DMA_STREAM_3) {
#endif
		priv->dma_isr  = &priv->reg_dma_base->lisr;
		priv->dma_ifcr = &priv->reg_dma_base->lifcr;
#ifndef CONFIG_ARCH_STM32F1
	} else {
		priv->dma_isr  = &priv->reg_dma_base->hisr;
		priv->dma_ifcr = &priv->reg_dma_base->hifcr;
	}
#endif

	port->private_data = &stm32_usart_priv[id];
	dev_set_drvdata(dev, port);

	/*
	 * Register the port
	 */
	rv = uart_add_one_port(&stm32_uart_driver, port);
	if (rv) {
		dev_err(dev, "%s: uart_add_one_port failed (%d)\n",
			__func__, rv);
		dev_set_drvdata(dev, NULL);
		priv->reg_usart_base = NULL;
		priv->reg_dma_base = NULL;
		goto out;
	}
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
