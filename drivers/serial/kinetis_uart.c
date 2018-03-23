/*
 * (C) Copyright 2012-2014
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
 *	Add support for wake up from STOP mode
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
#include <linux/dma-mapping.h>

#include <mach/uart.h>
#include <mach/dmac.h>

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
#define KINETIS_UART_BDH_RXEDGIE	(1<<6)

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
/* Transmit Data Register Empty Flag */
#define KINETIS_UART_S1_TDRE_MSK	(1 << 7)
/* Receive Data Register Full Flag */
#define KINETIS_UART_S1_RDRF_MSK	(1 << 5)
/* Idle Line Flag */
#define KINETIS_UART_S1_IDLE_MSK	(1 << 4)

/*
 * UART Status Register 2
 */
/* Transmit Data Register Empty Flag */
#define KINETIS_UART_S2_RXEDGIF		(1 << 6)

/*
 * UART Control Register 1
 */
/* 9-bit or 8-bit Mode Select */
#define KINETIS_UART_C1_M_MSK		(1 << 4)
/* Idle line counted after start or stop bit */
#define KINETIS_UART_C1_ILT_MSK		(1 << 2)
/* Parity Enable */
#define KINETIS_UART_C1_PE_MSK		(1 << 1)
/* Parity Type: 0=even parity, 1=odd parity */
#define KINETIS_UART_C1_PT_MSK		(1 << 0)

/*
 * UART Control Register 2
 */
/* Transmitter Interrupt or DMA Transfer Enable */
#define KINETIS_UART_C2_TIE_MSK		(1 << 7)
/* Receiver Full Interrupt or DMA Transfer Enable */
#define KINETIS_UART_C2_RIE_MSK		(1 << 5)
/* Idle Line Interrupt Enable */
#define KINETIS_UART_C2_ILIE_MSK	(1 << 4)
/* Transmitter Enable */
#define KINETIS_UART_C2_TE_MSK		(1 << 3)
/* Receiver Enable */
#define KINETIS_UART_C2_RE_MSK		(1 << 2)

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

/* The whole Rx DMA buffer should be filled in approximately 40ms */
#define DMA_RX_BUF_TIMECYCLE	40

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
#if defined(CONFIG_KINETIS_EDMA)
	/* UART Rx DMA channel number */
	int dma_ch_rx;
#endif /* CONFIG_KINETIS_EDMA */
	/* UART DMA receive buffer: CPU pointer and DMA address */
	u8 *rx_buf;
	dma_addr_t rx_dma_handle;
	unsigned int rx_buf_len;
	/* Offset in buffer of the received character to be handled next */
	int rx_cons_idx;
#if defined(CONFIG_PM)
	unsigned int baud;
	struct clk *suspend_clk;
	u32 *wakeup_cnt;
	u8 *wakeup_buf;
#endif /* CONFIG_PM */
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

#if defined(CONFIG_PM)

/*
 * Prepare to wake up from Rx Active Edge on a specified UART port.
 * This is called from low-level suspend code, with interrupts disabled.
 */
void kinetis_uart_prepare_to_suspend(
	int port, unsigned int *cnt, unsigned char *buf, unsigned char *bdr)
{
	struct kinetis_uart_priv *up = &kinetis_uart_priv[port];
	volatile struct kinetis_uart_regs *regs = up->regs;
	unsigned int clock, baud;
	u32 br_div;

	/*
	 * Sanity check: if the port is not enabled, we can't wake up on it.
	 * Bail out with no action in that scenario.
	 */
	if (!(regs->c2 & (KINETIS_UART_C2_RE_MSK | KINETIS_UART_C2_TE_MSK))) {
		goto Done;
	}

	/*
	 * Clear a pending RX active edge interrupt.
	 * Enable RX active enge interrupts.
	 * This will be used to wake up from Stop mode.
	 */
	regs->s2 |= KINETIS_UART_S2_RXEDGIF;
	regs->bdh |= KINETIS_UART_BDH_RXEDGIE;

	/*
	 * Store pointers to the wakeup count and buffer for this UART
	 */
	up->wakeup_cnt = cnt;
	up->wakeup_buf = buf;
	*cnt = 0;

	/*
	 * Store the current baudrate registers
	 */
	baud = up->baud;
	bdr[0] = regs->bdh;
	bdr[1] = regs->bdl;
	bdr[2] = regs->c4;

	/*
	 * Get the suspend clock for this particular UART
	 */
	clock = clk_get_rate(up->suspend_clk);

	/*
	 * Calculate new baudrates for the suspend clock.
	 * 32*SBR + BRFD = 2 * base_clk / baudrate
	 */
	br_div = 2 * clock / baud;

	/*
	 * Changes to BDH and BDL.
	 */
	bdr[3] =
		(regs->bdh & ~KINETIS_UART_BDH_SBR_MSK) |
		(((br_div >>
		(KINETIS_UART_BRFA_BITWIDTH + KINETIS_UART_BDL_BITWIDTH)) &
		KINETIS_UART_BDH_BITWIDTH_MSK) << KINETIS_UART_BDH_SBR_BITS);
	bdr[4] =
		((br_div >> KINETIS_UART_BRFA_BITWIDTH) &
		KINETIS_UART_BDL_BITWIDTH_MSK) << KINETIS_UART_BDL_SBR_BITS;

	/*
	 * Baudrate fine adjust
	 */
	bdr[5] =
		(regs->c4 & ~KINETIS_UART_C4_BRFA_MSK) |
		((br_div & KINETIS_UART_BRFA_BITWIDTH_MSK) <<
		KINETIS_UART_C4_BRFA_BITS);

Done:
	;

}
EXPORT_SYMBOL(kinetis_uart_prepare_to_suspend);

/*
 * IRQ handler for the Rx Active Edge condition
 */
static void kinetis_uart_rx_edge_irq(struct kinetis_uart_priv *up)
{
	struct tty_struct *tty = up->port.state->port.tty;
	volatile struct kinetis_uart_regs *regs = up->regs;
	unsigned int *p = up->wakeup_cnt;
	unsigned char *q = up->wakeup_buf;
	unsigned char ch;
	int i;

	/*
	 * Acknowldge and disable Rx Edge interrupts
	 */
	regs->s2 |= KINETIS_UART_S2_RXEDGIF;
	regs->bdh &= ~KINETIS_UART_BDH_RXEDGIE;

	/*
	 * Process characters in the Rx buffer.
	 * Those have been put to the buffer by low-level wakeup code.
	 */
	for (i = 0; i < *p; i++, q++) {
		ch = *q;
		up->port.icount.rx++;

		if (uart_handle_sysrq_char(&up->port, ch))
			;
		else
			tty_insert_flip_char(tty, ch, TTY_NORMAL);
	}

	tty_flip_buffer_push(tty);
}

#endif

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

#if defined(CONFIG_KINETIS_EDMA)
static void kinetis_uart_handle_dma_rx(struct kinetis_uart_priv *up)
{
	struct tty_struct *tty = up->port.state->port.tty;
	/* Buffer offset where DMA engine will write the next received byte */
	int prod_idx;

	/* Find "head" of the circular DMA buffer */
	prod_idx = kinetis_dma_ch_iter_done(up->dma_ch_rx);
	if (prod_idx < 0) {
		dev_err(up->port.dev, "kinetis_dma_ch_iter_done failed (%d).\n",
			prod_idx);
		goto out;
	}

	if (prod_idx > up->rx_cons_idx) {
		/* Feed to TTY subsystem a continuous chunk in buffer */
		tty_insert_flip_string(tty, up->rx_buf + up->rx_cons_idx,
			prod_idx - up->rx_cons_idx);
	} else if (prod_idx < up->rx_cons_idx) {
		/* prod_idx has wrapped. Handle the first part of data. */
		tty_insert_flip_string(tty, up->rx_buf + up->rx_cons_idx,
			up->rx_buf_len - up->rx_cons_idx);
		/* Handle the second part of data in the beginning of buffer. */
		tty_insert_flip_string(tty, up->rx_buf, prod_idx);
	} else {
		/* No new characters in the receive buffer */
		goto out;
	}

	/* We have processed all new data in buffer */
	up->rx_cons_idx = prod_idx;

	tty_flip_buffer_push(tty);

out:
	;
}
#endif /* CONFIG_KINETIS_EDMA */

static irqreturn_t kinetis_uart_status_irq(int irq, void *dev_id)
{
	struct kinetis_uart_priv *up = dev_id;
	volatile struct kinetis_uart_regs *regs = up->regs;
	u8 status;

#if defined(CONFIG_PM)
	/*
	 * This is an RX active edge interrupt.
	 * It can only occur when exiting a Stop mode.
	 */
	if ((regs->bdh & KINETIS_UART_BDH_RXEDGIE) &&
		(regs->s2 & KINETIS_UART_S2_RXEDGIF)) {
		kinetis_uart_rx_edge_irq(up);
	}
#endif

	status = regs->s1;

	/* Handle character transmission */
	if (status & KINETIS_UART_S1_TDRE_MSK)
		kinetis_uart_tx_irq(up);

	/* Handle received characters (Rx DMA disabled) */
	if (
#if defined(CONFIG_KINETIS_EDMA)
		up->dma_ch_rx < 0 &&
#endif /* CONFIG_KINETIS_EDMA */
		(status & KINETIS_UART_S1_RDRF_MSK))
		kinetis_uart_rx_irq(up);

#if defined(CONFIG_KINETIS_EDMA)
	/* Handle received characters (Rx DMA enabled) */
	if (up->dma_ch_rx >= 0 &&
		(status & KINETIS_UART_S1_IDLE_MSK)) {
		/* Clear S[IDLE] flag by reading from UARTx_D after UART_S */
		u8 tmp;
		tmp = regs->d;

		kinetis_uart_handle_dma_rx(up);
	}
#endif /* CONFIG_KINETIS_EDMA */

	return IRQ_HANDLED;
}

static irqreturn_t kinetis_uart_error_irq(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

#if defined(CONFIG_KINETIS_EDMA)
static void kinetis_uart_dma_irq(int ch, unsigned long flags, void *data)
{
	kinetis_uart_handle_dma_rx(data);
}

/*
 * Calculate the Rx DMA buffer size to ensure the whole buffer fills
 * in around 40ms (DMA_RX_BUF_TIMECYCLE).
 *
 * Based on uart_update_timeout() from serial_core.c
 */
static int kinetis_calc_bufsize(unsigned int cflag, unsigned int baud)
{
	unsigned int bits;

	/* byte size and parity */
	switch (cflag & CSIZE) {
	case CS5:
		bits = 7;
		break;
	case CS6:
		bits = 8;
		break;
	case CS7:
		bits = 9;
		break;
	default:
		bits = 10;
		break; /* CS8 */
	}

	if (cflag & CSTOPB)
		bits++;
	if (cflag & PARENB)
		bits++;

	/* We divide by 1000 because DMA_RX_BUF_TIMECYCLE is in milliseconds */
	return max(10u, baud * DMA_RX_BUF_TIMECYCLE / (1000u * bits));
}

/*
 * Free the DMA coherent UART Rx buffer
 */
static void kinetis_rx_dma_free(struct kinetis_uart_priv *up)
{
	if (up->rx_buf) {
		dma_free_coherent(NULL, up->rx_buf_len, up->rx_buf,
			up->rx_dma_handle);
		up->rx_buf = NULL;
	}
}

/*
 * This function should be called if up->dma_ch_rx is valid.
 * This function must be called with port->lock taken.
 */
static int __kinetis_stop_rx_dma(struct kinetis_uart_priv *up)
{
	volatile struct kinetis_uart_regs *regs = up->regs;
	int rv;

	/* Disable UART transmit DMA */
	regs->c5 &= ~KINETIS_UART_C5_TDMAS_MSK;
	/* Disable UART receive DMA */
	regs->c2 &= ~KINETIS_UART_C2_RIE_MSK;
	regs->c5 &= ~KINETIS_UART_C5_RDMAS_MSK;
	/* Disable "Idle Line" interrupt */
	regs->c2 &= ~KINETIS_UART_C2_ILIE_MSK;

	/* Stop DMA if it was running */
	rv = kinetis_dma_ch_disable(up->dma_ch_rx);

	return rv;
}

/*
 * This function should be called if up->dma_ch_rx is valid.
 * This function must be called with port->lock taken.
 */
static int __kinetis_start_rx_dma(struct kinetis_uart_priv *up,
				  unsigned int cflag, unsigned int baud)
{
	volatile struct kinetis_uart_regs *regs = up->regs;
	int rv;

	/* Allocate Rx DMA buffer */
	up->rx_cons_idx = 0;
	up->rx_buf_len = kinetis_calc_bufsize(cflag, baud);
	up->rx_buf = dma_alloc_coherent(
		NULL, up->rx_buf_len, &up->rx_dma_handle, GFP_KERNEL);
	if (!up->rx_buf) {
		rv = -ENOMEM;
		goto out;
	}

	/* Clear DMA channel TCD (transfer control descriptor) */
	rv = kinetis_dma_ch_init(up->dma_ch_rx);
	if (rv < 0)
		goto out;
	/* DMA source if the UARTx_D 8-bit register */
	rv = kinetis_dma_ch_set_src(up->dma_ch_rx, (dma_addr_t)&regs->d, 0,
		KINETIS_DMA_WIDTH_8BIT, 0);
	if (rv < 0)
		goto out;
	/* DMA destination is the UART Rx DMA buffer */
	rv = kinetis_dma_ch_set_dest(up->dma_ch_rx, up->rx_dma_handle, 1,
		KINETIS_DMA_WIDTH_8BIT, -up->rx_buf_len);
	if (rv < 0)
		goto out;
	/* Transfer 1 character at a time */
	rv = kinetis_dma_ch_set_nbytes(up->dma_ch_rx, 1);
	if (rv < 0)
		goto out;
	/* Wrap destination address when the buffer ends */
	rv = kinetis_dma_ch_set_iter_num(up->dma_ch_rx, up->rx_buf_len);
	if (rv < 0)
		goto out;
	/* Enable UART Rx DMA channel */
	rv = kinetis_dma_ch_enable(up->dma_ch_rx, 0);
	if (rv < 0)
		goto out;

	/* Disable UART transmit DMA */
	regs->c5 &= ~KINETIS_UART_C5_TDMAS_MSK;
	/* Enable UART receive DMA */
	regs->c5 |= KINETIS_UART_C5_RDMAS_MSK;
	regs->c2 |= KINETIS_UART_C2_RIE_MSK;
	/* Enable "Idle Line" interrupt */
	regs->c2 |= KINETIS_UART_C2_ILIE_MSK;
	/* IDLE is counted after a Stop bit */
	regs->c1 |= KINETIS_UART_C1_ILT_MSK;

	rv = 0;
	goto out;

out:
	return rv;
}

/*
 * This function should be called if up->dma_ch_rx is valid.
 *
 * We do not call kinetis_rx_dma_free() from this function, because that
 * requires interrupts to be enabled (due to dma_free_coherent() used).
 * This function must work correctly with interrupts disabled, because it
 * may be called from function kinetis_stop_rx() which executes with
 * interrupts disabled.
 *
 * The UART Rx DMA buffer will be freed anyway in the next call to
 * kinetis_start_rx_dma().
 */
static int kinetis_stop_rx_dma(struct kinetis_uart_priv *up)
{
	unsigned long flags;
	int rv;

	spin_lock_irqsave(&up->port.lock, flags);
	rv = __kinetis_stop_rx_dma(up);
	spin_unlock_irqrestore(&up->port.lock, flags);

	return rv;
}

/*
 * This function should be called if up->dma_ch_rx is valid.
 */
static int kinetis_start_rx_dma(struct kinetis_uart_priv *up,
				unsigned int cflag, unsigned int baud)
{
	unsigned long flags;
	int rv;

	/* Stop DMA if it was running */
	kinetis_stop_rx_dma(up);

	/* Free DMA buffer if it was allocated */
	kinetis_rx_dma_free(up);

	spin_lock_irqsave(&up->port.lock, flags);
	rv = __kinetis_start_rx_dma(up, cflag, baud);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * If __kinetis_start_rx_dma() failed but up->rx_buf was successfully
	 * allocated, then free this buffer.
	 */
	if (rv < 0)
		kinetis_rx_dma_free(up);

	return rv;
}

#endif /* CONFIG_KINETIS_EDMA */

/*
 * Request IRQs, enable interrupts
 */
static int kinetis_startup(struct uart_port *port)
{
	struct kinetis_uart_priv *up = kinetis_up(port);
	volatile struct kinetis_uart_regs *regs = up->regs;
	int rv;

#if defined(CONFIG_PM)
	/*
	 * Disable RX active edge interrupts.
	 */
	regs->s2 |= KINETIS_UART_S2_RXEDGIF;
	regs->bdh &= ~KINETIS_UART_BDH_RXEDGIE;
#endif

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
		goto err_irq_err;
	}

#if defined(CONFIG_KINETIS_EDMA)
	if (up->dma_ch_rx >= 0) {
		/* If using DMA, enable the UART Rx DMA channel */
		rv = kinetis_start_rx_dma(up, CS8, 115200);
		if (rv < 0)
			goto err_start_rx_dma;
	} else {
#endif /* CONFIG_KINETIS_EDMA */
		/* If not using DMA, enable UART receive interrupts */
		regs->c5 &= ~(KINETIS_UART_C5_TDMAS_MSK |
			KINETIS_UART_C5_RDMAS_MSK);
		regs->c2 |= KINETIS_UART_C2_RIE_MSK;
#if defined(CONFIG_KINETIS_EDMA)
	}
#endif /* CONFIG_KINETIS_EDMA */

	/*
	 * Enable receiver and transmitter
	 */
	regs->c2 |= KINETIS_UART_C2_RE_MSK | KINETIS_UART_C2_TE_MSK;

	rv = 0;
	goto out;

#if defined(CONFIG_KINETIS_EDMA)
err_start_rx_dma:
	free_irq(up->err_irq, up);
#endif /* CONFIG_KINETIS_EDMA */

err_irq_err:
	free_irq(up->stat_irq, up);
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
	struct kinetis_uart_priv *up = kinetis_up(port);
	volatile struct kinetis_uart_regs *regs = up->regs;

#if defined(CONFIG_KINETIS_EDMA)
	if (up->dma_ch_rx >= 0) {
		/* If using DMA, stop receive DMA */
		kinetis_stop_rx_dma(up);
	} else {
#endif /* CONFIG_KINETIS_EDMA */
		/* If not using DMA, disable receive IRQ */
		regs->c2 &= ~KINETIS_UART_C2_RIE_MSK;
#if defined(CONFIG_KINETIS_EDMA)
	}
#endif /* CONFIG_KINETIS_EDMA */

	/* Disable receiver */
	regs->c2 &= ~KINETIS_UART_C2_RE_MSK;
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
		baud = uart_get_baud_rate(port, termios,
					old, 0, base_clk / 16);

#if defined(CONFIG_KINETIS_EDMA)
	/*
	 * Disable Rx DMA while changing UART parameters
	 */
	if (up->dma_ch_rx >= 0)
		kinetis_stop_rx_dma(up);
#endif /* CONFIG_KINETIS_EDMA */

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

#if defined(CONFIG_KINETIS_EDMA)
	/*
	 * Disable Rx DMA while changing UART parameters
	 */
	if (up->dma_ch_rx >= 0)
		kinetis_start_rx_dma(up, termios->c_cflag, baud ? baud : 115200);
#endif /* CONFIG_KINETIS_EDMA */

#if defined(CONFIG_PM)
	up->baud =  baud ? baud : 115200;
#endif
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
	volatile struct kinetis_uart_regs *regs = kinetis_regs(port);

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
	volatile struct kinetis_uart_regs *regs = kinetis_regs(port);
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
#if defined(CONFIG_KINETIS_EDMA)
	struct resource *uart_rx_dma_res;
#endif /* CONFIG_KINETIS_EDMA */
	struct device *dev = &pdev->dev;
	struct kinetis_uart_data *pdata = dev->platform_data;
#if defined(CONFIG_PM)
	char suspend_clk_name[32];
#endif
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
#if defined(CONFIG_KINETIS_EDMA)
	uart_rx_dma_res   = platform_get_resource(pdev, IORESOURCE_DMA, 0);
#endif /* CONFIG_KINETIS_EDMA */

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
#if defined(CONFIG_KINETIS_EDMA)
	if (uart_rx_dma_res)
		up->dma_ch_rx = uart_rx_dma_res->start;
	else
		up->dma_ch_rx = -1;
#endif /* CONFIG_KINETIS_EDMA */

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
#if defined(CONFIG_PM)
	sprintf(suspend_clk_name, "kinetis-suspend-uart.%d", id);
	up->suspend_clk = clk_get(NULL, suspend_clk_name);
	if (!IS_ERR(up->suspend_clk)) {
		clk_enable(up->suspend_clk);
		up->baud = 115200;
	}
#endif

#if defined(CONFIG_KINETIS_EDMA)
	/*
	 * Acquire Rx DMA channel, if available
	 */
	if (up->dma_ch_rx >= 0) {
		rv = kinetis_dma_ch_get(up->dma_ch_rx);
		if (rv < 0) {
			dev_warn(dev, "%s: Failed to acquire Rx DMA channel %d "
				"(error %d), falling back to PIO mode.\n",
				__func__, up->dma_ch_rx, rv);

			/* Continue without DMA support */
			up->dma_ch_rx = -1;
		}
	}

	/*
	 * Request Rx DMA IRQ
	 */
	if (up->dma_ch_rx >= 0) {
		rv = kinetis_dma_ch_request_irq(
			up->dma_ch_rx, kinetis_uart_dma_irq,
			KINETIS_DMA_INTMAJOR | KINETIS_DMA_INTHALF, up);
		if (rv < 0) {
			kinetis_dma_ch_put(up->dma_ch_rx);

			dev_warn(dev, "%s: Failed to request Rx DMA IRQ for "
				"channel %d (error %d), falling back "
				"to PIO mode.\n",
				__func__, up->dma_ch_rx, rv);

			/* Continue without DMA support */
			up->dma_ch_rx = -1;
		}
	}
#endif /* CONFIG_KINETIS_EDMA */

	/*
	 * Register the port
	 */
	rv = uart_add_one_port(&kinetis_uart_driver, port);
	if (rv) {
		dev_err(dev, "%s: uart_add_one_port failed (%d)\n",
			__func__, rv);
		goto err_dma_rx_put;
	}

	goto out;

err_dma_rx_put:
#if defined(CONFIG_KINETIS_EDMA)
	if (up->dma_ch_rx >= 0) {
		kinetis_dma_ch_free_irq(up->dma_ch_rx, up);
		kinetis_dma_ch_put(up->dma_ch_rx);
	}
#endif /* CONFIG_KINETIS_EDMA */

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

#if defined(CONFIG_KINETIS_EDMA)
	/* Release Rx DMA channel */
	if (up->dma_ch_rx >= 0) {
		kinetis_dma_ch_free_irq(up->dma_ch_rx, up);
		kinetis_dma_ch_put(up->dma_ch_rx);
	}
#endif /* CONFIG_KINETIS_EDMA */

	/* Release clock */
	clk_disable(up->clk);
	clk_put(up->clk);
#if defined(CONFIG_PM)
	if (!IS_ERR(up->suspend_clk)) {
		clk_disable(up->suspend_clk);
		clk_put(up->suspend_clk);
	}
#endif

	/* Reset pointers */
	dev_set_drvdata(dev, NULL);
	up->regs = NULL;

out:
	return rv;
}

#if defined(CONFIG_PM)

static int kinetis_uart_suspend(
	struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int kinetis_uart_resume(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct uart_port *port = dev_get_drvdata(dev);
	struct kinetis_uart_priv *up = kinetis_up(port);
	volatile struct kinetis_uart_regs *regs = up->regs;

	/*
	 * Acknowldge and disable Rx Edge interrupts
	 */
	regs->s2 |= KINETIS_UART_S2_RXEDGIF;
	regs->bdh &= ~KINETIS_UART_BDH_RXEDGIE;

	return 0;
}

#endif

static struct platform_driver kinetis_platform_driver = {
	.probe		= kinetis_uart_probe,
	.remove		= __devexit_p(kinetis_uart_remove),
#if defined(CONFIG_PM)
	.suspend	= kinetis_uart_suspend,
	.resume		= kinetis_uart_resume,
#endif
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
