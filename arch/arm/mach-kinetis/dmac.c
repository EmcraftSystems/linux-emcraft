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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#include <mach/kinetis.h>
#include <mach/dmac.h>


/*
 * Start index of IRQs for DMA channels
 */
#define KINETIS_DMACH_IRQ_START		0

struct dma_ch_priv {
	/* Taken in IRQ handler and in kinetis_dma_ch_request_irq() */
	spinlock_t irq_lock;
	/* Taken in functions of this DMA driver */
	spinlock_t lock;
	/* If kinetis_dma_ch_get() was called */
	int in_use;

	/* DMA channel IRQ handler. Guarded with "irq_lock". */
	void (*irq_handler)(int ch, unsigned long flags, void *data);
	/* Driver-specific data passed to "irq_handler" as "data" argument */
	void *irq_data;
};

static struct dma_ch_priv dma_ch[KINETIS_DMA_CH_NUM];

/*
 * TCD (DMA transfer control descriptor)
 */
struct kinetis_dma_tcd {
	u32 saddr;	/* Source Address */
	s16 soff;	/* Signed Source Address Offset */
	u16 attr;	/* Transfer Attributes */
	u32 nbytes;	/* Byte Count */
	u32 slast;	/* Last Source Address Adjustment */
	u32 daddr;	/* Destination Address */
	s16 doff;	/* Signed Destination Address Offset */
	u16 citer;	/* Current Major Loop Count */
	u32 dlast_sga;	/* Last Destination Address Adjustment; S/G Address */
	u16 csr;	/* Control and Status */
	u16 biter;	/* Beginning Major Loop Count */
};

struct kinetis_dma_regs {
	u32 cr;		/* Control Register */
	u32 es;		/* Error Status Register */
	u32 rsv0;
	u32 erq;	/* Enable DMA Request */
	u32 rsv1;
	u32 eei;	/* Enable Error Interrupt */
	u8 ceei;	/* Clear Enable Error Interrupt */
	u8 seei;	/* Set Enable Error Interrupt */
	u8 cerq;	/* Clear Enable Request */
	u8 serq;	/* Set Enable Request */
	u8 cdne;	/* Clear DONE Status Bit */
	u8 ssrt;	/* Set START Bit */
	u8 cerr;	/* Clear Error */
	u8 cint;	/* Clear Interrupt Request */
	u32 rsv2;
	u32 intr;	/* Interrupt Request */
	u32 rsv3;
	u32 err;	/* Error Register */
	u32 rsv4;
	u32 hrs;	/* Hardware Request Status */
	u32 rsv5[50];
	u8 dchpri[32];	/* Channel n Priority */
	u32 rsv6[952];
	struct kinetis_dma_tcd tcd[32];		/* TCDs */
};

/*
 * eDMA register map base
 */
#define KINETIS_DMA_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00008000)
#define KINETIS_DMA		((volatile struct kinetis_dma_regs *) \
				KINETIS_DMA_BASE)

/*
 * DMA register fields
 */
/*
 * Clear Interrupt Request Register (DMA_CINT)
 */
#define KINETIS_DMA_CINT_CINT_BITS	0
/*
 * TCD Transfer Attributes (TCD:ATTR)
 */
/* Source data transfer size */
#define KINETIS_TCD_ATTR_SSIZE_BITS	8
#define KINETIS_TCD_ATTR_SSIZE_MSK	(7 << KINETIS_TCD_ATTR_SSIZE_BITS)
/* Destination data transfer size */
#define KINETIS_TCD_ATTR_DSIZE_BITS	0
#define KINETIS_TCD_ATTR_DSIZE_MSK	(7 << KINETIS_TCD_ATTR_DSIZE_BITS)
/*
 * TCD Control and Status (TCD:CSR)
 */
/* Enable an interrupt when major counter is half complete */
#define KINETIS_TCD_CSR_INTHALF_MSK	(1 << 2)
/* Enable an interrupt when major iteration count completes */
#define KINETIS_TCD_CSR_INTMAJOR_MSK	(1 << 1)

/*
 * Handle IRQ for a particular DMA channel (0..31)
 */
static void kinetis_dma_channel_isr(int ch)
{
	unsigned long fl;

	/*
	 * "irq_handler" must be accessed with "irq_lock" taken,
	 * otherwise there will be a race condition between this function and
	 * kinetis_dma_ch_request_irq().
	 *
	 * Therefore it is not allowed to call kinetis_dma_ch_request_irq()
	 * from a DMA channel IRQ handler registered in a previous call
	 * to kinetis_dma_ch_request_irq().
	 */
	spin_lock_irqsave(&dma_ch[ch].irq_lock, fl);

	if (dma_ch[ch].irq_handler) {
		dma_ch[ch].irq_handler(ch, 0, dma_ch[ch].irq_data);
	} else {
		/* IRQ for an unregistered DMA channel */
		pr_warning("Spurious IRQ for DMA channel %d\n", ch);
	}

	spin_unlock_irqrestore(&dma_ch[ch].irq_lock, fl);

	/* Clear interrupt request for DMA channel */
	KINETIS_DMA->cint = ch << KINETIS_DMA_CINT_CINT_BITS;
}

/*
 * Hardware IRQ handler (for IRQ0..IRQ15) for interrupt requests from pairs
 * of DMA channels:
 *    IRQ0  - channels 0, 16;
 *    IRQ1  - channels 1, 17;
 *    ...
 *    IRQ15 - channels 15, 31.
 */
static irqreturn_t kinetis_dma_isr(int irq, void *dev_id)
{
	/* (ch % 16) was previously saved here when calling request_irq() */
	int ch = (int)dev_id;

	/*
	 * Demultiplex (ch) and (ch + 16), i.e. add 16 if needed to get
	 * the real channel number.
	 */
	if (KINETIS_DMA->intr & (1 << ch))
		kinetis_dma_channel_isr(ch);

	ch += 16;
	if (KINETIS_DMA->intr & (1 << ch))
		kinetis_dma_channel_isr(ch);

	return IRQ_HANDLED;
}

/*
 * Returns if the channel number is in the valid range (0..31).
 */
static int kinetis_dma_ch_valid(int ch)
{
	return ch >= 0 && ch < KINETIS_DMA_CH_NUM;
}

/*
 * Acquire a DMA channel for exclusive access, similar to clk_get().
 */
int kinetis_dma_ch_get(int ch)
{
	unsigned long fl;
	int rv;

	if (!kinetis_dma_ch_valid(ch)) {
		rv = -EINVAL;
		goto out;
	}

	spin_lock_irqsave(&dma_ch[ch].lock, fl);

	/* Return EBUSY if channel already in use */
	rv = dma_ch[ch].in_use ? -EBUSY : 0;
	/* Mark channel as being in use */
	dma_ch[ch].in_use = 1;

	spin_unlock_irqrestore(&dma_ch[ch].lock, fl);

out:
	return rv;
}
EXPORT_SYMBOL(kinetis_dma_ch_get);

/*
 * Release a DMA channel from exclusive access, similar to clk_put().
 */
int kinetis_dma_ch_put(int ch)
{
	unsigned long fl;
	int rv;

	if (!kinetis_dma_ch_valid(ch)) {
		rv = -EINVAL;
		goto out;
	}

	spin_lock_irqsave(&dma_ch[ch].lock, fl);

	/* Return EBUSY if channel was not in use */
	rv = dma_ch[ch].in_use ? 0 : -EBUSY;
	/* Mark channel as being not in use */
	dma_ch[ch].in_use = 0;

	spin_unlock_irqrestore(&dma_ch[ch].lock, fl);

out:
	return rv;
}
EXPORT_SYMBOL(kinetis_dma_ch_put);

/*
 * Code blocks for wrapping DMA controller driver API functions
 * in spin_lock_irqsave/spin_unlock_irqrestore pairs and performing
 * other simple checks.
 *
 * Deny operation if channel number is out of range (0..31).
 * Deny operation if channel was not acquired (!in_use).
 */
/* Function preamble */
#define DMAAPI_LOCKED_BEGIN \
	unsigned long fl;					\
	int rv;							\
								\
	if (!kinetis_dma_ch_valid(ch)) {			\
		rv = -EINVAL;					\
		goto out;					\
	}							\
								\
	spin_lock_irqsave(&dma_ch[ch].lock, fl);		\
								\
	if (!dma_ch[ch].in_use) {				\
		rv = -ENODEV;					\
		goto unlock;					\
	}
/* Function postscript */
#define DMAAPI_LOCKED_END \
	rv = 0;							\
unlock:								\
	spin_unlock_irqrestore(&dma_ch[ch].lock, fl);		\
out:								\
	return rv;

/*
 * Enable or disable a DMA channel (by index ch: 0 .. 31) by setting a bit
 * in DMA_ERQ.
 */
static int __kinetis_dma_ch_enable(int ch, int enable, int single)
{
	DMAAPI_LOCKED_BEGIN

	if (single)
		KINETIS_DMA->tcd[ch].csr |= (1 << 3);

	/* Enable or disable channel */
	if (enable)
		KINETIS_DMA->erq |= 1 << ch;
	else
		KINETIS_DMA->erq &= ~(1 << ch);

	DMAAPI_LOCKED_END
}

/*
 * Enable a DMA channel (by index ch: 0 .. 31) by setting a bit in DMA_ERQ.
 */
int kinetis_dma_ch_enable(int ch, int single)
{
	return __kinetis_dma_ch_enable(ch, 1, single);
}
EXPORT_SYMBOL(kinetis_dma_ch_enable);

/*
 * Disable a DMA channel (by index ch: 0 .. 31) by clearing a bit in DMA_ERQ.
 */
int kinetis_dma_ch_disable(int ch)
{
	return __kinetis_dma_ch_enable(ch, 0, 0);
}
EXPORT_SYMBOL(kinetis_dma_ch_disable);

/*
 * Register an interrupt handler for a channel. The data argument will be
 * passed to the handler. In flags, one can pass KINETIS_DMA_INTMAJOR
 * and/or KINETIS_DMA_INTHALF to request interrupts on DMA buffer
 * completion (CITER == 0) or half-completion (CITER == BITER/2).
 *
 * The DMA driver code will clear the interrupt request flag
 * for the corresponding DMA channel.
 */
int kinetis_dma_ch_request_irq(
	int ch, void (*handler)(int ch, unsigned long flags, void *data),
	unsigned long flags, void *data)
{
	unsigned long fl;
	int rv;

	/* Deny operation if channel number is out of range */
	if (!kinetis_dma_ch_valid(ch)) {
		rv = -EINVAL;
		goto out;
	}

	spin_lock_irqsave(&dma_ch[ch].irq_lock, fl);
	spin_lock(&dma_ch[ch].lock);

	/* Deny operation if channel was not acquired */
	if (!dma_ch[ch].in_use) {
		rv = -ENODEV;
		goto unlock;
	}

	/* Deny operation if an IRQ handler is already installed */
	if (dma_ch[ch].irq_handler) {
		rv = -EEXIST;
		goto unlock;
	}

	/* Install new IRQ handler */
	dma_ch[ch].irq_handler = handler;
	dma_ch[ch].irq_data = data;

	/*
	 * Do not call request_irq() again if an IRQ handler is already
	 * installed for the paired channel (ch XOR 16).
	 */
	if (!dma_ch[ch ^ 16].irq_handler) {
		rv = request_irq(KINETIS_DMACH_IRQ_START + ch % 16,
				kinetis_dma_isr, IRQF_DISABLED,
				"Kinetis eDMA Controller", (void *)(ch % 16));
		if (rv) {
			pr_err("%s: request_irq(%d) failed (%d)\n",
				__func__, KINETIS_DMACH_IRQ_START + ch % 16, rv);
			goto err_irq;
		}
	}

	/* Enable or disable DMA buffer completion interrupt */
	if (flags & KINETIS_DMA_INTMAJOR)
		KINETIS_DMA->tcd[ch].csr |= KINETIS_TCD_CSR_INTMAJOR_MSK;
	else
		KINETIS_DMA->tcd[ch].csr &= ~KINETIS_TCD_CSR_INTMAJOR_MSK;

	/* Enable or disable DMA buffer half-completion interrupt */
	if (flags & KINETIS_DMA_INTHALF)
		KINETIS_DMA->tcd[ch].csr |= KINETIS_TCD_CSR_INTHALF_MSK;
	else
		KINETIS_DMA->tcd[ch].csr &= ~KINETIS_TCD_CSR_INTHALF_MSK;

	rv = 0;
	goto unlock;

err_irq:
	dma_ch[ch].irq_handler = NULL;
	dma_ch[ch].irq_data = NULL;
unlock:
	spin_unlock(&dma_ch[ch].lock);
	spin_unlock_irqrestore(&dma_ch[ch].irq_lock, fl);
out:
	return rv;
}
EXPORT_SYMBOL(kinetis_dma_ch_request_irq);

/*
 * Unregister the interrupt handler for a channel.
 */
int kinetis_dma_ch_free_irq(int ch, void *data)
{
	unsigned long fl;
	int rv;

	/* Deny operation if channel number is out of range */
	if (!kinetis_dma_ch_valid(ch)) {
		rv = -EINVAL;
		goto out;
	}

	spin_lock_irqsave(&dma_ch[ch].irq_lock, fl);
	spin_lock(&dma_ch[ch].lock);

	/* Deny operation if channel was not acquired */
	if (!dma_ch[ch].in_use) {
		rv = -ENODEV;
		goto unlock;
	}

	/* Deny operation if the "data" variable does not match */
	if (dma_ch[ch].irq_data != data) {
		rv = -EINVAL;
		goto unlock;
	}

	/* Deny operation if the IRQ handler is not installed */
	if (!dma_ch[ch].irq_handler) {
		rv = -EINVAL;
		goto unlock;
	}

	/* Remove new IRQ handler */
	dma_ch[ch].irq_handler = NULL;
	dma_ch[ch].irq_data = NULL;

	/* Disable DMA buffer completion and half-completion interrupts */
	KINETIS_DMA->tcd[ch].csr &=
		~(KINETIS_TCD_CSR_INTMAJOR_MSK | KINETIS_TCD_CSR_INTHALF_MSK);

	/*
	 * Call free_irq() if no IRQ handler is installed
	 * for the paired channel (ch XOR 16).
	 */
	if (!dma_ch[ch ^ 16].irq_handler)
		free_irq(KINETIS_DMACH_IRQ_START + ch % 16, (void *)(ch % 16));

	rv = 0;
	goto unlock;

unlock:
	spin_unlock(&dma_ch[ch].lock);
	spin_unlock_irqrestore(&dma_ch[ch].irq_lock, fl);
out:
	return rv;

}
EXPORT_SYMBOL(kinetis_dma_ch_free_irq);

/*
 * Clear all registers in TCD (transfer control descriptor) for a channel.
 * This function must be called before kinetis_dma_ch_set_*().
 */
int kinetis_dma_ch_init(int ch)
{
	DMAAPI_LOCKED_BEGIN

	/*
	 * Reset most of the TCD fields to 0.
	 */
	KINETIS_DMA->tcd[ch].saddr     = 0;
	KINETIS_DMA->tcd[ch].soff      = 0;
	KINETIS_DMA->tcd[ch].attr      = 0;
	KINETIS_DMA->tcd[ch].nbytes    = 0;
	KINETIS_DMA->tcd[ch].slast     = 0;
	KINETIS_DMA->tcd[ch].daddr     = 0;
	KINETIS_DMA->tcd[ch].doff      = 0;
	KINETIS_DMA->tcd[ch].citer     = 0;
	KINETIS_DMA->tcd[ch].dlast_sga = 0;
	KINETIS_DMA->tcd[ch].biter     = 0;

	/*
	 * Do not clear previously set flags INTMAJOR and INTHALF.
	 *
	 * This is useful for example if someone wants to call
	 * kinetis_dma_ch_init() after kinetis_dma_ch_request_irq().
	 */
	KINETIS_DMA->tcd[ch].csr &=
		KINETIS_TCD_CSR_INTMAJOR_MSK | KINETIS_TCD_CSR_INTHALF_MSK;

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(kinetis_dma_ch_init);

/*
 * Set respective fields in the channel's TCD: SADDR, SOFF, ATTR[SSIZE], SLAST.
 * These fields configure the DMA channel source.
 */
int kinetis_dma_ch_set_src(int ch, u32 saddr, s16 soff, u8 bitwidth, s32 slast)
{
	DMAAPI_LOCKED_BEGIN

	KINETIS_DMA->tcd[ch].attr =
		(KINETIS_DMA->tcd[ch].attr & ~KINETIS_TCD_ATTR_SSIZE_MSK) |
		(bitwidth << KINETIS_TCD_ATTR_SSIZE_BITS);
	KINETIS_DMA->tcd[ch].saddr = saddr;
	KINETIS_DMA->tcd[ch].soff = soff;
	KINETIS_DMA->tcd[ch].slast = slast;

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(kinetis_dma_ch_set_src);

/*
 * Set respective fields in the channel's TCD: DADDR, DOFF, ATTR[DSIZE], DLAST.
 * These fields configure the DMA channel destination.
 */
int kinetis_dma_ch_set_dest(int ch, u32 daddr, s16 doff, u8 bitwidth, s32 dlast)
{
	DMAAPI_LOCKED_BEGIN

	KINETIS_DMA->tcd[ch].attr =
		(KINETIS_DMA->tcd[ch].attr & ~KINETIS_TCD_ATTR_DSIZE_MSK) |
		(bitwidth << KINETIS_TCD_ATTR_DSIZE_BITS);
	KINETIS_DMA->tcd[ch].daddr = daddr;
	KINETIS_DMA->tcd[ch].doff = doff;
	KINETIS_DMA->tcd[ch].dlast_sga = dlast;

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(kinetis_dma_ch_set_dest);

/*
 * Set NBYTES in the DMA channel's TCD to nbytes.
 */
int kinetis_dma_ch_set_nbytes(int ch, u32 nbytes)
{
	DMAAPI_LOCKED_BEGIN

	KINETIS_DMA->tcd[ch].nbytes = nbytes;

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(kinetis_dma_ch_set_nbytes);

/*
 * Set CITER and BITER in the DMA channel's TCD to iter.
 */
int kinetis_dma_ch_set_iter_num(int ch, u16 iter)
{
	DMAAPI_LOCKED_BEGIN

	KINETIS_DMA->tcd[ch].citer = iter;
	KINETIS_DMA->tcd[ch].biter = iter;

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(kinetis_dma_ch_set_iter_num);

/*
 * Returns the number of major loops finished, i.e. TCD_BITER - TCD_CITER,
 * where BITER is the number of major loops (iterations) requested
 * and CITER is the number of major loops left to perform.
 */
int kinetis_dma_ch_iter_done(int ch)
{
	return KINETIS_DMA->tcd[ch].biter - KINETIS_DMA->tcd[ch].citer;
}
EXPORT_SYMBOL(kinetis_dma_ch_iter_done);

/*
 * Get DMA channel busy status
 */
int kinetis_dma_ch_is_active(int ch)
{
	return KINETIS_DMA->hrs & (1 << ch);
}
EXPORT_SYMBOL(kinetis_dma_ch_is_active);

/*
 * Initialize the DMA controller driver
 */
void __init kinetis_dmac_init(void)
{
	int ch;

	for (ch = 0; ch < ARRAY_SIZE(dma_ch); ch++) {
		spin_lock_init(&dma_ch[ch].lock);
		spin_lock_init(&dma_ch[ch].irq_lock);

		/*
		 * Clear INTMAJOR and INTHALF flags to disable the respective
		 * interrupts from DMA channels, kinetis_dma_ch_init()
		 * will not clear them.
		 */
		KINETIS_DMA->tcd[ch].csr = 0;
	}
}
