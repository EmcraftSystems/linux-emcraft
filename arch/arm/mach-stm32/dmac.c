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

#include <mach/dmac.h>
#include <mach/dmaregs.h>


/*
 * IRQ flag masks for registers DMA_LISR, DMA_HISR, DMA_LIFCR, DMA_HIFCR
 * in the DMA controller register map. These flags can be passed in the function
 * stm32_dma_clear_irq_flags() defined below.
 */
/* Transfer complete */
#define STM32_DMA_IRQF_TC	(1 << 5)
/* Half transfer */
#define STM32_DMA_IRQF_HT	(1 << 4)
/* Transfer error */
#define STM32_DMA_IRQF_TE	(1 << 3)
/* Direct mode error */
#define STM32_DMA_IRQF_DME	(1 << 2)
/* FIFO error */
#define STM32_DMA_IRQF_FE	(1 << 0)

/*
 * DMA channel FIFO control register (DMA_SxFCR)
 */
/* Direct mode disable */
#define STM32_DMA_FCR_DMDIS_BIT		2
#define STM32_DMA_FCR_DMDIS_MSK		(1 << STM32_DMA_FCR_DMDIS_BIT)
/* FIFO threshold selection */
#define STM32_DMA_FCR_FTH_BITS		0
#define STM32_DMA_FCR_FTH_MSK		(3 << STM32_DMA_FCR_FTH_BITS)

/*
 * Per-channel data structure used internally by this DMA controller driver
 */
struct dma_ch_priv {
	/* Taken in IRQ handler and in stm32_dma_ch_request_irq() */
	spinlock_t irq_lock;
	/* Taken in functions of this DMA driver */
	spinlock_t lock;
	/* If stm32_dma_ch_get() was called */
	int in_use;

	/* DMA channel IRQ handler. Guarded with "irq_lock". */
	void (*irq_handler)(int ch, unsigned long flags, void *data);
	/* Driver-specific data passed to "irq_handler" as "data" argument */
	void *irq_data;
};

static struct dma_ch_priv dma_ch[STM32F2_DMA_CH_NUM];

/*
 * Returns pointer to DMA channel registers by channel number
 * (0..7 for DMA1 controller, 8..15 for DMA2 controller).
 */
static inline volatile struct stm32_dma_ch_regs *dma_ch_regs(int ch)
{
	volatile struct stm32_dma_regs *dma_regs =
		ch < STM32F2_DMA_CH_NUM_DMA1 ? STM32_DMA1 : STM32_DMA2;
	return &dma_regs->s[ch % STM32F2_DMA_CH_NUM_DMA1];
}

/*
 * Clear interrupt flags for a DMA channel
 */
static void stm32_dma_clear_irq_flags(int ch, u32 flags)
{
	volatile struct stm32_dma_regs *dma_regs =
		ch < STM32F2_DMA_CH_NUM_DMA1 ? STM32_DMA1 : STM32_DMA2;
	volatile u32 *dma_ifcr =
		(ch & 4) ? &dma_regs->hifcr : &dma_regs->lifcr;

	/*
	 * Write "flags" to the DMA_xIFCR register corresponding to the selected
	 * DMA channel at the correct bit offset inside that register.
	 *
	 * If (ch % 4) is 2 or 3, left shift the mask by 16 bits.
	 * If (ch % 4) is 1 or 3, additionally left shift the mask by 6 bits.
	 */
	*dma_ifcr = flags << (((ch & 2) << 3) | ((ch & 1) * 6));
}

/*
 * Handle IRQ for a particular DMA channel (0..15). Each DMA channel has
 * its own IRQ.
 */
static irqreturn_t stm32_dma_isr(int irq, void *dev_id)
{
	/* "ch" was previously saved here when calling request_irq() */
	int ch = (int)dev_id;

	unsigned long fl;

	/*
	 * "irq_handler" must be accessed with "irq_lock" taken,
	 * otherwise there will be a race condition between this function and
	 * stm32_dma_ch_request_irq().
	 *
	 * Therefore it is not allowed to call stm32_dma_ch_request_irq()
	 * from a DMA channel IRQ handler registered in a previous call
	 * to stm32_dma_ch_request_irq().
	 */
	spin_lock_irqsave(&dma_ch[ch].irq_lock, fl);

	if (dma_ch[ch].irq_handler) {
		dma_ch[ch].irq_handler(ch, 0, dma_ch[ch].irq_data);
	} else {
		/* IRQ for an unregistered DMA channel */
		pr_warning("Spurious IRQ for DMA channel %d\n", ch);
	}

	spin_unlock_irqrestore(&dma_ch[ch].irq_lock, fl);

	/* Clear all possible DMA channel IRQ flags */
	stm32_dma_clear_irq_flags(ch,
		STM32_DMA_IRQF_TC | STM32_DMA_IRQF_HT | STM32_DMA_IRQF_TE |
		STM32_DMA_IRQF_DME | STM32_DMA_IRQF_FE);

	return IRQ_HANDLED;
}

/*
 * Returns if the channel number is in the valid range (0..15).
 */
static int stm32_dma_ch_valid(int ch)
{
	return ch >= 0 && ch < STM32F2_DMA_CH_NUM;
}

/*
 * Acquire a DMA channel for exclusive access, similar to clk_get().
 */
int stm32_dma_ch_get(int ch)
{
	unsigned long fl;
	int rv;

	if (!stm32_dma_ch_valid(ch)) {
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
EXPORT_SYMBOL(stm32_dma_ch_get);

/*
 * Release a DMA channel from exclusive access, similar to clk_put().
 */
int stm32_dma_ch_put(int ch)
{
	unsigned long fl;
	int rv;

	if (!stm32_dma_ch_valid(ch)) {
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
EXPORT_SYMBOL(stm32_dma_ch_put);

/*
 * Code blocks for wrapping DMA controller driver API functions
 * in spin_lock_irqsave/spin_unlock_irqrestore pairs and performing
 * other simple checks.
 *
 * Deny operation if channel number is out of range (0..15).
 * Deny operation if channel was not acquired (!in_use).
 */
/* Function preamble */
#define DMAAPI_LOCKED_BEGIN \
	unsigned long fl;					\
	int rv;							\
								\
	if (!stm32_dma_ch_valid(ch)) {				\
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
 * Enable or disable a DMA channel (by index ch: 0..15) by setting or clearing
 * a DMA_SxCR[EN] bit.
 */
static int __stm32_dma_ch_enable(int ch, int enable)
{
	volatile struct stm32_dma_ch_regs *ch_regs;
	DMAAPI_LOCKED_BEGIN

	/* Enable or disable channel */
	ch_regs = dma_ch_regs(ch);

	if (enable) {
		/*
		 * All channel IRQ flags must be cleared before enabling
		 * the channel.
		 */
		stm32_dma_clear_irq_flags(ch,
			STM32_DMA_IRQF_TC | STM32_DMA_IRQF_HT |
			STM32_DMA_IRQF_TE | STM32_DMA_IRQF_DME |
			STM32_DMA_IRQF_FE);

		ch_regs->cr |= STM32_DMA_CR_EN;
	} else {
		ch_regs->cr &= ~STM32_DMA_CR_EN;
	}

	DMAAPI_LOCKED_END
}

/*
 * Enable or disable a DMA channel (by index ch: 0..15) by setting
 * a DMA_SxCR[EN] bit.
 */
int stm32_dma_ch_enable(int ch)
{
	return __stm32_dma_ch_enable(ch, 1);
}
EXPORT_SYMBOL(stm32_dma_ch_enable);

/*
 * Enable or disable a DMA channel (by index ch: 0..15) by clearing
 * a DMA_SxCR[EN] bit.
 */
int stm32_dma_ch_disable(int ch)
{
	return __stm32_dma_ch_enable(ch, 0);
}
EXPORT_SYMBOL(stm32_dma_ch_disable);

/*
 * DMA channel to IRQ mapping. Every DMA channel ("stream" in terms of STM32F2)
 * has a dedicated IRQ.
 */
static const int stm32f2_dma_irq[STM32F2_DMA_CH_NUM] = {
	/* DMA1 controller channels */
	11, 12, 13, 14, 15, 16, 17, 47,
	/* DMA2 controller channels */
	56, 57, 58, 59, 60, 68, 69, 70,
};

/*
 * Register an interrupt handler for a channel. The data argument will be
 * passed to the handler. In flags, one can pass STM32_DMA_INTCOMPLETE
 * and/or STM32_DMA_INTHALF to request interrupts on DMA buffer completion or
 * half-completion.
 *
 * The DMA driver code will clear the interrupt request flag
 * for the corresponding DMA channel.
 */
int stm32_dma_ch_request_irq(
	int ch, void (*handler)(int ch, unsigned long flags, void *data),
	unsigned long flags, void *data)
{
	unsigned long fl;
	int rv;
	volatile struct stm32_dma_ch_regs *ch_regs;

	/* Deny operation if channel number is out of range */
	if (!stm32_dma_ch_valid(ch)) {
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
	 * Setup a real IRQ handler implemented by this DMA controller driver
	 */
	rv = request_irq(stm32f2_dma_irq[ch], stm32_dma_isr, IRQF_DISABLED,
		"STM32 DMA Controller", (void *)ch);
	if (rv) {
		pr_err("%s: request_irq(%d) failed (%d)\n",
			__func__, stm32f2_dma_irq[ch], rv);
		goto err_irq;
	}

	ch_regs = dma_ch_regs(ch);

	/* Enable or disable DMA buffer completion interrupt */
	if (flags & STM32_DMA_INTCOMPLETE)
		ch_regs->cr |= STM32_DMA_CR_TCIE;
	else
		ch_regs->cr &= ~STM32_DMA_CR_TCIE;

	/* Enable or disable DMA buffer half-completion interrupt */
	if (flags & STM32_DMA_INTHALF)
		ch_regs->cr |= STM32_DMA_CR_HTIE;
	else
		ch_regs->cr &= ~STM32_DMA_CR_HTIE;

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
EXPORT_SYMBOL(stm32_dma_ch_request_irq);

/*
 * Unregister the interrupt handler for a channel.
 */
int stm32_dma_ch_free_irq(int ch, void *data)
{
	unsigned long fl;
	int rv;

	/* Deny operation if channel number is out of range */
	if (!stm32_dma_ch_valid(ch)) {
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
	dma_ch_regs(ch)->cr &= ~(STM32_DMA_CR_TCIE | STM32_DMA_CR_HTIE);

	/*
	 * Always call free_irq(), because a DMA channel is the only user
	 * of its IRQ slot, i.e. no multiplexing of DMA IRQs is done by the CPU.
	 */
	free_irq(stm32f2_dma_irq[ch], (void *)ch);

	rv = 0;
	goto unlock;

unlock:
	spin_unlock(&dma_ch[ch].lock);
	spin_unlock_irqrestore(&dma_ch[ch].irq_lock, fl);
out:
	return rv;

}
EXPORT_SYMBOL(stm32_dma_ch_free_irq);

/*
 * Clear all DMA controller registers related to the selected DMA channel
 * and set a few static channel parameters.
 *
 * @dir = 0..2. Direction: peripheral-to-memory, memory-to-peripheral
 * or memory-to-memory.
 *
 * @fpctrl = 1 or 0. If peripheral is the flow controller.
 *
 * @pl = 0..3. Sets channel priority (3 is highest, 0 is lowest priority).
 *
 * @dbm = 1 or 0. Enables or disables the Double Buffer Mode.
 *
 * @circ = 1 or 0. Enables or disables the Circular Mode.
 *
 *
 * This function must be called before stm32_dma_ch_init_fifo()
 * and stm32_dma_ch_set_*().
 */
int stm32_dma_ch_init(int ch, u8 dir, u8 fpctrl, u8 pl, u8 dbm, u8 circ)
{
	volatile struct stm32_dma_ch_regs *ch_regs;
	DMAAPI_LOCKED_BEGIN

	/* Check function arguments */
	if (dir > 2 || fpctrl > 1 || pl > 3 || dbm > 1 || circ > 1) {
		rv = -EINVAL;
		goto unlock;
	}

	ch_regs = dma_ch_regs(ch);

	/* Cannot change parameters of an enabled DMA channel */
	if (ch_regs->cr & STM32_DMA_CR_EN) {
		rv = -EBUSY;
		goto unlock;
	}

	ch_regs->ndtr = 0;
	ch_regs->par = 0;
	ch_regs->m0ar = 0;
	ch_regs->m1ar = 0;
	ch_regs->fcr = 0;
	/*
	 * Do not clear CR[TCIE] and CR[HTIE] flags, because someone might call
	 * stm32_dma_ch_request_irq() before stm32_dma_init().
	 *
	 * Always disable CR[PINCOS], we currently do not support it.
	 */
	ch_regs->cr =
		(ch_regs->cr & ~(STM32_DMA_CR_DIR_MSK |
			STM32_DMA_CR_PFCTRL_MSK | STM32_DMA_CR_PL_MSK |
			STM32_DMA_CR_DBM | STM32_DMA_CR_CIRC |
			STM32_DMA_CR_PINC | STM32_DMA_CR_MINC |
			STM32_DMA_CR_PSIZE_MSK | STM32_DMA_CR_MSIZE_MSK |
			STM32_DMA_CR_PINCOS_MSK | STM32_DMA_CR_CT |
			STM32_DMA_CR_PBURST_MSK | STM32_DMA_CR_MBURST_MSK)) |
		((u32)dir << STM32_DMA_CR_DIR_BITS) |
		((u32)fpctrl << STM32_DMA_CR_PFCTRL_BIT) |
		((u32)pl << STM32_DMA_CR_PL_BIT) |
		((u32)dbm << STM32_DMA_CR_DBM_BIT) |
		((u32)circ << STM32_DMA_CR_CIRC_BIT);

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(stm32_dma_ch_init);

/*
 * Initialize DMA channel FIFO parameters in the respective DMA_SxFCR register
 * (FIFO control register).
 *
 * @burst = 1 or 0. Enable or disable burst mode. When set to 0, the Direct Mode
 * will be used.
 *
 * @threshold = 0..3. FIFO threshold:
 *    0 means 1/4 full FIFO;
 *    1 means 1/2 full FIFO;
 *    2 means 3/4 full FIFO;
 *    3 means full FIFO.
 */
int stm32_dma_ch_init_fifo(int ch, u8 burst, u8 threshold)
{
	volatile struct stm32_dma_ch_regs *ch_regs;
	DMAAPI_LOCKED_BEGIN

	/* Check function arguments */
	if (burst > 1 || threshold > 3) {
		rv = -EINVAL;
		goto unlock;
	}

	ch_regs = dma_ch_regs(ch);

	/* Cannot change parameters of an enabled DMA channel */
	if (ch_regs->cr & STM32_DMA_CR_EN) {
		rv = -EBUSY;
		goto unlock;
	}

	ch_regs->fcr =
		(ch_regs->fcr & ~(STM32_DMA_FCR_DMDIS_MSK |
			STM32_DMA_FCR_FTH_MSK)) |
		((u32)burst << STM32_DMA_FCR_DMDIS_BIT) |
		((u32)threshold << STM32_DMA_FCR_FTH_BITS);

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(stm32_dma_ch_init_fifo);

/*
 * Set fields related to the peripheral in the channel configuration
 * register (DMA_SxCR) and the DMA_SxPAR register.
 *
 * @addr (DMA_SxPAR). Peripheral address.
 *
 * @inc = 1 or 0. Enable or disable peripheral address increment (DMA_SxCR[PINC]).
 *
 * @bitwidth = 0..2. Peripheral data size (DMA_SxCR[PSIZE]): byte (8-bit),
 * half-word (16-bit) or word (32-bit).
 *
 * @burst = 0..3. Peripheral burst transfer configuration (DMA_SxCR[PBURST]).
 */
int stm32_dma_ch_set_periph(int ch, u32 addr, u8 inc, u8 bitwidth, u8 burst)
{
	volatile struct stm32_dma_ch_regs *ch_regs;
	DMAAPI_LOCKED_BEGIN

	/* Check function arguments */
	if (inc > 1 || bitwidth > 3 || burst > 3) {
		rv = -EINVAL;
		goto unlock;
	}

	ch_regs = dma_ch_regs(ch);

	/* Cannot change parameters of an enabled DMA channel */
	if (ch_regs->cr & STM32_DMA_CR_EN) {
		rv = -EBUSY;
		goto unlock;
	}

	ch_regs->par = (volatile void *)addr;
	ch_regs->cr =
		(ch_regs->cr & ~(STM32_DMA_CR_PINC | STM32_DMA_CR_PSIZE_MSK |
			STM32_DMA_CR_PBURST_MSK)) |
		((u32)inc << STM32_DMA_CR_PINC_BIT) |
		((u32)bitwidth << STM32_DMA_CR_PSIZE_BITS) |
		((u32)burst << STM32_DMA_CR_PBURST_BITS);

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(stm32_dma_ch_set_periph);

/*
 * Set fields related to memory in the channel configuration register (DMA_SxCR)
 * and the DMA_SxM0AR register. The DMA_SxM1AR register will not be written to.
 *
 * @addr (DMA_SxM0AR). Memory address.
 *
 * @inc = 1 or 0. Enable or disable memory address increment (DMA_SxCR[MINC]).
 *
 * @bitwidth = 0..2. Memory data size (DMA_SxCR[MSIZE]): byte (8-bit),
 * half-word (16-bit) or word (32-bit).
 *
 * @burst = 0..3. Memory burst transfer configuration (DMA_SxCR[MBURST]).
 */
int stm32_dma_ch_set_memory(int ch, u32 addr, u8 inc, u8 bitwidth, u8 burst)
{
	volatile struct stm32_dma_ch_regs *ch_regs;
	DMAAPI_LOCKED_BEGIN

	/* Check function arguments */
	if (inc > 1 || bitwidth > 3 || burst > 3) {
		rv = -EINVAL;
		goto unlock;
	}

	ch_regs = dma_ch_regs(ch);

	/* Cannot change parameters of an enabled DMA channel */
	if (ch_regs->cr & STM32_DMA_CR_EN) {
		rv = -EBUSY;
		goto unlock;
	}

	ch_regs->m0ar = (volatile void *)addr;
	ch_regs->cr =
		(ch_regs->cr & ~(STM32_DMA_CR_MINC | STM32_DMA_CR_MSIZE_MSK |
			STM32_DMA_CR_MBURST_MSK)) |
		((u32)inc << STM32_DMA_CR_MINC_BIT) |
		((u32)bitwidth << STM32_DMA_CR_MSIZE_BITS) |
		((u32)burst << STM32_DMA_CR_MBURST_BITS);

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(stm32_dma_ch_set_memory);

/*
 * Set the DMA_SxNDTR register to @nitems.
 */
int stm32_dma_ch_set_nitems(int ch, u16 nitems)
{
	volatile struct stm32_dma_ch_regs *ch_regs;
	DMAAPI_LOCKED_BEGIN

	ch_regs = dma_ch_regs(ch);

	/* Cannot change parameters of an enabled DMA channel */
	if (ch_regs->cr & STM32_DMA_CR_EN) {
		rv = -EBUSY;
		goto unlock;
	}

	ch_regs->ndtr = (u32)nitems;

	DMAAPI_LOCKED_END
}
EXPORT_SYMBOL(stm32_dma_ch_set_nitems);

/*
 * Initialize the DMA controller driver
 */
void __init stm32_dmac_init(void)
{
	int ch;

	for (ch = 0; ch < ARRAY_SIZE(dma_ch); ch++) {
		spin_lock_init(&dma_ch[ch].lock);
		spin_lock_init(&dma_ch[ch].irq_lock);
	}
}
