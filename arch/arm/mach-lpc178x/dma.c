/*
 * arch/arm/mach-lpc178x/dma.c
 *
 *  (was named linux/arch/arm/mach-lpc32xx/ma-lpc32xx.c)
 *  Copyright (C) 2008 NXP Semiconductors
 *  (Based on parts of the PNX4008 DMA driver)
 *
 * Customized for LPC178x/7x by:
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <asm/dma-mapping.h>
#include <asm/io.h>
#include <mach/dma.h>
#include <mach/dmac.h>

#define DMAIOBASE io_p2v(LPC178X_DMA_BASE)
#define VALID_CHANNEL(c) (((c) >= 0) && ((c) < MAX_DMA_CHANNELS))

/*
 * Bit fields of the DMA Request Select register
 */
#define LPC178X_SCC_DMACREQSEL_SEL6_I2S0_MSK	(1 << 6)
#define LPC178X_SCC_DMACREQSEL_SEL7_I2S1_MSK	(1 << 7)

static DEFINE_SPINLOCK(dma_lock);

struct dma_linked_list {
	u32 src;
	u32 dest;
	u32 next_lli;
	u32 ctrl;
};

/* For DMA linked list operation, a linked list of DMA descriptors
   is maintained along with some data to manage the list in software. */
struct dma_list_ctrl {
	/* DMA list descriptor */
	struct dma_linked_list dmall;
	/* Virtual address to next list entry */
	struct dma_list_ctrl *next_list_addr;
	/* Virtual address to previous list entry */
        struct dma_list_ctrl *prev_list_addr;
	/* Physical address to next list entry */
	u32 next_list_phy;
	/* Physical address to previous list entry */
        u32 prev_list_phy;
};

/* Each DMA channel has one of these structures */
struct dma_channel {
	char *name;
	void (*irq_handler) (int, int, void *);
	void *data;
	struct dma_config *dmacfg;
	u32 control;
	u32 config;
	u32 config_int_mask;

	int list_entries; /* Number of list entries */
	u32 list_size; /* Total size of allocated list in bytes */
	u32 list_vstart; /* Allocated (virtual) address of list */
	u32 list_pstart; /* Allocated (physical) address of list */
	int free_entries; /* Number of free descriptors */
	struct dma_list_ctrl *list_head, *list_tail, *list_curr;
};

struct dma_control {
	struct clk *clk;
	int num_clks;
	struct dma_channel dma_channels[MAX_DMA_CHANNELS];
};
static struct dma_control dma_ctrl;
static unsigned long flags;

static inline void __dma_regs_lock(void)
{
	spin_lock_irqsave(&dma_lock, flags);
}

static inline void __dma_regs_unlock(void)
{
	spin_unlock_irqrestore(&dma_lock, flags);
}

static inline void __dma_enable(int ch) {
	u32 ch_cfg = __raw_readl(DMACH_CONFIG_CH(DMAIOBASE, ch));
	ch_cfg |= DMAC_CHAN_ENABLE;
	__raw_writel(ch_cfg, DMACH_CONFIG_CH(DMAIOBASE, ch));
}

static inline void __dma_disable(int ch) {
	u32 ch_cfg = __raw_readl(DMACH_CONFIG_CH(DMAIOBASE, ch));
	ch_cfg &= ~DMAC_CHAN_ENABLE;
	__raw_writel(ch_cfg, DMACH_CONFIG_CH(DMAIOBASE, ch));
}

static void dma_clocks_up(void)
{
	/* Enable DMA clock if needed */
	if (dma_ctrl.num_clks == 0)
	{
		clk_enable(dma_ctrl.clk);
		__raw_writel(DMAC_CTRL_ENABLE, DMA_CONFIG(DMAIOBASE));
	}

	dma_ctrl.num_clks++;
}

static void dma_clocks_down(void)
{
	dma_ctrl.num_clks--;

	/* Disable DMA clock if needed */
	if (dma_ctrl.num_clks == 0)
	{
		__raw_writel(0, DMA_CONFIG(DMAIOBASE));
		clk_disable(dma_ctrl.clk);
	}
}

static int lpc178x_ch_setup(struct dma_config *dmachcfg)
{
	u32 tmpctrl, tmpcfg, tmp;
	int ch = dmachcfg->ch;

	/* Channel control setup */
	tmpctrl = 0;
	switch (dmachcfg->src_size)
	{
		case 1:
			tmpctrl |= DMAC_CHAN_SRC_WIDTH_8;
			break;

		case 2:
			tmpctrl |= DMAC_CHAN_SRC_WIDTH_16;
			break;

		case 4:
			tmpctrl |= DMAC_CHAN_SRC_WIDTH_32;
			break;

		default:
			return -EINVAL;
	}
	switch (dmachcfg->dst_size)
	{
		case 1:
			tmpctrl |= DMAC_CHAN_DEST_WIDTH_8;
			break;

		case 2:
			tmpctrl |= DMAC_CHAN_DEST_WIDTH_16;
			break;

		case 4:
			tmpctrl |= DMAC_CHAN_DEST_WIDTH_32;
			break;

		default:
			return -EINVAL;
	}
	if (dmachcfg->src_inc != 0)
	{
		tmpctrl |= DMAC_CHAN_SRC_AUTOINC;
	}
	if (dmachcfg->dst_inc != 0)
	{
		tmpctrl |= DMAC_CHAN_DEST_AUTOINC;
	}
	if (dmachcfg->tc_inten != 0)
	{
		tmpctrl |= DMAC_CHAN_INT_TC_EN;
	}
	tmpctrl |= dmachcfg->src_bsize | dmachcfg->dst_bsize;
	dma_ctrl.dma_channels[ch].control = tmpctrl;

	/* Channel config setup */
	tmpcfg = dmachcfg->src_prph | dmachcfg->dst_prph |
		dmachcfg->flowctrl;
	dma_ctrl.dma_channels[ch].config = tmpcfg;

	dma_ctrl.dma_channels[ch].config_int_mask = 0;
	if (dmachcfg->err_inten != 0)
	{
		dma_ctrl.dma_channels[ch].config_int_mask |=
			DMAC_CHAN_IE;
	}
	if (dmachcfg->tc_inten != 0)
	{
		dma_ctrl.dma_channels[ch].config_int_mask |=
			DMAC_CHAN_ITC;
	}

	tmp = __raw_readl(DMACH_CONFIG_CH(DMAIOBASE, ch));
	tmp &= ~DMAC_CHAN_ENABLE;
	__raw_writel(tmp, DMACH_CONFIG_CH(DMAIOBASE, ch));

	/* Clear interrupts for channel */
	__raw_writel((1 << ch), DMA_INT_TC_CLEAR(DMAIOBASE));
	__raw_writel((1 << ch), DMA_INT_ERR_CLEAR(DMAIOBASE));

	/* Write control and config words */
	__raw_writel(tmpctrl, DMACH_CONTROL(DMAIOBASE, ch));
	__raw_writel(tmpcfg, DMACH_CONFIG_CH(DMAIOBASE, ch));

	return 0;
}

int lpc178x_dma_ch_enable(int ch)
{
	if (!VALID_CHANNEL(ch) || !dma_ctrl.dma_channels[ch].name)
		return -EINVAL;

	__dma_regs_lock();
	__dma_enable(ch);
	__dma_regs_unlock();

	return 0;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_ch_enable);

int lpc178x_dma_ch_disable(int ch)
{
	if (!VALID_CHANNEL(ch) || !dma_ctrl.dma_channels[ch].name)
		return -EINVAL;

	__dma_regs_lock();
	__dma_disable(ch);
	__dma_regs_unlock();

	return 0;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_ch_disable);

int lpc178x_dma_ch_get(struct dma_config *dmachcfg, char *name,
		void *irq_handler, void *data) {
	int ret;

	if (!VALID_CHANNEL(dmachcfg->ch))
		return -EINVAL;

	/* If the channel is already enabled, return */
	if (dma_ctrl.dma_channels[dmachcfg->ch].name != NULL)
		return -ENODEV;

	/* Save channel data */
	dma_ctrl.dma_channels[dmachcfg->ch].dmacfg = dmachcfg;
	dma_ctrl.dma_channels[dmachcfg->ch].name = name;
	dma_ctrl.dma_channels[dmachcfg->ch].irq_handler = irq_handler;
	dma_ctrl.dma_channels[dmachcfg->ch].data = data;

	/* Setup channel */
	__dma_regs_lock();
	dma_clocks_up();
	ret = lpc178x_ch_setup(dmachcfg);
	__dma_regs_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_ch_get);

int lpc178x_dma_ch_put(int ch)
{
	u32 tmp;

	if (!VALID_CHANNEL(ch))
		return -EINVAL;

	/* If the channel is already disabled, return */
	if (dma_ctrl.dma_channels[ch].name == NULL)
		return -EINVAL;

	tmp = __raw_readl(DMACH_CONFIG_CH(DMAIOBASE, ch));
	tmp &= ~DMAC_CHAN_ENABLE;
	__raw_writel(tmp, DMACH_CONFIG_CH(DMAIOBASE, ch));

	__dma_regs_lock();
	lpc178x_dma_ch_disable(ch);
	dma_clocks_down();
	__dma_regs_unlock();

	dma_ctrl.dma_channels[ch].name = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_ch_put);

int lpc178x_dma_ch_pause_unpause(int ch, int pause) {
	u32 tmp;

	if (!VALID_CHANNEL(ch))
		return -EINVAL;

	/* If the channel is already disabled, return */
	if (dma_ctrl.dma_channels[ch].name == NULL)
		return -EINVAL;

	tmp = __raw_readl(DMACH_CONFIG_CH(DMAIOBASE, ch));
	if (pause) {
		tmp |= DMAC_CHAN_HALT;
	}
	else {
		tmp &= ~DMAC_CHAN_HALT;
	}
	__raw_writel(tmp, DMACH_CONFIG_CH(DMAIOBASE, ch));

	return 0;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_ch_pause_unpause);

int lpc178x_dma_start_pflow_xfer(int ch,
				void *src,
				void *dst,
				int enable)
{
	u32 tmp;

	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL))
		return -EINVAL;

	/* When starting a DMA transfer where the peripheral is the flow
	   controller, DMA must be previously disabled */
	tmp = __raw_readl(DMACH_CONFIG_CH(DMAIOBASE, ch));
	if (tmp & DMAC_CHAN_ENABLE)
		return -EBUSY;

	__dma_regs_lock();
	__raw_writel((u32) src, DMACH_SRC_ADDR(DMAIOBASE, ch));
	__raw_writel((u32) dst, DMACH_DEST_ADDR(DMAIOBASE, ch));
	__raw_writel(0, DMACH_LLI(DMAIOBASE, ch));
	__raw_writel(dma_ctrl.dma_channels[ch].control,
		DMACH_CONTROL(DMAIOBASE, ch));

	tmp = dma_ctrl.dma_channels[ch].config |
		dma_ctrl.dma_channels[ch].config_int_mask;
	if (enable != 0)
		tmp |= DMAC_CHAN_ENABLE;
	__raw_writel(tmp, DMACH_CONFIG_CH(DMAIOBASE, ch));

	__dma_regs_unlock();

	return 0;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_start_pflow_xfer);

int lpc178x_dma_is_active(int ch)
{
	int active = 0;

	if ((VALID_CHANNEL(ch)) && (dma_ctrl.dma_channels[ch].name != NULL)) {
		if (__raw_readl(DMACH_CONFIG_CH(DMAIOBASE, ch)) &
			DMAC_CHAN_ENABLE)
			active = 1;
	}

	return active;

}
EXPORT_SYMBOL_GPL(lpc178x_dma_is_active);

extern u32 lpc178x_dma_llist_v_to_p(int ch, u32 vlist)
{
	u32 pptr;

	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL) ||
		(dma_ctrl.dma_channels[ch].list_vstart == 0))
		return 0;

	pptr = vlist - dma_ctrl.dma_channels[ch].list_vstart;
	pptr += dma_ctrl.dma_channels[ch].list_pstart;

	return pptr;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_llist_v_to_p);

u32 lpc178x_dma_llist_p_to_v(int ch, u32 plist)
{
	u32 vptr;

	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL) ||
		(dma_ctrl.dma_channels[ch].list_vstart == 0))
		return 0;

	vptr = plist - dma_ctrl.dma_channels[ch].list_pstart;
	vptr += dma_ctrl.dma_channels[ch].list_vstart;

	return vptr;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_llist_p_to_v);

u32 lpc178x_dma_alloc_llist(int ch, int entries)
{
	int i;
	dma_addr_t dma_handle;
	struct dma_list_ctrl *pdmalist, *pdmalistst;

	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL))
		return 0;

	/*
	 * Limit number of list entries, but add 1 extra entry as a spot holder
	 * for the end of the list
	 */
	if (entries < 2) {
		entries = 2;
	}
	if (entries > 64) {
		entries = 64;
	}
	entries++;

	/* Save list information */
	dma_ctrl.dma_channels[ch].list_entries = entries;
	dma_ctrl.dma_channels[ch].list_size =
		(entries * sizeof(struct dma_list_ctrl));
	dma_ctrl.dma_channels[ch].list_vstart = (u32) dma_alloc_coherent(NULL,
		dma_ctrl.dma_channels[ch].list_size, &dma_handle, GFP_KERNEL);
	if (dma_ctrl.dma_channels[ch].list_vstart == 0) {
		/* No allocated DMA space */
		return 0;
	}
	dma_ctrl.dma_channels[ch].list_pstart = (u32) dma_handle;

	/* Setup list tail and head pointers */
	pdmalist = pdmalistst =
		(struct dma_list_ctrl *) dma_ctrl.dma_channels[ch].list_vstart;
	for (i = 0; i < entries; i++) {
		pdmalistst->next_list_addr = pdmalistst + 1;
		pdmalistst->prev_list_addr = pdmalistst - 1;
		pdmalistst->next_list_phy = lpc178x_dma_llist_v_to_p(
			ch, (u32) pdmalistst->next_list_addr);
		pdmalistst->prev_list_phy = lpc178x_dma_llist_v_to_p(
			ch, (u32) pdmalistst->prev_list_addr);
		pdmalistst++;
	}
	pdmalist[entries - 1].next_list_addr = pdmalist;
	pdmalist[entries - 1].next_list_phy = lpc178x_dma_llist_v_to_p(ch,
		(u32) pdmalist[entries - 1].next_list_addr);
	pdmalist->prev_list_addr = &pdmalist[entries - 1];
	pdmalist->prev_list_phy = lpc178x_dma_llist_v_to_p(
		ch, (u32) pdmalist->prev_list_addr);

	/* Save current free descriptors and current head/tail */
	dma_ctrl.dma_channels[ch].free_entries = entries - 1;
	dma_ctrl.dma_channels[ch].list_head = pdmalist;
	dma_ctrl.dma_channels[ch].list_tail = pdmalist;
	dma_ctrl.dma_channels[ch].list_curr = pdmalist;

	return dma_ctrl.dma_channels[ch].list_vstart;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_alloc_llist);

void lpc178x_dma_dealloc_llist(int ch) {

	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL) ||
		(dma_ctrl.dma_channels[ch].list_vstart == 0))
		return;

	dma_free_coherent(NULL, dma_ctrl.dma_channels[ch].list_size,
		(void *) dma_ctrl.dma_channels[ch].list_vstart,
		(dma_addr_t) dma_ctrl.dma_channels[ch].list_pstart);
	dma_ctrl.dma_channels[ch].list_head = 0;
	dma_ctrl.dma_channels[ch].list_tail = 0;
	dma_ctrl.dma_channels[ch].list_entries = 0;
	dma_ctrl.dma_channels[ch].free_entries = 0;
	dma_ctrl.dma_channels[ch].list_vstart = 0;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_dealloc_llist);

extern u32 lpc178x_dma_get_llist_head(int ch) {
	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL) ||
		(dma_ctrl.dma_channels[ch].list_vstart == 0))
		return 0;

	/* Return the current list pointer (virtual) for the
	   DMA channel */
	return lpc178x_dma_llist_p_to_v(ch,
		__raw_readl(DMACH_LLI(DMAIOBASE, ch)));
}
EXPORT_SYMBOL_GPL(lpc178x_dma_get_llist_head);

extern void lpc178x_dma_flush_llist(int ch) {
	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL) ||
		(dma_ctrl.dma_channels[ch].list_vstart == 0))
		return;

	/* Disable channel and clear LLI */
	__dma_regs_lock();
	__dma_disable(ch);
	__raw_writel(0, DMACH_LLI(DMAIOBASE, ch));
	__dma_regs_unlock();

	dma_ctrl.dma_channels[ch].list_head = (struct dma_list_ctrl *)
		dma_ctrl.dma_channels[ch].list_vstart;
	dma_ctrl.dma_channels[ch].list_tail = (struct dma_list_ctrl *)
		dma_ctrl.dma_channels[ch].list_vstart;
	dma_ctrl.dma_channels[ch].list_curr = (struct dma_list_ctrl *)
		dma_ctrl.dma_channels[ch].list_vstart;
	dma_ctrl.dma_channels[ch].free_entries =
		dma_ctrl.dma_channels[ch].list_entries - 1;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_flush_llist);

u32 lpc178x_dma_queue_llist_entry(int ch, void *src, void *dst, int size)
{
	struct dma_list_ctrl *plhead;
	u32 ctrl, cfg;

	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL) ||
		(dma_ctrl.dma_channels[ch].list_vstart == 0))
		return 0;

	/* Exit if all the buffers are used */
	if (dma_ctrl.dma_channels[ch].free_entries == 0) {
		return 0;
	}

	/* Next available DMA link descriptor */
	plhead = dma_ctrl.dma_channels[ch].list_head;

	/* Adjust size to number of transfers (vs bytes) */
	size = size / dma_ctrl.dma_channels[ch].dmacfg->dst_size;

	/* Setup control and config words */
	ctrl = dma_ctrl.dma_channels[ch].control | size;
	cfg = dma_ctrl.dma_channels[ch].config | DMAC_CHAN_ENABLE |
		dma_ctrl.dma_channels[ch].config_int_mask;

	/* Populate DMA linked data structure */
	plhead->dmall.src = (u32) src;
	plhead->dmall.dest = (u32) dst;
	plhead->dmall.next_lli = 0;
	plhead->dmall.ctrl = ctrl;

	__dma_regs_lock();

	/* Append this link to the end of the previous link */
	plhead->prev_list_addr->dmall.next_lli =
		lpc178x_dma_llist_v_to_p(ch, (u32) plhead);

	/* Decrement available buffers */
	dma_ctrl.dma_channels[ch].free_entries--;

	/*
	 * If the DMA channel is idle, then the buffer needs to be placed
	 * directly into the DMA registers
	 */
	if ((__raw_readl(DMACH_CONFIG_CH(DMAIOBASE, ch)) &
	    DMAC_CHAN_ENABLE) == 0) {
		/* DMA is disabled, so move the current buffer into the
		   channel registers and start transfer */
		__raw_writel((u32) src, DMACH_SRC_ADDR(DMAIOBASE, ch));
		__raw_writel((u32) dst, DMACH_DEST_ADDR(DMAIOBASE, ch));
		__raw_writel(0, DMACH_LLI(DMAIOBASE, ch));
		__raw_writel(ctrl, DMACH_CONTROL(DMAIOBASE, ch));
		__raw_writel(cfg, DMACH_CONFIG_CH(DMAIOBASE, ch));
	}
	else if (__raw_readl(DMACH_LLI(DMAIOBASE, ch)) == 0) {
		/* Update current entry to next entry */
		__raw_writel(dma_ctrl.dma_channels[ch].list_tail->next_list_phy,
			DMACH_LLI(DMAIOBASE, ch));

		/*
		 * If the channel was stopped before the next entry made it
		 * into the hardware descriptor, the next entry didn't make it
		 * there fast enough, so load the new descriptor here.
		 */
		if ((__raw_readl(DMACH_CONFIG_CH(DMAIOBASE, ch)) &
		    DMAC_CHAN_ENABLE) == 0) {
			__raw_writel((u32) src, DMACH_SRC_ADDR(DMAIOBASE, ch));
			__raw_writel((u32) dst, DMACH_DEST_ADDR(DMAIOBASE, ch));
			__raw_writel(0, DMACH_LLI(DMAIOBASE, ch));
			__raw_writel(ctrl, DMACH_CONTROL(DMAIOBASE, ch));
			__raw_writel(cfg, DMACH_CONFIG_CH(DMAIOBASE, ch));
		}
	}

	/* Process next link on next call */
	dma_ctrl.dma_channels[ch].list_head = plhead->next_list_addr;

	__dma_regs_unlock();

	return (u32) plhead;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_queue_llist_entry);

extern u32 lpc178x_get_free_llist_entry(int ch) {
	struct dma_list_ctrl *pltail;

	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL) ||
		(dma_ctrl.dma_channels[ch].list_vstart == 0))
		return 0;

	/* Exit if no entries to free */
	if (dma_ctrl.dma_channels[ch].free_entries ==
		dma_ctrl.dma_channels[ch].list_entries) {
		return 0;
	}

	/* Get tail pointer */
	pltail = dma_ctrl.dma_channels[ch].list_tail;

	/* Next tail */
	dma_ctrl.dma_channels[ch].list_tail = pltail->next_list_addr;

	/* Increment available buffers */
	dma_ctrl.dma_channels[ch].free_entries++;

	return (u32) pltail;
}
EXPORT_SYMBOL_GPL(lpc178x_get_free_llist_entry);

int lpc178x_dma_start_xfer(int ch, u32 config)
{
	struct dma_list_ctrl *plhead;

	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL) ||
		(dma_ctrl.dma_channels[ch].list_vstart == 0))
		return -1;

	plhead = dma_ctrl.dma_channels[ch].list_head;
	__dma_regs_lock();
	__raw_writel(plhead->dmall.src, DMACH_SRC_ADDR(DMAIOBASE, ch));
	__raw_writel(plhead->dmall.dest, DMACH_DEST_ADDR(DMAIOBASE, ch));
	__raw_writel(plhead->dmall.next_lli, DMACH_LLI(DMAIOBASE, ch));
	__raw_writel(plhead->dmall.ctrl, DMACH_CONTROL(DMAIOBASE, ch));
	__raw_writel(config, DMACH_CONFIG_CH(DMAIOBASE, ch));
	__dma_regs_unlock();

	return 0;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_start_xfer);

u32 lpc178x_dma_queue_llist(int ch, void *src, void *dst, int size, u32 ctrl)
{
	struct dma_list_ctrl *plhead;

	if ((!VALID_CHANNEL(ch)) || (dma_ctrl.dma_channels[ch].name == NULL) ||
		(dma_ctrl.dma_channels[ch].list_vstart == 0))
		return 0;

	/* Exit if all the buffers are used */
	if (dma_ctrl.dma_channels[ch].free_entries == 0) {
		return 0;
	}

	/* Next available DMA link descriptor */
	plhead = dma_ctrl.dma_channels[ch].list_curr;

	/* Populate DMA linked data structure */
	plhead->dmall.src = (u32) src;
	plhead->dmall.dest = (u32) dst;
	plhead->dmall.next_lli = 0;
	plhead->dmall.ctrl = ctrl;

	/* Append this link to the end of the previous link */
	plhead->prev_list_addr->dmall.next_lli =
		lpc178x_dma_llist_v_to_p(ch, (u32) plhead);

	/* Decrement available buffers */
	dma_ctrl.dma_channels[ch].free_entries--;

	/* Process next link on next call */
	dma_ctrl.dma_channels[ch].list_curr = plhead->next_list_addr;

	return (u32) plhead;
}
EXPORT_SYMBOL_GPL(lpc178x_dma_queue_llist);

extern void lpc178x_dma_force_burst(int ch, int src)
{
	__raw_writel(1 << src, DMA_SW_BURST_REQ(DMAIOBASE));
}
EXPORT_SYMBOL_GPL(lpc178x_dma_force_burst);

static irqreturn_t dma_irq_handler(int irq, void *dev_id)
{
	int i;
	unsigned long dint = __raw_readl(DMA_INT_STAT(DMAIOBASE));
	unsigned long tcint = __raw_readl(DMA_INT_TC_STAT(DMAIOBASE));
	unsigned long eint = __raw_readl(DMA_INT_ERR_STAT(DMAIOBASE));
	unsigned long i_bit;

	for (i = MAX_DMA_CHANNELS - 1; i >= 0; i--) {
		i_bit = 1 << i;
		if (dint & i_bit) {
			struct dma_channel *channel = &dma_ctrl.dma_channels[i];

			if (channel->name && channel->irq_handler) {
				int cause = 0;

				if (eint & i_bit) {
					__raw_writel(i_bit,
						DMA_INT_ERR_CLEAR(DMAIOBASE));
					cause |= DMA_ERR_INT;
				}
				if (tcint & i_bit) {
					__raw_writel(i_bit,
						DMA_INT_TC_CLEAR(DMAIOBASE));
					cause |= DMA_TC_INT;
				}

				channel->irq_handler(i, cause, channel->data);
			} else {
				/*
				 * IRQ for an unregistered DMA channel
				 */
				__raw_writel(i_bit,
					DMA_INT_ERR_CLEAR(DMAIOBASE));
				__raw_writel(i_bit,
					DMA_INT_TC_CLEAR(DMAIOBASE));
				printk(KERN_WARNING
				       "spurious IRQ for DMA channel %d\n", i);
			}
		}
	}

	return IRQ_HANDLED;
}

void __init lpc178x_dma_init(void)
{
	int ret;

	ret = request_irq(LPC178X_DMA_IRQ, dma_irq_handler, 0, "DMA", NULL);
	if (ret) {
		printk(KERN_CRIT "Wow!  Can't register IRQ for DMA\n");
		goto out;
	}

	/* Get DMA clock */
	dma_ctrl.clk = clk_get(NULL, "clk_dmac");
	if (IS_ERR(dma_ctrl.clk)) {
		ret = -ENODEV;
		goto errout;
	}
	clk_enable(dma_ctrl.clk);

	/* Clear DMA controller */
	__raw_writel(1, DMA_CONFIG(DMAIOBASE));
	__raw_writel(0xFF, DMA_INT_TC_CLEAR(DMAIOBASE));
	__raw_writel(0xFF, DMA_INT_ERR_CLEAR(DMAIOBASE));

	/*
	 * Configure multiplexing of DMA requests: enable two I2S DMA requests
	 * instead of SSP2_TX/RX enabled by default.
	 */
	LPC178X_SCC->dmacreqsel |=
		LPC178X_SCC_DMACREQSEL_SEL6_I2S0_MSK |
		LPC178X_SCC_DMACREQSEL_SEL7_I2S1_MSK;

	/* Clock is only enabled when needed to save power */
	clk_disable(dma_ctrl.clk);

	goto out;

errout:
	free_irq(LPC178X_DMA_IRQ, NULL);
out:
	return;
}
