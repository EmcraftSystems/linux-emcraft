/*
 * drivers/amba/pl080.c - ARM PrimeCell DMA Controller PL080 driver
 *
 * Copyright (C) 2006 ARM Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Documentation: ARM DDI 0196F
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/amba/bus.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>
#include <asm/dma.h>
#include <asm/mach/dma.h>
#include <linux/amba/dma.h>
#include <linux/amba/pl080.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#define DRIVER_NAME	"dmac-pl080"

/*
 * PM support is not complete. Turn it off.
 */
#undef CONFIG_PM

static int	pl080_request(dmach_t cnum, dma_t * cdata);
static void	pl080_free(dmach_t cnum, dma_t * cdata);
static void 	pl080_enable(dmach_t cnum, dma_t * cdata);
static void 	pl080_disable(dmach_t cnum, dma_t * cdata);
static int	pl080_residue(dmach_t cnum, dma_t * cdata);
static int	pl080_setspeed(dmach_t cnum, dma_t * cdata, int newspeed);

struct pl080_tag {
	/* pl080 register base */
	void __iomem *	base;
	/*
	 * - dma_pool for the LLIs
	 * (can't use dma_alloc_coherent/free because of the requirement for
	 * free() to have interrupts enabled when it is called)
	 */
	struct dma_pool * pool;
	/* LLI details for each DMA channel */
	chanlli * chanllis;
} pl080;
static struct amba_driver pl080_driver;

/* All int functions return 0 for success unless detailed */
static int pl080_request(dmach_t cnum, dma_t * cdata){
	return 0;
}

static void pl080_free(dmach_t chan_num, dma_t * cdata){
	unsigned int i, active_count = 0;

	/*
	 * Free any DMA memory in use for the LLIs
	 */
	if((pl080.chanllis[chan_num].va_list) && (pl080.chanllis[chan_num].bus_list)){
		dma_pool_free(pl080.pool, pl080.chanllis[chan_num].va_list, pl080.chanllis[chan_num].bus_list);
		pl080.chanllis[chan_num].num_entries = 0;
		pl080.chanllis[chan_num].va_list = NULL;
		pl080.chanllis[chan_num].bus_list = (dma_addr_t)NULL;
	}

	/*
	 * If no DMA channels active then deconfigure the DMAC entirely
	 */
	for(i = 0; i < MAX_DMA_CHANNELS; i++)
		active_count += dma_channel_active((dmach_t)i);

	if(!active_count)
	{
		unsigned int r;

		r = readl(pl080.base + PL080_OS_CFG);
		r &= PL080_MASK_CFG;
		r &= ~PL080_MASK_EN;
		writel(r, pl080.base + PL080_OS_CFG);
	}
}

/*
 * Enable the DMA channel & terminal count interrupts on it
 * TODO:: Currently hard coded for memory to peripheral transfer (DMA in control)
 *				- flow control setting should be stored in cdata
 */
inline void pl080_enable(dmach_t cnum, dma_t * cdata){
	void __iomem * cbase = pl080.base + PL080_OS_CHAN_BASE + (cnum * PL080_OS_CHAN);
	volatile unsigned int r = 0;

	/*
	 * Do not access config register until channel shows as disabled
	 */
	while((readl(pl080.base + PL080_OS_ENCHNS) & (1<< cnum)) & PL080_MASK_ENCHNS){
		;
	}
	mb();
	r = readl(cbase + PL080_OS_CCFG );
	mb();
	writel((r | PL080_MASK_CEN | PL080_MASK_INTTC | PL080_MASK_INTERR | PL080_FCMASK_M2P_DMA) &~(PL080_MASK_HALT), cbase + PL080_OS_CCFG );
}

/*
 * Disable without losing data in the channel's FIFO:
 * - Set the Halt bit so that subsequent DMA requests are ignored.
 * - Poll the Active bit until it reaches 0, indicating that there is no data
 * 	remaining in the channel's FIFO.
 * - Clear the Channel Enable bit.
 *
 * Currently not implemented correctly in the hardware
 *
 */
/*
 * TODO:: Currently hard coded for memory to peripheral transfer (DMA in control)
 *				- flow control setting should be stored in cdata
 */
inline void pl080_disable(dmach_t cnum, dma_t * cdata){
	void __iomem * cbase = pl080.base + PL080_OS_CHAN_BASE + (cnum * PL080_OS_CHAN);
	volatile unsigned int r = readl(cbase + PL080_OS_CCFG );

#ifdef ERRATUM_FIXED
	mb();
	writel(r | PL080_MASK_HALT, cbase + PL080_OS_CCFG );
	mb();
	while(readl(cbase + PL080_OS_CCFG ) & PL080_MASK_ACTIVE){
		;
	}
#else
	// printk("pl080_disable() - check if we can use the active flag\n");
	// printk(" - this code doesn't\n");
#endif
	r = readl(cbase + PL080_OS_CCFG );
	mb();
	writel(r & ~(PL080_MASK_CEN) & ~(PL080_MASK_INTTC) & ~(PL080_MASK_INTERR & ~(PL080_FCMASK_M2P_DMA)), cbase + PL080_OS_CCFG );
	mb();
	while(readl(cbase + PL080_OS_CCFG ) & PL080_MASK_CEN){
		;
	}
}

/* Disable the channel, read the control register, re-enable the channel */
/* May not be an accurate value - see TRM */
/* ASSUME returns bytes */
static int pl080_residue(dmach_t cnum, dma_t * cdata){
	void __iomem * cbase = pl080.base + PL080_OS_CHAN_BASE + (cnum * PL080_OS_CHAN) + PL080_OS_CCTL;
	volatile unsigned int r = 0;

	pl080_disable(cnum, cdata);
	mb();

	/* The number of source width transfers (AACI == 32 bits) completed */
	r = readl(cbase + PL080_OS_CCTL ) & PL080_MASK_TSFR_SIZE;
	mb();

	pl080_enable(cnum, cdata);

	return r * 4;
}
/* Not implemented - should return the new speed */
static int pl080_setspeed(dmach_t cnum, dma_t * cdata, int newspeed){
	return 0;
}

#ifdef PL080_IRQ_REQUIRED
/* Each device requesting DMA chains its interrupt handler to the DMA interrupt,
 * rather than this handler attaching this interrupt....
 */
static irqreturn_t pl080_irq(int irq, void *devid, struct pt_regs *regs)
{
	u32 mask = 0;

	return mask ? IRQ_HANDLED : IRQ_NONE;
}
#endif

/*
 * Power Management.
 */
#ifdef CONFIG_PM
#else
#define pl080_do_suspend	NULL
#define pl080_do_resume		NULL
#define pl080_suspend		NULL
#define pl080_resume		NULL
#endif

/*
 * Complete the DMA channel initialization
 * started by the AMBA DMA code
 * - Set the ops for the board to call
 */
char pl080_type[0x10] = "PL080 DMAC";
int pl080_init_dma(dma_t * dma_chan, struct dma_ops * ops){
	int i;

	for(i = 0; i < MAX_DMA_CHANNELS; i++){
		dma_chan[i].dma_base = (unsigned int)pl080.base;
	}
	ops->request 	= pl080_request;
	ops->free	= pl080_free;
	ops->enable	= pl080_enable ;
	ops->disable 	= pl080_disable;
	ops->residue 	= pl080_residue;
	ops->setspeed 	= pl080_setspeed;
	ops->type 	= pl080_type;

	return 0;
}

static int __devinit pl080_probe(struct amba_device *dev, void *id)
{
	int ret,i;

	ret = amba_request_regions(dev, NULL);
	if (ret)
		return ret;


	pl080.base = ioremap(dev->res.start, SZ_4K);
	if (!pl080.base) {
		ret = -ENOMEM;
		goto out;
	}
	/*
	 * Make one struct for each DMA channel
	 * to hold details of its LLIs
	 */
	pl080.chanllis = kmalloc(sizeof(chanlli) * MAX_DMA_CHANNELS, GFP_KERNEL);
	if (!pl080.chanllis) {
		ret = -ENOMEM;
		goto out;
	}
	for(i = 0; i < MAX_DMA_CHANNELS; i++){
		pl080.chanllis[i].num_entries = 0;
		pl080.chanllis[i].va_list = NULL;
		pl080.chanllis[i].bus_list = (dma_addr_t)NULL;
	}

	if(!(pl080.pool = dma_pool_create(pl080_driver.drv.name, (struct device *)dev,
			PL080_MAX_LLIS_SIZE, PL080_ALIGN, PL080_ALLOC))){

		kfree(pl080.chanllis);
		ret = -ENOMEM;
		goto out;
	}

 out:
	amba_release_regions(dev);
	return ret;
}

static int __devexit pl080_remove(struct amba_device *dev)
{
	if(pl080.pool){
		dma_pool_destroy(pl080.pool);
		pl080.pool = NULL;
	}
	return 0;
}

static struct amba_id pl080_ids[] = {
	{
		.id	= 0x00041080,
		.mask	= 0x000fffff,
	},
	{ 0, 0 },
};

static struct amba_driver pl080_driver = {
	.drv		= {
		.name	= DRIVER_NAME,
	},
	.probe		= pl080_probe,
	.remove		= __devexit_p(pl080_remove),
	.suspend	= pl080_suspend,
	.resume		= pl080_resume,
	.id_table	= pl080_ids,
};

/*
 * Module initialization
 */
static int __init pl080_init(void)
{
	pl080.base = NULL;
	pl080.chanllis = NULL; /* LLI details for each DMA channel */
	pl080.pool = NULL;

	return amba_driver_register(&pl080_driver);
}
/*
 * Module destruction
 */
static void __exit pl080_exit(void)
{
	amba_driver_unregister(&pl080_driver);
}

/*
 * Set up a circular linked list of period sized packets
 * We loop until stopped by another entity
 *
 * TODO Abstract this function for use by any device.
 * It has only been tested with VersatilePB/AACI
 *
 * - CAUTION ASSUMES FIXED dest, cword ?? other??
 *
 * All addresses stored in the LLI must be bus addresses
 * Set the lower bit of the bus address to ensure the correct bus is used
 * dma_chan[chan_num] holds the DMA buffer info
 *
 * Return bus address of first LLI
 */
dma_addr_t pl080_make_llis(dmach_t chan_num, unsigned int address, unsigned int length, unsigned int packet_size, pl080_lli * setting){

	int i;
	unsigned int num_llis = 0;
	/*
	 * Whether we increment the lli address to indicate the bus
	 * for this transfer
	 */
	unsigned int bus_increment = setting->next;
	pl080_lli * llis_bus = NULL;
	pl080_lli * llis_va = NULL;

	if(NULL != pl080.chanllis[chan_num].va_list){
		/*
		 * Repeated call - destroy previous LLIs
		 */
		dma_pool_free(pl080.pool, pl080.chanllis[chan_num].va_list, pl080.chanllis[chan_num].bus_list);
		pl080.chanllis[chan_num].num_entries = 0;
		pl080.chanllis[chan_num].va_list = NULL;
		pl080.chanllis[chan_num].bus_list = (dma_addr_t)NULL;
	}
	/*
 	 * dma_chan[chan_num] holds the DMA buffer info
	 */
	setting->start = address;
	num_llis = length / packet_size;

	/* Get memory for the LLIs */
	if(PL080_MAX_LLIS_SIZE < num_llis * sizeof(pl080_lli)){
		printk(KERN_ERR "pl080.c::make_lli_aaci() - 0x%08x bytesNeed for the LLIs, consider rebuilding with PL080_MAX_LLIS_SIZE increased\n", num_llis * sizeof(pl080_lli));
	} else {
		pl080.chanllis[chan_num].va_list = dma_pool_alloc(pl080.pool, GFP_KERNEL, &pl080.chanllis[chan_num].bus_list);
	}
	if(NULL == pl080.chanllis[chan_num].va_list){
		printk(KERN_ERR "pl080.c::make_lli_aaci() - Failed to get DMA memory for the LLIs\n");
		return (dma_addr_t)NULL;
	}

	pl080.chanllis[chan_num].num_entries = num_llis;

	llis_va	= (pl080_lli *) pl080.chanllis[chan_num].va_list;
	llis_bus = (pl080_lli *) pl080.chanllis[chan_num].bus_list;

	for(i = 0; i < num_llis - 1; i++){
		llis_va[i].start = (dma_addr_t)((unsigned int)setting->start + (i * packet_size));
		llis_va[i].dest	 = setting->dest;
		llis_va[i].cword = setting->cword;

		/*
		 * Adjust to access the memory on the correct DMA bus
		 */
		llis_va[i].next = (dma_addr_t)((unsigned int) &llis_bus[i + 1] + bus_increment);
	}
	llis_va[i].start = (dma_addr_t)((unsigned int)setting->start + (i * packet_size));
	llis_va[i].dest	= setting->dest;
	llis_va[i].cword = setting->cword;
	llis_va[i].next = (dma_addr_t)((unsigned int) &llis_bus[0] + bus_increment);

	/*
	 * Initial register setting
	 */
	setting->next = llis_va[0].next;

	// printk("\nPMP llis at %p\n", pl080.chanllis[chan_num].va_list);
	return pl080.chanllis[chan_num].bus_list;
}

/*
 *	Generalized interrupt handling
 *	i.e. not the terminal count part which will be device specific
 */
/* Handle only interrupts on the correct channel */
static int pl080_ignore_this_irq(dmach_t dma_chan){
	unsigned int r = readl(pl080.base + PL080_OS_ISR);
	return !(r & 1 << dma_chan);
}

/*
 * TODO:: Report the errors, rather than just clearing them
 */
static unsigned int errCtr = 0;
static void pl080_pre_irq(dmach_t dma_chan){

	unsigned int isr_err = readl(pl080.base + PL080_OS_ISR_ERR);
	if((1 << dma_chan) & isr_err){
		errCtr++;
	}
}
/* Clear any interrupts on the correct channel */
static void pl080_post_irq(dmach_t dma_chan){

	/* Finally clear the interrupt of both kinds */
	writel((1 << dma_chan), pl080.base + PL080_OS_ICLR_TC);
	writel((1 << dma_chan), pl080.base + PL080_OS_ICLR_ERR);
}

/*
 * Configure the DMAC
 *
 * LittleEndian, LittleEndian, disabled
 */
void pl080_configure(void){
	unsigned int reg;

	reg = readl(pl080.base + PL080_OS_CFG);
	reg &= PL080_MASK_CFG;
	reg |= PL080_MASK_EN;
	writel(reg, pl080.base + PL080_OS_CFG);
}

/*
 * Configure the DMA channel
 *
 * Set up interrupt calls for devices to use
 * TODO:: Find a neater way.....
 */
void pl080_configure_chan(dmach_t chan_num, struct amba_dma_data * data){
	/*
	 * Set address of DMA channel registers
	 */
	unsigned int chan_base = (unsigned int)pl080.base + PL080_OS_CHAN_BASE;
	chan_base += chan_num * PL080_OS_CHAN;

	/*
	 *	The interrupt handlers & destination are known
	 */
	/*
	 * Interrupt data
	 * - DMAC interrupt status register address
	 * - mask to use
	 * - DMAC interrupt clear address
	 * - mask(s) to use
	 */
	 data->irq_ignore = pl080_ignore_this_irq;
	 data->irq_pre = pl080_pre_irq;
	 data->irq_post = pl080_post_irq;

	/*
	 * Always configure the pl080 itself,
	 * in case this is the first channel configuration
	 */
	pl080_configure();

}
/*
 * Restart the LLIs by setting the channel LLI
 * register to point to the first entry
 *
 * Useful where e.g. the data supplier
 * restarts due to perceived underrun
 */
void pl080_reset_cycle(dmach_t cnum){
	void __iomem * cbase = (void __iomem * )((unsigned int)pl080.base + (unsigned int)PL080_OS_CHAN_BASE + (unsigned int)(cnum * PL080_OS_CHAN));
	writel((unsigned int)(((unsigned int)(pl080.chanllis[cnum].bus_list)) + 1), cbase + PL080_OS_CLLI);
}

/*
 * Set up the DMA channel registers for a transfer
 * whose LLIs are ready
 */
void pl080_transfer_configure(dmach_t chan_num, pl080_lli *setting, unsigned int ccfg){

	unsigned int chan_base = (unsigned int)pl080.base + PL080_OS_CHAN_BASE;
	chan_base += chan_num * PL080_OS_CHAN;

	writel(setting->start, chan_base + PL080_OS_CSRC);
	writel(setting->dest , chan_base + PL080_OS_CDST);
	writel(setting->next , chan_base + PL080_OS_CLLI);
	writel(setting->cword, chan_base + PL080_OS_CCTL);
	writel(ccfg , chan_base + PL080_OS_CCFG);
}

module_init(pl080_init);
module_exit(pl080_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ARM PrimeCell PL080 DMA Controller driver");

