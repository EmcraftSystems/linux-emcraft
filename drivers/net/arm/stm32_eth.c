/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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

#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

#include <mach/eth.h>

/*
 * Prefix used by driver in printk()s
 */
#define STM32_INFO			KERN_INFO STM32_ETH_DRV_NAME

/*
 * MACCR reg fields
 */
#define STM32_MAC_CR_RE			(1 << 2)	/* Receiver enable    */
#define STM32_MAC_CR_TE			(1 << 3)	/* Transmitter enable */
#define STM32_MAC_CR_DM			(1 << 11)	/* Duplex mode	      */
#define STM32_MAC_CR_FES		(1 << 14)	/* Fast Eth speed     */

/*
 * MACMIIAR reg fields
 */
#define STM32_MAC_MIIAR_MB		(1 << 0)	/* MII busy	      */
#define STM32_MAC_MIIAR_MW		(1 << 1)	/* MII write	      */

#define STM32_MAC_MIIAR_CR_BIT		2		/* Clock range	      */
#define STM32_MAC_MIIAR_CR_MSK		0x7

#define STM32_MAC_MIIAR_MR_BIT		6		/* MII register	      */
#define STM32_MAC_MIIAR_MR_MSK		0x1F

#define STM32_MAC_MIIAR_PA_BIT		11		/* PHY address	      */
#define STM32_MAC_MIIAR_PA_MSK		0x1F

/*
 * DMABMR reg fields
 */
#define STM32_MAC_DMABMR_SR		(1 << 0)	/* Software reset     */

#define STM32_MAC_DMABMR_PBL_BIT	8		/* Burst length	      */
#define STM32_MAC_DMABMR_PBL_MSK	0x3F

#define STM32_MAC_DMABMR_RTPR_BIT	14		/* Rx:Tx priority rat.*/
#define STM32_MAC_DMABMR_RTPR_MSK	0x3
#define STM32_MAC_DMABMR_RTPR_1_1	0x0		/* 1 : 1	      */
#define STM32_MAC_DMABMR_RTPR_2_1	0x1		/* 2 : 1	      */
#define STM32_MAC_DMABMR_RTPR_3_1	0x2		/* 3 : 1	      */
#define STM32_MAC_DMABMR_RTPR_4_1	0x3		/* 4 : 1	      */

#define STM32_MAC_DMABMR_FB		(1 << 16)	/* Fixed burst	      */

#define STM32_MAC_DMABMR_RDP_BIT	17		/* RX DMA PBL	      */
#define STM32_MAC_DMABMR_RDP_MSK	0x3F

#define STM32_MAC_DMABMR_USP		(1 << 23)	/* Use separate PBL   */
#define STM32_MAC_DMABMR_AAB		(1 << 25)	/* Adr-aligned beats  */

/*
 * DMASR reg fields
 */
#define STM32_MAC_DMASR_TS		(1 << 0)	/* Transmission done  */
#define STM32_MAC_DMASR_TBUS		(1 << 2)	/* Tx buf unavailable */
#define STM32_MAC_DMASR_ROS		(1 << 4)	/* Receive overflow   */
#define STM32_MAC_DMASR_TUS		(1 << 5)	/* Transmit underflow */
#define STM32_MAC_DMASR_RS		(1 << 6)	/* Reception done     */
#define STM32_MAC_DMASR_RBUS		(1 << 7)	/* Rx buf unavailable */
#define STM32_MAC_DMASR_AIS		(1 << 15)	/* Abnormal summary   */
#define STM32_MAC_DMASR_NIS		(1 << 16)	/* Normal int summary */

#define STM32_MAC_DMASR_TX_MSK		(STM32_MAC_DMASR_TS   | \
					 STM32_MAC_DMASR_TBUS | \
					 STM32_MAC_DMASR_TUS)
#define STM32_MAC_DMASR_RX_MSK		(STM32_MAC_DMASR_RS   | \
					 STM32_MAC_DMASR_ROS  | \
					 STM32_MAC_DMASR_RBUS)

/*
 * DMAOMR reg fields
 */
#define STM32_MAC_DMAOMR_SR		(1 << 1)	/* Start/stop rx      */
#define STM32_MAC_DMAOMR_ST		(1 << 13)	/* Start/stop tx      */
#define STM32_MAC_DMAOMR_FTF		(1 << 20)	/* Flush tx FIFO      */

/*
 * DMAIER reg fields
 */
#define STM32_MAC_DMAIER_TIE		(1 << 0)	/* Tx done interrupt  */
#define STM32_MAC_DMAIER_RIE		(1 << 6)	/* Rx done interrupt  */

/*
 * DMA transmit buffer descriptor bits
 */
#define STM32_DMA_TBD_DMA_OWN		(1 << 31)	/* DMA/CPU owns bd    */
#define STM32_DMA_TBD_LS		(1 << 29)	/* Last segment	      */
#define STM32_DMA_TBD_FS		(1 << 28)	/* First segment      */
#define STM32_DMA_TBD_TCH		(1 << 20)	/* 2nd address chained*/
#define STM32_DMA_TBD_ES		(1 << 15)	/* Error summary      */
#define STM32_DMA_TBD_NC		(1 << 10)	/* No carrier	      */
#define STM32_DMA_TBD_EC		(1 << 8)	/* Excessive collision*/
#define STM32_DMA_TBD_CC_BIT		3		/* Collistion count   */
#define STM32_DMA_TBD_CC_MSK		0xF
#define STM32_DMA_TBD_UF		(1 << 1)	/* Underflow	      */

/*
 * DMA receive buffer descriptor bits
 */
#define STM32_DMA_RBD_DMA_OWN		(1 << 31)	/* DMA/CPU owns bd    */
#define STM32_DMA_RBD_FL_BIT		16		/* Frame length	      */
#define STM32_DMA_RBD_FL_MSK		0x3FFF
#define STM32_DMA_RBD_ES		(1 << 15)	/* Error summary      */
#define STM32_DMA_RBD_LE		(1 << 12)	/* Length error	      */
#define STM32_DMA_RBD_OE		(1 << 11)	/* Overrun error      */
#define STM32_DMA_RBD_FS		(1 << 9)	/* First descriptor   */
#define STM32_DMA_RBD_LS		(1 << 8)	/* Last descriptor    */
#define STM32_DMA_RBD_CE		(1 << 1)	/* CRC error	      */

#define STM32_DMA_RBD_RCH		(1 << 14)	/* 2nd address chained*/

/*
 * STM32 ETH Normal DMA buffer descriptors
 */
struct stm32_eth_dma_bd {
	u32			stat;	/* Status			      */
	u32			ctrl;	/* Control, and buffer length	      */
	dma_addr_t		buf;	/* Buffer address		      */
	dma_addr_t		next;	/* Pointer to next BD in chain	      */
};

/*
 * Private ethernet device data
 */
struct stm32_eth_priv {
	/*
	 * Platform resources
	 */
	volatile struct stm32_mac_regs	*regs;
	int				irq;

	struct net_device		*dev;
	struct napi_struct		napi;
	struct net_device_stats		stat;

	spinlock_t			rx_lock;
	spinlock_t			tx_lock;

	/*
	 * PHY settings
	 */
	u32				phy_id;

	/*
	 * MII interface
	 */
	struct mii_if_info		mii;

	/*
	 * DMA buffer descriptors
	 */
	struct stm32_eth_dma_bd		*rx_bd;
	dma_addr_t			rx_bd_dma_addr;

	struct stm32_eth_dma_bd		*tx_bd;
	dma_addr_t			tx_bd_dma_addr;

	/*
	 * ETH DMAed buffers, and indexes
	 */
	struct sk_buff			**rx_skb;
	struct sk_buff			**tx_skb;

	u32				rx_done_idx;

	u32				tx_todo_idx;
	u32				tx_done_idx;

	u32				tx_pending;
	u32				tx_restart;
	u32				tx_blocked;

	/*
	 * Driver settings
	 */
	u32				frame_max_size;
	u32				rx_buf_num;
	u32				tx_buf_num;
};

/*
 * Prototypes
 */
static void stm32_eth_hw_stop(struct net_device *dev);
static void stm32_eth_buffers_free(struct net_device *dev);
static int  stm32_plat_remove(struct platform_device *pdev);

/*
 * Hw initialization
 */
static int stm32_eth_hw_start(struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);
	int			i, rv;

	stm->mii.mdio_read(dev, 1, MII_BMCR);
	stm->mii.mdio_read(dev, 1, MII_BMSR);

	/*
	 * Reset and configure
	 */
	stm32_eth_hw_stop(dev);

	/*
	 * Check if MAC is in 'reset' state
	 */
	if (stm->regs->dmabmr & STM32_MAC_DMABMR_SR) {
		rv = -EBUSY;
		goto out;
	}

	stm->regs->dmabmr = (32 << STM32_MAC_DMABMR_PBL_BIT) |
			    (32 << STM32_MAC_DMABMR_RDP_BIT) |
			    (STM32_MAC_DMABMR_RTPR_2_1 <<
			     STM32_MAC_DMABMR_RTPR_BIT) |
			    STM32_MAC_DMABMR_FB | STM32_MAC_DMABMR_USP |
			    STM32_MAC_DMABMR_AAB;

	/*
	 * Set MAC
	 */
	stm->regs->maca0hr = (dev->dev_addr[5] <<  8) |
			     (dev->dev_addr[4] <<  0);
	stm->regs->maca0lr = (dev->dev_addr[3] << 24) |
			     (dev->dev_addr[2] << 16) |
			     (dev->dev_addr[1] <<  8) |
			     (dev->dev_addr[0] <<  0);
	/*
	 * Set-up descriptor rings
	 */
	stm->regs->dmardlar = stm->rx_bd_dma_addr;
	stm->regs->dmatdlar = stm->tx_bd_dma_addr;

	/*
	 * Flush FIFOs, and enable transmitter and receiver
	 */
	stm->regs->dmaomr |= STM32_MAC_DMAOMR_FTF;
	for (i = 0; i < 10; i++) {
		if (!(stm->regs->dmaomr & STM32_MAC_DMAOMR_FTF))
			break;
		msleep(1);
	}
	if (i == 10) {
		printk(STM32_INFO ": FIFO flush timeout\n");
		rv = -EBUSY;
		goto out;
	}

	stm->regs->maccr |= STM32_MAC_CR_TE | STM32_MAC_CR_RE;

	/*
	 * Start DMA TX and RX
	 */
	stm->regs->dmaomr |= STM32_MAC_DMAOMR_ST | STM32_MAC_DMAOMR_SR;

	rv = 0;
out:
	return rv;
}

/*
 * Reset DMA, and stop net hw
 */
static void stm32_eth_hw_stop(struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);

	/*
	 * Stop DMA operations
	 *
	 * FIXME: do full MII restart instead (with DMABMR[SR]), note
	 * that in this case we'll have to do full MAC & PHY reiniialization,
	 * otherwise frames will be received, but on send we'll get 'No carrier'
	 * status in Tx bds, and there'll be no frames on wires really. Now
	 * things just work because MAC & PHY are inited in U-Boot
	 */
	stm->regs->dmaomr &= ~(STM32_MAC_DMAOMR_SR | STM32_MAC_DMAOMR_ST);
}

/*
 * Allocate buffer and descriptors necessary for net device
 */
static int stm32_eth_buffers_alloc(struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);
	int			rv, i;

	/*
	 * Allocate RX and TX buffer descriptors
	 */
	stm->rx_bd = dma_alloc_coherent(NULL,
			sizeof(struct stm32_eth_dma_bd) * stm->rx_buf_num,
			&stm->rx_bd_dma_addr, GFP_KERNEL | GFP_DMA);
	stm->tx_bd = dma_alloc_coherent(NULL,
			sizeof(struct stm32_eth_dma_bd) * stm->tx_buf_num,
			&stm->tx_bd_dma_addr, GFP_KERNEL | GFP_DMA);
	if (!stm->rx_bd || !stm->tx_bd) {
		rv = -ENOMEM;
		goto out;
	}

	/*
	 * Allocate skbs for RX buffers, init RX descriptors, and RX chain.
	 * We allocate 4 bytes more to have a place for CRC
	 */
	for (i = 0; i < stm->rx_buf_num; i++) {
		stm->rx_skb[i] = dev_alloc_skb(stm->frame_max_size + 4);
		if (!stm->rx_skb[i]) {
			rv = -ENOMEM;
			goto out;
		}

		stm->rx_bd[i].stat = STM32_DMA_RBD_DMA_OWN;
		stm->rx_bd[i].ctrl = STM32_DMA_RBD_RCH | stm->frame_max_size;
		stm->rx_bd[i].buf = dma_map_single(&dev->dev,
						   stm->rx_skb[i]->data,
						   stm->frame_max_size,
						  DMA_FROM_DEVICE);

		stm->rx_bd[i].next  = stm->rx_bd_dma_addr +
				      (sizeof(struct stm32_eth_dma_bd) *
				       ((i + 1) % stm->rx_buf_num));
	}

	/*
	 * We'll use direct skbs for sending, so just partially init TX
	 * descriptors, and init TX chain
	 */
	for (i = 0; i < stm->tx_buf_num; i++) {
		stm->tx_skb[i] = NULL;

		stm->tx_bd[i].stat = STM32_DMA_TBD_TCH;
		stm->tx_bd[i].ctrl = 0;
		stm->tx_bd[i].buf  = 0;
		stm->tx_bd[i].next = stm->tx_bd_dma_addr +
				     (sizeof(struct stm32_eth_dma_bd) *
				      ((i + 1) % stm->tx_buf_num));
	}

	rv = 0;
out:
	if (rv != 0)
		stm32_eth_buffers_free(dev);

	return rv;
}

/*
 * Free STM32 net device buffers and descriptors
 */
static void stm32_eth_buffers_free(struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);
	int			i;

	if (!stm->tx_bd)
		goto rx_free;

	for (i = 0; i < stm->tx_buf_num; i++) {
		if (!stm->tx_skb[i])
			continue;
		dma_unmap_single(&dev->dev, stm->tx_bd[i].buf,
				 stm->tx_skb[i]->len, DMA_TO_DEVICE);
		dev_kfree_skb(stm->tx_skb[i]);
		stm->tx_skb[i] = NULL;
	}

	dma_free_coherent(NULL,
			  sizeof(struct stm32_eth_dma_bd) * stm->tx_buf_num,
			  stm->tx_bd, stm->tx_bd_dma_addr);
	stm->tx_bd = NULL;

rx_free:
	if (!stm->rx_bd)
		goto out;

	for (i = 0; i < stm->rx_buf_num; i++) {
		if (!stm->rx_skb[i])
			continue;
		dma_unmap_single(&dev->dev, stm->rx_bd[i].buf,
				 stm->rx_skb[i]->len, DMA_FROM_DEVICE);
		dev_kfree_skb(stm->rx_skb[i]);
		stm->rx_skb[i] = NULL;
	}

	dma_free_coherent(NULL,
			  sizeof(struct stm32_eth_dma_bd) * stm->rx_buf_num,
			  stm->rx_bd, stm->rx_bd_dma_addr);
	stm->rx_bd = NULL;

out:
	return;
}

/*
 * Walk though the list of ready descriptors, fetch rxed frames,
 * and copy data to skbufs
 */
static int stm32_eth_rx_get(struct net_device *dev, int processed, int budget)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);

	while (processed < budget) {
		volatile struct stm32_eth_dma_bd	*bd;
		struct sk_buff				*skb;
		u32					stat, len, idx;

		/*
		 * Get next BD, and check if it's done by DMA
		 */
		idx = stm->rx_done_idx;
		bd = &stm->rx_bd[idx];
		if (bd->stat & STM32_DMA_RBD_DMA_OWN)
			break;

		/*
		 * Get status
		 */
		stat = bd->stat;
		if (stat & STM32_DMA_RBD_ES) {
			stm->stat.rx_errors++;
			if (stat & STM32_DMA_RBD_LE)
				stm->stat.rx_length_errors++;
			if (stat & STM32_DMA_RBD_OE)
				stm->stat.rx_fifo_errors++;
			if (stat & STM32_DMA_RBD_CE)
				stm->stat.rx_crc_errors++;
			if ((stat & (STM32_DMA_RBD_FS | STM32_DMA_RBD_LS)) !=
			    (STM32_DMA_RBD_FS | STM32_DMA_RBD_LS)) {
				stm->stat.rx_frame_errors++;
			}

			goto next;
		}

		/*
		 * Get length, and strip CRC
		 */
		len = (bd->stat >> STM32_DMA_RBD_FL_BIT) & STM32_DMA_RBD_FL_MSK;
		len -= 4;

		/*
		 * Allocate skbuff, and put received data there
		 */
		skb = netdev_alloc_skb_ip_align(dev, len);
		if (unlikely(!skb)) {
			stm->stat.rx_dropped++;
			goto next;
		}

		dma_sync_single_for_cpu(NULL, stm->rx_bd[idx].buf,
					len, DMA_FROM_DEVICE);
		skb_copy_to_linear_data(skb, stm->rx_skb[idx]->data, len);
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, dev);

		netif_receive_skb(skb);

		stm->stat.rx_packets++;
		stm->stat.rx_bytes += len;

next:
		/*
		 * Allow DMA to use current BD again, and switch to the next BD
		 */
		bd->stat = STM32_DMA_RBD_DMA_OWN;
		stm->rx_done_idx = (stm->rx_done_idx + 1) % stm->rx_buf_num;
		processed++;
	}

	return processed;
}

/*
 * This is a NAPI method. Called to received next 'budget' frames (maximum)
 */
static int stm32_eth_rx_poll(struct napi_struct *napi, int budget)
{
	struct stm32_eth_priv	*stm = container_of(napi, struct stm32_eth_priv,
						    napi);
	struct net_device	*dev = stm->dev;
	unsigned long		flags;
	int			rx = 0, more;

	do {
		more = 0;

		rx = stm32_eth_rx_get(dev, rx, budget);
		if (!(rx < budget))
			break;

		/*
		 * Enable Rx interrupts, check if there's smth to rx, and if
		 * so, disable Rx interrupts, and fetch next frame
		 */
		spin_lock_irqsave(&stm->rx_lock, flags);
		__napi_complete(napi);

		stm->regs->dmaier |= STM32_MAC_DMAIER_TIE |
				     STM32_MAC_DMAIER_RIE;
		if (!(stm->rx_bd[stm->rx_done_idx].stat &
		      STM32_DMA_RBD_DMA_OWN)) {
			stm->regs->dmaier &= ~STM32_MAC_DMAIER_RIE;
			more = 1;
		}

		spin_unlock_irqrestore(&stm->rx_lock, flags);
	} while (more && napi_reschedule(napi));

	return rx;
}

/*
 * Process TX-complete interrupt
 */
static void stm32_eth_tx_complete(struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);

	/*
	 * Process all sent frames, and exit on the first not-sent yet
	 */
	spin_lock(&stm->tx_lock);
	while (stm->tx_pending) {
		volatile struct stm32_eth_dma_bd	*bd;
		u32					stat, idx;

		/*
		 * Get next bd, and check if its done
		 */
		idx = stm->tx_done_idx;
		bd = &stm->tx_bd[idx];

		stat = bd->stat;
		if (stat & STM32_DMA_RBD_DMA_OWN)
			break;

		if (stat & STM32_DMA_TBD_ES) {
			stm->stat.tx_errors++;
			if (stat & STM32_DMA_TBD_NC)
				stm->stat.tx_carrier_errors++;
			if (stat & STM32_DMA_TBD_UF)
				stm->stat.tx_fifo_errors++;
		} else {
			stm->stat.tx_packets++;
			stm->stat.tx_bytes += stm->tx_skb[idx]->len;
		}

		if (stat & STM32_DMA_TBD_EC) {
			stm->stat.collisions += 16;
		} else {
			stm->stat.collisions += (stat >> STM32_DMA_TBD_CC_BIT) &
						STM32_DMA_TBD_CC_MSK;
		}

		dma_unmap_single(&dev->dev, stm->tx_bd[idx].buf,
				 stm->tx_skb[idx]->len, DMA_TO_DEVICE);
		dev_kfree_skb_irq(stm->tx_skb[idx]);
		stm->tx_skb[idx] = NULL;

		stm->tx_pending--;
		stm->tx_done_idx = (stm->tx_done_idx + 1) % stm->tx_buf_num;
	}

	if (stm->tx_pending && stm->tx_restart) {
		stm->tx_restart = 0;
		stm->regs->dmatpdr = 0;
	}

	if (unlikely(stm->tx_blocked)) {
		stm->tx_blocked = 0;
		spin_unlock(&stm->tx_lock);

		netif_wake_queue(dev);
	} else {
		spin_unlock(&stm->tx_lock);
	}
}

/*
 * Common MAC interrupt handler
 */
static irqreturn_t stm32_eth_irq(int irq, void *dev_id)
{
	struct net_device	*dev;
	struct stm32_eth_priv	*stm;
	irqreturn_t		rv;
	u32			status;

	dev = dev_id;
	stm = netdev_priv(dev);

	/*
	 * Get status, and ack
	 */
	status = stm->regs->dmasr;
	stm->regs->dmasr = status;

	if (!status) {
		rv = IRQ_NONE;
		goto out;
	}

	if (status & STM32_MAC_DMASR_RX_MSK) {
		/*
		 * Process RX events
		 */
		if (status & STM32_MAC_DMASR_ROS)
			stm->stat.rx_over_errors++;

		spin_lock(&stm->rx_lock);
		if (likely(napi_schedule_prep(&stm->napi))) {
			/*
			 * Disable RX interrupts, and do NAPI
			 */
			stm->regs->dmaier &= ~STM32_MAC_DMAIER_RIE;
			__napi_schedule(&stm->napi);
		}
		spin_unlock(&stm->rx_lock);
	}

	if (status & STM32_MAC_DMASR_TX_MSK) {
		/*
		 * Process TX events
		 */
		if (status & STM32_MAC_DMASR_TUS)
			stm->stat.tx_fifo_errors++;
		if (status & STM32_MAC_DMASR_TBUS)
			stm->tx_restart = 1;

		stm32_eth_tx_complete(dev);
	}

	rv = IRQ_HANDLED;
out:
	return rv;
}

/******************************************************************************
 * Net device interface
 ******************************************************************************/

/*
 * Open net device
 */
static int stm32_netdev_open(struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);
	int			rv;

	if (stm32_eth_buffers_alloc(dev)) {
		rv = -ENOMEM;
		goto out;
	}

	napi_enable(&stm->napi);

	if (stm32_eth_hw_start(dev)) {
		napi_disable(&stm->napi);
		stm32_eth_buffers_free(dev);
		rv = -EIO;
		goto out;
	}

	spin_lock_init(&stm->rx_lock);
	spin_lock_init(&stm->tx_lock);

	stm->rx_done_idx = 0;
	stm->tx_todo_idx = 0;
	stm->tx_done_idx = 0;

	stm->tx_pending = 0;
	stm->tx_restart = 1;
	stm->tx_blocked = 0;

	rv = request_irq(stm->irq, stm32_eth_irq,
			 IRQF_DISABLED | IRQF_SAMPLE_RANDOM /* IRQF_SHARED */,
			 dev->name, dev);
	if (rv) {
		napi_disable(&stm->napi);
		stm32_eth_hw_stop(dev);
		stm32_eth_buffers_free(dev);
		goto out;
	}

	/*
	 * Enable interrupts
	 */
	stm->regs->dmaier |= STM32_MAC_DMAIER_TIE | STM32_MAC_DMAIER_RIE |
			     STM32_MAC_DMASR_NIS | STM32_MAC_DMASR_AIS;

	netif_start_queue(dev);
	rv = 0;
out:
	return rv;
}

/*
 * Close net device
 */
static int stm32_netdev_close(struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);

	napi_disable(&stm->napi);
	netif_stop_queue(dev);

	stm->regs->dmaier &= ~(STM32_MAC_DMAIER_TIE | STM32_MAC_DMAIER_RIE |
			       STM32_MAC_DMASR_NIS | STM32_MAC_DMASR_AIS);
	free_irq(stm->irq, dev);

	stm32_eth_hw_stop(dev);
	stm32_eth_buffers_free(dev);

	return 0;
}

/*
 * Send skb buffer with the net device
 */
static int stm32_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);
	unsigned long		flags;
	int			rv, idx;

	/*
	 * Check params
	 */
	if (unlikely(skb->len > stm->frame_max_size)) {
		stm->stat.tx_dropped++;
		dev_kfree_skb(skb);
		rv = NETDEV_TX_OK;
		goto out;
	}

	/*
	 * We need atomical access, only while 'capturing' next tx bd;
	 * then we may unlock the spin: if we'll be preempted, and some
	 * other call to xmit() happens, then the next 'skb', even being
	 * programmed to bds, won't be sent until we complete our captured
	 * bd (since our one is the first in the bd list)
	 */
	spin_lock_irqsave(&stm->tx_lock, flags);
	idx = stm->tx_todo_idx;
	if (stm->tx_pending + 1 > stm->tx_buf_num) {
		stm->tx_blocked = 1;
		spin_unlock_irqrestore(&stm->tx_lock, flags);

		netif_stop_queue(dev);
		rv = NETDEV_TX_BUSY;
		printk(STM32_INFO ": TX queue full\n");
		goto out;
	}
	stm->tx_pending++;
	stm->tx_todo_idx = (stm->tx_todo_idx + 1) % stm->tx_buf_num;
	spin_unlock_irqrestore(&stm->tx_lock, flags);

	dev->trans_start = jiffies;

	/*
	 * We have no limitations on the buffer address alignment
	 */
	stm->tx_skb[idx] = skb;

	stm->tx_bd[idx].ctrl  = skb->len;
	stm->tx_bd[idx].buf   = dma_map_single(&dev->dev, skb->data,
					       skb->len, DMA_TO_DEVICE);
	stm->tx_bd[idx].stat |= STM32_DMA_TBD_FS | STM32_DMA_TBD_LS |
				STM32_DMA_TBD_DMA_OWN;
	spin_lock_irqsave(&stm->tx_lock, flags);

	if (stm->tx_restart) {
		stm->tx_restart = 0;
		stm->regs->dmatpdr = 0;
	}
	spin_unlock_irqrestore(&stm->tx_lock, flags);

	rv = NETDEV_TX_OK;
out:
	return rv;
}

/*
 * Get net device statistic
 */
static struct net_device_stats *stm32_netdev_get_stats(struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);

	return &stm->stat;
}

/*
 * Handle ioctl request
 */
static int stm32_netdev_ioctl(struct net_device *dev, struct ifreq *ifr,
			      int cmd)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);
	struct mii_ioctl_data	*data = if_mii(ifr);

	return generic_mii_ioctl(&stm->mii, data, cmd, NULL);
}

/*
 * STM32 net device methods
 */
static const struct net_device_ops	stm32_netdev_ops = {
	.ndo_open		= stm32_netdev_open,
	.ndo_stop		= stm32_netdev_close,
	.ndo_start_xmit		= stm32_netdev_xmit,
	.ndo_get_stats		= stm32_netdev_get_stats,
	.ndo_do_ioctl		= stm32_netdev_ioctl,

	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
};

/******************************************************************************
 * MDIO interface
 ******************************************************************************/

/*
 * Write 'data' to 'reg' of PHY with 'phy_id' address, connected to net device
 */
static void stm32_mdio_write(struct net_device *dev, int phy_id,
			     int reg, int data)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);
	u32			tmp;

	tmp  = stm->regs->macmiiar;
	tmp &= STM32_MAC_MIIAR_CR_MSK << STM32_MAC_MIIAR_CR_BIT;

	phy_id &= STM32_MAC_MIIAR_PA_MSK;
	tmp |= phy_id << STM32_MAC_MIIAR_PA_BIT;

	reg &= STM32_MAC_MIIAR_MR_MSK;
	tmp |= reg << STM32_MAC_MIIAR_MR_BIT;

	tmp |= STM32_MAC_MIIAR_MW | STM32_MAC_MIIAR_MB;

	stm->regs->macmiidr = data;
	stm->regs->macmiiar = tmp;

	for (tmp = 0; tmp < 10; tmp++) {
		if (!(stm->regs->macmiiar & STM32_MAC_MIIAR_MB))
			break;
		msleep(1);
	}
	if (tmp == 10)
		printk(STM32_INFO ": mdio write timeout\n");
}

/*
 * Read data from 'reg' of PHY with 'phy_id' address, connected to net device
 */
static int stm32_mdio_read(struct net_device *dev, int phy_id, int reg)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);
	u32			tmp;
	int			data;

	tmp = stm->regs->macmiiar;
	tmp &= STM32_MAC_MIIAR_CR_MSK << STM32_MAC_MIIAR_CR_BIT;

	phy_id &= STM32_MAC_MIIAR_PA_MSK;
	tmp |= phy_id << STM32_MAC_MIIAR_PA_BIT;

	reg &= STM32_MAC_MIIAR_MR_MSK;
	tmp |= reg << STM32_MAC_MIIAR_MR_BIT;

	tmp |= STM32_MAC_MIIAR_MB;

	stm->regs->macmiiar = tmp;

	for (tmp = 0; tmp < 10; tmp++) {
		if (!(stm->regs->macmiiar & STM32_MAC_MIIAR_MB))
			break;
		msleep(1);
	}
	if (tmp != 10) {
		data = stm->regs->macmiidr;
	} else {
		printk(STM32_INFO ": mdio read timeout\n");
		data = 0xFFFF;
	}

	return data;
}

/******************************************************************************
 * Platform driver interface
 ******************************************************************************/

/*
 * Platform bus binding
 */
static int __devinit stm32_plat_probe(struct platform_device *pdev)
{
	struct stm32_eth_data	*data;
	struct stm32_eth_priv	*stm;
	struct net_device	*dev;
	struct resource		*rs;
	int			rv;

	/*
	 * Get the platform specific data
	 */
	if (!pdev) {
		printk(STM32_INFO ": no device specified\n");
		rv = -EINVAL;
		goto out;
	}
	data = pdev->dev.platform_data;

	/*
	 * Create ethernet device
	 */
	dev = alloc_etherdev(sizeof(struct stm32_eth_priv));
	if (!dev) {
		printk(STM32_INFO ": etherdev allocation failed\n");
		rv = -ENOMEM;
		goto out;
	}
	memcpy(dev->dev_addr, data->mac_addr, ETH_ALEN);
	dev->netdev_ops = &stm32_netdev_ops;

	stm = netdev_priv(dev);

	stm->dev = dev;
	netif_napi_add(dev, &stm->napi, stm32_eth_rx_poll, 64);

	platform_set_drvdata(pdev, dev);

	rs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rs) {
		printk(STM32_INFO ": MAC reg base isn't specified\n");
		rv = -EINVAL;
		goto out;
	}
	stm->regs = (struct stm32_mac_regs *)rs->start;

	rs = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!rs) {
		printk(STM32_INFO ": MAC IRQ isn't specified\n");
		rv = -EINVAL;
		goto out;
	}
	stm->irq = rs->start;

	/*
	 * Set-up driver parameters passed by user
	 */
	stm->frame_max_size = data->frame_max_size;
	if (stm->frame_max_size < 64 ||
	    stm->frame_max_size > STM32_DMA_RBD_FL_MSK) {
		printk(STM32_INFO ": incorrect frame_max_size param value\n");
		rv = -EINVAL;
		goto out;
	}

	stm->rx_buf_num = data->rx_buf_num;
	stm->tx_buf_num = data->tx_buf_num;
	if (!stm->tx_buf_num || !stm->rx_buf_num) {
		printk(STM32_INFO ": incorrect xx_buf_num param value\n");
		rv = -EINVAL;
		goto out;
	}

	/*
	 * Allocate bufs for pointers
	 */
	stm->rx_skb = kmalloc(stm->rx_buf_num * sizeof(void *), GFP_KERNEL);
	stm->tx_skb = kmalloc(stm->tx_buf_num * sizeof(void *), GFP_KERNEL);
	if (!stm->rx_skb || !stm->tx_skb) {
		printk(STM32_INFO ": rx/tx (%d/%d) bufs allocation failed\n",
			stm->rx_buf_num, stm->tx_buf_num);
		rv = -ENOMEM;
		goto out;
	}

	stm->mii.phy_id = data->phy_id;
	stm->mii.phy_id_mask = 0x1F;
	stm->mii.reg_num_mask = 0x1F;
	stm->mii.dev = dev;
	stm->mii.mdio_read  = stm32_mdio_read;
	stm->mii.mdio_write = stm32_mdio_write;

	if (is_zero_ether_addr(dev->dev_addr))
		random_ether_addr(dev->dev_addr);

	rv = register_netdev(dev);
	if (rv) {
		printk(STM32_INFO ": netdev registration failed\n");
		rv = -ENODEV;
		goto out;
	}

	rv = 0;
out:
	if (rv != 0)
		stm32_plat_remove(pdev);
	return rv;
}

/*
 * Platform bus unbinding
 */
static int stm32_plat_remove(struct platform_device *pdev)
{
	struct net_device	*dev;
	struct stm32_eth_priv	*stm;

	if (!pdev)
		goto out;

	dev = platform_get_drvdata(pdev);
	if (!dev)
		goto out;
	platform_set_drvdata(pdev, NULL);

	stm = netdev_priv(dev);

	unregister_netdev(dev);
	stm32_eth_buffers_free(dev);

	if (stm->tx_skb) {
		kfree(stm->tx_skb);
		stm->tx_skb = NULL;
	}

	if (stm->rx_skb) {
		kfree(stm->rx_skb);
		stm->rx_skb = NULL;
	}

	free_netdev(dev);
out:
	return 0;
}

/*
 * Platform driver instance
 */
static struct platform_driver	stm32_eth_driver = {
	.probe		= stm32_plat_probe,
	.remove		= stm32_plat_remove,
	.driver		= {
		.name	= STM32_ETH_DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

/*
 * Module init
 */
static int __init stm32_eth_drv_init(void)
{
	printk(STM32_INFO ": init\n");

	return platform_driver_register(&stm32_eth_driver);
}

/*
 * Module exit
 */
static void __exit stm32_eth_drv_exit(void)
{
	printk(STM32_INFO ": cleanup\n");
	platform_driver_unregister(&stm32_eth_driver);
}

module_init(stm32_eth_drv_init);
module_exit(stm32_eth_drv_exit);

MODULE_ALIAS("platform:" STM32_ETH_DRV_NAME);
MODULE_DESCRIPTION("STM32 MAC driver");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_LICENSE("GPL");
