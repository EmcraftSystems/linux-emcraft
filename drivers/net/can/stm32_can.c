/*
 * (C) Copyright 2016
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 */

/*
 * STM32 bxCAN driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/clk.h>

#include <linux/can.h>
#include <linux/can/dev.h>

#include <mach/clock.h>

/******************************************************************************
 * Constants and macros
 ******************************************************************************/
/*
 * Driver name
 */
#define DRV_NAME		"stm32_bxcan"

/*
 * Define this to enable additional debug prints
 */
#undef CAN_DEBUG

#ifdef CAN_DEBUG
# define CDBG(x...)		printk(x)
#else
# define CDBG(x...)		do { } while (0)
#endif

/*
 * Timeouts, msec
 */
#define STM_CAN_INIT_TOUT	100	/* Initialization */
#define STM_CAN_RX_ACK_TOUT	10	/* RX mail-box ack */

/*
 * NAPI rx quota
 */
#define STM_NAPI_WEIGTHT	(STM_MB_RX_NUM * 3)

/*
 * Mailbox number
 */
#define STM_MB_TX_NUM		3
#define STM_MB_RX_NUM		2

/*
 * Number of filters
 */
#define STM_FLT_NUM		28

/*
 * CANx are clocked with APB1 clock
 */
#define STM_CAN_CLK		CLOCK_PCLK1

/*
 * CAN master control register
 */
#define STM_CAN_MCR_BASE	0x000
#define STM_CAN_MCR_ABOM	(1 << 6)
#define STM_CAN_MCR_AWUM	(1 << 5)
#define STM_CAN_MCR_SLEEP	(1 << 1)
#define STM_CAN_MCR_INRQ	(1 << 0)

/*
 * CAN master status register
 */
#define STM_CAN_MSR_BASE	0x004
#define STM_CAN_MSR_SLAK	(1 << 1)
#define STM_CAN_MSR_INAK	(1 << 0)

/*
 * CAN transmit status register
 */
#define STM_CAN_TSR_BASE	0x008
#define STM_CAN_TSR_TXOK(mb)	(2 << ((mb) << 3))
#define STM_CAN_TSR_RQCP(mb)	(1 << ((mb) << 3))
#define STM_CAN_TSR_RQCP_ANY	(STM_CAN_TSR_RQCP(0) | STM_CAN_TSR_RQCP(1) | \
				 STM_CAN_TSR_RQCP(2))

/*
 * CAN receive FIFO 0/1 register
 */
#define STM_CAN_RFR_BASE(x)	(0x00C + ((x) << 2))
#define STM_CAN_RFR_RFOM	(1 << 5)
#define STM_CAN_RFR_FMP(x)	((x) & 0x3)

/*
 * CAN interrupt enable register
 */
#define STM_CAN_IER_BASE	0x014
#define STM_CAN_IER_ERRIE	(1 << 15)
#define STM_CAN_IER_FMPIE(x)	(1 << (1 + ((x) * 3)))
#define STM_CAN_IER_TMEIE	(1 << 0)

/*
 * CAN bit timing register
 */
#define STM_CAN_BTR_BASE	0x01C
#define STM_CAN_BTR_SILM	(1 << 31)
#define STM_CAN_BTR_LBKM	(1 << 30)
#define STM_CAN_BTR_SJW(x)	((x) << 24)
#define STM_CAN_BTR_TS2(x)	((x) << 20)
#define STM_CAN_BTR_TS1(x)	((x) << 16)
#define STM_CAN_BTR_BRP(x)	((x) << 0)

/*
 * CAN TX/RX mailbox registers: identifier, data length control and time stamp,
 * data low/high
 */
#define STM_CAN_TIR_BASE(x)	(0x180 + ((x) << 4))
#define STM_CAN_RIR_BASE(x)	(0x1B0 + ((x) << 4))
#define STM_CAN_xIR_STID_SET(x)	(((x) & 0x7FF) << 21)
#define STM_CAN_xIR_EXID_SET(x)	(((x) & 0x1FFFFFFF) << 3)
#define STM_CAN_xIR_STID_GET(x)	(((x) >> 21) & 0x7FF)
#define STM_CAN_xIR_EXID_GET(x)	(((x) >> 3) & 0x1FFFFFFF)
#define STM_CAN_xIR_IDE		(1 << 2)
#define STM_CAN_xIR_RTR		(1 << 1)
#define STM_CAN_TIR_TXRQ	(1 << 0)

#define STM_CAN_TDTR_BASE(x)	(0x184 + ((x) << 4))
#define STM_CAN_RDTR_BASE(x)	(0x1B4 + ((x) << 4))
#define STM_CAN_xDTR_DLC_SET(x)	(((x) & 0xF) << 0)
#define STM_CAN_xDTR_DLC_GET(x)	(((x) >> 0) & 0xF)

#define STM_CAN_TDLR(x)		(0x188 + ((x) << 4))
#define STM_CAN_TDHR(x)		(0x18C + ((x) << 4))
#define STM_CAN_RDLR(x)		(0x1B8 + ((x) << 4))
#define STM_CAN_RDHR(x)		(0x1BC + ((x) << 4))

/*
 * CAN filter registers
 */
#define STM_CAN_FMR_BASE	0x200
#define STM_CAN_FMR_FINIT	(1 << 0)

#define STM_CAN_FM1R_BASE	0x204
#define STM_CAN_FM1R_IM(x)	(0 << (x))
#define STM_CAN_FM1R_MSK(x)	(1 << (x))

#define STM_CAN_FS1R_BASE	0x20C
#define STM_CAN_FS1R_32BIT(x)	(1 << (x))
#define STM_CAN_FS1R_MSK(x)	(1 << (x))

#define STM_CAN_FFA1R_BASE	0x214
#define STM_CAN_FFA1R_FIFO0(x)	(0 << (x))
#define STM_CAN_FFA1R_MSK(x)	(1 << (x))

#define STM_CAN_FA1R_BASE	0x21C
#define STM_CAN_FA1R_FACT(x)	(1 << (x))

#define STM_CAN_FIR1_BASE(x)	(0x240 + ((x) << 3))
#define STM_CAN_FIR2_BASE(x)	(0x244 + ((x) << 3))

/******************************************************************************
 * C-types
 ******************************************************************************/
/*
 * STM32 CAN private
 */
struct stm32_prv {
	struct can_priv			can;
	struct net_device		*ndev;
	struct device			*dev;
	void __iomem			*reg;
	struct napi_struct		napi;
	u32				rx_next;
};

/******************************************************************************
 * Local variables and function prototypes
 ******************************************************************************/
/*
 * CAN harware-dependent bit-timing constant
 * Used for calculating and checking bit-timing parameters
 */
static struct can_bittiming_const stm32_bittiming_const = {
	.tseg1_min	= 1,	/* Time segement 1 = prop_seg + phase_seg1 */
	.tseg1_max	= 16,
	.tseg2_min	= 1,	/* Time segement 2 = phase_seg2 */
	.tseg2_max	= 8,
	.sjw_max	= 4,	/* Synchronisation jump width */
	.brp_min	= 1,	/* Bit-rate prescaler */
	.brp_max	= 1024,
	.brp_inc	= 1,
};

/******************************************************************************
 * Functions local to this module
 ******************************************************************************/

/*
 * Dump CAN registers
 */
static void __maybe_unused stm32_can_dump(const char *who, struct stm32_prv *prv)
{
	int i;

	if (who)
		printk("%s CAN registers dump:\n", who);

	printk( " MCR=%08x;MSR=%08x;TSR=%08x;RF0R=%08x;\n"
		" IER=%08x;ESR=%08x;BTR=%08x;RF1R=%08x;\n",
		readl(prv->reg + 0x000), readl(prv->reg + 0x004),
		readl(prv->reg + 0x008), readl(prv->reg + 0x00C),
		readl(prv->reg + 0x014), readl(prv->reg + 0x018),
		readl(prv->reg + 0x01C), readl(prv->reg + 0x010));

	for (i = 0; i < STM_MB_TX_NUM; i++) {
		printk(" TX%d IR=%08x;DT=%08x;DL=%08x;DH=%08x;\n", i,
			readl(prv->reg + 0x180 + (i << 4)),
			readl(prv->reg + 0x184 + (i << 4)),
			readl(prv->reg + 0x188 + (i << 4)),
			readl(prv->reg + 0x18C + (i << 4)));
	}

	for (i = 0; i < STM_MB_RX_NUM; i++) {
		printk(" RX%d IR=%08x;DT=%08x;DL=%08x;DH=%08x;\n", i,
			readl(prv->reg + 0x1B0 + (i << 4)),
			readl(prv->reg + 0x1B4 + (i << 4)),
			readl(prv->reg + 0x1B8 + (i << 4)),
			readl(prv->reg + 0x1BC + (i << 4)));
	}

	printk( " FMR=%08x;FM1R=%08x;FS1R=%08x;FFA1R=%08x;FA1R=%08x;\n",
		readl(prv->reg + 0x200), readl(prv->reg + 0x204),
		readl(prv->reg + 0x20C), readl(prv->reg + 0x214),
		readl(prv->reg + 0x21C));

	for (i = 0; i < (STM_FLT_NUM / 2); i++) {
		printk(" F%02d=%08x.%08x;F%02d=%08x.%08x\n",
			(i << 1),
			readl(prv->reg + 0x240 + (i << 4)),
			readl(prv->reg + 0x244 + (i << 4)),
			(i << 1) + 1,
			readl(prv->reg + 0x248 + (i << 4)),
			readl(prv->reg + 0x24C + (i << 4)));
	}
}

/*
 * Enter/Exit CAN Initialization mode
 * @prv: stm32 can private
 * @in: 1 - enter Init; 0 - exit Init
 */
static int stm32_chip_init(struct stm32_prv *prv, int in)
{
	u32 val;
	int i;

	val = readl(prv->reg + STM_CAN_MCR_BASE);
	if (in)
		val |= STM_CAN_MCR_INRQ;
	else
		val &= ~STM_CAN_MCR_INRQ;

	writel(val, prv->reg + STM_CAN_MCR_BASE);
	for (i = 0; i < STM_CAN_INIT_TOUT; i++) {
		val = readl(prv->reg + STM_CAN_MSR_BASE) & STM_CAN_MSR_INAK;
		if (in) {
			if (val)
				break;
		} else {
			if (!val)
				break;
		}

		usleep_range(1000, 1000);
	}

	val = readl(prv->reg + STM_CAN_MSR_BASE) & STM_CAN_MSR_INAK;

	return in ? (val ? 0 : -ETIMEDOUT) : (!val ? 0 : -ETIMEDOUT);
}

static int stm32_chip_start(struct net_device *ndev)
{
	struct stm32_prv *prv = netdev_priv(ndev);
	u32 val;
	int i, rv;

	/*
	 * Switch Sleep -> Initialization mode
	 */
	val = readl(prv->reg + STM_CAN_MCR_BASE);
	val &= ~STM_CAN_MCR_SLEEP;
	val |= STM_CAN_MCR_INRQ;
	writel(val, prv->reg + STM_CAN_MCR_BASE);
	for (i = 0; i < STM_CAN_INIT_TOUT; i++) {
		val = readl(prv->reg + STM_CAN_MSR_BASE);
		val &= STM_CAN_MSR_INAK | STM_CAN_MSR_SLAK;
		if (val == STM_CAN_MSR_INAK)
			break;
		usleep_range(1000, 1000);
	}
	val = readl(prv->reg + STM_CAN_MSR_BASE);
	val &= STM_CAN_MSR_INAK | STM_CAN_MSR_SLAK;
	if (val != STM_CAN_MSR_INAK) {
		rv = -ETIMEDOUT;
		goto out;
	}

	prv->can.state = CAN_STATE_ERROR_ACTIVE;

	/*
	 * Switch to Normal mode
	 */
	rv = stm32_chip_init(prv, 0);
	if (rv)
		goto out;

	/*
	 * Set-up default rx filter:
	 * - use filter #0
	 * - Identifier/Mask mode
	 * - 32-bit scale
	 * - assign to RX FIFO0
	 * - receive all
	 */
	i = 0;
	val = readl(prv->reg + STM_CAN_FMR_BASE);
	writel(val | STM_CAN_FMR_FINIT, prv->reg + STM_CAN_FMR_BASE);

	val = readl(prv->reg + STM_CAN_FA1R_BASE);
	writel(val & ~STM_CAN_FA1R_FACT(i), prv->reg + STM_CAN_FA1R_BASE);

	val = readl(prv->reg + STM_CAN_FM1R_BASE) & ~STM_CAN_FM1R_MSK(i);
	writel(val | STM_CAN_FM1R_IM(i), prv->reg + STM_CAN_FM1R_BASE);

	val = readl(prv->reg + STM_CAN_FS1R_BASE);
	writel(val | STM_CAN_FS1R_32BIT(i), prv->reg + STM_CAN_FS1R_BASE);

	val = readl(prv->reg + STM_CAN_FFA1R_BASE) & ~STM_CAN_FFA1R_MSK(i);
	writel(val | STM_CAN_FFA1R_FIFO0(i), prv->reg + STM_CAN_FFA1R_BASE);

	writel(0, prv->reg + STM_CAN_FIR1_BASE(i));
	writel(0, prv->reg + STM_CAN_FIR2_BASE(i));

	val = readl(prv->reg + STM_CAN_FA1R_BASE);
	writel(val | STM_CAN_FA1R_FACT(i), prv->reg + STM_CAN_FA1R_BASE);

	val = readl(prv->reg + STM_CAN_FMR_BASE);
	writel(val & ~STM_CAN_FMR_FINIT, prv->reg + STM_CAN_FMR_BASE);

	/*
	 * Enable interrupts
	 */
	val = STM_CAN_IER_TMEIE | STM_CAN_IER_FMPIE(0) | STM_CAN_IER_FMPIE(1);
	writel(val, prv->reg + STM_CAN_IER_BASE);

out:
	if (rv)
		dev_err(prv->dev, "%s: error %d\n", __func__, rv);

	return rv;
}

static int stm32_chip_stop(struct net_device *ndev, enum can_state state)
{
	struct stm32_prv *prv = netdev_priv(ndev);
	u32 val;
	int i, rv;

	/*
	 * Switch Sleep mode
	 */
	val = readl(prv->reg + STM_CAN_MCR_BASE);
	val |= STM_CAN_MCR_SLEEP;
	writel(val, prv->reg + STM_CAN_MCR_BASE);
	for (i = 0; i < STM_CAN_INIT_TOUT; i++) {
		if (readl(prv->reg + STM_CAN_MSR_BASE) & STM_CAN_MSR_SLAK)
			break;
	}
	if (!(readl(prv->reg + STM_CAN_MSR_BASE) & STM_CAN_MSR_SLAK)) {
		rv = -ETIMEDOUT;
		goto out;
	}

	prv->can.state = state;
	rv = 0;
out:
	if (rv)
		dev_err(prv->dev, "%s: error %d\n", __func__, rv);

	return rv;
}

static irqreturn_t stm32_irq(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct net_device_stats *stats = &ndev->stats;
	struct stm32_prv *prv = netdev_priv(ndev);
	u32 i, val;

	CDBG("irq: rfr0=%08x,rfr1=%08x,tsr=%08x\n",
		readl(prv->reg + STM_CAN_RFR_BASE(0)),
		readl(prv->reg + STM_CAN_RFR_BASE(1)),
		readl(prv->reg + STM_CAN_TSR_BASE));

	/*
	 * Process RX
	 */
	for (i = 0; i < STM_MB_RX_NUM; i++) {
		val = readl(prv->reg + STM_CAN_RFR_BASE(i));
		if (!STM_CAN_RFR_FMP(val))
			continue;

		val = readl(prv->reg + STM_CAN_IER_BASE);
		val &= ~STM_CAN_IER_FMPIE(i);
		writel(val, prv->reg + STM_CAN_IER_BASE);

		napi_schedule(&prv->napi);
	}

	/*
	 * Process TX
	 */
	val = readl(prv->reg + STM_CAN_TSR_BASE);
	for (i = 0; i < STM_MB_TX_NUM; i++) {
		if (!(val & STM_CAN_TSR_RQCP(i)))
			continue;

		can_get_echo_skb(ndev, i);

		stats->tx_packets++;
		stats->tx_bytes += STM_CAN_xDTR_DLC_GET(
					readl(prv->reg + STM_CAN_TDTR_BASE(i)));

		writel(STM_CAN_TSR_RQCP(i), prv->reg + STM_CAN_TSR_BASE);
		netif_wake_queue(ndev);
	}

	return IRQ_HANDLED;
}

/******************************************************************************
 * CAN network driver interface
 ******************************************************************************/

static int stm32_set_bittiming(struct net_device *ndev)
{
	struct stm32_prv *prv = netdev_priv(ndev);
	struct can_bittiming *bt = &prv->can.bittiming;
	u32 val;
	int rv;

	/* Enter Initialization mode */
	rv = stm32_chip_init(prv, 1);
	if (rv) {
		dev_err(prv->dev, "enter initialization mode err (%d)\n", rv);
		goto out;
	}

	/* Configure BTR */
	val = readl(prv->reg + STM_CAN_BTR_BASE);
	val &= STM_CAN_BTR_SILM | STM_CAN_BTR_LBKM;
	val |= STM_CAN_BTR_SJW(bt->sjw - 1) |
	       STM_CAN_BTR_TS1(bt->prop_seg + bt->phase_seg1 - 1) |
	       STM_CAN_BTR_TS2(bt->phase_seg2 - 1) |
	       STM_CAN_BTR_BRP(bt->brp - 1);
	writel(val, prv->reg + STM_CAN_BTR_BASE);

	/* Exit Initialization mode */
	rv = stm32_chip_init(prv, 0);
	if (rv)
		dev_err(prv->dev, "exit initialization mode err (%d)\n", rv);
out:
	CDBG("%s: brp=%d,sjw=%d,prop=%d,ps1=%d,ps2=%d,rv=%d\n", __func__,
		bt->brp, bt->sjw, bt->prop_seg, bt->phase_seg1, bt->phase_seg2,
		rv);

	return rv;
}

static int stm32_set_mode(struct net_device *dev, enum can_mode mode)
{
	int	rv;

	switch (mode) {
	case CAN_MODE_START:
		rv = stm32_chip_start(dev);
		if (rv)
			break;

		netif_wake_queue(dev);
		rv = 0;
		break;
	default:
		rv = -EOPNOTSUPP;
		break;
	}

	return rv;
}

/******************************************************************************
 * Network device interface
 ******************************************************************************/

static int stm32_open(struct net_device *ndev)
{
	struct stm32_prv *prv = netdev_priv(ndev);
	struct device *dev = prv->dev;
	int rv, i;

	/*
	 * Check/determine and set bittime
	 */
	rv = open_candev(ndev);
	if (rv) {
		dev_err(dev, "open_candev() err (%d)\n", rv);
		goto out;
	}

	/*
	 * Register interrupt handler
	 */
	for (i = 0; i < 3; i++) {
		if (request_irq(ndev->irq + i, stm32_irq, IRQF_SHARED,
				ndev->name, ndev)) {
			dev_err(dev, "request_irq(%d) fail\n", ndev->irq + i);
			rv = -EAGAIN;
			goto out_candev;
		}
	}

	/*
	 * Start chip and queuing
	 */
	stm32_chip_start(ndev);
	napi_enable(&prv->napi);
	netif_start_queue(ndev);

	rv = 0;
	goto out;

out_candev:
	close_candev(ndev);
out:
	return rv;
}

static int stm32_close(struct net_device *ndev)
{
	struct stm32_prv *prv = netdev_priv(ndev);
	int i, rv;

	netif_stop_queue(ndev);
	napi_disable(&prv->napi);
	rv = stm32_chip_stop(ndev, CAN_STATE_STOPPED);
	if (rv)
		goto out;

	for (i = 0; i < 3; i++)
		free_irq(ndev->irq + i, ndev);

	close_candev(ndev);
	rv = 0;
out:
	return rv;
}

static netdev_tx_t stm32_start_xmit(struct sk_buff *skb,
				    struct net_device *ndev)
{
	struct stm32_prv *prv = netdev_priv(ndev);
	struct can_frame *cf = (void *)skb->data;
	netdev_tx_t rv;
	u32 val;
	int mb;

	/*
	 * Get empty TX mailbox
	 */
	for (mb = 0; mb < STM_MB_TX_NUM; mb++) {
		val = readl(prv->reg + STM_CAN_TIR_BASE(mb));
		if (!(val & STM_CAN_TIR_TXRQ))
			break;
	}
	if (!(mb < STM_MB_TX_NUM)) {
		rv = NETDEV_TX_BUSY;
		goto out;
	}

	can_put_echo_skb(skb, ndev, mb);

	/*
	 * Program mailbox, and trigger transmission
	 */
	writel(STM_CAN_xDTR_DLC_SET(cf->can_dlc),
		prv->reg + STM_CAN_TDTR_BASE(mb));
	writel(*(u32 *)(cf->data + 0), prv->reg + STM_CAN_TDLR(mb));
	writel(*(u32 *)(cf->data + 4), prv->reg + STM_CAN_TDHR(mb));

	if (cf->can_id & CAN_EFF_FLAG) {
		val = STM_CAN_xIR_EXID_SET(cf->can_id & CAN_EFF_MASK) |
		      STM_CAN_xIR_IDE;
	} else {
		val = STM_CAN_xIR_STID_SET(cf->can_id & CAN_SFF_MASK);
	}
	if (cf->can_id & CAN_RTR_FLAG)
		val |= STM_CAN_xIR_RTR;

	writel(val | STM_CAN_TIR_TXRQ, prv->reg + STM_CAN_TIR_BASE(mb));

	rv = NETDEV_TX_OK;
out:
	CDBG("%s: dlc=%d,mb=%d,rv=%d\n", __func__, cf->can_dlc, mb, rv);

	return rv;
}

static int stm32_poll(struct napi_struct *napi, int quota)
{
	struct net_device *ndev = napi->dev;
	struct net_device_stats *stats = &ndev->stats;
	struct stm32_prv *prv = netdev_priv(ndev);
	struct device *dev = prv->dev;
	struct can_frame *cf;
	struct sk_buff *skb;
	unsigned long flags;
	u32 val, i, x, t;
	int done;

	i = done = 0;
	while (done < quota && i < STM_MB_RX_NUM) {
		x = prv->rx_next;
		val = readl(prv->reg + STM_CAN_RFR_BASE(x));
		if (!STM_CAN_RFR_FMP(val)) {
			/*
			 * Switch to the next RxFIFO
			 */
			prv->rx_next = (prv->rx_next + 1) % STM_MB_RX_NUM;
			i++;
			continue;
		}

		skb = alloc_can_skb(ndev, &cf);
		if (unlikely(!skb)) {
			stats->rx_dropped++;
			break;
		}

		/*
		 * Read out the packet from the mail-box
		 */
		val = readl(prv->reg + STM_CAN_RIR_BASE(x));
		if (val & STM_CAN_xIR_IDE) {
			cf->can_id = STM_CAN_xIR_EXID_GET(val) & CAN_EFF_MASK;
			cf->can_id |= CAN_EFF_FLAG;
		} else {
			cf->can_id = STM_CAN_xIR_STID_GET(val) & CAN_SFF_MASK;
		}

		val = readl(prv->reg + STM_CAN_RDTR_BASE(x));
		cf->can_dlc = STM_CAN_xDTR_DLC_GET(val);

		*(u32 *)(cf->data + 0) = readl(prv->reg + STM_CAN_RDLR(x));
		*(u32 *)(cf->data + 4) = readl(prv->reg + STM_CAN_RDHR(x));

		/*
		 * Push packet to the upper layer
		 */
		netif_receive_skb(skb);
		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;

		done++;

		/*
		 * Ack mail-box
		 */
		writel(STM_CAN_RFR_RFOM, prv->reg + STM_CAN_RFR_BASE(x));
		for (t = 0; t < STM_CAN_RX_ACK_TOUT; t++) {
			val = readl(prv->reg + STM_CAN_RFR_BASE(x));
			if (!(val & STM_CAN_RFR_RFOM))
				break;
			usleep_range(1000, 1000);
		}
		if (readl(prv->reg + STM_CAN_RFR_BASE(i)) & STM_CAN_RFR_RFOM) {
			dev_err(dev, "rx%d ack timeout (%x)\n", x,
				readl(prv->reg + STM_CAN_RFR_BASE(x)));
			break;
		}
	}

	if (done == quota)
		goto more;

	/*
	 * If there are no more packets in RX FIFOs, then complete NAPI, and
	 * enable interrupts
	 */
	for (i = 0; i < STM_MB_RX_NUM; i++) {
		if (STM_CAN_RFR_FMP(readl(prv->reg + STM_CAN_RFR_BASE(i))))
			goto more;
	}

	napi_complete(napi);

	local_irq_save(flags);
	val = readl(prv->reg + STM_CAN_IER_BASE);
	for (i = 0; i < STM_MB_RX_NUM; i++)
		val |= STM_CAN_IER_FMPIE(i);
	writel(val, prv->reg + STM_CAN_IER_BASE);
	local_irq_restore(flags);

more:
	CDBG("%s: done=%d,quot=%d\n", __func__, done, quota);
	return done;
}

static const struct net_device_ops stm32_netdev_ops = {
	.ndo_open	= stm32_open,
	.ndo_stop	= stm32_close,
	.ndo_start_xmit	= stm32_start_xmit,
};

/******************************************************************************
 * Platform driver interface
 ******************************************************************************/

static int __init stm32_can_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct stm32_prv *prv;
	struct resource *res;
	struct device *dev;
	void __iomem *addr;
	int rv, irq;

	dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || irq <= 0) {
		dev_err(dev, "bad reg/irq %p/%d\n", res, irq);
		rv = -ENODEV;
		goto out;
	}

	if (!request_mem_region(res->start,
				resource_size(res), pdev->name)) {
		dev_err(dev, "request_mem(0x%x,0x%x,%s) fail\n", res->start,
			resource_size(res), pdev->name);
		rv = -EBUSY;
		goto out;
	}

	addr = ioremap_nocache(res->start, resource_size(res));
	if (!addr) {
		dev_err(dev, "ioremap(0x%x,0x%x) fail\n", res->start,
			resource_size(res));
		rv = -ENOMEM;
		goto out_release;
	}

	ndev = alloc_candev(sizeof(struct stm32_prv), STM_MB_TX_NUM);
	if (!ndev) {
		dev_err(dev, "alloc_candev(%d,%d) fail\n",
			sizeof(struct stm32_prv), STM_MB_TX_NUM);
		rv = -ENOMEM;
		goto out_unmap;
	}

	ndev->netdev_ops = &stm32_netdev_ops;
	ndev->irq = irq;
	ndev->flags |= IFF_ECHO;

	prv = netdev_priv(ndev);
	prv->can.clock.freq = stm32_clock_get(STM_CAN_CLK);
	prv->can.bittiming_const = &stm32_bittiming_const;
	prv->can.do_set_bittiming = stm32_set_bittiming;
	prv->can.do_set_mode = stm32_set_mode;
	prv->reg = addr;
	prv->ndev = ndev;
	prv->dev = dev;

	netif_napi_add(ndev, &prv->napi, stm32_poll, STM_NAPI_WEIGTHT);

	dev_set_drvdata(dev, ndev);
	SET_NETDEV_DEV(ndev, dev);

	rv = register_candev(ndev);
	if (rv) {
		dev_err(dev, "registering netdev failed (%d)\n", rv);
		goto out_free;
	}

	dev_info(dev, "device registered (reg=%p, irq=%d)\n", addr, irq);
	rv = 0;

	goto out;

out_free:
	free_netdev(ndev);
out_unmap:
	iounmap(addr);
out_release:
	release_mem_region(res->start, resource_size(res));
out:
	return rv;
}

static int __devexit stm32_can_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stm32_prv *prv = netdev_priv(ndev);
	struct resource *res;

	unregister_netdev(ndev);
	platform_set_drvdata(pdev, NULL);
	free_netdev(ndev);
	iounmap(prv->reg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	return 0;
}

static struct platform_driver stm32_can_driver = {
	.probe		= stm32_can_probe,
	.remove		= __devexit_p(stm32_can_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

/******************************************************************************
 * Kernel module interface
 ******************************************************************************/

static int __init stm32_can_module_init(void)
{
	printk(KERN_INFO "%s netdevice driver\n", DRV_NAME);
	return platform_driver_register(&stm32_can_driver);
}

static void __exit stm32_can_module_exit(void)
{
	platform_driver_unregister(&stm32_can_driver);
	printk(KERN_INFO "%s: driver removed\n", DRV_NAME);
}

module_init(stm32_can_module_init);
module_exit(stm32_can_module_exit);

MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRV_NAME " CAN netdevice driver");
