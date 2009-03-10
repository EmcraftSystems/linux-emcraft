/* 
   Copyright (C) 2004-2007 ARM Limited.

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License version 2
   as published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   As a special exception, if other files instantiate templates or use macros
   or inline functions from this file, or you compile this file and link it
   with other works to produce a work based on this file, this file does not
   by itself cause the resulting work to be covered by the GNU General Public
   License. However the source code for this file must still be made available
   in accordance with section (3) of the GNU General Public License.

   This exception does not invalidate any other reasons why a work based on
   this file might be covered by the GNU General Public License.
*/

#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <net/route.h>
#include <asm/hardware/rvidcc.h>

#ifdef CONFIG_DEBUG_DCC_KGDB
#include <linux/kgdb.h>
#endif

#include "rvidcc_config.h"
#include "rvidcc_interface.h"

#define ARM_DCC_VER   "1.0"
#define ARM_TTY_NAME  "ttyDCC"
#define ARM_ETH_NAME  "ethDCC"
#define ARM_DCC_MAJOR 204
#define ARM_DCC_MINOR 46
#define ARM_DCC_POLL_PERIOD 1 /* jiffies per poll: used in timed polled (non-interrupt) mode */

/* SMP not supported */
#ifdef CONFIG_SMP
#error DCC module does not support SMP
#endif

/* Interrupt defininitons for known achitectures that have them */
#ifdef CONFIG_ARCH_INTEGRATOR

#define DCC_RX_IRQ IRQ_CM_COMMRX
#define DCC_TX_IRQ IRQ_CM_COMMTX

#elif defined(CONFIG_ARCH_L7200)

#define DCC_RX_IRQ IRQ_DEBUG_RX
#define DCC_TX_IRQ IRQ_DEBUG_TX

#elif defined(CONFIG_ARCH_LH7A404)

#define DCC_RX_IRQ IRQ_COMMRX
#define DCC_TX_IRQ IRQ_COMMTX

#elif defined(CONFIG_ARCH_VERSATILE_PB)

#define DCC_RX_IRQ IRQ_COMMRx
#define DCC_TX_IRQ IRQ_COMMTx

#else

#define DCC_RX_IRQ 0
#define DCC_TX_IRQ 0

#endif

/* This section contains the primary OS driver functions and definitions */

/* Structure containing our internal state, etc. */

struct dccconfig dcc_config =
{
  .rx_interrupt = DCC_RX_IRQ,
  .tx_interrupt = DCC_TX_IRQ,
  .use_timer_poll = 1, /* use timer poll until interrupts are enabled (if ever) */
  .timer_poll_period = ARM_DCC_POLL_PERIOD
};

struct dccinfo
{
  struct tty_struct *tty;
  int tty_opens;
  int init_called;
  int kernel_in_panic;
  volatile unsigned long timerpoll_lock;
  volatile unsigned long tx_locked;
#ifndef CONFIG_DEBUG_DCC_RAW
  struct net_device *netdev;
  char mac_address[ETH_ALEN];
  struct sk_buff *pending_skb;
#endif
};
static struct dccinfo dcc_info;

/* Module parameters - IRQ's etc. an be set on the boot line */
module_param_named(rxirq, dcc_config.rx_interrupt, uint, 0444);
module_param_named(txirq, dcc_config.tx_interrupt, uint, 0444);
module_param_named(pollperiod, dcc_config.timer_poll_period, ulong, 0444);
MODULE_PARM_DESC(rxirq, "DCC receive IRQ number");
MODULE_PARM_DESC(txirq, "DCC transmit IRQ number");
MODULE_PARM_DESC(pollperiod, "DCC timer polling period");

/* Tasklet used as a "bottom half" - services data collected by interrupt or timed poll */
static void dcc_tasklet_action(unsigned long data);
static DECLARE_TASKLET(dcc_tasklet, dcc_tasklet_action, 0);
static void dcc_poll(void);

/*
 * Write data on the serial channel if no other write is in progress
 */
static size_t dcc_write_serial(const void *ptr, size_t len)
{
  size_t res = 0;

  if (!test_and_set_bit(0, &dcc_info.tx_locked))
  {
    res = rvidcc_write_serial(ptr, len);
    clear_bit(0, &dcc_info.tx_locked);
  }

  return res;
}

#ifndef RVIDCC_RAW
/*
 * Send a network packet if no other write is in progress
 */
static int dcc_transmit_ip_packet(void *packet, size_t length)
{
  int res = 0;

  if (!test_and_set_bit(0, &dcc_info.tx_locked))
  {
    res = rvidcc_transmit_ip_packet(packet, length);
    clear_bit(0, &dcc_info.tx_locked);
  }

  return res;
}
#endif

/* KGDB support
 *
 * KGDB and the console/tty channel are mutually exclusive, 
 * i.e. KGDB takes over the tty channel if it is enabled.
 */
#ifdef CONFIG_DEBUG_DCC_KGDB

#define KGDB_BUF_SIZE 1024 /* needs to be big enough to hold one GDB packet */
static int dcc_init_dcc(void);
static void dcc_print_banner(void);
static void dcc_tasklet_action(unsigned long data);

static char kgdb_buf[KGDB_BUF_SIZE];
static size_t kgdb_buf_chars = 0;

/* KGDB getchar routine */
static int kgdb_getDebugChar(void)
{
  char ch;

  /* The rest of the system is frozen at this point, so this is all that is running */
  while (!rvidcc_read_serial(&ch, 1))
  {
    rvidcc_poll();
    dcc_tasklet_action(1);
  }

  return (int)ch;
}

/* KGDB flush routine - ensures each GDB packet is sent as one message (more efficient) */
static void kgdb_flushDebugChar(void)
{
  size_t written = 0;

  /* The rest of the system is frozen at this point, so this is all that is running */
  while (written < kgdb_buf_chars)
  {
    written += dcc_write_serial(&kgdb_buf[written], kgdb_buf_chars - written);
    rvidcc_poll();
    dcc_tasklet_action(1);
  }

  kgdb_buf_chars = 0;
}

/* KGDB putchar routine */
static void kgdb_putDebugChar(int chr)
{
  kgdb_buf[kgdb_buf_chars++] = (char)chr;
  if (kgdb_buf_chars >= KGDB_BUF_SIZE)
    kgdb_flushDebugChar();
}

/* KGDB initialisation routine - can be called very early in kernel startup */
static int init_kgdbDcc(void)
{
  int err = 0;
  if (!dcc_info.init_called)
  {
    dcc_print_banner();
    err = dcc_init_dcc();
    if (!err)
      printk(KERN_INFO "kgdb: debugging over DCC enabled\n");
  }

  return err;
}

/* KGDB device driver structure - only one of these is allowed globally */
struct kgdb_io kgdb_io_ops =
{
  .read_char = kgdb_getDebugChar,
  .write_char = kgdb_putDebugChar,
  .init = init_kgdbDcc,
  .flush = kgdb_flushDebugChar
};

#else

/* TTY device driver routines. Must peacefully co-exist with console support.
 *
 * Not present when KGDB is enabled. KGDB has its own driver format (above).
 */
/* TTY device open routine */
static int dcc_chr_open(struct tty_struct *tty, struct file *filp)
{
  if (!dcc_info.tty_opens++)
    dcc_info.tty = tty;

  return 0;
}

/* TTY device close routine */
static void dcc_chr_close(struct tty_struct *tty, struct file *filp)
{
  if (!--dcc_info.tty_opens)
    dcc_info.tty = NULL;
}

/* TTY device write routine */
static int dcc_chr_write(struct tty_struct * tty,
                         const unsigned char *buf, int count)
{
  int bytes = 0;

  if (count == 0)
    return 0;

  bytes = (int)dcc_write_serial(buf, count);

  if (dcc_config.use_timer_poll)
    dcc_poll();

  return bytes;
}

/* TTY device routine to determine space left in output buffer */
static int dcc_chr_write_room(struct tty_struct *tty)
{
  int space;

  if (dcc_config.use_timer_poll)
    dcc_poll();

  space = rvidcc_serial_can_write() - 4 ; /* Leave 1 word spare for sentinel*/
  return space > 0 ? space : 0;
}

/* TTY device routine to determine number of chars waiting in input buffer */
static int dcc_chr_chars_in_buffer(struct tty_struct *tty)
{
  if (dcc_config.use_timer_poll)
    dcc_poll();

  return rvidcc_serial_can_read();
}

/* TTY device ioctl routine. Global and affects virtual Ethernet as well */
static int dcc_chr_ioctl(struct tty_struct *tty, struct file *filp, 
                         unsigned int cmd, unsigned long arg)
{
  int ret_val = 0;

  switch (cmd) {
    case DCC_SETPOLLPERIOD:
      dcc_config.timer_poll_period = arg;
      break;

    case DCC_GETPOLLPERIOD:
      ret_val = dcc_config.timer_poll_period;
      break;

    default:
      ret_val = -ENOIOCTLCMD;
      break;
  }

  return ret_val;
}

#endif /* CONFIG_DEBUG_DCC_KGDB */

/* Network (virtual Ethernet over DCC) device driver routines
 *
 * Co-exists with KGDB, and will run when KGDB is at a breakpoint.
 * Developer beware !
 */
#ifndef CONFIG_DEBUG_DCC_RAW

/* Network device driver structure */
static struct net_device* dcc_netdev;

/* Callback from the universal DCC driver when a MAC address arrives from the RVI */
void rvidcc_cb_set_mac_address(unsigned char *mac)
{
  memcpy(dcc_info.mac_address, mac, ETH_ALEN);

  printk(KERN_INFO "DCC: MAC address %02x:%02x:%02x:%02x:%02x:%02x\n", dcc_info.mac_address[0],
         dcc_info.mac_address[1], dcc_info.mac_address[2], dcc_info.mac_address[3],
         dcc_info.mac_address[4], dcc_info.mac_address[5]);
}


/* Network device address validation */
static int dcc_net_validate_addr(struct net_device *dev)
{
  /* Wait for the MAC address from the RVI */
  unsigned long startjif = jiffies;
  while (dcc_info.mac_address[0] == 0 && 
         dcc_info.mac_address[1] == 0 &&
         dcc_info.mac_address[2] == 0 &&
         dcc_info.mac_address[3] == 0 &&
         dcc_info.mac_address[4] == 0 &&
         dcc_info.mac_address[5] == 0)
  {
    if (dcc_config.use_timer_poll)
      dcc_poll();

    /* Wait for a maximum of 1/2 a second */
    if (startjif - jiffies > HZ/2)
    {
      printk(KERN_ERR "DCC: Virtual ethernet timed out waiting for MAC address\n");
      break;
    }
    schedule();
  }

  memcpy(dev->dev_addr, dcc_info.mac_address, ETH_ALEN);

  if (!is_valid_ether_addr(dev->dev_addr))
    return -EINVAL;

  return 0;
}

/* Network device open routine */
static int dcc_net_open(struct net_device *dev)
{
  netif_start_queue(dev);
  dcc_info.netdev = dev; /* mark as running */
  return 0;
}

/* Network device close routine */
static int dcc_net_close(struct net_device *dev)
{
  /* Discard any pending TX packet */
  if (dcc_info.pending_skb)
  {
    dev->stats.tx_dropped++;
    dev_kfree_skb(dcc_info.pending_skb);
    dcc_info.pending_skb = NULL;
  }

  /* Mark as not running, to discard incoming packets */
  dcc_info.netdev = NULL;

  netif_stop_queue(dev);
  return 0;
}

/* Called by universal driver when an IP packet is received */
void rvidcc_cb_ip_packet_received(void *packet, size_t length)
{
  /* Drop the packet if the device isn't open */
  if (dcc_info.netdev)
  {
    struct sk_buff *skb = dev_alloc_skb(length + 16);
    struct ethhdr *buf;

    if (!skb)
    {
      dcc_info.netdev->stats.rx_dropped++;
      return;
    }

    /* Create Ethernet header */
    skb_reserve(skb,2);	/* Force 16 byte alignment */
    buf = (struct ethhdr*)skb_put(skb, length + 14);
    memcpy(buf->h_dest, dcc_info.mac_address, 6);
    memcpy(&buf->h_source[3], &dcc_info.mac_address[3], 3);
    buf->h_source[0] = 0x00;
    buf->h_source[1] = 0x02;
    buf->h_source[2] = 0xf7;  /* RVI MAC address */
    buf->h_proto = htons(ETH_P_IP);

    /* Fill in the rest of the data */
    memcpy((char*)buf + sizeof(struct ethhdr), packet, length);
    skb->dev = dcc_info.netdev;
    skb->protocol = eth_type_trans(skb, dcc_info.netdev);
    dcc_info.netdev->stats.rx_packets++;
    dcc_info.netdev->stats.rx_bytes += length+14;
    netif_rx(skb);
  }
}

/* Network device packet transmit routine */
static int dcc_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
  switch (ntohs(((struct ethhdr*)skb->data)->h_proto))
  {
    case ETH_P_ARP: /* ARP protocol */
    {
      struct sk_buff *skbreply = dev_alloc_skb(skb->len);
      struct ethhdr *buf;
      struct arphdr *arp;
      unsigned char *arpdata, *origarpdata;

      if (!skbreply)
      {
        dcc_info.netdev->stats.tx_dropped++;
        dev_kfree_skb(skb);
        return 0;
      }

      /* Set up manufactured ARP reply data */
      buf = (struct ethhdr*)skb_put(skbreply, skb->len);
      arp = (struct arphdr*)((char*)buf + sizeof(struct ethhdr));
      arpdata = (unsigned char*)((char*)arp + sizeof(struct arphdr));
      origarpdata = skb->data + sizeof(struct ethhdr) + sizeof(struct arphdr);
      memcpy(buf, skb->data, skb->len);
      memcpy(buf->h_dest, dcc_info.mac_address, 6);
      memcpy(&buf->h_source[3], &dcc_info.mac_address[3], 3);
      buf->h_source[0] = 0x00;
      buf->h_source[1] = 0x02;
      buf->h_source[2] = 0xf7;  /* RVI MAC address */
      arp->ar_op = htons(ARPOP_REPLY);
      memcpy(&arpdata[0], buf->h_source, ETH_ALEN);
      memcpy(&arpdata[10], buf->h_dest, ETH_ALEN);
      memcpy(&arpdata[6], &origarpdata[16], 4);
      memcpy(&arpdata[16], &origarpdata[6], 4);

      skbreply->dev = dcc_info.netdev;
      skbreply->protocol = eth_type_trans(skbreply, dcc_info.netdev);
      dcc_info.netdev->stats.rx_packets++;
      dcc_info.netdev->stats.rx_bytes += skb->len;
      dcc_info.netdev->stats.tx_packets++;
      dcc_info.netdev->stats.tx_bytes += skb->len;
      dev_kfree_skb(skb);
      netif_rx(skbreply);
      break;
    }
    case ETH_P_IP:  /* IP protocol */
    {
      if (dcc_transmit_ip_packet(&skb->data[14], skb->len-14)) /* discard eth header */
      {
        dcc_info.netdev->stats.tx_packets++;
        dcc_info.netdev->stats.tx_bytes += skb->len;
        dev_kfree_skb(skb);
      }
      else
      {
        dcc_info.pending_skb = skb;
        netif_stop_queue(dev);
      }

      if (dcc_config.use_timer_poll)
        dcc_poll();

      break;
    }
    default:
      printk(KERN_WARNING "DCC: dropping packet ethertype 0x%04x\n", 
             (int)ntohs(((struct ethhdr*)skb->data)->h_proto));
      dcc_info.netdev->stats.tx_dropped++;
      dev_kfree_skb(skb);
  }

  return 0;
}

/* Universal driver callback to handle pseudo-ARP */
int rvidcc_cb_has_addr(unsigned char *ip_binary)
{
  u32 ip;
  memcpy(&ip, ip_binary, 4);

  if (!dcc_info.netdev || IN_LOOPBACK(ip) || IN_MULTICAST(ip))
    return 0;

  /* Return 1 if address is local to the target */
  return inet_addr_type(dev_net(dcc_info.netdev), ip) == RTN_LOCAL ? 1 : 0;
}

/* Network device initialisation routine */
static void dcc_net_init(struct net_device *dev)
{
  /* Like normal Ethernet, but with specific differences */
  ether_setup(dev);
  dev->tx_queue_len = 3;

  /* Driver routines */
  dev->validate_addr = dcc_net_validate_addr;
  dev->open = dcc_net_open;
  dev->stop = dcc_net_close;
  dev->hard_start_xmit = dcc_net_xmit;
}

#endif /* !CONFIG_DEBUG_DCC_RAW */

/* Data transfer handling functionality
 *
 * We support interrupt and timer polled functionality
 * if interrupts are not available.
 */
static void dcc_timer_poll(unsigned long arg);

static struct timer_list dcc_timer = {
  function:   dcc_timer_poll
};

/* Poll routine for when interrupts are not used */
static void dcc_poll(void)
{
  if (!test_and_set_bit(0, &dcc_info.timerpoll_lock) &&
      !dcc_info.kernel_in_panic)
  {
    rvidcc_poll();
    clear_bit(0, &dcc_info.timerpoll_lock);
  }
}

/* Self-recheduling timer poll routine */
static void dcc_timer_poll(unsigned long arg)
{
  dcc_poll();
  dcc_timer.expires = jiffies + dcc_config.timer_poll_period;
  add_timer(&dcc_timer);
}

/* DCC interrupt handler */
static irqreturn_t dcc_read_interrupt(int irq, void *devid)
{
  if (!dcc_info.kernel_in_panic)
    rvidcc_read();
  return IRQ_HANDLED;
}

/* DCC interrupt handler */
static irqreturn_t dcc_write_interrupt(int irq, void *devid)
{
  if (!dcc_info.kernel_in_panic)
    rvidcc_write();
  return IRQ_HANDLED;
}

/* Routine to process incoming DCC data from interrupt or timer poll */
static void dcc_tasklet_action(unsigned long data)
{
#ifndef CONFIG_DEBUG_DCC_RAW
  rvidcc_process();

  /* Try to transmit any pending packet */
  if (dcc_info.pending_skb && dcc_info.netdev)
  {
    if (dcc_transmit_ip_packet(&dcc_info.pending_skb->data[14], dcc_info.pending_skb->len-14))
    {
      dcc_info.netdev->stats.tx_packets++;
      dcc_info.netdev->stats.tx_bytes += dcc_info.pending_skb->len;
      dev_kfree_skb(dcc_info.pending_skb);
      dcc_info.pending_skb = NULL;
      netif_wake_queue(dcc_info.netdev);
    }
  }
#endif

  /* tasklet can only run on one CPU at a time and this is the only place
   * we do a serial read, so no locking needed here */
#ifdef CONFIG_DEBUG_DCC_KGDB
  if (data != 1 && rvidcc_serial_can_read())
  {
    /* Absorb ^C before triggering a hardcoded breakpoint */
    if (rvidcc_scan_input_for(0x03, 1))
    {
      char buf;
      rvidcc_read_serial(&buf, 1);
    }
    breakpoint();
  }
#else
  /* Receive any bytes that may be pending if the device has been opened */
  if (dcc_info.tty)
  {
    size_t data_available = rvidcc_serial_can_read();
    if (data_available)
    {
      unsigned char* buf;
      int len = tty_prepare_flip_string(dcc_info.tty, &buf, data_available);
      if (len)
      {
        rvidcc_read_serial(buf, len);
        tty_flip_buffer_push(dcc_info.tty);
      }
    }
  }
#endif
}

/* Universal driver callback called when data has arrived on DCC */
void rvidcc_cb_notify()
{
  tasklet_schedule(&dcc_tasklet);
}

/* Universal initialisation - ensures init only happens once */
static int dcc_init_dcc(void)
{
  int err = 0;
  if (!dcc_info.init_called)
  {
      err = rvidcc_init();
      if (err == 0)
          dcc_info.init_called = 1;
      else
          return -ENODEV;
  }

  return err;
}

/* Banner to be printed on initialisation */
static void dcc_print_banner(void)
{
  printk(KERN_INFO "RealView ICE DCC device driver %s " 
#ifdef CONFIG_DEBUG_DCC_RAW
         "[raw mode] "
#else
         "[tty_eth mode] "
#endif
         "(C)2004-2007 ARM Limited\n", ARM_DCC_VER);
}

static void dcc_drain_outbuf(void)
{
  size_t bytesPending;
  size_t lastBytesPending = 0;
  int failcount = 0;

  while ((bytesPending = rvidcc_outbuf_data()) > 0)
  {
    if (bytesPending >= lastBytesPending)
      ++failcount;

    if (failcount > 10)
    {
      /* The queue isn't draining: either the connection to the RVI is
       * lost or we can't process messages fast enough.  Either way, it's
       * bad to block forever, so bail out */
      break;
    }

    lastBytesPending = bytesPending;

    rvidcc_poll();
  }
}

#ifndef CONFIG_DEBUG_DCC_KGDB

/* TTY device driver structure */
static const struct tty_operations dcc_tty_ops = 
{
  .open           = dcc_chr_open,
  .close          = dcc_chr_close,
  .write          = dcc_chr_write,
  .write_room     = dcc_chr_write_room,
  .chars_in_buffer = dcc_chr_chars_in_buffer,
  .ioctl          = dcc_chr_ioctl
};

static struct tty_driver *dcc_tty_driver;

/* Create and install TTY driver */
static int dcc_init_tty_driver(void)
{
  int error = 0;

  dcc_tty_driver = alloc_tty_driver(1);
  if (!dcc_tty_driver)
    return -ENOMEM;
  dcc_tty_driver->owner = THIS_MODULE;
  dcc_tty_driver->driver_name = ARM_TTY_NAME;
  dcc_tty_driver->name = ARM_TTY_NAME;
  dcc_tty_driver->major = ARM_DCC_MAJOR;
  dcc_tty_driver->minor_start = ARM_DCC_MINOR;
  dcc_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
  dcc_tty_driver->subtype = SERIAL_TYPE_NORMAL;
  dcc_tty_driver->flags = TTY_DRIVER_REAL_RAW;
  dcc_tty_driver->init_termios = tty_std_termios;
  tty_set_operations(dcc_tty_driver, &dcc_tty_ops);
  if ((error = tty_register_driver(dcc_tty_driver)))
  {
    printk(KERN_ERR "DCC: Can't register tty device %d, error = %d\n",
           ARM_DCC_MINOR, error);
    put_tty_driver(dcc_tty_driver);
    dcc_tty_driver = NULL;
    return error;
  }
  return 0;
}

/* Console device print routine */
static void dcc_console_print(struct console *co, const char *buf, unsigned count)
{
  unsigned int bytes = 0;

  /* Send message out */
  bytes = (unsigned int)dcc_write_serial(buf, count);

  /* If buffer is full stall the system until there is space */
  /* A flood of printk's usually means the system is dying anyway */
  if (bytes < count)
  {
    unsigned long flags;
    local_irq_save(flags);

    while (bytes < count)
    {
      rvidcc_poll();
      bytes += (unsigned int)dcc_write_serial(buf + bytes, count - bytes);
    }

    local_irq_restore(flags);
  }

  /* Do an immediate flush if kernel has panicked */
  if (dcc_info.kernel_in_panic)
  {
    unsigned long flags;
    local_irq_save(flags);

    dcc_drain_outbuf();

    local_irq_restore(flags);
  }
}

/* Console device initialisation routine */
static struct tty_driver* dcc_console_device(struct console *co, int *index)
{
  *index = 0;
  return dcc_tty_driver;
}

/* Console device driver */
static struct console dcc_con_driver =
{
  .name       = ARM_TTY_NAME,
  .write      = dcc_console_print,
  .device     = dcc_console_device,
  .flags      = CON_PRINTBUFFER,
  .index      = -1,
};


/* Routine to ensure DCC is flushed on reboot or kernel panic */
static int dcc_reboot_handler(struct notifier_block *this,
                              unsigned long event, void *unused)
{
  unsigned long flags;
  local_irq_save(flags);

  dcc_drain_outbuf();

  local_irq_restore(flags);

  return NOTIFY_OK;
}

/* Reboot notifier structure */
static struct notifier_block dcc_reboot_notifier =
{
  dcc_reboot_handler,
  NULL,
  0
};

/* Called when kernel enters the panic state */
static int dcc_panic_handler(struct notifier_block *this,
                             unsigned long event, void *unused)
{
  dcc_info.kernel_in_panic = 1;
  return dcc_reboot_handler(this, event, unused);
}

/* Panic notifier structure */
static struct notifier_block dcc_panic_notifier =
{
  .notifier_call = dcc_panic_handler,
  .next = NULL,
  .priority = 0
};

/* Early console initialization.  Preceeds driver initialization. */
static int __init dcc_console_init(void)
{
  int err = 0;

  if (!dcc_info.init_called)
  {
    dcc_print_banner();
    err = dcc_init_dcc();
  }

  if (!err)
  {
    register_reboot_notifier(&dcc_reboot_notifier);
    atomic_notifier_chain_register(&panic_notifier_list, &dcc_panic_notifier);
    register_console(&dcc_con_driver);
  }

  return err;
}
#endif /* !CONFIG_DEBUG_DCC_KGDB */

/* Primary module (and devices) initialisation routine */
int __init dcc_init(void)
{
  int ret = 0;

  if (!dcc_info.init_called)
    dcc_print_banner();

  /* Detect whether timer polling when required */
  if (dcc_config.rx_interrupt == dcc_config.tx_interrupt)
  {
    dcc_config.use_timer_poll = 1;

    if (dcc_config.rx_interrupt != 0)
      printk(KERN_ERR "DCC: IRQ numbers must be different for Rx and Tx !!!!!\n");
  }
  else
    dcc_config.use_timer_poll = 0;

#ifndef CONFIG_DEBUG_DCC_KGDB
  ret = dcc_init_tty_driver();
#endif

#ifndef CONFIG_DEBUG_DCC_RAW
  if (!ret)
  {
    /* Install network device driver */
    dcc_netdev = alloc_netdev(0, ARM_ETH_NAME, dcc_net_init);
    if (dcc_netdev == NULL)
    {
      printk(KERN_ERR "DCC: Can't allocate net device\n");
      ret = -ENOMEM;
      goto error;
    }
    else
    {
      ret = register_netdev(dcc_netdev);

      if (ret)
      {
        printk(KERN_ERR "DCC: Can't register net device\n");
        free_netdev(dcc_netdev);
        goto error;
      }
    }
  }
#endif

  /* Initiate interrupts or timer polling as necessary */
  if (!ret)
  {
    if (!dcc_config.use_timer_poll)
    {
      ret = dcc_init_dcc();

      if (ret)
      {
        goto error;
      }
      else
      {
        if (request_irq(dcc_config.rx_interrupt, dcc_read_interrupt, 0, "dcc:rx", NULL) ||
            request_irq(dcc_config.tx_interrupt, dcc_write_interrupt, 0, "dcc:tx", NULL))
        {
          printk(KERN_ERR "DCC: Can't install interrupt handlers for IRQ%u and/or IRQ%u\n",
                 dcc_config.rx_interrupt, dcc_config.tx_interrupt);
          /* unregister tty & netdev drivers on error */
          ret = -EBUSY;
          goto error;
        }
        else
          printk(KERN_INFO "DCC: Using IRQ%u (Rx) and IRQ%u (Tx)\n",
                 dcc_config.rx_interrupt, dcc_config.tx_interrupt);
      }
    }
    else
    {
      init_timer(&dcc_timer);
      printk(KERN_INFO "DCC: Using timer polled mode\n");

      ret = dcc_init_dcc();
      if (ret)
      {
        del_timer_sync(&dcc_timer);
        goto error;
      }
      else
      {
        dcc_timer.expires = jiffies + dcc_config.timer_poll_period;
        add_timer(&dcc_timer);
      }
    }
  }

  return ret;

error:

#ifndef CONFIG_DEBUG_DCC_KGDB
  if (dcc_tty_driver)
  {
    tty_unregister_driver(dcc_tty_driver);
    put_tty_driver(dcc_tty_driver);
    dcc_tty_driver = NULL;
  }
#endif

#ifndef CONFIG_DEBUG_DCC_RAW
  if (dcc_netdev)
  {
    unregister_netdev(dcc_netdev);
    free_netdev(dcc_netdev);
    dcc_netdev = NULL;
  }
#endif

  return ret;
}

/* Module cleanup (exit) routine - only called if compiled as a module */
void __exit dcc_cleanup(void)
{
  if (!dcc_config.use_timer_poll)
  {
    free_irq(dcc_config.rx_interrupt, NULL);
    free_irq(dcc_config.tx_interrupt, NULL);
  }
  else
    del_timer_sync(&dcc_timer);

#ifndef CONFIG_DEBUG_DCC_KGDB
  tty_unregister_driver(dcc_tty_driver);
  put_tty_driver(dcc_tty_driver);
#endif

#ifndef CONFIG_DEBUG_DCC_RAW
  unregister_netdev(dcc_netdev);
  free_netdev(dcc_netdev);
#endif

#ifndef CONFIG_DEBUG_DCC_KGDB
  unregister_console(&dcc_con_driver);
  unregister_reboot_notifier(&dcc_reboot_notifier);
  atomic_notifier_chain_unregister(&panic_notifier_list, &dcc_panic_notifier);
#endif

  dcc_drain_outbuf();
}

#ifndef CONFIG_DEBUG_DCC_KGDB
console_initcall(dcc_console_init);
#endif
module_init(dcc_init);
module_exit(dcc_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ARM Ltd");
MODULE_DESCRIPTION("Driver for the ARM Debug Communications Channel");
MODULE_SUPPORTED_DEVICE(ARM_TTY_NAME);
