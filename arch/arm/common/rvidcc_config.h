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

#ifndef RVIDCC_CONFIG_H
#define RVIDCC_CONFIG_H

/*
 *
 * Platform headers that define basic types (e.g. size_t)
 * ------------------------------------------------------
 * 
 */
#include <linux/types.h>
#include <linux/netdevice.h>


/*
 * Put any platform specific definitions required by the interrupt macros here
 * ---------------------------------------------------------------------------
 */

/* Structure containing our internal state, etc. */
struct dccconfig
{
  unsigned int rx_interrupt;
  unsigned int tx_interrupt;
  unsigned long timer_poll_period;
  int use_timer_poll;

  volatile unsigned long irq_disable;
};

extern struct dccconfig dcc_config;


/*
 * Configuration
 * -------------
 */

/*
 * Define which ARM cores this binary is to be used on
 *  - RVIDCC_ARM79 for all ARM 7 and ARM 9 based cores
 *  - RVIDCC_ARM10 for all ARM 10 cores
 *  - RVIDCC_ARM11 for all ARM 11 and later cores (including Cortex)
 */
#define RVIDCC_ARM79
#define RVIDCC_ARM10
#define RVIDCC_ARM11

/*
 * Define RVIDCC_OUTBUF_DATA if the OS needs to check if all pending data
 * has been sent out to the ICE.
 */
#define RVIDCC_OUTBUF_DATA

/*
 * Define RVIDCC_RAW if the virtual ethernet functionality is not required.  See
 * rvidcc.c for more information.
 */
#ifdef CONFIG_DEBUG_DCC_RAW
#define RVIDCC_RAW
#endif

/*
 * Define RVIDCC_BIG_ENDIAN if your platform is big endian.
 */
#ifdef CONFIG_CPU_BIG_ENDIAN
#define RVIDCC_BIG_ENDIAN
#endif

/*
 * Define RVIDCC_SCAN_INPUT_FOR if the OS needs to peek ahead in the TTY data
 */
#ifdef CONFIG_DEBUG_DCC_KGDB
#define RVIDCC_SCAN_INPUT_FOR
#endif

/*
 * RVIDCC_BUFSIZE defines the size of the input and output buffers for DCC data
 */
#ifndef RVIDCC_BUFSIZE
#ifdef RVIDCC_RAW
#define RVIDCC_BUFSIZE 4096
#else
#define RVIDCC_BUFSIZE 8192
#endif
#endif

/*
 * RVIDCC_MAX_INLINE_POLLS defines the number of times to the driver should
 * check if the host is ready for more data in rvidcc_write or if the host has
 * sent more data in rvidcc_read.  Increasing this value may improve the data
 * transfer rate, but will increase the length of time spent in the interrupt
 * handler.  Adjust this value according to the performance and latency
 * requirements of the system.
 */
#define RVIDCC_MAX_INLINE_POLLS 500

/*
 * Interrupt configuration
 *
 * Define macros here to enable and disable interrupts.  If the OS uses
 * polling instead of interrupts, define as empty.
 *
 */

#define RVIDCC_ENABLE_INTERRUPTS

#define RVIDCC_DISABLE_INTERRUPTS

/* Interrupt control macros */
#define RVIDCC_ENABLE_WRITE_INTERRUPT do {                \
    if (!dcc_config.use_timer_poll) {                     \
      if (test_and_clear_bit(0, &dcc_config.irq_disable)) \
        enable_irq(dcc_config.tx_interrupt);              \
    }                                                   \
  } while(0)

#define RVIDCC_DISABLE_WRITE_INTERRUPT do {             \
    if (!dcc_config.use_timer_poll) {                     \
      if (!test_and_set_bit(0, &dcc_config.irq_disable))  \
        disable_irq(dcc_config.tx_interrupt);             \
    }                                                   \
  } while(0)

#define RVIDCC_ENABLE_READ_INTERRUPT do {               \
    if (!dcc_config.use_timer_poll) {                     \
      if (test_and_clear_bit(1, &dcc_config.irq_disable)) \
        enable_irq(dcc_config.rx_interrupt);              \
    }                                                   \
  } while(0)

#define RVIDCC_DISABLE_READ_INTERRUPT do {              \
    if (!dcc_config.use_timer_poll) {                     \
      if (!test_and_set_bit(1, &dcc_config.irq_disable))  \
        disable_irq(dcc_config.rx_interrupt);             \
    }                                                   \
  } while(0)


#endif /* RVIDCC_CONFIG_H */
