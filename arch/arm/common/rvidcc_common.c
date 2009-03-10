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

/*     ================================================================
       Universal RealView ICE DCC communications driver - How to use it
       ================================================================

   This driver is split into 2 sections: a generic section (this file) that
   handles the DCC hardware access and buffering and an OS-dependent file that
   integrates the driver into the OS network and TTY layers.  The interface
   between the sections is defined in rvidcc_interface.h, which contains
   prototypes and documentation for the functions in the generic section that
   can be called from the OS-dependent section and the callback functions
   called from the generic section that must be provided by the OS-dependent
   section.

   This driver is designed to be able to work with or without interrupts, to
   use raw DCC comms (as a TTY) or virtual Ethernet.  The driver is configured
   in the rvidcc_config.h header, including options to select which cores to
   support, whether to support ethernet, buffer sizes and how to
   enable/disable interrupts.  See the example configs for details of all
   options.

   All buffers used by the driver are in the system memory space.  All
   pointer arguments for functions called by or from the generic section are
   pointers to system memory.  It is the responsibility of the OS-dependent
   section to copy to/from user memory.

   ARM Ltd. highly recommends using interrupt-driven DCC communications, and
   to use the virtual Ethernet system, even if only a TTY is required.

   Raw DCC communications transfers data in 32-bit words. Thus if virtual
   Ethernet is not used, data transfers will contain null padding for blocks
   of data that do not have a multiple of 4 bytes. For most command-line
   applications, this is unacceptable. However protocols, such as the remote
   gdb protocol, will tolerate the extra nulls, provided they occur between
   packets. This, of course, requires that data be passed to the driver as
   packets, and not as byte-by-byte as some TTY drivers are in the habit of
   doing.

   Using the virtual Ethernet system does not force you to have to support
   networking, or an IP stack, etc. The virtual Ethernet system contains a
   TTY channel and an Ethernet channel, and if only the TTY channel is
   required, the Ethernet channel and its data can be ignored. Using this
   for TTY only has the advantage that data packets do not have to be a
   multiple of 4 bytes, and so it is suitable for command-line consoles.

   The virtual Ethernet system is not true Ethernet in that it can only carry
   IP traffic. It does, however, have a mechanism for obtaining a unique MAC
   address from the ICE, and the target will receive ARP-style queries to
   determine if the target has a particular IP address. Thus DHCP can be used
   to obtain an IP address from an external DHCP server, without the need for
   any special configuration of the ICE.
*/

#include "rvidcc_config.h"
#include "rvidcc_interface.h"


#ifdef __ARMCC_VERSION
#define INLINE __inline
#else
#define INLINE inline
#endif


#ifndef RVIDCC_RAW

#define UNCOMP_TTY_START 0xa5580000
#define UNCOMP_ETH_START 0xa55a0000
#define START_MASK       0xfffc0000
#define START_SENTINEL   UNCOMP_TTY_START

#endif /* RVIDCC_RAW */

#if !defined(RVIDCC_ARM79) && !defined(RVIDCC_ARM10) && !defined(RVIDCC_ARM11)
#warning No cores are enabled.  Define at least one of RVIDCC_ARM79, RVIDCC_ARM10 or RVIDCC_ARM11
#endif


#define DCC_ARM9_RBIT  (1 << 0)
#define DCC_ARM9_WBIT  (1 << 1)
#define DCC_ARM10_RBIT (1 << 7)
#define DCC_ARM10_WBIT (1 << 6)
#define DCC_ARM11_RBIT (1 << 30)
#define DCC_ARM11_WBIT (1 << 29)

/* Access primitives: x must be unsigned int */

#ifdef __ARMCC_VERSION

/* RVCT versions */
#define READ_CORE_ID(x) { __asm { mrc p15, 0, x, c0, c0, 0 } \
                           x = (x >> 4) & 0xFFF; }

#ifdef RVIDCC_ARM79
#define WRITE_ARM9_DCC(x) __asm { mcr p14, 0, x, c1, c0, 0 }
#define READ_ARM9_DCC(x) __asm { mrc p14, 0, x, c1, c0, 0 }
#define STATUS_ARM9_DCC(x) __asm { mrc p14, 0, x, c0, c0, 0 }
#define CAN_READ_ARM9_DCC(x) {STATUS_ARM9_DCC(x); x &= DCC_ARM9_RBIT;}
#define CAN_WRITE_ARM9_DCC(x) {STATUS_ARM9_DCC(x); x &= DCC_ARM9_WBIT; x = (x==0);}
#endif

#ifdef RVIDCC_ARM10
#define WRITE_ARM10_DCC(x) __asm { mcr p14, 0, x, c0, c5, 0 }
#define READ_ARM10_DCC(x) __asm { mrc p14, 0, x, c0, c5, 0 }
#define STATUS_ARM10_DCC(x) __asm { mrc p14, 0, x, c0, c1, 0 }
#define CAN_READ_ARM10_DCC(x) {STATUS_ARM10_DCC(x); x &= DCC_ARM10_RBIT;}
#define CAN_WRITE_ARM10_DCC(x) {STATUS_ARM10_DCC(x); x &= DCC_ARM10_WBIT; x = (x==0);}
#endif

#ifdef RVIDCC_ARM11
#define WRITE_ARM11_DCC(x) __asm { mcr p14, 0, x, c0, c5, 0 }
#define READ_ARM11_DCC(x) __asm { mrc p14, 0, x, c0, c5, 0 }
#define STATUS_ARM11_DCC(x) __asm { mrc p14, 0, x, c0, c1, 0 }
#define CAN_READ_ARM11_DCC(x) {STATUS_ARM11_DCC(x); x &= DCC_ARM11_RBIT;}
#define CAN_WRITE_ARM11_DCC(x) {STATUS_ARM11_DCC(x); x &= DCC_ARM11_WBIT; x = (x==0);}
#endif

#else

/* GCC versions */
#define READ_CORE_ID(x) { __asm__ ("mrc p15, 0, %0, c0, c0, 0\n" : "=r" (x)); \
                           x = (x >> 4) & 0xFFF; }

#ifdef RVIDCC_ARM79
#define WRITE_ARM9_DCC(x) __asm__ volatile ("mcr p14, 0, %0, c1, c0, 0\n" : : "r" (x))
#define READ_ARM9_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c1, c0, 0\n" : "=r" (x))
#define STATUS_ARM9_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c0, 0\n" : "=r" (x))
#define CAN_READ_ARM9_DCC(x) {STATUS_ARM9_DCC(x); x &= DCC_ARM9_RBIT;}
#define CAN_WRITE_ARM9_DCC(x) {STATUS_ARM9_DCC(x); x &= DCC_ARM9_WBIT; x = (x==0);}
#endif

#ifdef RVIDCC_ARM10
#define WRITE_ARM10_DCC(x) __asm__ volatile ("mcr p14, 0, %0, c0, c5, 0\n" : : "r" (x))
#define READ_ARM10_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c5, 0\n" : "=r" (x))
#define STATUS_ARM10_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c1, 0\n" : "=r" (x))
#define CAN_READ_ARM10_DCC(x) {STATUS_ARM10_DCC(x); x &= DCC_ARM10_RBIT;}
#define CAN_WRITE_ARM10_DCC(x) {STATUS_ARM10_DCC(x); x &= DCC_ARM10_WBIT; x = (x==0);}
#endif

#ifdef RVIDCC_ARM11
#define WRITE_ARM11_DCC(x) __asm__ volatile ("mcr p14, 0, %0, c0, c5, 0\n" : : "r" (x))
#define READ_ARM11_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c5, 0\n" : "=r" (x))
#define STATUS_ARM11_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c1, 0\n" : "=r" (x))
#define CAN_READ_ARM11_DCC(x) {STATUS_ARM11_DCC(x); x &= DCC_ARM11_RBIT;}
#define CAN_WRITE_ARM11_DCC(x) {STATUS_ARM11_DCC(x); x &= DCC_ARM11_WBIT; x = (x==0);}
#endif

#endif

static enum {arm9_and_earlier, arm10, arm11_and_later} arm_type = arm9_and_earlier;

/* Prototypes */
static void rvidcc_write_lowlevel(const void *ptr, size_t len);

/* Queues */

struct ringbuffer
{
    unsigned char *buf;
    size_t size;
    unsigned char* readPtr;
    unsigned char* writePtr;
};

static unsigned char dcc_inbuffer[RVIDCC_BUFSIZE];
static struct ringbuffer dcc_inbuf;
static unsigned char dcc_outbuffer[RVIDCC_BUFSIZE];
static struct ringbuffer dcc_outbuf;

#ifndef RVIDCC_RAW
#define DCC_TTY_INBUF_SIZE (RVIDCC_BUFSIZE/2)
static unsigned char dcc_temp_source[DCC_TTY_INBUF_SIZE];
static unsigned char dcc_tty_inbuffer[DCC_TTY_INBUF_SIZE];
static struct ringbuffer dcc_tty_inbuf;
#define DCC_TTY_INBUF      dcc_tty_inbuf

#else

#define DCC_TTY_INBUF      dcc_inbuf

#endif


/* This ringbuffer has no locks as it assumes only 1 reader and 1 writer and
 * assumes a 32-bit access is atomic */
INLINE static void ringbuf_advance_read(struct ringbuffer* ring_buf, size_t amount)
{
    unsigned char* temptr = ring_buf->readPtr + amount;
    while (temptr >= ring_buf->buf + ring_buf->size)
        temptr -= ring_buf->size;
    ring_buf->readPtr = temptr;
}


INLINE static void ringbuf_advance_write(struct ringbuffer* ring_buf, size_t amount)
{
    unsigned char* temptr = ring_buf->writePtr + amount;
    while (temptr >= ring_buf->buf + ring_buf->size)
        temptr -= ring_buf->size;
    ring_buf->writePtr = temptr;
}


INLINE static void ringbuf_get(struct ringbuffer* ring_buf,
                        unsigned char* dst, size_t amount)
{
    unsigned char* source = ring_buf->readPtr;
    size_t len_to_end = ring_buf->buf + ring_buf->size - source;
    if (amount > len_to_end)
    {
        memcpy(dst, source, len_to_end);
        memcpy(dst + len_to_end, ring_buf->buf, (amount - len_to_end));
    }
    else
        memcpy(dst, source, amount);
}


INLINE static void ringbuf_put(struct ringbuffer* ring_buf,
                        unsigned char* src, size_t amount)
{
    unsigned char* dest = ring_buf->writePtr;
    unsigned int len_to_end = ring_buf->buf + ring_buf->size - dest;
    if (amount > len_to_end)
    {
        memcpy(dest, src, len_to_end);
        memcpy(ring_buf->buf, src + len_to_end, amount - len_to_end);
    }
    else
        memcpy(dest, src, amount);
}


INLINE static size_t ringbuf_available_data(struct ringbuffer* ring_buf)
{
    int avail = ring_buf->writePtr - ring_buf->readPtr;

    if (avail < 0)
        /* handle wrap around */
        avail += ring_buf->size;

    return (size_t)avail;
}


INLINE static size_t ringbuf_available_space(struct ringbuffer* ring_buf)
{
    size_t avail;

    if (ring_buf->writePtr >= ring_buf->readPtr)
        avail = ring_buf->size - (ring_buf->writePtr - ring_buf->readPtr) - 1;
    else
        avail = ring_buf->readPtr - ring_buf->writePtr - 1;

    return avail;
}


// conditionally swap endian of a 32bit word
#ifdef RVIDCC_BIG_ENDIAN
#define to_little_endian(x) ((((x)&0xff) << 24) | (((x)&0xff00) << 8) | (((x)&0xff0000) >> 8) | (((x)&0xff000000) >> 24))
#else
#define to_little_endian(x) (x)
#endif


int rvidcc_init(void)
{
    register unsigned int id;
    int err = 0;
#ifndef RVIDCC_RAW
    unsigned int macRequest = to_little_endian(UNCOMP_ETH_START);
#endif

    RVIDCC_DISABLE_INTERRUPTS;

    /* check core type, raising error if core not supported */
    READ_CORE_ID(id);
    if ((id & 0xF00) == 0xA00)
    {
        arm_type = arm10;
#ifndef RVIDCC_ARM10
        err = RVIDCC_ERR_CORE_NOT_SUPPORTED;
#endif
    }
    else if (id >= 0xb00)
    {
        arm_type = arm11_and_later;
#ifndef RVIDCC_ARM11
        err = RVIDCC_ERR_CORE_NOT_SUPPORTED;
#endif
    }
    else
    {
        arm_type = arm9_and_earlier;
#ifndef RVIDCC_ARM79
        err = RVIDCC_ERR_CORE_NOT_SUPPORTED;
#endif
    }

    if (!err)
    {
        dcc_inbuf.buf = dcc_inbuffer;
        dcc_inbuf.size = RVIDCC_BUFSIZE;
        dcc_inbuf.readPtr  = dcc_inbuf.buf;
        dcc_inbuf.writePtr = dcc_inbuf.buf;

        dcc_outbuf.buf = dcc_outbuffer;
        dcc_outbuf.size = RVIDCC_BUFSIZE;
        dcc_outbuf.readPtr  = dcc_outbuf.buf;
        dcc_outbuf.writePtr = dcc_outbuf.buf;

#ifndef RVIDCC_RAW
        dcc_tty_inbuf.buf = dcc_tty_inbuffer;
        dcc_tty_inbuf.size = DCC_TTY_INBUF_SIZE;
        dcc_tty_inbuf.readPtr  = dcc_tty_inbuf.buf;
        dcc_tty_inbuf.writePtr = dcc_tty_inbuf.buf;

        /* request a MAC address */
        rvidcc_write_lowlevel(&macRequest, sizeof(unsigned int));
#endif

        /* Enable interrupts (write interrupt will be enabled when something is written) */
        RVIDCC_ENABLE_READ_INTERRUPT;
        RVIDCC_ENABLE_INTERRUPTS;
    }

    return err;
}


void rvidcc_write(void)
{
    register unsigned int reg;
    int pollcount = -1;
    int some_transfer = 0;

    switch (arm_type)
    {
#ifdef RVIDCC_ARM79
    case arm9_and_earlier:
    {
        /* Try to write everything available */
        while (ringbuf_available_data(&dcc_outbuf) >= sizeof(unsigned int))
        {
            /* On the first word, the write is aborted immediately if the DCC write
             * register is full.  On subsequent words, the driver waits for a
             * bit (RVIDCC_MAX_INLINE_POLLS times) as the RVI will probably
             * read the DCC register soon because it knows data is being sent. */
            do
            {
                CAN_WRITE_ARM9_DCC(reg);
            } while (!reg && pollcount != -1 && --pollcount > 0);

            if (!reg)
                break;

            reg = to_little_endian(*(unsigned int *)dcc_outbuf.readPtr);
            WRITE_ARM9_DCC(reg);
            ringbuf_advance_read(&dcc_outbuf, sizeof(unsigned int));
            pollcount = RVIDCC_MAX_INLINE_POLLS;
            some_transfer = 1;
        }

        pollcount = -1;
        break;
    }
#endif

#ifdef RVIDCC_ARM10
    case arm10:
    {
        /* Try to write everything available */
        while (ringbuf_available_data(&dcc_outbuf) >= sizeof(unsigned int))
        {
            do
            {
                CAN_WRITE_ARM10_DCC(reg);
            } while (!reg && pollcount != -1 && --pollcount > 0);

            if (!reg)
                break;

            reg = to_little_endian(*(unsigned int *)dcc_outbuf.readPtr);
            WRITE_ARM10_DCC(reg);
            ringbuf_advance_read(&dcc_outbuf, sizeof(unsigned int));
            pollcount = RVIDCC_MAX_INLINE_POLLS;
            some_transfer = 1;
        }

        pollcount = -1;
        break;
    }
#endif

#ifdef RVIDCC_ARM11
    case arm11_and_later:
    {
        /* Try to write everything available */
        while (ringbuf_available_data(&dcc_outbuf) >= sizeof(unsigned int))
        {
            do
            {
                CAN_WRITE_ARM11_DCC(reg);
            } while (!reg && pollcount != -1 && --pollcount > 0);

            if (!reg)
                break;

            reg = to_little_endian(*(unsigned int *)dcc_outbuf.readPtr);
            WRITE_ARM11_DCC(reg);
            ringbuf_advance_read(&dcc_outbuf, sizeof(unsigned int));
            pollcount = RVIDCC_MAX_INLINE_POLLS;
            some_transfer = 1;
        }

        pollcount = -1;
        break;
    }
#endif

    default:
        break;
    }

    if (some_transfer)
        rvidcc_cb_notify();

    /* Disable interrupt if nothing left to write */
    if (ringbuf_available_data(&dcc_outbuf) == 0)
        RVIDCC_DISABLE_WRITE_INTERRUPT;
}


/* this routine assumes it is non-interruptible by others that access the queue pointers */
/* this is written out longhand because it may be called from an interrupt routine */
void rvidcc_read(void)
{
    register unsigned int reg;
    int pollcount = -1;
    int some_transfer = 0;

    switch (arm_type)
    {
#ifdef RVIDCC_ARM79
    case arm9_and_earlier:
    {
        /* Try to read everything available */
        while (ringbuf_available_space(&dcc_inbuf) >= sizeof(unsigned int))
        {
            do
            {
                CAN_READ_ARM9_DCC(reg);
            } while (!reg && pollcount != -1 && --pollcount > 0);

            if (!reg)
                break;

            READ_ARM9_DCC(reg);
            *(unsigned int *)dcc_inbuf.writePtr = to_little_endian(reg);
            ringbuf_advance_write(&dcc_inbuf, sizeof(unsigned int));
            pollcount = RVIDCC_MAX_INLINE_POLLS;
            some_transfer = 1;
        }
        break;
    }
#endif

#ifdef RVIDCC_ARM10
    case arm10:
    {
        /* Try to read everything available */
        while (ringbuf_available_space(&dcc_inbuf) >= sizeof(unsigned int))
        {
            do
            {
                CAN_READ_ARM10_DCC(reg);
            } while (!reg && pollcount != -1 && --pollcount > 0);

            if (!reg)
                break;

            READ_ARM10_DCC(reg);
            *(unsigned int *)dcc_inbuf.writePtr = to_little_endian(reg);
            ringbuf_advance_write(&dcc_inbuf, sizeof(unsigned int));
            pollcount = RVIDCC_MAX_INLINE_POLLS;
            some_transfer = 1;
        }
        break;
    }
#endif

#ifdef RVIDCC_ARM11
    case arm11_and_later:
    {
        /* Try to read everything available */
        while (ringbuf_available_space(&dcc_inbuf) >= sizeof(unsigned int))
        {
            do
            {
                CAN_READ_ARM11_DCC(reg);
            } while (!reg && pollcount != -1 && --pollcount > 0);

            if (!reg)
                break;

            READ_ARM11_DCC(reg);
            *(unsigned int *)dcc_inbuf.writePtr = to_little_endian(reg);
            ringbuf_advance_write(&dcc_inbuf, sizeof(unsigned int));
            pollcount = RVIDCC_MAX_INLINE_POLLS;
            some_transfer = 1;
        }
        break;
    }
#endif

    default:
        break;
    }

    if (ringbuf_available_space(&dcc_inbuf) == 0)
        RVIDCC_DISABLE_READ_INTERRUPT;

    if (some_transfer)
        rvidcc_cb_notify();
}


/* this routine assumes it is not interrupted by others that access the queue pointers */
void rvidcc_poll(void)
{
    rvidcc_write();
    rvidcc_read();
}


#ifndef RVIDCC_RAW
void rvidcc_process(void)
{
    unsigned int sentinel = 0;
    int compsize = 0;

    /* Process received data */
    while (ringbuf_available_data(&dcc_inbuf) >= sizeof(unsigned int))
    {
        int bytes;
        unsigned char *buf;

        /* Search for start sentinel */
        sentinel = to_little_endian(*(unsigned int *)dcc_inbuf.readPtr);
        compsize = sentinel & 0xffff;

        /* Discard if not a start sentinel */
        if ((sentinel & START_MASK) != START_SENTINEL || 
            compsize > sizeof(dcc_temp_source) || compsize <= 0)
        {
            ringbuf_advance_read(&dcc_inbuf, sizeof(unsigned int));
            continue;
        }

        /* Sentinel found, come back later if not enough data */
        bytes = ringbuf_available_data(&dcc_inbuf);
        if (bytes < compsize + sizeof(unsigned int))
            break;

        /* Copy to a contiguous buffer */
        ringbuf_get(&dcc_inbuf,
                    dcc_temp_source, compsize + sizeof(unsigned int));

        sentinel &= 0xffff0000;
        buf = dcc_temp_source + sizeof(unsigned int);

        bytes = compsize;

        /* Round up to nearest whole word */
        if (compsize & 0x3)
            compsize = (compsize + 4) & ~0x3;

        /* advance by packet size + sentinel */
        ringbuf_advance_read(&dcc_inbuf, compsize + sizeof(unsigned int));

        /* allow more data to be received */
        RVIDCC_ENABLE_READ_INTERRUPT;

        /* Process TTY or IP packet accordingly */
        if (sentinel == UNCOMP_TTY_START)
        {
            ringbuf_put(&dcc_tty_inbuf, buf, bytes);
            ringbuf_advance_write(&dcc_tty_inbuf, bytes);
        }
        else
        {
            if (bytes == 4) /* ARP 'is this IP address you?' */
            {
                if (rvidcc_cb_has_addr(buf))
                    rvidcc_transmit_ip_packet(buf, 4);
            }
            else if (bytes == 6) /* set MAC address */
                rvidcc_cb_set_mac_address(buf);
            else
                rvidcc_cb_ip_packet_received(buf, (size_t)bytes);
        }
    }
}
#endif


#ifdef RVIDCC_SCAN_INPUT_FOR
int rvidcc_scan_input_for(char c, size_t span)
{
    int ret = 0;

    size_t bytesAvail = ringbuf_available_data(&DCC_TTY_INBUF);
    if (bytesAvail && span)
    {
        unsigned char *p = DCC_TTY_INBUF.readPtr;

        if (span > bytesAvail)
            span = bytesAvail;

        while (span--)
        {
            if (*p++ == c)
            {
                ret = 1;
                break;
            }

            if (p >= DCC_TTY_INBUF.buf + DCC_TTY_INBUF.size)
                p = DCC_TTY_INBUF.buf;
        }
    }   
    return ret;
}
#endif


#ifdef RVIDCC_OUTBUF_DATA
size_t rvidcc_outbuf_data(void)
{
    return ringbuf_available_data(&dcc_outbuf);
}
#endif


size_t rvidcc_serial_can_read(void)
{
    return ringbuf_available_data(&DCC_TTY_INBUF);
}


size_t rvidcc_serial_can_write(void)
{
    size_t space = ringbuf_available_space(&dcc_outbuf);

    // protocol only has 16 bits for packet size, so limit packets to this size
    if (space > (65536 - 4))
      space = (65536 - 4);

    return space;
}


size_t rvidcc_read_serial(void *ptr, size_t len)
{
    size_t bytes = ringbuf_available_data(&DCC_TTY_INBUF);
    if (bytes && len)
    {
        if (bytes > len)
            bytes = len;

        ringbuf_get(&DCC_TTY_INBUF, (unsigned char*)ptr, bytes);
#ifdef RVIDCC_RAW
        /* round up to 4 bytes */
        if (bytes & 0x3)
            ringbuf_advance_read(&DCC_TTY_INBUF, ((bytes + 4) & ~0x3));
        else
            ringbuf_advance_read(&DCC_TTY_INBUF, bytes);

        RVIDCC_ENABLE_READ_INTERRUPT;
#else
        ringbuf_advance_read(&DCC_TTY_INBUF, bytes);
#endif
    }
    else
        bytes = 0;

    return bytes;
}


static void rvidcc_write_lowlevel(const void *ptr, size_t len)
{
    size_t rem;

    ringbuf_put(&dcc_outbuf, (unsigned char*)ptr, len);

    ringbuf_advance_write(&dcc_outbuf, len & ~0x3); // advance whole words

    /* ensure excess bytes up to word boundary are null */
    rem = (len & 0x3);
    if (rem)
    {
        for (; rem < 4; rem++)
            *(dcc_outbuf.writePtr + rem) = '\0';

        ringbuf_advance_write(&dcc_outbuf, sizeof(unsigned long)); // advance remainder word
    }


    RVIDCC_ENABLE_WRITE_INTERRUPT;
}


size_t rvidcc_write_serial(const void *ptr, size_t len)
{
    size_t bytes;
#ifndef RVIDCC_RAW
    unsigned long sentinel;
#endif

    bytes = rvidcc_serial_can_write();

#ifndef RVIDCC_RAW
    if (bytes > sizeof(unsigned long))
    {
        /* Size limit the amount of data to be written, allowing for sentinel */
        if (len > (bytes - sizeof(unsigned long)))
            len = (bytes - sizeof(unsigned long));

        sentinel = to_little_endian(UNCOMP_TTY_START | (unsigned short)len);
        rvidcc_write_lowlevel(&sentinel, sizeof(unsigned long));
        rvidcc_write_lowlevel(ptr, len);
    }
#else
    if (bytes > 0)
    {
        /* Size limit the amount of data to be written */
        if (len > bytes)
            len = bytes;

        rvidcc_write_lowlevel(ptr, len);
    }
#endif
    else
        len = 0;

    return len;
}


#ifndef RVIDCC_RAW
int rvidcc_transmit_ip_packet(void *packet, size_t length)
{
    if (rvidcc_serial_can_write() >= (length + 4))
    {
        unsigned long sentinel = to_little_endian(UNCOMP_ETH_START | (unsigned short)length);
        rvidcc_write_lowlevel(&sentinel, 4);
        rvidcc_write_lowlevel(packet, length);
    }
    else
        length = 0;

    return length;
}
#endif

/* End of file */
