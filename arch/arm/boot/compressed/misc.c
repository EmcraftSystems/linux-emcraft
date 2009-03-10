/*
 * misc.c
 * 
 * This is a collection of several routines from gzip-1.0.3 
 * adapted for Linux.
 *
 * malloc by Hannu Savolainen 1993 and Matthias Urlichs 1994
 *
 * Modified for ARM Linux by Russell King
 *
 * Nicolas Pitre <nico@visuaide.com>  1999/04/14 :
 *  For this code to run directly from Flash, all constant variables must
 *  be marked with 'const' and all other variables initialized at run-time 
 *  only.  This way all non constant variables will end up in the bss segment,
 *  which should point to addresses in RAM and cleared to 0 on start.
 *  This allows for a much quicker boot time.
 */

unsigned int __machine_arch_type;

#include <linux/string.h>
#include <mach/uncompress.h>

#ifdef STANDALONE_DEBUG
#define putstr printf
#elif defined(CONFIG_DEBUG_ICEDCC) || defined(CONFIG_DEBUG_RVIDCC)
#define putstr icedcc_putstr
#else

static void putstr(const char *ptr)
{
	char c;

	while ((c = *ptr++) != '\0') {
		if (c == '\n')
			putc('\r');
		putc(c);
	}

	flush();
}

#endif

#define __ptr_t void *

/*
 * Optimised C version of memzero for the ARM.
 */
void __memzero (__ptr_t s, size_t n)
{
	union { void *vp; unsigned long *ulp; unsigned char *ucp; } u;
	int i;

	u.vp = s;

	for (i = n >> 5; i > 0; i--) {
		*u.ulp++ = 0;
		*u.ulp++ = 0;
		*u.ulp++ = 0;
		*u.ulp++ = 0;
		*u.ulp++ = 0;
		*u.ulp++ = 0;
		*u.ulp++ = 0;
		*u.ulp++ = 0;
	}

	if (n & 1 << 4) {
		*u.ulp++ = 0;
		*u.ulp++ = 0;
		*u.ulp++ = 0;
		*u.ulp++ = 0;
	}

	if (n & 1 << 3) {
		*u.ulp++ = 0;
		*u.ulp++ = 0;
	}

	if (n & 1 << 2)
		*u.ulp++ = 0;

	if (n & 1 << 1) {
		*u.ucp++ = 0;
		*u.ucp++ = 0;
	}

	if (n & 1)
		*u.ucp++ = 0;
}

static inline __ptr_t memcpy(__ptr_t __dest, __const __ptr_t __src,
			    size_t __n)
{
	int i = 0;
	unsigned char *d = (unsigned char *)__dest, *s = (unsigned char *)__src;

	for (i = __n >> 3; i > 0; i--) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (__n & 1 << 2) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (__n & 1 << 1) {
		*d++ = *s++;
		*d++ = *s++;
	}

	if (__n & 1)
		*d++ = *s++;

	return __dest;
}

/*
 * gzip delarations
 */
#define OF(args)  args
#define STATIC static

typedef unsigned char  uch;
typedef unsigned short ush;
typedef unsigned long  ulg;

#define WSIZE 0x8000		/* Window size must be at least 32k, */
				/* and a power of two */

static uch *inbuf;		/* input buffer */
static uch window[WSIZE];	/* Sliding window buffer */

static unsigned insize;		/* valid bytes in inbuf */
static unsigned inptr;		/* index of next byte to be processed in inbuf */
static unsigned outcnt;		/* bytes in output buffer */

/* gzip flag byte */
#define ASCII_FLAG   0x01 /* bit 0 set: file probably ascii text */
#define CONTINUATION 0x02 /* bit 1 set: continuation of multi-part gzip file */
#define EXTRA_FIELD  0x04 /* bit 2 set: extra field present */
#define ORIG_NAME    0x08 /* bit 3 set: original file name present */
#define COMMENT      0x10 /* bit 4 set: file comment present */
#define ENCRYPTED    0x20 /* bit 5 set: file is encrypted */
#define RESERVED     0xC0 /* bit 6,7:   reserved */

#define get_byte()  (inptr < insize ? inbuf[inptr++] : fill_inbuf())

/* Diagnostic functions */
#ifdef DEBUG
#  define Assert(cond,msg) {if(!(cond)) error(msg);}
#  define Trace(x) fprintf x
#  define Tracev(x) {if (verbose) fprintf x ;}
#  define Tracevv(x) {if (verbose>1) fprintf x ;}
#  define Tracec(c,x) {if (verbose && (c)) fprintf x ;}
#  define Tracecv(c,x) {if (verbose>1 && (c)) fprintf x ;}
#else
#  define Assert(cond,msg)
#  define Trace(x)
#  define Tracev(x)
#  define Tracevv(x)
#  define Tracec(c,x)
#  define Tracecv(c,x)
#endif

static int  fill_inbuf(void);
static void flush_window(void);
static void error(char *m);

extern char input_data[];
extern char input_data_end[];

static uch *output_data;
static ulg output_ptr;
static ulg bytes_out;

static void error(char *m);

static void putstr(const char *);

extern int end;
static ulg free_mem_ptr;
static ulg free_mem_end_ptr;

#ifdef STANDALONE_DEBUG
#define NO_INFLATE_MALLOC
#endif

#define ARCH_HAS_DECOMP_WDOG

#include "../../../../lib/inflate.c"

/* ===========================================================================
 * Fill the input buffer. This is called only when the buffer is empty
 * and at least one byte is really needed.
 */
int fill_inbuf(void)
{
	if (insize != 0)
		error("ran out of input data");

	inbuf = input_data;
	insize = &input_data_end[0] - &input_data[0];

	inptr = 1;
	return inbuf[0];
}

/* ===========================================================================
 * Write the output window window[0..outcnt-1] and update crc and bytes_out.
 * (Used for the decompressed data only.)
 */
void flush_window(void)
{
	ulg c = crc;
	unsigned n;
	uch *in, *out, ch;

	in = window;
	out = &output_data[output_ptr];
	for (n = 0; n < outcnt; n++) {
		ch = *out++ = *in++;
		c = crc_32_tab[((int)c ^ ch) & 0xff] ^ (c >> 8);
	}
	crc = c;
	bytes_out += (ulg)outcnt;
	output_ptr += (ulg)outcnt;
	outcnt = 0;
	putstr(".");
}

#ifndef arch_error
#define arch_error(x)
#endif

static void error(char *x)
{
	arch_error(x);

	putstr("\n\n");
	putstr(x);
	putstr("\n\n -- System halted");

	while(1);	/* Halt */
}

#ifndef STANDALONE_DEBUG

ulg
decompress_kernel(ulg output_start, ulg free_mem_ptr_p, ulg free_mem_ptr_end_p,
		  int arch_id)
{
	output_data		= (uch *)output_start;	/* Points to kernel start */
	free_mem_ptr		= free_mem_ptr_p;
	free_mem_end_ptr	= free_mem_ptr_end_p;
	__machine_arch_type	= arch_id;

	arch_decomp_setup();

	makecrc();
	putstr("Uncompressing Linux...");
	gunzip();
	putstr(" done, booting the kernel.\n");
	return output_ptr;
}
#else

char output_buffer[1500*1024];

int main()
{
	output_data = output_buffer;

	makecrc();
	putstr("Uncompressing Linux...");
	gunzip();
	putstr("done.\n");
	return 0;
}
#endif
	
#if defined(CONFIG_DEBUG_ICEDCC) || defined(CONFIG_DEBUG_RVIDCC)

#define _DCC_ARM9_RBIT  (1 << 0)
#define _DCC_ARM9_WBIT  (1 << 1)
#define _DCC_ARM10_RBIT (1 << 7)
#define _DCC_ARM10_WBIT (1 << 6)
#define _DCC_ARM11_RBIT (1 << 30)
#define _DCC_ARM11_WBIT (1 << 29)

#define _READ_CORE_ID(x) { __asm__ ("mrc p15, 0, %0, c0, c0, 0\n" : "=r" (x)); \
                           x = (x >> 4) & 0xFFF; }

#define _WRITE_ARM9_DCC(x) __asm__ volatile ("mcr p14, 0, %0, c1, c0, 0\n" : : "r" (x))
#define _READ_ARM9_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c1, c0, 0\n" : "=r" (x))
#define _STATUS_ARM9_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c0, 0\n" : "=r" (x))
#define _CAN_READ_ARM9_DCC(x) {_STATUS_ARM9_DCC(x); x &= _DCC_ARM9_RBIT;}
#define _CAN_WRITE_ARM9_DCC(x) {_STATUS_ARM9_DCC(x); x &= _DCC_ARM9_WBIT; x = (x==0);}

#define _WRITE_ARM10_DCC(x) __asm__ volatile ("mcr p14, 0, %0, c0, c5, 0\n" : : "r" (x))
#define _READ_ARM10_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c5, 0\n" : "=r" (x))
#define _STATUS_ARM10_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c1, 0\n" : "=r" (x))
#define _CAN_READ_ARM10_DCC(x) {_STATUS_ARM10_DCC(x); x &= _DCC_ARM10_RBIT;}
#define _CAN_WRITE_ARM10_DCC(x) {_STATUS_ARM10_DCC(x); x &= _DCC_ARM10_WBIT; x = (x==0);}

#define _WRITE_ARM11_DCC(x) __asm__ volatile ("mcr p14, 0, %0, c0, c5, 0\n" : : "r" (x))
#define _READ_ARM11_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c5, 0\n" : "=r" (x))
#define _STATUS_ARM11_DCC(x) __asm__ volatile ("mrc p14, 0, %0, c0, c1, 0\n" : "=r" (x))
#define _CAN_READ_ARM11_DCC(x) {_STATUS_ARM11_DCC(x); x &= _DCC_ARM11_RBIT;}
#define _CAN_WRITE_ARM11_DCC(x) {_STATUS_ARM11_DCC(x); x &= _DCC_ARM11_WBIT; x = (x==0);}

#define TIMEOUT_COUNT 0x4000000

void icedcc_putc(unsigned int ch)
{
    static enum {unknown, arm9_and_earlier, arm10, arm11_and_later} _arm_type = unknown;
    static int has_timed_out = 0;

    if (has_timed_out)
        return;

    if (_arm_type == unknown)
    {
        register unsigned int id;
        _READ_CORE_ID(id);

        if ((id & 0xF00) == 0xA00)
            _arm_type = arm10;
        else if (id >= 0xb00)
            _arm_type = arm11_and_later;
        else
            _arm_type = arm9_and_earlier;
    }

    if (_arm_type == arm9_and_earlier)
    {
        register unsigned int reg;
        unsigned int timeout_count = TIMEOUT_COUNT;
        while (--timeout_count)
        {
            _CAN_WRITE_ARM9_DCC(reg);
            if (reg)
                break;
        }
        if (timeout_count == 0)
            has_timed_out = 1;
        else
            _WRITE_ARM9_DCC(ch);
    }
    else if (_arm_type == arm10)
    {
        register unsigned int reg;
        unsigned int timeout_count = TIMEOUT_COUNT;
        while (--timeout_count)
        {
            _CAN_WRITE_ARM10_DCC(reg);
            if (reg)
                break;
        }
        if (timeout_count == 0)
            has_timed_out = 1;
        else
            _WRITE_ARM10_DCC(ch);
    }
    else
    {
        register unsigned int reg;
        unsigned int timeout_count = TIMEOUT_COUNT;
        while (--timeout_count)
        {
            _CAN_WRITE_ARM11_DCC(reg);
            if (reg)
                break;
        }
        if (timeout_count == 0)
            has_timed_out = 1;
        else
            _WRITE_ARM11_DCC(ch);
    }
}

static void icedcc_putstr(const char *ptr)
{
#if defined(CONFIG_DEBUG_ICEDCC)
	for (; *ptr != '\0'; ptr++)
		icedcc_putc(*ptr);
#else
    unsigned int sendbuf[16];
    unsigned short cnt;
    char *ptr1 = (char*)&sendbuf[1];
    unsigned int *wordptr;

    for (cnt=0; *ptr != '\0'; ptr++)
    {
        if (*ptr == '\n')
        {
            *ptr1++ = '\r';
            cnt++;
        }
        *ptr1++ = *ptr;
        cnt++;
    }

    *sendbuf = 0xa5580000 | cnt;

    while(cnt % 4)
    {
        *ptr1++ = '\0';
        cnt++;
    }

    cnt /= 4;

#ifdef CONFIG_DEBUG_DCC_RAW
    wordptr = &sendbuf[1];
#else
    wordptr = &sendbuf[0];
    cnt++;
#endif

    while(cnt--)
        icedcc_putc(*wordptr++);

#endif
}

#endif
