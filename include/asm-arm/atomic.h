/*
 *  linux/include/asm-arm/atomic.h
 *
 *  Copyright (C) 1996 Russell King.
 *  Copyright (C) 2002 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARM_ATOMIC_H
#define __ASM_ARM_ATOMIC_H

#include <linux/compiler.h>
#include <asm/system.h>

typedef struct { volatile int counter; } atomic_t;

#define ATOMIC_INIT(i)	{ (i) }

#ifdef __KERNEL__

#define atomic_read(v)	((v)->counter)
#define atomic_set(v,i)	(((v)->counter) = (i))

#if __LINUX_ARM_ARCH__ >= 6

#ifdef CONFIG_ARM_ERRATA_351422
static inline int atomic_backoff_delay(void)
{
	unsigned int delay;
	__asm__ __volatile__(
	"	mrc	p15, 0, %0, c0, c0, 5\n"
	"	and	%0, %0, #0xf\n"
	"	mov	%0, %0, lsl #8\n"
	"1:	subs	%0, %0, #1\n"
	"	bpl	1b\n"
	: "=&r" (delay)
	:
	: "cc" );

	return 1;
}
#else
#define atomic_backoff_delay()	1
#endif

/*
 * ARMv6 UP and SMP safe atomic ops.  We use load exclusive and
 * store exclusive to ensure that these are atomic.  We may loop
 * to ensure that the update happens.  Writing to 'v->counter'
 * without using the following operations WILL break the atomic
 * nature of these ops.
 */
#if 0	/* atomic set implemented as a simple STR by a previous patch */
static inline void atomic_set(atomic_t *v, int i)
{
	unsigned long tmp;

	do {
	__asm__ __volatile__("@ atomic_set\n"
"1:	ldrex	%0, [%1]\n"
"	strex	%0, %2, [%1]\n"
	: "=&r" (tmp)
	: "r" (&v->counter), "r" (i)
	: "cc");
	} while (tmp && atomic_backoff_delay());
}
#endif

static inline int atomic_add_return(int i, atomic_t *v)
{
	unsigned long tmp;
	int result;

	do {
	__asm__ __volatile__("@ atomic_add_return\n"
"1:	ldrex	%0, [%2]\n"
"	add	%0, %0, %3\n"
"	strex	%1, %0, [%2]\n"
	: "=&r" (result), "=&r" (tmp)
	: "r" (&v->counter), "Ir" (i)
	: "cc");
	} while (tmp && atomic_backoff_delay());

	return result;
}

static inline int atomic_sub_return(int i, atomic_t *v)
{
	unsigned long tmp;
	int result;

	do {
	__asm__ __volatile__("@ atomic_sub_return\n"
"1:	ldrex	%0, [%2]\n"
"	sub	%0, %0, %3\n"
"	strex	%1, %0, [%2]\n"
	: "=&r" (result), "=&r" (tmp)
	: "r" (&v->counter), "Ir" (i)
	: "cc");
	} while (tmp && atomic_backoff_delay());

	return result;
}

static inline int atomic_cmpxchg(atomic_t *ptr, int old, int new)
{
	unsigned long oldval, res;

	do {
		__asm__ __volatile__("@ atomic_cmpxchg\n"
		"ldrex	%1, [%2]\n"
		"mov	%0, #0\n"
		"teq	%1, %3\n"
		"it	eq\n"
		"strexeq %0, %4, [%2]\n"
		    : "=&r" (res), "=&r" (oldval)
		    : "r" (&ptr->counter), "Ir" (old), "r" (new)
		    : "cc");
	} while (res && atomic_backoff_delay());

	return oldval;
}

static inline void atomic_clear_mask(unsigned long mask, unsigned long *addr)
{
	unsigned long tmp, tmp2;

	do {
	__asm__ __volatile__("@ atomic_clear_mask\n"
"1:	ldrex	%0, [%2]\n"
"	bic	%0, %0, %3\n"
"	strex	%1, %0, [%2]\n"
	: "=&r" (tmp), "=&r" (tmp2)
	: "r" (addr), "Ir" (mask)
	: "cc");
	} while (tmp && atomic_backoff_delay());
}

#else /* ARM_ARCH_6 */

#include <asm/system.h>

#ifdef CONFIG_SMP
#error SMP not supported on pre-ARMv6 CPUs
#endif

static inline int atomic_add_return(int i, atomic_t *v)
{
	unsigned long flags;
	int val;

	raw_local_irq_save(flags);
	val = v->counter;
	v->counter = val += i;
	raw_local_irq_restore(flags);

	return val;
}

static inline int atomic_sub_return(int i, atomic_t *v)
{
	unsigned long flags;
	int val;

	raw_local_irq_save(flags);
	val = v->counter;
	v->counter = val -= i;
	raw_local_irq_restore(flags);

	return val;
}

static inline int atomic_cmpxchg(atomic_t *v, int old, int new)
{
	int ret;
	unsigned long flags;

	raw_local_irq_save(flags);
	ret = v->counter;
	if (likely(ret == old))
		v->counter = new;
	raw_local_irq_restore(flags);

	return ret;
}

static inline void atomic_clear_mask(unsigned long mask, unsigned long *addr)
{
	unsigned long flags;

	raw_local_irq_save(flags);
	*addr &= ~mask;
	raw_local_irq_restore(flags);
}

#endif /* __LINUX_ARM_ARCH__ */

#define atomic_xchg(v, new) (xchg(&((v)->counter), new))

static inline int atomic_add_unless(atomic_t *v, int a, int u)
{
	int c, old;

	c = atomic_read(v);
	while (c != u && (old = atomic_cmpxchg((v), c, c + a)) != c)
		c = old;
	return c != u;
}
#define atomic_inc_not_zero(v) atomic_add_unless((v), 1, 0)

#define atomic_add(i, v)	(void) atomic_add_return(i, v)
#define atomic_inc(v)		(void) atomic_add_return(1, v)
#define atomic_sub(i, v)	(void) atomic_sub_return(i, v)
#define atomic_dec(v)		(void) atomic_sub_return(1, v)

#define atomic_inc_and_test(v)	(atomic_add_return(1, v) == 0)
#define atomic_dec_and_test(v)	(atomic_sub_return(1, v) == 0)
#define atomic_inc_return(v)    (atomic_add_return(1, v))
#define atomic_dec_return(v)    (atomic_sub_return(1, v))
#define atomic_sub_and_test(i, v) (atomic_sub_return(i, v) == 0)

#define atomic_add_negative(i,v) (atomic_add_return(i, v) < 0)

/* Atomic operations are already serializing on ARM */
#define smp_mb__before_atomic_dec()	barrier()
#define smp_mb__after_atomic_dec()	barrier()
#define smp_mb__before_atomic_inc()	barrier()
#define smp_mb__after_atomic_inc()	barrier()

#include <asm-generic/atomic.h>
#endif
#endif
