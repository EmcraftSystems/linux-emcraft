/**
 * @file op_counter.h
 *
 * @remark Copyright 2004 Oprofile Authors
 * @remark Read the file COPYING
 *
 * @author Zwane Mwaikambo
 */

#ifndef OP_COUNTER_H
#define OP_COUNTER_H

/* Per performance monitor configuration as set via
 * oprofilefs.
 */
struct op_counter_config {
	unsigned long count;
	unsigned long enabled;
	unsigned long event;
	unsigned long unit_mask;
	unsigned long kernel;
	unsigned long user;
};

extern struct op_counter_config *counter_config;


/*
 * List of userspace counter numbers: we use the same layout for both
 * the V6 and V7 oprofile models.
 *  0- 7 CPU0 event counters and cycle counter
 *  8-15 CPU1 event counters and cycle counter
 * 16-23 CPU2 event counters and cycle counter
 * 24-31 CPU3 event counters and cycle counter
 * 32-39 SCU counters
 * 40-41 L2X0 counters
 */

#define PMU_COUNTERS_PER_CPU	8 /* 7 event counters, 1 cycle counter */
#define CCNT			(PMU_COUNTERS_PER_CPU - 1)
#define MAX_CPUS		4

#define COUNTER_CPUn_PMNm(N,M) ((N) * PMU_COUNTERS_PER_CPU + (M))
#define COUNTER_CPUn_CCNT(N)   ((N+1) * PMU_COUNTERS_PER_CPU - 1)

#define COUNTER_SCU_MN(N) (PMU_COUNTERS_PER_CPU * MAX_CPUS + (N))
#define NUM_SCU_COUNTERS 8

#define COUNTER_L2X0_EC(N) (COUNTER_SCU_MN(NUM_SCU_COUNTERS) + (N))
#define L2X0_NUM_COUNTERS 2

#define NUM_COUNTERS	COUNTER_L2X0_EC(L2X0_NUM_COUNTERS)

#endif /* OP_COUNTER_H */
