/**
 * @file op_v7.h
 * ARM V7 Performance Monitor Unit Driver
 *
 * @remark Copyright 2007 ARM SMP Development Team
 *
 * @remark Read the file COPYING
 */
#ifndef OP_V7_H
#define OP_V7_H

/*
 * V7 CP15 PMU
 */
#define PMCR_E		(1 << 0)	/* Enable */
#define PMCR_P		(1 << 1)	/* Count reset */
#define PMCR_C		(1 << 2)	/* Cycle counter reset */
#define PMCR_D		(1 << 3)	/* Cycle counter counts every 64th cpu cycle */

#define PMINTEN_PMN0	(1 << 0)
#define PMINTEN_CCNT	(1 << 31)

#define PMOVSR_PMN0	(1 << 0)
#define PMOVSR_CCNT	(1 << 31)

#define PMCNTEN_PMN0	(1 << 0)
#define PMCNTEN_CCNT	(1 << 31)


int v7_setup_pmu(void);
int v7_start_pmu(void);
int v7_stop_pmu(void);
int v7_request_interrupts(int *, int);
void v7_release_interrupts(int *, int);

#endif
