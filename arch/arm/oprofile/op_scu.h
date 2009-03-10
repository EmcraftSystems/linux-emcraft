/**
 * @file op_scu.c
 * MPCORE Snoop Control Unit Event Monitor Driver
 * @remark Copyright 2004 ARM SMP Development Team
 * @remark Copyright 2000-2004 Deepak Saxena <dsaxena@mvista.com>
 * @remark Copyright 2000-2004 MontaVista Software Inc
 * @remark Copyright 2004 Dave Jiang <dave.jiang@intel.com>
 * @remark Copyright 2004 Intel Corporation
 * @remark Copyright 2004 Zwane Mwaikambo <zwane@arm.linux.org.uk>
 * @remark Copyright 2004 Oprofile Authors
 *
 * @remark Read the file COPYING
 *
 * @author Zwane Mwaikambo
 */
#ifndef OP_SCU_H
#define OP_SCU_H

void scu_setup(void);
int scu_start(void);
void scu_stop(void);

#endif
