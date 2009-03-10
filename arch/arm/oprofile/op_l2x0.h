/**
 * @file op_model_l220.h
 * ARM L220/L230 Level 2 Cache Controller Event Counter Driver
 * @remark Copyright 2007 ARM SMP Development Team
 *
 * @remark Read the file COPYING
 */
#ifndef OP_MODEL_L2X0_H
#define OP_MODEL_L2X0_H

void l2x0_ec_setup(void);
int l2x0_ec_start(void);
void l2x0_ec_stop(void);

#endif
