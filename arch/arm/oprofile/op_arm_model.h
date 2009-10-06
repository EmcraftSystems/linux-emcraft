/**
 * @file op_arm_model.h
 * interface to ARM machine specific operations
 *
 * @remark Copyright 2004 Oprofile Authors
 * @remark Read the file COPYING
 *
 * @author Zwane Mwaikambo
 */

#ifndef OP_ARM_MODEL_H
#define OP_ARM_MODEL_H

struct op_arm_model_spec {
	int (*init)(void);
	unsigned int num_counters;
	int (*setup_ctrs)(void);
	int (*start)(void);
	void (*stop)(void);
	char *name;
};

extern struct op_arm_model_spec op_xscale_spec;
extern struct op_arm_model_spec op_armv6_spec;
extern struct op_arm_model_spec op_armv7_spec;

extern void arm_backtrace(struct pt_regs * const regs, unsigned int depth);

extern int __init op_arm_init(struct oprofile_operations *ops, struct op_arm_model_spec *spec);
extern void op_arm_exit(void);

/*
 * The macros need to be reimplemented as things we can call at runtime,
 * along with cpu_is_xscale in system.h
 */
#ifdef CONFIG_CACHE_L2X0
#define have_l2x0() 1
#else
#define have_l2x0() 0
#endif
#ifdef CONFIG_SMP
#define is_smp() 1
#else
#define is_smp() 0
#endif
#ifdef CONFIG_REALVIEW_EB_A9MP
#define cpu_is_a9() 1
#else
#define cpu_is_a9() 0
#endif
#if defined(CONFIG_REALVIEW_EB_ARM11MP) || defined(CONFIG_MACH_REALVIEW_PB11MP)
#define cpu_is_11mpcore() 1
#else
#define cpu_is_11mpcore() 0
#endif
#ifdef CONFIG_MACH_REALVIEW_PBX
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/board-pbx.h>
#undef cpu_is_11mpcore
#define cpu_is_11mpcore() core_tile_pbx11mp()
#undef cpu_is_a9
#define cpu_is_a9() core_tile_pbxa9mp()
#endif

/*
 * ARM11MPCore SCU event monitor support
 */
#ifdef CONFIG_SMP
#if defined(CONFIG_MACH_REALVIEW_EB)
#define SCU_EVENTMONITORS_VA_BASE __io_address(REALVIEW_EB11MP_SCU_BASE + 0x10)
#elif defined(CONFIG_MACH_REALVIEW_PB11MP)
#define SCU_EVENTMONITORS_VA_BASE __io_address(REALVIEW_TC11MP_SCU_BASE + 0x10)
#elif defined(CONFIG_MACH_REALVIEW_PBX)
#define SCU_EVENTMONITORS_VA_BASE __io_address(REALVIEW_PBX_TILE_SCU_BASE + 0x10)
#else
#error Cannot determine the base address of the SCU
#endif
#endif

/*
 * IRQ numbers for PMUs (A9MPCore and 11MPCore) and SCU (11MPCore only)
 */
#if defined(CONFIG_MACH_REALVIEW_EB) || defined(CONFIG_MACH_REALVIEW_PB11MP)
#define IRQ_PMU_CPU0	IRQ_TC11MP_PMU_CPU0
#define IRQ_PMU_CPU1	IRQ_TC11MP_PMU_CPU1
#define IRQ_PMU_CPU2	IRQ_TC11MP_PMU_CPU2
#define IRQ_PMU_CPU3	IRQ_TC11MP_PMU_CPU3
#define IRQ_PMU_SCU0	IRQ_TC11MP_PMU_SCU0
#define IRQ_PMU_SCU1	IRQ_TC11MP_PMU_SCU1
#define IRQ_PMU_SCU2	IRQ_TC11MP_PMU_SCU2
#define IRQ_PMU_SCU3	IRQ_TC11MP_PMU_SCU3
#define IRQ_PMU_SCU4	IRQ_TC11MP_PMU_SCU4
#define IRQ_PMU_SCU5	IRQ_TC11MP_PMU_SCU5
#define IRQ_PMU_SCU6	IRQ_TC11MP_PMU_SCU6
#define IRQ_PMU_SCU7	IRQ_TC11MP_PMU_SCU7
#elif defined(CONFIG_MACH_REALVIEW_PBX)
#define IRQ_PMU_CPU0	IRQ_PBX_PMU_CPU0
#define IRQ_PMU_CPU1	IRQ_PBX_PMU_CPU1
#define IRQ_PMU_CPU2	IRQ_PBX_PMU_CPU2
#define IRQ_PMU_CPU3	IRQ_PBX_PMU_CPU3
#define IRQ_PMU_SCU0	IRQ_PBX_PMU_SCU0
#define IRQ_PMU_SCU1	IRQ_PBX_PMU_SCU1
#define IRQ_PMU_SCU2	IRQ_PBX_PMU_SCU2
#define IRQ_PMU_SCU3	IRQ_PBX_PMU_SCU3
#define IRQ_PMU_SCU4	IRQ_PBX_PMU_SCU4
#define IRQ_PMU_SCU5	IRQ_PBX_PMU_SCU5
#define IRQ_PMU_SCU6	IRQ_PBX_PMU_SCU6
#define IRQ_PMU_SCU7	IRQ_PBX_PMU_SCU7
#elif defined(CONFIG_ARCH_OMAP2)
#define IRQ_PMU_CPU0   3
#else
#define IRQ_PMU_CPU0	NO_IRQ
#endif

#endif /* OP_ARM_MODEL_H */
