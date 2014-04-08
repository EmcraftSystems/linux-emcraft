/*
 * arch/arm/mach-kinetis/pm.c
 * Kinetis PM code	
 *
 * Copyright (C) 2012 Robert Brehm, <brehm@mci.sdu.dk>
 * Copyright (C) 2014 Emcraft Systems
 * Vladimir Khusainov, Emcraft Systems, <vlad@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>

#include <mach/power.h>
#include <mach/clock.h>
#include <mach/kinetis.h>
#include <mach/memory.h>
#include <mach/platform.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/power.h>
#include <mach/pm.h>

/*
 * Assembler-level entry point to suspend the system.
 * The second symbol is a dummy entry point to mark 
 * the end of the eSRAM-based suspend code. 
 */
extern void kinetis_suspend_to_ram(void);
extern void kinetis_suspend_to_ram_end(void);

/*
 * Location in eSRAM where the low-level suspend code will run from
 */
#define KINETIS_SUSPEND_CODE_BASE	0x20002000

/*
 * Various bit fields in various hardware registers
 */
#define KINETIS_SCB_SCR_DEEPSLEEP	(1<<2)
#define KINETIS_SIM_SOPT1_USBSSTBY	(1<<30)
#define KINETIS_SIM_SOPT1CFG_USSWE	(1<<26)
#define KINETIS_SIM_FCFG1_FTFDIS	(1<<0)
#define KINETIS_SMC_PMRPOT_AVLP		(1<<5)
#define KINETIS_SMC_PMCTRL_STOPM_RUN	0x0
#define KINETIS_SMC_PMCTRL_STOPM_VLPS	0x2
#define KINETIS_MCG_C1_IREFSTEN		(1<<0)
#define KINETIS_MCG_C2_LP		(1<<1)
#define KINETIS_MCG_C5_PLLSTEN0		(1<<5)
#define KINETIS_MCG_C11_PLLSTEN1	(1<<5)
#define KINETIS_OSC0_CR_EREFSTEN	(1<<5)
#define KINETIS_OSC1_CR_EREFSTEN	(1<<5)
#define KINETIS_WDOG_STCTRLH_STOPE	(1<<6)

/*
 * Local to the module globals
 */
static u32 kinetis_pm_sim_fcfg1;

/*
 * Validate suspend state
 * @state		State being entered
 * @returns		1->valid, 0->invalid
 */
static int kinetis_pm_valid(suspend_state_t state)
{
	int ret;

	switch (state) {

	case PM_SUSPEND_ON:
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

/*
 * Prepare system hardware to suspend
 */
static void kinetis_pm_prepare_to_suspend(void)
{
	/*
	 * Disable internal Flash
	 */
	kinetis_pm_sim_fcfg1 = readl(&KINETIS_SIM->fcfg1);
	writel(KINETIS_SIM_FCFG1_FTFDIS, &KINETIS_SIM->fcfg1);

	/*
	 * WDOG_STCTLH[STOPEN] = 0. 
	 * Disable Watchdog on STOP.
	 */
	writew(readw(&KINETIS_WDOG->stctrlh) & ~KINETIS_WDOG_STCTRLH_STOPE,
		 &KINETIS_WDOG->stctrlh);

	/*
 	 * Put USB voltage regulator in stand-by on STOP
 	 */
	writel(readl(&KINETIS_SIM->sopt1cfg) | KINETIS_SIM_SOPT1CFG_USSWE, 
		&KINETIS_SIM->sopt1cfg);
	writel(readl(&KINETIS_SIM->sopt1) | KINETIS_SIM_SOPT1_USBSSTBY,
		&KINETIS_SIM->sopt1);

	/*
	 * Disable internal reference clock on STOP.
	 */
	writeb(readb(&KINETIS_MCG->c1) & ~KINETIS_MCG_C1_IREFSTEN,
		&KINETIS_MCG->c1);

	/*
	 * Disable FLL (or PLL) in bypass modes.
	 */
	writeb(readb(&KINETIS_MCG->c2) | KINETIS_MCG_C2_LP,
		&KINETIS_MCG->c2);
	
	/*
	 * Disable PLL0 on STOP.
	 */
	writeb(readb(&KINETIS_MCG->c5) & ~KINETIS_MCG_C5_PLLSTEN0,
		&KINETIS_MCG->c5);

	/*
	 * Disable PLL1 on STOP.
	 */
	writeb(readb(&KINETIS_MCG->c11) & ~KINETIS_MCG_C11_PLLSTEN1,
		&KINETIS_MCG->c11);
	
	/*
	 * Disable OSC0 on STOP.
	 */
	writeb(readb(&KINETIS_OSC0->cr) & ~KINETIS_OSC0_CR_EREFSTEN,
		&KINETIS_OSC0->cr);

	/*
	 * Disable OSC1 on STOP.
	 */
	kinetis_periph_enable(KINETIS_CG_OSC1, 1);
	writeb(readb(&KINETIS_OSC1->cr) & ~KINETIS_OSC1_CR_EREFSTEN,
		&KINETIS_OSC1->cr);
	kinetis_periph_enable(KINETIS_CG_OSC1, 0);

	/*
	 * Disable peripheral clocks, having stored
	 * the current state of the clock registers
	 */
	kinetis_periph_push();
	kinetis_periph_enable(KINETIS_CG_OSC1, 0);
	kinetis_periph_enable(KINETIS_CG_UART4, 0);
	kinetis_periph_enable(KINETIS_CG_UART5, 0);
	kinetis_periph_enable(KINETIS_CG_ENET, 0);
	kinetis_periph_enable(KINETIS_CG_DAC0, 0);
	kinetis_periph_enable(KINETIS_CG_DAC1, 0);
	kinetis_periph_enable(KINETIS_CG_RNGA, 0);
	kinetis_periph_enable(KINETIS_CG_FLEXCAN1, 0);
	kinetis_periph_enable(KINETIS_CG_NFC, 0);
	kinetis_periph_enable(KINETIS_CG_SPI2, 0);
	// kinetis_periph_enable(KINETIS_CG_DDR, 0);
	kinetis_periph_enable(KINETIS_CG_SAI1, 0);
	kinetis_periph_enable(KINETIS_CG_ESDHC, 0);
	kinetis_periph_enable(KINETIS_CG_LCDC, 0);
	kinetis_periph_enable(KINETIS_CG_FTM2, 0);
	kinetis_periph_enable(KINETIS_CG_FTM3, 0);
	kinetis_periph_enable(KINETIS_CG_ADC1, 0);
	kinetis_periph_enable(KINETIS_CG_ADC3, 0);
	kinetis_periph_enable(KINETIS_CG_EWM, 0);
	kinetis_periph_enable(KINETIS_CG_CMT, 0);
	kinetis_periph_enable(KINETIS_CG_I2C0, 0);
	kinetis_periph_enable(KINETIS_CG_I2C1, 0);
	kinetis_periph_enable(KINETIS_CG_UART0, 0);
	kinetis_periph_enable(KINETIS_CG_UART1, 0);
	//kinetis_periph_enable(KINETIS_CG_UART2, 0);
	kinetis_periph_enable(KINETIS_CG_UART3, 0);
	kinetis_periph_enable(KINETIS_CG_USBFS, 0);
	kinetis_periph_enable(KINETIS_CG_CMP, 0);
	kinetis_periph_enable(KINETIS_CG_VREF, 0);
	kinetis_periph_enable(KINETIS_CG_LLWU, 0);
	kinetis_periph_enable(KINETIS_CG_LPTIMER, 0);
	// kinetis_periph_enable(KINETIS_CG_REGFILE, 0);
	kinetis_periph_enable(KINETIS_CG_DRYICE, 0);
	kinetis_periph_enable(KINETIS_CG_DRYICESECREG, 0);
	kinetis_periph_enable(KINETIS_CG_TSI, 0);
	kinetis_periph_enable(KINETIS_CG_PORTA, 0);
	kinetis_periph_enable(KINETIS_CG_PORTB, 0);
	kinetis_periph_enable(KINETIS_CG_PORTC, 0);
	kinetis_periph_enable(KINETIS_CG_PORTD, 0);
	kinetis_periph_enable(KINETIS_CG_PORTE, 0);
	kinetis_periph_enable(KINETIS_CG_PORTF, 0);
	kinetis_periph_enable(KINETIS_CG_DMAMUX0, 0);
	kinetis_periph_enable(KINETIS_CG_DMAMUX1, 0);
	kinetis_periph_enable(KINETIS_CG_FLEXCAN0, 0);
	kinetis_periph_enable(KINETIS_CG_SPI0, 0);
	kinetis_periph_enable(KINETIS_CG_SPI1, 0);
	kinetis_periph_enable(KINETIS_CG_SAI0, 0);
	kinetis_periph_enable(KINETIS_CG_CRC, 0);
	kinetis_periph_enable(KINETIS_CG_USBHS, 0);
	kinetis_periph_enable(KINETIS_CG_USBDCD, 0);
	kinetis_periph_enable(KINETIS_CG_PDB, 0);
	kinetis_periph_enable(KINETIS_CG_PIT, 0);
	kinetis_periph_enable(KINETIS_CG_FTM0, 0);
	kinetis_periph_enable(KINETIS_CG_FTM1, 0);
	kinetis_periph_enable(KINETIS_CG_ADC0, 0);
	kinetis_periph_enable(KINETIS_CG_ADC2, 0);
	// kinetis_periph_enable(KINETIS_CG_RTC, 0);
	kinetis_periph_enable(KINETIS_CG_FLEXBUS, 0);
	kinetis_periph_enable(KINETIS_CG_DMA, 0);
	kinetis_periph_enable(KINETIS_CG_MPU, 0);
}

/*
 * Prepare system hardware to resume
 */
static void kinetis_pm_prepare_to_resume(void)
{

	/*
	 * Restore peripheral clocks
	 */
	kinetis_periph_pop();

	/*
	 * Restore the previous state of the internal Flash
	 */
	writel(kinetis_pm_sim_fcfg1, &KINETIS_SIM->fcfg1);
}

/*
 * Enter suspend
 * @state		State being entered
 * @returns		0->success, <0->error code
 */
static int kinetis_pm_enter(suspend_state_t state)
{
	void (* ptr)(void);
	int ret = 0;

	/*
	 * Prepare the system hardware to suspend
	 */
	kinetis_pm_prepare_to_suspend();

	/*
	 * Allow very low power modes.
	 * Switch to Very Low Power Stop mode on WFI.
	 * Enter DEEP SLEEP (vs SLEEP) on WFI.
	 */
	writeb(KINETIS_SMC_PMRPOT_AVLP, &KINETIS_SMC->pmprot);
	writeb(KINETIS_SMC_PMCTRL_STOPM_VLPS, &KINETIS_SMC->pmctrl);
	writel(readl(&KINETIS_SCB->scr) | KINETIS_SCB_SCR_DEEPSLEEP,
		&KINETIS_SCB->scr);
	
	/*
	 * Jump to suspend code in SRAM
	 */
	ptr = (void *)(KINETIS_SUSPEND_CODE_BASE + 1);
	ptr();

	/*
	 * Switch to Normal Stop mode on WFI.
	 * Disable DEEP SLEEP on WFI.
	 */
	writeb(KINETIS_SMC_PMCTRL_STOPM_RUN, &KINETIS_SMC->pmctrl);
	writel(readl(&KINETIS_SCB->scr) & ~KINETIS_SCB_SCR_DEEPSLEEP,
		&KINETIS_SCB->scr);

	/*
	 * Prepare the system hardware to suspend
	 */
	kinetis_pm_prepare_to_resume();

	return ret;
}

/*
 * Power Management operations 
 */
static struct platform_suspend_ops kinetis_pm_ops = {
	.valid = kinetis_pm_valid,
	.enter = kinetis_pm_enter,
};

/*
 * Device data structure
 */
static struct platform_driver kinetis_pm_driver = {
	.driver = {
		   .name = "kinetis_pm",
	},
};

/*
 * Driver init
 * @returns		0->success, <0->error code
 */
static int __init kinetis_pm_init(void)
{
	int ret;

	/*
	 * Relocate low-level suspend code to SRAM.
	 * It will run with no DDR available.
	 */
	memcpy((unsigned char *) KINETIS_SUSPEND_CODE_BASE, 
		(unsigned char *) kinetis_suspend_to_ram,
		(unsigned char *) kinetis_suspend_to_ram_end -
		(unsigned char *) kinetis_suspend_to_ram);

	/*
 	 * Register the PM driver
 	 */
	if (platform_driver_register(&kinetis_pm_driver) != 0) {
		printk(KERN_ERR "kinetis_pm_driver register failed\n");
		ret = -ENODEV;
		goto Done;
	}

	/*
 	 * Register PM operations
 	 */
	suspend_set_ops(&kinetis_pm_ops);

	/*
	 * Here, means success
	 */
	printk(KERN_INFO "Power Management for Freescale Kinetis\n");
	ret = 0;
	goto Done;

Done:
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit kinetis_pm_cleanup(void)
{
	platform_driver_unregister(&kinetis_pm_driver);
}

module_init(kinetis_pm_init);
module_exit(kinetis_pm_cleanup);

MODULE_AUTHOR("Robert Brehm");
MODULE_DESCRIPTION("Kinetis PM driver");
MODULE_LICENSE("GPL");

