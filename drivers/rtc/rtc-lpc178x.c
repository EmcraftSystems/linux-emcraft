/*
 * LPC178X On-Chip Real Time Clock Driver
 *
 * (C) Copyright 2012,2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * (C) Copyright 2013
 * Intmash, <www.intmash.ru>
 * Gilmanov Ildar <gil_ildar@mail.ru>
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

#include <linux/bcd.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

struct lpc178x_rtc {
	struct rtc_device *rtc_dev;
	struct rtc_time alarm_tm;
	spinlock_t lock;
	void *base;
	int  irq;
};

/*
 * RTC and Event Monitor/Recorder Interrupt ID
 * Counter Increment (RTCCIF), Alarm (RTCALF), EV0, EV1, EV2
 */
#define LPC178X_IRQ_RTC	(17)

/*
 * Consolidated Time Register 0 (CTIME0 - 0x4002 4014)
 */
struct lpc178x_cons_time_regs
{
	u32 sec : 6;		/* Seconds register. Min = 0, Max = 59 */
	u32 reserve_0 : 2;	/* Reserve bits */

	u32 min : 6;		/* Minutes register. Min = 0, Max = 59 */
	u32 reserve_1 : 2;	/* Reserve bits */

	u32 hour : 5;		/* Hours register. Min = 0, Max = 23 */
	u32 reserve_2 : 3;	/* Reserve bits */

	u32 dow : 3;		/* Day Of Week register. Min = 0, Max = 6 */
	u32 reserve_4 : 5;	/* Reserve bits */
};

/*
 * Consolidated Time Register 1 (CTIME1 - 0x4002 4018)
 */
struct lpc178x_cons_date_regs
{
	u32 dom : 5;		/* Day Of Month register. Min = 1, Max = 28,29,30,31 */
	u32 reserve_3 : 3;	/* Reserve bits */

	u32 month : 4;		/* Month register. Min = 1, Max = 12 */
	u32 reserve_6 : 4;	/* Reserve bits */

	u32 year : 12;		/* Year register. Min = 0, Max = 4095 */
	u32 reserve_7 : 4;	/* Reserve bits */
};

/*
 * Consolidated Time Register 2 (CTIME2 - 0x4002 401C)
 */
struct lpc178x_cons_doy_regs
{
	u32 doy : 12;		/* Day Of Year register. Min = 1, Max = 365,366 */
	u32 reserve_5 : 19;	/* Reserve bits */
};

struct lpc178x_date_time_regs
{
	u32 sec : 6;		/* Seconds register. Min = 0, Max = 59 */
	u32 reserve_0 : 26;	/* Reserve bits */

	u32 min : 6;		/* Minutes register. Min = 0, Max = 59 */
	u32 reserve_1 : 26;	/* Reserve bits */

	u32 hour : 5;		/* Hours register. Min = 0, Max = 23 */
	u32 reserve_2 : 27;	/* Reserve bits */

	u32 dom : 5;		/* Day Of Month register. Min = 1, Max = 28,29,30,31 */
	u32 reserve_3 : 27;	/* Reserve bits */

	u32 dow : 3;		/* Day Of Week register. Min = 0, Max = 6. Started from Sunday */
	u32 reserve_4 : 29;	/* Reserve bits */

	u32 doy : 9;		/* Day Of Year register. Min = 1, Max = 365,366 */
	u32 reserve_5 : 23;	/* Reserve bits */

	u32 month : 4;		/* Month register. Min = 1, Max = 12 */
	u32 reserve_6 : 28;	/* Reserve bits */

	u32 year : 12;		/* Year register. Min = 0, Max = 4095 */
	u32 reserve_7 : 20;	/* Reserve bits */
};

struct lpc178x_rtc_regs {
	u32 int_loc_r;								/* RTC Interrupt Location Register. Offset 0 */
	u32 reserve;								/* Reserve Register. Offset 4 */
	u32 clock_control_r;						/* RTC Clock Control Register. Offset 8 */
	u32 counter_inc_int_r;						/* RTC Counter Increment Interrupt Register. Offset 12 */
	u32 alarm_mask_r;							/* RTC Alarm Mask Register. Offset 16 */
	u32 cons_time_0_r;							/* RTC Consolidated Time Register 0. Offset 20 */
	u32 cons_time_1_r;							/* RTC Consolidated Time Register 1. Offset 24 */
	u32 cons_time_2_r;							/* RTC Consolidated Time Register 2. Offset 28 */
	struct lpc178x_date_time_regs date_time_r;	/* RTC date and time registers. Offset 32 */
	u32 calibration_r;							/* RTC calibration register. Offset 64 */
	u32 bkp_r[5];								/* RTC backup registers. Offset 68 */
	u32 auxiliary_enable_r;						/* RTC Auxiliary Enable Register. Offset 88 */
	u32 auxiliary_control_r;					/* RTC Auxiliary Control Register. Offset 92 */
	struct lpc178x_date_time_regs alarm_r;		/* RTC alarm registers. Offset 96 */
};

const struct lpc178x_date_time_regs DEFAULT_DATE_TIME =
{
	.sec = 0,		/* Seconds register. Min = 0, Max = 59 */
	.min = 0,		/* Minutes register. Min = 0, Max = 59 */
	.hour = 0,		/* Hours register. Min = 0, Max = 23 */
	.dom = 1,		/* Day Of Month register. Min = 1, Max = 28,29,30,31 */
	.dow = 2,		/* Day Of Week register. Min = 0, Max = 6 */
	.doy = 1,		/* Day Of Year register. Min = 1, Max = 365,366 */
	.month = 1,		/* Month register. Min = 1, Max = 12 */
	.year = 2013	/* Year register. Min = 0, Max = 4095 */
};

#define LPC178X_RTC_EPOCH			(1900)

#define LPC178X_RTC_BASE		(0x40024000)
#define LPC178X_RTC_CONS_TIME_SHIFT	0x14
#define LPC178X_RTC_CONS_DATE_SHIFT	0x18
#define LPC178X_RTC_CONS_DOY_SHIFT	0x1C

#define LPC178X_RTC					((volatile struct lpc178x_rtc_regs*)rtc->base)
#define LPC178X_RTC_CONS_TIME		((volatile struct lpc178x_cons_time_regs*)(rtc->base + LPC178X_RTC_CONS_TIME_SHIFT))
#define LPC178X_RTC_CONS_DATE		((volatile struct lpc178x_cons_date_regs*)(rtc->base + LPC178X_RTC_CONS_DATE_SHIFT))
#define LPC178X_RTC_CONS_DOY		((volatile struct lpc178x_cons_doy_regs*)(rtc->base + LPC178X_RTC_CONS_DOY_SHIFT))


/*****************************************************************************
 *
 * RTC Inerrupt Location Register
 * The Interrupt Location Register is a 2-bit register that specifies which blocks are
 * generating an interrupt (see Table 621). Writing a one to the appropriate bit clears the
 * corresponding interrupt. Writing a zero has no effect. This allows the programmer to read
 * this register and write back the same value to clear only the interrupt that is detected by the read.
 *
 ******************************************************************************/
/*
 * When one, the Counter Increment Interrupt block generated an interrupt.
 * Writing a one to this bit location clears the counter increment interrupt.
 */
#define LPC178X_RTC_ILR_CIF_MSK	(1 << 0)
/*
 * When one, the alarm registers generated an interrupt.
 * Writing a one to this bit location clears the alarm interrupt.
 */
#define LPC178X_RTC_ILR_ALF_MSK	(1 << 1)
/*****************************************************************************/


/*****************************************************************************
 *
 * RTC Clock Control Register
 * The clock register is a 4-bit register that controls the operation of the clock divide circuit.
 * Each bit of the clock register is described in Table 622. All NC bits in this register should
 * be initialized when the RTC is first turned on.
 *
 ******************************************************************************/
/*
 * Clock enable
 * 	1:	The time counters are enabled.
 * 	0:	The time counters are disabled so that they may be initialized.
*/
#define LPC178X_RTC_CCR_CLKEN_MSK	(1 << 0)
/*
 * CTC Reset
 * 	1:	The elements in the internal oscillator divider are reset, andCCR[1] is changed to zero.
 * 		This is the divider that generates the 1 Hz clock from the 32.768 kHz crystal.
 * 		The state of the divider is not visible to software.
 * 	0:	No effect
 * 	*/
#define LPC178X_RTC_CCR_CTCRST_MSK	(1 << 1)
/*
 * Calibration counter enable
 * 1:	The calibration counter is disabled and reset to zero.
 * 0:	The calibration counter is enabled and counting, using the 1 Hz clock. When the
 * 		calibration counter is equal to the value of the CALIBRATION register, the counter resets
 * 		and repeats counting up to the value of the CALIBRATION register. See Section 29.6.4.2
 * 		and Section 29.6.5.
 * */
#define LPC178X_RTC_CCR_CCALEN_MSK	(1 << 4)
/*****************************************************************************/


/*****************************************************************************
 *
 * RTC Counter Increment Interrupt Register
 * The Counter Increment Interrupt Register (CIIR) gives the ability to generate an interrupt
 * every time a counter is incremented. This interrupt remains valid until cleared by writing a
 * 1 to bit 0 of the Interrupt Location Register (ILR[0]).
 * When 1, an increment of the value generates an interrupt.
 *
 ******************************************************************************/
#define LPC178X_RTC_CII_SEC_MSK	(1 << 0)
#define LPC178X_RTC_CII_MIN_MSK	(1 << 1)
#define LPC178X_RTC_CII_HOUR_MSK	(1 << 2)
#define LPC178X_RTC_CII_DOM_MSK	(1 << 3)
#define LPC178X_RTC_CII_DOW_MSK	(1 << 4)
#define LPC178X_RTC_CII_DOY_MSK	(1 << 5)
#define LPC178X_RTC_CII_MON_MSK	(1 << 6)
#define LPC178X_RTC_CII_YEAR_MSK	(1 << 7)
/*****************************************************************************/


/*****************************************************************************
 *
 * RTC Alarm Mask Register
 * The Alarm Mask Register (AMR) allows the user to mask any of the alarm registers.
 * Table 624 shows the relationship between the bits in the AMR and the alarms. For the
 * alarm function, every non-masked alarm register must match the corresponding time
 * counter for an interrupt to be generated. The interrupt is generated only when the counter
 * comparison first changes from no match to match. The interrupt is removed when a one is
 * written to the appropriate bit of the Interrupt Location Register (ILR). If all mask bits are
 * set, then the alarm is disabled.
 * When 1, the value is not compared for the alarm.
 *
 ******************************************************************************/
#define LPC178X_RTC_AMR_SEC_MSK	(1 << 0)
#define LPC178X_RTC_AMR_MIN_MSK	(1 << 1)
#define LPC178X_RTC_AMR_HOUR_MSK	(1 << 2)
#define LPC178X_RTC_AMR_DOM_MSK	(1 << 3)
#define LPC178X_RTC_AMR_DOW_MSK	(1 << 4)
#define LPC178X_RTC_AMR_DOY_MSK	(1 << 5)
#define LPC178X_RTC_AMR_MON_MSK	(1 << 6)
#define LPC178X_RTC_AMR_YEAR_MSK	(1 << 7)
/*****************************************************************************/


/*****************************************************************************
 *
 * RTC Auxiliary Control Register
 * The RTC Auxiliary Control register contains added interrupt flags for functions that are not
 * part of the Real Time Clock itself (the part recording the passage of time and generating
 * other time related functions).
 * On the LPC178x/177x, the only added interrupt flag is for failure of the RTC oscillator.
 *
 ******************************************************************************/
/*
 * RTC Oscillator Fail detect flag.
 * Read: this bit is set if the RTC oscillator stops, and when RTC power is first turned on.
 * An interrupt will occur when this bit is set, the RTC_OSCFEN bit in RTC_AUXEN is a 1,
 * and the RTC interrupt is enabled in the NVIC.
 * Write: writing a 1 to this bit clears the flag.
*/
#define LPC178X_RTC_AUX_OSCF_MSK	(1 << 4)
/*
 * When 0: the RTC_ALARM pin reflects the RTC alarm status.
 * When 1: the RTC_ALARM pin indicates Deep Power-down mode.
*/
#define LPC178X_RTC_AUX_PDOUT_MSK	(1 << 6)
/*****************************************************************************/


/*****************************************************************************
 *
 * RTC Auxiliary Enable Register
 * The RTC Auxiliary Enable Register controls whether additional interrupt sources
 * represented in the RTC Auxiliary control register are enabled.
 *
 ******************************************************************************/
/*
 * Oscillator Fail Detect interrupt enable.
 * When 0: the RTC Oscillator Fail detect interrupt is disabled.
 * When 1: the RTC Oscillator Fail detect interrupt is enabled. See Section 29.6.2.5.
 */
#define LPC178X_RTC_AUXEN_OSCFEN_MSK	(1 << 4)
/*****************************************************************************/


/*****************************************************************************
 *
 * RTC Calibration register
 * The register is used to calibrate the time counter.
 *
 ******************************************************************************/
/*
 * If enabled, the calibration counter counts up to this value.
 * The maximum value is 131, 072 corresponding to about 36.4 hours. Calibration is disabled if CALVAL = 0.
 */
#define LPC178X_RTC_CALIBRATION_CALVAL_MSK	(0x1FFFF)
/*
 * Calibration direction NC
 * 	1:	Backward calibration. When CALVAL is equal to the calibration counter, the RTC
 * 		timers will stop incrementing for 1 second.
 *	0:	Forward calibration. When CALVAL is equal to the calibration counter, the RTC timers will jump by 2 seconds.
 */
#define LPC178X_RTC_CALIBRATION_CALDIR_MSK	(1 << 17)
/*****************************************************************************/


/*
 * RTC interrupt handler
 */
static irqreturn_t lpc178x_rtc_irq(int irq, void *dev_id)
{
	unsigned long events = 0;
	struct lpc178x_rtc *rtc = (struct lpc178x_rtc *)dev_id;
	u32 status;

	spin_lock(&rtc->lock);

	status = LPC178X_RTC->int_loc_r;

	/* clear event flags */
	LPC178X_RTC->int_loc_r = status & (LPC178X_RTC_ILR_CIF_MSK | LPC178X_RTC_ILR_ALF_MSK);

	if (status & LPC178X_RTC_ILR_CIF_MSK){
		/* Update interrupt (1 Hz) */
		events |= (RTC_UF | RTC_IRQF);
	}

	if (status & LPC178X_RTC_ILR_ALF_MSK){
		/* Alarm interrupt */
		events |= (RTC_AF | RTC_IRQF);
	}

	if (events) {
		rtc_update_irq(rtc->rtc_dev, 1, events);
	}

	spin_unlock(&rtc->lock);

	return events ? IRQ_HANDLED : IRQ_NONE;
}

static void lpc178x_rtc_set_alarm_mask(struct lpc178x_rtc *rtc)
{
	struct rtc_time *tm = &rtc->alarm_tm;
	/* Prepare the value for the RTC alarm mask register */
	u32 tmp = 0;

	/*
	 * Set year
	 */
	if (tm->tm_year >= 0) {
		tmp &= ~LPC178X_RTC_AMR_YEAR_MSK;
	} else {
		/* Skip year comparison */
		tmp |= LPC178X_RTC_AMR_YEAR_MSK;
	}

	/*
	 * Set month
	 */
	if (tm->tm_mon >= 0) {
		tmp &= ~LPC178X_RTC_AMR_MON_MSK;
	} else {
		/* Skip month comparison */
		tmp |= LPC178X_RTC_AMR_MON_MSK;
	}

	/*
	 * Set day of month
	 */
	if (tm->tm_mday >= 0) {
		tmp &= ~LPC178X_RTC_AMR_DOM_MSK;
	} else {
		/* Skip day of month comparison */
		tmp |= LPC178X_RTC_AMR_DOM_MSK;
	}

	/*
	 * Set day of week
	 */
	if (tm->tm_wday >= 0) {
		tmp &= ~LPC178X_RTC_AMR_DOW_MSK;
	} else {
		/* Skip day of week comparison */
		tmp |= LPC178X_RTC_AMR_DOW_MSK;
	}

	/*
	 * Set day of year
	 */
	if (tm->tm_yday >= 0) {
		tmp &= ~LPC178X_RTC_AMR_DOY_MSK;
	} else {
		/* Skip day of year comparison */
		tmp |= LPC178X_RTC_AMR_DOY_MSK;
	}

	/*
	 * Set hour
	 */
	if (tm->tm_hour >= 0) {
		tmp &= ~LPC178X_RTC_AMR_HOUR_MSK;
	} else {
		/* Skip hour comparison */
		tmp |= LPC178X_RTC_AMR_HOUR_MSK;
	}

	/*
	 * Set min
	 */
	if (tm->tm_min >= 0) {
		tmp &= ~LPC178X_RTC_AMR_MIN_MSK;
	} else {
		/* Skip min comparison */
		tmp |= LPC178X_RTC_AMR_MIN_MSK;
	}

	/*
	 * Set seconds
	 */
	if (tm->tm_sec >= 0) {
		tmp &= ~LPC178X_RTC_AMR_SEC_MSK;
	} else {
		/* Skip sec comparison */
		tmp |= LPC178X_RTC_AMR_SEC_MSK;
	}

	/* Write to the alarm register */
	LPC178X_RTC->alarm_mask_r = tmp;
}

/*
 * Enable or disable alarm interrupts
 */
static int lpc178x_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct lpc178x_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	if(enabled) {
		/* Enable the alarm interrupt */
		lpc178x_rtc_set_alarm_mask(rtc);
	} else {
		/* Disable the alarm interrupt */
		LPC178X_RTC->alarm_mask_r = 0xFF;
	}

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Enable or disable the update interrupts
 */
static int lpc178x_rtc_update_irq_enable(struct device *dev, unsigned int enabled)
{
	struct lpc178x_rtc *rtc = dev_get_drvdata(dev);

	/* Nothing to do */
	if (enabled == !!(LPC178X_RTC->counter_inc_int_r & LPC178X_RTC_AMR_SEC_MSK))
		return 0;

	spin_lock_irq(&rtc->lock);

	if (enabled) {
		/* Enable Counter Increment Interrupts (1Hz) */
		LPC178X_RTC->counter_inc_int_r = LPC178X_RTC_CII_SEC_MSK;
	} else {
		/* Disable Counter Increment Interrupts */
		LPC178X_RTC->counter_inc_int_r = 0;
	}

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Read current time and date from the RTC
 */
static int lpc178x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct lpc178x_rtc *rtc = dev_get_drvdata(dev);
	struct lpc178x_cons_time_regs cons_time;
	struct lpc178x_cons_date_regs cons_date;
	struct lpc178x_cons_doy_regs cons_doy;

	spin_lock_irq(&rtc->lock);

	/* We use this temp structs to avoid change time while we filling tm struct */
	cons_time = *LPC178X_RTC_CONS_TIME;
	cons_date = *LPC178X_RTC_CONS_DATE;
	cons_doy = *LPC178X_RTC_CONS_DOY;

	tm->tm_sec = cons_time.sec;
	tm->tm_min = cons_time.min;
	tm->tm_hour = cons_time.hour;
	tm->tm_wday = cons_time.dow;

	tm->tm_mon = cons_date.month - 1;
	tm->tm_mday = cons_date.dom;
	tm->tm_year = cons_date.year - LPC178X_RTC_EPOCH;

	tm->tm_yday = cons_doy.doy - 1;

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Write time and date to the RTC
 */
static int lpc178x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct lpc178x_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	/*
	 * The time counters are disabled so that they may be initialized
	 * and the elements in the internal oscillator divider are reset
	 */
	LPC178X_RTC->clock_control_r = LPC178X_RTC_CCR_CTCRST_MSK;

	LPC178X_RTC->date_time_r.sec = tm->tm_sec;
	LPC178X_RTC->date_time_r.min = tm->tm_min;
	LPC178X_RTC->date_time_r.hour = tm->tm_hour;

	LPC178X_RTC->date_time_r.dom = tm->tm_mday;
	LPC178X_RTC->date_time_r.month = tm->tm_mon + 1;
	LPC178X_RTC->date_time_r.year = tm->tm_year + LPC178X_RTC_EPOCH;
	LPC178X_RTC->date_time_r.dow = tm->tm_wday;
	LPC178X_RTC->date_time_r.doy = tm->tm_yday + 1;

	/*
	 * The time counters are enabled.
	 */
	LPC178X_RTC->clock_control_r = LPC178X_RTC_CCR_CLKEN_MSK;

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Read the time and date the alarm
 */
static int lpc178x_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct lpc178x_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;

	spin_lock_irq(&rtc->lock);

	/*
	 * We use rtc->alarm_tm instead registers to avoid read wrong data until ALARM interrupt not enabled
	 */
	*tm = rtc->alarm_tm;

	alrm->enabled = !!(LPC178X_RTC->int_loc_r & LPC178X_RTC_ILR_ALF_MSK);
	alrm->pending = 0;

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Set alarm.
 */
static int lpc178x_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct lpc178x_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	int rv;

	if (rtc_valid_tm(tm)) {
		rv = -EINVAL;
		return rv;
	}

	spin_lock_irq(&rtc->lock);

	/* Disable the alarm interrupt */
	LPC178X_RTC->alarm_mask_r = 0xFF;

	/* Clear the alarm interrupt */
	LPC178X_RTC->int_loc_r |= LPC178X_RTC_ILR_ALF_MSK;

	rtc->alarm_tm = *tm;

	/*
	 * Set year
	 */
	if (tm->tm_year >= 0) {
		LPC178X_RTC->alarm_r.year = tm->tm_year + LPC178X_RTC_EPOCH;
	} else {
		LPC178X_RTC->alarm_r.year = 0;
	}

	/*
	 * Set month
	 */
	if (tm->tm_mon >= 0) {
		LPC178X_RTC->alarm_r.month = tm->tm_mon + 1;
	} else {
		LPC178X_RTC->alarm_r.month = 1;
	}

	/*
	 * Set day of month
	 */
	if (tm->tm_mday >= 0) {
		LPC178X_RTC->alarm_r.dom = tm->tm_mday;
	} else {
		LPC178X_RTC->alarm_r.dom = 1;
	}

	/*
	 * Set day of week
	 */
	if (tm->tm_wday >= 0) {
		LPC178X_RTC->alarm_r.dow = tm->tm_wday;
	} else {
		LPC178X_RTC->alarm_r.dow = 0;
	}

	/*
	 * Set day of year
	 */
	if (tm->tm_wday >= 0) {
		LPC178X_RTC->alarm_r.doy = tm->tm_yday + 1;
	} else {
		LPC178X_RTC->alarm_r.doy = 1;
	}

	/*
	 * Set hour
	 */
	if (tm->tm_hour >= 0) {
		LPC178X_RTC->alarm_r.hour = tm->tm_hour;
	} else {
		LPC178X_RTC->alarm_r.hour = 0;
	}

	/*
	 * Set min
	 */
	if (tm->tm_min >= 0) {
		LPC178X_RTC->alarm_r.min = tm->tm_min;
	} else {
		LPC178X_RTC->alarm_r.min = 0;
	}

	/*
	 * Set seconds
	 */
	if (tm->tm_sec >= 0) {
		LPC178X_RTC->alarm_r.sec = tm->tm_sec;
	} else {
		LPC178X_RTC->alarm_r.sec = 0;
	}

	/* Enable Alarm interrupts if needed */
	if (alrm->enabled){
		lpc178x_rtc_set_alarm_mask(rtc);
	}

	spin_unlock_irq(&rtc->lock);
	rv = 0;

	return rv;
}

static struct rtc_class_ops lpc178x_rtc_ops = {
	.read_time		= lpc178x_rtc_read_time,
	.set_time		= lpc178x_rtc_set_time,
	.read_alarm		= lpc178x_rtc_read_alarm,
	.set_alarm		= lpc178x_rtc_set_alarm,
	.alarm_irq_enable	= lpc178x_rtc_alarm_irq_enable,
	.update_irq_enable	= lpc178x_rtc_update_irq_enable,
};

static void lpc178x_rtc_restore_alarm(struct lpc178x_rtc *rtc)
{
	struct rtc_time *tm = &rtc->alarm_tm;
	u32 reg = LPC178X_RTC->alarm_mask_r;

	if(reg & LPC178X_RTC_AMR_SEC_MSK) {
		tm->tm_sec = -1;
	} else {
		tm->tm_sec = LPC178X_RTC->alarm_r.sec;
	}

	if(reg & LPC178X_RTC_AMR_MIN_MSK) {
		tm->tm_min = -1;
	} else {
		tm->tm_min = LPC178X_RTC->alarm_r.min;
	}

	if(reg & LPC178X_RTC_AMR_HOUR_MSK) {
		tm->tm_hour = -1;
	} else {
		tm->tm_hour = LPC178X_RTC->alarm_r.hour;
	}

	if(reg & LPC178X_RTC_AMR_MON_MSK) {
		tm->tm_mon = -1;
	} else {
		tm->tm_mon = LPC178X_RTC->alarm_r.month - 1;
	}

	if(reg & LPC178X_RTC_AMR_YEAR_MSK) {
		tm->tm_year = -1;
	} else {
		tm->tm_year = LPC178X_RTC->alarm_r.year - 1990;
	}

	if(reg & LPC178X_RTC_AMR_DOM_MSK) {
		tm->tm_mday = -1;
	} else {
		tm->tm_mday = LPC178X_RTC->alarm_r.dom;
	}

	if(reg & LPC178X_RTC_AMR_DOW_MSK) {
		tm->tm_wday = -1;
	} else {
		tm->tm_wday = LPC178X_RTC->alarm_r.dow;
	}

	if(reg & LPC178X_RTC_AMR_DOY_MSK) {
		tm->tm_yday = -1;
	} else {
		tm->tm_yday = LPC178X_RTC->alarm_r.doy - 1;
	}
}

/*
 * RTC device initialization function
 */
static int __devinit lpc178x_rtc_probe(struct platform_device *pdev)
{
	struct lpc178x_rtc *rtc;
	struct device *dev = &pdev->dev;
	struct resource *res;
	void (*plat_init)(void);
	int rv = 0;

	/* Allocate memory for our RTC struct */
	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);
	if (unlikely(!rtc)) {
		rv = -ENOMEM;
		printk(KERN_ERR "Can not allocate memory for RTC\n");
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		res = request_mem_region(res->start, resource_size(res),
			"rtc-lpc1788");
		if (!res) {
			rv = -ENODEV;
			goto err;
		}
		rtc->base = (void __iomem *)res->start;
	}
	else {
		pr_debug("Could not find MEM resource");
		rtc->base = (void __iomem *)LPC178X_RTC_BASE;
	}

	rtc->irq = LPC178X_IRQ_RTC;
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res)
		rtc->irq = res->start;

	if ((plat_init = pdev->dev.platform_data))
		plat_init();

	platform_set_drvdata(pdev, rtc);
	device_init_wakeup(dev, 1);

	spin_lock_init(&rtc->lock);

	lpc178x_rtc_restore_alarm(rtc);

	/* Register our RTC with the RTC framework */
	rtc->rtc_dev = rtc_device_register(pdev->name, dev, &lpc178x_rtc_ops, THIS_MODULE);
	if (unlikely(IS_ERR(rtc->rtc_dev))) {
		rv = PTR_ERR(rtc->rtc_dev);
		printk(KERN_ERR "Can not register RTC device\n");
		goto err;
	}

	/* Handle RTC interrupts */
	rv = request_irq(rtc->irq, lpc178x_rtc_irq, 0, pdev->name, rtc);
	if (unlikely(rv)){
		printk(KERN_ERR "Can not get IRQ for RTC\n");
		goto err_irq;
	}

	if(LPC178X_RTC->auxiliary_control_r & LPC178X_RTC_AUX_OSCF_MSK)
	{
		/* RTC power is first turned on */
		printk(KERN_INFO "RTC power is first turned on\n");
		LPC178X_RTC->date_time_r = DEFAULT_DATE_TIME;

		LPC178X_RTC->auxiliary_control_r |= LPC178X_RTC_AUX_OSCF_MSK;
	}

	rv = 0;
	goto out;

err_irq:
	rtc_device_unregister(rtc->rtc_dev);
err:
	kfree(rtc);
out:
	return rv;
}

static int __devexit lpc178x_rtc_remove(struct platform_device *pdev)
{
	struct lpc178x_rtc *rtc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	free_irq(rtc->irq, dev);

	rtc_device_unregister(rtc->rtc_dev);
	platform_set_drvdata(pdev, NULL);

	kfree(rtc);

	return 0;
}

static struct platform_driver lpc178x_rtc_driver = {
	.driver		= {
		.name	= "rtc-lpc178x",
		.owner	= THIS_MODULE,
	},
	.probe		= lpc178x_rtc_probe,
	.remove		= __devexit_p(lpc178x_rtc_remove),
};

static int __init lpc178x_rtc_init(void)
{
	return platform_driver_register(&lpc178x_rtc_driver);
}

static void __exit lpc178x_rtc_exit(void)
{
	platform_driver_unregister(&lpc178x_rtc_driver);
}

module_init(lpc178x_rtc_init);
module_exit(lpc178x_rtc_exit);

MODULE_DESCRIPTION("LPC178X On-Chip Real Time Clock Driver");
MODULE_AUTHOR("Alexander Potashev <aspotashev@emcraft.com>");
MODULE_AUTHOR("Intmash Ltd. <www.intmash.ru>, Gilmanov Ildar <gil_ildar@mail.ru>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtc-lpc178x");
