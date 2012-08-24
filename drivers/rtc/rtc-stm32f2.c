/*
 * STM32F2 On-Chip Real Time Clock Driver
 *
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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

#include <mach/exti.h>

struct stm32f2_rtc {
	struct rtc_device *rtc_dev;
	spinlock_t lock;
};

/* RTC Wakeup interrupt through the EXTI line */
#define STM32F2_IRQ_RTC_WAKEUP	3
/* RTC Alarms (A and B) through EXTI line interrupt */
#define STM32F2_IRQ_RTC_ALARM	41


struct stm32f2_rtc_regs {
	u32 tr;		/* RTC time register */
	u32 dr;		/* RTC date register */
	u32 cr;		/* RTC control register */
	u32 isr;	/* RTC initialization and status register */
	u32 prer;	/* RTC prescaler register */
	u32 wutr;	/* RTC wakeup timer register */
	u32 calibr;	/* RTC calibration register */
	u32 alrmar;	/* RTC alarm A register */
	u32 alrmbr;	/* RTC alarm B register */
	u32 wpr;	/* RTC write protection register */
	u32 rsv0[2];
	u32 tstr;	/* RTC time stamp time register */
	u32 tsdr;	/* RTC time stamp date register */
	u32 rsv1[2];
	u32 tafcr;	/* RTC tamper and alternate function configuration */
	u32 rsv2[3];
	u32 bkpr[20];	/* RTC backup registers */
};

#define STM32F2_RTC_BASE	0x40002800
#define STM32_RTC		((volatile struct stm32f2_rtc_regs *)STM32F2_RTC_BASE)

/*
 * RTC time register (RTC_TR)
 */
/* Seconds in BCD format */
#define STM32F2_RTC_TR_SEC_BITS		0
#define STM32F2_RTC_TR_SEC_MSK		(0x7f << STM32F2_RTC_TR_SEC_BITS)
/* Minutes in BCD format */
#define STM32F2_RTC_TR_MIN_BITS		8
#define STM32F2_RTC_TR_MIN_MSK		(0x7f << STM32F2_RTC_TR_MIN_BITS)
/* Hours in BCD format */
#define STM32F2_RTC_TR_HOUR_BITS	16
#define STM32F2_RTC_TR_HOUR_MSK		(0x3f << STM32F2_RTC_TR_HOUR_BITS)

/*
 * RTC date register (RTC_DR)
 */
/* Day of month in BCD format (1..31) */
#define STM32F2_RTC_DR_MDAY_BITS	0
#define STM32F2_RTC_DR_MDAY_MSK		(0x3f << STM32F2_RTC_DR_MDAY_BITS)
/* Month in BCD format (1..12) */
#define STM32F2_RTC_DR_MON_BITS		8
#define STM32F2_RTC_DR_MON_MSK		(0x1f << STM32F2_RTC_DR_MON_BITS)
/* Year's last two digits in BCD format */
#define STM32F2_RTC_DR_YEAR_BITS	16
#define STM32F2_RTC_DR_YEAR_MSK		(0xff << STM32F2_RTC_DR_YEAR_BITS)
/* Day of week (1=Monday, 7=Sunday) */
#define STM32F2_RTC_DR_WDAY_BITS	13
#define STM32F2_RTC_DR_WDAY_MSK		(7 << STM32F2_RTC_DR_WDAY_BITS)

/*
 * RTC control register (RTC_CR)
 */
/* Wakeup timer interrupt enable */
#define STM32F2_RTC_CR_WUTIE_MSK	(1 << 14)
/* Alarm B interrupt enable */
#define STM32F2_RTC_CR_ALRBIE_MSK	(1 << 13)
/* Alarm A interrupt enable */
#define STM32F2_RTC_CR_ALRAIE_MSK	(1 << 12)
/* Wakeup timer enable */
#define STM32F2_RTC_CR_WUTE_MSK		(1 << 10)
/* Alarm B enable */
#define STM32F2_RTC_CR_ALRBE_MSK	(1 << 9)
/* Alarm A enable */
#define STM32F2_RTC_CR_ALRAE_MSK	(1 << 8)
/* Hour format (0=24h, 1=AM/PM) */
#define STM32F2_RTC_CR_FMT_MSK		(1 << 6)
/* Wakeup clock selection */
#define STM32F2_RTC_CR_WUCKSEL_BITS	0
#define STM32F2_RTC_CR_WUCKSEL_MSK	(7 << STM32F2_RTC_CR_WUCKSEL_BITS)
#define STM32F2_RTC_CR_WUCKSEL_DIV2	(3 << STM32F2_RTC_CR_WUCKSEL_BITS)

/*
 * RTC initialization and status register (RTC_ISR)
 */
/* Wakeup timer flag */
#define STM32F2_RTC_ISR_WUTF_MSK	(1 << 10)
/* Alarm B flag */
#define STM32F2_RTC_ISR_ALRBF_MSK	(1 << 9)
/* Alarm A flag */
#define STM32F2_RTC_ISR_ALRAF_MSK	(1 << 8)
/* Initialization mode control bit */
#define STM32F2_RTC_ISR_INIT_MSK	(1 << 7)
/* Initialization mode flag */
#define STM32F2_RTC_ISR_INITF_MSK	(1 << 6)
/* Registers synchronization flag */
#define STM32F2_RTC_ISR_RSF_MSK		(1 << 5)
/* Initialization status flag */
#define STM32F2_RTC_ISR_INITS_MSK	(1 << 4)
/* Wakeup timer write flag */
#define STM32F2_RTC_ISR_WUTWF_MSK	(1 << 2)
/* Alarm B write flag */
#define STM32F2_RTC_ISR_ALRBWF_MSK	(1 << 1)
/* Alarm A write flag */
#define STM32F2_RTC_ISR_ALRAWF_MSK	(1 << 0)

/*
 * RTC alarm register (RTC_ALRMAR, RTC_ALRMBR)
 */
/* Date and day don't care in alarm comparison */
#define STM32F2_RTC_ALRMR_DATEMASK_MSK	(1 << 31)
/* Hours don't care in alarm comparison */
#define STM32F2_RTC_ALRMR_HOURMASK_MSK	(1 << 23)
/* Minutes don't care in alarm comparison */
#define STM32F2_RTC_ALRMR_MINMASK_MSK	(1 << 15)
/* Seconds don't care in alarm comparison */
#define STM32F2_RTC_ALRMR_SECMASK_MSK	(1 << 7)
/* Week day selection: 0=day in month; 1=day of week */
#define STM32F2_RTC_ALRMR_WDSEL_MSK	(1 << 30)
/* Date/day of week */
#define STM32F2_RTC_ALRMR_DAY_BITS	24
#define STM32F2_RTC_ALRMR_MDAY_MSK	(0x3f << STM32F2_RTC_ALRMR_DAY_BITS)
#define STM32F2_RTC_ALRMR_WDAY_MSK	(0x7 << STM32F2_RTC_ALRMR_DAY_BITS)
/* Hours in BCD format */
#define STM32F2_RTC_ALRMR_HOUR_BITS	16
#define STM32F2_RTC_ALRMR_HOUR_MSK	(0x3f << STM32F2_RTC_ALRMR_HOUR_BITS)
/* Minutes in BCD format */
#define STM32F2_RTC_ALRMR_MIN_BITS	8
#define STM32F2_RTC_ALRMR_MIN_MSK	(0x7f << STM32F2_RTC_ALRMR_MIN_BITS)
/* Seconds in BCD format */
#define STM32F2_RTC_ALRMR_SEC_BITS	0
#define STM32F2_RTC_ALRMR_SEC_MSK	(0x7f << STM32F2_RTC_ALRMR_SEC_BITS)

/*
 * Disable or enable write-protection of RTC module registers
 */
static void stm32f2_rtc_write_enable(int enable)
{
	if (enable) {
		/* Disable write protection */
		STM32_RTC->wpr = 0xca;
		STM32_RTC->wpr = 0x53;
	} else {
		/* Enable write protection */
		STM32_RTC->wpr = 0xff;
	}
}

/*
 * Enter or exit the initialization mode
 */
static void stm32f2_rtc_enable_init_mode(int enable)
{
	if (enable) {
		/* Enter initialization mode */
		if (!(STM32_RTC->isr & STM32F2_RTC_ISR_INITF_MSK)) {
			/* Switch to the initialization mode */
			STM32_RTC->isr = STM32F2_RTC_ISR_INIT_MSK;
			/* Wait until the initialization mode is entered */
			while (!(STM32_RTC->isr & STM32F2_RTC_ISR_INITF_MSK));
		}
	} else {
		/* Exit initialization mode */
		STM32_RTC->isr &= ~STM32F2_RTC_ISR_INIT_MSK;
	}
}

/*
 * RTC alarm interrupt handler
 */
static irqreturn_t stm32f2_rtc_alarm_irq(int irq, void *dev_id)
{
	unsigned long events = 0;
	struct stm32f2_rtc *rtc = (struct stm32f2_rtc *)dev_id;
	u32 status;
	u32 enabled_irqs;

	spin_lock(&rtc->lock);

	status = STM32_RTC->isr;
	enabled_irqs = STM32_RTC->cr;

	stm32f2_rtc_write_enable(1);
	/* clear event flags, otherwise new events won't be received */
	STM32_RTC->isr &= ~(status &
		(STM32F2_RTC_ISR_ALRBF_MSK | STM32F2_RTC_ISR_ALRAF_MSK));
	stm32f2_rtc_write_enable(0);

	if ((status & STM32F2_RTC_ISR_ALRAF_MSK) &&
	    (enabled_irqs & STM32F2_RTC_CR_ALRAIE_MSK)) {
		/* Normal alarm interrupt */
		events |= (RTC_AF | RTC_IRQF);
	}

	if ((status & STM32F2_RTC_ISR_ALRBF_MSK) &&
	    (enabled_irqs & STM32F2_RTC_CR_ALRBIE_MSK)) {
		/* Update interrupt (1 Hz) */
		events |= (RTC_UF | RTC_IRQF);
	}

	if (events) {
		stm32_exti_clear_pending(STM32F2_EXTI_LINE_RTC_ALARM);
		rtc_update_irq(rtc->rtc_dev, 1, events);
	}

	spin_unlock(&rtc->lock);

	return events ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * RTC wakeup interrupt handler
 */
static irqreturn_t stm32f2_rtc_wakeup_irq(int irq, void *dev_id)
{
	struct stm32f2_rtc *rtc = (struct stm32f2_rtc *)dev_id;
	irqreturn_t rv;

	spin_lock(&rtc->lock);

	if (!(STM32_RTC->isr & STM32F2_RTC_ISR_WUTF_MSK) ||
	    !(STM32_RTC->cr & STM32F2_RTC_CR_WUTIE_MSK)) {
		rv = IRQ_NONE;
		goto out;
	}

	/* Clear event flag, otherwise new events won't be received */
	stm32f2_rtc_write_enable(1);
	STM32_RTC->isr &= ~STM32F2_RTC_ISR_WUTF_MSK;
	stm32f2_rtc_write_enable(0);

	stm32_exti_clear_pending(STM32F2_EXTI_LINE_RTC_WAKEUP);

	/* Pass periodic interrupt event to the kernel */
	rtc_update_irq(rtc->rtc_dev, 1, RTC_PF | RTC_IRQF);

	rv = IRQ_HANDLED;
out:
	spin_unlock(&rtc->lock);
	return rv;
}

/*
 * Set frequency of the periodic interrupt
 *
 * On STM32F2 we implement the periodic interrupt by means of the Wakeup
 * Interrupt. The maximum possible wakeup interrupt rate for a 32.768 kHz
 * crystal is 8.192 kHz.
 */
static int stm32f2_rtc_irq_set_freq(struct device *dev, int freq)
{
	struct stm32f2_rtc *rtc = dev_get_drvdata(dev);
	int rv;
	u32 tmp;
	u32 wakeup_state;

	if (freq <= 0 || freq > 8192) {
		rv = -EINVAL;
		goto out;
	}

	/* Input clock for the RTC wakeup counter is RTCLK/2 = 16384 Hz */
	tmp = 16384 / freq;
	/*
	 * The wakeup counter register is minus 1-encoded.
	 *
	 * Setting WUT[15:0] to 0x0000 with WUCKSEL[2:0]=011 (RTCCLK/2)
	 * is forbidden.
	 */
	if (tmp < 2)
		tmp = 1;
	else
		tmp--;

	spin_lock_irq(&rtc->lock);

	stm32f2_rtc_write_enable(1);

	/* Save wakeup timer state */
	wakeup_state = (STM32_RTC->cr & STM32F2_RTC_CR_WUTE_MSK);
	/* Disable wakeup timer */
	STM32_RTC->cr &= ~STM32F2_RTC_CR_WUTE_MSK;
	/* Poll write flag to make sure the access to registers is allowed */
	while (!(STM32_RTC->isr & STM32F2_RTC_ISR_WUTWF_MSK));

	/* Choose input clock for the RTC wakeup counter to be RTCLK/2 */
	STM32_RTC->cr =
		(STM32_RTC->cr & ~STM32F2_RTC_CR_WUCKSEL_MSK) |
		STM32F2_RTC_CR_WUCKSEL_DIV2;
	/* Set load value for the wakeup timer counter */
	STM32_RTC->wutr = tmp & 0xffff;

	/* Enable wakeup timer again if it was previously enabled */
	STM32_RTC->cr |= wakeup_state;

	stm32f2_rtc_write_enable(0);

	spin_unlock_irq(&rtc->lock);
	rv = 0;
out:
	return rv;
}

/*
 * Enable or disable periodic interrupts
 */
static int stm32f2_rtc_irq_set_state(struct device *dev, int enabled)
{
	struct stm32f2_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	stm32f2_rtc_write_enable(1);

	if (enabled) {
		/* Enable wakeup timer and interrupt */
		STM32_RTC->cr |=
			STM32F2_RTC_CR_WUTE_MSK | STM32F2_RTC_CR_WUTIE_MSK;
	} else {
		/* Disable wakeup timer and interrupt */
		STM32_RTC->cr &=
			~(STM32F2_RTC_CR_WUTE_MSK | STM32F2_RTC_CR_WUTIE_MSK);
	}

	/* Clear wakeup event flag, otherwise new events won't be received */
	STM32_RTC->isr &= ~STM32F2_RTC_ISR_WUTF_MSK;

	stm32f2_rtc_write_enable(0);

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Enable or disable the alarm interrupt for a given alarm.
 *
 * alarm = 0 selects Alarm A.
 * alarm = 1 selects Alarm B.
 */
static void stm32f2_alarm_irq_enable(unsigned int alarm, unsigned int enable)
{
	u32 irq_enable_msk =
		alarm ? STM32F2_RTC_CR_ALRBIE_MSK : STM32F2_RTC_CR_ALRAIE_MSK;
	u32 irq_flag_msk =
		alarm ? STM32F2_RTC_ISR_ALRBF_MSK : ~STM32F2_RTC_ISR_ALRAF_MSK;

	stm32f2_rtc_write_enable(1);

	/* Alarm interrupt enable or disable */
	if (enable)
		STM32_RTC->cr |= irq_enable_msk;
	else
		STM32_RTC->cr &= ~irq_enable_msk;

	/* Clear alarm event flag, otherwise new events won't be received */
	STM32_RTC->isr &= ~irq_flag_msk;

	stm32f2_rtc_write_enable(0);
}

/*
 * Enable or disable alarm interrupts
 */
static int stm32f2_rtc_alarm_irq_enable(
	struct device *dev, unsigned int enabled)
{
	struct stm32f2_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	/* We expose Alarm A to the kernel */
	stm32f2_alarm_irq_enable(0, enabled);

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Configure and enable an RTC alarm
 *
 * alarm = 0 selects Alarm A.
 * alarm = 1 selects Alarm B.
 *
 * "mday", "wday", "hour", "min" and "sec" can be set to (-1) to bypass
 * comparison of the respective component of time and date with the current
 * time.
 */
static void stm32f2_setup_alarm(
	unsigned int alarm,
	int mday, int wday, int hour, int min, int sec)
{
	u32 enable_msk =
		alarm ? STM32F2_RTC_CR_ALRBE_MSK : STM32F2_RTC_CR_ALRAE_MSK;
	u32 write_flag_msk =
		alarm ? STM32F2_RTC_ISR_ALRBWF_MSK : STM32F2_RTC_ISR_ALRAWF_MSK;
	u32 tmp;

	stm32f2_rtc_write_enable(1);

	/* Disable alarm */
	STM32_RTC->cr &= ~enable_msk;
	/* Poll write flag to make sure the access to registers is allowed */
	while (!(STM32_RTC->isr & write_flag_msk));

	/* Prepare the value for the RTC alarm register */
	tmp = 0;

	/*
	 * Set date or day of week
	 */
	if (mday < 0 && wday < 0) {	/* Day is don't care */
		tmp |= STM32F2_RTC_ALRMR_DATEMASK_MSK;
	} else if (mday > 0) {		/* Date is selected (ignoring "wday") */
		tmp |= (bin2bcd(mday) << STM32F2_RTC_ALRMR_DAY_BITS) &
			STM32F2_RTC_ALRMR_MDAY_MSK;
	} else {			/* Day of week is selected */
		tmp |= STM32F2_RTC_ALRMR_WDSEL_MSK;
		tmp |= ((wday ? wday : 7) << STM32F2_RTC_ALRMR_DAY_BITS) &
			STM32F2_RTC_ALRMR_WDAY_MSK;
	}

	/*
	 * Set hours
	 */
	if (hour >= 0) {
		tmp |= (bin2bcd(hour) << STM32F2_RTC_ALRMR_HOUR_BITS) &
			STM32F2_RTC_ALRMR_HOUR_MSK;
	} else {
		/* Skip hours comparison */
		tmp |= STM32F2_RTC_ALRMR_HOURMASK_MSK;
	}

	/*
	 * Set minutes
	 */
	if (min >= 0) {
		tmp |= (bin2bcd(min) << STM32F2_RTC_ALRMR_MIN_BITS) &
			STM32F2_RTC_ALRMR_MIN_MSK;
	} else {
		/* Skip minutes comparison */
		tmp |= STM32F2_RTC_ALRMR_MINMASK_MSK;
	}

	/*
	 * Set seconds
	 */
	if (sec >= 0) {
		tmp |= (bin2bcd(sec) << STM32F2_RTC_ALRMR_SEC_BITS) &
			STM32F2_RTC_ALRMR_SEC_MSK;
	} else {
		/* Skip seconds comparison */
		tmp |= STM32F2_RTC_ALRMR_SECMASK_MSK;
	}

	/* Write to the alarm register */
	*(alarm ? &STM32_RTC->alrmbr : &STM32_RTC->alrmar) = tmp;

	/* Enable alarm */
	STM32_RTC->cr |= enable_msk;

	stm32f2_rtc_write_enable(0);
}

/*
 * Enable or disable the update interrupts
 */
static int stm32f2_rtc_update_irq_enable(
	struct device *dev, unsigned int enabled)
{
	struct stm32f2_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	if (enabled) {
		/* Use Alarm B to emulate update interrupts (1 Hz) */
		stm32f2_setup_alarm(1, -1, -1, -1, -1, -1);
		stm32f2_alarm_irq_enable(1, 1);
	} else {
		/* Disable interrupts from Alarm B */
		stm32f2_alarm_irq_enable(1, 0);

		/* Disable Alarm B */
		stm32f2_rtc_write_enable(1);
		STM32_RTC->cr &= ~STM32F2_RTC_CR_ALRBE_MSK;
		stm32f2_rtc_write_enable(0);

		stm32_exti_clear_pending(STM32F2_EXTI_LINE_RTC_ALARM);
	}

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Read current time and date from the RTC
 */
static int stm32f2_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct stm32f2_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	/* Time in BCD format */
	tm->tm_sec = bcd2bin((STM32_RTC->tr & STM32F2_RTC_TR_SEC_MSK) >>
		STM32F2_RTC_TR_SEC_BITS);
	tm->tm_min = bcd2bin((STM32_RTC->tr & STM32F2_RTC_TR_MIN_MSK) >>
		STM32F2_RTC_TR_MIN_BITS);
	tm->tm_hour = bcd2bin((STM32_RTC->tr & STM32F2_RTC_TR_HOUR_MSK) >>
		STM32F2_RTC_TR_HOUR_BITS);

	/* Date in BCD format */
	tm->tm_mday = bcd2bin((STM32_RTC->dr & STM32F2_RTC_DR_MDAY_MSK) >>
		STM32F2_RTC_DR_MDAY_BITS);
	tm->tm_mon = bcd2bin((STM32_RTC->dr & STM32F2_RTC_DR_MON_MSK) >>
		STM32F2_RTC_DR_MON_BITS) - 1;
	/* Assume the year is between 2001 and 2099 */
	tm->tm_year = bcd2bin((STM32_RTC->dr & STM32F2_RTC_DR_YEAR_MSK) >>
		STM32F2_RTC_DR_YEAR_BITS) + 100;
	/* Kernel thinks 0=Sunday. RTC thinks 7=Sunday, 0=invalid */
	tm->tm_wday = ((STM32_RTC->dr & STM32F2_RTC_DR_WDAY_MSK) >>
		STM32F2_RTC_DR_WDAY_BITS) % 7;

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Write time and date to the RTC
 */
static int stm32f2_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct stm32f2_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	/* Disable write-protection; enter initialization mode */
	stm32f2_rtc_write_enable(1);
	stm32f2_rtc_enable_init_mode(1);

	/* Time in BCD format */
	STM32_RTC->tr =
		(STM32_RTC->tr & ~(STM32F2_RTC_TR_SEC_MSK |
			STM32F2_RTC_TR_MIN_MSK | STM32F2_RTC_TR_HOUR_MSK)) |
		((bin2bcd(tm->tm_sec) << STM32F2_RTC_TR_SEC_BITS) &
			STM32F2_RTC_TR_SEC_MSK) |
		((bin2bcd(tm->tm_min) << STM32F2_RTC_TR_MIN_BITS) &
			STM32F2_RTC_TR_MIN_MSK) |
		((bin2bcd(tm->tm_hour) << STM32F2_RTC_TR_HOUR_BITS) &
			STM32F2_RTC_TR_HOUR_MSK);
	/* Date in BCD format */
	STM32_RTC->dr =
		(STM32_RTC->dr &
			~(STM32F2_RTC_DR_MDAY_MSK | STM32F2_RTC_DR_MON_MSK |
			STM32F2_RTC_DR_YEAR_MSK | STM32F2_RTC_DR_WDAY_MSK)) |
		((bin2bcd(tm->tm_mday) << STM32F2_RTC_DR_MDAY_BITS) &
			STM32F2_RTC_DR_MDAY_MSK) |
		((bin2bcd(tm->tm_mon + 1) << STM32F2_RTC_DR_MON_BITS) &
			STM32F2_RTC_DR_MON_MSK) |
		((bin2bcd(tm->tm_year % 100) << STM32F2_RTC_DR_YEAR_BITS) &
			STM32F2_RTC_DR_YEAR_MSK) |
		(((tm->tm_wday ? tm->tm_wday : 7) << STM32F2_RTC_DR_WDAY_BITS) &
			STM32F2_RTC_DR_WDAY_MSK);

	/* Exit initialization mode; enable write-protection */
	stm32f2_rtc_enable_init_mode(0);
	stm32f2_rtc_write_enable(0);

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Read the time and date the alarm (Alarm A) is set to
 */
static int stm32f2_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct stm32f2_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	u32 reg;

	spin_lock_irq(&rtc->lock);

	reg = STM32_RTC->alrmar;

	/* Comparison of year and month not supported by STM32F2 RTC alarm */
	tm->tm_year = -1;
	tm->tm_mon = -1;

	if (reg & STM32F2_RTC_ALRMR_DATEMASK_MSK) {
		/* Alarm triggers every day */
		tm->tm_mday = -1;
		tm->tm_wday = -1;
	} else if (reg & STM32F2_RTC_ALRMR_WDSEL_MSK) {
		/* Alarm is set to a day of week */
		tm->tm_mday = -1;
		/* 0 = Sun */
		tm->tm_wday = ((reg & STM32F2_RTC_ALRMR_WDAY_MSK) >>
			STM32F2_RTC_ALRMR_DAY_BITS) % 7;
	} else {
		/* Alarm is set to a day of month */
		tm->tm_mday = bcd2bin((reg & STM32F2_RTC_ALRMR_MDAY_MSK) >>
			STM32F2_RTC_ALRMR_DAY_BITS);
		tm->tm_wday = -1;
	}

	if (reg & STM32F2_RTC_ALRMR_HOURMASK_MSK) {
		tm->tm_hour = -1;
	} else {
		tm->tm_hour = bcd2bin((reg & STM32F2_RTC_ALRMR_HOUR_MSK) >>
			STM32F2_RTC_ALRMR_HOUR_BITS);
	}

	if (reg & STM32F2_RTC_ALRMR_MINMASK_MSK) {
		tm->tm_min = -1;
	} else {
		tm->tm_min = bcd2bin((reg & STM32F2_RTC_ALRMR_MIN_MSK) >>
			STM32F2_RTC_ALRMR_MIN_BITS);
	}

	if (reg & STM32F2_RTC_ALRMR_SECMASK_MSK) {
		tm->tm_sec = -1;
	} else {
		tm->tm_sec = bcd2bin((reg & STM32F2_RTC_ALRMR_SEC_MSK) >>
			STM32F2_RTC_ALRMR_SEC_BITS);
	}

	alrm->enabled = !!(STM32_RTC->cr & STM32F2_RTC_CR_ALRAE_MSK);
	alrm->pending = !!(STM32_RTC->isr & STM32F2_RTC_ISR_ALRAF_MSK);

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Set normal alarm.
 *
 * We use Alarm A, because Alarm B is already used to emulate update interrupt.
 */
static int stm32f2_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct stm32f2_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	int rv;

	if (rtc_valid_tm(tm)) {
		rv = -EINVAL;
		goto out;
	}

	spin_lock_irq(&rtc->lock);

	/* Set Alarm A */
	stm32f2_setup_alarm(
		0, tm->tm_mday, tm->tm_wday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	/* Enable Alarm A interrupts if needed */
	if (alrm->enabled)
		stm32f2_alarm_irq_enable(0, 1);

	spin_unlock_irq(&rtc->lock);
	rv = 0;
out:
	return rv;
}

static struct rtc_class_ops stm32f2_rtc_ops = {
	.read_time		= stm32f2_rtc_read_time,
	.set_time		= stm32f2_rtc_set_time,
	.read_alarm		= stm32f2_rtc_read_alarm,
	.set_alarm		= stm32f2_rtc_set_alarm,
	.irq_set_state		= stm32f2_rtc_irq_set_state,
	.irq_set_freq		= stm32f2_rtc_irq_set_freq,
	.alarm_irq_enable	= stm32f2_rtc_alarm_irq_enable,
	.update_irq_enable	= stm32f2_rtc_update_irq_enable,
};

/*
 * RTC device initialization function
 */
static int __devinit stm32f2_rtc_probe(struct platform_device *pdev)
{
	struct stm32f2_rtc *rtc;
	struct device *dev = &pdev->dev;
	int rv;

	/* Allocate memory for our RTC struct */
	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);
	if (unlikely(!rtc)) {
		rv = -ENOMEM;
		goto out;
	}

	platform_set_drvdata(pdev, rtc);
	device_init_wakeup(dev, 1);

	spin_lock_init(&rtc->lock);

	/* Register our RTC with the RTC framework */
	rtc->rtc_dev = rtc_device_register(
		pdev->name, dev, &stm32f2_rtc_ops, THIS_MODULE);
	if (unlikely(IS_ERR(rtc->rtc_dev))) {
		rv = PTR_ERR(rtc->rtc_dev);
		goto err;
	}

	/* Handle RTC alarm interrupts */
	rv = request_irq(STM32F2_IRQ_RTC_ALARM, stm32f2_rtc_alarm_irq, 0, pdev->name, rtc);
	if (unlikely(rv))
		goto err_reg;

	/* Handle RTC wakeup interrupts */
	rv = request_irq(STM32F2_IRQ_RTC_WAKEUP, stm32f2_rtc_wakeup_irq, 0, pdev->name, rtc);
	if (unlikely(rv))
		goto err_alarm_irq;

	rv = 0;
	goto out;

err_alarm_irq:
	free_irq(STM32F2_IRQ_RTC_ALARM, dev);
err_reg:
	rtc_device_unregister(rtc->rtc_dev);
err:
	kfree(rtc);
out:
	return rv;
}

static int __devexit stm32f2_rtc_remove(struct platform_device *pdev)
{
	struct stm32f2_rtc *rtc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	free_irq(STM32F2_IRQ_RTC_WAKEUP, dev);
	free_irq(STM32F2_IRQ_RTC_ALARM, dev);
	rtc_device_unregister(rtc->rtc_dev);
	platform_set_drvdata(pdev, NULL);
	kfree(rtc);

	return 0;
}

static struct platform_driver stm32f2_rtc_driver = {
	.driver		= {
		.name	= "rtc-stm32f2",
		.owner	= THIS_MODULE,
	},
	.probe		= stm32f2_rtc_probe,
	.remove		= __devexit_p(stm32f2_rtc_remove),
};

static int __init stm32f2_rtc_init(void)
{
	return platform_driver_register(&stm32f2_rtc_driver);
}

static void __exit stm32f2_rtc_exit(void)
{
	platform_driver_unregister(&stm32f2_rtc_driver);
}

module_init(stm32f2_rtc_init);
module_exit(stm32f2_rtc_exit);

MODULE_DESCRIPTION("STM32F2 On-Chip Real Time Clock Driver");
MODULE_AUTHOR("Alexander Potashev <aspotashev@emcraft.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtc-stm32f2");
