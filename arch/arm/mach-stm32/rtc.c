/*
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

#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/stm32.h>
#include <mach/rtc.h>
#include <mach/exti.h>

/*
 * Power Control module (PWR) register map
 */
struct stm32f2_pwr_regs {
	u32 cr;		/* PWR power control register */
	u32 csr;	/* PWR power control/status register */
};

/*
 * PWR power control register
 */
/* Disable backup domain write protection */
#define STM32F2_PWR_CR_DBP_MSK		(1 << 8)

/*
 * RCC Backup domain control register
 */
/* RTC clock enable */
#define STM32F2_RCC_BDCR_RTCEN_MSK	(1 << 15)
/* RTC clock source selection */
#define STM32F2_RCC_BDCR_RTCSEL_BITS	8
#define STM32F2_RCC_BDCR_RTCSEL_MSK	(3 << STM32F2_RCC_BDCR_RTCSEL_BITS)
#define STM32F2_RCC_BDCR_RTCSEL_LSE	(1 << STM32F2_RCC_BDCR_RTCSEL_BITS)
/* External low-speed oscillator ready */
#define STM32F2_RCC_BDCR_LSERDY_MSK	(1 << 1)
/*  External low-speed oscillator enable */
#define STM32F2_RCC_BDCR_LSEON_MSK	(1 << 0)

/* STM32 ENR bit for Power Control module */
#define STM32_RCC_ENR_PWREN		(1 << 28)

#if defined(CONFIG_STM32_RTC)
/*
 * PWR registers base
 */
#define STM32F2_PWR_BASE		0x40007000
#define STM32F2_PWR			((volatile struct stm32f2_pwr_regs *) \
					STM32F2_PWR_BASE)

static struct platform_device rtc_device = {
	.name = "rtc-stm32f2",
	.id   = -1,
};
#endif

void __init stm32_rtc_init(void)
{
#if defined(CONFIG_STM32_RTC)
	int rv;

	/* Enable PWR clock to access the RTC module */
	STM32_RCC->apb1enr |= STM32_RCC_ENR_PWREN;

	/* Allow access to RTC */
	STM32F2_PWR->cr |= STM32F2_PWR_CR_DBP_MSK;

	/* Disable the low-speed external oscillator */
	STM32_RCC->bdcr = 0;
	/* Enable the low-speed external oscillator */
	STM32_RCC->bdcr = STM32F2_RCC_BDCR_LSEON_MSK;

	/* Wait till LSE is ready */
	while (!(STM32_RCC->bdcr & STM32F2_RCC_BDCR_LSERDY_MSK));

	/* Select low-speed external oscillator (LSE) for RTC */
	STM32_RCC->bdcr =
		(STM32_RCC->bdcr & ~STM32F2_RCC_BDCR_RTCSEL_MSK) |
		STM32F2_RCC_BDCR_RTCSEL_LSE;

	/* Enable the RTC */
	STM32_RCC->bdcr |= STM32F2_RCC_BDCR_RTCEN_MSK;

	/* Enable RTC event lines in the event controller (EXTI) */
	stm32_exti_enable_int(STM32F2_EXTI_LINE_RTC_ALARM, 1);
	stm32_exti_enable_int(STM32F2_EXTI_LINE_RTC_WAKEUP, 1);

	/*
	 * Register the STM32F2 Real-Time Clock device
	 */
	rv = platform_device_register(&rtc_device);
	if (rv != 0)
		pr_err("%s: Failed to register RTC device\n", __func__);
#endif
}
