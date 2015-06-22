//======================================================================
// Driver for the Epson RTC module RX-6110 SA
//
// (C) Copyright 2015
// Emcraft Systems, <www.emcraft.com>
// Anton Protopopov <antonp@emcraft.com>
//
// Copyright(C) SEIKO EPSON CORPORATION 2013. All rights reserved.
//
// Derived from RX-8025 driver:
// Copyright (C) 2009 Wolfgang Grandegger <wg@grandegger.com>
//
// Copyright (C) 2005 by Digi International Inc.
// All rights reserved.
//
// Modified by fengjh at rising.com.cn
// <http://lists.lm-sensors.org/mailman/listinfo/lm-sensors>
// 2006.11
//
// Code cleanup by Sergei Poselenov, <sposelenov@emcraft.com>
// Converted to new style by Wolfgang Grandegger <wg@grandegger.com>
// Alarm and periodic interrupt added by Dmitry Rakhchev <rda@emcraft.com>
//
//
// This driver software is distributed as is, without any warranty of any kind,
// either express or implied as further specified in the GNU Public License. This
// software may be used and distributed according to the terms of the GNU Public
// License, version 2 as published by the Free Software Foundation.
// See the file COPYING in the main directory of this archive for more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.
//======================================================================

#if 1
#define DEBUG
#include <linux/device.h>
// #undef DEBUG
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/rtc.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio.h>


// RX-6110 Register definitions
#define RX6110_REG_SEC		0x10
#define RX6110_REG_MIN		0x11
#define RX6110_REG_HOUR		0x12
#define RX6110_REG_WDAY		0x13
#define RX6110_REG_MDAY		0x14
#define RX6110_REG_MONTH	0x15
#define RX6110_REG_YEAR		0x16
// 0x17 is reserved
#define RX6110_REG_ALMIN	0x18
#define RX6110_REG_ALHOUR	0x19
#define RX6110_REG_ALWDAY	0x1A
#define RX6110_REG_TCOUNT0	0x1B
#define RX6110_REG_TCOUNT1	0x1C
#define RX6110_REG_EXT		0x1D
#define RX6110_REG_FLAG		0x1E
#define RX6110_REG_CTRL		0x1F
#define RX6110_REG_USER0	0x20
#define RX6110_REG_USER1	0x21
#define RX6110_REG_USER2	0x22
#define RX6110_REG_USER3	0x23
#define RX6110_REG_USER4	0x24
#define RX6110_REG_USER5	0x25
#define RX6110_REG_USER6	0x26
#define RX6110_REG_USER7	0x27
#define RX6110_REG_USER8	0x28
#define RX6110_REG_USER9	0x29
#define RX6110_REG_USERA	0x2A
#define RX6110_REG_USERB	0x2B
#define RX6110_REG_USERC	0x2C
#define RX6110_REG_USERD	0x2D
#define RX6110_REG_USERE	0x2E
#define RX6110_REG_USERF	0x2F
// 0x30 is reserved
// 0x31 is reserved
#define RX6110_REG_IRQ		0x32

// Extension Register (1Dh) bit positions
#define RX6110_BIT_EXT_TSEL		(7 << 0)
#define RX6110_BIT_EXT_WADA		(1 << 3)
#define RX6110_BIT_EXT_TE		(1 << 4)
#define RX6110_BIT_EXT_USEL		(1 << 5)
#define RX6110_BIT_EXT_FSEL		(3 << 6)

// Flag Register (1Eh) bit positions
#define RX6110_BIT_FLAG_VLF		(1 << 1)
#define RX6110_BIT_FLAG_AF		(1 << 3)
#define RX6110_BIT_FLAG_TF		(1 << 4)
#define RX6110_BIT_FLAG_UF		(1 << 5)

// Control Register (1Fh) bit positions
#define RX6110_BIT_CTRL_TSTP	(1 << 2)
#define RX6110_BIT_CTRL_AIE		(1 << 3)
#define RX6110_BIT_CTRL_TIE		(1 << 4)
#define RX6110_BIT_CTRL_UIE		(1 << 5)
#define RX6110_BIT_CTRL_STOP	(1 << 6)
#define RX6110_BIT_CTRL_TEST	(1 << 7)


static const struct i2c_device_id rx6110_id[] = {
	{ "rx6110-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rx6110_id);

struct rx6110_data {
	struct i2c_client *client;
	struct rtc_device *rtc;
	struct work_struct work_1;
	struct work_struct work_2;
	u8 ctrlreg;
	int irq_1;
	int irq_2;
	unsigned exiting:1;
};

typedef struct {
	u8 number;
	u8 value;
}reg_data;

#define SE_RTC_REG_READ		_IOWR('p', 0x20, reg_data)
#define SE_RTC_REG_WRITE	_IOW('p',  0x21, reg_data)

//----------------------------------------------------------------------
// rx6110_get_week_day()
//
//----------------------------------------------------------------------
static int rx6110_get_week_day( u8 reg_week_day )
{
	int i, tm_wday = -1;

	for ( i=0; i < 7; i++ )
	{
		if ( reg_week_day & 1 )
		{
			tm_wday = i;
			break;
		}
		reg_week_day >>= 1;
	}

	return 	tm_wday;
}

//----------------------------------------------------------------------
// rx6110_read_reg()
// reads a rx6110 register (see Register defines)
// See also rx6110_read_regs() to read multiple registers.
//
//----------------------------------------------------------------------
static int rx6110_read_reg(struct i2c_client *client, int number, u8 *value)
{
	int ret = i2c_smbus_read_byte_data(client, number) ;

	//check for error
	if (ret < 0) {
		dev_err(&client->dev, "Unable to read register #%d\n", number);
		return ret;
	}

	*value = ret;
	return 0;
}

//----------------------------------------------------------------------
// rx6110_read_regs()
// reads a specified number of rx6110 registers (see Register defines)
// See also rx6110_read_reg() to read single register.
//
//----------------------------------------------------------------------
static int rx6110_read_regs(struct i2c_client *client, int number, u8 length, u8 *values)
{
	int ret = i2c_smbus_read_i2c_block_data(client, number, length, values);

	//check for length error
	if (ret != length) {
		dev_err(&client->dev, "Unable to read registers #%d..#%d\n", number, number + length - 1);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

//----------------------------------------------------------------------
// rx6110_write_reg()
// writes a rx6110 register (see Register defines)
// See also rx6110_write_regs() to write multiple registers.
//
//----------------------------------------------------------------------
static int rx6110_write_reg(struct i2c_client *client, int number, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, number, value);

	//check for error
	if (ret)
		dev_err(&client->dev, "Unable to write register #%d\n", number);

	return ret;
}

//----------------------------------------------------------------------
// rx6110_write_regs()
// writes a specified number of rx6110 registers (see Register defines)
// See also rx6110_write_reg() to write a single register.
//
//----------------------------------------------------------------------
static int rx6110_write_regs(struct i2c_client *client, int number, u8 length, u8 *values)
{
	int ret = i2c_smbus_write_i2c_block_data(client, number, length, values);

	//check for error
	if (ret)
		dev_err(&client->dev, "Unable to write registers #%d..#%d\n", number, number + length - 1);

	return ret;
}

//----------------------------------------------------------------------
// rx6110_irq_1()
// irq handler
//
//----------------------------------------------------------------------
static irqreturn_t rx6110_irq_1(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct rx6110_data *rx6110 = i2c_get_clientdata(client);

	disable_irq_nosync(irq);
	schedule_work(&rx6110->work_1);


	return IRQ_HANDLED;
}

//----------------------------------------------------------------------
// rx6110_work_1()
//
//----------------------------------------------------------------------
static void rx6110_work_1(struct work_struct *work)
{
	struct rx6110_data *rx6110 = container_of(work, struct rx6110_data, work_1);
	struct i2c_client *client = rx6110->client;
	struct mutex *lock = &rx6110->rtc->ops_lock;
	u8 status;

	mutex_lock(lock);

	if (rx6110_read_reg(client, RX6110_REG_FLAG, &status))
		goto out;

	// check VLF
	if ((status & RX6110_BIT_FLAG_VLF))
		dev_warn(&client->dev, "Frequency stop was detected, \
							probably due to a supply voltage drop\n");

	dev_dbg(&client->dev, "%s: RX6110_REG_FLAG: %xh\n", __func__, status);

	// periodic "fixed-cycle" timer
	if (status & RX6110_BIT_FLAG_TF) {
		status &= ~RX6110_BIT_FLAG_TF;
		local_irq_disable();
		rtc_update_irq(rx6110->rtc, 1, RTC_PF | RTC_IRQF);
		local_irq_enable();
	}

	// alarm function
	if (status & RX6110_BIT_FLAG_AF) {
		status &= ~RX6110_BIT_FLAG_AF;
		local_irq_disable();
		rtc_update_irq(rx6110->rtc, 1, RTC_AF | RTC_IRQF);
		local_irq_enable();
	}

	// time update function
	if (status & RX6110_BIT_FLAG_UF) {
		status &= ~RX6110_BIT_FLAG_UF;
		local_irq_disable();
		rtc_update_irq(rx6110->rtc, 1, RTC_UF | RTC_IRQF);
		local_irq_enable();
	}

	// acknowledge IRQ (clear flags)
	rx6110_write_reg(client, RX6110_REG_FLAG, status);

out:
	if (!rx6110->exiting)
	{
		if (rx6110->irq_1 > 0)
			enable_irq(rx6110->irq_1);
		else
			enable_irq(client->irq);
	}

	mutex_unlock(lock);
}

#if 0
//----------------------------------------------------------------------
// rx6110_irq_2()
// irq handler
//
//----------------------------------------------------------------------
static irqreturn_t rx6110_irq_2(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct rx6110_data *rx6110 = i2c_get_clientdata(client);

	disable_irq_nosync(irq);
	schedule_work(&rx6110->work_2);


	return IRQ_HANDLED;
}

//----------------------------------------------------------------------
// rx6110_work_2()
//
//----------------------------------------------------------------------
static void rx6110_work_2(struct work_struct *work)
{
	struct rx6110_data *rx6110 = container_of(work, struct rx6110_data, work_2);
	struct i2c_client *client = rx6110->client;
	struct mutex *lock = &rx6110->rtc->ops_lock;
	u8 status;

	mutex_lock(lock);

	if (rx6110_read_reg(client, RX6110_REG_FLAG, &status))
		goto out;

	// check VLF
	if ((status & RX6110_BIT_FLAG_VLF))
		dev_warn(&client->dev, "Frequency stop was detected, \
								probably due to a supply voltage drop\n");

	dev_dbg(&client->dev, "%s: RX6110_REG_FLAG: %xh\n", __func__, status);

	// periodic "fixed-cycle" timer
	if (status & RX6110_BIT_FLAG_TF) {
		status &= ~RX6110_BIT_FLAG_TF;
		local_irq_disable();
		rtc_update_irq(rx6110->rtc, 1, RTC_PF | RTC_IRQF);
		local_irq_enable();
	}

	// acknowledge IRQ (clear flags)
	rx6110_write_reg(client, RX6110_REG_FLAG, status);

out:
	if (!rx6110->exiting)
	{
		if (rx6110->irq_2 > 0)
			enable_irq(rx6110->irq_2);
		else
			enable_irq(client->irq);
	}

	mutex_unlock(lock);
}
#endif

//----------------------------------------------------------------------
// rx6110_get_time()
// gets the current time from the rx6110 registers
//
//----------------------------------------------------------------------
static int rx6110_get_time(struct device *dev, struct rtc_time *dt)
{
	struct rx6110_data *rx6110 = dev_get_drvdata(dev);
	u8 date[7];
	int err;

	err = rx6110_read_regs(rx6110->client, RX6110_REG_SEC, 7, date);
	if (err)
		return err;

	dev_dbg(dev, "%s: read 0x%02x 0x%02x "
		"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", __func__,
		date[0], date[1], date[2], date[3], date[4], date[5], date[6]);

	//Note: need to subtract 0x10 for index as register offset starts at 0x10
	dt->tm_sec  = bcd2bin(date[RX6110_REG_SEC  -0x10] & 0x7f);
	dt->tm_min  = bcd2bin(date[RX6110_REG_MIN  -0x10] & 0x7f);
	dt->tm_hour = bcd2bin(date[RX6110_REG_HOUR -0x10] & 0x3f);	//only 24-hour clock
	dt->tm_mday = bcd2bin(date[RX6110_REG_MDAY -0x10] & 0x3f);
	dt->tm_mon  = bcd2bin(date[RX6110_REG_MONTH-0x10] & 0x1f) - 1;
	dt->tm_year = bcd2bin(date[RX6110_REG_YEAR -0x10]);
	dt->tm_wday = rx6110_get_week_day(date[RX6110_REG_WDAY-0x10] & 0x7f);

	if (dt->tm_year < 70)
		dt->tm_year += 100;

	dev_dbg(dev, "%s: date %ds %dm %dh %dwd %dmd %dm %dy\n", __func__,
		dt->tm_sec, dt->tm_min, dt->tm_hour, dt->tm_wday,
		dt->tm_mday, dt->tm_mon, dt->tm_year);

	return rtc_valid_tm(dt);
}

//----------------------------------------------------------------------
// rx6110_set_time()
// Sets the current time in the rx6110 registers
//
//----------------------------------------------------------------------
static int rx6110_set_time(struct device *dev, struct rtc_time *dt)
{
	struct rx6110_data *rx6110 = dev_get_drvdata(dev);
	u8 date[7];
	u8 ctrl;
	int ret;

	dev_dbg(dev, "%s: date %ds %dm %dh %dwd %dmd %dm %dy\n", __func__,
		dt->tm_sec, dt->tm_min, dt->tm_hour, dt->tm_wday,
		dt->tm_mday, dt->tm_mon, dt->tm_year);

	//set STOP bit before changing clock/calendar
	rx6110_read_reg(rx6110->client, RX6110_REG_CTRL, &ctrl);
	rx6110->ctrlreg = ctrl | RX6110_BIT_CTRL_STOP;
	rx6110_write_reg(rx6110->client, RX6110_REG_CTRL, rx6110->ctrlreg);

	//Note: need to subtract 0x10 for index as register offset starts at 0x10
	date[RX6110_REG_SEC  -0x10] = bin2bcd(dt->tm_sec);
	date[RX6110_REG_MIN  -0x10] = bin2bcd(dt->tm_min);
	date[RX6110_REG_HOUR -0x10] = bin2bcd(dt->tm_hour);		//only 24hr time

	date[RX6110_REG_MDAY -0x10] = bin2bcd(dt->tm_mday);
	date[RX6110_REG_MONTH-0x10] = bin2bcd(dt->tm_mon + 1);
	date[RX6110_REG_YEAR -0x10] = bin2bcd(dt->tm_year % 100);
	date[RX6110_REG_WDAY -0x10] = 1 << (dt->tm_wday);

	dev_dbg(dev, "%s: write 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		__func__, date[0], date[1], date[2], date[3], date[4], date[5], date[6]);

	ret =  rx6110_write_regs(rx6110->client, RX6110_REG_SEC, 7, date);

	//clear STOP bit after changing clock/calendar
	rx6110_read_reg(rx6110->client, RX6110_REG_CTRL, &ctrl);
	rx6110->ctrlreg = ctrl & ~RX6110_BIT_CTRL_STOP;
	rx6110_write_reg(rx6110->client, RX6110_REG_CTRL, rx6110->ctrlreg);

	return ret;
}

//----------------------------------------------------------------------
// rx6110_init_client()
// initializes the rx6110
//
//----------------------------------------------------------------------
static int rx6110_init_client(struct i2c_client *client, int *need_reset)
{
	struct rx6110_data *rx6110 = i2c_get_clientdata(client);
	u8 ctrl[3]={0,0,0};
	int need_clear = 0;
	int err;

	//set reserved register 0x17 with specified value of 0xB8
	err = rx6110_write_reg(client, 0x17, 0xB8);
	if (err)
		goto out;

	//set reserved register 0x30 with specified value of 0x00
	err = rx6110_write_reg(client, 0x30, 0x00);
	if (err)
		goto out;

	//set reserved register 0x31 with specified value of 0x10
	err = rx6110_write_reg(client, 0x31, 0x10);
	if (err)
		goto out;

	//set reserved register 0x32 with default value
	err = rx6110_write_reg(client, RX6110_REG_IRQ, 0x00);
	if (err)
		goto out;

	//get current extension, flag, control register values
	err = rx6110_read_regs(client, RX6110_REG_EXT, 3, ctrl);
	if (err)
		goto out;

	//check for VLF Flag (set at power-on)
	if ((ctrl[1] & RX6110_BIT_FLAG_VLF)) {
		dev_warn(&client->dev, "Frequency stop was detected, probably due to a supply voltage drop\n");
		*need_reset = 1;
	}

	//check for Alarm Flag
	if (ctrl[1] & RX6110_BIT_FLAG_AF) {
		dev_warn(&client->dev, "Alarm was detected\n");
		need_clear = 1;
	}

	//check for Periodic Timer Flag
	if (ctrl[1] & RX6110_BIT_FLAG_TF) {
		dev_warn(&client->dev, "Periodic timer was detected\n");
		need_clear = 1;
	}

	//check for Update Timer Flag
	if (ctrl[1] & RX6110_BIT_FLAG_UF) {
		dev_warn(&client->dev, "Update timer was detected\n");
		need_clear = 1;
	}

	//reset or clear needed?
	if (*need_reset) {
		//clear 1d, 1e, 1f registers
		ctrl[0] = ctrl[1] = ctrl[2] = 0;
		err = rx6110_write_regs(client, RX6110_REG_EXT, 3, ctrl);
		if (err)
			goto out;
	}
	else if(need_clear){
		err = rx6110_write_reg(client, RX6110_REG_FLAG, 0);
		if (err)
			goto out;
	}

	//set "test bit" and reserved bits of control register zero
	rx6110->ctrlreg = (ctrl[2] & ~RX6110_BIT_CTRL_TEST);
out:
	return err;

}

//----------------------------------------------------------------------
// rx6110_read_alarm()
// reads current Alarm
//
//----------------------------------------------------------------------
static int rx6110_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct rx6110_data *rx6110 = dev_get_drvdata(dev);
	struct i2c_client *client = rx6110->client;
	u8 alarmvals[3];		//< minute, hour, week or month day values
	u8 ctrl[2];				//< exts and flags
	int err;

#if 0
	if (client->irq <= 0)
		return -EINVAL;
#endif

	//get current minute, hour alarm, week or month day values
	err = rx6110_read_regs(client, RX6110_REG_ALMIN, 3, alarmvals);
	if (err)
		return err;
	dev_dbg(dev, "%s: minutes:0x%02x hours:0x%02x w/m:0x%02x\n",
		__func__, alarmvals[0], alarmvals[1], alarmvals[2]);


	//get current extension, flag register values
	err = rx6110_read_regs(client, RX6110_REG_EXT, 2, ctrl);
	if (err)
		return err;
	dev_dbg(dev, "%s: exts:0x%02x flags:0x%02x\n", __func__, ctrl[0], ctrl[1]);

	// Hardware alarm precision is 1 minute
	t->time.tm_sec = 0;
	//0x7f filters AE bit currently
	t->time.tm_min = bcd2bin(alarmvals[0] & 0x7f);
	//0x3f filters AE bit currently, also 24hr only
	t->time.tm_hour = bcd2bin(alarmvals[1] & 0x3f);
	if ( ctrl[0] & RX6110_BIT_EXT_WADA )
	{
		t->time.tm_mday = (alarmvals[2]&0x80) ? -1 : bcd2bin(alarmvals[2] & 0x3f);
		t->time.tm_wday = -1;
	}
	else
	{
		t->time.tm_mday = -1;
		t->time.tm_wday = (alarmvals[2]&0x80) ? -1 : rx6110_get_week_day(alarmvals[2] & 0x7f);
	}

	t->time.tm_mon  = -1;
	t->time.tm_year = -1;

	dev_dbg(dev, "%s: date %ds %dm %dh %dwd %dmd %dm %dy\n", __func__,
		t->time.tm_sec, t->time.tm_min, t->time.tm_hour, t->time.tm_wday,
		t->time.tm_mday, t->time.tm_mon, t->time.tm_year);

	t->enabled = !!(rx6110->ctrlreg & RX6110_BIT_CTRL_AIE);		//check if interrupt is enabled
	t->pending = (ctrl[1] & RX6110_BIT_FLAG_AF) && t->enabled;	//check if flag is triggered

	return err;
}

//----------------------------------------------------------------------
// rx6110_set_alarm()
// sets Alarm
//
// Notes: - currently filters the AE bits (bit 7)
//        - assumes WADA setting is week (week/day)
//----------------------------------------------------------------------
static int rx6110_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rx6110_data *rx6110 = dev_get_drvdata(dev);
	u8 alarmvals[3];		//minute, hour, day
	u8 extreg;				//extension register
	u8 flagreg;				//flag register
	int err;

#if 0
	if (client->irq <= 0)
		return -EINVAL;
#endif

	dev_dbg(dev, "%s: date %ds %dm %dh %dwd %dmd %dm %dy\n", __func__,
		t->time.tm_sec, t->time.tm_min, t->time.tm_hour, t->time.tm_wday,
		t->time.tm_mday, t->time.tm_mon, t->time.tm_year);

	//get current extension register
	err = rx6110_read_reg(client, RX6110_REG_EXT, &extreg);
	if (err <0)
		return err;

	//get current flag register
	err = rx6110_read_reg(client, RX6110_REG_FLAG, &flagreg);
	if (err <0)
		return err;

	// Hardware alarm precision is 1 minute
	alarmvals[0] = bin2bcd(t->time.tm_min);
	alarmvals[1] = bin2bcd(t->time.tm_hour);
	alarmvals[2] = bin2bcd(t->time.tm_mday);
	dev_dbg(dev, "%s: write 0x%02x 0x%02x 0x%02x\n", __func__, alarmvals[0], alarmvals[1], alarmvals[2]);

	//check interrupt enable and disable
	if (rx6110->ctrlreg & RX6110_BIT_CTRL_AIE) {
		rx6110->ctrlreg &= ~RX6110_BIT_CTRL_AIE;
		err = rx6110_write_reg(rx6110->client, RX6110_REG_CTRL, rx6110->ctrlreg);
		if (err)
			return err;
	}

	//write the new minute and hour values
	err = rx6110_write_regs(rx6110->client, RX6110_REG_ALMIN, 2, alarmvals);
	if (err)
		return err;

	//set Week/Day bit
	// Week setting is typically not used, so we will assume "day" setting
	extreg |= RX6110_BIT_EXT_WADA;		//set to "day of month"
	err = rx6110_write_reg(rx6110->client, RX6110_REG_EXT, extreg);
	if (err)
		return err;

	//set Day of Month register
	if (alarmvals[2] == 0) {
		alarmvals[2] |= 0x80;	//turn on AE bit to ignore day of month (no zero day)
	}
	err = rx6110_write_reg(rx6110->client, RX6110_REG_ALWDAY, alarmvals[2]);

	if (err)
		return err;

	//clear Alarm Flag
	flagreg &= ~RX6110_BIT_FLAG_AF;
	err = rx6110_write_reg(rx6110->client, RX6110_REG_FLAG, flagreg);
	if (err)
		return err;

	//re-enable interrupt if required
	if (t->enabled) {
		//set update interrupt enable
# if 0
		if ( rx6110->rtc->uie_rtctimer.enabled )
			rx6110->ctrlreg |= RX6110_BIT_CTRL_UIE;
		//set alarm interrupt enable
		if ( rx6110->rtc->aie_timer.enabled )
			rx6110->ctrlreg |= RX6110_BIT_CTRL_AIE | RX6110_BIT_CTRL_UIE;
#endif

		err = rx6110_write_reg(rx6110->client, RX6110_REG_CTRL, rx6110->ctrlreg);
		if (err)
			return err;
	}

	return 0;
}

//----------------------------------------------------------------------
// rx6110_alarm_irq_enable()
// sets enables Alarm IRQ
//
// Todo: -
//
//----------------------------------------------------------------------
static int rx6110_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rx6110_data *rx6110 = dev_get_drvdata(dev);
	u8 flagreg;
	u8 ctrl;
	int err;

	//get the current ctrl settings
	ctrl = rx6110->ctrlreg;

	if (enabled)
	{
#if 0
		//set update interrupt enable
		if ( rx6110->rtc->uie_rtctimer.enabled )
			ctrl |= RX6110_BIT_CTRL_UIE;
		//set alarm interrupt enable
		if ( rx6110->rtc->aie_timer.enabled )
			ctrl |= RX6110_BIT_CTRL_AIE | RX6110_BIT_CTRL_UIE;
#endif
	}
	else
	{
#if 0
		//clear update interrupt enable
		if ( ! rx6110->rtc->uie_rtctimer.enabled )
			ctrl &= ~RX6110_BIT_CTRL_UIE;
		//clear alarm interrupt enable
		if ( ! rx6110->rtc->aie_timer.enabled )
		{
			if ( rx6110->rtc->uie_rtctimer.enabled )
				ctrl &= ~RX6110_BIT_CTRL_AIE;
			else
				ctrl &= ~(RX6110_BIT_CTRL_AIE | RX6110_BIT_CTRL_UIE);
		}
#endif
	}

	//clear alarm flag
	err = rx6110_read_reg(client, RX6110_REG_FLAG, &flagreg);
	if (err <0)
		return err;
	flagreg &= ~RX6110_BIT_FLAG_AF;
	err = rx6110_write_reg(rx6110->client, RX6110_REG_FLAG, flagreg);
	if (err)
		return err;

	//update the Control register if the setting changed
	if (ctrl != rx6110->ctrlreg) {
		rx6110->ctrlreg = ctrl;
		err = rx6110_write_reg(rx6110->client, RX6110_REG_CTRL, rx6110->ctrlreg);
		if (err)
			return err;
	}

	return 0;
}

//---------------------------------------------------------------------------
// rx6110_ioctl()
//
//---------------------------------------------------------------------------
static int rx6110_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct rx6110_data *rx6110 = dev_get_drvdata(dev);
	//struct mutex *lock = &rx6110->rtc->ops_lock;
	int ret = 0;
	void __user *argp = (void __user *)arg;
	reg_data reg;

	dev_dbg(dev, "%s: cmd=%x\n", __func__, cmd);

	switch (cmd) {
		case SE_RTC_REG_READ:
			if (copy_from_user(&reg, argp, sizeof(reg)))
				return -EFAULT;
			if ( reg.number < RX6110_REG_SEC || reg.number > RX6110_REG_IRQ )
				return -EFAULT;
			//mutex_lock(lock);
			ret = rx6110_read_reg(client, reg.number, &reg.value);
			//mutex_unlock(lock);
			if (! ret )
				return copy_to_user(argp, &reg, sizeof(reg)) ? -EFAULT : 0;
			break;

		case SE_RTC_REG_WRITE:
			if (copy_from_user(&reg, argp, sizeof(reg)))
				return -EFAULT;
			if ( reg.number < RX6110_REG_SEC || reg.number > RX6110_REG_IRQ )
				return -EFAULT;
			//mutex_lock(lock);
			ret = rx6110_write_reg(client, reg.number, reg.value);
			//mutex_unlock(lock);
			break;

#if 0
		case RTC_VL_READ:
			//mutex_lock(lock);
			ret = rx6110_read_reg(client, RX6110_REG_FLAG, &reg.value);
			//mutex_unlock(lock);
			if (! ret)
			{
				int tmp = !!(reg.value & RX6110_BIT_FLAG_VLF);
				return copy_to_user(argp, &tmp, sizeof(tmp)) ? -EFAULT : 0;
			}
			break;

		case RTC_VL_CLR:
			//mutex_lock(lock);
			ret = rx6110_read_reg(client, RX6110_REG_FLAG, &reg.value);
			if (! ret)
			{
				reg.value &= ~RX6110_BIT_FLAG_VLF;
				ret = rx6110_write_reg(client, RX6110_REG_FLAG, reg.value);
			}
			//mutex_unlock(lock);
			break;
#endif

		default:
			return -ENOIOCTLCMD;
	}

	return ret;
}

static struct rtc_class_ops rx6110_rtc_ops = {
	.read_time = rx6110_get_time,
	.set_time = rx6110_set_time,
	.read_alarm = rx6110_read_alarm,
	.set_alarm = rx6110_set_alarm,
	.alarm_irq_enable = rx6110_alarm_irq_enable,
	.ioctl = rx6110_ioctl,
};

//----------------------------------------------------------------------
// rx6110_probe()
// probe routine for the rx6110 driver
//
//----------------------------------------------------------------------
static int rx6110_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rx6110_data *rx6110;
	int err, i, irqs_success = 0, need_reset = 0;
	// const char * irq_name[2] = {"rx6110-irq_1", "rx6110-irq_2"};

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&adapter->dev, "doesn't support required functionality\n");
		err = -EIO;
		goto errout;
	}

	rx6110 = devm_kzalloc(&client->dev, sizeof(struct rx6110_data), GFP_KERNEL);
	if (!rx6110) {
		dev_err(&adapter->dev, "failed to alloc memory\n");
		err = -ENOMEM;
		goto errout;
	}

	rx6110->client = client;
	i2c_set_clientdata(client, rx6110);

	err = rx6110_init_client(client, &need_reset);
	if (err)
		goto errout;

	if (need_reset) {
		struct rtc_time tm;
		rtc_time_to_tm(0, &tm);		// set to 1970/1/1
		rx6110_set_time(&client->dev, &tm);
		dev_warn(&client->dev, " - time reset to 1970/1/1\n");
	}

	rx6110->rtc = rtc_device_register(client->name, &client->dev, &rx6110_rtc_ops, THIS_MODULE);

	if (IS_ERR(rx6110->rtc)) {
		err = PTR_ERR(rx6110->rtc);
		dev_err(&client->dev, "unable to register the class device\n");
		goto errout;
	}

	// get interrupts
	rx6110->irq_1 = rx6110->irq_2 = -1;
	for ( i=0; i < 2; i++ )
	{
#if 0
		int gpio = of_get_named_gpio(np, irq_name[i], 0);
		if (gpio_is_valid(gpio)) {
			int irq;
			err = devm_gpio_request_one(&client->dev, gpio, GPIOF_DIR_IN,
															irq_name[i]);
			if (err) {
				dev_err(&client->dev, "cannot request %s\n", irq_name[i]);
				goto errout_reg;
			}
			irq = gpio_to_irq(gpio);
			dev_dbg(&client->dev, "%s %d\n", irq_name[i], irq);
			if (irq <= 0) {
				dev_warn(&client->dev, "Failed to "
					"convert gpio #%d to %s\n",
					gpio, irq_name[i]);
				goto errout_reg;
			}
			err = devm_request_threaded_irq(&client->dev,irq, NULL,
											i==0 ? rx6110_irq_1 : rx6110_irq_2,
											IRQF_TRIGGER_LOW | IRQF_ONESHOT,
											irq_name[i],
											client);
			if (err) {
				dev_err(&client->dev, "unable to request %s\n", irq_name[i]);
				goto errout_reg;
			}
			if (i == 0)
			{
				rx6110->irq_1 = irq;
				INIT_WORK(&rx6110->work_1, rx6110_work_1);
			}
			else
			{
				rx6110->irq_2 = irq;
				INIT_WORK(&rx6110->work_2, rx6110_work_2);
			}
			irqs_success++;
		} else {
				dev_warn(&client->dev, "%s missing or invalid\n",
										irq_name[i]);
		}
#endif
	}

	// another irq request try if one failed above
	if ( ! irqs_success && client->irq > 0 ){
		dev_info(&client->dev, "IRQ %d supplied\n", client->irq);
		err = devm_request_threaded_irq(&client->dev,client->irq, NULL, rx6110_irq_1, IRQF_TRIGGER_LOW | IRQF_ONESHOT,"rx6110", client);

		if (err) {
			dev_err(&client->dev, "unable to request IRQ\n");
			goto errout_reg;
		}
		INIT_WORK(&rx6110->work_1, rx6110_work_1);
	}


	rx6110->rtc->irq_freq = 1;
	rx6110->rtc->max_user_freq = 1;

	return 0;

errout_reg:
	rtc_device_unregister(rx6110->rtc);

errout:
	dev_err(&adapter->dev, "probing for rx6110 failed\n");
	return err;
}

//----------------------------------------------------------------------
// rx6110_remove()
// remove routine for the rx6110 driver
//
// Todo: - maybe change kzalloc to devm_kzalloc
//       -
//----------------------------------------------------------------------
static int rx6110_remove(struct i2c_client *client)
{
	struct rx6110_data *rx6110 = i2c_get_clientdata(client);
	struct mutex *lock = &rx6110->rtc->ops_lock;

	if (client->irq > 0 || rx6110->irq_1 > 0 || rx6110->irq_2 > 0) {
		mutex_lock(lock);
		rx6110->exiting = 1;
		mutex_unlock(lock);

		//cancel_work
		if (rx6110->irq_1 > 0 || client->irq > 0)
			cancel_work_sync(&rx6110->work_1);
		if (rx6110->irq_2 > 0)
			cancel_work_sync(&rx6110->work_2);
	}

	rtc_device_unregister(rx6110->rtc);

	return 0;
}

static struct i2c_driver rx6110_driver = {
	.driver = {
		.name = "rtc-rx6110-i2c",
		.owner = THIS_MODULE,
	},
	.probe		= rx6110_probe,
	.remove		= rx6110_remove,
	.id_table	= rx6110_id,
};

static int __init rx6110_init(void)
{
	return i2c_add_driver(&rx6110_driver);
}

static void __exit rx6110_cleanup(void)
{
	i2c_del_driver(&rx6110_driver);
}

module_init(rx6110_init);
module_exit(rx6110_cleanup);

MODULE_AUTHOR("Val Krutov <val.krutov@erd.epson.com>");
MODULE_DESCRIPTION("RX-6110 SA RTC driver");
MODULE_LICENSE("GPL");
