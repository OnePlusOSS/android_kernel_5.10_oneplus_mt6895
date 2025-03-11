/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#endif

#include "flashlight-core.h"
#include "flashlight-dt.h"
/* device tree should be defined in flashlight-dt.h */
#ifndef MilkWayS3_DTNAME
#define MilkWayS3_DTNAME "mediatek,flashlights_24267"
#endif
#ifndef MilkWayS3_DTNAME_I2C
#define MilkWayS3_DTNAME_I2C "mediatek,strobe_main"
#endif
#define MilkWayS3_NAME "flashlights_24267"

/* define registers */
#define MilkWayS3_REG_DEVICE_ID            (0x00)
#define MilkWayS3_REG_LED_CTRL1            (0x80)
#define MilkWayS3_REG_TLED1_FLASH_BR_CTR   (0x81)
#define MilkWayS3_REG_TLED2_FLASH_BR_CTR   (0x82)
#define MilkWayS3_REG_FLED_TIMER           (0x83)
#define MilkWayS3_REG_TLED1_TORCH_BR_CTR   (0x84)
#define MilkWayS3_REG_TLED2_TORCH_BR_CTR   (0x85)
#define MilkWayS3_REG_LED_PRO              (0x86)
#define MilkWayS3_REG_LED_STAT1            (0x87)
#define MilkWayS3_REG_LED_STAT2            (0x88)
#define MilkWayS3_REG_LED_FLG              (0x89)
#define MilkWayS3_REG_LED_MASK             (0x8A)
#define MilkWayS3_REG_FL_TX_REPORT         (0x8B)

#define MilkWayS3_DISABLE              (0x01)
#define MilkWayS3_ENABLE_LED1_TORCH    (0x21)
#define MilkWayS3_ENABLE_LED1_FLASH    (0x81)
#define MilkWayS3_ENABLE_LED2_TORCH    (0x11)
#define MilkWayS3_ENABLE_LED2_FLASH    (0x41)
#define MilkWayS3_TORCH_RAMP_TIME      (0x00)
#define MilkWayS3_FLASH_TIMEOUT        (0x0A)

/* define channel, level */
#define MilkWayS3_CHANNEL_NUM          2
#define MilkWayS3_CHANNEL_CH1          0
#define MilkWayS3_CHANNEL_CH2          1
/* define level */
#define MilkWayS3_LEVEL_NUM 28
#define MilkWayS3_LEVEL_TORCH 8

#define MilkWayS3_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(milkways3_mutex);
static struct work_struct milkways3_work_ch1;
static struct work_struct milkways3_work_ch2;

/* define pinctrl */

/* define device id */
#define USE_SC6607_IC	0x66h

extern void oplus_chg_set_camera_on(bool val);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct i2c_client *milkways3_flashlight_client;


/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *milkways3_i2c_client;

/* platform data */
struct milkways3_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* milkways3 chip data */
struct milkways3_chip_data {
	struct i2c_client *client;
	struct milkways3_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int milkways3_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct milkways3_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* i2c wrapper function */
static int milkways3_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct milkways3_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}

/******************************************************************************
 * milkways3 operations
 *****************************************************************************/

static const int *milkways3_current;
static const unsigned char *milkways3_torch_level;
static const unsigned char *milkways3_flash_level;

static const int sc6607_current[MilkWayS3_LEVEL_NUM] = {
	25,   50,   75,   100,  125,  150,  175,  200,  250,  300,
	350,  400,  450,  500,  550,  600,  650,  700,  750,  800,
	850,  900,  950,  1000, 1050, 1100, 1150, 1200
};

/*Offset: 25mA(b0000000)
Step:12.5mA
Range: 25mA(b0000000)~500mA(b0100110~b1111111)*/
static const unsigned char sc6607_torch_level[MilkWayS3_LEVEL_NUM] = {
	0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*Offset:25mA(b0000000)
Step:12.5mA
Range: 25mA(b0000000)~1.5A(b1110110~b1111111)*/
static const unsigned char sc6607_flash_level[MilkWayS3_LEVEL_NUM] = {
	0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x12, 0x16,
	0x1A, 0x1E, 0x22, 0x26, 0x2A, 0x2E, 0x32, 0x36, 0x3A, 0x3E,
	0x42, 0x46, 0x4A, 0x4E, 0x52, 0x56, 0x5A, 0x5E
};

static volatile unsigned char milkways3_reg_enable;
static volatile int milkways3_level_ch1 = -1;
static volatile int milkways3_level_ch2 = -1;
static volatile int milkways3_flash_mode = -1;
static volatile int milkways3_charge_mode = 0;
static volatile int milkways3_charge_enable = 0;

static int milkways3_is_torch(int level)
{
	if (level >= MilkWayS3_LEVEL_TORCH)
		return -1;

	return 0;
}

static int milkways3_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= MilkWayS3_LEVEL_NUM)
		level = MilkWayS3_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
static int milkways3_enable_ch1(void)
{
	unsigned char reg, val;

	reg = MilkWayS3_REG_LED_CTRL1;
	if (!milkways3_is_torch(milkways3_level_ch1)) {
		/* torch mode */
		milkways3_reg_enable = MilkWayS3_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		if (milkways3_charge_enable == 0){
			oplus_chg_set_camera_on(1);
			milkways3_charge_enable = 1;
			pr_info("oplus_chg_set_camera_on set milkways3_charge_enable 1");
		}
		milkways3_reg_enable = MilkWayS3_ENABLE_LED1_FLASH;
	}
	val = milkways3_reg_enable;

	return milkways3_write_reg(milkways3_i2c_client, reg, val);
}

static int milkways3_enable_ch2(void)
{
	unsigned char reg, val;

	reg = MilkWayS3_REG_LED_CTRL1;
	if (!milkways3_is_torch(milkways3_level_ch2)) {
		/* torch mode */
		milkways3_reg_enable |= MilkWayS3_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		if (milkways3_charge_enable == 0){
			oplus_chg_set_camera_on(1);
			milkways3_charge_enable = 1;
		}
		milkways3_reg_enable |= MilkWayS3_ENABLE_LED2_FLASH;
	}
	val = milkways3_reg_enable;

	return milkways3_write_reg(milkways3_i2c_client, reg, val);
}

static int milkways3_enable(int channel)
{
	if (channel == MilkWayS3_CHANNEL_CH1)
		milkways3_enable_ch1();
	else if (channel == MilkWayS3_CHANNEL_CH2)
		milkways3_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}
	return 0;
}

/* flashlight disable function */
static int milkways3_disable_ch1(void)
{
	unsigned char reg, val;

	reg = MilkWayS3_REG_LED_CTRL1;
	val = MilkWayS3_DISABLE;
	return milkways3_write_reg(milkways3_i2c_client, reg, val);
}

static int milkways3_disable_ch2(void)
{
	unsigned char reg, val;

	reg = MilkWayS3_REG_LED_CTRL1;
	val = MilkWayS3_DISABLE;

	return milkways3_write_reg(milkways3_i2c_client, reg, val);
}

static int milkways3_disable(int channel)
{
	if (channel == MilkWayS3_CHANNEL_CH1) {
		milkways3_disable_ch1();
		pr_info("MilkWayS3_CHANNEL_CH1\n");
	} else if (channel == MilkWayS3_CHANNEL_CH2) {
		milkways3_disable_ch2();
		pr_info("MilkWayS3_CHANNEL_CH2\n");
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	if (milkways3_charge_enable == 1) {
		oplus_chg_set_camera_on(0);
		milkways3_charge_enable = 0;
		pr_info("oplus_chg_set_camera_on set milkways3_charge_enable 0");
	} else if (milkways3_flash_mode == 3 && milkways3_charge_mode == 1 && milkways3_charge_enable == 0) {
		oplus_chg_set_camera_on(1);
		milkways3_charge_enable = 1;
		milkways3_charge_mode = 0;
	}

	return 0;
}

/* set flashlight level */
static int milkways3_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = milkways3_verify_level(level);

	if (!milkways3_is_torch(level)) {
		/* set torch brightness level */
		reg = MilkWayS3_REG_TLED1_TORCH_BR_CTR;
		val = milkways3_torch_level[level];
	} else {
		/* set flash brightness level */
		reg = MilkWayS3_REG_TLED1_FLASH_BR_CTR;
		val = milkways3_flash_level[level];
	}
	milkways3_level_ch1 = level;
	ret = milkways3_write_reg(milkways3_i2c_client, reg, val);

	return ret;
}

int milkways3_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = milkways3_verify_level(level);

	if (!milkways3_is_torch(level)) {
		/* set torch brightness level */
		reg = MilkWayS3_REG_TLED2_TORCH_BR_CTR;
		val = milkways3_torch_level[level];
	} else {
		/* set flash brightness level */
		reg = MilkWayS3_REG_TLED2_FLASH_BR_CTR;
		val = milkways3_flash_level[level];
	}
	milkways3_level_ch1 = level;
	ret = milkways3_write_reg(milkways3_i2c_client, reg, val);

	return ret;
}

static int milkways3_set_level(int channel, int level)
{
	if (channel == MilkWayS3_CHANNEL_CH1)
		milkways3_set_level_ch1(level);
	else if (channel == MilkWayS3_CHANNEL_CH2)
		milkways3_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}
/* flashlight init */
int milkways3_init(void)
{
	int ret = 0;

	/* clear enable register */
	ret = milkways3_write_reg(milkways3_i2c_client, MilkWayS3_REG_LED_CTRL1, MilkWayS3_DISABLE);

	/* set torch current ramp time and flash timeout */
	ret |= milkways3_write_reg(milkways3_i2c_client, MilkWayS3_REG_TLED1_FLASH_BR_CTR, 0x56);
	ret |= milkways3_write_reg(milkways3_i2c_client, MilkWayS3_REG_TLED2_FLASH_BR_CTR, 0x00);
	ret |= milkways3_write_reg(milkways3_i2c_client, MilkWayS3_REG_FLED_TIMER, 0x9f);
	ret |= milkways3_write_reg(milkways3_i2c_client, MilkWayS3_REG_TLED1_TORCH_BR_CTR, 0x06);
	ret |= milkways3_write_reg(milkways3_i2c_client, MilkWayS3_REG_TLED2_TORCH_BR_CTR, 0x00);
	ret |= milkways3_write_reg(milkways3_i2c_client, MilkWayS3_REG_LED_PRO , 0x02);
	ret |= milkways3_write_reg(milkways3_i2c_client, MilkWayS3_REG_LED_MASK, 0x48);
	ret |= milkways3_write_reg(milkways3_i2c_client, MilkWayS3_REG_FL_TX_REPORT, 0x01);

	milkways3_flash_mode = -1;
	milkways3_charge_mode = 0;
	milkways3_charge_enable = 0;
  	if (ret < 0)
  		pr_info("Failed to init.\n");

	return ret;
}

/* flashlight uninit */
int milkways3_uninit(void)
{
	milkways3_disable(MilkWayS3_CHANNEL_CH1);
	milkways3_disable(MilkWayS3_CHANNEL_CH2);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer milkways3_timer_ch1;
static struct hrtimer milkways3_timer_ch2;
static unsigned int milkways3_timeout_ms[MilkWayS3_CHANNEL_NUM];

static void milkways3_work_disable_ch1(struct work_struct *data)
{
	pr_info("ht work queue callback\n");
	milkways3_disable_ch1();
}

static void milkways3_work_disable_ch2(struct work_struct *data)
{
	pr_info("lt work queue callback\n");
	milkways3_disable_ch2();
}

static enum hrtimer_restart milkways3_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&milkways3_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart milkways3_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&milkways3_work_ch2);
	return HRTIMER_NORESTART;
}

int milkways3_timer_start(int channel, ktime_t ktime)
{
	if (channel == MilkWayS3_CHANNEL_CH1)
		hrtimer_start(&milkways3_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == MilkWayS3_CHANNEL_CH2)
		hrtimer_start(&milkways3_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int milkways3_timer_cancel(int channel)
{
	if (channel == MilkWayS3_CHANNEL_CH1)
		hrtimer_cancel(&milkways3_timer_ch1);
	else if (channel == MilkWayS3_CHANNEL_CH2)
		hrtimer_cancel(&milkways3_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int milkways3_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= MilkWayS3_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		milkways3_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		milkways3_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (milkways3_timeout_ms[channel]) {
				ktime = ktime_set(milkways3_timeout_ms[channel] / 1000,
						(milkways3_timeout_ms[channel] % 1000) * 1000000);
				milkways3_timer_start(channel, ktime);
			}
			milkways3_enable(channel);
		} else {
			milkways3_disable(channel);
			milkways3_timer_cancel(channel);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = MilkWayS3_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = MilkWayS3_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = milkways3_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = milkways3_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_info("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = MilkWayS3_HW_TIMEOUT;
		break;
/*
	case FLASH_IOC_SET_FLASH_MODE:
		pr_info("FLASH_IOC_SET_FLASH_MODE(%d)\n", (int)fl_arg->arg);
		milkways3_flash_mode = fl_arg->arg;
		break;
*/
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int milkways3_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int milkways3_release(void)
{
	/* uninit chip and clear usage count */
/*
	mutex_lock(&milkways3_mutex);
	use_count--;
	if (!use_count)
		milkways3_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&milkways3_mutex);

	pr_info("Release: %d\n", use_count);
*/
	return 0;
}

static int milkways3_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&milkways3_mutex);
	if (set) {
		if (!use_count)
			ret = milkways3_init();
		use_count++;
		pr_info("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = milkways3_uninit();
		if (use_count < 0)
			use_count = 0;
		if (milkways3_charge_enable == 1) {
			oplus_chg_set_camera_on(0);
			milkways3_charge_enable = 0;
			pr_info("oplus_chg_set_camera_on set milkways3_charge_enable 0");
		}
		pr_info("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&milkways3_mutex);

	return ret;
}

static ssize_t milkways3_strobe_store(struct flashlight_arg arg)
{
	milkways3_set_driver(1);
	milkways3_set_level(arg.channel, arg.level);
	milkways3_timeout_ms[arg.channel] = 0;
	milkways3_enable(arg.channel);
	msleep(arg.dur);
	milkways3_disable(arg.channel);
	//milkways3_release(NULL);
	milkways3_set_driver(0);
	return 0;
}

static struct flashlight_operations milkways3_ops = {
	milkways3_open,
	milkways3_release,
	milkways3_ioctl,
	milkways3_strobe_store,
	milkways3_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int milkways3_chip_init(struct milkways3_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * milkways3_init();
	 */

	return 0;
}

static int milkways3_parse_dt(struct device *dev,
		struct milkways3_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				MilkWayS3_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int milkways3_check_i2c(void)
{
	int ret;
	ret = milkways3_read_reg(milkways3_i2c_client, MilkWayS3_REG_LED_CTRL1);
	return ret;
}

static int milkways3_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct milkways3_chip_data *chip;
	struct milkways3_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	int i;
	bool curProject = false;
	pr_info("milkways3_i2c_probe Probe start.\n");

	curProject = is_project(24267) || is_project(24268) || is_project(24269);

	if (!curProject) {
		err = -ENODEV;
		goto err_out;
	}

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct milkways3_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct milkways3_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = milkways3_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	milkways3_i2c_client = client;

	if (milkways3_check_i2c() < 0) {
		pr_info("milkways3_i2c_probe I2C match fail.\n");
		goto err_free;
	}

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&milkways3_work_ch1, milkways3_work_disable_ch1);
	INIT_WORK(&milkways3_work_ch2, milkways3_work_disable_ch2);

	/* init timer */
	hrtimer_init(&milkways3_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	milkways3_timer_ch1.function = milkways3_timer_func_ch1;
	hrtimer_init(&milkways3_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	milkways3_timer_ch2.function = milkways3_timer_func_ch2;
	milkways3_timeout_ms[MilkWayS3_CHANNEL_CH1] = 100;
	milkways3_timeout_ms[MilkWayS3_CHANNEL_CH2] = 100;

	/* init chip hw */
	milkways3_chip_init(chip);

	milkways3_current = sc6607_current;
	milkways3_torch_level = sc6607_torch_level;
	milkways3_flash_level = sc6607_flash_level;

	/* register flashlight operations */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&milkways3_ops)) {
				pr_err("Failed to register flashlight device.\n");
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(MilkWayS3_NAME, &milkways3_ops)) {
			pr_err("Failed to register flashlight device.\n");
			err = -EFAULT;
			goto err_free;
		}
	}

	// milkways3_create_sysfs(client);

	use_count = 0;

	pr_info("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int milkways3_i2c_remove(struct i2c_client *client)
{
	struct milkways3_platform_data *pdata = dev_get_platdata(&client->dev);
	struct milkways3_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_info("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(MilkWayS3_NAME);
	/* flush work queue */
	flush_work(&milkways3_work_ch1);
	flush_work(&milkways3_work_ch2);

	/* unregister flashlight operations */
	flashlight_dev_unregister(MilkWayS3_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	pr_info("Remove done.\n");

	return 0;
}

static const struct i2c_device_id milkways3_i2c_id[] = {
	{MilkWayS3_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id milkways3_i2c_of_match[] = {
	{.compatible = MilkWayS3_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, milkways3_i2c_of_match);
#endif

static struct i2c_driver milkways3_i2c_driver = {
	.driver = {
		   .name = MilkWayS3_NAME,
#ifdef CONFIG_OF
		   .of_match_table = milkways3_i2c_of_match,
#endif
		   },
	.probe = milkways3_i2c_probe,
	.remove = milkways3_i2c_remove,
	.id_table = milkways3_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int milkways3_probe(struct platform_device *dev)
{
	pr_info("Probe start %s.\n", MilkWayS3_DTNAME_I2C);

	/* init pinctrl */
	if (0) {
		pr_err("No pinctrl, check project failed.\n");
		return -1;
	}

	if (i2c_add_driver(&milkways3_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -1;
	}

	pr_info("Probe done.\n");

	return 0;
}

static int milkways3_remove(struct platform_device *dev)
{
	pr_info("Remove start.\n");

	i2c_del_driver(&milkways3_i2c_driver);

	pr_info("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id milkways3_of_match[] = {
	{.compatible = MilkWayS3_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, milkways3_of_match);
#else
static struct platform_device milkways3_platform_device[] = {
	{
		.name = MilkWayS3_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, milkways3_platform_device);
#endif

static struct platform_driver milkways3_platform_driver = {
	.probe = milkways3_probe,
	.remove = milkways3_remove,
	.driver = {
		.name = MilkWayS3_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = milkways3_of_match,
#endif
	},
};

static int __init flashlight_milkways3_init(void)
{
	int ret;

	pr_info("flashlight_milkways3-Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&milkways3_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&milkways3_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_info("flashlight_milkways3 Init done.\n");

	return 0;
}

static void __exit flashlight_milkways3_exit(void)
{
	pr_info("flashlight_milkways3-Exit start.\n");

	platform_driver_unregister(&milkways3_platform_driver);

	pr_info("flashlight_milkways3 Exit done.\n");
}


module_init(flashlight_milkways3_init);
module_exit(flashlight_milkways3_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight MilkWayS3 Driver");

