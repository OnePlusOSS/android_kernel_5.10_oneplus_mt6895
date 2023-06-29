// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/pm_runtime.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
#include "flashlight-core.h"

#include <linux/power_supply.h>
#endif

#define AW36518_22282_NAME	"aw36518_22282"

/* registers definitions */
/* TODO: define register */
#define AW36518_22282_REG_CHIP_ID               0x00
#define AW36518_22282_REG_ENABLE                0x01
#define AW36518_22282_REG_IVFM                  0x02
#define AW36518_22282_REG_LED_FLASH_BRIGHTNESS  0x03
#define AW36518_22282_REG_LED_TORCH_BRIGHTNESS  0x05
#define AW36518_22282_REG_BOOST_CONF            0x07
#define AW36518_22282_REG_TIMING_CONF           0x08
#define AW36518_22282_REG_FLAG1                 0x0A
#define AW36518_22282_REG_FLAG2                 0x0B
#define AW36518_22282_REG_DEVICE_ID             0x0C
#define AW36518_22282_REG_LAST_FLASH            0x0D
/* AW36518_22282A DEFINED ONLY */
#define AW36518_22282_REG_VERSION_ID            0x25
#define AW36518_22282_REG_ADJUST                0x31
#define AW36518_22282_REG_WPRT                  0x69

/* bit 0-1 */
#define AW36518_22282_MASK_ENABLE_LED           0x03
#define AW36518_22282_ENABLE_OFF                0x00
#define AW36518_22282_ENABLE_ON                 0x03
/* bit 2-3 */
#define AW36518_22282_MASK_ENABLE_MODE          0x0C
#define AW36518_22282_ENABLE_MODE_STANDBY       0x00
#define AW36518_22282_ENABLE_MODE_IR_DRIVE      0x04
#define AW36518_22282_ENABLE_MODE_TORCH         0x08
#define AW36518_22282_ENABLE_MODE_FLASH         0x0c
/* bit 4-7 */
#define AW36518_22282_ENABLE_TORCH_ON           0x10
#define AW36518_22282_ENABLE_STROBE_ON          0x20
#define AW36518_22282_ENABLE_STROBE_TYPE_EDGE   0x40  // default level triggered
#define AW36518_22282_ENABLE_TX_PIN_ON          0x80

/* Fault Mask */
#define AW36518_22282_FAULT_1_FLASH_TIMEOUT     (1 << 0)
#define AW36518_22282_FAULT_1_UVLO              (1 << 1)
#define AW36518_22282_FAULT_1_THERMAL_SHUTDOWN  (1 << 2)
#define AW36518_22282_FAULT_1_CURRENT_LIMIT     (1 << 3)
#define AW36518_22282_FAULT_1_LED_SHORT_1       (1 << 4)
#define AW36518_22282_FAULT_1_LED_SHORT_2       (1 << 5)
#define AW36518_22282_FAULT_1_VOUT_SHORT        (1 << 6)
#define AW36518_22282_FAULT_1_TX                (1 << 7)
#define AW36518_22282_FAULT_2_OVP               (1 << 1)
#define AW36518_22282_FAULT_2_IVFM_TRIP         (1 << 2)

/* constsant definitions */
#define AW36518_22282_CHANNEL_NUM   1
#define AW36518_22282_CHANNEL_CH1   0

/* 2.94mA~1.5A, 5.87mA/level */
#define AW36518_22282_FLASH_BRT_MIN 2940
#define AW36518_22282_FLASH_BRT_STEP 5870
#define AW36518_22282_FLASH_BRT_MAX 1500000

/* 40ms~1.6s, 16 levels*/
#define AW36518_22282_FLASH_TOUT_MIN 200
#define AW36518_22282_FLASH_TOUT_STEP 200
#define AW36518_22282_FLASH_TOUT_MAX 1600
#define AW36518_22282_FLASH_TOUT_DEFAULT 600

/* 0.75mA~386mA, 1.51mA/level */
#define AW36518_22282_TORCH_BRT_MIN 750
#define AW36518_22282_TORCH_BRT_STEP 1510
#define AW36518_22282_TORCH_BRT_MAX 386000


#define AW36518_22282_FLASH_BRT_uA_TO_REG(a)	\
	((a) < AW36518_22282_FLASH_BRT_MIN ? 0 :	\
	 (((a) - AW36518_22282_FLASH_BRT_MIN) / AW36518_22282_FLASH_BRT_STEP))
#define AW36518_22282_FLASH_BRT_REG_TO_uA(a)		\
	((a) * AW36518_22282_FLASH_BRT_STEP + AW36518_22282_FLASH_BRT_MIN)

#define AW36518_22282_TORCH_BRT_uA_TO_REG(a)	\
	((a) < AW36518_22282_TORCH_BRT_MIN ? 0 :	\
	 (((a) - AW36518_22282_TORCH_BRT_MIN) / AW36518_22282_TORCH_BRT_STEP))
#define AW36518_22282_TORCH_BRT_REG_TO_uA(a)		\
	((a) * AW36518_22282_TORCH_BRT_STEP + AW36518_22282_TORCH_BRT_MIN)

#define AW36518_22282_LEVEL_TORCH 15
#define AW36518_22282_LEVEL_NUM   32
#define AW36518_22282_LEVEL_FLASH AW36518_22282_LEVEL_NUM

static const int aw36518_22282_current[AW36518_22282_LEVEL_NUM] = {
	  25,   50,  75, 100, 125, 150, 175,  200,  225,  250,
	 275,  300, 325, 350, 375, 400, 450,  500,  550,  600,
	 650,  700, 750, 800, 850, 900, 950, 1000, 1050, 1100,
	1150, 1200
};

enum aw36518_22282_led_id {
	AW36518_22282_LED0 = 0,
	AW36518_22282_LED_MAX
};

/* struct aw36518_22282_platform_data
 *
 * @max_flash_timeout: flash timeout
 * @max_flash_brt: flash mode led brightness
 * @max_torch_brt: torch mode led brightness
 */
struct aw36518_22282_platform_data {
	u32 max_flash_timeout;
	u32 max_flash_brt[AW36518_22282_LED_MAX];
	u32 max_torch_brt[AW36518_22282_LED_MAX];
};

enum aw36518_22282_ic_version {
	FLASHLIGHT_IC_AW36518_22282 = 0,
	FLASHLIGHT_IC_AW36518_22282A,
	FLASHLIGHT_IC_VER_NUM
};

/**
 * struct aw36518_22282_flash
 *
 * @dev: pointer to &struct device
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: V4L2 LED mode
 * @ctrls_led: V4L2 controls
 * @subdev_led: V4L2 subdev
 */
struct aw36518_22282_flash {
	struct device *dev;
	struct aw36518_22282_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	enum v4l2_flash_led_mode led_mode;
	struct v4l2_ctrl_handler ctrls_led[AW36518_22282_LED_MAX];
	struct v4l2_subdev subdev_led[AW36518_22282_LED_MAX];
	struct pinctrl *aw36518_22282_hwen_pinctrl;
	struct pinctrl_state *aw36518_22282_hwen_high;
	struct pinctrl_state *aw36518_22282_hwen_low;
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
	struct flashlight_device_id flash_dev_id[AW36518_22282_LED_MAX];
#endif
	int level;
	enum aw36518_22282_ic_version ic_version;
};

/* define work queue */
static struct work_struct aw36518_22282_work;

/* define usage count */
static int use_count;

static struct aw36518_22282_flash *aw36518_22282_flash_data;

#define to_aw36518_22282_flash(_ctrl, _no)	\
	container_of(_ctrl->handler, struct aw36518_22282_flash, ctrls_led[_no])

/* define pinctrl */
/* TODO: define pinctrl */
#define AW36518_22282_PINCTRL_PIN_HWEN 0
#define AW36518_22282_PINCTRL_PINSTATE_LOW 0
#define AW36518_22282_PINCTRL_PINSTATE_HIGH 1
#define AW36518_22282_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define AW36518_22282_PINCTRL_STATE_HWEN_LOW  "hwen_low"
/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int aw36518_22282_pinctrl_init(struct aw36518_22282_flash *flash)
{
	int ret = 0;

	/* get pinctrl */
	flash->aw36518_22282_hwen_pinctrl = devm_pinctrl_get(flash->dev);
	if (IS_ERR(flash->aw36518_22282_hwen_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(flash->aw36518_22282_hwen_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	flash->aw36518_22282_hwen_high = pinctrl_lookup_state(
			flash->aw36518_22282_hwen_pinctrl,
			AW36518_22282_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(flash->aw36518_22282_hwen_high)) {
		pr_info("Failed to init (%s)\n",
			AW36518_22282_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(flash->aw36518_22282_hwen_high);
	}
	flash->aw36518_22282_hwen_low = pinctrl_lookup_state(
			flash->aw36518_22282_hwen_pinctrl,
			AW36518_22282_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(flash->aw36518_22282_hwen_low)) {
		pr_info("Failed to init (%s)\n", AW36518_22282_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(flash->aw36518_22282_hwen_low);
	}

	return ret;
}

static int aw36518_22282_pinctrl_set(struct aw36518_22282_flash *flash, int pin, int state)
{
	int ret = 0;

	if (IS_ERR(flash->aw36518_22282_hwen_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case AW36518_22282_PINCTRL_PIN_HWEN:
		if (state == AW36518_22282_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(flash->aw36518_22282_hwen_low))
			pinctrl_select_state(flash->aw36518_22282_hwen_pinctrl,
					flash->aw36518_22282_hwen_low);
		else if (state == AW36518_22282_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(flash->aw36518_22282_hwen_high))
			pinctrl_select_state(flash->aw36518_22282_hwen_pinctrl,
					flash->aw36518_22282_hwen_high);
		else
			pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("pin(%d) state(%d)\n", pin, state);

	return ret;
}

static int aw36518_22282_check_ic_version(struct aw36518_22282_flash *flash)
{
	int rval = EINVAL;
	unsigned int reg_val = 0;

	rval = regmap_read(flash->regmap, AW36518_22282_REG_DEVICE_ID, &reg_val);
	pr_debug("[%s]: FLASHLIGHT_DEVICE_ID = 0x%x\n", __func__, rval);
	rval = regmap_read(flash->regmap, AW36518_22282_REG_VERSION_ID, &reg_val);
	pr_debug("[%s]: FLASHLIGHT_VERSION_ID = 0x%x\n", __func__, rval);
	if (rval < 0) {
		/* addr not available, recognized as AW36518_22282 */
		pr_debug("[%s]: FLASHLIGHT_IC_VERSION = FLASHLIGHT_IC_AW36518_22282\n", __func__);
		flash->ic_version = FLASHLIGHT_IC_AW36518_22282;
	} else if (reg_val & 0x04) {
		pr_debug("[%s]: FLASHLIGHT_IC_VERSION = FLASHLIGHT_IC_AW36518_22282A\n", __func__);
		flash->ic_version = FLASHLIGHT_IC_AW36518_22282A;
	} else {
		pr_debug("[%s]: FLASHLIGHT_IC_VERSION = FLASHLIGHT_IC_AW36518_22282\n", __func__);
		flash->ic_version = FLASHLIGHT_IC_AW36518_22282;
	}
	return 0;
}

/* enable mode control */
static int aw36518_22282_mode_ctrl(struct aw36518_22282_flash *flash)
{
	int rval = -EINVAL;

	/* TODO: wrap mode ctrl function */
	if (FLASHLIGHT_IC_AW36518_22282A == flash->ic_version) {
		/* set internal sampling delay */
		rval = regmap_update_bits(flash->regmap, AW36518_22282_REG_ADJUST, 0xff, 0x02);
		if (rval < 0) return rval;
		rval = regmap_update_bits(flash->regmap, AW36518_22282_REG_WPRT, 0xff, 0x0C);
		if (rval < 0) return rval;
	}

	switch (flash->led_mode) {
	case V4L2_FLASH_LED_MODE_NONE:
		/* turn off */
		rval = regmap_update_bits(flash->regmap, AW36518_22282_REG_ENABLE,
			AW36518_22282_MASK_ENABLE_MODE, AW36518_22282_ENABLE_MODE_STANDBY);
		break;
	case V4L2_FLASH_LED_MODE_TORCH:
		/* torch mode */
		rval = regmap_update_bits(flash->regmap, AW36518_22282_REG_ENABLE,
			AW36518_22282_MASK_ENABLE_MODE, AW36518_22282_ENABLE_MODE_TORCH);
		break;
	case V4L2_FLASH_LED_MODE_FLASH:
		/* flash mode */
		rval = regmap_update_bits(flash->regmap, AW36518_22282_REG_ENABLE,
			AW36518_22282_MASK_ENABLE_MODE, AW36518_22282_ENABLE_MODE_FLASH);
		break;
	}
	return rval;
}

/* led1/2 enable/disable */
static int aw36518_22282_enable_ctrl(struct aw36518_22282_flash *flash,
			      enum aw36518_22282_led_id led_no, bool on)
{
	int rval = 0;
	unsigned int reg_val_0a = 0;
	unsigned int reg_val_0b = 0;

	pr_info_ratelimited("%s %d on:%d\n", __func__, led_no, on);
	/* TODO: wrap enable function */
	if (led_no == AW36518_22282_LED0) {
		if (FLASHLIGHT_IC_AW36518_22282A == flash->ic_version) {
			/* set internal sampling delay */
			rval = regmap_update_bits(flash->regmap, AW36518_22282_REG_ADJUST, 0xff, 0x02);
			if (rval < 0) return rval;
			rval = regmap_update_bits(flash->regmap, AW36518_22282_REG_WPRT, 0xff, 0x0c);
			if (rval < 0) return rval;
		}

	regmap_read(flash->regmap, AW36518_22282_REG_FLAG1, &reg_val_0a);
	regmap_read(flash->regmap, AW36518_22282_REG_FLAG2, &reg_val_0b);
	pr_info("[%s] read reg 0x0A = 0x%x, 0x0B = 0x%x\n",__func__,reg_val_0a,reg_val_0b);

		if (on) {
			/* enable led 0*/
			rval = regmap_update_bits(flash->regmap, AW36518_22282_REG_ENABLE,
					AW36518_22282_MASK_ENABLE_LED, AW36518_22282_ENABLE_ON);
		} else {
			/* disable led 0*/
			rval = regmap_update_bits(flash->regmap, AW36518_22282_REG_ENABLE,
					AW36518_22282_MASK_ENABLE_LED, AW36518_22282_ENABLE_OFF);
		}
	}
	return rval;
}

/* torch1/2 brightness control */
static int aw36518_22282_torch_brt_ctrl(struct aw36518_22282_flash *flash,
				 enum aw36518_22282_led_id led_no, unsigned int brt)
{
	int rval = 0;
	u8 br_bits;

	/* TODO: wrap set torch brightness function */
	pr_info_ratelimited("%s %d brt:%u\n", __func__, led_no, brt);
	if (brt < AW36518_22282_TORCH_BRT_MIN)
		return aw36518_22282_enable_ctrl(flash, led_no, false);
	if (brt > AW36518_22282_TORCH_BRT_MAX)
		brt = AW36518_22282_TORCH_BRT_MAX;

	br_bits = AW36518_22282_TORCH_BRT_uA_TO_REG(brt);
	if (led_no == AW36518_22282_LED0)
		rval = regmap_update_bits(flash->regmap,
					  AW36518_22282_REG_LED_TORCH_BRIGHTNESS, 0xff, br_bits);
	return rval;
}

/* flash1/2 brightness control */
static int aw36518_22282_flash_brt_ctrl(struct aw36518_22282_flash *flash,
				 enum aw36518_22282_led_id led_no, unsigned int brt)
{
	int rval = 0;
	u8 br_bits;

	/* TODO: wrap set flash brightness function */
	pr_info_ratelimited("%s %d brt:%u\n", __func__, led_no, brt);
	if (brt < AW36518_22282_FLASH_BRT_MIN)
		return aw36518_22282_enable_ctrl(flash, led_no, false);
	if (brt > AW36518_22282_FLASH_BRT_MAX)
		brt = AW36518_22282_FLASH_BRT_MAX;

	br_bits = AW36518_22282_FLASH_BRT_uA_TO_REG(brt);
	if (led_no == AW36518_22282_LED0)
		rval = regmap_update_bits(flash->regmap,
					  AW36518_22282_REG_LED_FLASH_BRIGHTNESS, 0xff, br_bits);
	return rval;
}

/* flash1/2 timeout control */
static int aw36518_22282_flash_tout_ctrl(struct aw36518_22282_flash *flash,
				unsigned int tout)
{
	int rval = 0;
	u8 tout_bits;

	/* TODO: wrap set flash timeout function */
	if (tout <= 400)
		tout_bits = (tout / 40) - 1;
	else
		tout_bits = (tout / 200) + 7;
	rval = regmap_update_bits(flash->regmap,
					  AW36518_22282_REG_TIMING_CONF, 0x0f, tout_bits);
	return rval;
}

/* v4l2 controls  */
static int aw36518_22282_get_ctrl(struct v4l2_ctrl *ctrl, enum aw36518_22282_led_id led_no)
{
	struct aw36518_22282_flash *flash = to_aw36518_22282_flash(ctrl, led_no);
	int rval = -EINVAL;

	mutex_lock(&flash->lock);

	/* TODO: wrap get hw fault function */
	if (ctrl->id == V4L2_CID_FLASH_FAULT) {
		s32 fault = 0;
		unsigned int reg_val = 0;

		rval = regmap_read(flash->regmap, AW36518_22282_REG_FLAG1, &reg_val);
		if (rval < 0)
			goto out;
		if (reg_val & AW36518_22282_FAULT_1_LED_SHORT_1)
			fault |= V4L2_FLASH_FAULT_SHORT_CIRCUIT;
		if (reg_val & AW36518_22282_FAULT_1_LED_SHORT_2)
			fault |= V4L2_FLASH_FAULT_SHORT_CIRCUIT;
		if (reg_val & AW36518_22282_FAULT_1_THERMAL_SHUTDOWN)
			fault |= V4L2_FLASH_FAULT_OVER_TEMPERATURE;
		if (reg_val & AW36518_22282_FAULT_1_FLASH_TIMEOUT)
			fault |= V4L2_FLASH_FAULT_TIMEOUT;
		ctrl->cur.val = fault;
	}

out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int aw36518_22282_set_ctrl(struct v4l2_ctrl *ctrl, enum aw36518_22282_led_id led_no)
{
	struct aw36518_22282_flash *flash = to_aw36518_22282_flash(ctrl, led_no);
	int rval = -EINVAL;

	mutex_lock(&flash->lock);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		flash->led_mode = ctrl->val;
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH)
			rval = aw36518_22282_mode_ctrl(flash);
		else
			rval = 0;
		if (flash->led_mode == V4L2_FLASH_LED_MODE_NONE)
			aw36518_22282_enable_ctrl(flash, led_no, false);
		else if (flash->led_mode == V4L2_FLASH_LED_MODE_TORCH)
			aw36518_22282_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_SOURCE:
		break;

	case V4L2_CID_FLASH_STROBE:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		flash->led_mode = V4L2_FLASH_LED_MODE_FLASH;
		rval = aw36518_22282_mode_ctrl(flash);
		aw36518_22282_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
		rval = aw36518_22282_mode_ctrl(flash);
		aw36518_22282_enable_ctrl(flash, led_no, false);
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		rval = aw36518_22282_flash_tout_ctrl(flash, ctrl->val);
		break;

	case V4L2_CID_FLASH_INTENSITY:
		rval = aw36518_22282_flash_brt_ctrl(flash, led_no, ctrl->val);
		break;

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		rval = aw36518_22282_torch_brt_ctrl(flash, led_no, ctrl->val);
		break;
	}

err_out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int aw36518_22282_led0_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return aw36518_22282_get_ctrl(ctrl, AW36518_22282_LED0);
}

static int aw36518_22282_led0_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return aw36518_22282_set_ctrl(ctrl, AW36518_22282_LED0);
}

static const struct v4l2_ctrl_ops aw36518_22282_led_ctrl_ops[AW36518_22282_LED_MAX] = {
	[AW36518_22282_LED0] = {
			.g_volatile_ctrl = aw36518_22282_led0_get_ctrl,
			.s_ctrl = aw36518_22282_led0_set_ctrl,
			},
};

static int aw36518_22282_init_controls(struct aw36518_22282_flash *flash,
				enum aw36518_22282_led_id led_no)
{
	struct v4l2_ctrl *fault;
	u32 max_flash_brt = flash->pdata->max_flash_brt[led_no];
	u32 max_torch_brt = flash->pdata->max_torch_brt[led_no];
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led[led_no];
	const struct v4l2_ctrl_ops *ops = &aw36518_22282_led_ctrl_ops[led_no];

	v4l2_ctrl_handler_init(hdl, 8);

	/* flash mode */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_LED_MODE,
			       V4L2_FLASH_LED_MODE_TORCH, ~0x7,
			       V4L2_FLASH_LED_MODE_NONE);
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;

	/* flash source */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_STROBE_SOURCE,
			       0x1, ~0x3, V4L2_FLASH_STROBE_SOURCE_SOFTWARE);

	/* flash strobe */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE, 0, 0, 0, 0);

	/* flash strobe stop */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE_STOP, 0, 0, 0, 0);

	/* flash strobe timeout */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TIMEOUT,
			  AW36518_22282_FLASH_TOUT_MIN,
			  flash->pdata->max_flash_timeout,
			  AW36518_22282_FLASH_TOUT_STEP,
			  flash->pdata->max_flash_timeout);

	/* flash brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  AW36518_22282_FLASH_BRT_MIN, max_flash_brt,
			  AW36518_22282_FLASH_BRT_STEP, max_flash_brt);

	/* torch brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  AW36518_22282_TORCH_BRT_MIN, max_torch_brt,
			  AW36518_22282_TORCH_BRT_STEP, max_torch_brt);

	/* fault */
	fault = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_FAULT, 0,
				  V4L2_FLASH_FAULT_OVER_VOLTAGE
				  | V4L2_FLASH_FAULT_OVER_TEMPERATURE
				  | V4L2_FLASH_FAULT_SHORT_CIRCUIT
				  | V4L2_FLASH_FAULT_TIMEOUT, 0, 0);
	if (fault != NULL)
		fault->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (hdl->error)
		return hdl->error;

	flash->subdev_led[led_no].ctrl_handler = hdl;
	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops aw36518_22282_ops = {
	.core = NULL,
};

static const struct regmap_config aw36518_22282_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static void aw36518_22282_v4l2_i2c_subdev_init(struct v4l2_subdev *sd,
		struct i2c_client *client,
		const struct v4l2_subdev_ops *ops)
{
	v4l2_subdev_init(sd, ops);
	sd->flags |= V4L2_SUBDEV_FL_IS_I2C;
	/* the owner is the same as the i2c_client's driver owner */
	sd->owner = client->dev.driver->owner;
	sd->dev = &client->dev;
	/* i2c_client and v4l2_subdev point to one another */
	v4l2_set_subdevdata(sd, client);
	i2c_set_clientdata(client, sd);
	/* initialize name */
	snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		client->dev.driver->name, i2c_adapter_id(client->adapter),
		client->addr);
}

static int aw36518_22282_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = pm_runtime_get_sync(sd->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(sd->dev);
		return ret;
	}

	return 0;
}

static int aw36518_22282_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pr_info("%s\n", __func__);

	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops aw36518_22282_int_ops = {
	.open = aw36518_22282_open,
	.close = aw36518_22282_close,
};

static int aw36518_22282_subdev_init(struct aw36518_22282_flash *flash,
			      enum aw36518_22282_led_id led_no, char *led_name)
{
	struct i2c_client *client = to_i2c_client(flash->dev);
	struct device_node *np = flash->dev->of_node, *child;
	const char *fled_name = "flash";
	int rval;

	// pr_info("%s %d", __func__, led_no);

	aw36518_22282_v4l2_i2c_subdev_init(&flash->subdev_led[led_no],
				client, &aw36518_22282_ops);
	flash->subdev_led[led_no].flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->subdev_led[led_no].internal_ops = &aw36518_22282_int_ops;
	strscpy(flash->subdev_led[led_no].name, led_name,
		sizeof(flash->subdev_led[led_no].name));

	for (child = of_get_child_by_name(np, fled_name); child;
			child = of_find_node_by_name(child, fled_name)) {
		int rv;
		u32 reg = 0;

		rv = of_property_read_u32(child, "reg", &reg);
		if (rv)
			continue;

		if (reg == led_no)
			flash->subdev_led[led_no].fwnode = of_fwnode_handle(child);
	}

	rval = aw36518_22282_init_controls(flash, led_no);
	if (rval)
		goto err_out;
	rval = media_entity_pads_init(&flash->subdev_led[led_no].entity, 0, NULL);
	if (rval < 0)
		goto err_out;
	flash->subdev_led[led_no].entity.function = MEDIA_ENT_F_FLASH;

	rval = v4l2_async_register_subdev(&flash->subdev_led[led_no]);
	if (rval < 0)
		goto err_out;

	return rval;

err_out:
	v4l2_ctrl_handler_free(&flash->ctrls_led[led_no]);
	return rval;
}

/* flashlight init */
static int aw36518_22282_init(struct aw36518_22282_flash *flash)
{
	int rval = 0;
	unsigned int reg_val;

	/* TODO: wrap init function */
	aw36518_22282_pinctrl_set(flash, AW36518_22282_PINCTRL_PIN_HWEN, AW36518_22282_PINCTRL_PINSTATE_HIGH);

	aw36518_22282_check_ic_version(flash);

	/* set timeout */
	rval = aw36518_22282_flash_tout_ctrl(flash, AW36518_22282_FLASH_TOUT_DEFAULT);
	if (rval < 0)
		return rval;
	/* output disable */
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
	flash->level = -1;
	rval = aw36518_22282_mode_ctrl(flash);
	if (rval < 0)
		return rval;

	rval = regmap_update_bits(flash->regmap,
				  AW36518_22282_REG_LED_TORCH_BRIGHTNESS, 0xff, 0x00);
	if (rval < 0)
		return rval;
	rval = regmap_update_bits(flash->regmap,
				  AW36518_22282_REG_LED_FLASH_BRIGHTNESS, 0xff, 0x00);
	if (rval < 0)
		return rval;

	/* reset faults */
	rval = regmap_read(flash->regmap, AW36518_22282_REG_FLAG1, &reg_val);
	return rval;
}

/* flashlight uninit */
static int aw36518_22282_uninit(struct aw36518_22282_flash *flash)
{
	aw36518_22282_pinctrl_set(flash,
			AW36518_22282_PINCTRL_PIN_HWEN, AW36518_22282_PINCTRL_PINSTATE_LOW);

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw36518_22282_timer;
static unsigned int aw36518_22282_timeout_ms[AW36518_22282_CHANNEL_NUM];

static void aw36518_22282_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	aw36518_22282_uninit(aw36518_22282_flash_data);
}

static enum hrtimer_restart aw36518_22282_timer_func(struct hrtimer *timer)
{
	schedule_work(&aw36518_22282_work);
	return HRTIMER_NORESTART;
}

static int aw36518_22282_flash_open(void)
{
	pr_info("aw36518_22282_flash_test open\n");
	return 0;
}

static int aw36518_22282_flash_release(void)
{
	pr_info("aw36518_22282_flash_test release\n");
	return 0;
}

/* legacy flashlight ops */
static int aw36518_22282_is_torch(int level)
{
	if (level >= AW36518_22282_LEVEL_TORCH)
		return -1;

	return 0;
}

static int aw36518_22282_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= AW36518_22282_LEVEL_NUM)
		level = AW36518_22282_LEVEL_NUM - 1;

	return level;
}

static int aw36518_22282_set_level(int channel, int level)
{
	aw36518_22282_flash_data->level = aw36518_22282_verify_level(level);
	if (!aw36518_22282_is_torch(aw36518_22282_flash_data->level)) {
		aw36518_22282_torch_brt_ctrl(aw36518_22282_flash_data, channel,
				1000 * aw36518_22282_current[aw36518_22282_flash_data->level]);
	} else {
		aw36518_22282_flash_brt_ctrl(aw36518_22282_flash_data, channel,
				1000 * aw36518_22282_current[aw36518_22282_flash_data->level]);
	}
	return 0;
}

static int aw36518_22282_operate(int channel, int enable)
{
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	if (enable && (aw36518_22282_flash_data->level >= 0)) {
		/* set timer */
		if (aw36518_22282_timeout_ms[channel]) {
			s = aw36518_22282_timeout_ms[channel] / 1000;
			ns = aw36518_22282_timeout_ms[channel] % 1000 * 1000000;
			ktime = ktime_set(s, ns);
			hrtimer_start(&aw36518_22282_timer, ktime,
					HRTIMER_MODE_REL);
		}

		if (!aw36518_22282_is_torch(aw36518_22282_flash_data->level)) {
			aw36518_22282_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
		} else {
			aw36518_22282_flash_data->led_mode = V4L2_FLASH_LED_MODE_FLASH;
		}
		aw36518_22282_mode_ctrl(aw36518_22282_flash_data);
		aw36518_22282_enable_ctrl(aw36518_22282_flash_data, channel, true);
	} else {
		aw36518_22282_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
		aw36518_22282_mode_ctrl(aw36518_22282_flash_data);
		aw36518_22282_enable_ctrl(aw36518_22282_flash_data, channel, false);
		hrtimer_cancel(&aw36518_22282_timer);
	}
	return 0;
}
/* legacy flashlight ops end */

static int aw36518_22282_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36518_22282_operate(channel, (int)fl_arg->arg);
		break;

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36518_22282_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36518_22282_set_level(channel, (int)fl_arg->arg);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = AW36518_22282_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = AW36518_22282_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = aw36518_22282_current[(int)fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = AW36518_22282_FLASH_TOUT_DEFAULT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw36518_22282_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	//mutex_lock(&aw36518_22282_mutex);
	if (set) {
		if (!use_count)
			ret = aw36518_22282_init(aw36518_22282_flash_data);
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = aw36518_22282_uninit(aw36518_22282_flash_data);
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	//mutex_unlock(&aw36518_22282_mutex);

	return 0;
}

static ssize_t aw36518_22282_strobe_store(struct flashlight_arg arg)
{
	pr_debug("aw36518_22282_strobe_store start (%d, %d, %d), (%d, %d)\n",
			arg.type, arg.ct, arg.part,
			arg.level, arg.dur);
	aw36518_22282_set_driver(1);
	aw36518_22282_set_level(arg.channel, arg.level);
	aw36518_22282_timeout_ms[arg.channel] = 0;
	aw36518_22282_operate(arg.channel, 1);

	msleep(arg.dur);

	aw36518_22282_operate(arg.channel, 0);
	aw36518_22282_set_driver(0);
	pr_debug("aw36518_22282_strobe_store end\n");
	return 0;
}

static struct flashlight_operations aw36518_22282_flash_ops = {
	aw36518_22282_flash_open,
	aw36518_22282_flash_release,
	aw36518_22282_ioctl,
	aw36518_22282_strobe_store,
	aw36518_22282_set_driver
};

static int aw36518_22282_parse_dt(struct aw36518_22282_flash *flash)
{
	struct device_node *np, *cnp;
	struct device *dev = flash->dev;
	u32 decouple = 1;
	int i = 0;

	if (!dev || !dev->of_node)
		return -ENODEV;

	np = dev->of_node;
	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type",
					&flash->flash_dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"ct", &flash->flash_dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"part", &flash->flash_dev_id[i].part))
			goto err_node_put;
		snprintf(flash->flash_dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				flash->subdev_led[i].name);
		flash->flash_dev_id[i].channel = i;
		flash->flash_dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				flash->flash_dev_id[i].type,
				flash->flash_dev_id[i].ct,
				flash->flash_dev_id[i].part,
				flash->flash_dev_id[i].name,
				flash->flash_dev_id[i].channel,
				flash->flash_dev_id[i].decouple);
		if (flashlight_dev_register_by_device_id(&flash->flash_dev_id[i],
			&aw36518_22282_flash_ops))
			return -EFAULT;
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int aw36518_22282_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct aw36518_22282_flash *flash;
	struct aw36518_22282_platform_data *pdata = dev_get_platdata(&client->dev);
	int rval;

	pr_info("%s:%d", __func__, __LINE__);

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	flash->regmap = devm_regmap_init_i2c(client, &aw36518_22282_regmap);
	if (IS_ERR(flash->regmap)) {
		rval = PTR_ERR(flash->regmap);
		return rval;
	}

	/* if there is no platform data, use chip default value */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		pdata->max_flash_timeout = AW36518_22282_FLASH_TOUT_MAX;
		/* led 1 */
		pdata->max_flash_brt[AW36518_22282_LED0] = AW36518_22282_FLASH_BRT_MAX;
		pdata->max_torch_brt[AW36518_22282_LED0] = AW36518_22282_TORCH_BRT_MAX;
	}
	flash->pdata = pdata;
	flash->dev = &client->dev;
	mutex_init(&flash->lock);
	aw36518_22282_flash_data = flash;

	rval = aw36518_22282_pinctrl_init(flash);
	if (rval < 0)
		return rval;

	rval = aw36518_22282_subdev_init(flash, AW36518_22282_LED0, "aw36518_22282-led0");
	if (rval < 0)
		return rval;

	/* init work queue */
	INIT_WORK(&aw36518_22282_work, aw36518_22282_work_disable);

	/* init timer */
	hrtimer_init(&aw36518_22282_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36518_22282_timer.function = aw36518_22282_timer_func;
	aw36518_22282_timeout_ms[AW36518_22282_CHANNEL_CH1] = AW36518_22282_FLASH_TOUT_DEFAULT;

	pm_runtime_enable(flash->dev);

	rval = aw36518_22282_parse_dt(flash);

	i2c_set_clientdata(client, flash);

	pr_info("%s:%d", __func__, __LINE__);
	return 0;
}

static int aw36518_22282_remove(struct i2c_client *client)
{
	struct aw36518_22282_flash *flash = i2c_get_clientdata(client);
	unsigned int i;

	for (i = AW36518_22282_LED0; i < AW36518_22282_LED_MAX; i++) {
		v4l2_device_unregister_subdev(&flash->subdev_led[i]);
		v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
		media_entity_cleanup(&flash->subdev_led[i].entity);
	}

	pm_runtime_disable(&client->dev);

	/* flush work queue */
	flush_work(&aw36518_22282_work);

	pm_runtime_set_suspended(&client->dev);
	return 0;
}

static int __maybe_unused aw36518_22282_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw36518_22282_flash *flash = i2c_get_clientdata(client);

	pr_info("%s %d", __func__, __LINE__);

	return aw36518_22282_uninit(flash);
}

static int __maybe_unused aw36518_22282_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw36518_22282_flash *flash = i2c_get_clientdata(client);

	pr_info("%s %d", __func__, __LINE__);

	return aw36518_22282_init(flash);
}

static const struct i2c_device_id aw36518_22282_id_table[] = {
	{AW36518_22282_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw36518_22282_id_table);

static const struct of_device_id aw36518_22282_of_table[] = {
	{ .compatible = "mediatek,aw36518_22282" },
	{ },
};
MODULE_DEVICE_TABLE(of, aw36518_22282_of_table);

static const struct dev_pm_ops aw36518_22282_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(aw36518_22282_suspend, aw36518_22282_resume, NULL)
};

static struct i2c_driver aw36518_22282_i2c_driver = {
	.driver = {
		   .name = AW36518_22282_NAME,
		   .pm = &aw36518_22282_pm_ops,
		   .of_match_table = aw36518_22282_of_table,
		   },
	.probe = aw36518_22282_probe,
	.remove = aw36518_22282_remove,
	.id_table = aw36518_22282_id_table,
};

module_i2c_driver(aw36518_22282_i2c_driver);

MODULE_AUTHOR("Roger-HY Wang <roger-hy.wang@mediatek.com>");
MODULE_DESCRIPTION("AW36518_22282 LED flash driver");
MODULE_LICENSE("GPL");
