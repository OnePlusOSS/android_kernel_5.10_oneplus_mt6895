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

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
#include "flashlight-core.h"

#include <linux/power_supply.h>
#endif

#define LM3642_NAME	"lm3642_22823"
#define LM3642_I2C_ADDR	(0x63)

/* registers definitions */
#define REG_VERSION         0x00
#define REG_ENABLE		    0x0A
#define REG_LED0_FLASH_BR	0x09
#define REG_LED1_FLASH_BR	0x09
#define REG_LED0_TORCH_BR	0x09
#define REG_LED1_TORCH_BR	0x09
#define REG_FLASH_TOUT		0x08
#define REG_FLAG1		    0x0B

/* fault mask */
#define FAULT_TIMEOUT	(1<<0)
#define FAULT_THERMAL_SHUTDOWN	(1<<1)
#define FAULT_LEDX_SHORT_CIRCUIT	(1<<2)
#define FAULT_OVER_VOLTAGE	(1<<3)

/*  FLASH Brightness
 *	min 93750uA, step 93000uA, max 1500000uA
 */
#define LM3642_FLASH_BRT_MIN 93000
#define LM3642_FLASH_BRT_STEP 93000
#define LM3642_FLASH_BRT_MAX 1500000
#define LM3642_FLASH_BRT_uA_TO_REG(a)	\
	((a) < LM3642_FLASH_BRT_MIN ? 0 :	\
	 (((a) - LM3642_FLASH_BRT_MIN) / LM3642_FLASH_BRT_STEP))

/*  FLASH TIMEOUT DURATION
 *	min 32ms, step 32ms, max 1024ms
 */
#define LM3642_FLASH_TOUT_MIN 100
#define LM3642_FLASH_TOUT_STEP 100
#define LM3642_FLASH_TOUT_MAX 800

/*  TORCH BRT
 *	min 93000uA, step 93000uA, max 187500uA
 */
#define LM3642_TORCH_BRT_MIN 93000
#define LM3642_TORCH_BRT_STEP 93000
#define LM3642_TORCH_BRT_MAX 187500
#define LM3642_TORCH_BRT_uA_TO_REG(a)	\
	((a) < LM3642_TORCH_BRT_MIN ? 0 :	\
	((((a) - LM3642_TORCH_BRT_MIN)*2) / LM3642_TORCH_BRT_STEP + 1))

/* define mutex and work queue */
static DEFINE_MUTEX(lm3642_mutex);

enum lm3642_led_id {
	LM3642_LED0 = 0,
	LM3642_LED1,
	LM3642_LED_MAX
};

/* struct lm3642_platform_data
 *
 * @max_flash_timeout: flash timeout
 * @max_flash_brt: flash mode led brightness
 * @max_torch_brt: torch mode led brightness
 */
struct lm3642_platform_data {
	u32 max_flash_timeout;
	u32 max_flash_brt[LM3642_LED_MAX];
	u32 max_torch_brt[LM3642_LED_MAX];
};

enum led_enable {
	MODE_SHDN = 0x0,
	MODE_TORCH = 0x02,
	MODE_FLASH = 0x03,
};

/**
 * struct lm3642_flash
 *
 * @dev: pointer to &struct device
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: V4L2 LED mode
 * @ctrls_led: V4L2 controls
 * @subdev_led: V4L2 subdev
 */
struct lm3642_flash {
	struct device *dev;
	struct lm3642_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	enum v4l2_flash_led_mode led_mode;
	struct v4l2_ctrl_handler ctrls_led[LM3642_LED_MAX];
	struct v4l2_subdev subdev_led[LM3642_LED_MAX];
	struct device_node *dnode[LM3642_LED_MAX];
	struct pinctrl *lm3642_hwen_pinctrl;
	struct pinctrl_state *lm3642_hwen_high;
	struct pinctrl_state *lm3642_hwen_low;
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
	struct flashlight_device_id flash_dev_id[LM3642_LED_MAX];
#endif
	struct thermal_cooling_device *cdev;
	int need_cooler;
	unsigned long max_state;
	unsigned long target_state;
	unsigned long target_current[LM3642_LED_MAX];
	unsigned long ori_current[LM3642_LED_MAX];
};

/* define usage count */
static int use_count = 0;

static struct lm3642_flash *lm3642_flash_data;

#define to_lm3642_flash(_ctrl, _no)	\
	container_of(_ctrl->handler, struct lm3642_flash, ctrls_led[_no])

static int lm3642_set_driver(int set);

/* define pinctrl */
#define LM3642_PINCTRL_PIN_HWEN 0
#define LM3642_PINCTRL_PINSTATE_LOW 0
#define LM3642_PINCTRL_PINSTATE_HIGH 1
#define LM3642_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define LM3642_PINCTRL_STATE_HWEN_LOW  "hwen_low"
/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int lm3642_pinctrl_init(struct lm3642_flash *flash)
{
	int ret = 0;

	/* get pinctrl */
	flash->lm3642_hwen_pinctrl = devm_pinctrl_get(flash->dev);
	if (IS_ERR(flash->lm3642_hwen_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(flash->lm3642_hwen_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	flash->lm3642_hwen_high = pinctrl_lookup_state(
			flash->lm3642_hwen_pinctrl,
			LM3642_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(flash->lm3642_hwen_high)) {
		pr_info("Failed to init (%s)\n",
			LM3642_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(flash->lm3642_hwen_high);
	}
	flash->lm3642_hwen_low = pinctrl_lookup_state(
			flash->lm3642_hwen_pinctrl,
			LM3642_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(flash->lm3642_hwen_low)) {
		pr_info("Failed to init (%s)\n", LM3642_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(flash->lm3642_hwen_low);
	}

	return ret;
}

static int lm3642_pinctrl_set(struct lm3642_flash *flash, int pin, int state)
{
	int ret = 0;

	if (IS_ERR(flash->lm3642_hwen_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case LM3642_PINCTRL_PIN_HWEN:
		if (state == LM3642_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(flash->lm3642_hwen_low))
			pinctrl_select_state(flash->lm3642_hwen_pinctrl,
					flash->lm3642_hwen_low);
		else if (state == LM3642_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(flash->lm3642_hwen_high))
			pinctrl_select_state(flash->lm3642_hwen_pinctrl,
					flash->lm3642_hwen_high);
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

/* enable mode control */
static int lm3642_mode_ctrl(struct lm3642_flash *flash)
{
	int rval = -EINVAL;

	pr_info_ratelimited("%s mode:%d", __func__, flash->led_mode);
	switch (flash->led_mode) {
	case V4L2_FLASH_LED_MODE_NONE:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x03, MODE_SHDN);
		break;
	case V4L2_FLASH_LED_MODE_TORCH:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x03, MODE_TORCH);
		break;
	case V4L2_FLASH_LED_MODE_FLASH:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x03, MODE_FLASH);
		break;
	}
	return rval;
}

/* led1/2 enable/disable */
static int lm3642_enable_ctrl(struct lm3642_flash *flash,
			      enum lm3642_led_id led_no, bool on)
{
	int rval = 0;

	if (led_no < 0 || led_no >= LM3642_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}
	pr_info_ratelimited("%s led:%d enable:%d", __func__, led_no, on);

    if (flash->led_mode == V4L2_FLASH_LED_MODE_TORCH) {
        if (on)
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x03, 0x02);
		else
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x03, 0x00);
    } else if(flash->led_mode == V4L2_FLASH_LED_MODE_FLASH) {
        if (on)
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x03, 0x03);
		else
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x03, 0x00);
    }
	pr_info("%s: return val:%d", __func__,  rval);
	return rval;
}

static int lm3642_select_led(struct lm3642_flash *flash,
										int led_num)
{
	int rval;
	pr_info("engineer cam select led->%d", led_num);
	rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x03, 0x03);
	return rval;
}

/* torch1/2 brightness control */
static int lm3642_torch_brt_ctrl(struct lm3642_flash *flash,
				 enum lm3642_led_id led_no, unsigned int brt)
{
	int rval = -EINVAL;
	u8 br_bits;

	if (led_no < 0 || led_no >= LM3642_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}
	pr_info_ratelimited("%s ledNo:%d, brt:%u\n", __func__, led_no, brt);
	if (brt < LM3642_TORCH_BRT_MIN)
		return lm3642_enable_ctrl(flash, led_no, false);

	br_bits = LM3642_TORCH_BRT_uA_TO_REG(brt);
	br_bits <<= 4;
	pr_info_ratelimited("%s current:%duA, reg:0x%x\n", __func__, brt, br_bits);
	if (led_no == LM3642_LED0)
		rval = regmap_update_bits(flash->regmap,
					  REG_LED0_TORCH_BR, 0x70, br_bits);
	else
		rval = regmap_update_bits(flash->regmap,
					  REG_LED1_TORCH_BR, 0x70, br_bits);

	return rval;
}

/* flash1/2 brightness control */
static int lm3642_flash_brt_ctrl(struct lm3642_flash *flash,
				 enum lm3642_led_id led_no, unsigned int brt)
{
	int rval;
	u8 br_bits;

	if (led_no < 0 || led_no >= LM3642_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}
	pr_info("%s %d brt:%u", __func__, led_no, brt);
	if (brt < LM3642_FLASH_BRT_MIN)
		return lm3642_enable_ctrl(flash, led_no, false);

	br_bits = LM3642_FLASH_BRT_uA_TO_REG(brt);
	pr_info_ratelimited("%s current:%duA, reg:0x%x\n", __func__, brt, br_bits);
	if (led_no == LM3642_LED0)
		rval = regmap_update_bits(flash->regmap,
					  REG_LED0_FLASH_BR, 0x0F, br_bits);
	else
		rval = regmap_update_bits(flash->regmap,
					  REG_LED1_FLASH_BR, 0x0F, br_bits);

	return rval;
}

/* flash1/2 timeout control */
static int lm3642_flash_tout_ctrl(struct lm3642_flash *flash,
				unsigned int tout)
{
	int rval;
	u8 tout_bits;

	pr_info("%s tout:%u", __func__, tout);

	tout_bits = (tout / LM3642_FLASH_TOUT_STEP)-1;

	rval = regmap_update_bits(flash->regmap,
				  REG_FLASH_TOUT, 0x07, tout_bits);

	return rval;
}

/* v4l2 controls  */
static int lm3642_get_ctrl(struct v4l2_ctrl *ctrl, enum lm3642_led_id led_no)
{
	struct lm3642_flash *flash = to_lm3642_flash(ctrl, led_no);
	int rval = -EINVAL;

	mutex_lock(&flash->lock);

	if (ctrl->id == V4L2_CID_FLASH_FAULT) {
		s32 fault = 0;
		unsigned int reg_val = 0;

		rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
		if (rval < 0)
			goto out;
		if (reg_val & FAULT_LEDX_SHORT_CIRCUIT)
			fault |= V4L2_FLASH_FAULT_SHORT_CIRCUIT;
		if (reg_val & FAULT_THERMAL_SHUTDOWN)
			fault |= V4L2_FLASH_FAULT_OVER_TEMPERATURE;
		if (reg_val & FAULT_TIMEOUT)
			fault |= V4L2_FLASH_FAULT_TIMEOUT;
        if(reg_val & FAULT_OVER_VOLTAGE)
            fault |= V4L2_FLASH_FAULT_OVER_VOLTAGE;
		ctrl->cur.val = fault;
	}

out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int lm3642_set_ctrl(struct v4l2_ctrl *ctrl, enum lm3642_led_id led_no)
{
	struct lm3642_flash *flash = to_lm3642_flash(ctrl, led_no);
	int rval = -EINVAL;
    unsigned int boost_reg_val;

    rval = regmap_read(flash->regmap, REG_FLAG1, &boost_reg_val);
	pr_info("%s ledNo:%d CtrlID:0x%x, boost_reg_val:0x%x, ctrlVal:%d", __func__, led_no, ctrl->id, boost_reg_val, ctrl->val);
	mutex_lock(&flash->lock);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		flash->led_mode = ctrl->val;
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH)
			rval = lm3642_mode_ctrl(flash);
		else
			rval = 0;
		if (flash->led_mode == V4L2_FLASH_LED_MODE_NONE)
			lm3642_enable_ctrl(flash, led_no, false);
		else if (flash->led_mode == V4L2_FLASH_LED_MODE_TORCH)
			rval = lm3642_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_SOURCE:
	    rval = regmap_update_bits(flash->regmap,
					REG_ENABLE, 0x20, (ctrl->val) << 5);
		if (rval < 0)
			goto err_out;
		break;

	case V4L2_CID_FLASH_STROBE:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		flash->led_mode = V4L2_FLASH_LED_MODE_FLASH;
		rval = lm3642_mode_ctrl(flash);
		rval = lm3642_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		lm3642_enable_ctrl(flash, led_no, false);
		flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
		rval = lm3642_mode_ctrl(flash);
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		rval = lm3642_flash_tout_ctrl(flash, ctrl->val);
		break;

	case V4L2_CID_FLASH_INTENSITY:
		rval = lm3642_flash_brt_ctrl(flash, led_no, ctrl->val);
		break;

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		rval = lm3642_torch_brt_ctrl(flash, led_no, ctrl->val);
		break;
	default:
	    break;
	}

err_out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int lm3642_led1_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return lm3642_get_ctrl(ctrl, LM3642_LED1);
}

static int lm3642_led1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return lm3642_set_ctrl(ctrl, LM3642_LED1);
}

static int lm3642_led0_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return lm3642_get_ctrl(ctrl, LM3642_LED0);
}

static int lm3642_led0_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return lm3642_set_ctrl(ctrl, LM3642_LED0);
}

static const struct v4l2_ctrl_ops lm3642_led_ctrl_ops[LM3642_LED_MAX] = {
	[LM3642_LED0] = {
			.g_volatile_ctrl = lm3642_led0_get_ctrl,
			.s_ctrl = lm3642_led0_set_ctrl,
			},
	[LM3642_LED1] = {
			.g_volatile_ctrl = lm3642_led1_get_ctrl,
			.s_ctrl = lm3642_led1_set_ctrl,
			}
};

static int lm3642_init_controls(struct lm3642_flash *flash,
				enum lm3642_led_id led_no)
{
	struct v4l2_ctrl *fault;
	u32 max_flash_brt = flash->pdata->max_flash_brt[led_no];
	u32 max_torch_brt = flash->pdata->max_torch_brt[led_no];
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led[led_no];
	const struct v4l2_ctrl_ops *ops = &lm3642_led_ctrl_ops[led_no];

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
			  LM3642_FLASH_TOUT_MIN,
			  flash->pdata->max_flash_timeout,
			  LM3642_FLASH_TOUT_STEP,
			  flash->pdata->max_flash_timeout);

	/* flash brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  LM3642_FLASH_BRT_MIN, max_flash_brt,
			  LM3642_FLASH_BRT_STEP, max_flash_brt);

	/* torch brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  LM3642_TORCH_BRT_MIN, max_torch_brt,
			  LM3642_TORCH_BRT_STEP, max_torch_brt);

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

	if (led_no < 0 || led_no >= LM3642_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}

	flash->subdev_led[led_no].ctrl_handler = hdl;
	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops lm3642_ops = {
	.core = NULL,
};

static const struct regmap_config lm3642_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static void lm3642_v4l2_i2c_subdev_init(struct v4l2_subdev *sd,
		struct i2c_client *client,
		const struct v4l2_subdev_ops *ops)
{
	int ret = 0;

	v4l2_subdev_init(sd, ops);
	sd->flags |= V4L2_SUBDEV_FL_IS_I2C;
	/* the owner is the same as the i2c_client's driver owner */
	sd->owner = client->dev.driver->owner;
	sd->dev = &client->dev;
	/* i2c_client and v4l2_subdev point to one another */
	v4l2_set_subdevdata(sd, client);
	i2c_set_clientdata(client, sd);
	/* initialize name */
	ret = snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		client->dev.driver->name, i2c_adapter_id(client->adapter),
		client->addr);
	if (ret < 0)
		pr_info("snprintf failed\n");
}

static int lm3642_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
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

static int lm3642_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pr_info("%s\n", __func__);

	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops lm3642_int_ops = {
	.open = lm3642_open,
	.close = lm3642_close,
};

static int lm3642_subdev_init(struct lm3642_flash *flash,
			      enum lm3642_led_id led_no, char *led_name)
{
	struct i2c_client *client = to_i2c_client(flash->dev);
	struct device_node *np = flash->dev->of_node, *child;
	const char *fled_name = "flash";
	int rval;

	// pr_info("%s %d", __func__, led_no);
	if (led_no < 0 || led_no >= LM3642_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}

	lm3642_v4l2_i2c_subdev_init(&flash->subdev_led[led_no],
				client, &lm3642_ops);
	flash->subdev_led[led_no].flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->subdev_led[led_no].internal_ops = &lm3642_int_ops;
	strscpy(flash->subdev_led[led_no].name, led_name,
		sizeof(flash->subdev_led[led_no].name));

	for (child = of_get_child_by_name(np, fled_name); child;
			child = of_find_node_by_name(child, fled_name)) {
		int rv;
		u32 reg = 0;

		rv = of_property_read_u32(child, "reg", &reg);
		if (rv)
			continue;

		if (reg == led_no) {
			flash->dnode[led_no] = child;
			flash->subdev_led[led_no].fwnode =
				of_fwnode_handle(flash->dnode[led_no]);
		}
	}

	rval = lm3642_init_controls(flash, led_no);
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
static int lm3642_init(struct lm3642_flash *flash)
{
	int rval = 0;
	unsigned int reg_val;
	/* lm3642 revision */
	int is_lm3642lt_ver;

	lm3642_pinctrl_set(flash, LM3642_PINCTRL_PIN_HWEN, LM3642_PINCTRL_PINSTATE_HIGH);

	/* set timeout */
	rval = lm3642_flash_tout_ctrl(flash, 400);
	if (rval < 0)
		return rval;

	/* output disable */
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
	rval = lm3642_mode_ctrl(flash);

	rval = regmap_update_bits(flash->regmap,
				  REG_LED0_TORCH_BR, 0x7F, 0x00);
	if (rval < 0)
		return rval;
    /*get version*/
	rval = regmap_read(flash->regmap, REG_VERSION, &is_lm3642lt_ver);
    if(!rval){
		pr_info("flash driver IC version:0x%x.\n", is_lm3642lt_ver);
    }else{
		pr_info("get flash driver IC version fail.\n");
    }
	/* reset faults */
	rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
    if(!rval){
		pr_info("get flash driver IC Reg flags(IC Status):0x%x.\n", reg_val);
    }
	return rval;
}

/* flashlight uninit */
static int lm3642_uninit(struct lm3642_flash *flash)
{
	lm3642_pinctrl_set(flash,
			LM3642_PINCTRL_PIN_HWEN, LM3642_PINCTRL_PINSTATE_LOW);

	return 0;
}

static int lm3642_flash_open(void)
{
	return 0;
}

static int lm3642_flash_release(void)
{
	return 0;
}

static int lm3642_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	int ret;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_ONOFF:
		pr_info_ratelimited("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if ((int)fl_arg->arg) {
			lm3642_torch_brt_ctrl(lm3642_flash_data, channel, 25000);
			lm3642_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
			lm3642_mode_ctrl(lm3642_flash_data);
			lm3642_enable_ctrl(lm3642_flash_data, channel, true);
		} else {
			if (lm3642_flash_data->led_mode != V4L2_FLASH_LED_MODE_NONE) {
				lm3642_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
				lm3642_mode_ctrl(lm3642_flash_data);
				lm3642_enable_ctrl(lm3642_flash_data, channel, false);
			}
		}
		break;
	case OPLUS_FLASH_IOC_SELECT_LED_NUM:
		if (fl_arg->arg == LM3642_LED0 || fl_arg->arg == LM3642_LED1) {
			lm3642_flash_data->led_mode = V4L2_FLASH_LED_MODE_FLASH;
			 ret = lm3642_select_led(lm3642_flash_data, fl_arg->arg);
			if (ret < 0) {
				pr_err("engineer cam set led[%d] fail", fl_arg->arg);
				return ret;
			}
		} else {
			lm3642_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
			lm3642_mode_ctrl(lm3642_flash_data);
			lm3642_enable_ctrl(lm3642_flash_data, channel, false);
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int lm3642_set_driver(int set)
{
	int ret = 0;

    pr_debug("Enter into %s, set is:%d use_count:%d\n", __func__, set, use_count);
	/* set chip and usage count */
	//mutex_lock(&lm3642_mutex);
	if (set) {
		if (!use_count)
			ret = lm3642_init(lm3642_flash_data);
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = lm3642_uninit(lm3642_flash_data);
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	//mutex_unlock(&lm3642_mutex);

	return 0;
}

static ssize_t lm3642_strobe_store(struct flashlight_arg arg)
{
	lm3642_set_driver(1);
	//lm3642_set_level(arg.channel, arg.level);
	//lm3642_timeout_ms[arg.channel] = 0;
	//lm3642_enable(arg.channel);
	lm3642_torch_brt_ctrl(lm3642_flash_data, arg.channel,
				arg.level);
	lm3642_enable_ctrl(lm3642_flash_data, arg.channel, true);
	lm3642_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
	lm3642_mode_ctrl(lm3642_flash_data);
	msleep(arg.dur);
	//lm3642_disable(arg.channel);
	lm3642_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
	lm3642_mode_ctrl(lm3642_flash_data);
	lm3642_enable_ctrl(lm3642_flash_data, arg.channel, false);
	lm3642_set_driver(0);
	return 0;
}

static struct flashlight_operations lm3642_flash_ops = {
	lm3642_flash_open,
	lm3642_flash_release,
	lm3642_ioctl,
	lm3642_strobe_store,
	lm3642_set_driver
};

static int lm3642_parse_dt(struct lm3642_flash *flash)
{
	struct device_node *np, *cnp;
	struct device *dev = flash->dev;
	u32 decouple = 0;
	int i = 0, ret = 0;

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
		ret = snprintf(flash->flash_dev_id[i].name,
				FLASHLIGHT_NAME_SIZE,
				flash->subdev_led[i].name);
		if (ret < 0)
			pr_info("snprintf failed\n");
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
			&lm3642_flash_ops))
			return -EFAULT;
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int lm3642_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct lm3642_flash *flash;
	struct lm3642_platform_data *pdata = dev_get_platdata(&client->dev);
	int rval;

	pr_info("%s:%d", __func__, __LINE__);

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	flash->regmap = devm_regmap_init_i2c(client, &lm3642_regmap);
	if (IS_ERR(flash->regmap)) {
		rval = PTR_ERR(flash->regmap);
		return rval;
	}

	/* if there is no platform data, use chip default value */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		pdata->max_flash_timeout = LM3642_FLASH_TOUT_MAX;
		/* led 1 */
		pdata->max_flash_brt[LM3642_LED0] = LM3642_FLASH_BRT_MAX;
		pdata->max_torch_brt[LM3642_LED0] = LM3642_TORCH_BRT_MAX;
		/* led 2 */
		pdata->max_flash_brt[LM3642_LED1] = LM3642_FLASH_BRT_MAX;
		pdata->max_torch_brt[LM3642_LED1] = LM3642_TORCH_BRT_MAX;
	}
	flash->pdata = pdata;
	flash->dev = &client->dev;
	mutex_init(&flash->lock);
	lm3642_flash_data = flash;

	rval = lm3642_pinctrl_init(flash);
	if (rval < 0)
		return rval;

	rval = lm3642_subdev_init(flash, LM3642_LED0, "lm3642_22823-led0");
	if (rval < 0)
		return rval;

	rval = lm3642_subdev_init(flash, LM3642_LED1, "lm3642_22823-led1");
	if (rval < 0)
		return rval;

	pm_runtime_enable(flash->dev);
	rval = lm3642_parse_dt(flash);

	i2c_set_clientdata(client, flash);

	pr_info("%s:%d", __func__, __LINE__);
	return 0;
}

static int lm3642_remove(struct i2c_client *client)
{
	struct lm3642_flash *flash = i2c_get_clientdata(client);
	unsigned int i;

	for (i = LM3642_LED0; i < LM3642_LED_MAX; i++) {
		v4l2_device_unregister_subdev(&flash->subdev_led[i]);
		v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
		media_entity_cleanup(&flash->subdev_led[i].entity);
	}

	pm_runtime_disable(&client->dev);

	pm_runtime_set_suspended(&client->dev);
	return 0;
}

static int __maybe_unused lm3642_suspend(struct device *dev)
{
	pr_info("%s %d", __func__, __LINE__);

	return 0;
}

static int __maybe_unused lm3642_resume(struct device *dev)
{
	pr_info("%s %d", __func__, __LINE__);

	return 0;
}

static const struct i2c_device_id lm3642_id_table[] = {
	{LM3642_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3642_id_table);

static const struct of_device_id lm3642_of_table[] = {
	{ .compatible = "mediatek,lm3642_22823" },
	{ },
};
MODULE_DEVICE_TABLE(of, lm3642_of_table);

static const struct dev_pm_ops lm3642_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(lm3642_suspend, lm3642_resume, NULL)
};
static struct i2c_driver lm3642_i2c_driver = {
	.driver = {
		   .name = LM3642_NAME,
           .pm = &lm3642_pm_ops,
		   .of_match_table = lm3642_of_table,
		   },
	.probe = lm3642_probe,
	.remove = lm3642_remove,
	.id_table = lm3642_id_table,
};

module_i2c_driver(lm3642_i2c_driver);

MODULE_AUTHOR("Roger-HY Wang <roger-hy.wang@mediatek.com>");
MODULE_DESCRIPTION("Texas Instruments LM3642 LED flash driver");
MODULE_LICENSE("GPL");
