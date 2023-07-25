// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DRIVER_NAME "ak7314af"
#define AK7314AF_I2C_SLAVE_ADDR 0x18

#define LOG_INF(format, args...)                                               \
	pr_debug(DRIVER_NAME " [%s] " format, __func__, ##args)

#define AK7314AF_NAME				"ak7314af"
#define AK7314AF_MAX_FOCUS_POS			1023
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define AK7314AF_FOCUS_STEPS			1
#define AK7314AF_SET_POSITION_ADDR		0x00

#define AK7314AF_CMD_DELAY			0xff
#define AK7314AF_CTRL_DELAY_US			10000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define AK7314AF_MOVE_STEPS			16
#define AK7314AF_MOVE_DELAY_US			1000
#define AK7314AF_STABLE_TIME_US			1000
#define AK7314AF_ACTIVE_CODE			105

/* ak7314af device structure */
struct ak7314af_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
};

static inline struct ak7314af_device *to_ak7314af_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct ak7314af_device, ctrls);
}

static inline struct ak7314af_device *sd_to_ak7314af_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ak7314af_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};


static int ak7314af_set_position(struct ak7314af_device *ak7314af, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7314af->sd);
	int retry = 3;
	int ret;

	while (--retry > 0) {
		ret = i2c_smbus_write_word_data(client, AK7314AF_SET_POSITION_ADDR,
					 swab16(val << 6));
		if (ret < 0) {
			usleep_range(AK7314AF_MOVE_DELAY_US,
				     AK7314AF_MOVE_DELAY_US + 1000);
			#ifdef OPLUS_FEATURE_CAMERA_COMMON
			pr_info("ak7314af Set postition:%u fail.", val);
			#endif
		} else {
			break;
		}
	}
	return ret;
}

static int ak7314af_release(struct ak7314af_device *ak7314af)
{
	int ret, val;
	struct i2c_client *client = v4l2_get_subdevdata(&ak7314af->sd);

	for (val = round_down(ak7314af->focus->val, AK7314AF_MOVE_STEPS);
			val >= 16; val = (val/2)) {
		ret = ak7314af_set_position(ak7314af, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(AK7314AF_MOVE_DELAY_US,
				AK7314AF_MOVE_DELAY_US + 1000);
	}
	i2c_smbus_write_byte_data(client, 0x02, 0x20);
	msleep(10);

	/*
	 * Wait for the motor to stabilize after the last movement
	 * to prevent the motor from shaking.
	 */
	usleep_range(AK7314AF_STABLE_TIME_US - AK7314AF_MOVE_DELAY_US,
			AK7314AF_STABLE_TIME_US - AK7314AF_MOVE_DELAY_US + 1000);

	return 0;
}

static int ak7314af_init(struct ak7314af_device *ak7314af)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7314af->sd);
	int ret = 0, val = 0;
	static bool first_open = true;

	LOG_INF("+\n");

	client->addr = AK7314AF_I2C_SLAVE_ADDR >> 1;
	//ret = i2c_smbus_read_byte_data(client, 0x02);

	LOG_INF("Check HW version: %x\n", ret);

	ak7314af_set_position(ak7314af, AK7314AF_ACTIVE_CODE);

	/* 00:active mode , 10:Standby mode , x1:Sleep mode */
	ret = i2c_smbus_write_byte_data(client, 0x02, 0x00);

	val = ak7314af->focus->val;
	ak7314af_set_position(ak7314af, val);
	if (first_open)
	{
		val = AK7314AF_ACTIVE_CODE;
		ak7314af_set_position(ak7314af, val);
		first_open = false;
	}
	LOG_INF("-\n");

	return 0;
}

/* Power handling */
static int ak7314af_power_off(struct ak7314af_device *ak7314af)
{
	int ret;

	LOG_INF("%s\n", __func__);

	ret = ak7314af_release(ak7314af);
	if (ret)
		LOG_INF("ak7314af release failed!\n");

	ret = regulator_disable(ak7314af->vin);
	if (ret)
		return ret;

	ret = regulator_disable(ak7314af->vdd);
	if (ret)
		return ret;

	if (ak7314af->vcamaf_pinctrl && ak7314af->vcamaf_off)
		ret = pinctrl_select_state(ak7314af->vcamaf_pinctrl,
					ak7314af->vcamaf_off);

	return ret;
}

static int ak7314af_power_on(struct ak7314af_device *ak7314af)
{
	int ret;

	LOG_INF("%s\n", __func__);

	ret = regulator_enable(ak7314af->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(ak7314af->vdd);
	if (ret < 0)
		return ret;

	if (ak7314af->vcamaf_pinctrl && ak7314af->vcamaf_on)
		ret = pinctrl_select_state(ak7314af->vcamaf_pinctrl,
					ak7314af->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(AK7314AF_CTRL_DELAY_US, AK7314AF_CTRL_DELAY_US + 100);

	ret = ak7314af_init(ak7314af);
	if (ret < 0)
		goto fail;

	return 0;

fail:
	regulator_disable(ak7314af->vin);
	regulator_disable(ak7314af->vdd);
	if (ak7314af->vcamaf_pinctrl && ak7314af->vcamaf_off) {
		pinctrl_select_state(ak7314af->vcamaf_pinctrl,
				ak7314af->vcamaf_off);
	}

	return ret;
}

static int ak7314af_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct ak7314af_device *ak7314af = to_ak7314af_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = ak7314af_set_position(ak7314af, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops ak7314af_vcm_ctrl_ops = {
	.s_ctrl = ak7314af_set_ctrl,
};

static int ak7314af_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct ak7314af_device *ak7314af = sd_to_ak7314af_vcm(sd);

	LOG_INF("%s\n", __func__);

	ret = ak7314af_power_on(ak7314af);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int ak7314af_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ak7314af_device *ak7314af = sd_to_ak7314af_vcm(sd);

	LOG_INF("%s\n", __func__);

	ak7314af_power_off(ak7314af);

	return 0;
}

static const struct v4l2_subdev_internal_ops ak7314af_int_ops = {
	.open = ak7314af_open,
	.close = ak7314af_close,
};

static const struct v4l2_subdev_ops ak7314af_ops = { };

static void ak7314af_subdev_cleanup(struct ak7314af_device *ak7314af)
{
	v4l2_async_unregister_subdev(&ak7314af->sd);
	v4l2_ctrl_handler_free(&ak7314af->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&ak7314af->sd.entity);
#endif
}

static int ak7314af_init_controls(struct ak7314af_device *ak7314af)
{
	struct v4l2_ctrl_handler *hdl = &ak7314af->ctrls;
	const struct v4l2_ctrl_ops *ops = &ak7314af_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	ak7314af->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, AK7314AF_MAX_FOCUS_POS, AK7314AF_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	ak7314af->sd.ctrl_handler = hdl;

	return 0;
}

static int ak7314af_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ak7314af_device *ak7314af;
	int ret;

	LOG_INF("%s\n", __func__);

	ak7314af = devm_kzalloc(dev, sizeof(*ak7314af), GFP_KERNEL);
	if (!ak7314af)
		return -ENOMEM;

	ak7314af->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(ak7314af->vin)) {
		ret = PTR_ERR(ak7314af->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	ak7314af->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ak7314af->vdd)) {
		ret = PTR_ERR(ak7314af->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	ak7314af->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ak7314af->vcamaf_pinctrl)) {
		ret = PTR_ERR(ak7314af->vcamaf_pinctrl);
		ak7314af->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		ak7314af->vcamaf_on = pinctrl_lookup_state(
			ak7314af->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(ak7314af->vcamaf_on)) {
			ret = PTR_ERR(ak7314af->vcamaf_on);
			ak7314af->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		ak7314af->vcamaf_off = pinctrl_lookup_state(
			ak7314af->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(ak7314af->vcamaf_off)) {
			ret = PTR_ERR(ak7314af->vcamaf_off);
			ak7314af->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&ak7314af->sd, client, &ak7314af_ops);
	ak7314af->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ak7314af->sd.internal_ops = &ak7314af_int_ops;

	ret = ak7314af_init_controls(ak7314af);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&ak7314af->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	ak7314af->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&ak7314af->sd);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	ak7314af_subdev_cleanup(ak7314af);
	return ret;
}

static int ak7314af_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7314af_device *ak7314af = sd_to_ak7314af_vcm(sd);

	LOG_INF("%s\n", __func__);

	ak7314af_subdev_cleanup(ak7314af);

	return 0;
}

static const struct i2c_device_id ak7314af_id_table[] = {
	{ AK7314AF_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ak7314af_id_table);

static const struct of_device_id ak7314af_of_table[] = {
	{ .compatible = "mediatek,ak7314af" },
	{ },
};
MODULE_DEVICE_TABLE(of, ak7314af_of_table);

static struct i2c_driver ak7314af_i2c_driver = {
	.driver = {
		.name = AK7314AF_NAME,
		.of_match_table = ak7314af_of_table,
	},
	.probe_new  = ak7314af_probe,
	.remove = ak7314af_remove,
	.id_table = ak7314af_id_table,
};

module_i2c_driver(ak7314af_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("AK7314AF VCM driver");
MODULE_LICENSE("GPL v2");
