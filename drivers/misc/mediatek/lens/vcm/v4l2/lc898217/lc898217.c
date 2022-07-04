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

#define DRIVER_NAME "lc898217"
#define LC898217_I2C_SLAVE_ADDR 0xE4

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define LC898217_NAME				"lc898217"
#define LC898217_MAX_FOCUS_POS			1023
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define LC898217_FOCUS_STEPS			1
#define LC898217_SET_POSITION_ADDR		0x84 //liupeng: need change

#define LC898217_CMD_DELAY			0xff
#define LC898217_CTRL_DELAY_US			5000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define LC898217_MOVE_STEPS			16
#define LC898217_MOVE_DELAY_US			1000
#define LC898217_STABLE_TIME_US			1000

/* lc898217 device structure */
struct lc898217_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
};

static inline struct lc898217_device *to_lc898217_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct lc898217_device, ctrls);
}

static inline struct lc898217_device *sd_to_lc898217_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct lc898217_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

static int lc898217_set_position(struct lc898217_device *lc898217, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lc898217->sd);

	return i2c_smbus_write_word_data(client, LC898217_SET_POSITION_ADDR,
					 swab16(val));
}

static int lc898217_release(struct lc898217_device *lc898217)
{
	int ret, val;

	for (val = round_down(lc898217->focus->val, LC898217_MOVE_STEPS);
	     val >= 16; val = (val/2)) {
		ret = lc898217_set_position(lc898217, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(LC898217_MOVE_DELAY_US,
			     LC898217_MOVE_DELAY_US + 1000);
	}

	/*
	 * Wait for the motor to stabilize after the last movement
	 * to prevent the motor from shaking.
	 */
	usleep_range(LC898217_STABLE_TIME_US - LC898217_MOVE_DELAY_US,
		     LC898217_STABLE_TIME_US - LC898217_MOVE_DELAY_US + 1000);

	return 0;
}

static int lc898217_init(struct lc898217_device *lc898217)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lc898217->sd);
	int ret;

	LOG_INF("+\n");

	client->addr = LC898217_I2C_SLAVE_ADDR >> 1;

	ret = i2c_smbus_write_byte_data(client, 0xF6, 0x00);
	ret = i2c_smbus_write_byte_data(client, 0x96, 0x20);
	ret = i2c_smbus_write_byte_data(client, 0x98, 0x00);

	ret = i2c_smbus_read_byte_data(client, 0xF0);

	LOG_INF("Check HW version: %x\n", ret);

	if (ret == 0x72) {
		int wait_cnt = 20;

		ret = i2c_smbus_write_byte_data(client, 0xE0, 0x01);

		while (wait_cnt > 0) {
			ret = i2c_smbus_read_byte_data(client, 0xB3);

			if (ret == 0)
				break;

			wait_cnt--;
		}
	}

	LOG_INF("-\n");

	return 0;
}

/* Power handling */
static int lc898217_power_off(struct lc898217_device *lc898217)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lc898217->sd);
	int ret;

	LOG_INF("%s\n", __func__);

	ret = i2c_smbus_write_byte_data(client, 0x98, 0xC0);
	if (ret == 0)
		ret = i2c_smbus_write_byte_data(client, 0x96, 0x28);
	if (ret == 0)
		ret = i2c_smbus_write_byte_data(client, 0xF6, 0x80);

	ret = lc898217_release(lc898217);
	if (ret)
		LOG_INF("lc898217 release failed!\n");

	ret = regulator_disable(lc898217->vin);
	if (ret)
		return ret;

	ret = regulator_disable(lc898217->vdd);
	if (ret)
		return ret;

	if (lc898217->vcamaf_pinctrl && lc898217->vcamaf_off)
		ret = pinctrl_select_state(lc898217->vcamaf_pinctrl,
					lc898217->vcamaf_off);

	return ret;
}

static int lc898217_power_on(struct lc898217_device *lc898217)
{
	int ret;

	LOG_INF("%s\n", __func__);

	ret = regulator_enable(lc898217->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(lc898217->vdd);
	if (ret < 0)
		return ret;

	if (lc898217->vcamaf_pinctrl && lc898217->vcamaf_on)
		ret = pinctrl_select_state(lc898217->vcamaf_pinctrl,
					lc898217->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(LC898217_CTRL_DELAY_US, LC898217_CTRL_DELAY_US + 100);

	ret = lc898217_init(lc898217);
	if (ret < 0)
		goto fail;

	return 0;

fail:
	regulator_disable(lc898217->vin);
	regulator_disable(lc898217->vdd);
	if (lc898217->vcamaf_pinctrl && lc898217->vcamaf_off) {
		pinctrl_select_state(lc898217->vcamaf_pinctrl,
				lc898217->vcamaf_off);
	}

	return ret;
}

static int lc898217_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct lc898217_device *lc898217 = to_lc898217_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = lc898217_set_position(lc898217, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops lc898217_vcm_ctrl_ops = {
	.s_ctrl = lc898217_set_ctrl,
};

static int lc898217_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct lc898217_device *lc898217 = sd_to_lc898217_vcm(sd);

	LOG_INF("%s\n", __func__);

	ret = lc898217_power_on(lc898217);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int lc898217_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct lc898217_device *lc898217 = sd_to_lc898217_vcm(sd);

	LOG_INF("%s\n", __func__);

	lc898217_power_off(lc898217);

	return 0;
}

static const struct v4l2_subdev_internal_ops lc898217_int_ops = {
	.open = lc898217_open,
	.close = lc898217_close,
};

static const struct v4l2_subdev_ops lc898217_ops = { };

static void lc898217_subdev_cleanup(struct lc898217_device *lc898217)
{
	v4l2_async_unregister_subdev(&lc898217->sd);
	v4l2_ctrl_handler_free(&lc898217->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&lc898217->sd.entity);
#endif
}

static int lc898217_init_controls(struct lc898217_device *lc898217)
{
	struct v4l2_ctrl_handler *hdl = &lc898217->ctrls;
	const struct v4l2_ctrl_ops *ops = &lc898217_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	lc898217->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, LC898217_MAX_FOCUS_POS, LC898217_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	lc898217->sd.ctrl_handler = hdl;

	return 0;
}

static int lc898217_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lc898217_device *lc898217;
	int ret;

	LOG_INF("%s\n", __func__);

	lc898217 = devm_kzalloc(dev, sizeof(*lc898217), GFP_KERNEL);
	if (!lc898217)
		return -ENOMEM;

	lc898217->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(lc898217->vin)) {
		ret = PTR_ERR(lc898217->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	lc898217->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(lc898217->vdd)) {
		ret = PTR_ERR(lc898217->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	lc898217->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lc898217->vcamaf_pinctrl)) {
		ret = PTR_ERR(lc898217->vcamaf_pinctrl);
		lc898217->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		lc898217->vcamaf_on = pinctrl_lookup_state(
			lc898217->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(lc898217->vcamaf_on)) {
			ret = PTR_ERR(lc898217->vcamaf_on);
			lc898217->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		lc898217->vcamaf_off = pinctrl_lookup_state(
			lc898217->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(lc898217->vcamaf_off)) {
			ret = PTR_ERR(lc898217->vcamaf_off);
			lc898217->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&lc898217->sd, client, &lc898217_ops);
	lc898217->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	lc898217->sd.internal_ops = &lc898217_int_ops;

	ret = lc898217_init_controls(lc898217);
	if (ret)
		goto err_cleanup;

#if defined(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&lc898217->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	lc898217->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&lc898217->sd);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	lc898217_subdev_cleanup(lc898217);
	return ret;
}

static int lc898217_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898217_device *lc898217 = sd_to_lc898217_vcm(sd);

	LOG_INF("%s\n", __func__);

	lc898217_subdev_cleanup(lc898217);

	return 0;
}

static const struct i2c_device_id lc898217_id_table[] = {
	{ LC898217_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lc898217_id_table);

static const struct of_device_id lc898217_of_table[] = {
	{ .compatible = "mediatek,lc898217" },
	{ },
};
MODULE_DEVICE_TABLE(of, lc898217_of_table);

static struct i2c_driver lc898217_i2c_driver = {
	.driver = {
		.name = LC898217_NAME,
		.of_match_table = lc898217_of_table,
	},
	.probe_new  = lc898217_probe,
	.remove = lc898217_remove,
	.id_table = lc898217_id_table,
};

module_i2c_driver(lc898217_i2c_driver);

MODULE_AUTHOR("Sam Hung <Sam.Hung@mediatek.com>");
MODULE_DESCRIPTION("LC898217 VCM driver");
MODULE_LICENSE("GPL v2");
