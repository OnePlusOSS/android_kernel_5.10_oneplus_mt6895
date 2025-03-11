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

#define DRIVER_NAME "dw9827c_23081"

#define DW9827C_23081_I2C_SLAVE_ADDR 0x18

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define DW9827C_23081_NAME				"dw9827c_23081"
#define DW9827C_23081_MAX_FOCUS_POS			1023
#define DW9827C_23081_ORIGIN_FOCUS_POS		0
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define DW9827C_23081_FOCUS_STEPS			1
#define DW9827C_23081_SET_POSITION_ADDR		0x00

#define DW9827C_23081_CMD_DELAY			0xff
#define DW9827C_23081_CTRL_DELAY_US			10000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define DW9827C_23081_MOVE_STEPS			100
#define DW9827C_23081_MOVE_DELAY_US			1000
#define DW9827C_23081_INIT_DELAY_US			3000

#define DW9827C_23081_MOVE_INIT_POS			230

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif /* OPLUS_FEATURE_CAMERA_COMMON */

/* dw9827c_23081 device structure */
struct dw9827c_23081_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
	/* active or standby mode */
	bool active;
};

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char delay;
};

static struct regval_list dw9827c_23081_init_regs[] = {
	{0x02, 0x40, 3},
	{0x04, 0x01, 3},
	{0x00, ((DW9827C_23081_MOVE_INIT_POS << 6) >> 8) & 0xff, 0},
	{0x01, (DW9827C_23081_MOVE_INIT_POS << 6) & 0xff       , 0},
	{0x02, 0x00, 3},
};

#define VCM_IOC_POWER_ON         _IO('V', BASE_VIDIOC_PRIVATE + 3)
#define VCM_IOC_POWER_OFF        _IO('V', BASE_VIDIOC_PRIVATE + 4)

static inline struct dw9827c_23081_device *to_dw9827c_23081_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9827c_23081_device, ctrls);
}

static inline struct dw9827c_23081_device *sd_to_dw9827c_23081_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9827c_23081_device, sd);
}

static int dw9827c_23081_set_position(struct dw9827c_23081_device *dw9827c_23081, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_23081->sd);
	int retry = 3;
	int ret;

	LOG_INF("dw9827c_23081 Set postition:%u.", val);
	while (--retry > 0) {
        ret = i2c_smbus_write_word_data(client, DW9827C_23081_SET_POSITION_ADDR,
                                        swab16(val << 6));
		if (ret < 0) {
			usleep_range(DW9827C_23081_MOVE_DELAY_US,
				     DW9827C_23081_MOVE_DELAY_US + 1000);
			#ifdef OPLUS_FEATURE_CAMERA_COMMON
			LOG_INF("dw9827c_23081 Set postition:%u fail.", val);
			#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
		} else {
			break;
		}
	}
	return ret;
}

static int dw9827c_23081_release(struct dw9827c_23081_device *dw9827c_23081)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_23081->sd);

	diff_dac = DW9827C_23081_ORIGIN_FOCUS_POS - dw9827c_23081->focus->val;

	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		DW9827C_23081_MOVE_STEPS;

	val = dw9827c_23081->focus->val;

	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (DW9827C_23081_MOVE_STEPS*(-1)) : DW9827C_23081_MOVE_STEPS);

		ret = dw9827c_23081_set_position(dw9827c_23081, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(DW9827C_23081_MOVE_DELAY_US,
			     DW9827C_23081_MOVE_DELAY_US + 1000);
	}

	// last step to origin
	ret = dw9827c_23081_set_position(dw9827c_23081, DW9827C_23081_ORIGIN_FOCUS_POS);
	if (ret) {
		LOG_INF("%s I2C failure: %d",
			__func__, ret);
		return ret;
	}

	i2c_smbus_write_byte_data(client, 0x02, 0x20);
	dw9827c_23081->active = false;
	LOG_INF("-\n");

	return 0;
}


static int dw9827c_23081_write_array(struct dw9827c_23081_device *dw9827c_23081,
			      struct regval_list *vals, u32 len)
{
	unsigned int i;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_23081->sd);

	for (i = 0; i < len; i++) {
		LOG_INF("Init write [0x%x, 0x%x, %d]", vals[i].reg_num, vals[i].value, vals[i].delay);
		ret = i2c_smbus_write_byte_data(client, vals[i].reg_num, vals[i].value);
		if (ret < 0)
			return ret;
		if(vals[i].delay) {
			usleep_range(vals[i].delay * 1000, vals[i].delay * 1000 + 1000);
		}
	}

	return 0;
}


static int dw9827c_23081_init(struct dw9827c_23081_device *dw9827c_23081)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_23081->sd);
	int ret = 0;

	LOG_INF("+\n");

	client->addr = DW9827C_23081_I2C_SLAVE_ADDR >> 1;
	ret = i2c_smbus_read_byte_data(client, 0x02);

	LOG_INF("Check HW version: %x\n", ret);

	ret = i2c_smbus_read_byte_data(client, 0x03);

	LOG_INF("product_id: %x\n", ret);

	LOG_INF("dw9827c_23081_init ret: %d", ret);

	ret = dw9827c_23081_write_array(dw9827c_23081, dw9827c_23081_init_regs,
                    ARRAY_SIZE(dw9827c_23081_init_regs));

	dw9827c_23081->active = true;

	LOG_INF("-\n");

	return 0;
}

/* Power handling */
static int dw9827c_23081_power_off(struct dw9827c_23081_device *dw9827c_23081)
{
	int ret;

	LOG_INF("+\n");

	ret = dw9827c_23081_release(dw9827c_23081);
	if (ret)
		LOG_INF("dw9827c_23081 release failed!\n");

	ret = regulator_disable(dw9827c_23081->vin);
	if (ret)
		return ret;

	ret = regulator_disable(dw9827c_23081->vdd);
	if (ret)
		return ret;

	if (dw9827c_23081->vcamaf_pinctrl && dw9827c_23081->vcamaf_off)
		ret = pinctrl_select_state(dw9827c_23081->vcamaf_pinctrl,
					dw9827c_23081->vcamaf_off);

	LOG_INF("-\n");

	return ret;
}

static int dw9827c_23081_power_on(struct dw9827c_23081_device *dw9827c_23081)
{
	int ret;

	LOG_INF("+\n");

	ret = regulator_enable(dw9827c_23081->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dw9827c_23081->vdd);
	if (ret < 0)
		return ret;

	if (dw9827c_23081->vcamaf_pinctrl && dw9827c_23081->vcamaf_on)
		ret = pinctrl_select_state(dw9827c_23081->vcamaf_pinctrl,
					dw9827c_23081->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(DW9827C_23081_CTRL_DELAY_US, DW9827C_23081_CTRL_DELAY_US + 100);

	ret = dw9827c_23081_init(dw9827c_23081);
	if (ret < 0)
		goto fail;

	LOG_INF("-\n");

	return 0;

fail:
	regulator_disable(dw9827c_23081->vin);
	regulator_disable(dw9827c_23081->vdd);
	if (dw9827c_23081->vcamaf_pinctrl && dw9827c_23081->vcamaf_off) {
		pinctrl_select_state(dw9827c_23081->vcamaf_pinctrl,
				dw9827c_23081->vcamaf_off);
	}

	return ret;
}

static int dw9827c_23081_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct dw9827c_23081_device *dw9827c_23081 = to_dw9827c_23081_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = dw9827c_23081_set_position(dw9827c_23081, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops dw9827c_23081_vcm_ctrl_ops = {
	.s_ctrl = dw9827c_23081_set_ctrl,
};

static int dw9827c_23081_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct dw9827c_23081_device *dw9827c_23081 = sd_to_dw9827c_23081_vcm(sd);

	ret = dw9827c_23081_power_on(dw9827c_23081);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int dw9827c_23081_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9827c_23081_device *dw9827c_23081 = sd_to_dw9827c_23081_vcm(sd);

	dw9827c_23081_power_off(dw9827c_23081);

	return 0;
}

static long dw9827c_23081_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct dw9827c_23081_device *dw9827c_23081 = sd_to_dw9827c_23081_vcm(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_23081->sd);
	LOG_INF("+\n");
	client->addr = DW9827C_23081_I2C_SLAVE_ADDR >> 1;

	switch (cmd) {
	case VCM_IOC_POWER_ON:
	{
		// customized area
		/* 00:active mode , 10:Standby mode , x1:Sleep mode */
		if (dw9827c_23081->active)
			return 0;
		ret = i2c_smbus_write_byte_data(client, 0x02, 0x00);
		if (ret) {
			LOG_INF("I2C failure!!!\n");
		} else {
			dw9827c_23081->active = true;
			LOG_INF("stand by mode, power on!!!\n");
		}
	}
	break;
	case VCM_IOC_POWER_OFF:
	{
		// customized area
		if (!dw9827c_23081->active)
			return 0;
		ret = i2c_smbus_write_byte_data(client, 0x02, 0x40);
		if (ret) {
			LOG_INF("I2C failure!!!\n");
		} else {
			dw9827c_23081->active = false;
			LOG_INF("stand by mode, power off !!!!!!!!!!!!\n");
		}
	}
	break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static const struct v4l2_subdev_internal_ops dw9827c_23081_int_ops = {
	.open = dw9827c_23081_open,
	.close = dw9827c_23081_close,
};

static struct v4l2_subdev_core_ops dw9827c_23081_ops_core = {
	.ioctl = dw9827c_23081_ops_core_ioctl,
};

static const struct v4l2_subdev_ops dw9827c_23081_ops = {
	.core = &dw9827c_23081_ops_core,
};

static void dw9827c_23081_subdev_cleanup(struct dw9827c_23081_device *dw9827c_23081)
{
	v4l2_async_unregister_subdev(&dw9827c_23081->sd);
	v4l2_ctrl_handler_free(&dw9827c_23081->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9827c_23081->sd.entity);
#endif
}

static int dw9827c_23081_init_controls(struct dw9827c_23081_device *dw9827c_23081)
{
	struct v4l2_ctrl_handler *hdl = &dw9827c_23081->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9827c_23081_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9827c_23081->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, DW9827C_23081_MAX_FOCUS_POS, DW9827C_23081_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	dw9827c_23081->sd.ctrl_handler = hdl;

	return 0;
}

static int dw9827c_23081_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9827c_23081_device *dw9827c_23081;
	int ret;

	LOG_INF("+\n");

	dw9827c_23081 = devm_kzalloc(dev, sizeof(*dw9827c_23081), GFP_KERNEL);
	if (!dw9827c_23081)
		return -ENOMEM;

	dw9827c_23081->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(dw9827c_23081->vin)) {
		ret = PTR_ERR(dw9827c_23081->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	dw9827c_23081->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dw9827c_23081->vdd)) {
		ret = PTR_ERR(dw9827c_23081->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	dw9827c_23081->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(dw9827c_23081->vcamaf_pinctrl)) {
		ret = PTR_ERR(dw9827c_23081->vcamaf_pinctrl);
		dw9827c_23081->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl. ret: 0x%x\n", ret);
	} else {
		dw9827c_23081->vcamaf_on = pinctrl_lookup_state(
			dw9827c_23081->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(dw9827c_23081->vcamaf_on)) {
			ret = PTR_ERR(dw9827c_23081->vcamaf_on);
			dw9827c_23081->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl. ret: 0x%x\n", ret);
		}

		dw9827c_23081->vcamaf_off = pinctrl_lookup_state(
			dw9827c_23081->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(dw9827c_23081->vcamaf_off)) {
			ret = PTR_ERR(dw9827c_23081->vcamaf_off);
			dw9827c_23081->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl. ret: 0x%x\n", ret);
		}
	}

	v4l2_i2c_subdev_init(&dw9827c_23081->sd, client, &dw9827c_23081_ops);
	dw9827c_23081->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9827c_23081->sd.internal_ops = &dw9827c_23081_int_ops;

	ret = dw9827c_23081_init_controls(dw9827c_23081);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9827c_23081->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9827c_23081->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&dw9827c_23081->sd);
	if (ret < 0)
		goto err_cleanup;

	LOG_INF("-\n");

	return 0;

err_cleanup:
	dw9827c_23081_subdev_cleanup(dw9827c_23081);
	return ret;
}

static int dw9827c_23081_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9827c_23081_device *dw9827c_23081 = sd_to_dw9827c_23081_vcm(sd);

	LOG_INF("+\n");

	dw9827c_23081_subdev_cleanup(dw9827c_23081);

	LOG_INF("-\n");
	return 0;
}

static const struct i2c_device_id dw9827c_23081_id_table[] = {
	{ DW9827C_23081_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9827c_23081_id_table);

static const struct of_device_id dw9827c_23081_of_table[] = {
	{ .compatible = "mediatek,dw9827c_23081" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9827c_23081_of_table);

static struct i2c_driver dw9827c_23081_i2c_driver = {
	.driver = {
		.name = DW9827C_23081_NAME,
		.of_match_table = dw9827c_23081_of_table,
	},
	.probe_new  = dw9827c_23081_probe,
	.remove = dw9827c_23081_remove,
	.id_table = dw9827c_23081_id_table,
};

module_i2c_driver(dw9827c_23081_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("DW9827C_23081 VCM driver");
MODULE_LICENSE("GPL v2");
