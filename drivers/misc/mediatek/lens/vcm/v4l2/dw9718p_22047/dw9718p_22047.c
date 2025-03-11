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

#define DW9718AF_NAME				"dw9718afp_22047"
#define DW9718AF_MAX_FOCUS_POS			1023
#define DW9718AF_ORIGIN_FOCUS_POS			0
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define DW9718AF_FOCUS_STEPS			1
#define DW9718AF_CONTROL_REG			0x02
#define DW9718AF_SET_POSITION_ADDR		0x03
#define DW9718AF_STATUS_REG				0x05

#define DW9718AF_CONTROL_POWER_DOWN		BIT(0)
#define DW9718AF_AAC_MODE_EN			BIT(1)

#define DW9718AF_CMD_DELAY			0xff
#define DW9718AF_CTRL_DELAY_US			5000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define DW9718AF_MOVE_STEPS         30
#define DW9718AF_MOVE_DELAY_US      1000
#define DW9718AF_INIT_DELAY_US      1000

#define DW9718AF_RELEASE_START_CODE   250
#define DW9718AF_INIT_START_CODE      120
#define DW9718AF_INIT_SECOND_CODE     150
#define DW9718AF_INIT_THIRD_CODE      200
#define DW9718AF_RELEASE_SECOND_CODE  200
#define DW9718AF_RELEASE_THIRD_CODE   150
#define DW9718_MIN_CODE               90
#define DW9718AF_RELEASE_STEPS        30

/* dw9718af device structure */
struct dw9718af_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
};

static inline struct dw9718af_device *to_dw9718af_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9718af_device, ctrls);
}

static inline struct dw9718af_device *sd_to_dw9718af_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9718af_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char delay;
};

static struct regval_list dw9718af_init_regs[] = {
	{0x02, 0x01, 0},
	{0x02, 0x00, 1},
	{0x02, 0x02, 0},
	{0x06, 0x40, 0},
	{0x07, 0x5F, 0},
};

static int dw9718af_write_smbus(struct dw9718af_device *dw9718af, unsigned char reg,
			      unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9718af->sd);
	int ret = 0;

	if (reg == DW9718AF_CMD_DELAY  && value == DW9718AF_CMD_DELAY)
		usleep_range(DW9718AF_CTRL_DELAY_US,
			     DW9718AF_CTRL_DELAY_US + 100);
	else
		ret = i2c_smbus_write_byte_data(client, reg, value);
	return ret;
}

static int dw9718af_write_array(struct dw9718af_device *dw9718af,
			      struct regval_list *vals, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		pr_info("Init write [%d, %d]", vals[i].reg_num, vals[i].value);
		ret = dw9718af_write_smbus(dw9718af, vals[i].reg_num,
					 vals[i].value);
		if (ret < 0)
			return ret;

		usleep_range(vals[i].delay * DW9718AF_INIT_DELAY_US,
						vals[i].delay * DW9718AF_INIT_DELAY_US + 1000);
	}
	return 0;
}

static int dw9718af_set_position(struct dw9718af_device *dw9718af, u16 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&dw9718af->sd);
    int loop_time = 0, status = 0;
    pr_debug("dw9718 set position is %d", val);
    /*wait for I2C bus idle*/
    while (loop_time < 20) {
        status = i2c_smbus_read_byte_data(client, DW9718AF_STATUS_REG);
        status = status & 0x01;//get reg 05 status
        pr_debug("dw9718p 0x05 status:%x", status);
        if(status == 0){
            break;
        }
        loop_time++;
        usleep_range(DW9718AF_MOVE_DELAY_US, DW9718AF_MOVE_DELAY_US + 100);
    }
    if (loop_time >= 20)
    {
        pr_debug("waiting 0x05 flag timeout!");
    }
	return i2c_smbus_write_word_data(client, DW9718AF_SET_POSITION_ADDR,
					 swab16(val));
}

static int dw9718af_release(struct dw9718af_device *dw9718af)
{
    int ret, val;
    struct i2c_client *client = v4l2_get_subdevdata(&dw9718af->sd);
    pr_info("%s +\n", __func__);
    val = dw9718af->focus->val;
    val = val > DW9718AF_RELEASE_START_CODE ? DW9718AF_RELEASE_START_CODE : val;
    dw9718af_set_position(dw9718af, val);
    if (val >= DW9718AF_RELEASE_SECOND_CODE) {
        val = DW9718AF_RELEASE_SECOND_CODE;
        dw9718af_set_position(dw9718af, val);
        val = DW9718AF_RELEASE_THIRD_CODE;
        dw9718af_set_position(dw9718af, val);
    }
    while (val > DW9718_MIN_CODE) {
        val = val - DW9718AF_RELEASE_STEPS;
        ret = dw9718af_set_position(dw9718af, val);
        if (ret) {
            pr_info("%s I2C failure: %d",
                __func__, ret);
            return ret;
        }
    }

    ret = i2c_smbus_write_byte_data(client, DW9718AF_CONTROL_REG,
                    DW9718AF_CONTROL_POWER_DOWN);
    if (ret)
        return ret;

    pr_info("%s -\n", __func__);
    return 0;
}

static int dw9718af_init(struct dw9718af_device *dw9718af)
{
    int ret, val;
    pr_info("%s +\n", __func__);

    ret = dw9718af_write_array(dw9718af, dw9718af_init_regs,
                    ARRAY_SIZE(dw9718af_init_regs));
    if (ret)
        return ret;

    for (val = DW9718AF_INIT_START_CODE;
            val <= DW9718AF_INIT_SECOND_CODE;
        val += DW9718AF_MOVE_STEPS) {
        ret = dw9718af_set_position(dw9718af, val);
        pr_info("dw9718af set af_reg state is %d set val is %d", ret, val);
        if (ret) {
            pr_info("%s I2C failure: %d",
                __func__, ret);
            return ret;
        }
    }
    val = DW9718AF_INIT_THIRD_CODE;
    dw9718af_set_position(dw9718af, val);
	pr_info("%s -\n", __func__);
    return 0;
}

/* Power handling */
static int dw9718af_power_off(struct dw9718af_device *dw9718af)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = dw9718af_release(dw9718af);
	if (ret)
		pr_info("dw9718af release failed!\n");

	ret = regulator_disable(dw9718af->vin);
	if (ret)
		return ret;

	ret = regulator_disable(dw9718af->vdd);
	if (ret)
		return ret;

	if (dw9718af->vcamaf_pinctrl && dw9718af->vcamaf_off)
		ret = pinctrl_select_state(dw9718af->vcamaf_pinctrl,
					dw9718af->vcamaf_off);

	return ret;
}

static int dw9718af_power_on(struct dw9718af_device *dw9718af)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = regulator_enable(dw9718af->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dw9718af->vdd);
	if (ret < 0)
		return ret;

	if (dw9718af->vcamaf_pinctrl && dw9718af->vcamaf_on)
		ret = pinctrl_select_state(dw9718af->vcamaf_pinctrl,
					dw9718af->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(DW9718AF_CTRL_DELAY_US, DW9718AF_CTRL_DELAY_US + 100);

	ret = dw9718af_init(dw9718af);
	if (ret < 0)
		goto fail;

	return 0;

fail:
	regulator_disable(dw9718af->vin);
	regulator_disable(dw9718af->vdd);
	if (dw9718af->vcamaf_pinctrl && dw9718af->vcamaf_off) {
		pinctrl_select_state(dw9718af->vcamaf_pinctrl,
				dw9718af->vcamaf_off);
	}

	return ret;
}

static int dw9718af_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct dw9718af_device *dw9718af = to_dw9718af_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		ret = dw9718af_set_position(dw9718af, ctrl->val);
		if (ret) {
			pr_info("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops dw9718af_vcm_ctrl_ops = {
	.s_ctrl = dw9718af_set_ctrl,
};

static int dw9718af_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct dw9718af_device *dw9718af = sd_to_dw9718af_vcm(sd);

	pr_info("%s\n", __func__);

	ret = dw9718af_power_on(dw9718af);
	if (ret < 0) {
		pr_info("%s power on fail, ret = %d",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int dw9718af_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9718af_device *dw9718af = sd_to_dw9718af_vcm(sd);

	pr_info("%s\n", __func__);

	dw9718af_power_off(dw9718af);

	return 0;
}

static const struct v4l2_subdev_internal_ops dw9718af_int_ops = {
	.open = dw9718af_open,
	.close = dw9718af_close,
};

static const struct v4l2_subdev_ops dw9718af_ops = { };

static void dw9718af_subdev_cleanup(struct dw9718af_device *dw9718af)
{
	v4l2_async_unregister_subdev(&dw9718af->sd);
	v4l2_ctrl_handler_free(&dw9718af->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9718af->sd.entity);
#endif
}

static int dw9718af_init_controls(struct dw9718af_device *dw9718af)
{
	struct v4l2_ctrl_handler *hdl = &dw9718af->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9718af_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9718af->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, DW9718AF_MAX_FOCUS_POS, DW9718AF_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	dw9718af->sd.ctrl_handler = hdl;

	return 0;
}

static int dw9718af_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9718af_device *dw9718af;
	int ret;

	pr_info("%s\n", __func__);

	dw9718af = devm_kzalloc(dev, sizeof(*dw9718af), GFP_KERNEL);
	if (!dw9718af)
		return -ENOMEM;

	dw9718af->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(dw9718af->vin)) {
		ret = PTR_ERR(dw9718af->vin);
		if (ret != -EPROBE_DEFER)
			pr_info("cannot get vin regulator\n");
		return ret;
	}

	dw9718af->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dw9718af->vdd)) {
		ret = PTR_ERR(dw9718af->vdd);
		if (ret != -EPROBE_DEFER)
			pr_info("cannot get vdd regulator\n");
		return ret;
	}

	dw9718af->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(dw9718af->vcamaf_pinctrl)) {
		ret = PTR_ERR(dw9718af->vcamaf_pinctrl);
		dw9718af->vcamaf_pinctrl = NULL;
		pr_info("cannot get pinctrl\n");
	} else {
		dw9718af->vcamaf_on = pinctrl_lookup_state(
			dw9718af->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(dw9718af->vcamaf_on)) {
			ret = PTR_ERR(dw9718af->vcamaf_on);
			dw9718af->vcamaf_on = NULL;
			pr_info("cannot get vcamaf_on pinctrl\n");
		}

		dw9718af->vcamaf_off = pinctrl_lookup_state(
			dw9718af->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(dw9718af->vcamaf_off)) {
			ret = PTR_ERR(dw9718af->vcamaf_off);
			dw9718af->vcamaf_off = NULL;
			pr_info("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&dw9718af->sd, client, &dw9718af_ops);
	dw9718af->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9718af->sd.internal_ops = &dw9718af_int_ops;

	ret = dw9718af_init_controls(dw9718af);
	if (ret)
		goto err_cleanup;

#if defined(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9718af->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9718af->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&dw9718af->sd);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	dw9718af_subdev_cleanup(dw9718af);
	return ret;
}

static int dw9718af_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9718af_device *dw9718af = sd_to_dw9718af_vcm(sd);

	pr_info("%s\n", __func__);

	dw9718af_subdev_cleanup(dw9718af);

	return 0;
}

static const struct i2c_device_id dw9718af_id_table[] = {
	{ DW9718AF_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9718af_id_table);

static const struct of_device_id dw9718af_of_table[] = {
	{ .compatible = "mediatek,dw9718p_22047" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9718af_of_table);

static struct i2c_driver dw9718af_i2c_driver = {
	.driver = {
		.name = DW9718AF_NAME,
		.of_match_table = dw9718af_of_table,
	},
	.probe_new  = dw9718af_probe,
	.remove = dw9718af_remove,
	.id_table = dw9718af_id_table,
};

module_i2c_driver(dw9718af_i2c_driver);

MODULE_AUTHOR("Dongchun Zhu <dongchun.zhu@mediatek.com>");
MODULE_DESCRIPTION("DW9718AF VCM driver");
MODULE_LICENSE("GPL v2");
