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

#define DW9800S_TELE_NAME				"dw9800s_tele_23081"
#define DW9800S_TELE_MAX_FOCUS_POS			1023
#define DW9800S_TELE_ORIGIN_FOCUS_POS			512
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define DW9800S_TELE_FOCUS_STEPS			1
#define DW9800S_TELE_CONTROL_REG			0x02
#define DW9800S_TELE_SET_POSITION_ADDR		0x03
#define DW9800S_TELE_STATUS_REG				0x05

#define DW9800S_TELE_CONTROL_POWER_DOWN		BIT(0)
#define DW9800S_TELE_AAC_MODE_EN			BIT(1)

#define DW9800S_TELE_CMD_DELAY			0xff
#define DW9800S_TELE_CTRL_DELAY_US			5000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define DW9800S_TELE_MOVE_STEPS         50
#define DW9800S_TELE_MOVE_DELAY_US      1000
#define DW9800S_TELE_INIT_DELAY_US      1000

/* dw9800s_tele device structure */
struct dw9800s_tele_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
};

static inline struct dw9800s_tele_device *to_dw9800s_tele_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9800s_tele_device, ctrls);
}

static inline struct dw9800s_tele_device *sd_to_dw9800s_tele_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9800s_tele_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char delay;
};

static struct regval_list dw9800s_tele_init_regs[] = {
    {0x02, 0x01, 0},
    {0x02, 0x00, 1},
    {0x02, 0x02, 0},
    {0x06, 0x40, 0},
    {0x07, 0x6F, 1},
};

static int dw9800s_tele_write_smbus(struct dw9800s_tele_device *dw9800s_tele, unsigned char reg,
			      unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800s_tele->sd);
	int ret = 0;

	if (reg == DW9800S_TELE_CMD_DELAY  && value == DW9800S_TELE_CMD_DELAY)
		usleep_range(DW9800S_TELE_CTRL_DELAY_US,
			     DW9800S_TELE_CTRL_DELAY_US + 100);
	else
		ret = i2c_smbus_write_byte_data(client, reg, value);
	return ret;
}

static int dw9800s_tele_write_array(struct dw9800s_tele_device *dw9800s_tele,
			      struct regval_list *vals, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		pr_info("Init write [%d, %d]", vals[i].reg_num, vals[i].value);
		ret = dw9800s_tele_write_smbus(dw9800s_tele, vals[i].reg_num,
					 vals[i].value);
		if (ret < 0)
			return ret;

		usleep_range(vals[i].delay * DW9800S_TELE_INIT_DELAY_US,
						vals[i].delay * DW9800S_TELE_INIT_DELAY_US + 1000);
	}
	return 0;
}

static int dw9800s_tele_set_position(struct dw9800s_tele_device *dw9800s_tele, u16 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&dw9800s_tele->sd);
    int loop_time = 0, status = 0;
    pr_info("dw9800s tele set position is %d", val);
    /*wait for I2C bus idle*/
    while (loop_time < 20) {
        status = i2c_smbus_read_byte_data(client, DW9800S_TELE_STATUS_REG);
        status = status & 0x01;//get reg 05 status
        pr_debug("dw9800s_tele 0x05 status:%x", status);
        if(status == 0){
            break;
        }
        loop_time++;
        usleep_range(DW9800S_TELE_MOVE_DELAY_US, DW9800S_TELE_MOVE_DELAY_US + 100);
    }
    if (loop_time >= 20)
    {
        pr_debug("waiting 0x05 flag timeout!");
    }
	return i2c_smbus_write_word_data(client, DW9800S_TELE_SET_POSITION_ADDR,
					 swab16(val));
}

static int dw9800s_tele_release(struct dw9800s_tele_device *dw9800s_tele)
{
    int ret, val;
    int diff_dac = 0;
    int nStep_count = 0;
    int i = 0;
    struct i2c_client *client = v4l2_get_subdevdata(&dw9800s_tele->sd);

    diff_dac = DW9800S_TELE_ORIGIN_FOCUS_POS - dw9800s_tele->focus->val;

    nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
        DW9800S_TELE_MOVE_STEPS;

    val = dw9800s_tele->focus->val;

    for (i = 0; i < nStep_count; ++i) {
        val += (diff_dac < 0 ? (DW9800S_TELE_MOVE_STEPS*(-1)) : DW9800S_TELE_MOVE_STEPS);
        ret = dw9800s_tele_set_position(dw9800s_tele, val);
        if (ret) {
            pr_info("%s I2C failure: %d",
                __func__, ret);
            return ret;
        }
        usleep_range(DW9800S_TELE_MOVE_DELAY_US, DW9800S_TELE_MOVE_DELAY_US + 1000);
    }

    // last step to origin
    ret = dw9800s_tele_set_position(dw9800s_tele, DW9800S_TELE_ORIGIN_FOCUS_POS);
    if (ret) {
        pr_info("%s I2C failure: %d",
            __func__, ret);
        return ret;
    }

    ret = i2c_smbus_write_byte_data(client, DW9800S_TELE_CONTROL_REG,
                    DW9800S_TELE_CONTROL_POWER_DOWN);
    if (ret)
        return ret;

    pr_info("%s -\n", __func__);
    return 0;
}

static int dw9800s_tele_init(struct dw9800s_tele_device *dw9800s_tele)
{
    int ret, val;
    pr_info("%s +\n", __func__);

    ret = dw9800s_tele_write_array(dw9800s_tele, dw9800s_tele_init_regs,
                    ARRAY_SIZE(dw9800s_tele_init_regs));
    if (ret)
        return ret;

    val = DW9800S_TELE_ORIGIN_FOCUS_POS;
    ret = dw9800s_tele_set_position(dw9800s_tele, val);
    if (ret)
        return ret;
    pr_info("%s -\n", __func__);
    return 0;
}

/* Power handling */
static int dw9800s_tele_power_off(struct dw9800s_tele_device *dw9800s_tele)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = dw9800s_tele_release(dw9800s_tele);
	if (ret)
		pr_info("dw9800s_tele release failed!\n");

	ret = regulator_disable(dw9800s_tele->vin);
	if (ret)
		return ret;

	ret = regulator_disable(dw9800s_tele->vdd);
	if (ret)
		return ret;

	if (dw9800s_tele->vcamaf_pinctrl && dw9800s_tele->vcamaf_off)
		ret = pinctrl_select_state(dw9800s_tele->vcamaf_pinctrl,
					dw9800s_tele->vcamaf_off);

	return ret;
}

static int dw9800s_tele_power_on(struct dw9800s_tele_device *dw9800s_tele)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = regulator_enable(dw9800s_tele->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dw9800s_tele->vdd);
	if (ret < 0)
		return ret;

	if (dw9800s_tele->vcamaf_pinctrl && dw9800s_tele->vcamaf_on)
		ret = pinctrl_select_state(dw9800s_tele->vcamaf_pinctrl,
					dw9800s_tele->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(DW9800S_TELE_CTRL_DELAY_US, DW9800S_TELE_CTRL_DELAY_US + 100);

	ret = dw9800s_tele_init(dw9800s_tele);
	if (ret < 0)
		goto fail;

	return 0;

fail:
	regulator_disable(dw9800s_tele->vin);
	regulator_disable(dw9800s_tele->vdd);
	if (dw9800s_tele->vcamaf_pinctrl && dw9800s_tele->vcamaf_off) {
		pinctrl_select_state(dw9800s_tele->vcamaf_pinctrl,
				dw9800s_tele->vcamaf_off);
	}

	return ret;
}

static int dw9800s_tele_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct dw9800s_tele_device *dw9800s_tele = to_dw9800s_tele_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		ret = dw9800s_tele_set_position(dw9800s_tele, ctrl->val);
		if (ret) {
			pr_info("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops dw9800s_tele_vcm_ctrl_ops = {
	.s_ctrl = dw9800s_tele_set_ctrl,
};

static int dw9800s_tele_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct dw9800s_tele_device *dw9800s_tele = sd_to_dw9800s_tele_vcm(sd);

	pr_info("%s\n", __func__);

	ret = dw9800s_tele_power_on(dw9800s_tele);
	if (ret < 0) {
		pr_info("%s power on fail, ret = %d",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int dw9800s_tele_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9800s_tele_device *dw9800s_tele = sd_to_dw9800s_tele_vcm(sd);

	pr_info("%s\n", __func__);

	dw9800s_tele_power_off(dw9800s_tele);

	return 0;
}

static const struct v4l2_subdev_internal_ops dw9800s_tele_int_ops = {
	.open = dw9800s_tele_open,
	.close = dw9800s_tele_close,
};

static const struct v4l2_subdev_ops dw9800s_tele_ops = { };

static void dw9800s_tele_subdev_cleanup(struct dw9800s_tele_device *dw9800s_tele)
{
	v4l2_async_unregister_subdev(&dw9800s_tele->sd);
	v4l2_ctrl_handler_free(&dw9800s_tele->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9800s_tele->sd.entity);
#endif
}

static int dw9800s_tele_init_controls(struct dw9800s_tele_device *dw9800s_tele)
{
	struct v4l2_ctrl_handler *hdl = &dw9800s_tele->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9800s_tele_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9800s_tele->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, DW9800S_TELE_MAX_FOCUS_POS, DW9800S_TELE_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	dw9800s_tele->sd.ctrl_handler = hdl;

	return 0;
}

static int dw9800s_tele_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9800s_tele_device *dw9800s_tele;
	int ret;

	pr_info("%s\n", __func__);

	dw9800s_tele = devm_kzalloc(dev, sizeof(*dw9800s_tele), GFP_KERNEL);
	if (!dw9800s_tele)
		return -ENOMEM;

	dw9800s_tele->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(dw9800s_tele->vin)) {
		ret = PTR_ERR(dw9800s_tele->vin);
		if (ret != -EPROBE_DEFER)
			pr_info("cannot get vin regulator\n");
		return ret;
	}

	dw9800s_tele->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dw9800s_tele->vdd)) {
		ret = PTR_ERR(dw9800s_tele->vdd);
		if (ret != -EPROBE_DEFER)
			pr_info("cannot get vdd regulator\n");
		return ret;
	}

	dw9800s_tele->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(dw9800s_tele->vcamaf_pinctrl)) {
		ret = PTR_ERR(dw9800s_tele->vcamaf_pinctrl);
		dw9800s_tele->vcamaf_pinctrl = NULL;
		pr_info("cannot get pinctrl\n");
	} else {
		dw9800s_tele->vcamaf_on = pinctrl_lookup_state(
			dw9800s_tele->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(dw9800s_tele->vcamaf_on)) {
			ret = PTR_ERR(dw9800s_tele->vcamaf_on);
			dw9800s_tele->vcamaf_on = NULL;
			pr_info("cannot get vcamaf_on pinctrl\n");
		}

		dw9800s_tele->vcamaf_off = pinctrl_lookup_state(
			dw9800s_tele->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(dw9800s_tele->vcamaf_off)) {
			ret = PTR_ERR(dw9800s_tele->vcamaf_off);
			dw9800s_tele->vcamaf_off = NULL;
			pr_info("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&dw9800s_tele->sd, client, &dw9800s_tele_ops);
	dw9800s_tele->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9800s_tele->sd.internal_ops = &dw9800s_tele_int_ops;

	ret = dw9800s_tele_init_controls(dw9800s_tele);
	if (ret)
		goto err_cleanup;

#if defined(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9800s_tele->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9800s_tele->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&dw9800s_tele->sd);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	dw9800s_tele_subdev_cleanup(dw9800s_tele);
	return ret;
}

static int dw9800s_tele_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9800s_tele_device *dw9800s_tele = sd_to_dw9800s_tele_vcm(sd);

	pr_info("%s\n", __func__);

	dw9800s_tele_subdev_cleanup(dw9800s_tele);

	return 0;
}

static const struct i2c_device_id dw9800s_tele_id_table[] = {
	{ DW9800S_TELE_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9800s_tele_id_table);

static const struct of_device_id dw9800s_tele_of_table[] = {
	{ .compatible = "mediatek,dw9800s_tele_23081" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9800s_tele_of_table);

static struct i2c_driver dw9800s_tele_i2c_driver = {
	.driver = {
		.name = DW9800S_TELE_NAME,
		.of_match_table = dw9800s_tele_of_table,
	},
	.probe_new  = dw9800s_tele_probe,
	.remove = dw9800s_tele_remove,
	.id_table = dw9800s_tele_id_table,
};

module_i2c_driver(dw9800s_tele_i2c_driver);

MODULE_AUTHOR("Dongchun Zhu <dongchun.zhu@mediatek.com>");
MODULE_DESCRIPTION("DW9800S_TELE VCM driver");
MODULE_LICENSE("GPL v2");
