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
#include <soc/oplus/system/oplus_project.h>

#define JD5516W_I2C_SLAVE_ADDR  0x18
#define JD5516W_NAME				"jd5516w_23251"
#define JD5516W_MAX_FOCUS_POS			1023
#define JD5516W_ORIGIN_FOCUS_POS			0
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define JD5516W_FOCUS_STEPS			1
#define JD5516W_CONTROL_REG			0x02
#define JD5516W_SET_POSITION_ADDR		0x03
#define JD5516W_STATUS_REG             0X05

#define JD5516W_CONTROL_POWER_DOWN		BIT(0)

#define JD5516W_CMD_DELAY			0xff
#define JD5516W_CTRL_DELAY_US			5000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define JD5516W_MOVE_STEPS          50
#define JD5516W_MOVE_SECOND_STEPS   50
#define JD5516W_MOVE_DELAY_US       1000
#define JD5516W_INIT_DELAY_US       1000

#define JD5516W_RELEASE_START_CODE  800
#define JD5516W_RELEASE_SECOND_CODE 700
#define JD5516W_RELEASE_THIRD_CODE  300
#define JD5516W_RELEASE_FOURTH_CODE 400
#define JD5516W_MIDDLE_CODE         512
#define JD5516W_INIT_START_CODE     304
#define JD5516W_INIT_SECOND_CODE    532
#define JD5516W_INIT_THIRD_CODE     200
#define JD5516W_MIN_CODE            90
#define JD5516W_RELEASE_STEPS       30
#define JD5516W_RELEASE_10MS        10000
#define JD5516W_RELEASE_1MS         1000

/* jd5516w device structure */
struct jd5516w_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
};

static inline struct jd5516w_device *to_jd5516w_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct jd5516w_device, ctrls);
}

static inline struct jd5516w_device *sd_to_jd5516w_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct jd5516w_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char delay;
};

//Need update later.
static struct regval_list jd5516w_init_regs[] = {
	{0x02, 0x01, 0},
	{0x02, 0x00, 1},
	{0x02, 0x02, 0},
	{0x06, 0x40, 0},
	{0x07, 0x08, 0},
	{0x08, 0x01, 0},
};

static int jd5516w_write_smbus(struct jd5516w_device *jd5516w, unsigned char reg,
			      unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&jd5516w->sd);
	int ret = 0;

	if (reg == JD5516W_CMD_DELAY  && value == JD5516W_CMD_DELAY)
		usleep_range(JD5516W_CTRL_DELAY_US,
			     JD5516W_CTRL_DELAY_US + 100);
	else
		ret = i2c_smbus_write_byte_data(client, reg, value);
	return ret;
}

static int jd5516w_write_array(struct jd5516w_device *jd5516w,
			      struct regval_list *vals, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		pr_info("Init write [%d, %d]", vals[i].reg_num, vals[i].value);
		ret = jd5516w_write_smbus(jd5516w, vals[i].reg_num,
					 vals[i].value);
		if (ret < 0)
			return ret;

		usleep_range(vals[i].delay * JD5516W_INIT_DELAY_US,
						vals[i].delay * JD5516W_INIT_DELAY_US + 1000);
	}
	return 0;
}

static int jd5516w_set_position(struct jd5516w_device *jd5516w, u16 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&jd5516w->sd);
    int loop_time = 0, status = 0;
    pr_info("jd5516w set position is %d", val);
    /*wait for I2C bus idle*/
    while (loop_time < 20) {
        status = i2c_smbus_read_byte_data(client, JD5516W_STATUS_REG);
        status = status & 0x01;//get reg 05 status
        pr_debug("jd5516w 0x05 status:%x", status);
        if(status == 0){
            break;
        }
        loop_time++;
        usleep_range(JD5516W_MOVE_DELAY_US, JD5516W_MOVE_DELAY_US + 100);
    }
    if (loop_time >= 20) {
        pr_info("watting 0x05 flag timeout!");
    }

    return i2c_smbus_write_word_data(client, JD5516W_SET_POSITION_ADDR, swab16(val & 0x03FF));
}

static int jd5516w_release(struct jd5516w_device *jd5516w)
{
    int ret = 0, val;
    unsigned long af_step = JD5516W_MOVE_STEPS;
    struct i2c_client *client = v4l2_get_subdevdata(&jd5516w->sd);
    pr_info("%s +\n", __func__);
    val = jd5516w->focus->val;
    if(val > JD5516W_MIDDLE_CODE) {
        if( val > JD5516W_RELEASE_START_CODE) {
            val = JD5516W_RELEASE_START_CODE;
            if (jd5516w_set_position(jd5516w, val)) {
                pr_info("set I2C failed when moving the motor\n");
                ret = -1;
            }
        }
        msleep(5);
        while (val > JD5516W_MIDDLE_CODE + af_step) {
            if (val > JD5516W_RELEASE_SECOND_CODE) {
                af_step = JD5516W_MOVE_SECOND_STEPS;
            }
            val = val - af_step;
            if (jd5516w_set_position(jd5516w, val)) {
                pr_info("set I2C failed when moving the motor\n");
                ret = -1;
                break;
            }
            msleep(5);
        }
    } else if(val < JD5516W_MIDDLE_CODE){
        if( val < JD5516W_RELEASE_THIRD_CODE) {
            val = JD5516W_RELEASE_THIRD_CODE;
            if (jd5516w_set_position(jd5516w, val) == 0) {
                pr_info("set I2C failed when moving the motor\n");
                ret = -1;
            }
        }
        msleep(5);
        while (val < JD5516W_MIDDLE_CODE - af_step) {
            if (val > JD5516W_RELEASE_FOURTH_CODE) {
                af_step = JD5516W_MOVE_SECOND_STEPS;
            }
            val = val + af_step;
            if (jd5516w_set_position(jd5516w, val)) {
                pr_info("set I2C failed when moving the motor\n");
                ret = -1;
                break;
            }
            msleep(5);
        }
    }
    ret = i2c_smbus_write_byte_data(client, JD5516W_CONTROL_REG,
                    JD5516W_CONTROL_POWER_DOWN);
    if (ret)
        return ret;
    pr_info("%s -\n", __func__);
    return 0;
}

static int jd5516w_init(struct jd5516w_device *jd5516w)
{
    int ret;
    struct i2c_client *client = v4l2_get_subdevdata(&jd5516w->sd);
    pr_info("%s\n", __func__);
    client->addr = JD5516W_I2C_SLAVE_ADDR >> 1;

    ret = jd5516w_write_array(jd5516w, jd5516w_init_regs,
                    ARRAY_SIZE(jd5516w_init_regs));
    if (ret)
        return ret;
    pr_info("%s jd5516w driver init success.\n", __func__);
    return 0;
}

/* Power handling */
static int jd5516w_power_off(struct jd5516w_device *jd5516w)
{
	int ret;

	pr_info("%s\n", __func__);
	ret = jd5516w_release(jd5516w);
	if (ret)
		pr_info("jd5516w release failed!\n");

	ret = regulator_disable(jd5516w->vin);
	if (ret)
		return ret;

	ret = regulator_disable(jd5516w->vdd);
	if (ret)
		return ret;

	if (jd5516w->vcamaf_pinctrl && jd5516w->vcamaf_off)
		ret = pinctrl_select_state(jd5516w->vcamaf_pinctrl,
					jd5516w->vcamaf_off);

	return ret;
}

static int jd5516w_power_on(struct jd5516w_device *jd5516w)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = regulator_enable(jd5516w->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(jd5516w->vdd);
	if (ret < 0)
		return ret;

	if (jd5516w->vcamaf_pinctrl && jd5516w->vcamaf_on)
		ret = pinctrl_select_state(jd5516w->vcamaf_pinctrl,
					jd5516w->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(JD5516W_CTRL_DELAY_US, JD5516W_CTRL_DELAY_US + 100);

	ret = jd5516w_init(jd5516w);
	if (ret < 0)
		goto fail;

	return 0;

fail:
	regulator_disable(jd5516w->vin);
	regulator_disable(jd5516w->vdd);
	if (jd5516w->vcamaf_pinctrl && jd5516w->vcamaf_off) {
		pinctrl_select_state(jd5516w->vcamaf_pinctrl,
				jd5516w->vcamaf_off);
	}

	return ret;
}

static int jd5516w_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct jd5516w_device *jd5516w = to_jd5516w_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		ret = jd5516w_set_position(jd5516w, ctrl->val);
		if (ret) {
			pr_info("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops jd5516w_vcm_ctrl_ops = {
	.s_ctrl = jd5516w_set_ctrl,
};

static int jd5516w_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct jd5516w_device *jd5516w = sd_to_jd5516w_vcm(sd);
    struct i2c_client *client = v4l2_get_subdevdata(&jd5516w->sd);
	pr_info("%s\n", __func__);

	ret = jd5516w_power_on(jd5516w);
	if (ret < 0) {
		pr_info("%s power on fail, ret = %d",
			__func__, ret);
		return ret;
	}
    //get driver IC Version
	ret = i2c_smbus_read_byte_data(client, 0x01);
	pr_info("JD5516W driver IC version: 0x%x\n", ret);
    //get driver IC Info
	ret = i2c_smbus_read_byte_data(client, 0x00);
	pr_info("JD5516W driver IC Info: 0x%x\n", ret);
	return 0;
}

static int jd5516w_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct jd5516w_device *jd5516w = sd_to_jd5516w_vcm(sd);

	pr_info("%s\n", __func__);

	jd5516w_power_off(jd5516w);

	return 0;
}

static const struct v4l2_subdev_internal_ops jd5516w_int_ops = {
	.open = jd5516w_open,
	.close = jd5516w_close,
};

static const struct v4l2_subdev_ops jd5516w_ops = { };

static void jd5516w_subdev_cleanup(struct jd5516w_device *jd5516w)
{
	v4l2_async_unregister_subdev(&jd5516w->sd);
	v4l2_ctrl_handler_free(&jd5516w->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&jd5516w->sd.entity);
#endif
}

static int jd5516w_init_controls(struct jd5516w_device *jd5516w)
{
	struct v4l2_ctrl_handler *hdl = &jd5516w->ctrls;
	const struct v4l2_ctrl_ops *ops = &jd5516w_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	jd5516w->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, JD5516W_MAX_FOCUS_POS, JD5516W_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	jd5516w->sd.ctrl_handler = hdl;

	return 0;
}

static int jd5516w_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct jd5516w_device *jd5516w;
	int ret;

	pr_info("%s\n", __func__);

	jd5516w = devm_kzalloc(dev, sizeof(*jd5516w), GFP_KERNEL);
	if (!jd5516w)
		return -ENOMEM;

	jd5516w->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(jd5516w->vin)) {
		ret = PTR_ERR(jd5516w->vin);
		if (ret != -EPROBE_DEFER)
			pr_info("cannot get vin regulator\n");
		return ret;
	}

	jd5516w->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(jd5516w->vdd)) {
		ret = PTR_ERR(jd5516w->vdd);
		if (ret != -EPROBE_DEFER)
			pr_info("cannot get vdd regulator\n");
		return ret;
	}

	jd5516w->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(jd5516w->vcamaf_pinctrl)) {
		ret = PTR_ERR(jd5516w->vcamaf_pinctrl);
		jd5516w->vcamaf_pinctrl = NULL;
		pr_info("cannot get pinctrl\n");
	} else {
		jd5516w->vcamaf_on = pinctrl_lookup_state(
			jd5516w->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(jd5516w->vcamaf_on)) {
			ret = PTR_ERR(jd5516w->vcamaf_on);
			jd5516w->vcamaf_on = NULL;
			pr_info("cannot get vcamaf_on pinctrl\n");
		}

		jd5516w->vcamaf_off = pinctrl_lookup_state(
			jd5516w->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(jd5516w->vcamaf_off)) {
			ret = PTR_ERR(jd5516w->vcamaf_off);
			jd5516w->vcamaf_off = NULL;
			pr_info("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&jd5516w->sd, client, &jd5516w_ops);
	jd5516w->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	jd5516w->sd.internal_ops = &jd5516w_int_ops;

	ret = jd5516w_init_controls(jd5516w);
	if (ret)
		goto err_cleanup;

#if defined(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&jd5516w->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	jd5516w->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&jd5516w->sd);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	jd5516w_subdev_cleanup(jd5516w);
	return ret;
}

static int jd5516w_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct jd5516w_device *jd5516w = sd_to_jd5516w_vcm(sd);

	pr_info("%s\n", __func__);

	jd5516w_subdev_cleanup(jd5516w);

	return 0;
}

static const struct i2c_device_id jd5516w_id_table[] = {
	{ JD5516W_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, jd5516w_id_table);

static const struct of_device_id jd5516w_of_table[] = {
	{ .compatible = "mediatek,jd5516w_23251" },
	{ },
};
MODULE_DEVICE_TABLE(of, jd5516w_of_table);

static struct i2c_driver jd5516w_i2c_driver = {
	.driver = {
		.name = JD5516W_NAME,
		.of_match_table = jd5516w_of_table,
	},
	.probe_new  = jd5516w_probe,
	.remove = jd5516w_remove,
	.id_table = jd5516w_id_table,
};

module_i2c_driver(jd5516w_i2c_driver);

MODULE_AUTHOR("Dongchun Zhu <dongchun.zhu@mediatek.com>");
MODULE_DESCRIPTION("JD5516W VCM driver");
MODULE_LICENSE("GPL v2");
