// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

//#define DEBUG

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>

#include "kd_imgsensor_define_v4l2.h"
#include "adaptor.h"
#include "adaptor-hw.h"

#define INST_OPS(__ctx, __field, __idx, __hw_id, __set, __unset) do {\
	if (__ctx->__field[__idx]) { \
		__ctx->hw_ops[__hw_id].set = __set; \
		__ctx->hw_ops[__hw_id].unset = __unset; \
		__ctx->hw_ops[__hw_id].data = (void *)__idx; \
	} \
} while (0)

static const char * const clk_names[] = {
	ADAPTOR_CLK_NAMES
};

static const char * const reg_names[] = {
	ADAPTOR_REGULATOR_NAMES
};

static const char * const state_names[] = {
	ADAPTOR_STATE_NAMES
};

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static bool sg_aon_power_status = false;
static bool sg_cam_power_status = false;
#endif

static struct clk *get_clk_by_freq(struct adaptor_ctx *ctx, int freq)
{
	switch (freq) {
	case 6:
		return ctx->clk[CLK_6M];
	case 12:
		return ctx->clk[CLK_12M];
	case 13:
		return ctx->clk[CLK_13M];
	case 19:
		return ctx->clk[CLK_19_2M];
	case 24:
		return ctx->clk[CLK_24M];
	case 26:
		return ctx->clk[CLK_26M];
	case 52:
		return ctx->clk[CLK_52M];
	}

	return NULL;
}

static int set_mclk(struct adaptor_ctx *ctx, void *data, int val)
{
	int ret;
	struct clk *mclk, *mclk_src;
	#if defined(OPLUS_FEATURE_CAMERA_COMMON) && defined(CONFIG_OPLUS_CAM_EVENT_REPORT_MODULE)
	unsigned char payload[PAYLOAD_LENGTH] = {0x00};
	scnprintf(payload, sizeof(payload), "NULL$$EventField@@%s$$FieldData@@0x=%x$$detailData@@sn=%s",
		acquireEventField(EXCEP_CLOCK), (CAM_RESERVED_ID << 20 | CAM_MODULE_ID << 12 | EXCEP_CLOCK), ctx->subdrv->name);
	#endif /* OPLUS_FEATURE_CAMERA_COMMON */
	mclk = ctx->clk[CLK_MCLK];
	mclk_src = get_clk_by_freq(ctx, val);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if ((NULL == mclk) || (NULL == mclk_src)) {
		dev_err(ctx->dev, "In %s no clk val %d\n", __func__, val);
		return EFAULT;
	}
#endif

	ret = clk_prepare_enable(mclk);
	if (ret) {
		dev_err(ctx->dev, "failed to enable mclk\n");
		#if defined(OPLUS_FEATURE_CAMERA_COMMON) && defined(CONFIG_OPLUS_CAM_EVENT_REPORT_MODULE)
		cam_olc_raise_exception(EXCEP_CLOCK, payload);
		#endif /* OPLUS_FEATURE_CAMERA_COMMON */
		return ret;
	}

	ret = clk_prepare_enable(mclk_src);
	if (ret) {
		dev_err(ctx->dev, "failed to enable mclk_src\n");
		#if defined(OPLUS_FEATURE_CAMERA_COMMON) && defined(CONFIG_OPLUS_CAM_EVENT_REPORT_MODULE)
		cam_olc_raise_exception(EXCEP_CLOCK, payload);
		#endif /* OPLUS_FEATURE_CAMERA_COMMON */
		return ret;
	}

	ret = clk_set_parent(mclk, mclk_src);
	if (ret) {
		dev_err(ctx->dev, "failed to set mclk's parent\n");
		return ret;
	}

	return 0;
}

static int unset_mclk(struct adaptor_ctx *ctx, void *data, int val)
{
	struct clk *mclk, *mclk_src;

	mclk = ctx->clk[CLK_MCLK];
	mclk_src = get_clk_by_freq(ctx, val);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if ((NULL == mclk) || (NULL == mclk_src)) {
		dev_err(ctx->dev, "In %s no clk val %d\n", __func__, val);
		return -EFAULT;
	}
#endif

	clk_disable_unprepare(mclk_src);
	clk_disable_unprepare(mclk);

	return 0;
}

static int set_reg(struct adaptor_ctx *ctx, void *data, int val)
{
	unsigned long long ret, idx;
	struct regulator *reg;
	#if defined(OPLUS_FEATURE_CAMERA_COMMON) && defined(CONFIG_OPLUS_CAM_EVENT_REPORT_MODULE)
	unsigned char payload[PAYLOAD_LENGTH] = {0x00};
	#endif
	idx = (unsigned long long)data;
	reg = ctx->regulator[idx];

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (NULL == reg) {
		dev_err(ctx->dev, "In %s no regulator\n", __func__);
		return -EFAULT;
	}

	if(val == 0) {
		ret = regulator_disable(reg);
		pr_err("%s  regulator_disable ret:%d", __func__, ret);
		return ret;
	}
#endif
	ret = regulator_set_voltage(reg, val, val);
	if (ret) {
		#ifndef OPLUS_FEATURE_CAMERA_COMMON
		//Add for DFX camera log report
		dev_dbg(ctx->dev, "failed to set voltage %s %d\n",
				reg_names[idx], val);
		#else
		dev_err(ctx->dev, "failed to set voltage %s %d\n",
				reg_names[idx], val);
		#endif /* OPLUS_FEATURE_CAMERA_COMMON */
	}

	ret = regulator_enable(reg);
	if (ret) {
		#ifndef OPLUS_FEATURE_CAMERA_COMMON
		dev_dbg(ctx->dev, "failed to enable %s\n",
				reg_names[idx]);
		#else
		dev_err(ctx->dev, "failed to enable %s\n",
				reg_names[idx]);
		#endif
		#if defined(OPLUS_FEATURE_CAMERA_COMMON) && defined(CONFIG_OPLUS_CAM_EVENT_REPORT_MODULE)
		scnprintf(payload, sizeof(payload), "NULL$$EventField@@%s$$FieldData@@0x%x$$detailData@@sn=%s, reg_names=%s",
			acquireEventField(EXCEP_VOLTAGE), (CAM_RESERVED_ID << 20 | CAM_MODULE_ID << 12 | EXCEP_VOLTAGE),
			ctx->subdrv->name, reg_names[idx]);
		cam_olc_raise_exception(EXCEP_VOLTAGE, payload);
		#endif /* OPLUS_FEATURE_CAMERA_COMMON */
		return ret;
	}

	return 0;
}

static int unset_reg(struct adaptor_ctx *ctx, void *data, int val)
{
	unsigned long long ret, idx;
	struct regulator *reg;

	idx = (unsigned long long)data;
	reg = ctx->regulator[idx];

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (NULL == reg) {
		dev_err(ctx->dev, "In %s no regulator\n", __func__);
		return -EFAULT;
	}
#endif

	ret = regulator_disable(reg);
	if (ret) {
		#ifndef OPLUS_FEATURE_CAMERA_COMMON
		dev_dbg(ctx->dev, "failed to disable %s\n",
				reg_names[idx]);
		#else
		dev_err(ctx->dev, "failed to disable %s\n",
				reg_names[idx]);
		#endif
		return ret;
	}

	return 0;
}

static int set_state(struct adaptor_ctx *ctx, void *data, int val)
{
	unsigned long long idx, x;
	int ret;
	#if defined(OPLUS_FEATURE_CAMERA_COMMON) && defined(CONFIG_OPLUS_CAM_EVENT_REPORT_MODULE)
	unsigned char payload[PAYLOAD_LENGTH] = {0x00};
	#endif /* OPLUS_FEATURE_CAMERA_COMMON */
	idx = (unsigned long long)data;
	x = idx + val;

	#ifndef OPLUS_FEATURE_CAMERA_COMMON
	ret = pinctrl_select_state(ctx->pinctrl, ctx->state[x]);
	#else /* OPLUS_FEATURE_CAMERA_COMMON */
	if (ctx->pinctrl) {
		ret = pinctrl_select_state(ctx->pinctrl, ctx->state[x]);
	} else{
		ret = -1;
		dev_err(ctx->dev, "pinctrl is NULL!\n");
	}
	#endif /* OPLUS_FEATURE_CAMERA_COMMON */

	if (ret < 0) {
		dev_err(ctx->dev, "fail to select %s\n", state_names[x]);
		#if defined(OPLUS_FEATURE_CAMERA_COMMON) && defined(CONFIG_OPLUS_CAM_EVENT_REPORT_MODULE)
		scnprintf(payload, sizeof(payload), "NULL$$EventField@@%s$$FieldData@@0x%x$$detailData@@sn=%s, state_names=%s",
			acquireEventField(EXCEP_GPIO), (CAM_RESERVED_ID << 20 | CAM_MODULE_ID << 12 | EXCEP_GPIO),
			ctx->subdrv->name, state_names[idx]);
		cam_olc_raise_exception(EXCEP_GPIO, payload);
		#endif /* OPLUS_FEATURE_CAMERA_COMMON */
		return ret;
	}

	return 0;
}

static int unset_state(struct adaptor_ctx *ctx, void *data, int val)
{
	return set_state(ctx, data, 0);
}

static int set_state_div2(struct adaptor_ctx *ctx, void *data, int val)
{
	return set_state(ctx, data, val >> 1);
}

static int set_state_boolean(struct adaptor_ctx *ctx, void *data, int val)
{
	return set_state(ctx, data, !!val);
}

static int set_state_mipi_switch(struct adaptor_ctx *ctx, void *data, int val)
{
	return set_state(ctx, (void *)STATE_MIPI_SWITCH_ON, 0);
}

static int unset_state_mipi_switch(struct adaptor_ctx *ctx, void *data,
	int val)
{
	return set_state(ctx, (void *)STATE_MIPI_SWITCH_OFF, 0);
}

static int reinit_pinctrl(struct adaptor_ctx *ctx)
{
	int i;
	struct device *dev = ctx->dev;

	/* pinctrl */
	ctx->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ctx->pinctrl)) {
		dev_err(dev, "fail to get pinctrl\n");
		return PTR_ERR(ctx->pinctrl);
	}

	/* pinctrl states */
	for (i = 0; i < STATE_MAXCNT; i++) {
		ctx->state[i] = pinctrl_lookup_state(
				ctx->pinctrl, state_names[i]);
		if (IS_ERR(ctx->state[i])) {
			ctx->state[i] = NULL;
			dev_dbg(dev, "no state %s\n", state_names[i]);
		}
	}

	return 0;
}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
/*Added by rentianzhi@CamDrv, release the hw resource for Explorer AON driver, 20220124*/
static int reinit_supply(struct adaptor_ctx *ctx)
{
	int i;
	struct device *dev = ctx->dev;
	/* supplies */
	for (i = 0; i < REGULATOR_MAXCNT; i++) {
		ctx->regulator[i] = devm_regulator_get_optional(
				dev, reg_names[i]);
		if (IS_ERR(ctx->regulator[i])) {
			ctx->regulator[i] = NULL;
			dev_err(dev, "In %s:no reg %s\n", __func__, reg_names[i]);
		}
	}
	return 0;
}

static int reinit_clk(struct adaptor_ctx *ctx)
{
	int i;
	struct device *dev = ctx->dev;
	/* clocks */
	for (i = 0; i < CLK_MAXCNT; i++) {
		ctx->clk[i] = devm_clk_get(dev, clk_names[i]);
		if (IS_ERR(ctx->clk[i])) {
			ctx->clk[i] = NULL;
			dev_dbg(dev, "In %s:no clk %s\n", __func__, clk_names[i]);
		}
	}
	return 0;
}

/**
 * @brief aon hardware camera power on; when the function be called, hw_mutex need use before and after functions
 *        mutex_lock(&ctx->hw_mutex);
 *        ````
 *        adaptor_hw_aon_power_on();
 *        ````
 *        mutex_unlock(&ctx->hw_mutex);
 * @param ctx
 * @return int
 */
int adaptor_hw_aon_power_on(struct adaptor_ctx *ctx)
{
    int i;
    const struct subdrv_pw_seq_entry *ent;
    struct adaptor_hw_ops *op;

    /*Added by nielinxin@CamDrv, update cam status, 20230721*/
    if (sg_cam_power_status || sg_aon_power_status) {
	    dev_info(ctx->dev, "[%s] cam(%d)/aon(%d) already powered skip\n",
		     __func__, sg_cam_power_status, sg_aon_power_status);
	    return 0;
    }

    dev_info(ctx->dev, "[%s]+\n", __func__);

    /* may be released for mipi switch */
    if (!ctx->pinctrl)
        reinit_pinctrl(ctx);

    /*Added by rentianzhi@CamDrv, release the hw resource for Explorer AON driver, 20220124*/
    if (ctx->support_explorer_aon_fl == 1 && ctx->idx == IMGSENSOR_SENSOR_IDX_SUB) {
        dev_info(ctx->dev, "In %s:start to reinit supply and clk for front sensor.\n", __func__);
        /*reinit the supply*/
        reinit_supply(ctx);
        /*reinit the clk*/
        reinit_clk(ctx);
    }

    for (i = 0; i < ctx->subdrv->aon_pw_seq_cnt; i++) {
        ent = &ctx->subdrv->aon_pw_seq[i];
        op = &ctx->hw_ops[ent->id];
        if (!op->set) {
            dev_dbg(ctx->dev, "cannot set comp %d val %d\n",
                ent->id, ent->val);
            continue;
        }
        op->set(ctx, op->data, ent->val);
        if (ent->delay)
            mdelay(ent->delay);
    }

    sg_aon_power_status = true;
    dev_info(ctx->dev, "[%s]-\n", __func__);
    return 0;
}

/**
 * @brief aon hardware camera power off; when the function be called, hw_mutex need use before and after functions
 *        mutex_lock(&ctx->hw_mutex);
 *        ````
 *        adaptor_hw_aon_power_off();
 *        ````
 *        mutex_unlock(&ctx->hw_mutex);
 * @param ctx
 * @return int
 */
int adaptor_hw_aon_power_off(struct adaptor_ctx *ctx)
{
    int i;
    const struct subdrv_pw_seq_entry *ent;
    struct adaptor_hw_ops *op;

    if ((true == sg_cam_power_status) || (false == sg_aon_power_status)) {
	    dev_info(ctx->dev, "[%s] cam(%d)/aon(%d) already powered skip\n",
		     __func__, sg_cam_power_status, sg_aon_power_status);
	    return 0;
    }

    dev_info(ctx->dev, "[%s]+\n", __func__);

    for (i = ctx->subdrv->aon_pw_seq_cnt - 1; i >= 0; i--) {
        ent = &ctx->subdrv->aon_pw_seq[i];
        op = &ctx->hw_ops[ent->id];
        if (!op->unset)
            continue;
        if (ent->id == HW_ID_MCLK)
            continue;
        op->unset(ctx, op->data, ent->val);
        //msleep(ent->delay);
    }

    /* the pins of mipi switch are shared. free it for another users */
    if (ctx->state[STATE_MIPI_SWITCH_ON] ||
        ctx->state[STATE_MIPI_SWITCH_OFF]) {
        devm_pinctrl_put(ctx->pinctrl);
        ctx->pinctrl = NULL;
    }

    /*Added by rentianzhi@CamDrv, release the hw resource for Explorer AON driver, 20220124*/
    if (ctx->support_explorer_aon_fl == 1 && ctx->idx == IMGSENSOR_SENSOR_IDX_SUB) {
        dev_info(ctx->dev, "In adaptor_hw_power_off:Start to release the supply and clk resource for front camera.\n");
        /* clocks */
        for (i = 0; i < CLK_MAXCNT; i++) {
            if (ctx->clk[i] != NULL) {
                devm_clk_put(ctx->dev, ctx->clk[i]);
                ctx->clk[i] = NULL;
                dev_dbg(ctx->dev, "In %s:Release clk:%s ok.\n", __func__, clk_names[i]);
            }
        }
        /* supplies */
        for (i = 0; i < REGULATOR_MAXCNT; i++) {
            if (ctx->regulator[i] != NULL) {
                devm_regulator_put(ctx->regulator[i]);
                ctx->regulator[i] = NULL;
                dev_dbg(ctx->dev, "In %s:Release regulator:%s ok.\n", __func__, reg_names[i]);
            }
        }
        /*pinctrl*/
        if (ctx->pinctrl != NULL)
            devm_pinctrl_put(ctx->pinctrl);
        ctx->pinctrl = NULL;
    }

    sg_aon_power_status = false;
    dev_info(ctx->dev, "[%s]-\n", __func__);
    return 0;
}

#endif
int do_hw_power_on(struct adaptor_ctx *ctx)
{
	int i;
	const struct subdrv_pw_seq_entry *ent;
	struct adaptor_hw_ops *op;

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	/*Added by nielinxin@CamDrv, update cam status, 20230721*/
	mutex_lock(&ctx->hw_mutex);
	if (sg_aon_power_status) {
		dev_info(ctx->dev,
			 "[%s] cam(%d)/aon(%d) already powered powerdown aon\n",
			 __func__, sg_cam_power_status, sg_aon_power_status);
		adaptor_hw_aon_power_off(ctx);
	}
#endif

	if (ctx->sensor_ws)
		__pm_stay_awake(ctx->sensor_ws);
	else
		dev_dbg(ctx->dev, "%s fail to __pm_stay_awake\n",
			__func__);
	/* may be released for mipi switch */
	if (!ctx->pinctrl)
		reinit_pinctrl(ctx);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
/*Added by rentianzhi@CamDrv, release the hw resource for Explorer AON driver, 20220124*/
	if (ctx->support_explorer_aon_fl == 1 && ctx->idx == IMGSENSOR_SENSOR_IDX_SUB) {
		dev_info(ctx->dev, "In %s:start to reinit supply and clk for front sensor.\n", __func__);
		/*reinit the supply*/
		reinit_supply(ctx);
		/*reinit the clk*/
		reinit_clk(ctx);
	}
#endif
	op = &ctx->hw_ops[HW_ID_MIPI_SWITCH];
	if (op->set)
		op->set(ctx, op->data, 0);

	for (i = 0; i < ctx->subdrv->pw_seq_cnt; i++) {
		ent = &ctx->subdrv->pw_seq[i];
		op = &ctx->hw_ops[ent->id];
		if (!op->set) {
			dev_dbg(ctx->dev, "cannot set comp %d val %d\n",
				ent->id, ent->val);
			continue;
		}
		op->set(ctx, op->data, ent->val);
		if (ent->delay)
			mdelay(ent->delay);
	}

	if (ctx->subdrv->ops->power_on)
		subdrv_call(ctx, power_on, NULL);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	sg_cam_power_status = true;
	mutex_unlock(&ctx->hw_mutex);
#endif
	//dev_dbg(ctx->dev, "%s\n", __func__);

	return 0;
}

int adaptor_hw_power_on(struct adaptor_ctx *ctx)
{

#ifndef IMGSENSOR_USE_PM_FRAMEWORK
	dev_dbg(ctx->dev, "%s power ref cnt = %d\n", __func__, ctx->power_refcnt);
	ctx->power_refcnt++;
	if (ctx->power_refcnt > 1) {
		dev_dbg(ctx->dev, "%s already powered, cnt = %d\n", __func__, ctx->power_refcnt);
		return 0;
	}
#endif
	return do_hw_power_on(ctx);
}

int do_hw_power_off(struct adaptor_ctx *ctx)
{
	int i;
	const struct subdrv_pw_seq_entry *ent;
	struct adaptor_hw_ops *op;

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	/*Added by nielinxin@CamDrv, update cam status, 20230721*/
	mutex_lock(&ctx->hw_mutex);
	sg_cam_power_status = false;
#endif

	/* call subdrv close function before pwr off */
	subdrv_call(ctx, close);

	if (ctx->subdrv->ops->power_off)
		subdrv_call(ctx, power_off, NULL);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if(ctx->subdrv->pw_off_seq != NULL) {
		for (i = ctx->subdrv->pw_off_seq_cnt - 1; i >= 0; i--) {
			ent = &ctx->subdrv->pw_off_seq[i];
			op = &ctx->hw_ops[ent->id];
			if (!op->unset)
				continue;
			op->unset(ctx, op->data, ent->val);
			//msleep(ent->delay);
		}
	} else {
#endif
		for (i = ctx->subdrv->pw_seq_cnt - 1; i >= 0; i--) {
			ent = &ctx->subdrv->pw_seq[i];
			op = &ctx->hw_ops[ent->id];
			if (!op->unset)
				continue;
			op->unset(ctx, op->data, ent->val);
			//msleep(ent->delay);
		}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	}
#endif
	op = &ctx->hw_ops[HW_ID_MIPI_SWITCH];
	if (op->unset)
		op->unset(ctx, op->data, 0);

	/* the pins of mipi switch are shared. free it for another users */
	if (ctx->state[STATE_MIPI_SWITCH_ON] ||
		ctx->state[STATE_MIPI_SWITCH_OFF]) {
		devm_pinctrl_put(ctx->pinctrl);
		ctx->pinctrl = NULL;
	}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
/*Added by rentianzhi@CamDrv, release the hw resource for Explorer AON driver, 20220124*/
	if (ctx->support_explorer_aon_fl == 1 && ctx->idx == IMGSENSOR_SENSOR_IDX_SUB) {
		dev_info(ctx->dev, "In adaptor_hw_power_off:Start to release the supply and clk resource for front camera.\n");
		/* clocks */
		for (i = 0; i < CLK_MAXCNT; i++) {
			if (ctx->clk[i] != NULL) {
				devm_clk_put(ctx->dev, ctx->clk[i]);
				ctx->clk[i] = NULL;
				dev_dbg(ctx->dev, "In %s:Release clk:%s ok.\n", __func__, clk_names[i]);
			}
		}
		/* supplies */
		for (i = 0; i < REGULATOR_MAXCNT; i++) {
			if (ctx->regulator[i] != NULL) {
				devm_regulator_put(ctx->regulator[i]);
				ctx->regulator[i] = NULL;
				dev_dbg(ctx->dev, "In %s:Release regulator:%s ok.\n", __func__, reg_names[i]);
			}
		}
		/*pinctrl*/
		devm_pinctrl_put(ctx->pinctrl);
		ctx->pinctrl = NULL;
	}
#endif

	if (ctx->sensor_ws)
		__pm_relax(ctx->sensor_ws);
	else
		dev_dbg(ctx->dev, "%s fail to __pm_relax\n",
			__func__);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	mutex_unlock(&ctx->hw_mutex);
#endif
	//dev_dbg(ctx->dev, "%s\n", __func__);
	return 0;

}
int adaptor_hw_power_off(struct adaptor_ctx *ctx)
{

#ifndef IMGSENSOR_USE_PM_FRAMEWORK

	if (!ctx->power_refcnt) {
		dev_dbg(ctx->dev, "%s power ref cnt = %d, skip due to not power on yet\n",
			__func__, ctx->power_refcnt);
		return 0;
	}
	dev_dbg(ctx->dev, "%s power ref cnt = %d\n", __func__, ctx->power_refcnt);
	ctx->power_refcnt--;
	if (ctx->power_refcnt > 0) {
		dev_dbg(ctx->dev, "%s skip due to cnt = %d\n", __func__, ctx->power_refcnt);
		return 0;
	}
	ctx->power_refcnt = 0;
	ctx->is_sensor_inited = 0;
	ctx->is_sensor_scenario_inited = 0;
#endif
	return do_hw_power_off(ctx);
}

int adaptor_hw_init(struct adaptor_ctx *ctx)
{
	int i;
	struct device *dev = ctx->dev;

	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	ctx->subctx.is_esd_enable = false;
	#endif

	/* clocks */
	for (i = 0; i < CLK_MAXCNT; i++) {
		ctx->clk[i] = devm_clk_get(dev, clk_names[i]);
		if (IS_ERR(ctx->clk[i])) {
			ctx->clk[i] = NULL;
			dev_dbg(dev, "no clk %s\n", clk_names[i]);
		}
	}

	/* supplies */
	for (i = 0; i < REGULATOR_MAXCNT; i++) {
		ctx->regulator[i] = devm_regulator_get_optional(
				dev, reg_names[i]);
		if (IS_ERR(ctx->regulator[i])) {
			ctx->regulator[i] = NULL;
			dev_dbg(dev, "no reg %s\n", reg_names[i]);
		}
	}

	/* pinctrl */
	ctx->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ctx->pinctrl)) {
		dev_err(dev, "fail to get pinctrl\n");
		return PTR_ERR(ctx->pinctrl);
	}

	/* pinctrl states */
	for (i = 0; i < STATE_MAXCNT; i++) {
		ctx->state[i] = pinctrl_lookup_state(
				ctx->pinctrl, state_names[i]);
		if (IS_ERR(ctx->state[i])) {
			ctx->state[i] = NULL;
			dev_dbg(dev, "no state %s\n", state_names[i]);
		}
	}

	/* install operations */

	INST_OPS(ctx, clk, CLK_MCLK, HW_ID_MCLK, set_mclk, unset_mclk);

	INST_OPS(ctx, regulator, REGULATOR_AVDD, HW_ID_AVDD,
			set_reg, unset_reg);

	INST_OPS(ctx, regulator, REGULATOR_DVDD, HW_ID_DVDD,
			set_reg, unset_reg);

	INST_OPS(ctx, regulator, REGULATOR_DOVDD, HW_ID_DOVDD,
			set_reg, unset_reg);

	INST_OPS(ctx, regulator, REGULATOR_AFVDD, HW_ID_AFVDD,
			set_reg, unset_reg);

	INST_OPS(ctx, regulator, REGULATOR_AVDD1, HW_ID_AVDD1,
			set_reg, unset_reg);

	if (ctx->state[STATE_MIPI_SWITCH_ON])
		ctx->hw_ops[HW_ID_MIPI_SWITCH].set = set_state_mipi_switch;

	if (ctx->state[STATE_MIPI_SWITCH_OFF])
		ctx->hw_ops[HW_ID_MIPI_SWITCH].unset = unset_state_mipi_switch;

	INST_OPS(ctx, state, STATE_MCLK_OFF, HW_ID_MCLK_DRIVING_CURRENT,
			set_state_div2, unset_state);

	INST_OPS(ctx, state, STATE_RST_LOW, HW_ID_RST,
			set_state, unset_state);

	INST_OPS(ctx, state, STATE_PDN_LOW, HW_ID_PDN,
			set_state, unset_state);

	INST_OPS(ctx, state, STATE_AVDD_OFF, HW_ID_AVDD,
			set_state_boolean, unset_state);

	INST_OPS(ctx, state, STATE_DVDD_OFF, HW_ID_DVDD,
			set_state_boolean, unset_state);

	INST_OPS(ctx, state, STATE_DOVDD_OFF, HW_ID_DOVDD,
			set_state_boolean, unset_state);

	INST_OPS(ctx, state, STATE_AFVDD_OFF, HW_ID_AFVDD,
			set_state_boolean, unset_state);

	INST_OPS(ctx, state, STATE_AVDD1_OFF, HW_ID_AVDD1,
			set_state_boolean, unset_state);

	/* the pins of mipi switch are shared. free it for another users */
	if (ctx->state[STATE_MIPI_SWITCH_ON] ||
		ctx->state[STATE_MIPI_SWITCH_OFF]) {
		devm_pinctrl_put(ctx->pinctrl);
		ctx->pinctrl = NULL;
	}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
/*Added by rentianzhi@CamDrv, release the hw resource for Explorer AON driver, 20220124*/
	if (ctx->support_explorer_aon_fl == 1 && ctx->idx == IMGSENSOR_SENSOR_IDX_SUB) {
		dev_info(ctx->dev, "In adaptor_hw_power_off:Start to release the supply and clk resource for front camera.\n");
		/* clocks */
		for (i = 0; i < CLK_MAXCNT; i++) {
			if (ctx->clk[i] != NULL) {
				devm_clk_put(ctx->dev, ctx->clk[i]);
				ctx->clk[i] = NULL;
				dev_dbg(ctx->dev, "In %s:Release clk:%s ok.\n", __func__, clk_names[i]);
			}
		}
		/* supplies */
		for (i = 0; i < REGULATOR_MAXCNT; i++) {
			if (ctx->regulator[i] != NULL) {
				devm_regulator_put(ctx->regulator[i]);
				ctx->regulator[i] = NULL;
				dev_dbg(ctx->dev, "In %s:Release regulator:%s ok.\n", __func__, reg_names[i]);
			}
		}
		/*pinctrl*/
		devm_pinctrl_put(ctx->pinctrl);
		ctx->pinctrl = NULL;
	}
	sg_cam_power_status = false;
#endif
	return 0;
}

int adaptor_hw_sensor_reset(struct adaptor_ctx *ctx)
{
	dev_info(ctx->dev, "%s %d|%d|%d\n",
		__func__,
		ctx->is_streaming,
		ctx->is_sensor_inited,
		ctx->power_refcnt);

	if (ctx->is_streaming == 1 &&
		ctx->is_sensor_inited == 1 &&
		ctx->power_refcnt > 0) {

		do_hw_power_off(ctx);
		do_hw_power_on(ctx);

		#ifdef OPLUS_FEATURE_CAMERA_COMMON
		ctx->subctx.is_esd_enable = true;
		#endif

		return 0;
	}
	dev_info(ctx->dev, "%s skip to reset due to either integration or else\n",
		__func__);

	return -1;
}


