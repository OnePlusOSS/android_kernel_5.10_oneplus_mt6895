// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/sched/clock.h>
#include <linux/delay.h>
#include <uapi/linux/sched/types.h>
#include <linux/pinctrl/consumer.h>

#ifndef DRM_CMDQ_DISABLE
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#else
#include "mtk-cmdq-ext.h"
#endif
#if defined(CONFIG_PXLW_IRIS)
#include "iris_mtk_api.h"
#include "iris_api.h"
#endif
#include "mtk_drm_drv.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_helper.h"
#include "mtk_drm_assert.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_trace.h"
#include "mtk_dsi.h"
#include <soc/oplus/system/oplus_project.h>

//#ifdef OPLUS_FEATURE_DISPLAY
#include <mt-plat/mtk_boot_common.h>
//#endif /* OPLUS_FEATURE_DISPLAY */
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
#include "oplus_display_temp_compensation.h"
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

#define ESD_TRY_CNT 5
//#ifndef OPLUS_FEATURE_ESD
//#define ESD_CHECK_PERIOD 2000 /* ms */
//#else
#define ESD_CHECK_PERIOD 5000 /* ms */
#define TIMEOUT_MS 20
static DEFINE_MUTEX(pinctrl_lock);
extern unsigned int esd_mode;
extern unsigned int oplus_esd_check_read_mode;
extern unsigned int ffl_backlight_backup;
unsigned long esd_flag = 0;
EXPORT_SYMBOL(esd_flag);

/* #ifdef OPLUS_FEATURE_DISPLAY */
bool read_ddic_once = true;
extern void ddic_dsi_read_cmd_test(unsigned int case_num);
extern int oplus_get_panel_serial_number_interface(struct mtk_dsi *mtk_dsi, int level);
static count_irq_sta = 1;
unsigned int dsi0te_err = 1;
extern void gpio_dump_regs_range(int start, int end);
extern unsigned int get_project(void);
/* #endif  */
extern bool panel_is_aries(void);
extern bool panel_is_sonic(void);
extern bool panel_is_sonic_s(void);
/* pinctrl implementation */
long _set_state(struct drm_crtc *crtc, const char *name)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct pinctrl_state *pState = 0;
	long ret = 0;

	mutex_lock(&pinctrl_lock);
	if (!priv->pctrl) {
		DDPPR_ERR("this pctrl is null\n");
		ret = -1;
		goto exit;
	}

	pState = pinctrl_lookup_state(priv->pctrl, name);
	if (IS_ERR(pState)) {
		DDPPR_ERR("lookup state '%s' failed\n", name);
		ret = PTR_ERR(pState);
		goto exit;
	}

	/* select state! */
	DDPMSG("%s %s\n", __func__, name);
	ret = pinctrl_select_state(priv->pctrl, pState);
	if (ret)
		DDPPR_ERR("%s set %s fail:%d\n", __func__, name, ret);

exit:
	mutex_unlock(&pinctrl_lock);
	return ret; /* Good! */
#else
	return 0; /* Good! */
#endif
}

long disp_dts_gpio_init(struct device *dev, struct mtk_drm_private *private)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
	long ret = 0;
	struct pinctrl *pctrl;

	/* retrieve */
	pctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pctrl)) {
		DDPPR_ERR("Cannot find disp pinctrl!");
		ret = PTR_ERR(pctrl);
		goto exit;
	}

	private->pctrl = pctrl;

exit:
	return ret;
#else
	return 0;
#endif
}

static inline int _can_switch_check_mode(struct drm_crtc *crtc,
					 struct mtk_panel_ext *panel_ext)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	int ret = 0;

	if (panel_ext->params->cust_esd_check == 0 &&
	    panel_ext->params->lcm_esd_check_table[0].cmd != 0 &&
	    mtk_drm_helper_get_opt(priv->helper_opt,
				   MTK_DRM_OPT_ESD_CHECK_SWITCH))
		ret = 1;

	return ret;
}

static inline int _lcm_need_esd_check(struct mtk_panel_ext *panel_ext)
{
	int ret = 0;

	/*#ifdef OPLUS_FEATURE_DISPLAY*/
	if (panel_ext->params->esd_check_enable == 1) {
		ret = 1;
	}
	/*#endif*/

	return ret;
}

static inline int need_wait_esd_eof(struct drm_crtc *crtc,
				    struct mtk_panel_ext *panel_ext)
{
	int ret = 1;

	/*
	 * 1.vdo mode
	 * 2.cmd mode te
	 */
	if (!mtk_crtc_is_frame_trigger_mode(crtc))
		ret = 0;

	if (panel_ext->params->cust_esd_check == 0)
		ret = 0;

	return ret;
}

static void esd_cmdq_timeout_cb(struct cmdq_cb_data data)
{
	struct drm_crtc *crtc = data.data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct cmdq_client *cl;
	dma_addr_t trig_pc = 0;
	u64 *inst;

	if (!crtc) {
		DDPMSG("%s find crtc fail\n", __func__);
		return;
	}

	DDPMSG("read flush fail\n");
	esd_ctx->chk_sta = 0xff;
	mtk_drm_crtc_analysis(crtc);
	mtk_drm_crtc_dump(crtc);

	if ((mtk_crtc->trig_loop_cmdq_handle) &&
			(mtk_crtc->trig_loop_cmdq_handle->cl)) {
		cl = (struct cmdq_client *)mtk_crtc->trig_loop_cmdq_handle->cl;

		DDPMSG("++++++ Dump trigger loop ++++++\n");
		cmdq_thread_dump(cl->chan, mtk_crtc->trig_loop_cmdq_handle,
				&inst, &trig_pc);
		cmdq_dump_pkt(mtk_crtc->trig_loop_cmdq_handle, trig_pc, true);

		DDPMSG("------ Dump trigger loop ------\n");
	} else {
		DDPMSG("------ %s No valid trigger loop ------\n", __func__);
	}

	if ((mtk_crtc->event_loop_cmdq_handle) &&
			(mtk_crtc->event_loop_cmdq_handle->cl)) {
		cl = (struct cmdq_client *)mtk_crtc->event_loop_cmdq_handle->cl;

		DDPMSG("++++++ Dump event loop ++++++\n");
		cmdq_thread_dump(cl->chan, mtk_crtc->event_loop_cmdq_handle,
				&inst, &trig_pc);
		cmdq_dump_pkt(mtk_crtc->event_loop_cmdq_handle, trig_pc, true);

		DDPMSG("------ Dump event loop ------\n");
	}
}


int _mtk_esd_check_read(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp = NULL;
	struct mtk_panel_ext *panel_ext = NULL;
	struct cmdq_pkt *cmdq_handle = NULL, *cmdq_handle2 = NULL;
	struct mtk_drm_esd_ctx *esd_ctx = NULL;
	int ret = 0;
	oplus_esd_check_read_mode = 1;

	DDPINFO("[ESD%u]ESD read panel\n", drm_crtc_index(crtc));


	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s:invalid output comp\n", __func__);
		return -EINVAL;
	}

	if (mtk_drm_is_idle(crtc) && mtk_dsi_is_cmd_mode(output_comp))
		return 0;

	mtk_ddp_comp_io_cmd(output_comp, NULL, REQ_PANEL_EXT, &panel_ext);
	if (unlikely(!(panel_ext && panel_ext->params))) {
		DDPPR_ERR("%s:can't find panel_ext handle\n", __func__);
		return -EINVAL;
	}

	cmdq_handle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	cmdq_handle->err_cb.cb = esd_cmdq_timeout_cb;
	cmdq_handle->err_cb.data = crtc;

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 1);

	if (mtk_dsi_is_cmd_mode(output_comp)) {
		/* frame done gce event revise, fix by Faker at 2022/10/31 */
		cmdq_pkt_wfe(cmdq_handle,
				     mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);

		if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_SECOND_PATH, 1);
		else
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_FIRST_PATH, 1);
		/* frame done gce event revise, end */

		//#ifdef OPLUS_BUG_STATABILITY
		cmdq_pkt_clear_event(cmdq_handle,
				     mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		//#endif

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
		if (oplus_temp_compensation_is_supported()) {
			oplus_temp_compensation_temp_check(output_comp, cmdq_handle);
		}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, ESD_CHECK_READ,
				    (void *)mtk_crtc);

		//#ifdef OPLUS_BUG_STATABILITY
		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		/* frame done gce event revise, fix by Faker at 2022/10/31 */
		cmdq_pkt_set_event(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
		/* frame done gce event revise, end */
		//#endif
	} else { /* VDO mode */
		if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_SECOND_PATH, 1);
		else
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_FIRST_PATH, 1);

		if (mtk_crtc->msync2.msync_on) {
			u32 vfp_early_stop = 1;

			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, DSI_VFP_EARLYSTOP,
							&vfp_early_stop);
		}

		if (!panel_is_aries() || (panel_ext->params->esd_te_check_gpio == 0)) {
			CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 2);

			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, DSI_STOP_VDO_MODE,
					NULL);

			CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 3);


			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, ESD_CHECK_READ,
					(void *)mtk_crtc);

			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
					DSI_START_VDO_MODE, NULL);

			mtk_disp_mutex_trigger(mtk_crtc->mutex[0], cmdq_handle);
			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, COMP_REG_START,
					NULL);
		}
	}
	esd_ctx = mtk_crtc->esd_ctx;
	esd_ctx->chk_sta = 0;

	//#ifndef OPLUS_FEATURE_ESD
	//cmdq_pkt_flush(cmdq_handle);
	//#else
	ret = cmdq_pkt_flush(cmdq_handle);
	if (ret != 0) {
		pr_err("%s: error esd read flush process\n", __func__);
		goto done;
	}
	//#endif

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 4);


	mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_READ_EPILOG,
				    NULL);
	if (esd_ctx->chk_sta == 0xff) {
		ret = -1;
		if (need_wait_esd_eof(crtc, panel_ext)) {
			/* TODO: set ESD_EOF event through CPU is better */
			mtk_crtc_pkt_create(&cmdq_handle2, crtc,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);

			cmdq_pkt_set_event(
				cmdq_handle2,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
			cmdq_pkt_flush(cmdq_handle2);
			cmdq_pkt_destroy(cmdq_handle2);
		}
		goto done;
	}

	ret = mtk_ddp_comp_io_cmd(output_comp, NULL, ESD_CHECK_CMP,
				  (void *)mtk_crtc);
	#ifdef OPLUS_FEATURE_DISPLAY
	if (ret != 0) {
		if (output_comp->id == DDP_COMPONENT_DSI0) {
			dsi0te_err = 0;
		}
	}
	#endif
done:
	cmdq_pkt_destroy(cmdq_handle);
	return ret;
}

static irqreturn_t _esd_check_ext_te_irq_handler(int irq, void *data)
{
	struct mtk_drm_esd_ctx *esd_ctx = (struct mtk_drm_esd_ctx *)data;

	atomic_set(&esd_ctx->ext_te_event, 1);
	wake_up_interruptible(&esd_ctx->ext_te_wq);

	return IRQ_HANDLED;
}
#ifdef OPLUS_FEATURE_DISPLAY
static void esd_read_cpu(struct mtk_drm_crtc *mtk_crtc)
{
	struct mtk_ddp_comp *output_comp;
	int enable_readic = 1;
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s:invalid output comp\n", __func__);
		return;
	}
	if (output_comp->id == DDP_COMPONENT_DSI0) {
		gpio_dump_regs_range(119, 121);
		gpio_dump_regs_range(21, 23);
		gpio_dump_regs_range(19, 21);
		gpio_dump_regs_range(40, 42);
	}
	/* only work at frame trigger mode */
	if (!mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
		return;

	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);

		DDPMSG("%s, READ_EINT fail stop cancel all gce jobs\n", __func__);

		if (mtk_crtc->gce_obj.client[CLIENT_CFG])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_DSI_CFG])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_SUB_CFG])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SUB_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP]);
		if (mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP]);

	if (output_comp->id == DDP_COMPONENT_DSI0) {
		if (dsi0te_err == 0) {
			enable_readic = 0;
		}
		mtk_ddp_comp_io_cmd(output_comp, NULL, OPLUS_GET_INFO, &enable_readic);
		dsi0te_err = 1;
	}

}
#endif
static int _mtk_esd_check_eint(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	int ret = 1;
	oplus_esd_check_read_mode = 0;

	DDPINFO("[ESD%u]ESD check eint\n", drm_crtc_index(crtc));

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s:invalid ESD context\n", __func__);
		return -EINVAL;
	}

	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_DUAL_TE) &&
			(atomic_read(&mtk_crtc->d_te.te_switched) == 1))
		atomic_set(&mtk_crtc->d_te.esd_te1_en, 1);
	else
		enable_irq(esd_ctx->eint_irq);

	/* check if there is TE in the last 2s, if so ESD check is pass */
	if (panel_is_sonic() || panel_is_sonic_s()){
		if (wait_event_interruptible_timeout(
				esd_ctx->ext_te_wq,
				atomic_read(&esd_ctx->ext_te_event),
				HZ/2) > 0){
			atomic_set(&esd_ctx->ext_te_event, 0);
			if (wait_event_interruptible_timeout(
					esd_ctx->ext_te_wq,
					atomic_read(&esd_ctx->ext_te_event),
					HZ/2) > 0) {
					ret = 0;
			}
		}
	} else {
	if (wait_event_interruptible_timeout(
		    esd_ctx->ext_te_wq,
		    atomic_read(&esd_ctx->ext_te_event),
		    HZ / 2) > 0)
		ret = 0;
	} /* panel_is_sonic() panel_is_sonic_s()end */

	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_DUAL_TE) &&
			(atomic_read(&mtk_crtc->d_te.te_switched) == 1))
		atomic_set(&mtk_crtc->d_te.esd_te1_en, 0);
	else
		disable_irq(esd_ctx->eint_irq);
	atomic_set(&esd_ctx->ext_te_event, 0);

	return ret;
}

static void mtk_drm_release_esd_eint(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_ddp_comp *output_comp;

	if (unlikely(esd_ctx->eint_irq == -1)) {
		DDPPR_ERR("%s %u release eint_irq %d\n",
			__func__, drm_crtc_index(crtc), esd_ctx->eint_irq);
		return;
	}
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (IS_ERR_OR_NULL(output_comp)) {
		DDPPR_ERR("%s null output_comp\n", __func__);
		return;
	}

	free_irq(esd_ctx->eint_irq, esd_ctx);
	/*
	 * TE pinmux HW reg would be changed after free_irq, need to change pinctrl
	 * sw state as well. for next time set state to TE mode would not be skipped
	 * at the pinctrl_select_state.
	 */
	if (output_comp->id == DDP_COMPONENT_DSI0)
		_set_state(crtc, "mode_te_gpio");
	else
		_set_state(crtc, "mode_te_gpio1");

	esd_ctx->eint_irq = -1;
}

static int mtk_drm_request_eint(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_ddp_comp *output_comp;
	struct device_node *node;
	u32 ints[2] = {0, 0};
	char *compat_str = NULL;
	int ret = 0, retry = 5;

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s:invalid ESD context\n", __func__);
		return -EINVAL;
	}

	if (unlikely(esd_ctx->eint_irq != -1)) {
		DDPPR_ERR("%s: reentry with inited eint_irq %d\n", __func__, esd_ctx->eint_irq);
		return -EINVAL;
	}

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s:invalid output comp\n", __func__);
		return -EINVAL;
	}

	mtk_ddp_comp_io_cmd(output_comp, NULL, REQ_ESD_EINT_COMPAT,
			    &compat_str);
	if (unlikely(!compat_str)) {
		DDPPR_ERR("%s: invalid compat string\n", __func__);
		return -EINVAL;
	}
	node = of_find_compatible_node(NULL, NULL, compat_str);
	if (unlikely(!node)) {
		DDPPR_ERR("can't find ESD TE eint compatible node %s\n", compat_str);
		return -EINVAL;
	}

	of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
	esd_ctx->eint_irq = irq_of_parse_and_map(node, 0);

	while((ret = request_irq(esd_ctx->eint_irq, _esd_check_ext_te_irq_handler,
			  IRQF_TRIGGER_RISING, "ESD_TE-eint", esd_ctx)) && retry)
	{
		DDPPR_ERR("eint irq line %u not available! %d\n", esd_ctx->eint_irq, ret);
		retry--;
		msleep(5);
	}
	if (ret) {
		DDPPR_ERR("irq not available! %d\n", ret);
		return ret;
	}

	disable_irq(esd_ctx->eint_irq);
	DDPPR_ERR("[OPLUS]request count_irq_sta = %d, esd_ctx->eint_irq= %u for debug\n", ++count_irq_sta, esd_ctx->eint_irq);

	/* mode_te_te1 mapping to non-primary display's TE */
	if (output_comp->id == DDP_COMPONENT_DSI0)
		_set_state(crtc, "mode_te_te");
	else
		_set_state(crtc, "mode_te_te1");

	/* in order to not all project DTS assign pinctrl mode 'mode_te_gpio', */
	/* add flag to decide need release eint during ESD check switch. */
	if (esd_ctx->need_release_eint == -1) {
		struct mtk_drm_private *priv = crtc->dev->dev_private;
		struct pinctrl_state *pState;

		if (!(priv && priv->pctrl)) {
			esd_ctx->need_release_eint = 0;
			return ret;
		}

		pState = pinctrl_lookup_state(priv->pctrl, "mode_te_gpio");
		if (IS_ERR(pState)) {
			esd_ctx->need_release_eint = 0;
			return ret;
		}

		pState = pinctrl_lookup_state(priv->pctrl, "mode_te_gpio1");
		if (IS_ERR(pState)) {
			esd_ctx->need_release_eint = 0;
			return ret;
		}
		esd_ctx->need_release_eint = 1;
	}

	return ret;
}

static int mtk_drm_esd_check(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_ext *panel_ext;
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	int ret = 0;
#ifdef OPLUS_FEATURE_DISPLAY
	unsigned int prj_id = get_project();
#endif

	CRTC_MMP_EVENT_START(drm_crtc_index(crtc), esd_check, 0, 0);

	if (mtk_crtc->enabled == 0) {
		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 0, 99);
		DDPINFO("[ESD] CRTC %d disable. skip esd check\n",
			drm_crtc_index(crtc));
		goto done;
	}

	panel_ext = mtk_crtc->panel_ext;
	if (unlikely(!(panel_ext && panel_ext->params))) {
		DDPPR_ERR("can't find panel_ext handle\n");
		ret = -EINVAL;
		goto done;
	}

	/* Check panel EINT */
	if (panel_ext->params->cust_esd_check == 0 &&
	    esd_ctx->chk_mode == READ_EINT) {
		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 1, 0);
		ret = _mtk_esd_check_eint(crtc);
	} else { /* READ LCM CMD  */
		if (panel_is_sonic_s()){
			ret = 0;
			DDPPR_ERR("[ESD] READ LCM CMD\n");
		}else{
			CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 0);
			ret = _mtk_esd_check_read(crtc);
		}
	}

	/* switch ESD check mode */
	//#ifndef OPLUS_FEATURE_ESD
	//if (_can_switch_check_mode(crtc, panel_ext) &&
	    //!mtk_crtc_is_frame_trigger_mode(crtc))
	//#else
	if (_can_switch_check_mode(crtc, panel_ext))
	//#endif
		esd_ctx->chk_mode =
			(esd_ctx->chk_mode == READ_EINT) ? READ_LCM : READ_EINT;

#ifdef OPLUS_FEATURE_DISPLAY
	if((prj_id == 22017 || prj_id == 23081) && (get_eng_version() == AGING)) {
		esd_ctx->chk_mode = READ_EINT;
	}
#endif

done:
	CRTC_MMP_EVENT_END(drm_crtc_index(crtc), esd_check, 0, ret);
	return ret;
}

static int mtk_drm_esd_recover(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	struct cmdq_pkt *cmdq_handle = NULL;
	int ret = 0;

	CRTC_MMP_EVENT_START(drm_crtc_index(crtc), esd_recovery, 0, 0);
	if (crtc->state && !crtc->state->active) {
		DDPMSG("%s: crtc is inactive\n", __func__);
		return 0;
	}
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s: invalid output comp\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 1);
	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		DDPMSG("%s, stop cancel all gce jobs\n", __func__);
		if (mtk_crtc->gce_obj.client[CLIENT_CFG])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_DSI_CFG])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_SUB_CFG])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SUB_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP]);
		if (mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP])
			cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP]);
	}
	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);
	if (IS_ERR_OR_NULL(cmdq_handle)) {
		DDPPR_ERR("%s: invalid output comp\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery,
		(unsigned long)cmdq_handle, 1);

	/* flush cmdq with stop_vdo_mode before it set DSI_START to 0 */
	if (mtk_crtc->is_mml) {
		DDPMSG("%s, %d mml stop\n", __func__, __LINE__);
		mtk_crtc_mml_racing_stop_sync(crtc, cmdq_handle);
	}

	if (mtk_crtc_is_frame_trigger_mode(crtc))
		mtk_ddp_comp_io_cmd(output_comp, NULL,
			CONNECTOR_PANEL_DISABLE_NOWAIT, NULL);
	else
		mtk_ddp_comp_io_cmd(output_comp, NULL,
			CONNECTOR_PANEL_DISABLE, NULL);

	mtk_gce_backup_slot_save(mtk_crtc, __func__);
	if (mtk_crtc_is_frame_trigger_mode(crtc))
		mtk_drm_crtc_disable(crtc, false, true);
	else
		mtk_drm_crtc_disable(crtc, true, true);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 2);

	if (mtk_drm_helper_get_opt(priv->helper_opt,
		MTK_DRM_OPT_MMQOS_SUPPORT)) {
		if (drm_crtc_index(crtc) == 0)
			mtk_disp_set_hrt_bw(mtk_crtc,
				mtk_crtc->qos_ctx->last_hrt_req);
	}

	mtk_drm_crtc_enable(crtc, true);
	mtk_gce_backup_slot_restore(mtk_crtc, __func__);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 3);

	if (mtk_crtc->is_mml)
		mtk_crtc_mml_racing_resubmit(crtc, NULL);

	mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_PANEL_ENABLE, NULL);

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 4);

	mtk_crtc_hw_block_ready(crtc);
	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_ESD_EOF]);

		cmdq_pkt_flush(cmdq_handle);
	}
	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 5);

	cmdq_pkt_destroy(cmdq_handle);
done:
	CRTC_MMP_EVENT_END(drm_crtc_index(crtc), esd_recovery, 0, ret);

	return 0;
}

static int mtk_drm_esd_check_worker_kthread(void *data)
{
	struct sched_param param = {.sched_priority = 87};
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct mtk_drm_private *private = NULL;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct mtk_drm_esd_ctx *esd_ctx = NULL;
	int ret = 0;
	int i = 0;
	int recovery_flg = 0;
	bool check_te = false, te_timeout = false;
	unsigned int crtc_idx;

//#ifdef OPLUS_FEATURE_DISPLAY
		struct mtk_ddp_comp *comp = NULL;
		struct mtk_dsi *dsi = NULL;
		bool is_doze_mode = false;
//#endif
#ifdef OPLUS_FEATURE_DISPLAY
	unsigned int prj_id = get_project();
#endif

	sched_setscheduler(current, SCHED_RR, &param);

	if (!crtc) {
		DDPPR_ERR("%s invalid CRTC context, stop thread\n", __func__);

		return -EINVAL;
	}

	private = crtc->dev->dev_private;
	mtk_crtc = to_mtk_crtc(crtc);
	esd_ctx = mtk_crtc->esd_ctx;
	crtc_idx = drm_crtc_index(crtc);
//#ifdef OPLUS_FEATURE_DISPLAY
	comp = mtk_ddp_comp_request_output(mtk_crtc);
	dsi = container_of(comp, struct mtk_dsi, ddp_comp);
//#endif


	while (1) {
//#ifdef OPLUS_FEATURE_DISPLAY
		if (mtk_crtc->panel_ext->params->use_free_pointer_check) {
			DDPINFO("[ESD]mtk_drm_esd_check_worker_kthread use-after-free check\n");
			if (crtc_idx) {
				continue;
			}
			comp = mtk_ddp_comp_request_output(mtk_crtc);
			if (!(comp->funcs) || !(crtc->state)) {
				continue;
			}
		}
//#endif
		if (panel_is_sonic() || panel_is_sonic_s()){
			msleep(3000);
		}
		else
			msleep(ESD_CHECK_PERIOD);
		if (!panel_is_aries() && !mtk_crtc->panel_ext->params->esd_te_check_gpio) {
			//#ifndef OPLUS_FEATURE_ESD
			/*ret = wait_event_interruptible(
				esd_ctx->check_task_wq,
				atomic_read(&esd_ctx->check_wakeup) &&
				(atomic_read(&esd_ctx->target_time) ||
				(panel_ext->params->cust_esd_check == 0) &&
				 (esd_ctx->chk_mode == READ_EINT)));*/
			//#else
			ret = wait_event_interruptible_timeout(
				esd_ctx->check_task_wq,
				atomic_read(&esd_ctx->check_wakeup) &&
				(atomic_read(&esd_ctx->target_time) ||
					esd_ctx->chk_mode == READ_EINT), msecs_to_jiffies(TIMEOUT_MS));
			//#endif

			//#ifndef OPLUS_BUG_STABILITY
			//if (ret < 0) {
			//#else
			if (ret < 0 || (ffl_backlight_backup == 0 || ffl_backlight_backup == 1)){
			//#endif
				DDPINFO("[ESD]check thread waked up accidently\n");
				continue;
			}
		} else {
			// Aries add i2c backlight=0 judge
		}

		if (!mtk_crtc->panel_ext->params->move_esd_readdate_back) {
			if (read_ddic_once && comp && dsi) {
				if (mtk_dsi_is_cmd_mode(comp)) {
					DDPMSG("[ESD] get_boot_mode() is %d\n", get_boot_mode());
					DDPMSG("[ESD] Read DDIC lcm_id1 0xDA valve\n");
					ddic_dsi_read_cmd_test(5);
					DDPMSG("[ESD] Read DDIC lcm_id2 0xDB valve\n");
					ddic_dsi_read_cmd_test(6);
					DDPMSG("[ESD] Read DDIC 0xDC valve\n");
					ddic_dsi_read_cmd_test(7);
					read_ddic_once = false;
					DDPMSG("[ESD] get panel serial number\n");
					oplus_get_panel_serial_number_interface(dsi, 2);
				}
			}
		}

//#ifdef OPLUS_FEATURE_DISPLAY
                /* check doze mode in usual scene */
		comp = mtk_ddp_comp_request_output(mtk_crtc);
		if (!(comp && comp->funcs && comp->funcs->io_cmd))
				return -EINVAL;
		if (crtc->state && crtc->state->enable) {
			comp->funcs->io_cmd(comp, NULL, DSI_GET_AOD_STATE, &is_doze_mode);
			pr_err("[ESD]check doze mode=%d\n",is_doze_mode);
			if (is_doze_mode) {
					pr_err("[ESD] is in aod doze mode, skip esd check!\n");
					continue;
			}
		}
//#endif

		if (esd_ctx->chk_en == 0)
			continue;

		if (mtk_crtc_is_frame_trigger_mode(crtc) &&
			esd_ctx->chk_mode == READ_LCM)
			check_te = true;

		te_timeout = false;
		if (check_te == true) {
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
		if (oplus_temp_compensation_is_supported()) {
			oplus_temp_compensation_data_update();
		}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */
			ret = wait_event_interruptible(
				esd_ctx->check_task_wq,
				atomic_read(&esd_ctx->check_wakeup));
			mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 1);
			atomic_set(&esd_ctx->int_te_event, 0);
			ret = wait_event_interruptible_timeout(
				esd_ctx->int_te_wq,
				atomic_read(&esd_ctx->int_te_event) ||
				atomic_read(&esd_ctx->check_wakeup) == 0,
				HZ);
			DDPINFO("%s, wait te time:%d, esd:%d\n",
				__func__, HZ - ret,  atomic_read(&esd_ctx->check_wakeup));
			if (ret < 0) {
				DDPINFO("[ESD]check thread waked up accidently\n");
				continue;
			}
			if (ret == 0 && esd_ctx->chk_active) {
				DDPPR_ERR("%s: internal TE time out:%d, ret:%llu, esd:%d\n",
					__func__, HZ, ret,
					atomic_read(&esd_ctx->check_wakeup));
				te_timeout = true;
		//#ifndef OPLUS_BUG_STABILITY
				esd_flag = 1;
		//#endif
				DDPMSG("%s, stop cancel all gce jobs\n", __func__);
				if (mtk_crtc->gce_obj.client[CLIENT_CFG])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_CFG]);
				if (mtk_crtc->gce_obj.client[CLIENT_DSI_CFG])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);
				if (mtk_crtc->gce_obj.client[CLIENT_SUB_CFG])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SUB_CFG]);
				if (mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP]);
				if (mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP])
					cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP]);
			}
		} else {
			if (panel_is_aries() || mtk_crtc->panel_ext->params->esd_te_check_gpio) {
				ret = wait_event_interruptible_timeout(
						esd_ctx->check_task_wq,
						atomic_read(&esd_ctx->check_wakeup) &&
						(atomic_read(&esd_ctx->target_time) ||
						esd_ctx->chk_mode == READ_EINT), msecs_to_jiffies(TIMEOUT_MS));
			} else {
				ret = wait_event_interruptible(
					esd_ctx->check_task_wq,
					atomic_read(&esd_ctx->check_wakeup) &&
					(atomic_read(&esd_ctx->target_time) ||
					 esd_ctx->chk_mode == READ_EINT));
			}
			if (ret < 0) {
				DDPINFO("[ESD]check thread waked up accidently\n");
				continue;
			}
		}

		/* #ifdef OPLUS_FEATURE_DISPLAY */
		if (atomic_read(&esd_ctx->target_flag)) {
			DDPDBG("[ESD] %d, target_flag:%d\n", __LINE__,atomic_read(&esd_ctx->target_flag));
			msleep(ESD_CHECK_PERIOD/2);
			atomic_set(&esd_ctx->target_flag, 0);
			DDPDBG("[ESD] %d, target_flag:%d\n", __LINE__,atomic_read(&esd_ctx->target_flag));
			//#ifdef OPLUS_FEATURE_DISPLAY
                        /* check doze mode in unusual scene when doze_enable happens during ESD_CHECK_PERIOD/2 */
			if (crtc->state && crtc->state->enable) {
				comp->funcs->io_cmd(comp, NULL, DSI_GET_AOD_STATE, &is_doze_mode);
				pr_err("[ESD]check again doze mode=%d\n", is_doze_mode);
				if (is_doze_mode) {
						pr_err("[ESD] is in aod doze mode, again skip esd check!\n");
						continue;
				}
			}
			//#endif
		}
		/* #endif */

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
		if (oplus_temp_compensation_is_supported()) {
			oplus_temp_compensation_get_ntc_temp();
		}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

		mutex_lock(&private->commit.lock);
		DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
		mtk_drm_trace_begin("esd");
		if (!mtk_drm_is_idle(crtc))
			atomic_set(&esd_ctx->target_time, 0);

		/* 1. esd check & recovery */
		if (!esd_ctx->chk_active) {
			DDPMSG("%s, %d, esd recover is disabled\n", __func__, __LINE__);
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
			mutex_unlock(&private->commit.lock);
			continue;
		}

		i = 0; /* repeat */
		do {
			if (te_timeout == false || i > 0) {
				ret = mtk_drm_esd_check(crtc);
				//#ifndef OPLUS_FEATURE_ESD
				//if (!ret) /* success */
					//break;
				//#else
				if (!esd_mode && !ret) /* success */
					break;
				esd_flag = 1;
				//#endif
			}
			DDPPR_ERR(
				"[ESD%u]esd check fail, will do esd recovery. te timeout:%d try=%d\n",
				crtc_idx, te_timeout, i);
#ifdef OPLUS_FEATURE_DISPLAY
			gpio_dump_regs_range(119, 121);
			gpio_dump_regs_range(21, 23);
			gpio_dump_regs_range(19, 21);
			gpio_dump_regs_range(40, 42);
			if (get_eng_version() == AGING || get_eng_version() == PREVERSION) {
				esd_read_cpu(mtk_crtc);
			}
			if ((prj_id == 22017) && (get_eng_version() == AGING)) {
				DDPMSG("%s, READ_ESD TE fail cancel esd recover in aging\n", __func__);
			} else {
				mtk_drm_esd_recover(crtc);
			}
#else
			mtk_drm_esd_recover(crtc);
#endif

#if defined(CONFIG_PXLW_IRIS)
			DDPINFO("%s notify esd recovery\n", __func__);
			atomic_set(&private->idle_need_repaint, 1);
			drm_trigger_repaint(DRM_REPAINT_FOR_ESD, crtc->dev);
#endif
			if (!panel_is_aries() && !mtk_crtc->panel_ext->params->esd_te_check_gpio)
				msleep(2000);
			//#ifdef OPLUS_FEATURE_ESD
			esd_mode = 0;
			esd_flag = 0;
			//#endif
			recovery_flg = 1;
		} while (++i < ESD_TRY_CNT);

		if (ret != 0) {
			DDPPR_ERR(
				"[ESD%u]esd recover %d times failed, max:%d, disable esd check, ret:%d\n",
				crtc_idx, i, ESD_TRY_CNT, ret);
			mtk_disp_esd_check_switch(crtc, false);
			/*
			 * disable ESD check might release TE pin to GPIO mode when connector
			 * switch enabled, need restore TE pin back to TE mode.
			 */
			if (esd_ctx->need_release_eint == 1)
				mtk_drm_request_eint(crtc);
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
			mutex_unlock(&private->commit.lock);
			break;
		} else if (recovery_flg) {
			DDPINFO("[ESD%u] esd recovery success\n", crtc_idx);
			recovery_flg = 0;
		}
		mtk_drm_trace_end();
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		mutex_unlock(&private->commit.lock);

		if (mtk_crtc->panel_ext->params->move_esd_readdate_back) {
			if (read_ddic_once && comp && dsi && oplus_esd_check_read_mode) {
				if (mtk_dsi_is_cmd_mode(comp)) {
					DDPMSG("[ESD] get panel serial number2\n");
					oplus_get_panel_serial_number_interface(dsi, 2);
					read_ddic_once = false;
				}
			}
		}

		/* 2. other check & recovery */
		if (kthread_should_stop())
			break;
	}
	return 0;
}

void mtk_disp_esd_check_switch(struct drm_crtc *crtc, bool enable)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_panel_ext *panel_ext;

	if (!mtk_drm_helper_get_opt(priv->helper_opt,
					   MTK_DRM_OPT_ESD_CHECK_RECOVERY))
		return;

	if (unlikely(!esd_ctx)) {
		DDPINFO("%s:invalid ESD context, crtc id:%d\n",
			__func__, drm_crtc_index(crtc));
		return;
	}

	panel_ext = mtk_crtc->panel_ext;
	if (!(panel_ext && panel_ext->params)) {
		DDPMSG("can't find panel_ext handle\n");
		return;
	}

	if (0 == _lcm_need_esd_check(panel_ext))
		return;

	DDPINFO("%s %u, esd chk active: %d\n", __func__, drm_crtc_index(crtc), enable);
	esd_ctx->chk_active = enable;

	/* release eint for connector switch; crtc might check differrent eint irq */
	if (esd_ctx->need_release_eint == 1) {
		if (!enable) /* release EINT if exist */
			mtk_drm_release_esd_eint(crtc);
		else /* request EINT before enable ESD check */
			mtk_drm_request_eint(crtc);
	}
	DDPPR_ERR("[OPLUS]%s %u, esd chk active: %d for debug\n", __func__, drm_crtc_index(crtc), enable);
	atomic_set(&esd_ctx->check_wakeup, enable);

	/* #ifdef OPLUS_FEATURE_DISPLAY */
	atomic_set(&esd_ctx->target_flag, enable);
	/* #endif */

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check,
			0xffffffff, enable);
	if (enable)
		wake_up_interruptible(&esd_ctx->check_task_wq);
	else
		wake_up_interruptible(&esd_ctx->int_te_wq);
}

static void mtk_disp_esd_chk_deinit(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s:invalid ESD context\n", __func__);
		return;
	}

	/* Stop ESD task */
	mtk_disp_esd_check_switch(crtc, false);

	/* Stop ESD kthread */
	kthread_stop(esd_ctx->disp_esd_chk_task);

	kfree(esd_ctx);
}

static void mtk_disp_esd_chk_init(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_ext *panel_ext;
	struct mtk_drm_esd_ctx *esd_ctx;

	panel_ext = mtk_crtc->panel_ext;
	if (!(panel_ext && panel_ext->params)) {
		DDPMSG("can't find panel_ext handle\n");
		return;
	}

	if (_lcm_need_esd_check(panel_ext) == 0)
		return;

	DDPINFO("create ESD thread\n");
	/* primary display check thread init */
	esd_ctx = kzalloc(sizeof(*esd_ctx), GFP_KERNEL);
	if (!esd_ctx) {
		DDPPR_ERR("allocate ESD context failed!\n");
		return;
	}
	mtk_crtc->esd_ctx = esd_ctx;

	esd_ctx->eint_irq = -1;
	esd_ctx->need_release_eint = -1;
	esd_ctx->chk_en = 1;
	esd_ctx->disp_esd_chk_task = kthread_create(
		mtk_drm_esd_check_worker_kthread, crtc, "disp_echk");

	init_waitqueue_head(&esd_ctx->check_task_wq);
	init_waitqueue_head(&esd_ctx->ext_te_wq);
	init_waitqueue_head(&esd_ctx->int_te_wq);
	atomic_set(&esd_ctx->check_wakeup, 0);
	atomic_set(&esd_ctx->ext_te_event, 0);
	atomic_set(&esd_ctx->int_te_event, 0);
	atomic_set(&esd_ctx->target_time, 0);
	if (panel_ext->params->cust_esd_check == 1)
		esd_ctx->chk_mode = READ_LCM;
	else
		esd_ctx->chk_mode = READ_EINT;
	mtk_drm_request_eint(crtc);

	wake_up_process(esd_ctx->disp_esd_chk_task);
}

void mtk_disp_chk_recover_deinit(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;

	output_comp = (mtk_crtc) ? mtk_ddp_comp_request_output(mtk_crtc) : NULL;

	/* only support ESD check for DSI output interface */
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_ESD_CHECK_RECOVERY) &&
			output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI)
		mtk_disp_esd_chk_deinit(crtc);
}

void mtk_disp_chk_recover_init(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;

	output_comp = (mtk_crtc) ? mtk_ddp_comp_request_output(mtk_crtc) : NULL;

	/* only support ESD check for DSI output interface */
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_ESD_CHECK_RECOVERY) &&
			output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI)
		mtk_disp_esd_chk_init(crtc);
}
