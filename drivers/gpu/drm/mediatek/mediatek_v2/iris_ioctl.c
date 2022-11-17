#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <drm/drm_vblank.h>
#include "mtk_drm_crtc.h"
#include "mtk_drm_ddp.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_debugfs.h"
#include "mtk_drm_drv.h"
#include "mtk_drm_mmp.h"
#include "mtk_log.h"

#include "iris_api.h"
#include "iris_uapi.h"
#include "iris_lightup.h"
#include "iris_lightup_ocp.h"
#include "iris_lp.h"
#include "iris_lut.h"
#include "iris_pq.h"
#include "iris_mode_switch.h"
#include "iris_ioctl.h"
#include "iris_i3c.h"
#include "iris_frc.h"
#include "iris_loop_back.h"
#include "iris_log.h"
#include "iris_i2c.h"
#include "iris_mtk_api.h"

// for game station settings via i2c
uint32_t CM_CNTL[14] = {
	0xf0560000, 0x8020e000,
	0xf0560000, 0x820e000,
	0xf0560008, 0x00000000,
	0xf056000c, 0x6e,
	0xf056000c, 0x5f,
	0xf0560110, 0x00000000,
	0xf0560140, 0x00000100
};

// 0: mipi, 1: i2c
static int adb_type = 0;
static int iris_i2c_ver = 1;

void iris_ddp_mutex_lock(void)
{
#if !defined(IRIS_I2C_ENABLE)
	struct iris_cfg *pcfg = iris_get_cfg();
	struct drm_crtc *crtc1 = pcfg->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc1);

	//IRIS_LOGD("%s:%d wait vblank+\n", __func__, __LINE__);
	//drm_wait_one_vblank(pcfg->drm, 0);
	//IRIS_LOGD("%s:%d wait vblank-\n", __func__, __LINE__);

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	mtk_drm_idlemgr_kick(__func__, crtc1, 0);
	mtk_drm_set_idlemgr(crtc1, 0, 0);
#endif
}

void iris_ddp_mutex_unlock(void)
{
#if !defined(IRIS_I2C_ENABLE)
	struct iris_cfg *pcfg = iris_get_cfg();
	struct drm_crtc *crtc1 = pcfg->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc1);

	mtk_drm_set_idlemgr(crtc1, 1, 0);
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
#endif
}

static int mdss_mipi_dsi_command(void __user *values)
{
	int ret = 0;
	struct msmfb_mipi_dsi_cmd cmd;
	struct iris_cmd_desc desc;
	char read_response_buf[16] = {0};
	struct iris_cmd_set cmdset = {
		.count = 1,
		.cmds = &desc,
		.state = DSI_CMD_SET_STATE_LP,
	};
	struct iris_ocp_dsi_tool_input iris_ocp_input={0,0,0,0,0};

	ret = copy_from_user(&cmd, values, sizeof(cmd));
	if (ret) {
		IRIS_LOGE("can not copy from user");
		return -EPERM;
	}

	IRIS_LOGD("#### %s:%d vc=%u d=%02x f=%hu l=%hu", __func__, __LINE__,
		   cmd.vc, cmd.dtype, cmd.flags, cmd.length);

	IRIS_LOGD("#### %s:%d %x, %x, %x", __func__, __LINE__,
		   cmd.iris_ocp_type, cmd.iris_ocp_addr, cmd.iris_ocp_size);

	if (cmd.length < SZ_4K && cmd.payload) {
		desc.msg.tx_buf = kvzalloc(cmd.length, GFP_KERNEL);
		if (!desc.msg.tx_buf)
			return -ENOMEM;
		ret = copy_from_user((char *)desc.msg.tx_buf, cmd.payload, cmd.length);
		if (ret) {
			ret = -EPERM;
			goto err;
		}
	} else
		return -EINVAL;

	desc.msg.type = cmd.dtype;
	desc.last_command = (cmd.flags & MSMFB_MIPI_DSI_COMMAND_LAST) > 0;
	desc.msg.tx_len = cmd.length;

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_HS)
		cmdset.state = DSI_CMD_SET_STATE_HS;

	iris_ddp_mutex_lock();

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_TO_PANEL) {
		if (iris_is_pt_mode())
			iris_pt_send_panel_cmd(&cmdset);
		else
			iris_dsi_send_cmds(cmdset.cmds, cmdset.count, cmdset.state);
	} else if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_T){
		u32 pktCnt = (cmd.iris_ocp_type >> 8) & 0xFF;

		//only test LUT send command
		if ((cmd.iris_ocp_type & 0xF) == PXLW_DIRECTBUS_WRITE) {
			u8 lut_type = (cmd.iris_ocp_type >> 8) & 0xFF;
			u8 lut_index = (cmd.iris_ocp_type >> 16) & 0xFF;
			u8 lut_parse = (cmd.iris_ocp_type >> 24) & 0xFF;
			u32 lut_pkt_index = cmd.iris_ocp_addr;
			if (lut_parse) // only parse firmware when value is not zero;
				iris_parse_lut_cmds(LOAD_CALIBRATED_OR_GOLDEN);
			iris_send_lut(lut_type, lut_index, lut_pkt_index);
		} else { // test ocp wirte
			if(pktCnt > DSI_CMD_CNT)
				pktCnt = DSI_CMD_CNT;

			if(cmd.iris_ocp_size < OCP_MIN_LEN)
				cmd.iris_ocp_size = OCP_MIN_LEN;

			iris_ocp_input.iris_ocp_type = cmd.iris_ocp_type & 0xF;
			iris_ocp_input.iris_ocp_cnt = pktCnt;
			iris_ocp_input.iris_ocp_addr = cmd.iris_ocp_addr;
			iris_ocp_input.iris_ocp_value = cmd.iris_ocp_value;
			iris_ocp_input.iris_ocp_size = cmd.iris_ocp_size;

			if(pktCnt)
				iris_write_test_muti_pkt(&iris_ocp_input);
			else
				iris_write_test(cmd.iris_ocp_addr, cmd.iris_ocp_type & 0xF, cmd.iris_ocp_size);
				//iris_ocp_bitmask_write(ctrl,cmd.iris_ocp_addr,cmd.iris_ocp_size,cmd.iris_ocp_value);
		}
	} else
		iris_dsi_send_cmds(cmdset.cmds, cmdset.count, cmdset.state);

	iris_ddp_mutex_unlock();

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK) {
		// Both length of cmd.response and read_response_buf are 16.
		memcpy(cmd.response, read_response_buf, sizeof(cmd.response));
	}
	ret = copy_to_user(values, &cmd, sizeof(cmd));
	if (ret)
		ret = -EPERM;
err:
	kvfree(desc.msg.tx_buf);
	return ret;
}


int iris_operate_tool(struct msm_iris_operate_value *argp)
{
	int ret = -1;
	uint32_t parent_type = 0;
	struct iris_cfg *pcfg = NULL;

	// FIXME: copy_from_user() is failed.
	// ret = copy_from_user(&configure, argp, sizeof(configure));
	// if (ret) {
	//	pr_err("1st %s type = %d, value = %d\n",
	//		__func__, configure.type, configure.count);
	//	return -EPERM;
	// }
	IRIS_LOGI("%s type = %d, value = %d", __func__, argp->type, argp->count);

	pcfg = iris_get_cfg();
	if (pcfg == NULL || pcfg->valid < PARAM_PARSED) {
		IRIS_LOGE("Target display does not exist!");
		return -EPERM;
	}

	parent_type = argp->type & 0xff;
	switch (parent_type) {
	case IRIS_OPRT_TOOL_DSI:
		ret = mdss_mipi_dsi_command(argp->values);
		break;
	default:
		IRIS_LOGE("could not find right opertat type = %d", argp->type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static bool iris_special_config(u32 type)
{
	switch (type) {
	case IRIS_OSD_ENABLE:
	case IRIS_OSD_AUTOREFRESH:
	case IRIS_OSD_OVERFLOW_ST:
	case IRIS_DBG_KERNEL_LOG_LEVEL:
	case USER_DEMO_WND:
	case IRIS_MEMC_LEVEL:
	case IRIS_WAIT_VSYNC:
	case IRIS_CHIP_VERSION:
	case IRIS_FW_UPDATE:
	case IRIS_DEBUG_SET:
		return true;
	}

	return false;
}

static bool _iris_is_valid_type(u32 display, u32 type)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return false;

	if (!iris_special_config(type)
			&& type != IRIS_ANALOG_BYPASS_MODE
			&& !iris_is_pt_mode())
		return false;

	if (type != IRIS_DBG_KERNEL_LOG_LEVEL
			&& pcfg->chip_ver == IRIS3_CHIP_VERSION)
		return false;

	return true;
}

void iris_plus_configure_abyp(int value)
{
	/*need to remove PQ bit*/
	int mode = value & PT_ABP_MASK;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(), switch Iris mode to: %u", __func__, value);

	if (iris_get_default_work_mode() == ABP_SLEEP_MODE
			&& (mode == ANALOG_BYPASS_MODE)) {
		mode = ABP_SLEEP_MODE;
		value += (ABP_SLEEP_MODE - ANALOG_BYPASS_MODE);
	}

	if (mode == pcfg->abypss_ctrl.abypass_mode) {
		IRIS_LOGI("%s Same bypass mode: %d", __func__, mode);
		return;
	}

	if (mode == PASS_THROUGH_MODE)
		iris_abypass_switch_proc(value, false, true);
	else {
		iris_panel_nits_set(0, true, value);
		iris_quality_setting_off();

		if (mode == ANALOG_BYPASS_MODE
				&& pcfg->abypss_ctrl.abypass_mode == ABP_SLEEP_MODE)
			iris_abypass_switch_proc(PASS_THROUGH_MODE, false, true);

		iris_abypass_switch_proc(value, true, true);
	}
}

void iris_configure_abyp(int value)
{
	/*need to remove PQ bit*/
	int mode = value & PT_ABP_MASK;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (mode == pcfg->abypss_ctrl.abypass_mode) {
		IRIS_LOGD("Same bypass mode");
		return;
	}
	IRIS_LOGI("%s(), switch Iris mode to: %u", __func__, value);

	if (mode == PASS_THROUGH_MODE)
		iris_abypass_switch_proc(value, false, true);
	else {
		iris_panel_nits_set(0, true, value);
		iris_quality_setting_off();
		iris_abypass_switch_proc(value, true, true);
	}
}


static int _iris_configure(u32 display, u32 type, u32 value)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	switch (type) {
	case IRIS_PEAKING:
		pqlt_cur_setting->pq_setting.peaking = value & 0xf;
		if (pqlt_cur_setting->pq_setting.peaking > 7)
			goto error;

		iris_peaking_level_set(pqlt_cur_setting->pq_setting.peaking);
		break;
	case IRIS_CM_6AXES:
		pqlt_cur_setting->pq_setting.cm6axis = value & 0x3;
		iris_cm_6axis_level_set(pqlt_cur_setting->pq_setting.cm6axis);
		break;
	case IRIS_CM_FTC_ENABLE:
		pqlt_cur_setting->cmftc = value;
		iris_cm_ftc_enable_set(pqlt_cur_setting->cmftc);
		break;
	case IRIS_S_CURVE:
		pqlt_cur_setting->scurvelevel = value & 0x7;
		if (pqlt_cur_setting->scurvelevel > 4)
			goto error;

		iris_scurve_enable_set(pqlt_cur_setting->scurvelevel);
		break;
	case IRIS_CM_COLOR_TEMP_MODE:
	#ifndef COLOR_TEMP_PLUS
		pqlt_cur_setting->pq_setting.cmcolortempmode = value & 0x3;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode > 2)
			goto error;
	#else
		pqlt_cur_setting->pq_setting.cmcolortempmode = value & 0x7;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode >= IRIS_COLOR_TEMP_MAX)
			goto error;
	#endif
		if (iris_gamut_valid_get() == false) {
			iris_cm_color_gamut_set(0);
			iris_gamut_valid_set(true);
		}
		iris_cm_colortemp_mode_set(pqlt_cur_setting->pq_setting.cmcolortempmode);
		break;
	case IRIS_CM_COLOR_GAMUT_PRE:
		iris_cm_color_gamut_pre_set(value & 0x03);
		break;
	case IRIS_CM_COLOR_GAMUT:
		pqlt_cur_setting->pq_setting.cmcolorgamut = value;
		if (pqlt_cur_setting->pq_setting.cmcolorgamut > 6)
			goto error;

		iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut);
		break;
	case IRIS_DBC_LCE_POWER:
		if (value == 0)
			iris_dbclce_power_set(false);
		else if (value == 1)
			iris_dbclce_power_set(true);
		else if (value == 2)
			iris_lce_dynamic_pmu_mask_set(false);
		else if (value == 3)
			iris_lce_dynamic_pmu_mask_set(true);

		break;
	case IRIS_DBC_LCE_DATA_PATH:
		iris_dbclce_datapath_set(value & 0x01);
		break;
	case IRIS_LCE_MODE:
		if (pqlt_cur_setting->pq_setting.lcemode != (value & 0x1)) {
			pqlt_cur_setting->pq_setting.lcemode = value & 0x1;
			iris_lce_mode_set(pqlt_cur_setting->pq_setting.lcemode);
		}
		break;
	case IRIS_LCE_LEVEL:
		if (pqlt_cur_setting->pq_setting.lcelevel != (value & 0x7)) {
			pqlt_cur_setting->pq_setting.lcelevel = value & 0x7;
			if (pqlt_cur_setting->pq_setting.lcelevel > 5)
				goto error;

			iris_lce_level_set(pqlt_cur_setting->pq_setting.lcelevel);
		}
		break;
	case IRIS_LCE_GAIN:
		pqlt_cur_setting->lce_gain_reset = value & 0x1;
		iris_lce_gain_reset(pqlt_cur_setting->lce_gain_reset, true);
		break;
	case IRIS_GRAPHIC_DET_ENABLE:
		pqlt_cur_setting->pq_setting.graphicdet = value & 0x1;
		iris_lce_graphic_det_set(pqlt_cur_setting->pq_setting.graphicdet);
		break;
	case IRIS_AL_ENABLE:

		if (pqlt_cur_setting->pq_setting.alenable != (value & 0x1)) {
			pqlt_cur_setting->pq_setting.alenable = value & 0x1;

			/*check the case here*/
			if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass)
				iris_lce_al_set(pqlt_cur_setting->pq_setting.alenable);
			else
				iris_ambient_light_lut_set(iris_sdr2hdr_lut2ctl_get());
		}
		break;
	case IRIS_DBC_LEVEL:
		pqlt_cur_setting->pq_setting.dbc = value & 0x3;
		iris_dbc_level_set(pqlt_cur_setting->pq_setting.dbc);
		break;
	case IRIS_BLC_PWM_ENABLE:
		iris_pwm_enable_set(value & 0x1);
		break;
	case IRIS_DEMO_MODE:
		pqlt_cur_setting->pq_setting.demomode = value & 0x3;
		break;
	case IRIS_DYNAMIC_POWER_CTRL:
		if (value & 0x01) {
			IRIS_LOGI(" [%s, %d] open psr_mif osd first address eco.", __func__, __LINE__);
			iris_psf_mif_dyn_addr_set(true);
			iris_dynamic_power_set(value & 0x01);
		} else {
			IRIS_LOGI(" [%s, %d] close psr_mif osd first address eco.", __func__, __LINE__);
			iris_dynamic_power_set(value & 0x01);
			iris_psf_mif_dyn_addr_set(false);
		}
		break;
	case IRIS_DMA_LOAD:
		iris_dma_trigger_load();
		break;
	case IRIS_SDR2HDR:
		iris_set_sdr2hdr_mode((value & 0xf00) >> 8);
		value = value & 0xff;
		if (value/10 == 4) {/*magic code to enable YUV input.*/
			iris_set_yuv_input(true);
			value -= 40;
		} else if (value/10 == 6) {
			iris_set_HDR10_YCoCg(true);
			value -= 60;
		} else if (value == 55) {
			iris_set_yuv_input(true);
			return 0;
		} else if (value == 56) {
			iris_set_yuv_input(false);
			return 0;
		} else {
			iris_set_yuv_input(false);
			iris_set_HDR10_YCoCg(false);
		}

		if (pqlt_cur_setting->pq_setting.sdr2hdr > SDR709_2_2020 || value > SDR709_2_2020)
			goto error;
		if (pqlt_cur_setting->pq_setting.sdr2hdr != value) {
			pqlt_cur_setting->pq_setting.sdr2hdr = value;

			iris_sdr2hdr_level_set(pqlt_cur_setting->pq_setting.sdr2hdr);
		}
		break;
	case IRIS_READING_MODE:
		pqlt_cur_setting->pq_setting.readingmode = value & 0x1;
		iris_reading_mode_set(pqlt_cur_setting->pq_setting.readingmode);
		break;
	case IRIS_COLOR_TEMP_VALUE:
		pqlt_cur_setting->colortempvalue = value;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL)
			iris_cm_color_temp_set();
		break;
	case IRIS_CCT_VALUE:
		pqlt_cur_setting->cctvalue = value;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_AUTO)
			iris_cm_color_temp_set();
		break;
	case IRIS_LUX_VALUE:
		/* move to iris_configure_ex*/
		pqlt_cur_setting->luxvalue = value;
		if (pqlt_cur_setting->pq_setting.alenable == 1) {

			if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass)
				iris_lce_lux_set();
			else
				iris_ambient_light_lut_set(iris_sdr2hdr_lut2ctl_get());
		}
		break;
	case IRIS_HDR_MAXCLL:
		pqlt_cur_setting->maxcll = value;
		break;
	case IRIS_ANALOG_BYPASS_MODE:
		pcfg->abyp_ops.configure_abyp(value);
		break;
	case IRIS_DBG_LOOP_BACK_MODE:
		pcfg->loop_back_mode = value;
		break;
	case IRIS_HDR_PANEL_NITES_SET:
		if (pqlt_cur_setting->al_bl_ratio != value) {
			pqlt_cur_setting->al_bl_ratio = value;
			iris_panel_nits_set(value, false, pqlt_cur_setting->pq_setting.sdr2hdr);
		}
		break;
	case IRIS_PEAKING_IDLE_CLK_ENABLE:
		iris_peaking_idle_clk_enable(value & 0x01);
		break;
	case IRIS_CM_MAGENTA_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_MAGENTA_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_RED_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_RED_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_YELLOW_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_YELLOW_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_GREEN_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_GREEN_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_BLUE_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_BLUE_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_CM_CYAN_GAIN:
		iris_cm_6axis_seperate_gain(IRIS_CYAN_GAIN_TYPE, value & 0x3f);
		break;
	case IRIS_DBC_LED_GAIN:
		iris_dbc_led0d_gain_set(value & 0x3f);
		break;
	case IRIS_SCALER_FILTER_LEVEL:
		iris_scaler_filter_update(SCALER_INPUT, value & 0x7);
		break;
	case IRIS_SCALER_PP_FILTER_LEVEL:
		iris_scaler_filter_update(SCALER_PP, value & 0x1);
		break;
	case IRIS_HDR_PREPARE:
		if ((value == 0) || ((value == 1) && !iris_get_debug_cap()) || (value == 2))
			iris_hdr_csc_prepare();
		else if (value == 3)
			iris_set_skip_dma(true);
		break;
	case IRIS_HDR_COMPLETE:
		if ((value == 3) || (value == 5))
			iris_set_skip_dma(false);
		if ((value == 0) || ((value == 1) && !iris_get_debug_cap()))
			iris_hdr_csc_complete(value);
		else if (value >= 2)
			iris_hdr_csc_complete(value);

		if (value != 2 && value != 4) {
			if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass)
				iris_panel_nits_set(0, true, value);
			else
				iris_panel_nits_set(PANEL_BL_MAX_RATIO, false, value);
		}
		break;
	case IRIS_DEBUG_CAP:
		iris_set_debug_cap(value & 0x01);
		break;
	case IRIS_FW_UPDATE:
		// Need do multi-thread protection.
		if (value < LOAD_METHOD_CNT) {
			/* before parsing firmware, free ip & opt buffer which alloc for LUT,
			 * if loading firmware failed before, need realloc seq space after
			 * updating firmware
			 */
			u8 firmware_status = iris_get_fw_load_status();

			iris_free_ipopt_buf(IRIS_LUT_PIP_IDX);
			iris_parse_lut_cmds(value);
			if (firmware_status == FIRMWARE_LOAD_FAIL) {
				iris_free_seq_space();
				iris_alloc_seq_space();
			}
			if (iris_get_fw_load_status() == FIRMWARE_LOAD_SUCCESS) {
				if (iris_is_pt_mode()) {
					iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut);
					iris_scaler_gamma_enable(false, 1);
				}
				iris_update_fw_load_status(FIRMWARE_IN_USING);
			}
		}
		break;
	case IRIS_DBG_KERNEL_LOG_LEVEL:
		iris_set_loglevel(value);
		break;
	case IRIS_MODE_SET:
		iris_mode_switch_proc(value);
		break;
	case IRIS_VIDEO_FRAME_RATE_SET:
		iris_set_video_frame_rate_ms(value);
		break;
	case IRIS_OUT_FRAME_RATE_SET:
		iris_set_out_frame_rate(value);
		break;
	case IRIS_OSD_ENABLE:
		if (pcfg->iris_initialized == false) {
			IRIS_LOGI("iris not initialized");
			break;
		}
		if ((value == 1) || (value == 0)) {
			IRIS_LOGI("call iris_switch_osd_blending(%d)", value);
			iris_switch_osd_blending(value);
		} else if (value == 0x10) {
			IRIS_LOGI("power on iris mipi2 rx");
			iris_set_second_channel_power(true);
		} else if (value == 0x20) {
			IRIS_LOGI("reset pwil_v6, power off bulksram");
			iris_second_channel_post(value);
		} else if (value == 0x21) {
			IRIS_LOGI("power off iris mipi2 rx");
			iris_set_second_channel_power(false);
		} else if (value == 0x40) {
			iris_dom_set(0);
		} else if (value == 0x41) {
			iris_dom_set(2);
		} else if (value == 0x80 || value == 0x81) {
			pcfg->dual_setting = value == 0x81;
			if (pcfg->dual_test & 0x10)
				pcfg->dual_setting = true;

			if (!(pcfg->dual_test & 0x4))
				iris_dual_setting_switch(pcfg->dual_setting);
		} else
			IRIS_LOGE("IRIS_OSD_ENABLE, invalid val=%d", value);
		break;
	case IRIS_OSD_AUTOREFRESH:
		// Always use secondary display.
		iris_osd_autorefresh(value);
		break;
	case IRIS_FRC_LOW_LATENCY:
		iris_frc_app_info(value);
		break;
	case IRIS_PANEL_TE:
		iris_set_panel_te(value);
		break;
	case IRIS_AP_TE:
		iris_set_ap_te(value);
		break;
	case IRIS_N2M_ENABLE:
		iris_set_n2m_enable(value);
		break;
	case IRIS_WAIT_VSYNC:
		return iris_wait_vsync();
	case IRIS_MEMC_LEVEL:
		if (value <= 3) {
			pcfg->frc_setting.memc_level = value;
			pcfg->frc_setting.short_video = 0;
		} else if (value >= 4 && value <= 7) {
			pcfg->frc_setting.memc_level = value - 4;
			pcfg->frc_setting.short_video = 1;
		} else if (value >= 8 && value <= 11) {
			pcfg->frc_setting.memc_level = value - 8;
			pcfg->frc_setting.short_video = 2;
		}
		break;
	case IRIS_MEMC_OSD:
		pcfg->frc_setting.memc_osd = value;
		break;
	case USER_DEMO_WND:
		if(value > 5)
			goto error;
		iris_fi_demo_window(value);
		break;
	case IRIS_CHIP_VERSION:
		pcfg->chip_value[0] = value;
		break;
	case IRIS_OSD_LAYER_EMPTY:
		pcfg->osd_layer_empty = value;
		IRIS_LOGD("osd_layer_empty: %d", pcfg->osd_layer_empty);
		break;
	case IRIS_SET_METADATA:
		if (pcfg->metadata != 0) {
			IRIS_LOGI("last meta not sent!");
			goto error;
		}
		pcfg->metadata = value;
		IRIS_LOGD("metadata: %x", pcfg->metadata);
		break;
	case IRIS_SET_METADATA_LOCK:	// lock by panel_lock
		if (pcfg->metadata != 0)
			IRIS_LOGI("last meta not sent!");
		pcfg->metadata = value;
		IRIS_LOGD("metadata: %x", pcfg->metadata);
		iris_set_metadata(false);
		break;
	default:
		goto error;
	}

	return 0;

error:
	return -EINVAL;

}

int iris_configure(u32 display, u32 type, u32 value)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;
	ktime_t ktime = 0;

	IRIS_LOGI("%s(), display: %u, type: 0x%04x(%u), value: %#x(%u), current Iris mode: %d",
			__func__,
			display, type, type, value, value, pcfg->abypss_ctrl.abypass_mode);
	if (!_iris_is_valid_type(display, type))
		return -EPERM;

	switch (type) {
	case IRIS_DEMO_MODE:
	case IRIS_HDR_MAXCLL:
	case IRIS_DEBUG_CAP:
	case IRIS_VIDEO_FRAME_RATE_SET:
	case IRIS_OUT_FRAME_RATE_SET:
	case IRIS_OSD_AUTOREFRESH:
	case IRIS_DBG_KERNEL_LOG_LEVEL:
	case IRIS_FRC_LOW_LATENCY:
	case IRIS_N2M_ENABLE:
	case IRIS_WAIT_VSYNC:
	case IRIS_MEMC_LEVEL:
	case IRIS_MEMC_OSD:
	case USER_DEMO_WND:
	case IRIS_CHIP_VERSION:
	case IRIS_OSD_LAYER_EMPTY:
	case IRIS_SET_METADATA:
		/* don't lock panel_lock */
		return _iris_configure(display, type, value);
	}

	if (IRIS_IF_LOGI())
		ktime = ktime_get();

	iris_ddp_mutex_lock();
	rc = _iris_configure(display, type, value);
	iris_ddp_mutex_unlock();

	if (IRIS_IF_LOGI())
		IRIS_LOGI("%s(), spend %u us for type 0x%04x(%u) value %#x(%u)",
				__func__,
				(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime),
				type, type, value, value);

	return rc;
}

int iris_configure_t(uint32_t display, u32 type, void __user *argp)
{
	int ret = -1;
	uint32_t value = 0;

	ret = copy_from_user(&value, argp, sizeof(uint32_t));
	if (ret) {
		IRIS_LOGE("can not copy from user");
		return -EPERM;
	}

	ret = iris_configure(display, type, value);
	return ret;
}

static int _iris_configure_ex(u32 display, u32 type, u32 count, u32 *values)
{
	int ret = -1;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	struct msmfb_iris_ambient_info iris_ambient;
	struct msmfb_iris_maxcll_info iris_maxcll;
	struct msmfb_iris_ambient_info *iris_ambient_lut = NULL;
	struct msmfb_iris_maxcll_info *iris_maxcll_lut = NULL;
	uint8_t i = 0;
#ifndef COLOR_TEMP_PLUS
	u32 TempValue = 0;
	bool is_phone;
#endif
	u32 LutPos = 0;

	if (!_iris_is_valid_type(display, type))
		return -EPERM;

	switch (type) {
	case IRIS_LUX_VALUE:
		iris_ambient = *(struct msmfb_iris_ambient_info *)(values);
		iris_ambient_lut = iris_get_ambient_lut();
		iris_ambient_lut->ambient_lux = iris_ambient.ambient_lux;
		pqlt_cur_setting->luxvalue = iris_ambient_lut->ambient_lux;

		if (iris_ambient.lut_lut2_payload != NULL) {
			ret = copy_from_user(iris_ambient_lut->lut_lut2_payload, iris_ambient.lut_lut2_payload, sizeof(uint32_t)*LUT_LEN);
			if (ret) {
				IRIS_LOGE("can not copy from user sdr2hdr");
				goto error1;
			}
			LutPos = iris_sdr2hdr_lut2ctl_get();
			if (LutPos == 15) {
				LutPos = 0;
				iris_update_ambient_lut(AMBINET_SDR2HDR_LUT, 1);
			} else if (LutPos == 1)
				LutPos = 0;
			else if (!LutPos)
				LutPos = 1;
			iris_update_ambient_lut(AMBINET_SDR2HDR_LUT, LutPos);
		} else if (iris_sdr2hdr_lut2ctl_get() == 0xFFE00000)
			LutPos = 0xFFE00000;
		else
			LutPos = 0;

		if (pqlt_cur_setting->pq_setting.alenable == 1) {
			if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass) {
				iris_sdr2hdr_lut2ctl_set(LutPos);
				iris_lce_lux_set();
			} else
				iris_ambient_light_lut_set(LutPos);
		} else
			iris_sdr2hdr_lut2ctl_set(LutPos);
		break;
	case IRIS_HDR_MAXCLL:
		iris_maxcll_lut = iris_get_maxcll_info();
		iris_maxcll = *(struct msmfb_iris_maxcll_info *)(values);
		iris_maxcll_lut->mMAXCLL = iris_maxcll.mMAXCLL;

		if (iris_maxcll.lut_luty_payload != NULL) {
			ret = copy_from_user(iris_maxcll_lut->lut_luty_payload, iris_maxcll.lut_luty_payload, sizeof(uint32_t)*LUT_LEN);
			if (ret) {
				IRIS_LOGE("can not copy lut y from user sdr2hdr");
				goto error1;
			}
		}
		if (iris_maxcll.lut_lutuv_payload != NULL) {
			ret = copy_from_user(iris_maxcll_lut->lut_lutuv_payload, iris_maxcll.lut_lutuv_payload, sizeof(uint32_t)*LUT_LEN);
			if (ret) {
				IRIS_LOGE("can not copy lut uv from user sdr2hdr");
				goto error1;
			}
		}
		LutPos = iris_sdr2hdr_lutyctl_get();
		if (!LutPos)
			LutPos = 1;
		else if ((LutPos == 15) || (LutPos == 1))
			LutPos = 0;
		iris_update_maxcll_lut(AMBINET_HDR_GAIN, LutPos);
		iris_maxcll_lut_set(LutPos);
		break;
	case IRIS_CM_RATIO_SET:
		pqlt_cur_setting->cmlut_type = values[0];
		pqlt_cur_setting->cmlut_step = values[1];
		pqlt_cur_setting->cmlut_max_step = values[2];
		iris_cm_ratio_step_set(values[0], values[1], values[2]);
		break;
	case IRIS_CCF1_UPDATE:
		/* Nothing to do for Iirs5*/
		break;
	case IRIS_CCF2_UPDATE:
		/* Nothing to do for Iirs5*/
		break;
	case IRIS_HUE_SAT_ADJ:
		IRIS_LOGD("cm csc value: csc0 = 0x%x, csc1 = 0x%x, csc2 = 0x%x, csc3 = 0x%x, csc4 = 0x%x", values[0], values[1], values[2], values[3], values[4]);
		IRIS_LOGD("game mode %d", values[5]);
		if (values[5] == 1) {
			for (i = 0; i <= 4; i++) {
				if (pcfg->iris_i2c_write) {
					if (pcfg->iris_i2c_write(CM_CNTL[10] + i*4, values[i]) < 0)
						IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x", CM_CNTL[10] + i*4, values[i]);
				} else {
					IRIS_LOGE("Game Station is not connected");
				}
			}
		} else {
			iris_cm_csc_level_set(IRIS_IP_CM, &values[0]);
		}
		break;
	case IRIS_CONTRAST_DIMMING:
		IRIS_LOGI("dpp csc value: csc0 = 0x%x, csc1 = 0x%x, csc2 = 0x%x, csc3 = 0x%x, csc4 = 0x%x",
				values[0], values[1], values[2], values[3], values[4]);
		iris_cm_csc_level_set(IRIS_IP_DPP, &values[0]);
		break;
	case IRIS_COLOR_TEMP_VALUE:
	#ifndef COLOR_TEMP_PLUS
		is_phone = (count > 1) ? (values[1] == 0) : true;
		pqlt_cur_setting->colortempvalue = values[0];

		if (is_phone) {
			if (count > 3) {
				pqlt_cur_setting->min_colortempvalue = values[2];
				pqlt_cur_setting->max_colortempvalue = values[3];
			} else {
				pqlt_cur_setting->min_colortempvalue = 0;
				pqlt_cur_setting->max_colortempvalue = 0;
			}
			if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL)
				iris_cm_color_temp_set();
		} else {
			TempValue = iris_cm_ratio_set_for_iic();
			IRIS_LOGD("set reg=0x%x, val=0x%x", CM_CNTL[4], TempValue);
			if (pcfg->iris_i2c_write) {
				if (pcfg->iris_i2c_write(CM_CNTL[4], TempValue) < 0)
					IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x", CM_CNTL[4], TempValue);
			} else {
				IRIS_LOGE("Game Station is not connected");
			}
		}
	#else
		if (values[0] == 2) {
			pqlt_cur_setting->colortempplusvalue = values[1] | (values[2] << 16);
			iris_cm_color_temp_set();
		}
	#endif
		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_SET:
		if (adb_type == 0) {
			iris_ocp_write_val(values[0], values[1]);
		} else if (adb_type == 1) {
			if (iris_i2c_ver == 0) {
				if (pcfg->iris_i2c_write) {
					if (pcfg->iris_i2c_write(values[0], values[1]) < 0)
						IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x", values[0], values[1]);
				} else {
					IRIS_LOGE("Game Station is not connected");
				}
			} else {
				IRIS_LOGI("%s, addr = %x, value = %x\n", __func__, values[0], values[1]);
				iris_i2c_single_conver_ocp_write(values, 1);
			}
		}
		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_SET2:
		iris_ocp_write_vals(values[0], values[1], count-2, values+2);
		break;
	case IRIS_CM_6AXES:
		// phone
		pqlt_cur_setting->pq_setting.cm6axis = values[0] & 0x3;
		iris_cm_6axis_level_set(pqlt_cur_setting->pq_setting.cm6axis);

		// game station
		if (pcfg->iris_i2c_write) {
			if (pcfg->iris_i2c_write(CM_CNTL[0], values[1] ? CM_CNTL[3] : CM_CNTL[1]) < 0)
				IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[0]);
			else if (pcfg->iris_i2c_write(CM_CNTL[12], CM_CNTL[13]) < 0)
				IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[12]);
		} else {
			IRIS_LOGE("Game Station is not connected");
		}
		break;
	case IRIS_CM_COLOR_TEMP_MODE:
		// phone
		pqlt_cur_setting->pq_setting.cmcolortempmode = values[0] & 0x3;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode > 2)
			goto error;

		iris_cm_colortemp_mode_set(pqlt_cur_setting->pq_setting.cmcolortempmode);

		// game station
		if (pcfg->iris_i2c_write) {
			if (pcfg->iris_i2c_write(CM_CNTL[6], values[1] ? CM_CNTL[9] : CM_CNTL[7]) < 0)
				IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[6]);
			else if (pcfg->iris_i2c_write(CM_CNTL[12], CM_CNTL[13]) < 0)
				IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[12]);
		} else {
			IRIS_LOGE("Game Station is not connected");
		}
		break;
	case IRIS_CSC_MATRIX:
		if (count > 9) {
			if (values[0] == 1)
				iris_cm_csc_level_set(IRIS_IP_CM, &values[2]);
			else if (values[0] == 2)
				iris_cm_csc_level_set(IRIS_IP_DPP, &values[2]);
			else
				return -EPERM;
		} else
			return -EPERM;
		break;
	case IRIS_DBG_SEND_PACKAGE:
		ret = iris_send_ipopt_cmds(values[0], values[1]);
		IRIS_LOGD("iris config sends package: ip: %#x, opt: %#x, send: %d.",
				values[0], values[1], ret);
		break;
	case IRIS_MEMC_OSD_PROTECT:
		IRIS_LOGD("OSD protect setting: Top_left_pos = 0x%x, bot_right_pos = 0x%x, OSDwinID = 0x%x, OSDwinIDEn = 0x%x, DynCompensate = 0x%x",
				values[0], values[1], values[2], values[3], values[4]);
		ret = iris_fi_osd_protect_window(values[0], values[1], values[2], values[3], values[4]);
		if(pcfg->pwil_mode == FRC_MODE)
			iris_fi_demo_window_cal(true);
		if (ret)
			goto error;
		break;
	case IRIS_BRIGHTNESS_CHIP:
		iris_brightness_level_set(&values[0]);
		break;
	case IRIS_LCE_DEMO_WINDOW:
		iris_lce_demo_window_set(values[0], values[1], values[2]);
		break;
	case IRIS_WAIT_VSYNC:
		if (count > 2)
			iris_set_pending_panel_brightness(values[0], values[1], values[2]);
		break;
	case IRIS_DEBUG_SET:
		if (count == 2)
			iris_debug_mode_switch_set(values[0], values[1]);
		break;
	default:
		goto error;
	}

	return 0;

error:
	return -EINVAL;
error1:
	return -EPERM;
}

int iris_configure_ex(u32 display, u32 type, u32 count, u32 *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;

	IRIS_LOGI("%s(), type: 0x%04x(%d), value: %#x(%d), count: %d, abyp mode: %d",
			__func__,
			type, type, values[0], values[0], count, pcfg->abypss_ctrl.abypass_mode);
	if (!_iris_is_valid_type(display, type))
		return -EPERM;

	switch (type) {
	case IRIS_WAIT_VSYNC:
	case IRIS_DEBUG_SET:
		/* don't lock panel_lock */
		return _iris_configure_ex(display, type, count, values);
	}

	iris_ddp_mutex_lock();
	rc = _iris_configure_ex(display, type, count, values);
	iris_ddp_mutex_unlock();
	return rc;
}

static int iris_configure_ex_t(uint32_t display, uint32_t type,
								uint32_t count, void __user *values)
{
	int ret = -1;
	uint32_t *val = NULL;

	val = vmalloc(sizeof(uint32_t) * count);
	if (!val) {
		IRIS_LOGE("can not vmalloc space");
		return -ENOSPC;
	}
	ret = copy_from_user(val, values, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("can not copy from user");
		vfree(val);
		return -EPERM;
	}

	ret = iris_configure_ex(display, type, count, val);

	vfree(val);
	return ret;
}

int iris_configure_get(u32 display, u32 type, u32 count, u32 *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 reg_addr, reg_val;

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -EINVAL;

	switch (type) {
	case IRIS_PEAKING:
		*values = pqlt_cur_setting->pq_setting.peaking;
		break;
	case IRIS_CM_6AXES:
		*values = pqlt_cur_setting->pq_setting.cm6axis;
		break;
	case IRIS_CM_FTC_ENABLE:
		*values = pqlt_cur_setting->cmftc;
		break;
	case IRIS_S_CURVE:
		*values = pqlt_cur_setting->scurvelevel;
		break;
	case IRIS_CM_COLOR_TEMP_MODE:
		*values = pqlt_cur_setting->pq_setting.cmcolortempmode;
		break;
	case IRIS_CM_COLOR_GAMUT:
		*values = pqlt_cur_setting->pq_setting.cmcolorgamut;
		break;
	case IRIS_LCE_MODE:
		*values = pqlt_cur_setting->pq_setting.lcemode;
		break;
	case IRIS_LCE_LEVEL:
		*values = pqlt_cur_setting->pq_setting.lcelevel;
		break;
	case IRIS_LCE_GAIN:
		*values = pqlt_cur_setting->lce_gain_reset;
	case IRIS_GRAPHIC_DET_ENABLE:
		*values = pqlt_cur_setting->pq_setting.graphicdet;
		break;
	case IRIS_AL_ENABLE:
		*values = pqlt_cur_setting->pq_setting.alenable;
		break;
	case IRIS_DBC_LEVEL:
		*values = pqlt_cur_setting->pq_setting.dbc;
		break;
	case IRIS_DEMO_MODE:
		*values = pqlt_cur_setting->pq_setting.demomode;
		break;
	case IRIS_SDR2HDR:
		*values = pqlt_cur_setting->pq_setting.sdr2hdr;
		break;
	case IRIS_LUX_VALUE:
		*values = pqlt_cur_setting->luxvalue;
		break;
	case IRIS_READING_MODE:
		*values = pqlt_cur_setting->pq_setting.readingmode;
		break;
	case IRIS_DYNAMIC_POWER_CTRL:
		*values = iris_dynamic_power_get();
		break;
	case IRIS_HDR_MAXCLL:
		*values = pqlt_cur_setting->maxcll;
		break;
	case IRIS_ANALOG_BYPASS_MODE:
		if (1) {
			int mode = pcfg->abypss_ctrl.abypass_mode;

			if ((iris_get_default_work_mode() == ABP_SLEEP_MODE)
				&& (mode == ABP_SLEEP_MODE))
				mode = ANALOG_BYPASS_MODE;

			*values = mode;
		}
		break;
	case IRIS_DBG_LOOP_BACK_MODE:
		*values = pcfg->loop_back_mode;
		break;
	case IRIS_DBG_LOOP_BACK_MODE_RES:
		*values = pcfg->loop_back_mode_res;
		break;
	case IRIS_CM_COLOR_GAMUT_PRE:
		*values = pqlt_cur_setting->source_switch;
		break;
	case IRIS_CCT_VALUE:
		*values = pqlt_cur_setting->cctvalue;
		break;
	case IRIS_COLOR_TEMP_VALUE:
		*values = pqlt_cur_setting->colortempvalue;
		break;
	case IRIS_CHIP_VERSION:
		if (*values == 1)
			*values = pcfg->chip_value[1];
		else if (*values == 2) {
			if (pcfg->chip_id != 0xFF)
				*values = pcfg->chip_id;
			else
				return -EINVAL;
		}
		else {
			*values = 0;
			if (1 /*iris_is_chip_supported()*/)
				*values |= (1 << IRIS5_VER);
			if (iris_is_softiris_supported())
				*values |= (1 << IRISSOFT_VER);
			if (iris_is_dual_supported())
				*values |= (1 << IRIS5DUAL_VER);
			if (*values == 0)
				return -EFAULT;
		}
		break;
	case IRIS_PANEL_TYPE:
		*values = pcfg->panel_type;
		break;
	case IRIS_PANEL_NITS:
		*values = pcfg->panel_nits;
		break;
	case IRIS_MCF_DATA:
		/* get MCF from panel */
		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_GET:
		IRIS_LOGI("%s:%d, pcfg->abypss_ctrl.abypass_mode = %d",
				__func__, __LINE__,
				pcfg->abypss_ctrl.abypass_mode);
		if (!iris_is_pt_mode() && (adb_type == 0))
			return -ENOTCONN;

		if (false) {
			if (!mutex_trylock(&pcfg->lock_send_pkt)) {
				IRIS_LOGE("%s:%d lock_send_pkt is locked!", __func__, __LINE__);
				mutex_lock(&pcfg->lock_send_pkt);
			}
			iris_ddp_mutex_lock();
			*values = iris_ocp_read(*values, DSI_CMD_SET_STATE_HS);
			iris_ddp_mutex_unlock();
			mutex_unlock(&pcfg->lock_send_pkt);
		} else if (true) {
			reg_addr = *values;
			if (iris_i2c_ver == 0) {
				if (pcfg->iris_i2c_read) {
					if (pcfg->iris_i2c_read(reg_addr, &reg_val) < 0)
						IRIS_LOGE("i2c read reg fails, reg=0x%x", reg_addr);
					else
						*values = reg_val;
				} else {
					IRIS_LOGE("Game Station is not connected");
				}
			} else {
				iris_i2c_conver_ocp_read(values, 1, 0);
				IRIS_LOGI("%s: addr = %x, value = %x\n", __func__, reg_addr, *values);
			}
		}
		break;
	case IRIS_DBG_KERNEL_LOG_LEVEL:
		*values = iris_get_loglevel();
		break;
	case IRIS_VIDEO_FRAME_RATE_SET:
		*values = (u32)pcfg->frc_setting.in_fps * 1000;
		break;
	case IRIS_OUT_FRAME_RATE_SET:
		*values = pcfg->frc_setting.out_fps;
		break;
	case IRIS_OSD_ENABLE:
		*values = pcfg->osd_on ? 1 : 0;
		break;
	case IRIS_OSD_AUTOREFRESH:
		*values = pcfg->iris_osd_autorefresh ? 1 : 0;
		break;
	case IRIS_OSD_OVERFLOW_ST:
		*values = iris_get_osd_overflow_st();
		break;
	case IRIS_MIPI2RX_PWRST:
		*values = 0;
		if (0 /*pcfg->panel2->power_info.refcount*/)
			*values |= 1;
		if (pcfg->mipi2_pwr_st)
			*values |= (1 << 1);
		break;
	case IRIS_DUAL2SINGLE_ST:
		iris_ddp_mutex_lock();
		if (pcfg->mipi2_pwr_st == false) {
			IRIS_LOGI("mipi2 rx has been power off");
			*values = 1;
		} else
			*values = iris_get_dual2single_status();
		iris_ddp_mutex_unlock();
		break;
	case IRIS_WORK_MODE:
		*values = ((int)pcfg->pwil_mode<<16) | ((int)pcfg->tx_mode<<8) | ((int)pcfg->rx_mode);
		break;
	case IRIS_PANEL_TE:
		*values = pcfg->panel_te;
		break;
	case IRIS_AP_TE:
		*values = pcfg->ap_te;
		IRIS_LOGI("get IRIS_AP_TE: %d", pcfg->ap_te);
		break;
	case IRIS_MODE_SET:
		if (!mutex_trylock(&pcfg->lock_send_pkt)) {
			IRIS_LOGE("%s:%d lock_send_pkt is locked!", __func__, __LINE__);
			mutex_lock(&pcfg->lock_send_pkt);
		}
		iris_ddp_mutex_lock();
		*values = iris_mode_switch_update();
		iris_ddp_mutex_unlock();
		mutex_unlock(&pcfg->lock_send_pkt);
		break;
	case IRIS_N2M_ENABLE:
		*values = pcfg->n2m_enable;
		break;
	case IRIS_MEMC_LEVEL:
		*values = pcfg->frc_setting.memc_level;
		break;
	case IRIS_MEMC_OSD:
		*values = pcfg->frc_setting.memc_osd;
		break;
	case IRIS_PARAM_VALID:
		iris_send_cont_splash();
		*values = pcfg->valid;
		break;
	case IRIS_GET_METADATA:
		*values = pcfg->metadata;
		break;
	case IRIS_DEBUG_GET:
		if (count == 2)
			iris_debug_mode_switch_get(values[0], &values[1]);
		break;
	case IRIS_GET_PANEL_INFO:
		strlcpy((char *)values, pcfg->panel_name, sizeof(uint32_t) * count);
		break;
	default:
		return -EFAULT;
	}

	IRIS_LOGI("%s(), type: 0x%04x(%d), value: %d",
			__func__,
			type, type, *values);
	return 0;
}

int iris_configure_get_t(uint32_t display, uint32_t type,
		uint32_t count, void __user *values)
{
	int ret = -1;
	uint32_t *val = NULL;

	val = vmalloc(count * sizeof(uint32_t));
	if (val == NULL) {
		IRIS_LOGE("could not vmalloc space for func = %s", __func__);
		return -ENOSPC;
	}
	ret = copy_from_user(val, values, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("can not copy from user");
		vfree(val);
		return -EPERM;
	}
	ret = iris_configure_get(display, type, count, val);
	if (ret) {
		IRIS_LOGE("get error");
		vfree(val);
		return ret;
	}
	ret = copy_to_user(values, val, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("copy to user error");
		vfree(val);
		return -EPERM;
	}
	vfree(val);
	return ret;
}

int iris_operate_conf(struct msm_iris_operate_value *argp)
{
	int ret = -1;
	uint32_t parent_type = 0;
	uint32_t child_type = 0;
	uint32_t display_type = 0;
	struct iris_cfg *pcfg = NULL;

	IRIS_LOGD("%s type=0x%04x", __func__, argp->type);

	parent_type = argp->type & 0xff;
	child_type = (argp->type >> 8) & 0xff;
	// always set to 0 when only one iris_cfg
	display_type = 0; //(argp->type >> 16) & 0xff;
	pcfg = iris_get_cfg();
	if (pcfg == NULL || pcfg->valid < PARAM_PARSED) {
		if (child_type == IRIS_WAIT_VSYNC || child_type == IRIS_CHIP_VERSION) {
			IRIS_LOGV("%s allow type: 0x%04x(%u) for Soft Iris", __func__, child_type, child_type);
		} else {
			IRIS_LOGE("Target display does not exist!");
			return -EPERM;
		}
	}

	IRIS_ATRACE_INT("iris_operate_conf", child_type);
	switch (parent_type) {
	case IRIS_OPRT_CONFIGURE:
		ret = iris_configure_t(display_type, child_type, argp->values);
		break;
	case IRIS_OPRT_CONFIGURE_NEW:
		ret = iris_configure_ex_t(display_type, child_type, argp->count, argp->values);
		break;
	case IRIS_OPRT_CONFIGURE_NEW_GET:
		ret = iris_configure_get_t(display_type, child_type, argp->count, argp->values);
		break;
	default:
		IRIS_LOGE("could not find right operate type = %d", argp->type);
		break;
	}
	IRIS_ATRACE_INT("iris_operate_conf", 0);

	return ret;
}

static ssize_t iris_adb_type_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	tot = scnprintf(bp, sizeof(bp), "%d\n", adb_type);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static ssize_t iris_adb_type_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	adb_type = val;

	return count;
}

static const struct file_operations iris_adb_type_write_fops = {
	.open = simple_open,
	.write = iris_adb_type_write,
	.read = iris_adb_type_read,
};

int iris_dbgfs_adb_type_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (debugfs_create_file("adb_type", 0644, pcfg->dbg_root, NULL,
				&iris_adb_type_write_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

/* Iris log level definition, for 'iris_log.h' */
static int iris_log_level = 2;

void iris_set_loglevel(int level)
{
	iris_log_level = level;
}

inline int iris_get_loglevel(void)
{
	return iris_log_level;
}
