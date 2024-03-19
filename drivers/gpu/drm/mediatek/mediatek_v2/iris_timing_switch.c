// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
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
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/gpio.h>
#include <video/mipi_display.h>

#include "drm_internal.h"
#include "mtk_drm_crtc.h"
#include "mtk_panel_ext.h"
#include "mtk_drm_ddp.h"

#include "iris_mtk_api.h"
#include "iris_api.h"
#include "iris_lightup.h"
#include "iris_lightup_ocp.h"
#include "iris_mode_switch.h"
#include "iris_frc.h"
#include "iris_lp.h"
#include "iris_log.h"


//#define to_dsi_display(x) container_of(x, struct dsi_display, host)

enum {
	SWITCH_ABYP_TO_ABYP = 0,
	SWITCH_ABYP_TO_PT,
	SWITCH_PT_TO_ABYP,
	SWITCH_PT_TO_PT,
	SWITCH_NONE,
};

#define SWITCH_CASE(case)[SWITCH_##case] = #case
static const char * const switch_case_name[] = {
	SWITCH_CASE(ABYP_TO_ABYP),
	SWITCH_CASE(ABYP_TO_PT),
	SWITCH_CASE(PT_TO_ABYP),
	SWITCH_CASE(PT_TO_PT),
	SWITCH_CASE(NONE),
};
#undef SWITCH_CASE

enum {
	TIMING_SWITCH_NORMAL_SEQ,
	TIMING_SWITCH_SPECIAL_SEQ,
	TIMING_SWITCH_SEQ_CNT,
};

static uint32_t switch_case;
static struct iris_ctrl_seq tm_switch_seq[TIMING_SWITCH_SEQ_CNT];
static uint32_t cmd_list_index;
static uint32_t panel_tm_num;
static uint8_t *tm_cmd_map_arry;
static struct dsi_mode_info *panel_tm_arry;
static uint32_t spec_tm_switch_num;
static uint8_t *spec_tm_switch_inst;
static uint32_t tm_2nd_num;
static uint8_t *tm_2nd_list;
static uint32_t cur_tm_index;
static uint32_t new_tm_index;
static uint32_t last_pt_tm_index;


void iris_init_timing_switch(void)
{
	IRIS_LOGI("%s()", __func__);
	switch_case = SWITCH_ABYP_TO_ABYP;
	cmd_list_index = IRIS_DTSI_PIP_IDX_START;
}

void iris_deinit_timing_switch(void)
{
	if (panel_tm_arry) {
		vfree(panel_tm_arry);
		panel_tm_arry = NULL;
	}

	if (tm_cmd_map_arry) {
		vfree(tm_cmd_map_arry);
		tm_cmd_map_arry = NULL;
	}

	if (spec_tm_switch_inst) {
		vfree(spec_tm_switch_inst);
		spec_tm_switch_inst = NULL;
	}

	if (tm_2nd_list) {
		vfree(tm_2nd_list);
		tm_2nd_list = NULL;
	}
}

static bool _iris_support_timing_switch(void)
{
	if (panel_tm_arry == NULL || panel_tm_num == 0)
		return false;

	return true;
}

static void iris_set_panel_timing(uint32_t index,
		const struct dsi_mode_info *timing)
{
	if (!iris_is_chip_supported())
		return;

	//if (display == NULL || timing == NULL || index >= panel_tm_num)
		//return;

	// for primary display only, skip secondary
	//if (strcmp(display->display_type, "primary"))
		//return;

	if (!_iris_support_timing_switch())
		return;

	IRIS_LOGI("%s(), timing@%u: %ux%u@%uHz",
			__func__, index,
			timing->h_active, timing->v_active, timing->refresh_rate);
	memcpy(&panel_tm_arry[index], timing, sizeof(struct dsi_mode_info));
}

void iris_init_panel_timing(void *connector)
{
	struct drm_connector *conn = (struct drm_connector *)connector;
	struct drm_display_mode *mode;
	struct dsi_mode_info timing = {0};
	int i = 0;

	if (!conn) {
		IRIS_LOGE("connector is null");
		return;
	}
	IRIS_LOGD("connector name=%s", conn->name);

	list_for_each_entry(mode, &conn->modes, head) {
		iris_sync_timing(&timing, mode);
		iris_set_panel_timing(i, &timing);
		i++;
	}
}
EXPORT_SYMBOL(iris_init_panel_timing);

static void _iris_init_param(struct device_node *np)
{
	int32_t pnl_tm_num = 0;
	int32_t special_switch_num = 0;
	int32_t timing_2nd_num = 0;

	pnl_tm_num = of_property_count_u8_elems(np, "pxlw,timing-cmd-map");
	if (pnl_tm_num < 1)
		pnl_tm_num = 0;

	special_switch_num = of_property_count_u8_elems(np, "pxlw,special-timing-switch-instance");
	if (special_switch_num < 1)
		special_switch_num = 0;

	timing_2nd_num = of_property_count_u8_elems(np, "pxlw,timing-2nd-list");
	if (timing_2nd_num < 1)
		timing_2nd_num = 0;

	panel_tm_num = pnl_tm_num;
	spec_tm_switch_num = special_switch_num;
	tm_2nd_num = timing_2nd_num;
	panel_tm_arry = NULL;
	tm_cmd_map_arry = NULL;
	spec_tm_switch_inst = NULL;
	tm_2nd_list = NULL;
	cur_tm_index = 0;
	new_tm_index = 0;
	last_pt_tm_index = 0;

	IRIS_LOGI("%s(), panel timing num: %d, special timing switch case num: %d, 2nd timing num: %d",
			__func__, pnl_tm_num, special_switch_num, timing_2nd_num);
	if (panel_tm_num > 1) {
		u32 buf_size = panel_tm_num
			* sizeof(struct dsi_mode_info);
		panel_tm_arry = vzalloc(buf_size);
		tm_cmd_map_arry = vzalloc(panel_tm_num);
	}

	if (spec_tm_switch_num > 0)
		spec_tm_switch_inst = vzalloc(spec_tm_switch_num * sizeof(u8));

	if (tm_2nd_num > 0)
		tm_2nd_list = vzalloc(tm_2nd_num * sizeof(u8));
}

static int32_t _iris_parse_timing_cmd_map(struct device_node *np)
{
	int32_t rc = 0;

	rc = of_property_read_u8_array(np, "pxlw,timing-cmd-map",
			tm_cmd_map_arry, panel_tm_num);
	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse timing cmd map", __func__);
		return rc;
	}

	return rc;
}

static int32_t _iris_parse_special_timing_switch_instance(
		struct device_node *np)
{
	int32_t rc = 0;
	uint32_t i = 0;

	if (spec_tm_switch_num == 0)
		return 0;

	rc = of_property_read_u8_array(np, "pxlw,special-timing-switch-instance",
			spec_tm_switch_inst, spec_tm_switch_num);
	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse timing cmd map", __func__);
		return rc;
	}

	for (i = 0; i < spec_tm_switch_num; i++) {
		u8 val = spec_tm_switch_inst[i];
		if (val >= 16)
			spec_tm_switch_inst[i] = val / 16 * 10 + val % 16;

		IRIS_LOGI("%s(), special switch case[%d] = %02d",
				__func__,
				i, spec_tm_switch_inst[i]);
	}

	return rc;
}

static int32_t _iris_parse_2nd_timing_list(struct device_node *np)
{
	int32_t rc = 0;
	uint32_t i = 0;

	if (tm_2nd_num == 0)
		return 0;

	rc = of_property_read_u8_array(np, "pxlw,timing-2nd-list",
			tm_2nd_list, tm_2nd_num);
	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse 2nd timing list", __func__);
		return rc;
	}

	for (i = 0; i < tm_2nd_num; i++) {
		u8 val = tm_2nd_list[i];
		if (val >= 16)
			tm_2nd_list[i] = val / 16 * 10 + val % 16;

		IRIS_LOGI("%s(), 2nd timing[%d] = %02d",
				__func__,
				i, tm_2nd_list[i]);
	}

	return rc;
}

static int32_t _iris_parse_tm_switch_seq(struct device_node *np)
{
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-timing-switch-sequence";

	rc = iris_parse_optional_seq(np, key, &tm_switch_seq[TIMING_SWITCH_NORMAL_SEQ]);
	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse %s seq", __func__, key);
		return rc;
	}

	if (spec_tm_switch_num == 0)
		return 0;

	key = "pxlw,iris-timing-switch-sequence-1";
	rc = iris_parse_optional_seq(np, key, &tm_switch_seq[TIMING_SWITCH_SPECIAL_SEQ]);
	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse %s seq", __func__, key);
		return rc;
	}

	return rc;
}

int32_t iris_parse_timing_switch_info(struct device_node *np)
{
	int32_t rc = 0;
	int32_t cmd_idx = 0;

	_iris_init_param(np);

	if (panel_tm_num == 0 || panel_tm_num == 1)
		return 0;

	rc = _iris_parse_timing_cmd_map(np);
	if (rc)
		IRIS_LOGI("%s(), [optional] have not timing cmd map", __func__);

	rc = _iris_parse_special_timing_switch_instance(np);
	if (rc)
		IRIS_LOGI("%s(), [optional] have not special timing switch instance", __func__);

	rc = _iris_parse_2nd_timing_list(np);
	if (rc)
		IRIS_LOGI("%s(), [optional] have not 2nd timing list", __func__);

	rc = _iris_parse_tm_switch_seq(np);
	if (rc)
		IRIS_LOGI("%s(), [optional] have not timing switch sequence", __func__);

	for (cmd_idx = IRIS_DTSI_PIP_IDX_START + 1; cmd_idx <= IRIS_DTSI_PIP_IDX_END; cmd_idx++) {
		rc = iris_parse_dtsi_cmd(np, cmd_idx);
		if (rc)
			IRIS_LOGI("%s(), [optional] have not cmds list %d", __func__, cmd_idx);
	}

	return 0;
}

static bool _iris_is_sepcial_switch(void)
{
	u8 switch_instance = (last_pt_tm_index * 10 + new_tm_index) & 0xFF;
	u32 i = 0;

	IRIS_LOGI("%s(), switch instance: %02d, from timing@%u to timing@%u",
			__func__,
			switch_instance,
			last_pt_tm_index,
			new_tm_index);
	if (spec_tm_switch_num == 0 || spec_tm_switch_inst == NULL)
		return false;

	for (i = 0; i < spec_tm_switch_num; i++) {
		if (switch_instance == spec_tm_switch_inst[i])
			return true;
	}

	return false;
}

void iris_send_timing_switch_pkt(void)
{
	struct iris_ctrl_seq *pseq = &tm_switch_seq[TIMING_SWITCH_NORMAL_SEQ];
	struct iris_ctrl_opt *arr = NULL;

	//SDE_ATRACE_BEGIN("iris_send_timing_switch_pkt");
	IRIS_LOGI("%s(), cmd list index: %02x", __func__, cmd_list_index);
	if (_iris_is_sepcial_switch())
		pseq = &tm_switch_seq[TIMING_SWITCH_SPECIAL_SEQ];

	if (pseq == NULL) {
		IRIS_LOGE("%s(), seq is NULL", __func__);
		//SDE_ATRACE_END("iris_send_timing_switch_pkt");
		return;
	}
	arr = pseq->ctrl_opt;

	iris_send_assembled_pkt(arr, pseq->cnt);
	udelay(100);
	//SDE_ATRACE_END("iris_send_timing_switch_pkt");
}

static uint32_t _iris_get_timing_index(const struct dsi_mode_info *timing)
{
	uint32_t i = 0;

	if (!_iris_support_timing_switch())
		return 0;

	for (i = 0; i < panel_tm_num; i++) {
		struct dsi_mode_info *t = &panel_tm_arry[i];

		if (timing->v_active == t->v_active &&
			timing->h_active == t->h_active &&
			timing->refresh_rate == t->refresh_rate)
			return i;
	}

	return 0;
}

static uint32_t _iris_generate_switch_case(const struct dsi_mode_info *new_timing)
{
	bool cur_pt_mode = false;
	u32 new_cmd_list_idx = 0;

	if (!_iris_support_timing_switch())
		return SWITCH_ABYP_TO_ABYP;

	cur_pt_mode = iris_is_pt_mode();
	new_tm_index = _iris_get_timing_index(new_timing);
	new_cmd_list_idx = tm_cmd_map_arry[new_tm_index];

	IRIS_LOGI("%s(), for new timing: %ux%u@%u, index: timing@%u, new cmd list index: %u, iris current mode '%s'",
			__func__,
			new_timing->h_active,
			new_timing->v_active,
			new_timing->refresh_rate,
			new_tm_index,
			new_cmd_list_idx,
			cur_pt_mode ? "PT" : "ABYP");
	if (new_cmd_list_idx != IRIS_DTSI_NONE)
		cmd_list_index = new_cmd_list_idx;

	if (cur_pt_mode) {
		if (new_cmd_list_idx == IRIS_DTSI_NONE)
			return SWITCH_PT_TO_ABYP;

		return SWITCH_PT_TO_PT;
	}

	return SWITCH_ABYP_TO_ABYP;
}

static bool _iris_is_clock_switched(const struct dsi_mode_info *new_timing)
{
	struct dsi_mode_info *cur_timing = &panel_tm_arry[cur_tm_index];

	IRIS_LOGD("%s(), switch clock from %llu to %llu",
			__func__,
			cur_timing->clk_rate_hz, new_timing->clk_rate_hz);

	if (cur_timing->clk_rate_hz != new_timing->clk_rate_hz)
		return true;

	return false;
}

static bool _iris_is_res_switched(const struct dsi_mode_info *new_timing)
{
	struct dsi_mode_info *cur_timing = &panel_tm_arry[cur_tm_index];

	IRIS_LOGD("%s(), switch resolution from %ux%u to %ux%u",
			__func__,
			cur_timing->h_active, cur_timing->v_active,
			new_timing->h_active, new_timing->v_active);

	if (cur_timing->h_active != new_timing->h_active
			|| cur_timing->v_active != new_timing->v_active)
		return true;

	return false;
}

static void _iris_switch_res(const struct dsi_mode_info *new_timing)
{
	if (!_iris_is_res_switched(new_timing))
		return;

	if (_iris_is_clock_switched(new_timing))
		return;

	iris_send_timing_switch_pkt();
}

static bool _iris_is_freq_switched(const struct dsi_mode_info *new_timing)
{
	struct dsi_mode_info *cur_timing = &panel_tm_arry[cur_tm_index];

	IRIS_LOGD("%s(), switch framerate from %u to %u",
			__func__,
			cur_timing->refresh_rate, new_timing->refresh_rate);

	if (cur_timing->refresh_rate != new_timing->refresh_rate)
		return true;

	return false;
}

bool iris_is_same_timing_from_last_pt(void)
{
	if (!_iris_support_timing_switch())
		return true;

	return new_tm_index == last_pt_tm_index;
}

bool iris_is_clk_switched_from_last_pt(void)
{
	if (!_iris_support_timing_switch())
		return false;

	if (new_tm_index == last_pt_tm_index)
		return false;

	if (panel_tm_arry[new_tm_index].clk_rate_hz
			!= panel_tm_arry[last_pt_tm_index].clk_rate_hz)
		return true;

	return false;
}

bool iris_is_res_switched_from_last_pt(void)
{
	if (!_iris_support_timing_switch())
		return false;

	if (new_tm_index == last_pt_tm_index)
		return false;

	if ((panel_tm_arry[new_tm_index].h_active
				!= panel_tm_arry[last_pt_tm_index].h_active)
			|| (panel_tm_arry[new_tm_index].v_active
				!= panel_tm_arry[last_pt_tm_index].v_active))
		return true;

	return false;
}

bool iris_is_freq_switched_from_last_pt(void)
{
	if (!_iris_support_timing_switch())
		return false;

	if (new_tm_index == last_pt_tm_index)
		return false;

	if (panel_tm_arry[new_tm_index].refresh_rate
			!= panel_tm_arry[last_pt_tm_index].refresh_rate)
		return true;

	return false;
}

bool iris_is_between_main_2nd_from_last_pt(void)
{
	if (!_iris_support_timing_switch())
		return false;

	if (new_tm_index == last_pt_tm_index)
		return false;

	if (tm_cmd_map_arry[new_tm_index] == tm_cmd_map_arry[cur_tm_index])
		return true;

	return false;
}

void iris_update_last_pt_timing_index(void)
{
	last_pt_tm_index = new_tm_index;
}

static bool _iris_is_2nd_timing_enable(void)
{
	if (tm_2nd_num > 0 && tm_2nd_list)
		return true;

	return false;
}

static bool _iris_between_main_2nd(const struct dsi_mode_info *new_timing)
{
	uint32_t new_tm_idx = 0;

	if (!_iris_is_2nd_timing_enable())
		return false;

	if (!new_timing)
		return false;

	new_tm_idx = _iris_get_timing_index(new_timing);
	if (new_tm_idx == cur_tm_index)
		return false;

	if (tm_cmd_map_arry[new_tm_idx] == tm_cmd_map_arry[cur_tm_index]) {
		IRIS_LOGI("%s(), switch between main and 2nd, from %u to %u",
				__func__, cur_tm_index, new_tm_idx);
		return true;
	}

	return false;
}

bool iris_belongs_to_2nd_timing(const struct dsi_mode_info *new_timing)
{
	uint32_t new_tm_idx = 0;
	uint32_t i = 0;

	if (!new_timing)
		return false;

	if (panel_tm_num == 0 || tm_2nd_num == 0)
		return false;

	new_tm_idx = _iris_get_timing_index(new_timing);
	for (i = 0; i < tm_2nd_num; i++) {
		if (new_tm_idx == tm_2nd_list[i])
			return true;
	}

	return false;
}

bool iris_is_abyp_timing(const struct dsi_mode_info *new_timing)
{
	uint32_t tm_index = 0;

	if (!new_timing)
		return false;

	if (tm_cmd_map_arry == NULL)
		return false;

	tm_index = _iris_get_timing_index(new_timing);
	if (tm_cmd_map_arry[tm_index] == IRIS_DTSI_NONE)
		return true;

	return false;
}

void iris_update_cur_timing(const struct dsi_mode_info *cur_timing)
{
	if (!cur_timing)
		return;

	cur_tm_index = _iris_get_timing_index(cur_timing);

	IRIS_LOGI("%s(), current timing %ux%u@%u, index: %u",
			__func__,
			cur_timing->h_active, cur_timing->v_active,
			cur_timing->refresh_rate, cur_tm_index);
}

static void _iris_switch_clock_rate(const struct dsi_mode_info *new_timing)
{
	if (!new_timing)
		return;

	if (!_iris_is_clock_switched(new_timing))
		return;

	IRIS_LOGI("%s(), switch clock to %llu",
			__func__, new_timing->clk_rate_hz);

	if (_iris_between_main_2nd(new_timing))
		return;

	iris_send_timing_switch_pkt();

	iris_set_out_frame_rate(new_timing->refresh_rate);
	iris_update_frc_fps(new_timing->refresh_rate & 0xFF);
}

static void _iris_switch_freq(const struct dsi_mode_info *new_timing)
{
	bool use_2nd_timing = false;

	if (!new_timing)
		return;

	if (!_iris_is_freq_switched(new_timing))
		return;

	if (_iris_is_clock_switched(new_timing) && !_iris_between_main_2nd(new_timing))
		return;

	use_2nd_timing = iris_belongs_to_2nd_timing(new_timing);

	if (_iris_is_2nd_timing_enable()) {

		iris_send_ipopt_cmds(IRIS_IP_SYS, use_2nd_timing ? 0xA1 : 0xA0);
		iris_send_ipopt_cmds(IRIS_IP_SYS, use_2nd_timing ? 0x28 : 0x27);
		udelay(100);
		iris_send_ipopt_cmds(IRIS_IP_RX, use_2nd_timing ? 0xF2 : 0xF1);
		iris_send_ipopt_cmds(IRIS_IP_RX, use_2nd_timing ? 0xE1 : 0xE0);
		iris_send_ipopt_cmds(IRIS_IP_TX, use_2nd_timing ? 0x4 : 0x0);
	}

	iris_set_out_frame_rate(new_timing->refresh_rate);

	if (_iris_is_2nd_timing_enable()) {
		iris_send_ipopt_cmds(IRIS_IP_RX, use_2nd_timing ? 0xF2 : 0xF1);
		if (iris_is_dual_supported()) {
			iris_send_ipopt_cmds(IRIS_IP_RX_2, use_2nd_timing ? 0xF2 : 0xF1);
			iris_send_ipopt_cmds(IRIS_IP_BLEND, use_2nd_timing ? 0xF1 : 0xF0);
		}
		udelay(2000); //delay 2ms
	}

	iris_update_frc_fps(new_timing->refresh_rate & 0xFF);
}

uint32_t iris_get_cmd_list_index(void)
{
	return cmd_list_index;
}

void iris_sync_bitmask(struct iris_update_regval *pregval)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t orig_val = 0;
	uint32_t *data = NULL;
	uint32_t val = 0;
	struct iris_ip_opt *popt = NULL;
	int32_t type = 0;

	if (!_iris_support_timing_switch())
		return;

	if (!pregval) {
		IRIS_LOGE("%s(), invalid input", __func__);
		return;
	}

	ip = pregval->ip;
	opt_id = pregval->opt_id;

	for (type = IRIS_DTSI_PIP_IDX_START; type <= IRIS_DTSI_PIP_IDX_END; type++) {
		if (type == iris_get_cmd_list_index())
			continue;

		popt = iris_find_specific_ip_opt(ip, opt_id, type);
		if (popt == NULL) {
			IRIS_LOGW("%s(), can't find ip: 0x%02x opt: 0x%02x, from type: %d",
					__func__, ip, opt_id, type);
			continue;
		} else if (popt->cmd_cnt != 1) {
			IRIS_LOGW("%s(), invalid bitmask for ip: 0x%02x, opt: 0x%02x, type: %d, popt len: %d",
					__func__, ip, opt_id, type, popt->cmd_cnt);
			continue;
		}

		data = (uint32_t *)popt->cmd[0].msg.tx_buf;

		orig_val = cpu_to_le32(data[2]);
		val = orig_val & (~pregval->mask);
		val |= (pregval->value & pregval->mask);
		data[2] = val;
	}
}

int iris_post_switch(
		struct iris_cmd_set *switch_cmds,
		struct dsi_mode_info *new_timing)
{
	int rc = 0;
	int lightup_opt = iris_lightup_opt_get();

	//SDE_ATRACE_BEGIN("iris_post_switch");
	switch_case = _iris_generate_switch_case(new_timing);
	IRIS_LOGI("%s(), post switch to: %ux%u@%uHz, cmd list index: %u, switch case: %s",
			__func__,
			new_timing->h_active,
			new_timing->v_active,
			new_timing->refresh_rate,
			cmd_list_index,
			switch_case_name[switch_case]);

	if (switch_cmds == NULL) {
		//SDE_ATRACE_END("iris_post_switch");
		return 0;
	}

	if (lightup_opt & 0x8) {
		rc = iris_dsi_send_cmds(switch_cmds->cmds, switch_cmds->count, switch_cmds->state);
		IRIS_LOGI("%s(), post switch Force ABYP", __func__);
		//SDE_ATRACE_END("iris_post_switch");
		return rc;
	}

	if (iris_is_pt_mode())
		rc = iris_pt_send_panel_cmd(switch_cmds);
	else
		rc = iris_dsi_send_cmds(switch_cmds->cmds, switch_cmds->count, switch_cmds->state);

	//SDE_ATRACE_END("iris_post_switch");
	IRIS_LOGD("%s(), return %d", __func__, rc);

	return rc;
}

static int _iris_switch(
		struct iris_cmd_set *switch_cmds,
		struct dsi_mode_info *new_timing)
{
	int rc = 0;
	int lightup_opt = iris_lightup_opt_get();
	u32 refresh_rate = new_timing->refresh_rate;
	ktime_t ktime = 0;

	IRIS_LOGI("%s(), new timing index: timing@%u", __func__, new_tm_index);

	if (IRIS_IF_LOGI())
		ktime = ktime_get();

	//SDE_ATRACE_BEGIN("iris_switch");
	iris_update_panel_ap_te(refresh_rate);

	if (lightup_opt & 0x8) {
		if (switch_cmds)
			rc = iris_dsi_send_cmds(switch_cmds->cmds, switch_cmds->count, switch_cmds->state);
		IRIS_LOGI("%s(), force switch from ABYP to ABYP, total cost '%d us'",
				__func__,
				(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));
		//SDE_ATRACE_END("iris_switch");
		return rc;
	}

	if (switch_case == SWITCH_ABYP_TO_ABYP) {
		if (switch_cmds)
			rc = iris_dsi_send_cmds(switch_cmds->cmds, switch_cmds->count, switch_cmds->state);
	}
	if (switch_case == SWITCH_PT_TO_PT) {
		if (switch_cmds)
			rc = iris_pt_send_panel_cmd(switch_cmds);
		_iris_switch_clock_rate(new_timing);
		_iris_switch_freq(new_timing);
		_iris_switch_res(new_timing);
		if (0)//panel->qsync_mode > 0)
			iris_qsync_set(true);
		iris_update_last_pt_timing_index();
	}

	if (switch_case == SWITCH_PT_TO_ABYP) {
		iris_abypass_switch_proc(ANALOG_BYPASS_MODE, false, true);
		if (switch_cmds)
			rc = iris_dsi_send_cmds(switch_cmds->cmds, switch_cmds->count, switch_cmds->state);
	}

	cur_tm_index = new_tm_index;

	//SDE_ATRACE_END("iris_switch");
	IRIS_LOGI("%s(), return %d, total cost '%d us'",
			__func__,
			rc, (u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

	return rc;
}

int iris_switch(void *dev, int id)
{
	int ret = -EINVAL;

	if(dev) {
		struct mtk_drm_crtc *mtk_crtc = (struct mtk_drm_crtc *)dev;
		struct drm_display_mode *dsp_mode = NULL;
		struct dsi_mode_info new_timing;

		dsp_mode = mtk_drm_crtc_avail_disp_mode(&mtk_crtc->base, id);
		iris_get_cfg()->dsp_mode = dsp_mode;
		iris_sync_timing(&new_timing, dsp_mode);
		switch_case = _iris_generate_switch_case(&new_timing);
		iris_sync_cur_timing();
		ret = _iris_switch(NULL, &new_timing);
	}
	return ret;
}


uint32_t iris_get_cont_type_with_timing_switch(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t type = IRIS_CONT_SPLASH_NONE;
	uint32_t sw_case = SWITCH_NONE;

	if (pcfg->valid >= PARAM_PARSED)
		sw_case = _iris_generate_switch_case(&pcfg->timing);

	IRIS_LOGI("%s(), switch case: %s, rate: %d, v: %d",
			__func__,
			switch_case_name[sw_case],
			pcfg->timing.refresh_rate,
			pcfg->timing.v_active);

	switch (sw_case) {
	case SWITCH_PT_TO_PT:
		type = IRIS_CONT_SPLASH_LK;
		break;
	case SWITCH_ABYP_TO_ABYP:
	case SWITCH_ABYP_TO_PT:
		type = IRIS_CONT_SPLASH_BYPASS_PRELOAD;
		break;
	case SWITCH_PT_TO_ABYP:
		// This case does not happen
	default:
		type = IRIS_CONT_SPLASH_NONE;
		break;
	}

	return type;
}

bool iris_same_resolution_diff_refresh(int32_t cmd_list_index)
{
	uint32_t tm_index = 0;
	struct dsi_mode_info *cur_timing = &panel_tm_arry[cur_tm_index];
	struct dsi_mode_info *timing;

	if (!_iris_support_timing_switch())
		return false;

	if (tm_cmd_map_arry == NULL)
		return false;


	for (tm_index = 0; tm_index < panel_tm_num; tm_index++) {
		if (tm_cmd_map_arry[tm_index] == cmd_list_index)
			break;
	}
	if (tm_index >= panel_tm_num)
		return false;

	timing = &panel_tm_arry[tm_index];
	if (cur_timing->v_active == timing->v_active &&
	    cur_timing->h_active == timing->h_active) {
		if (cur_timing->refresh_rate != timing->refresh_rate)
			return true;
		else
			return false;
	}
	return false;
}
