/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef __DSI_IRIS_TIMING_SWITCH__
#define __DSI_IRIS_TIMING_SWITCH__


void iris_init_timing_switch(void);
void iris_deinit_timing_switch(void);
int32_t iris_parse_timing_switch_info(struct device_node *np);
void iris_send_timing_switch_pkt(void);
uint32_t iris_get_cont_type_with_timing_switch(void);
bool iris_belongs_to_2nd_timing(const struct dsi_mode_info *new_timing);
bool iris_is_abyp_timing(const struct dsi_mode_info *new_timing);
void iris_update_cur_timing(const struct dsi_mode_info *cur_timing);
bool iris_is_same_timing_from_last_pt(void);
bool iris_is_clk_switched_from_last_pt(void);
bool iris_is_res_switched_from_last_pt(void);
bool iris_is_freq_switched_from_last_pt(void);
bool iris_is_between_main_2nd_from_last_pt(void);
void iris_update_last_pt_timing_index(void);
uint32_t iris_get_cmd_list_index(void);
void iris_sync_bitmask(struct iris_update_regval *pregval);
uint32_t iris_generate_switch_case(const struct dsi_mode_info *new_timing);

#endif //__DSI_IRIS_TIMING_SWITCH__
