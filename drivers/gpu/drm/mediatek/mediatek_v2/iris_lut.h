/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_LUT_H_
#define _DSI_IRIS_LUT_H_

enum {
	LOAD_CALIBRATED_OR_GOLDEN = 0,
	LOAD_GOLDEN_ONLY,
	LOAD_CALIBRATED_ONLY,
	LOAD_METHOD_CNT
};

int iris_parse_lut_cmds(uint32_t flag);
int iris_send_lut(u8 lut_type, u8 lut_table_index, u32 lut_abtable_index);
void iris_update_ambient_lut(enum LUT_TYPE lutType, u32 lutPos);
void iris_update_maxcll_lut(enum LUT_TYPE lutType, u32 lutpos);
u8 iris_get_fw_load_status(void);
void iris_update_fw_load_status(u8 value);
void iris_update_gamma(void);
int iris_dbgfs_fw_calibrate_status_init(void);

#endif // _DSI_IRIS_LUT_H_
