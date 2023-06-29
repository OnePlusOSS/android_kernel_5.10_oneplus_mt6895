/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_API_H_
#define _DSI_IRIS_API_H_

#include "iris_def.h"
#include "iris_log.h"

int iris_pt_send_panel_cmd(struct iris_cmd_set *cmdset);
int iris_enable(struct iris_cmd_set *on_cmds);
int iris_disable(struct iris_cmd_set *off_cmds);
void iris_set_valid(int step);
int iris_conver_one_panel_cmd(u8 *dest, u8 *src, int max);
void iris_panel_cmd_passthrough(unsigned int cmd, unsigned char count, unsigned char *para_list,
	uint8_t *buffer, uint8_t buffer_size, unsigned char rd_cmd);

int iris_parse_param(void *dev);

bool iris_is_pt_mode(void);
int iris_get_hdr_enable(void);


int iris_check_hdr_backlight(u8 *bl, u32 len);
int iris_dbgfs_status_init(void);
void iris_dbgfs_status_deinit(void);

void iris_ddp_mutex_lock(void);
void iris_ddp_mutex_unlock(void);

void iris_init_panel_timing(void *dev);
void iris_query_capability(void);

#endif // _DSI_IRIS_API_H_
