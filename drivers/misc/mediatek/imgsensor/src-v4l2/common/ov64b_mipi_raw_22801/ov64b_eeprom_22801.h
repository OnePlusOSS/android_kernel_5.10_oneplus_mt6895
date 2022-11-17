/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __OV64B_EEPROM_22801_H__
#define __OV64B_EEPROM_22801_H__

#include "kd_camera_typedef.h"
#include "kd_eeprom_oplus.h"
#include "adaptor-subdrv.h"

unsigned int read_ov64b_eeprom_info_22801(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);

#endif