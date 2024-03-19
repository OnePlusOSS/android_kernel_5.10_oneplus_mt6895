/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __GC02M1_EEPROM_21143_H__
#define __GC02M1_EEPROM_21143_H__

#include "kd_camera_typedef.h"
#include "kd_eeprom_oplus.h"
#include "adaptor-subdrv.h"

unsigned int read_gc02m1_eeprom_info_21143(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);

#endif //__GC02M1_EEPROM_21143_H__
