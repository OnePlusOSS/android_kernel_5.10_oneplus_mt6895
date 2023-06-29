/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __S5K3P9SP_EEPROM_21143_H__
#define __S5K3P9SP_EEPROM_21143_H__

#include "kd_camera_typedef.h"
#include "kd_eeprom_oplus.h"
#include "adaptor-subdrv.h"

unsigned int read_s5k3p9sp_eeprom_info_21143(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);

#endif