/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __OMEGAC2FRONT_EEPROM_23081_H__
#define __OMEGAC2FRONT_EEPROM_23081_H__

#include "kd_camera_typedef.h"
#include "kd_eeprom_oplus.h"
#include "adaptor-subdrv.h"

unsigned int read_omegac2front_eeprom_info_23081(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);
bool omegac2front_read_eeprom_state_23081(struct subdrv_ctx *ctx);

#endif