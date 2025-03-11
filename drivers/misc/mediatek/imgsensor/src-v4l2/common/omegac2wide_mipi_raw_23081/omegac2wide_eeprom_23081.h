/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __OMEGAC2WIDE_23081_EEPROM_H__
#define __OMEGAC2WIDE_23081_EEPROM_H__

#include "kd_camera_typedef.h"
#include "kd_eeprom_oplus.h"
#include "adaptor-subdrv.h"

void omegac2wide_read_SPC_23081(struct subdrv_ctx *ctx, u8 *data);
void omegac2wide_read_DCC_23081(struct subdrv_ctx *ctx,
        kal_uint16 addr, u8 *data, kal_uint32 size);
bool omegac2wide_read_eeprom_23081(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size);
unsigned int read_omegac2wide_eeprom_info_23081(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);
bool omegac2wide_read_eeprom_state_23081(struct subdrv_ctx *ctx);
#endif

