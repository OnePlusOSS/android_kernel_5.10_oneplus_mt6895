/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __IMX350_EEPROM_21641_H__
#define __IMX350_EEPROM_21641_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"

#include "kd_eeprom_oplus.h"

void imx355_read_SPC_21641(struct subdrv_ctx *ctx, u8 *data);
void imx355_read_DCC_21641(struct subdrv_ctx *ctx,
        kal_uint16 addr, u8 *data, kal_uint32 size);
unsigned int read_imx355_eeprom_info_21641(struct subdrv_ctx *ctx, kal_uint16 meta_id,
				     BYTE *data, int size);
#endif