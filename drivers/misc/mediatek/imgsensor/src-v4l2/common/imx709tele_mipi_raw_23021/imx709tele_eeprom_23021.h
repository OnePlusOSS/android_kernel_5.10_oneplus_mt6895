/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __IMX709TELE_EEPROM_23021_H__
#define __IMX709TELE_EEPROM_23021_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"
#include "kd_eeprom_oplus.h"

#define Sleep(ms) mdelay(ms)

#define IMX709TELE_EEPROM_SLAVE_ADDRESS 0xA0
#define IMX709TELE_MAX_OFFSET 0xFFFF
#define IMX709TELE_EEPROM_LOCK_REGISTER_23021  (0xE000)

#define LRC_EEPROM_FLAG_ADDR  0x2104
#define OTP_LRC_OFFSET 0x2000
#define LRC_SIZE 260

#define OTP_QSC_OFFSET 0x1900
#define QSC_SIZE 1560

#define IMX709TELE_STEREO_START_ADDR_23021  (0x3100)

struct EEPROM_PDAF_INFO {
    kal_uint16 LRC_addr;
    unsigned int LRC_size;
};

/*
 * LRC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx709tele_LRC_23021(struct subdrv_ctx *ctx, kal_uint16 *data);

unsigned int read_imx709tele_QSC_23021(struct subdrv_ctx *ctx, kal_uint8 *data);

bool read_imx709tele_eeprom_23021(struct subdrv_ctx *ctx, kal_uint16 addr,
                   BYTE *data, int size);

unsigned int read_imx709tele_eeprom_info_23021(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);
#endif
