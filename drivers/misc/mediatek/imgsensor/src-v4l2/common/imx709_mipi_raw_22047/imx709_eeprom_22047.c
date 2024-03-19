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

#define PFX "IMX709_pdafotp"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"
#include "imx709mipiraw_Sensor_22047.h"
#include "imx709_eeprom_22047.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

static struct EEPROM_PDAF_INFO eeprom_pdaf_info[] = {
    {
        .LRC_addr = OTP_LRC_OFFSET,
        .LRC_size = LRC_SIZE,
    },
};

static DEFINE_MUTEX(gimx709_eeprom_mutex);

bool read_imx709_eeprom_22047(struct subdrv_ctx *ctx, kal_uint16 addr,
                   BYTE *data, int size)
{
    if (addr + size > IMX709_MAX_OFFSET) {
        return false;
    }

    if (adaptor_i2c_rd_p8(ctx->i2c_client,
            IMX709_EEPROM_SLAVE_ADDRESS >> 1, addr, data, size) < 0) {
        return false;
    }

    return true;
}

unsigned int read_imx709_LRC_22047(struct subdrv_ctx *ctx, kal_uint16 *data)
{
    kal_uint16 idx = 0, sensor_startL_reg = 0xCE00,
           sensor_startR_reg = 0xCF00;
    static BYTE imx709_LRC_data[LRC_SIZE] = { 0 };
    static unsigned int readed_size = 0;
    struct EEPROM_PDAF_INFO *pinfo =
        (struct EEPROM_PDAF_INFO *)&eeprom_pdaf_info[0];
    kal_uint8 lrc_flag = 0;

    LOG_INF("read imx709 LRC, otp_offset = %d, size = %u\n",
        pinfo->LRC_addr, pinfo->LRC_size);

    mutex_lock(&gimx709_eeprom_mutex);
    if ((readed_size == 0) &&
        read_imx709_eeprom_22047(ctx, pinfo->LRC_addr, imx709_LRC_data,
                   pinfo->LRC_size)) {
        readed_size = pinfo->LRC_size;
    }
    mutex_unlock(&gimx709_eeprom_mutex);

    for (idx = 0; idx < LRC_SIZE; idx++) {
        if (idx < LRC_SIZE / 2) {
            //LRC_Left
            data[2 * idx] = sensor_startL_reg++;
        } else {
            //LRC_Right
            data[2 * idx] = sensor_startR_reg++;
        }
        data[2 * idx + 1] = imx709_LRC_data[idx];
    }

    for (idx = 0; idx < LRC_SIZE; idx++) {
        if (idx < LRC_SIZE / 2) {
            //LRC_Left
            LOG_INF("In %s: LRC_Left value[0x%x]:0x%x", __func__,
                 data[2 * idx], data[2 * idx + 1]);
        } else {
            //LRC_RIGHT
            LOG_INF("In %s: LRC_Right value[0x%x]:0x%x", __func__,
                 data[2 * idx], data[2 * idx + 1]);
        }
    }
    read_imx709_eeprom_22047(ctx, pinfo->LRC_addr + pinfo->LRC_size, &lrc_flag, 1);
    LOG_INF("LRC flag0x%x[1:valid, other:Invalid]", lrc_flag);
    return readed_size;
}

unsigned int read_imx709_QSC_22047(struct subdrv_ctx *ctx, kal_uint8 *data)
{
    kal_uint16 idx = 0;
    kal_uint8 qsc_ver = 0;

    read_imx709_eeprom_22047(ctx, OTP_QSC_OFFSET, data, QSC_SIZE);

    for (idx = 0; idx < QSC_SIZE; idx++) {
        LOG_INF("alex qsc data imx709_QSC_setting[0x%x] = 0x%x", idx, data[idx]);
    }

    read_imx709_eeprom_22047(ctx, 0x2A32, &qsc_ver, 1);
    LOG_INF("QSC Version: 0x%x", qsc_ver);
    return 0;
}

struct eeprom_map_info imx709_eeprom_info_22047[] = {
    { EEPROM_META_MODULE_ID, 0x0000, 0x000F, 0x0010, 2, true },
    { EEPROM_META_SENSOR_ID, 0x0006, 0x000F, 0x0010, 2, true },
    { EEPROM_META_LENS_ID, 0x0008, 0x000F, 0x0010, 2, true },
    { EEPROM_META_VCM_ID, 0x000A, 0x000F, 0x0010, 2, true },
    { EEPROM_META_MIRROR_FLIP, 0x000E, 0x000F, 0x0010, 1, true },
    { EEPROM_META_MODULE_SN, 0x00B0, 0x00C1, 0x00C2, 17, true },
    { EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
};

unsigned int read_imx709_eeprom_info_22047(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size)
{
    kal_uint16 addr;
    int readsize;

    if (meta_id != imx709_eeprom_info_22047[meta_id].meta)
        return -1;

    if (size != imx709_eeprom_info_22047[meta_id].size)
        return -1;

    addr = imx709_eeprom_info_22047[meta_id].start;
    readsize = imx709_eeprom_info_22047[meta_id].size;

    read_imx709_eeprom_22047(ctx, addr, data, readsize);

    return 0;
}
