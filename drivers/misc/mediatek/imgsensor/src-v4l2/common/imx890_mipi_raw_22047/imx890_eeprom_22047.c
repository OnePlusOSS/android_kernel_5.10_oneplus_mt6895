// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     imx890_eeprom.c
 *
 * Project:
 * --------
 * Description:
 * ------------
 *     Add APIs to read from EEPROM
 *
 ****************************************************************************/
#define PFX "IMX890_pdafotp_22047"
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
#include "imx890mipiraw_Sensor_22047.h"
#include "imx890_eeprom_22047.h"
#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define Sleep(ms) mdelay(ms)

struct EEPROM_PDAF_INFO {
    kal_uint16 LRC_addr;
    unsigned int LRC_size;
    kal_uint16 DCC_addr;
    unsigned int DCC_size;
};

enum EEPROM_PDAF_INFO_FMT {
    MTK_FMT = 0,
    OP_FMT,
    FMT_MAX
};

static struct EEPROM_PDAF_INFO eeprom_pdaf_info[] = {
    {/* MTK_FMT */
        .LRC_addr = 0x14FE,
        .LRC_size = LRC_SIZE,
        .DCC_addr = 0x763,
        .DCC_size = DCC_SIZE
    },
    {/* OP_FMT */
        .LRC_addr = 0x700,
        .LRC_size = LRC_SIZE,
        .DCC_addr = 0xB06,
        .DCC_size = DCC_SIZE
    },
};

static DEFINE_MUTEX(gimx890_eeprom_mutex_22047);

bool read_imx890_eeprom_22047(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size)
{
    if (addr + size > IMX890_MAX_OFFSET_22047) {
        return false;
    }

    if (adaptor_i2c_rd_p8(ctx->i2c_client,
            IMX890_EEPROM_READ_ID_22047 >> 1, addr, data, size) < 0) {
        return false;
    }

    return true;
}

static struct EEPROM_PDAF_INFO *get_eeprom_pdaf_info(struct subdrv_ctx *ctx)
{
    static struct EEPROM_PDAF_INFO *pinfo;
    BYTE read_data[4];

    mutex_lock(&gimx890_eeprom_mutex_22047);
    if (pinfo == NULL) {
        read_imx890_eeprom_22047(ctx, 0x1, read_data, 4);
        if (((read_data[3] << 24) |
             (read_data[2] << 16) |
             (read_data[1] << 8) |
             read_data[0]) == MTK_IDENTITY_VALUE) {
            pinfo = &eeprom_pdaf_info[MTK_FMT];
        } else {
            pinfo = &eeprom_pdaf_info[OP_FMT];
        }
    }
    mutex_unlock(&gimx890_eeprom_mutex_22047);

    return pinfo;
}

unsigned int read_imx890_LRC_22047(struct subdrv_ctx *ctx, BYTE *data)
{
    static BYTE IMX890_LRC_data_22047[LRC_SIZE] = { 0 };
    static unsigned int readed_size;
    struct EEPROM_PDAF_INFO *pinfo = get_eeprom_pdaf_info(ctx);

    LOG_INF("read imx890 LRC, addr = %d, size = %u\n",
        pinfo->LRC_addr, pinfo->LRC_size);

    mutex_lock(&gimx890_eeprom_mutex_22047);
    if ((readed_size == 0) &&
        read_imx890_eeprom_22047(ctx, pinfo->LRC_addr,
                   IMX890_LRC_data_22047, pinfo->LRC_size)) {
        readed_size = pinfo->LRC_size;
    }
    mutex_unlock(&gimx890_eeprom_mutex_22047);

    memcpy(data, IMX890_LRC_data_22047, pinfo->LRC_size);
    return readed_size;
}

unsigned int read_imx890_DCC_22047(struct subdrv_ctx *ctx, BYTE *data)
{
    static BYTE IMX890_DCC_data_22047[DCC_SIZE] = { 0 };
    static unsigned int readed_size;
    struct EEPROM_PDAF_INFO *pinfo = get_eeprom_pdaf_info(ctx);

    LOG_INF("read imx890 DCC, addr = %d, size = %u\n",
        pinfo->DCC_addr, pinfo->DCC_size);

    mutex_lock(&gimx890_eeprom_mutex_22047);
    if ((readed_size == 0) &&
        read_imx890_eeprom_22047(ctx, pinfo->DCC_addr,
                   IMX890_DCC_data_22047, pinfo->DCC_size)) {
        readed_size = pinfo->DCC_size;
    }
    mutex_unlock(&gimx890_eeprom_mutex_22047);

    memcpy(data, IMX890_DCC_data_22047, pinfo->DCC_size);
    return readed_size;
}

struct eeprom_map_info imx890_eeprom_info_22047[] = {
    { EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
    { EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
    { EEPROM_META_LENS_ID, 0x0008, 0x0010, 0x0011, 2, true },
    { EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
    { EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
    { EEPROM_META_MODULE_SN, 0x00B0, 0x00c1, 0x00c2, 17, true },
    { EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
    { EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0, false },
    { EEPROM_META_STEREO_MW_MAIN_DATA, IMX890_STEREO_MW_START_ADDR_22047, 0x3588, 0x3589, CALI_DATA_MASTER_LENGTH, true },
    { EEPROM_META_STEREO_MT_MAIN_DATA, IMX890_STEREO_MT_START_ADDR_22047, 0x3c24, 0x3c25, CALI_DATA_MASTER_LENGTH, true },
    { EEPROM_META_DISTORTION_DATA, 0x0000, 0x0000, 0x0000, 0, false },
};

unsigned int read_imx890_eeprom_info_22047(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size)
{
    kal_uint16 addr;
    int readsize;

    if (meta_id != imx890_eeprom_info_22047[meta_id].meta)
        return -1;

    if (size != imx890_eeprom_info_22047[meta_id].size)
        return -1;

    addr = imx890_eeprom_info_22047[meta_id].start;
    readsize = imx890_eeprom_info_22047[meta_id].size;

    read_imx890_eeprom_22047(ctx, addr, data, readsize);

    return 0;
}
