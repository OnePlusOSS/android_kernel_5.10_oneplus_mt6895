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
#define PFX "IMX890_pdafotp_22021"
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
#include "imx890mipiraw_Sensor_22021.h"
#include "imx890_eeprom_22021.h"
#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define Sleep(ms) mdelay(ms)

#define IMX890_EEPROM_READ_ID_22021  0xA3
#define IMX890_EEPROM_WRITE_ID_22021 0xA2
#define IMX890_I2C_SPEED_22021       100
#define IMX890_MAX_OFFSET_22021      0x8000

#define MTK_IDENTITY_VALUE 0x010B00FF
#define LRC_SIZE 140
#define DCC_SIZE 96

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

static DEFINE_MUTEX(gimx890_eeprom_mutex_22021);

static bool selective_read_eeprom(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data)
{
    if (addr > IMX890_MAX_OFFSET_22021)
        return false;

    if (adaptor_i2c_rd_u8(ctx->i2c_client,
            IMX890_EEPROM_READ_ID_22021 >> 1, addr, data) < 0) {
        return false;
    }
    return true;
}

static bool read_imx890_eeprom_22021(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size)
{
    int i = 0;
    int offset = addr;

    for (i = 0; i < size; i++) {
        if (!selective_read_eeprom(ctx, offset, &data[i]))
            return false;
        offset++;
    }
    return true;
}

static struct EEPROM_PDAF_INFO *get_eeprom_pdaf_info(struct subdrv_ctx *ctx)
{
    static struct EEPROM_PDAF_INFO *pinfo;
    BYTE read_data[4];

    mutex_lock(&gimx890_eeprom_mutex_22021);
    if (pinfo == NULL) {
        read_imx890_eeprom_22021(ctx, 0x1, read_data, 4);
        if (((read_data[3] << 24) |
             (read_data[2] << 16) |
             (read_data[1] << 8) |
             read_data[0]) == MTK_IDENTITY_VALUE) {
            pinfo = &eeprom_pdaf_info[MTK_FMT];
        } else {
            pinfo = &eeprom_pdaf_info[OP_FMT];
        }
    }
    mutex_unlock(&gimx890_eeprom_mutex_22021);

    return pinfo;
}

unsigned int read_imx890_LRC_22021(struct subdrv_ctx *ctx, BYTE *data)
{
    static BYTE IMX890_LRC_data_22021[LRC_SIZE] = { 0 };
    static unsigned int readed_size;
    struct EEPROM_PDAF_INFO *pinfo = get_eeprom_pdaf_info(ctx);

    LOG_INF("read imx890 LRC, addr = %d, size = %u\n",
        pinfo->LRC_addr, pinfo->LRC_size);

    mutex_lock(&gimx890_eeprom_mutex_22021);
    if ((readed_size == 0) &&
        read_imx890_eeprom_22021(ctx, pinfo->LRC_addr,
                   IMX890_LRC_data_22021, pinfo->LRC_size)) {
        readed_size = pinfo->LRC_size;
    }
    mutex_unlock(&gimx890_eeprom_mutex_22021);

    memcpy(data, IMX890_LRC_data_22021, pinfo->LRC_size);
    return readed_size;
}

unsigned int read_imx890_DCC_22021(struct subdrv_ctx *ctx, BYTE *data)
{
    static BYTE IMX890_DCC_data_21641[DCC_SIZE] = { 0 };
    static unsigned int readed_size;
    struct EEPROM_PDAF_INFO *pinfo = get_eeprom_pdaf_info(ctx);

    LOG_INF("read imx890 DCC, addr = %d, size = %u\n",
        pinfo->DCC_addr, pinfo->DCC_size);

    mutex_lock(&gimx890_eeprom_mutex_22021);
    if ((readed_size == 0) &&
        read_imx890_eeprom_22021(ctx, pinfo->DCC_addr,
                   IMX890_DCC_data_21641, pinfo->DCC_size)) {
        readed_size = pinfo->DCC_size;
    }
    mutex_unlock(&gimx890_eeprom_mutex_22021);

    memcpy(data, IMX890_DCC_data_21641, pinfo->DCC_size);
    return readed_size;
}

struct eeprom_map_info imx890_eeprom_info_22021[] = {
    { EEPROM_META_MODULE_ID, 0x0000, 0x000F, 0x0010, 2, true },
    { EEPROM_META_SENSOR_ID, 0x0006, 0x000F, 0x0010, 2, true },
    { EEPROM_META_LENS_ID, 0x0008, 0x000F, 0x0010, 2, true },
    { EEPROM_META_VCM_ID, 0x000A, 0x000F, 0x0010, 2, true },
    { EEPROM_META_MIRROR_FLIP, 0x000E, 0x000F, 0x0010, 1, true },
    { EEPROM_META_MODULE_SN, 0x00B0, 0x000F, 0x0010, 17, true },
    { EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 7, true },
    { EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0, false },
    { EEPROM_META_STEREO_MW_MAIN_DATA, 0x2B00, 0x3199, 0x319A, CALI_DATA_MASTER_LENGTH, true },
    { EEPROM_META_STEREO_MT_MAIN_DATA, 0x31C0, 0x3859, 0x385A, CALI_DATA_MASTER_LENGTH, true },
};

unsigned int read_imx890_eeprom_info_22021(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size)
{
    kal_uint16 addr;
    int readsize;

    if (meta_id != imx890_eeprom_info_22021[meta_id].meta)
        return -1;

    if (size != imx890_eeprom_info_22021[meta_id].size)
        return -1;

    addr = imx890_eeprom_info_22021[meta_id].start;
    readsize = imx890_eeprom_info_22021[meta_id].size;

    read_imx890_eeprom_22021(ctx, addr, data, readsize);

    return 0;
}
