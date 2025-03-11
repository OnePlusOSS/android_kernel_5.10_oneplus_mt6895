// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     imx882_eeprom.c
 *
 * Project:
 * --------
 * Description:
 * ------------
 *     Add APIs to read from EEPROM
 *
 ****************************************************************************/
#define PFX "IMX882_pdafotp_23021"
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
#include "imx882mipiraw_Sensor_23021.h"
#include "imx882_eeprom_23021.h"
#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define Sleep(ms) mdelay(ms)


static DEFINE_MUTEX(gimx882_eeprom_mutex_23021);

bool read_imx882_eeprom_23021(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size)
{
    if (addr + size > IMX882_MAX_OFFSET_23021) {
        return false;
    }

    if (adaptor_i2c_rd_p8(ctx->i2c_client,
            IMX882_EEPROM_READ_ID_23021 >> 1, addr, data, size) < 0) {
        return false;
    }

    return true;
}

struct eeprom_map_info imx882_eeprom_info_23021[] = {
    { EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
    { EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
    { EEPROM_META_LENS_ID, 0x0008, 0x0010, 0x0011, 2, true },
    { EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
    { EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
    { EEPROM_META_MODULE_SN, 0x00B0, 0x00c7, 0x00c8, 23, true },
    { EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
    { EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0, false },
    { EEPROM_META_STEREO_MW_MAIN_DATA, IMX882_STEREO_MW_START_ADDR_23021, 0xffff, 0xffff, CALI_DATA_MASTER_LENGTH, true },
    { EEPROM_META_STEREO_MT_MAIN_DATA, IMX882_STEREO_MT_START_ADDR_23021, 0xffff, 0xffff, CALI_DATA_MASTER_LENGTH, true },
    { EEPROM_META_DISTORTION_DATA, 0x0000, 0x0000, 0x0000, 0, false },
};

unsigned int read_imx882_eeprom_info_23021(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size)
{
    kal_uint16 addr;
    int readsize;

    if (meta_id != imx882_eeprom_info_23021[meta_id].meta)
        return -1;

    if (size != imx882_eeprom_info_23021[meta_id].size)
        return -1;

    addr = imx882_eeprom_info_23021[meta_id].start;
    readsize = imx882_eeprom_info_23021[meta_id].size;

    read_imx882_eeprom_23021(ctx, addr, data, readsize);

    return 0;
}
