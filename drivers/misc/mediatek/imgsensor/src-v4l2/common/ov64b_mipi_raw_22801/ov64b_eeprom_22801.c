// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

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

#define PFX "ov64b_pdafotp_22801"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"
#include "ov64b_eeprom_22801.h"
#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define OV64B_EEPROM_READ_ID_22801 0xA0
#define OV64B_EEPROM_WRITE_ID_22801 0xA0
#define OV64B_MAX_OFFSET_22801 0x3FFF

static bool ov64b_selective_read_eeprom_22801(struct subdrv_ctx *ctx,
        kal_uint16 addr, BYTE *data)
{
    if (addr > OV64B_MAX_OFFSET_22801)
        return false;
    if (adaptor_i2c_rd_u8(ctx->i2c_client,
        OV64B_EEPROM_READ_ID_22801 >> 1, addr, data) < 0)
        return false;
    return true;
}

static bool ov64b_read_eeprom_22801(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size)
{
    int i = 0;
    int offset = addr;

    LOG_INF("enter _read_eeprom size = %d\n", size);
    for (i = 0; i < size; i++) {
        if (!ov64b_selective_read_eeprom_22801(ctx, offset, &data[i]))
            return false;
        /* LOG_INF("read_eeprom 0x%0x %d\n",offset, data[i]); */
        offset++;
    }
    return true;
}

struct eeprom_map_info ov64b_eeprom_info_22801[] = {
    { EEPROM_META_MODULE_ID, 0x0000, 0x000F, 0x0010, 2, true },
    { EEPROM_META_SENSOR_ID, 0x0006, 0x000F, 0x0010, 2, true },
    { EEPROM_META_LENS_ID, 0x0008, 0x000F, 0x0010, 2, true },
    { EEPROM_META_VCM_ID, 0x000A, 0x000F, 0x0010, 2, true },
    { EEPROM_META_MIRROR_FLIP, 0x000E, 0x000F, 0x0010, 1, true },
    { EEPROM_META_MODULE_SN, 0x00B0, 0x000C1, 0x00C2, 17, true },
    { EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
};

unsigned int read_ov64b_eeprom_info_22801(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size)
{
    kal_uint16 addr;
    int readsize;

    if (meta_id != ov64b_eeprom_info_22801[meta_id].meta)
        return -1;

    if (size != ov64b_eeprom_info_22801[meta_id].size)
        return -1;

    addr = ov64b_eeprom_info_22801[meta_id].start;
    readsize = ov64b_eeprom_info_22801[meta_id].size;

    ov64b_read_eeprom_22801(ctx, addr, data, readsize);

    return 0;
}

