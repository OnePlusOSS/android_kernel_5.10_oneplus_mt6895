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

#define PFX "omegac2tele_pdafotp_23081"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"
#include "omegac2tele_eeprom_23081.h"
#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define OMEGAC2TELE_EEPROM_READ_ID_23081 0xA1
#define OMEGAC2TELE_EEPROM_WRITE_ID_23081 0xA0
#define OMEGAC2TELE_MAX_OFFSET_23081 0x3FFF
#define OMEGAC2TELE_EEPROM_HEADER_ID_ADDR_23081 0x00000006
#define OMEGAC2TELE_EEPROM_HEADER_ID_SIZE_23081 4
#define OMEGAC2TELE_EEPROM_HEADER_ID_VALUE_23081 0x01A9010B

static bool omegac2tele_read_eeprom_23081(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size)
{
    if (addr + size > OMEGAC2TELE_MAX_OFFSET_23081) {
        return false;
    }

    if (adaptor_i2c_rd_p8(ctx->i2c_client,
            OMEGAC2TELE_EEPROM_READ_ID_23081 >> 1, addr, data, size) < 0) {
        return false;
    }

    return true;
}

struct eeprom_map_info omegac2tele_eeprom_info_23081[] = {
    { EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
    { EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
    { EEPROM_META_LENS_ID, 0x0008, 0x0010, 0x0011, 2, true },
    { EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
    { EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
    { EEPROM_META_MODULE_SN, 0x00B0, 0x00c7, 0x00c8, 23, true },
    { EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
};

unsigned int read_omegac2tele_eeprom_info_23081(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size)
{
    kal_uint16 addr;
    int readsize;

    if (meta_id != omegac2tele_eeprom_info_23081[meta_id].meta)
        return -1;

    if (size != omegac2tele_eeprom_info_23081[meta_id].size)
        return -1;

    addr = omegac2tele_eeprom_info_23081[meta_id].start;
    readsize = omegac2tele_eeprom_info_23081[meta_id].size;

    omegac2tele_read_eeprom_23081(ctx, addr, data, readsize);

    return 0;
}

bool omegac2tele_read_eeprom_state_23081(struct subdrv_ctx *ctx)
{
    BYTE header_id_data[OMEGAC2TELE_EEPROM_HEADER_ID_SIZE_23081] = { 0 };
    kal_uint32 header_id = 0;
    kal_bool ret = true;
    ret = omegac2tele_read_eeprom_23081(ctx, OMEGAC2TELE_EEPROM_HEADER_ID_ADDR_23081,
            &(header_id_data[0]), OMEGAC2TELE_EEPROM_HEADER_ID_SIZE_23081);
    header_id = (header_id_data[0] | (header_id_data[1] << 8)
                    | (header_id_data[2] << 16) | (header_id_data[3] << 24));
    LOG_INF("header_id = 0x%x\n", header_id);
    if (header_id != OMEGAC2TELE_EEPROM_HEADER_ID_VALUE_23081) {
        return false;
    }
    return ret;
}

