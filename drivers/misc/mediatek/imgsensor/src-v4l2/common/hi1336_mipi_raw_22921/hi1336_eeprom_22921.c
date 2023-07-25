// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     hi1336_eeprom.c
 *
 * Project:
 * --------
 * Description:
 * ------------
 *     Add APIs to read from EEPROM
 *
 ****************************************************************************/
#define PFX "HI1336_otp_22921"
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
#include "hi1336_eeprom_22921.h"
#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define Sleep(ms) mdelay(ms)

#define HI1336_EEPROM_READ_ID_22921  0xA0
#define HI1336_EEPROM_WRITE_ID_22921 0xA0
#define HI1336_MAX_OFFSET_22921 0x8000


static bool selective_read_eeprom(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data)
{
	if (addr > HI1336_MAX_OFFSET_22921)
		return false;

	if (adaptor_i2c_rd_u8(ctx->i2c_client,
		HI1336_EEPROM_READ_ID_22921 >> 1, addr, data) < 0) {
		return false;
	}
	return true;
}

static bool read_hi1336_eeprom_22921(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size)
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

struct eeprom_map_info hi1336_eeprom_info_22921[] = {
	{ EEPROM_META_MODULE_ID, 0x0001, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_SENSOR_ID, 0x0002, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_LENS_ID, 0x0003, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_VCM_ID, 0x0004, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_MIRROR_FLIP, 0x0008, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_MODULE_SN, 0x000a, 0x000F, 0x0010, 19, true },
};

unsigned int read_hi1336_eeprom_info_22921(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != hi1336_eeprom_info_22921[meta_id].meta)
		return -1;

	if (size != hi1336_eeprom_info_22921[meta_id].size)
		return -1;

	addr = hi1336_eeprom_info_22921[meta_id].start;
	readsize = hi1336_eeprom_info_22921[meta_id].size;

	read_hi1336_eeprom_22921(ctx, addr, data, readsize);

	return 0;
}
