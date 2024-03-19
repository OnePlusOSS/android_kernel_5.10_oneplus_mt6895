// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx766_eeprom.c
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 Add APIs to read from EEPROM
 *
 ****************************************************************************/
#define PFX "IMX766_pdafotp_21641"
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
#include "imx766mipiraw_Sensor_21641.h"
#include "imx766_eeprom_21641.h"
#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define Sleep(ms) mdelay(ms)

#define IMX766_EEPROM_READ_ID_21641  0xA0
#define IMX766_EEPROM_WRITE_ID_21641 0xA1
#define IMX766_I2C_SPEED_21641       100
#define IMX766_MAX_OFFSET_21641      0x8000

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

static DEFINE_MUTEX(gimx766_eeprom_mutex_21641);

static bool read_imx766_eeprom_21641(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client,
			IMX766_EEPROM_READ_ID_21641 >> 1, addr, data, size) < 0) {
		return false;
	}
	return true;
}

static struct EEPROM_PDAF_INFO *get_eeprom_pdaf_info(struct subdrv_ctx *ctx)
{
	static struct EEPROM_PDAF_INFO *pinfo;
	BYTE read_data[4];

	mutex_lock(&gimx766_eeprom_mutex_21641);
	if (pinfo == NULL) {
		read_imx766_eeprom_21641(ctx, 0x1, read_data, 4);
		if (((read_data[3] << 24) |
		     (read_data[2] << 16) |
		     (read_data[1] << 8) |
		     read_data[0]) == MTK_IDENTITY_VALUE) {
			pinfo = &eeprom_pdaf_info[MTK_FMT];
		} else {
			pinfo = &eeprom_pdaf_info[OP_FMT];
		}
	}
	mutex_unlock(&gimx766_eeprom_mutex_21641);

	return pinfo;
}

unsigned int read_imx766_LRC_21641(struct subdrv_ctx *ctx, BYTE *data)
{
	static BYTE IMX766_LRC_data_21641[LRC_SIZE] = { 0 };
	static unsigned int readed_size;
	struct EEPROM_PDAF_INFO *pinfo = get_eeprom_pdaf_info(ctx);

	LOG_INF("read imx766 LRC, addr = %d, size = %u\n",
		pinfo->LRC_addr, pinfo->LRC_size);

	mutex_lock(&gimx766_eeprom_mutex_21641);
	if ((readed_size == 0) &&
	    read_imx766_eeprom_21641(ctx, pinfo->LRC_addr,
			       IMX766_LRC_data_21641, pinfo->LRC_size)) {
		readed_size = pinfo->LRC_size;
	}
	mutex_unlock(&gimx766_eeprom_mutex_21641);

	memcpy(data, IMX766_LRC_data_21641, pinfo->LRC_size);
	return readed_size;
}

unsigned int read_imx766_DCC_21641(struct subdrv_ctx *ctx, BYTE *data)
{
	static BYTE IMX766_DCC_data_21641[DCC_SIZE] = { 0 };
	static unsigned int readed_size;
	struct EEPROM_PDAF_INFO *pinfo = get_eeprom_pdaf_info(ctx);

	LOG_INF("read imx766 DCC, addr = %d, size = %u\n",
		pinfo->DCC_addr, pinfo->DCC_size);

	mutex_lock(&gimx766_eeprom_mutex_21641);
	if ((readed_size == 0) &&
	    read_imx766_eeprom_21641(ctx, pinfo->DCC_addr,
			       IMX766_DCC_data_21641, pinfo->DCC_size)) {
		readed_size = pinfo->DCC_size;
	}
	mutex_unlock(&gimx766_eeprom_mutex_21641);

	memcpy(data, IMX766_DCC_data_21641, pinfo->DCC_size);
	return readed_size;
}

struct eeprom_map_info imx766_eeprom_info_21641[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x000F, 0x0010, 17, true },
};

unsigned int read_imx766_eeprom_info_21641(struct subdrv_ctx *ctx, kal_uint16 meta_id,
				     BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != imx766_eeprom_info_21641[meta_id].meta)
		return -1;

	if (size != imx766_eeprom_info_21641[meta_id].size)
		return -1;

	addr = imx766_eeprom_info_21641[meta_id].start;
	readsize = imx766_eeprom_info_21641[meta_id].size;

	read_imx766_eeprom_21641(ctx, addr, data, readsize);

	return 0;
}
