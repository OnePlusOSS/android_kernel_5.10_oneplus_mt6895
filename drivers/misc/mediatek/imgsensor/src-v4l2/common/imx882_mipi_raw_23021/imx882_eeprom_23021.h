/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     imx882_eeprom.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *     Add APIs to read from EEPROM
 *
 ****************************************************************************/
#ifndef __IMX882_EEPROM_23021_H__
#define __IMX882_EEPROM_23021_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"

#include "kd_eeprom_oplus.h"


#define IMX882_EEPROM_READ_ID_23021  0xA1
#define IMX882_EEPROM_WRITE_ID_23021 0xA0
#define IMX882_I2C_SPEED_23021       100
#define IMX882_MAX_OFFSET_23021      0x8000
#define IMX882_SENSOR_ID             0x8206
#define IMX882_SENSOR_ID_2           0x8202

#define IMX882_STEREO_MW_START_ADDR_23021  (0x2FBA)
#define IMX882_STEREO_MT_START_ADDR_23021  (0x3655)

#define IMX882_EEPROM_LOCK_REGISTER_23021  (0xE000)

/*
 * DCC
 *
 * @param data Buffer
 * @return size of data
 */
bool read_imx882_eeprom_23021(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size);
unsigned int read_imx882_eeprom_info_23021(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);
#endif
