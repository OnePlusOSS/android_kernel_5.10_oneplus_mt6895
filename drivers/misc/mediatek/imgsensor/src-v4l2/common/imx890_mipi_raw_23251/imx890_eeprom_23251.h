/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     imx890_eeprom.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *     Add APIs to read from EEPROM
 *
 ****************************************************************************/
#ifndef __IMX890_EEPROM_23251_H__
#define __IMX890_EEPROM_23251_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"

#include "kd_eeprom_oplus.h"


#define IMX890_EEPROM_READ_ID_23251  0xA1
#define IMX890_EEPROM_WRITE_ID_23251 0xA0
#define IMX890_I2C_SPEED_23251       100
#define IMX890_MAX_OFFSET_23251      0x8000
#define IMX890_SENSOR_ID            0x0890

#define IMX890_STEREO_MT_START_ADDR_23251  (0x358B)
#define IMX890_STEREO_MW_START_ADDR_23251  (0x2EF0)
#define IMX890_EEPROM_LOCK_REGISTER_23251  (0xE000)

#define MTK_IDENTITY_VALUE 0x010B00FF
#define LRC_SIZE 140
#define DCC_SIZE 96

/*
 * LRC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx890_LRC_23251(struct subdrv_ctx *ctx, BYTE *data);

/*
 * DCC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx890_DCC_23251(struct subdrv_ctx *ctx, BYTE *data);
bool read_imx890_eeprom_23251(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size);
unsigned int read_imx890_eeprom_info_23251(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);
#endif
