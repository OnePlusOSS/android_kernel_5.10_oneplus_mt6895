/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     imx766_eeprom.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *     Add APIs to read from EEPROM
 *
 ****************************************************************************/
#ifndef __IMX766_EEPROM_21121_H__
#define __IMX766_EEPROM_21121_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"

#include "kd_eeprom_oplus.h"

/*
 * LRC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx766_LRC_21121(struct subdrv_ctx *ctx, BYTE *data);

/*
 * DCC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx766_DCC_21121(struct subdrv_ctx *ctx, BYTE *data);
unsigned int read_imx766_eeprom_info_21121(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);
#endif
