/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx766dual_eeprom.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 Add APIs to read from EEPROM
 *
 ****************************************************************************/
#ifndef __IMX766DUAL_EEPROM_H__
#define __IMX766DUAL_EEPROM_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"

#include "kd_eeprom_oplus.h"

/*
 * LRC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx766dual_LRC(struct subdrv_ctx *ctx, BYTE *data);

/*
 * DCC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx766dual_DCC(struct subdrv_ctx *ctx, BYTE *data);

/*
 * read eeprom data
 *
 * @param meta_id info_id
 * @param data Buffer
 * @param size ReadSize
 * @return size of data
 */
unsigned int read_imx766dual_eeprom_info(struct subdrv_ctx *ctx,
					 kal_uint16 meta_id, BYTE *data,
					 int size);
#endif
