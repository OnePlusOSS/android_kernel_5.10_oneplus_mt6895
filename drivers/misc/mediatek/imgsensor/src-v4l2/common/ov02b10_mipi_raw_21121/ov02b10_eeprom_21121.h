/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov02b10_eeprom.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 Add APIs to read from EEPROM
 *
 ****************************************************************************/
#ifndef __OV02B10_EEPROM_21121_H__
#define __OV02B10_EEPROM_21121_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"

#include "kd_eeprom_oplus.h"


unsigned int read_ov02b10_eeprom_info_21121(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);
#endif
