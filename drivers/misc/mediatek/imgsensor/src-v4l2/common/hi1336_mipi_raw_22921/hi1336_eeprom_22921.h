/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 hi1336_eeprom.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 Add APIs to read from EEPROM
 *
 ****************************************************************************/
#ifndef __HI1336_EEPROM_22921_H__
#define __HI1336_EEPROM_22921_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"

#include "kd_eeprom_oplus.h"

unsigned int read_hi1336_eeprom_info_22921(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size);

#endif
