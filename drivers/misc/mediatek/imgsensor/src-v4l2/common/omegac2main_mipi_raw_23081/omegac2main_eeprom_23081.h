/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     omegac2main_eeprom.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *     Add APIs to read from EEPROM
 *
 ****************************************************************************/
#ifndef __OMEGAC2MAIN_EEPROM_23081_H__
#define __OMEGAC2MAIN_EEPROM_23081_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"

#include "kd_eeprom_oplus.h"


#define OMEGAC2MAIN_EEPROM_READ_ID_23081  0xA1
#define OMEGAC2MAIN_EEPROM_WRITE_ID_23081 0xA0
#define OMEGAC2MAIN_I2C_SPEED_23081       100
#define OMEGAC2MAIN_MAX_OFFSET_23081      0x8000
#define OMEGAC2MAIN_SENSOR_ID             0x8206
#define OMEGAC2MAIN_SENSOR_ID_2           0x8202

#define OMEGAC2MAIN_STEREO_MW_START_ADDR_23081  (0x2FBA)
#define OMEGAC2MAIN_STEREO_MT_START_ADDR_23081  (0x3655)

#define OMEGAC2MAIN_EEPROM_LOCK_REGISTER_23081  (0xE000)

/*
 * DCC
 *
 * @param data Buffer
 * @return size of data
 */
bool read_omegac2main_eeprom_23081(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size);
unsigned int read_omegac2main_eeprom_info_23081(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);
bool omegac2main_read_eeprom_state_23081(struct subdrv_ctx *ctx);
#endif
