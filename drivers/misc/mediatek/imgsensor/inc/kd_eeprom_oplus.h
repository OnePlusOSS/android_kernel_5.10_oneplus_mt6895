/*
 * Copyright (C) 2021 OPLUS.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef KD_EEPROM_OPLLUS
#define KD_EEPROM_OPLLUS

#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

enum {
	EEPROM_META_MODULE_ID = 0,
	EEPROM_META_SENSOR_ID,
	EEPROM_META_LENS_ID,
	EEPROM_META_VCM_ID,
	EEPROM_META_MIRROR_FLIP,
	EEPROM_META_MODULE_SN,
	EEPROM_META_AF_CODE,
	EEPROM_META_STEREO_DATA,
	EEPROM_META_STEREO_MW_MAIN_DATA,
	EEPROM_META_STEREO_MT_MAIN_DATA,
	EEPROM_META_DISTORTION_DATA,
	EEPROM_META_MAX,
};

enum {
	EEPROM_STEREODATA = 0,
	EEPROM_STEREODATA_MT_MAIN,
	EEPROM_STEREODATA_MW_MAIN,
};

struct eeprom_map_info {
	kal_uint16 meta;
	kal_uint16 start;
	kal_uint16 valid;
	kal_uint16 checksum;
	int size;
	kal_bool present;
};
#endif
