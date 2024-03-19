/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#ifndef __MEDIATEK_BOOT_MODE_H
#define __MEDIATEK_BOOT_MODE_H

#define BOOT_CHARGER	1
#define BOOT_RECOVERY	2
#define BOOT_BOOTLOADER	3
#define BOOT_DM_VERITY	4
#define BOOT_KPOC	5
#define BOOT_DDR_RSVD	6
#define BOOT_META	7
#define BOOT_RPMBPK	8
//#ifdef OPLUS_BUG_STABILITY
#define BOOT_KERNEL	9
#define BOOT_SILENCE	10
#define BOOT_SAU	11
#define BOOT_AGINGTEST	12
#define BOOT_FACTORY	13
#define BOOT_EDL    14
//#endif

#endif
