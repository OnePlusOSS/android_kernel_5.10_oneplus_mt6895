/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_BACK_H_
#define _DSI_IRIS_BACK_H_

enum LOOP_BACK_TYPE {
	EFIFO_LOOP_BACK = 0,
	PURE_LOOP_BACK,
	EMA4_LOOP_BACK,
};

enum PLL_NCO_TYPE {
	PLL_NCO = 0,
	PLL_NCO_58,
};

enum SRAM_BIST_ERR_TYPE {
	ERR_SRAM_1ST_W	= 0x1,
	ERR_SRAM_1ST_R	= 0x2,
	ERR_SRAM_2ND_R	= 0x3,
	ERR_SRAM_3RD_R	= 0x4,
	ERR_SRAM_4TH_R	= 0x5,
};

enum LOOP_BACK_ERR_TYPE {
	ERR_NO_ERR			= 0,
	ERR_DTG_MEASURE		= 1,
	ERR_DTG_MEASURE_58	= 2,
	ERR_SRAM_BIST_5A	= 3,
	ERR_SRAM_BIST_A5	= 4,
	ERR_PURE_LOOP_BACK	= 5,
	ERR_EFIFO_LOOP_BACK	= 6,
	ERR_DUAL_PT			= 7,
	ERR_EMA4_LOOP_BACK  = 8,
	ERR_I2C				= 9,
};

#define BIT_PLL_NCO			(1 << 0)
#define BIT_PLL_NCO_58      (1 << 1)
#define BIT_SRAM_5A_BIST	(1 << 2)
#define BIT_SRAM_A5_BIST    (1 << 3)
#define BIT_PURE_LOOPBACK	(1 << 4)
#define BIT_EFIFO_LOOPBACK  (1 << 5)
#define BIT_DUAL_PT         (1 << 6)
#define BIT_EMA4_LOOPBACK	(1 << 7)
#define BIT_I2C             (1 << 8)


int32_t iris_parse_loopback_info(struct device_node *np, struct iris_cfg *pcfg);

u32 iris_loop_back_verify(enum LOOP_BACK_TYPE type);

/* API in kernel for recovery mode */
int iris_loop_back_validate(void);

int iris_loop_back_init(void);

#endif // _DSI_IRIS_BACK_H_
