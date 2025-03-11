// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/debugfs.h>

#include "mtk_drm_debugfs.h"

#include "iris_api.h"
#include "iris_uapi.h"
#include "iris_i3c.h"
#include "iris_lightup.h"
#include "iris_lightup_ocp.h"
#include "iris_lp.h"
#include "iris_pq.h"
#include "iris_ioctl.h"
#include "iris_lut.h"
#include "iris_mode_switch.h"
#include "iris_gpio.h"
#include "iris_frc.h"
#include "iris_log.h"
#include "iris_loop_back.h"
#include "iris_mtk_api.h"

static u32 iris_loopback_flag = 0xffffff3d;
static int iris_i2c_test_verify(void);

int32_t iris_parse_loopback_info(struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u32 loop_back_mode = 0;
	u32 loop_back_mode_res = 0;

	rc = of_property_read_u32(np, "pxlw,loop-back-mode", &loop_back_mode);
	if (!rc)
		IRIS_LOGE("get property: pxlw, loop-back-mode: %d", loop_back_mode);
	pcfg->loop_back_mode = loop_back_mode;

	rc = of_property_read_u32(np, "pxlw,loop-back-mode-res", &loop_back_mode_res);
	if (!rc)
		IRIS_LOGE("get property: pxlw, loop-back-mode-res: %d", loop_back_mode_res);
	pcfg->loop_back_mode_res = loop_back_mode_res;

	return 0;
}


void iris_loop_back_reset(void)
{
	iris_send_one_wired_cmd(IRIS_POWER_DOWN_SYS);
	usleep_range(3500, 3501);
	iris_send_one_wired_cmd(IRIS_POWER_UP_SYS);
	usleep_range(3500, 3501);
}

static u32 addrs[66] = {
	0xf000004c, 0xf000004c, 0xf0000048, 0xf1680008, 0xf16e0008, 0xf1a20044,
	0xf1a40044, 0xf158000c, 0xf1580290, 0xf1560118, 0xf1a00060, 0xf1520058,
	0xf10c0000, 0xf1500404, 0xf12c0000, 0xf12d0000, 0xf1640054, 0xf1200020,
	0xf120002c, 0xf120009c, 0xf1210000, 0xf1240004, 0xf1240008, 0xf124000c,
	0xf1240018, 0xf124003c, 0xf1240074, 0xf1240150, 0xf1240170, 0xf1241004,
	0xf1241084, 0xf1241098, 0xf124109c, 0xf12410b0, 0xf12410e8, 0xf1240000,
	0xf1250000, 0xf1280008, 0xf1280038, 0xf12800c4, 0xf1281004, 0xf1281014,
	0xf1281028, 0xf1290000, 0xf1220000, 0xf1220004, 0xf1220008, 0xf1220014,
	0xf122001c, 0xf1220064, 0xf16400b8, 0xf1a40000, 0xf1a40008, 0xf1a40018,
	0xf1a4001c, 0xf1a40024, 0xf1a40028, 0xf1a4002c, 0xf1500098, 0xf1500000,
	0xf1580000, 0xf1580014, 0xf1580290, 0xf1400024, 0xf140002c, 0xf141ff00
};

static u32 values[66] = {
	0x0c011800, 0x0e019c00, 0x000026a0, 0x00000800, 0x00000800, 0x00001FFF,
	0x00001FFF, 0x00000800, 0x00000001, 0x00003FFF, 0x00010800, 0x00003FFF,
	0x00001484, 0x00000800, 0x0000d04d, 0x00000000, 0x000013ff, 0x020002c3,
	0x0000000a, 0x0000000c, 0x0000000f, 0x00401384, 0x30800065, 0x50208800,
	0x04380438, 0x00000020, 0xffffffff, 0x00000545, 0x00000003, 0x00020888,
	0xe4100010, 0x0000005a, 0x00040000, 0x11210100, 0x0000005a, 0xa0e8000c,
	0x00000100, 0x00000001, 0x04380438, 0x00000003, 0x00020888, 0x0000005a,
	0x00000001, 0x00000004, 0xe0008007, 0x21008801, 0x4780010e, 0x00044100,
	0x20000186, 0x00000002, 0xb46c343c, 0x00037762, 0x00080000, 0x00020003,
	0x00020003, 0x00000019, 0x09800438, 0x00080000, 0x00000000, 0xd0840421,
	0x00010040, 0x00000010, 0x00000002, 0x00020000, 0x00200195, 0x00000101
};

static u32 addrs_efifo[68] = {
	0xf000004c, 0xf000004c, 0xf0000048, 0xf1680008, 0xf16e0008, 0xf1a20044,
	0xf1a40044, 0xf158000c, 0xf1580290, 0xf1560118, 0xf1a00060, 0xf1520058,
	0xf10c0000, 0xf1500404, 0xf12c0000, 0xf12d0000, 0xf1640054, 0xf1200020,
	0xf120002c, 0xf120009c, 0xf1210000, 0xf1240004, 0xf1240008, 0xf124000c,
	0xf1240018, 0xf124003c, 0xf1240074, 0xf1240150, 0xf1240170, 0xf1241004,
	0xf1241084, 0xf1241098, 0xf124109c, 0xf12410b0, 0xf12410e8, 0xf1240000,
	0xf1250000, 0xf1280008, 0xf1280038, 0xf12800c4, 0xf1281004, 0xf1281014,
	0xf1281028, 0xf1290000, 0xf1220000, 0xf1220004, 0xf1220008, 0xf1220014,
	0xf122001c, 0xf1220064, 0xf16400b8, 0xf1a40000, 0xf1a40008, 0xf1a40018,
	0xf1a4001c, 0xf1a40024, 0xf1a40028, 0xf1a4002c, 0xf1500098, 0xf1500000,
	0xf1580000, 0xf1580014, 0xf1580290, 0xf1400004, 0xf1400024, 0xf1400028,
	0xf140002c, 0xf141ff00
};

static u32 values_efifo[68] = {
	0x0c011800, 0x0e109c20, 0x000032a0, 0x00000800, 0x00000800, 0x00001FFF,
	0x00001FFF, 0x00000800, 0x00000001, 0x00003FFF, 0x00010800, 0x00003FFF,
	0x00001484, 0x00000800, 0x0000d04d, 0x00000000, 0x000013ff, 0x020002c3,
	0x0000000a, 0x0000000c, 0x0000000f, 0x00401384, 0x30800065, 0x50208800,
	0x04380438, 0x00000020, 0xffffffff, 0x00000545, 0x00000003, 0x00020888,
	0xe4100010, 0x0000005a, 0x00040000, 0x11210100, 0x0000005a, 0xa0e8000c,
	0x00000100, 0x00000001, 0x04380438, 0x00000003, 0x00020888, 0x0000005a,
	0x00000001, 0x00000004, 0xe0008007, 0x21008801, 0x4780010e, 0x00044100,
	0x20000186, 0x00000002, 0xb46c343c, 0x00037762, 0x00080000, 0x00020003,
	0x00020003, 0x00000019, 0x09800438, 0x00080000, 0x00000000, 0xd0840421,
	0x00010040, 0x00000010, 0x00000002, 0x14298085, 0x00020000, 0x0043fff0,
	0x00200195, 0x00000101
};

static u32 addrs_2slice[86] = {
	0xf1680000, 0xf1680004, 0xf168000c, 0xf1680010, 0xf1680014, 0xf1680018,
	0xf168001c, 0xf1680020, 0xf1680024, 0xf1680028, 0xf168002c, 0xf1680030,
	0xf1680034, 0xf1680038, 0xf168003c, 0xf1680040, 0xf1680044, 0xf1680048,
	0xf168004c, 0xf1680050, 0xf1680054, 0xf1680058, 0xf168005c, 0xf1680060,
	0xf1680008, 0xf169ff00, 0xf1640000, 0xf1640004, 0xf1640008, 0xf164000c,
	0xf1640010, 0xf1640014, 0xf1640018, 0xf164001c, 0xf1640020, 0xf1640024,
	0xf1640054, 0xf1640064, 0xf1640068, 0xf164006c, 0xf1640070, 0xf1640074,
	0xf1640078, 0xf164007c, 0xf1640080, 0xf1640084, 0xf1640088, 0xf164008c,
	0xf1640090, 0xf1640094, 0xf1640098, 0xf164009c, 0xf16400a0, 0xf16400a4,
	0xf16400a8, 0xf16400ac, 0xf16400b0, 0xf16400b4, 0xf16400b8, 0xF1640114,
	0xf16e0000, 0xf16e0004, 0xf16e000c, 0xf16e0010, 0xf16e0014, 0xf16e0018,
	0xf16e001c, 0xf16e0020, 0xf16e0024, 0xf16e0028, 0xf16e002c, 0xf16e0030,
	0xf16e0034, 0xf16e0038, 0xf16e003c, 0xf16e0040, 0xf16e0044, 0xf16e0048,
	0xf16e004c, 0xf16e0050, 0xf16e0054, 0xf16e0058, 0xf16e005c, 0xf16e0060,
	0xf16e0008, 0xf16fff00
};

static u32 values_2slice[86] = {
	0x00480150, 0x01e001e0, 0x89000011, 0x80078030, 0x10003804, 0x1c021c02,
	0x2a040002, 0x84012000, 0x0c000700, 0x5c066706, 0xf0100018, 0x00200c03,
	0x330b0b06, 0x382a1c0e, 0x69625446, 0x7b797770, 0x02017e7d, 0x40090001,
	0xfc19be09, 0xf819fa19, 0x781a381a, 0xf62ab61a, 0x742b342b, 0xf463743b,
	0x00000800, 0x00000001, 0x00000003, 0x00000002, 0x00B40168, 0x0002021C,
	0x00B4010E, 0x000021C0, 0x00002200, 0x000010E0, 0xC0002000, 0x00B4021C,
	0x00000000, 0x89000011, 0x80078030, 0x10003804, 0x1C021C02, 0x0E020002,
	0x84012000, 0x0C000700, 0x5C066706, 0xF0100018, 0x00200C03, 0x330B0B06,
	0x382A1C0E, 0x69625446, 0x7B797770, 0x02017E7D, 0x40090001, 0xFC19BE09,
	0xF819FA19, 0x781A381A, 0xF62AB61A, 0x742B342B, 0xF463743B, 0x00000004,
	0x00480150, 0x01E001E0, 0x89000011, 0x80078030, 0x10003804, 0x1C021C02,
	0x2A040002, 0x84012000, 0x0C000700, 0x5C066706, 0xF0100018, 0x00200C03,
	0x330B0B06, 0x382A1C0E, 0x69625446, 0x7B797770, 0x02017E7D, 0x40090001,
	0xFC19BE09, 0xF819FA19, 0x781A381A, 0xF62AB61A, 0x742B342B, 0xF463743B,
	0x00000800, 0x00000001
};

void iris_ocp_i3c_write(u32 addr, u32 value)
{
	u32 values[2];

	values[0] = addr;
	values[1] = value;

	IRIS_LOGD("i3c write, addr = %x, value = %x", addr, value);
	iris_i2c_ocp_single_write(values, 1);
}

u32 iris_ocp_i3c_read(u32 addr, u32 mode)
{
	u32 values[2];
	u32 ret = 0;

	values[0] = addr;
	values[1] = mode;

	ret = iris_i2c_ocp_read(values, 1, 0);
	if (ret)
		pr_err("%s error!\n", __func__);

	return values[0];
}

static int iris_i2c_test_verify(void)
{
	int rc = 0;
	uint32_t val[10] = {10};

	val[0] = 0xf001fff8;
	rc = iris_i2c_conver_ocp_read(val, 1, false);
	IRIS_LOGD("%s,%d: value = 0x%x", __func__, __LINE__, val[0]);
	if (rc) {
		IRIS_LOGE("%s(%d), failed to read, return: %d",
				__func__, __LINE__, rc);
		return rc;
	}

	val[0] = 0xf1800000;
	iris_i2c_conver_ocp_read(val, 1, false);
	val[1] = val[0] | 0x1;
	val[0] = 0xf1800000;
	iris_i2c_single_conver_ocp_write(val, 1);
	rc = iris_i2c_conver_ocp_read(val, 1, false);
	IRIS_LOGD("%s,%d: value = 0x%x", __func__, __LINE__, val[0]);

	return rc;

}

static u32 iris_sram_bist_verify(u32 test_value)
{
	u32 ramctrl_bist_status = 0;
	u32 err_base = (test_value == 0x5a5a5a5a) ? 0x30 : 0x40;

	IRIS_LOGI("%s() enter, test_value = 0x%08x\n", __func__, test_value);

	iris_ocp_i3c_write(0xf0000060, 0x0f0303fe);
	iris_ocp_i3c_write(0xf00000f4, 0x00000644);
	msleep(1);

	iris_ocp_i3c_write(0xf0000004, 0x002a80af);
	iris_ocp_i3c_write(0xf0000008, 0x0010f008);
	iris_ocp_i3c_write(0xf0000000, 0x00000081);
	iris_ocp_i3c_write(0xf0000000, 0x00000083);
	iris_ocp_i3c_write(0xf0000000, 0x00000081);

	iris_ocp_i3c_write(0xf0000020, 0x002a80ac);
	iris_ocp_i3c_write(0xf0000024, 0x0000f00e);
	iris_ocp_i3c_write(0xf000001c, 0x00000081);
	iris_ocp_i3c_write(0xf000001c, 0x00000083);
	iris_ocp_i3c_write(0xf000001c, 0x00000081);
	msleep(10);

	iris_ocp_i3c_write(0xf0000048, 0x000032a0);
	iris_ocp_i3c_write(0xf000004c, 0x0e109c20);
	iris_ocp_i3c_write(0xf0000050, 0x00000000);
	msleep(1);

	//sram 1st write
	iris_ocp_i3c_write(0xf1040000, 0x00000000);
	iris_ocp_i3c_write(0xf1040000, 0x00002000);
	iris_ocp_i3c_write(0xf104002c, 0x00020000);
	iris_ocp_i3c_write(0xf1040030, 0x0043ffff);
	iris_ocp_i3c_write(0xf1040038, 0x0001ffff);
	iris_ocp_i3c_write(0xf1040034, test_value);
	iris_ocp_i3c_write(0xf104003c, 0x00000003);
	msleep(5);
	ramctrl_bist_status = iris_ocp_i3c_read(0xf1040040, DSI_CMD_SET_STATE_HS);
	if (ramctrl_bist_status != 1) {
		IRIS_LOGE("%s(), sram 1st write, ramctrl_bist_status = %d\n", __func__, ramctrl_bist_status);
		return err_base + ERR_SRAM_1ST_W;
	}

	//sram 1st read
	iris_ocp_i3c_write(0xf104003c, 0x00000008);
	iris_ocp_i3c_write(0xf104003c, 0x00000000);
	iris_ocp_i3c_write(0xf1040030, 0x0018fc00);
	iris_ocp_i3c_write(0xf1040038, 0x0001ffff);
	iris_ocp_i3c_write(0xf104003c, 0x00000005);
	msleep(5);
	ramctrl_bist_status = iris_ocp_i3c_read(0xf1040040, DSI_CMD_SET_STATE_HS);
	if (ramctrl_bist_status != 2) {
		IRIS_LOGE("%s(), sram 1st read, ramctrl_bist_status = %d\n", __func__, ramctrl_bist_status);
		return err_base + ERR_SRAM_1ST_R;
	}

	//sram 2nd read
	iris_ocp_i3c_write(0xf104003c, 0x00000008);
	iris_ocp_i3c_write(0xf104003c, 0x00000000);
	iris_ocp_i3c_write(0xf104002c, 0x0018f800);
	iris_ocp_i3c_write(0xf1040030, 0x00300200);
	iris_ocp_i3c_write(0xf104003c, 0x00000005);
	msleep(5);
	ramctrl_bist_status = iris_ocp_i3c_read(0xf1040040, DSI_CMD_SET_STATE_HS);
	if (ramctrl_bist_status != 2) {
		IRIS_LOGE("%s(), sram 2nd read, ramctrl_bist_status = %d\n", __func__, ramctrl_bist_status);
		return err_base + ERR_SRAM_2ND_R;
	}

	//sram 3rd read
	iris_ocp_i3c_write(0xf104003c, 0x00000008);
	iris_ocp_i3c_write(0xf104003c, 0x00000000);
	iris_ocp_i3c_write(0xf104002c, 0x00300000);
	iris_ocp_i3c_write(0xf1040030, 0x0043ffff);
	iris_ocp_i3c_write(0xf104003c, 0x00000005);
	msleep(5);
	ramctrl_bist_status = iris_ocp_i3c_read(0xf1040040, DSI_CMD_SET_STATE_HS);
	if (ramctrl_bist_status != 2) {
		IRIS_LOGE("%s(), sram 3rd read, ramctrl_bist_status = %d\n", __func__, ramctrl_bist_status);
		return err_base + ERR_SRAM_3RD_R;
	}

	//sram 4th read
	iris_ocp_i3c_write(0xf104003c, 0x00000008);
	iris_ocp_i3c_write(0xf104003c, 0x00000000);
	iris_ocp_i3c_write(0xf104002c, 0x00020000);
	iris_ocp_i3c_write(0xf1040030, 0x0043ffff);
	iris_ocp_i3c_write(0xf104003c, 0x00000005);
	msleep(5);
	ramctrl_bist_status = iris_ocp_i3c_read(0xf1040040, DSI_CMD_SET_STATE_HS);
	if (ramctrl_bist_status != 2) {
		IRIS_LOGE("%s(), sram 4th read, ramctrl_bist_status = %d\n", __func__, ramctrl_bist_status);
		return err_base + ERR_SRAM_4TH_R;
	}

	IRIS_LOGI("%s() exit!\n", __func__);
	return ERR_NO_ERR;
}

u32 iris_loop_back_verify(enum LOOP_BACK_TYPE type)
{

	u32 i, r, g, b;
	u32 ret = 0;
	u32 standard_rgbsum[3] = {0x40d1a890, 0x318c343c, 0x37839da4};

	IRIS_LOGI("%s(), %s enter!\n", __func__,
		type == PURE_LOOP_BACK ? "pure loop back" : \
		(type == EFIFO_LOOP_BACK ? "efifo loop back" : "ema4 loop back"));


	iris_ocp_i3c_write(0xf0000044, 0x00000400);
	iris_ocp_i3c_write(0xf00000c0, 0x00000055);
	iris_ocp_i3c_write(0xf0000060, 0x0f0303fe);
	iris_ocp_i3c_write(0xf0000060, 0x0f0303f6);
	iris_ocp_i3c_write(0xf0000060, 0x0f0303fe);
	msleep(100);

	//iris_ocp_i3c_write(0xf0000050, 0x00003f00);
	//pclk
	//iris_ocp_i3c_write(0xf0000004, 0x002b00af);
	if (type == EMA4_LOOP_BACK)
		iris_ocp_i3c_write(0xf0000004, 0x002b00a4);
	else
		iris_ocp_i3c_write(0xf0000004, 0x002b00af);
	iris_ocp_i3c_write(0xf0000008, 0x0010f008);
	iris_ocp_i3c_write(0xf0000000, 0x00000081);
	iris_ocp_i3c_write(0xf0000000, 0x00000083);
	iris_ocp_i3c_write(0xf0000000, 0x00000081);
	//mclk
	if (type == EMA4_LOOP_BACK)
		iris_ocp_i3c_write(0xf0000020, 0x002b00ab);
	else
		iris_ocp_i3c_write(0xf0000020, 0x002b00ac);
	//iris_ocp_i3c_write(0xf0000020, 0x002b00ac);
	iris_ocp_i3c_write(0xf0000024, 0x0000f00e);
	iris_ocp_i3c_write(0xf000001c, 0x00000081);
	iris_ocp_i3c_write(0xf000001c, 0x00000083);
	iris_ocp_i3c_write(0xf000001c, 0x00000081);
	msleep(10);
	iris_ocp_i3c_write(0xf0000050, 0x00000000);
	if (type == EMA4_LOOP_BACK)
		iris_ocp_i3c_write(0xf00000d4, 0x19440303);

	iris_ocp_i3c_write(0xf120005c, 0x00fffffe);

	if (type == PURE_LOOP_BACK) {
		for (i = 0; i < 66; i++)
			iris_ocp_i3c_write(addrs[i], values[i]);
	} else {
		for (i = 0; i < 68; i++)
			iris_ocp_i3c_write(addrs_efifo[i], values_efifo[i]);
	}

	msleep(1);
	iris_ocp_i3c_write(0xf1200020, 0x02000ac3);
	iris_ocp_i3c_write(0xf1210000, 0x3);
	msleep(20);
	iris_ocp_i3c_write(0xf1200020, 0x020002c3);

	r = iris_ocp_i3c_read(0xf12401a8, DSI_CMD_SET_STATE_HS);
	g = iris_ocp_i3c_read(0xf12401ac, DSI_CMD_SET_STATE_HS);
	b = iris_ocp_i3c_read(0xf12401b0, DSI_CMD_SET_STATE_HS);
	IRIS_LOGD("%s(), r = 0x%08x, g = 0x%08x, b = 0x%08x\n", __func__, r, g, b);

	if ((r == standard_rgbsum[0]) && (g == standard_rgbsum[1]) && (b == standard_rgbsum[2]))
		ret = ERR_NO_ERR;
	else {
		if (type == PURE_LOOP_BACK)
			ret = ERR_PURE_LOOP_BACK;
		else if (type == EFIFO_LOOP_BACK)
			ret = ERR_EFIFO_LOOP_BACK;
		else
			ret = ERR_EMA4_LOOP_BACK;
	}

	IRIS_LOGI("%s(), %s exit, ret = %d!\n", __func__,
		type == PURE_LOOP_BACK ? "pure loop back" : \
		(type == EFIFO_LOOP_BACK ? "efifo loop back" : "ema4 loop back"),
		ret);

	return ret;
}

u32 iris_dual_pt_verify(void)
{

	u32 i, r0, g0, b0, r1, g1, b1;
	u32 ret = 0;
	u32 standard_rgbsum[3] = {0x10361ecc, 0x110ca190, 0x10db25c8};

	IRIS_LOGI("%s(), enter!\n", __func__);

	iris_ocp_i3c_write(0xf0000044, 0x00000400);
	iris_ocp_i3c_write(0xf00000c0, 0x00000055);
	iris_ocp_i3c_write(0xf00000d4, 0x3f440303);
	//iris_ocp_i3c_write(0xf00000f4, 0x00000644);
	//iris_ocp_i3c_write(0xf00000ec, 0x00000001);
	//iris_ocp_i3c_write(0xf00000ec, 0x00000003);
	iris_ocp_i3c_write(0xf0000060, 0x0f0303fe);
	iris_ocp_i3c_write(0xf0000060, 0x0f0303f6);
	iris_ocp_i3c_write(0xf0000060, 0x0f0303fe);
	msleep(100);
	iris_ocp_i3c_write(0xf0000050, 0x00000000);
	//pclk
	iris_ocp_i3c_write(0xf0000004, 0x002a80af);
	iris_ocp_i3c_write(0xf0000008, 0x0010f008);
	iris_ocp_i3c_write(0xf0000000, 0x00000081);
	iris_ocp_i3c_write(0xf0000000, 0x00000083);
	iris_ocp_i3c_write(0xf0000000, 0x00000081);
	//mclk
	iris_ocp_i3c_write(0xf0000020, 0x002a80ac);
	iris_ocp_i3c_write(0xf0000024, 0x0000f00e);
	iris_ocp_i3c_write(0xf000001c, 0x00000081);
	iris_ocp_i3c_write(0xf000001c, 0x00000083);
	iris_ocp_i3c_write(0xf000001c, 0x00000081);
	msleep(10);
	iris_ocp_i3c_write(0xf0000048, 0x000032a0);
	iris_ocp_i3c_write(0xf000004c, 0x0e109c20);
	msleep(10);
	//iris_ocp_i3c_write(0xf0000050, 0x00000000);
	iris_ocp_i3c_write(0xf1640054, 0x000013ff);
	iris_ocp_i3c_write(0xf1680008, 0x00000800);
	iris_ocp_i3c_write(0xf16e0008, 0x00000800);

	for (i = 0; i < 86; i++)
		iris_ocp_i3c_write(addrs_2slice[i], values_2slice[i]);

	iris_ocp_i3c_write(0xf12c0000, 0x0000d04d);
	iris_ocp_i3c_write(0xf12d0000, 0x00000000);
	iris_ocp_i3c_write(0xf1240004, 0x00403384);
	iris_ocp_i3c_write(0xf1240008, 0x38000065);
	iris_ocp_i3c_write(0xf124000c, 0x52a00800);
	iris_ocp_i3c_write(0xf1240018, 0x04380438);
	iris_ocp_i3c_write(0xf124003c, 0x00000020);
	iris_ocp_i3c_write(0xf1240074, 0xffffffff);
	iris_ocp_i3c_write(0xf1240150, 0x00000505);
	iris_ocp_i3c_write(0xf1240170, 0x00000003);
	iris_ocp_i3c_write(0xf1241004, 0x00020888);
	iris_ocp_i3c_write(0xf1241084, 0xe4100010);
	iris_ocp_i3c_write(0xf1241098, 0x0000005a);
	iris_ocp_i3c_write(0xf124109c, 0x00040000);
	iris_ocp_i3c_write(0xf12410b0, 0x11000100);
	iris_ocp_i3c_write(0xf12410d8, 0x00000020);
	iris_ocp_i3c_write(0xf12410e0, 0x00020000);
	iris_ocp_i3c_write(0xf12410e8, 0x00000195);
	iris_ocp_i3c_write(0xf1250000, 0x00000300);
	iris_ocp_i3c_write(0xf1280000, 0xa0800008);
	iris_ocp_i3c_write(0xf12800a4, 0x00000545);
	iris_ocp_i3c_write(0xf12800c4, 0x00000003);
	iris_ocp_i3c_write(0xf1281004, 0x00020888);
	iris_ocp_i3c_write(0xf1281028, 0x00000001);
	iris_ocp_i3c_write(0xf1280038, 0x04380438);
	iris_ocp_i3c_write(0xf1281014, 0x0000005a);
	iris_ocp_i3c_write(0xf1280008, 0x00000001);
	iris_ocp_i3c_write(0xf1290000, 0x00000004);
	iris_ocp_i3c_write(0xf1220000, 0xe0008007);
	iris_ocp_i3c_write(0xf1220004, 0x21008801);
	iris_ocp_i3c_write(0xf1220008, 0x4780010e);
	iris_ocp_i3c_write(0xf1220014, 0x00044100);
	iris_ocp_i3c_write(0xf122001c, 0x20000186);
	iris_ocp_i3c_write(0xf1220064, 0x00000002);
	iris_ocp_i3c_write(0xf1640114, 0x00000006);
	iris_ocp_i3c_write(0xf1a40000, 0x00037762);
	iris_ocp_i3c_write(0xf1a40008, 0x00080000);
	iris_ocp_i3c_write(0xf1a40018, 0x00020003);
	iris_ocp_i3c_write(0xf1a4001c, 0x00020003);
	iris_ocp_i3c_write(0xf1a40024, 0x00000019);
	iris_ocp_i3c_write(0xf1a40028, 0x09800438);
	iris_ocp_i3c_write(0xf1a4002c, 0x00080000);
	iris_ocp_i3c_write(0xf1500098, 0x00000000);
	iris_ocp_i3c_write(0xf1500000, 0xd0840421);
	iris_ocp_i3c_write(0xf1580000, 0x00010040);
	iris_ocp_i3c_write(0xf1580014, 0x00000010);
	iris_ocp_i3c_write(0xf1580290, 0x00000002);
	iris_ocp_i3c_write(0xf189c010, 0x01000107);
	iris_ocp_i3c_write(0xf189c010, 0x0500800a);
	iris_ocp_i3c_write(0xf189c014, 0xab000010);
	iris_ocp_i3c_write(0xf189c014, 0x80078020);
	iris_ocp_i3c_write(0xf189c014, 0x10003804);
	iris_ocp_i3c_write(0xf189c014, 0x38043804);
	iris_ocp_i3c_write(0xf189c014, 0x1c035901);
	iris_ocp_i3c_write(0xf189c014, 0x16012000);
	iris_ocp_i3c_write(0xf189c014, 0x0c000f00);
	iris_ocp_i3c_write(0xf189c014, 0x2e036706);
	iris_ocp_i3c_write(0xf189c014, 0x28160018);
	iris_ocp_i3c_write(0xf189c014, 0x00200c03);
	iris_ocp_i3c_write(0xf189c014, 0x330b0b06);
	iris_ocp_i3c_write(0xf189c014, 0x382a1c0e);
	iris_ocp_i3c_write(0xf189c014, 0x69625446);
	iris_ocp_i3c_write(0xf189c014, 0x7b797770);
	iris_ocp_i3c_write(0xf189c014, 0x02017e7d);
	iris_ocp_i3c_write(0xf189c014, 0x40090001);
	iris_ocp_i3c_write(0xf189c014, 0xfc19be09);
	iris_ocp_i3c_write(0xf189c014, 0xf819fa19);
	iris_ocp_i3c_write(0xf189c014, 0x781a381a);
	iris_ocp_i3c_write(0xf189c014, 0xf62ab61a);
	iris_ocp_i3c_write(0xf189c014, 0x742b342b);
	iris_ocp_i3c_write(0xf189c014, 0xf46b743b);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c014, 0x00000000);
	iris_ocp_i3c_write(0xf189c010, 0x05000539);
	iris_ocp_i3c_write(0xf189c014, 0x0400002a);
	iris_ocp_i3c_write(0xf189c014, 0x00000037);
	iris_ocp_i3c_write(0xf189c010, 0x05000539);
	iris_ocp_i3c_write(0xf189c014, 0x0700002b);
	iris_ocp_i3c_write(0xf189c014, 0x0000007f);
	iris_ocp_i3c_write(0xf1800a08, 0x000004f7);
	iris_ocp_i3c_write(0xf1800808, 0x0e010200);
	iris_ocp_i3c_write(0xf18c0210, 0x0000027f);
	iris_ocp_i3c_write(0xf1400004, 0x14080085);
	iris_ocp_i3c_write(0xf1400024, 0x00020000);
	iris_ocp_i3c_write(0xf140002c, 0x00200195);
	iris_ocp_i3c_write(0xf141ff00, 0x00000101);
	iris_ocp_i3c_write(0xf120005c, 0x0024e520);
	iris_ocp_i3c_write(0xf120009c, 0x00000010);
	iris_ocp_i3c_write(0xf1240000, 0xa0e8401c);
	iris_ocp_i3c_write(0xf1200020, 0x02010243);
	iris_ocp_i3c_write(0xf1210000, 0x0000000f);

	msleep(20);

	r0 = iris_ocp_i3c_read(0xf12401a8, DSI_CMD_SET_STATE_HS);
	g0 = iris_ocp_i3c_read(0xf12401ac, DSI_CMD_SET_STATE_HS);
	b0 = iris_ocp_i3c_read(0xf12401b0, DSI_CMD_SET_STATE_HS);
	IRIS_LOGI("%s(), r0 = 0x%08x, g0 = 0x%08x, b0 = 0x%08x\n", __func__, r0, g0, b0);
	r1 = iris_ocp_i3c_read(0xf128007c, DSI_CMD_SET_STATE_HS);
	g1 = iris_ocp_i3c_read(0xf1280080, DSI_CMD_SET_STATE_HS);
	b1 = iris_ocp_i3c_read(0xf1280084, DSI_CMD_SET_STATE_HS);
	IRIS_LOGI("%s(), r1 = 0x%08x, g1 = 0x%08x, b1 = 0x%08x\n", __func__, r1, g1, b1);

	if (((r0 == standard_rgbsum[0]) && (g0 == standard_rgbsum[1]) && (b0 == standard_rgbsum[2])) &&
		((r1 == standard_rgbsum[0]) && (g1 == standard_rgbsum[1]) && (b1 == standard_rgbsum[2])))
		ret = ERR_NO_ERR;
	else {
		ret = ERR_DUAL_PT;
	}

	IRIS_LOGI("%s() exit, ret = %d!\n", __func__, ret);

	return ret;
}

u32 iris_dtg_measure_verify(enum PLL_NCO_TYPE type)
{
	u32 reg = 0;
	u32 ret = 0;
	u32 standard_val = 0;

	IRIS_LOGI("%s() %s enter\n", __func__, type == PLL_NCO ? "pll nco" : "pll nco 5/8");

	iris_ocp_i3c_write(0xf0000004, 0x002a80af);
	iris_ocp_i3c_write(0xf0000008, 0x0000f00e);
	iris_ocp_i3c_write(0xf0000008, 0x0000f80e);
	iris_ocp_i3c_write(0xf0000008, 0x0000f00e);
	iris_ocp_i3c_write(0xf0000000, 0x00000081);
	iris_ocp_i3c_write(0xf0000000, 0x00000083);
	iris_ocp_i3c_write(0xf0000000, 0x00000081);

	iris_ocp_i3c_write(0xf0000020, 0x002a80ac);
	if (type == PLL_NCO) {
		iris_ocp_i3c_write(0xf0000024, 0x0000f00e);
		iris_ocp_i3c_write(0xf0000024, 0x0000f80e);
		iris_ocp_i3c_write(0xf0000024, 0x0000f00e);
	} else {
		iris_ocp_i3c_write(0xf0000024, 0x0000f00d);
		iris_ocp_i3c_write(0xf0000024, 0x0000f80d);
		iris_ocp_i3c_write(0xf0000024, 0x0000f00d);
	}
	iris_ocp_i3c_write(0xf000001c, 0x00000081);
	iris_ocp_i3c_write(0xf000001c, 0x00000083);
	iris_ocp_i3c_write(0xf000001c, 0x00000081);
	msleep(10);

	iris_ocp_i3c_write(0xf0000044, 0x00000400);
	iris_ocp_i3c_write(0xf1240000, 0xa0e8000c);
	iris_ocp_i3c_write(0xf0000048, 0x000072a0);
	iris_ocp_i3c_write(0xf000004c, 0x06109c20);
	iris_ocp_i3c_write(0xf124016c, 0x0020f580);
	iris_ocp_i3c_write(0xf1240150, 0x00000547);
	iris_ocp_i3c_write(0xf1250000, 0x00000300);
	iris_ocp_i3c_write(0xf1200020, 0x02018303);
	iris_ocp_i3c_write(0xf1210000, 0x00000003);
	msleep(20);

	reg = iris_ocp_i3c_read(0xf120ff0c, DSI_CMD_SET_STATE_HS);
	IRIS_LOGI("%s, %d: reg = 0x%08x\n", __func__, __LINE__, reg);

	if (type == PLL_NCO)
		standard_val = 0x0008d000;
	else
		standard_val = 0x00098000;

	if ((reg & 0xfffff000) == standard_val)
		ret = ERR_NO_ERR;
	else {
		if (type == PLL_NCO)
			ret = ERR_DTG_MEASURE;
		else
			ret = ERR_DTG_MEASURE_58;
	}

	IRIS_LOGI("%s() exit, ret = %d!\n", __func__, ret);

	return ret;
}

int iris_loop_back_validate(void)
{

	int rc = 0;
	int temp = 0;

	IRIS_LOGI("[%s:%d] enter.", __func__, __LINE__);

	iris_loop_back_reset();

	msleep(10);
	temp = iris_ocp_i3c_read(0xf00000d0, DSI_CMD_SET_STATE_HS);
	IRIS_LOGD("%s,%d: value = 0x%x", __func__, __LINE__, temp);

	msleep(10);

	temp = iris_ocp_i3c_read(0xf1800000, DSI_CMD_SET_STATE_HS);
	IRIS_LOGD("%s,%d: value = 0x%x", __func__, __LINE__, temp);
	temp &= (~0x1);
	IRIS_LOGD("%s,%d: value = 0x%x", __func__, __LINE__, temp);
	iris_ocp_i3c_write(0xf1800000, temp);
	temp = iris_ocp_i3c_read(0xf1800000, DSI_CMD_SET_STATE_HS);
	IRIS_LOGD("%s,%d: value = 0x%x", __func__, __LINE__, temp);

	IRIS_LOGD("%s(), iris_loopback_flag = 0x%08x", __func__, iris_loopback_flag);
	if (iris_loopback_flag & BIT_PLL_NCO) {
		IRIS_LOGD("%s(), step 1 (pll nco) execute!", __func__);
		//step 1
		rc = iris_dtg_measure_verify(PLL_NCO);
		if (rc) {
			IRIS_LOGE("[%s:%d] step1: dtg measure(pll nco) verify rc = %d", __func__, __LINE__, rc);
			return rc;
		}
	}

	if (iris_loopback_flag & BIT_PLL_NCO_58) {
		IRIS_LOGD("%s(), step 2 (pll nco 5/8) execute!", __func__);
		//step 2
		iris_loop_back_reset();
		rc = iris_dtg_measure_verify(PLL_NCO_58);
		if (rc) {
			IRIS_LOGE("[%s:%d] step2: dtg measure(pll nco 58) verify rc = %d", __func__, __LINE__, rc);
			return rc;
		}
	}

	if (iris_loopback_flag & BIT_SRAM_5A_BIST) {
		IRIS_LOGD("%s(), step 3 (sram bist 0x5a5a5a5a) execute!", __func__);
		iris_loop_back_reset();

		//step 3
		rc = iris_sram_bist_verify(0x5a5a5a5a);
		if (rc) {
			IRIS_LOGE("[%s:%d] step3: sram bist verify rc = %d", __func__, __LINE__, rc);
			return ERR_SRAM_BIST_5A;
		}
	}

	if (iris_loopback_flag & BIT_SRAM_A5_BIST) {
		IRIS_LOGD("%s(), step 4 (sram bist 0xa5a5a5a5) execute!", __func__);
		iris_loop_back_reset();

		//step 4
		rc = iris_sram_bist_verify(0xa5a5a5a5);
		if (rc) {
			IRIS_LOGE("[%s:%d] step4: sram bist verify rc = %d", __func__, __LINE__, rc);
			return ERR_SRAM_BIST_A5;
		}
	}

	if (iris_loopback_flag & BIT_PURE_LOOPBACK) {
		IRIS_LOGD("%s(), step 5 (pure loopback) execute!", __func__);
		iris_loop_back_reset();

		//step 5
		rc = iris_loop_back_verify(PURE_LOOP_BACK);
		if (rc) {
			IRIS_LOGE("[%s:%d] step5: pure loop back verify rc = %d", __func__, __LINE__, rc);
			return rc;
		}
	}

	if (iris_loopback_flag & BIT_EFIFO_LOOPBACK) {
		IRIS_LOGD("%s(), step 6 (efifo loopback) execute!", __func__);
		iris_loop_back_reset();

		//step 6
		rc = iris_loop_back_verify(EFIFO_LOOP_BACK);
		if (rc) {
			IRIS_LOGE("[%s:%d] step6: efifo loop back verify rc = %d", __func__, __LINE__, rc);
			return rc;
		}
	}

	if (iris_loopback_flag & BIT_DUAL_PT) {
		IRIS_LOGD("%s(), step 7 (dual pt) execute!", __func__);
		//step 7
		iris_loop_back_reset();

		rc = iris_dual_pt_verify();
		if (rc) {
			IRIS_LOGE("[%s:%d] step7: dual pt verify rc = %d", __func__, __LINE__, rc);
			return rc;
		}
	}

	if (iris_loopback_flag & BIT_EMA4_LOOPBACK) {
		IRIS_LOGD("%s(), step 8 (ema4 loopback) execute!", __func__);
		iris_loop_back_reset();

		//step 8
		rc = iris_loop_back_verify(EMA4_LOOP_BACK);
		if (rc) {
			IRIS_LOGE("[%s:%d] step8: ema4 loop back verify rc = %d", __func__, __LINE__, rc);
			return rc;
		}
	}

	if (iris_loopback_flag & BIT_I2C) {
		IRIS_LOGD("%s(), step 9 (i2c verify) execute!", __func__);
		//step 9
		rc = iris_i2c_test_verify();
		if (rc) {
			IRIS_LOGE("[%s:%d] step9: i2c read rc = %d", __func__, __LINE__, rc);
			return ERR_I2C;
		}
	}

	iris_loop_back_reset();

	iris_abyp_lp(ABYP_POWER_DOWN_PLL);

	IRIS_LOGI("[%s:%d] exit.", __func__, __LINE__);

	return rc;
}

static ssize_t _iris_dbg_loop_back_ops(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	uint32_t temp, values[2];

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	if (val == 0) {
		iris_reset();
		IRIS_LOGE("iris reset.");
	} else if (val == 1) {
		iris_exit_abyp(true);
		IRIS_LOGE("iris exit abyp.");
	} else if (val == 2) {
		iris_ocp_write_val(0xf00000c0, 0x0);
		IRIS_LOGE("enable analog bypass.");
	} else if (val == 3) {
		values[0] = 0xf1800000;
		values[1] = 0;
		iris_i2c_ocp_read(values, 1, 0);
		temp = values[0];
		IRIS_LOGD("%s(%d), value = 0x%x", __func__, __LINE__, temp);
		temp &= (~0x1);
		IRIS_LOGD("%s(%d), value = 0x%x", __func__, __LINE__, temp);
		values[0] = 0xf1800000;
		values[1] = temp;
		iris_i2c_ocp_single_write(values, 1);
		values[0] = 0xf1800000;
		values[1] = 0;
		iris_i2c_ocp_read(values, 1, 0);
		temp = values[0];
		IRIS_LOGD("%s(%d), value = 0x%x", __func__, __LINE__, temp);
		IRIS_LOGE("%s(%d), disable mipi rx", __func__, __LINE__);
	} else if (val == 4) {
		iris_loop_back_verify(PURE_LOOP_BACK);
	} else if (val == 5) {
		temp = 0x400;
		values[0] = 0xf0000044;
		values[1] = temp;
		iris_i2c_ocp_single_write(values, 1);
		IRIS_LOGE("%s(%d), rst dtg!", __func__, __LINE__);
	} else if (val == 6) {
		temp = 0x55;
		IRIS_LOGD("%s(%d), value = 0x%x", __func__, __LINE__, temp);
		values[0] = 0xf00000c0;
		values[1] = temp;
		iris_i2c_ocp_single_write(values, 1);
		IRIS_LOGE("%s(%d), disable ulps!", __func__, __LINE__);
	} else if (val == 7) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 0x26);
	} else if (val == 8) {
		iris_loop_back_verify(EFIFO_LOOP_BACK);
	} else {
		pr_err("%s(%d), parameter error!", __func__, __LINE__);
	}

	return count;
}

static ssize_t _iris_dbg_loop_back_test(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	ktime_t ktime0;
	ktime_t ktime1;
	uint32_t timeus = 0;
	struct iris_cfg *pcfg;
	int tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	pcfg = iris_get_cfg();

	//mutex_lock(&pcfg->panel->panel_lock);
	ktime0 = ktime_get();
	ret = iris_loop_back_validate();
	ktime1 = ktime_get();
	timeus = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
	//mutex_unlock(&pcfg->panel->panel_lock);
	IRIS_LOGI("%s(), spend time %d us, return: %d", __func__, timeus, ret);


	tot = scnprintf(bp, sizeof(bp), "0x%02x\n", ret);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;

}

static const struct file_operations iris_loop_back_fops = {
	.open = simple_open,
	.write = _iris_dbg_loop_back_ops,
	.read = _iris_dbg_loop_back_test,
};

int iris_loop_back_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("iris_loopback_flag", 0644, pcfg->dbg_root,
			(u32 *)&iris_loopback_flag);
	if (debugfs_create_file("iris_loop_back",	0644, pcfg->dbg_root, NULL,
				&iris_loop_back_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}
