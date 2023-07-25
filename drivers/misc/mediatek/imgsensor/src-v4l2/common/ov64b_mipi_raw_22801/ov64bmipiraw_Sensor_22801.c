// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     OV64B2Qmipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 * Setting version:
 * ------------
 *   update full pd setting for OV64B2QEB_03B
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define PFX "OV64B2Q_camera_sensor"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "ov64bmipiraw_Sensor_22801.h"
#include "ov64b_Sensor_setting_22801.h"
#include "ov64b_ana_gain_table_22801.h"
#include "ov64b_eeprom_22801.h"

#define MULTI_WRITE 1
#define FPT_PDAF_SUPPORT 1

#define SEAMLESS_ 1
#define SEAMLESS_DEBUG 0
static bool _is_seamless;
static unsigned int _is_initFlag;


#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor_16(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)
#define seq_write_cmos_sensor(...) subdrv_i2c_wr_seq_p8(__VA_ARGS__)

#define _I2C_BUF_SIZE 4096
static kal_uint16 _i2c_data[_I2C_BUF_SIZE];
static unsigned int _size_to_write;
#define OTP_SIZE    0x4000
static kal_uint8 otp_data[OTP_SIZE] = {0};

#define stereo_start_add    0x2000
static kal_uint8 stereo_data[CALI_DATA_MASTER_LENGTH] = {0};

#define OV64B_EEPROM_WRITE_ID  0xA0

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV64B_SENSOR_ID_22801,

	.checksum_value = 0x388c7147,//test_Pattern_mode

	.pre = {
		.pclk = 115200000,
		.linelength = 696,
		.framelength = 3600,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 3468,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 460,
		.mipi_pixel_rate = 952000000,
	},
	.cap = {
		.pclk = 115200000,
		.linelength = 696,
		.framelength = 3600,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 3468,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 460,
		.mipi_pixel_rate = 952000000,
	},
	.normal_video = {
		.pclk = 115200000,
		.linelength = 936,
		.framelength = 4102,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 2600,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 960000000,
	},
	.hs_video = {
		.pclk = 115200000,
		.linelength = 792,
		.framelength = 2424,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 2256,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
		.mipi_pixel_rate = 802560000,
	},
	.slim_video = {
		.pclk = 115200000,
		.linelength = 504,
		.framelength = 1904,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1296,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
		.mipi_pixel_rate = 679680000,
	},
	.custom1 = {
		.pclk = 115200000,
		.linelength = 336,
		.framelength = 1428,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1296,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 2400,
		.mipi_pixel_rate = 998400000,
	},
	.custom2 = {
		.pclk = 115200000,
		.linelength = 1224,
		.framelength = 3920,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 3468,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
		.mipi_pixel_rate = 518400000,
	},
	.custom3 = {
		.pclk = 115200000,
		.linelength = 1008,
		.framelength = 7618,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 9248,
		.grabwindow_height = 6936,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
		.mipi_pixel_rate = 1200000000,
	},
	.custom4 = {
		.pclk = 115200000,
		.linelength = 696,
		.framelength = 2758,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 2600,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
		.mipi_pixel_rate = 920000000,
	},
	.custom5 = {
		.pclk = 115200000,
		.linelength = 696,
		.framelength = 3600,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 3468,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 460,
		.mipi_pixel_rate = 952000000,
	},
	.custom6 = {
		.pclk = 115200000,
		.linelength = 936,
		.framelength = 4102,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 3468,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 679680000,
	},
	.custom7 = {
		.pclk = 115200000,
		.linelength = 1320,
		.framelength = 2910,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4624,
		.grabwindow_height = 2600,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 296,
		.mipi_pixel_rate = 518400000,
	},
	.custom8 = {//need update
		.pclk = 115200000,
		.linelength = 768,
		.framelength = 1250,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 1200,
		.mipi_pixel_rate = 546000000,
	},
	.custom9 = {
		.pclk = 115200000,
		.linelength = 1008,
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2312,
		.grabwindow_height = 1736,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 518400000,
	},
	.custom10 = {//need update
		.pclk = 115200000,
		.linelength = 1872,
		.framelength = 6152,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8000,
		.grabwindow_height = 6000,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 100,
		.mipi_pixel_rate = 548000000,
	},
	.margin = 31,					/* sensor framelength & shutter margin */
	.min_shutter = 8,				/* min shutter */
	.min_gain = BASEGAIN * 1, /*1x gain*/
	.max_gain = (kal_uint32)(BASEGAIN * 15.5f), /*15.5x * 1024  gain*/
	.min_gain_iso = 100,
	.exp_step = 1,
	.gain_step = 1, /*minimum step = 4 in 1x~2x gain*/
	.gain_type = 1,/*to be modify,no gain table for sony*/
	.max_frame_length = 0xffffe9,     /* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,		//check
	.ae_sensor_gain_delay_frame = 0,//check
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 15,			//support sensor mode num

	.cap_delay_frame = 3,			//enter capture delay frame num
	.pre_delay_frame = 2,			//enter preview delay frame num
	.video_delay_frame = 2,			//enter video delay frame num
	.hs_video_delay_frame = 2,		//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,	//enter slim video delay frame num
	.custom1_delay_frame = 2,		//enter custom1 delay frame num
	.custom2_delay_frame = 2,		//enter custom2 delay frame num
	.custom3_delay_frame = 2,		//enter custom3 delay frame num
	.custom4_delay_frame = 2,		//enter custom4 delay frame num
	.custom5_delay_frame = 2,		//enter custom5 delay frame num
	.custom6_delay_frame = 2,		//enter custom6 delay frame num
	.custom7_delay_frame = 2,		//enter custom6 delay frame num
	.custom8_delay_frame = 2,		//enter custom6 delay frame num
	.custom9_delay_frame = 2,		//enter custom6 delay frame num
	.custom10_delay_frame = 2,		//enter custom6 delay frame num
	.frame_time_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
#ifdef CPHY_3TRIO
	.mipi_sensor_type = MIPI_CPHY,
#else
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
#endif
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
#ifdef CPHY_3TRIO
	.mipi_lane_num = SENSOR_MIPI_3_LANE,//mipi lane num
#else
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
#endif
	.i2c_addr_table = {0x6d, 0xff},
	.i2c_speed = 1000,
	.xtalk_flag = KAL_FALSE,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[17] = {
    {9280, 6944, 16,   0, 9248, 6944, 4624, 3472, 0, 2, 4624, 3468, 0, 0, 4624, 3468}, /*Preview*/
    {9280, 6944, 16,   0, 9248, 6944, 4624, 3472, 0, 2, 4624, 3468, 0, 0, 4624, 3468}, /*capture*/
    {9280, 6944, 16, 864, 9248, 5216, 4624, 2608, 0, 4, 4624, 2600, 0, 0, 4624, 2600}, /*normal video*/
    {9280, 6944,640,1216, 8000, 4512, 4000, 2256, 0, 0, 4000, 2256, 0, 0, 4000, 2256}, /*hs_video*/
    {9280, 6944, 32, 864, 9216, 5216, 2304, 1304, 0, 4, 2304, 1296, 0, 0, 2304, 1296}, /*slim video*/
    {9280, 6944, 32, 864, 9216, 5216, 2304, 1304, 0, 4, 2304, 1296, 0, 0, 2304, 1296}, /*custom1*/
    {9280, 6944, 16,   0, 9248, 6944, 4624, 3472, 0, 2, 4624, 3468, 0, 0, 4624, 3468}, /*custom2*/
    {9280, 6944,  0,   0, 9280, 6944, 9280, 6944,16, 4, 9248, 6936, 0, 0, 9248, 6936}, /*custom3*/
    {9280, 6944, 16, 864, 9248, 5216, 4624, 2608, 0, 4, 4624, 2600, 0, 0, 4624, 2600}, /*custom4*/
    {9280, 6944, 2320, 1736, 4640, 3472, 4640, 3472, 8, 2, 4624, 3468, 0, 0, 4624, 3468},  /*custom5*/
    {9280, 6944, 16,   0, 9248, 6944, 4624, 3472, 0, 2, 4624, 3468, 0, 0, 4624, 3468}, /*custom6*/
    {9280, 6944, 16, 864, 9248, 5216, 4624, 2608, 0, 4, 4624, 2600, 0, 0, 4624, 2600},  /*custom7*/
    {9280, 6944, 16, 864, 9248, 5216, 4624, 2608, 0, 4, 4624, 2600, 0, 0, 4624, 2600}, /*custom8*/
    {9280, 6944, 16,   0, 9248, 6944, 2312, 1736, 0, 0, 2312, 1736, 4, 4, 2304, 1728}, /*custom9*/
    {9280, 6944, 16,   0, 9248, 6944, 2312, 1736, 0, 0, 2312, 1736, 4, 4, 2304, 1728} /*custom10*/
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[5] = {//liupeng: need to update
    /*4624*3468 1136(pxiel)*860 568pixel/line * (860*2)line 46fps cap pre*/
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
        0x00, 0x2b, 0x1210, 0x0D8C, 0x00, 0x00, 0x0280, 0x0001,
        0x01, 0x2B, 0x0470, 0x035C, 0x03, 0x00, 0x0000, 0x0000
    },
    /*4624*3468 1136(pxiel)*860 568pixel/line * (860*2)line 30/24fps custom2/custom6*/
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
        0x00, 0x2b, 0x1210, 0x0D8C, 0x00, 0x00, 0x0280, 0x0001,
        0x01, 0x2B, 0x0470, 0x02EC, 0x03, 0x00, 0x0000, 0x0000
    },
    /*4624*2600 1136(pxiel)*648 568pixel/line*(648*2)line 60fps/30fps */
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
        0x00, 0x2b, 0x1210, 0x0A28, 0x00, 0x00, 0x0280, 0x0001,
        0x01, 0x2B, 0x0470, 0x0288, 0x03, 0x00, 0x0000, 0x0000
    },
    /* 4000*2256 992(pxiel)*564 992pixel/line*(564*2)line 60fps*/
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
        0x00, 0x2b, 0x0FA0, 0x08D0, 0x00, 0x00, 0x0280, 0x0001,
        0x01, 0x2B, 0x03E0, 0x0234, 0x03, 0x00, 0x0000, 0x0000
    },
    /*4624*3468 576(pxiel)*864 576pixel/line * (864*2)line 46fps_custom5*/
    {
        0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
        0x00, 0x2b, 0x1210, 0x0A28, 0x00, 0x00, 0x0280, 0x0001,
        0x01, 0x2B, 0x0240, 0x0360, 0x03, 0x00, 0x0000, 0x0000
    },
};


#if FPT_PDAF_SUPPORT
/*PD information update*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
     .i4OffsetX = 40,
     .i4OffsetY = 14,
     .i4PitchX = 16,
     .i4PitchY = 16,
     .i4PairNum = 8,
     .i4SubBlkW = 8,
     .i4SubBlkH = 4,
     .i4PosL = {{47, 16}, {55, 16}, {43, 20}, {51, 20},
                {47, 24}, {55, 24}, {43, 28}, {51, 28} },
     .i4PosR = {{46, 16}, {54, 16}, {42, 20}, {50, 20},
                {46, 24}, {54, 24}, {42, 28}, {50, 28} },
     .iMirrorFlip = 3,
     .i4BlockNumX = 284,
     .i4BlockNumY = 215,
     .i4Crop = { {0, 0}, {0, 0}, {0, 434}, {312, 606}, {1160, 1086},
        {1160, 1086}, {0, 0}, {0, 0}, {0, 434}, {0, 0}, {1160, 1086},
        {0, 434}, {0, 434}, {1160, 870}, {1160, 870}},
};
#endif

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765	/*trans# max is 255, each 3 bytes*/
#else
#define I2C_BUFFER_LEN 3
#endif

#define OV64B_EEPROM_READ_ID  0xA0
static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	u8 data;

	adaptor_i2c_rd_u8(ctx->i2c_client, OV64B_EEPROM_READ_ID >> 1, addr, &data);

	return (u16)data;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, OV64B_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

/*OVPD-1:720Bytes & Crosstalk 288Bytes*/
static kal_uint8 ov64b_QSC_OVPD_setting[720];
static kal_uint8 ov64b_QSC_CT_setting[288];
static kal_uint8 ct_qsc_flag = 0;
static kal_uint8 crossTalk_flag = 0;
static kal_uint16 senaddr_crotalk = 0x5A40;
static kal_uint16 senaddr_ovpd = 0x5F80;

static void read_EepromQSC(struct subdrv_ctx *ctx)
{
    kal_uint16 addr_ovpd = 0x190C;
    kal_uint16 addr_crotalk = 0x1C00;
    kal_uint16 i = 0;
    //kal_uint32 Dac_master = 0, Dac_mac = 0, Dac_inf = 0;
    //kal_uint32 vcmID = 0, lensID = 0;
    /*Read OVPD*/
    for (i = 0; i < 720; i ++) {
        ov64b_QSC_OVPD_setting[i] = read_cmos_eeprom_8(ctx, (addr_ovpd+i));
    }
    /*Read crosstalk*/
    for (i = 0; i < 288; i ++) {
        ov64b_QSC_CT_setting[i] = read_cmos_eeprom_8(ctx, (addr_crotalk+i));
    }
}

static void write_sensor_OVPD_QSC(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    seq_write_cmos_sensor(ctx, senaddr_ovpd, ov64b_QSC_OVPD_setting, sizeof(ov64b_QSC_OVPD_setting) / sizeof(kal_uint8));
    LOG_INF("%s end\n", __func__);
}

static void write_sensor_CT_QSC(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    if(!ct_qsc_flag) { //crosstalk Flag valid
		seq_write_cmos_sensor(ctx, senaddr_crotalk, ov64b_QSC_CT_setting, sizeof(ov64b_QSC_CT_setting) / sizeof(kal_uint8));
		ct_qsc_flag = 1;
    }
    LOG_INF("%s end\n", __func__);
}

static void write_sensor_crossTalk_data(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    if(!crossTalk_flag) { //crosstalk Flag valid
        seq_write_cmos_sensor(ctx, XTALK_GLOBAL_ADDR, addr_data_pair_crossTalk_ov64b2q, sizeof(addr_data_pair_crossTalk_ov64b2q) / sizeof(kal_uint8));
        crossTalk_flag = 1;
    }
    LOG_INF("%s end\n", __func__);
}

#define   WRITE_DATA_MAX_LENGTH     (16)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, OV64B_EEPROM_WRITE_ID >> 1,
            addr, para, WRITE_DATA_MAX_LENGTH);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    if ( enable ) {
        adaptor_i2c_wr_u8(ctx->i2c_client, OV64B_EEPROM_WRITE_ID >> 1, 0x8000, 0x0E);
    } else {
        adaptor_i2c_wr_u8(ctx->i2c_client, OV64B_EEPROM_WRITE_ID >> 1, 0x8000, 0x00);
    }
    return ret;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
            ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
	kal_int32  ret = ERROR_NONE;
	kal_uint16 data_base, data_length;
	kal_uint32 idx, idy;
	kal_uint8 *pData;
	UINT32 i = 0;
	if(pStereodata != NULL) {
		LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
					   pStereodata->uSensorId,
					   pStereodata->uDeviceId,
					   pStereodata->baseAddr,
					   pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == OV64B_SENSOR_ID_22801) && (data_length == CALI_DATA_MASTER_LENGTH)
			&& (data_base == OV64B_STEREO_START_ADDR)) {
			LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);
			idx = data_length/WRITE_DATA_MAX_LENGTH;
			idy = data_length%WRITE_DATA_MAX_LENGTH;
			/* close write protect */
			write_eeprom_protect(ctx, 0);
			msleep(6);
			for (i = 0; i < idx; i++ ) {
				ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*i),
						&pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
				if (ret != ERROR_NONE) {
					LOG_INF("write_eeprom error: i= %d\n", i);
					/* open write protect */
					write_eeprom_protect(ctx, 1);
					msleep(6);
					return -1;
				}
				msleep(6);
			}
			ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*idx),
					&pData[WRITE_DATA_MAX_LENGTH*idx], idy);
			if (ret != ERROR_NONE) {
				LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
				/* open write protect */
				write_eeprom_protect(ctx, 1);
				msleep(6);
				return -1;
			}
			//msleep(6);
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			msleep(6);
			LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, OV64B_STEREO_START_ADDR));
			LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, OV64B_STEREO_START_ADDR+39));
			LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, OV64B_STEREO_START_ADDR+40));
			LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, OV64B_STEREO_START_ADDR+1556));
			LOG_INF("write_Module_data Write end\n");
		}
	} else {
		LOG_INF("ov64b write_Module_data pStereodata is null\n");
		return -1;
	}
	return ret;
}

static void set_dummy(struct subdrv_ctx *ctx)
{

	if (!_is_seamless) {
		memset(_i2c_data, 0x0, sizeof(_i2c_data));
		_size_to_write = 0;
		_i2c_data[_size_to_write++] = 0x380c;
		_i2c_data[_size_to_write++] = ctx->line_length >> 8;
		_i2c_data[_size_to_write++] = 0x380d;
		_i2c_data[_size_to_write++] = ctx->line_length & 0xFF;
		_i2c_data[_size_to_write++] = 0x380e;
		_i2c_data[_size_to_write++] = ctx->frame_length >> 8;
		_i2c_data[_size_to_write++] = 0x380f;
		_i2c_data[_size_to_write++] = ctx->frame_length & 0xFF;
		table_write_cmos_sensor(ctx, _i2c_data,
		_size_to_write);
		//ctx->frame_length = (ctx->frame_length  >> 1) << 1;
		//write_cmos_sensor_8(ctx, 0x3208, 0x00);
		//write_cmos_sensor_8(ctx, 0x380c, ctx->line_length >> 8);
		//write_cmos_sensor_8(ctx, 0x380d, ctx->line_length & 0xFF);
		//write_cmos_sensor_8(ctx, 0x380e, ctx->frame_length >> 8);
		//write_cmos_sensor_8(ctx, 0x380f, ctx->frame_length & 0xFF);
		//write_cmos_sensor_8(ctx, 0x3208, 0x10);
		//write_cmos_sensor_8(ctx, 0x3208, 0xa0);
	} else {
		_i2c_data[_size_to_write++] = 0x3840;
		_i2c_data[_size_to_write++] = ctx->frame_length >> 16;
		_i2c_data[_size_to_write++] = 0x380e;
		_i2c_data[_size_to_write++] = ctx->frame_length >> 8;
		_i2c_data[_size_to_write++] = 0x380f;
		_i2c_data[_size_to_write++] = ctx->frame_length & 0xFF;
	}
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = ctx->frame_length;

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;

	ctx->frame_length = (frame_length > ctx->min_frame_length) ?
			frame_length : ctx->min_frame_length;
	ctx->dummy_line = ctx->frame_length -
		ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length) {
		ctx->frame_length = imgsensor_info.max_frame_length;
		ctx->dummy_line = ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;

}

static void set_max_framerate_video(struct subdrv_ctx *ctx, UINT16 framerate,
					kal_bool min_framelength_en)
{
	set_max_framerate(ctx, framerate, min_framelength_en);
	set_dummy(ctx);
}

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_8(ctx, 0x0100, 0X01);
		ctx->is_streaming = KAL_TRUE;
	} else {
		write_cmos_sensor_8(ctx, 0x0100, 0x00);
		ctx->is_streaming = KAL_FALSE;
	}
	mdelay(10);
	return ERROR_NONE;
}
static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;

	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	shutter = (shutter < imgsensor_info.min_shutter) ?
				imgsensor_info.min_shutter : shutter;
	shutter = (shutter >
				(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
				(imgsensor_info.max_frame_length - imgsensor_info.margin) :
				shutter;

	//frame_length and shutter should be an even number.
	shutter = (shutter >> 1) << 1;
	ctx->frame_length = (ctx->frame_length >> 1) << 1;

	if (ctx->autoflicker_en == KAL_TRUE) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 /
			ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
			set_max_framerate(ctx, realtime_fps, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			set_max_framerate(ctx, realtime_fps, 0);
		}
	}
	/*Warning : shutter must be even. Odd might happen Unexpected Results */
	if (!_is_seamless) {
		if(_is_initFlag) {
			memset(_i2c_data, 0x0, sizeof(_i2c_data));
			_size_to_write = 0;

			_i2c_data[_size_to_write++] = 0x3840;
			_i2c_data[_size_to_write++] = ctx->frame_length >> 16;
			_i2c_data[_size_to_write++] = 0x380e;
			_i2c_data[_size_to_write++] = ctx->frame_length >> 8;
			_i2c_data[_size_to_write++] = 0x380f;
			_i2c_data[_size_to_write++] = ctx->frame_length & 0xFE;
			_i2c_data[_size_to_write++] = 0x3500;
			_i2c_data[_size_to_write++] = (shutter >> 16) & 0xFF;
			_i2c_data[_size_to_write++] = 0x3501;
			_i2c_data[_size_to_write++] = (shutter >> 8) & 0xFF;
			_i2c_data[_size_to_write++] = 0x3502;
			_i2c_data[_size_to_write++] = (shutter)  & 0xFE;
			table_write_cmos_sensor(ctx, _i2c_data,
			_size_to_write);
			_is_initFlag = 0;
		} else {
			memset(_i2c_data, 0x0, sizeof(_i2c_data));
			_size_to_write = 0;

			_i2c_data[_size_to_write++] = 0x3208;
			_i2c_data[_size_to_write++] = 0x0a;//old 0x01,new 0x0a is change group10 to cache
			_i2c_data[_size_to_write++] = 0x3840;
			_i2c_data[_size_to_write++] = ctx->frame_length >> 16;
			_i2c_data[_size_to_write++] = 0x380e;
			_i2c_data[_size_to_write++] = ctx->frame_length >> 8;
			_i2c_data[_size_to_write++] = 0x380f;
			_i2c_data[_size_to_write++] = ctx->frame_length & 0xFE;
			_i2c_data[_size_to_write++] = 0x3500;
			_i2c_data[_size_to_write++] = (shutter >> 16) & 0xFF;
			_i2c_data[_size_to_write++] = 0x3501;
			_i2c_data[_size_to_write++] = (shutter >> 8) & 0xFF;
			_i2c_data[_size_to_write++] = 0x3502;
			_i2c_data[_size_to_write++] = (shutter) & 0xFE;
			_i2c_data[_size_to_write++] = 0x3208;
			_i2c_data[_size_to_write++] = 0x1a;//old 0x01,new 0x0a is change group10 to cache
			_i2c_data[_size_to_write++] = 0x3208;
			_i2c_data[_size_to_write++] = 0xaa;//old 0x01,new 0x0a is change group10 to cache
			table_write_cmos_sensor(ctx, _i2c_data,
			_size_to_write);
		}
	} else {
		_i2c_data[_size_to_write++] = 0x3840;
		_i2c_data[_size_to_write++] = ctx->frame_length >> 16;
		_i2c_data[_size_to_write++] = 0x380e;
		_i2c_data[_size_to_write++] = ctx->frame_length >> 8;
		_i2c_data[_size_to_write++] = 0x380f;
		_i2c_data[_size_to_write++] = ctx->frame_length & 0xFE;
		_i2c_data[_size_to_write++] = 0x3500;
		_i2c_data[_size_to_write++] = (shutter >> 16) & 0xFF;
		_i2c_data[_size_to_write++] = 0x3501;
		_i2c_data[_size_to_write++] = (shutter >> 8) & 0xFF;
		_i2c_data[_size_to_write++] = 0x3502;
		_i2c_data[_size_to_write++] = (shutter)  & 0xFE;
	}

	LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d _is_seamless %d\n",
			shutter, ctx->frame_length, realtime_fps, _is_seamless);

}
//should not be kal_uint16 -- can't reach long exp
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	ctx->shutter = shutter;
	write_shutter(ctx, shutter);
}

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint16 iReg = 0x0000;

	//platform 1xgain = 64, sensor driver 1*gain = 0x100
	iReg = gain*256/BASEGAIN;

    if(iReg < 0x100)    //sensor 1xGain
    {
        iReg = 0X100;
    }
    if(iReg > 0x3fff)    //sensor 64xGain
    {
        iReg = 0x3fff;
    }
	return iReg;		/* sensorGlobalGain */
}

static kal_uint32 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
	kal_uint16 reg_gain;
	kal_uint32 max_gain = imgsensor_info.max_gain;

	if (gain < imgsensor_info.min_gain || gain > max_gain) {
		LOG_INF("Error gain setting\n");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else if (gain > max_gain)
			gain = max_gain;
	}

	reg_gain = gain2reg(ctx, gain);
	ctx->gain = reg_gain;

	if (!_is_seamless) {
	    if(reg_gain > 0xf00) {
			memset(_i2c_data, 0x0, sizeof(_i2c_data));
			_size_to_write = 0;
			_i2c_data[_size_to_write++] = 0x03508;
			_i2c_data[_size_to_write++] =  0xf;
			_i2c_data[_size_to_write++] = 0x03509;
			_i2c_data[_size_to_write++] =  0x00;
			reg_gain = reg_gain*40/150; // calculate 15xAgain+ Dgain
			_i2c_data[_size_to_write++] = 0x0350A;
			_i2c_data[_size_to_write++] =  (reg_gain >> 10 )& 0xF;
			_i2c_data[_size_to_write++] = 0x0350B;
			_i2c_data[_size_to_write++] =  (reg_gain >> 2) & 0xFF;
			_i2c_data[_size_to_write++] = 0x0350C;
			_i2c_data[_size_to_write++] =  (reg_gain & 0x3) << 6;

			table_write_cmos_sensor(ctx, _i2c_data,
			_size_to_write);
		} else {
			memset(_i2c_data, 0x0, sizeof(_i2c_data));
			_size_to_write = 0;
			_i2c_data[_size_to_write++] = 0x03508;
			_i2c_data[_size_to_write++] =  (reg_gain >> 8);
			_i2c_data[_size_to_write++] = 0x03509;
			_i2c_data[_size_to_write++] =  (reg_gain&0xff);
			_i2c_data[_size_to_write++] = 0x0350A;
			_i2c_data[_size_to_write++] =  0x01;
			_i2c_data[_size_to_write++] = 0x0350B;
			_i2c_data[_size_to_write++] =  0x00;
			_i2c_data[_size_to_write++] = 0x0350C;
			_i2c_data[_size_to_write++] =  0x00;

			table_write_cmos_sensor(ctx, _i2c_data,
			_size_to_write);
		}
	} else {
		_i2c_data[_size_to_write++] = 0x03508;
		_i2c_data[_size_to_write++] =  reg_gain >> 8;
		_i2c_data[_size_to_write++] = 0x03509;
		_i2c_data[_size_to_write++] =  reg_gain & 0xff;
	}

	LOG_INF("gain = %d , reg_gain = 0x%x\n", gain, reg_gain);

	return gain;
}

static void set_frame_length(struct subdrv_ctx *ctx, kal_uint16 frame_length)
{
	if (frame_length > 1)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;

	/* Extend frame length */
	if (!_is_seamless) {

		memset(_i2c_data, 0x0, sizeof(_i2c_data));
		_size_to_write = 0;
	}

	_i2c_data[_size_to_write++] = 0x3840;
	_i2c_data[_size_to_write++] = ctx->frame_length >> 16;
	_i2c_data[_size_to_write++] = 0x380e;
	_i2c_data[_size_to_write++] = ctx->frame_length >> 8;
	_i2c_data[_size_to_write++] = 0x380f;
	_i2c_data[_size_to_write++] = ctx->frame_length & 0xFF;

	if (!_is_seamless) {
		table_write_cmos_sensor(ctx, _i2c_data,
					_size_to_write);
	}

	LOG_INF("Framelength: set=%d/input=%d/min=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length);
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static void set_shutter_frame_length(struct subdrv_ctx *ctx, kal_uint16 shutter,
					kal_uint16 target_frame_length)
{
	if (target_frame_length > 1)
		ctx->dummy_line = target_frame_length - ctx->frame_length;
	ctx->frame_length = ctx->frame_length + ctx->dummy_line;
	ctx->min_frame_length = ctx->frame_length;
	set_shutter(ctx, shutter);
}
/* ITD: Modify Dualcam By Jesse 190924 End */

static void set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
                kal_uint32 *shutters, kal_uint16 shutter_cnt,
                kal_uint16 frame_length)
{
    if (shutter_cnt == 1) {
        ctx->shutter = shutters[0];

        if (shutters[0] > ctx->min_frame_length - imgsensor_info.margin)
            ctx->frame_length = shutters[0] + imgsensor_info.margin;
        else
            ctx->frame_length = ctx->min_frame_length;

        if (frame_length > ctx->frame_length)
            ctx->frame_length = frame_length;
        if (ctx->frame_length > imgsensor_info.max_frame_length)
            ctx->frame_length = imgsensor_info.max_frame_length;

        if (shutters[0] < imgsensor_info.min_shutter)
            shutters[0] = imgsensor_info.min_shutter;

        /* Update Shutter */
        write_cmos_sensor_8(ctx, 0x3208, 0x01);
        write_cmos_sensor_8(ctx, 0x3840, ctx->frame_length >> 16);
        write_cmos_sensor_8(ctx, 0x380e, ctx->frame_length >> 8);
        write_cmos_sensor_8(ctx, 0x380f, ctx->frame_length & 0xFE);
        write_cmos_sensor_8(ctx, 0x3500, (shutters[0] >> 16) & 0xFF);
        write_cmos_sensor_8(ctx, 0x3501, (shutters[0] >> 8) & 0xFF);
        write_cmos_sensor_8(ctx, 0x3502, shutters[0] & 0xFE);
        write_cmos_sensor_8(ctx, 0x3208, 0x11);
        write_cmos_sensor_8(ctx, 0x3208, 0xa1);
        LOG_INF("shutters[0] =%d, framelength =%d\n", shutters[0], ctx->frame_length);
    }
}

static void ihdr_write_shutter_gain(struct subdrv_ctx *ctx, kal_uint16 le,
				kal_uint16 se, kal_uint16 gain)
{
}

static void night_mode(struct subdrv_ctx *ctx, kal_bool enable)
{
}

static void sensor_init(struct subdrv_ctx *ctx)
{
	write_cmos_sensor_8(ctx, 0x0103, 0x01);//SW Reset, need delay
	mdelay(5);
	LOG_INF("%s start\n", __func__);
	table_write_cmos_sensor(ctx,
		addr_data_pair_init_OV64B2q_22801,
		sizeof(addr_data_pair_init_OV64B2q_22801) / sizeof(kal_uint16));
	_is_initFlag = 1;
	write_sensor_crossTalk_data(ctx);//need to update
	LOG_INF("%s end\n", __func__);
}

static void preview_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("preview_setting RES_4624x3468_30.00fps\n", __func__);
	if (!_is_seamless) {
		_length = sizeof(addr_data_pair_preview_OV64B2q_22801) / sizeof(kal_uint16);
		table_write_cmos_sensor(ctx,
			addr_data_pair_preview_OV64B2q_22801,
			_length);
	} else {
		_length = sizeof(addr_data_pair_seamless_custom5_2_preview_OV64B2q_22801) / sizeof(kal_uint16);
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_seamless_custom5_2_preview_OV64B2q_22801,
			sizeof(addr_data_pair_seamless_custom5_2_preview_OV64B2q_22801));
		_size_to_write += _length;
	}
	LOG_INF("%s end\n", __func__);
}

static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	int _length = 0;

	LOG_INF("%s currefps = %d\n", __func__, currefps);
	_length = sizeof(addr_data_pair_capture_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
			addr_data_pair_capture_OV64B2q_22801,
			_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);
		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_capture_OV64B2q_22801,
			sizeof(addr_data_pair_capture_OV64B2q_22801));
		_size_to_write += _length;
	}

}

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	int _length = 0;

	LOG_INF("%s RES_4000x3000_zsl_30fps\n", __func__);
	_length = sizeof(addr_data_pair_video_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
			addr_data_pair_video_OV64B2q_22801,
			_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_video_OV64B2q_22801,
			sizeof(addr_data_pair_video_OV64B2q_22801));
		_size_to_write += _length;
	}


}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("%s RES_1280x720_120fps\n", __func__);
	_length = sizeof(addr_data_pair_hs_video_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_hs_video_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_hs_video_OV64B2q_22801,
			sizeof(addr_data_pair_hs_video_OV64B2q_22801));
		_size_to_write += _length;
	}

}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("%s RES_3840x2160_30fps\n", __func__);
	_length = sizeof(addr_data_pair_slim_video_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_slim_video_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_slim_video_OV64B2q_22801,
			sizeof(addr_data_pair_slim_video_OV64B2q_22801));
		_size_to_write += _length;
	}

}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static void custom1_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom1_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom1_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_custom1_OV64B2q_22801,
			sizeof(addr_data_pair_custom1_OV64B2q_22801));
		_size_to_write += _length;
	}

	LOG_INF("%s end\n", __func__);
}	/*	custom1_setting  */

static void custom2_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom2_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom2_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_custom2_OV64B2q_22801,
			sizeof(addr_data_pair_custom2_OV64B2q_22801));
		_size_to_write += _length;
	}

}	/*	custom2_setting  */

static void custom3_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom3_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom3_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_custom3_OV64B2q_22801,
			sizeof(addr_data_pair_custom3_OV64B2q_22801));
		_size_to_write += _length;
	}
}	/*	custom3_setting  */

static void custom4_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");

	_length = sizeof(addr_data_pair_custom4_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom4_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_custom4_OV64B2q_22801,
			sizeof(addr_data_pair_custom4_OV64B2q_22801));
		_size_to_write += _length;
	}

}	/*	custom4_setting  */

static void custom5_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");
	if (!_is_seamless) {
		_length = sizeof(addr_data_pair_custom5_OV64B2q_22801) / sizeof(kal_uint16);
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom5_OV64B2q_22801,
		_length);
	} else {
		_length = sizeof(addr_data_pair_seamless_preview_2_custom5_OV64B2q_22801) / sizeof(kal_uint16);
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_seamless_preview_2_custom5_OV64B2q_22801,
			sizeof(addr_data_pair_seamless_preview_2_custom5_OV64B2q_22801));
		_size_to_write += _length;
	}
}	/*	custom5_setting  */

static void custom6_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom6_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom6_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_custom6_OV64B2q_22801,
			sizeof(addr_data_pair_custom6_OV64B2q_22801));
		_size_to_write += _length;
	}


}	/*	custom6_setting  */

static void custom7_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom7_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom7_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_custom7_OV64B2q_22801,
			sizeof(addr_data_pair_custom7_OV64B2q_22801));
		_size_to_write += _length;
	}


}	/*	custom7_setting  */

static void custom8_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom8_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom8_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_custom8_OV64B2q_22801,
			sizeof(addr_data_pair_custom8_OV64B2q_22801));
		_size_to_write += _length;
	}


}	/*	custom8_setting  */
static void custom9_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom9_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom9_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_custom9_OV64B2q_22801,
			sizeof(addr_data_pair_custom9_OV64B2q_22801));
		_size_to_write += _length;
	}


}	/*	custom9_setting  */

static void custom10_setting(struct subdrv_ctx *ctx)
{
	int _length = 0;

	LOG_INF("E\n");
	_length = sizeof(addr_data_pair_custom10_OV64B2q_22801) / sizeof(kal_uint16);
	if (!_is_seamless) {
		table_write_cmos_sensor(ctx,
		addr_data_pair_custom10_OV64B2q_22801,
		_length);
	} else {
		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

		if (_size_to_write + _length > _I2C_BUF_SIZE) {
			LOG_INF("_too much i2c data for fast siwtch %d\n",
				_size_to_write + _length);
			return;
		}
		memcpy((void *) (_i2c_data + _size_to_write),
			addr_data_pair_custom10_OV64B2q_22801,
			sizeof(addr_data_pair_custom10_OV64B2q_22801));
		_size_to_write += _length;
		_i2c_data[_size_to_write++] = 0x5001;
		_i2c_data[_size_to_write++] = 0x3;
	}


}	/*	custom10_setting  */
//
//static void custom11_setting(struct subdrv_ctx *ctx)
//{
//	int _length = 0;
//
//	LOG_INF("E\n");
//	_length = sizeof(addr_data_pair_custom11_OV64B2q_22801) / sizeof(kal_uint16);
//	if (!_is_seamless) {
//		table_write_cmos_sensor(ctx,
//		addr_data_pair_custom11_OV64B2q_22801,
//		_length);
//	} else {
//		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
//			__func__, _is_seamless, _size_to_write);
//
//		if (_size_to_write + _length > _I2C_BUF_SIZE) {
//			LOG_INF("_too much i2c data for fast siwtch %d\n",
//				_size_to_write + _length);
//			return;
//		}
//		memcpy((void *) (_i2c_data + _size_to_write),
//			addr_data_pair_custom11_OV64B2q_22801,
//			sizeof(addr_data_pair_custom11_OV64B2q_22801));
//		_size_to_write += _length;
//		_i2c_data[_size_to_write++] = 0x5001;
//		_i2c_data[_size_to_write++] = 0x3;
//	}
//
//
//}	/*	custom11_setting  */
//static void custom12_setting(struct subdrv_ctx *ctx)
//{
//	int _length = 0;
//
//	LOG_INF("E\n");
//	_length = sizeof(addr_data_pair_custom12_OV64B2q_22801) / sizeof(kal_uint16);
//	if (!_is_seamless) {
//		table_write_cmos_sensor(ctx,
//		addr_data_pair_custom12_OV64B2q_22801,
//		_length);
//	} else {
//		LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
//			__func__, _is_seamless, _size_to_write);
//
//		if (_size_to_write + _length > _I2C_BUF_SIZE) {
//			LOG_INF("_too much i2c data for fast siwtch %d\n",
//				_size_to_write + _length);
//			return;
//		}
//		memcpy((void *) (_i2c_data + _size_to_write),
//			addr_data_pair_custom12_OV64B2q_22801,
//			sizeof(addr_data_pair_custom12_OV64B2q_22801));
//		_size_to_write += _length;
//		_i2c_data[_size_to_write++] = 0x5001;
//		_i2c_data[_size_to_write++] = 0x3;
//	}
//
//
//}	/*	custom10_setting  */
/* ITD: Modify Dualcam By Jesse 190924 End */
//static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
//{
//	u8 data;
//
//	adaptor_i2c_rd_u8(ctx->i2c_client, 0xA0 >> 1, addr, &data);
//
//	return (u16)data;
//}

//read module info
static BYTE ov64b_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static void read_module_data(struct subdrv_ctx *ctx)
{
    kal_uint16 idx = 0;
    kal_uint32 Dac_master = 0, Dac_mac = 0, Dac_inf = 0;
    read_ov64b_eeprom_info_22801(ctx, EEPROM_META_MODULE_ID,
                &(ov64b_common_data[0]), 2);
    read_ov64b_eeprom_info_22801(ctx, EEPROM_META_SENSOR_ID,
                &(ov64b_common_data[2]), 2);
    read_ov64b_eeprom_info_22801(ctx, EEPROM_META_LENS_ID,
                &(ov64b_common_data[4]), 2);
    read_ov64b_eeprom_info_22801(ctx, EEPROM_META_VCM_ID,
                &(ov64b_common_data[6]), 2);
    read_ov64b_eeprom_info_22801(ctx, EEPROM_META_MODULE_SN,
                &(ov64b_common_data[8]), 17);
    read_ov64b_eeprom_info_22801(ctx, EEPROM_META_AF_CODE,
                &(ov64b_common_data[25]), 6);
    Dac_mac = ((ov64b_common_data[26] << 8) | (ov64b_common_data[25]));
    Dac_inf = ((ov64b_common_data[28] << 8) | (ov64b_common_data[27]));
    Dac_master = (5*Dac_mac+36*Dac_inf)/41;
    memcpy(&ov64b_common_data[29], &Dac_master, 4);

    for (idx = 0; idx < 32; idx = idx + 4)
        LOG_INF("In %s:cam data: %02x %02x %02x %02x\n", __func__,
               ov64b_common_data[idx], ov64b_common_data[idx + 1],
               ov64b_common_data[idx + 2],
               ov64b_common_data[idx + 3]);
}

/* (EEPROM)PDAF step1: 0x1634~0x190F [4:11]->setting_1 [12:731]->->setting_2 */
#define PDC_SIZE_1 8 // 11-4+1
#define PDC_SIZE_2 720 // 731-12+1
#define PDC_TABLE_SIZE_1 (PDC_SIZE_1*2)
#define PDC_TABLE_SIZE_2 (PDC_SIZE_2*2)
#define PDC_EEPROM_ADDR 0x1638
#define PDC_OTP_ADDR_1 0x5C0E
#define PDC_OTP_ADDR_2 0x5900

#if SEQUENTIAL_WRITE_EN
static kal_uint8 OV64B_PDC_setting_1[PDC_SIZE_1];
static kal_uint8 OV64B_PDC_setting_2[PDC_SIZE_2];
#else
static kal_uint16 OV64B_PDC_setting_1[PDC_TABLE_SIZE_1];
static kal_uint16 OV64B_PDC_setting_2[PDC_TABLE_SIZE_2];
#endif

static void read_sensor_Cali(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;
	kal_uint16 eeprom_PDC_addr = PDC_EEPROM_ADDR;
#if SEQUENTIAL_WRITE_EN == 0
	kal_uint16 sensor_PDC_addr1 = PDC_OTP_ADDR_1;
	kal_uint16 sensor_PDC_addr2 = PDC_OTP_ADDR_2;
#endif
	/* setting_1 */
	for (idx = 0; idx < PDC_SIZE_1; idx++) {
		eeprom_PDC_addr = PDC_EEPROM_ADDR + idx;
#if SEQUENTIAL_WRITE_EN
		OV64B_PDC_setting_1[idx] = read_cmos_eeprom_8(ctx, eeprom_PDC_addr);
#else
		sensor_PDC_addr1 = PDC_OTP_ADDR_1 + idx;
		OV64B_PDC_setting_1[2 * idx] = sensor_PDC_addr1;
		OV64B_PDC_setting_1[2 * idx + 1] =
			read_cmos_eeprom_8(ctx, eeprom_PDC_addr);
#endif
	}
	/* setting_2 */
	for (idx = 0; idx < PDC_SIZE_2; idx++) {
		eeprom_PDC_addr = PDC_EEPROM_ADDR + PDC_SIZE_1 + idx;
#if SEQUENTIAL_WRITE_EN
		OV64B_PDC_setting_2[idx] = read_cmos_eeprom_8(ctx, eeprom_PDC_addr);
#else
		sensor_PDC_addr2 = PDC_OTP_ADDR_2 + idx;
		OV64B_PDC_setting_2[2 * idx] = sensor_PDC_addr2;
		OV64B_PDC_setting_2[2 * idx + 1] =
			read_cmos_eeprom_8(ctx, eeprom_PDC_addr);
#endif
	}

	ctx->is_read_preload_eeprom = 1;
}

//static void write_sensor_PDC(struct subdrv_ctx *ctx)
//{
//#if SEQUENTIAL_WRITE_EN
//	seq_write_cmos_sensor(ctx, PDC_OTP_ADDR_1,
//		OV64B_PDC_setting_1, sizeof(OV64B_PDC_setting_1));
//	seq_write_cmos_sensor(ctx, PDC_OTP_ADDR_2,
//		OV64B_PDC_setting_2, sizeof(OV64B_PDC_setting_2));
//#else
//	table_write_cmos_sensor(ctx, OV64B_PDC_setting_1,
//		sizeof(OV64B_PDC_setting_1)/sizeof(kal_uint16));
//	table_write_cmos_sensor(ctx, OV64B_PDC_setting_2,
//		sizeof(OV64B_PDC_setting_2)/sizeof(kal_uint16));
//#endif
//}

static kal_uint32 return_sensor_id(struct subdrv_ctx *ctx)
{
	return ((read_cmos_sensor_8(ctx, 0x300a) << 8) | (read_cmos_sensor_8(ctx, 0x300b)));
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	LOG_INF("read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data, OTP_SIZE);
	LOG_INF("read_otp_info end\n");
}

static void read_stereo_data(struct subdrv_ctx *ctx)
{
	LOG_INF("read_stereo_data begin\n");
	read_cmos_eeprom_p8(ctx, stereo_start_add, stereo_data, CALI_DATA_MASTER_LENGTH);
	LOG_INF("read_stereo_data end\n");
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
	do {
		*sensor_id = return_sensor_id(ctx);
		if (*sensor_id == OV64B_SENSOR_ID) {
			*sensor_id = imgsensor_info.sensor_id;
			read_module_data(ctx);
			read_stereo_data(ctx);
			read_EepromQSC(ctx);//need update
			LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
				ctx->i2c_write_id, *sensor_id);
			return ERROR_NONE;
		}
		retry--;
	} while (retry > 0);
	i++;
	retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		LOG_INF("%s: 0x%x fail\n", __func__, *sensor_id);
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

static int open(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
	do {
		sensor_id = return_sensor_id(ctx);
	if (sensor_id == OV64B_SENSOR_ID) {
		sensor_id = imgsensor_info.sensor_id;
		LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
			ctx->i2c_write_id, sensor_id);
		break;
	}
		retry--;
	} while (retry > 0);
	i++;
	if (sensor_id == imgsensor_info.sensor_id)
		break;
	retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		LOG_INF("Open sensor id: 0x%x fail\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	sensor_init(ctx);

	//write_sensor_PDC(ctx);
	write_sensor_OVPD_QSC(ctx);//need to update
	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->shutter = 0x3D0;
	ctx->gain = 0x100;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	//ctx->ihdr_en = 0;
	ctx->test_pattern = KAL_FALSE;
	ctx->current_fps = imgsensor_info.pre.max_framerate;

	imgsensor_info.xtalk_flag = KAL_FALSE;

	return ERROR_NONE;
}

static int close(struct subdrv_ctx *ctx)
{
	_is_seamless = KAL_FALSE;
	_size_to_write = 0;
	ct_qsc_flag = 0;
	crossTalk_flag = 0;
	return ERROR_NONE;
}   /*  close  */

static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	write_sensor_CT_QSC(ctx);
	preview_setting(ctx);
	return ERROR_NONE;
}

static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	capture_setting(ctx, ctx->current_fps);
	return ERROR_NONE;
} /* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	normal_video_setting(ctx, ctx->current_fps);
	return ERROR_NONE;
}

static kal_uint32 hs_video(struct subdrv_ctx *ctx,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	hs_video_setting(ctx);
	return ERROR_NONE;
}

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	slim_video_setting(ctx);
	return ERROR_NONE;
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static kal_uint32 Custom1(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	ctx->pclk = imgsensor_info.custom1.pclk;
	ctx->line_length = imgsensor_info.custom1.linelength;
	ctx->frame_length = imgsensor_info.custom1.framelength;
	ctx->min_frame_length = imgsensor_info.custom1.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom1_setting(ctx);
	return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	ctx->pclk = imgsensor_info.custom2.pclk;
	ctx->line_length = imgsensor_info.custom2.linelength;
	ctx->frame_length = imgsensor_info.custom2.framelength;
	ctx->min_frame_length = imgsensor_info.custom2.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom2_setting(ctx);
	return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	ctx->pclk = imgsensor_info.custom3.pclk;
	ctx->line_length = imgsensor_info.custom3.linelength;
	ctx->frame_length = imgsensor_info.custom3.framelength;
	ctx->min_frame_length = imgsensor_info.custom3.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom3_setting(ctx);
	return ERROR_NONE;
}   /*  Custom3*/


static kal_uint32 Custom4(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	ctx->pclk = imgsensor_info.custom4.pclk;
	ctx->line_length = imgsensor_info.custom4.linelength;
	ctx->frame_length = imgsensor_info.custom4.framelength;
	ctx->min_frame_length = imgsensor_info.custom4.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom4_setting(ctx);
	return ERROR_NONE;
}   /*  Custom4	*/

static kal_uint32 Custom5(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	ctx->pclk = imgsensor_info.custom5.pclk;
	ctx->line_length = imgsensor_info.custom5.linelength;
	ctx->frame_length = imgsensor_info.custom5.framelength;
	ctx->min_frame_length = imgsensor_info.custom5.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom5_setting(ctx);
	return ERROR_NONE;
} /*	Custom5 */

static kal_uint32 Custom6(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM6;
	ctx->pclk = imgsensor_info.custom6.pclk;
	ctx->line_length = imgsensor_info.custom6.linelength;
	ctx->frame_length = imgsensor_info.custom6.framelength;
	ctx->min_frame_length = imgsensor_info.custom6.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom6_setting(ctx);
	return ERROR_NONE;
} /*	Custom6 */

static kal_uint32 Custom7(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM7;
	ctx->pclk = imgsensor_info.custom7.pclk;
	ctx->line_length = imgsensor_info.custom7.linelength;
	ctx->frame_length = imgsensor_info.custom7.framelength;
	ctx->min_frame_length = imgsensor_info.custom7.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom7_setting(ctx);
	return ERROR_NONE;
} /*	Custom7 */

static kal_uint32 Custom8(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM8;
	ctx->pclk = imgsensor_info.custom8.pclk;
	ctx->line_length = imgsensor_info.custom8.linelength;
	ctx->frame_length = imgsensor_info.custom8.framelength;
	ctx->min_frame_length = imgsensor_info.custom8.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom8_setting(ctx);
	return ERROR_NONE;
} /*	Custom8 */

static kal_uint32 Custom9(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM9;
	ctx->pclk = imgsensor_info.custom9.pclk;
	ctx->line_length = imgsensor_info.custom9.linelength;
	ctx->frame_length = imgsensor_info.custom9.framelength;
	ctx->min_frame_length = imgsensor_info.custom9.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom9_setting(ctx);
	return ERROR_NONE;
} /*	Custom9 */

static kal_uint32 Custom10(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM10;
	ctx->pclk = imgsensor_info.custom10.pclk;
	ctx->line_length = imgsensor_info.custom10.linelength;
	ctx->frame_length = imgsensor_info.custom10.framelength;
	ctx->min_frame_length = imgsensor_info.custom10.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom10_setting(ctx);
	return ERROR_NONE;
} /*	Custom10 */

//static kal_uint32 Custom11(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
//					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
//{
//	LOG_INF("%s E\n", __func__);
//
//	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM11;
//	ctx->pclk = imgsensor_info.custom11.pclk;
//	ctx->line_length = imgsensor_info.custom11.linelength;
//	ctx->frame_length = imgsensor_info.custom11.framelength;
//	ctx->min_frame_length = imgsensor_info.custom11.framelength;
//	ctx->autoflicker_en = KAL_FALSE;
//	custom11_setting(ctx);
//	return ERROR_NONE;
//} /*	Custom11 */
//
//static kal_uint32 Custom12(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
//					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
//{
//	LOG_INF("%s E\n", __func__);
//
//	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM12;
//	ctx->pclk = imgsensor_info.custom12.pclk;
//	ctx->line_length = imgsensor_info.custom12.linelength;
//	ctx->frame_length = imgsensor_info.custom12.framelength;
//	ctx->min_frame_length = imgsensor_info.custom12.framelength;
//	ctx->autoflicker_en = KAL_FALSE;
//	custom12_setting(ctx);
//	return ERROR_NONE;
//} /*	Custom12 */

/* ITD: Modify Dualcam By Jesse 190924 End */

static int get_resolution(struct subdrv_ctx *ctx,
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	int i = 0;

	for (i = SENSOR_SCENARIO_ID_MIN; i < SENSOR_SCENARIO_ID_MAX; i++) {
		if (i < imgsensor_info.sensor_mode_num) {
			sensor_resolution->SensorWidth[i] = imgsensor_winsize_info[i].w2_tg_size;
			sensor_resolution->SensorHeight[i] = imgsensor_winsize_info[i].h2_tg_size;
		} else {
			sensor_resolution->SensorWidth[i] = 0;
			sensor_resolution->SensorHeight[i] = 0;
		}
	}

	return ERROR_NONE;
}   /*  get_resolution  */

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
		      MSDK_SENSOR_INFO_STRUCT *sensor_info,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = KAL_FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	//sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_PREVIEW] =
		imgsensor_info.pre_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_CAPTURE] =
		imgsensor_info.cap_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_VIDEO] =
		imgsensor_info.video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO] =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_SLIM_VIDEO] =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM1] =
		imgsensor_info.custom1_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM2] =
		imgsensor_info.custom2_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM3] =
		imgsensor_info.custom3_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM4] =
		imgsensor_info.custom4_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM5] =
		imgsensor_info.custom5_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM6] =
		imgsensor_info.custom6_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM7] =
		imgsensor_info.custom7_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM8] =
		imgsensor_info.custom8_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM9] =
		imgsensor_info.custom9_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM10] =
		imgsensor_info.custom10_delay_frame;
//	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM11] =
//		imgsensor_info.custom11_delay_frame;
//	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM12] =
//		imgsensor_info.custom12_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
#if FPT_PDAF_SUPPORT
/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/
	sensor_info->PDAF_Support = 2;
#else
	sensor_info->PDAF_Support = 0;
#endif

	//sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;   // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	return ERROR_NONE;
}   /*  get_info  */


static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->current_scenario_id = scenario_id;

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		preview(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		capture(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		normal_video(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		hs_video(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		slim_video(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		Custom1(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		Custom2(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		Custom3(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM4:
		Custom4(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM5:
		Custom5(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM6:
		Custom6(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM7:
		Custom7(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM8:
		Custom8(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM9:
		Custom9(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM10:
		Custom10(ctx, image_window, sensor_config_data);
	break;
	//case SENSOR_SCENARIO_ID_CUSTOM11:
	//	Custom11(ctx, image_window, sensor_config_data);
	//break;
	//case SENSOR_SCENARIO_ID_CUSTOM12:
	//	Custom12(ctx, image_window, sensor_config_data);
	//break;

/* ITD: Modify Dualcam By Jesse 190924 End */
	default:
		LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
	return ERROR_INVALID_SCENARIO_ID;
	}

	return ERROR_NONE;
}   /* control(ctx) */

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;

	if ((framerate == 300) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 296;
	else if ((framerate == 150) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 146;
	else
		ctx->current_fps = framerate;

	set_max_framerate_video(ctx, ctx->current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx, kal_bool enable,
			UINT16 framerate)
{
	DEBUG_LOG(ctx, "enable = %d, framerate = %d\n",
		enable, framerate);

	if (enable) //enable auto flicker
		ctx->autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		ctx->autoflicker_en = KAL_FALSE;

	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frameHeight;

	DEBUG_LOG(ctx, "scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	if (framerate == 0)
		return ERROR_NONE;

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	    frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
		ctx->dummy_line =
			(frameHeight > imgsensor_info.pre.framelength) ?
			(frameHeight - imgsensor_info.pre.framelength):0;
	    ctx->frame_length = imgsensor_info.pre.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	    frameHeight = imgsensor_info.normal_video.pclk / framerate * 10 /
				imgsensor_info.normal_video.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.normal_video.framelength) ?
		(frameHeight - imgsensor_info.normal_video.framelength):0;
	    ctx->frame_length = imgsensor_info.normal_video.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	    frameHeight = imgsensor_info.cap.pclk / framerate * 10 /
			imgsensor_info.cap.linelength;

		ctx->dummy_line =
			(frameHeight > imgsensor_info.cap.framelength) ?
			(frameHeight - imgsensor_info.cap.framelength):0;
	    ctx->frame_length = imgsensor_info.cap.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	    frameHeight = imgsensor_info.hs_video.pclk / framerate * 10 /
			imgsensor_info.hs_video.linelength;
		ctx->dummy_line =
			(frameHeight > imgsensor_info.hs_video.framelength) ?
			(frameHeight - imgsensor_info.hs_video.framelength):0;
		ctx->frame_length = imgsensor_info.hs_video.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
	    frameHeight = imgsensor_info.slim_video.pclk / framerate * 10 /
			imgsensor_info.slim_video.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.slim_video.framelength) ?
			(frameHeight - imgsensor_info.slim_video.framelength):0;
	    ctx->frame_length = imgsensor_info.slim_video.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
	    frameHeight = imgsensor_info.custom1.pclk / framerate * 10 /
			imgsensor_info.custom1.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom1.framelength) ?
			(frameHeight - imgsensor_info.custom1.framelength):0;
	    ctx->frame_length = imgsensor_info.custom1.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
	    frameHeight = imgsensor_info.custom2.pclk / framerate * 10 /
			imgsensor_info.custom2.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom2.framelength) ?
			(frameHeight - imgsensor_info.custom2.framelength):0;
	    ctx->frame_length = imgsensor_info.custom2.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		frameHeight = imgsensor_info.custom3.pclk / framerate * 10 /
			imgsensor_info.custom3.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom3.framelength) ?
			(frameHeight - imgsensor_info.custom3.framelength):0;
		ctx->frame_length = imgsensor_info.custom3.framelength +
			ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM4:
	    frameHeight = imgsensor_info.custom4.pclk / framerate * 10 /
			imgsensor_info.custom4.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom4.framelength) ?
			(frameHeight - imgsensor_info.custom4.framelength):0;
	    ctx->frame_length = imgsensor_info.custom4.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM5:
		frameHeight = imgsensor_info.custom5.pclk / framerate * 10 /
			imgsensor_info.custom5.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom5.framelength) ?
			(frameHeight - imgsensor_info.custom5.framelength):0;
		ctx->frame_length = imgsensor_info.custom5.framelength +
			ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM6:
		frameHeight = imgsensor_info.custom6.pclk / framerate * 10 /
			imgsensor_info.custom6.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom6.framelength) ?
			(frameHeight - imgsensor_info.custom6.framelength):0;
		ctx->frame_length = imgsensor_info.custom6.framelength +
			ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM7:
		frameHeight = imgsensor_info.custom7.pclk / framerate * 10 /
			imgsensor_info.custom7.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom7.framelength) ?
			(frameHeight - imgsensor_info.custom7.framelength):0;
		ctx->frame_length = imgsensor_info.custom7.framelength +
			ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM8:
		frameHeight = imgsensor_info.custom8.pclk / framerate * 10 /
			imgsensor_info.custom8.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom8.framelength) ?
			(frameHeight - imgsensor_info.custom8.framelength):0;
		ctx->frame_length = imgsensor_info.custom8.framelength +
			ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM9:
		frameHeight = imgsensor_info.custom9.pclk / framerate * 10 /
			imgsensor_info.custom9.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom9.framelength) ?
			(frameHeight - imgsensor_info.custom9.framelength):0;
		ctx->frame_length = imgsensor_info.custom9.framelength +
			ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_CUSTOM10:
		frameHeight = imgsensor_info.custom10.pclk / framerate * 10 /
			imgsensor_info.custom10.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.custom10.framelength) ?
			(frameHeight - imgsensor_info.custom10.framelength):0;
		ctx->frame_length = imgsensor_info.custom10.framelength +
			ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	//case SENSOR_SCENARIO_ID_CUSTOM11:
	//	frameHeight = imgsensor_info.custom11.pclk / framerate * 10 /
	//		imgsensor_info.custom11.linelength;
	//	ctx->dummy_line = (frameHeight >
	//		imgsensor_info.custom11.framelength) ?
	//		(frameHeight - imgsensor_info.custom11.framelength):0;
	//	ctx->frame_length = imgsensor_info.custom11.framelength +
	//		ctx->dummy_line;
	//	ctx->min_frame_length = ctx->frame_length;
	//	if (ctx->frame_length > ctx->shutter)
	//		set_dummy(ctx);
	//break;
	//case SENSOR_SCENARIO_ID_CUSTOM12:
	//	frameHeight = imgsensor_info.custom12.pclk / framerate * 10 /
	//		imgsensor_info.custom12.linelength;
	//	ctx->dummy_line = (frameHeight >
	//		imgsensor_info.custom12.framelength) ?
	//		(frameHeight - imgsensor_info.custom12.framelength):0;
	//	ctx->frame_length = imgsensor_info.custom12.framelength +
	//		ctx->dummy_line;
	//	ctx->min_frame_length = ctx->frame_length;
	//	if (ctx->frame_length > ctx->shutter)
	//		set_dummy(ctx);
	//break;
/* ITD: Modify Dualcam By Jesse 190924 End */
	default:  //coding with  preview scenario by default
	    frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.pre.framelength) ?
			(frameHeight - imgsensor_info.pre.framelength):0;
	    ctx->frame_length = imgsensor_info.pre.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
	break;
	}

	DEBUG_LOG(ctx, "scenario_id = %d, framerate = %d done\n", scenario_id, framerate);

	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
			enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MUINT32 *framerate)
{
	LOG_INF("[3058]scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	    *framerate = imgsensor_info.pre.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	    *framerate = imgsensor_info.normal_video.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	    *framerate = imgsensor_info.cap.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
	break;
/* ITD: Modify Dualcam By Jesse 190924 Start */
	case SENSOR_SCENARIO_ID_CUSTOM1:
	    *framerate = imgsensor_info.custom1.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
	    *framerate = imgsensor_info.custom2.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
	    *framerate = imgsensor_info.custom3.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_CUSTOM4:
	    *framerate = imgsensor_info.custom4.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_CUSTOM5:
	    *framerate = imgsensor_info.custom5.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_CUSTOM6:
	    *framerate = imgsensor_info.custom6.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_CUSTOM7:
	    *framerate = imgsensor_info.custom7.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_CUSTOM8:
	    *framerate = imgsensor_info.custom8.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_CUSTOM9:
	    *framerate = imgsensor_info.custom9.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_CUSTOM10:
	    *framerate = imgsensor_info.custom10.max_framerate;
	break;
	//case SENSOR_SCENARIO_ID_CUSTOM11:
	//    *framerate = imgsensor_info.custom11.max_framerate;
	//break;
	//case SENSOR_SCENARIO_ID_CUSTOM12:
	//    *framerate = imgsensor_info.custom12.max_framerate;
	//break;
/* ITD: Modify Dualcam By Jesse 190924 End */
	default:
	break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_bool enable)
{
	DEBUG_LOG(ctx, "Test_Pattern enable: %d\n", enable);
	if (enable) {
		write_cmos_sensor_8(ctx, 0x350a, 0x00);
		write_cmos_sensor_8(ctx, 0x401a, 0x00);
		write_cmos_sensor_8(ctx, 0x3019, 0xf0);
		write_cmos_sensor_8(ctx, 0x4308, 0x01);
		write_cmos_sensor_8(ctx, 0x4300, 0x00);
		write_cmos_sensor_8(ctx, 0x4302, 0x00);
		write_cmos_sensor_8(ctx, 0x4304, 0x00);
		write_cmos_sensor_8(ctx, 0x4306, 0x00);
	} else {
		write_cmos_sensor_8(ctx, 0x350a, 0x01);
		write_cmos_sensor_8(ctx, 0x401a, 0x40);
		write_cmos_sensor_8(ctx, 0x3019, 0xd2);
		write_cmos_sensor_8(ctx, 0x4308, 0x00);
		write_cmos_sensor_8(ctx, 0x4300, 0x00);
		write_cmos_sensor_8(ctx, 0x4302, 0x00);
		write_cmos_sensor_8(ctx, 0x4304, 0x00);
		write_cmos_sensor_8(ctx, 0x4306, 0x00);
	}

	ctx->test_pattern = enable;
	return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(struct subdrv_ctx *ctx)
{
	/*
	UINT32 temperature = 0;
	INT32 temperature_convert = 0;
	INT32 read_sensor_temperature = 0;
	//TEMP_SEN_CTL
	write_cmos_sensor_8(ctx, 0x4d12, 0x01);
	read_sensor_temperature = read_cmos_sensor_8(ctx, 0x4d13);

	temperature = (read_sensor_temperature << 8) |
		read_sensor_temperature;
	if (temperature < 0xc000)
		temperature_convert = temperature / 256;
	else
		temperature_convert = 192 - temperature / 256;

    if (temperature_convert < -64) {
        //LOG_INF("Temperature too low: %d\n",
                //temperature_convert);
        temperature_convert = -64;
    }
    */
    return 20;
}

static kal_uint32 seamless_switch(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
	kal_uint32 shutter, kal_uint32 gain,
	kal_uint32 shutter_2ndframe, kal_uint32 gain_2ndframe)
{
	int _length = 0;
	#if SEAMLESS_DEBUG
	int k = 0;
	#endif

	_is_seamless = KAL_TRUE;
	memset(_i2c_data, 0x0, sizeof(_i2c_data));
	_size_to_write = 0;

	LOG_INF("%s %d, %d, %d, %d, %d sizeof(_i2c_data) %d\n", __func__,
		scenario_id, shutter, gain, shutter_2ndframe, gain_2ndframe, sizeof(_i2c_data));

	_length = sizeof(addr_data_pair_seamless_switch_step1_OV64B2q_22801) / sizeof(kal_uint16);

	if (_length > _I2C_BUF_SIZE) {
		LOG_INF("_too much i2c data for fast siwtch\n");
		return ERROR_NONE;
	}

	memcpy((void *)(_i2c_data + _size_to_write),
		addr_data_pair_seamless_switch_step1_OV64B2q_22801,
		sizeof(addr_data_pair_seamless_switch_step1_OV64B2q_22801));
	_size_to_write += _length;



	control(ctx, scenario_id, NULL, NULL);
	if (shutter != 0)
		set_shutter(ctx, shutter);
	if (gain != 0)
		set_gain(ctx, gain);

	_length = sizeof(addr_data_pair_seamless_switch_step2_OV64B2q_22801) / sizeof(kal_uint16);

	if (_size_to_write + _length > _I2C_BUF_SIZE) {
		LOG_INF("_too much i2c data for fast siwtch\n");
		return ERROR_NONE;
	}

	memcpy((void *)(_i2c_data + _size_to_write),
		addr_data_pair_seamless_switch_step2_OV64B2q_22801,
		sizeof(addr_data_pair_seamless_switch_step2_OV64B2q_22801));
	_size_to_write += _length;

	if (shutter_2ndframe != 0)
		set_shutter(ctx, shutter_2ndframe);
	if (gain_2ndframe != 0)
		set_gain(ctx, gain_2ndframe);

	_length = sizeof(addr_data_pair_seamless_switch_step3_OV64B2q_22801) / sizeof(kal_uint16);
	if (_size_to_write + _length > _I2C_BUF_SIZE) {
		LOG_INF("_too much i2c data for fast siwtch\n");
		return ERROR_NONE;
	}
	memcpy((void *)(_i2c_data + _size_to_write),
		addr_data_pair_seamless_switch_step3_OV64B2q_22801,
		sizeof(addr_data_pair_seamless_switch_step3_OV64B2q_22801));
	_size_to_write += _length;

	LOG_INF("%s _is_seamless %d, _size_to_write %d\n",
			__func__, _is_seamless, _size_to_write);

	#if SEAMLESS_DEBUG
	for (k = 0; k < _size_to_write; k += 2)
		LOG_INF("before write k = %d, 0x%04x , 0x%02x\n", k,  _i2c_data[k], _i2c_data[k+1]);
	#endif

	table_write_cmos_sensor(ctx,
		_i2c_data,
		_size_to_write);

	#if SEAMLESS_DEBUG
	LOG_INF("===========================================\n");
	for (k = 0; k < _size_to_write; k += 2)
		LOG_INF("after write k = %d, 0x%04x , 0x%02x\n",
			k,  _i2c_data[k], read_cmos_sensor_8(ctx, _i2c_data[k]));
	#endif

	_is_seamless = KAL_FALSE;
	LOG_INF("exit\n");
	return ERROR_NONE;
}

static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
			UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	int ret = ERROR_NONE;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	UINT32 *pAeCtrls = NULL;
	UINT32 *pScenarios = NULL;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

#if FPT_PDAF_SUPPORT
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
#endif
	//LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_SENSOR_OTP_ALL:
		{
			LOG_INF("get otp data");
			if(otp_data[0] == 0) {
				read_otp_info(ctx);
			} else {
				LOG_INF("otp data has already read");
			}
			memcpy(feature_return_para_32, (UINT32 *)otp_data, sizeof(otp_data));
			*feature_para_len = sizeof(otp_data);
			break;
		}
	case SENSOR_FEATURE_GET_OUTPUT_FORMAT_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM3:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		case SENSOR_SCENARIO_ID_CUSTOM5:
		case SENSOR_SCENARIO_ID_CUSTOM6:
		case SENSOR_SCENARIO_ID_CUSTOM7:
		case SENSOR_SCENARIO_ID_CUSTOM8:
		case SENSOR_SCENARIO_ID_CUSTOM9:
		case SENSOR_SCENARIO_ID_CUSTOM10:
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
	if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
		*(feature_data + 0) =
			sizeof(OV64B_ana_gain_table);
	} else {
		memcpy((void *)(uintptr_t) (*(feature_data + 1)),
		(void *)OV64B_ana_gain_table,
		sizeof(OV64B_ana_gain_table));
	}
		break;
	case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
		pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data+1)));
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			*pScenarios = SENSOR_SCENARIO_ID_CUSTOM5;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM5:
			*pScenarios = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		case SENSOR_SCENARIO_ID_CUSTOM3:
		case SENSOR_SCENARIO_ID_CUSTOM6:
		case SENSOR_SCENARIO_ID_CUSTOM7:
		case SENSOR_SCENARIO_ID_CUSTOM8:
		case SENSOR_SCENARIO_ID_CUSTOM9:
		default:
			*pScenarios = 0xff;
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %d %d\n",
			*feature_data, *pScenarios);
		break;
	case SENSOR_FEATURE_SEAMLESS_SWITCH:
		pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data+1)));
		if (pAeCtrls)
			seamless_switch(ctx, (*feature_data), *pAeCtrls,
				*(pAeCtrls+1), *(pAeCtrls+4), *(pAeCtrls+5));
		else
			seamless_switch(ctx, (*feature_data), 0, 0, 0, 0);
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MAX_EXP_LINE:
		*(feature_data + 2) =
			imgsensor_info.max_frame_length - imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		case SENSOR_SCENARIO_ID_CUSTOM5:
		case SENSOR_SCENARIO_ID_CUSTOM6:
		case SENSOR_SCENARIO_ID_CUSTOM7:
		case SENSOR_SCENARIO_ID_CUSTOM8:
		case SENSOR_SCENARIO_ID_CUSTOM9:
		case SENSOR_SCENARIO_ID_CUSTOM10:
			*(feature_data + 2) = 2;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
		default:
			*(feature_data + 2) = 1;
			break;
		}
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom4.pclk;
				break;
		case SENSOR_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom5.pclk;
				break;
		case SENSOR_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom6.pclk;
				break;
		case SENSOR_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom7.pclk;
				break;
		case SENSOR_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom8.pclk;
				break;
		case SENSOR_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom9.pclk;
				break;
		case SENSOR_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom10.pclk;
				break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= 1000000;
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom4.framelength << 16)
				+ imgsensor_info.custom4.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom5.framelength << 16)
				+ imgsensor_info.custom5.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom6.framelength << 16)
				+ imgsensor_info.custom6.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom7.framelength << 16)
				+ imgsensor_info.custom7.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom8.framelength << 16)
				+ imgsensor_info.custom8.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom9.framelength << 16)
				+ imgsensor_info.custom9.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom10.framelength << 16)
				+ imgsensor_info.custom10.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD:
	    *feature_return_para_16++ = ctx->line_length;
	    *feature_return_para_16 = ctx->frame_length;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = ctx->pclk;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_SENSOR_OTP:
		LOG_INF("set SENSOR_OTP");
		ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
		if (ret == ERROR_NONE)
		    return ERROR_NONE;
		else
		    return ERROR_MSDK_IS_ACTIVATED;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	    set_shutter(ctx, *feature_data);
	break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	    night_mode(ctx, (BOOL) * feature_data);
	break;
	case SENSOR_FEATURE_SET_GAIN:
	    set_gain(ctx, (UINT32) * feature_data);
	break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
	case SENSOR_FEATURE_SET_REGISTER:
		if (sensor_reg_data->RegAddr == 0xff)
			seamless_switch(ctx, sensor_reg_data->RegData, 1920, 369, 960, 369);
		else
			write_cmos_sensor_8(ctx, sensor_reg_data->RegAddr,
							sensor_reg_data->RegData);
	break;
	case SENSOR_FEATURE_GET_REGISTER:
	    sensor_reg_data->RegData =
			read_cmos_sensor_8(ctx, sensor_reg_data->RegAddr);
	break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
	    *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
	    set_video_mode(ctx, *feature_data);
	break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
	    get_imgsensor_id(ctx, feature_return_para_32);
	break;
	case SENSOR_FEATURE_GET_EEPROM_COMDATA:
	    memcpy(feature_return_para_32, ov64b_common_data,
	        OPLUS_CAMERA_COMMON_DATA_LENGTH);
	    *feature_para_len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
	    break;
	case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
		LOG_INF("ov64b get STEREO_DATA");
		memcpy(feature_return_para_32, (UINT32 *)stereo_data, sizeof(stereo_data));
		*feature_para_len = sizeof(stereo_data);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
	    set_auto_flicker_mode(ctx, (BOOL)*feature_data_16,
			*(feature_data_16+1));
	break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			*(feature_data+1));
	break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	    get_default_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
	break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode(ctx, (BOOL)*feature_data);
	break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	    *feature_return_para_32 = imgsensor_info.checksum_value;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    ctx->current_fps = *feature_data_32;
		LOG_INF("current fps :%d\n", ctx->current_fps);
	break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	    //LOG_INF("GET_CROP_INFO scenarioId:%d\n",
		//	*feature_data_32);

	    wininfo = (struct  SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
/* ITD: Modify Dualcam By Jesse 190924 Start */
		case SENSOR_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[5],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[6],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[7],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM4:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[8],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM5:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[9],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM6:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[10],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM7:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[11],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM8:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[12],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM9:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[13],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM10:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[14],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
/* ITD: Modify Dualcam By Jesse 190924 End */
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		}
	break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	    LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
	    ihdr_write_shutter_gain(ctx, (UINT16)*feature_data,
			(UINT16)*(feature_data+1),
				(UINT16)*(feature_data+2));
	break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
			switch (*feature_data) {
			case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.cap.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.slim_video.mipi_pixel_rate;
				break;
/* ITD: Modify Dualcam By Jesse 190924 Start */
			case SENSOR_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom1.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom2.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM3:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom3.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM4:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom4.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM5:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom5.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM6:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom6.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM7:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom7.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM8:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom8.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM9:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom9.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM10:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom10.mipi_pixel_rate;
				break;
/* ITD: Modify Dualcam By Jesse 190924 End */
			case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
	break;

	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16) *feature_data);

		pvcinfo =
	    (struct SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:/*4624*2600*/
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		case SENSOR_SCENARIO_ID_CUSTOM7:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:/*4000*2256*/
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[3],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;

		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_CUSTOM6:
		case SENSOR_SCENARIO_ID_CUSTOM2:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;

#if FPT_PDAF_SUPPORT
/******************** PDAF START ********************/
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)
			(uintptr_t)(*(feature_data+1));

		switch (*feature_data) { //need to updata
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM6:
			imgsensor_pd_info.i4BlockNumX = 284;  //4624*3468
			imgsensor_pd_info.i4BlockNumY = 215;
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			imgsensor_pd_info.i4BlockNumX = 284;  //4624*2600
			imgsensor_pd_info.i4BlockNumY = 162;
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			imgsensor_pd_info.i4BlockNumX = 248;  //4000*2256
			imgsensor_pd_info.i4BlockNumY = 141;
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		//case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		//case SENSOR_SCENARIO_ID_SLIM_VIDEO:

		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		break;
	case SENSOR_FEATURE_SET_PDAF:
			ctx->pdaf_mode = *feature_data_16;
		break;
/******************** PDAF END ********************/
#endif
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = get_sensor_temperature(ctx);
		*feature_para_len = 4;
	break;
/* ITD: Modify Dualcam By Jesse 190924 Start */
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		LOG_INF("SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME\n");
		set_shutter_frame_length(ctx, (UINT16)*feature_data, (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/* margin info by scenario */
		*(feature_data + 1) = 1; /* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
/* ITD: Modify Dualcam By Jesse 190924 End */
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(ctx, KAL_FALSE);
		break;

	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(ctx, *feature_data);
		streaming_control(ctx, KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case SENSOR_SCENARIO_ID_CUSTOM3:
		case SENSOR_SCENARIO_ID_CUSTOM5:
			*feature_return_para_32 = 1000;
			break;
		default:
			*feature_return_para_32 = 1470; /*BINNING_AVERAGED*/
			break;
		}
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_PRELOAD_EEPROM_DATA:
		/*get eeprom preloader data*/
		*feature_return_para_32 = ctx->is_read_preload_eeprom;
		*feature_para_len = 4;
		if (ctx->is_read_preload_eeprom != 1)
			read_sensor_Cali(ctx);
		break;
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME:
		set_multi_shutter_frame_length(ctx, (UINT32 *)(*feature_data),
				(UINT16) (*(feature_data + 1)),
				(UINT16) (*(feature_data + 2)));
		break;
	default:
	break;
	}

	return ERROR_NONE;
}   /*  feature_control(ctx)  */

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {//4624*3468
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0D8C,
		},
	},
	{
		.bus.csi2 = {//1136(pxiel)*860
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x035C,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {//4624*3468
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0D8C,
		},
	},
	{
		.bus.csi2 = {//1136(pxiel)*860
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x035C,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {//4624*2600
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0A28,
		},
	},
	{
		.bus.csi2 = {//1136(pxiel)*648
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x0288,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {//4000*2256
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FA0,
			.vsize = 0x08D0,
		},
	},
	{
		.bus.csi2 = {//992(pxiel)*564
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x03E0,
			.vsize = 0x0234,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {//2304*1296
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0900,
			.vsize = 0x0510,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {//2304*1296
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0900,
			.vsize = 0x0510,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {//4624*3468
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0D8C,
		},
	},
	{
		.bus.csi2 = {//1136(pxiel)*860
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x035C,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {//9248*6936
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2420,
			.vsize = 0x1B18,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {//4624*2600
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0A28,
		},
	},
	{
		.bus.csi2 = {//1136(pxiel)*648
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x0288,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {//4624*3468
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0D8C,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {//4624*3468
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0D8C,
		},
	},
	{
		.bus.csi2 = {//1136(pxiel)*860
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x035C,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {//4624*2600
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0A28,
		},
	},
	{
		.bus.csi2 = {//1136(pxiel)*648
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x0288,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0780,
			.vsize = 0x0438,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x0288,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
		.bus.csi2 = {//2312*1736
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0908,
			.vsize = 0x06c8,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1f40,
			.vsize = 0x1770,
		},
	},
};
static int get_frame_desc(struct subdrv_ctx *ctx,
		int scenario_id, struct mtk_mbus_frame_desc *fd)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_prev);
		memcpy(fd->entry, frame_desc_prev, sizeof(frame_desc_prev));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cap);
		memcpy(fd->entry, frame_desc_cap, sizeof(frame_desc_cap));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_vid);
		memcpy(fd->entry, frame_desc_vid, sizeof(frame_desc_vid));
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_hs_vid);
		memcpy(fd->entry, frame_desc_hs_vid, sizeof(frame_desc_hs_vid));
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_slim_vid);
		memcpy(fd->entry, frame_desc_slim_vid, sizeof(frame_desc_slim_vid));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus1);
		memcpy(fd->entry, frame_desc_cus1, sizeof(frame_desc_cus1));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus2);
		memcpy(fd->entry, frame_desc_cus2, sizeof(frame_desc_cus2));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus3);
		memcpy(fd->entry, frame_desc_cus3, sizeof(frame_desc_cus3));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM4:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus4);
		memcpy(fd->entry, frame_desc_cus4, sizeof(frame_desc_cus4));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM5:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus5);
		memcpy(fd->entry, frame_desc_cus5, sizeof(frame_desc_cus5));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM6:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus6);
		memcpy(fd->entry, frame_desc_cus6, sizeof(frame_desc_cus6));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM7:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus7);
		memcpy(fd->entry, frame_desc_cus7, sizeof(frame_desc_cus7));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM8:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus8);
		memcpy(fd->entry, frame_desc_cus8, sizeof(frame_desc_cus8));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM9:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus9);
		memcpy(fd->entry, frame_desc_cus9, sizeof(frame_desc_cus9));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM10:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus10);
		memcpy(fd->entry, frame_desc_cus10, sizeof(frame_desc_cus10));
		break;
	default:
		return -1;
	}

	return 0;
}
#endif

static const struct subdrv_ctx defctx = {

	.ana_gain_def = 0x100,
	.ana_gain_max = 992,
	.ana_gain_min = 64,
	.ana_gain_step = 1,
	.exposure_def = 0x3D0,
	.exposure_max = 0xffffe9 - 31,
	.exposure_min = 8,
	.exposure_step = 1,
	.frame_time_delay_frame = 2,
	.margin = 31,
	.max_frame_length = 0xffffe9,
	.is_streaming = KAL_FALSE,

	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,
	.gain = 0x100,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	//.ihdr_en = 0,
	.i2c_write_id = 0x20,
	.ae_ctrl_gph_en = 0,
};

static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int get_temp(struct subdrv_ctx *ctx, int *temp)
{
	*temp = get_sensor_temperature(ctx) * 1000;
	return 0;
}
static int get_csi_param(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id,
	struct mtk_csi_param *csi_param)
{
	csi_param->not_fixed_trail_settle = 0;
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		csi_param->dphy_clk_settle = 44;
		csi_param->dphy_trail = 85;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		csi_param->dphy_trail = 78;
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		csi_param->dphy_trail = 76;
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		csi_param->dphy_trail = 72;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		csi_param->dphy_trail = 77;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		csi_param->dphy_trail = 85;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		csi_param->dphy_trail = 74;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM4:
		csi_param->dphy_trail = 84;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM5:
		csi_param->dphy_trail = 83;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM6:
		csi_param->dphy_trail = 73;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM7:
		csi_param->dphy_trail = 84;
		break;
	default:
		break;
	}
	return 0;
}

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = get_info,
	.get_resolution = get_resolution,
	.control = control,
	.feature_control = feature_control,
	.close = close,
#ifdef IMGSENSOR_VC_ROUTING
	.get_frame_desc = get_frame_desc,
#endif
	.get_temp = get_temp,
	.get_csi_param = get_csi_param,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
			{HW_ID_RST, 0, 1},
			{HW_ID_MCLK, 24, 0},
			{HW_ID_MCLK_DRIVING_CURRENT, 6, 2},
			{HW_ID_AVDD, 2800000, 2},
			{HW_ID_DOVDD, 1800000, 1},
			{HW_ID_DVDD, 1100000, 5},
			{HW_ID_RST, 1, 5},
};

const struct subdrv_entry ov64b_mipi_raw_22801_entry = {
	.name = "ov64b_mipi_raw_22801",
	.id = OV64B_SENSOR_ID_22801,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

