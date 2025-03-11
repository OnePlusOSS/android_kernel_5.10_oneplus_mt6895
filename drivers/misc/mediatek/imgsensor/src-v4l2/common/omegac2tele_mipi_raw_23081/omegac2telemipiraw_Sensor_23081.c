// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 omegac2telemipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"

#include "omegac2telemipiraw_Sensor_23081.h"
#include "omegac2tele_ana_gain_table_23081.h"
#include "omegac2tele_Sensor_setting_23081.h"
#include "omegac2tele_eeprom_23081.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor_16(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define write_cmos_sensor_16(...) subdrv_i2c_wr_u16(__VA_ARGS__)
#define omegac2tele_table_write_cmos_sensor_8_23081(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)
#define omegac2tele_table_write_cmos_sensor_16_23081(...) subdrv_i2c_wr_regs_u16(__VA_ARGS__)

static int omegac2tele_i2c_burst_wr_regs_u16(struct subdrv_ctx *ctx, u16 * list, u32 len);
static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx,
		u16 addr, u16 *list, u32 len);

#define OMEGAC2TELE_EEPROM_WRITE_ID_23081 0xA0
#define OMEGAC2TELE_EEPROM_READ_ID_23081  0xA1

#define OMEGAC2TELE_VCM_WRITE_ID_23081 0x18
#define OMEGAC2TELE_VCM_READ_ID_23081 0x19

#define OMEGAC2TELE_UNIQUE_SENSOR_ID_ADDR_23081 0x0A24
#define OMEGAC2TELE_UNIQUE_SENSOR_ID_LENGHT_23081 6

#define OMEGAC2TELE_SENSOR_ID 0x38E5

#define PFX "OMEGAC2TELE_camera_sensor_23081"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define _I2C_BUF_SIZE 256
static kal_uint16 _i2c_data[_I2C_BUF_SIZE];
static unsigned int _size_to_write;
#define OTP_SIZE    0x4000
static kal_uint8 otp_data[OTP_SIZE] = {0};
static struct oplus_get_eeprom_common_data omegac2tele_common_data = {0};
#define MAX_BURST_LEN  2048
static u8 * msg_buf = NULL;

typedef uint64_t kal_uint64;

static bool bNeedSetNormalMode = KAL_FALSE;
static BYTE omegac2tele_unique_id[OMEGAC2TELE_UNIQUE_SENSOR_ID_LENGHT_23081] = { 0 };

static u16 otp_flag = 0;

static void commit_write_sensor(struct subdrv_ctx *ctx)
{
	if (_size_to_write) {
		omegac2tele_table_write_cmos_sensor_16_23081(ctx, _i2c_data, _size_to_write);
		memset(_i2c_data, 0x0, sizeof(_i2c_data));
		_size_to_write = 0;
	}
}

static void set_cmos_sensor_16(struct subdrv_ctx *ctx,
			kal_uint16 reg, kal_uint16 val)
{
	if (_size_to_write > _I2C_BUF_SIZE - 2)
		commit_write_sensor(ctx);

	_i2c_data[_size_to_write++] = reg;
	_i2c_data[_size_to_write++] = val;
}

static struct imgsensor_info_struct imgsensor_info = {
		.sensor_id = OMEGAC2TELE_SENSOR_ID_23081,

		.checksum_value = 0x31e3fbe2,

		.pre = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 6408,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4096,
			.grabwindow_height = 3072,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1324800000,
			.max_framerate = 300,
		},
		.cap = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 6408,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4096,
			.grabwindow_height = 3072,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1324800000,
			.max_framerate = 300,
		},
		.normal_video = {
			.pclk = 920000000,
			.linelength = 4848,
			.framelength = 6312,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4096,
			.grabwindow_height = 2304,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1324800000,
			.max_framerate = 300,
		},
		.hs_video = {
			.pclk = 920000000,
			.linelength = 4848,
			.framelength = 3156,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4096,
			.grabwindow_height = 2304,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1280000000,
			.max_framerate = 600,
		},
		.slim_video = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 1602,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 2048,
			.grabwindow_height = 1152,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 668800000,
			.max_framerate = 1200,
		},
		.custom1 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 6408,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 2048,
			.grabwindow_height = 1152,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1324800000,
			.max_framerate = 300,
		},
		.custom2 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 6408,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 2048,
			.grabwindow_height = 1536,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 668800000,
			.max_framerate = 300,
		},
		.custom3 = {
			.pclk = 920000000,
			.linelength = 9600,
			.framelength = 6344,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 8192,
			.grabwindow_height = 6144,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 902400000,
			.max_framerate = 150,
		},
		.custom4 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 6408,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4096,
			.grabwindow_height = 2304,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1324800000,
			.max_framerate = 300,
		},
		.custom5 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 12816,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 2048,
			.grabwindow_height = 1536,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 668800000,
			.max_framerate = 150,
		},
		.custom6 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 12816,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 2048,
			.grabwindow_height = 1152,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 668800000,
			.max_framerate = 150,
		},
		.custom7 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 6408,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 1836,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1324800000,
			.max_framerate = 300,
		},
		.custom8 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 12816,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 1836,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1324800000,
			.max_framerate = 150,
		},
		.custom9 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 6408,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 2448,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1324800000,
			.max_framerate = 300,
		},
		.custom10 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 12816,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 2048,
			.grabwindow_height = 1536,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 668800000,
			.max_framerate = 150,
		},
		.custom11 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 8008,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4096,
			.grabwindow_height = 3072,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1280000000,
			.max_framerate = 240,
		},
		.custom12 = {
			.pclk = 920000000,
			.linelength = 9200,
			.framelength = 3332,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4096,
			.grabwindow_height = 3072,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1324800000,
			.max_framerate = 300,
		},
        .custom13 = {
			.pclk = 920000000,
			.linelength = 4784,
			.framelength = 8008,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 4096,
			.grabwindow_height = 3072,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 1280000000,
			.max_framerate = 240,
		},
		.margin = 24,
		.min_shutter = 8,
		.min_gain = BASEGAIN*1, /*1x gain*/
		.max_gain = BASEGAIN * 80, /*80x gain*/
		.min_gain_iso = 100,
		.gain_step = 2,
		.gain_type = 2,
		.max_frame_length = 0xfffc,//0xffff-3,
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
		.ihdr_support = 0,/*1, support; 0,not support*/
		.ihdr_le_firstline = 0,/*1,le first; 0, se first*/
		.sensor_mode_num = 18,/*support sensor mode num*/
		.frame_time_delay_frame = 2,
		.cap_delay_frame = 2,
		.pre_delay_frame = 2,
		.video_delay_frame = 3,
		.hs_video_delay_frame = 3,
		.slim_video_delay_frame = 3,
		.custom1_delay_frame = 2,
		.custom2_delay_frame = 2,
		.custom3_delay_frame = 2,
		.custom4_delay_frame = 2,
		.custom5_delay_frame = 2,
		.custom6_delay_frame = 2,
		.custom7_delay_frame = 2,
		.custom8_delay_frame = 2,
		.custom9_delay_frame = 2,
		.custom10_delay_frame = 2,
		.custom11_delay_frame = 2,
		.custom12_delay_frame = 2,
		.custom13_delay_frame = 2,

		.isp_driving_current = ISP_DRIVING_4MA,
		.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
		.mipi_sensor_type = MIPI_OPHY_NCSI2,
		.mipi_settle_delay_mode = 1,
		.sensor_output_dataformat =
			SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
		.mclk = 24,
		.mipi_lane_num = SENSOR_MIPI_4_LANE,
		.i2c_addr_table = {0x5A, 0xff},
		.i2c_speed = 1000,
};

/* Sensor output window information */
/*no mirror flip*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[18] = {
	{ 8192, 6144,     0,    0, 8192, 6144, 4096,  3072,    0,    0, 4096,  3072,      0,    0, 4096, 3072}, /*Preview*/
	{ 8192, 6144,     0,    0, 8192, 6144, 4096,  3072,    0,    0, 4096,  3072,      0,    0, 4096, 3072}, /*capture*/
	{ 8192, 6144,     0,  768, 8192, 4608, 4096,  2304,    0,    0, 4096,  2304,      0,    0, 4096, 2304}, /*video*/
	{ 8192, 6144,     0,  768, 8192, 4608, 4096,  2304,    0,    0, 4096,  2304,      0,    0, 4096, 2304}, /*hs_video,don't use*/
	{ 8192, 6144,     0,  768, 8192, 4608, 2048,  1152,    0,    0, 2048,  1152,      0,    0, 2048, 1152}, /* slim video*/
	{ 8192, 6144,     0,  768, 8192, 4608, 2048,  1152,    0,    0, 2048,  1152,      0,    0, 2048, 1152}, /*custom1*/
	{ 8192, 6144,     0,    0, 8192, 6144, 2048,  1536,    0,    0, 2048,  1536,      0,    0, 2048, 1536}, /*custom2 */
	{ 8192, 6144,     0,    0, 8192, 6144, 8192,  6144,    0,    0, 8192,  6144,      0,    0, 8192, 6144}, /*custom3 */
	{ 8192, 6144,     0,  768, 8192, 4608, 4096,  2304,    0,    0, 4096,  2304,      0,    0, 4096, 2304}, /*custom4*/
	{ 8192, 6144,     0,    0, 8192, 6144, 2048,  1536,    0,    0, 2048,  1536,      0,    0, 2048, 1536}, /*custom5*/
	{ 8192, 6144,     0,  768, 8192, 4608, 2048,  1152,    0,    0, 2048,  1152,      0,    0, 2048, 1152}, /*custom6 */
	{ 8192, 6144,     0,  768, 8192, 4608, 4096,  2304,  416,  234, 3264,  1836,      0,    0, 3264, 1836}, /*custom7 */
	{ 8192, 6144,     0,  768, 8192, 4608, 4096,  2304,  416,  234, 3264,  1836,      0,    0, 3264, 1836}, /*custom8*/
	{ 8192, 6144,     0,    0, 8192, 6144, 4096,  3072,  416,  312, 3264,  2448,      0,    0, 3264, 2448}, /*custom9*/
	{ 8192, 6144,     0,    0, 8192, 6144, 2048,  1536,    0,    0, 2048,  1536,      0,    0, 2048, 1536}, /*custom10 */
	{ 8192, 6144,     0,    0, 8192, 6144, 4096,  3072,    0,    0, 4096,  3072,      0,    0, 4096, 3072}, /*custom11 */
	{ 8192, 6144,     0, 1536, 8192, 3072, 8192,  3072, 2048,    0, 4096,  3072,      0,    0, 4096, 3072}, /*custom12*/
	{ 8192, 6144,     0,    0, 8192, 6144, 4096,  3072,    0,    0, 4096,  3072,      0,    0, 4096, 3072}, /*custom13 */
};

static struct SENSOR_SETTING_INFO_STRUCT sensor_setting_info[SENSOR_SCENARIO_ID_MAX] = {
        {NORMAL_MASK},    //mode 0
        {NORMAL_MASK},    //mode 1
        {NORMAL_MASK},    //mode 2
        {HS_VIDEO_MASK},     //mode 3
        {UNUSE_MASK},     //mode 4
        {UNUSE_MASK},     //mode 5
        {UNUSE_MASK},      //mode 6
        {UNUSE_MASK},     //mode 7
        {UNUSE_MASK},      //mode 8
        {UNUSE_MASK},     //mode 9
        {UNUSE_MASK},     //mode 10
        {UNUSE_MASK},     //mode 11
        {UNUSE_MASK},     //mode 12
        {UNUSE_MASK},     //mode 13
        {UNUSE_MASK},     //mode 14
        {PORTRAIT_MASK},      //mode 15
        {INSENSORZOOM_MASK},      //mode 16
        {UNUSE_MASK},     //mode 17
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX = 0,
    .i4PitchY = 0,
    .i4PairNum = 0,
    .i4SubBlkW = 0,
    .i4SubBlkH = 0,
    .i4PosL = {{0, 0},{0, 0},{0, 0},{0, 0}},
    .i4PosR = {{0, 0},{0, 0},{0, 0},{0, 0}},
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,
    .iMirrorFlip = 0,
    .i4Crop = { {0, 0}, {0, 0}, {8, 304}, {0, 0}, {0, 0},
                {0, 0}, {0, 0}, {0, 0}, {0, 0}, {824, 620},
                {8, 304}, {0, 0},{8, 304}, {0, 0}, {824, 620},
                {8, 8}, {2048, 1536}},
};

static struct SENSOR_OTP_INFO_STRUCT cloud_otp_info[OPLUS_CAM_CAL_DATA_MAX] = {
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0000, 16}},
	}, /*OPLUS_CAM_CAL_DATA_MODULE_VERSION*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0000, 16}},
	}, /*OPLUS_CAM_CAL_DATA_PART_NUMBER*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0b00, 1868}},
	}, /*OPLUS_CAM_CAL_DATA_SHADING_TABLE*/
	{
		.OtpInfoLen = 5,
		.OtpInfo = {{0x0020, 16}, {0x0044, 16}, {0x0060, 4}, {0x006c, 4}, {0x0092, 6}},
		.isAFCodeOffset = KAL_FALSE,
	},/*OPLUS_CAM_CAL_DATA_3A_GAIN*/
	{
		.OtpInfoLen = 2,
		.OtpInfo = {{0x1300, 496}, {0x1500, 1004}},
	},/*OPLUS_CAM_CAL_DATA_PDAF*/
	{
		.OtpInfoLen = 6,
		.OtpInfo = {{0x0000, 2}, {0x0006, 2}, {0x0008, 2}, {0x000a, 2}, {0x00b0, 23}, {0x0092, 6}},
		.isAFCodeOffset = KAL_FALSE,
	}, /*OPLUS_CAM_CAL_DATA_CAMERA_INFO*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0008, 2}},
	}, /*OPLUS_CAM_CAL_DATA_DUMP*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0008, 2}},
	}, /*OPLUS_CAM_CAL_DATA_LENS_ID*/
	{
		.OtpInfoLen = 0,
	}, /*OPLUS_CAM_CAL_DATA_QSC*/
	{
		.OtpInfoLen = 0,
	}, /*OPLUS_CAM_CAL_DATA_LRC*/
	{
		.OtpInfoLen = 1,
		.OtpInfo = {{0x0000, 16384}},
	}, /*OPLUS_CAM_CAL_DATA_ALL*/
};

static void set_dummy(struct subdrv_ctx *ctx)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);
	set_cmos_sensor_16(ctx, 0x0340, ctx->frame_length);
	set_cmos_sensor_16(ctx, 0x0342, ctx->line_length);

	commit_write_sensor(ctx);
}	/*	set_dummy  */

static void set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror)
{
	kal_uint8 itemp;

	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor_8(ctx, 0x0101);
	itemp &= ~0x03;

	switch (image_mirror) {

	case IMAGE_NORMAL:
	write_cmos_sensor_8(ctx, 0x0101, itemp);
	break;

	case IMAGE_V_MIRROR:
	write_cmos_sensor_8(ctx, 0x0101, itemp | 0x02);
	break;

	case IMAGE_H_MIRROR:
	write_cmos_sensor_8(ctx, 0x0101, itemp | 0x01);
	break;

	case IMAGE_HV_MIRROR:
	write_cmos_sensor_8(ctx, 0x0101, itemp | 0x03);
	break;
	}
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
	kal_bool min_framelength_en)
{
	kal_uint32 frame_length = ctx->frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	if (frame_length >= ctx->min_frame_length)
		ctx->frame_length = frame_length;
	else
		ctx->frame_length = ctx->min_frame_length;

	ctx->dummy_line =
		ctx->frame_length - ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length) {
		ctx->frame_length = imgsensor_info.max_frame_length;
		ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;
}	/*	set_max_framerate  */

static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint64 CintR = 0;
	kal_uint64 Time_Farme = 0;

	if (shutter > ctx->min_frame_length - imgsensor_info.margin) {
		ctx->frame_length = shutter + imgsensor_info.margin;
	} else {
		ctx->frame_length = ctx->min_frame_length;
	}
	if (ctx->frame_length > imgsensor_info.max_frame_length) {
		ctx->frame_length = imgsensor_info.max_frame_length;
	}

	if (shutter < imgsensor_info.min_shutter) {
		shutter = imgsensor_info.min_shutter;
	}

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(ctx, 296,0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(ctx, 146,0);
		} else {
			// Extend frame length
			write_cmos_sensor_16(ctx, 0x0340, ctx->frame_length);
		}
	} else {
		// Extend frame length
		write_cmos_sensor_16(ctx, 0x0340, ctx->frame_length);
	}

	if (shutter >= 0xFFF0) {  // need to modify line_length & PCLK
		bNeedSetNormalMode = KAL_TRUE;

		CintR = ( (unsigned long long)shutter) / 128;
		Time_Farme = CintR + 0x0002;  // 1st framelength
		LOG_INF("CintR =%d \n", CintR);

		write_cmos_sensor_16(ctx, 0x0340, Time_Farme & 0xFFFF);  // Framelength
		write_cmos_sensor_16(ctx, 0x0202, CintR & 0xFFFF);  //shutter
		write_cmos_sensor_16(ctx, 0x0702, 0x0700);
		write_cmos_sensor_16(ctx, 0x0704, 0x0700);
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			write_cmos_sensor_16(ctx, 0x0702, 0x0000);
			write_cmos_sensor_16(ctx, 0x0704, 0x0000);
			bNeedSetNormalMode = KAL_FALSE;
		}

		write_cmos_sensor_16(ctx, 0x0340, ctx->frame_length);
		write_cmos_sensor_16(ctx, 0x0202, ctx->shutter);
	}
	LOG_INF("shutter =%d, framelength =%d \n", shutter,ctx->frame_length);
}	/*	write_shutter  */

/*
 ************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************
 */
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	ctx->shutter = shutter;

	write_shutter(ctx, shutter);
}	/*	set_shutter */

static void set_multi_shutter_frame_length(struct subdrv_ctx *ctx, kal_uint32 *shutters,
                     kal_uint16 shutter_cnt, kal_uint16 frame_length)
{
    LOG_INF("Before shutters[0] =%d, framelength =%d/%d\n",
        shutters[0], ctx->frame_length, frame_length);
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

        write_cmos_sensor_16(ctx, 0x0340, ctx->frame_length & 0xFFFF);
        write_cmos_sensor_16(ctx, 0x0202, shutters[0]);
    }
    LOG_INF("Exit shutters[0] =%d, framelength =%d/%d\n",
        shutters[0], ctx->frame_length, frame_length);

}    /* set_multi_shutter_frame_length */

static void set_frame_length(struct subdrv_ctx *ctx, kal_uint16 frame_length)
{
	if (frame_length > 1)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;

	/* Extend frame length */
	write_cmos_sensor_16(ctx, 0x0340, ctx->frame_length & 0xFFFF);

	pr_debug("Framelength: set=%d/input=%d/min=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length);
}

static void set_shutter_frame_length(struct subdrv_ctx *ctx,
	kal_uint32 shutter, kal_uint32 frame_length)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	kal_uint64 CintR = 0;
	kal_uint64 Time_Farme = 0;

	ctx->shutter = shutter;

	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;

	ctx->frame_length = ctx->frame_length + dummy_line;

	if (shutter > ctx->frame_length - imgsensor_info.margin)
	    ctx->frame_length = shutter + imgsensor_info.margin;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(ctx, 146, 0);
		} else {
			/* Extend frame length */
			write_cmos_sensor_16(ctx, 0x0340, ctx->frame_length);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16(ctx, 0x0340, ctx->frame_length);
	}

	if (shutter >= 0xFFF0) {
		bNeedSetNormalMode = KAL_TRUE;

		CintR = (5013 * (unsigned long long)shutter) / 321536;
		Time_Farme = CintR + 0x0002;
		LOG_INF("CintR =%d \n", CintR);

		write_cmos_sensor_16(ctx, 0x0340, Time_Farme & 0xFFFF);
		write_cmos_sensor_16(ctx, 0x0202, CintR & 0xFFFF);
		write_cmos_sensor_16(ctx, 0x0702, 0x0600);
		write_cmos_sensor_16(ctx, 0x0704, 0x0600);
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			write_cmos_sensor_16(ctx, 0x0702, 0x0000);
			write_cmos_sensor_16(ctx, 0x0704, 0x0000);
			bNeedSetNormalMode = KAL_FALSE;
		}

		write_cmos_sensor_16(ctx, 0x0340, ctx->frame_length);
		write_cmos_sensor_16(ctx, 0x0202, ctx->shutter);
	}

	LOG_INF("Exit! shutter =%d, framelength =%d/%d, dummy_line=%d\n",
		shutter, ctx->frame_length, frame_length, dummy_line);
}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint16 reg_gain = 0x0;
	kal_uint16 gain_value = gain;

	kal_uint32 min_gain = imgsensor_info.min_gain;
	kal_uint32 max_gain = imgsensor_info.max_gain;

	if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3
		|| ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM12) {
		max_gain = 16 * BASEGAIN;
		min_gain = 1  * BASEGAIN;
	}
	if (gain_value < min_gain || gain_value > max_gain) {
		LOG_INF("Error: gain(%d) value out of range", gain);
		if (gain_value < min_gain) {
			gain_value = min_gain;
		} else if (gain_value > max_gain) {
			gain_value = max_gain;
		}
	}

	reg_gain = gain * 32 / BASEGAIN;
	return (kal_uint16)reg_gain;
}

/*
 ************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x400)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************
 */
static kal_uint32 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
	kal_uint16 reg_gain;

	reg_gain = gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16(ctx, 0x0204, reg_gain);

	return gain;
}	/*	set_gain  */

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	int timeout = ctx->current_fps ? (10000 / ctx->current_fps) + 1 : 101;
	int i = 0;
	int framecnt = 0;

	LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_8(ctx, 0x0100, 0X01);
	} else {
		write_cmos_sensor_8(ctx, 0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mDELAY(5);
			framecnt = read_cmos_sensor_8(ctx, 0x0005);
			if (framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		LOG_INF("Stream Off Fail! framecnt= %d.\n", framecnt);
	}
	return ERROR_NONE;
}

static void sensor_init(struct subdrv_ctx *ctx)
{
	/*Global setting */
	LOG_INF("E\n");
	if (otp_flag == 0x010F || otp_flag == 0x011F || ((otp_flag >> 8) == 0x02)){
		LOG_INF("modules with OTP data\n");
		omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_short_init_1_23081,
				sizeof(addr_data_pair_short_init_1_23081)  / sizeof(kal_uint16));
		mdelay(5);
		omegac2tele_i2c_burst_wr_regs_u16(ctx, addr_data_pair_short_init_2_23081,
				sizeof(addr_data_pair_short_init_2_23081)  / sizeof(kal_uint16));
	}else{
		LOG_INF(" modules without OTP data\n");
		omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_init_1_23081,
				sizeof(addr_data_pair_init_1_23081) / sizeof(kal_uint16));
		mdelay(5);
		omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_init_2_23081,
				sizeof(addr_data_pair_init_2_23081) / sizeof(kal_uint16));
	};
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("X\n");
}	/*	sensor_init  */

static void preview_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_preview_23081,
		   sizeof(addr_data_pair_preview_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
}	/*	preview_setting  */

static void capture_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_capture_23081,
		   sizeof(addr_data_pair_capture_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
}	/*	capture_setting  */

static void normal_video_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_normal_video_23081,
		sizeof(addr_data_pair_normal_video_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
}	/*	normal_video_setting  */

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_hs_video_23081,
		sizeof(addr_data_pair_hs_video_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	hs_video_setting  */

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_slim_video_23081,
		   sizeof(addr_data_pair_slim_video_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	slim_video_setting  */

static void custom1_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom1_23081,
		   sizeof(addr_data_pair_custom1_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom1_setting  */

static void custom2_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom2_23081,
		   sizeof(addr_data_pair_custom2_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom2_setting  */

static void custom3_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom3_23081,
		   sizeof(addr_data_pair_custom3_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom3_setting  */

static void custom4_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom4_23081,
		   sizeof(addr_data_pair_custom4_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom4_setting  */

static void custom5_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom5_23081,
		   sizeof(addr_data_pair_custom5_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom5_setting  */

static void custom6_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom6_23081,
		   sizeof(addr_data_pair_custom6_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom6_setting  */

static void custom7_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom7_23081,
		   sizeof(addr_data_pair_custom7_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom7_setting  */

static void custom8_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom8_23081,
		   sizeof(addr_data_pair_custom8_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom8_setting  */

static void custom9_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom9_23081,
		   sizeof(addr_data_pair_custom9_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom9_setting  */

static void custom10_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom10_23081,
		   sizeof(addr_data_pair_custom10_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom10_setting  */

static void custom11_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom11_23081,
		   sizeof(addr_data_pair_custom11_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom11_setting  */

static void custom12_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom12_23081,
		   sizeof(addr_data_pair_custom12_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom12_setting  */

static void custom13_setting(struct subdrv_ctx *ctx)
{
	omegac2tele_table_write_cmos_sensor_16_23081(ctx, addr_data_pair_custom13_23081,
		   sizeof(addr_data_pair_custom13_23081) / sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("E\n");
}	/*	custom13_setting  */

static void read_module_data(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;

	memset(&omegac2tele_common_data, 0x00, sizeof(omegac2tele_common_data));

	omegac2tele_common_data.header[EEPROM_MODULE_ID] = 2;
	read_omegac2tele_eeprom_info_23081(ctx, EEPROM_META_MODULE_ID,
				&(omegac2tele_common_data.data[idx]), 2);
	imgsensor_info.module_id = (kal_uint16)(omegac2tele_common_data.data[idx + 1] << 8) |
				omegac2tele_common_data.data[idx];
	idx += omegac2tele_common_data.header[EEPROM_MODULE_ID];

	omegac2tele_common_data.header[EEPROM_SENSOR_ID] = 2;
	read_omegac2tele_eeprom_info_23081(ctx, EEPROM_META_SENSOR_ID,
				&(omegac2tele_common_data.data[idx]), 2);
	idx += omegac2tele_common_data.header[EEPROM_SENSOR_ID];

	omegac2tele_common_data.header[EEPROM_LENS_ID] = 2;
	read_omegac2tele_eeprom_info_23081(ctx, EEPROM_META_LENS_ID,
				&(omegac2tele_common_data.data[idx]), 2);
	idx += omegac2tele_common_data.header[EEPROM_LENS_ID];

	omegac2tele_common_data.header[EEPROM_VCM_ID] = 2;
	read_omegac2tele_eeprom_info_23081(ctx, EEPROM_META_VCM_ID,
				&(omegac2tele_common_data.data[idx]), 2);
	idx += omegac2tele_common_data.header[EEPROM_VCM_ID];

	omegac2tele_common_data.header[EEPROM_MODULE_SN] = 23;
	read_omegac2tele_eeprom_info_23081(ctx, EEPROM_META_MODULE_SN,
				&(omegac2tele_common_data.data[idx]), 23);
	idx += omegac2tele_common_data.header[EEPROM_MODULE_SN];

	omegac2tele_common_data.header[EEPROM_AF_CODE_MACRO] = 2;
	omegac2tele_common_data.header[EEPROM_AF_CODE_INFINITY] = 2;
	omegac2tele_common_data.header[EEPROM_AF_CODE_MIDDLE] = 2;
	read_omegac2tele_eeprom_info_23081(ctx, EEPROM_META_AF_CODE,
				&(omegac2tele_common_data.data[idx]), 6);

	for (idx = 0; idx < 40; idx = idx + 4) {
		LOG_INF("In %s:common data: %02x %02x %02x %02x\n", __func__,
			omegac2tele_common_data.data[idx], omegac2tele_common_data.data[idx + 1],
			omegac2tele_common_data.data[idx + 2],
			omegac2tele_common_data.data[idx + 3]);
	}
}

#define FOUR_CELL_SIZE 3072
#define FOUR_CELL_ADDR 0x150F

#define OMEGAC2TELE_XTALK_START_ADDR  0x0E00
#define OMEGAC2TELE_XTALK_DATA_SIZE   2050
#define OMEGAC2TELE_EEPROM_READ_ID  0xA1
static kal_uint8  omegac2tele_data_xtalk[OMEGAC2TELE_XTALK_DATA_SIZE];

static char four_cell_data[FOUR_CELL_SIZE + 2];

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, OMEGAC2TELE_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, OMEGAC2TELE_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static unsigned int read_4cell_data(char *data)
{
	if (data != NULL) {
		memcpy((void*)(data) , (void*)omegac2tele_data_xtalk, OMEGAC2TELE_XTALK_DATA_SIZE);
		printk("omegac2tele read 4cell data[0]=%d, data[10]=%d, data[100]=%d, data[2049]=%d \n", data[2], data[12], data[102], data[2047]);
	}
	return 0;
}

static void read_4cell_from_eeprom_omegac2tele(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 2;
	for (idx =2 ; idx <OMEGAC2TELE_XTALK_DATA_SIZE ; idx++) {
		omegac2tele_data_xtalk[idx]=read_cmos_eeprom_8(ctx,OMEGAC2TELE_XTALK_START_ADDR+idx-2);
	}
	/* First two bytes record the length of xtalk*/
	omegac2tele_data_xtalk[0]= 0;
	omegac2tele_data_xtalk[1]= 8;
}

static void read_four_cell_from_eeprom(struct subdrv_ctx *ctx, char *data)
{
	int ret, i;
	char temp;

	if (data != NULL) {
		LOG_INF("return data\n");
		memcpy(data, four_cell_data, FOUR_CELL_SIZE);
	} else {
		LOG_INF("Need to read from EEPROM\n");
		/* Check I2C is normal */
		ret = adaptor_i2c_rd_u8(ctx->i2c_client,
			OMEGAC2TELE_EEPROM_READ_ID_23081 >> 1, FOUR_CELL_ADDR, &temp);
		if (ret < 0) {
			LOG_INF("read from EEPROM failed\n");
			return;
		}
		four_cell_data[0] = (FOUR_CELL_SIZE & 0xFF);/*Low*/
		four_cell_data[1] = ((FOUR_CELL_SIZE >> 8) & 0xFF);/*High*/
		/*Multi-Read*/
		for (i = 2; i < (FOUR_CELL_SIZE + 2); i++)
			adaptor_i2c_rd_u8(ctx->i2c_client,
				OMEGAC2TELE_EEPROM_READ_ID_23081 >> 1, FOUR_CELL_ADDR, &four_cell_data[i]);
		ctx->is_read_four_cell = 1;
	}
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	LOG_INF("read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data, OTP_SIZE);
	LOG_INF("read_otp_info end\n");
}

static void read_unique_sensorid(struct subdrv_ctx *ctx)
{
	u8 i = 0;
	LOG_INF("read sensor unique sensorid");
	while (imgsensor_info.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
		subdrv_i2c_wr_u16(ctx, 0x0100, 0x0100);
		mdelay(30);
		subdrv_i2c_wr_u16(ctx, 0x0A02, 0x0000);
		subdrv_i2c_wr_u16(ctx, 0x0A00, 0x0100);
		mdelay(1);
		if (adaptor_i2c_rd_p8(ctx->i2c_client, ctx->i2c_write_id >> 1, OMEGAC2TELE_UNIQUE_SENSOR_ID_ADDR_23081,
				&(omegac2tele_unique_id[0]), OMEGAC2TELE_UNIQUE_SENSOR_ID_LENGHT_23081) < 0) {
			LOG_INF("Read sensor unique sensorid fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
		}
		i++;
	}
}

static void omegac2tele_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	*len = OMEGAC2TELE_UNIQUE_SENSOR_ID_LENGHT_23081;
	LOG_INF("get unique sensorid");
	memcpy(feature_return_para_32, omegac2tele_unique_id,
		OMEGAC2TELE_UNIQUE_SENSOR_ID_LENGHT_23081);
	LOG_INF("para :%x, get unique sensorid", *para);
}

static bool dump_i2c_enable = false;

static void dump_i2c_buf(struct subdrv_ctx *ctx, u8 * buf, u32 length)
{
	int i;
	char *out_str = NULL;
	char *strptr = NULL;
	size_t buf_size = _I2C_BUF_SIZE * sizeof(char);
	size_t remind = buf_size;
	int num = 0;

	out_str = kzalloc(buf_size + 1, GFP_KERNEL);
	if (!out_str)
		return;

	strptr = out_str;
	memset(out_str, 0, buf_size + 1);

	num = snprintf(strptr, remind,"[ ");
	remind -= num;
	strptr += num;

	for (i = 0 ; i < length; i ++) {
		num = snprintf(strptr, remind,"0x%02x, ", buf[i]);

		if (num <= 0) {
			LOG_INF("snprintf return negative at line %d\n", __LINE__);
			kfree(out_str);
			return;
		}

		remind -= num;
		strptr += num;

		if (remind <= 20) {
			LOG_INF(" write %s\n", out_str);
			memset(out_str, 0, buf_size + 1);
			strptr = out_str;
			remind = buf_size;
		}
	}

	num = snprintf(strptr, remind," ]");
	remind -= num;
	strptr += num;

	LOG_INF(" write %s\n", out_str);
	strptr = out_str;
	remind = buf_size;

	kfree(out_str);
}

static int omegac2tele_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx, u16 * list, u32 len)
{
	adapter_i2c_burst_wr_regs_u16(ctx, ctx->i2c_write_id >> 1, list, len);
	return 	0;
}

static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx ,
		u16 addr, u16 *list, u32 len)
{
	struct i2c_client *i2c_client = ctx->i2c_client;
	struct i2c_msg  msg;
	struct i2c_msg *pmsg = &msg;

	u8 *pbuf = NULL;
	u16 *plist = NULL;
	u16 *plist_end = NULL;

	u32 sent = 0;
	u32 total = 0;
	u32 per_sent = 0;
	int ret, i;

	if(!msg_buf) {
		LOG_INF("malloc msg_buf retry");
		msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
		if(!msg_buf) {
			LOG_INF("malloc error");
			return -ENOMEM;
		}
	}

	/* each msg contains addr(u16) + val(u16 *) */
	sent = 0;
	total = len / 2;
	plist = list;
	plist_end = list + len - 2;

	LOG_INF("len(%u)  total(%u)", len, total);

	while (sent < total) {

		per_sent = 0;
		pmsg = &msg;
		pbuf = msg_buf;

		pmsg->addr = addr;
		pmsg->flags = i2c_client->flags;
		pmsg->buf = pbuf;

		pbuf[0] = plist[0] >> 8;    //address
		pbuf[1] = plist[0] & 0xff;

		pbuf[2] = plist[1] >> 8;  //data 1
		pbuf[3] = plist[1] & 0xff;

		pbuf += 4;
		pmsg->len = 4;
		per_sent += 1;

		for (i = 0; i < total - sent - 1; i++) {  //Maximum number of remaining cycles - 1
			if(plist[0] + 2 == plist[2] ) {  //Addresses are consecutive
				pbuf[0] = plist[3] >> 8;
				pbuf[1] = plist[3] & 0xff;

				pbuf += 2;
				pmsg->len += 2;
				per_sent += 1;
				plist += 2;

				if(pmsg->len >= MAX_BURST_LEN) {
					break;
				}
			}
		}
		plist += 2;

		if(dump_i2c_enable) {
			LOG_INF("pmsg->len(%d) buff: ", pmsg->len);
			dump_i2c_buf(ctx, msg_buf, pmsg->len);
		}

		ret = i2c_transfer(i2c_client->adapter, pmsg, 1);

		if (ret < 0) {
			dev_info(&i2c_client->dev,
				"i2c transfer failed (%d)\n", ret);
			return -EIO;
		}

		sent += per_sent;

		LOG_INF("sent(%u)  total(%u)  per_sent(%u)", sent, total, per_sent);
	}

	return 0;
}

static void omegac2tele_get_eeprom_probe_state(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	int *eeprom_probe_state = (int*)para;
	*eeprom_probe_state = omegac2tele_read_eeprom_state_23081(ctx);
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************
 */
static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			*sensor_id =
				((read_cmos_sensor_8(ctx, 0x0000) << 8)
				| read_cmos_sensor_8(ctx, 0x0001));
			LOG_INF("read out sensor id 0x%x\n",
				*sensor_id);
			if (*sensor_id == OMEGAC2TELE_SENSOR_ID) {
				*sensor_id = imgsensor_info.sensor_id;
				read_module_data(ctx);
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, *sensor_id);
				read_4cell_from_eeprom_omegac2tele(ctx);
				read_unique_sensorid(ctx);
				write_cmos_sensor_16(ctx, 0xFCFC, 0x4000);
				otp_flag = read_cmos_sensor_16(ctx, 0x0010);
				LOG_INF("otp_flag = 0x%x\n", otp_flag);
				msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
				if(!msg_buf) {
					LOG_INF("boot stage, malloc msg_buf error");
				}
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n",
				ctx->i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id !=  imgsensor_info.sensor_id) {
/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************
 */
static int open(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = ((read_cmos_sensor_8(ctx, 0x0000) << 8) |
				read_cmos_sensor_8(ctx, 0x0001));
			if (sensor_id == OMEGAC2TELE_SENSOR_ID) {
				sensor_id = imgsensor_info.sensor_id;
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n",
				ctx->i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id !=  sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init(ctx);

	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->shutter = 0x3D0;
	ctx->gain = BASEGAIN * 4;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	ctx->ihdr_mode = 0;
	ctx->test_pattern = KAL_FALSE;
	ctx->current_fps = imgsensor_info.pre.max_framerate;

	return ERROR_NONE;
}	/*	open  */

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int close(struct subdrv_ctx *ctx)
{
	streaming_control(ctx, KAL_FALSE);

	LOG_INF("E\n");
	return ERROR_NONE;
}	/*	close  */

/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	preview_setting(ctx);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	capture_setting(ctx);

	LOG_INF("X\n");
	return ERROR_NONE;
} /* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	normal_video_setting(ctx);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;

	hs_video_setting(ctx);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;

	slim_video_setting(ctx);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/*	slim_video	 */

static kal_uint32 custom1(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	ctx->pclk = imgsensor_info.custom1.pclk;
	ctx->line_length = imgsensor_info.custom1.linelength;
	ctx->frame_length = imgsensor_info.custom1.framelength;
	ctx->min_frame_length = imgsensor_info.custom1.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;

	custom1_setting(ctx);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/*	custom1	 */

static kal_uint32 custom2(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	ctx->pclk = imgsensor_info.custom2.pclk;
	ctx->line_length = imgsensor_info.custom2.linelength;
	ctx->frame_length = imgsensor_info.custom2.framelength;
	ctx->min_frame_length = imgsensor_info.custom2.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;

	custom2_setting(ctx);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/*	custom2	 */

static kal_uint32 custom3(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	ctx->pclk = imgsensor_info.custom3.pclk;
	ctx->line_length = imgsensor_info.custom3.linelength;
	ctx->frame_length = imgsensor_info.custom3.framelength;
	ctx->min_frame_length = imgsensor_info.custom3.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;

	custom3_setting(ctx);
	LOG_INF("X\n");
	return ERROR_NONE;
}	/*	custom2	 */


static kal_uint32 custom4(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    ctx->pclk = imgsensor_info.custom4.pclk;
    ctx->line_length = imgsensor_info.custom4.linelength;
    ctx->frame_length = imgsensor_info.custom4.framelength;
    ctx->min_frame_length = imgsensor_info.custom4.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;

    custom4_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}    /* custom4 */

static kal_uint32 custom5(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    ctx->pclk = imgsensor_info.custom5.pclk;
    ctx->line_length = imgsensor_info.custom5.linelength;
    ctx->frame_length = imgsensor_info.custom5.framelength;
    ctx->min_frame_length = imgsensor_info.custom5.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    custom5_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}

static kal_uint32 custom6(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM6;
    ctx->pclk = imgsensor_info.custom6.pclk;
    ctx->line_length = imgsensor_info.custom6.linelength;
    ctx->frame_length = imgsensor_info.custom6.framelength;
    ctx->min_frame_length = imgsensor_info.custom6.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    custom6_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}

static kal_uint32 custom7(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM7;
    ctx->pclk = imgsensor_info.custom7.pclk;
    ctx->line_length = imgsensor_info.custom7.linelength;
    ctx->frame_length = imgsensor_info.custom7.framelength;
    ctx->min_frame_length = imgsensor_info.custom7.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    custom7_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}

static kal_uint32 custom8(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM8;
    ctx->pclk = imgsensor_info.custom8.pclk;
    ctx->line_length = imgsensor_info.custom8.linelength;
    ctx->frame_length = imgsensor_info.custom8.framelength;
    ctx->min_frame_length = imgsensor_info.custom8.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    custom8_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}

static kal_uint32 custom9(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM9;
    ctx->pclk = imgsensor_info.custom9.pclk;
    ctx->line_length = imgsensor_info.custom9.linelength;
    ctx->frame_length = imgsensor_info.custom9.framelength;
    ctx->min_frame_length = imgsensor_info.custom9.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    custom9_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}

static kal_uint32 custom10(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM10;
    ctx->pclk = imgsensor_info.custom10.pclk;
    ctx->line_length = imgsensor_info.custom10.linelength;
    ctx->frame_length = imgsensor_info.custom10.framelength;
    ctx->min_frame_length = imgsensor_info.custom10.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    custom10_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}

static kal_uint32 custom11(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM11;
    ctx->pclk = imgsensor_info.custom11.pclk;
    ctx->line_length = imgsensor_info.custom11.linelength;
    ctx->frame_length = imgsensor_info.custom11.framelength;
    ctx->min_frame_length = imgsensor_info.custom11.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    custom11_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}

static kal_uint32 custom12(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM12;
    ctx->pclk = imgsensor_info.custom12.pclk;
    ctx->line_length = imgsensor_info.custom12.linelength;
    ctx->frame_length = imgsensor_info.custom12.framelength;
    ctx->min_frame_length = imgsensor_info.custom12.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    custom12_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}

static kal_uint32 custom13(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM13;
    ctx->pclk = imgsensor_info.custom13.pclk;
    ctx->line_length = imgsensor_info.custom13.linelength;
    ctx->frame_length = imgsensor_info.custom13.framelength;
    ctx->min_frame_length = imgsensor_info.custom13.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    custom13_setting(ctx);
	LOG_INF("X\n");
    return ERROR_NONE;
}

static kal_uint32 seamless_switch(struct subdrv_ctx *ctx,
        enum SENSOR_SCENARIO_ID_ENUM scenario_id, uint32_t *ae_ctrl)
{
	ctx->extend_frame_length_en = KAL_FALSE;
	ctx->current_scenario_id =  scenario_id;

	switch (scenario_id) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		{
		ctx->sensor_mode = scenario_id;
		ctx->autoflicker_en = KAL_FALSE;
		ctx->pclk = imgsensor_info.pre.pclk;
		ctx->line_length = imgsensor_info.pre.linelength;
		ctx->frame_length = imgsensor_info.pre.framelength;
		ctx->min_frame_length = imgsensor_info.pre.framelength;

		write_cmos_sensor_16(ctx, 0xFCFC, 0x4000);
		write_cmos_sensor_16(ctx, 0x0B30, 0x0101);

		if (ae_ctrl) {
			LOG_INF("call SENSOR_SCENARIO_ID_NORMAL_PREVIEW %d %d",
					ae_ctrl[0], ae_ctrl[5]);
			set_shutter(ctx, ae_ctrl[0]);
			set_gain(ctx, ae_ctrl[5]);
		}

	}
	break;
	case SENSOR_SCENARIO_ID_CUSTOM12:
	{
		ctx->sensor_mode = scenario_id;
		ctx->autoflicker_en = KAL_FALSE;
		ctx->pclk = imgsensor_info.custom12.pclk;
		ctx->line_length = imgsensor_info.custom12.linelength;
		ctx->frame_length = imgsensor_info.custom12.framelength;
		ctx->min_frame_length = imgsensor_info.custom12.framelength;

		write_cmos_sensor_16(ctx, 0xFCFC, 0x4000);
		write_cmos_sensor_16(ctx, 0x0B30, 0x0100);

		if (ae_ctrl) {
			LOG_INF("call SENSOR_SCENARIO_ID_NORMAL_custom12 %d %d",
					ae_ctrl[0], ae_ctrl[5]);
			set_shutter(ctx, ae_ctrl[0]);
			set_gain(ctx, ae_ctrl[5]);
		}
	}
	break;
	default:
	{
		LOG_INF("%s error! wrong setting in set_seamless_switch = %d",
				__func__, scenario_id);
		return 0xff;
	}
	}

	ctx->fast_mode_on = KAL_TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	LOG_INF("%s success, scenario is switched to %d", __func__, scenario_id);
	return 0;
}

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
} /* get_resolution */

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_INFO_STRUCT *sensor_info,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity =
		SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity =
		SENSOR_CLOCK_POLARITY_LOW;
	/* not use */
	sensor_info->SensorHsyncPolarity =
		SENSOR_CLOCK_POLARITY_LOW;
	/* inverse with datasheet*/
	sensor_info->SensorVsyncPolarity =
		SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType =
		imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType =
		imgsensor_info.mipi_sensor_type;
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
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM11] =
        imgsensor_info.custom11_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM12] =
        imgsensor_info.custom12_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM13] =
        imgsensor_info.custom13_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;
	/* not use */
	sensor_info->SensorDrivingCurrent =
		imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame =
		imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV_QPD;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x*/
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x*/
	sensor_info->SensorPacketECCOrder = 1;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

	return ERROR_NONE;
}	/*	get_info  */

static int control(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
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
		custom1(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		custom2(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		custom3(ctx, image_window, sensor_config_data);
		break;
    case SENSOR_SCENARIO_ID_CUSTOM4:
        custom4(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM5:
        custom5(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM6:
        custom6(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM7:
        custom7(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM8:
        custom8(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM9:
        custom9(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM10:
        custom10(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM11:
        custom11(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM12:
        custom12(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM13:
        custom13(ctx, image_window, sensor_config_data);
        break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control(ctx) */

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	if (framerate == 0) {
		/* Dynamic frame rate*/
		return ERROR_NONE;
	}
	if ((framerate == 300) &&
			(ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 296;
	else if ((framerate == 150) &&
			(ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 146;
	else
		ctx->current_fps = framerate;
	set_max_framerate(ctx, ctx->current_fps, 1);
	set_dummy(ctx);

	return ERROR_NONE;
}

static void omegac2tele_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;

	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D82, awb_gain->ABS_GAIN_R * 2); //red 1024(1x)
	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D86, awb_gain->ABS_GAIN_B * 2); //blue

	LOG_INF("ABS_GAIN_GR(%d) ABS_GAIN_R(%d) ABS_GAIN_B(%d) ABS_GAIN_GB(%d)",
		awb_gain->ABS_GAIN_GR, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B, awb_gain->ABS_GAIN_GB);

	return;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx, kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	if (enable) {/*enable auto flicker*/
		ctx->autoflicker_en = KAL_TRUE;
	} else {/*Cancel Auto flick*/
		ctx->autoflicker_en = KAL_FALSE;
	}
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		frame_length = imgsensor_info.pre.pclk /
			framerate * 10 /
			imgsensor_info.pre.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.pre.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
			framerate * 10 /
			imgsensor_info.normal_video.linelength;
		ctx->dummy_line =
			(frame_length >
				imgsensor_info.normal_video.framelength) ?
			(frame_length -
				imgsensor_info.normal_video.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.normal_video.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		if (ctx->current_fps != imgsensor_info.cap.max_framerate) {
			LOG_INF("Warning: current_fps %d fps is not support",
				framerate);
			LOG_INF("so use cap's setting: %d fps!\n",
				imgsensor_info.cap.max_framerate / 10);
		}
		frame_length = imgsensor_info.cap.pclk /
			framerate * 10 /
			imgsensor_info.cap.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.cap.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk /
			framerate * 10 /
			imgsensor_info.hs_video.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) :
			0;
		ctx->frame_length =
			imgsensor_info.hs_video.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk /
			framerate * 10 /
			imgsensor_info.slim_video.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) :
			0;
		ctx->frame_length =
			imgsensor_info.slim_video.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk /
				framerate * 10 /
				imgsensor_info.custom1.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom1.framelength) ?
				(frame_length - imgsensor_info.custom1.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom1.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk /
				framerate * 10 /
				imgsensor_info.custom2.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom2.framelength) ?
				(frame_length - imgsensor_info.custom2.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom2.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk /
				framerate * 10 /
				imgsensor_info.custom3.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom3.framelength) ?
				(frame_length - imgsensor_info.custom3.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom3.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk /
				framerate * 10 /
				imgsensor_info.custom4.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom4.framelength) ?
				(frame_length - imgsensor_info.custom4.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom4.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk /
				framerate * 10 /
				imgsensor_info.custom5.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom5.framelength) ?
				(frame_length - imgsensor_info.custom5.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom5.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM6:
		frame_length = imgsensor_info.custom6.pclk /
				framerate * 10 /
				imgsensor_info.custom6.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom6.framelength) ?
				(frame_length - imgsensor_info.custom6.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom6.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM7:
		frame_length = imgsensor_info.custom7.pclk /
				framerate * 10 /
				imgsensor_info.custom7.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom7.framelength) ?
				(frame_length - imgsensor_info.custom7.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom7.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM8:
		frame_length = imgsensor_info.custom8.pclk /
				framerate * 10 /
				imgsensor_info.custom8.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom8.framelength) ?
				(frame_length - imgsensor_info.custom8.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom8.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM9:
		frame_length = imgsensor_info.custom9.pclk /
				framerate * 10 /
				imgsensor_info.custom9.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom9.framelength) ?
				(frame_length - imgsensor_info.custom9.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom9.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM10:
		frame_length = imgsensor_info.custom10.pclk /
				framerate * 10 /
				imgsensor_info.custom10.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom10.framelength) ?
				(frame_length - imgsensor_info.custom10.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom10.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM11:
		frame_length = imgsensor_info.custom11.pclk /
				framerate * 10 /
				imgsensor_info.custom11.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom11.framelength) ?
				(frame_length - imgsensor_info.custom11.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom11.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM12:
		frame_length = imgsensor_info.custom12.pclk /
				framerate * 10 /
				imgsensor_info.custom12.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom12.framelength) ?
				(frame_length - imgsensor_info.custom12.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom12.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM13:
		frame_length = imgsensor_info.custom13.pclk /
				framerate * 10 /
				imgsensor_info.custom13.linelength;
		ctx->dummy_line =
				(frame_length > imgsensor_info.custom13.framelength) ?
				(frame_length - imgsensor_info.custom13.framelength) :
				0;
		ctx->frame_length =
				imgsensor_info.custom13.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
				set_dummy(ctx);
		break;

	default:/*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk /
			framerate * 10 /
			imgsensor_info.pre.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.pre.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

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
    case SENSOR_SCENARIO_ID_CUSTOM11:
        *framerate = imgsensor_info.custom11.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM12:
        *framerate = imgsensor_info.custom12.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM13:
        *framerate = imgsensor_info.custom13.max_framerate;
        break;
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_uint32 mode)
{
	LOG_INF("mode: %d\n", mode);

	//1:Solid Color 2:Color bar 5:black
	if (mode) {
		switch(mode) {
		case 5:
			write_cmos_sensor_16(ctx, 0xFCFC, 0x4000);
			write_cmos_sensor_16(ctx, 0x020C, 0x0000);
			write_cmos_sensor_16(ctx, 0x020E, 0x0000);
			write_cmos_sensor_16(ctx, 0x0210, 0x0000);
			write_cmos_sensor_16(ctx, 0x0212, 0x0000);
			write_cmos_sensor_16(ctx, 0x0214, 0x0000);
			write_cmos_sensor_16(ctx, 0x0230, 0x0000);
			write_cmos_sensor_16(ctx, 0x0232, 0x0000);
			write_cmos_sensor_16(ctx, 0x0234, 0x0000);
			write_cmos_sensor_16(ctx, 0x0236, 0x0000);
			write_cmos_sensor_16(ctx, 0x0240, 0x0000);
			write_cmos_sensor_16(ctx, 0x0242, 0x0000);
			write_cmos_sensor_16(ctx, 0x0244, 0x0000);
			write_cmos_sensor_16(ctx, 0x0246, 0x0000);
			break;
		default:
			write_cmos_sensor_16(ctx, 0x0600, mode); /*100% Color bar*/
			break;
		}
	} else if (ctx->test_pattern) {
			write_cmos_sensor_16(ctx, 0x0600, 0x0000); /*No pattern*/
			write_cmos_sensor_16(ctx, 0xFCFC, 0x4000);
			write_cmos_sensor_16(ctx, 0x020C, 0x0000);
			write_cmos_sensor_16(ctx, 0x020E, 0x0100);
			write_cmos_sensor_16(ctx, 0x0210, 0x0100);
			write_cmos_sensor_16(ctx, 0x0212, 0x0100);
			write_cmos_sensor_16(ctx, 0x0214, 0x0100);
			write_cmos_sensor_16(ctx, 0x0230, 0x0100);
			write_cmos_sensor_16(ctx, 0x0232, 0x0100);
			write_cmos_sensor_16(ctx, 0x0234, 0x0100);
			write_cmos_sensor_16(ctx, 0x0236, 0x0100);
			write_cmos_sensor_16(ctx, 0x0240, 0x0100);
			write_cmos_sensor_16(ctx, 0x0242, 0x0100);
			write_cmos_sensor_16(ctx, 0x0244, 0x0100);
			write_cmos_sensor_16(ctx, 0x0246, 0x0100);
		LOG_INF("prv mode: %d, exit test_pattern\n", ctx->test_pattern);
	}

	ctx->test_pattern = mode;
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_data(struct subdrv_ctx *ctx, struct mtk_test_pattern_data *data)
{
	pr_debug("test_patterndata mode = %d  R = %x, Gr = %x,Gb = %x,B = %x\n", ctx->test_pattern,
		data->Channel_R >> 22, data->Channel_Gr >> 22,
		data->Channel_Gb >> 22, data->Channel_B >> 22);

	if (1 == ctx->test_pattern) {
		write_cmos_sensor_16(ctx, 0x0602, (data->Channel_R >> 22) & 0x3ff);
		write_cmos_sensor_16(ctx, 0x0604, (data->Channel_Gr >> 22) & 0x3ff);
		write_cmos_sensor_16(ctx, 0x0606, (data->Channel_Gb >> 22) & 0x3ff);
		write_cmos_sensor_16(ctx, 0x0608, (data->Channel_B >> 22) & 0x3ff);
	}

	return ERROR_NONE;
}

static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	UINT32 *pAeCtrls;
	uint32_t *pScenarios;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_SETTING_INFO_STRUCT *sensorinfo;
	struct SENSOR_OTP_INFO_STRUCT *cloudinfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	// LOG_INF("feature_id = %d\n", feature_id);
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
		case SENSOR_SCENARIO_ID_CUSTOM11:
		case SENSOR_SCENARIO_ID_CUSTOM12:
		case SENSOR_SCENARIO_ID_CUSTOM13:
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(omegac2tele_ana_gain_table_23081);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)omegac2tele_ana_gain_table_23081,
			sizeof(omegac2tele_ana_gain_table_23081));
		}
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		switch(*feature_data) {
			case SENSOR_SCENARIO_ID_CUSTOM3:
			case SENSOR_SCENARIO_ID_CUSTOM12:
				*(feature_data + 2) = 16 * BASEGAIN;
				break;
			default:
				*(feature_data + 2) = imgsensor_info.max_gain;
				break;
		}
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
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
		case SENSOR_SCENARIO_ID_CUSTOM11:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.custom11.pclk;
				break;
		case SENSOR_SCENARIO_ID_CUSTOM12:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.custom12.pclk;
				break;
		case SENSOR_SCENARIO_ID_CUSTOM13:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.custom13.pclk;
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
			= 2500000;
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
        	case SENSOR_SCENARIO_ID_CUSTOM11:
            		*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                		= (imgsensor_info.custom11.framelength << 16)
                		+ imgsensor_info.custom11.linelength;
            	break;
        	case SENSOR_SCENARIO_ID_CUSTOM12:
            		*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                		= (imgsensor_info.custom12.framelength << 16)
                		+ imgsensor_info.custom12.linelength;
            	break;
        	case SENSOR_SCENARIO_ID_CUSTOM13:
            		*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                		= (imgsensor_info.custom13.framelength << 16)
                		+ imgsensor_info.custom13.linelength;
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
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		/*night_mode(ctx, (BOOL) *feature_data);*/
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT32) * feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_16(ctx, sensor_reg_data->RegAddr,
			sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_16(ctx, sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM */
		/* or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module.*/
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(ctx, feature_return_para_32);
		break;
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = (UINT32)imgsensor_info.module_id;
		*feature_para_len = 4;
		break;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	case SENSOR_FEATURE_GET_SENSOR_SETTING_INFO:
        LOG_INF("SENSOR_FEATURE_GET_SENSOR_SETTING_INFO scenarioId:%d\n",
            (UINT32) *feature_data);
        sensorinfo =
            (struct SENSOR_SETTING_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
        switch (*feature_data) {
            case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
            case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
            case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
            case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
            case SENSOR_SCENARIO_ID_SLIM_VIDEO:
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
            case SENSOR_SCENARIO_ID_CUSTOM11:
            case SENSOR_SCENARIO_ID_CUSTOM12:
                memcpy((void *)sensorinfo,
                (void *)&sensor_setting_info[*feature_data],
                sizeof(struct SENSOR_SETTING_INFO_STRUCT));
                break;
            default:
                sensorinfo->sensor_scenario_usage = UNUSE_MASK;
                break;
        }
        break;
#endif
	case SENSOR_FEATURE_SET_AWB_GAIN:
		omegac2tele_set_awb_gain(ctx, feature_para, feature_para_len);
		break;
	case SENSOR_FEATURE_GET_EEPROM_COMDATA:
		if(*feature_para_len == sizeof(omegac2tele_common_data) && feature_return_para_32 != feature_para_len) {
			memcpy(feature_return_para_32, &omegac2tele_common_data,
				sizeof(omegac2tele_common_data));
		}
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(ctx, (BOOL)*feature_data_16,
			*(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode(ctx, (UINT32)*feature_data);
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN_DATA:
		set_test_pattern_data(ctx, (struct mtk_test_pattern_data *)feature_data);
		break;
	/*for factory mode auto testing*/
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		ctx->current_fps = *feature_data_32;
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO:");
		LOG_INF("scenarioId:%d\n", *feature_data_32);
		wininfo =
			(struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data + 1));
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
        case SENSOR_SCENARIO_ID_CUSTOM11:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[15],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM12:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[16],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM13:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[17],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT16) *feature_data);
		PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data) {
			default :
				memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
					sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT16) *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
			case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			case SENSOR_SCENARIO_ID_CUSTOM1:
			case SENSOR_SCENARIO_ID_CUSTOM2:
			case SENSOR_SCENARIO_ID_CUSTOM5:
			case SENSOR_SCENARIO_ID_CUSTOM6:
			case SENSOR_SCENARIO_ID_CUSTOM7:
			case SENSOR_SCENARIO_ID_CUSTOM8:
			case SENSOR_SCENARIO_ID_CUSTOM9:
			case SENSOR_SCENARIO_ID_CUSTOM11:
			case SENSOR_SCENARIO_ID_CUSTOM12:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
			case SENSOR_SCENARIO_ID_CUSTOM3:
			case SENSOR_SCENARIO_ID_CUSTOM4:
			case SENSOR_SCENARIO_ID_CUSTOM10:
			case SENSOR_SCENARIO_ID_CUSTOM13:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		break;
	case SENSOR_FEATURE_SEAMLESS_SWITCH:
	{
		LOG_INF("SENSOR_FEATURE_SEAMLESS_SWITCH");
		if ((feature_data + 1) != NULL)
			pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		else
			LOG_INF("warning! no ae_ctrl input");

		if (feature_data == NULL) {
			LOG_INF("error! input scenario is null!");
			return ERROR_INVALID_SCENARIO_ID;
		}
		LOG_INF("call seamless_switch");
		seamless_switch(ctx, (*feature_data), pAeCtrls);
	}
		break;
	case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
		if ((feature_data + 1) != NULL)
			pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		else {
			LOG_INF("input pScenarios vector is NULL!\n");
			return ERROR_INVALID_SCENARIO_ID;
		}
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			*pScenarios = SENSOR_SCENARIO_ID_CUSTOM12;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM12:
			*pScenarios = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
			break;
		default:
			*pScenarios = 0xff;
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %d %d\n",
				*feature_data, *pScenarios);
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		/*
			HDR_NONE = 0,
			HDR_RAW = 1,
			HDR_CAMSV = 2,
			HDR_RAW_ZHDR = 9,
			HDR_MultiCAMSV = 10,
			HDR_RAW_STAGGER_2EXP = 0xB,
			HDR_RAW_STAGGER_MIN = HDR_RAW_STAGGER_2EXP,
			HDR_RAW_STAGGER_3EXP = 0xC,
			HDR_RAW_STAGGER_MAX = HDR_RAW_STAGGER_3EXP,
		 */
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0xb;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n",
			*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_EXPOSURE_COUNT_BY_SCENARIO:
		if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM4) {
			*(feature_data + 1) = 2;
		} else {
			*(feature_data + 1) = 1;
		}
	break;
	case SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO:
		if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM8) {
			switch (*(feature_data + 1)) {
			case HDR_RAW_STAGGER_2EXP:
				*(feature_data + 2) = SENSOR_SCENARIO_ID_CUSTOM4;//custom4 was the 2 exp mode for preview mode
			break;
		default:
				break;
			}
		} else if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM4) {
			switch (*(feature_data + 1)) {
			case HDR_NONE:
				*(feature_data + 2) = SENSOR_SCENARIO_ID_CUSTOM8;//normal_video mode for video preview mode
				break;
			default:
				break;
			}
		} else if (*feature_data == SENSOR_SCENARIO_ID_NORMAL_PREVIEW) {
			switch (*(feature_data + 1)) {
			case HDR_RAW_STAGGER_2EXP:
				*(feature_data + 2) = SENSOR_SCENARIO_ID_CUSTOM4;
				break;
			default:
				break;
			}
		}
		LOG_INF("SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO %d %d %d\n",
				(UINT16) *feature_data,
				(UINT16) *(feature_data + 1),
				(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
		if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM4) {
			switch (*(feature_data + 1)) {
			case VC_STAGGER_NE:
				*(feature_data + 2) = 32757;   //need repare
				break;
			case VC_STAGGER_ME:
				*(feature_data + 2) = 32757;
				break;
			case VC_STAGGER_SE:
				*(feature_data + 2) = 32757;
				break;
			default:
				*(feature_data + 2) = 32757;
			break;
		}
		} else {
			*(feature_data + 2) = 0;
		}
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER://for 2EXP
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER, LE=%d, SE=%d\n",
				(UINT32) *feature_data, (UINT32) *(feature_data + 1));
		// implement write shutter for NE/SE
		// hdr_write_tri_shutter(ctx, (UINT32)*feature_data,
		// 		0,
		// 		(UINT32)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_DUAL_GAIN://for 2EXP
		LOG_INF("SENSOR_FEATURE_SET_DUAL_GAIN, LE=%d, SE=%d\n",
				(UINT32)*feature_data, (UINT32)*(feature_data + 1));
		// implement write gain for NE/SE
		// hdr_write_tri_gain(ctx,
		// 		(UINT32)*feature_data,
		// 		0,
		// 		(UINT32)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		ctx->pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(ctx, (UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME:
		set_multi_shutter_frame_length(ctx, (UINT32 *)(*feature_data),
					(UINT16) (*(feature_data + 1)),
					(UINT16) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_4CELL_DATA:
		{
			int type = (kal_uint16)(*feature_data);
			if (type == FOUR_CELL_CAL_TYPE_ALL) {
				LOG_INF("SENSOR_FEATURE_GET_4CELL_DATA type=%d\n", type);
				read_4cell_data((char *)(*(feature_data+1)));
			} else if (type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
				LOG_INF("SENSOR_FEATURE_GET_4CELL_DATA type=%d\n", type);
				read_4cell_data((char *)(*(feature_data+1)));
			} else {
				memset((void *)(*(feature_data+1)), 0, 4);
				LOG_INF("No type %d buffer on this sensor\n", type);
			}
			break;
		}
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(ctx, KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME");
		LOG_INF("shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(ctx, *feature_data);
		streaming_control(ctx, KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
			case SENSOR_SCENARIO_ID_CUSTOM3:
			case SENSOR_SCENARIO_ID_CUSTOM12:
				*feature_return_para_32 = 1240; /*BINNING_NONE*/
			break;
			default:
				*feature_return_para_32 = 1000; /*BINNING_AVERAGED*/
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		*feature_return_para_32 = ctx->current_ae_effective_frame;
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
			case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
				rate =
				imgsensor_info.cap.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
				rate =
				imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
				rate =
				imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
				rate =
				imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
				rate =
					imgsensor_info.pre.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM1:
				rate =
						imgsensor_info.custom1.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM2:
				rate =
						imgsensor_info.custom2.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM3:
				rate =
						imgsensor_info.custom3.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM4:
				rate =
						imgsensor_info.custom4.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM5:
				rate =
						imgsensor_info.custom5.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM6:
				rate =
						imgsensor_info.custom6.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM7:
				rate =
						imgsensor_info.custom7.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM8:
				rate =
						imgsensor_info.custom8.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM9:
				rate =
						imgsensor_info.custom9.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM10:
				rate =
						imgsensor_info.custom10.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM11:
				rate =
						imgsensor_info.custom11.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM12:
				rate =
						imgsensor_info.custom12.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_CUSTOM13:
				rate =
						imgsensor_info.custom13.mipi_pixel_rate;
				break;
			default:
					rate = 0;
					break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_PRELOAD_EEPROM_DATA:
		/*get eeprom preloader data*/
		*feature_return_para_32 = ctx->is_read_four_cell;
		*feature_para_len = 4;
		if (ctx->is_read_four_cell != 1)
			read_four_cell_from_eeprom(ctx, NULL);
		break;
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		*(feature_data + 1) = 0; /* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_GET_UNIQUE_SENSORID:
		omegac2tele_get_unique_sensorid(ctx, feature_para, feature_para_len);
		break;
	case SENSOR_FEATURE_GET_EEPROM_PROBE_STATE:
		omegac2tele_get_eeprom_probe_state(ctx, feature_para, feature_para_len);
		break;
	case SENSOR_FEATURE_GET_CLOUD_OTP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CLOUD_OTP_INFO otp_type:%d\n",
			(UINT32) *feature_data);
		cloudinfo =
			(struct SENSOR_OTP_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data) {
			case OPLUS_CAM_CAL_DATA_MODULE_VERSION:
			case OPLUS_CAM_CAL_DATA_PART_NUMBER:
			case OPLUS_CAM_CAL_DATA_SHADING_TABLE:
			case OPLUS_CAM_CAL_DATA_3A_GAIN:
			case OPLUS_CAM_CAL_DATA_PDAF:
			case OPLUS_CAM_CAL_DATA_CAMERA_INFO:
			case OPLUS_CAM_CAL_DATA_DUMP:
			case OPLUS_CAM_CAL_DATA_LENS_ID:
			case OPLUS_CAM_CAL_DATA_QSC:
			case OPLUS_CAM_CAL_DATA_LRC:
			case OPLUS_CAM_CAL_DATA_ALL:
				memcpy((void *)cloudinfo,
					(void *)&cloud_otp_info[*feature_data],
					sizeof(struct SENSOR_OTP_INFO_STRUCT));
				break;
			default:
				break;
		}
		break;
	default:
		break;
	}
	return ERROR_NONE;
}	/*	feature_control(ctx)  */

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {//4096*3072
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096, //uint:Byte, width*10/8
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {//4096*3072
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096, //uint:Byte, width*10/8
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {//4096*2304
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096, //uint:Byte, width*10/8
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {//4096*2304
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096, //uint:Byte, width*10/8
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {//2048*1152
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2048, //uint:Byte, width*10/8
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2048, //uint:Byte, width*10/8
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2048, //uint:Byte, width*10/8
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {//8192*6144
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 8192,
			.vsize = 6144,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2048, //uint:Byte, width*10/8
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2048, //uint:Byte, width*10/8
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1836,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 3264, //uint:Byte, width*10/8
			.vsize = 456,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1836,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 3264, //uint:Byte, width*10/8
			.vsize = 456,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};


static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 3264, //uint:Byte, width*10/8
			.vsize = 612,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus11[] = {
	{
		.bus.csi2 = {//4096*3072
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096, //uint:Byte, width*10/8
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus12[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2048, //uint:Byte, width*10/8
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus13[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
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
    case SENSOR_SCENARIO_ID_CUSTOM11:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_cus11);
        memcpy(fd->entry, frame_desc_cus11, sizeof(frame_desc_cus11));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM12:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_cus12);
        memcpy(fd->entry, frame_desc_cus12, sizeof(frame_desc_cus12));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM13:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_cus13);
        memcpy(fd->entry, frame_desc_cus13, sizeof(frame_desc_cus13));
        break;
	default:
		return -1;
	}
	return 0;
}
#endif

static const struct subdrv_ctx defctx = {

	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 32,
	.ana_gain_min = BASEGAIN,
	.ana_gain_step = 32,
	.exposure_def = 0x3D0,
	.exposure_max = 0xfffc - 3,
	.exposure_min = 3,
	.exposure_step = 1,
	.frame_time_delay_frame = 2,
	.margin = 3,
	.max_frame_length = 0xfffc,

	.mirror = IMAGE_NORMAL, //mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,	/*current shutter*/
	.gain = BASEGAIN * 4,			/*current gain*/
	.dummy_pixel = 0,		/*current dummypixel*/
	.dummy_line = 0,		/*current dummyline*/
	.current_fps = 0,
	/*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_ae_effective_frame = 2,

	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x5A,
};

static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
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
		csi_param->dphy_trail = 85;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		csi_param->dphy_trail = 81;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		csi_param->dphy_trail = 78;
		break;
	default:
		csi_param->dphy_trail = 82;
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
	.get_csi_param = get_csi_param,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
    {HW_ID_MCLK, 24, 2},
    {HW_ID_RST, 0, 1},
    {HW_ID_DOVDD, 1800000, 1},
    {HW_ID_DOVDD, 0, 3},
    {HW_ID_DOVDD, 1800000, 1},
    {HW_ID_DOVDD, 0, 3},
    {HW_ID_DOVDD, 1800000, 1},
    {HW_ID_AVDD, 2204000, 1},
    {HW_ID_DVDD, 1000000, 1},
    {HW_ID_AFVDD, 2800000, 1},
    {HW_ID_RST, 1, 2},
    {HW_ID_MCLK_DRIVING_CURRENT, 4, 7}
};

static struct subdrv_pw_seq_entry pw_off_seq[] = {
    {HW_ID_MCLK, 24, 2},
    {HW_ID_RST, 0, 1},
    {HW_ID_DOVDD, 1800000, 1},
    {HW_ID_AVDD, 2204000, 1},
    {HW_ID_DVDD, 1000000, 1},
    {HW_ID_AFVDD, 2800000, 1},
    {HW_ID_RST, 1, 2},
    {HW_ID_MCLK_DRIVING_CURRENT, 4, 7}
};

const struct subdrv_entry omegac2tele_mipi_raw_23081_entry = {
	.name = "omegac2tele_mipi_raw_23081",
	.id = OMEGAC2TELE_SENSOR_ID_23081,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.pw_off_seq = pw_off_seq,
	.pw_off_seq_cnt = ARRAY_SIZE(pw_off_seq),
	.ops = &ops,
};
