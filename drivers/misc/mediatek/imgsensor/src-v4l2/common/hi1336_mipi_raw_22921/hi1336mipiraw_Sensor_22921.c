	// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 hi1336_mipiraw_Sensor.c
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
#include "hi1336_eeprom_22921.h"
#include "hi1336mipiraw_Sensor_22921.h"
#include "hi1336_ana_gain_table_22921.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"
#include "adaptor.h"

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define write_cmos_sensor(...) subdrv_i2c_wr_u16(__VA_ARGS__)
#define table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u16(__VA_ARGS__)

#define PFX "HI1336_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define FPT_PDAF_SUPPORT 1
static DEFINE_SPINLOCK(imgsensor_drv_lock);
static int g_pattern_mode_status = 0;
static void sensor_init(struct subdrv_ctx *ctx);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI1336_SENSOR_ID_22921,
	.checksum_value = 0x4f1b1d5e, // 0x6d01485c Auto Test Mode ..
	.pre = {
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 3330,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4160,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 571200000,
	},
	.cap = {
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 3330,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4160,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 571200000,
	},
	.normal_video = {
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 3330,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4160,
		.grabwindow_height = 2340,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 571200000,
	},
	.hs_video = {
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 1665,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
		.mipi_pixel_rate = 285600000,
	},
	.slim_video = {
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 832,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
		.mipi_pixel_rate = 190400000,
	},
	.margin = 4,
	.min_shutter = 4,
	.min_gain = BASEGAIN,
	.max_gain = 16 * BASEGAIN,
	.min_gain_iso = 100,
	.exp_step = 1,
	.gain_step = 64,
	.gain_type = 3,
	.max_frame_length = 0xffffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0, //1, support; 0,not support
	.ihdr_le_firstline = 0, //1,le first ; 0, se first
	.temperature_support = 0,/* 1, support; 0,not support */
	.sensor_mode_num = 5, //support sensor mode num
	.frame_time_delay_frame = 3, //The delay frame of setting frame length
	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x40, 0xff},
	.i2c_speed = 400,
};

// Sensor output window information
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
	{ 4224, 3136,   0, 6,   4224, 3124,  4224, 3124,  32,  2, 4160, 3120, 0, 0, 4160, 3120}, // preview (4160 x 3120)
	{ 4224, 3136,   0, 6,   4224, 3124,  4224, 3124,  32,  2, 4160, 3120, 0, 0, 4160, 3120}, // capture (4160 x 3120)
	{ 4224, 3136,   0, 396, 4224, 2344,  4224, 2344,  32,  2, 4160, 2340, 0, 0, 4160, 2340}, // VIDEO (4160 x 2340)
	{ 4224, 3136,   0, 484, 4224, 2168,  2112, 1084,  96,  2, 1920, 1080, 0, 0, 1920, 1080}, // hight speed video (1920 x 1080)
	{ 4224, 3136,   0, 482, 4224, 2172,  1408,  724,  64,  2, 1280,  720, 0, 0, 1280,  720}, // slim video (1280 x 720)
};

#if FPT_PDAF_SUPPORT
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[]=
{
	/* preview mode setting */
	{
		0x02, //VC_Num
		0x0a, //VC_PixelNum
		0x00, //ModeSelect    /* 0:auto 1:direct */
		0x00, //EXPO_Ratio    /* 1/1, 1/2, 1/4, 1/8 */
		0x00, //0DValue       /* 0D Value */
		0x00, //RG_STATSMODE  /* STATS divistion mode 0:16x16  1:8x8  2:4x4  3:1x1 */
		0x00, 0x2B, 0x1040, 0x0C30,   // VC0 Maybe image data?
		0x00, 0x00, 0x0000, 0x0000,   // VC1 MVHDR
		0x01, 0x2B, 0x0100, 0x0300,   // VC2 PDAF
		0x00, 0x00, 0x0000, 0x0000
	}, // VC3
	/* Capture mode setting */
	{
		0x02, //VC_Num
		0x0a, //VC_PixelNum
		0x00, //ModeSelect    /* 0:auto 1:direct */
		0x00, //EXPO_Ratio    /* 1/1, 1/2, 1/4, 1/8 */
		0x00, //0DValue       /* 0D Value */
		0x00, //RG_STATSMODE  /* STATS divistion mode 0:16x16  1:8x8  2:4x4  3:1x1 */
		0x00, 0x2B, 0x1040, 0x0C30,   // VC0 Maybe image data?
		0x00, 0x00, 0x0000, 0x0000,   // VC1 MVHDR
		0x01, 0x2B, 0x0100, 0x0300,   // VC2 PDAF
		0x00, 0x00, 0x0000, 0x0000
	}, // VC3
	/* Video mode setting */
	{
		0x02, //VC_Num
		0x0a, //VC_PixelNum
		0x00, //ModeSelect    /* 0:auto 1:direct */
		0x00, //EXPO_Ratio    /* 1/1, 1/2, 1/4, 1/8 */
		0x00, //0DValue       /* 0D Value */
		0x00, //RG_STATSMODE  /* STATS divistion mode 0:16x16  1:8x8  2:4x4  3:1x1 */
		0x00, 0x2B, 0x1040, 0x0924,   // VC0 Maybe image data?
		0x00, 0x00, 0x0000, 0x0000,   // VC1 MVHDR
		0x01, 0x2B, 0x0100, 0x0248,   // VC2 PDAF
		0x00, 0x00, 0x0000, 0x0000
	}, // VC3
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX  = 32,
	.i4OffsetY  = 24,
	.i4PitchX   = 32,
	.i4PitchY   = 32,
	.i4PairNum  = 8,
	.i4SubBlkW  = 16,
	.i4SubBlkH  = 8,
	.i4BlockNumX = 128,
	.i4BlockNumY = 96,
	.iMirrorFlip = IMAGE_NORMAL,
	.i4PosR = { {37,24}, {53,24}, {37,36}, {53,36}, {37,40}, {53,40}, {37,52}, {53,52} },
	.i4PosL = { {37,28}, {53,28}, {37,32}, {53,32}, {37,44}, {53,44}, {37,48}, {53,48} },
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_normal_video =
{
	.i4OffsetX  = 32,
	.i4OffsetY  = 2,
	.i4PitchX   = 32,
	.i4PitchY   = 32,
	.i4PairNum  = 8,
	.i4SubBlkW  = 16,
	.i4SubBlkH  = 8,
	.i4BlockNumX = 128,
	.i4BlockNumY = 73,
	.iMirrorFlip = IMAGE_NORMAL,
	.i4PosR = { {37,2}, {53,2}, {37,14}, {53,14}, {37,18}, {53,18}, {37,30}, {53,30} },
	.i4PosL = { {37,6}, {53,6}, {37,10}, {53,10}, {37,22}, {53,22}, {37,26}, {53,26} },
};
#endif

static void set_dummy(struct subdrv_ctx *ctx)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	write_cmos_sensor(ctx, 0x020e, ctx->frame_length & 0xFFFF);
	write_cmos_sensor(ctx, 0x0206, (ctx->line_length & 0xFFFF)/4);
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = ctx->frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	spin_lock(&imgsensor_drv_lock);
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
	spin_unlock(&imgsensor_drv_lock);
	set_dummy(ctx);
}

static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk * 10 / (ctx->line_length * ctx->frame_length);
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146,0);
		else{
			write_cmos_sensor(ctx, 0x020e, ctx->frame_length);
		}
	}
	else{
		// Extend frame length
		write_cmos_sensor(ctx, 0x020e, ctx->frame_length);
	}

	// Update Shutter
	write_cmos_sensor(ctx, 0x020A, shutter & 0xFFFF);
	write_cmos_sensor(ctx, 0x020C, (shutter & 0xFFFF0000) >> 16);

	LOG_INF("shutter =%d, framelength =%d", shutter,ctx->frame_length);
}

/*************************************************************************
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
 *************************************************************************/
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	LOG_INF("set_shutter");

	ctx->shutter = shutter;

	write_shutter(ctx, shutter);
}

/*************************************************************************
 * FUNCTION
 *  set_shutter_frame_length
 *
 * DESCRIPTION
 *  for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(struct subdrv_ctx *ctx,kal_uint16 shutter,
	kal_uint16 frame_length, kal_bool auto_extend_en)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	ctx->shutter = shutter;
	spin_lock(&imgsensor_drv_lock);
	// Change frame time
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;

	ctx->frame_length = ctx->frame_length + dummy_line;

	if (shutter > ctx->frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter)
		? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk * 10 / (ctx->line_length * ctx->frame_length);
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else {
			// Extend frame length
			write_cmos_sensor(ctx, 0x020E, ctx->frame_length);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(ctx, 0x020E, ctx->frame_length);
	}

	/* Update Shutter */
	write_cmos_sensor(ctx, 0x020A, shutter & 0xFFFF);
	write_cmos_sensor(ctx, 0x020C, (shutter & 0xFFFF0000) >> 16 );

	LOG_INF("frame_length = %d , shutter = %d \n", ctx->frame_length, shutter);
}

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint16 reg_gain = 0x0;
	reg_gain =  gain *16 /BASEGAIN -  1 * 16;
	return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X    */
	/* [4:9] = M meams M X         */
	/* Total gain = M + N /16 X   */

	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting : %d\n",gain);

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

	reg_gain = gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_8(ctx, 0x0213, reg_gain&0xff);/* max = 0xf0*/
	return gain;
}

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable) {
		write_cmos_sensor(ctx, 0x0b00, 0x0100);
	} else {
		write_cmos_sensor(ctx, 0x0b00, 0x0000);
	}

	mdelay(10);
	return ERROR_NONE;
}

static void sensor_init(struct subdrv_ctx *ctx)
{
	LOG_INF("%s E\n", __func__);

	table_write_cmos_sensor(ctx, init_setting_array,
		sizeof(init_setting_array) / sizeof(kal_uint16));
}

static void preview_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("%s E\n", __func__);

	table_write_cmos_sensor(ctx, preview_setting_array,
		sizeof(preview_setting_array) / sizeof(kal_uint16));
}

static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_INF("%s E! currefps:%d\n", __func__, currefps);

	table_write_cmos_sensor(ctx, capture_setting_array,
		sizeof(capture_setting_array) / sizeof(kal_uint16));
}

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_INF("%s currefps:%d", __func__, currefps);

	table_write_cmos_sensor(ctx, normal_video_setting_array,
		sizeof(normal_video_setting_array) / sizeof(kal_uint16));
}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("%s", __func__);

	table_write_cmos_sensor(ctx, hs_video_setting_array,
		sizeof(hs_video_setting_array) / sizeof(kal_uint16));
}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("%s", __func__);
	table_write_cmos_sensor(ctx, slim_video_setting_array,
		sizeof(slim_video_setting_array) / sizeof(kal_uint16));
}

static BYTE hi1336_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static void read_module_data(struct subdrv_ctx *ctx)
{
	read_hi1336_eeprom_info_22921(ctx, EEPROM_META_MODULE_ID,
		&(hi1336_common_data[0]), 1);
	imgsensor_info.module_id = hi1336_common_data[0];
	read_hi1336_eeprom_info_22921(ctx, EEPROM_META_SENSOR_ID,
		&(hi1336_common_data[1]), 1);
	read_hi1336_eeprom_info_22921(ctx, EEPROM_META_LENS_ID,
		&(hi1336_common_data[2]), 1);
	read_hi1336_eeprom_info_22921(ctx, EEPROM_META_VCM_ID,
		&(hi1336_common_data[3]), 1);

	LOG_INF("In %s: cam data: %02x %02x %02x %02x\n", __func__,
		hi1336_common_data[0], hi1336_common_data[1],
		hi1336_common_data[2], hi1336_common_data[3]);
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
 *************************************************************************/
static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	static bool first_read = KAL_TRUE;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_8(ctx, 0x0716) << 8) | read_cmos_sensor_8(ctx, 0x0717));
			LOG_INF("i2c read sensor_id addr: 0x%x, sensor id: 0x%x imgsensor_info.sensor_id %x\n", ctx->i2c_write_id, *sensor_id,imgsensor_info.sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",ctx->i2c_write_id, *sensor_id);
				if(first_read){
					read_module_data(ctx);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			else{
				LOG_INF("Read sensor id fail, id: 0x%x\n",ctx->i2c_write_id);
				retry--;
			}
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
	/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
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
 *************************************************************************/
static int open(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;
	LOG_INF("%s", __func__);

	// sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor_8(ctx, 0x0716) << 8) | read_cmos_sensor_8(ctx, 0x0717));
			LOG_INF("i2c read sensor_id addr: 0x%x, sensor id: 0x%x imgsensor_info.sensor_id %x\n", ctx->i2c_write_id, sensor_id,imgsensor_info.sensor_id);
			if (sensor_id == imgsensor_info.sensor_id) {
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
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init(ctx);

	spin_lock(&imgsensor_drv_lock);
	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->shutter = 0x100;
	ctx->gain = 0xe0;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	ctx->ihdr_mode = 0;
	ctx->test_pattern = KAL_FALSE;
	ctx->current_fps = imgsensor_info.pre.max_framerate;

	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

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
	LOG_INF("E\n");

	return ERROR_NONE;
}


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
static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting(ctx);

	return ERROR_NONE;
}

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
static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;

	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(ctx, ctx->current_fps);

	return ERROR_NONE;
}

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(ctx, ctx->current_fps);

	return ERROR_NONE;
}

static kal_uint32 hs_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting(ctx);

	return ERROR_NONE;
}

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting(ctx);

	return ERROR_NONE;
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
}

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	// not use
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	// inverse with datasheet
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; // not use
	sensor_info->SensorResetActiveHigh = FALSE; // not use
	sensor_info->SensorResetDelayCount = 5; // not use

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;

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
	sensor_info->SensorMasterClockSwitch = 0; // not use
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	// The frame of setting shutter default 0 for TG int
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	// The frame of setting sensor gain
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	// change pdaf support mode to pdaf VC mode
#if FPT_PDAF_SUPPORT
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV;
#else
	sensor_info->PDAF_Support = 0;
#endif
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; // not use
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; // not use
	sensor_info->SensorPixelClockCount = 3; // not use
	sensor_info->SensorDataLatchCount = 2; // not use
	sensor_info->SensorWidthSampling = 0; // 0 is default 1x
	sensor_info->SensorHightSampling = 0; // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;
	return ERROR_NONE;
}


static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	ctx->current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);

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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	/* //LOG_INF("framerate = %d\n ", framerate); */
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 296;
	else if ((framerate == 150) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 146;
	else
		ctx->current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(ctx, ctx->current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx,
	kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);

	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		ctx->autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		ctx->autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,	MUINT32 framerate)
{
	kal_uint32 frame_length;
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		ctx->frame_length =
			imgsensor_info.pre.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk
			/ framerate * 10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		ctx->dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
			? (frame_length - imgsensor_info.normal_video.  framelength) : 0;

		ctx->frame_length =
			imgsensor_info.normal_video.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		if (ctx->current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
			, framerate, imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10
			/ imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		ctx->dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			? (frame_length - imgsensor_info.cap.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.cap.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk
			/ framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		ctx->dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			? (frame_length - imgsensor_info.hs_video.  framelength) : 0;

		ctx->frame_length =
			imgsensor_info.hs_video.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk
			/ framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		ctx->dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.  framelength) : 0;

		ctx->frame_length =
			imgsensor_info.slim_video.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;
	default: /* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		ctx->frame_length =
			imgsensor_info.pre.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_INF("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
		scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
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
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_bool enable)
{
    LOG_INF("enable: %d g_pattern_mode_status: %d \n", enable,g_pattern_mode_status);
    if (enable) {
        write_cmos_sensor(ctx, 0x0b04, 0x037d);
        write_cmos_sensor(ctx, 0x0C0A, 0x0100);
    } else if (g_pattern_mode_status && !enable){
        write_cmos_sensor(ctx, 0x0b00, 0x0000);  //stream off
        write_cmos_sensor(ctx, 0x0b04, 0x037e);
        write_cmos_sensor(ctx, 0x0C0A, 0x0000);
        write_cmos_sensor(ctx, 0x0b00, 0x0100);  //stream on
    }else{
        write_cmos_sensor(ctx, 0x0b04, 0x037e);
        write_cmos_sensor(ctx, 0x0C0A, 0x0000);
    }
    g_pattern_mode_status = enable;
    spin_lock(&imgsensor_drv_lock);
    ctx->test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

#if FPT_PDAF_SUPPORT
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
#endif

	/*LOG_INF("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_OUTPUT_FORMAT_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(hi1336_ana_gain_table);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)hi1336_ana_gain_table,
			sizeof(hi1336_ana_gain_table));
		}
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
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
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
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
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
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT32) * feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		break;
	case SENSOR_FEATURE_GET_REGISTER:
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
	case SENSOR_FEATURE_GET_MODULE_INFO:
		*feature_return_para_32 = (hi1336_common_data[2] << 8) |
			(hi1336_common_data[3] & 0xFF);
		*feature_para_len = 4;
		LOG_INF("hi1336 GET_MODULE_CamInfo:%d 0x%02x\n", *feature_para_len, *feature_data_32);
		break;
	case SENSOR_FEATURE_GET_MODULE_SN:
		break;
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = (UINT32)imgsensor_info.module_id;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_EEPROM_COMDATA:
		memcpy(feature_return_para_32, hi1336_common_data,
			OPLUS_CAMERA_COMMON_DATA_LENGTH);
		*feature_para_len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
		break;
	case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
		break;
	case SENSOR_FEATURE_GET_DISTORTIONPARAMS:
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(ctx, (BOOL) (*feature_data_16),
			*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM) *feature_data, *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
				(MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode(ctx, (BOOL) (*feature_data));
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		ctx->current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		ctx->ihdr_mode = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		wininfo =
			(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
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
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_LSC_TBL:
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(ctx,(UINT16) (*feature_data),
			(UINT16) (*(feature_data + 1)),
			(BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/* 1, if driver support new sw frame sync
		* set_shutter_frame_length() support third para auto_extend_en */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
			streaming_control(ctx, KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(ctx, *feature_data);
		streaming_control(ctx, KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*feature_return_para_32 = 1; /* NON */
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
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
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
#if FPT_PDAF_SUPPORT
	case SENSOR_FEATURE_GET_PDAF_DATA:
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)
			(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
			case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
				memcpy((void*)PDAFinfo, (void*)&imgsensor_pd_info,
					sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
				memcpy((void*)PDAFinfo, (void*)&imgsensor_pd_info_normal_video,
					sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			default:
				break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n",
			*feature_data);
		switch (*feature_data) {
			case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
				*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 1;
				break;
			case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
				*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 1; // type2 - VC enable
				break;
			case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
				*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 1;
				break;
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
				*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
		}
		break;
		case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d", (*feature_para_len));
			break;
		case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
			LOG_INF("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d", (*feature_para_len));
			break;
		case SENSOR_FEATURE_SET_PDAF:
			LOG_INF("PDAF mode :%d\n", *feature_data_16);
			ctx->pdaf_mode = *feature_data_16;
			break;
		case SENSOR_FEATURE_GET_VC_INFO:
			pvcinfo = (struct SENSOR_VC_INFO_STRUCT*)(uintptr_t)(*(feature_data + 1));
			switch (*feature_data_32) {
			case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
				memcpy((void*)pvcinfo, (void*)&SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
				break;
			case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
				memcpy((void*)pvcinfo, (void*)&SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
				break;
			case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
				memcpy((void*)pvcinfo, (void*)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
				break;
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			default:
				LOG_INF("error: get wrong vc_INFO id = %d", *feature_data_32);
				break;
			}
			break;
#endif
	default:
		break;
	}

	return ERROR_NONE;
}

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4160,
			.vsize = 3120,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x100,
			.vsize = 0x300,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4160,
			.vsize = 3120,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x100,
			.vsize = 0x300,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4160,
			.vsize = 2340,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x100,
			.vsize = 0x248,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1920,
			.vsize = 1080,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1280,
			.vsize = 720,
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
	default:
		return -1;
	}
	return 0;
}
#endif
static kal_uint32 get_sensor_temperature(struct subdrv_ctx *ctx)
{
	return 20;
}

static const struct subdrv_ctx defctx = {
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_min = BASEGAIN,
	.ana_gain_step = 4,
	.exposure_def = 0x3D0,
	.exposure_max = 0xffff,
	.exposure_min = 4,
	.exposure_step = 2,
	.frame_time_delay_frame = 3,
	.margin = 22,
	.max_frame_length = 0xffff,
	.mirror = IMAGE_NORMAL,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	* INIT, Preview, Capture, Video,High Speed Video, Slim Video*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0, /* current shutter */
	.gain = BASEGAIN * 4, /* current gain */
	.dummy_pixel = 0, /* current dummypixel */
	.dummy_line = 0, /* current dummyline */
	/* full size current fps : 24fps for PIP,* 30fps for Normal or ZSD */
	.current_fps = 300,
	/* auto flicker enable: KAL_FALSE for disable auto flicker,
	 * KAL_TRUE for enable auto flicker*/
	.autoflicker_en = KAL_FALSE,
	/* test pattern mode or not.
	 * KAL_FALSE for in test pattern mode,
	 * KAL_TRUE for normal output */
	.test_pattern = KAL_FALSE,
	/* current scenario id */
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	/* sensor need support LE, SE with HDR feature */
	.ihdr_mode = KAL_FALSE,
	.i2c_write_id = 0x40, /* record current sensor's i2c write id */
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
    csi_param->legacy_phy = 0;
    csi_param->not_fixed_trail_settle = 0;
    csi_param->dphy_trail = 74;
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
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DVDD, 1100000, 1},
	{HW_ID_MCLK, 24, 0},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_AFVDD, 2800000, 3},
	{HW_ID_RST, 1, 3},
};

const struct subdrv_entry hi1336_mipi_raw_22921_entry = {
	.name = "hi1336_mipi_raw_22921",
	.id = HI1336_SENSOR_ID_22921,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};
