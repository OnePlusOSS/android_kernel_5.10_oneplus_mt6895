 /*
 *
 * Filename:
 * ---------
 *     gc02m1mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *-----------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 */

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
#include "kd_imgsensor_errcode.h"

#include "gc02m1mipiraw_Sensor_21143.h"
#include "gc02m1_eeprom_21143.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define gc02m1_read_cmos_sensor_8_8(...) gc02m1_subdrv_i2c_rd_u8_u8(__VA_ARGS__)
#define gc02m1_write_cmos_sensor_8_8(...) gc02m1_subdrv_i2c_wr_u8_u8(__VA_ARGS__)
#define gc02m1_table_write_cmos_sensor_8(...) gc02m1_subdrv_i2c_wr_regs_u8(__VA_ARGS__)

/************************** Modify Following Strings for Debug **************************/
#define PFX "gc02m1_camera_sensor"
#define LOG_1 LOG_INF("GC02M1, MIPI 1LANE\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)

#define MULTI_WRITE    1
#define OTP_SIZE    0x2000
static kal_uint8 otp_data[OTP_SIZE] = {0};

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = GC02M1_SENSOR_ID_21143,
	.checksum_value = 0xf7375923,
	.pre = {
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 67200000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 67200000,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 67200000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 67200000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 67200000,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 67200000,
		.max_framerate = 300,
	},

	.min_gain = BASEGAIN * 1,
	.max_gain = BASEGAIN * 12,
	.min_gain_iso = 100,
	.gain_step = 1,
	.gain_type = 4,
	.margin = 16,
	.min_shutter = 4,
	.max_frame_length = 0x3fff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 5,

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_1_LANE,
	.i2c_addr_table = {0x6e, 0x6f, 0xff},
	.i2c_speed = 400,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, /* Preview */
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, /* capture */
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, /* video */
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, /* HS video */
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}  /* slim video */
};

static void set_dummy(struct subdrv_ctx *ctx)
{
    LOG_INF("set_dummy() Enter\n");
	gc02m1_write_cmos_sensor_8_8(ctx, 0xfe, 0x00);
	gc02m1_write_cmos_sensor_8_8(ctx, 0x41, (ctx->frame_length >> 8) & 0x3f);
	gc02m1_write_cmos_sensor_8_8(ctx, 0x42, ctx->frame_length & 0xff);
    LOG_INF("set_dummy() out\n");
}

static kal_uint32 return_sensor_id(struct subdrv_ctx *ctx)
{
	return ((gc02m1_read_cmos_sensor_8_8(ctx, 0xf0) << 8) | gc02m1_read_cmos_sensor_8_8(ctx, 0xf1));
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = ctx->frame_length;

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;

	ctx->frame_length = (frame_length > ctx->min_frame_length)
		? frame_length : ctx->min_frame_length;
	ctx->dummy_line = ctx->frame_length - ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length) {
		ctx->frame_length = imgsensor_info.max_frame_length;
		ctx->dummy_line = ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;
	set_dummy(ctx);
}

static void set_shutter(struct subdrv_ctx *ctx, kal_uint16 shutter)
{    kal_uint16 realtime_fps = 0;
    /* kal_uint32 frame_length = 0; */
    ctx->shutter = shutter;

	/* if shutter bigger than frame_length, should extend frame length first */
	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;

	if (ctx->autoflicker_en) {
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else
			set_max_framerate(ctx, realtime_fps, 0);
	} else
		set_max_framerate(ctx, realtime_fps, 0);
	gc02m1_write_cmos_sensor_8_8(ctx, 0xfe, 0x00);
	gc02m1_write_cmos_sensor_8_8(ctx, 0x03, (shutter >> 8) & 0x3f);
	gc02m1_write_cmos_sensor_8_8(ctx, 0x04, shutter  & 0xff);
	LOG_INF("shutter = %d, framelength = %d\n", shutter, ctx->frame_length);
}

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint16 gain)
{
	kal_uint16 reg_gain = gain;

	if (reg_gain < GC02M1_SENSOR_GAIN_BASE)
		reg_gain = GC02M1_SENSOR_GAIN_BASE;
	else if (reg_gain > GC02M1_SENSOR_GAIN_MAX)
		reg_gain = GC02M1_SENSOR_GAIN_MAX;

	return (kal_uint16)reg_gain;
}

static kal_uint16 set_gain(struct subdrv_ctx *ctx, kal_uint16 gain)
{
	kal_uint16 reg_gain;
	kal_uint32 temp_gain;
	kal_int16 gain_index;
	kal_uint16 GC02M1_AGC_Param[GC02M1_SENSOR_GAIN_MAX_VALID_INDEX][2] = {
		{  1024,  0 },
		{  1536,  1 },
		{  2035,  2 },
		{  2519,  3 },
		{  3165,  4 },
		{  3626,  5 },
		{  4147,  6 },
		{  4593,  7 },
		{  5095,  8 },
		{  5697,  9 },
		{  6270, 10 },
		{  6714, 11 },
		{  7210, 12 },
		{  7686, 13 },
		{  8214, 14 },
		{ 10337, 15 },
	};

	reg_gain = gain2reg(ctx, gain);

	for (gain_index = GC02M1_SENSOR_GAIN_MAX_VALID_INDEX - 1; gain_index >= 0; gain_index--)
		if (reg_gain >= GC02M1_AGC_Param[gain_index][0])
			break;
    LOG_INF("set_gain() Enter\n");
	gc02m1_write_cmos_sensor_8_8(ctx, 0xfe, 0x00);
	gc02m1_write_cmos_sensor_8_8(ctx, 0xb6, GC02M1_AGC_Param[gain_index][1]);
	temp_gain = reg_gain * GC02M1_SENSOR_DGAIN_BASE / GC02M1_AGC_Param[gain_index][0];
	gc02m1_write_cmos_sensor_8_8(ctx, 0xb1, (temp_gain >> 8) & 0x1f);
	gc02m1_write_cmos_sensor_8_8(ctx, 0xb2, temp_gain & 0xff);
	LOG_INF("GC02M1_AGC_Param[gain_index][1] = 0x%x, temp_gain = 0x%x, reg_gain = %d\n",
		GC02M1_AGC_Param[gain_index][1], temp_gain, reg_gain);

	return reg_gain;
}

static void ihdr_write_shutter_gain(struct subdrv_ctx *ctx, kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}

/*static void set_mirror_flip(kal_uint8 image_mirror)
*{
*	LOG_INF("image_mirror = %d\n", image_mirror);
*}
*/

static void night_mode(struct subdrv_ctx *ctx, kal_bool enable)
{
	/* No Need to implement this function */
}

kal_uint16 addr_data_pair_init_gc02m1_21143[] = {
	/*system*/
	0xfc, 0x01,
	0xf4, 0x41,
	0xf5, 0xc0,
	0xf6, 0x44,
	0xf8, 0x38,
	0xf9, 0x82,
	0xfa, 0x00,
	0xfd, 0x80,
	0xfc, 0x81,
	0xfe, 0x03,
	0x01, 0x0b,
	0xf7, 0x01,
	0xfc, 0x80,
	0xfc, 0x80,
	0xfc, 0x80,
	0xfc, 0x8e,

	/*CISCTL*/
	0xfe, 0x00,
	0x87, 0x09,
	0xee, 0x72,
	0xfe, 0x01,
	0x8c, 0x90,
	0xfe, 0x00,
	0x90, 0x00,
	0x03, 0x04,
	0x04, 0x7d,
	0x41, 0x04,
	0x42, 0xf4,
	0x05, 0x04,
	0x06, 0x48,
	0x07, 0x00,
	0x08, 0x18,
	0x9d, 0x18,
	0x09, 0x00,
	0x0a, 0x02,
	0x0d, 0x04,
	0x0e, 0xbc,
	0x17, 0x83,
	0x19, 0x04,
	0x24, 0x00,
	0x56, 0x20,
	0x5b, 0x00,
	0x5e, 0x01,

	/*analog Register width*/
	0x21, 0x3c,
	0x44, 0x20,
	0xcc, 0x01,

	/*analog mode*/
	0x1a, 0x04,
	0x1f, 0x11,
	0x27, 0x30,
	0x2b, 0x00,
	0x33, 0x00,
	0x53, 0x90,
	0xe6, 0x50,

	/*analog voltage*/
	0x39, 0x07,
	0x43, 0x04,
	0x46, 0x2a,
	0x7c, 0xa0,
	0xd0, 0xbe,
	0xd1, 0x60,
	0xd2, 0x40,
	0xd3, 0xf3,
	0xde, 0x1d,

	/*analog current*/
	0xcd, 0x05,
	0xce, 0x6f,

	/*CISCTL RESET*/
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,
	0xfe, 0x04,
	0xe0, 0x01,
	0xfe, 0x00,

	/*ISP*/
	0xfe, 0x01,
	0x53, 0x44,
	0x87, 0x53,
	0x89, 0x03,

	/*Gain*/
	0xfe, 0x00,
	0xb0, 0x74,
	0xb1, 0x04,
	0xb2, 0x00,
	0xb6, 0x00,
	0xfe, 0x04,
	0xd8, 0x00,
	0xc0, 0x40,
	0xc0, 0x00,
	0xc0, 0x00,
	0xc0, 0x00,
	0xc0, 0x60,
	0xc0, 0x00,
	0xc0, 0xc0,
	0xc0, 0x2a,
	0xc0, 0x80,
	0xc0, 0x00,
	0xc0, 0x00,
	0xc0, 0x40,
	0xc0, 0xa0,
	0xc0, 0x00,
	0xc0, 0x90,
	0xc0, 0x19,
	0xc0, 0xc0,
	0xc0, 0x00,
	0xc0, 0xD0,
	0xc0, 0x2F,
	0xc0, 0xe0,
	0xc0, 0x00,
	0xc0, 0x90,
	0xc0, 0x39,
	0xc0, 0x00,
	0xc0, 0x01,
	0xc0, 0x20,
	0xc0, 0x04,
	0xc0, 0x20,
	0xc0, 0x01,
	0xc0, 0xe0,
	0xc0, 0x0f,
	0xc0, 0x40,
	0xc0, 0x01,
	0xc0, 0xe0,
	0xc0, 0x1a,
	0xc0, 0x60,
	0xc0, 0x01,
	0xc0, 0x20,
	0xc0, 0x25,
	0xc0, 0x80,
	0xc0, 0x01,
	0xc0, 0xa0,
	0xc0, 0x2c,
	0xc0, 0xa0,
	0xc0, 0x01,
	0xc0, 0xe0,
	0xc0, 0x32,
	0xc0, 0xc0,
	0xc0, 0x01,
	0xc0, 0x20,
	0xc0, 0x38,
	0xc0, 0xe0,
	0xc0, 0x01,
	0xc0, 0x60,
	0xc0, 0x3c,
	0xc0, 0x00,
	0xc0, 0x02,
	0xc0, 0xa0,
	0xc0, 0x40,
	0xc0, 0x80,
	0xc0, 0x02,
	0xc0, 0x18,
	0xc0, 0x5c,
	0xfe, 0x00,
	0x9f, 0x10,

	/*BLK*/
	0xfe, 0x00,
	0x26, 0x20,
	0xfe, 0x01,
	0x40, 0x22,
	0x46, 0x7f,
	0x49, 0x0f,
	0x4a, 0xf0,
	0xfe, 0x04,
	0x14, 0x80,
	0x15, 0x80,
	0x16, 0x80,
	0x17, 0x80,

	/*ant _blooming*/
	0xfe, 0x01,
	0x41, 0x20,
	0x4c, 0x00,
	0x4d, 0x0c,
	0x44, 0x08,
	0x48, 0x03,

	/*Window 1600X1200*/
	0xfe, 0x01,
	0x90, 0x01,
	0x91, 0x00,
	0x92, 0x06,
	0x93, 0x00,
	0x94, 0x06,
	0x95, 0x04,
	0x96, 0xb0,
	0x97, 0x06,
	0x98, 0x40,

	/*mipi*/
	0xfe, 0x03,
	0x01, 0x23,
	0x03, 0xce,
	0x04, 0x48,
	0x15, 0x00,
	0x21, 0x10,
	0x22, 0x05,
	0x23, 0x20,
	0x25, 0x20,
	0x26, 0x08,
	0x29, 0x06,
	0x2a, 0x0a,
	0x2b, 0x08,

	/*out*/
	0xfe, 0x01,
	0x8c, 0x10,
	0xfe, 0x00,
	0x3e, 0x00,
};

kal_uint16 addr_data_pair_preview_gc02m1_21143[] = {
	0xfe, 0x00,
	0x3e, 0x90,
};

kal_uint16 addr_data_pair_capture_gc02m1_21143[] = {
	0xfe, 0x00,
	0x3e, 0x90,
};

kal_uint16 addr_data_pair_normal_video_gc02m1_21143[] = {
	0xfe, 0x00,
	0x3e, 0x90,
};

kal_uint16 addr_data_pair_hs_video_gc02m1_21143[] = {
	0xfe, 0x00,
	0x3e, 0x90,
};

kal_uint16 addr_data_pair_slim_video_gc02m1_21143[] = {
	0xfe, 0x00,
	0x3e, 0x90,
};

static void sensor_init(struct subdrv_ctx *ctx)
{
	LOG_INF("===========sensor_init() Enter===========\n");
	gc02m1_table_write_cmos_sensor_8(ctx,
	    addr_data_pair_init_gc02m1_21143,
		sizeof(addr_data_pair_init_gc02m1_21143) /
		sizeof(kal_uint16));

    LOG_INF("8_8_0xfc actually read out: 0x%x(0x01)\n", gc02m1_read_cmos_sensor_8_8(ctx, 0xfc));
    LOG_INF("8_8_0xf4 actually read out: 0x%x(0x41)\n", gc02m1_read_cmos_sensor_8_8(ctx, 0xf4));
    LOG_INF("8_8_0xf5 actually read out: 0x%x(0xc0)\n", gc02m1_read_cmos_sensor_8_8(ctx, 0xf5));
    LOG_INF("8_8_0xf6 actually read out: 0x%x(0x44)\n", gc02m1_read_cmos_sensor_8_8(ctx, 0xf6));

	LOG_INF("===========sensor_init() Exit===========\n");
}

static void preview_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("===========preview_setting() Enter===========\n");
	gc02m1_table_write_cmos_sensor_8(ctx,
		addr_data_pair_preview_gc02m1_21143,
		sizeof(addr_data_pair_preview_gc02m1_21143) /
		sizeof(kal_uint16));

    LOG_INF("8_8_0xfe actually read out: 0x%x(0x00)\n", gc02m1_read_cmos_sensor_8_8(ctx, 0xfe));
    LOG_INF("8_8_0x3e actually read out: 0x%x(0x90)\n", gc02m1_read_cmos_sensor_8_8(ctx, 0x3e));
    LOG_INF("===========preview_setting() Exit===========\n");
}

static void capture_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("capture_setting() Enter\n");
	gc02m1_table_write_cmos_sensor_8(ctx,
		addr_data_pair_capture_gc02m1_21143,
		sizeof(addr_data_pair_capture_gc02m1_21143) /
		sizeof(kal_uint16));
}

static void normal_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("normal_video_setting() Enter\n");
	gc02m1_table_write_cmos_sensor_8(ctx,
	    addr_data_pair_normal_video_gc02m1_21143,
		sizeof(addr_data_pair_normal_video_gc02m1_21143) /
		sizeof(kal_uint16));
}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("hs_video_setting() Enter\n");
	gc02m1_table_write_cmos_sensor_8(ctx,
		addr_data_pair_hs_video_gc02m1_21143,
		sizeof(addr_data_pair_hs_video_gc02m1_21143) /
		sizeof(kal_uint16));
}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("slim_video_setting() Enter\n");
	gc02m1_table_write_cmos_sensor_8(ctx,
		addr_data_pair_slim_video_gc02m1_21143,
		sizeof(addr_data_pair_slim_video_gc02m1_21143) /
		sizeof(kal_uint16));
}

static BYTE gc02m1_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static void read_module_data(struct subdrv_ctx *ctx)
{
    kal_uint16 idx = 0;
    read_gc02m1_eeprom_info_21143(ctx, EEPROM_META_MODULE_ID,
                &(gc02m1_common_data[0]), 2);
    read_gc02m1_eeprom_info_21143(ctx, EEPROM_META_SENSOR_ID,
                &(gc02m1_common_data[2]), 2);
    read_gc02m1_eeprom_info_21143(ctx, EEPROM_META_LENS_ID,
                &(gc02m1_common_data[4]), 2);
    read_gc02m1_eeprom_info_21143(ctx, EEPROM_META_VCM_ID,
                &(gc02m1_common_data[6]), 2);
    read_gc02m1_eeprom_info_21143(ctx, EEPROM_META_MODULE_SN,
                &(gc02m1_common_data[8]), 17);

    for (idx = 0; idx < 30; idx = idx + 4)
        LOG_INF("In %s: cam data: %02x %02x %02x %02x\n", __func__,
               gc02m1_common_data[idx], gc02m1_common_data[idx + 1],
               gc02m1_common_data[idx + 2],
               gc02m1_common_data[idx + 3]);
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_INF("set_test_pattern_mode() Enter, enable: %d\n", enable);

	if (enable) {
		gc02m1_write_cmos_sensor_8_8(ctx, 0xfe, 0x01);
		gc02m1_write_cmos_sensor_8_8(ctx, 0x8c, 0x11);
		gc02m1_write_cmos_sensor_8_8(ctx, 0x8d, 0x0c);
		gc02m1_write_cmos_sensor_8_8(ctx, 0xfe, 0x00);
	} else {
		gc02m1_write_cmos_sensor_8_8(ctx, 0xfe, 0x01);
		gc02m1_write_cmos_sensor_8_8(ctx, 0x8c, 0x10);
		gc02m1_write_cmos_sensor_8_8(ctx, 0x8d, 0x04);
		gc02m1_write_cmos_sensor_8_8(ctx, 0xfe, 0x00);
	}
	ctx->test_pattern = enable;
	return ERROR_NONE;
}

#define GC02M1_EEPROM_READ_ID  0xA4
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, GC02M1_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	LOG_INF("read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data, OTP_SIZE);
	LOG_INF("read_otp_info end\n");
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			*sensor_id = return_sensor_id(ctx);
            LOG_INF("return_sensor_id: 0x%x\n", *sensor_id);
            if (*sensor_id == GC02M1_SENSOR_ID){
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", ctx->i2c_write_id,*sensor_id);
                read_module_data(ctx);
                return ERROR_NONE;
            }
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", ctx->i2c_write_id, *sensor_id);
			retry--;
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

static int open(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;
	LOG_INF("open() Enter\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = return_sensor_id(ctx);
			if (sensor_id == GC02M1B_SENSOR_ID) {
				sensor_id = imgsensor_info.sensor_id;
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", ctx->i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", ctx->i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
	    retry = 2;
	}

	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in */
	sensor_init(ctx);


	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	ctx->hdr_mode = 0;
	ctx->test_pattern = KAL_FALSE;
	ctx->current_fps = imgsensor_info.pre.max_framerate;
    LOG_INF("open() Exit\n");
	return ERROR_NONE;
}

static int close(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
}

static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	/* ctx->video_mode = KAL_FALSE; */
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	preview_setting(ctx);
	return ERROR_NONE;
}

static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (ctx->current_fps == imgsensor_info.cap1.max_framerate) {
		ctx->pclk = imgsensor_info.cap1.pclk;
		ctx->line_length = imgsensor_info.cap1.linelength;
		ctx->frame_length = imgsensor_info.cap1.framelength;
		ctx->min_frame_length = imgsensor_info.cap1.framelength;
		ctx->autoflicker_en = KAL_FALSE;
	} else {
		if (ctx->current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				ctx->current_fps, imgsensor_info.cap.max_framerate / 10);
		ctx->pclk = imgsensor_info.cap.pclk;
		ctx->line_length = imgsensor_info.cap.linelength;
		ctx->frame_length = imgsensor_info.cap.framelength;
		ctx->min_frame_length = imgsensor_info.cap.framelength;
		ctx->autoflicker_en = KAL_FALSE;
	}
	capture_setting(ctx);
	return ERROR_NONE;
}

static kal_uint32 normal_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	normal_video_setting(ctx);
	return ERROR_NONE;
}

static kal_uint32 hs_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	hs_video_setting(ctx);
	return ERROR_NONE;
}

static kal_uint32 slim_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
	return ERROR_NONE;
}

static int get_resolution(struct subdrv_ctx *ctx, MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("get_resolution() Enter\n");
/*
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;
	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;
	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
*/
	{
		kal_uint16 i = 0;
		for (i = SENSOR_SCENARIO_ID_MIN; i < SENSOR_SCENARIO_ID_MAX; i++) {
			if (i < imgsensor_info.sensor_mode_num) {
				sensor_resolution->SensorWidth[i] = imgsensor_winsize_info[i].w2_tg_size;
				sensor_resolution->SensorHeight[i] = imgsensor_winsize_info[i].h2_tg_size;
			} else {
				sensor_resolution->SensorWidth[i] = 0;
				sensor_resolution->SensorHeight[i] = 0;
			}
		}
	}
	return ERROR_NONE;
}

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("get_info() Enter, scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	//sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;
/*
	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
*/
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

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	//sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	//sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;
    LOG_INF("get_info() Exit\n");
/*
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}
*/
	return ERROR_NONE;
}

static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("control() Enter, scenario_id = %d\n", scenario_id);
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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
    LOG_INF("control() Exit\n");
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	/*This Function not used after ROME*/
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	/***********
	 *if (framerate == 0)	 //Dynamic frame rate
	 *	return ERROR_NONE;
	 *if ((framerate == 300) && (ctx->autoflicker_en == KAL_TRUE))
	 *	ctx->current_fps = 296;
	 *else if ((framerate == 150) && (ctx->autoflicker_en == KAL_TRUE))
	 *	ctx->current_fps = 146;
	 *else
	 *	ctx->current_fps = framerate;
	 *set_max_framerate(ctx, ctx->current_fps, 1);
	 ********/
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx, kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	if (enable) /* enable auto flicker */
		ctx->autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		ctx->autoflicker_en = KAL_FALSE;
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		ctx->dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length = imgsensor_info.pre.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
			imgsensor_info.normal_video.linelength;
		ctx->dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		ctx->frame_length = imgsensor_info.normal_video.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		if (ctx->current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			ctx->dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
				(frame_length - imgsensor_info.cap1.framelength) : 0;
			ctx->frame_length = imgsensor_info.cap1.framelength + ctx->dummy_line;
			ctx->min_frame_length = ctx->frame_length;
		} else {
			if (ctx->current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			ctx->dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
				(frame_length - imgsensor_info.cap.framelength) : 0;
			ctx->frame_length = imgsensor_info.cap.framelength + ctx->dummy_line;
			ctx->min_frame_length = ctx->frame_length;
		}
		set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		ctx->dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		ctx->frame_length = imgsensor_info.hs_video.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		ctx->dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		ctx->frame_length = imgsensor_info.slim_video.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		break;
	default:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		ctx->dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length = imgsensor_info.pre.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("get_default_framerate_by_scenario() Enter, scenario_id = %d, framerate = %d\n", scenario_id, *framerate);

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

static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	LOG_INF("feature_control() Enter, feature_id = %d, feature_data = %d \n", feature_id, *feature_data);
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
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
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
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
			case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.cap.pclk;
			 break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.normal_video.pclk;
			 break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.hs_video.pclk;
			 break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			 *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.slim_video.pclk;
			 break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.pre.pclk;
			break;
		}
		break;
  	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.cap.framelength << 16) + imgsensor_info.cap.linelength;
			 break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.normal_video.framelength << 16) + imgsensor_info.normal_video.linelength;
			 break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.hs_video.framelength << 16) + imgsensor_info.hs_video.linelength;
			 break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.slim_video.framelength << 16) + imgsensor_info.slim_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.pre.framelength << 16) + imgsensor_info.pre.linelength;
			break;
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
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*feature_return_para_32 = 1;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		default:
			*feature_return_para_32 = 1;
			break;
		}
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		*(feature_data + 1) = 0;
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode(ctx, (BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
        LOG_INF("feature_control() case SENSOR_FEATURE_SET_REGISTER Enter\n");
		gc02m1_write_cmos_sensor_8_8(ctx, sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = gc02m1_read_cmos_sensor_8_8(ctx, sensor_reg_data->RegAddr);
		LOG_INF("adb_i2c_read 0x%x = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
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
		memcpy(feature_return_para_32, gc02m1_common_data,
			OPLUS_CAMERA_COMMON_DATA_LENGTH);
		*feature_para_len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(ctx, (BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(ctx, (enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(ctx, (enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode(ctx, (BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps: %d\n", (UINT32)*feature_data);
		ctx->current_fps = *feature_data;
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable: %d\n", (BOOL)*feature_data);
		ctx->hdr_mode = (BOOL)*feature_data;
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId: %d\n", (UINT32)*feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE = %d, SE = %d, Gain = %d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain(ctx, (UINT16)*feature_data, (UINT16)*(feature_data + 1),
			(UINT16)*(feature_data + 2));
		break;
	default:
		break;
	}
	return ERROR_NONE;
}

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {//1600*1200
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0640,
			.vsize = 0x04b0,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {//1600*1200
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0640,
			.vsize = 0x04b0,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {//1600*1200
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0640,
			.vsize = 0x04b0,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {//1600*1200
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0640,
			.vsize = 0x04b0,
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
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_vid);
		memcpy(fd->entry, frame_desc_vid, sizeof(frame_desc_vid));
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


static const struct subdrv_ctx defctx = {

	.ana_gain_def = 0x40,
	.ana_gain_max = 768,
	.ana_gain_min = 64,
	.ana_gain_step = 1,
	.exposure_def = 0x3ED,
	.exposure_max = 0x3fff - 16,
	.exposure_min = 4,
	.exposure_step = 1,
	//.frame_time_delay_frame = None,
	.margin = 16,
	.max_frame_length = 0x3fff,

	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3ED,
	.gain = 0x40,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	.hdr_mode = 0,
	.i2c_write_id = 0x6e,
};

static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
    LOG_INF("init_ctx() Enter\n");
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int get_csi_param(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id,
	struct mtk_csi_param *csi_param)
{
	csi_param->legacy_phy = 0;
	csi_param->not_fixed_trail_settle = 0;
	csi_param->dphy_trail = 260;
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
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_AVDD, 2804000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry gc02m1_mipi_raw_21143_entry = {
	.name = "gc02m1_mipi_raw_21143",
	.id = GC02M1_SENSOR_ID_21143,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

