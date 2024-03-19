// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 hi846mipiraw_Sensor.c
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

#include "hi846mipiraw_Sensor_22921.h"
#include "hi846_ana_gain_table_22921.h"
#include "hi846_Sensor_setting_22921.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"
#include "adaptor.h"

#define DEBUG_LOG_EN 0

#define PFX "HI846_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define LOG_DEBUG(...) do { if ((DEBUG_LOG_EN)) LOG_INF(__VA_ARGS__); } while (0)

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor_16(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define write_cmos_sensor_16(...) subdrv_i2c_wr_u16(__VA_ARGS__)
#define hi846_table_write_cmos_sensor_8(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)
#define hi846_table_write_cmos_sensor_16(...) subdrv_i2c_wr_regs_u16(__VA_ARGS__)
#define hi846_burst_write_cmos_sensor_16(...) subdrv_i2c_wr_p16(__VA_ARGS__)

#define FPT_PDAF_SUPPORT 0

#define MODULE_INFO_SIZE 7

#define OTP_SET_ADDR_H 0x070A
#define OTP_SET_ADDR_L 0x070B
#define OTP_RW_FLAG    0x0702
#define OTP_READ_FLAG  0x0708
#define MAX_GROUP_NUM 3
static int module_flag_addr_tbl[MAX_GROUP_NUM] = {0x0201, 0x097C, 0x10F7};
static int module_data_addr_tbl[MAX_GROUP_NUM] = {0x0202, 0x097D, 0x10F8};
unsigned char hi846_data_info[MODULE_INFO_SIZE + 1] = {0};/*Add flag and check sum*/
unsigned char hi846_module_id = 0;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI846_SENSOR_ID_22921,
	.checksum_value = 0xdf4593fd,
	.pre = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2526,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 288000000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2526,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 288000000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2526,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 1840,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 288000000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 842,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 144000000,
		.max_framerate = 900,
	},
	.slim_video = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2526,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 288000000,
		.max_framerate = 300,
	},
	.custom1 = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 3158,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		// following for GetDefaultFramerateByScenario()
		.max_framerate = 240,
		.mipi_pixel_rate = 288000000, // (360M*4/10)
	},
	.custom2 = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2526,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		// following for GetDefaultFramerateByScenario()
		.max_framerate = 300,
		.mipi_pixel_rate = 144000000, // (360M*4/10)
	},
	.custom3 = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 5052,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		// following for GetDefaultFramerateByScenario()
		.max_framerate = 150,
		.mipi_pixel_rate = 144000000, //(360M*4/10)
	},
	.margin = 6,
	.min_shutter = 6,
	.min_gain = BASEGAIN, // 1x gain
	.max_gain = BASEGAIN * 16, // 16x gain
	.min_gain_iso = 100,
	.exp_step = 1,
	.gain_step = 4,
	.gain_type = 3,
	.max_frame_length = 0xffff, // 0xffff
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0, // 1, support; 0,not support
	.ihdr_le_firstline = 0, // 1,le first; 0, se first
	.sensor_mode_num = 8, // support sensor mode num
	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	.custom1_delay_frame = 3, // enter custom1 delay frame num
	.custom2_delay_frame = 3, // enter custom2 delay frame num
	.custom3_delay_frame = 3, // enter custom3 delay frame num
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL, // 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x40, 0xff},
	.i2c_speed = 400,
};

// Sensor output window information
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
	{ 3264, 2448, 0,   0, 3264, 2448, 3264, 2448,   0, 0, 3264, 2448, 0, 0, 3264, 2448}, // preview
	{ 3264, 2448, 0,   0, 3264, 2448, 3264, 2448,   0, 0, 3264, 2448, 0, 0, 3264, 2448}, // capture
	{ 3264, 2448, 0, 304, 3264, 1840, 3264, 1840,   0, 0, 3264, 1840, 0, 0, 3264, 1840}, // video
	{ 3264, 2448, 0, 684, 3264, 1440, 1632,  720, 176, 0, 1280,  720, 0, 0, 1280,  720}, // hight speed video
	{ 3264, 2448, 0,   0, 3264, 2448, 3264, 2448,   0, 0, 3264, 2448, 0, 0, 3264, 2448}, // slim video
	{ 3264, 2448, 0,   0, 3264, 2448, 3264, 2448,   0, 0, 3264, 2448, 0, 0, 3264, 2448},
	{ 3264, 2448, 0,   0, 3264, 2448, 1632, 1224,   0, 0, 1632, 1224, 0, 0, 1632, 1224},
	{ 3264, 2448, 0,   0, 3264, 2448, 1632, 1224,   0, 0, 1632, 1224, 0, 0, 1632, 1224},
};

static void set_dummy(struct subdrv_ctx *ctx)
{
	DEBUG_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);
	write_cmos_sensor_16(ctx, 0x0006, ctx->frame_length & 0xFFFF);
	write_cmos_sensor_16(ctx, 0x0008, ctx->line_length & 0xFFFF);
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate,
	kal_bool min_framelength_en)
{
	kal_uint32 frame_length = ctx->frame_length;

	LOG_DEBUG("framerate = %d, min framelength should enable %d\n",
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
}

static void write_shutter(struct subdrv_ctx *ctx, kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk * 10 / (ctx->line_length * ctx->frame_length);
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else
			write_cmos_sensor_16(ctx, 0x0006, ctx->frame_length);
	} else {
		write_cmos_sensor_16(ctx, 0x0006, ctx->frame_length);
	}
	/* Update Shutter*/
	write_cmos_sensor_16(ctx, 0x0074, (shutter & 0xFFFF));

	LOG_INF("shutter =%d, framelength =%d", shutter, ctx->frame_length);
}

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
static void set_shutter(struct subdrv_ctx *ctx, kal_uint16 shutter)
{
	LOG_INF("set_shutter read 0x095A register : 0x%x\n", read_cmos_sensor_16(ctx, 0x095A));
	ctx->shutter = shutter;
	write_shutter(ctx, shutter);
}

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint16 reg_gain = 0x0;
	reg_gain =  gain *16 /BASEGAIN -  1 * 16;
	return (kal_uint16) reg_gain;
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

	write_cmos_sensor_8(ctx, 0x0077,reg_gain&0xff);/* max = 0xf0*/

	return gain;
}

static void set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);
	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor_16(ctx, 0x000e,0x0000); // B
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor_16(ctx, 0x000e,0x0100); // Gb
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor_16(ctx, 0x000e,0x0200); // Gr
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor_16(ctx, 0x000e,0x0300); // R
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}
}

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_16(ctx, 0x0a00, 0x0100);
	} else {
		write_cmos_sensor_16(ctx, 0x0a00, 0x0000);
	}

	mdelay(5);
	LOG_INF("streaming_control read 0x095A register : 0x%x\n", read_cmos_sensor_16(ctx, 0x095A));
	return ERROR_NONE;
}

static void sensor_init(struct subdrv_ctx *ctx)
{
	/*Global setting */
	LOG_INF("E\n");

	hi846_table_write_cmos_sensor_16(ctx, addr_data_pair_init_hi846,
		sizeof(addr_data_pair_init_hi846) / sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void preview_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");

	hi846_table_write_cmos_sensor_16(ctx, addr_data_pair_preview_hi846,
		sizeof(addr_data_pair_preview_hi846) / sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void capture_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");

	hi846_table_write_cmos_sensor_16(ctx, addr_data_pair_capture_30fps_hi846,
		sizeof(addr_data_pair_capture_30fps_hi846) / sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void normal_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");

	hi846_table_write_cmos_sensor_16(ctx, addr_data_pair_video_hi846,
		sizeof(addr_data_pair_video_hi846) / sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");

	hi846_table_write_cmos_sensor_16(ctx, addr_data_pair_hs_video_hi846,
		sizeof(addr_data_pair_hs_video_hi846) / sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");

	hi846_table_write_cmos_sensor_16(ctx, addr_data_pair_slim_video_hi846,
		sizeof(addr_data_pair_slim_video_hi846) / sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void custom1_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");

	hi846_table_write_cmos_sensor_16(ctx, addr_data_pair_custom1_hi846,
		sizeof(addr_data_pair_custom1_hi846)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void custom2_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");

	hi846_table_write_cmos_sensor_16(ctx, addr_data_pair_custom2_hi846,
		sizeof(addr_data_pair_custom2_hi846)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void custom3_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");

	hi846_table_write_cmos_sensor_16(ctx, addr_data_pair_custom3_hi846,
		sizeof(addr_data_pair_custom3_hi846)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

static int selecte_hi846_group_number(struct subdrv_ctx *ctx)
{
	int otp_grp_flag = 0;
	int i = 0, group_num = 0;

	/* read flag */
	for(i = MAX_GROUP_NUM-1; i >= 0; i--){
		write_cmos_sensor_8(ctx, OTP_SET_ADDR_H, (module_flag_addr_tbl[i]>>8)&0xff);
		write_cmos_sensor_8(ctx, OTP_SET_ADDR_L, (module_flag_addr_tbl[i])&0xff);
		write_cmos_sensor_8(ctx, OTP_RW_FLAG, 0x01);
		otp_grp_flag = read_cmos_sensor_8(ctx, OTP_READ_FLAG);
		LOG_INF("module module_flag_addr_tbl: 0x%x, otp_grp_flag: 0x%x\n", module_flag_addr_tbl[i], otp_grp_flag);
		if (otp_grp_flag == 0x55) {
			group_num = i;
			LOG_INF("select group number: group %d selected\n", i);
			break;
		}
	}

	return group_num;
}

static int read_hi846_module_data_info(struct subdrv_ctx *ctx, int group_num)
{
	int year = 0, month = 0, day = 0;
	int position_id = 0,lens_id = 0,vcm_id = 0;
	int check_sum = 0, check_sum_cal = 0;
	int i = 0, number = 0, rc = 0;

	number = group_num;
	if(module_data_addr_tbl[number] != 0){
		write_cmos_sensor_8(ctx, OTP_SET_ADDR_H, (module_data_addr_tbl[number]>>8)&0xff);
		write_cmos_sensor_8(ctx, OTP_SET_ADDR_L, module_data_addr_tbl[number]&0xff);
		write_cmos_sensor_8(ctx, OTP_RW_FLAG, 0x01);
		for(i = 0; i < MODULE_INFO_SIZE + 1; i++){  // add flag and checksum
			hi846_data_info[i]=read_cmos_sensor_8(ctx, OTP_READ_FLAG);
		}

		for(i = 0; i < MODULE_INFO_SIZE; i++){
			check_sum_cal += hi846_data_info[i];
		}

		check_sum_cal = check_sum_cal % 255 + 1;
		hi846_module_id = hi846_data_info[0];
		position_id = hi846_data_info[1];
		lens_id = hi846_data_info[2];
		vcm_id = hi846_data_info[3];
		year = hi846_data_info[4] + 2021 - 0x15;
		month = hi846_data_info[5];
		day = hi846_data_info[6];
		check_sum = hi846_data_info[MODULE_INFO_SIZE];

		pr_err("=== HI846 INFO module_id=0x%x ===\n", hi846_module_id);
		pr_err("=== HI846 INFO date is %d-%d-%d ===\n", year, month, day);
		pr_err("=== HI846 INFO position_id=0x%x, lens_id=0x%x,vcm_id=0x%x ===\n", position_id, lens_id, vcm_id);
		pr_err("=== HI846 INFO check_sum=0x%x,check_sum_cal=0x%x ===\n", check_sum, check_sum_cal);
		if(check_sum == check_sum_cal){
			pr_err("=== check_sum_module success ===\n");
			hi846_module_id = 1;
		}else{
			pr_err("=== check_sum_module failed ===\n\n");
			hi846_module_id = 0;
		}

		if(hi846_module_id)
		{
			rc = 1;
		}
	}
	return rc;
}

static void read_module_data(struct subdrv_ctx *ctx)
{
	int group_num = 0, ret = 0;
	pr_err("come to %s:%d E!\n", __func__, __LINE__);

	/* 1. sensor init */
	write_cmos_sensor_16(ctx, 0x0A00, 0x0000); //tg_pmem_sckpw/sdly
	write_cmos_sensor_16(ctx, 0x2000, 0x0000); //tg_pmem_sckpw/sdly
	write_cmos_sensor_16(ctx, 0x2002, 0x00FF); //tg_pmem_sckpw/sdly
	write_cmos_sensor_16(ctx, 0x2004, 0x0000); //tg_pmem_rom_dly
	write_cmos_sensor_16(ctx, 0x2008, 0x3FFF); // firmware start address-ROM
	write_cmos_sensor_16(ctx, 0x23FE, 0xC056); // BGR enable
	write_cmos_sensor_16(ctx, 0x0A00, 0x0000); // STRB(OTP Busy) output enable
	write_cmos_sensor_16(ctx, 0x0E04, 0x0012); // STRB(OTP Busy) output drivability
	write_cmos_sensor_16(ctx, 0x0F08, 0x2F04); // Analog PLL setting
	write_cmos_sensor_16(ctx, 0x0F30, 0x001F); // Analog CLKGEN setting
	write_cmos_sensor_16(ctx, 0x0F36, 0x001F); // Analog CLKGEN setting
	write_cmos_sensor_16(ctx, 0x0F04, 0x3A00); // PLL enable
	write_cmos_sensor_16(ctx, 0x0F32, 0x025A); // mipi disable
	write_cmos_sensor_16(ctx, 0x0F38, 0x0256); // TG PMEM CEN anable
	write_cmos_sensor_16(ctx, 0x0F2A, 0x4124); // TG MCU enable
	write_cmos_sensor_16(ctx, 0x006A, 0x0100); // ROM OTP Continuous W/R mode enable
	write_cmos_sensor_16(ctx, 0x004C, 0x0100); // Stream ON

	/* 2. init OTP setting*/
	write_cmos_sensor_8(ctx, 0x0A02, 0x01); //Fast sleep on
	write_cmos_sensor_8(ctx, 0x0A00, 0x00);//stand by on
	mdelay(10);
	write_cmos_sensor_8(ctx, 0x0F02, 0x00);//pll disable
	write_cmos_sensor_8(ctx, 0x071A, 0x01);//CP TRIM_H
	write_cmos_sensor_8(ctx, 0x071B, 0x09);//IPGM TRIM_H
	write_cmos_sensor_8(ctx, 0x0D04, 0x00);//Fsync(OTP busy)Output Enable
	write_cmos_sensor_8(ctx, 0x0D00, 0x07);//Fsync(OTP busy)Output Drivability
	write_cmos_sensor_8(ctx, 0x003E, 0x10);//OTP r/w mode
	write_cmos_sensor_8(ctx, 0x0A00, 0x01);//standby off
	mdelay(1);

	/* 3. read eeprom data */
	group_num = selecte_hi846_group_number(ctx);
	mdelay(1);
	ret = read_hi846_module_data_info(ctx, group_num);
	if(ret != 1){
		pr_err("=== hi846_module_data invalid ===\n");
	}else{
		pr_err("=== hi846_module_data success ===\n");
	}

	/* 4. disable otp function */
	write_cmos_sensor_8(ctx, 0x0a00, 0x00);
	mdelay(10);
	write_cmos_sensor_8(ctx, 0x003e, 0x00);
	write_cmos_sensor_8(ctx, 0x0a00, 0x01);
	mdelay(1);
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
	static bool first_read = KAL_TRUE;
	LOG_INF("get_imgsensor_id E\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			*sensor_id =
				((read_cmos_sensor_8(ctx, 0x0F17) << 8) | read_cmos_sensor_8(ctx, 0x0F16));
			LOG_INF("read out sensor id 0x%x\n", *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", ctx->i2c_write_id, *sensor_id);
				if(first_read){
					read_module_data(ctx);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", ctx->i2c_write_id);
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

	LOG_INF("get_imgsensor_id X\n");
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

	LOG_INF("[open]: PLATFORM:MT6983,MIPI 4LANE\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = ((read_cmos_sensor_8(ctx, 0x0F17) << 8) | read_cmos_sensor_8(ctx, 0x0F16));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", ctx->i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", ctx->i2c_write_id);
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
	ctx->shutter = 0x9D8;
	ctx->gain = 0x10;
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
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("X\n");
	LOG_INF("preview read 0x095A register : 0x%x\n", read_cmos_sensor_16(ctx, 0x095A));
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

	LOG_INF("Caputre fps:%d\n", ctx->current_fps);
	capture_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("X\n");
	return ERROR_NONE;
}

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
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("X\n");
	return ERROR_NONE;
}

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
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("X\n");
	return ERROR_NONE;
}

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
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("X\n");
	return ERROR_NONE;
}

static kal_uint32 custom1(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	ctx->pclk = imgsensor_info.custom1.pclk;
	ctx->line_length = imgsensor_info.custom1.linelength;
	ctx->frame_length = imgsensor_info.custom1.framelength;
	ctx->min_frame_length = imgsensor_info.custom1.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	custom1_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}

static kal_uint32 custom2(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	ctx->pclk = imgsensor_info.custom2.pclk;
	ctx->line_length = imgsensor_info.custom2.linelength;
	ctx->frame_length = imgsensor_info.custom2.framelength;
	ctx->min_frame_length = imgsensor_info.custom2.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	custom2_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}

static kal_uint32 custom3(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	ctx->pclk = imgsensor_info.custom3.pclk;
	ctx->line_length = imgsensor_info.custom3.linelength;
	ctx->frame_length = imgsensor_info.custom3.framelength;
	ctx->min_frame_length = imgsensor_info.custom3.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	custom3_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}

static int get_resolution(struct subdrv_ctx *ctx,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	int i = 0;

	for (i = SENSOR_SCENARIO_ID_MIN; i < SENSOR_SCENARIO_ID_MAX; i++) {
		if (i < imgsensor_info.sensor_mode_num &&
			i < ARRAY_SIZE(imgsensor_winsize_info)) {
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
	LOG_DEBUG("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	// not use
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	// inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; // not use
	sensor_info->SensorResetActiveHigh = FALSE; // not use
	sensor_info->SensorResetDelayCount = 5; // not use

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

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

	sensor_info->SensorMasterClockSwitch = 0;
	// not use
	sensor_info->SensorDrivingCurrent =
		imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame =
		imgsensor_info.ae_shut_delay_frame;
	// The frame of setting shutter default 0 for TG int
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	// The frame of setting sensor gain
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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	LOG_DEBUG("framerate = %d\n ", framerate);
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

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx, kal_bool enable, UINT16 framerate)
{
	(void) framerate;

	LOG_DEBUG("enable = %d\n", enable);
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

	DEBUG_LOG(ctx, "scenario_id = %d, framerate = %d\n", scenario_id, framerate);

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
		if (ctx->current_fps !=
			imgsensor_info.cap.max_framerate)
		LOG_INF("fps %d fps not support,use cap: %d fps!\n",
		framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk /
			framerate * 10 / imgsensor_info.cap.linelength;
		ctx->dummy_line = (frame_length >
			imgsensor_info.cap.framelength) ?
		(frame_length - imgsensor_info.cap.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.cap.framelength +
			ctx->dummy_line;
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
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
			/ imgsensor_info.custom1.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom1.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		//if (ctx->frame_length > ctx->shutter)
		set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
			/ imgsensor_info.custom2.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom2.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
        //if (ctx->frame_length > ctx->shutter)
            set_dummy(ctx);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM3:
        frame_length = imgsensor_info.custom3.pclk / framerate * 10
                / imgsensor_info.custom3.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom3.framelength)
            ? (frame_length - imgsensor_info.custom3.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom3.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        //if (ctx->frame_length > ctx->shutter)
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
	LOG_DEBUG("scenario_id = %d\n", scenario_id);

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
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0 : Normal, 1 : Solid Color, 2 : Color Bar, 3 : Shade Color Bar, 4 : PN9
		write_cmos_sensor_16(ctx, 0x0A04, 0x0141);
		write_cmos_sensor_16(ctx, 0x020A, 0x0100);
	} else {
		write_cmos_sensor_16(ctx, 0x020A, 0x0000);
	}
	ctx->test_pattern = enable;
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

#if FPT_PDAF_SUPPORT
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
#endif
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_INF("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
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
			*(feature_data + 1)
				= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(hi846_ana_gain_table);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)hi846_ana_gain_table,
			sizeof(hi846_ana_gain_table));
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
		write_cmos_sensor_16(ctx, sensor_reg_data->RegAddr,
			sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_16(ctx, sensor_reg_data->RegAddr);
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
		*feature_return_para_32 = (hi846_data_info[2] << 8) |
			(hi846_data_info[3] & 0xFF);
		*feature_para_len = 4;
		LOG_INF("hi846 GET_MODULE_CamInfo:%d 0x%02x\n", *feature_para_len, *feature_data_32);
		break;
	case SENSOR_FEATURE_GET_MODULE_SN:
		break;
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = (UINT32)imgsensor_info.module_id;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_EEPROM_COMDATA:
		memcpy(feature_return_para_32, hi846_data_info,
			MODULE_INFO_SIZE + 1);
		*feature_para_len = MODULE_INFO_SIZE + 1;
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
		ctx->current_fps = (UINT16)*feature_data_32;
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		ctx->ihdr_mode = (UINT8)*feature_data_32;
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
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
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
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM3:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
#if FPT_PDAF_SUPPORT
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo =
			(struct SET_PD_BLOCK_INFO_T *)
				(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM3:
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
		default:
			break;
		}
#endif
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF(
		    "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n",
			*feature_data);
#if FPT_PDAF_SUPPORT
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		}
#else
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
		break;
#endif
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
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
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
			.hsize = 3264,
			.vsize = 2448,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1840,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1280,
			.vsize = 720,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1632,
			.vsize = 1224,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1632,
			.vsize = 1224,
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
		fd->num_entries = ARRAY_SIZE(frame_desc_cust1);
		memcpy(fd->entry, frame_desc_cust1, sizeof(frame_desc_cust1));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cust2);
		memcpy(fd->entry, frame_desc_cust2, sizeof(frame_desc_cust2));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cust3);
		memcpy(fd->entry, frame_desc_cust3, sizeof(frame_desc_cust3));
		break;
	default:
		return -1;
	}
	return 0;
}
#endif

static const struct subdrv_ctx defctx = {
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_min = BASEGAIN,
	.ana_gain_step = 1,
	.exposure_def = 0x9D8,
	.exposure_max = 0xffff - 6,
	.exposure_min = 6,
	.exposure_step = 1,
	.margin = 6,
	.max_frame_length = 0xffff,
	.mirror = IMAGE_H_MIRROR, // mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x9D8, // current shutter
	.gain = BASEGAIN * 4, // current gain
	.dummy_pixel = 0, // current dummypixel*/
	.dummy_line = 0, // current dummyline
	.current_fps = 300,
	// full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	.ihdr_mode = KAL_FALSE, // sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x40,
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
	csi_param->dphy_trail = 76;

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
	.get_csi_param = get_csi_param,
#ifdef IMGSENSOR_VC_ROUTING
	.get_frame_desc = get_frame_desc,
#endif
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DVDD, 1200000, 1},
	{HW_ID_MCLK, 24, 0},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_RST, 1, 3},
};

const struct subdrv_entry hi846_mipi_raw_22921_entry = {
	.name = "hi846_mipi_raw_22921",
	.id = HI846_SENSOR_ID_22921,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};
