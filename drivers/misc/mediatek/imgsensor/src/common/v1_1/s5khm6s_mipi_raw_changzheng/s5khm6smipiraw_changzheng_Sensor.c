/*
 * Copyright (C) 2016 MediaTek Inc.
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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5khm6smipiraw_changzheng_Sensor.c
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
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imgsensor_sensor_list.h"

#include "s5khm6smipiraw_changzheng_Sensor.h"
#include "s5khm6smipiraw_changzheng_setting.h"

#define PFX "S5KHM6S_camera_sensor"
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)

#define MULTI_WRITE 1

#define MaxAnalogGain 64

#ifndef ODM_HQ_EDIT
/*Cong.Zhou@ODM_HQ Cam.Drv 20191218 for decrease poweron time*/
#define ODM_HQ_EDIT
#endif

static kal_uint8 deviceInfo_register_value = 0x00;
extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
#define DEVICE_VERSION_CHANGZHENG_S5KHM6S    "s5khm6s"

#if MULTI_WRITE
#define I2C_BUFFER_LEN 1020 /*trans# max is 255, each 4 bytes*/
#else
#define I2C_BUFFER_LEN 4
#endif

static kal_uint16 camstate;
//static kal_uint32 revision;
static kal_uint16 lastcamstate;

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static bool bIsLongExposure = KAL_FALSE;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KHM6S_SENSOR_ID_CHANGZHENG,
	.checksum_value = 0xb1f1b3cc,

	.pre = {
		.pclk = 1640000000,
		.linelength  = 11232,
		.framelength = 4864,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 963200000,
               .jn1_binning_mode = 3,
	},
	.cap = {
		.pclk = 1640000000,
		.linelength  = 11232,
		.framelength = 4864,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 963200000,
               .jn1_binning_mode = 3,
	},
	.normal_video = {
		.pclk = 1640000000,
		.linelength  = 11088,
		.framelength = 4928,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4000,
		.grabwindow_height = 2252,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 936000000,
		.jn1_binning_mode = 3,
	},
	.hs_video = {
		.pclk = 820000000,
		.linelength  = 8648,
		.framelength = 790,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
		.mipi_pixel_rate = 963200000,
		.jn1_binning_mode = 3,
	},
	.slim_video = {
		.pclk = 1640000000,
		.linelength  = 11088,
		.framelength = 4928,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4000,
		.grabwindow_height = 2252,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 936000000,
               .jn1_binning_mode = 3,
	},
	.custom1 = {
		.pclk = 1640000000,
		.linelength  = 12256,
		.framelength = 5568,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
		.mipi_pixel_rate = 793600000,
               .jn1_binning_mode = 3,
	},
	.custom2 = {
		.pclk = 1640000000,
		.linelength  = 11264,
		.framelength = 2426,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
		.mipi_pixel_rate = 936000000,
               .jn1_binning_mode = 3,
	},
	.custom3 = {
		.pclk = 1640000000,
		.linelength  = 22200,
		.framelength = 9228,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 12000,
		.grabwindow_height = 9000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 80,
		.mipi_pixel_rate = 936000000,
               .jn1_binning_mode = 0,
	},
	.custom4 = {
		.pclk = 1640000000,
		.linelength  = 5728,
		.framelength = 1192,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 2400,
		.mipi_pixel_rate = 936000000,
               .jn1_binning_mode = 3,
	},


	.margin = 25,		/* sensor framelength & shutter margin */
	.min_shutter = 12,	/* min shutter */

	.min_gain = 64,
	.max_gain = 4096,
	.min_gain_iso = 100,
	.gain_step = 2,
	.gain_type = 2, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC

	.max_frame_length = 0xfffd,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 9,
	.cap_delay_frame = 3,	/* enter capture delay frame num */
	.pre_delay_frame = 3,	/* enter preview delay frame num */
	.video_delay_frame = 3,	/* enter video delay frame num */
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
	.custom1_delay_frame = 3,
	.custom2_delay_frame = 3,
	.custom3_delay_frame = 3,
	.custom4_delay_frame = 3,
	//.custom5_delay_frame = 1,
	.frame_time_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = 1, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0xff},
	.i2c_speed = 1000, // i2c read/write speed
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* NORMAL information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,

	/* test pattern mode or not.
	 * KAL_FALSE for in test pattern mode,
	 * KAL_TRUE for normal output
	 */
	.test_pattern = KAL_FALSE,

	/* current scenario id */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	/* sensor need support LE, SE with HDR feature */

	.hdr_mode = KAL_FALSE,
	/* HDR Mode : 0: disable HDR, 1:IHDR, 2:HDR, 9:ZHDR */
	.i2c_write_id = 0x20,
	/* record current sensor's i2c write id */
    //#ifdef ODM_HQ_EDIT
    /*Cong.Zhou@ODM_HQ Cam.Drv 20191030 long exp n+2, n+1 will cause switch slowly*/
    .current_ae_effective_frame = 2,
    //#endif

};

//int chip_id;
/* VC_Num, VC_PixelNum, ModeSelect, EXPO_Ratio, ODValue, RG_STATSMODE */
/* VC0_ID, VC0_DataType, VC0_SIZEH, VC0_SIZE,
 * VC1_ID, VC1_DataType, VC1_SIZEH, VC1_SIZEV
 */
/* VC2_ID, VC2_DataType, VC2_SIZEH, VC2_SIZE,
 * VC3_ID, VC3_DataType, VC3_SIZEH, VC3_SIZEV
 */
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[4] = {
	// Preview mode setting 2000(pxiel)*1500
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 4000, 3000, 0x00, 0x00, 0x0000, 0x0000,
		0x01, 0x2b, 2000, 1500, 0x00, 0x00, 0x0000, 0x0000
	},
	// Video mode setting 2000(pxiel)*1126
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 4000, 2252, 0x00, 0x00, 0x0000, 0x0000,
		0x01, 0x2b, 2000, 1126, 0x00, 0x00, 0x0000, 0x0000
	},
	//1080p
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 1920, 1080, 0x00, 0x00, 0x0000, 0x0000,
		0x01, 0x2b, 960, 540, 0x00, 0x00, 0x0000, 0x0000
	},
	// Full crop 12M mode setting 664(pxiel)*500
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 4000, 3000, 0x00, 0x00, 0x0000, 0x0000,
		0x01, 0x2b, 1332, 1000, 0x00, 0x00, 0x0000, 0x0000
	},
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[9] = {
	{12000, 9000,    0,    0, 12000, 9000,  4000, 3000,
		0,     0, 4000, 3000,     0,    0,  4000, 3000}, // Preview
	{12000, 9000,    0,    0, 12000, 9000,  4000, 3000,
		0,     0, 4000, 3000,     0,    0,  4000, 3000}, // capture
	{12000, 9000,    0, 1122, 12000, 6756,  4000, 2252,
		0,     0, 4000, 2252,     0,    0,  4000, 2252}, // video
	{12000, 9000, 4080, 3420, 3840, 2160,  1280,  720,
		0,     0, 1280,  720,     0,    0,  1280,  720}, // hight speed video
	{12000, 9000,    0, 1122, 12000, 6756,  4000, 2252,
		0,     0, 4000, 2252,     0,    0,  4000, 2252}, // slim video
	{12000, 9000,    0,    0, 12000, 9000,  4000, 3000,
		0,     0, 4000, 3000,     0,    0,  4000, 3000}, //Custom1
	{12000, 9000,  240, 1260, 11520, 6480,  3840, 2160,
		0,     0, 3840, 2160,     0,    0,  3840, 2160}, // Custom2
	{12000, 9000,    0,    0, 12000, 9000, 12000, 9000,
		0,     0, 12000, 9000,    0,    0, 12000, 9000}, // custom3
	{12000, 9000, 3120, 2880,  5760, 3240,  1920, 1080,
		0,     0, 1920, 1080,     0,    0,  1920, 1080}, // Custom4
//	{12000, 9000,    0,    0, 12000, 9000,  4000, 3000,
//		0,     0, 4000, 3000,     0,    0,  4000, 3000}, // custom5
};


static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info[1] = {
	{
		.i4OffsetX	= 0,
		.i4OffsetY	= 0,
		.i4PitchX	= 2,
		.i4PitchY	= 4,
		.i4PairNum	= 1,
		.i4SubBlkW	= 2,
		.i4SubBlkH	= 4,
		.i4BlockNumX = 2000,
		.i4BlockNumY = 750,
		.iMirrorFlip = 0,
		.i4PosL = {
			{1, 0}, //{3, 0},
		},
		.i4PosR = {
			{0, 0}, //{2, 0},
		},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_normal_video_pd_info = {
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  = 4,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {
		{9, 8}, {11, 11}, {15, 12}, {13, 15}
	},
	.i4PosR = {
		{8, 8}, {10, 11}, {14, 12}, {12, 15}
	},
	.iMirrorFlip = 0,
	.i4BlockNumX = 480,
	.i4BlockNumY = 268,
	.i4Crop = { {0,0}, {0,0}, {120,456}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}},
};
#define RWB_ID_OFFSET 0x0F73
#define EEPROM_READ_ID  0x19
#define EEPROM_WRITE_ID   0x18


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {
		(char)(addr >> 8), (char)(addr & 0xFF),
		(char)(para >> 8), (char)(para & 0xFF) };
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	int ret = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
	ret = iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

    printk("[%s] read i2c: 0x%x addr:0x%x value:0x%x, ret = %d\n", __func__, imgsensor.i2c_write_id , addr, get_byte, ret);

	return get_byte;
}
/*
static kal_uint16 read_cmos_sensor_16(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	int ret = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
	ret = iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 2, imgsensor.i2c_write_id);

	printk("[%s] read i2c: (slave, reg_addr, value) = (0x%x, 0x%x, 0x%x), ret = %d\n", __func__, imgsensor.i2c_write_id, addr, get_byte, ret);

	return get_byte;
}
*/
static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {
		(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	pr_debug("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);

	/* return; //for test */
	write_cmos_sensor(0x0340, imgsensor.frame_length);
	write_cmos_sensor(0x0342, imgsensor.line_length);
}				/*      set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	pr_debug("framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
		imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;

		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*      set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{

	kal_uint16 realtime_fps = 0;
	//#ifdef ODM_HQ_EDIT
	/*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
	//int framecnt = 0;
	//#endif
	pr_debug("===brad shutter:%d\n", shutter);
	if (shutter > 65530) {  //linetime=10160/960000000<< maxshutter=3023622-line=32s
		/*enter long exposure mode */
/*
		kal_uint32 exposure_time;
		kal_uint16 new_framelength;
		kal_uint16 long_shutter;
		kal_uint16 shift = 0;
		kal_uint16 pow_shift = 0;
*/

		bIsLongExposure = KAL_TRUE;

		/*streaming_control(KAL_FALSE);*/
		write_cmos_sensor(0x6028, 0x4000);
		//#ifdef ODM_HQ_EDIT
		/*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
		//write_cmos_sensor_8(0x0100, 0x00); /*stream off*/

/*
		while (1) {
			framecnt = read_cmos_sensor_8(0x0005);
			LOG_INF(" Stream Off oning at framecnt=0x%x.\n", framecnt);
			if (framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at framecnt=0x%x.\n", framecnt);
				break;
			}
		}
*/
		//#endif

		//write_cmos_sensor(0x0340, 0x45e2);
		//write_cmos_sensor(0x0202, 0x45de);
		write_cmos_sensor(0x0340, (shutter>>6)+3);
		write_cmos_sensor(0x0202, (shutter>>6));
		write_cmos_sensor(0x0702, 0x0600);
		write_cmos_sensor(0x0704, 0x0600);

		//#ifdef ODM_HQ_EDIT
		/*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
		//write_cmos_sensor_8(0x0100, 0x01);  /*stream on*/
		//#endif

		/* Frame exposure mode customization for LE*/
		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
        //#ifdef ODM_HQ_EDIT
        /*Cong.Zhou@ODM_HQ Cam.Drv 20191030 long exp n+2, n+1 will cause switch slowly*/
        imgsensor.current_ae_effective_frame = 2;
        //#endif
	} else {
		//#ifdef ODM_HQ_EDIT
		/*Cong.Zhou@ODM_HQ Cam.Drv 20191030 long exp n+2, n+1 will cause switch slowly*/
		imgsensor.current_ae_effective_frame = 2;
		//#endif
		if (bIsLongExposure) {
			pr_debug("brad enter normal shutter.\n");
			write_cmos_sensor(0x6028, 0x4000);
			//#ifdef ODM_HQ_EDIT
			/*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
			//write_cmos_sensor_8(0x0100, 0x00); /*stream off*/
/*
			while (1) {
				framecnt = read_cmos_sensor_8(0x0005);
				pr_debug("Stream Off oning at framecnt=0x%x.\n", framecnt);
				if (framecnt == 0xFF) {
					pr_debug(" Stream Off OK at framecnt=0x%x.\n", framecnt);
					break;
				}
			}
*/
			//#endif

			write_cmos_sensor(0x0340, imgsensor.frame_length);
			write_cmos_sensor(0x0202, shutter);
			write_cmos_sensor(0x0702, 0x0000);
			write_cmos_sensor(0x0704, 0x0000);

			//#ifdef ODM_HQ_EDIT
			/*Jiefeng.Wang@ODM_HQ Cam.Drv 20191108 temp modify long exp no effect*/
			//write_cmos_sensor_8(0x0100, 0x01);  /*stream on*/
			//#endif
			bIsLongExposure = KAL_FALSE;
			pr_debug("===brad enter normal shutter shutter = %d, imgsensor.frame_lengths = 0x%x, imgsensor.line_length = 0x%x\n",
				shutter, imgsensor.frame_length, imgsensor.line_length);
		}

		spin_lock(&imgsensor_drv_lock);
		if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = shutter + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length = imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (shutter < imgsensor_info.min_shutter)
			shutter = imgsensor_info.min_shutter;

		if (imgsensor.autoflicker_en) {
			realtime_fps = imgsensor.pclk
				/ imgsensor.line_length * 10 / imgsensor.frame_length;

			if (realtime_fps >= 297 && realtime_fps <= 305)
				set_max_framerate(296, 0);
			else if (realtime_fps >= 147 && realtime_fps <= 150)
				set_max_framerate(146, 0);
			else {
				/* Extend frame length */
				write_cmos_sensor(0x0340, imgsensor.frame_length);
			}
		} else {
			/* Extend frame length */
			write_cmos_sensor(0x0340, imgsensor.frame_length);
			pr_debug("(else)imgsensor.frame_length = %d\n",
				imgsensor.frame_length);
		}

		if (imgsensor.jn1_binning_mode == 1)  //4sum
			shutter = (shutter>>1)<<1;
		if (imgsensor.jn1_binning_mode == 3)  //4sumA2A2
			shutter = (shutter>>2)<<2;

		/* Update Shutter */
		write_cmos_sensor(0x0202, shutter);
		pr_debug("shutter =%d, framelength =%d jn1_binning_mode= %d\n",
			shutter, imgsensor.frame_length, imgsensor.jn1_binning_mode);
	}
}				/*      write_shutter  */

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
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}				/*      set_shutter */

static void set_shutter_frame_length(kal_uint16 shutter,
			kal_uint16 frame_length, kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	if(frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	//
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length -
		imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
		imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk /
			imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			// Extend frame length
			//write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length);
			//write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		// Extend frame length
		//write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340,  imgsensor.frame_length);
		//write_cmos_sensor(0x0104, 0x00);
	}

	// Update Shutter
	//write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0202, shutter);
	//write_cmos_sensor(0x0104, 0x00);
	printk("S5KHM6S_set_shutter_frame_length() shutter =%d, frame_length = %d, dummy_line = %d, auto_extend_en = %d, imgsensor.frame_length =%d\n",
		shutter, frame_length, dummy_line, auto_extend_en, imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0;

	reg_gain = gain / 2;
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
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* gain=1024;//for test */
	/* return; //for test */

	if (gain < BASEGAIN || gain > MaxAnalogGain * BASEGAIN) {
		pr_debug("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > MaxAnalogGain * BASEGAIN)
			gain = MaxAnalogGain * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, reg_gain);
	/* write_cmos_sensor_8(0x0204,(reg_gain>>8)); */
	/* write_cmos_sensor_8(0x0205,(reg_gain&0xff)); */

	return gain;
}				/*      set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{

	kal_uint8 itemp;

	pr_debug("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor_8(0x0101);
	itemp &= ~0x03;

	switch (image_mirror) {

	case IMAGE_NORMAL:
		write_cmos_sensor_8(0x0101, itemp);
		break;

	case IMAGE_V_MIRROR:
		write_cmos_sensor_8(0x0101, itemp | 0x02);
		break;

	case IMAGE_H_MIRROR:
		write_cmos_sensor_8(0x0101, itemp | 0x01);
		break;

	case IMAGE_HV_MIRROR:
		write_cmos_sensor_8(0x0101, itemp | 0x03);
		break;
	}
}

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
}				/*      night_mode      */
#endif



/*************************************************************************
 * FUNCTION
 *	check_stremoff
 *
 * DESCRIPTION
 *	waiting function until sensor streaming finish.
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

static void check_streamoff(void)
{
	unsigned int i = 0;
	int timeout = (10000 / imgsensor.current_fps) + 1;

	mdelay(3);
	for (i = 0; i < timeout; i++) {
		if (read_cmos_sensor_8(0x0005) != 0xFF)
			mdelay(1);
		else
			break;
	}
	pr_debug(" %s exit! %d\n", __func__, i);
}

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable) {
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0100, 0x0100);
	} else {
		write_cmos_sensor(0xFCFC, 0x4000);
		write_cmos_sensor(0x0100, 0x0000);
		check_streamoff();
	}
	return ERROR_NONE;
}

static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];
		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}

#if MULTI_WRITE
	if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
		iBurstWriteReg_multi(puSendCmd, tosend,
			imgsensor.i2c_write_id, 4, 1000);

			tosend = 0;
	}
#else
		iWriteRegI2CTiming(puSendCmd, 4,
			imgsensor.i2c_write_id, imgsensor_info.i2c_speed);

		tosend = 0;
#endif

	}
	return 0;
}

static void sensor_init(void)
{
	LOG_INF("s5khm6sp--- E\n");
	
	write_cmos_sensor(0xFCFC, 0x4000);
	write_cmos_sensor(0x0000, 0x01B0);
	write_cmos_sensor(0x0000, 0x1AD6);
	write_cmos_sensor(0xfcfc, 0x4000);
	write_cmos_sensor(0x6010, 0x0001);
	mdelay(30);
	

	table_write_cmos_sensor(addr_data_pair_init_hm2,
		sizeof(addr_data_pair_init_hm2) / sizeof(kal_uint16));
	LOG_INF("s5khm6sp--- X\n");
	LOG_INF("X");

}				/*      sensor_init  */

static void preview_setting(void)
{
	LOG_INF("s5khmsp--- preview_setting_0719  E\n");

	table_write_cmos_sensor(addr_data_pair_pre_hm2,
			sizeof(addr_data_pair_pre_hm2) / sizeof(kal_uint16));
	LOG_INF("s5khmsp--- preview_setting_0719  X\n");
}

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	LOG_INF("%s E! currefps:%d\n", __func__, currefps);

	LOG_INF("s5khmsp--- capture_setting  E\n");
	table_write_cmos_sensor(addr_data_pair_cap_hm2,
			sizeof(addr_data_pair_cap_hm2) / sizeof(kal_uint16));
	LOG_INF("s5khmsp--- capture_setting  X\n");
}

static void normal_video_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(addr_data_pair_video_hm2,
		sizeof(addr_data_pair_video_hm2) / sizeof(kal_uint16));
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(addr_data_pair_hs_hm2,
			sizeof(addr_data_pair_hs_hm2) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(addr_data_pair_slim_hm2,
		sizeof(addr_data_pair_slim_hm2) / sizeof(kal_uint16));
}

static void custom1_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(addr_data_pair_cus1_hm2,
		sizeof(addr_data_pair_cus1_hm2) / sizeof(kal_uint16));
}

static void custom2_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(addr_data_pair_cus2_hm2,
		sizeof(addr_data_pair_cus2_hm2) / sizeof(kal_uint16));
}

static void custom3_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(addr_data_pair_cus3_hm2,
		sizeof(addr_data_pair_cus3_hm2) / sizeof(kal_uint16));
}

static void custom4_setting(void)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(addr_data_pair_cus4_hm2,
		sizeof(addr_data_pair_cus4_hm2) / sizeof(kal_uint16));
}

#define S5KHM6D_CHANGZHENG_EEPROM_I2C_ADDR 0xA0
static BYTE s5khm6s_changzheng_common_data[CAMERA_EEPPROM_COMDATA_LENGTH] = { 0 };

static kal_uint16 read_s5khm6s_changzheng_eeprom_module(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, S5KHM6D_CHANGZHENG_EEPROM_I2C_ADDR);

	return get_byte;
}

static void read_s5khm6s_changzheng_module_data()
{
	kal_uint16 idx = 0;
	kal_uint16 sn_length = 17;
	kal_uint32 sn_starAddr = 0xB0;
	kal_uint32 vcmAddr = 0x0C;
	kal_uint32 lensAddr = 0x08;
	kal_uint32 macAddr_lsb = 0x92;
	kal_uint32 macAddr_msb = 0x93;
	kal_uint32 infAddr_lsb = 0x94;
	kal_uint32 infAddr_msb = 0x95;

	memset(s5khm6s_changzheng_common_data, 0,sizeof(s5khm6s_changzheng_common_data));
	// QR
	for(idx = 0; idx < sn_length; idx++)
	{
		s5khm6s_changzheng_common_data[8 + idx] = read_s5khm6s_changzheng_eeprom_module(sn_starAddr + idx);
	}
	//vcm
	s5khm6s_changzheng_common_data[40] = read_s5khm6s_changzheng_eeprom_module(vcmAddr);
	s5khm6s_changzheng_common_data[41] = read_s5khm6s_changzheng_eeprom_module(vcmAddr + 1);
	//lensid
	s5khm6s_changzheng_common_data[44] = read_s5khm6s_changzheng_eeprom_module(lensAddr);
	s5khm6s_changzheng_common_data[45] = read_s5khm6s_changzheng_eeprom_module(lensAddr + 1);
	//macro
	s5khm6s_changzheng_common_data[48] = read_s5khm6s_changzheng_eeprom_module(macAddr_lsb);
	s5khm6s_changzheng_common_data[49] = read_s5khm6s_changzheng_eeprom_module(macAddr_msb);
	//inf
	s5khm6s_changzheng_common_data[52] = read_s5khm6s_changzheng_eeprom_module(infAddr_lsb);
	s5khm6s_changzheng_common_data[53] = read_s5khm6s_changzheng_eeprom_module(infAddr_msb);

	for (idx = 0; idx < CAMERA_EEPPROM_COMDATA_LENGTH; idx = idx + 4)
		LOG_INF("cam data: %02x %02x %02x %02x\n",
		       s5khm6s_changzheng_common_data[idx],
		       s5khm6s_changzheng_common_data[idx + 1],
		       s5khm6s_changzheng_common_data[idx + 2],
		       s5khm6s_changzheng_common_data[idx + 3]);
}
static kal_uint32 return_sensor_id(void)
{
    kal_uint32 sensor_id = 0;
    kal_uint16 sensor_id_0 = read_cmos_sensor_8(0x0000);
    kal_uint16 sensor_id_1 = read_cmos_sensor_8(0x0001);

    LOG_INF("[%s] sensor_id: sensor_id_0:0x%x sensor_id_1:0x%x", __func__, sensor_id_0, sensor_id_1);
    sensor_id = ((sensor_id_0 << 8) & 0xFF00) | (sensor_id_1 & 0xFF);

	LOG_INF("[%s] sensor_id: 0x%x", __func__, sensor_id);

	return sensor_id;
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
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 5;
	printk("s5khm6sp get_imgsensor_id----1  0x%x\n",imgsensor_info.sensor_id);	//0x1ad6
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			//mdelay(2000);
			printk("s5khm6sp get_imgsensor_id----2  0x%x\n",*sensor_id);	//0x1ad6
			//*sensor_id = 0x1ad3;
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
						imgsensor.i2c_write_id, *sensor_id);
				imgsensor_info.module_id = 0x05;
				if (deviceInfo_register_value == 0x00) {
					register_imgsensor_deviceinfo("Cam_r", DEVICE_VERSION_CHANGZHENG_S5KHM6S, imgsensor_info.module_id);
					read_s5khm6s_changzheng_module_data();
					deviceInfo_register_value = 0x01;
				}
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n",
					imgsensor.i2c_write_id);
		    retry--;
            mdelay(2);
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
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
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	pr_debug("%s", __func__);
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n",
					imgsensor.i2c_write_id);
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
	sensor_init();
	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.jn1_binning_mode = imgsensor_info.pre.jn1_binning_mode;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}   /*  open  */

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
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

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
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	printk("s5khmsp---    preview");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.jn1_binning_mode = imgsensor_info.pre.jn1_binning_mode;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
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
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		LOG_INF("cap1-15fps: use cap1's setting: %d fps!\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
		LOG_INF("cap2-24fps: use cap2's setting: %d fps!\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else{
		LOG_INF("current_fps %d is not support,use cap1\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.jn1_binning_mode = imgsensor_info.normal_video.jn1_binning_mode;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.jn1_binning_mode = imgsensor_info.slim_video.jn1_binning_mode;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 * Custom1
 *
 * DESCRIPTION
 *   This function start the sensor Custom1.
 *
 * PARAMETERS
 *   *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.jn1_binning_mode = imgsensor_info.custom1.jn1_binning_mode;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.jn1_binning_mode = imgsensor_info.custom2.jn1_binning_mode;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.jn1_binning_mode = imgsensor_info.custom3.jn1_binning_mode;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 600; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom3_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	imgsensor.pclk = imgsensor_info.custom4.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom4.linelength;
	imgsensor.frame_length = imgsensor_info.custom4.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom4_setting();
	return ERROR_NONE;
}   /*  Custom4   */
#if 0
static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	imgsensor.pclk = imgsensor_info.custom5.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom5.linelength;
	imgsensor.frame_length = imgsensor_info.custom5.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}   /*  Custom5   */
#endif
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth
		= imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight
		= imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorPreviewWidth
		= imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight
		= imgsensor_info.pre.grabwindow_height;
	sensor_resolution->SensorVideoWidth
		= imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight
		= imgsensor_info.normal_video.grabwindow_height;
	sensor_resolution->SensorHighSpeedVideoWidth
		= imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight
		= imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth
		= imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight
		= imgsensor_info.slim_video.grabwindow_height;
	sensor_resolution->SensorCustom1Width
		= imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height
		= imgsensor_info.custom1.grabwindow_height;
	sensor_resolution->SensorCustom2Width
		= imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height
		= imgsensor_info.custom2.grabwindow_height;
	sensor_resolution->SensorCustom3Width
		= imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height
		= imgsensor_info.custom3.grabwindow_height;
	sensor_resolution->SensorCustom4Width
		= imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height
		= imgsensor_info.custom4.grabwindow_height;
	//sensor_resolution->SensorCustom5Width
	//	= imgsensor_info.custom5.grabwindow_width;
	//sensor_resolution->SensorCustom5Height
	//	= imgsensor_info.custom5.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat
		= imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	//sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame
		= imgsensor_info.ae_sensor_gain_delay_frame;
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

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	sensor_info->PDAF_Support = 2;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM2:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM4:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	break;
	//case MSDK_SCENARIO_ID_CUSTOM5:
	//    sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
	//    sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
	//    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
	//		= imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
	//break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount
			= imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}
	return ERROR_NONE;
}	/*	get_info  */

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
						MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d   %d\n", scenario_id, MSDK_SCENARIO_ID_CAMERA_PREVIEW);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	lastcamstate = camstate;
	camstate = scenario_id;
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
	    Custom1(image_window, sensor_config_data); // Custom1
	break;
	case MSDK_SCENARIO_ID_CUSTOM2:
	    Custom2(image_window, sensor_config_data); // Custom1
	break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    Custom3(image_window, sensor_config_data); // Custom1
	break;
	case MSDK_SCENARIO_ID_CUSTOM4:
	    Custom4(image_window, sensor_config_data); // Custom4
	break;
	//case MSDK_SCENARIO_ID_CUSTOM5:
	//    Custom5(image_window, sensor_config_data); // Custom1
	//		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10
			/ imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength)
			? (frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
					, framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10
				/ imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			  ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 /
			imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength)
			? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
			/ imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
			/ imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		// set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10
			/ imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength)
			? (frame_length - imgsensor_info.custom3.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10
			/ imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength)
			? (frame_length - imgsensor_info.custom4.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	#if 0
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk / framerate * 10
			/ imgsensor_info.custom5.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength)
			? (frame_length - imgsensor_info.custom5.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length
			= imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
		break;
	#endif
	default:  //coding with  preview scenario by default
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length
			= imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		LOG_INF("error scenario_id = %d, use preview\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
	    *framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
	    *framerate = imgsensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    *framerate = imgsensor_info.custom3.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
	    *framerate = imgsensor_info.custom4.max_framerate;
		break;
	//case MSDK_SCENARIO_ID_CUSTOM5:
	//    *framerate = imgsensor_info.custom5.max_framerate;
	//	break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 jn1_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	pr_debug("%s\n", __func__);

	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

	pr_debug("[%s] ABS_GAIN_GR:%d, grgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GR,	grgain_32);
	pr_debug("[%s] ABS_GAIN_R:%d, rgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_R, rgain_32);
	pr_debug("[%s] ABS_GAIN_B:%d, bgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_B, bgain_32);
	pr_debug("[%s] ABS_GAIN_GB:%d, gbgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GB,	gbgain_32);

	write_cmos_sensor(0x6028, 0x4000);

	write_cmos_sensor(0x0d82, (pSetSensorAWB->ABS_GAIN_R >> 1));

	write_cmos_sensor(0x0d84, (pSetSensorAWB->ABS_GAIN_GR >> 1));

	write_cmos_sensor(0x0d86, (pSetSensorAWB->ABS_GAIN_B >> 1));

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x0600, 0x0002);
	} else {
		write_cmos_sensor(0x0600, 0x0000);
	};
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(void)
{
	UINT8 temperature;
	INT32 temperature_convert;

	temperature = read_cmos_sensor_8(0x013a);

	if (temperature >= 0x0 && temperature <= 0x78)
		temperature_convert = temperature;
	else
		temperature_convert = -1;

	/*pr_info("temp_c(%d), read_reg(%d), enable %d\n",
	 *	temperature_convert, temperature, read_cmos_sensor_8(0x0138));
	 */

	return temperature_convert;
}
#define FOUR_CELL_XTALK_CAL_SIZE 2612
#define FOUR_CELL_XTALK_CAL_ADDR 0x1AA0
static char FOUR_CELL_XTALK_CAL[FOUR_CELL_XTALK_CAL_SIZE + 2];

#define FOUR_CELL_SENSOR_XTALK_CAL_SIZE  768
#define FOUR_CELL_SENSOR_XTALK_CAL_ADDR  0x2500
static char FOUR_CELL_SENSOR_XTALK_CAL[FOUR_CELL_SENSOR_XTALK_CAL_SIZE + 2];

#define FOUR_CELL_PD_XTALK_CAL_SIZE 4000
#define FOUR_CELL_PD_XTALK_CAL_ADDR 0x2810
static char FOUR_CELL_PD_XTALK_CAL[FOUR_CELL_PD_XTALK_CAL_SIZE + 2];

#define FOUR_CELL_SW_GGC_CAL_SIZE 626
#define FOUR_CELL_SW_GGC_CAL_ADDR 0x3860
static char FOUR_CELL_SW_GGC_CAL[FOUR_CELL_SW_GGC_CAL_SIZE + 2];

#define FOUR_CELL_REMOSAIC_CAL_SIZE (FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE + FOUR_CELL_PD_XTALK_CAL_SIZE + FOUR_CELL_SW_GGC_CAL_SIZE)

static void read_4cell_from_eeprom(char* src, int addr, int len)
{
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
	if (src[0] == 0 && src[1] == 0) {
		src[0] = (len & 0xff);
		src[1] = ((len >> 8) & 0xff);
		iReadRegI2C(pu_send_cmd, 2, &src[2], len, 0xA1);
	}
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
			     UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB =
		(struct SET_SENSOR_AWB_GAIN *) feature_para;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
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
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(feature_data + 2) = 2;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
		default:
			*(feature_data + 2) = 1;
			break;
		}
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		*(feature_data + 1) = 1;
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
#if 0
		pr_debug(
			"feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
#endif
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom4.framelength << 16)
				+ imgsensor_info.custom4.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.custom1.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.custom2.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.custom3.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.custom4.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.pre.pclk;
			break;
		}
		break;

	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	/* night_mode((BOOL) *feature_data); no need to implement this mode */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;

	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8(sensor_reg_data->RegAddr,
			sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_8(sensor_reg_data->RegAddr);
		break;

	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or
		 * just return LENS_DRIVER_ID_DO_NOT_CARE
		 */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) (*feature_data_16),
					*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
	    (enum MSDK_SCENARIO_ID_ENUM) *feature_data, *(feature_data + 1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
			  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) (*feature_data));
		break;

	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		pr_debug("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.hdr_mode = (UINT8)*feature_data_32;
		// imgsensor.hdr_mode = 9;
		// force set hdr_mode to zHDR
		spin_unlock(&imgsensor_drv_lock);
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		/* pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
		 *	(UINT32) *feature_data);
		 */

		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[5],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[6],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[7],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[8],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));

/* ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),
 * (UINT16)*(feature_data+2));
 */
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		jn1_awb_gain(pSetSensorAWB);
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
/* ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1)); */
		break;

	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	{

		int type = (kal_uint16)(*feature_data);
		char *data = (char *)(uintptr_t)(*(feature_data+1));

		if (type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
			pr_debug("Read remosaic Start");
			read_4cell_from_eeprom(FOUR_CELL_XTALK_CAL,
				FOUR_CELL_XTALK_CAL_ADDR, FOUR_CELL_XTALK_CAL_SIZE);
			/* for dump otp data
			for(i=0; i<FOUR_CELL_XTALK_CAL_SIZE; i++){
				printk("FOUR_CELL_CAL_TYPE_GAIN_TBL Read FOUR_CELL_XTALK_CAL[%d] = %02x \n", i, FOUR_CELL_XTALK_CAL[i] );
			}
			*/
			read_4cell_from_eeprom(FOUR_CELL_SENSOR_XTALK_CAL,
				FOUR_CELL_SENSOR_XTALK_CAL_ADDR, FOUR_CELL_SENSOR_XTALK_CAL_SIZE);
			read_4cell_from_eeprom(FOUR_CELL_PD_XTALK_CAL,
				FOUR_CELL_PD_XTALK_CAL_ADDR, FOUR_CELL_PD_XTALK_CAL_SIZE);
			read_4cell_from_eeprom(FOUR_CELL_SW_GGC_CAL,
				FOUR_CELL_SW_GGC_CAL_ADDR, FOUR_CELL_SW_GGC_CAL_SIZE);

			memcpy(data, FOUR_CELL_XTALK_CAL, FOUR_CELL_XTALK_CAL_SIZE+2);
			memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE), FOUR_CELL_SENSOR_XTALK_CAL+2, FOUR_CELL_SENSOR_XTALK_CAL_SIZE);
			memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE), FOUR_CELL_PD_XTALK_CAL+2, FOUR_CELL_PD_XTALK_CAL_SIZE);
			memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE + FOUR_CELL_PD_XTALK_CAL_SIZE), FOUR_CELL_SW_GGC_CAL+2, FOUR_CELL_SW_GGC_CAL_SIZE);

			/* for dump otp data
			for(i=0; i<FOUR_CELL_REMOSAIC_CAL_SIZE + 2; i++){
				printk("Read remosaic TYPE_GAIN_TBL data[%d] = %02x \n", i, (UINT16)data[i]);
			}
			*/
			pr_debug("Read remosaic = %02x %02x %02x %02x %02x %02x\n",
				(UINT16)data[0], (UINT16)data[1],
				(UINT16)data[2], (UINT16)data[3],
				(UINT16)data[4], (UINT16)data[5]);
		}
		if (type == FOUR_CELL_CAL_TYPE_ALL) {
			pr_debug("Read remosaic Start");
			read_4cell_from_eeprom(FOUR_CELL_XTALK_CAL,
				FOUR_CELL_XTALK_CAL_ADDR, FOUR_CELL_XTALK_CAL_SIZE);
			/* for dump otp data
			for(i=0; i<FOUR_CELL_XTALK_CAL_SIZE; i++){
				printk("FOUR_CELL_CAL_TYPE_ALL Read FOUR_CELL_XTALK_CAL[%d] = %02x \n", i, FOUR_CELL_XTALK_CAL[i] );
			}
			*/
			read_4cell_from_eeprom(FOUR_CELL_SENSOR_XTALK_CAL,
				FOUR_CELL_SENSOR_XTALK_CAL_ADDR, FOUR_CELL_SENSOR_XTALK_CAL_SIZE);
			read_4cell_from_eeprom(FOUR_CELL_PD_XTALK_CAL,
				FOUR_CELL_PD_XTALK_CAL_ADDR, FOUR_CELL_PD_XTALK_CAL_SIZE);
			read_4cell_from_eeprom(FOUR_CELL_SW_GGC_CAL,
				FOUR_CELL_SW_GGC_CAL_ADDR, FOUR_CELL_SW_GGC_CAL_SIZE);

			memcpy(data, FOUR_CELL_XTALK_CAL, FOUR_CELL_XTALK_CAL_SIZE+2);
			memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE), FOUR_CELL_SENSOR_XTALK_CAL+2, FOUR_CELL_SENSOR_XTALK_CAL_SIZE);
			memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE), FOUR_CELL_PD_XTALK_CAL+2, FOUR_CELL_PD_XTALK_CAL_SIZE);
			memcpy(data+(2 + FOUR_CELL_XTALK_CAL_SIZE + FOUR_CELL_SENSOR_XTALK_CAL_SIZE + FOUR_CELL_PD_XTALK_CAL_SIZE), FOUR_CELL_SW_GGC_CAL+2, FOUR_CELL_SW_GGC_CAL_SIZE);

			/* for dump otp data
			for(i=0; i<FOUR_CELL_REMOSAIC_CAL_SIZE + 2; i++){
				printk("Read remosaic type-all data[%d] = %02x \n", i, (UINT16)data[i]);
			}
			*/
			pr_debug("Read remosaic = %02x %02x %02x %02x %02x %02x\n",
				(UINT16)data[0], (UINT16)data[1],
				(UINT16)data[2], (UINT16)data[3],
				(UINT16)data[4], (UINT16)data[5]);
		}

		break;
	}

	case SENSOR_FEATURE_GET_PDAF_INFO:
		pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16)*feature_data);
		PDAFinfo =
			(struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM1://stereo
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW: //2x2 binning
			memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo,(void *)&imgsensor_normal_video_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: //full
		default:
			break;
		}
		break;

	case SENSOR_FEATURE_GET_VC_INFO:
		pr_debug("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
		pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		pr_debug("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT16)*feature_data);
		//PDAF capacity enable or not
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			//need to check
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_PDAF:
		pr_debug("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
		 /*
		 * case SENSOR_FEATURE_GET_PDAF_DATA: //get cal data from eeprom
		 * pr_debug("SENSOR_FEATURE_GET_PDAF_DATA\n");
		 * read_2T7_eeprom((kal_uint16 )(*feature_data),
		 * (char*)(uintptr_t)(*(feature_data+1)),
		 * (kal_uint32)(*(feature_data+2)));
		 * pr_debug("SENSOR_FEATURE_GET_PDAF_DATA success\n");
		 * break;
		 */
		/******************** PDAF END   <<< *********/
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16)(*feature_data),
						(UINT16)(*(feature_data + 1)),(BOOL)(*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = get_sensor_temperature();
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.cap.pclk /
			(imgsensor_info.cap.linelength - 80))*
			imgsensor_info.cap.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.normal_video.pclk /
			(imgsensor_info.normal_video.linelength - 80))*
			imgsensor_info.normal_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.hs_video.pclk /
			(imgsensor_info.hs_video.linelength - 80))*
			imgsensor_info.hs_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.slim_video.pclk /
			(imgsensor_info.slim_video.linelength - 80))*
			imgsensor_info.slim_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom1.pclk /
			(imgsensor_info.custom1.linelength - 80))*
			imgsensor_info.custom1.grabwindow_width;

			break;

		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom2.pclk /
			(imgsensor_info.custom2.linelength - 80))*
			imgsensor_info.custom2.grabwindow_width;

			break;

		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom3.pclk /
			(imgsensor_info.custom3.linelength - 80))*
			imgsensor_info.custom3.grabwindow_width;

			break;

		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom4.pclk /
			(imgsensor_info.custom4.linelength - 80))*
			imgsensor_info.custom4.grabwindow_width;

			break;

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.pre.pclk /
			(imgsensor_info.pre.linelength - 80))*
			imgsensor_info.pre.grabwindow_width;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	    LOG_INF("SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE\n");
		memcpy(feature_return_para_32,
		&imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		LOG_INF("SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE\n");
		*feature_return_para_32 =  imgsensor.current_ae_effective_frame;
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom3.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom4.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
//#ifdef OPLUS_FEATURE_CAMERA_COMMON
	case SENSOR_FEATURE_GET_MODULE_INFO:
		break;
	case SENSOR_FEATURE_GET_MODULE_SN:
		break;
	case SENSOR_FEATURE_SET_SENSOR_OTP:
		break;
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		break;
	case SENSOR_FEATURE_GET_EEPROM_COMDATA:
		memcpy(feature_return_para_32, s5khm6s_changzheng_common_data,
				CAMERA_EEPPROM_COMDATA_LENGTH);
		*feature_para_len = CAMERA_EEPPROM_COMDATA_LENGTH;
		break;
	case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
		break;
	case SENSOR_FEATURE_GET_DISTORTIONPARAMS:
		break;
//#endif /* OPLUS_FEATURE_CAMERA_COMMON */
	default:
		break;
	}

	return ERROR_NONE;
}				

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5KHM6S_MIPI_RAW_CHANGZHENG_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	/* S5KHM6SP_MIPI_RAW22693_SensorInit */
