// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX355mipi_Sensor.c
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

#include "imx355_eeprom_21143.h"
#include "imx355_ana_gain_table_21143.h"
#include "imx355mipiraw_Sensor_21143.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor_16(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define write_cmos_sensor_16(...) subdrv_i2c_wr_u16(__VA_ARGS__)
#define read_cmos_sensor(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define write_cmos_sensor(...) subdrv_i2c_wr_u8(__VA_ARGS__)

#define imx355_table_write_cmos_sensor_21143(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)

/************************Modify Following Strings for Debug********************/
#define PFX "IMX355_camera_sensor_21143"
#define LOG_1 LOG_INF("IMX355,MIPI 4LANE\n")
/************************   Modify end    *************************************/

#undef IMX355_24_FPS

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

#define H_FOV 63
#define V_FOV 49

#define IMX355_EEPROM_SLAVE_ADDRESS 0xA2
extern Eeprom_DistortionParamsRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr);

static DEFINE_SPINLOCK(imgsensor_drv_lock);
#define OTP_SIZE    0x4000
static kal_uint8 otp_data[OTP_SIZE] = {0};

#define stereo_start_add    0x2E00
static kal_uint8 stereo_data[CALI_DATA_SLAVE_LENGTH] = {0};

/*******************************************************************************
 * Proifling
 *****************************************************************************/
#define PROFILE 0
#if PROFILE
static struct timeval tv1, tv2;
static DEFINE_SPINLOCK(kdsensor_drv_lock);
/****************************************************************************
 *
 *****************************************************************************/
static void KD_SENSOR_PROFILE_INIT(struct subdrv_ctx *ctx)
{
    do_gettimeofday(&tv1);
}

/****************************************************************************
 *
 ****************************************************************************/
static void KD_SENSOR_PROFILE(struct subdrv_ctx *ctx, char *tag)
{
    unsigned long TimeIntervalUS;

    spin_lock(&kdsensor_drv_lock);

    do_gettimeofday(&tv2);
    TimeIntervalUS =
      (tv2.tv_sec - tv1.tv_sec) * 1000000 + (tv2.tv_usec - tv1.tv_usec);
    tv1 = tv2;

    spin_unlock(&kdsensor_drv_lock);
    LOG_INF("[%s]Profile = %lu us\n", tag, TimeIntervalUS);
}
#else
static void KD_SENSOR_PROFILE_INIT(struct subdrv_ctx *ctx)
{
}

static void KD_SENSOR_PROFILE(struct subdrv_ctx *ctx, char *tag)
{
}
#endif


#define BYTE               unsigned char

/* static BOOL read_spc_flag = FALSE; */

/*support ZHDR*/
/* #define imx355_ZHDR */

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX355_SENSOR_ID_21143,
    .checksum_value = 0xD1EFF68B,
    .pre = {
        .pclk = 280800000,    /* record different mode's pclk */
        .linelength = 3672,    /* record different mode's linelength */
        .framelength = 2548, /* record different mode's framelength */
        .startx = 0, /* record different mode's startx of grabwindow */
        .starty = 0, /* record different mode's starty of grabwindow */

        /* record different mode's width of grabwindow */
        .grabwindow_width = 3264,
        /* record different mode's height of grabwindow */
        .grabwindow_height = 2448,

        /* following for MIPIDataLowPwr2HighSpeedSettleDelayCount
         * by different scenario
         */
        .mipi_data_lp2hs_settle_dc = 85,    /* unit , ns */
        .mipi_pixel_rate = 280800000,
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 280800000,
        .linelength = 3672,
        .framelength = 2548,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,    /* unit , ns */
        .mipi_pixel_rate = 280800000,
        .max_framerate = 300,
    },
    .normal_video = {/*3264*1840@30 Tsh:23ms*/
        .pclk = 280800000,
        .linelength = 3672,
        .framelength = 2548,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1840,
        .mipi_pixel_rate = 280800000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 288000000,
        .linelength = 1836,
        .framelength = 870,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1408,
        .grabwindow_height = 792,
        .mipi_pixel_rate = 288000000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 1800,
    },
    .slim_video = {
        .pclk = 280800000,
        .linelength = 3672,
        .framelength = 2548 ,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_pixel_rate = 280800000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .custom1 = {
        .pclk = 201600000,
        .linelength = 3672,
        .framelength = 2286,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2304,
        .grabwindow_height = 1728,
        .mipi_pixel_rate = 201600000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 240,
    },
    .custom2 = {
        .pclk = 144000000,
        .linelength = 1836,
        .framelength = 2614,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1640,
        .grabwindow_height = 1232,
        .mipi_pixel_rate = 144000000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .custom3 = {
        .pclk = 280800000,
        .linelength = 3672,
        .framelength = 2548 ,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_pixel_rate = 280800000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .custom4 = {
        .pclk = 211200000,
        .linelength = 3672,
        .framelength = 1916,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1840,
        .mipi_pixel_rate = 211200000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
     },
    .custom5 = {
        .pclk = 144000000,
        .linelength = 1836,
        .framelength = 2614,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1640,
        .grabwindow_height = 1232,
        .mipi_pixel_rate = 144000000,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
     },
    .margin = 10,        /* sensor framelength & shutter margin */
    .min_shutter = 1,    /* min shutter */
    .min_gain = BASEGAIN * 1, /*1x gain*/
    .max_gain = BASEGAIN * 16, /*16x gain*/
    .min_gain_iso = 100,
    .gain_step = 1,
    .gain_type = 0,

    /* max framelength by sensor register's limitation */
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    /* shutter delay frame for AE cycle,
     * 2 frame with ispGain_delay-shut_delay=2-0=2
     */
    .ae_sensor_gain_delay_frame = 0,

    /* sensor gain delay frame for AE cycle,
     * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
     */
    .ae_ispGain_delay_frame = 2,    /* isp gain delay frame for AE cycle */
    .ihdr_support = 0,    /* 1, support; 0,not support */
    .ihdr_le_firstline = 0,    /* 1,le first ; 0, se first */
    .temperature_support = 1,    /* 1, support; 0,not support */
    .sensor_mode_num = 10,    /* support sensor mode num */
    .frame_time_delay_frame = 3,
    .cap_delay_frame = 2,    /* enter capture delay frame num */
    .pre_delay_frame = 4,    /* enter preview delay frame num */
    .video_delay_frame = 2,    /* enter video delay frame num */
    .hs_video_delay_frame = 2, /* enter high speed video  delay frame num */
    .slim_video_delay_frame = 2,
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,
    .custom5_delay_frame = 2,
    .isp_driving_current = ISP_DRIVING_8MA,    /* mclk driving current */

    /* sensor_interface_type */
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

    /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
    .mipi_sensor_type = MIPI_OPHY_NCSI2,
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
    /* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */

    /* sensor output first pixel color */
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
    .mclk = 24,    /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
    .mipi_lane_num = SENSOR_MIPI_4_LANE,    /* mipi lane num */
    .i2c_addr_table = {0x34, 0xff},
/* record sensor support all write id addr, only supprt 4must end with 0xff */
    .i2c_speed = 1000,    /* i2c read/write speed */
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
    {3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//pre
    {3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//cap
    {3280, 2464,   0, 312,   3280, 1840, 3280, 1840,   8,   0, 3264, 1840, 0, 0, 3264, 1840},//video
    {3280, 2464,   0,   0,   3280, 2464, 1640, 1232, 116, 220, 1408,  792, 0, 0, 1408,  792},//hs
    {3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//slim
    {3280, 2464,   0, 368,   3280, 1728, 3280, 1728, 488,   0, 2304, 1728, 0, 0, 2304, 1728},//custom1
    {3280, 2464,   0,   0,   3280, 2464, 1640, 1232,   0,   0, 1640, 1232, 0, 0, 1640, 1232},//custom2
    {3280, 2464,   8,   8,   3264, 2448, 3264, 2448,   0,   0, 3264, 2448, 0, 0, 3264, 2448},//custom3
    {3280, 2464,   8, 312,   3264, 1840, 3264, 1840,   0,   0, 3264, 1840, 0, 0, 3264, 1840},//custom4
    {3280, 2464,   0,   0,   3280, 2464, 1640, 1232,   0,   0, 1640, 1232, 0, 0, 1640, 1232},//custom5
};

static void set_dummy(struct subdrv_ctx *ctx)
{
    LOG_INF("dummyline = %d, dummypixels = %d\n",
              ctx->dummy_line, ctx->dummy_pixel);
    write_cmos_sensor_8(ctx, 0x0350, 0x00); /* Disable auto extend */
    write_cmos_sensor(ctx, 0x0104, 0x01);
    write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
    write_cmos_sensor(ctx, 0x0341, ctx->frame_length & 0xFF);
    write_cmos_sensor(ctx, 0x0342, ctx->line_length >> 8);
    write_cmos_sensor(ctx, 0x0343, ctx->line_length & 0xFF);
    write_cmos_sensor(ctx, 0x0104, 0x00);
}                /*    set_dummy  */

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

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
    kal_uint32 frame_length = ctx->frame_length;
    /* unsigned long flags; */

    LOG_INF("framerate = %d, min framelength should enable %d\n",
            framerate,
            min_framelength_en);

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
    set_dummy(ctx);
}                /*    set_max_framerate  */

/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
#define MAX_CIT_LSHIFT 7
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
    kal_uint16 l_shift = 1;
    kal_uint16 realtime_fps = 0;
    LOG_INF("Enter! shutter =%d, framelength =%d\n", shutter, ctx->frame_length);
    ctx->shutter = shutter;
    /* write_shutter(shutter); */
    /* 0x3500, 0x3501, 0x3502 will increase VBLANK
     * to get exposure larger than frame exposure
     */
    /* AE doesn't update sensor gain at capture mode,
     * thus extra exposure lines must be updated here.
     */
    spin_lock(&imgsensor_drv_lock);
    if (shutter > ctx->min_frame_length - imgsensor_info.margin)
        ctx->frame_length = shutter + imgsensor_info.margin;
    else
        ctx->frame_length = ctx->min_frame_length;
    if (ctx->frame_length > imgsensor_info.max_frame_length)
        ctx->frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);

    shutter = (shutter < imgsensor_info.min_shutter)
        ? imgsensor_info.min_shutter : shutter;

    if (ctx->autoflicker_en) {
        realtime_fps = ctx->pclk /
            ctx->line_length * 10 / ctx->frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(ctx, 296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(ctx, 146, 0);
        else {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
            write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
    }

    /* long expsoure */
    if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
        for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
            if ((shutter >> l_shift) <
            (imgsensor_info.max_frame_length - imgsensor_info.margin))
                break;
        }
        if (l_shift > MAX_CIT_LSHIFT) {
            LOG_INF("Unable to set such a long exposure %d, set to max\n",
                shutter);
            l_shift = MAX_CIT_LSHIFT;
        }
        shutter = shutter >> l_shift;
        /* imgsensor_info.max_frame_length; */
        ctx->frame_length = shutter + imgsensor_info.margin;

        write_cmos_sensor_8(ctx, 0x0104, 0x01);
        write_cmos_sensor_8(ctx, 0x3060, l_shift & 0x7);
        write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
        write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
        write_cmos_sensor_8(ctx, 0x0104, 0x00);
/* Frame exposure mode customization for LE*/
        ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
        ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
        ctx->current_ae_effective_frame = 2;
    } else {
        write_cmos_sensor_8(ctx, 0x0104, 0x01);
        write_cmos_sensor_8(ctx, 0x3060, read_cmos_sensor_8(ctx, 0x3060) & 0xf8);
        write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
        write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
        write_cmos_sensor_8(ctx, 0x0104, 0x00);
        ctx->current_ae_effective_frame = 2;
    }

    /* Update Shutter */
    write_cmos_sensor(ctx, 0x0104, 0x01);
    write_cmos_sensor(ctx, 0x0350, 0x01); /* enable auto extend */
    write_cmos_sensor(ctx, 0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor(ctx, 0x0203, shutter & 0xFF);
    write_cmos_sensor(ctx, 0x0104, 0x00);
    LOG_INF("Exit! shutter =%d, framelength =%d\n",
        shutter, ctx->frame_length);


} /*    set_shutter */

static void set_shutter_frame_length(struct subdrv_ctx *ctx,
            kal_uint16 shutter, kal_uint16 frame_length,
            kal_bool auto_extend_en)
{    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;

    ctx->shutter = shutter;
    /* LOG_INF("shutter =%d, frame_time =%d\n", shutter, frame_time); */

    /* 0x3500, 0x3501, 0x3502 will increase VBLANK
     * to get exposure larger than frame exposure
     */
    /* AE doesn't update sensor gain at capture mode,
     * thus extra exposure lines must be updated here.
     */

    /* OV Recommend Solution */
/* if shutter bigger than frame_length, should extend frame length first */
    /*Change frame time */
    if (frame_length > 1)
        dummy_line = frame_length - ctx->frame_length;
    ctx->frame_length = ctx->frame_length + dummy_line;

    /*  */
    if (shutter > ctx->frame_length - imgsensor_info.margin)
        ctx->frame_length = shutter + imgsensor_info.margin;

    if (ctx->frame_length > imgsensor_info.max_frame_length)
        ctx->frame_length = imgsensor_info.max_frame_length;
    shutter = (shutter < imgsensor_info.min_shutter)
        ? imgsensor_info.min_shutter : shutter;
    shutter =
    (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
    ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if (ctx->autoflicker_en) {
        realtime_fps = ctx->pclk /
            ctx->line_length * 10 / ctx->frame_length;

        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(ctx, 296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(ctx, 146, 0);
        else {
            /* Extend frame length */
            write_cmos_sensor(ctx, 0x0104, 0x01);
            write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
            write_cmos_sensor(ctx, 0x0341,
                ctx->frame_length & 0xFF);
            write_cmos_sensor(ctx, 0x0104, 0x00);
        }
    } else {
        /* Extend frame length */
        write_cmos_sensor(ctx, 0x0104, 0x01);
        write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
        write_cmos_sensor(ctx, 0x0341, ctx->frame_length & 0xFF);
        write_cmos_sensor(ctx, 0x0104, 0x00);
    }

    /* Update Shutter */
    write_cmos_sensor(ctx, 0x0104, 0x01);
    if (auto_extend_en)
        write_cmos_sensor(ctx, 0x0350, 0x01); /* Enable auto extend */
    else
        write_cmos_sensor(ctx, 0x0350, 0x00); /* Disable auto extend */
    write_cmos_sensor(ctx, 0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor(ctx, 0x0203, shutter & 0xFF);
    write_cmos_sensor(ctx, 0x0104, 0x00);

    LOG_INF(
        "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
        shutter,
        ctx->frame_length, frame_length,
        dummy_line, read_cmos_sensor(ctx, 0x0350));
}            /* set_shutter_frame_length */

static void set_frame_length(struct subdrv_ctx *ctx, kal_uint16 frame_length)
{
    if (frame_length > 1)
        ctx->frame_length = frame_length;
    if (ctx->frame_length > imgsensor_info.max_frame_length)
        ctx->frame_length = imgsensor_info.max_frame_length;
    if (ctx->min_frame_length > ctx->frame_length)
        ctx->frame_length = ctx->min_frame_length;
    /* Extend frame length */
    write_cmos_sensor(ctx, 0x0104, 0x01);
    write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
    write_cmos_sensor(ctx, 0x0341, ctx->frame_length & 0xFF);
    write_cmos_sensor(ctx, 0x0104, 0x00);

    LOG_INF("Framelength: set=%d/input=%d/min=%d\n",
        ctx->frame_length, frame_length, ctx->min_frame_length);
}

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
        write_cmos_sensor_8(ctx, 0x0104, 0x01);
        write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
        write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
        write_cmos_sensor_8(ctx, 0x0202, (shutters[0] >> 8) & 0xFF);
        write_cmos_sensor_8(ctx, 0x0203, shutters[0] & 0xFF);
        write_cmos_sensor_8(ctx, 0x0104, 0x00);//grouphold end

        LOG_INF("shutters[0] =%d, framelength =%d\n",
            shutters[0], ctx->frame_length);
    }
}

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0;
    reg_gain = 1024 - (1024*BASEGAIN)/gain;
    return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
    kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    /*  */
    if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
        LOG_INF("Error gain setting");

        if (gain < imgsensor_info.min_gain)
            gain = imgsensor_info.min_gain;
        else
            gain = imgsensor_info.max_gain;
    }


    reg_gain = gain2reg(ctx, gain);
    ctx->gain = reg_gain;
    LOG_INF("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

    write_cmos_sensor(ctx, 0x0104, 0x01);
    /* Global analog Gain for Long expo */
    write_cmos_sensor(ctx, 0x0204, (reg_gain >> 8) & 0xFF);
    write_cmos_sensor(ctx, 0x0205, reg_gain & 0xFF);
    write_cmos_sensor_8(ctx, 0x0104, 0x00);

    return gain;
}                /*    set_gain  */

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765    /* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
#endif


kal_uint16 addr_data_pair_init_imx355_21143[] = {
    0x0106, 0x01,
    0x0136, 0x18,
    0x0137, 0x00,
    0x4348, 0x16,
    0x4350, 0x19,
    0x4408, 0x0A,
    0x440C, 0x0B,
    0x4411, 0x5F,
    0x4412, 0x2C,
    0x4623, 0x00,
    0x462C, 0x0F,
    0x462D, 0x00,
    0x462E, 0x00,
    0x4684, 0x54,
    0x480A, 0x07,
    0x4908, 0x07,
    0x4909, 0x07,
    0x490D, 0x0A,
    0x491E, 0x0F,
    0x4921, 0x06,
    0x4923, 0x28,
    0x4924, 0x28,
    0x4925, 0x29,
    0x4926, 0x29,
    0x4927, 0x1F,
    0x4928, 0x20,
    0x4929, 0x20,
    0x492A, 0x20,
    0x492C, 0x05,
    0x492D, 0x06,
    0x492E, 0x06,
    0x492F, 0x06,
    0x4930, 0x03,
    0x4931, 0x04,
    0x4932, 0x04,
    0x4933, 0x05,
    0x595E, 0x01,
    0x5963, 0x01,
};

static void sensor_init(struct subdrv_ctx *ctx)
{
    LOG_INF("E\n");
    imx355_table_write_cmos_sensor_21143(ctx, addr_data_pair_init_imx355_21143,
        sizeof(addr_data_pair_init_imx355_21143)/sizeof(kal_uint16));
}    /*    sensor_init  */

kal_uint16 addr_data_pair_preview_imx355_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0E,
    0x0343, 0x58,
    0x0340, 0x09,
    0x0341, 0xF4,
    0x0344, 0x00,
    0x0345, 0x08,
    0x0346, 0x00,
    0x0347, 0x08,
    0x0348, 0x0C,
    0x0349, 0xC7,
    0x034A, 0x09,
    0x034B, 0x97,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x75,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x0A,
    0x0821, 0xF8,
    0x3088, 0x04,
    0x6813, 0x02,
    0x6835, 0x07,
    0x6836, 0x00,
    0x6837, 0x04,
    0x684D, 0x07,
    0x684E, 0x00,
    0x684F, 0x04,
    0x0202, 0x09,
    0x0203, 0xEA,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0xAF,
    0x080C, 0x00,
    0x080D, 0x2F,
    0x080E, 0x00,
    0x080F, 0x57,
    0x0810, 0x00,
    0x0811, 0x2F,
    0x0812, 0x00,
    0x0813, 0x2F,
    0x0814, 0x00,
    0x0815, 0x2F,
    0x0816, 0x00,
    0x0817, 0xBF,
    0x0818, 0x00,
    0x0819, 0x27,
    0x30A2, 0x00,
    0x30A3, 0xE3,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static void preview_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("E %s.\n", __func__);
    imx355_table_write_cmos_sensor_21143(ctx, addr_data_pair_preview_imx355_21143,
    sizeof(addr_data_pair_preview_imx355_21143) / sizeof(kal_uint16));
    /* zvhdr_setting(ctx); */
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}                /*    preview_setting  */

kal_uint16 addr_data_pair_capture_imx355_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0E,
    0x0343, 0x58,
    0x0340, 0x09,
    0x0341, 0xF4,
    0x0344, 0x00,
    0x0345, 0x08,
    0x0346, 0x00,
    0x0347, 0x08,
    0x0348, 0x0C,
    0x0349, 0xC7,
    0x034A, 0x09,
    0x034B, 0x97,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x75,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x0A,
    0x0821, 0xF8,
    0x3088, 0x04,
    0x6813, 0x02,
    0x6835, 0x07,
    0x6836, 0x00,
    0x6837, 0x04,
    0x684D, 0x07,
    0x684E, 0x00,
    0x684F, 0x04,
    0x0202, 0x09,
    0x0203, 0xEA,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0xAF,
    0x080C, 0x00,
    0x080D, 0x2F,
    0x080E, 0x00,
    0x080F, 0x57,
    0x0810, 0x00,
    0x0811, 0x2F,
    0x0812, 0x00,
    0x0813, 0x2F,
    0x0814, 0x00,
    0x0815, 0x2F,
    0x0816, 0x00,
    0x0817, 0xBF,
    0x0818, 0x00,
    0x0819, 0x27,
    0x30A2, 0x00,
    0x30A3, 0xE3,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
    int timeout = 100;
    int i = 0;
    int framecnt = 0;
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
    if (enable) {
        write_cmos_sensor(ctx, 0x0100, 0x01);
        mdelay(10);
    } else {
        write_cmos_sensor(ctx, 0x0100, 0x00);
        for (i = 0; i < timeout; i++) {
            mdelay(5);
            framecnt = read_cmos_sensor(ctx, 0x0005);
            LOG_INF("framecnt = %d.\n", framecnt);
            if (framecnt == 0xFF) {
                LOG_INF(" Stream Off OK at i=%d.\n", i);
                return ERROR_NONE;
            }
        }
        LOG_INF("Stream Off Fail! framecnt=%d.\n", framecnt);
    }
    return ERROR_NONE;
}

static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d hdr:%d pdaf:%d\n",
        currefps, ctx->hdr_mode, ctx->pdaf_mode);

    imx355_table_write_cmos_sensor_21143(ctx, addr_data_pair_capture_imx355_21143,
    sizeof(addr_data_pair_capture_imx355_21143) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}

kal_uint16 addr_data_pair_video_imx355_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0E,
    0x0343, 0x58,
    0x0340, 0x09,
    0x0341, 0xF4,
    0x0344, 0x00,
    0x0345, 0x08,
    0x0346, 0x01,
    0x0347, 0x38,
    0x0348, 0x0C,
    0x0349, 0xC7,
    0x034A, 0x08,
    0x034B, 0x67,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x30,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x75,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x0A,
    0x0821, 0xF8,
    0x3088, 0x04,
    0x6813, 0x02,
    0x6835, 0x07,
    0x6836, 0x00,
    0x6837, 0x04,
    0x684D, 0x07,
    0x684E, 0x00,
    0x684F, 0x04,
    0x0202, 0x09,
    0x0203, 0xEA,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0xAF,
    0x080C, 0x00,
    0x080D, 0x2F,
    0x080E, 0x00,
    0x080F, 0x57,
    0x0810, 0x00,
    0x0811, 0x2F,
    0x0812, 0x00,
    0x0813, 0x2F,
    0x0814, 0x00,
    0x0815, 0x2F,
    0x0816, 0x00,
    0x0817, 0xBF,
    0x0818, 0x00,
    0x0819, 0x27,
    0x30A2, 0x00,
    0x30A3, 0xE3,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
    LOG_INF("E! %s:%d\n", __func__, currefps);

    imx355_table_write_cmos_sensor_21143(ctx, addr_data_pair_video_imx355_21143,
    sizeof(addr_data_pair_video_imx355_21143) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}

kal_uint16 addr_data_pair_hs_video_imx355_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x07,
    0x0343, 0x2C,
    0x0340, 0x03,
    0x0341, 0x66,
    0x0344, 0x00,
    0x0345, 0xE8,
    0x0346, 0x01,
    0x0347, 0xB8,
    0x0348, 0x0B,
    0x0349, 0xE7,
    0x034A, 0x07,
    0x034B, 0xE7,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x034C, 0x05,
    0x034D, 0x80,
    0x034E, 0x03,
    0x034F, 0x18,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x78,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x0B,
    0x0821, 0x40,
    0x3088, 0x04,
    0x6813, 0x02,
    0x6835, 0x07,
    0x6836, 0x00,
    0x6837, 0x04,
    0x684D, 0x07,
    0x684E, 0x00,
    0x684F, 0x04,
    0x0202, 0x03,
    0x0203, 0x5C,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0x6F,
    0x080C, 0x00,
    0x080D, 0x2F,
    0x080E, 0x00,
    0x080F, 0x57,
    0x0810, 0x00,
    0x0811, 0xDF,
    0x0812, 0x00,
    0x0813, 0x2F,
    0x0814, 0x00,
    0x0815, 0x2F,
    0x0816, 0x00,
    0x0817, 0xBF,
    0x0818, 0x00,
    0x0819, 0x27,
    0x30A2, 0x00,
    0x30A3, 0x4F,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static void hs_video_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("E %s.\n", __func__);
    imx355_table_write_cmos_sensor_21143(ctx, addr_data_pair_hs_video_imx355_21143,
    sizeof(addr_data_pair_hs_video_imx355_21143) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_uint8 modes, struct SET_SENSOR_PATTERN_SOLID_COLOR *pTestpatterndata)
{
    kal_uint16 Color_R, Color_Gr, Color_Gb, Color_B;
    pr_debug("set_test_pattern enum: %d\n", modes);

    if (modes) {
        write_cmos_sensor_8(ctx,0x0600, modes>>4);
        write_cmos_sensor_8(ctx,0x0601, modes);
        if (modes == 1 && (pTestpatterndata != NULL)) { //Solid Color
            Color_R = (pTestpatterndata->COLOR_R >> 16) & 0xFFFF;
            Color_Gr = (pTestpatterndata->COLOR_Gr >> 16) & 0xFFFF;
            Color_B = (pTestpatterndata->COLOR_B >> 16) & 0xFFFF;
            Color_Gb = (pTestpatterndata->COLOR_Gb >> 16) & 0xFFFF;
            write_cmos_sensor_8(ctx,0x0602, Color_R >> 8);
            write_cmos_sensor_8(ctx,0x0603, Color_R & 0xFF);
            write_cmos_sensor_8(ctx,0x0604, Color_Gr >> 8);
            write_cmos_sensor_8(ctx,0x0605, Color_Gr & 0xFF);
            write_cmos_sensor_8(ctx,0x0606, Color_B >> 8);
            write_cmos_sensor_8(ctx,0x0607, Color_B & 0xFF);
            write_cmos_sensor_8(ctx,0x0608, Color_Gb >> 8);
            write_cmos_sensor_8(ctx,0x0609, Color_Gb & 0xFF);
        }
    } else {
        write_cmos_sensor(ctx,0x0600, 0x0000); /*No pattern*/
        write_cmos_sensor(ctx,0x0601, 0x0000);
    }

    ctx->test_pattern = modes;
    return ERROR_NONE;
}

static kal_uint16 slim_video_setting_array_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0E,
    0x0343, 0x58,
    0x0340, 0x09,
    0x0341, 0xF4,
    0x0344, 0x00,
    0x0345, 0x08,
    0x0346, 0x00,
    0x0347, 0x08,
    0x0348, 0x0C,
    0x0349, 0xC7,
    0x034A, 0x09,
    0x034B, 0x97,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x75,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x0A,
    0x0821, 0xF8,
    0x3088, 0x04,
    0x6813, 0x02,
    0x6835, 0x07,
    0x6836, 0x00,
    0x6837, 0x04,
    0x684D, 0x07,
    0x684E, 0x00,
    0x684F, 0x04,
    0x0202, 0x09,
    0x0203, 0xEA,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0xAF,
    0x080C, 0x00,
    0x080D, 0x2F,
    0x080E, 0x00,
    0x080F, 0x57,
    0x0810, 0x00,
    0x0811, 0x2F,
    0x0812, 0x00,
    0x0813, 0x2F,
    0x0814, 0x00,
    0x0815, 0x2F,
    0x0816, 0x00,
    0x0817, 0xBF,
    0x0818, 0x00,
    0x0819, 0x27,
    0x30A2, 0x00,
    0x30A3, 0xE3,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static void slim_video_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("E %s.\n", __func__);
    imx355_table_write_cmos_sensor_21143(ctx, slim_video_setting_array_21143,
        sizeof(slim_video_setting_array_21143) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}

static kal_uint16 custom1_setting_array_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0E,
    0x0343, 0x58,
    0x0340, 0x08,
    0x0341, 0xEE,
    0x0344, 0x01,
    0x0345, 0xE8,
    0x0346, 0x01,
    0x0347, 0x70,
    0x0348, 0x0A,
    0x0349, 0xE7,
    0x034A, 0x08,
    0x034B, 0x2F,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x034C, 0x09,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0xC0,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x54,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x07,
    0x0821, 0xE0,
    0x3088, 0x04,
    0x6813, 0x01,
    0x6835, 0x00,
    0x6836, 0x00,
    0x6837, 0x02,
    0x684D, 0x00,
    0x684E, 0x00,
    0x684F, 0x02,
    0x0202, 0x08,
    0x0203, 0xE4,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0xB7,
    0x080C, 0x00,
    0x080D, 0x27,
    0x080E, 0x00,
    0x080F, 0x3F,
    0x0810, 0x00,
    0x0811, 0x27,
    0x0812, 0x00,
    0x0813, 0x1F,
    0x0814, 0x00,
    0x0815, 0x1F,
    0x0816, 0x00,
    0x0817, 0x8F,
    0x0818, 0x00,
    0x0819, 0x1F,
    0x30A2, 0x00,
    0x30A3, 0xDD,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static void custom1_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("E %s.\n", __func__);
    imx355_table_write_cmos_sensor_21143(ctx, custom1_setting_array_21143,
        sizeof(custom1_setting_array_21143) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}

static kal_uint16 custom2_setting_array_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x07,
    0x0343, 0x2C,
    0x0340, 0x0A,
    0x0341, 0x36,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x0C,
    0x0349, 0xCF,
    0x034A, 0x09,
    0x034B, 0x9F,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x034C, 0x06,
    0x034D, 0x68,
    0x034E, 0x04,
    0x034F, 0xD0,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x3C,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x05,
    0x0821, 0xA0,
    0x3088, 0x02,
    0x6813, 0x01,
    0x6835, 0x00,
    0x6836, 0x00,
    0x6837, 0x02,
    0x684D, 0x00,
    0x684E, 0x00,
    0x684F, 0x02,
    0x0202, 0x0A,
    0x0203, 0x2C,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0x57,
    0x080C, 0x00,
    0x080D, 0x1F,
    0x080E, 0x00,
    0x080F, 0x2F,
    0x0810, 0x00,
    0x0811, 0xDF,
    0x0812, 0x00,
    0x0813, 0x17,
    0x0814, 0x00,
    0x0815, 0x17,
    0x0816, 0x00,
    0x0817, 0x6F,
    0x0818, 0x00,
    0x0819, 0x17,
    0x30A2, 0x00,
    0x30A3, 0x2F,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static void custom2_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("E %s.\n", __func__);
    imx355_table_write_cmos_sensor_21143(ctx, custom2_setting_array_21143,
        sizeof(custom2_setting_array_21143) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}

static kal_uint16 custom3_setting_array_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0E,
    0x0343, 0x58,
    0x0340, 0x09,
    0x0341, 0xF4,
    0x0344, 0x00,
    0x0345, 0x08,
    0x0346, 0x00,
    0x0347, 0x08,
    0x0348, 0x0C,
    0x0349, 0xC7,
    0x034A, 0x09,
    0x034B, 0x97,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x75,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x0A,
    0x0821, 0xF8,
    0x3088, 0x04,
    0x6813, 0x02,
    0x6835, 0x07,
    0x6836, 0x00,
    0x6837, 0x04,
    0x684D, 0x07,
    0x684E, 0x00,
    0x684F, 0x04,
    0x0202, 0x09,
    0x0203, 0xEA,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0xAF,
    0x080C, 0x00,
    0x080D, 0x2F,
    0x080E, 0x00,
    0x080F, 0x57,
    0x0810, 0x00,
    0x0811, 0x2F,
    0x0812, 0x00,
    0x0813, 0x2F,
    0x0814, 0x00,
    0x0815, 0x2F,
    0x0816, 0x00,
    0x0817, 0xBF,
    0x0818, 0x00,
    0x0819, 0x27,
    0x30A2, 0x00,
    0x30A3, 0xE3,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static void custom3_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("E %s.\n", __func__);
    imx355_table_write_cmos_sensor_21143(ctx, custom3_setting_array_21143,
        sizeof(custom3_setting_array_21143) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}

static kal_uint16 custom4_setting_array_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x0E,
    0x0343, 0x58,
    0x0340, 0x07,
    0x0341, 0x7C,
    0x0344, 0x00,
    0x0345, 0x08,
    0x0346, 0x01,
    0x0347, 0x38,
    0x0348, 0x0C,
    0x0349, 0xC7,
    0x034A, 0x08,
    0x034B, 0x67,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x30,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x58,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x08,
    0x0821, 0x40,
    0x3088, 0x04,
    0x6813, 0x01,
    0x6835, 0x00,
    0x6836, 0x00,
    0x6837, 0x02,
    0x684D, 0x00,
    0x684E, 0x00,
    0x684F, 0x02,
    0x0202, 0x07,
    0x0203, 0x72,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0x5F,
    0x080C, 0x00,
    0x080D, 0x27,
    0x080E, 0x00,
    0x080F, 0x3F,
    0x0810, 0x00,
    0x0811, 0xDF,
    0x0812, 0x00,
    0x0813, 0x1F,
    0x0814, 0x00,
    0x0815, 0x1F,
    0x0816, 0x00,
    0x0817, 0x8F,
    0x0818, 0x00,
    0x0819, 0x1F,
    0x30A2, 0x00,
    0x30A3, 0x37,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static void custom4_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("E %s.\n", __func__);
    imx355_table_write_cmos_sensor_21143(ctx, custom4_setting_array_21143,
        sizeof(custom4_setting_array_21143) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}

static kal_uint16 custom5_setting_array_21143[] = {
    0x0808, 0x02,
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x07,
    0x0343, 0x2C,
    0x0340, 0x0A,
    0x0341, 0x36,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x0C,
    0x0349, 0xCF,
    0x034A, 0x09,
    0x034B, 0x9F,
    0x0220, 0x00,
    0x0222, 0x01,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x034C, 0x06,
    0x034D, 0x68,
    0x034E, 0x04,
    0x034F, 0xD0,
    0x0301, 0x05,
    0x0303, 0x01,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x78,
    0x030B, 0x01,
    0x030D, 0x04,
    0x030E, 0x00,
    0x030F, 0x3C,
    0x0310, 0x00,
    0x0700, 0x00,
    0x0701, 0x10,
    0x0820, 0x05,
    0x0821, 0xA0,
    0x3088, 0x02,
    0x6813, 0x01,
    0x6835, 0x00,
    0x6836, 0x00,
    0x6837, 0x02,
    0x684D, 0x00,
    0x684E, 0x00,
    0x684F, 0x02,
    0x0202, 0x0A,
    0x0203, 0x2C,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x080A, 0x00,
    0x080B, 0x57,
    0x080C, 0x00,
    0x080D, 0x1F,
    0x080E, 0x00,
    0x080F, 0x2F,
    0x0810, 0x00,
    0x0811, 0xDF,
    0x0812, 0x00,
    0x0813, 0x17,
    0x0814, 0x00,
    0x0815, 0x17,
    0x0816, 0x00,
    0x0817, 0x6F,
    0x0818, 0x00,
    0x0819, 0x17,
    0x30A2, 0x00,
    0x30A3, 0x2F,
    0x30A0, 0x00,
    0x30A1, 0x0F,
};

static void custom5_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("E %s.\n", __func__);
    imx355_table_write_cmos_sensor_21143(ctx, custom5_setting_array_21143,
        sizeof(custom5_setting_array_21143) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("Leave: %s.\n", __func__);
}

#define IMX355_EEPROM_READ_ID  0xA2
static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, IMX355_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, IMX355_EEPROM_READ_ID >> 1,
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

static void read_stereo_data(struct subdrv_ctx *ctx)
{
	LOG_INF("read_stereo_data begin\n");
	read_cmos_eeprom_p8(ctx, stereo_start_add, stereo_data, CALI_DATA_SLAVE_LENGTH);
	LOG_INF("read_stereo_data end\n");
}

static BYTE imx355_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static void read_module_data(struct subdrv_ctx *ctx)
{
    kal_uint16 idx = 0;
    read_imx355_eeprom_info_21143(ctx, EEPROM_META_MODULE_ID,
                &(imx355_common_data[0]), 2);
    read_imx355_eeprom_info_21143(ctx, EEPROM_META_SENSOR_ID,
                &(imx355_common_data[2]), 2);
    read_imx355_eeprom_info_21143(ctx, EEPROM_META_LENS_ID,
                &(imx355_common_data[4]), 2);
    read_imx355_eeprom_info_21143(ctx, EEPROM_META_VCM_ID,
                &(imx355_common_data[6]), 2);
    read_imx355_eeprom_info_21143(ctx, EEPROM_META_MODULE_SN,
                &(imx355_common_data[8]), 17);

    for (idx = 0; idx < 30; idx = idx + 4)
        LOG_INF("In %s: cam data: %02x %02x %02x %02x\n", __func__,
               imx355_common_data[idx], imx355_common_data[idx + 1],
               imx355_common_data[idx + 2],
               imx355_common_data[idx + 3]);
}

#define DISTORTIONPARAMS_DATA_SIZE      0x0D71
#define DISTORTIONPARAMS_DATA_OFFSET    0x0CA0
static kal_uint8 distortionparams_data[DISTORTIONPARAMS_DATA_SIZE] = {0};

static void read_distortionparams_data(struct subdrv_ctx *ctx)
{
    kal_uint16 idx = 0;
    for (idx = 0 ; idx < DISTORTIONPARAMS_DATA_SIZE ; idx++) {
        distortionparams_data[idx] = read_cmos_eeprom_8(ctx, DISTORTIONPARAMS_DATA_OFFSET + idx);
    }
}

#define IMX355_EEPROM_WRITE_ID  0xA2
#define   WRITE_DATA_MAX_LENGTH     (16)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
    kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, IMX355_EEPROM_WRITE_ID >> 1,
            addr, para, WRITE_DATA_MAX_LENGTH);
    return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    if ( enable ) {
        adaptor_i2c_wr_u8(ctx->i2c_client, IMX355_EEPROM_WRITE_ID >> 1, 0xE000, 0xA3);
    } else {
        adaptor_i2c_wr_u8(ctx->i2c_client, IMX355_EEPROM_WRITE_ID >> 1, 0xE000, 0xA2);
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
		if ((pStereodata->uSensorId == IMX355_SENSOR_ID_21143) && (data_length == CALI_DATA_SLAVE_LENGTH)
			&& (data_base == IMX355_STEREO_START_ADDR)) {
			LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1544]);
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
			LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, IMX355_STEREO_START_ADDR));
			LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, IMX355_STEREO_START_ADDR+39));
			LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, IMX355_STEREO_START_ADDR+40));
			LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, IMX355_STEREO_START_ADDR+1544));
			LOG_INF("write_Module_data Write end\n");
		}
	} else {
		LOG_INF("imx355 write_Module_data pStereodata is null\n");
		return -1;
	}
	return ret;
}

/*************************************************************************
 * FUNCTION
 *    get_imgsensor_id
 *
 * DESCRIPTION
 *    This function get the sensor ID
 *
 * PARAMETERS
 *    *sensorID : return the sensor ID
 *
 * RETURNS
 *    None
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
            *sensor_id = ((read_cmos_sensor_8(ctx, 0x0016) << 8)
                    | read_cmos_sensor_8(ctx,0x0017));
            if (*sensor_id == IMX355_SENSOR_ID) {
                *sensor_id = imgsensor_info.sensor_id;
                if (first_read) {
                    LOG_INF("read module data when first poweron");
                    read_module_data(ctx);
                    read_stereo_data(ctx);
                    first_read = KAL_FALSE;
                }
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", ctx->i2c_write_id, *sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n", ctx->i2c_write_id);
            retry--;
        } while (retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int open(struct subdrv_ctx *ctx)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;

    LOG_1;

    KD_SENSOR_PROFILE_INIT(ctx);
    /* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
     * we should detect the module used i2c address
     */
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            get_imgsensor_id(ctx, &sensor_id);
            if (sensor_id == IMX355_SENSOR_ID) {
                sensor_id = imgsensor_info.sensor_id;
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
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    KD_SENSOR_PROFILE(ctx, "imx355_open_1");
    /* initail sequence write in  */
    sensor_init(ctx);

    KD_SENSOR_PROFILE(ctx, "sensor_init");


    ctx->autoflicker_en = KAL_FALSE;
    ctx->sensor_mode = IMGSENSOR_MODE_INIT;
    ctx->pclk = imgsensor_info.pre.pclk;
    ctx->shutter = 0x3D0;
    ctx->gain = 0x100;
    ctx->frame_length = imgsensor_info.pre.framelength;
    ctx->line_length = imgsensor_info.pre.linelength;
    ctx->min_frame_length = imgsensor_info.pre.framelength;
    ctx->dummy_pixel = 0;
    ctx->dummy_line = 0;
    ctx->hdr_mode = 0;
    ctx->test_pattern = 0;
    ctx->current_fps = imgsensor_info.pre.max_framerate;

    KD_SENSOR_PROFILE(ctx, "imx355_open_2");
    return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int close(struct subdrv_ctx *ctx)
{
    write_cmos_sensor(ctx, 0x0100, 0x00);    /*stream off */
    return ERROR_NONE;
}                /*    close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    KD_SENSOR_PROFILE_INIT(ctx);

    ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
    ctx->pclk = imgsensor_info.pre.pclk;
    /* ctx->video_mode = KAL_FALSE; */
    ctx->line_length = imgsensor_info.pre.linelength;
    ctx->frame_length = imgsensor_info.pre.framelength;
    ctx->min_frame_length = imgsensor_info.pre.framelength;
    ctx->autoflicker_en = KAL_FALSE;

    KD_SENSOR_PROFILE(ctx, "pre_lock");

    preview_setting(ctx);

    KD_SENSOR_PROFILE(ctx, "pre_setting");
    return ERROR_NONE;
}                /*    preview   */

/*************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    KD_SENSOR_PROFILE_INIT(ctx);

    ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
    ctx->pclk = imgsensor_info.cap.pclk;
    ctx->line_length = imgsensor_info.cap.linelength;
    ctx->frame_length = imgsensor_info.cap.framelength;
    ctx->min_frame_length = imgsensor_info.cap.framelength;
    ctx->autoflicker_en = KAL_FALSE;


    KD_SENSOR_PROFILE(ctx, "cap_lock");

    capture_setting(ctx, ctx->current_fps);    /*Full mode */

    KD_SENSOR_PROFILE(ctx, "cap_setting");

    return ERROR_NONE;
}                /* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    KD_SENSOR_PROFILE_INIT(ctx);

    ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
    ctx->pclk = imgsensor_info.normal_video.pclk;
    ctx->line_length = imgsensor_info.normal_video.linelength;
    ctx->frame_length = imgsensor_info.normal_video.framelength;
    ctx->min_frame_length = imgsensor_info.normal_video.framelength;
    /* ctx->current_fps = 300; */
    ctx->autoflicker_en = KAL_FALSE;

    KD_SENSOR_PROFILE(ctx, "nv_lock");

    normal_video_setting(ctx, ctx->current_fps);

    KD_SENSOR_PROFILE(ctx, "nv_setting");
    /* set_mirror_flip(sensor_config_data->SensorImageMirror); */

    return ERROR_NONE;
}                /*    normal_video   */

static kal_uint32 hs_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    KD_SENSOR_PROFILE_INIT(ctx);

    ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    ctx->pclk = imgsensor_info.hs_video.pclk;
    /* ctx->video_mode = KAL_TRUE; */
    ctx->line_length = imgsensor_info.hs_video.linelength;
    ctx->frame_length = imgsensor_info.hs_video.framelength;
    ctx->min_frame_length = imgsensor_info.hs_video.framelength;
    ctx->dummy_line = 0;
    ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;

    KD_SENSOR_PROFILE(ctx, "hv_lock");

    hs_video_setting(ctx);

    KD_SENSOR_PROFILE(ctx, "hv_setting");
    /* set_mirror_flip(sensor_config_data->SensorImageMirror); */
    return ERROR_NONE;
}                /*    hs_video   */

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

    return ERROR_NONE;
}   /* slim_video */

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
    ctx->autoflicker_en = KAL_FALSE;
    custom1_setting(ctx);

    return ERROR_NONE;
}   /* custom1 */

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
    ctx->autoflicker_en = KAL_FALSE;
    custom2_setting(ctx);

    return ERROR_NONE;
}   /* custom2 */

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
    ctx->autoflicker_en = KAL_FALSE;

    custom3_setting(ctx);

    return ERROR_NONE;
}   /* custom3 */

static kal_uint32 custom4(struct subdrv_ctx *ctx,
        MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    ctx->pclk = imgsensor_info.custom4.pclk;
    ctx->line_length = imgsensor_info.custom4.linelength;
    ctx->frame_length = imgsensor_info.custom4.framelength;
    ctx->min_frame_length = imgsensor_info.custom4.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom4_setting(ctx);

    return ERROR_NONE;
}   /* custom4 */

static kal_uint32 custom5(struct subdrv_ctx *ctx,
        MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    ctx->pclk = imgsensor_info.custom5.pclk;
    ctx->line_length = imgsensor_info.custom5.linelength;
    ctx->frame_length = imgsensor_info.custom5.framelength;
    ctx->min_frame_length = imgsensor_info.custom5.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom5_setting(ctx);

    return ERROR_NONE;
}   /* custom5 */

static int get_resolution(
        struct subdrv_ctx *ctx,
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
}                /*    get_resolution    */

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
               MSDK_SENSOR_INFO_STRUCT *sensor_info,
               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    /*LOG_INF("scenario_id = %d\n", scenario_id); */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    /* not use */
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

    /* inverse with datasheet */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4;    /* not use */
    sensor_info->SensorResetActiveHigh = FALSE;    /* not use */
    sensor_info->SensorResetDelayCount = 5;    /* not use */

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


    sensor_info->SensorMasterClockSwitch = 0;    /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

    sensor_info->AESensorGainDelayFrame =
        imgsensor_info.ae_sensor_gain_delay_frame;
    sensor_info->AEISPGainDelayFrame =
        imgsensor_info.ae_ispGain_delay_frame;

    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

/* PDAF_SUPPORT_CAMSV; */
/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode */
    sensor_info->PDAF_Support = 0;
    sensor_info->HDR_Support = 0;    /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR */

    sensor_info->SensorHorFOV = H_FOV;
    sensor_info->SensorVerFOV = V_FOV;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3;    /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2;    /* not use */
    sensor_info->SensorPixelClockCount = 3;    /* not use */
    sensor_info->SensorDataLatchCount = 2;    /* not use */

    sensor_info->SensorWidthSampling = 0;    /* 0 is default 1x */
    sensor_info->SensorHightSampling = 0;    /* 0 is default 1x */
    sensor_info->SensorPacketECCOrder = 1;
    sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

    return ERROR_NONE;
}                /*    get_info  */

static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
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
    default:
        LOG_INF("Error ScenarioId setting");
        preview(ctx, image_window, sensor_config_data);
        return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}                /* control(ctx) */

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{                /* This Function not used after ROME */
    LOG_INF("framerate = %d\n ", framerate);
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
    if (enable) /*enable auto flicker*/
        ctx->autoflicker_en = KAL_TRUE;
    else /*Cancel Auto flick*/
        ctx->autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
        enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
    case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        frame_length =
            imgsensor_info.pre.pclk /
            framerate * 10 / imgsensor_info.pre.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
            ? (frame_length - imgsensor_info.pre.framelength) : 0;
        ctx->frame_length =
            imgsensor_info.pre.framelength + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
        break;
    case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        if (framerate == 0)
            return ERROR_NONE;
        frame_length =
            imgsensor_info.normal_video.pclk /
            framerate * 10 / imgsensor_info.normal_video.linelength;

        ctx->dummy_line =
        (frame_length > imgsensor_info.normal_video.framelength)
        ? (frame_length - imgsensor_info.normal_video.framelength) : 0;

        ctx->frame_length =
         imgsensor_info.normal_video.framelength + ctx->dummy_line;

        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
        break;
    case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        if (ctx->current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF(
            "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                 framerate, imgsensor_info.cap.max_framerate / 10);

        frame_length = imgsensor_info.cap.pclk /
            framerate * 10 / imgsensor_info.cap.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.cap.framelength)
            ? (frame_length - imgsensor_info.cap.framelength) : 0;
        ctx->frame_length =
            imgsensor_info.cap.framelength + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
        break;
    case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        frame_length = imgsensor_info.hs_video.pclk /
            framerate * 10 / imgsensor_info.hs_video.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.hs_video.framelength)
            ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
        ctx->frame_length =
            imgsensor_info.hs_video.framelength + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
        break;
    case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        frame_length = imgsensor_info.slim_video.pclk / framerate * 10
            / imgsensor_info.slim_video.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.slim_video.framelength)
            ? (frame_length - imgsensor_info.slim_video.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.slim_video.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
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
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
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
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
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
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM4:
        frame_length = imgsensor_info.custom4.pclk / framerate * 10
                / imgsensor_info.custom4.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom4.framelength)
            ? (frame_length - imgsensor_info.custom4.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom4.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM5:
        frame_length = imgsensor_info.custom5.pclk / framerate * 10
                / imgsensor_info.custom5.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom5.framelength)
            ? (frame_length - imgsensor_info.custom5.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom5.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
        }
        break;
    default:        /* coding with  preview scenario by default */
        frame_length = imgsensor_info.pre.pclk /
            framerate * 10 / imgsensor_info.pre.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
            ? (frame_length - imgsensor_info.pre.framelength) : 0;
        ctx->frame_length =
            imgsensor_info.pre.framelength + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            set_dummy(ctx);
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
    /*LOG_INF("scenario_id = %d\n", scenario_id); */

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
    default:
        break;
    }

    return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(struct subdrv_ctx *ctx)
{
    /*
    UINT8 temperature = 0;
    INT32 temperature_convert = 0;

    temperature = read_cmos_sensor_8(ctx, 0x013a);

    if (temperature >= 0x0 && temperature <= 0x60)
        temperature_convert = temperature;
    else if (temperature >= 0x61 && temperature <= 0x7F)
        temperature_convert = 97;
    else if (temperature >= 0x80 && temperature <= 0xE2)
        temperature_convert = -30;
    else
        temperature_convert = (INT8)temperature | 0xFFFFFF0;

    LOG_INF("temp_c(%d), read_reg(%d)\n", temperature_convert, temperature);
    */
    return 20;
}

static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
                  UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    INT32 *feature_return_para_i32 = (INT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *)feature_para;
    int ret = ERROR_NONE;
/* unsigned long long *feature_return_data =
 * (unsigned long long*)feature_para;
 */

    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
        (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

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
    case SENSOR_FEATURE_GET_DISTORTIONPARAMS:
        {
            LOG_INF("get distortionparams data");
            read_distortionparams_data(ctx);
            memcpy(feature_return_para_32, (UINT32 *)distortionparams_data, sizeof(distortionparams_data));
            *feature_para_len = sizeof(distortionparams_data);
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
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
    case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
        if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
            *(feature_data + 0) =
                sizeof(imx355_ana_gain_table_21143);
        } else {
            memcpy((void *)(uintptr_t) (*(feature_data + 1)),
            (void *)imx355_ana_gain_table_21143,
            sizeof(imx355_ana_gain_table_21143));
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
        case SENSOR_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom1.pclk;
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
        case SENSOR_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
            = (imgsensor_info.custom1.framelength << 16)
                + imgsensor_info.custom1.linelength;
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
        break;
    case SENSOR_FEATURE_SET_GAIN:
        set_gain(ctx, (UINT16) *feature_data);
        break;
    case SENSOR_FEATURE_SET_DUAL_GAIN:
        break;
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        write_cmos_sensor(ctx,
            sensor_reg_data->RegAddr, sensor_reg_data->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        sensor_reg_data->RegData =
            read_cmos_sensor(ctx, sensor_reg_data->RegAddr);
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
        set_video_mode(ctx, *feature_data);
        break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        get_imgsensor_id(ctx, feature_return_para_32);
        break;
    case SENSOR_FEATURE_GET_EEPROM_COMDATA:
        memcpy(feature_return_para_32, imx355_common_data,
                OPLUS_CAMERA_COMMON_DATA_LENGTH);
        *feature_para_len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
        break;
    case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
        LOG_INF("imx355 get STEREO_DATA");
        memcpy(feature_return_para_32, (UINT32 *)stereo_data, sizeof(stereo_data));
        *feature_para_len = sizeof(stereo_data);
        break;
    case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
        set_auto_flicker_mode(ctx,
            (BOOL)(*feature_data_16), *(feature_data_16 + 1));
        break;
    case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
        set_max_framerate_by_scenario(ctx,
            (enum MSDK_SCENARIO_ID_ENUM)(*feature_data),
                          *(feature_data + 1));
        break;
    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
        get_default_framerate_by_scenario(ctx,
            (enum MSDK_SCENARIO_ID_ENUM)(*feature_data),
            (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
        break;
    case SENSOR_FEATURE_GET_PDAF_DATA:
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
        set_test_pattern_mode(ctx, (UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *)(feature_data+1));
        break;

    /* for factory mode auto testing */
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
        *feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_FRAMERATE:
        LOG_INF("current fps :%d\n", *feature_data_32);
        ctx->current_fps = (UINT16)*feature_data_32;
        break;
    case SENSOR_FEATURE_GET_CROP_INFO:
        LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
            (UINT32) *feature_data);

        wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
        memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[(*feature_data_32)<SENSOR_SCENARIO_ID_CUSTOM5?(*feature_data_32):SENSOR_SCENARIO_ID_CUSTOM5], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
    //     switch (*feature_data_32) {
    //     case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
    //         memcpy((void *)wininfo,
    //             (void *)&imgsensor_winsize_info[1],
    //             sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
    //         break;
    //     case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
    //         memcpy((void *)wininfo,
    //             (void *)&imgsensor_winsize_info[2],
    //             sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
    //         break;
    //     case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
    //         memcpy((void *)wininfo,
    //             (void *)&imgsensor_winsize_info[3],
    //             sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
    //         break;
    //     case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
    //         memcpy((void *)wininfo,
    //             (void *)&imgsensor_winsize_info[0],
    //             sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
    //         break;
    //     default:
    //         break;
    //     }
        break;
        /*HDR CMD */
    case SENSOR_FEATURE_SET_HDR:
        LOG_INF("hdr enable :%d\n", *feature_data_32);
        ctx->hdr_mode = (UINT8)*feature_data_32;
        break;
    case SENSOR_FEATURE_SET_HDR_SHUTTER:
        LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
            (UINT16) *feature_data,
            (UINT16) *(feature_data + 1));
        break;
    case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
        *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
        break;
    case SENSOR_FEATURE_SET_AWB_GAIN:
        break;
        /*PDAF CMD */

    case SENSOR_FEATURE_SET_PDAF:
        LOG_INF("PDAF mode :%d\n", *feature_data_16);
        ctx->pdaf_mode = *feature_data_16;
        break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        set_shutter_frame_length(ctx,
            (UINT16)(*feature_data), (UINT16)(*(feature_data + 1)),
            (BOOL) (*(feature_data + 2)));
        break;
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        /*
         * 1, if driver support new sw frame sync
         * set_shutter_frame_length(ctx) support third para auto_extend_en
         */
        *(feature_data + 1) = 1;
        /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
        *feature_return_para_i32 = get_sensor_temperature(ctx);
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
            (*feature_para_len));
        break;
    case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
        LOG_INF("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
            (*feature_para_len));
        break;

    case SENSOR_FEATURE_SET_PDFOCUS_AREA:
        LOG_INF(
            "SENSOR_FEATURE_SET_imx355_PDFOCUS_AREA Start Pos=%d, Size=%d\n",
            (UINT32) *feature_data, (UINT32) *(feature_data + 1));
        break;
        /*End of PDAF */
    case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
        streaming_control(ctx, KAL_FALSE);
        break;
    case SENSOR_FEATURE_SET_STREAMING_RESUME:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME\n");
        streaming_control(ctx, KAL_TRUE);
        break;

    case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
    {
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.cap.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.normal_video.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.hs_video.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.slim_video.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom1.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom2.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom3.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom4.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom5.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
    }
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
}                /*    feature_control(ctx)  */


#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {//3264*2448
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0990,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {//3264*2448
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0990,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {//3264*1840
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0730,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {//1408*792
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0580,
			.vsize = 0x0318,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {//3264*2448
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0990,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_custom1[] = {
	{
		.bus.csi2 = {//2304*1728
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0900,
			.vsize = 0x06C0,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_custom2[] = {
	{
		.bus.csi2 = {//1640*1232
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0668,
			.vsize = 0x04D0,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_custom3[] = {
	{
		.bus.csi2 = {//3264*2448
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0990,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_custom4[] = {
	{
		.bus.csi2 = {//3264*1840
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,
			.vsize = 0x0730,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_custom5[] = {
	{
		.bus.csi2 = {//1640*1232
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0668,
			.vsize = 0x04D0,
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
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_hs_vid);
		memcpy(fd->entry, frame_desc_hs_vid, sizeof(frame_desc_hs_vid));
		break;
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
	case SENSOR_SCENARIO_ID_CUSTOM1:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_custom1);
		memcpy(fd->entry, frame_desc_custom1, sizeof(frame_desc_custom1));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_custom2);
		memcpy(fd->entry, frame_desc_custom2, sizeof(frame_desc_custom2));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_custom3);
		memcpy(fd->entry, frame_desc_custom3, sizeof(frame_desc_custom3));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM4:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_custom4);
		memcpy(fd->entry, frame_desc_custom4, sizeof(frame_desc_custom4));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM5:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_custom5);
		memcpy(fd->entry, frame_desc_custom5, sizeof(frame_desc_custom5));
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
    .exposure_def = 0x3D0,
    /* support long exposure at most 128 times) */
    .exposure_max = (0xffff * 128) - 10,
    .exposure_min = 8,
    .exposure_step = 1,
    .frame_time_delay_frame = 3,
    .margin = 10,
    .max_frame_length = 0xffff,

    .mirror = IMAGE_HV_MIRROR,    /* mirrorflip information */
    .sensor_mode = IMGSENSOR_MODE_INIT,
    .shutter = 0x3D0,    /* current shutter */
    .gain = BASEGAIN * 4,        /* current gain */
    .dummy_pixel = 0,    /* current dummypixel */
    .dummy_line = 0,    /* current dummyline */
    /* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
    .current_fps = 300,
    .autoflicker_en = KAL_FALSE,
    /* auto flicker enable: KAL_FALSE for disable auto flicker,
     * KAL_TRUE for enable auto flicker
     */
    .test_pattern = KAL_FALSE,

    /* current scenario id */
    .current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
    .hdr_mode = 0,    /* sensor need support LE, SE with HDR feature */
    .i2c_write_id = 0x34,    /* record current sensor's i2c write id */
};

static int get_temp(struct subdrv_ctx *ctx, int *temp)
{
    *temp = get_sensor_temperature(ctx) * 1000;
    return 0;
}

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
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
	case SENSOR_SCENARIO_ID_CUSTOM3:
		csi_param->dphy_trail = 69;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		csi_param->dphy_trail = 86;
		csi_param->dphy_data_settle = 0x10;
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
    .get_temp = get_temp,
    #ifdef IMGSENSOR_VC_ROUTING
    .get_frame_desc = get_frame_desc,
    #endif
    .get_csi_param = get_csi_param,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
    {HW_ID_MCLK, 24, 0},
    //{HW_ID_PDN, 0, 0},
    {HW_ID_RST, 0, 0},
    {HW_ID_AVDD, 2700000, 0},
    {HW_ID_DVDD, 1204000, 0},
    {HW_ID_DOVDD, 1804000, 1},
    {HW_ID_MCLK_DRIVING_CURRENT, 6, 5},
    //{HW_ID_PDN, 1, 0},
    {HW_ID_RST, 1, 1},
};

const struct subdrv_entry imx355_mipi_raw_21143_entry = {
    .name = "imx355_mipi_raw_21143",
    .id = IMX355_SENSOR_ID_21143,
    .pw_seq = pw_seq,
    .pw_seq_cnt = ARRAY_SIZE(pw_seq),
    .ops = &ops,
};

