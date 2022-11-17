// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     OV02B10mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 * Setting version:
 * ------------
 *   update full pd setting for OV02B10EB_03B
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#define PFX "OV02B10_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include "ov02b10_eeprom_21121.h"
#include "ov02b10mipiraw_Sensor_21121.h"
#include "ov02b10_Sensor_setting_21121.h"
#include "ov02b10_ana_gain_table_21121.h"



//address:8bit;Value:8bit
#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8_u8(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8_u8(__VA_ARGS__)
//address:16bit;Value:8bit
#define read_cmos_sensor(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define write_cmos_sensor(...) subdrv_i2c_wr_u8(__VA_ARGS__)

#define table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u8_u8(__VA_ARGS__)


#define LOG_INF(format, args...)    \
    pr_err(PFX "[%s] " format, __func__, ##args)

#define MULTI_WRITE 1


static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = OV02B10_SENSOR_ID_21121,

    .checksum_value = 0x388c7147,//test_Pattern_mode

    .pre = {
        .pclk = 16500000,    /*record different mode's pclk*/
        .linelength  = 448,    /*record different mode's linelength*/
        .framelength = 1221,    /*record different mode's framelength*/
        .startx = 0, /*record different mode's startx of grabwindow*/
        .starty = 0,    /*record different mode's starty of grabwindow*/
        .grabwindow_width  = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 342857143,
    },
    .cap = {
        .pclk = 16500000,
        .linelength  = 448,
        .framelength = 1221,
        .startx = 0,
        .starty = 0,
        .grabwindow_width  = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 342857143,
    },
    .normal_video = { /* cap*/
        .pclk = 16500000,
        .linelength  = 448,
        .framelength = 1221,
        .startx = 0,
        .starty = 0,
        .grabwindow_width  = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 342857143,
    },
    .hs_video = {
        .pclk = 16500000,
        .linelength  = 448,
        .framelength = 1221,
        .startx = 0,
        .starty = 0,
        .grabwindow_width  = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 342857143,
    },
    .slim_video = {/*pre*/
        .pclk = 16500000,
        .linelength  = 448,
        .framelength = 1221,
        .startx = 0,
        .starty = 0,
        .grabwindow_width  = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 342857143,
    },

    .custom1 = {
        .pclk = 16500000,
        .linelength  = 448,
        .framelength = 1536,
        .startx = 0,
        .starty = 0,
        .grabwindow_width  = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 240,
        .mipi_pixel_rate = 342857143,
    },
    .margin = 7,                    /* sensor framelength & shutter margin */
    .min_shutter = 4,                /* min shutter */
    .min_gain = BASEGAIN * 1, /*1x gain*/
    .max_gain = (kal_uint32)(BASEGAIN * 15.5f), /*15.5x * 1024  gain*/
    .min_gain_iso = 100,
    .gain_step = 1, /*minimum step = 4 in 1x~2x gain*/
    .gain_type = 1,/*to be modify,no gain table for sony*/
    .max_frame_length = 0x7fff,     /* max framelength by sensor register's limitation */
    .ae_shut_delay_frame = 0,        //check
    .ae_sensor_gain_delay_frame = 0,//check
    .ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,
    .ihdr_le_firstline = 0,
    .sensor_mode_num = 6,            //support sensor mode num

    .cap_delay_frame = 3,            //enter capture delay frame num
    .pre_delay_frame = 3,            //enter preview delay frame num
    .video_delay_frame = 3,            //enter video delay frame num
    .hs_video_delay_frame = 3,        //enter high speed video  delay frame num
    .slim_video_delay_frame = 3,    //enter slim video delay frame num
    .custom1_delay_frame = 3,        //enter custom1 delay frame num
    .frame_time_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_6MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
    .mipi_sensor_type = MIPI_CPHY,
    /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
    .mipi_settle_delay_mode = 0,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz

    .mipi_lane_num = SENSOR_MIPI_3_LANE,//mipi lane num
    .i2c_addr_table = {0x7A,0xff},
    .i2c_speed = 400,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[17] = {
    {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0, 0, 1600, 1200, 0, 0, 1600, 1200}, // Preview
    {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0, 0, 1600, 1200, 0, 0, 1600, 1200}, // capture
    {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0, 0, 1600, 1200, 0, 0, 1600, 1200}, // video
    {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0, 0, 1600, 1200, 0, 0, 1600, 1200}, // hight video 120
    {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0, 0, 1600, 1200, 0, 0, 1600, 1200}, // slim video
    {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0, 0, 1600, 1200, 0, 0, 1600, 1200}, // custom1
};

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765    /*trans# max is 255, each 3 bytes*/
#else
#define I2C_BUFFER_LEN 3
#endif

static void set_dummy(struct subdrv_ctx *ctx)
{
    kal_uint32 frame_length = 0, line_length = 0;
    if (ctx->frame_length%2 != 0) {
        ctx->frame_length = ctx->frame_length - ctx->frame_length%2;
    }
    pr_info("imgsensor.frame_length = %d, line_length = %d\n", ctx->frame_length, ctx->line_length);
    frame_length = ctx->frame_length;
    line_length = ctx->line_length;
    write_cmos_sensor_8(ctx, 0xfd, 0x01);
    write_cmos_sensor_8(ctx, 0x14, (frame_length-1220) >> 8);
    write_cmos_sensor_8(ctx, 0x15, (frame_length-1220) & 0xFF);
    write_cmos_sensor_8(ctx, 0xfe, 0x02);
} /*set_dummy*/

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
    kal_uint32 frame_length = ctx->frame_length;

    pr_info("framerate = %d, min framelength should enable %d\n", framerate,
        min_framelength_en);
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
    set_dummy(ctx);
}

static void set_max_framerate_video(struct subdrv_ctx *ctx, UINT16 framerate,
                    kal_bool min_framelength_en)
{
    set_max_framerate(ctx, framerate, min_framelength_en);
}

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
    pr_info("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
    if (enable){
        write_cmos_sensor_8(ctx, 0xfd, 0X03);
        write_cmos_sensor_8(ctx, 0xc2, 0X01);
    } else {
        write_cmos_sensor_8(ctx, 0xfd, 0X03);
        write_cmos_sensor_8(ctx, 0xc2, 0X00);
    }
    mdelay(70);
    return ERROR_NONE;
}
static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;

    if (shutter > ctx->min_frame_length - imgsensor_info.margin)
        ctx->frame_length = shutter + imgsensor_info.margin;
    else
        ctx->frame_length = ctx->min_frame_length;
    if (ctx->frame_length > imgsensor_info.max_frame_length)
        ctx->frame_length = imgsensor_info.max_frame_length;
    if (shutter < imgsensor_info.min_shutter)
        shutter = imgsensor_info.min_shutter;

    /* long expsoure */
    if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
        shutter = imgsensor_info.max_frame_length - imgsensor_info.margin;
    }
    frame_length =  ctx->frame_length;
    if (ctx->autoflicker_en) {
        realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
        LOG_INF("autoflicker enable, realtime_fps = %d\n",
            realtime_fps);
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(ctx, 296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(ctx, 146, 0);
        else {
            /* Extend frame length*/
            write_cmos_sensor_8(ctx, 0xfd, 0X01);
            write_cmos_sensor_8(ctx, 0x14, (frame_length - 1220) >> 8);
            write_cmos_sensor_8(ctx, 0x15, (frame_length - 1220) & 0xFF);
            write_cmos_sensor_8(ctx, 0xfe, 0X02);
        }
    } else {
            /* Extend frame length*/
            write_cmos_sensor_8(ctx, 0xfd, 0X01);
            write_cmos_sensor_8(ctx, 0x14, (frame_length - 1220) >> 8);
            write_cmos_sensor_8(ctx, 0x15, (frame_length - 1220) & 0xFF);
            write_cmos_sensor_8(ctx, 0xfe, 0X02);
    }
    /* Update Shutter */
    write_cmos_sensor_8(ctx, 0xfd, 0x01);
    write_cmos_sensor_8(ctx, 0x0f, (shutter) & 0xFF);
    write_cmos_sensor_8(ctx, 0x0e, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(ctx, 0xfe, 0x02);

    LOG_INF("shutter =%d, framelength =%d\n", shutter, ctx->frame_length);
}

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
static void set_shutter_w_gph(struct subdrv_ctx *ctx, kal_uint32 shutter, kal_bool gph)
{
    ctx->shutter = shutter;
    write_shutter(ctx, shutter, gph);
}

//should not be kal_uint16 -- can't reach long exp
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
    pr_info("Enter into set_shutter, shutter:%u", shutter);
    ctx->shutter = shutter;
    set_shutter_w_gph(ctx, shutter, KAL_FALSE);
    pr_info("Exit from set_shutter");
}

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
    kal_uint16 iReg = 0x0000;

    //platform 1xgain = 1024, sensor driver 1*gain = 0x10
    iReg = gain*16/BASEGAIN;
    return iReg;        /* sensorGlobalGain */
}

static kal_uint32 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
    kal_uint16 reg_gain;
    kal_uint32 max_gain = imgsensor_info.max_gain;

    if (gain < imgsensor_info.min_gain || gain > max_gain) {
        pr_info("Error gain setting\n");

        if (gain < imgsensor_info.min_gain)
            gain = imgsensor_info.min_gain;
        else if (gain > max_gain)
            gain = max_gain;
    }

    reg_gain = gain2reg(ctx, gain);
    ctx->gain = reg_gain;

    pr_info("gain = %d , reg_gain = 0x%x\n", gain, reg_gain);
    write_cmos_sensor_8(ctx, 0xfd, 0x01);
    write_cmos_sensor_8(ctx, 0x22, (reg_gain & 0xFF));
    write_cmos_sensor_8(ctx, 0xfe, 0x02);
    return gain;
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

static void ihdr_write_shutter_gain(struct subdrv_ctx *ctx, kal_uint16 le,
                kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
    if (ctx->ihdr_mode) {
        if (le > ctx->min_frame_length - imgsensor_info.margin) {
            ctx->frame_length = le + imgsensor_info.margin;
        } else {
            ctx->frame_length = ctx->min_frame_length;
        }
        if (ctx->frame_length > imgsensor_info.max_frame_length) {
            ctx->frame_length = imgsensor_info.max_frame_length;
        }
        if (le < imgsensor_info.min_shutter) {
            le = imgsensor_info.min_shutter;
        }
        if (se < imgsensor_info.min_shutter) {
            se = imgsensor_info.min_shutter;
        }

        /* Extend frame length first*/
        write_cmos_sensor(ctx, 0x380e, ctx->frame_length >> 8);
        write_cmos_sensor(ctx, 0x380f, ctx->frame_length & 0xFF);
        write_cmos_sensor(ctx, 0x3502, (le << 4) & 0xFF);
        write_cmos_sensor(ctx, 0x3501, (le >> 4) & 0xFF);
        write_cmos_sensor(ctx, 0x3500, (le >> 12) & 0x0F);
        write_cmos_sensor(ctx, 0x3512, (se << 4) & 0xFF);
        write_cmos_sensor(ctx, 0x3511, (se >> 4) & 0xFF);
        write_cmos_sensor(ctx, 0x3510, (se >> 12) & 0x0F);
        set_gain(ctx, gain);
    }
}

static void set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);

    switch(image_mirror) {
    case IMAGE_NORMAL:
        //.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
        write_cmos_sensor_8(ctx, 0xfd, 0x01);
        write_cmos_sensor_8(ctx, 0x12, 0x00);
        write_cmos_sensor_8(ctx, 0x01, 0x01);
        break;

    case IMAGE_H_MIRROR:
        //.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_GB,
        write_cmos_sensor_8(ctx, 0xfd, 0x01);
        write_cmos_sensor_8(ctx, 0x12, 0x01);
        write_cmos_sensor_8(ctx, 0x01, 0x01);
        break;

    case IMAGE_V_MIRROR:
        //.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_GR,
        write_cmos_sensor_8(ctx, 0xfd, 0x01);
        write_cmos_sensor_8(ctx, 0x12, 0x02);
        write_cmos_sensor_8(ctx, 0x01, 0x01);
        break;

    case IMAGE_HV_MIRROR:
        //.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
        write_cmos_sensor_8(ctx, 0xfd, 0x01);
        write_cmos_sensor_8(ctx, 0x12, 0x03);
        write_cmos_sensor_8(ctx, 0x01, 0x01);
        break;

    default:
        LOG_INF("Error image_mirror setting\n");
    }
}

static void sensor_init(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    table_write_cmos_sensor(ctx,
        addr_data_pair_init_ov02b10_21121,
        sizeof(addr_data_pair_init_ov02b10_21121) / sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("%s end\n", __func__);
}

static void preview_setting(struct subdrv_ctx *ctx)
{
    int _length = 0;

    pr_info("%s RES_1600x1200_30fps\n", __func__);
    _length = sizeof(addr_data_pair_preview_ov02b10_21121) / sizeof(kal_uint16);
    table_write_cmos_sensor(ctx, addr_data_pair_preview_ov02b10_21121, _length);
    pr_info("%s end\n", __func__);
}

static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
    int _length = 0;

    pr_info("%s capture res_1600x1200_30fs\n", __func__);
    _length = sizeof(addr_data_pair_capture_ov02b10_21121) / sizeof(kal_uint16);
    table_write_cmos_sensor(ctx, addr_data_pair_capture_ov02b10_21121, _length);
    pr_info("%s end\n", __func__);
}

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
    int _length = 0;

    pr_info("%s normal video res_1600x1200_30fs\n", __func__);
    _length = sizeof(addr_data_pair_video_ov02b10_21121) / sizeof(kal_uint16);
    table_write_cmos_sensor(ctx,
            addr_data_pair_video_ov02b10_21121,
            _length);
    pr_info("%s end\n", __func__);
}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
    int _length = 0;

    pr_info("%s hs video res_1600x1200_30fs\n", __func__);
    _length = sizeof(addr_data_pair_hs_video_ov02b10_21121) / sizeof(kal_uint16);
    table_write_cmos_sensor(ctx,
        addr_data_pair_hs_video_ov02b10_21121,
        _length);
    pr_info("%s end\n", __func__);
}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
    int _length = 0;

    pr_info("%s RES_3840x2160_30fps\n", __func__);
    _length = sizeof(addr_data_pair_slim_video_ov02b10_21121) / sizeof(kal_uint16);
    table_write_cmos_sensor(ctx,
        addr_data_pair_slim_video_ov02b10_21121,
        _length);
    pr_info("%s end\n", __func__);
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static void custom1_setting(struct subdrv_ctx *ctx)
{
    int _length = 0;

    pr_info("custom1_setting_start\n");
    _length = sizeof(addr_data_pair_custom1_21121) / sizeof(kal_uint16);
    table_write_cmos_sensor(ctx,
        addr_data_pair_custom1_21121,
        _length);

    pr_info("%s end\n", __func__);
}    /*    custom1_setting  */

static BYTE ov02b10_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static void read_module_data(struct subdrv_ctx *ctx)
{
    kal_uint16 idx = 0;
    read_ov02b10_eeprom_info_21121(ctx, EEPROM_META_MODULE_ID,
                &(ov02b10_common_data[0]), 1);
    imgsensor_info.module_id = (kal_uint16)(ov02b10_common_data[1] << 8) |
                   ov02b10_common_data[0];
    read_ov02b10_eeprom_info_21121(ctx, EEPROM_META_SENSOR_ID,
                &(ov02b10_common_data[2]), 1);
    read_ov02b10_eeprom_info_21121(ctx, EEPROM_META_LENS_ID,
                &(ov02b10_common_data[4]), 1);
    read_ov02b10_eeprom_info_21121(ctx, EEPROM_META_VCM_ID,
                &(ov02b10_common_data[6]), 1);
    read_ov02b10_eeprom_info_21121(ctx, EEPROM_META_MODULE_SN,
                &(ov02b10_common_data[8]), 17);

    for (idx = 0; idx < 21; idx = idx + 4)
        LOG_INF("In %s: cam data: %02x %02x %02x %02x\n", __func__,
               ov02b10_common_data[idx], ov02b10_common_data[idx + 1],
               ov02b10_common_data[idx + 2],
               ov02b10_common_data[idx + 3]);
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
        ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
        do {
            write_cmos_sensor_8(ctx, 0xfd, 0x00);
            *sensor_id = ((read_cmos_sensor(ctx, 0x0200) << 8) | read_cmos_sensor(ctx, 0x0300));
            pr_info("In get_imgsensor_id:i2c write id: 0x%x, sensor id: 0x%x, imgsensor_info.sensor_id:0x%x\n",
                    ctx->i2c_write_id, *sensor_id, imgsensor_info.sensor_id);
            if (*sensor_id == OV02B10_SENSOR_ID) {
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
                    ctx->i2c_write_id, *sensor_id);
                if(first_read){
                    read_module_data(ctx);
                    first_read = KAL_FALSE;
                }
                return ERROR_NONE;
            }
            retry--;
        } while (retry > 0);
        i++;
        retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        LOG_INF("%s: 0x%x fail, reg[0x0200]:0x%x, reg[0x300]:0x%x\n", 
                __func__, *sensor_id, read_cmos_sensor(ctx, 0x0200), read_cmos_sensor(ctx, 0x0300));
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
    kal_uint8 retry = 1;
    kal_uint32 sensor_id = 0;

    /*add for delay i2c clock when power up*/
    mdelay(9);
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
        do {
            sensor_id = ((read_cmos_sensor(ctx, 0x0200) << 8) | read_cmos_sensor(ctx, 0x0300));
            pr_info("In open:i2c write id: 0x%x, sensor id: 0x%x, imgsensor_info.sensor_id:0x%x\n",
                    ctx->i2c_write_id, sensor_id, imgsensor_info.sensor_id);
            if (sensor_id == OV02B10_SENSOR_ID) {
                sensor_id = imgsensor_info.sensor_id;
                pr_info("i2c write id: 0x%x, sensor id: 0x%x\n",
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
        pr_info("Open sensor id: 0x%x fail\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    sensor_init(ctx);
    mdelay(10);
    ctx->autoflicker_en = KAL_FALSE;
    ctx->sensor_mode = IMGSENSOR_MODE_INIT;
    ctx->shutter = 0x0400;
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

    return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
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
    streaming_control(ctx, KAL_FALSE);
    return ERROR_NONE;
}   /*  close  */


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
    pr_info("%s E\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
    ctx->pclk = imgsensor_info.pre.pclk;
    //ctx->video_mode = KAL_FALSE;
    ctx->line_length = imgsensor_info.pre.linelength;
    ctx->frame_length = imgsensor_info.pre.framelength;
    ctx->min_frame_length = imgsensor_info.pre.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    preview_setting(ctx);
    set_mirror_flip(ctx, ctx->mirror);
    mdelay(10);
    return ERROR_NONE;
}

static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_info("%s E\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;

    ctx->pclk = imgsensor_info.cap.pclk;
    ctx->line_length = imgsensor_info.cap.linelength;
    ctx->frame_length = imgsensor_info.cap.framelength;
    ctx->min_frame_length = imgsensor_info.cap.framelength;
    ctx->autoflicker_en = KAL_FALSE;

    capture_setting(ctx, ctx->current_fps);
    set_mirror_flip(ctx, ctx->mirror);
    /*
    if (ctx->test_pattern == KAL_FALSE) {
        write_cmos_sensor(ctx, 0x5000, (read_cmos_sensor(ctx, 0x5000) & 0xBF) | 0X00);
    }*/
    mdelay(10);
    return ERROR_NONE;
} /* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_info("%s E\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
    ctx->pclk = imgsensor_info.normal_video.pclk;
    ctx->line_length = imgsensor_info.normal_video.linelength;
    ctx->frame_length = imgsensor_info.normal_video.framelength;
    ctx->min_frame_length = imgsensor_info.normal_video.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    normal_video_setting(ctx, ctx->current_fps);
    set_mirror_flip(ctx, ctx->mirror);
    mdelay(10);
    return ERROR_NONE;
}

static kal_uint32 hs_video(struct subdrv_ctx *ctx,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_info("%s E\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    ctx->pclk = imgsensor_info.hs_video.pclk;
    //ctx->video_mode = KAL_TRUE;
    ctx->line_length = imgsensor_info.hs_video.linelength;
    ctx->frame_length = imgsensor_info.hs_video.framelength;
    ctx->min_frame_length = imgsensor_info.hs_video.framelength;
    ctx->dummy_line = 0;
    ctx->dummy_pixel = 0;
    //ctx->current_fps = 300;
    ctx->autoflicker_en = KAL_FALSE;
    hs_video_setting(ctx);
    set_mirror_flip(ctx, ctx->mirror);
    mdelay(10);
    return ERROR_NONE;
}

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_info("%s E\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    ctx->pclk = imgsensor_info.slim_video.pclk;
    //ctx->video_mode = KAL_TRUE;
    ctx->line_length = imgsensor_info.slim_video.linelength;
    ctx->frame_length = imgsensor_info.slim_video.framelength;
    ctx->min_frame_length = imgsensor_info.slim_video.framelength;
    ctx->dummy_line = 0;
    ctx->dummy_pixel = 0;
    ctx->autoflicker_en = KAL_FALSE;
    slim_video_setting(ctx);
    set_mirror_flip(ctx, ctx->mirror);
    mdelay(10);
    return ERROR_NONE;
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static kal_uint32 Custom1(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_info("%s E\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    ctx->pclk = imgsensor_info.custom1.pclk;
    //ctx->video_mode = KAL_FALSE;
    ctx->line_length = imgsensor_info.custom1.linelength;
    ctx->frame_length = imgsensor_info.custom1.framelength;
    ctx->min_frame_length = imgsensor_info.custom1.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom1_setting(ctx);
    set_mirror_flip(ctx, ctx->mirror);
    mdelay(10);
    return ERROR_NONE;
}   /*  Custom1   */

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
    pr_info("scenario_id = %d\n", scenario_id);
    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
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
    /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/
    sensor_info->PDAF_Support = 0;

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
    default:
        pr_info("Error ScenarioId setting");
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
    pr_info("enable = %d, framerate = %d\n",
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

    pr_info("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

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
    pr_info("scenario_id = %d, framerate = %d done\n", scenario_id, framerate);
    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
            enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MUINT32 *framerate)
{
    pr_info("[3058]scenario_id = %d\n", scenario_id);

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
    default:
    break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_uint32 mode)
{
    pr_info("Test_Pattern mode: %d\n", mode);

    //1:Solid Color 2:Color bar 5:black
    if (mode) {
        switch(mode) {
        case 5:
            write_cmos_sensor_8(ctx, 0xfd, 0x03);
            write_cmos_sensor_8(ctx, 0x8c, 0x00);
            write_cmos_sensor_8(ctx, 0x8e, 0x00);
            write_cmos_sensor_8(ctx, 0x90, 0x00);
            write_cmos_sensor_8(ctx, 0x92, 0x00);
            write_cmos_sensor_8(ctx, 0x9b, 0x00);
            write_cmos_sensor_8(ctx, 0xfe, 0x02);
            break;
        default:
            write_cmos_sensor_8(ctx, 0xfd, 0x03);
            write_cmos_sensor_8(ctx, 0x81, 0x01);
            break;
        }
    } else if (ctx->test_pattern) {
        write_cmos_sensor_8(ctx, 0xfd, 0x03);
        write_cmos_sensor_8(ctx, 0x81, 0x00);
        write_cmos_sensor_8(ctx, 0x8c, 0x40);
        write_cmos_sensor_8(ctx, 0x8e, 0x40);
        write_cmos_sensor_8(ctx, 0x90, 0x40);
        write_cmos_sensor_8(ctx, 0x92, 0x40);
        write_cmos_sensor_8(ctx, 0x9b, 0x46);
        write_cmos_sensor_8(ctx, 0xfe, 0x02);
    }
    ctx->test_pattern = mode;
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

    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
        (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    pr_info("feature_id = %d\n", feature_id);
    switch (feature_id) {
    case SENSOR_FEATURE_GET_OUTPUT_FORMAT_BY_SCENARIO:
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        case SENSOR_SCENARIO_ID_CUSTOM1:
            *(feature_data + 1)
            = (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
                imgsensor_info.sensor_output_dataformat;
            break;
        }
    break;
    case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
        if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
            *(feature_data + 0) =
                sizeof(ov02b10_ana_gain_table_21121);
        } else {
            memcpy((void *)(uintptr_t) (*(feature_data + 1)),
            (void *)ov02b10_ana_gain_table_21121,
            sizeof(ov02b10_ana_gain_table_21121));
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
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        case SENSOR_SCENARIO_ID_CUSTOM1:
            *(feature_data + 2) = 2;
            break;
        default:
            *(feature_data + 2) = 1;
            break;
        }
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
#ifdef OPLUS_FEATURE_CAMERA_COMMON
    case SENSOR_FEATURE_GET_MODULE_INFO:
    *feature_return_para_32++ = (ov02b10_common_data[1] << 24) |
        (ov02b10_common_data[0] << 16) |
        (ov02b10_common_data[3] << 8) |
        (ov02b10_common_data[2] & 0xFF);
    *feature_return_para_32 = (ov02b10_common_data[5] << 24) |
        (ov02b10_common_data[4] << 16) |
        (ov02b10_common_data[7] << 8) |
        (ov02b10_common_data[6] & 0xFF);
    *feature_para_len = 8;
    LOG_INF("ov02b10 GET_MODULE_CamInfo:%d %d\n", *feature_para_len,
            *feature_data_32);
    break;

    case SENSOR_FEATURE_GET_MODULE_SN:
    *feature_return_para_32++ = (ov02b10_common_data[11] << 24) |
        (ov02b10_common_data[10] << 16) |
        (ov02b10_common_data[9] << 8) |
        (ov02b10_common_data[8] & 0xFF);
    *feature_para_len = 4;
    LOG_INF("ov02b10 GET_MODULE_SN:%d %d\n", *feature_para_len,
            *feature_data_32);
    break;

    //case SENSOR_FEATURE_SET_SENSOR_OTP:
    //    kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
    //    LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP length :%d\n", (UINT32)*feature_para_len);
    //    ret = write_Module_data((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
    //    if (ret == ERROR_NONE)
    //        return ERROR_NONE;
    //    else
    //        return ERROR_MSDK_IS_ACTIVATED;

    case SENSOR_FEATURE_CHECK_MODULE_ID:
    *feature_return_para_32 = (UINT32)imgsensor_info.module_id;
    *feature_para_len = 4;
    break;

    case SENSOR_FEATURE_GET_EEPROM_COMDATA:
    memcpy(feature_return_para_32, ov02b10_common_data,
            OPLUS_CAMERA_COMMON_DATA_LENGTH);
    *feature_para_len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
    break;
    case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
    break;
    case SENSOR_FEATURE_GET_DISTORTIONPARAMS:
    break;
#endif
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
        set_test_pattern_mode(ctx, (UINT32)*feature_data);
    break;
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
        *feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len = 4;
    break;
    case SENSOR_FEATURE_SET_FRAMERATE:
        ctx->current_fps = *feature_data_32;
        pr_info("current fps :%d\n", ctx->current_fps);
    break;
    case SENSOR_FEATURE_GET_CROP_INFO:
        //pr_info("GET_CROP_INFO scenarioId:%d\n",
        //    *feature_data_32);

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
        case SENSOR_SCENARIO_ID_CUSTOM1:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[5],
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
        pr_info("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
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
            case SENSOR_SCENARIO_ID_CUSTOM1:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.custom1.mipi_pixel_rate;
                break;
            case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
            default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                    imgsensor_info.pre.mipi_pixel_rate;
                break;
            }
    break;
    case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
        LOG_INF("This sensor can get temperature 20\n");
        *feature_return_para_i32 = 20;
        *feature_para_len = 4;
    break;
/* ITD: Modify Dualcam By Jesse 190924 Start */
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        pr_info("SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME\n");
        set_shutter_frame_length(ctx, (UINT16)*feature_data, (UINT16)*(feature_data+1));
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
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
            *feature_return_para_32 = 2; /*BINNING_SUMMED*/
            break;
        default:
            *feature_return_para_32 = 1; /*BINNING_AVERAGED*/
            break;
        }
        pr_info("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_ESD_RESET_BY_USER:
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
            default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
                break;
        }
        break;
    default:
    break;
    }
    return ERROR_NONE;
}   /*  feature_control(ctx)  */

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_inf[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 1600,
            .vsize = 1200,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
};
static int get_frame_desc(struct subdrv_ctx *ctx,
        int scenario_id, struct mtk_mbus_frame_desc *fd)
{
    fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
    fd->num_entries = ARRAY_SIZE(frame_desc_inf);
    memcpy(fd->entry, frame_desc_inf, sizeof(frame_desc_inf));
    return 0;
}
#endif

static const struct subdrv_ctx defctx = {

    .ana_gain_def = 4 * BASEGAIN,
    .ana_gain_max = 15.5 * BASEGAIN,
    .ana_gain_min = BASEGAIN,
    .ana_gain_step = 4,
    .exposure_def = 0x3D0,
    .exposure_max = 0xffffe9 - 22,
    .exposure_min = 4,
    .exposure_step = 1,
    .frame_time_delay_frame = 3,
    .margin = 22,
    .max_frame_length = 0xffffe9,

    .mirror = IMAGE_HV_MIRROR,
    .sensor_mode = IMGSENSOR_MODE_INIT,
    .shutter = 0x3D0,
    .gain = 4 * BASEGAIN,
    .dummy_pixel = 0,
    .dummy_line = 0,
    .current_fps = 300,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
    .ihdr_mode = 0,
    .i2c_write_id = 0x7a,
};

static int init_ctx(struct subdrv_ctx *ctx,
        struct i2c_client *i2c_client, u8 i2c_write_id)
{
    memcpy(ctx, &defctx, sizeof(*ctx));
    ctx->i2c_client = i2c_client;
    ctx->i2c_write_id = i2c_write_id;
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
};

static struct subdrv_pw_seq_entry pw_seq[] = {
    {HW_ID_RST, 0, 1},
    {HW_ID_DOVDD, 1800000, 1},
    {HW_ID_AVDD, 2804000, 8},
    {HW_ID_MCLK, 24, 7},
    {HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
    {HW_ID_RST, 1, 10},
};

const struct subdrv_entry ov02b10_mipi_raw_21121_entry = {
    .name = "ov02b10_mipi_raw_21121",
    .id = OV02B10_SENSOR_ID_21121,
    .pw_seq = pw_seq,
    .pw_seq_cnt = ARRAY_SIZE(pw_seq),
    .ops = &ops,
};

