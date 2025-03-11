// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include <soc/oplus/device_info.h>
#include <soc/oplus/system/boot_mode.h>
#include <mt-plat/mtk_boot_common.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/msm_drm_notify.h>
#include <drm/drm_panel.h>
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif
#include "../mediatek/mediatek_v2/mtk_corner_pattern/oplus_23662_tianma_mtk_data_hw_roundedpattern.h"

#include "../../../misc/mediatek/gate_ic/gate_i2c.h"

#include "ktz8866.h"
#define CHANGE_FPS_EN 1

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
extern enum boot_mode_t get_boot_mode(void);
#else
extern int get_boot_mode(void);
#endif
#else
/* extern unsigned int silence_mode; */
extern int get_boot_mode(void);
#endif
#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
#endif

static int esd_brightness;
extern unsigned long esd_flag;
extern unsigned int g_shutdown_flag;
static int current_esd_fps;

/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */
/****************TPS65132***********/
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
//#include "lcm_i2c.h"

static char bl_tb0[] = { 0x51, 0x0f, 0xff };
static char bl_open[] = { 0x53, 0x2C };
/*esd check*/
static char fps_open[] = 	{ 0xB9, 0x83, 0x10, 0x21, 0x55, 0x00 };
static char fps_BD[] =     	{ 0xBD, 0x00 };
static char fps120_e2[] =   { 0xE2, 0x00 };
static char fps60_e2[] =    { 0xE2, 0x50 };
static char fps40_e2[] =    { 0xE2, 0x60 };
static char fps_close[] =    { 0xB9, 0x00, 0x00, 0x00, 0x00, 0x00 };
//TO DO: You have to do that remove macro BYPASSI2C and solve build error
//otherwise voltage will be unstable
#ifdef HIMAX_WAKE_UP
extern uint8_t wake_flag_drm;
#endif
extern unsigned int oplus_max_normal_brightness;

struct boe {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
	bool prepared;
	bool enabled;

	unsigned int gate_ic;

	int error;
};

#define boe_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		boe_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define boe_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		boe_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct boe *panel_to_boe(struct drm_panel *panel)
{
	return container_of(panel, struct boe, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int boe_dcs_read(struct boe *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void boe_panel_get_data(struct boe *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = boe_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void boe_dcs_write(struct boe *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void boe_panel_init(struct boe *ctx)
{
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5 * 1000, 10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(55 * 1000, 65 * 1000);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	boe_dcs_write_seq_static(ctx, 0xB9, 0x83,0x10,0x21,0x55,0x00);
	boe_dcs_write_seq_static(ctx, 0xB1,  0x2C,0xAF,0xAF,0x2B,0xEB,0x43,0xE1,0x50,0x36,0x36,0x36,0x36,0x1A,0x8B,0x11,0x65,0x00,0x88,0xFA,0xFF,0xFF,0x8F,0xFF,0x08,0x9B,0x33);
	boe_dcs_write_seq_static(ctx, 0xB2,  0x00,0x47,0xB0,0xD0,0x00,0x12,0x92,0x3C,0x38,0x22,0x00,0x00,0x00,0x88,0xF5);
	boe_dcs_write_seq_static(ctx, 0xB4,  0x3C,0x3C,0x3C,0x3C,0x3C,0x3C,0x49,0x3B,0x7C,0x70,0x01,0x46);
	boe_dcs_write_seq_static(ctx, 0xE9, 0xCD);
	boe_dcs_write_seq_static(ctx, 0xBA, 0x84);
	boe_dcs_write_seq_static(ctx, 0xE9, 0x3F);
	boe_dcs_write_seq_static(ctx, 0xBC, 0x1B,0x04);
	boe_dcs_write_seq_static(ctx, 0xBE, 0x20);
	boe_dcs_write_seq_static(ctx, 0xC0,  0x3A,0x3A,0x22,0x11,0x22,0xA0,0x61,0x08,0xF5,0x03);
	boe_dcs_write_seq_static(ctx, 0xE9, 0xCC);
	boe_dcs_write_seq_static(ctx, 0xC7, 0x80);
	boe_dcs_write_seq_static(ctx, 0xE9, 0x3F);
	boe_dcs_write_seq_static(ctx, 0xE9, 0xC6);
	boe_dcs_write_seq_static(ctx, 0xC8, 0x97);
	boe_dcs_write_seq_static(ctx, 0xE9, 0x3F);
	boe_dcs_write_seq_static(ctx, 0xC9, 0x00,0x1E,0x13,0x88,0x01);
	boe_dcs_write_seq_static(ctx, 0xCB, 0x08,0x13,0x07,0x00,0x07,0x35);
	boe_dcs_write_seq_static(ctx, 0xCC, 0x02,0x03,0x4C);
	boe_dcs_write_seq_static(ctx, 0xE9, 0xC4);
	boe_dcs_write_seq_static(ctx, 0xD0, 0x03);
	boe_dcs_write_seq_static(ctx, 0xE9, 0x3F);
	boe_dcs_write_seq_static(ctx, 0xD1, 0x37,0x06,0x00,0x02,0x04,0x0C,0xFF);
	boe_dcs_write_seq_static(ctx, 0xD3,  0x06,0x00,0x00,0x00,0x40,0x04,0x08,0x04,0x08,0x37,0x47,0x64,0x4B,0x11,0x11,0x03,0x03,0x32,0x10,0x0C,0x00,0x0C,0x32,0x10,0x08,0x00,0x08,0x32,0x17,0xE7,0x07,0xE7,0x00,0x00);
	usleep_range(5 * 1000, 6 * 1000);
	boe_dcs_write_seq_static(ctx, 0xD5,  0x24,0x24,0x24,0x24,0x07,0x06,0x07,0x06,0x05,0x04,0x05,0x04,0x03,0x02,0x03,0x02,0x01,0x00,0x01,0x00,0x21,0x20,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x1F,0x1F,0x1F,0x1F,0x1E,0x1E,0x1E,0x1E,0x18,0x18,0x18,0x18);
	usleep_range(5 * 1000, 6 * 1000);
	boe_dcs_write_seq_static(ctx, 0xD8,  0xAA,0xAA,0xAA,0xAA,0xFA,0xA0,0xAA,0xAA,0xAA,0xAA,0xFA,0xA0);
	boe_dcs_write_seq_static(ctx, 0xE7,  0x23,0x0A,0x0A,0x36,0x37,0x46,0x03,0xA6,0x4C,0x14,0x14,0x00,0x00,0x00,0x00,0x12,0x05,0x02,0x02,0x0A,0x33,0x03,0x84);
	usleep_range(5 * 1000, 6 * 1000);
	boe_dcs_write_seq_static(ctx, 0xE1,  0x11,0x00,0x00,0x89,0x30,0x80,0x07,0xD0,0x02,0x58,0x00,0x14,0x02,0x58,0x02,0x58,0x02,0x00,0x02,0x2C,0x00,0x20,0x02,0x02,0x00,0x08,0x00,0x0C,0x05,0x0E,0x04,0x94,0x18,0x00,0x10,0xF0,0x03,0x0C,0x20,0x00,0x06,0x0B,0x0B,0x33,0x0E);

	boe_dcs_write_seq_static(ctx, 0xBD, 0x01);
	boe_dcs_write_seq_static(ctx, 0xB1, 0x01,0xBF,0x11);
	boe_dcs_write_seq_static(ctx, 0xCB, 0x80);

	boe_dcs_write_seq_static(ctx, 0xD2, 0xF0);
	boe_dcs_write_seq_static(ctx, 0xE9, 0xC9);
	boe_dcs_write_seq_static(ctx, 0xD3, 0x84);
	usleep_range(5 * 1000, 6 * 1000);
	boe_dcs_write_seq_static(ctx, 0xE9, 0x3F);
	boe_dcs_write_seq_static(ctx, 0xD8,  0xFF,0xFF,0xFA,0xAF,0xFF,0xA0,0xFF,0xFF,0xFA,0xAF,0xFF,0xA0,0xFF,0xFF,0xFA,0xAF,0xFF,0xA0,0xFF,0xFF,0xFA,0xAF,0xFF,0xA0,0xFF,0xFF,0xFA,0xAF,0xFF,0xA0,0xFF,0xFF,0xFA,0xAF,0xFF,0xA0);
	boe_dcs_write_seq_static(ctx, 0xE1,  0x40,0x09,0xBE,0x19,0xFC,0x19,0xFA,0x19,0xF8,0x1A,0x38,0x1A,0x78,0x1A,0xB6,0x2A,0xF6,0x2B,0x34,0x2B,0x74,0x3B,0x74,0x6B,0x74);
	boe_dcs_write_seq_static(ctx, 0xE2,  0x3C,0x3C,0x3C,0x3C,0x3C,0x3C,0x49,0x3F,0x7C,0x70,0x04,0x68,0x01,0x4B,0x01,0x00,0x20,0x01,0x03,0x00,0x00,0x0F,0x0F,0x0F,0x0F,0x00,0x00,0x00,0x15,0x10,0x10,0x28,0x39,0x4B,0x02,0xA3,0x4D,0x14,0x14,0x00,0x00,0x87,0x01,0x0A,0x4F,0x09,0x50,0x00,0x20,0x4B,0x00,0x38,0x00,0x07,0x35,0x00);
	boe_dcs_write_seq_static(ctx, 0xE7,  0x02,0x00,0x90,0x01,0xD2,0x09,0xBF,0x0A,0xA0,0x00,0x00);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x02);

	boe_dcs_write_seq_static(ctx, 0xD8,  0xFF,0xFF,0xFA,0xAF,0xFF,0xA0,0xFF,0xFF,0xFA,0xAF,0xFF,0xA0);
	boe_dcs_write_seq_static(ctx, 0xE2,  0x3C,0x3C,0x3C,0x3C,0x3C,0x3C,0x49,0x3F,0x7C,0x70,0x04,0x68,0x01,0x4B,0x01,0x00,0x20,0x01,0x03,0x00,0x00,0x0F,0x0F,0x0F,0x0F,0x00,0x00,0x00,0x27,0x18,0x18,0x3A,0x39,0x4B,0x02,0xE2,0x49,0x14,0x14,0x00,0x00,0xC9,0x01,0x0A,0x57,0x0A,0x58,0x00,0x20,0x4B,0x00,0x38,0x00,0x07,0x35,0x00);
	boe_dcs_write_seq_static(ctx, 0xE7,  0xFE,0x03,0xFE,0x03,0xFE,0x03,0x02,0x02,0x02,0x25,0x00,0x25,0x81,0x02,0x40,0x00,0x20,0x49,0x04,0x03,0x02,0x01,0x00,0x00,0x00,0x00,0x01,0x00);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x03);
	boe_dcs_write_seq_static(ctx, 0xD8,  0xAA,0xAA,0xAA,0xAA,0xFA,0xA0,0xAA,0xAA,0xAA,0xAA,0xFA,0xA0,0xFF,0xFF,0xFA,0xAF,0xFF,0xA0,0xFF,0xFF,0xFA,0xAF,0xFF,0xA0,0x55,0x55,0x55,0x55,0x55,0x50,0x55,0x55,0x55,0x55,0x55,0x50);
	boe_dcs_write_seq_static(ctx, 0xE1, 0x01);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x00);
	boe_dcs_write_seq_static(ctx, 0xE9, 0xC4);
	boe_dcs_write_seq_static(ctx, 0xBA, 0x96);
	boe_dcs_write_seq_static(ctx, 0xE9, 0x3F);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x01);

	boe_dcs_write_seq_static(ctx, 0xE9, 0xC5);
	boe_dcs_write_seq_static(ctx, 0xBA, 0x4F);
	boe_dcs_write_seq_static(ctx, 0xE9, 0x3F);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x03);
	boe_dcs_write_seq_static(ctx, 0xE9, 0xC6);
	boe_dcs_write_seq_static(ctx, 0xB4, 0x03,0xFF,0xF8);
	boe_dcs_write_seq_static(ctx, 0xE9, 0x3F);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x00);
	/* boe_dcs_write_seq_static(ctx, 0xE2, 0x00);   50 60FPS 00 120FPS */
	boe_dcs_write_seq_static(ctx, 0x35, 0x00); /* TE */
	boe_dcs_write_seq_static(ctx, 0x5E, 0x00,0x16);
	/* CABC */
	boe_dcs_write_seq_static(ctx, 0xE4, 0x2D,0x03,0x41,0x2C,0x60,0x60,0x60,0x60,0x00);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x01);
	boe_dcs_write_seq_static(ctx, 0xE4, 0xE1,0xE1,0xE1,0xE1,0xE1,0xE1,0xE1,0xE1,0xC5,0xB0,0x9E,0x8E,0x7F,0x73,0x67,0x5B,0x4F,0x45,0x3B,0x36,0x2D,0x28,0x23,0x20,0x1A,0x15,0x12,0x0D,0x06,0x05,0x03,0x54,0x55,0x55);
	usleep_range(5 * 1000, 5 * 1010);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x03);
	boe_dcs_write_seq_static(ctx, 0xE4, 0x3C,0x5B,0x65,0x7A,0x8E,0xA2,0xB7,0xD6,0xE5,0xFF,0xFF,0x03,0xE0,0xFF,0x13,0x32,0x46,0x5B,0x84,0xCB,0xE0,0xFA,0xFF,0x03,0xC1,0xE0,0x09,0x1D,0x32,0x46,0x6F,0xC6,0xDB,0xFA,0xFF,0x03);
	usleep_range(5 * 1000, 5 * 1010);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x00);
	boe_dcs_write_seq_static(ctx, 0xB9, 0x00, 0x00, 0x00, 0x00, 0x00);
	boe_dcs_write_seq_static(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	/* msleep(120); */
	usleep_range(120 * 1000, 120 * 1010);
	boe_dcs_write_seq_static(ctx, 0x29, 0x00); // MIPI_DCS_SET_DISPLAY_ON
	/* msleep(20); */
	usleep_range(20 * 1000, 20 * 1010);

	pr_info("%s-\n", __func__);
}
static int boe_disable(struct drm_panel *panel)
{
	struct boe *ctx = panel_to_boe(panel);

	if (!ctx->enabled)
		return 0;

	pr_info("%s line = %d\n", __func__,__LINE__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int boe_unprepare(struct drm_panel *panel)
{

	struct boe *ctx = panel_to_boe(panel);
	int mode = 0;
	int blank = 0;
	int flag_poweroff = 1;

	if (!ctx->prepared)
		return 0;

	mode = get_boot_mode();
	if ((mode != MSM_BOOT_MODE__FACTORY) && (mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
		if(tp_gesture_enable_notifier && tp_gesture_enable_notifier(0) && (g_shutdown_flag == 0) && (esd_flag == 0)) {
			flag_poweroff = 0;
		} else {
			flag_poweroff = 1;
		}
	}

	boe_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(50);
	boe_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(150);
	pr_err("[TP]flag_poweroff = %d\n",flag_poweroff);
	if (flag_poweroff == 1) {
		ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	ctx->bias_neg =
		devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	usleep_range(1000, 1001);

	ctx->bias_pos =
		devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	/*boe_dcs_write_seq_static(ctx, 0xB9, 0x83, 0x10, 0x21, 0x55, 0x00);
	boe_dcs_write_seq_static(ctx, 0xBD, 0x00);
	boe_dcs_write_seq_static(ctx, 0xB1, 0x2D); */
	#ifdef HIMAX_WAKE_UP
		//if (wake_flag_drm == 0)
			//lcd_set_bl_bias_reg(ctx->dev, 0);
	#else
			//lcd_set_bl_bias_reg(ctx->dev, 0);
	#endif
		#define LCD_CTL_RST_OFF 0x12
		#define LCD_CTL_CS_OFF  0x1A
		blank = LCD_CTL_RST_OFF;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		blank = LCD_CTL_CS_OFF;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		usleep_range(110000, 110001); /* 100ms vsp and vsn pull low to hight time boe_unprepare -> boe_prepare from IC request*/
	}
	ctx->error = 0;
	ctx->prepared = false;
	return 0;
}

static int boe_prepare(struct drm_panel *panel)
{
	struct boe *ctx = panel_to_boe(panel);
	int ret;
	int blank = 0;

	pr_info("%s+\n", __func__);
	if (ctx->prepared)
		return 0;

	// lcd reset H -> L -> L
	//ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	//gpiod_set_value(ctx->reset_gpio, 1);
	//usleep_range(10000, 10001);
	//gpiod_set_value(ctx->reset_gpio, 0);
	//msleep(20);
	//gpiod_set_value(ctx->reset_gpio, 1);
	//devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	// end
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5001);// 5ms

	ctx->bias_pos =
		devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	usleep_range(3000, 3010);// 3ms
	ctx->bias_neg =
		devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	usleep_range(5 * 1000, 10 * 1000);// 7ms

	#define LCD_CTL_TP_LOAD_FW 0x10
	#define LCD_CTL_CS_ON  0x19
	blank = LCD_CTL_CS_ON;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	usleep_range(5000, 5100);
	blank = LCD_CTL_TP_LOAD_FW;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	usleep_range(5000, 5100);
#ifdef HIMAX_WAKE_UP
	//if (wake_flag_drm == 0)
		//lcd_set_bl_bias_reg(ctx->dev, 1);
#else
		//lcd_set_bl_bias_reg(ctx->dev, 1);
#endif
//	_lcm_i2c_write_bytes(0x0, 0xf);
//	_lcm_i2c_write_bytes(0x1, 0xf);
	boe_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0) {
		boe_unprepare(panel);
		pr_info("%s11111-\n", __func__);
	}
	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	boe_panel_get_data(ctx);
#endif

#ifdef LCD_LOAD_TP_FW
	lcd_queue_load_tp_fw();
#endif

	pr_info("%s-\n", __func__);
	return ret;
}

static int boe_enable(struct drm_panel *panel)
{
	struct boe *ctx = panel_to_boe(panel);

	if (ctx->enabled)
		return 0;

	pr_info("%s line = %d\n", __func__,__LINE__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 341330,
	.hdisplay = 1200,
	.hsync_start = 1200 + 52,//HFP
	.hsync_end = 1200 + 52 + 20,//HSA
	.htotal = 1200 + 52 + 20 + 40,//HBP
	.vdisplay = 2000,
	.vsync_start = 2000 + 148,/* 2316 60FPS */
	.vsync_end = 2000 + 148 + 8,//VSA
	.vtotal = 2000 + 148 + 8 + 12,//VBP
};

#if CHANGE_FPS_EN
static const struct drm_display_mode performance_mode_40hz = {
	.clock = 341330,
	.hdisplay = 1200,
	.hsync_start = 1200 + 52,//HFP
	.hsync_end = 1200 + 52 + 20,//HSA
	.htotal = 1200 + 52 + 20 + 40,//HBP
	.vdisplay = 2000,
	.vsync_start = 2000 + 4484,//VFP
	.vsync_end = 2000 + 4484 + 8,//VSA
	.vtotal = 2000 + 4484 + 8 + 12,//VBP
};

static const struct drm_display_mode performance_mode_60hz = {
	.clock = 341330,
	.hdisplay = 1200,
	.hsync_start = 1200 + 52,//HFP
	.hsync_end = 1200 + 52 + 20,//HSA
	.htotal = 1200 + 52 + 20 + 40,//HBP
	.vdisplay = 2000,
	.vsync_start = 2000 + 2316, /* 2316 60FPS */
	.vsync_end = 2000 + 2316 + 8,
	.vtotal = 2000 + 2316 + 8 + 12,
};
#endif

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.change_fps_by_vfp_send_cmd = 1,
	.vfp_low_power = 148,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {0},

	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2000,
		.pic_width = 1200,
		.slice_height = 20, /* 5 */
		.slice_width = 600,
		.chunk_size = 600,
		.xmit_delay = 512,
		.dec_delay = 556,
		.scale_value = 32,
		.increment_interval = 514, /* 117 */
		.decrement_interval = 8,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 1294, /* 6144 */
		.slice_bpg_offset = 1172, /* 4686 */
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 4,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 0,
		.rc_range_parameters[1].range_max_qp = 4,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 1,
		.rc_range_parameters[2].range_max_qp = 5,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 1,
		.rc_range_parameters[3].range_max_qp = 6,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 3,
		.rc_range_parameters[4].range_max_qp = 7,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 3,
		.rc_range_parameters[5].range_max_qp = 7,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 3,
		.rc_range_parameters[6].range_max_qp = 7,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 3,
		.rc_range_parameters[7].range_max_qp = 8,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 3,
		.rc_range_parameters[8].range_max_qp = 9,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 3,
		.rc_range_parameters[9].range_max_qp = 10,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 5,
		.rc_range_parameters[10].range_max_qp = 11,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 5,
		.rc_range_parameters[11].range_max_qp = 12,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 5,
		.rc_range_parameters[12].range_max_qp = 13,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 7,
		.rc_range_parameters[13].range_max_qp = 13,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 13,
		.rc_range_parameters[14].range_max_qp = 13,
		.rc_range_parameters[14].range_bpg_offset = -12
		},
	.data_rate = 920, /* 943 */
	.data_rate_khz = 920190, /* 943307 */
	.lfr_enable = 0,
	.lfr_minimum_fps = 120,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 6, {0xb9, 0x83, 0x10, 0x21, 0x55, 0x00} },
		.dfps_cmd_table[1] = {0, 2, {0xBD, 0x00} },
		.dfps_cmd_table[2] = {0, 2, {0xE2, 0x00} },
		.dfps_cmd_table[3] = {0, 6, {0xb9, 0x00, 0x00, 0x00, 0x00, 0x00} },
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.vfp = 148,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.rotate = 1,
	.phy_timcon = {
		.hs_trail = 0x0B,
		.clk_hs_post = 0x0F,
	},
	.oplus_display_global_dre = 1,
};

#if CHANGE_FPS_EN
static struct mtk_panel_params ext_params_40hz = {
	.change_fps_by_vfp_send_cmd = 1,
	.vfp_low_power = 4484,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {0},

	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2000,
		.pic_width = 1200,
		.slice_height = 5,
		.slice_width = 600,
		.chunk_size = 600,
		.xmit_delay = 512,
		.dec_delay = 556,
		.scale_value = 32,
		.increment_interval = 117,
		.decrement_interval = 8,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 6144,
		.slice_bpg_offset = 4686,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 4,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 0,
		.rc_range_parameters[1].range_max_qp = 4,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 1,
		.rc_range_parameters[2].range_max_qp = 5,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 1,
		.rc_range_parameters[3].range_max_qp = 6,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 3,
		.rc_range_parameters[4].range_max_qp = 7,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 3,
		.rc_range_parameters[5].range_max_qp = 7,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 3,
		.rc_range_parameters[6].range_max_qp = 7,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 3,
		.rc_range_parameters[7].range_max_qp = 8,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 3,
		.rc_range_parameters[8].range_max_qp = 9,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 3,
		.rc_range_parameters[9].range_max_qp = 10,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 5,
		.rc_range_parameters[10].range_max_qp = 11,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 5,
		.rc_range_parameters[11].range_max_qp = 12,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 5,
		.rc_range_parameters[12].range_max_qp = 13,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 7,
		.rc_range_parameters[13].range_max_qp = 13,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 13,
		.rc_range_parameters[14].range_max_qp = 13,
		.rc_range_parameters[14].range_bpg_offset = -12
		},
	.data_rate = 920, /* 943 */
	.data_rate_khz = 920190, /* 943307 */
	.lfr_enable = 0,
	.lfr_minimum_fps = 40,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 40,
		.dfps_cmd_table[0] = {0, 6, {0xb9, 0x83, 0x10, 0x21, 0x55, 0x00} },
		.dfps_cmd_table[1] = {0, 2, {0xBD, 0x00} },
		.dfps_cmd_table[2] = {0, 2, {0xE2, 0x60} }, /* E2h = 0x50 --> 60FPS E2h = 0x00 --> 120FPS */
		.dfps_cmd_table[3] = {0, 6, {0xb9, 0x00, 0x00, 0x00, 0x00, 0x00} },
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.vfp = 4484,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.rotate = 1,
	.phy_timcon = {
		.hs_trail = 0x0B,
		.clk_hs_post = 0x0F,
	},
	.oplus_display_global_dre = 1,
};

static struct mtk_panel_params ext_params_60hz = {
	.change_fps_by_vfp_send_cmd = 1,
	.vfp_low_power = 2316, /* 60 FPS */
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {0},

	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2000,
		.pic_width = 1200,
		.slice_height = 20, /* 5 */
		.slice_width = 600,
		.chunk_size = 600,
		.xmit_delay = 512,
		.dec_delay = 556,
		.scale_value = 32,
		.increment_interval = 514, /* 117 */
		.decrement_interval = 8,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 1294, /* 6144 */
		.slice_bpg_offset = 1172, /* 4686 */
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 4,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 0,
		.rc_range_parameters[1].range_max_qp = 4,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 1,
		.rc_range_parameters[2].range_max_qp = 5,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 1,
		.rc_range_parameters[3].range_max_qp = 6,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 3,
		.rc_range_parameters[4].range_max_qp = 7,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 3,
		.rc_range_parameters[5].range_max_qp = 7,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 3,
		.rc_range_parameters[6].range_max_qp = 7,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 3,
		.rc_range_parameters[7].range_max_qp = 8,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 3,
		.rc_range_parameters[8].range_max_qp = 9,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 3,
		.rc_range_parameters[9].range_max_qp = 10,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 5,
		.rc_range_parameters[10].range_max_qp = 11,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 5,
		.rc_range_parameters[11].range_max_qp = 12,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 5,
		.rc_range_parameters[12].range_max_qp = 13,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 7,
		.rc_range_parameters[13].range_max_qp = 13,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 13,
		.rc_range_parameters[14].range_max_qp = 13,
		.rc_range_parameters[14].range_bpg_offset = -12
		},
	.data_rate = 920, /* 943 */
	.data_rate_khz = 920190, /* 943307 */
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 6, {0xb9, 0x83, 0x10, 0x21, 0x55, 0x00} },
		.dfps_cmd_table[1] = {0, 2, {0xBD, 0x00} },
		.dfps_cmd_table[2] = {0, 2, {0xE2, 0x50} },
		.dfps_cmd_table[3] = {0, 6, {0xb9, 0x00, 0x00, 0x00, 0x00, 0x00} },
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.vfp = 2316,  /* 60 FPS */
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.rotate = 1,
	.phy_timcon = {
		.hs_trail = 0x0B,
		.clk_hs_post = 0x0F,
	},
	.oplus_display_global_dre = 1,
};
#endif

static void cabc_switch(void *dsi, dcs_write_gce cb,void *handle, unsigned int cabc_mode)
{
    char bl_tb1[] = {0x55, 0x00}; /* no cabc ui pictures videoes*/

    pr_err("%s cabc = %d\n", __func__, cabc_mode);

    if (cabc_mode == 1) {
        bl_tb1[1] = 1;
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    } else if (cabc_mode == 2) {
        bl_tb1[1] = 2;
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    } else if (cabc_mode == 3) {
        bl_tb1[1] = 3;
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    } else if (cabc_mode == 0) {
        bl_tb1[1] = 0; /* cabc off */
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    }else {
        bl_tb1[1] = 1; /* default */
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    }
    /* cabc_lastlevel = cabc_mode; */
}


static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int map_exp[4096] = {0};

static void init_global_exp_backlight(void)
{
        int lut_index[41] = {0, 3, 99, 144, 187, 227, 264, 300, 334, 366, 397, 427, 456, 484, 511, 537, 563, 587, 611, 635, 658, 680,
                                                702, 723, 744, 764, 784, 804, 823, 842, 861, 879, 897, 915, 933, 950, 967, 984, 1000, 1016, 1023};
        int lut_value1[41] = {0, 4, 6, 14, 24, 37, 52, 69, 87, 107, 128, 150, 173, 197, 222, 248, 275, 302, 330, 358, 387, 416, 446,
                                                477, 507, 539, 570, 602, 634, 667, 700, 733, 767, 801, 835, 869, 903, 938, 973, 1008, 1023};
        int index_start = 0, index_end = 0;
        int value1_start = 0, value1_end = 0;
        int i,j;
        int index_len = sizeof(lut_index) / sizeof(int);
        int value_len = sizeof(lut_value1) / sizeof(int);
        if (index_len == value_len) {
                for (i = 0; i < index_len - 1; i++) {
                        index_start = lut_index[i] * oplus_max_normal_brightness / 1023;
                        index_end = lut_index[i+1] * oplus_max_normal_brightness / 1023;
                        value1_start = lut_value1[i] * oplus_max_normal_brightness / 1023;
                        value1_end = lut_value1[i+1] * oplus_max_normal_brightness / 1023;
                        for (j = index_start; j <= index_end; j++) {
                                map_exp[j] = value1_start + (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
                        }
                }
        }
}


static int boe_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	unsigned int bl_level = level;
	/* bl_level = map_exp[level]; */
	unsigned int mode;

	mode = get_boot_mode();

	if (mode == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0)
		level = 2047;
	/* if (silence_mode == 1)
		level = 0; */

	if ((level > 0) && (level < oplus_max_normal_brightness)) {
		bl_level = map_exp[level];
	}

	pr_info("%s bl_level: %d,%d mode = %d %d \n", __func__, bl_level,level,mode);

	//lcd cabc backlight
	if (bl_level > 4095)
		bl_level = 4095;
	bl_tb0[1] = bl_level >> 8;
	bl_tb0[2] = bl_level & 0xFF;
	esd_brightness = level;

	pr_info("%s level = %d,backlight = %d,bl_tb0[1] = 0x%x,bl_tb0[2] = 0x%x\n",
		__func__, level, bl_level, bl_tb0[1], bl_tb0[2]);

	//bl_tb0[2] = 0xFF; // for to test

	if (!cb)
		return -1;

	//pr_info("bl_tb0 %d  %s bl_tb0[1] = 0x%x,bl_tb0[2] = 0x%x\n", __LINE__, __func__,bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	//pr_info("bl_tb0 %d  %s\n", __LINE__, __func__);
	cb(dsi, handle, bl_open, ARRAY_SIZE(bl_open));

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x07, 0xff};

	pr_err("%s esd_backlight = %d,current_esd_fps = %d\n", __func__, esd_brightness,current_esd_fps);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;

	/*lcd*/
	if(1 == current_esd_fps){
		cb(dsi, handle, fps_open, ARRAY_SIZE(fps_open));
		cb(dsi, handle, fps_BD, ARRAY_SIZE(fps_BD));
		cb(dsi, handle, fps60_e2, ARRAY_SIZE(fps60_e2));
		cb(dsi, handle, fps_close, ARRAY_SIZE(fps_close));
	}else if(2 == current_esd_fps){
		cb(dsi, handle, fps_open, ARRAY_SIZE(fps_open));
		cb(dsi, handle, fps_BD, ARRAY_SIZE(fps_BD));
		cb(dsi, handle, fps40_e2, ARRAY_SIZE(fps40_e2));
		cb(dsi, handle, fps_close, ARRAY_SIZE(fps_close));
	}else {
		cb(dsi, handle, fps_open, ARRAY_SIZE(fps_open));
		cb(dsi, handle, fps_BD, ARRAY_SIZE(fps_BD));
		cb(dsi, handle, fps120_e2, ARRAY_SIZE(fps120_e2));
		cb(dsi, handle, fps_close, ARRAY_SIZE(fps_close));
	}

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_open, ARRAY_SIZE(bl_open));

	return 1;
}

struct drm_display_mode *get_mode_by_id_hfp(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}
static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, mode);

	pr_info("drm_mode_vrefresh(m) =%d", drm_mode_vrefresh(m));

	if (ext && m && drm_mode_vrefresh(m) == 120){
		ext->params = &ext_params;
		current_esd_fps = 0;
	}
#if CHANGE_FPS_EN
	else if (ext && m && drm_mode_vrefresh(m) == 40){
		ext->params = &ext_params_40hz;
		current_esd_fps = 2;
	}
	else if (ext && m && drm_mode_vrefresh(m) == 60){
		ext->params = &ext_params_60hz;
		current_esd_fps = 1;
	}
#endif
	else
		ret = 1;

	return ret;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct boe *ctx = panel_to_boe(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = boe_setbacklight_cmdq,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.ext_param_set = mtk_panel_ext_param_set,
	//.mode_switch = mode_switch,
	.ata_check = panel_ata_check,
	.cabc_switch = cabc_switch,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *	   become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *	  display the first valid frame after starting to receive
	 *	  video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *	   turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *		 to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int boe_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;
#if CHANGE_FPS_EN
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;
#endif

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 default_mode.hdisplay, default_mode.vdisplay,
			 drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	printk("lcm_pack_modes:mode->name[%s] mode->type[%u] htotal=%u vtotal =%u\n",
                mode->name, mode->type, mode->htotal, mode->vtotal);
	drm_mode_probed_add(connector, mode);
#if CHANGE_FPS_EN
	mode2 = drm_mode_duplicate(connector->dev, &performance_mode_40hz);
	if (!mode2) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_40hz.hdisplay, performance_mode_40hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_40hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);

	mode3 = drm_mode_duplicate(connector->dev, &performance_mode_60hz);
	if (!mode3) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_60hz.hdisplay, performance_mode_60hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_60hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode3);
#endif
	connector->display_info.width_mm = 152;
	connector->display_info.height_mm = 248;

	return 1;
}

static const struct drm_panel_funcs boe_drm_funcs = {
	.disable = boe_disable,
	.unprepare = boe_unprepare,
	.prepare = boe_prepare,
	.enable = boe_enable,
	.get_modes = boe_get_modes,
};

static int boe_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct boe *ctx;
	struct device_node *backlight;
	unsigned int value;
	int ret;

	pr_info("%s+ boe ,hx83102j,vdo,120hz\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct boe), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_u32(dev->of_node, "gate-ic", &value);
	if (ret < 0)
		value = 0;
	else
		ctx->gate_ic = value;

	pr_info(" %d  %s,ctx->ctx->gate_ic = %d \n", __LINE__, __func__,ctx->gate_ic);

	value = 0;
	ret = of_property_read_u32(dev->of_node, "rc-enable", &value);
	if (ret < 0)
		value = 0;
	else {
		ext_params.round_corner_en = value;
#if CHANGE_FPS_EN
		ext_params_40hz.round_corner_en = value;
		ext_params_60hz.round_corner_en = value;
#endif
	}

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	pr_info(" %d  %s,ctx->reset_gpio = %d \n", __LINE__, __func__,ctx->reset_gpio);

	devm_gpiod_put(dev, ctx->reset_gpio);
	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_info(dev, "cannot get bias-gpios 0 %ld\n",
			 PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	pr_info(" %d  %s,ctx->bias_pos = %x \n", __LINE__, __func__,ctx->bias_pos);
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_info(dev, "cannot get bias-gpios 1 %ld\n",
			 PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	pr_info(" %d  %s,ctx->bias_neg = %x \n", __LINE__, __func__,ctx->bias_neg);
	devm_gpiod_put(dev, ctx->bias_neg);


	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &boe_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	pr_info(" %d  %s\n", __LINE__, __func__);
	if (ret < 0)
		return ret;

#endif

	oplus_max_normal_brightness = 3768;
	init_global_exp_backlight();

	/* wanhang */
	//register_device_proc("lcd", "HIMAX83102J", "boe");
	register_device_proc("lcd", "HIMAX83102J", "ice_boe");
	pr_info("%s- boe,hx83102j,vdo,120hz\n", __func__);

	return ret;
}

static int boe_remove(struct mipi_dsi_device *dsi)
{
	struct boe *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id boe_of_match[] = {
	{
	    .compatible = "oplus23662_ice_boe_himax83102j_fhdp_dsi_vdo,lcm",
	},
	{}
};

MODULE_DEVICE_TABLE(of, boe_of_match);

static struct mipi_dsi_driver boe_driver = {
	.probe = boe_probe,
	.remove = boe_remove,
	.driver = {
		.name = "oplus23662_ice_boe_himax83102j_fhdp_dsi_vdo,lcm",
		.owner = THIS_MODULE,
		.of_match_table = boe_of_match,
	},
};

module_mipi_dsi_driver(boe_driver);

MODULE_AUTHOR("zhangxian <xian.zhang@tinno.com>");
MODULE_DESCRIPTION("ICETRON BOE HX83102J VDO 120HZ LCD Panel Driver");
MODULE_LICENSE("GPL v2");

