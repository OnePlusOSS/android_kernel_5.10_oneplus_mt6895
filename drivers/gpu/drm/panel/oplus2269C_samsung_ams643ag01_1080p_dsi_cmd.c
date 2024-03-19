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
#include "../../../misc/mediatek/wl2868c/wl2868c.h"
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/oplus2269C_AMS643AG01_data_hw_roundedpattern.h"
#endif

u32 flag_writ = 0;
EXPORT_SYMBOL(flag_writ);
u32 flag_hbm = 0;
EXPORT_SYMBOL(flag_hbm);

extern int wl2868c_set_ldo_enable(enum WL2868C_SELECT, uint32_t);
extern int wl2868c_set_ldo_disable(enum WL2868C_SELECT);
extern unsigned long oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
extern unsigned int seed_mode;
static int esd_brightness;
static bool aod_state = false;
extern void disp_aal_set_dre_en(int enable);
extern int oplus_dc_alpha;
extern int oplus_dc_enable_real;
extern int oplus_dc_enable;
//int oplus_dc_threshold;
extern void lcdinfo_notify(unsigned long val, void *v);
#define OPLUS_DC_BACKLIGHT_THRESHOLD 1200
#define REGFLAG_CMD       	0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define LCM_BRIGHTNESS_TYPE 2
#define REGFLAG_DELAY     0xFFFC
#define REGFLAG_UDELAY    0xFFFB

enum MODE_ID {
    FHD_SDC60 = 0,
    FHD_SDC90 = 1,
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_gpio;
	struct drm_display_mode *m;

	bool prepared;
	bool enabled;

	unsigned int gate_ic;

	int error;
	bool hbm_en;
	bool hbm_wait;
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[129];
};

struct ba {
	u32 brightness;
	u32 alpha;
};

static struct ba brightness_seed_alpha_lut_dc[] = {
	{0, 2000},
	{10, 1959},
	{30, 1956},
	{60, 1953},
	{90, 1942},
	{100, 1938},
	{140, 1930},
	{200, 1920},
	{300, 1860},
	{400, 1790},
	{500, 1700},
	{600, 1560},
	{700, 1400},
	{800, 1210},
	{900, 980},
	{1000, 690},
	{1100, 380},
	{1200, 0},
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
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
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		return -EINVAL;
	}

    m_vrefresh = drm_mode_vrefresh(m);

    if (m_vrefresh == 60) {
    	ret = FHD_SDC60;
    } else if (m_vrefresh == 90) {
    	ret = FHD_SDC90;
    } else {
    	ret = FHD_SDC90;
    }
	return ret;
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;


static int lcm_panel_bias_regulator_init(void)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static int lcm_panel_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
#endif

static struct LCM_setting_table lcm_normal_to_aod_sam[] = {
	{REGFLAG_CMD,1,{0x28}},
	{REGFLAG_DELAY,34,{}},
	/* AOD Mode ON */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x91,0x01}},
	{REGFLAG_CMD,2,{0x53,0x24}},
	{REGFLAG_CMD,2,{0xBB,0x1D}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	/* CRC Disable*/
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x5D,0x00}},
	{REGFLAG_CMD,2,{0x62,0x01}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_to_normal[] = {
	/* Aod ctrl setting */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,5,{0xBB,0x11,0x0C,0x50,0x10}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	/* Aod mode off */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x91,0x02}},
	{REGFLAG_CMD,2,{0x53,0x20}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,2, {0x53,0x20}},
	{REGFLAG_CMD,3, {0x51,0x00,0x00}},
};

static struct LCM_setting_table lcm_setbrightness_hbm[] = {
	{REGFLAG_CMD,3, {0x51,0x00,0x00}},
	{REGFLAG_CMD,2, {0x53,0xE0}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	/*HBM ON*/
	{REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2, {0x53,0xE0}},
	{REGFLAG_CMD,3, {0x51,0x0F,0xFF}},
	{REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode0[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x00}},
	{REGFLAG_CMD,2,{0x62, 0x01}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	//{REGFLAG_CMD,22,{0x62,0xDA,0x00,0x00,0x0C,0xD7,0x03,0x09,0x05,0xC4,0x16,0xFA,0xE2,0xF7,0x00,0xE4,0xE9,0xE6,0x03,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,22,{0x62,0xFF,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode1[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x00}},
	{REGFLAG_CMD,2,{0x62, 0x01}},
	{REGFLAG_CMD,3,{0xB0, 0x01, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xFF,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode2[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x00}},
	{REGFLAG_CMD,2,{0x62, 0x01}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xFF,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode3[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x00}},
	{REGFLAG_CMD,2,{0x62, 0x01}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xFF,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	struct drm_display_mode *m = ctx->m;

	mode_id = get_mode_enum(m);
	pr_err("%s, mode_id = %d\n", __func__, mode_id);
	/*PPS Setting*/
	lcm_dcs_write_seq_static(ctx,0x9E, 0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x60, 0x04, 0x38,\
                 0x00, 0x1E, 0x02, 0x1C, 0x02, 0x1C, 0x02, 0x00, 0x02, 0x0E,\
                 0x00, 0x20, 0x02, 0xE3, 0x00, 0x07, 0x00, 0x0C, 0x03, 0x50,\
                 0x03, 0x64, 0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,\
                 0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38, 0x46, 0x54,\
                 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B, 0x7D, 0x7E, 0x01, 0x02,\
                 0x01, 0x00, 0x09, 0x40, 0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA,\
                 0x19, 0xF8, 0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,\
                 0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4, 0x00, 0x00,\
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xC2, 0x14);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0x01);
	/* Display On Setting */
	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(120);
	/* TE vsync ON */
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x44, 0x09, 0x60);
	/* CASET/PASET Setting */
	lcm_dcs_write_seq_static(ctx, 0x2A, 0x00, 0x00, 0x04, 0x37);
	lcm_dcs_write_seq_static(ctx, 0x2B, 0x00, 0x00, 0x09, 0x5F);
	/* FQ CON Setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x27, 0xF2);
	lcm_dcs_write_seq_static(ctx, 0xF2, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/* Freuency Setting */
	if (mode_id == 0) {
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x00, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	} else if (mode_id == 1) {
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x08, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	} else {
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x08, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	}
	/*Elvss offset setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0xB4, 0x63);
	lcm_dcs_write_seq_static(ctx, 0x63, 0xEA, 0xFE, 0xDF, 0xF0, 0xFF, 0x3F, 0xF6, 0xFF, 0x9F, 0xFC, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/* Dimming on Hbm off*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x91, 0x63);
	lcm_dcs_write_seq_static(ctx, 0x63, 0x30);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/* Seed Setting */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x72, 0x2C, 0x21, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xF8, 0x00);
	if (seed_mode == 101){ //101--lcm_seed_mode1  sRGB
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x5D, 0x00);
		lcm_dcs_write_seq_static(ctx, 0x62, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB0, 0x01, 0x62);
		lcm_dcs_write_seq_static(ctx, 0x62, 0xFF,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	}else if (seed_mode == 100){ //100--lcm_seed_mode2  DCI-P3
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x5D, 0x00);
		lcm_dcs_write_seq_static(ctx, 0x62, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB0, 0x2B, 0x62);
		lcm_dcs_write_seq_static(ctx, 0x62, 0xFF,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	}else if (seed_mode ==103){ //103--lcm_seed_mode3  DCI-P3
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x5D, 0x00);
		lcm_dcs_write_seq_static(ctx, 0x62, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB0, 0x2B, 0x62);
		lcm_dcs_write_seq_static(ctx, 0x62, 0xFF,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	} else { //102--lcm_seed_mode0  DCI-P3
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x5D, 0x00);
		lcm_dcs_write_seq_static(ctx, 0x62, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB0, 0x2B, 0x62);
		//lcm_dcs_write_seq_static(ctx, 0x62, 0xDA,0x00,0x00,0x0C,0xD7,0x03,0x09,0x05,0xC4,0x16,0xFA,0xE2,0xF7,0x00,0xE4,0xE9,0xE6,0x03,0xFF,0xFF,0xFF);
		lcm_dcs_write_seq_static(ctx, 0x62, 0xFF,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	}
	lcm_dcs_write_seq_static(ctx, 0x72, 0x2C, 0x01, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	msleep(20);
	/* Display On*/
	lcm_dcs_write_seq_static(ctx, 0x29);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x0A, 0xB5);
	lcm_dcs_write_seq_static(ctx, 0xB5, 0x48, 0x60, 0x00, 0x00, 0x03);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(10);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(120);


	ctx->error = 0;
	ctx->prepared = false;


#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#endif
	ctx->hbm_en = false;

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		pr_err("%s failed!!\n", __func__);
		return 0;
	}
	pr_err("debug for lcm %s\n", __func__);
	aod_state = false;

	lcm_panel_init(ctx);
	pr_err("%s + lcm_panel_init,resume status\n", __func__);
	/*if(power_mode == 2)
	{
		DDPINFO("%s + lcm_panel_init,resume status\n", __func__);
		lcm_panel_init(ctx);
	}*/

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 163070,
	.hdisplay = 1080,
	.hsync_start = 1080 + 10,
	.hsync_end = 1080 + 10 + 24,
	.htotal = 1080 + 10 + 24 + 10,
	.vdisplay = 2400,
	.vsync_start = 2400 + 8,
	.vsync_end = 2400 + 8 + 4,
	.vtotal = 2400 + 8 + 4 + 6,
};

static const struct drm_display_mode performance_mode = {
	.clock = 243734,
	.hdisplay = 1080,
	.hsync_start = 1080 + 8,
	.hsync_end = 1080 + 8 + 24,
	.htotal = 1080 + 8 + 24 + 8,
	.vdisplay = 2400,
	.vsync_start = 2400 + 8,
	.vsync_end = 2400 + 8 + 4,
	.vtotal = 2400 + 8 + 4 + 6,
};
#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
#if 0
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
#endif
	return 0;
}

static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int seed_mode)
{
	int i, k;
	struct LCM_setting_table *seed_table = NULL;

	pr_err("debug for lcm %s, seed_mode=%d\n", __func__, seed_mode);

	if (seed_mode == 102)
		seed_table = kmemdup(lcm_seed_mode0, sizeof(lcm_seed_mode0), GFP_KERNEL);
	else if (seed_mode == 101)
		seed_table = kmemdup(lcm_seed_mode1, sizeof(lcm_seed_mode1), GFP_KERNEL);
	else if (seed_mode == 100)
		seed_table = kmemdup(lcm_seed_mode2, sizeof(lcm_seed_mode2), GFP_KERNEL);
	else if (seed_mode == 103)
		seed_table = kmemdup(lcm_seed_mode3, sizeof(lcm_seed_mode3), GFP_KERNEL);

	if (!seed_table)
		return 0;

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++) {
		if ((seed_table[i].para_list[0] == 0x62) && (seed_table[i].count == 22)){
			for (k = 1; k < seed_table[i].count; k++) {
				seed_table[i].para_list[k]= seed_table[i].para_list[k] * (2000 - oplus_dc_alpha) / 2000;
			}
		}
	}

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++){
		cb(dsi, handle, seed_table[i].para_list, seed_table[i].count);
	}

	kfree(seed_table);

	//memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;

	pr_err("debug for lcm %s\n", __func__);

	/* Switch back to VDO mode */
	for (i = 0; i < (sizeof(lcm_aod_to_normal) / sizeof(struct LCM_setting_table)); i++) {
		unsigned cmd;
		cmd = lcm_aod_to_normal[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
					msleep(lcm_aod_to_normal[i].count);
				break;

			case REGFLAG_UDELAY:
				udelay(lcm_aod_to_normal[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				//judge last backlight hbm or not
				if (flag_hbm == 1) {
					if (lcm_aod_to_normal[i].para_list[0] == 0x53) {
						//send temp 0x53 order, in order not to change original lcm_aod_to_normal
						unsigned char tmp1[] = {0x53, 0xE0};
						cb(dsi, handle, tmp1, ARRAY_SIZE(tmp1));
						continue;
					}
				}
				cb(dsi, handle, lcm_aod_to_normal[i].para_list, lcm_aod_to_normal[i].count);
		}
	}
	panel_set_seed(dsi, cb, handle, seed_mode);
	aod_state = false;
	return 0;
}

static int panel_doze_enable_start(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	//cmd = 0x10;
	//cb(dsi, handle, &cmd, 1);
	msleep(16);

	return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i=0;
	pr_err("debug for lcm %s\n", __func__);
	aod_state = true;

	for (i = 0; i < (sizeof(lcm_normal_to_aod_sam) / sizeof(struct LCM_setting_table)); i++) {
		unsigned cmd;
		cmd = lcm_normal_to_aod_sam[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				msleep(lcm_normal_to_aod_sam[i].count);
				break;

			case REGFLAG_UDELAY:
				udelay(lcm_normal_to_aod_sam[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				cb(dsi, handle, lcm_normal_to_aod_sam[i].para_list, lcm_normal_to_aod_sam[i].count);
		}
	}

	return 0;
}

static int panel_doze_post_disp_on(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
    char post_backlight_on1[] = {0x29};
	pr_err("debug for lcm %s\n", __func__);
	cb(dsi, handle, post_backlight_on1, ARRAY_SIZE(post_backlight_on1));
	return 0;
}


static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	unsigned char id[3] = {0x00, 0x00, 0x00};
	ssize_t ret;

	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	pr_info("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	pr_info("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);

	return 0;
}

static int interpolate(int x, int xa, int xb, int ya, int yb)
{
	int bf, factor, plus;
	int sub = 0;

	bf = 2 * (yb - ya) * (x - xa) / (xb - xa);
	factor = bf / 2;
	plus = bf % 2;
	if ((xa - xb) && (yb - ya))
		sub = 2 * (x - xa) * (x - xb) / (yb - ya) / (xa - xb);

	return ya + factor + plus + sub;
}

static int oplus_seed_bright_to_alpha(int brightness)
{
	int level = ARRAY_SIZE(brightness_seed_alpha_lut_dc);
	int i = 0;
	int alpha;

	for (i = 0; i < ARRAY_SIZE(brightness_seed_alpha_lut_dc); i++) {
		if (brightness_seed_alpha_lut_dc[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = brightness_seed_alpha_lut_dc[0].alpha;
	else if (i == level)
		alpha = brightness_seed_alpha_lut_dc[level - 1].alpha;
	else
		alpha = interpolate(brightness,
			brightness_seed_alpha_lut_dc[i-1].brightness,
			brightness_seed_alpha_lut_dc[i].brightness,
			brightness_seed_alpha_lut_dc[i-1].alpha,
			brightness_seed_alpha_lut_dc[i].alpha);

	return alpha;
}

static int oplus_lcm_dc_backlight(void *dsi, dcs_write_gce cb,void *handle,unsigned int level,int hbm_en)
{
	int i, k;
	struct LCM_setting_table *seed_table = NULL;
	int seed_alpha = oplus_seed_bright_to_alpha(level);
	static int last_level = OPLUS_DC_BACKLIGHT_THRESHOLD;
	static int last_seed_alpha = 0;
	if (!oplus_dc_enable_real || hbm_en || level >= OPLUS_DC_BACKLIGHT_THRESHOLD ||
	    level < 4) {
		goto dc_disable;
	}else
	{
		lcm_seed_mode0[4].para_list[1] = 0x86;
		lcm_seed_mode0[5].para_list[1] = 0x00;
		lcm_seed_mode1[4].para_list[1] = 0x06;
		lcm_seed_mode1[5].para_list[1] = 0x00;
		lcm_seed_mode2[4].para_list[1] = 0x86;
		lcm_seed_mode2[5].para_list[1] = 0x00;
		lcm_seed_mode3[4].para_list[1] = 0x86;
		lcm_seed_mode3[5].para_list[1] = 0x00;
		printk("%s:%d,DC data setting success!" ,__func__,__LINE__);
	}

	if (((level < last_level) && (seed_alpha < last_seed_alpha)) ||
		((level > last_level) && (seed_alpha > last_seed_alpha))){
		seed_alpha = last_seed_alpha;
	}
	last_level = level;
	last_seed_alpha = seed_alpha;

	if (oplus_dc_alpha == seed_alpha)
		goto dc_enable;

	if (seed_mode == 102)
		seed_table = kmemdup(lcm_seed_mode0, sizeof(lcm_seed_mode0), GFP_KERNEL);
	else if (seed_mode == 101)
		seed_table = kmemdup(lcm_seed_mode1, sizeof(lcm_seed_mode1), GFP_KERNEL);
	else if (seed_mode == 100)
		seed_table = kmemdup(lcm_seed_mode2, sizeof(lcm_seed_mode2), GFP_KERNEL);
	else if (seed_mode == 103)
		seed_table = kmemdup(lcm_seed_mode3, sizeof(lcm_seed_mode3), GFP_KERNEL);

	if (!seed_table)
		goto dc_disable;

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++) {
		if ((seed_table[i].para_list[0] == 0x62) && (seed_table[i].count == 22)){
			for (k = 1; k < seed_table[i].count; k++) {
				seed_table[i].para_list[k]= seed_table[i].para_list[k] * (2000 - seed_alpha) / 2000;
			}
		}
	}

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++){
		cb(dsi, handle, seed_table[i].para_list, seed_table[i].count);
	}

	kfree(seed_table);
	if (!oplus_dc_alpha)
		pr_err("Enter DC");

	oplus_dc_alpha = seed_alpha;

dc_enable:
	return OPLUS_DC_BACKLIGHT_THRESHOLD;

dc_disable:
	lcm_seed_mode0[4].para_list[1] = 0x00;
	lcm_seed_mode0[5].para_list[1] = 0x01;
	lcm_seed_mode1[4].para_list[1] = 0x00;
	lcm_seed_mode1[5].para_list[1] = 0x01;
	lcm_seed_mode2[4].para_list[1] = 0x00;
	lcm_seed_mode2[5].para_list[1] = 0x01;
	lcm_seed_mode3[4].para_list[1] = 0x00;
	lcm_seed_mode3[5].para_list[1] = 0x01;
	if (oplus_dc_alpha && level != 0) {
		if (seed_mode == 102){ //DCI-P3
			for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode0[i].para_list, lcm_seed_mode0[i].count);
			}
		} else if (seed_mode == 101){ //sRGB
			for (i = 0; i < sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode1[i].para_list, lcm_seed_mode1[i].count);
			}
		} else if (seed_mode == 100){
			for (i = 0; i < sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode2[i].para_list, lcm_seed_mode2[i].count);
			}
		} else if (seed_mode == 103){
			for (i = 0; i < sizeof(lcm_seed_mode3)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode3[i].para_list, lcm_seed_mode3[i].count);
			}
		}
		pr_err("exit DC");
	}

	oplus_dc_alpha = 0;

	return level;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
    unsigned int mapped_level = 0;
	char bl_tb0[] = {0x51, 0x03, 0xFF};
	char bl_tb1[] = {0x53, 0x20};
	char post_backlight_on1[] = {0x29};
	if (!cb)
		return -1;
	if (level > 2047 && level < 3727)
		level = 2047;
	mapped_level = oplus_lcm_dc_backlight(dsi,cb,handle, level, 0);
	//mapped_level = level;
	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;
	esd_brightness = level;
	//DDPPR_ERR("flag_writ=%d,mapped_level=%d\n",flag_writ,mapped_level);
	pr_err("debug for display panel backlight value,func:=%s,level :=%d, mapped_level := %d\n", __func__, level, mapped_level);
 	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}
	if(level == 1 || mapped_level == 1){
		pr_info("enter aod mode, ignore set backlight to 1\n");
		if (aod_state == 1) {
			cb(dsi, handle, post_backlight_on1, ARRAY_SIZE(post_backlight_on1));
		}
	} else if (level <= BRIGHTNESS_HALF){
	    if(flag_hbm == 1){
			bl_tb1[1] = 0x20;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 0;
		}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if (level > BRIGHTNESS_HALF && level <= BRIGHTNESS_MAX){
	    if(flag_hbm == 0){
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 1;
			}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	}

	return 0;

}

static int lcm_set_dc_backlight(void *dsi, dcs_write_gce cb,
           void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xFF};

	pr_err("lks  lcm_set_dc_backlight ,backlight = %d\n", oplus_display_brightness);
	bl_tb0[1] = oplus_display_brightness >> 8;
	bl_tb0[2] = oplus_display_brightness & 0xFF;

	oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;

}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x07, 0xff};

	//pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static void lcm_setbrightness(void *dsi,
			      dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	unsigned int hbm_brightness = 0;
	int i = 0;

	printk("[soso] %s level is %d\n", __func__, level);

	if (level > BRIGHTNESS_HALF) {
		hbm_brightness = level - BRIGHTNESS_HALF;
		BL_LSB = hbm_brightness >> 8;
		BL_MSB = hbm_brightness & 0xFF;

		lcm_setbrightness_hbm[0].para_list[1] = BL_LSB;
		lcm_setbrightness_hbm[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_hbm)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_hbm[i].para_list, lcm_setbrightness_hbm[i].count);
		}
	} else {
		BL_LSB = level >> 8;
		BL_MSB = level & 0xFF;

		lcm_setbrightness_normal[1].para_list[1] = BL_LSB;
		lcm_setbrightness_normal[1].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_normal)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_normal[i].para_list, lcm_setbrightness_normal[i].count);
		}
	}
}

static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;
	//int level = 0;
	if (!cb)
		return -1;

	pr_err("oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		printk("[soso] %s : %d ! backlight %d !\n",__func__, hbm_mode, oplus_display_brightness);
	}

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	//char hbm_tb[] = {0x53, 0xe0};
	struct lcm *ctx = panel_to_lcm(panel);
	int i = 0;

	if (!cb)
		return -1;
	if (ctx->hbm_en == en)
		goto done;

	pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		if (oplus_display_brightness <= BRIGHTNESS_HALF)
			flag_hbm = 0;
		else
			flag_hbm = 1;
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
		printk("[soso] %s : %d !  backlight %d !\n",__func__, en, oplus_display_brightness);
	}
	lcdinfo_notify(1, &en);
	ctx->hbm_en = en;
	ctx->hbm_wait = true;
done:
	return 0;
}


static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*state = ctx->hbm_en;
}

static void panel_hbm_set_state(struct drm_panel *panel, bool state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->hbm_en = state;
}

static void panel_hbm_get_wait_state(struct drm_panel *panel, bool *wait)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*wait = ctx->hbm_wait;
}

static bool panel_hbm_set_wait_state(struct drm_panel *panel, bool wait)
{
	struct lcm *ctx = panel_to_lcm(panel);
	bool old = ctx->hbm_wait;

	ctx->hbm_wait = wait;
	return old;
}

static struct mtk_panel_params ext_params = {
	.pll_clk = 299,
	.data_rate = 598,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_aod_status_again_skip = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9D, .mask_list[0] = 0x9D,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0E, .count = 1, .para_list[0] = 0x80,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	},
	.skip_unnecessary_switch = true,
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
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 30,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 739,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 848,
		.slice_bpg_offset = 868,
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
		},


#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.hbm_en_time = 1,
	.hbm_dis_time = 0,
	.oplus_need_hbm_wait = 1,
	.oplus_hbm_on_sync_with_flush = 1,
	.oplus_hbm_off_sync_with_flush = 0,
	//.oplus_wait_te_num = 3,
	.oplus_samsung_panel =0,
	//before hbm need wait
	.oplus_need_before_hbm_wait= 1,
	//delay TE num
	.before_hbm_en_time = 2,
	.before_hbm_dis_time = 0,
	//delay time: us
	.before_hbm_en_delay_time =8000,
	.oplus_dc_then_hbm_on = 0,
	.oplus_display_global_dre = 1,
	.use_free_pointer_check = 1,
	.dyn_fps = {
            .switch_en = 1, .vact_timing_fps = 60,
        },
	.dyn = {
		.switch_en = 1,
		.pll_clk = 296,
		.data_rate = 592,
	},
};
static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 299,
	.data_rate = 598,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_aod_status_again_skip = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9D, .mask_list[0] = 0x9D,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0E, .count = 1, .para_list[0] = 0x80,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	},
	.skip_unnecessary_switch = true,
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
        .pic_height = 2400,
        .pic_width = 1080,
        .slice_height = 30,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 739,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 848,
        .slice_bpg_offset = 868,
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
        },

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.hbm_en_time = 1,
	.hbm_dis_time = 0,
	.oplus_need_hbm_wait = 0,
	.oplus_hbm_on_sync_with_flush = 0,
	.oplus_hbm_off_sync_with_flush = 0,
	.oplus_samsung_panel =0,
	.oplus_need_before_hbm_wait=1,
	.before_hbm_en_time = 2,
	.before_hbm_dis_time = 0,
	.before_hbm_en_delay_time =8000,
	.oplus_dc_then_hbm_on = 0,
	.oplus_display_global_dre = 1,
	.use_free_pointer_check = 1,

	.dyn_fps = {
            .switch_en = 1, .vact_timing_fps = 90,
        },
	.dyn = {
		.switch_en = 1,
		.pll_clk = 296,
		.data_rate = 592,
	},
};
struct drm_display_mode *get_mode_by_id_hfp(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;
	printk("wangc  get_mode_by_id_hfp enter 633\n");
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
	printk(" mtk_panel_ext_param_set enter \n");
	if (drm_mode_vrefresh(m) == 60)
		ext->params = &ext_params;
	else if (drm_mode_vrefresh(m) == 90)
		ext->params = &ext_params_90hz;
	else
		ret = 1;
	return ret;
}

static void mode_switch_60_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);
	if (stage == BEFORE_DSI_POWERDOWN) {
	} else if (stage == AFTER_DSI_POWERON) {
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x08, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
		usleep_range(3*1000, 4*1000);
    }
}
static void mode_switch_90_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);
	if (stage == BEFORE_DSI_POWERDOWN) {
	} else if (stage == AFTER_DSI_POWERON) {
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x00, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
		usleep_range(3*1000, 4*1000);
	}
}
static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, dst_mode);
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);
	printk(" mode_switch enter \n");
	if (drm_mode_vrefresh(m) == 90) { /* 60 switch to 90 */
		mode_switch_60_to_90(panel, stage);
			printk("mode_switch enter 90hz \n");
	} else if (drm_mode_vrefresh(m) == 60) { /* 90 switch to 60 */
		mode_switch_90_to_60(panel, stage);
		printk(" mode_switch enter 60hz \n");
	} else
		ret = 1;

	ctx->m = m;
	return ret;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_err("lsy1-debug for lcm %s, ctx->prepared=%d\n", __func__, ctx->prepared);

	if (ctx->prepared)
		return 0;

	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	msleep(5);
	wl2868c_set_ldo_enable(LDO3,3000);
	msleep(5);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(5);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	pr_err("lsy5-debug for lcm %s, ctx->prepared=%d\n", __func__, ctx->prepared);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(5000, 5100);
	return 0;
}
static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_err("lsy2-debug for lcm %s  ctx->unprepared %d \n", __func__,ctx->prepared);

	if (ctx->prepared)
		return 0;

	usleep_range(5000, 5100);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(5);
	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 0);
	msleep(5);
	wl2868c_set_ldo_disable(LDO3);
	msleep(10);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);

	pr_err("lsy6-debug for lcm %s, ctx->prepared=%d\n", __func__, ctx->prepared);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	msleep(13);

	return 0;
}
static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.doze_enable_start = panel_doze_enable_start,
	.doze_post_disp_on = panel_doze_post_disp_on,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.set_hbm = lcm_set_hbm,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.hbm_get_state = panel_hbm_get_state,
	.hbm_set_state = panel_hbm_set_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.set_seed = panel_set_seed,
	.set_dc_backlight = lcm_set_dc_backlight,
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

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	mode2 = drm_mode_duplicate(connector->dev, &performance_mode);
	if (!mode2) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			performance_mode.hdisplay, performance_mode.vdisplay,
			drm_mode_vrefresh(&performance_mode));
		return -ENOMEM;
	}
	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode2);
	connector->display_info.width_mm = 67;
	connector->display_info.height_mm = 149;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	unsigned int value;
	int ret;
	int oplus_dc_threshold = 260;

	pr_info("%s+\n", __func__);
	oplus_dc_threshold = OPLUS_DC_BACKLIGHT_THRESHOLD;
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

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_u32(dev->of_node, "gate-ic", &value);
	if (ret < 0)
		value = 0;
	else
		ctx->gate_ic = value;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}
	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	msleep(2);
	//lcm_panel_ldo3_enable(ctx->dev);
	msleep(5);
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	register_device_proc("lcd", "AMS643AG01", "oplus2269C");
	//disp_aal_set_dre_en(1);
	pr_info("%s-\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
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

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "oplus2269C_samsung_ams643ag01_1080p_dsi_cmd,lcm", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus2269C_samsung_ams643ag01_1080p_dsi_cmd,lcm",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("oplus2269C_samsung_ams643ag01_1080p_dsi_cmd CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");

