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
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <soc/oplus/device_info.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "../mediatek/mediatek_v2/mtk_corner_pattern/data_hw_roundedpattern_r_lisa.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/data_hw_roundedpattern_l_lisa.h"
#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#if defined(CONFIG_PXLW_IRIS)
#include "../mediatek/mediatek_v2/iris_api.h"
#include "../mediatek/mediatek_v2/iris_mtk_api.h"
#endif
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#include <mt-plat/mtk_boot_common.h>
#include "../../../misc/mediatek/lpm/inc/lpm_module.h"
#include "../../../misc/mediatek/lpm/modules/include/mt6895/mt6895_pwr_ctrl.h"
#include "../oplus/oplus_drm_disp_panel.h"

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2
#define DC_BAKLIGHT_THERSHOLD   2313
extern int oplus_dc_alpha;
extern int oplus_dc_enable_real;
static unsigned int esd_brightness = 1023;
static u32 flag_hbm = 0;
extern unsigned int oplus_display_brightness;
static bool aod_state = false;
extern void disp_aal_set_dre_en(int enable);
extern unsigned int oplus_enhance_mipi_strength;
extern bool pq_trigger;
extern bool g_is_silky_panel;
extern bool is_forbid_enter_ulps_aod_state;
static bool hbm_flag = false;
/*struct ba {
       u32 brightness;
       u32 alpha;
};

static struct ba brightness_seed_alpha_lut_dc[] = {
       {4, 20},
       {120, 34},
       {240, 47},
       {360, 68},
       {480, 92},
       {590, 138},
       {700, 195},
       {810, 248},
       {920, 313},
       {1030, 400},
       {1140, 511},
       {1240, 609},
       {1340, 705},
       {1440, 844},
       {1530, 970},
       {1610, 1083},
       {1680, 1187},
       {1740, 1279},
       {1800, 1361},
       {1850, 1425},
       {1900, 1521},
       {1950, 1637},
       {2000, 1715},
       {2070, 1850},
       {2140, 1996},
       {2210, 2120},
       {2260, 2218},
       {2300, 2285},
       {2313, 2313},
};
*/
struct LCM_setting_table {
  	unsigned int cmd;
 	unsigned char count;
 	unsigned char para_list[128];
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
        {REGFLAG_CMD,3, {0x51, 0x00, 0x00}},
//        {REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
//        {REGFLAG_CMD,2, {0xB2, 0x11}},
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
        {REGFLAG_CMD,3, {0x51, 0x0f, 0xff}},
//        {REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
//        {REGFLAG_CMD,2, {0xB2, 0x01}},
};

static struct LCM_setting_table lcm_seed_setting[] = {
	{REGFLAG_CMD,6,{0xF0, 0x55,0xAA,0x52,0x08,0x09}},
	{REGFLAG_CMD,108,{0xE0,0x00,0x00,0xA5,0x4E,0x47,0x00,0xFF,0x0F,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x85,0x10,0x0A,0xC0,0xC0,0x0E,0xF8,0x60,0x0F,0x18,0x61,0x0E,0xDC,0x00,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x85,0x10,0x0A,0xC0,0xC0,0x0E,0xF8,0x60,0x0F,0x18,0x01,0x0F,0x00,0x01,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x80,0x79,0x75,0x71,0x6D,0x69,0x65,0x5D,0x55,0x49,0x35,0x19,0x14,0x12,0x0D,0x01,0x00,0x00,0x00,0x00,0x00}},
	{REGFLAG_CMD,6,{0xF0, 0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2,{0xB2,0x91}},
	{REGFLAG_CMD,2,{0x6F,0x0F}},
	{REGFLAG_CMD,9,{0xB2,0x00,0x00,0x09,0x08,0x29,0x08,0x0F,0xFF}},

	{REGFLAG_CMD,37,{0xB3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x09,0x09,0x0A,0x0B,0x62,0x0B,0x63,0x0D,0xBB,0x0D,0xBB,0x0F,0xFF}},

	{REGFLAG_CMD,2,{0x6F,0x2C}},
	{REGFLAG_CMD,39,{0xB3,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x30,0x00,0x20,0x00,0x20,0x00,0x20}},
	{REGFLAG_CMD,39,{0xB4,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x48,0x00,0x30,0x00,0x30,0x00,0x30}},
	{REGFLAG_CMD,2,{0x6F,0x30}},
	{REGFLAG_CMD,39,{0xB4,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x24,0x00,0x18,0x00,0x18,0x00,0x18}},

	{REGFLAG_CMD,2,{0x6F,0xAC}},
	{REGFLAG_CMD,21,{0xB2,0x01,0xE1,0x03,0x6A,0x03,0x6A,0x06,0x6F,0x09,0x98,0x09,0x98,0x0C,0xF7,0x0C,0xF7,0x0F,0xFF,0x0F,0xFF}},
	{REGFLAG_CMD,2,{0x6F,0xC0}},
	{REGFLAG_CMD,21,{0xB2,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF}},
	{REGFLAG_CMD,2,{0x6F,0xD4}},
	{REGFLAG_CMD,21,{0xB2,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF}},
	{REGFLAG_CMD,2,{0x6F,0xE8}},
	{REGFLAG_CMD,11,{0xB2,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF}},
	{REGFLAG_CMD,2,{0x6F,0x52}},
	{REGFLAG_CMD,21,{0xB3,0x01,0xCF,0x01,0xD0,0x03,0x9D,0x05,0x6B,0x05,0x6C,0x07,0x39,0x07,0x3A,0x09,0x08,0x09,0x08,0x09,0x09}},
	{REGFLAG_CMD,2,{0x6F,0x66}},
	{REGFLAG_CMD,21,{0xB3,0x09,0x0A,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF}},
	{REGFLAG_CMD,2,{0x6F,0x7A}},
	{REGFLAG_CMD,21,{0xB3,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF}},
	{REGFLAG_CMD,2,{0x6F,0x8E}},
	{REGFLAG_CMD,9,{0xB3,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0xFF}},
//	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_seed_exit[] = {
	{REGFLAG_CMD,6,{0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2,{0xB2,0x11}},
	{REGFLAG_CMD,2,{0x6F,0x0F}},
	{REGFLAG_CMD,9,{0xB2,0x50,0x20,0x49,0x08,0x29,0x08,0x2F,0xFF}},
	{REGFLAG_CMD,37,{0xB3,0x00,0x01,0x01,0xCF,0x01,0xD0,0x03,0x9D,0x03,0x9E,0x05,0x6B,0x05,0x6C,0x07,0x39,0x07,0x3A,0x09,0x08,0x09,0x08,0x09,0x09,0x09,0x0A,0x0B,0x62,0x0B,0x63,0x0D,0xBB,0x0D,0xBB,0x0F,0xFF}},
	{REGFLAG_CMD,2,{0x6F,0x2C}},
        {REGFLAG_CMD,39,{0xB3,0x0C,0xC0,0x0C,0x1C,0x0B,0x25,0x0B,0x25,0x0A,0x18,0x0A,0x18,0x08,0xFD,0x08,0xFD,0x05,0x89,0x05,0x89,0x00,0x40,0x00,0x40,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x30,0x00,0x20,0x00,0x20,0x00,0x20}},
	{REGFLAG_CMD,39,{0xB4,0x13,0x20,0x12,0x2A,0x10,0xB8,0x10,0xB8,0x0F,0x24,0x0F,0x24,0x0D,0x7C,0x0D,0x7C,0x08,0x4E,0x08,0x4E,0x00,0x60,0x00,0x60,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x48,0x00,0x30,0x00,0x30,0x00,0x30}},
	{REGFLAG_CMD,2,{0x6F,0x30}},
        {REGFLAG_CMD,39,{0xB4,0x09,0x90,0x09,0x15,0x08,0x5C,0x08,0x5C,0x07,0x92,0x07,0x92,0x06,0xBE,0x06,0xBE,0x04,0x27,0x04,0x27,0x00,0x30,0x00,0x30,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x24,0x00,0x18,0x00,0x18,0x00,0x18}},
	{REGFLAG_CMD,6,{0xF0,0x55,0xAA,0x52,0x08,0x09}},
	{REGFLAG_CMD,108,{0xE0,0x00,0x00,0xA5,0x4E,0x47,0x00,0xFF,0x0F,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x85,0x10,0x0A,0xC0,0xC0,0x0E,0xF8,0x60,0x0F,0x24,0x91,0x1A,0x18,0x02,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x85,0x10,0x0A,0xC0,0xC0,0x0E,0xF8,0x60,0x0F,0x24,0x01,0x28,0x00,0x04,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x10,0x00,0x01,0x80,0x79,0x75,0x71,0x6D,0x69,0x65,0x5D,0x55,0x49,0x35,0x19,0x14,0x12,0x0D,0x01,0x00,0x00,0x00,0x00,0x00}},
//	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_to_normal[] = {
	{REGFLAG_CMD,6,{0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2,{0x6F,0x0D}},
	{REGFLAG_CMD,2,{0xB5,0x4F}},
	{REGFLAG_CMD,2,{0x65,0x00}},
	{REGFLAG_CMD,1,{0x38}},
	{REGFLAG_CMD,1,{0x2C}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_high_mode[] = {
	{REGFLAG_CMD, 5, {0x51,0x00,0x00,0x0F,0xFE}},
	{REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD, 2, {0x6F,0x2E}},
	{REGFLAG_CMD, 3, {0xB4,0x04,0xC8}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
	{REGFLAG_CMD, 5, {0x51,0x00,0x00,0x0F,0xFE}},
	{REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD, 2, {0x6F,0x2E}},
	{REGFLAG_CMD, 3, {0xB4,0x08,0x9C}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct lcm_pmic_info {
	struct regulator *reg_vufs18;
	struct regulator *reg_vmch3p0;
};

struct jdi {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *vddr1p5_enable_gpio;
	struct gpio_desc *te_switch_gpio,*te_out_gpio;
	struct gpio_desc *pw_1p8_gpio, *pw_reset_gpio;
	bool prepared;
	bool enabled;
	int error;
};

#define jdi_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,                          \
				 "DCS sequence too big for stack");            \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define jdi_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct jdi *panel_to_jdi(struct drm_panel *panel)
{
	return container_of(panel, struct jdi, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int jdi_dcs_read(struct jdi *ctx, u8 cmd, void *data, size_t len)
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

static void jdi_panel_get_data(struct jdi *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = jdi_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif
/*static int interpolate(int x, int xa, int xb, int ya, int yb)
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
}*/
static void jdi_dcs_write(struct jdi *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && iris_is_pt_mode()) {
		ret = 0;
		iris_panel_cmd_passthrough((int)*addr, len -1, (u8 *)&data[1], NULL, 0, 0);
		pr_err("%s %p name =%s\n", __func__, dsi, dsi->name);
	} else {
#endif
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
#if defined(CONFIG_PXLW_IRIS)
	}
#endif
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}
#if 1
#if 0
static struct regulator *vmc_ldo;
static int lcm_panel_vmc_ldo_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;
    pr_err("get lcm_panel_ldo3_regulator_init\n");

	/* please only get regulator once in a driver */
	vmc_ldo = devm_regulator_get(dev, "vmcldo");
	if (IS_ERR(vmc_ldo)) { /* handle return value */
		ret = PTR_ERR(vmc_ldo);
		pr_err("get vmc_ldo fail, error: %d\n", ret);
		//return ret;
	}
	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_vmc_ldo_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_vmc_ldo_regulator_init(dev);

	/* set voltage with min & max*/
	ret = regulator_set_voltage(vmc_ldo, 3000000, 3000000);
	if (ret < 0)
		pr_err("set voltage vmc_ldo fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(vmc_ldo);
	if (ret < 0)
		pr_err("enable regulator vmc_ldo fail, ret = %d\n", ret);
	retval |= ret;
    pr_err("get lcm_panel_vmc_ldo_enable\n");

	return retval;
}

static int lcm_panel_vmc_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_vmc_ldo_regulator_init(dev);

	ret = regulator_disable(vmc_ldo);
	if (ret < 0)
		pr_err("disable regulator vmc_ldo fail, ret = %d\n", ret);
	retval |= ret;
    pr_err("disable regulator vmc_ldo\n");

	return retval;
}
#endif
static struct regulator *vufs_ldo;
static int regulator_vufs_inited;
static int lcm_panel_vufs_ldo_regulator_init(struct device *dev)
{
        int ret = 0;

        if (regulator_vufs_inited)
                return ret;
    pr_err("get lcm_panel_vufs_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        vufs_ldo = devm_regulator_get(dev, "vufsldo");
        if (IS_ERR(vufs_ldo)) { /* handle return value */
                ret = PTR_ERR(vufs_ldo);
                pr_err("get vufs_ldo fail, error: %d\n", ret);
                //return ret;
        }
        regulator_vufs_inited = 1;
        return ret; /* must be 0 */

}

static int lcm_panel_vufs_ldo_enable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_vufs_ldo_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vufs_ldo)) {
        	ret = regulator_set_voltage(vufs_ldo, 1800000, 1800000);
        	if (ret < 0)
                	pr_err("set voltage vufs_ldo fail, ret = %d\n", ret);
        	retval |= ret;
	}

        /* enable regulator */
	if (!IS_ERR_OR_NULL(vufs_ldo)) {
        	ret = regulator_enable(vufs_ldo);
        	if (ret < 0)
                	pr_err("enable regulator vufs_ldo fail, ret = %d\n", ret);
        	retval |= ret;
	}
    	pr_err("get lcm_panel_vufs_ldo_enable\n");

        return retval;
}

static int lcm_panel_vufs_ldo_disable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_vufs_ldo_regulator_init(dev);

	if (!IS_ERR_OR_NULL(vufs_ldo)) {
        	ret = regulator_disable(vufs_ldo);
        	if (ret < 0)
                	pr_err("disable regulator vufs_ldo fail, ret = %d\n", ret);
        	retval |= ret;
	}
	return ret;
}

static struct regulator *vmch_ldo;
static int regulator_vmch_inited;
static int lcm_panel_vmch_ldo_regulator_init(struct device *dev)
{
        int ret = 0;

        if (regulator_vmch_inited)
                return ret;
    pr_err("get lcm_panel_vmch_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        vmch_ldo = devm_regulator_get(dev, "vmchldo");
        if (IS_ERR(vmch_ldo)) { /* handle return value */
                ret = PTR_ERR(vmch_ldo);
                pr_err("get vmch_ldo fail, error: %d\n", ret);
                //return ret;
        }
        regulator_vmch_inited = 1;
        return ret; /* must be 0 */

}

static int lcm_panel_vmch_ldo_enable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_vmch_ldo_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vmch_ldo)) {
        	ret = regulator_set_voltage(vmch_ldo, 3000000, 3000000);
        	if (ret < 0)
                	pr_err("set voltage vmch_ldo fail, ret = %d\n", ret);
        	retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vmch_ldo)) {
        	ret = regulator_enable(vmch_ldo);
        	if (ret < 0)
                	pr_err("enable regulator vmch_ldo fail, ret = %d\n", ret);
        	retval |= ret;
	}
    	pr_err("get lcm_panel_vmch_ldo_enable\n");

        return retval;
}

static int lcm_panel_vmch_ldo_disable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_vmch_ldo_regulator_init(dev);

	if (!IS_ERR_OR_NULL(vmch_ldo)) {
        	ret = regulator_disable(vmch_ldo);
        	if (ret < 0)
                	pr_err("disable regulator vmch_ldo fail, ret = %d\n", ret);
        	retval |= ret;
	}
	return ret;
}
#endif
static void jdi_panel_init(struct jdi *ctx)
{
#if defined(CONFIG_PXLW_IRIS)
	iris_set_valid(2);
	iris_enable(NULL);
#endif
	jdi_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x07);
	jdi_dcs_write_seq_static(ctx, 0xC0,0x00);
	jdi_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x04);
	jdi_dcs_write_seq_static(ctx, 0xFF,0xAA,0x55,0xA5,0x80);
	jdi_dcs_write_seq_static(ctx, 0x6F,0x1D);
	jdi_dcs_write_seq_static(ctx, 0xF2,0x05);
    jdi_dcs_write_seq_static(ctx, 0x6F,0x20);
	jdi_dcs_write_seq_static(ctx, 0xF7,0x32);
	jdi_dcs_write_seq_static(ctx, 0xFF,0xAA,0x55,0xA5,0x81);
	/*VESA 10bit3x slice 12 for BOE*/
	jdi_dcs_write_seq_static(ctx, 0xFF,0xAA,0x55,0xA5,0x80);
	jdi_dcs_write_seq_static(ctx, 0x6F,0xA3);
	jdi_dcs_write_seq_static(ctx, 0xF6,0x00, 0x01, 0x04, 0x56, 0x77, 0x77, 0x77, 0x99, 0x9B, 0xF0, 0x00, 0x02, 0x78, 0x9A, 0xBB, 0xBC, 0xDD, 0xEE, 0xFF, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x91, 0xAB, 0x2A, 0x00, 0x0C, 0xC1, 0x9A, 0x01, 0xD8, 0x01, 0x03, 0x00, 0x0A, 0x08, 0xBB, 0x0A, 0x5F, 0x10, 0xEC);
	jdi_dcs_write_seq_static(ctx,0x6F,0x0F);    /* OSC=101.6Mhz */
	jdi_dcs_write_seq_static(ctx,0xFD,0x01);
	jdi_dcs_write_seq_static(ctx,0x6F,0x10);
	jdi_dcs_write_seq_static(ctx,0xFD,0x80);
	jdi_dcs_write_seq_static(ctx,0xFF,0xAA,0x55,0xA5,0x00); 
    jdi_dcs_write_seq_static(ctx,0x35);
    jdi_dcs_write_seq_static(ctx,0x53,0x20);
    jdi_dcs_write_seq_static(ctx,0x51,0x0D,0xBB,0x0F,0xFE);
    jdi_dcs_write_seq_static(ctx,0x2A,0x00,0x00,0x04,0x37);
    jdi_dcs_write_seq_static(ctx,0x2B,0x00,0x00,0x09,0x6B);
    /*VESA 8bit3x slice 20 for BOE*/
//    jdi_dcs_write_seq_static(ctx,0x91,0x89,0x28,0x00,0x0C,0xC2,0x00,0x03,0x1C,0x02,0x8C,0x00,0x0F,0x05,0x0E,0x02,0x8B,0x10,0xF0);
	/*PPS Table1*/
    jdi_dcs_write_seq_static(ctx,0x03,0x01);
    jdi_dcs_write_seq_static(ctx,0x90,0x11);
    jdi_dcs_write_seq_static(ctx,0x2C);
    /* ELVSS swire offset */
    jdi_dcs_write_seq_static(ctx,0x82,0xAE);
    jdi_dcs_write_seq_static(ctx,0x2F,0x01);//01 60 02 90
    jdi_dcs_write_seq_static(ctx,0x35);
	jdi_dcs_write_seq_static(ctx,0x44, 0x09, 0x20);
	jdi_dcs_write_seq_static(ctx,0x51,0x00,0x00);
	jdi_dcs_write_seq_static(ctx,0xF0,0x55,0xAA,0x52,0x08,0x01);
	jdi_dcs_write_seq_static(ctx,0xC3,0x99,0x01,0x98,0xD0,0x22,0x02,0x00);
	jdi_dcs_write_seq_static(ctx,0x11);
    msleep(64);
    jdi_dcs_write_seq_static(ctx,0x29);
	//msleep(20);
#if defined(CONFIG_PXLW_IRIS)
	iris_set_valid(4);
#endif
	pr_info("%s-\n", __func__);
}

static int jdi_disable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}
#if defined(CONFIG_PXLW_IRIS)
	iris_disable(NULL);
	iris_set_valid(2);
#endif

	ctx->enabled = false;

	return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{

	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->prepared)
		return 0;

	pr_info("[lh]%s+\n", __func__);
	jdi_dcs_write_seq_static(ctx, 0xF0,0x55,0xAA,0x52,0x08,0x01);
	jdi_dcs_write_seq_static(ctx,0xCE,0x0C);
	msleep(5);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(25);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(125);
       /* keep vcore off */
	lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
	pr_info("%s, call lpm_smc_spm_dbg keep vcore off for display off!\n", __func__);
/*	 ctx->pw_1p8_gpio = devm_gpiod_get(ctx->dev, "pw-gpio", GPIOD_OUT_HIGH);
          gpiod_set_value(ctx->pw_1p8_gpio, 0);
          devm_gpiod_put(ctx->dev, ctx->pw_1p8_gpio);

	 ctx->pw_reset_gpio = devm_gpiod_get(ctx->dev, "pw-reset", GPIOD_OUT_HIGH);
          gpiod_set_value(ctx->pw_reset_gpio, 0);
          devm_gpiod_put(ctx->dev, ctx->pw_reset_gpio);
*/
/*	 ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	  gpiod_set_value(ctx->reset_gpio, 0);
	  devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	lcm_panel_vufs_ldo_disable(ctx->dev);
	msleep(10);
	 ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);
	msleep(10);
	lcm_panel_vmc_ldo_disable(ctx->dev);
*/
	ctx->error = 0;
	ctx->prepared = false;
	pr_err("[lh]%s-\n", __func__);

	return 0;
}

static int jdi_prepare(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("[lh]%s +\n", __func__);
/*	lcm_panel_vufs_ldo_enable(ctx->dev);
	msleep(10);
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);

	msleep(10);

	lcm_panel_vmc_ldo_enable(ctx->dev);
*/
	pr_err("%s iris_reset begin\n", __func__);
	usleep_range(5000, 5001);
	ctx->pw_reset_gpio = devm_gpiod_get(ctx->dev, "pw-reset", GPIOD_OUT_LOW);
	gpiod_set_value(ctx->pw_reset_gpio, 0);
	usleep_range(1000, 1001);
	gpiod_set_value(ctx->pw_reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->pw_reset_gpio);
	pr_err("%s iris_reset end\n", __func__);

        usleep_range(1000, 1001);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(1000, 1001);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(1000, 1001);
	gpiod_set_value(ctx->reset_gpio, 1);
	//msleep(10);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(10);
	jdi_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	jdi_panel_get_data(ctx);
#endif

	pr_info("%s-\n", __func__);
	return ret;
}

static int jdi_enable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

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
	.clock = 166800,
	.hdisplay = 1080,
	.hsync_start = 1080 + 9,//HFP
	.hsync_end = 1080 + 9 + 2,//HSA
	.htotal = 1080 + 9 + 2 + 21,//HBP
	.vdisplay = 2412,
	.vsync_start = 2412 + 52,//VFP
	.vsync_end = 2412 + 52 + 14,//VSA
	.vtotal = 2412 + 52+ 14 + 22,//VBP
};

static const struct drm_display_mode performance_mode_90hz = {
	.clock = 250200,
	.hdisplay = 1080,
	.hsync_start = 1080 + 9,//HFP
	.hsync_end = 1080 + 9 + 2,//HSA
	.htotal = 1080 + 9 + 2 + 21,//HBP
	.vdisplay = 2412,
	.vsync_start = 2412 + 52,//VFP
	.vsync_end = 2412 + 52 + 14,//VSA
	.vtotal = 2412 + 52+ 14 + 22,//VBP
};

static const struct drm_display_mode performance_mode_120hz = {
	.clock = 333600,
	.hdisplay = 1080,
	.hsync_start = 1080 + 9,//HFP
	.hsync_end = 1080 + 9 + 2,//HSA
	.htotal = 1080 + 9 + 2 + 21,//HBP
	.vdisplay = 2412,
	.vsync_start = 2412 + 52,//VFP
	.vsync_end = 2412 + 52 + 14,//VSA
	.vtotal = 2412 + 52+ 14 + 22,//VBP
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 480,
        .phy_timcon = {
            .hs_trail = 12,
            .clk_trail = 16,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
                .cmd = 0xFA, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x00,
        },
	.lcm_esd_check_table[1] = {
                .cmd = 0x03, .count = 1, .para_list[0] = 0x01, .mask_list[0] = 0x01,
        },
        .lcm_esd_check_table[2] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
        },
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
        .corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
        .corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
        .corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.vendor = "boe_lisa",
	.manufacture = "boe",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.dsc_params = {
		.enable = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 40,
        .rct_on = 1,
/*        .bit_per_channel = 8,
        .dsc_line_buf_depth = 9,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
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
*/
        .bit_per_channel = 10,
        .dsc_line_buf_depth = 11,
        .bp_enable = 1,
        .bit_per_pixel = 160,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 675,
        .xmit_delay = 512,
        .dec_delay = 472,
        .scale_value = 25,
        .increment_interval = 346,
        .decrement_interval = 10,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2655,
        .initial_offset = 5632,
        .final_offset = 3312,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
		},
	.data_rate = 960,
        .color_vivid_status = true,
        .color_srgb_status = true,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
        .color_softiris_status = true,
        .color_dual_panel_status = false,
        .color_dual_brightness_status = true,
	//.oplus_wait_te = 0,
	.oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 17,
        .oplus_ofp_hbm_off_delay = 0,
        .oplus_need_wait_ms_time = true,
	//.oplus_uiready_before_time = 17,
        .dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 60,
        },
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 105,
	.oplus_display_global_dre = 1,
	.oplus_cmdq_pkt_set_event = 1,
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 480,
        .phy_timcon = {
            .hs_trail = 12,
            .clk_trail = 16,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
                .cmd = 0xFA, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x00,
        },
	.lcm_esd_check_table[1] = {
                .cmd = 0x03, .count = 1, .para_list[0] = 0x01, .mask_list[0] = 0x01,
        },
        .lcm_esd_check_table[2] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
        },
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
        .corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
        .corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
        .corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
        .vendor = "boe_lisa",
        .manufacture = "boe",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.dsc_params = {
		.enable = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 40,
        .rct_on = 1,
/*        .bit_per_channel = 8,
        .dsc_line_buf_depth = 9,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
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
*/
        .bit_per_channel = 10,
        .dsc_line_buf_depth = 11,
        .bp_enable = 1,
        .bit_per_pixel = 160,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 675,
        .xmit_delay = 512,
        .dec_delay = 472,
        .scale_value = 25,
        .increment_interval = 346,
        .decrement_interval = 10,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2655,
        .initial_offset = 5632,
        .final_offset = 3312,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
		},
	.data_rate = 960,
        .color_vivid_status = true,
        .color_srgb_status = true,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
        .color_softiris_status = true,
        .color_dual_panel_status = false,
        .color_dual_brightness_status = true,
//	.oplus_wait_te = 0,
	.oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 6,
        .oplus_ofp_hbm_off_delay = 0,
        .oplus_need_wait_ms_time = true,
//	.oplus_uiready_before_time = 12,
        .dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 90,
        },
        .cmd_null_pkt_en = 1,
        .cmd_null_pkt_len = 105,
        .oplus_display_global_dre = 1,
        .oplus_cmdq_pkt_set_event = 1,
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 480,
        .phy_timcon = {
            .hs_trail = 12,
            .clk_trail = 16,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
                .cmd = 0xFA, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x00,
        },
	.lcm_esd_check_table[1] = {
                .cmd = 0x03, .count = 1, .para_list[0] = 0x01, .mask_list[0] = 0x01,
        },
        .lcm_esd_check_table[2] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
        },
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
        .corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
        .corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
        .corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
        .vendor = "boe_lisa",
        .manufacture = "boe",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.dsc_params = {
		.enable = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 40,
        .rct_on = 1,
/*        .bit_per_channel = 8,
        .dsc_line_buf_depth = 9,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 287,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
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
*/
        .bit_per_channel = 10,
        .dsc_line_buf_depth = 11,
        .bp_enable = 1,
        .bit_per_pixel = 160,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 675,
        .xmit_delay = 512,
        .dec_delay = 472,
        .scale_value = 25,
        .increment_interval = 346,
        .decrement_interval = 10,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2655,
        .initial_offset = 5632,
        .final_offset = 3312,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
		},
	.data_rate = 960,
        .color_vivid_status = true,
        .color_srgb_status = true,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
        .color_softiris_status = true,
        .color_dual_panel_status = false,
        .color_dual_brightness_status = true,
//	.oplus_wait_te = 1,
	.oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 2,
        .oplus_ofp_hbm_off_delay = 0,
        .oplus_need_wait_ms_time = true,
//	.oplus_uiready_before_time = 9,
	.dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 120,
        },
        .cmd_null_pkt_en = 1,
        .cmd_null_pkt_len = 105,
        .oplus_display_global_dre = 1,
        .oplus_cmdq_pkt_set_event = 1,
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

#if defined(CONFIG_PXLW_IRIS)
static void iris_panel_commandq(void *dsi, dcs_write_gce cb,
		void *handle, char *src, int s_len)
{
	char dest[512] = {0,};
	int len = 0;
	len = iris_conver_one_panel_cmd(dest, src, s_len);
	if (len > 0)
		cb(dsi, handle, dest, len);
}
#endif

static int oplus_lcm_dc_backlight(void *dsi, dcs_write_gce cb,void *handle,unsigned int level,int hbm_en)
{
	int i;
	//int seed_alpha = oplus_seed_bright_to_alpha(level);
	int seed_alpha = level;

	if (pq_trigger == 0)
		return level;
	if (!oplus_dc_enable_real || hbm_en || level < 4 || level > DC_BAKLIGHT_THERSHOLD) {
		goto dc_disable;
	}

	if (!oplus_dc_alpha) {
#if defined(CONFIG_PXLW_IRIS)
		if (iris_is_chip_supported()) {
			for (i = 0; i < sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]); i++) {
				iris_panel_commandq(dsi, cb, handle, lcm_seed_setting[i].para_list,lcm_seed_setting[i].count);
			}
		} else {
#endif /*CONFIG_PXLW_IRIS*/
		for (i = 0; i < sizeof(lcm_seed_setting)/sizeof(lcm_seed_setting[0]); i++){
			cb(dsi, handle, lcm_seed_setting[i].para_list, lcm_seed_setting[i].count);
		}
#if defined(CONFIG_PXLW_IRIS)
		}
#endif
	}

	if (!oplus_dc_alpha)
		pr_err("Enter DC");

	oplus_dc_alpha = seed_alpha;
	return seed_alpha;

dc_disable:
	if (oplus_dc_alpha) {
		if (level != 0) {
#if defined(CONFIG_PXLW_IRIS)
			if (iris_is_chip_supported()) {
				for (i = 0; i < sizeof(lcm_seed_exit)/sizeof(lcm_seed_exit[0]); i++) {
					iris_panel_commandq(dsi, cb, handle, lcm_seed_exit[i].para_list, lcm_seed_exit[i].count);
				}
			} else {
#endif /*CONFIG_PXLW_IRIS*/
			for (i = 0; i < sizeof(lcm_seed_exit)/sizeof(lcm_seed_exit[0]); i++){
				cb(dsi, handle, lcm_seed_exit[i].para_list, lcm_seed_exit[i].count);
			}
#if defined(CONFIG_PXLW_IRIS)
			}
#endif
			pr_err("exit DC");
		}
	}

	oplus_dc_alpha = 0;
	return level;
}
static int jdi_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xFF};
//	char bl_tb1[] = {0xB2, 0x01};
//	char bl_tb2[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0)
		level = 3515;
	level = oplus_lcm_dc_backlight(dsi,cb,handle, level, 0);
	if (level > 1 && level < 4)
		level = 4;
	if ((level == 0) && (hbm_flag == true)) {
		pr_err("hbm mode, ignore setbacklight");
		return 0;
	}
	pr_info("%s backlight = -%d\n", __func__, level);
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;

	if (!cb)
		return -1;

	if (level ==1) {
		pr_err("enter aod!!!\n");
		return 0;
	}

//	if (level >2047)
//		bl_tb1[1] = 0x01;
//	else
//		bl_tb1[1] = 0x11;
	esd_brightness = level;
#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported())
		iris_panel_commandq(dsi, cb, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	else
#endif
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

//	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
//	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	oplus_display_brightness = level;

	return 0;
}

static void lcm_setbrightness(void *dsi,
			      dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	int i = 0;

	printk("[lh] %s level is %d\n", __func__, level);
	        if (level > 1 && level <4)
                	level = 4;
		BL_LSB = level >> 8;
		BL_MSB = level & 0xFF;

		lcm_setbrightness_normal[0].para_list[1] = BL_LSB;
		lcm_setbrightness_normal[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_normal)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_normal[i].para_list, lcm_setbrightness_normal[i].count);
		}
}
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;
	int level = 0;

	if (!cb)
		return -1;

	pr_err("[lh]oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
		lcm_setbrightness(dsi, cb, handle, level);  //level
		printk("[lh] %s : %d ! backlight %d !\n",__func__, hbm_mode, oplus_display_brightness);
	}

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	//char hbm_tb[] = {0x53, 0xe0};
	int i = 0;
	int level = 0;
	if (!cb)
		return -1;

	pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
		lcm_setbrightness(dsi, cb, handle,level);
		printk("[soso] %s : %d ! backlight %d !\n",__func__, en, oplus_display_brightness);
		if (level <= BRIGHTNESS_HALF)
			flag_hbm = 0;
		else
			flag_hbm = 1;
	}
	flag_hbm = en;
	//lcdinfo_notify(1, &en);
	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
	void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};

	//pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
 	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

 	return 1;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct jdi *ctx = panel_to_jdi(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}
#if 0
static unsigned long panel_doze_get_mode_flags(struct drm_panel *panel, int doze_en)
{
	unsigned long mode_flags;

	//DDPINFO("%s doze_en:%d\n", __func__, doze_en);
	if (doze_en) {
		mode_flags = MIPI_DSI_MODE_LPM
		       | MIPI_DSI_MODE_EOT_PACKET
		       | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	} else {
		mode_flags = MIPI_DSI_MODE_VIDEO
		       | MIPI_DSI_MODE_VIDEO_BURST
		       | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
		       | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	}

	pr_err("debug for %s, mode flags =%d, doze_en = %d\n", __func__,mode_flags,doze_en);
	return mode_flags;
}
#endif

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;
	pr_err("debug for lcm %s", __func__);

	/*if (oplus_fp_notify_down_delay)
		aod_finger_unlock_flag = 1;*/

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
				cb(dsi, handle, lcm_aod_to_normal[i].para_list, lcm_aod_to_normal[i].count);
		}
	}

	/*if (aod_finger_unlock_flag == 1) {
		struct lcm *ctx = panel_to_lcm(panel);
		DDPINFO("finger unlock in aod\n");
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
		ctx->hbm_en = true;
		ctx->hbm_wait = false;
		aod_finger_unlock_flag = 0;
	}*/

       /* keep vcore on */
       lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
       pr_info("%s, call lpm_smc_spm_dbg keep vcore off for exit AOD!\n", __func__);

	aod_state = false;
	return 0;
}

static struct LCM_setting_table lcm_normal_to_aod_sam[] = {
/*   {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x04}},
   {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
   {REGFLAG_CMD, 2, {0x6F,0x1D}},
   {REGFLAG_CMD, 2, {0xF2,0x05}},
   {REGFLAG_CMD, 2, {0x6F,0x20}},
   {REGFLAG_CMD, 2, {0xF7,0x32}},
   {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
   {REGFLAG_CMD, 2, {0x6F,0x0F}},
   {REGFLAG_CMD, 2, {0xFD,0x01}},
   {REGFLAG_CMD, 2, {0x6F,0x10}},
   {REGFLAG_CMD, 2, {0xFD,0x80}},

    {REGFLAG_CMD, 1, {0x35}},
    {REGFLAG_CMD, 2, {0x53,0x20}},
    {REGFLAG_CMD, 5, {0x51,0x0D,0xBB,0x0F,0xFE}},
    {REGFLAG_CMD, 5, {0x2A,0x00,0x00,0x04,0x37}},
    {REGFLAG_CMD, 5, {0x2B,0x00,0x00,0x09,0x6B}},

    {REGFLAG_CMD, 19, {0x91,0xAB,0x28,0x00,0x0C,0xC2,0x00,0x02,0x0E,0x01,0x1F,0x00,0x07,0x08,0xBB,0x08,0x7A,0x10,0xF0}},
    {REGFLAG_CMD, 2, {0x03,0x01}},
    {REGFLAG_CMD, 2, {0x90,0x11}},
    {REGFLAG_CMD, 1, {0x2C}},
    {REGFLAG_CMD, 2, {0x82,0xAE}},
    {REGFLAG_CMD, 2, {0x2F,0x01}},
    {REGFLAG_CMD, 3, {0x51,0x00,0x00}},
    {REGFLAG_CMD, 1, {0x11}},
    {REGFLAG_DELAY,120,{}},
    {REGFLAG_CMD, 1, {0x29}},*/
    //{REGFLAG_DELAY,20,{}},

    //{REGFLAG_CMD, 1, {0x28}},
    {REGFLAG_CMD, 5, {0x51,0x00,0x00,0x0F,0xFE}},
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
    {REGFLAG_CMD, 2, {0x6F,0x2E}},
    {REGFLAG_CMD, 3, {0xB4,0x04,0xC8}},
    {REGFLAG_DELAY,15,{}},
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
    {REGFLAG_CMD, 2, {0x6F,0x0D}},
    {REGFLAG_CMD, 2, {0xB5,0x50}},
    {REGFLAG_CMD, 2, {0x2F,0x01}},
    {REGFLAG_CMD, 1, {0x39}},
    {REGFLAG_DELAY,30,{}},
    {REGFLAG_CMD, 2, {0x65,0x01}},
    {REGFLAG_CMD, 1, {0x2C}},
    //{REGFLAG_DELAY,20,{}},

    /* Display on */
    //{REGFLAG_CMD, 1, {0x29}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

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
       /* keep vcore off */
       lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x1);
       pr_info("%s, call lpm_smc_spm_dbg keep vcore on for enter AOD!\n", __func__);


	return 0;
}

#if 0
static int panel_doze_enable_start(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	cmd = 0x10;
	cb(dsi, handle, &cmd, 1);

	return 0;
}

static int panel_doze_enable_end(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	int send_buf[3];
	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x29;
	cb(dsi, handle, &cmd, 1);
	send_buf[0] = 0xF0;
	send_buf[1] = 0x5A;
	send_buf[2] = 0x5A;
	cb(dsi, handle, send_buf, 3);
	send_buf[0] = 0xF2;
	send_buf[1] = 0x0F;
	cb(dsi, handle, send_buf, 2);
	send_buf[0] = 0xF0;
	send_buf[1] = 0xA5;
	send_buf[2] = 0xA5;
	cb(dsi, handle, send_buf, 3);

	return 0;
}

static int panel_doze_post_disp_on(void *dsi, dcs_write_gce cb, void *handle)
{

	int cmd = 0;

	pr_err("debug for boe lcm %s\n", __func__);

	cmd = 0x29;
	cb(dsi, handle, &cmd, 1);
	//msleep(2);

	return 0;
}


static int panel_doze_post_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{

	int cmd = 0;

	pr_err("debug for boe lcm %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);

	return 0;
}

static int lcm_panel_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

	pr_err("boe lcm: %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	msleep(10);

	cmd = 0x10;
	cb(dsi, handle, &cmd, 1);
	msleep(120);

	return 0;
}
#endif

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	pr_err("debug for lcm %s\n", __func__);
	if (level == 0) {
		for (i = 0; i < sizeof(lcm_aod_high_mode)/sizeof(struct LCM_setting_table); i++){
#if defined(CONFIG_PXLW_IRIS)
                        iris_panel_commandq(dsi, cb, handle, lcm_aod_high_mode[i].para_list,lcm_aod_high_mode[i].count);
#else
			cb(dsi, handle, lcm_aod_high_mode[i].para_list, lcm_aod_high_mode[i].count);
#endif
		}
	} else {
		for (i = 0; i < sizeof(lcm_aod_low_mode)/sizeof(struct LCM_setting_table); i++){
#if defined(CONFIG_PXLW_IRIS)
                        iris_panel_commandq(dsi, cb, handle, lcm_aod_low_mode[i].para_list, lcm_aod_low_mode[i].count);
#else
			cb(dsi, handle, lcm_aod_low_mode[i].para_list, lcm_aod_low_mode[i].count);
#endif
		}
	}
	printk("[soso] %s : %d !\n",__func__, level);

	//memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;


	if (ctx->prepared)
		return 0;

	pr_info("[lh]debug for samsung_amb670 lcm %s\n", __func__);
	/*
	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	msleep(5);
	fan53870_ldo1_set_voltage(1120000);
	msleep(10);
	lcm_panel_ldo3_enable(ctx->dev);
	*/
	//enable te
/*	ctx->te_switch_gpio = devm_gpiod_get(ctx->dev, "te_switch", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->te_switch_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->te_switch_gpio);
	usleep_range(5000, 5100);*/
	//enable 1.5V
	ctx->pw_1p8_gpio = devm_gpiod_get(ctx->dev, "pw-1p8", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->pw_1p8_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->pw_1p8_gpio);
	usleep_range(5000, 5001);
/*
        ctx->pw_reset_gpio = devm_gpiod_get(ctx->dev, "pw-reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->pw_reset_gpio, 1);
        msleep(5);
        gpiod_set_value(ctx->pw_reset_gpio, 0);
        msleep(5);
        gpiod_set_value(ctx->pw_reset_gpio, 1);
        devm_gpiod_put(ctx->dev, ctx->pw_reset_gpio);*/
	lcm_panel_vufs_ldo_enable(ctx->dev);
	usleep_range(1000, 1100);
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);
	usleep_range(1000, 1100);
	//set vddi 3.0v
	lcm_panel_vmch_ldo_enable(ctx->dev);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	usleep_range(1000, 1100);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;


	if (ctx->prepared)
		return 0;

	pr_info("[lh]debug for amb670yf01 lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(5);
	//lcm_panel_ldo3_disable(ctx->dev);
	//set vddi 3.0v
        lcm_panel_vmch_ldo_disable(ctx->dev);
        usleep_range(5000, 5100);
	//enable 1.5V
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);
	msleep(5);
	//set vddi 3.0v
	lcm_panel_vufs_ldo_disable(ctx->dev);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);
	
	ctx->pw_reset_gpio = devm_gpiod_get(ctx->dev, "pw-reset", GPIOD_OUT_HIGH);
    gpiod_set_value(ctx->pw_reset_gpio, 0);
    devm_gpiod_put(ctx->dev, ctx->pw_reset_gpio);
	ctx->pw_1p8_gpio = devm_gpiod_get(ctx->dev, "pw-1p8", GPIOD_OUT_HIGH);
    gpiod_set_value(ctx->pw_1p8_gpio, 0);
    devm_gpiod_put(ctx->dev, ctx->pw_1p8_gpio);
	//msleep(110);
	return 0;
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

	if (!m) {
		pr_err("%s:%d invalid display_mode\n", __func__, __LINE__);
		return ret;
	}

	if (drm_mode_vrefresh(m) == 60)
		ext->params = &ext_params;
	else if (drm_mode_vrefresh(m) == 90)
		ext->params = &ext_params_90hz;
	else if (drm_mode_vrefresh(m) == 120)
		ext->params = &ext_params_120hz;
	else
		ret = 1;

	return ret;
}

static void mode_switch_to_120(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	pr_info("%s\n", __func__);

        jdi_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x09);
        jdi_dcs_write_seq_static(ctx, 0x6F, 0x6B);
        jdi_dcs_write_seq_static(ctx, 0xE0, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x2F, 0x03);
}

static void mode_switch_to_90(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	pr_info("%s\n", __func__);

        jdi_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x09);
        jdi_dcs_write_seq_static(ctx, 0x6F, 0x6B);
        jdi_dcs_write_seq_static(ctx, 0xE0, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x2F, 0x02);
}

static void mode_switch_to_60(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	pr_info("%s\n", __func__);

        jdi_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x09);
        jdi_dcs_write_seq_static(ctx, 0x6F, 0x6B);
        jdi_dcs_write_seq_static(ctx, 0xE0, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x2F, 0x01);
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, dst_mode);
	if (!m) {
		pr_err("%s invalid drm_display_mode\n", __func__);
		return 1;
	}

	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);

	if (drm_mode_vrefresh(m) == 60) { /* 60 switch to 120 */
		mode_switch_to_60(panel);
	} else if (drm_mode_vrefresh(m) == 90) { /* 1200 switch to 60 */
		mode_switch_to_90(panel);
	} else if (drm_mode_vrefresh(m) == 120) { /* 1200 switch to 60 */
		mode_switch_to_120(panel);
	} else
		ret = 1;

	return ret;
}

static char esd_check_pre_cmd0[] = {0xFF, 0xAA, 0x55, 0xA5, 0x81};
static char esd_check_pre_cmd1[] = {0x6F, 0x0F};

static int lcm_esd_check_precondition(void *dsi, dcs_write_gce cb, void *handle)
{
#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		iris_panel_commandq(dsi, cb, handle, esd_check_pre_cmd0, ARRAY_SIZE(esd_check_pre_cmd0));
		iris_panel_commandq(dsi, cb, handle, esd_check_pre_cmd1, ARRAY_SIZE(esd_check_pre_cmd1));
	} else {
#endif /* CONFIG_PXLW_IRIS */
	cb(dsi, handle, esd_check_pre_cmd0, ARRAY_SIZE(esd_check_pre_cmd0));
	cb(dsi, handle, esd_check_pre_cmd1, ARRAY_SIZE(esd_check_pre_cmd1));
#if defined(CONFIG_PXLW_IRIS)
	}
#endif

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = jdi_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.set_hbm = lcm_set_hbm,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.esd_check_precondition = lcm_esd_check_precondition,
};
#endif

static int jdi_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 default_mode.hdisplay, default_mode.vdisplay,
			 drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);


	mode2 = drm_mode_duplicate(connector->dev, &performance_mode_90hz);
	if (!mode2) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_90hz.hdisplay, performance_mode_90hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_90hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);

	mode3 = drm_mode_duplicate(connector->dev, &performance_mode_120hz);
	if (!mode3) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_120hz.hdisplay, performance_mode_120hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_120hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode3);
	
	connector->display_info.width_mm = 70;
	connector->display_info.height_mm = 155;

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported())
		iris_init_panel_timing(connector);
#endif

	return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
	.disable = jdi_disable,
	.unprepare = jdi_unprepare,
	.prepare = jdi_prepare,
	.enable = jdi_enable,
	.get_modes = jdi_get_modes,
};

static bool is_probe_finish = false;
static int jdi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct jdi *ctx;
	struct device_node *backlight;
	int ret;
	pr_info("[boe] nt37701a %s+\n", __func__);

	if (is_probe_finish == true)
		return 0;
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

	ctx = devm_kzalloc(dev, sizeof(struct jdi), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
/*
	g_pmic = kzalloc(sizeof(struct lcm_pmic_info), GFP_KERNEL);
	if (!g_pmic) {
		pr_err("[lh]fail to alloc lcm_pmic_info (ENOMEM)\n");
		return -ENOMEM;
	}*/
	mipi_dsi_set_drvdata(dsi, ctx);
	pr_err("[lh]%s %d\n",__func__,__LINE__);
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

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
	devm_gpiod_put(dev, ctx->reset_gpio);
/*
	ctx->te_switch_gpio = devm_gpiod_get(dev, "te_switch", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->te_switch_gpio)) {
		dev_err(dev, "%s: cannot get te_switch_gpio %ld\n",
			__func__, PTR_ERR(ctx->te_switch_gpio));
		return PTR_ERR(ctx->te_switch_gpio);
	}
	gpiod_set_value(ctx->te_switch_gpio, 1);
	devm_gpiod_put(dev, ctx->te_switch_gpio);


	ctx->te_out_gpio = devm_gpiod_get(dev, "te_out", GPIOD_IN);
	if (IS_ERR(ctx->te_out_gpio)) {
		dev_err(dev, "%s: cannot get te_out_gpio %ld\n",
			__func__, PTR_ERR(ctx->te_out_gpio));
		return PTR_ERR(ctx->te_out_gpio);
	}

	devm_gpiod_put(dev, ctx->te_out_gpio);
*/
	lcm_panel_vufs_ldo_enable(ctx->dev);
	usleep_range(5000, 5100);
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p5_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddr1p5_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p5_enable_gpio));
		return PTR_ERR(ctx->vddr1p5_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vddr1p5_enable_gpio);

	usleep_range(5000, 5100);

        lcm_panel_vmch_ldo_enable(ctx->dev);
/*	ctx->pw_1p8_gpio = devm_gpiod_get(dev, "pw-1p8", GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->pw_1p8_gpio)) {
                dev_info(dev, "cannot get pw-1p8-gpios %ld\n",
                         PTR_ERR(ctx->pw_1p8_gpio));
                return PTR_ERR(ctx->pw_1p8_gpio);
        }
	gpiod_set_value(ctx->pw_1p8_gpio, 1);
        devm_gpiod_put(dev, ctx->pw_1p8_gpio);

	ctx->pw_1p8_gpio = devm_gpiod_get(dev, "pw-reset", GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->pw_reset_gpio)) {
                dev_info(dev, "cannot get pw-reset-gpios %ld\n",
                         PTR_ERR(ctx->pw_reset_gpio));
                return PTR_ERR(ctx->pw_reset_gpio);
        }
        gpiod_set_value(ctx->pw_reset_gpio, 1);
	usleep(5000);
	gpiod_set_value(ctx->pw_reset_gpio, 0);
	usleep(5000);
	gpiod_set_value(ctx->pw_reset_gpio, 1);
	devm_gpiod_put(dev, ctx->pw_reset_gpio);	
*/
	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &jdi_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	is_probe_finish = true;
	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif
	register_device_proc("lcd", "boe_lisa", "boe");
	disp_aal_set_dre_en(1);
	oplus_enhance_mipi_strength = 1;
	flag_silky_panel = BL_SETTING_DELAY_60HZ;
	pr_info("%s- lcm,nt37701a,vdo,60hz\n", __func__);

	return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
{
	struct jdi *ctx = mipi_dsi_get_drvdata(dsi);
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

static const struct of_device_id jdi_of_match[] = {
	{
	    .compatible = "oplus21641,boe,nt37701a,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
		.name = "oplus21641_boe_nt37701a_fhd_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = jdi_of_match,
	},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("JDI NT36672E VDO 60HZ AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
