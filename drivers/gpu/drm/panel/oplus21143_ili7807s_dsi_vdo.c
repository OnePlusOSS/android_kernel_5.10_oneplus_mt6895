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

#include <soc/oplus/device_info.h>
#include <soc/oplus/system/boot_mode.h>

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

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   2047
#define LCM_BRIGHTNESS_TYPE 2
static unsigned int esd_brightness = 1023;
static u32 flag_hbm = 0;
extern unsigned int oplus_display_brightness;
static int aod_finger_unlock_flag = 0;
static bool aod_state = false;
static bool is_pd_with_guesture = false;
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
extern enum boot_mode_t get_boot_mode(void);
#endif
#else
extern int get_boot_mode(void);
#endif

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
#endif
struct LCM_setting_table {
  	unsigned int cmd;
 	unsigned char count;
 	unsigned char para_list[128];
};
static struct LCM_setting_table lcm_setbrightness_hbm[] = {
	{REGFLAG_CMD,3, {0x51, 0x00, 0x00}},
//	{REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
//	{REGFLAG_CMD,2, {0xB2, 0x01}},
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

static struct LCM_setting_table lcm_aod_to_normal[] = {
	{REGFLAG_CMD,6,{0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2,{0x6F,0x0D}},
	{REGFLAG_CMD,2,{0xB5,0x4F}},
	{REGFLAG_CMD,2,{0x65,0x00}},
	{REGFLAG_DELAY,30,{}},
	{REGFLAG_CMD,1,{0x38}},
	{REGFLAG_CMD,1,{0x2C}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_high_mode[] = {
	{REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2, {0x6F,0x26}},
	{REGFLAG_CMD,11,{0xB4,0x04,0xA2,0x04,0xA2,0x04,0xA2,0x04,0xA2,0x04,0xA2}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
	{REGFLAG_CMD,6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{REGFLAG_CMD,2, {0x6F,0x26}},
	{REGFLAG_CMD,11, {0xB4,0x08,0x8A,0x08,0x8A,0x08,0x8A,0x08,0x8A,0x08,0x8A}},
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

	bool hbm_en;
	bool hbm_wait;
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

static void jdi_dcs_write(struct jdi *ctx, const void *data, size_t len)
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
#if 1
#if 0
static struct regulator *vmc_ldo;
static int lcm_panel_vmc_ldo_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited){
		return ret;
	}
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
	if (ret < 0){
		pr_err("set voltage vmc_ldo fail, ret = %d\n", ret);
	}
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(vmc_ldo);
	if (ret < 0){
		pr_err("enable regulator vmc_ldo fail, ret = %d\n", ret);
	}
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
	if (ret < 0){
		pr_err("disable regulator vmc_ldo fail, ret = %d\n", ret);
	}
	retval |= ret;
    pr_err("disable regulator vmc_ldo\n");

	return retval;
}
#endif
static struct regulator *vufs_ldo;
static int lcm_panel_vufs_ldo_regulator_init(struct device *dev)
{
        static int regulator_inited;
        int ret = 0;

        if (regulator_inited){
                return ret;
		}
    pr_err("get lcm_panel_vufs_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        vufs_ldo = devm_regulator_get(dev, "vufsldo");
        if (IS_ERR(vufs_ldo)) { /* handle return value */
                ret = PTR_ERR(vufs_ldo);
                pr_err("get vufs_ldo fail, error: %d\n", ret);
                //return ret;
        }
        regulator_inited = 1;
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
		if (ret < 0) {
			pr_err("set voltage vufs_ldo fail, ret = %d\n", ret);
		}
		retval |= ret;
	}
		/* enable regulator */
	if (!IS_ERR_OR_NULL(vufs_ldo)) {
		ret = regulator_enable(vufs_ldo);
		if (ret < 0) {
			pr_err("enable regulator vufs_ldo fail, ret = %d\n", ret);
		}
		retval |= ret;
		pr_err("get lcm_panel_vufs_ldo_enable\n");
	}
	return retval;
}

static int lcm_panel_vufs_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_vufs_ldo_regulator_init(dev);
	if (!IS_ERR_OR_NULL(vufs_ldo)) {
		ret = regulator_disable(vufs_ldo);
		if (ret < 0) {
			pr_err("disable regulator vufs_ldo fail, ret = %d\n", ret);
		}
		retval |= ret;
	}
	return ret;
}

//static struct regulator *vmch_ldo;
/* static int lcm_panel_vmch_ldo_regulator_init(struct device *dev)
{
        static int regulator_vmch_inited;
        int ret = 0;

        if (regulator_vmch_inited)
                return ret;
    pr_err("get lcm_panel_vmch_ldo_regulator_init\n");

        // please only get regulator once in a driver
        vmch_ldo = devm_regulator_get(dev, "vmchldo");
        if (IS_ERR(vmch_ldo)) { // handle return value
                ret = PTR_ERR(vmch_ldo);
                pr_err("get vmch_ldo fail, error: %d\n", ret);
                //return ret;
        }
        regulator_vmch_inited = 1;
        return ret; // must be 0

} */

/* static int lcm_panel_vmch_ldo_enable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_vmch_ldo_regulator_init(dev);

        // set voltage with min & max
        ret = regulator_set_voltage(vmch_ldo, 3000000, 3000000);
        if (ret < 0)
                pr_err("set voltage vmch_ldo fail, ret = %d\n", ret);
        retval |= ret;

        // enable regulator
        ret = regulator_enable(vmch_ldo);
        if (ret < 0)
                pr_err("enable regulator vmch_ldo fail, ret = %d\n", ret);
        retval |= ret;
    pr_err("get lcm_panel_vmch_ldo_enable\n");

        return retval;
} */

/* static int lcm_panel_vmch_ldo_disable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_vmch_ldo_regulator_init(dev);

        ret = regulator_disable(vmch_ldo);
        if (ret < 0)
                pr_err("disable regulator vmch_ldo fail, ret = %d\n", ret);
        retval |= ret;
        return ret;
} */
#endif
static void jdi_panel_init(struct jdi *ctx)
{
    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x06);
    jdi_dcs_write_seq_static(ctx,0x08,0x20);
    jdi_dcs_write_seq_static(ctx,0x3E,0xE2);
    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x03);
    jdi_dcs_write_seq_static(ctx,0x83,0x20);
    jdi_dcs_write_seq_static(ctx,0x84,0x01);

    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x03);
    jdi_dcs_write_seq_static(ctx,0x86,0x47);
    jdi_dcs_write_seq_static(ctx,0x87,0x56);
    jdi_dcs_write_seq_static(ctx,0xAF,0x18);
    jdi_dcs_write_seq_static(ctx,0x8C,0xE2);
    jdi_dcs_write_seq_static(ctx,0x8D,0xE3);
    jdi_dcs_write_seq_static(ctx,0x8E,0xE5);
    jdi_dcs_write_seq_static(ctx,0x8F,0xE6);
    jdi_dcs_write_seq_static(ctx,0x90,0xE9);
    jdi_dcs_write_seq_static(ctx,0x91,0xEB);
    jdi_dcs_write_seq_static(ctx,0x92,0xEE);
    jdi_dcs_write_seq_static(ctx,0x93,0xF0);
    jdi_dcs_write_seq_static(ctx,0x94,0xF6);
    jdi_dcs_write_seq_static(ctx,0x95,0xFA);
    jdi_dcs_write_seq_static(ctx,0x96,0xBF);
    jdi_dcs_write_seq_static(ctx,0x97,0xC4);
    jdi_dcs_write_seq_static(ctx,0x98,0xC7);
    jdi_dcs_write_seq_static(ctx,0x99,0xC9);
    jdi_dcs_write_seq_static(ctx,0x9A,0xCF);
    jdi_dcs_write_seq_static(ctx,0x9B,0xD4);
    jdi_dcs_write_seq_static(ctx,0x9C,0xDA);
    jdi_dcs_write_seq_static(ctx,0x9D,0xE2);
    jdi_dcs_write_seq_static(ctx,0x9E,0xEB);
    jdi_dcs_write_seq_static(ctx,0x9F,0xFA);

    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x06);
    jdi_dcs_write_seq_static(ctx,0x48,0x0F);
    jdi_dcs_write_seq_static(ctx,0x4D,0x80);
    jdi_dcs_write_seq_static(ctx,0x4E,0x40);
    jdi_dcs_write_seq_static(ctx,0xC7,0x05);

    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x00);
    jdi_dcs_write_seq_static(ctx,0x53,0x24);
    jdi_dcs_write_seq_static(ctx,0x55,0x00);
    jdi_dcs_write_seq_static(ctx,0x35,0x00);
    jdi_dcs_write_seq_static(ctx,0x51,0x0F,0xFF);


    // jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x17);
    // jdi_dcs_write_seq_static(ctx,0x20,0x00,0x00,0x00,0x00,0x00,0x11,0x00,0x00,0x89,0x30,0x80,0x09,0x6C,0x04,0x38,0x00,0x0c,0x02,0x1c,0x02,0x1c,0x02,0x00,0x02,0x0e,0x00,0x20,0x01,0x1f,0x00,0x07,0x00,0x0c,0x08,0xbb,0x08,0x7a,0x18,0x00,0x10,0xf0,0x03,0x0c,0x20,0x00,0x06,0x0b,0x0b,0x33,0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7b,0x7d,0x7e,0x01,0x02,0x01,0x00,0x09,0x40,0x09,0xbe,0x19,0xfc,0x19,0xfa,0x19,0xf8,0x1a,0x38,0x1a,0x78,0x1a,0xb6,0x2a,0xF6,0x2B,0x34,0x2B,0x74,0x3b,0x74,0x6B,0xF4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0B);
    jdi_dcs_write_seq_static(ctx,0xE7,0x4B);
    jdi_dcs_write_seq_static(ctx,0xE8,0x03);

    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x00);
    jdi_dcs_write_seq_static(ctx,0x11,0x00);
    msleep(100);

    /* jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x11);
    jdi_dcs_write_seq_static(ctx,0x00,0x01);
    jdi_dcs_write_seq_static(ctx,0x03,0x07);
    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0E);
    jdi_dcs_write_seq_static(ctx,0x21,0x2E);
    jdi_dcs_write_seq_static(ctx,0x23,0x2E);
    jdi_dcs_write_seq_static(ctx,0x20,0x03);
    jdi_dcs_write_seq_static(ctx,0x25,0x11);
    jdi_dcs_write_seq_static(ctx,0x26,0x24);
    jdi_dcs_write_seq_static(ctx,0x27,0x20);
    jdi_dcs_write_seq_static(ctx,0x29,0x58);
    jdi_dcs_write_seq_static(ctx,0x2D,0x5B);
    jdi_dcs_write_seq_static(ctx,0x41,0x17);
    jdi_dcs_write_seq_static(ctx,0x43,0x17);
    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x1E);
    jdi_dcs_write_seq_static(ctx,0x00,0x2E);
    jdi_dcs_write_seq_static(ctx,0x08,0x2E);
    jdi_dcs_write_seq_static(ctx,0x09,0x2E);
    jdi_dcs_write_seq_static(ctx,0xA4,0x1D);
    jdi_dcs_write_seq_static(ctx,0xA5,0x7F);
    jdi_dcs_write_seq_static(ctx,0xA6,0x7F);
    jdi_dcs_write_seq_static(ctx,0xA7,0x53); */

    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x02);
    jdi_dcs_write_seq_static(ctx,0x1B,0x00);
    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x04);
    jdi_dcs_write_seq_static(ctx,0xBD,0x01);
    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x07);
    jdi_dcs_write_seq_static(ctx,0x29,0xCF);
    jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x00);
    jdi_dcs_write_seq_static(ctx,0x29,0x00);
    msleep(21);
    pr_info("SYQ %s-\n", __func__);

}

static int jdi_disable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->enabled){
		return 0;
	}

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{

	struct jdi *ctx = panel_to_jdi(panel);
	pr_err("[lh]%s+\n", __func__);

	if (!ctx->prepared){
		return 0;
	}

	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(100);

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
	int mode;
	int blank;
	pr_err("[lh]%s +\n", __func__);
	if (ctx->prepared){
		return 0;
	}
/*
	ctx->pw_1p8_gpio = devm_gpiod_get(ctx->dev, "pw-1p8", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->pw_1p8_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->pw_1p8_gpio);
	msleep(5);
	ctx->pw_reset_gpio = devm_gpiod_get(ctx->dev, "pw-reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->pw_reset_gpio, 1);
	msleep(5);
	gpiod_set_value(ctx->pw_reset_gpio, 0);
        msleep(5);
        gpiod_set_value(ctx->pw_reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->pw_reset_gpio);
*/
/*	lcm_panel_vufs_ldo_enable(ctx->dev);
	msleep(10);
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);

	msleep(10);

	lcm_panel_vmc_ldo_enable(ctx->dev);

	// lcd reset H -> L -> H
*/
	msleep(10);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 1);
	//msleep(10);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(15);

	mode = get_boot_mode();
	pr_info("[TP] in dis_panel_power_on,mode = d%\n", mode);
	if ((mode != MSM_BOOT_MODE__FACTORY) &&(mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)){
		#define LCD_CTL_TP_LOAD_FW 0x10
		#define LCD_CTL_CS_ON  0x19
		blank = LCD_CTL_CS_ON;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		pr_err("[TP]TP CS will chang to spi mode and high\n");
		usleep_range(5000, 5100);
		blank = LCD_CTL_TP_LOAD_FW;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		pr_info("[TP] start to load fw!\n");

	}

	jdi_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0){
		jdi_unprepare(panel);
	}

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
    .clock = 374695,
    .hdisplay = 1080,
    .hsync_start = 1080 + 86,//HFP
    .hsync_end = 1080 + 86 + 4,//HSA
    .htotal = 1080 +86 + 4 + 84,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 2530,//VFP
    .vsync_end = 2412 + 2530 + 4,//VSA
    .vtotal = 2412 + 2530+ 4 + 34,//VBP
};

static const struct drm_display_mode performance_mode_90hz = {
    .clock = 374695,
    .hdisplay = 1080,
    .hsync_start = 1080 + 86,//HFP
    .hsync_end = 1080 + 86 + 4,//HSA
    .htotal = 1080 + 86 + 4 + 84,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 870,//VFP
    .vsync_end = 2412 + 870 + 4,//VSA
    .vtotal = 2412 + 870+ 4 + 34,//VBP
};

static const struct drm_display_mode performance_mode_120hz = {
    .clock = 374695,
    .hdisplay = 1080,
    .hsync_start = 1080 + 86,//HFP
    .hsync_end = 1080 + 86 + 4,//HSA
    .htotal = 1080 +86 + 4 + 84,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 40,//VFP
    .vsync_end = 2412 + 40 + 4,//VSA
    .vtotal = 2412 + 40+ 4 + 34,//VBP
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
    .pll_clk = 553,
    .cust_esd_check = 0,
    .esd_check_enable = 0,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
    },
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
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 170,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 67,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 7072,
        .flatness_minqp = 3,
        .flatness_maxqp = 12,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 11,
        .rc_quant_incr_limit1 = 11,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
        },
    .data_rate = 1107,
	.vendor = "ILI7807s_JDI_QQtang",
    .manufacture = "QQtang_ili_jdi3276",
};

static struct mtk_panel_params ext_params_90hz = {
    .pll_clk = 553,
    .cust_esd_check = 0,
    .esd_check_enable = 0,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
    },
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
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 170,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 67,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 7072,
        .flatness_minqp = 3,
        .flatness_maxqp = 12,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 11,
        .rc_quant_incr_limit1 = 11,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
        },
    .data_rate = 1107,
	.vendor = "ILI7807s_JDI_QQtang",
    .manufacture = "QQtang_ili_jdi3276",
};

static struct mtk_panel_params ext_params_120hz = {
    .pll_clk = 553,
    .cust_esd_check = 0,
    .esd_check_enable = 0,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
    },
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
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 170,
        .dec_delay = 526,
        .scale_value = 32,
        .increment_interval = 67,
        .decrement_interval = 7,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 7072,
        .flatness_minqp = 3,
        .flatness_maxqp = 12,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 11,
        .rc_quant_incr_limit1 = 11,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
        },
    .data_rate = 1107,
	.vendor = "ILI7807s_JDI_QQtang",
    .manufacture = "QQtang_ili_jdi3276",
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int jdi_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xFF};
//	char bl_tb1[] = {0xB2, 0x01};
//	char bl_tb2[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
	if (level > 4095){
		level = 4095;
	}
	pr_info("%s backlight = -%d\n", __func__, level);
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;

	if (!cb){
		return -1;
	}
	if (level ==1) {
		pr_err("enter aod!!!\n");
		return 0;
	}

//	if (level >2047)
//		bl_tb1[1] = 0x01;
//	else
//		bl_tb1[1] = 0x11;
	esd_brightness = level;
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
	unsigned int hbm_brightness = 0;
	int i = 0;

	printk("[lh] %s level is %d\n", __func__, level);

	if (level > BRIGHTNESS_HALF) {
		hbm_brightness = level;
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

		lcm_setbrightness_normal[0].para_list[1] = BL_LSB;
		lcm_setbrightness_normal[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_normal)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_normal[i].para_list, lcm_setbrightness_normal[i].count);
		}
	}
}
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;

	if (!cb){
		return -1;
	}

	pr_err("[lh]oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		//oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		//level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);  //level
		printk("[lh] %s : %d ! backlight %d !\n",__func__, hbm_mode, oplus_display_brightness);
	}

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	//char hbm_tb[] = {0x53, 0xe0};
	struct jdi *ctx = panel_to_jdi(panel);
	int i = 0;
	int level = 0;
	if (!cb){
		return -1;
	}
	if (ctx->hbm_en == en){
		goto done;
	}
	pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		level = oplus_display_brightness;
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		printk("[soso] %s : %d ! backlight %d !\n",__func__, en, oplus_display_brightness);
		if (level <= BRIGHTNESS_HALF)
			flag_hbm = 0;
		else
			flag_hbm = 1;
	}
	//lcdinfo_notify(1, &en);
	ctx->hbm_en = en;
	if (aod_finger_unlock_flag) {
		ctx->hbm_wait = false;
		aod_finger_unlock_flag = 0;
	}
	else{
		ctx->hbm_wait = true;
	}
done:
	return 0;
}

/*
static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
{
	struct jdi *ctx = panel_to_jdi(panel);

	*state = ctx->hbm_en;
}

static void panel_hbm_set_state(struct drm_panel *panel, bool state)
{
	struct jdi *ctx = panel_to_jdi(panel);

	ctx->hbm_en = state;
}

static void panel_hbm_get_wait_state(struct drm_panel *panel, bool *wait)
{
	struct jdi *ctx = panel_to_jdi(panel);

	*wait = ctx->hbm_wait;
}

static bool panel_hbm_set_wait_state(struct drm_panel *panel, bool wait)
{
	struct jdi *ctx = panel_to_jdi(panel);
	bool old = ctx->hbm_wait;

	ctx->hbm_wait = wait;
	return old;
}*/
/* static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
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
} */

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

extern bool oplus_fp_notify_down_delay;
static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;
	pr_err("debug for lcm %s, oplus_fp_notify_down_delay=%d\n", __func__, oplus_fp_notify_down_delay);

	/*if (oplus_fp_notify_down_delay)
		aod_finger_unlock_flag = 1;
    */
	// Switch back to VDO mode
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

	aod_state = false;

	return 0;
}
static struct LCM_setting_table lcm_normal_to_aod_sam[] = {
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

    {REGFLAG_CMD, 19, {0x91,0x89,0x28,0x00,0x14,0xC2,0x00,0x03,0x1C,0x02,0x8C,0x00,0x0F,0x05,0x0E,0x02,0x8B,0x10,0xF0}},
    {REGFLAG_CMD, 2, {0x03,0x01}},
    {REGFLAG_CMD, 2, {0x90,0x01}},
    {REGFLAG_CMD, 1, {0x2C}},
    {REGFLAG_CMD, 2, {0x82,0xAE}},
    {REGFLAG_CMD, 2, {0x2F,0x01}},
    {REGFLAG_CMD, 1, {0x11}},
    {REGFLAG_DELAY,120,{}},
    {REGFLAG_CMD, 1, {0x29}},
    //{REGFLAG_DELAY,20,{}},

    //{REGFLAG_CMD, 1, {0x28}},
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
			cb(dsi, handle, lcm_aod_high_mode[i].para_list, lcm_aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(lcm_aod_low_mode)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_aod_low_mode[i].para_list, lcm_aod_low_mode[i].count);
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

	pr_err("[lh]debug for samsung_amb670 lcm %s\n", __func__);

	if (ctx->prepared){
		return 0;
	}
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
/*	ctx->pw_1p8_gpio = devm_gpiod_get(ctx->dev, "pw-1p8", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->pw_1p8_gpio, 1);
        devm_gpiod_put(ctx->dev, ctx->pw_1p8_gpio);
        msleep(5);
        ctx->pw_reset_gpio = devm_gpiod_get(ctx->dev, "pw-reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->pw_reset_gpio, 1);
        msleep(5);
        gpiod_set_value(ctx->pw_reset_gpio, 0);
        msleep(5);
        gpiod_set_value(ctx->pw_reset_gpio, 1);
        devm_gpiod_put(ctx->dev, ctx->pw_reset_gpio);*/
	msleep(5);
	lcm_panel_vufs_ldo_enable(ctx->dev);
	msleep(5);
	/* ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio); */
    	ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "cannot get bias-gpios 0 %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
    gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

    usleep_range(5000, 5100);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "cannot get bias-gpios 1 %ld\n",
		PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
    gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	usleep_range(5000, 5100);
	//set vddi 3.0v
	//lcm_panel_vmch_ldo_enable(ctx->dev);

	ret = ctx->error;
	if (ret < 0){
		jdi_unprepare(panel);
	}
	usleep_range(5000, 5100);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;
	int blank = 0;
	int mode = 0;

	pr_err("[lh]debug for amb670yf01 lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);

	if (ctx->prepared){
		return 0;
	}
	if((mode != MSM_BOOT_MODE__FACTORY) && (mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
		if(tp_gesture_enable_notifier && tp_gesture_enable_notifier(0)) {
			is_pd_with_guesture = true;
			pr_err("[TP] tp gesture  is enable,Display not to poweroff\n");
			return 0;
		} else {
			is_pd_with_guesture = false;
			#define LCD_CTL_RST_OFF 0x12
			#define LCD_CTL_CS_OFF  0x1A
			blank = LCD_CTL_RST_OFF;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_err("[TP] tp gesture is disable, Display goto power off , And TP reset will low\n");
			blank = LCD_CTL_CS_OFF;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_err("[TP]TP CS will chang to gpio mode and low\n");

		}
	}

	if((mode == MSM_BOOT_MODE__FACTORY) || (mode == MSM_BOOT_MODE__RF) || (mode == MSM_BOOT_MODE__WLAN)) {
			#define LCD_CTL_RST_OFF 0x12
			#define LCD_CTL_CS_OFF  0x1A
			blank = LCD_CTL_RST_OFF;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_err("[TP] Now mode is %d\n",mode);
			pr_err("TP reset will low\n");
			blank = LCD_CTL_CS_OFF;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_err("[TP]TP CS will chang to gpio mode and low\n");
	}

/*
	ctx->pw_1p8_gpio = devm_gpiod_get(ctx->dev, "pw-gpio", GPIOD_OUT_HIGH);
          gpiod_set_value(ctx->pw_1p8_gpio, 0);
          devm_gpiod_put(ctx->dev, ctx->pw_1p8_gpio);

         ctx->pw_reset_gpio = devm_gpiod_get(ctx->dev, "pw-reset", GPIOD_OUT_HIGH);
          gpiod_set_value(ctx->pw_reset_gpio, 0);
          devm_gpiod_put(ctx->dev, ctx->pw_reset_gpio);
	usleep_range(5000, 5100);*/
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(5);
	//lcm_panel_ldo3_disable(ctx->dev);
	//set vddi 3.0v
        //lcm_panel_vmch_ldo_disable(ctx->dev);
        usleep_range(5000, 5100);
	//enable 1.5V
	/* ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);
	usleep_range(5000, 5100); */
	//set vddi 3.0v
		ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "cannot get bias-gpios 1 %ld\n",
		PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
    gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
    	

    usleep_range(5000, 5100);
ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "cannot get bias-gpios 0 %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
    gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

    usleep_range(5000, 5100);
	lcm_panel_vufs_ldo_disable(ctx->dev);

	ret = ctx->error;
	if (ret < 0){
		jdi_unprepare(panel);
	}
	//msleep(110);
	return 0;
}

struct drm_display_mode *get_mode_by_id_hfp(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode){
			return m;
		}
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

    if (drm_mode_vrefresh(m) == 60){
        ext->params = &ext_params;
	}
    else if (drm_mode_vrefresh(m) == 90){
        ext->params = &ext_params_90hz;
	}
    else if (drm_mode_vrefresh(m) == 120){
        ext->params = &ext_params_120hz;
	}
    else{
        ret = 1;
	}

	return ret;
}

/*static void mode_switch_to_120(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	pr_info("%s\n", __func__);

        jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
	jdi_dcs_write_seq_static(ctx,0x38,0x11);
	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
}

static void mode_switch_to_90(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	pr_info("%s\n", __func__);

	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
        jdi_dcs_write_seq_static(ctx,0x38,0x12);
        jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
}

static void mode_switch_to_60(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	pr_info("%s\n", __func__);

	jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
        jdi_dcs_write_seq_static(ctx,0x38,0x13);
        jdi_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);

}*/

/* static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
    return ret;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, dst_mode);

	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);

	if (drm_mode_vrefresh(m) == 60) {  //60 switch to 120
		mode_switch_to_60(panel);
	} else if (drm_mode_vrefresh(m) == 90) { // 1200 switch to 60
		mode_switch_to_90(panel);
	} else if (drm_mode_vrefresh(m) == 120) { // 1200 switch to 60
		mode_switch_to_120(panel);
	} else
		ret = 1;
} */

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = jdi_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	// .mode_switch = mode_switch,
	//.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.set_hbm = lcm_set_hbm,
	//.hbm_get_state = panel_hbm_get_state,
	//.hbm_set_state = panel_hbm_set_state,
	//.hbm_get_wait_state = panel_hbm_get_wait_state,
	//.hbm_set_wait_state = panel_hbm_set_wait_state,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.set_aod_light_mode = panel_set_aod_light_mode,
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

	connector->display_info.width_mm = 68;
	connector->display_info.height_mm = 153;

	return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
	.disable = jdi_disable,
	.unprepare = jdi_unprepare,
	.prepare = jdi_prepare,
	.enable = jdi_enable,
	.get_modes = jdi_get_modes,
};

static int jdi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct jdi *ctx;
	struct device_node *backlight;
	int ret;
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	unsigned int fp_type = 0x01;
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	pr_info("[tianma] ili7838a %s+\n", __func__);

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
	if (!ctx){
		return -ENOMEM;
	}
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
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight){
			return -EPROBE_DEFER;
		}
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
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
	/*ctx->vddr1p5_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p5_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddr1p5_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p5_enable_gpio));
		return PTR_ERR(ctx->vddr1p5_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vddr1p5_enable_gpio);

	usleep_range(5000, 5100);*/

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "cannot get bias-gpios 0 %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
    gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

    usleep_range(5000, 5100);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "cannot get bias-gpios 1 %ld\n",
		PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
    gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

    usleep_range(5000, 5100);

        //lcm_panel_vmch_ldo_enable(ctx->dev);
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

	ret = mipi_dsi_attach(dsi);
	if (ret < 0){
		drm_panel_remove(&ctx->panel);
	}

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0){
		return ret;
	}

#endif
	register_device_proc("lcd", "ILI7807s_JDI_QQtang", "QQtang_ili_jdi3276");

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	oplus_ofp_set_fp_type(&fp_type);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

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
	    .compatible = "oplus21143,ili7807s,vdo",
	},
	{}
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
		.name = "oplus21143_ili7807s_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = jdi_of_match,
	},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("JDI NT36672E VDO 60HZ AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
