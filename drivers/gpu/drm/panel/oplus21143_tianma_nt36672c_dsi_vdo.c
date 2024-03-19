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

#include <linux/regulator/driver.h>
#include "../../../regulator/internal.h"

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

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/oplus_21143_tianma_mtk_data_hw_roundedpattern.h"
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
#define MAX_NORMAL_BRIGHTNESS   3276
#define LCM_BRIGHTNESS_TYPE 2
static unsigned int esd_brightness = 1023;
static u32 flag_hbm = 0;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
static int aod_finger_unlock_flag = 0;
static bool is_pd_with_guesture = false;
extern unsigned int g_shutdown_flag;
// static bool aod_state = false;
static int bl_gamma = 0;
static unsigned int cabc_lastlevel = 1;
static unsigned int last_brightness = 0;
static int flag_i2c = 1;
static int flag_dimming = 0;
static int poweroff_success = 1;
extern unsigned long esd_flag;
static bool is_probe_finish = false;
static bool is_probe_double = false;
extern int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
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

/* static struct LCM_setting_table lcm_aod_to_normal[] = {
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
}; */

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
static struct regulator_dev *rdev_origin;
static int lcm_panel_vufs_ldo_regulator_init(struct device *dev)
{
        static int regulator_inited;
        int ret = 0;

        if (is_probe_double == true)
            pr_info("%s panel probe double\n", __func__);

        if (regulator_inited)
            return ret;

        pr_err("get lcm_panel_vufs_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        vufs_ldo = devm_regulator_get(dev, "vufsldo");
        if (IS_ERR_OR_NULL(vufs_ldo)) { /* handle return value */
                ret = PTR_ERR(vufs_ldo);
                pr_err("get vufs_ldo fail, error: %d\n", ret);
                //return ret;
        } else {
            rdev_origin = vufs_ldo->rdev;
            if (!IS_ERR_OR_NULL(rdev_origin)) {
                pr_info("%s-%d rdev_origin inited: %p\n", __func__, __LINE__,rdev_origin);
            }
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
        if (IS_ERR_OR_NULL(vufs_ldo->rdev) || (vufs_ldo->rdev != rdev_origin)) {
            pr_err("%s-%d vufs_ldo->rdev is invalid\n", __func__, __LINE__);
            if (!IS_ERR_OR_NULL(rdev_origin)) {
                vufs_ldo->rdev = rdev_origin;
                pr_err("%s-%d vufs_ldo->rdev restore: %p\n", __func__, __LINE__,vufs_ldo->rdev);
            }
        }
        ret = regulator_set_voltage(vufs_ldo, 1800000, 1800000);
        if (ret < 0) {
            pr_err("set voltage vufs_ldo fail, ret = %d\n", ret);
        }
        retval |= ret;
    }
        /* enable regulator */
    if (!IS_ERR_OR_NULL(vufs_ldo)) {
        if (IS_ERR_OR_NULL(vufs_ldo->rdev) || (vufs_ldo->rdev != rdev_origin)) {
            pr_err("%s-%d vufs_ldo->rdev is invalid\n", __func__, __LINE__);
            if (!IS_ERR_OR_NULL(rdev_origin)) {
                vufs_ldo->rdev = rdev_origin;
                pr_err("%s-%d vufs_ldo->rdev restore: %p\n", __func__, __LINE__,vufs_ldo->rdev);
            }
        }
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
        if (IS_ERR_OR_NULL(vufs_ldo->rdev) || (vufs_ldo->rdev != rdev_origin)) {
            pr_err("%s-%d vufs_ldo->rdev is invalid\n", __func__, __LINE__);
            if (!IS_ERR_OR_NULL(rdev_origin)) {
                vufs_ldo->rdev = rdev_origin;
                pr_err("%s-%d vufs_ldo->rdev restore: %p\n", __func__, __LINE__,vufs_ldo->rdev);
            }
        }
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
/* jdi_dcs_write_seq_static(ctx,0xFF, 0x10);
jdi_dcs_write_seq_static(ctx,0xFB, 0x01);
jdi_dcs_write_seq_static(ctx,0x3B, 0x03,0x14,0x36,0x04,0x04);
jdi_dcs_write_seq_static(ctx,0xB0, 0x00);
jdi_dcs_write_seq_static(ctx,0xC0, 0x00);
jdi_dcs_write_seq_static(ctx,0xFF, 0x25);

    jdi_dcs_write_seq_static(ctx,0xFB, 0x01);
    jdi_dcs_write_seq_static(ctx,0x18, 0x22);
    jdi_dcs_write_seq_static(ctx,0xFF, 0xE0);
    jdi_dcs_write_seq_static(ctx,0xFB, 0x01);
    jdi_dcs_write_seq_static(ctx,0x35, 0x82);

    jdi_dcs_write_seq_static(ctx,0xFF, 0xF0);
    jdi_dcs_write_seq_static(ctx,0xFB, 0x01);
    jdi_dcs_write_seq_static(ctx,0x1C, 0x01);
    jdi_dcs_write_seq_static(ctx,0x33, 0x01);
    jdi_dcs_write_seq_static(ctx,0x5A, 0x00);
    jdi_dcs_write_seq_static(ctx,0xFF, 0xD0);
    jdi_dcs_write_seq_static(ctx,0xFB, 0x01);
    jdi_dcs_write_seq_static(ctx,0x53, 0x22);
    jdi_dcs_write_seq_static(ctx,0x54, 0x02);

    jdi_dcs_write_seq_static(ctx,0xFF, 0xC0);
    jdi_dcs_write_seq_static(ctx,0xFB, 0x01);
    jdi_dcs_write_seq_static(ctx,0x9C, 0x11);
    jdi_dcs_write_seq_static(ctx,0x9D, 0x11);
    jdi_dcs_write_seq_static(ctx,0xFF, 0x10);
    jdi_dcs_write_seq_static(ctx,0x53, 0x2C);
    jdi_dcs_write_seq_static(ctx,0x55, 0x00);
    jdi_dcs_write_seq_static(ctx,0x35, 0x00);
    jdi_dcs_write_seq_static(ctx,0x11,0x00);
    msleep(121);
    jdi_dcs_write_seq_static(ctx,0x29,0x00);
    msleep(51);
    pr_info("SYQ %s-\n", __func__); */

    jdi_dcs_write_seq_static(ctx,0xFF, 0x10);
    jdi_dcs_write_seq_static(ctx,0xFB, 0x01);
    jdi_dcs_write_seq_static(ctx,0x36, 0x00);
    jdi_dcs_write_seq_static(ctx,0x3B, 0x03,0x14,0x36,0x04,0x04);
    jdi_dcs_write_seq_static(ctx,0xB0, 0x00);
    jdi_dcs_write_seq_static(ctx,0xC0, 0x03);
    jdi_dcs_write_seq_static(ctx,0xC1, 0x89,0x28,0x00,0x0C,0x00,0xAA,0x02,0x0E,0x00,0x43,0x00,0x07,0x08,0xBB,0x08,0x7A);
    jdi_dcs_write_seq_static(ctx,0xC2, 0x1B,0xA0);

    jdi_dcs_write_seq_static(ctx,0xFF,0x20);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x01,0x66);
    jdi_dcs_write_seq_static(ctx,0x06,0x64);
    jdi_dcs_write_seq_static(ctx,0x07,0x28);
    jdi_dcs_write_seq_static(ctx,0x17,0x66);
    jdi_dcs_write_seq_static(ctx,0x1B,0x01);
    jdi_dcs_write_seq_static(ctx,0x2F,0x83);
    jdi_dcs_write_seq_static(ctx,0x5C,0x90);
    jdi_dcs_write_seq_static(ctx,0x5E,0xB6);
    jdi_dcs_write_seq_static(ctx,0x69,0xD0);
    jdi_dcs_write_seq_static(ctx,0x95,0xD1);
    jdi_dcs_write_seq_static(ctx,0x96,0xD1);
    jdi_dcs_write_seq_static(ctx,0xF2,0x65);
    jdi_dcs_write_seq_static(ctx,0xF3,0x54);
    jdi_dcs_write_seq_static(ctx,0xF4,0x65);
    jdi_dcs_write_seq_static(ctx,0xF5,0x54);
    jdi_dcs_write_seq_static(ctx,0xF6,0x65);
    jdi_dcs_write_seq_static(ctx,0xF7,0x54);
    jdi_dcs_write_seq_static(ctx,0xF8,0x65);
    jdi_dcs_write_seq_static(ctx,0xF9,0x54);

    jdi_dcs_write_seq_static(ctx,0xFF,0x23);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x00,0x80);
    jdi_dcs_write_seq_static(ctx,0x07,0x00);
    jdi_dcs_write_seq_static(ctx,0x08,0x01);
    jdi_dcs_write_seq_static(ctx,0x09,0x04);
    jdi_dcs_write_seq_static(ctx,0x11,0x01);
    jdi_dcs_write_seq_static(ctx,0x12,0x96);
    jdi_dcs_write_seq_static(ctx,0x15,0x68);
    jdi_dcs_write_seq_static(ctx,0x16,0x0B);

    jdi_dcs_write_seq_static(ctx,0xFF,0x24);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x03,0x13);
    jdi_dcs_write_seq_static(ctx,0x04,0x15);
    jdi_dcs_write_seq_static(ctx,0x05,0x17);
    jdi_dcs_write_seq_static(ctx,0x09,0x26);
    jdi_dcs_write_seq_static(ctx,0x0A,0x27);
    jdi_dcs_write_seq_static(ctx,0x0B,0x28);
    jdi_dcs_write_seq_static(ctx,0x0C,0x24);
    jdi_dcs_write_seq_static(ctx,0x0D,0x01);
    jdi_dcs_write_seq_static(ctx,0x0E,0x2F);
    jdi_dcs_write_seq_static(ctx,0x0F,0x2D);
    jdi_dcs_write_seq_static(ctx,0x10,0x2E);
    jdi_dcs_write_seq_static(ctx,0x11,0x2C);
    jdi_dcs_write_seq_static(ctx,0x12,0x8B);
    jdi_dcs_write_seq_static(ctx,0x13,0x8C);
    jdi_dcs_write_seq_static(ctx,0x16,0x0F);
    jdi_dcs_write_seq_static(ctx,0x17,0x22);
    jdi_dcs_write_seq_static(ctx,0x1B,0x13);
    jdi_dcs_write_seq_static(ctx,0x1C,0x15);
    jdi_dcs_write_seq_static(ctx,0x1D,0x17);
    jdi_dcs_write_seq_static(ctx,0x21,0x26);
    jdi_dcs_write_seq_static(ctx,0x22,0x27);
    jdi_dcs_write_seq_static(ctx,0x23,0x28);
    jdi_dcs_write_seq_static(ctx,0x24,0x24);
    jdi_dcs_write_seq_static(ctx,0x25,0x01);
    jdi_dcs_write_seq_static(ctx,0x26,0x2F);
    jdi_dcs_write_seq_static(ctx,0x27,0x2D);
    jdi_dcs_write_seq_static(ctx,0x28,0x2E);
    jdi_dcs_write_seq_static(ctx,0x29,0x2C);
    jdi_dcs_write_seq_static(ctx,0x2A,0x8B);
    jdi_dcs_write_seq_static(ctx,0x2B,0x8C);
    jdi_dcs_write_seq_static(ctx,0x30,0x0F);
    jdi_dcs_write_seq_static(ctx,0x31,0x22);
    jdi_dcs_write_seq_static(ctx,0x32,0x00);
    jdi_dcs_write_seq_static(ctx,0x33,0x03);
    jdi_dcs_write_seq_static(ctx,0x35,0x01);
    jdi_dcs_write_seq_static(ctx,0x36,0x01);
    jdi_dcs_write_seq_static(ctx,0x4E,0x2F);
    jdi_dcs_write_seq_static(ctx,0x4F,0x30);
    jdi_dcs_write_seq_static(ctx,0x53,0x2F);
    jdi_dcs_write_seq_static(ctx,0x71,0x28);
    jdi_dcs_write_seq_static(ctx,0x77,0x80);
    jdi_dcs_write_seq_static(ctx,0x79,0x04);
    jdi_dcs_write_seq_static(ctx,0x7A,0x02);
    jdi_dcs_write_seq_static(ctx,0x7B,0x8D);
    jdi_dcs_write_seq_static(ctx,0x7D,0x03);
    jdi_dcs_write_seq_static(ctx,0x80,0x03);
    jdi_dcs_write_seq_static(ctx,0x81,0x03);
    jdi_dcs_write_seq_static(ctx,0x82,0x13);
    jdi_dcs_write_seq_static(ctx,0x84,0x31);
    jdi_dcs_write_seq_static(ctx,0x85,0x13);
    jdi_dcs_write_seq_static(ctx,0x86,0x22);
    jdi_dcs_write_seq_static(ctx,0x87,0x31);
    jdi_dcs_write_seq_static(ctx,0x90,0x13);
    jdi_dcs_write_seq_static(ctx,0x92,0x31);
    jdi_dcs_write_seq_static(ctx,0x93,0x13);
    jdi_dcs_write_seq_static(ctx,0x94,0x22);
    jdi_dcs_write_seq_static(ctx,0x95,0x31);
    jdi_dcs_write_seq_static(ctx,0x9C,0xF4);
    jdi_dcs_write_seq_static(ctx,0x9D,0x01);
    jdi_dcs_write_seq_static(ctx,0xA0,0x0D);
    jdi_dcs_write_seq_static(ctx,0xA2,0x0D);
    jdi_dcs_write_seq_static(ctx,0xA3,0x02);
    jdi_dcs_write_seq_static(ctx,0xA4,0x03);
    jdi_dcs_write_seq_static(ctx,0xA5,0x03);
    jdi_dcs_write_seq_static(ctx,0xC6,0x40);
    jdi_dcs_write_seq_static(ctx,0xC9,0x00);
    jdi_dcs_write_seq_static(ctx,0xD9,0x80);
    jdi_dcs_write_seq_static(ctx,0xE9,0x02);

    jdi_dcs_write_seq_static(ctx,0xFF,0x25);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x0F,0x1B);
    jdi_dcs_write_seq_static(ctx,0x18,0x20);
    jdi_dcs_write_seq_static(ctx,0x19,0xE4);
    jdi_dcs_write_seq_static(ctx,0x20,0x00);
    jdi_dcs_write_seq_static(ctx,0x21,0x40);
    jdi_dcs_write_seq_static(ctx,0x66,0x40);
    jdi_dcs_write_seq_static(ctx,0x67,0x29);
    jdi_dcs_write_seq_static(ctx,0x68,0x50);
    jdi_dcs_write_seq_static(ctx,0x69,0x60);
    jdi_dcs_write_seq_static(ctx,0x6B,0x00);
    jdi_dcs_write_seq_static(ctx,0x71,0x6D);
    jdi_dcs_write_seq_static(ctx,0x77,0x60);
    jdi_dcs_write_seq_static(ctx,0x78,0xA5);
    jdi_dcs_write_seq_static(ctx,0x7D,0x40);
    jdi_dcs_write_seq_static(ctx,0x7E,0x2D);
    jdi_dcs_write_seq_static(ctx,0xC0,0x4D);
    jdi_dcs_write_seq_static(ctx,0xC1,0xA9);
    jdi_dcs_write_seq_static(ctx,0xC2,0xD2);
    jdi_dcs_write_seq_static(ctx,0xC4,0x11);
    jdi_dcs_write_seq_static(ctx,0xD6,0x80);
    jdi_dcs_write_seq_static(ctx,0xD7,0x82);
    jdi_dcs_write_seq_static(ctx,0xDA,0x02);
    jdi_dcs_write_seq_static(ctx,0xDD,0x02);
    jdi_dcs_write_seq_static(ctx,0xE0,0x02);
    jdi_dcs_write_seq_static(ctx,0xF0,0x00);
    jdi_dcs_write_seq_static(ctx,0xF1,0x04);

    jdi_dcs_write_seq_static(ctx,0xFF,0x26);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x00,0x10);
    jdi_dcs_write_seq_static(ctx,0x01,0xEB);
    jdi_dcs_write_seq_static(ctx,0x03,0x00);
    jdi_dcs_write_seq_static(ctx,0x04,0xEB);
    jdi_dcs_write_seq_static(ctx,0x05,0x08);
    jdi_dcs_write_seq_static(ctx,0x06,0x0E);
    jdi_dcs_write_seq_static(ctx,0x08,0x0E);
    jdi_dcs_write_seq_static(ctx,0x14,0x06);
    jdi_dcs_write_seq_static(ctx,0x15,0x01);
    jdi_dcs_write_seq_static(ctx,0x74,0xAF);
    jdi_dcs_write_seq_static(ctx,0x81,0x0D);
    jdi_dcs_write_seq_static(ctx,0x83,0x02);
    jdi_dcs_write_seq_static(ctx,0x84,0x03);
    jdi_dcs_write_seq_static(ctx,0x85,0x01);
    jdi_dcs_write_seq_static(ctx,0x86,0x03);
    jdi_dcs_write_seq_static(ctx,0x87,0x01);
    jdi_dcs_write_seq_static(ctx,0x88,0x02);
    jdi_dcs_write_seq_static(ctx,0x8A,0x1A);
    jdi_dcs_write_seq_static(ctx,0x8B,0x11);
    jdi_dcs_write_seq_static(ctx,0x8C,0x24);
    jdi_dcs_write_seq_static(ctx,0x8E,0x42);
    jdi_dcs_write_seq_static(ctx,0x8F,0x11);
    jdi_dcs_write_seq_static(ctx,0x90,0x11);
    jdi_dcs_write_seq_static(ctx,0x91,0x11);
    jdi_dcs_write_seq_static(ctx,0x9A,0x80);
    jdi_dcs_write_seq_static(ctx,0x9B,0x04);
    jdi_dcs_write_seq_static(ctx,0x9C,0x00);
    jdi_dcs_write_seq_static(ctx,0x9D,0x00);
    jdi_dcs_write_seq_static(ctx,0x9E,0x00);

    jdi_dcs_write_seq_static(ctx,0xFF,0x27);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x01,0x6C);
    jdi_dcs_write_seq_static(ctx,0x20,0x81);
    jdi_dcs_write_seq_static(ctx,0x21,0x6B);
    jdi_dcs_write_seq_static(ctx,0x25,0x81);
    jdi_dcs_write_seq_static(ctx,0x26,0x96);
    jdi_dcs_write_seq_static(ctx,0x6E,0x23);
    jdi_dcs_write_seq_static(ctx,0x6F,0x01);
    jdi_dcs_write_seq_static(ctx,0x70,0x00);
    jdi_dcs_write_seq_static(ctx,0x71,0x00);
    jdi_dcs_write_seq_static(ctx,0x72,0x00);
    jdi_dcs_write_seq_static(ctx,0x73,0x21);
    jdi_dcs_write_seq_static(ctx,0x74,0x03);
    jdi_dcs_write_seq_static(ctx,0x75,0x00);
    jdi_dcs_write_seq_static(ctx,0x76,0x00);
    jdi_dcs_write_seq_static(ctx,0x77,0x00);
    jdi_dcs_write_seq_static(ctx,0x7D,0x09);
    jdi_dcs_write_seq_static(ctx,0x7E,0x6F);
    jdi_dcs_write_seq_static(ctx,0x80,0x23);
    jdi_dcs_write_seq_static(ctx,0x82,0x09);
    jdi_dcs_write_seq_static(ctx,0x83,0x6F);
    jdi_dcs_write_seq_static(ctx,0x88,0x03);
    jdi_dcs_write_seq_static(ctx,0x89,0x01);
    jdi_dcs_write_seq_static(ctx,0xE3,0x01);
    jdi_dcs_write_seq_static(ctx,0xE4,0xE4);
    jdi_dcs_write_seq_static(ctx,0xE5,0x02);
    jdi_dcs_write_seq_static(ctx,0xE6,0xD7);
    jdi_dcs_write_seq_static(ctx,0xE9,0x02);
    jdi_dcs_write_seq_static(ctx,0xEA,0x1D);
    jdi_dcs_write_seq_static(ctx,0xEB,0x03);
    jdi_dcs_write_seq_static(ctx,0xEC,0x2B);

    jdi_dcs_write_seq_static(ctx,0xFF,0x2A);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x00,0x91);
    jdi_dcs_write_seq_static(ctx,0x03,0x20);
    jdi_dcs_write_seq_static(ctx,0x07,0x64);
    jdi_dcs_write_seq_static(ctx,0x0A,0x70);
    jdi_dcs_write_seq_static(ctx,0x0C,0x09);
    jdi_dcs_write_seq_static(ctx,0x0D,0x40);
    jdi_dcs_write_seq_static(ctx,0x0E,0x02);
    jdi_dcs_write_seq_static(ctx,0x0F,0x00);
    jdi_dcs_write_seq_static(ctx,0x11,0xF1);
    jdi_dcs_write_seq_static(ctx,0x15,0x0E);
    jdi_dcs_write_seq_static(ctx,0x16,0xB9);
    jdi_dcs_write_seq_static(ctx,0x19,0x0E);
    jdi_dcs_write_seq_static(ctx,0x1A,0x8D);
    jdi_dcs_write_seq_static(ctx,0x1B,0x14);
    jdi_dcs_write_seq_static(ctx,0x1D,0x36);
    jdi_dcs_write_seq_static(ctx,0x1E,0x36);
    jdi_dcs_write_seq_static(ctx,0x1F,0x36);
    jdi_dcs_write_seq_static(ctx,0x20,0x37);
    jdi_dcs_write_seq_static(ctx,0x28,0xE1);
    jdi_dcs_write_seq_static(ctx,0x29,0x1F);
    jdi_dcs_write_seq_static(ctx,0x2A,0xFF);
    jdi_dcs_write_seq_static(ctx,0x2B,0x00);
    jdi_dcs_write_seq_static(ctx,0x2D,0x05);
    jdi_dcs_write_seq_static(ctx,0x2F,0x06);
    jdi_dcs_write_seq_static(ctx,0x30,0x1E);
    jdi_dcs_write_seq_static(ctx,0x31,0x8D);
    jdi_dcs_write_seq_static(ctx,0x33,0x1E);
    jdi_dcs_write_seq_static(ctx,0x34,0xE3);
    jdi_dcs_write_seq_static(ctx,0x35,0x45);
    jdi_dcs_write_seq_static(ctx,0x36,0xCB);
    jdi_dcs_write_seq_static(ctx,0x37,0xDC);
    jdi_dcs_write_seq_static(ctx,0x38,0x4B);
    jdi_dcs_write_seq_static(ctx,0x39,0xC5);
    jdi_dcs_write_seq_static(ctx,0x3A,0x1E);
    jdi_dcs_write_seq_static(ctx,0x46,0x40);
    jdi_dcs_write_seq_static(ctx,0x47,0x02);
    jdi_dcs_write_seq_static(ctx,0x4A,0xF1);
    jdi_dcs_write_seq_static(ctx,0x4E,0x0E);
    jdi_dcs_write_seq_static(ctx,0x4F,0xB9);
    jdi_dcs_write_seq_static(ctx,0x52,0x0E);
    jdi_dcs_write_seq_static(ctx,0x53,0x8D);
    jdi_dcs_write_seq_static(ctx,0x54,0x14);
    jdi_dcs_write_seq_static(ctx,0x56,0x36);
    jdi_dcs_write_seq_static(ctx,0x57,0x4E);
    jdi_dcs_write_seq_static(ctx,0x58,0x4E);
    jdi_dcs_write_seq_static(ctx,0x59,0x4E);
    jdi_dcs_write_seq_static(ctx,0x60,0x80);
    jdi_dcs_write_seq_static(ctx,0x61,0xDF);
    jdi_dcs_write_seq_static(ctx,0x62,0x17);
    jdi_dcs_write_seq_static(ctx,0x63,0xDB);
    jdi_dcs_write_seq_static(ctx,0x65,0x05);
    jdi_dcs_write_seq_static(ctx,0x66,0x04);
    jdi_dcs_write_seq_static(ctx,0x67,0x54);
    jdi_dcs_write_seq_static(ctx,0x68,0xA5);
    jdi_dcs_write_seq_static(ctx,0x6A,0xE0);
    jdi_dcs_write_seq_static(ctx,0x6B,0xE1);
    jdi_dcs_write_seq_static(ctx,0x6C,0x30);
    jdi_dcs_write_seq_static(ctx,0x6D,0x00);
    jdi_dcs_write_seq_static(ctx,0x6E,0xDC);
    jdi_dcs_write_seq_static(ctx,0x6F,0x34);
    jdi_dcs_write_seq_static(ctx,0x70,0xFC);
    jdi_dcs_write_seq_static(ctx,0x71,0x14);
    jdi_dcs_write_seq_static(ctx,0x7A,0x0E);
    jdi_dcs_write_seq_static(ctx,0x7B,0x40);
    jdi_dcs_write_seq_static(ctx,0x7C,0x03);
    jdi_dcs_write_seq_static(ctx,0x7F,0xA0);
    jdi_dcs_write_seq_static(ctx,0x83,0x0E);
    jdi_dcs_write_seq_static(ctx,0x84,0xB9);
    jdi_dcs_write_seq_static(ctx,0x87,0x0E);
    jdi_dcs_write_seq_static(ctx,0x88,0x8D);
    jdi_dcs_write_seq_static(ctx,0x89,0x14);
    jdi_dcs_write_seq_static(ctx,0x8B,0x36);
    jdi_dcs_write_seq_static(ctx,0x8C,0x75);
    jdi_dcs_write_seq_static(ctx,0x8D,0x75);
    jdi_dcs_write_seq_static(ctx,0x8E,0x75);
    jdi_dcs_write_seq_static(ctx,0x95,0x80);
    jdi_dcs_write_seq_static(ctx,0x96,0x99);
    jdi_dcs_write_seq_static(ctx,0x97,0x0B);
    jdi_dcs_write_seq_static(ctx,0x98,0xF3);
    jdi_dcs_write_seq_static(ctx,0x9A,0x03);
    jdi_dcs_write_seq_static(ctx,0x9B,0x03);
    jdi_dcs_write_seq_static(ctx,0x9C,0x0F);
    jdi_dcs_write_seq_static(ctx,0x9D,0x73);
    jdi_dcs_write_seq_static(ctx,0x9F,0xE7);
    jdi_dcs_write_seq_static(ctx,0xA0,0x9B);
    jdi_dcs_write_seq_static(ctx,0xA2,0x21);
    jdi_dcs_write_seq_static(ctx,0xA3,0xAA);
    jdi_dcs_write_seq_static(ctx,0xA4,0x98);
    jdi_dcs_write_seq_static(ctx,0xA5,0x23);
    jdi_dcs_write_seq_static(ctx,0xA6,0xA8);
    jdi_dcs_write_seq_static(ctx,0xA7,0x0F);

    jdi_dcs_write_seq_static(ctx,0xFF,0x2C);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x00,0x02);
    jdi_dcs_write_seq_static(ctx,0x01,0x02);
    jdi_dcs_write_seq_static(ctx,0x02,0x02);
    jdi_dcs_write_seq_static(ctx,0x03,0x12);
    jdi_dcs_write_seq_static(ctx,0x04,0x12);
    jdi_dcs_write_seq_static(ctx,0x05,0x12);
    jdi_dcs_write_seq_static(ctx,0x0D,0x01);
    jdi_dcs_write_seq_static(ctx,0x0E,0x01);
    jdi_dcs_write_seq_static(ctx,0x17,0x42);
    jdi_dcs_write_seq_static(ctx,0x18,0x42);
    jdi_dcs_write_seq_static(ctx,0x19,0x42);
    jdi_dcs_write_seq_static(ctx,0x2D,0xAF);
    jdi_dcs_write_seq_static(ctx,0x2F,0x10);
    jdi_dcs_write_seq_static(ctx,0x30,0xEB);
    jdi_dcs_write_seq_static(ctx,0x32,0x00);
    jdi_dcs_write_seq_static(ctx,0x33,0xEB);
    jdi_dcs_write_seq_static(ctx,0x35,0x14);
    jdi_dcs_write_seq_static(ctx,0x37,0x14);
    jdi_dcs_write_seq_static(ctx,0x4D,0x12);
    jdi_dcs_write_seq_static(ctx,0x4E,0x02);
    jdi_dcs_write_seq_static(ctx,0x4F,0x0B);
    jdi_dcs_write_seq_static(ctx,0x53,0x02);
    jdi_dcs_write_seq_static(ctx,0x54,0x02);
    jdi_dcs_write_seq_static(ctx,0x55,0x02);
    jdi_dcs_write_seq_static(ctx,0x56,0x1F);
    jdi_dcs_write_seq_static(ctx,0x58,0x1F);
    jdi_dcs_write_seq_static(ctx,0x59,0x1F);
    jdi_dcs_write_seq_static(ctx,0x61,0x01);
    jdi_dcs_write_seq_static(ctx,0x62,0x01);
    jdi_dcs_write_seq_static(ctx,0x6B,0x69);
    jdi_dcs_write_seq_static(ctx,0x6C,0x69);
    jdi_dcs_write_seq_static(ctx,0x6D,0x69);
    jdi_dcs_write_seq_static(ctx,0x80,0xAF);
    jdi_dcs_write_seq_static(ctx,0x81,0x10);
    jdi_dcs_write_seq_static(ctx,0x82,0xEB);
    jdi_dcs_write_seq_static(ctx,0x84,0x00);
    jdi_dcs_write_seq_static(ctx,0x85,0xEB);
    jdi_dcs_write_seq_static(ctx,0x87,0x1E);
    jdi_dcs_write_seq_static(ctx,0x89,0x1E);
    jdi_dcs_write_seq_static(ctx,0x9D,0x1F);
    jdi_dcs_write_seq_static(ctx,0x9E,0x02);
    jdi_dcs_write_seq_static(ctx,0x9F,0x0B);

    jdi_dcs_write_seq_static(ctx,0xFF,0xE0);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x35,0x82);

    jdi_dcs_write_seq_static(ctx,0xFF,0xF0);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x1C,0x01);
    jdi_dcs_write_seq_static(ctx,0x33,0x01);
    jdi_dcs_write_seq_static(ctx,0x5A,0x00);

    jdi_dcs_write_seq_static(ctx,0xFF,0xD0);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x53,0x22);
    jdi_dcs_write_seq_static(ctx,0x54,0x02);

    jdi_dcs_write_seq_static(ctx,0xFF,0xC0);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x9C,0x11);
    jdi_dcs_write_seq_static(ctx,0x9D,0x11);

    jdi_dcs_write_seq_static(ctx,0xFF,0x2B);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0xB7,0x21);
    jdi_dcs_write_seq_static(ctx,0xB8,0x12);
    jdi_dcs_write_seq_static(ctx,0xC0,0x01);

    jdi_dcs_write_seq_static(ctx,0xFF,0x10);
    jdi_dcs_write_seq_static(ctx,0xFB,0x01);
    jdi_dcs_write_seq_static(ctx,0x53,0x24);
    jdi_dcs_write_seq_static(ctx,0x35,0x00);
    jdi_dcs_write_seq_static(ctx,0x11,0x00);
    usleep_range(105*1000, 105*1000+100);
    jdi_dcs_write_seq_static(ctx,0x29,0x00);
    usleep_range(45*1000, 45*1000+100);
    pr_info("debug for %s-\n", __func__);
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

    ctx->enabled = false;
    pr_err("debug for %s\n", __func__);
    return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{

    struct jdi *ctx = panel_to_jdi(panel);
    int mode = 0;
    int flag_poweroff = 1;
    pr_err("debug for %s+\n", __func__);

    if (!ctx->prepared) {
        return 0;
    }
    mode = get_boot_mode();
	pr_err("[TP]g_shutdown_flag=%d\n",g_shutdown_flag);
    if ((mode != MSM_BOOT_MODE__FACTORY) && (mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
        if(tp_gesture_enable_notifier && tp_gesture_enable_notifier(0) && (g_shutdown_flag == 0) && (esd_flag == 0)) {
            flag_poweroff = 0;
        } else {
            flag_poweroff = 1;
        }
    }

    jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
    usleep_range(25*1000, 25*1000+100);
    jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
    usleep_range(70*1000, 70*1000+100);

    if (flag_poweroff == 1) {
        ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->reset_gpio, 0);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);
        usleep_range(5*1000, 5*1000+100);
    }
    ctx->error = 0;
    ctx->prepared = false;
    pr_err("debug for %s-\n", __func__);

    return 0;
}

static int jdi_prepare(struct drm_panel *panel)
{
    struct jdi *ctx = panel_to_jdi(panel);
    int ret;

    int mode;
    int blank;
    pr_err("%s +\n", __func__);
    if (ctx->prepared)
        return 0;

    usleep_range(12000, 12100);
    ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
    gpiod_set_value(ctx->reset_gpio, 1);
    usleep_range(5000, 5100);
    gpiod_set_value(ctx->reset_gpio, 0);
    usleep_range(5000, 5100);
    gpiod_set_value(ctx->reset_gpio, 1);
    devm_gpiod_put(ctx->dev, ctx->reset_gpio);
    usleep_range(15000, 15100);


    mode = get_boot_mode();
    pr_info("[TP] in dis_panel_power_on,mode = d%\n", mode);
    if ((mode != MSM_BOOT_MODE__FACTORY) &&(mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
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
    if (ret < 0)
        jdi_unprepare(panel);

    ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
    jdi_panel_get_data(ctx);
#endif

    pr_info("debug for %s-\n", __func__);
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
    pr_err("debug for %s\n", __func__);
    return 0;
}

static const struct drm_display_mode default_mode = {
    .clock = 373644,
    .hdisplay = 1080,
    .hsync_start = 1080 + 81,//HFP
    .hsync_end = 1080 + 81 + 22,//HSA
    .htotal = 1080 +81 + 22 + 70,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 2538,//VFP
    .vsync_end = 2412 + 2538 + 10,//VSA
    .vtotal = 2412 + 2538+ 10 + 10,//VBP
};

static const struct drm_display_mode performance_mode_90hz = {
    .clock = 373494,
    .hdisplay = 1080,
    .hsync_start = 1080 + 81,//HFP
    .hsync_end = 1080 + 81 + 22,//HSA
    .htotal = 1080 +81 + 22 + 70,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 880,//VFP
    .vsync_end = 2412 + 880 + 10,//VSA
    .vtotal = 2412 + 880+ 10 + 10,//VBP
};

static const struct drm_display_mode performance_mode_120hz = {
    .clock = 373794,
    .hdisplay = 1080,
    .hsync_start = 1080 + 81,//HFP
    .hsync_end = 1080 + 81 + 22,//HSA
    .htotal = 1080 +81 + 22 + 70,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 54,//VFP
    .vsync_end = 2412 + 54 + 10,//VSA
    .vtotal = 2412 + 54+ 10 + 10,//VBP
};

static const struct drm_display_mode performance_mode_30hz = {
    .clock = 373719,
    .hdisplay = 1080,
    .hsync_start = 1080 + 81,//HFP
    .hsync_end = 1080 + 81 + 22,//HSA
    .htotal = 1080 +81 + 22 + 70,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 7510,//VFP
    .vsync_end = 2412 + 7510 + 10,//VSA
    .vtotal = 2412 + 7510+ 10 + 10,//VBP
};

static const struct drm_display_mode performance_mode_45hz = {
    .clock = 373607,
    .hdisplay = 1080,
    .hsync_start = 1080 + 81,//HFP
    .hsync_end = 1080 + 81 + 22,//HSA
    .htotal = 1080 +81 + 22 + 70,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 4194,//VFP
    .vsync_end = 2412 + 4194 + 10,//VSA
    .vtotal = 2412 + 4194+ 10 + 10,//VBP
};

static const struct drm_display_mode performance_mode_48hz = {
    .clock = 373614,
    .hdisplay = 1080,
    .hsync_start = 1080 + 81,//HFP
    .hsync_end = 1080 + 81 + 22,//HSA
    .htotal = 1080 +81 + 22 + 70,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 3780,//VFP
    .vsync_end = 2412 + 3780 + 10,//VSA
    .vtotal = 2412 + 3780+ 10 + 10,//VBP
};

static const struct drm_display_mode performance_mode_50hz = {
    .clock = 373644,
    .hdisplay = 1080,
    .hsync_start = 1080 + 81,//HFP
    .hsync_end = 1080 + 81 + 22,//HSA
    .htotal = 1080 +81 + 22 + 70,//HBP
    .vdisplay = 2412,
    .vsync_start = 2412 + 3532,//VFP
    .vsync_end = 2412 + 3532 + 10,//VSA
    .vtotal = 2412 + 3532+ 10 + 10,//VBP
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
    .pll_clk = 550,
    .phy_timcon = {
        .hs_trail = 15,
        .clk_trail = 15,
    },
    .cust_esd_check = 0,
    .esd_check_enable = 1,
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
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
    .round_corner_en = 0,
    .corner_pattern_height = ROUND_CORNER_H_TOP,
    .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    .corner_pattern_tp_size = sizeof(top_rc_pattern),
    .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
    .data_rate = 1100,
    .vendor = "NT36672C_TIANMA_QQtang",
    .manufacture = "QQtang_nt_tianma36672c",
};

static struct mtk_panel_params ext_params_90hz = {
    .pll_clk = 550,
    .phy_timcon = {
        .hs_trail = 15,
        .clk_trail = 15,
    },
    .cust_esd_check = 0,
    .esd_check_enable = 1,
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
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
    .round_corner_en = 0,
    .corner_pattern_height = ROUND_CORNER_H_TOP,
    .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    .corner_pattern_tp_size = sizeof(top_rc_pattern),
    .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
    .data_rate = 1100,
    .vendor = "NT36672C_TIANMA_QQtang",
    .manufacture = "QQtang_nt_tianma36672c",
};

static struct mtk_panel_params ext_params_120hz = {
    .pll_clk = 550,
    .phy_timcon = {
        .hs_trail = 15,
        .clk_trail = 15,
    },
    .cust_esd_check = 0,
    .esd_check_enable = 1,
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
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
    .round_corner_en = 0,
    .corner_pattern_height = ROUND_CORNER_H_TOP,
    .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    .corner_pattern_tp_size = sizeof(top_rc_pattern),
    .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
    .data_rate = 1100,
    .vendor = "NT36672C_TIANMA_QQtang",
    .manufacture = "QQtang_nt_tianma36672c",
};
static struct mtk_panel_params ext_params_30hz = {
    .pll_clk = 550,
    .phy_timcon = {
        .hs_trail = 15,
        .clk_trail = 15,
    },
    .cust_esd_check = 0,
    .esd_check_enable = 1,
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
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
    .round_corner_en = 0,
    .corner_pattern_height = ROUND_CORNER_H_TOP,
    .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    .corner_pattern_tp_size = sizeof(top_rc_pattern),
    .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
    .data_rate = 1100,
    .vendor = "NT36672C_TIANMA_QQtang",
    .manufacture = "QQtang_nt_tianma36672c",
};

static struct mtk_panel_params ext_params_45hz = {
    .pll_clk = 550,
    .phy_timcon = {
        .hs_trail = 15,
        .clk_trail = 15,
    },
    .cust_esd_check = 0,
    .esd_check_enable = 1,
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
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
    .round_corner_en = 0,
    .corner_pattern_height = ROUND_CORNER_H_TOP,
    .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    .corner_pattern_tp_size = sizeof(top_rc_pattern),
    .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
    .data_rate = 1100,
    .vendor = "NT36672C_TIANMA_QQtang",
    .manufacture = "QQtang_nt_tianma36672c",
};

static struct mtk_panel_params ext_params_48hz = {
    .pll_clk = 550,
    .phy_timcon = {
        .hs_trail = 15,
        .clk_trail = 15,
    },
    .cust_esd_check = 0,
    .esd_check_enable = 1,
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
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
    .round_corner_en = 0,
    .corner_pattern_height = ROUND_CORNER_H_TOP,
    .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    .corner_pattern_tp_size = sizeof(top_rc_pattern),
    .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
    .data_rate = 1100,
    .vendor = "NT36672C_TIANMA_QQtang",
    .manufacture = "QQtang_nt_tianma36672c",
};

static struct mtk_panel_params ext_params_50hz = {
    .pll_clk = 550,
    .phy_timcon = {
        .hs_trail = 15,
        .clk_trail = 15,
    },
    .cust_esd_check = 0,
    .esd_check_enable = 1,
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
    .lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
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
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
    .round_corner_en = 0,
    .corner_pattern_height = ROUND_CORNER_H_TOP,
    .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
    .corner_pattern_tp_size = sizeof(top_rc_pattern),
    .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
    .data_rate = 1100,
    .vendor = "NT36672C_TIANMA_QQtang",
    .manufacture = "QQtang_nt_tianma36672c",
};

static void cabc_switch(void *dsi, dcs_write_gce cb,void *handle, unsigned int cabc_mode)
{
    char bl_tb1[] = {0x55, 0x00};
    char bl_tb2[] = {0xFF, 0x10};
    char bl_tb3[] = {0xFB, 0x01};
    char bl_tb4[] = {0x53, 0x2C};
    pr_err("%s cabc = %d\n", __func__, cabc_mode);
    if (cabc_mode > 3)
        return;
    cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
    cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
    if (flag_dimming == 1) {
        cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
    }
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
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    }
    cabc_lastlevel = cabc_mode;
}

static int panel_ata_check(struct drm_panel *panel)
{
    /* Customer test by own ATA tool */
    return 1;
}

static void jdi_gamma_enter(void *dsi, dcs_write_gce cb, void *handle)
{
    char bl_tb1[] = {0xFF, 0x20};
    char bl_tb2[] = {0xFB, 0x01};

    char bl_tb3[] = {0xAE,0x01};

    char bl_tb4[] = {0x95, 0x09};
    char bl_tb5[] = {0x96, 0x09};
    char bl_tb6[] = {0xB0 ,0x00 ,0x00 ,0x00 ,0x05 ,0x00 ,0x0D ,0x00 ,0x15 ,0x00 ,0x1D ,0x00 ,0x24 ,0x00 ,0x2B ,0x00 ,0x31};
    char bl_tb7[] = {0xB1 ,0x00 ,0x38 ,0x00 ,0x4F ,0x00 ,0x65 ,0x00 ,0x8D ,0x00 ,0xB2 ,0x00 ,0xF9 ,0x01 ,0x40 ,0x01 ,0x42};
    char bl_tb8[] = {0xB2 ,0x01 ,0x8D ,0x01 ,0xE8 ,0x02 ,0x2E ,0x02 ,0x7B ,0x02 ,0xB2 ,0x02 ,0xF4 ,0x03 ,0x07 ,0x03 ,0x1B};
    char bl_tb9[] = {0xB3 ,0x03 ,0x31 ,0x03 ,0x4A ,0x03 ,0x69 ,0x03 ,0x90 ,0x03 ,0xB6 ,0x03 ,0xBC ,0x00 ,0x00};

    cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
    cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
    cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
    cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
    cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));

    cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
    cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
    cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
    cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));

}

static void jdi_gamma_exit(void *dsi, dcs_write_gce cb, void *handle)
{
    char bl_tb1[] = {0xFF, 0x20};
    char bl_tb2[] = {0xFB, 0x01};

    char bl_tb3[] = {0xAE,0x01};

    char bl_tb4[] = {0x95, 0xD1};
    char bl_tb5[] = {0x96, 0xD1};
    char bl_tb6[] = {0xB0 ,0x00 ,0x00 ,0x00 ,0x20 ,0x00 ,0x47 ,0x00 ,0x65 ,0x00 ,0x7E ,0x00 ,0x95 ,0x00 ,0xA9 ,0x00 ,0xBB};
    char bl_tb7[] = {0xB1 ,0x00 ,0xCC ,0x01 ,0x06 ,0x01 ,0x2E ,0x01 ,0x70 ,0x01 ,0x9E ,0x01 ,0xE9 ,0x02 ,0x22 ,0x02 ,0x24};
    char bl_tb8[] = {0xB2 ,0x02 ,0x5B ,0x02 ,0x99 ,0x02 ,0xC3 ,0x02 ,0xF7 ,0x03 ,0x1B ,0x03 ,0x43 ,0x03 ,0x52 ,0x03 ,0x5F};
    char bl_tb9[] = {0xB3 ,0x03 ,0x70 ,0x03 ,0x82 ,0x03 ,0x98 ,0x03 ,0xAC ,0x03 ,0xD0 ,0x03 ,0xD8 ,0x00 ,0x00};

    cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
    cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
    cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
    cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
    cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));

    cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
    cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
    cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
    cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));

}

static int jdi_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
                 unsigned int level)
{
    char bl_tb0[] = {0x51, 0x07, 0xFF};
    char bl_tb1[] = {0x55, 0x00};
    char bl_tb2[] = {0xFF, 0x10};
    char bl_tb3[] = {0xFB, 0x01};
    char bl_tb4[] = {0x53, 0x0C};
    char bl_tb5[] = {0x53, 0x24};
    char bl_tb6[] = {0x53, 0x2C};
    char bl_tb7[] = {0x68, 0x01, 0x01};
    char bl_tb8[] = {0x22, 0x00};
    if (flag_i2c == 1) {
        _lcm_i2c_write_bytes(0x03, 0x43);
        flag_i2c = 0;
    }

    if (level > 4095) {
        level = 4095;
    }
    if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
        level = 2047;
    }
    bl_tb0[1] = level >> 8;
    bl_tb0[2] = level & 0xFF;

    if (!cb) {
        return -1;
    }
    if (level < 14 && level > 0  && bl_gamma == 0) {
        bl_gamma = 1;
        jdi_gamma_enter(dsi,cb,handle);
    } else if (level > 13 && bl_gamma == 1) {
        bl_gamma = 0;
        jdi_gamma_exit(dsi,cb,handle);
    } else if (level == 0) {
        bl_gamma = 0;
    }
    cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
    cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
    if (level == 0) {
        flag_dimming = 0;
        if (g_shutdown_flag == 1) {
            cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
        }
        cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
        cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
    } else if (flag_dimming == 1) {
        cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
        flag_dimming = 0;
    }
    if (last_brightness == 0) {
        usleep_range(15*1000, 15*1000+100);
        if (cabc_lastlevel != 0) {
            bl_tb1[1] = cabc_lastlevel;
            cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
            if (cabc_lastlevel == 3) {
                cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
            } else {
                cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
            }
        } else {
            cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
        }
        flag_dimming = 1;
    }

    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

    esd_brightness = level;
    oplus_display_brightness = level;
    last_brightness = level;
    pr_info("debug for %s backlight = %d bl_gamma = %d cabc_lastlevel = %d\n", __func__, level,bl_gamma,cabc_lastlevel);
    return 0;
}

static void lcm_setbrightness(void *dsi,
                  dcs_write_gce cb, void *handle, unsigned int level)
{
    unsigned int BL_MSB = 0;
    unsigned int BL_LSB = 0;
    unsigned int hbm_brightness = 0;
    int i = 0;

    printk("%s level is %d\n", __func__, level);

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

    if (!cb)
        return -1;

    pr_err("oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

    if(hbm_mode == 1) {
        //oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
        for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
            cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
        }
    } else if (hbm_mode == 0) {
        //level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
        lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);  //level
        printk("%s : %d ! backlight %d !\n",__func__, hbm_mode, oplus_display_brightness);
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
    if (!cb)
        return -1;
    if (ctx->hbm_en == en)
        goto done;

    pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

    if(en == 1) {
        for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
            cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
        }
    } else if (en == 0) {
        level = oplus_display_brightness;
        lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
        printk("%s : %d ! backlight %d !\n",__func__, en, oplus_display_brightness);
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
    else
        ctx->hbm_wait = true;
done:
    return 0;
}


/* static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
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
} */
static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
    void *handle)
{
    char bl_tb0[] = {0x51, 0x03, 0xff};

    //pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
    bl_tb0[1] = esd_brightness >> 8;
    bl_tb0[2] = esd_brightness & 0xFF;
     if (!cb)
        return -1;
    pr_err("%s esd_brightness=%x bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, esd_brightness,bl_tb0[1], bl_tb0[2]);
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

extern bool oplus_fp_notify_down_delay;
/* static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
    //struct lcm *ctx = panel_to_lcm(panel);
    unsigned int i=0;
    pr_err("debug for lcm %s, oplus_fp_notify_down_delay=%d\n", __func__, oplus_fp_notify_down_delay);

    //if (oplus_fp_notify_down_delay)
     //   aod_finger_unlock_flag = 1;
    
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

    // if (aod_finger_unlock_flag == 1) {
    //     struct lcm *ctx = panel_to_lcm(panel);
    //     DDPINFO("finger unlock in aod\n");
    //     for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
    //         cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
    //     }
    //     ctx->hbm_en = true;
    //     ctx->hbm_wait = false;
    //     aod_finger_unlock_flag = 0;
    // }

    aod_state = false;

    return 0;
} */
/* static struct LCM_setting_table lcm_normal_to_aod_sam[] = {
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

    // Display on 
    //{REGFLAG_CMD, 1, {0x29}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
}; */

/* static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
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
} */

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

/* static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
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
} */

static int lcm_panel_poweron(struct drm_panel *panel)
{
    struct jdi *ctx = panel_to_jdi(panel);
    int ret;

    if (ctx->prepared)
        return 0;
    usleep_range(5000, 5100);

    if (poweroff_success == 0) {
        pr_err("Display not to poweron before\n");
    } else {
        lcm_panel_vufs_ldo_enable(ctx->dev);
        pr_err("Display  poweron\n");
    }

    usleep_range(5000, 5100);
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
    _lcm_i2c_write_bytes(0x03, 0x43);
    ret = ctx->error;
    if (ret < 0)
        jdi_unprepare(panel);

    usleep_range(5000, 5100);
    pr_err("debug for samsung_amb670 lcm %s\n", __func__);
    return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
    struct jdi *ctx = panel_to_jdi(panel);
    int ret;
    int blank = 0;
    int mode = 0;
    int flag_poweroff = 1;
    if (ctx->prepared) {
        return 0;
    }


    mode = get_boot_mode();
	pr_err("[TP]g_shutdown_flag=%d\n",g_shutdown_flag);
    if ((mode != MSM_BOOT_MODE__FACTORY) && (mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
        if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0) && (g_shutdown_flag == 0) && (esd_flag == 0)) {
            is_pd_with_guesture = true;
            flag_poweroff = 0;
            pr_err("[TP] tp gesture  is enable,Display not to poweroff\n");
            //return 0;
        } else {
            is_pd_with_guesture = false;
            flag_poweroff = 1;
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


    poweroff_success = 0;
    if (flag_poweroff == 1) {
        ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->bias_neg)) {
            dev_err(ctx->dev, "cannot get bias-gpios 1 %ld\n",
            PTR_ERR(ctx->bias_neg));
            return PTR_ERR(ctx->bias_neg);
        }
        gpiod_set_value(ctx->bias_neg, 0);
        devm_gpiod_put(ctx->dev, ctx->bias_neg);

        usleep_range(5*1000, 5*1000+100);

        ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->bias_pos)) {
            dev_err(ctx->dev, "cannot get bias-gpios 0 %ld\n",
                PTR_ERR(ctx->bias_pos));
            return PTR_ERR(ctx->bias_pos);
        }
        gpiod_set_value(ctx->bias_pos, 0);
        devm_gpiod_put(ctx->dev, ctx->bias_pos);

        usleep_range(5*1000, 5*1000+100);
        lcm_panel_vufs_ldo_disable(ctx->dev);
        poweroff_success = 1;
        ret = ctx->error;
        if (ret < 0)
            jdi_unprepare(panel);

        usleep_range(70*1000, 70*1000+100);
        pr_err("debug for 21143tianma lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);
    }
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
    if (m == NULL) {
        pr_err("%s:%d invalid display_mode\n", __func__, __LINE__);
        return -1;
    }
    if (drm_mode_vrefresh(m) == 60)
        ext->params = &ext_params;
    else if (drm_mode_vrefresh(m) == 90)
        ext->params = &ext_params_90hz;
    else if (drm_mode_vrefresh(m) == 120)
        ext->params = &ext_params_120hz;
    else if (drm_mode_vrefresh(m) == 30)
        ext->params = &ext_params_30hz;
    else if (drm_mode_vrefresh(m) == 45)
        ext->params = &ext_params_45hz;
    else if (drm_mode_vrefresh(m) == 48)
        ext->params = &ext_params_48hz;
    else if (drm_mode_vrefresh(m) == 50)
        ext->params = &ext_params_50hz;
    else
        ret = 1;
    //pr_err("debug for ext_params 550 %d %s\n", drm_mode_vrefresh(m),__func__);
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
    .esd_backlight_recovery = oplus_esd_backlight_recovery,
    .hbm_set_cmdq = panel_hbm_set_cmdq,
    .set_hbm = lcm_set_hbm,
    // .hbm_get_state = panel_hbm_get_state,
    // .hbm_set_state = panel_hbm_set_state,
    // .hbm_get_wait_state = panel_hbm_get_wait_state,
    // .hbm_set_wait_state = panel_hbm_set_wait_state,
    // .doze_enable = panel_doze_enable,
    // .doze_disable = panel_doze_disable,
    // .set_aod_light_mode = panel_set_aod_light_mode,
    .cabc_switch = cabc_switch,
};
#endif

static int jdi_get_modes(struct drm_panel *panel,
                    struct drm_connector *connector)
{
    struct drm_display_mode *mode;
    struct drm_display_mode *mode2;
    struct drm_display_mode *mode3;
    struct drm_display_mode *mode4;
    struct drm_display_mode *mode5;
    struct drm_display_mode *mode6;
    struct drm_display_mode *mode7;

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

    mode4 = drm_mode_duplicate(connector->dev, &performance_mode_30hz);
    if (!mode4) {
        dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
             performance_mode_30hz.hdisplay, performance_mode_30hz.vdisplay,
             drm_mode_vrefresh(&performance_mode_30hz));
        return -ENOMEM;
    }

    drm_mode_set_name(mode4);
    mode4->type = DRM_MODE_TYPE_DRIVER;
    drm_mode_probed_add(connector, mode4);

    mode5 = drm_mode_duplicate(connector->dev, &performance_mode_45hz);
    if (!mode5) {
        dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
             performance_mode_45hz.hdisplay, performance_mode_45hz.vdisplay,
             drm_mode_vrefresh(&performance_mode_45hz));
        return -ENOMEM;
    }

    drm_mode_set_name(mode5);
    mode5->type = DRM_MODE_TYPE_DRIVER;
    drm_mode_probed_add(connector, mode5);

    mode6 = drm_mode_duplicate(connector->dev, &performance_mode_48hz);
    if (!mode6) {
        dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
             performance_mode_48hz.hdisplay, performance_mode_48hz.vdisplay,
             drm_mode_vrefresh(&performance_mode_48hz));
        return -ENOMEM;
    }

    drm_mode_set_name(mode6);
    mode6->type = DRM_MODE_TYPE_DRIVER;
    drm_mode_probed_add(connector, mode6);

    mode7 = drm_mode_duplicate(connector->dev, &performance_mode_50hz);
    if (!mode7) {
        dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
             performance_mode_50hz.hdisplay, performance_mode_50hz.vdisplay,
             drm_mode_vrefresh(&performance_mode_50hz));
        return -ENOMEM;
    }

    drm_mode_set_name(mode7);
    mode7->type = DRM_MODE_TYPE_DRIVER;
    drm_mode_probed_add(connector, mode7);

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

    pr_info("debug for oplus21143_tianma_nt36672c_dsi_vdo %s+\n", __func__);

    if (is_probe_finish == true)
        is_probe_double = true;

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
    mipi_dsi_set_drvdata(dsi, ctx);
    ctx->dev = dev;
    dsi->lanes = 4;
    dsi->format = MIPI_DSI_FMT_RGB888;
    dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
             | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;

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
    devm_gpiod_put(ctx->dev, ctx->reset_gpio);
    lcm_panel_vufs_ldo_enable(ctx->dev);
    usleep_range(5000, 5100);

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
    //_lcm_i2c_write_bytes(0x03, 0x43);
    ctx->prepared = true;
    ctx->enabled = true;
    drm_panel_init(&ctx->panel, dev, &jdi_drm_funcs, DRM_MODE_CONNECTOR_DSI);

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
    oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
    register_device_proc("lcd", "NT36672C_TIANMA_QQtang", "QQtang_nt_tianma36672c");
    is_probe_finish = true;

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
    oplus_ofp_set_fp_type(&fp_type);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

    pr_info("debug for %s end lcm,oplus21143_tianma_nt36672c_dsi_vdo\n", __func__);

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
        .compatible = "oplus21143,tianma,nt36672c,vdo",
    },
    {}
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
    .probe = jdi_probe,
    .remove = jdi_remove,
    .driver = {
        .name = "oplus21143_tianma_nt36672c_dsi_vdo",
        .owner = THIS_MODULE,
        .of_match_table = jdi_of_match,
    },
};

static int __init jdi_drv_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&jdi_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register jdi driver: %d\n",
			__func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit jdi_drv_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&jdi_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}
module_init(jdi_drv_init);
module_exit(jdi_drv_exit);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("JDI NT36672E VDO 60HZ AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
