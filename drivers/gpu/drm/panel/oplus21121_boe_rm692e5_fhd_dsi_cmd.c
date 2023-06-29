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

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#include <mt-plat/mtk_boot_common.h>
#include "../mediatek/mediatek_v2/mtk-cmdq-ext.h"

#include "../oplus/oplus_drm_disp_panel.h"
#include "../../../misc/mediatek/lpm/inc/lpm_module.h"
#include "../../../misc/mediatek/lpm/modules/include/mt6895/mt6895_pwr_ctrl.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/data_hw_roundedpattern_r_boe.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/data_hw_roundedpattern_l_boe.h"
#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2
static unsigned int esd_brightness = 1023;
extern unsigned int oplus_display_brightness;
static bool aod_state = false;
extern void lcdinfo_notify(unsigned long val, void *v);
extern unsigned int oplus_enhance_mipi_strength;

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

static struct LCM_setting_table lcm_aod_to_normal[] = {
	{REGFLAG_CMD,2,{0xFE,0x40}},
	{REGFLAG_CMD,2,{0xB3,0x4F}}, //50=FD off 4F=FD on
	{REGFLAG_CMD,2,{0xFE,0x00}},
	{REGFLAG_CMD,1,{0x38}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_high_mode[] = {
	//aod 50nit
	{REGFLAG_CMD,2,{0xFE,0x74}},
	{REGFLAG_CMD,2,{0xEC,0x77}},
	{REGFLAG_CMD,2,{0xED,0x77}},
	{REGFLAG_CMD,2,{0xEF,0xFF}},
	{REGFLAG_CMD,2,{0xF0,0xFF}},
	{REGFLAG_CMD,2,{0xF2,0xFF}},
	{REGFLAG_CMD,2,{0xF3,0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_aod_low_mode[] = {
	//aod 10nit
	{REGFLAG_CMD,2,{0xFE,0x74}},
	{REGFLAG_CMD,2,{0xEC,0x11}},
	{REGFLAG_CMD,2,{0xED,0x11}},
	{REGFLAG_CMD,2,{0xEF,0x98}},
	{REGFLAG_CMD,2,{0xF0,0x98}},
	{REGFLAG_CMD,2,{0xF2,0x98}},
	{REGFLAG_CMD,2,{0xF3,0x98}},
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
	struct gpio_desc *vddr_aod_enable_gpio;
	struct gpio_desc *te_switch_gpio,*te_out_gpio;
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
static struct regulator_dev *rdev_vufs_origin;
static int lcm_panel_vufs_ldo_regulator_init(struct device *dev)
{
        static int regulator_vufs_inited;
        int ret = 0;

        if (regulator_vufs_inited)
                return ret;
    pr_err("get lcm_panel_vufs_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        vufs_ldo = devm_regulator_get(dev, "vufsldo");
        if (IS_ERR_OR_NULL(vufs_ldo)) { /* handle return value */
                ret = PTR_ERR(vufs_ldo);
                pr_err("get vufs_ldo fail, error: %d\n", ret);
                //return ret;
        } else {
            rdev_vufs_origin = vufs_ldo->rdev;
            if (!IS_ERR_OR_NULL(rdev_vufs_origin)) {
                pr_info("%s-%d rdev_vufs_origin inited: %p\n", __func__, __LINE__,rdev_vufs_origin);
            }
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
        if (IS_ERR_OR_NULL(vufs_ldo->rdev) || (vufs_ldo->rdev != rdev_vufs_origin)) {
            pr_err("%s-%d vufs_ldo->rdev is invalid\n", __func__, __LINE__);
            if (!IS_ERR_OR_NULL(rdev_vufs_origin)) {
                vufs_ldo->rdev = rdev_vufs_origin;
                pr_err("%s-%d vufs_ldo->rdev restore: %p\n", __func__, __LINE__,vufs_ldo->rdev);
            }
        }
		ret = regulator_set_voltage(vufs_ldo, 1800000, 1800000);
		if (ret < 0)
			pr_err("set voltage vufs_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vufs_ldo)) {
        if (IS_ERR_OR_NULL(vufs_ldo->rdev) || (vufs_ldo->rdev != rdev_vufs_origin)) {
            pr_err("%s-%d vufs_ldo->rdev is invalid\n", __func__, __LINE__);
            if (!IS_ERR_OR_NULL(rdev_vufs_origin)) {
                vufs_ldo->rdev = rdev_vufs_origin;
                pr_err("%s-%d vufs_ldo->rdev restore: %p\n", __func__, __LINE__,vufs_ldo->rdev);
            }
        }
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
        if (IS_ERR_OR_NULL(vufs_ldo->rdev) || (vufs_ldo->rdev != rdev_vufs_origin)) {
            pr_err("%s-%d vufs_ldo->rdev is invalid\n", __func__, __LINE__);
            if (!IS_ERR_OR_NULL(rdev_vufs_origin)) {
                vufs_ldo->rdev = rdev_vufs_origin;
                pr_err("%s-%d vufs_ldo->rdev restore: %p\n", __func__, __LINE__,vufs_ldo->rdev);
            }
        }
		ret = regulator_disable(vufs_ldo);
		if (ret < 0)
			pr_err("disable regulator vufs_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	return ret;
}

static struct regulator *vmch_ldo;
static struct regulator_dev *rdev_vmch_origin;
static int lcm_panel_vmch_ldo_regulator_init(struct device *dev)
{
        static int regulator_vmch_inited;
        int ret = 0;

        if (regulator_vmch_inited)
                return ret;
    pr_err("get lcm_panel_vmch_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        vmch_ldo = devm_regulator_get(dev, "vmchldo");
        if (IS_ERR_OR_NULL(vmch_ldo)) { /* handle return value */
                ret = PTR_ERR(vmch_ldo);
                pr_err("get vmch_ldo fail, error: %d\n", ret);
                //return ret;
        } else {
            rdev_vmch_origin = vmch_ldo->rdev;
            if (!IS_ERR_OR_NULL(rdev_vmch_origin)) {
                pr_info("%s-%d rdev_vmch_origin inited: %p\n", __func__, __LINE__,rdev_vmch_origin);
            }
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
        if (IS_ERR_OR_NULL(vmch_ldo->rdev) || (vmch_ldo->rdev != rdev_vmch_origin)) {
            pr_err("%s-%d vmch_ldo->rdev is invalid\n", __func__, __LINE__);
            if (!IS_ERR_OR_NULL(rdev_vmch_origin)) {
                vmch_ldo->rdev = rdev_vmch_origin;
                pr_err("%s-%d vmch_ldo->rdev restore: %p\n", __func__, __LINE__,vmch_ldo->rdev);
            }
        }
		ret = regulator_set_voltage(vmch_ldo, 3000000, 3000000);
		if (ret < 0)
			pr_err("set voltage vmch_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vmch_ldo)) {
        if (IS_ERR_OR_NULL(vmch_ldo->rdev) || (vmch_ldo->rdev != rdev_vmch_origin)) {
            pr_err("%s-%d vmch_ldo->rdev is invalid\n", __func__, __LINE__);
            if (!IS_ERR_OR_NULL(rdev_vmch_origin)) {
                vmch_ldo->rdev = rdev_vmch_origin;
                pr_err("%s-%d vmch_ldo->rdev restore: %p\n", __func__, __LINE__,vmch_ldo->rdev);
            }
        }
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
        if (IS_ERR_OR_NULL(vmch_ldo->rdev) || (vmch_ldo->rdev != rdev_vmch_origin)) {
            pr_err("%s-%d vmch_ldo->rdev is invalid\n", __func__, __LINE__);
            if (!IS_ERR_OR_NULL(rdev_vmch_origin)) {
                vmch_ldo->rdev = rdev_vmch_origin;
                pr_err("%s-%d vmch_ldo->rdev restore: %p\n", __func__, __LINE__,vmch_ldo->rdev);
            }
        }
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
    //dither on
    jdi_dcs_write_seq_static(ctx,0xFE,0x72);
    jdi_dcs_write_seq_static(ctx,0x42,0x31);
    jdi_dcs_write_seq_static(ctx,0xFE,0x16);
    jdi_dcs_write_seq_static(ctx,0x8A,0x00);
    //60HZ Demura
    jdi_dcs_write_seq_static(ctx,0xFE,0xF4);
    jdi_dcs_write_seq_static(ctx,0x00,0xFF);
    jdi_dcs_write_seq_static(ctx,0x01,0xB3);
    jdi_dcs_write_seq_static(ctx,0x02,0xCC);
    jdi_dcs_write_seq_static(ctx,0x03,0x29);
    jdi_dcs_write_seq_static(ctx,0x04,0x74);
    jdi_dcs_write_seq_static(ctx,0x05,0x72);
    jdi_dcs_write_seq_static(ctx,0x06,0xDD);
    jdi_dcs_write_seq_static(ctx,0x07,0x25);
    jdi_dcs_write_seq_static(ctx,0x08,0x18);
    jdi_dcs_write_seq_static(ctx,0x09,0x61);
    jdi_dcs_write_seq_static(ctx,0x0A,0x35);
    jdi_dcs_write_seq_static(ctx,0x0B,0x01);
    jdi_dcs_write_seq_static(ctx,0x0C,0xF4);
    jdi_dcs_write_seq_static(ctx,0x0D,0x0C);
    //90HZ Demura
    jdi_dcs_write_seq_static(ctx,0xFE,0xF4);
    jdi_dcs_write_seq_static(ctx,0x1B,0xF0);
    jdi_dcs_write_seq_static(ctx,0x1C,0x7F);
    jdi_dcs_write_seq_static(ctx,0x1D,0xDE);
    jdi_dcs_write_seq_static(ctx,0x1E,0x13);
    jdi_dcs_write_seq_static(ctx,0x1F,0x13);
    jdi_dcs_write_seq_static(ctx,0x20,0x2A);
    jdi_dcs_write_seq_static(ctx,0x21,0xD7);
    jdi_dcs_write_seq_static(ctx,0x22,0x5D);
    jdi_dcs_write_seq_static(ctx,0x23,0x82);
    jdi_dcs_write_seq_static(ctx,0x24,0x11);
    jdi_dcs_write_seq_static(ctx,0x25,0x56);
    jdi_dcs_write_seq_static(ctx,0x26,0x13);
    jdi_dcs_write_seq_static(ctx,0x27,0x40);
    jdi_dcs_write_seq_static(ctx,0x28,0xCF);
    jdi_dcs_write_seq_static(ctx,0x29,0x00);
    //120HZ Demura
    jdi_dcs_write_seq_static(ctx,0xFE,0xF4);
    jdi_dcs_write_seq_static(ctx,0x0D,0xC0);
    jdi_dcs_write_seq_static(ctx,0x0E,0xFF);
    jdi_dcs_write_seq_static(ctx,0x0F,0xF9);
    jdi_dcs_write_seq_static(ctx,0x10,0x0B);
    jdi_dcs_write_seq_static(ctx,0x11,0x5E);
    jdi_dcs_write_seq_static(ctx,0x12,0xB2);
    jdi_dcs_write_seq_static(ctx,0x13,0x5C);
    jdi_dcs_write_seq_static(ctx,0x14,0x77);
    jdi_dcs_write_seq_static(ctx,0x15,0x09);
    jdi_dcs_write_seq_static(ctx,0x16,0x46);
    jdi_dcs_write_seq_static(ctx,0x17,0x58);
    jdi_dcs_write_seq_static(ctx,0x18,0x4D);
    jdi_dcs_write_seq_static(ctx,0x19,0x00);
    jdi_dcs_write_seq_static(ctx,0x1A,0x3D);
    jdi_dcs_write_seq_static(ctx,0x1B,0x03);

    //pps setting 540*12 10BIT VESA1.1 3x
    jdi_dcs_write_seq_static(ctx,0xFE,0xD2);
    jdi_dcs_write_seq_static(ctx,0x50,0x11);
    jdi_dcs_write_seq_static(ctx,0x51,0xab);
    jdi_dcs_write_seq_static(ctx,0x52,0x30);
    jdi_dcs_write_seq_static(ctx,0x53,0x09);
    jdi_dcs_write_seq_static(ctx,0x54,0x6c);
    jdi_dcs_write_seq_static(ctx,0x55,0x04);
    jdi_dcs_write_seq_static(ctx,0x56,0x38);
    jdi_dcs_write_seq_static(ctx,0x58,0x00);
    jdi_dcs_write_seq_static(ctx,0x59,0x0c);
    jdi_dcs_write_seq_static(ctx,0x5a,0x02);
    jdi_dcs_write_seq_static(ctx,0x5b,0x1c);
    jdi_dcs_write_seq_static(ctx,0x5c,0x01);
    jdi_dcs_write_seq_static(ctx,0x5d,0x9a);
    jdi_dcs_write_seq_static(ctx,0x5e,0x19);
    jdi_dcs_write_seq_static(ctx,0x5f,0x01);
    jdi_dcs_write_seq_static(ctx,0x60,0x03);
    jdi_dcs_write_seq_static(ctx,0x61,0x00);
    jdi_dcs_write_seq_static(ctx,0x62,0x0a);
    jdi_dcs_write_seq_static(ctx,0x63,0x0c);
    jdi_dcs_write_seq_static(ctx,0x64,0x08);
    jdi_dcs_write_seq_static(ctx,0x65,0xbb);
    jdi_dcs_write_seq_static(ctx,0x66,0x0a);
    jdi_dcs_write_seq_static(ctx,0x67,0x5f);
    jdi_dcs_write_seq_static(ctx,0x68,0x16);
    jdi_dcs_write_seq_static(ctx,0x69,0x00);
    jdi_dcs_write_seq_static(ctx,0x6a,0x10);
    jdi_dcs_write_seq_static(ctx,0x6b,0xec);
    jdi_dcs_write_seq_static(ctx,0x6c,0x07);
    jdi_dcs_write_seq_static(ctx,0x6d,0x10);
    jdi_dcs_write_seq_static(ctx,0x6e,0x20);
    jdi_dcs_write_seq_static(ctx,0x6f,0x00);
    jdi_dcs_write_seq_static(ctx,0x70,0x06);
    jdi_dcs_write_seq_static(ctx,0x71,0x0f);
    jdi_dcs_write_seq_static(ctx,0x72,0x0f);
    jdi_dcs_write_seq_static(ctx,0x73,0x33);
    jdi_dcs_write_seq_static(ctx,0x74,0x0e);
    jdi_dcs_write_seq_static(ctx,0x75,0x1c);
    jdi_dcs_write_seq_static(ctx,0x76,0x2a);
    jdi_dcs_write_seq_static(ctx,0x77,0x38);
    jdi_dcs_write_seq_static(ctx,0x78,0x46);
    jdi_dcs_write_seq_static(ctx,0x79,0x54);
    jdi_dcs_write_seq_static(ctx,0x7a,0x62);
    jdi_dcs_write_seq_static(ctx,0x7b,0x69);
    jdi_dcs_write_seq_static(ctx,0x7c,0x70);
    jdi_dcs_write_seq_static(ctx,0x7d,0x77);
    jdi_dcs_write_seq_static(ctx,0x7e,0x79);
    jdi_dcs_write_seq_static(ctx,0x7f,0x7b);
    jdi_dcs_write_seq_static(ctx,0x80,0x7d);
    jdi_dcs_write_seq_static(ctx,0x81,0x7e);
    jdi_dcs_write_seq_static(ctx,0x82,0x01);
    jdi_dcs_write_seq_static(ctx,0x83,0xc2);
    jdi_dcs_write_seq_static(ctx,0x84,0x22);
    jdi_dcs_write_seq_static(ctx,0x85,0x00);
    jdi_dcs_write_seq_static(ctx,0x86,0x2a);
    jdi_dcs_write_seq_static(ctx,0x87,0x40);
    jdi_dcs_write_seq_static(ctx,0x88,0x32);
    jdi_dcs_write_seq_static(ctx,0x89,0xbe);
    jdi_dcs_write_seq_static(ctx,0x8a,0x3a);
    jdi_dcs_write_seq_static(ctx,0x8b,0xfc);
    jdi_dcs_write_seq_static(ctx,0x8c,0x3a);
    jdi_dcs_write_seq_static(ctx,0x8d,0xfa);
    jdi_dcs_write_seq_static(ctx,0x8e,0x3a);
    jdi_dcs_write_seq_static(ctx,0x8f,0xf8);
    jdi_dcs_write_seq_static(ctx,0x90,0x3b);
    jdi_dcs_write_seq_static(ctx,0x91,0x38);
    jdi_dcs_write_seq_static(ctx,0x92,0x3b);
    jdi_dcs_write_seq_static(ctx,0x93,0x78);
    jdi_dcs_write_seq_static(ctx,0x94,0x3b);
    jdi_dcs_write_seq_static(ctx,0x95,0x76);
    jdi_dcs_write_seq_static(ctx,0x96,0x4b);
    jdi_dcs_write_seq_static(ctx,0x97,0xb6);
    jdi_dcs_write_seq_static(ctx,0x98,0x4b);
    jdi_dcs_write_seq_static(ctx,0x99,0xB6);
    jdi_dcs_write_seq_static(ctx,0x9a,0x4B);
    jdi_dcs_write_seq_static(ctx,0x9b,0xF4);
    jdi_dcs_write_seq_static(ctx,0x9c,0x5B);
    jdi_dcs_write_seq_static(ctx,0x9d,0xF4);
    jdi_dcs_write_seq_static(ctx,0x9e,0x7C);
    jdi_dcs_write_seq_static(ctx,0x9f,0x34);
    jdi_dcs_write_seq_static(ctx,0xa2,0x02);
    jdi_dcs_write_seq_static(ctx,0xa3,0xa3);
    jdi_dcs_write_seq_static(ctx,0xa4,0x00);
    jdi_dcs_write_seq_static(ctx,0xa5,0x00);
    jdi_dcs_write_seq_static(ctx,0xa6,0x00);
    jdi_dcs_write_seq_static(ctx,0xa7,0x00);
    jdi_dcs_write_seq_static(ctx,0xa9,0x00);
    jdi_dcs_write_seq_static(ctx,0xaa,0x00);
    jdi_dcs_write_seq_static(ctx,0xa0,0xa0);

    //ESD recover code
    jdi_dcs_write_seq_static(ctx,0xFE,0xD4);  //Page D4
    jdi_dcs_write_seq_static(ctx,0x40,0x03);  // lvd/mipi error flag enable
    jdi_dcs_write_seq_static(ctx,0xFE,0xFD);  //Page FD
    jdi_dcs_write_seq_static(ctx,0x80,0x06);  //error report to 0A00 enable
    jdi_dcs_write_seq_static(ctx,0x83,0x00);  //command 1 OFF  00 Page
    jdi_dcs_write_seq_static(ctx,0xFE,0xA0);  //Page A0
    jdi_dcs_write_seq_static(ctx,0x06,0x36);  //lvd function VCI/VDDI/AVDD
    jdi_dcs_write_seq_static(ctx,0xFE,0xA1);  //Page A1
    jdi_dcs_write_seq_static(ctx,0xB3,0x7F);  //hs command OFF
    jdi_dcs_write_seq_static(ctx,0xFE,0xA1);
    jdi_dcs_write_seq_static(ctx,0xC3,0xC3);
    jdi_dcs_write_seq_static(ctx,0xC4,0x9C);
    jdi_dcs_write_seq_static(ctx,0xC5,0x1E);
    jdi_dcs_write_seq_static(ctx,0xC6,0x23);

    //common setting
    jdi_dcs_write_seq_static(ctx,0xFE,0x38);
    jdi_dcs_write_seq_static(ctx,0x18,0x13);

    jdi_dcs_write_seq_static(ctx,0xFE,0x40);
    jdi_dcs_write_seq_static(ctx,0xBD,0x00);

    jdi_dcs_write_seq_static(ctx,0xFE,0xD0);
    jdi_dcs_write_seq_static(ctx,0x86,0x14);
    jdi_dcs_write_seq_static(ctx,0xFE,0xD4);
    jdi_dcs_write_seq_static(ctx,0x14,0x03);
    jdi_dcs_write_seq_static(ctx,0x1E,0x00);
    jdi_dcs_write_seq_static(ctx,0x1F,0x03);
    jdi_dcs_write_seq_static(ctx,0x20,0x05);
    jdi_dcs_write_seq_static(ctx,0x21,0x03);
    jdi_dcs_write_seq_static(ctx,0x22,0x50);
    jdi_dcs_write_seq_static(ctx,0x23,0x03);
    jdi_dcs_write_seq_static(ctx,0x24,0xE8);
    jdi_dcs_write_seq_static(ctx,0xFE,0xA1);
    jdi_dcs_write_seq_static(ctx,0xCA,0x80);
    /* black frame config */
    jdi_dcs_write_seq_static(ctx,0xFE,0x40);
    jdi_dcs_write_seq_static(ctx,0x84,0x22);

    jdi_dcs_write_seq_static(ctx,0xFE,0x00);
    jdi_dcs_write_seq_static(ctx,0xFA,0x01);  //01-VESA ON  07-VESA OFF

    jdi_dcs_write_seq_static(ctx,0xC2,0x08);
    jdi_dcs_write_seq_static(ctx,0x35,0x00);
    //jdi_dcs_write_seq_static(ctx,0x51,0x0D,0xBB);

    jdi_dcs_write_seq_static(ctx,0x11);
    msleep(100);
    jdi_dcs_write_seq_static(ctx,0x29);
    //msleep(20);
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

	ctx->enabled = false;

	return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{

	struct jdi *ctx = panel_to_jdi(panel);
	pr_err("[lh]%s+\n", __func__);

	if (!ctx->prepared)
		return 0;

	msleep(5);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(25);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(100);
	/* keep vcore off */
	lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
	pr_info("%s, call lpm_smc_spm_dbg keep vcore off for display off!\n", __func__);

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

	pr_err("[lh]%s +\n", __func__);
	if (ctx->prepared)
		return 0;

/*	lcm_panel_vufs_ldo_enable(ctx->dev);
	msleep(10);
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);

	msleep(10);

	lcm_panel_vmc_ldo_enable(ctx->dev);
*/
	// lcd reset H -> L -> H
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(5);
	gpiod_set_value(ctx->reset_gpio, 1);
	//msleep(10);
	//devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(50);
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
	.pll_clk = 415,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 14,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0B, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x00, //no need check value
        },
	.lcm_esd_check_table[1] = {
                .cmd = 0xFA, .count = 1, .para_list[0] = 0x01, .mask_list[0] = 0x01,
        },
	.lcm_esd_check_table[2] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
        },
	.oplus_esd_val_compare_skip = 1,
	.oplus_serial_para0 = 0xA1,
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.vendor = "boe_rm692e5",
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
        .xmit_delay = 410,
        .dec_delay = 472,
        .scale_value = 25,
        .increment_interval = 259,
        .decrement_interval = 10,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2655,
        .initial_offset = 5632,
        .final_offset = 4332,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,

		},
	.data_rate = 830,
	.oplus_ofp_aod_off_insert_black = 2,
	.oplus_ofp_aod_off_black_frame_total_time = 84,
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 0,
	.oplus_ofp_hbm_off_delay = 0,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 60,
	},
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 415,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 14,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0B, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x00, //no need check vaule
        },
	.lcm_esd_check_table[1] = {
                .cmd = 0xFA, .count = 1, .para_list[0] = 0x01, .mask_list[0] = 0x01,
        },
	.lcm_esd_check_table[2] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
        },
	.oplus_esd_val_compare_skip = 1,
	.oplus_serial_para0 = 0xA1,
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.vendor = "boe_rm692e5",
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
        .xmit_delay = 410,
        .dec_delay = 472,
        .scale_value = 25,
        .increment_interval = 259,
        .decrement_interval = 10,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2655,
        .initial_offset = 5632,
        .final_offset = 4332,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,

		},
	.data_rate = 830,
	.oplus_ofp_aod_off_insert_black = 2,
	.oplus_ofp_aod_off_black_frame_total_time = 78,
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 0,
	.oplus_ofp_hbm_off_delay = 0,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
	},
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 485,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 14,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0B, .count = 1, .para_list[0] = 0x00, .mask_list[0] = 0x00, //no need check vaule
        },
	.lcm_esd_check_table[1] = {
                .cmd = 0xFA, .count = 1, .para_list[0] = 0x01, .mask_list[0] = 0x01,
        },
	.lcm_esd_check_table[2] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
        },
	.oplus_esd_val_compare_skip = 1,
	.oplus_serial_para0 = 0xA1,
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 105,
	.vendor = "boe_rm692e5",
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
        .xmit_delay = 410,
        .dec_delay = 472,
        .scale_value = 25,
        .increment_interval = 259,
        .decrement_interval = 10,
        .line_bpg_offset = 12,
        .nfl_bpg_offset = 2235,
        .slice_bpg_offset = 2655,
        .initial_offset = 5632,
        .final_offset = 4332,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
		},
	.data_rate = 970,
	.oplus_ofp_aod_off_insert_black = 2,
	.oplus_ofp_aod_off_black_frame_total_time = 75,
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 9,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 9,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
	},
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int jdi_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	unsigned int mapped_level = 0;
	char bl_tb0[] = {0x51, 0x07, 0xFF};
//	char bl_tb1[] = {0xB2, 0x01};
//	char bl_tb2[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0)
		level = 2047;
	pr_info("%s backlight = -%d\n", __func__, level);
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	mapped_level = level;

	if (!cb)
		return -1;

	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
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
	int i = 0;

	printk("[lh] %s level is %d\n", __func__, level);
		BL_LSB = level >> 8;
		BL_MSB = level & 0xFF;

		lcm_setbrightness_normal[0].para_list[1] = BL_LSB;
		lcm_setbrightness_normal[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_normal)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_normal[i].para_list, lcm_setbrightness_normal[i].count);
		}
}

static int oplus_send_cmd_before_dsi_read(struct drm_panel *panel,void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0xFE, 0xD2};
	char bl_tb1[] = {0x0C, 0x60};
	char bl_tb2[] = {0xFE, 0x88};
	char bl_tb3[] = {0x19, 0x81};
	char bl_tb4[] = {0x18, 0x01};
	char bl_tb5[] = {0x1B, 0x0F};  //0x1B,0x0F ,Read NUM=16: 0x00~0x0E
	char bl_tb6[] = {0x00, 0xA1};
	char bl_tb7[] = {0x02, 0x00};
	char bl_tb8[] = {0x03, 0xC2};
	char bl_tb9[] = {0xFE, 0x00};

	pr_err("oplus_send_cmd_before_dsi_read");
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
	cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));

	return 0;
}

static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;

	if (!cb)
		return -1;

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
	int i = 0;
	int level = 0;
	if (!cb)
		return -1;

	pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		level = oplus_display_brightness;
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		printk("[soso] %s : %d ! backlight %d !\n",__func__, en, oplus_display_brightness);
	}
	lcdinfo_notify(1, &en);
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

	gpiod_set_value(ctx->reset_gpio, on);
	//devm_gpiod_put(ctx->dev, ctx->reset_gpio);

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
	pr_err("debug for lcm %s\n", __func__);

	/*if (oplus_fp_notify_down_delay)
		aod_finger_unlock_flag = 1;*/

	/* Switch back to VDO mode */
	for (i = 0; i < (sizeof(lcm_aod_to_normal) / sizeof(struct LCM_setting_table)); i++) {
		unsigned cmd;
		cmd = lcm_aod_to_normal[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(lcm_aod_to_normal[i].count * 1000, lcm_aod_to_normal[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(lcm_aod_to_normal[i].count * 1000), CMDQ_GPR_R14);
				}
				break;

			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(lcm_aod_to_normal[i].count, lcm_aod_to_normal[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(lcm_aod_to_normal[i].count), CMDQ_GPR_R14);
				}
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
	/* keep vcore off */
	lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
	pr_info("%s, call lpm_smc_spm_dbg keep vcore off for exit AOD!\n", __func__);

	return 0;
}

static struct LCM_setting_table lcm_normal_to_aod_sam[] = {
    //common setting
    {REGFLAG_CMD,2,{0xFE,0x38}},
    {REGFLAG_CMD,2,{0x18,0x13}},

    {REGFLAG_CMD,2,{0xFE,0x40}},
    {REGFLAG_CMD,2,{0xBD,0x00}},

    {REGFLAG_CMD,2,{0xFE,0x00}},
    {REGFLAG_CMD,2,{0xFA,0x01}},

    {REGFLAG_CMD,2,{0xC2,0x08}},
    {REGFLAG_CMD,2,{0x35,0x00}},

    //enter aod 50nit
    //{REGFLAG_CMD,2,{0xFE,0x74}},
    //{REGFLAG_CMD,2,{0xEC,0x77}},
    //{REGFLAG_CMD,2,{0xED,0x77}},
    //{REGFLAG_CMD,2,{0xEF,0xFF}},
    //{REGFLAG_CMD,2,{0xF0,0xFF}},
    //{REGFLAG_CMD,2,{0xF2,0xFF}},
    //{REGFLAG_CMD,2,{0xF3,0xFF}},
    {REGFLAG_CMD,2,{0xFE,0x40}},
    {REGFLAG_CMD,2,{0xB3,0x50}},      //50=FD off 4F=FD on
    {REGFLAG_CMD,2,{0xFE,0x00}},
    {REGFLAG_CMD,1,{0x39}},           // Idle mode on
    {REGFLAG_CMD,3,{0x51,0x0F,0xFE}}, //DBV

    {REGFLAG_CMD, 1, {0x11}},
    {REGFLAG_DELAY,120,{}},
    {REGFLAG_CMD, 1, {0x29}},

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
	/* keep vcore on */
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

	pr_err("[lh]debug for rm692e5 lcm %s, ctx->prepared %d\n", __func__, ctx->prepared);

	if (ctx->prepared)
		return 0;

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
	usleep_range(5000, 5001);

	lcm_panel_vufs_ldo_enable(ctx->dev);
	msleep(5);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	//devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->vddr_aod_enable_gpio, 1);
	//devm_gpiod_put(ctx->dev, ctx->vddr_aod_enable_gpio);
	usleep_range(5000, 5100);
	//set vddi 3.0v
	lcm_panel_vmch_ldo_enable(ctx->dev);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	usleep_range(5000, 5100);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;

	pr_err("[lh]debug for rm692e5 lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);

	if (ctx->prepared)
		return 0;

	gpiod_set_value(ctx->reset_gpio, 0);
	//devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(5);
	//lcm_panel_ldo3_disable(ctx->dev);
	//set vddi 3.0v
        lcm_panel_vmch_ldo_disable(ctx->dev);
        usleep_range(5000, 5100);
	//enable 1.5V
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	//devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->vddr_aod_enable_gpio, 0);
	//devm_gpiod_put(ctx->dev, ctx->vddr_aod_enable_gpio);
	usleep_range(5000, 5100);
	//set vddi 3.0v
	lcm_panel_vufs_ldo_disable(ctx->dev);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

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

static void mode_switch_to_120(struct drm_panel *panel, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (stage == AFTER_DSI_POWERON) {
		pr_info("%s\n", __func__);
		jdi_dcs_write_seq_static(ctx,0xFE,0xD4);
		jdi_dcs_write_seq_static(ctx,0x14,0x03);
		jdi_dcs_write_seq_static(ctx,0x25,0x00);
		jdi_dcs_write_seq_static(ctx,0x26,0x03);
		jdi_dcs_write_seq_static(ctx,0x27,0x05);
		jdi_dcs_write_seq_static(ctx,0x28,0x03);
		jdi_dcs_write_seq_static(ctx,0x29,0xDF);
		jdi_dcs_write_seq_static(ctx,0x2A,0x03);
		jdi_dcs_write_seq_static(ctx,0x2B,0xE8);
		jdi_dcs_write_seq_static(ctx,0xFE,0x40); //PAGE 40
		jdi_dcs_write_seq_static(ctx,0xBD,0x05);
		jdi_dcs_write_seq_static(ctx,0xFE,0x00); //PAGE 00
	}
}

static void mode_switch_to_90(struct drm_panel *panel, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		pr_info("%s\n", __func__);
		jdi_dcs_write_seq_static(ctx,0xFE,0xD4);
		jdi_dcs_write_seq_static(ctx,0x14,0x03);
		jdi_dcs_write_seq_static(ctx,0x2D,0x00);
		jdi_dcs_write_seq_static(ctx,0x2F,0x03);
		jdi_dcs_write_seq_static(ctx,0x30,0x05);
		jdi_dcs_write_seq_static(ctx,0x31,0x03);
		jdi_dcs_write_seq_static(ctx,0x32,0x50);
		jdi_dcs_write_seq_static(ctx,0x33,0x03);
		jdi_dcs_write_seq_static(ctx,0x34,0xE8);
		jdi_dcs_write_seq_static(ctx,0xFE,0x40);
		jdi_dcs_write_seq_static(ctx,0xBD,0x06);
		jdi_dcs_write_seq_static(ctx,0xFE,0x00);
	}
}

static void mode_switch_to_60(struct drm_panel *panel, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		pr_info("%s\n", __func__);
		jdi_dcs_write_seq_static(ctx,0xFE,0xD4);
		jdi_dcs_write_seq_static(ctx,0x14,0x03);
		jdi_dcs_write_seq_static(ctx,0x1E,0x00);
		jdi_dcs_write_seq_static(ctx,0x1F,0x03);
		jdi_dcs_write_seq_static(ctx,0x20,0x05);
		jdi_dcs_write_seq_static(ctx,0x21,0x03);
		jdi_dcs_write_seq_static(ctx,0x22,0x50);
		jdi_dcs_write_seq_static(ctx,0x23,0x03);
		jdi_dcs_write_seq_static(ctx,0x24,0xE8);
		jdi_dcs_write_seq_static(ctx,0xFE,0x40);
		jdi_dcs_write_seq_static(ctx,0xBD,0x00);
		jdi_dcs_write_seq_static(ctx,0xFE,0x00);
	}
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, dst_mode);

	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);

	if (drm_mode_vrefresh(m) == 60) { /* 60 switch to 120 */
		mode_switch_to_60(panel, stage);
	} else if (drm_mode_vrefresh(m) == 90) { /* 1200 switch to 60 */
		mode_switch_to_90(panel, stage);
	} else if (drm_mode_vrefresh(m) == 120) { /* 1200 switch to 60 */
		mode_switch_to_120(panel, stage);
	} else
		ret = 1;

	return ret;
}

static char esd_check_pre_cmd0[] = {0xFF, 0xAA, 0x55, 0xA5, 0x81};
static char esd_check_pre_cmd1[] = {0x6F, 0x0F};

static int lcm_esd_check_precondition(void *dsi, dcs_write_gce cb, void *handle)
{
	cb(dsi, handle, esd_check_pre_cmd0, ARRAY_SIZE(esd_check_pre_cmd0));
	cb(dsi, handle, esd_check_pre_cmd1, ARRAY_SIZE(esd_check_pre_cmd1));

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
	.send_cmd_before_dsi_read = oplus_send_cmd_before_dsi_read,
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
	unsigned int fp_type = 0x08;
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	pr_info("[boe] rm692e5 %s+\n", __func__);

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
	//devm_gpiod_put(dev, ctx->reset_gpio);
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
	ctx->vddr_aod_enable_gpio = devm_gpiod_get(dev, "vddr-aod-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr_aod_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddr_aod_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr_aod_enable_gpio));
		return PTR_ERR(ctx->vddr_aod_enable_gpio);
	}
	//devm_gpiod_put(dev, ctx->vddr1p5_enable_gpio);

	usleep_range(5000, 5100);

    lcm_panel_vmch_ldo_enable(ctx->dev);

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
	register_device_proc("lcd", "boe_rm692e5", "boe");
	oplus_enhance_mipi_strength = 1;

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	oplus_ofp_set_fp_type(&fp_type);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	pr_info("%s- lcm,rm692e5,vdo,60hz\n", __func__);
	flag_silky_panel = FRAME_SYNC_DELAY_60HZ_120HZ;
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
	    .compatible = "oplus21121,boe,rm692e5,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
		.name = "oplus21121_boe_rm692e5_fhd_dsi_cmd",
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
