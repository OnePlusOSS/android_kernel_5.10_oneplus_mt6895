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

#include "../oplus/oplus_display_panel_power.h"
#include "../oplus/oplus_drm_disp_panel.h"
//#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "../oplus/oplus_adfr_ext.h"
//#endif
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
#include "../oplus/oplus_display_temp_compensation.h"
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#include <mt-plat/mtk_boot_common.h>
#include "../mediatek/mediatek_v2/mtk-cmdq-ext.h"
#include "../oplus/oplus_drm_disp_panel.h"
#include "../../../misc/mediatek/lpm/inc/lpm_module.h"
#include "../../../misc/mediatek/lpm/modules/include/mt6895/mt6895_pwr_ctrl.h"
#include "../../../misc/mediatek/wl2868c/wl2868c.h"
#include "AC160_P_7_A0001_cmd_panel.h"

#include "../mediatek/mediatek_v2/mtk_corner_pattern/ac160_p_7_a0001_data_hw_roundedpattern_l_tm.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/ac160_p_7_a0001_data_hw_roundedpattern_r_tm.h"

/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#include "../oplus/oplus_drm_disp_panel.h"

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2
#define FHD_LCM_WIDTH 1080
#define FHD_LCM_HEIGHT 2412

#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern unsigned int oplus_enhance_mipi_strength;
extern unsigned int get_PCB_Version(void);
static unsigned int pcb_version = 0;
static bool panel_power_on = false;
extern bool pwm_power_on;

extern void lcdinfo_notify(unsigned long val, void *v);
extern unsigned int oplus_enhance_mipi_strength;
extern int wl2868c_set_ldo_enable(enum WL2868C_SELECT, uint32_t);
extern int wl2868c_set_ldo_disable(enum WL2868C_SELECT);
extern void gpio_dump_regs_range(int start, int end);
extern int oplus_display_panel_dbv_probe(struct device *dev);
#define MAX_CMDLINE_PARAM_LEN 512
extern char oplus_lcm_id2[MAX_CMDLINE_PARAM_LEN];
int lcm_id2 = 0;

extern struct oplus_demura_setting_table demura_setting;
extern bool oplus_display_get_demura_support(void);
extern void oplus_display_get_demura_cfg(struct device *dev);

struct lcm_pmic_info {
	struct regulator *reg_vufs18;
	struct regulator *reg_vmch3p0;
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *vddr1p2_enable_gpio;
	struct gpio_desc *vddr_aod_enable_gpio;
	struct gpio_desc *vci_enable_gpio;
	struct drm_display_mode *m;
	struct gpio_desc *te_switch_gpio,*te_out_gpio;
	bool prepared;
	bool enabled;
	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,                          \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
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
		DISP_ERR("error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	DISP_INFO("+\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		DISP_INFO("0x%08x\n",buffer[0] | (buffer[1] << 8));
		DISP_INFO("return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

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
		DISP_ERR("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static struct regulator *vddr_ldo;
static int lcm_panel_1p2_ldo_regulator_init(struct device *dev)
{
        static int regulator_1p2_inited;
        int ret = 0;

        if (regulator_1p2_inited)
                return ret;
		DISP_INFO("get lcm_panel_1p2_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        vddr_ldo = devm_regulator_get(dev, "vddr");
        if (IS_ERR_OR_NULL(vddr_ldo)) { /* handle return value */
                ret = PTR_ERR(vddr_ldo);
                DISP_ERR("get vddr_ldo fail, error: %d\n", ret);
        }
        regulator_1p2_inited = 1;
        return ret; /* must be 0 */

}

static int lcm_panel_1p2_ldo_enable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_1p2_ldo_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vddr_ldo)) {
		ret = regulator_set_voltage(vddr_ldo, 1200000, 1200000);
		if (ret < 0)
			DISP_ERR("set voltage vddr_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vddr_ldo)) {
		ret = regulator_enable(vddr_ldo);
		if (ret < 0)
			DISP_ERR("enable regulator vddr_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	DISP_INFO("get lcm_panel_1p2_ldo_enable\n");

        return retval;
}

static int lcm_panel_1p2_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_1p2_ldo_regulator_init(dev);

	if (!IS_ERR_OR_NULL(vddr_ldo)) {
		ret = regulator_disable(vddr_ldo);
		if (ret < 0)
			DISP_ERR("disable regulator vddr_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	return ret;
}

static struct regulator *vufs_ldo;
static int lcm_panel_vufs_ldo_regulator_init(struct device *dev)
{
        static int regulator_vufs_inited;
        int ret = 0;

        if (regulator_vufs_inited)
                return ret;
		DISP_INFO("get lcm_panel_vufs_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        vufs_ldo = devm_regulator_get(dev, "vufsldo");
        if (IS_ERR_OR_NULL(vufs_ldo)) { /* handle return value */
                ret = PTR_ERR(vufs_ldo);
                DISP_ERR("get vufs_ldo fail, error: %d\n", ret);
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
			DISP_ERR("set voltage vufs_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vufs_ldo)) {
		ret = regulator_enable(vufs_ldo);
		if (ret < 0)
			DISP_ERR("enable regulator vufs_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	DISP_INFO("get lcm_panel_vufs_ldo_enable\n");

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
			DISP_ERR("disable regulator vufs_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	return ret;
}

static struct regulator *wl2868c_ldo;
static int lcm_panel_wl2868c_ldo_regulator_init(struct device *dev)
{
        static int regulator_wl2868c_inited;
        int ret = 0;

        if (regulator_wl2868c_inited)
                return ret;
		DISP_INFO("get lcm_panel_wl2868c_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        wl2868c_ldo = devm_regulator_get(dev, "vci");
        if (IS_ERR_OR_NULL(wl2868c_ldo)) { /* handle return value */
                ret = PTR_ERR(wl2868c_ldo);
                DISP_ERR("get wl2868c_ldo fail, error: %d\n", ret);
        }
        regulator_wl2868c_inited = 1;
        return ret; /* must be 0 */

}

static int lcm_panel_wl2868c_ldo_enable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_wl2868c_ldo_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(wl2868c_ldo)) {
		ret = regulator_set_voltage(wl2868c_ldo, 3004000, 3004000);
		if (ret < 0)
			DISP_ERR("set voltage wl2868c_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(wl2868c_ldo)) {
		ret = regulator_enable(wl2868c_ldo);
		if (ret < 0)
			DISP_ERR("enable regulator wl2868c_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	DISP_INFO("get lcm_panel_wl2868c_ldo_enable\n");

        return retval;
}

static int lcm_panel_wl2868c_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_wl2868c_ldo_regulator_init(dev);

	if (!IS_ERR_OR_NULL(wl2868c_ldo)) {
		ret = regulator_disable(wl2868c_ldo);
		if (ret < 0)
			DISP_ERR("disable regulator wl2868c_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	return ret;
}

static void push_table(struct lcm *ctx, struct LCM_setting_table *table, unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			usleep_range(table[i].count*1000, table[i].count*1000 + 100);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 100);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write(ctx, table[i].para_list, table[i].count);
			break;
		}
	}
}

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle)
{
	unsigned int i = 0;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	if (lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		pr_err("out of mtk_ddic_dsi_cmd \n");
		return 0;
	}

	for (i = 0; i < lcm_cmd_count; i++) {
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].cmd_num = table[i].count;
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].para_list = table[i].para_list;
	}
	send_cmd_to_ddic.is_hs = 1;
	send_cmd_to_ddic.is_package = 1;
	send_cmd_to_ddic.cmd_count = lcm_cmd_count;
	cb(dsi, handle, &send_cmd_to_ddic);

	return 0;
}

static int inline get_panel_es_ver(void)
{
	int ret = 0;
	if (lcm_id2 > 0 && lcm_id2 <=1) {
		ret = ES_EVB;
	} else if (lcm_id2 == 2) {
		ret = ES_T0;
	} else if (lcm_id2 == 3) {
		ret = ES_EVT;
	} else if (lcm_id2 == 4) {
		ret = ES_DVT;
	} else if (lcm_id2 >= 5) {
		ret = ES_PVT;
	} else {
		ret = ES_EVB;
	}
	return ret;
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
        } else if (m_vrefresh == 120) {
                ret = FHD_SDC120;
        } else {
        	ret = FHD_SDC60;
        }
	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	struct drm_display_mode *m = ctx->m;

	mode_id = get_mode_enum(m);
	panel_power_on = true;
	pwm_power_on = true;
	switch (mode_id) {
		case FHD_SDC60:
			if (lcm_id2 > 0 && lcm_id2 <= 2) {
				push_table(ctx, init_setting_60Hz_evt, sizeof(init_setting_60Hz_evt)/sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, init_setting_60Hz, sizeof(init_setting_60Hz)/sizeof(struct LCM_setting_table));
			}
			break;
		case FHD_SDC90:
			if (lcm_id2 > 0 && lcm_id2 <= 2) {
				push_table(ctx, init_setting_90Hz_evt, sizeof(init_setting_90Hz_evt)/sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, init_setting_90Hz, sizeof(init_setting_90Hz)/sizeof(struct LCM_setting_table));
			}
			break;
		case FHD_SDC120:
			if (lcm_id2 > 0 && lcm_id2 <= 2) {
				push_table(ctx, init_setting_120Hz_evt, sizeof(init_setting_120Hz_evt)/sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, init_setting_120Hz, sizeof(init_setting_120Hz)/sizeof(struct LCM_setting_table));
			}
			break;
		default:
			if (lcm_id2 > 0 && lcm_id2 <= 2) {
				push_table(ctx, init_setting_60Hz_evt, sizeof(init_setting_60Hz_evt)/sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, init_setting_60Hz, sizeof(init_setting_60Hz)/sizeof(struct LCM_setting_table));
			}
			break;
    }
	DISP_INFO("mode_id=%d\n", mode_id);
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

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared) {
		return 0;
	}
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	/* Wait > 20ms,Actual 22ms */
	usleep_range(22000, 22100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	/* Wait > 120ms,Actual 125ms */
	usleep_range(125000, 125100);
	/* keep vcore off */
	lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
	DISP_DEBUG(" call lpm_smc_spm_dbg keep vcore off for display off!\n");

	ctx->error = 0;
	ctx->prepared = false;
	DISP_INFO("success\n");

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	DISP_INFO("success\n");
	return ret;
}

static const struct drm_display_mode disp_mode_60Hz = {
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

static const struct drm_display_mode disp_mode_90Hz = {
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

static const struct drm_display_mode disp_mode_120Hz = {
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

static struct mtk_panel_params ext_params_60Hz = {
	.pll_clk = 300,
	.phy_timcon = {
		.hs_trail = 11,
		.clk_trail = 11,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	// .esd_check_multi = 1,
	// .esd_te_check_gpio = 1,
	.move_esd_readdate_back = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
	},
	.lcm_esd_check_table[1] = {
  		.cmd = 0x91,
  		.count = 1,
  		.para_list[0] = 0xAB,
  		.mask_list[0] = 0xAB,
  	},
	.lcm_esd_check_table[2] = {
  		.cmd = 0x03,
  		.count = 1,
  		.para_list[0] = 0x00,
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
    .color_vivid_status = true,
    .color_srgb_status = true,
    .color_softiris_status = false,
    .color_dual_panel_status = false,
    .color_dual_brightness_status = true,
    .color_oplus_calibrate_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.skip_unnecessary_switch = true,
	.vendor = "AC160_A0001",
	.manufacture = "P_7",
	.panel_type = 2,
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
        .dsc_cfg_change = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 40,
        .rct_on = 1,
        .bit_per_channel = 10,
        .dsc_line_buf_depth = 11,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 549,
        .scale_value = 32,
        .increment_interval = 276,
        .decrement_interval = 7,
        .line_bpg_offset = 13,
        .nfl_bpg_offset = 2421,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
		},
	.data_rate = 600,
	.oplus_serial_para0 = 0xA3,
//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_te_count = 1,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 60,
		.apollo_limit_superior_us = 10000, .apollo_limit_inferior_us = 13596,
		.apollo_transfer_time_us = 8200,
	},
	.panel_bpp = 10,
};

static struct mtk_panel_params ext_params_90Hz = {
    .pll_clk = 300,
    .phy_timcon = {
        .hs_trail = 11,
        .clk_trail = 11,
    },
    .cust_esd_check = 0,
    .esd_check_enable = 1,
    //.esd_check_multi = 1,
    //.esd_te_check_gpio = 1,
    .move_esd_readdate_back = 1,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
    },
    .lcm_esd_check_table[1] = {
  		.cmd = 0x91,
  		.count = 1,
  		.para_list[0] = 0xAB,
  		.mask_list[0] = 0xAB,
    },
    .lcm_esd_check_table[2] = {
  		.cmd = 0x03,
  		.count = 1,
  		.para_list[0] = 0x00,
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
    .color_vivid_status = true,
    .color_srgb_status = true,
    .color_softiris_status = false,
    .color_dual_panel_status = false,
    .color_dual_brightness_status = true,
    .color_oplus_calibrate_status = true,
    .cmd_null_pkt_en = 1,
    .cmd_null_pkt_len = 0,
    .skip_unnecessary_switch = true,
    .vendor = "AC160_A0001",
    .manufacture = "P_7",
    .panel_type = 2,
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
        .dsc_cfg_change = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 40,
        .rct_on = 1,
        .bit_per_channel = 10,
        .dsc_line_buf_depth = 11,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 549,
        .scale_value = 32,
        .increment_interval = 276,
        .decrement_interval = 7,
        .line_bpg_offset = 13,
        .nfl_bpg_offset = 2421,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
        },
    .data_rate = 600,
    .oplus_serial_para0 = 0xA3,
//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
    .oplus_ofp_need_keep_apart_backlight = false,
    .oplus_ofp_hbm_on_delay = 0,
    .oplus_ofp_pre_hbm_off_delay = 2,
    .oplus_ofp_hbm_off_delay = 0,
    .oplus_te_count = 1,
    .oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
    .oplus_ofp_aod_off_insert_black = 1,
    .oplus_ofp_aod_off_black_frame_total_time = 42,
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
    .dyn_fps = {
        .switch_en = 1, .vact_timing_fps = 90,
	.apollo_limit_superior_us = 2910, .apollo_limit_inferior_us = 10000,
	.apollo_transfer_time_us = 8400,
    },
    .panel_bpp = 10,
};

static struct mtk_panel_params ext_params_120Hz = {
    .pll_clk = 553,
    .phy_timcon = {
        .hs_trail = 14,
        .clk_trail = 16,
    },
    .cust_esd_check = 0,
    .esd_check_enable = 1,
    // .esd_check_multi = 1,
    // .esd_te_check_gpio = 1,
    .move_esd_readdate_back = 1,
    .lcm_esd_check_table[0] = {
        .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0x9C,
    },
    .lcm_esd_check_table[1] = {
  		.cmd = 0x91,
  		.count = 1,
  		.para_list[0] = 0xAB,
  		.mask_list[0] = 0xAB,
    },
    .lcm_esd_check_table[2] = {
  		.cmd = 0x03,
  		.count = 1,
  		.para_list[0] = 0x00,
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
    .color_vivid_status = true,
    .color_srgb_status = true,
    .color_softiris_status = false,
    .color_dual_panel_status = false,
    .color_dual_brightness_status = true,
    .color_oplus_calibrate_status = true,
    .cmd_null_pkt_en = 1,
    .cmd_null_pkt_len = 0,
    .skip_unnecessary_switch = true,
    .vendor = "AC160_A0001",
    .manufacture = "P_7",
    .panel_type = 2,
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
        .dsc_cfg_change = 1,
        .ver = 17,
        .slice_mode = 1,
        .rgb_swap = 0,
        .dsc_cfg = 40,
        .rct_on = 1,
        .bit_per_channel = 10,
        .dsc_line_buf_depth = 11,
        .bp_enable = 1,
        .bit_per_pixel = 128,
        .pic_height = 2412,
        .pic_width = 1080,
        .slice_height = 12,
        .slice_width = 540,
        .chunk_size = 540,
        .xmit_delay = 512,
        .dec_delay = 549,
        .scale_value = 32,
        .increment_interval = 276,
        .decrement_interval = 7,
        .line_bpg_offset = 13,
        .nfl_bpg_offset = 2421,
        .slice_bpg_offset = 2170,
        .initial_offset = 6144,
        .final_offset = 4336,
        .flatness_minqp = 7,
        .flatness_maxqp = 16,
        .rc_model_size = 8192,
        .rc_edge_factor = 6,
        .rc_quant_incr_limit0 = 15,
        .rc_quant_incr_limit1 = 15,
        .rc_tgt_offset_hi = 3,
        .rc_tgt_offset_lo = 3,
        },
    .data_rate = 1106,
    .oplus_serial_para0 = 0xA3,
//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
    .oplus_ofp_need_keep_apart_backlight = false,
    .oplus_ofp_hbm_on_delay = 0,
    .oplus_ofp_pre_hbm_off_delay = 2,
    .oplus_ofp_hbm_off_delay = 0,
    .oplus_te_count = 1,
    .oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
    .oplus_ofp_aod_off_insert_black = 1,
    .oplus_ofp_aod_off_black_frame_total_time = 42,
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
    .dyn_fps = {
        .switch_en = 1, .vact_timing_fps = 120,
	.apollo_limit_superior_us = 0, .apollo_limit_inferior_us = 6898,
	.apollo_transfer_time_us = 6200,
    },
    .panel_bpp = 10,
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	int i = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 1) {
		DISP_INFO("enter aod\n");
		return 0;
	} else if (level > 4094) {
		level = 4094;
	}

	if (panel_power_on == true || level == 0) {
		DISP_INFO("[INFO][%s:%d]backlight lvl:%u.\n", __func__, __LINE__, level);
		panel_power_on = false;
	} else {
		DISP_DEBUG("[INFO][%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		level = 2047;
	}
	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

    bl_level[0].para_list[1] = level >> 8;
    bl_level[0].para_list[2] = level & 0xFF;

    for (i = 0; i < sizeof(bl_level)/sizeof(struct LCM_setting_table); i++) {
        cb(dsi, handle, bl_level[i].para_list, bl_level[i].count);
    }

	oplus_display_brightness = level;
	return 0;
}

static int oplus_display_panel_set_hbm_max(void *dsi, dcs_write_gce_pack cb, void *handle, unsigned int en)
{
	unsigned int lcm_cmd_count = 0;
	unsigned int level = oplus_display_brightness;

	if (!dsi || !cb) {
		DISP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (en) {
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_on) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, dsi_switch_hbm_apl_on, lcm_cmd_count, cb, handle);
		DISP_INFO("Enter hbm max APL mode\n");
	} else if (!en) {
		lcm_cmd_count = sizeof(dsi_switch_hbm_apl_off) / sizeof(struct LCM_setting_table);
		dsi_switch_hbm_apl_off[lcm_cmd_count-1].para_list[1] = level >> 8;
		dsi_switch_hbm_apl_off[lcm_cmd_count-1].para_list[2] = level & 0xFF;
		panel_send_pack_hs_cmd(dsi, dsi_switch_hbm_apl_off, lcm_cmd_count, cb, handle);
		DISP_INFO("hbm_max APL off, restore backlight:%d\n", level);
	}

	return 0;
}

static int oplus_display_panel_set_demura_bl(void *dsi, dcs_write_gce_pack cb, void *handle, int bl_demura_mode)
{
	int index = 5;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		if (panel_power_on)
			bl_demura_mode = OPLUS_DEMURA_DBV_MODE3;
		else
			return 0;
	}
	switch (bl_demura_mode) {
		case OPLUS_DEMURA_DBV_MODE0:
			index = sizeof(dsi_demura0_bl) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi, dsi_demura0_bl, index, cb, handle);
			break;
		case OPLUS_DEMURA_DBV_MODE1:
			index = sizeof(dsi_demura1_bl) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi, dsi_demura1_bl, index, cb, handle);
			break;
		case OPLUS_DEMURA_DBV_MODE2:
			index = sizeof(dsi_demura2_bl) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi, dsi_demura2_bl, index, cb, handle);
			break;
		case OPLUS_DEMURA_DBV_MODE3:
			index = sizeof(dsi_demura3_bl) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi, dsi_demura3_bl, index, cb, handle);
			break;
		case OPLUS_DEMURA_DBV_MODE4:
			index = sizeof(dsi_demura4_bl) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi, dsi_demura4_bl, index, cb, handle);
			break;
		default:
			DISP_ERR("Unsupported mode\n");
			break;
	}
	DISP_INFO("bl_demura_mode=%d, demura_bl_1=%d demura_bl_2=%d demura_bl_3=%d demura_bl_4=%d\n", bl_demura_mode,demura_setting.demura_switch_dvb1,demura_setting.demura_switch_dvb2, demura_setting.demura_switch_dvb3, demura_setting.demura_switch_dvb4);
	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	int bl_demura_mode = OPLUS_DEMURA_DBV_MODE_MAX;
	int index = 5;
	int i = 0;

	if (!cb)
		return -EINVAL;
	if ((level > 1) && (level < demura_setting.demura_switch_dvb1)) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE0;
		index = sizeof(dsi_demura0_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
		}
	} else if ((level >= demura_setting.demura_switch_dvb1) && (level < demura_setting.demura_switch_dvb2)) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE1;
		index = sizeof(dsi_demura1_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
		}
	} else if ((level >= demura_setting.demura_switch_dvb2) && (level < demura_setting.demura_switch_dvb3)) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE2;
		index = sizeof(dsi_demura2_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura2_bl[i].para_list, dsi_demura2_bl[i].count);
		}
	} else if ((level >= demura_setting.demura_switch_dvb3) && (level < demura_setting.demura_switch_dvb4)) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE3;
		index = sizeof(dsi_demura3_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura3_bl[i].para_list, dsi_demura3_bl[i].count);
		}
	} else if (level >= demura_setting.demura_switch_dvb4) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE4;
		index = sizeof(dsi_demura4_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura4_bl[i].para_list, dsi_demura4_bl[i].count);
		}
	}
	if (bl_demura_mode != OPLUS_DEMURA_DBV_MODE_MAX) {
		panel_power_on = false;
		pwm_power_on = false;
	}

    esd_bl_level[0].para_list[1] = level >> 8;
    esd_bl_level[0].para_list[2] = level & 0xFF;

    for (i = 0; i < sizeof(esd_bl_level)/sizeof(struct LCM_setting_table); i++) {
        cb(dsi, handle, esd_bl_level[i].para_list, esd_bl_level[i].count);
    }
    DISP_INFO("paralist[1]=%x, paralist[2]=%x, bl_demura_mode=%d\n", esd_bl_level[0].para_list[1], esd_bl_level[0].para_list[2], bl_demura_mode);

    return 0;
}

static int lcm_set_hbm(void *dsi, dcs_write_gce cb, void *handle, unsigned int hbm_mode)
{
	int i = 0;
    unsigned int level = oplus_display_brightness;

	if (!dsi || !cb) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("oplus_display_brightness=%d, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(HBM_on_setting)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, HBM_on_setting[i].para_list, HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
        HBM_off_setting[0].para_list[1] = level >> 8;
        HBM_off_setting[0].para_list[2] = level & 0xFF;
        for (i = 0; i < sizeof(HBM_off_setting)/sizeof(struct LCM_setting_table); i++) {
            cb(dsi, handle, HBM_off_setting[i].para_list, HBM_off_setting[i].count);
        }
	    DISP_DEBUG("paralist[1]=%x, paralist[2]=%x\n", HBM_off_setting[0].para_list[1], HBM_off_setting[0].para_list[2]);
	}
	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle, bool en)
{
	int i = 0;
    unsigned int level = oplus_display_brightness;

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	OFP_INFO("oplus_display_brightness=%d, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		for (i = 0; i < sizeof(HBM_on_setting)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, HBM_on_setting[i].para_list, HBM_on_setting[i].count);
		}
	} else if (en == 0) {
        HBM_off_setting[0].para_list[1] = level >> 8;
        HBM_off_setting[0].para_list[2] = level & 0xFF;
        for (i = 0; i < sizeof(HBM_off_setting)/sizeof(struct LCM_setting_table); i++) {
            cb(dsi, handle, HBM_off_setting[i].para_list, HBM_off_setting[i].count);
        }
	    DISP_DEBUG("paralist[1]=%x, paralist[2]=%x\n", HBM_off_setting[0].para_list[1], HBM_off_setting[0].para_list[2]);
	}
	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
    unsigned int cmd;
	for (i = 0; i < (sizeof(AOD_off_setting) / sizeof(struct LCM_setting_table)); i++) {

		cmd = AOD_off_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count * 1000, AOD_off_setting[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count, AOD_off_setting[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, AOD_off_setting[i].para_list, AOD_off_setting[i].count);
		}
	}

	lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	DISP_DEBUG("%s:success\n", __func__);

	return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;

	for (i = 0; i < (sizeof(AOD_on_setting)/sizeof(struct LCM_setting_table)); i++) {
		cmd = AOD_on_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(AOD_on_setting[i].count * 1000, AOD_on_setting[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(AOD_on_setting[i].count, AOD_on_setting[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, AOD_on_setting[i].para_list, AOD_on_setting[i].count);
		}
	}

	DISP_DEBUG("%s:success\n", __func__);

	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_bl_level[i].para_list, aod_high_bl_level[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_bl_level[i].para_list, aod_low_bl_level[i].count);
		}
	}
    DISP_DEBUG("%s:success %d !\n", __func__, level);

	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DISP_INFO("%s:on=%d\n", __func__,on);

	gpiod_set_value(ctx->reset_gpio, on);

	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared) {
		return 0;
	}
	// lcd reset H -> L -> H
	if(IS_ERR(ctx->reset_gpio)){
		DISP_ERR("cannot get reset-gpios %ld\n",PTR_ERR(ctx->reset_gpio));
	}
	gpiod_set_value(ctx->reset_gpio, 1);
	/* Wait > 1ms, actual 3ms */
	usleep_range(3000, 3100);
	gpiod_set_value(ctx->reset_gpio, 0);
	/* Wait > 10us, actual 2ms */
	usleep_range(2000, 2100);
	gpiod_set_value(ctx->reset_gpio, 1);
	/* Wait > 20ms, actual 25ms */
	usleep_range(25000, 25100);
	DISP_INFO("%s:Successful\n", __func__);

	return 0;
}
static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	//iovcc enable 1.8V
	lcm_panel_vufs_ldo_enable(ctx->dev);
	/* Wait > 1ms, actual 5ms */
	usleep_range(5000, 5100);
	//enable vcore 1p2 for boe is high 1.22v
	// gpiod_set_value(ctx->vddr_aod_enable_gpio, 1);
	// usleep_range(1000, 1100);
	if (((pcb_version > 0) && (pcb_version < 7)) || (pcb_version == 9)) {
		lcm_panel_1p2_ldo_enable(ctx->dev);
	} else {
		gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);
	}
	/* Wait no limits, actual 3ms */
	usleep_range(3000, 3100);
	//enable ldo 3p0
	lcm_panel_wl2868c_ldo_enable(ctx->dev);
	/* Wait > 10ms, actual 12ms */
	usleep_range(12000, 12100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	DISP_INFO("%s:Successful. pcb_version=%d\n", __func__, pcb_version);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	/* Wait > 1ms, actual 5ms */
	usleep_range(5000, 5100);
	//disable ldo 3p0
	lcm_panel_wl2868c_ldo_disable(ctx->dev);
	/* Wait no limits, actual 5ms */
	usleep_range(5000, 5100);
	//disable vcore1.2V
	if (((pcb_version > 0) && (pcb_version < 7)) || (pcb_version == 9)) {
		lcm_panel_1p2_ldo_disable(ctx->dev);
	} else {
		gpiod_set_value(ctx->vddr1p2_enable_gpio, 0);
	}
	/* Wait > 1ms, actual 5ms */
	usleep_range(5000, 5100);
	//set vddi 1.8v
	lcm_panel_vufs_ldo_disable(ctx->dev);
	/* power off Foolproof, actual 70ms*/
	usleep_range(70000, 70100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	DISP_INFO("%s: Successful.pcb_version=%d\n", __func__, pcb_version);
	return 0;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector, unsigned int mode)
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

static int mtk_panel_ext_param_set(struct drm_panel *panel, struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);
	DISP_INFO("%s: mode=%d, vrefresh=%d\n", __func__, mode, drm_mode_vrefresh(m));

	if (m_vrefresh == 60) {
		ext->params = &ext_params_60Hz;
	} else if (m_vrefresh == 90) {
		ext->params = &ext_params_90Hz;
	} else if (m_vrefresh == 120) {
		ext->params = &ext_params_120Hz;
	} else {
		ext->params = &ext_params_60Hz;
		ret = 1;
	}

	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60) {
		*ext_param = &ext_params_60Hz;
	} else if (m_vrefresh == 120) {
		*ext_param = &ext_params_120Hz;
	} else if (m_vrefresh == 90) {
		*ext_param = &ext_params_90Hz;
	} else {
		*ext_param = &ext_params_60Hz;
	}

	if (*ext_param)
		pr_debug("[LCM] data_rate:%d\n", (*ext_param)->data_rate);
	else
		pr_err("[LCM] ext_param is NULL;\n");

	return ret;
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	static int last_data_rate = 900;
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct lcm *ctx = panel_to_lcm(panel);
	DISP_INFO("lcm cur_mode = %d dst_mode %d\n", cur_mode, dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	if (stage == BEFORE_DSI_POWERDOWN) {
		if (drm_mode_vrefresh(m) == 60) {
			if (lcm_id2 > 0 && lcm_id2 <= 2) {
				push_table(ctx, mode_switch_to_60_evt, sizeof(mode_switch_to_60_evt) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, mode_switch_to_60, sizeof(mode_switch_to_60) / sizeof(struct LCM_setting_table));
			}
			DISP_INFO("lcm timing switch to 60\n");
		} else if (drm_mode_vrefresh(m) == 90) {
			if (lcm_id2 > 0 && lcm_id2 <= 2) {
				push_table(ctx, mode_switch_to_90_evt, sizeof(mode_switch_to_90_evt) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, mode_switch_to_90, sizeof(mode_switch_to_90) / sizeof(struct LCM_setting_table));
			}
			DISP_INFO("lcm timing switch to 90\n");
		} else if (drm_mode_vrefresh(m) == 120) {
			if (lcm_id2 > 0 && lcm_id2 <= 2) {
				push_table(ctx, mode_switch_to_120_evt, sizeof(mode_switch_to_120_evt) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, mode_switch_to_120, sizeof(mode_switch_to_120) / sizeof(struct LCM_setting_table));
			}
			DISP_INFO("lcm timing switch to 120\n");
		}
	}
	ctx->m = m;
	if (ext->params->data_rate != last_data_rate) {
		ret = 1;
		DISP_INFO("lcm need to change mipi clk, data_rate=%d,last_data_rate=%d\n",ext->params->data_rate,last_data_rate);
		last_data_rate = ext->params->data_rate;
	}
	return ret;
}

static unsigned int lcm_get_reg_ldo(struct regulator *ldo)
{
	unsigned int volt = 0;

	if (regulator_is_enabled(ldo))
		/* regulator_get_voltage return volt with uV */
		volt = regulator_get_voltage(ldo) ;

	return volt;
}

void lcm_read_info(struct drm_panel *panel, int read_ic)
{
	unsigned int volt = 0;

	/*read the 1.8v*/
	if (!IS_ERR_OR_NULL(vufs_ldo)) {
		volt = lcm_get_reg_ldo(vufs_ldo);
		DISP_INFO("get the Regulator vufs_ldo =%d.\n",volt);
	}

	/*read the 1.2v*/
	if (!IS_ERR_OR_NULL(vddr_ldo)) {
		if (((pcb_version > 0) && (pcb_version < 7)) || (pcb_version == 9)) {
			volt = lcm_get_reg_ldo(vddr_ldo);
			DISP_INFO("get the Regulator vddr_ldo =%d.\n",volt);
		}
	}

	/*read the 3.3v*/
	if (!IS_ERR_OR_NULL(wl2868c_ldo)) {
		volt = lcm_get_reg_ldo(wl2868c_ldo);
		DISP_INFO("get the Regulator wl2868c_ldo =%d.\n",volt);
	}

	gpio_dump_regs_range(120, 120);
	gpio_dump_regs_range(149, 149);

	DISP_INFO("read_ic = %d. pcb_version=%d\n", read_ic, pcb_version);

	return;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_reset = lcm_panel_reset,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.mode_switch = mode_switch,
	.set_hbm = lcm_set_hbm,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.oplus_get_info = lcm_read_info,
	.lcm_set_hbm_max = oplus_display_panel_set_hbm_max,
	.lcm_demura_set_bl = oplus_display_panel_set_demura_bl,
};

static int lcm_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode[3];
    struct lcm *ctx = panel_to_lcm(panel);

	mode[0] = drm_mode_duplicate(connector->dev, &disp_mode_60Hz);
	if (!mode[0]) {
		DISP_ERR("failed to add mode %ux%ux@%u\n", disp_mode_60Hz.hdisplay, disp_mode_60Hz.vdisplay, drm_mode_vrefresh(&disp_mode_60Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[0]);
	DISP_DEBUG("clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n", mode[0]->clock, mode[0]->htotal,
		mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

	mode[1] = drm_mode_duplicate(connector->dev, &disp_mode_90Hz);
	if (!mode[1]) {
		DISP_ERR("failed to add mode %ux%ux@%u\n", disp_mode_90Hz.hdisplay, disp_mode_90Hz.vdisplay, drm_mode_vrefresh(&disp_mode_90Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[1]);
	mode[1]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[1]);

	mode[2] = drm_mode_duplicate(connector->dev, &disp_mode_120Hz);
	if (!mode[2]) {
		DISP_ERR("failed to add mode %ux%ux@%u\n", disp_mode_120Hz.hdisplay, disp_mode_120Hz.vdisplay, drm_mode_vrefresh(&disp_mode_120Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[2]);
	mode[2]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[2]);

	connector->display_info.width_mm = 70;
	connector->display_info.height_mm = 155;

    if (!ctx->m) {
    	ctx->m = get_mode_by_id(connector, 0);
    	DISP_INFO("ctx->m init: mode_id %d\n",get_mode_enum(ctx->m));
    }
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
	int ret;
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	unsigned int fp_type = 0x08;
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	DISP_INFO("[LCM] %s+ p_7 AC160_A0001 Start\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				DISP_ERR("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			DISP_ERR("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		DISP_ERR("skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(ctx->reset_gpio)) {
		DISP_ERR("cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 1);
	//devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	lcm_panel_vufs_ldo_enable(ctx->dev);

	usleep_range(5000, 5100);
	pcb_version = get_PCB_Version();
	DISP_INFO("get PCB Version = %u\n", pcb_version);
	if (((pcb_version > 0) && (pcb_version < 7)) || (pcb_version == 9)) {
		lcm_panel_1p2_ldo_enable(ctx->dev);
	} else {
		ctx->vddr1p2_enable_gpio = devm_gpiod_get(ctx->dev, "1p2", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
			DISP_ERR("cannot get vddr1p2_enable_gpio %ld\n",PTR_ERR(ctx->vddr1p2_enable_gpio));
			return PTR_ERR(ctx->vddr1p2_enable_gpio);
		}
	}

	usleep_range(5000, 5100);

	lcm_panel_wl2868c_ldo_enable(ctx->dev);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_60Hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif

	oplus_display_get_demura_cfg(dev);

	register_device_proc("lcd", "AC160_A0001", "P_7");
	flag_silky_panel = BL_SETTING_DELAY_60HZ;
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	oplus_display_panel_dbv_probe(dev);
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	oplus_ofp_set_fp_type(&fp_type);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
	sscanf(oplus_lcm_id2, "%d", &lcm_id2);
	DISP_INFO("AC160_A0001, oplus_lcm_id2:%s lcm_id2:%d End\n", oplus_lcm_id2, lcm_id2);
	oplus_enhance_mipi_strength = 1;
	//g_is_silky_panel = true;
	pwm_power_on = true;
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
	{
	    .compatible = "ac160,p_7,a0001,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "ac160_p_7_a0001_cmd_panel",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register lcm driver: %d\n", __func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit lcm_drv_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}

module_init(lcm_drv_init);
module_exit(lcm_drv_exit);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("ac160,p_7,a0001,OLED Driver");
MODULE_LICENSE("GPL v2");
