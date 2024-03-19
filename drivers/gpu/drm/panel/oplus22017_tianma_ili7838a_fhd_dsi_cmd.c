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
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#include <mt-plat/mtk_boot_common.h>
#include "../mediatek/mediatek_v2/mtk-cmdq-ext.h"
#include "../oplus/oplus_drm_disp_panel.h"
#include "../../../misc/mediatek/lpm/inc/lpm_module.h"
#include "../../../misc/mediatek/lpm/modules/include/mt6895/mt6895_pwr_ctrl.h"

#include "../mediatek/mediatek_v2/mtk_corner_pattern/data_hw_roundedpattern_l_tianma_linhai.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/data_hw_roundedpattern_r_tianma_linhai.h"
/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#include "oplus22017_tianma_ili7838a_fhd_dsi_cmd.h"

static unsigned int esd_brightness = 1023;
extern unsigned int oplus_display_brightness;
extern unsigned int last_backlight;
static bool aod_state = false;

extern void lcdinfo_notify(unsigned long val, void *v);
extern unsigned int oplus_enhance_mipi_strength;

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
	struct gpio_desc *vddr1p5_enable_gpio;
	struct gpio_desc *vddr_aod_enable_gpio;
	struct gpio_desc *vci_enable_gpio;
	struct gpio_desc *te_switch_gpio,*te_out_gpio;
	struct drm_display_mode *m;
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
#if 1
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
	DISP_DEBUG("get lcm_panel_vufs_ldo_enable\n");

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
#endif

static void push_table(struct lcm *ctx, struct LCM_setting_table *table,
		unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			usleep_range(table[i].count * 1000, table[i].count * 1000 + 100);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 1000);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write(ctx, table[i].para_list,
				table[i].count);
			break;
		}
	}
}
static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = -1;
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
	}
	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	struct drm_display_mode *m = ctx->m;

	mode_id = get_mode_enum(m);
	DISP_INFO("mode_id=%d\n", mode_id);
	switch (mode_id) {
	case FHD_SDC60:
		push_table(ctx, dsi_on_cmd_sdc60, sizeof(dsi_on_cmd_sdc60)/sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC90:
		push_table(ctx, dsi_on_cmd_sdc90, sizeof(dsi_on_cmd_sdc90)/sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC120:
		push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120)/sizeof(struct LCM_setting_table));
		break;
	default:
		push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120)/sizeof(struct LCM_setting_table));
		break;
	}

	DISP_INFO("Successful\n");
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
	DISP_DEBUG("prepared=%d\n",ctx->prepared);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(20000, 20100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(100000, 100100);
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

	DISP_DEBUG("prepared=%d\n",ctx->prepared);
	if (ctx->prepared)
		return 0;

	// lcd reset H -> L -> H
	usleep_range(10000, 10100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(15000, 15100);
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
	.pll_clk = 418,
	.phy_timcon = {
	    .hs_trail = 13,
	    .clk_trail = 14,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
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
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.vendor = "tianma_ili7838a",
	.manufacture = "tianma",
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
	.data_rate = 836,
	.oplus_serial_para0 = 0xA3,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
        .oplus_ofp_aod_off_insert_black = 2,
        .oplus_ofp_aod_off_black_frame_total_time = 67,
        .oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 0,
        .oplus_ofp_hbm_off_delay = 0,
        .dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 60,
        },
	.panel_bpp = 10,
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 418,
	.phy_timcon = {
            .hs_trail = 13,
            .clk_trail = 14,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
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
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.vendor = "tianma_ili7838a",
	.manufacture = "tianma",
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
	.data_rate = 836,
	.oplus_serial_para0 = 0xA3,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
        .oplus_ofp_aod_off_insert_black = 2,
        .oplus_ofp_aod_off_black_frame_total_time = 56,
        .oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 0,
        .oplus_ofp_hbm_off_delay = 0,
        .dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 90,
        },
	.panel_bpp = 10,
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 489,
	.phy_timcon = {
            .hs_trail = 13,
            .clk_trail = 14,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
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
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 105,
	.vendor = "tianma_ili7838a",
	.manufacture = "tianma",
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
	.data_rate = 978,
	.oplus_serial_para0 = 0xA3,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
        .oplus_ofp_aod_off_insert_black = 2,
        .oplus_ofp_aod_off_black_frame_total_time = 50,
        .oplus_ofp_need_keep_apart_backlight = true,
        .oplus_ofp_hbm_on_delay = 0,
        .oplus_ofp_pre_hbm_off_delay = 2,
        .oplus_ofp_hbm_off_delay = 0,
	.dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 120,
        },
	.panel_bpp = 10,
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	unsigned int mapped_level = 0;
	char bl_tb0[] = {0x51, 0x07, 0xFF};
//	char bl_tb1[] = {0xB2, 0x01};
//	char bl_tb2[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0)
		level = 2047;

	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	mapped_level = level;
	DISP_INFO("backlight=%d,bl_tb0[1]=0x%x,bl_tb0[2]=0x%x,mapped_level=%d \n", level,bl_tb0[1] ,bl_tb0[2],mapped_level);

	if (!cb)
		return -1;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	if (level ==1) {
		DISP_ERR("enter aod!!!\n");
		return 0;
	}

	esd_brightness = level;
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	oplus_display_brightness = level;

	return 0;
}

static void lcm_setbrightness(void *dsi,
			      dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	int i = 0;

	DISP_DEBUG("level is %d\n",level);
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

	if (!cb)
		return -1;

	DISP_DEBUG("oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		//oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		//level = oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 0);
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);  //level
		DISP_DEBUG(" %d ! backlight %d !\n",hbm_mode, oplus_display_brightness);
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

	DISP_INFO("oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		level = oplus_display_brightness;
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		DISP_DEBUG(": %d ! backlight %d !\n", en, oplus_display_brightness);
	}
	lcdinfo_notify(1, &en);
	return 0;
}


static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
	void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};


	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
 	if (!cb)
		return -1;
	DISP_ERR(" bl_tb0[1]=%x, bl_tb0[2]=%x\n", bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

 	return 1;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	gpiod_set_value(ctx->reset_gpio, on);

	return 0;
}


static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;

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


	aod_state = false;
	/* keep vcore off */
	lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL, MT_LPM_SMC_ACT_SET, PW_REG_SPM_VCORE_REQ, 0x0);
	lcm_setbacklight_cmdq(dsi, cb, handle, last_backlight);
	DISP_DEBUG(" call lpm_smc_spm_dbg keep vcore off for exit AOD!\n");
	DISP_DEBUG("success\n");

	return 0;
}

static struct LCM_setting_table lcm_normal_to_aod_sam[] = {


    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},
    {REGFLAG_CMD, 1, {0x39}},
    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x0C}},
    {REGFLAG_CMD, 2, {0xBE,0x01}},
    {REGFLAG_CMD, 4, {0xFF,0x78,0x38,0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i=0;

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
	DISP_DEBUG("call lpm_smc_spm_dbg keep vcore on for enter AOD then Succ!\n");

	return 0;
}


static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	DISP_DEBUG("+ %s\n");
	if (level == 0) {
		for (i = 0; i < sizeof(lcm_aod_high_mode)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_aod_high_mode[i].para_list, lcm_aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(lcm_aod_low_mode)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_aod_low_mode[i].para_list, lcm_aod_low_mode[i].count);
		}
	}
	DISP_DEBUG(" success %d !\n" ,level);

	return 0;
}
static int oplus_lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		DISP_ERR("error %ld reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void oplus_lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	ret = oplus_lcm_dcs_read(ctx, 0x0A, buffer, 1);
	DISP_INFO("0x0A return %d data(0x%08x) to dsi engine\n",ret, buffer[0] | (buffer[1] << 8));
	ret = oplus_lcm_dcs_read(ctx, 0x0E, buffer, 1);
	DISP_INFO("return 0x0E %d data(0x%08x) to dsi engine\n",ret, buffer[0] | (buffer[1] << 8));

}

void lcm_read_info(struct drm_panel *panel, int read_ic)
{
	unsigned int ret = 0;
	struct lcm *ctx = panel_to_lcm(panel);
	/* VCI LDO GPIO enable */
	if (IS_ERR(ctx->vci_enable_gpio))
		DISP_ERR("get vci enable gpio failed\n");
	else {
		ret = gpiod_get_value(ctx->vci_enable_gpio);
		DISP_INFO("vci_enable_gpio = %d\n", ret);
	}

	/* vcore 1p2 GPIO enable */
	if (IS_ERR(ctx->vddr_aod_enable_gpio))
		DISP_ERR("get vddr_aod gpio failed\n");
	else {
		ret = gpiod_get_value(ctx->vddr_aod_enable_gpio);
		DISP_INFO("vddr_aod_gpio = %d\n", ret);
	}

	/* vddr1p5 GPIO enable */
	if (IS_ERR(ctx->vddr1p5_enable_gpio))
		DISP_ERR("get vddr1p5 gpio failed\n");
	else {
		ret = gpiod_get_value(ctx->vddr1p5_enable_gpio);
		DISP_INFO("vddr1p5_gpio = %d\n", ret);
	}

	/* Reset GPIO enable */
	if (IS_ERR(ctx->reset_gpio))
		DISP_ERR("get reset gpio failed\n");
	else {
		ret = gpiod_get_value(ctx->reset_gpio);
		DISP_INFO("reset_gpio = %d\n", ret);
	}
	pr_info("read_ic = %d\n", read_ic);
	/* DDIC register detect */
	if (read_ic > 0) {
		oplus_lcm_panel_get_data(ctx);
	}
	return;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	DISP_DEBUG("tianma_ili7838a lcm ctx->prepared %d\n", ctx->prepared);

	if (ctx->prepared)
		return 0;

	//iovcc enable 1.8V
	lcm_panel_vufs_ldo_enable(ctx->dev);
	usleep_range(2000, 2100);
	//enable vci 3p0
	gpiod_set_value(ctx->vci_enable_gpio, 1);
	usleep_range(3000, 3100);
	//enable vcore 1p2 tm is low 1.126v
	gpiod_set_value(ctx->vddr_aod_enable_gpio, 1);
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	usleep_range(5000, 5100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);


	DISP_INFO(" success %d \n");
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	DISP_DEBUG(" tianma_ili7838alcm ctx->prepared %d \n",ctx->prepared);

	if (ctx->prepared)
		return 0;

	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 5100);
	//disable iovcc 1.5V
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	usleep_range(5000, 5100);
	//disable vci 3p0
	gpiod_set_value(ctx->vci_enable_gpio, 0);
	usleep_range(10000, 10100);
	//set vddi 0v
	lcm_panel_vufs_ldo_disable(ctx->dev);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	//msleep(110);
	usleep_range(50000, 50100);
	DISP_INFO(" success %d \n");
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
		pr_err("%s invalid drm_display_mode\n", __func__);
		return 1;
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

static void mode_switch_to_120(struct drm_panel *panel, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == AFTER_DSI_POWERON) {
		lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
		lcm_dcs_write_seq_static(ctx,0x38,0x11);
		lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
		DISP_DEBUG("set success\n");
	}
}

static void mode_switch_to_90(struct drm_panel *panel, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
		lcm_dcs_write_seq_static(ctx,0x38,0x12);
		lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
		//usleep_range(16*1000, 17*1000);
		DISP_DEBUG("set success\n");

	}
}

static void mode_switch_to_60(struct drm_panel *panel, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x02);
		lcm_dcs_write_seq_static(ctx,0x38,0x13);
		lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x38,0x00);
		//usleep_range(16*1000, 17*1000);
		DISP_DEBUG("set success\n");

	}
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct lcm *ctx = NULL;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, dst_mode);
	if (!m) {
		pr_err("invalid drm_display_mode\n");
		return 1;
	}

	if (panel) {
		ctx = panel_to_lcm(panel);
	}

	DISP_INFO("cur_mode = %d dst_mode %d\n", cur_mode, dst_mode);

	if (drm_mode_vrefresh(m) == 60) { /* 60 switch to 120 */
		mode_switch_to_60(panel, stage);
	} else if (drm_mode_vrefresh(m) == 90) { /* 1200 switch to 60 */
		mode_switch_to_90(panel, stage);
	} else if (drm_mode_vrefresh(m) == 120) { /* 1200 switch to 60 */
		mode_switch_to_120(panel, stage);
	} else
		ret = 1;

	if (ctx) {
		ctx->m = m;
	}

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
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.set_hbm = lcm_set_hbm,
	.oplus_get_info = lcm_read_info,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.esd_check_precondition = lcm_esd_check_precondition,
};
#endif

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		DISP_ERR("failed to add mode %ux%ux@%u\n",
			 default_mode.hdisplay, default_mode.vdisplay,
			 drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);


	mode2 = drm_mode_duplicate(connector->dev, &performance_mode_90hz);
	if (!mode2) {
		DISP_ERR("failed to add mode %ux%ux@%u\n",
			 performance_mode_90hz.hdisplay, performance_mode_90hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_90hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);

	mode3 = drm_mode_duplicate(connector->dev, &performance_mode_120hz);
	if (!mode3) {
		DISP_ERR("failed to add mode %ux%ux@%u\n",
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

	DISP_INFO("tianma7838a +\n");

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
		DISP_ERR("cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	lcm_panel_vufs_ldo_enable(ctx->dev);
	usleep_range(5000, 5100);
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p5_enable_gpio)) {
		DISP_ERR(" cannot get vddr1p5_enable_gpio %ld\n",PTR_ERR(ctx->vddr1p5_enable_gpio));
		return PTR_ERR(ctx->vddr1p5_enable_gpio);
	}
	ctx->vddr_aod_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-aod-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr_aod_enable_gpio)) {
		DISP_ERR(" cannot get vddr_aod_enable_gpio %ld\n",PTR_ERR(ctx->vddr_aod_enable_gpio));
		return PTR_ERR(ctx->vddr_aod_enable_gpio);
	}

	usleep_range(5000, 5100);

	ctx->vci_enable_gpio = devm_gpiod_get(ctx->dev, "3p0-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_enable_gpio)) {
		DISP_ERR(" cannot get vci_enable_gpio %ld\n",PTR_ERR(ctx->vci_enable_gpio));
		return PTR_ERR(ctx->vci_enable_gpio);
	}

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
	register_device_proc("lcd", "tianma_ili7838a", "tianma");
	oplus_enhance_mipi_strength = 1;
	flag_silky_panel = BL_SETTING_DELAY_60HZ;

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	oplus_ofp_set_fp_type(&fp_type);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	DISP_ERR("success\n");

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
	    .compatible = "oplus22071,tianma,ili7838a,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus22017_tianma_ili7838a_fhd_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	int ret = 0;

	DISP_INFO("start\n");
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		DISP_ERR("Failed to register jdi driver: %d\n",ret);

	mtk_panel_unlock();
	DISP_INFO("end ret=%d\n",ret);
	return 0;
}

static void __exit lcm_drv_exit(void)
{
	DISP_INFO("start\n");
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	DISP_INFO("end\n");
}
module_init(lcm_drv_init);
module_exit(lcm_drv_exit);


MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("oplus22071,tianma,ili7838a,OLED Driver");
MODULE_LICENSE("GPL v2");
