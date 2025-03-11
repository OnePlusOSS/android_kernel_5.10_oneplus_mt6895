/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
//#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/of_graph.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include <linux/of_graph.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_log.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* add for cmdq_pkt_sleep */
#include "../mediatek/mediatek_v2/mtk-cmdq-ext.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/oplus22281_AMS643AG01_data_hw_roundedpattern.h"
#endif
/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#include "oplus22281_samsung_ams643ag01_1080p_dsi_cmd.h"

//extern int pmic_ldo_2_set_voltage_uv(unsigned int set_uV);
//extern int pmic_ldo_2_set_disable(void);
extern unsigned int seed_mode;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_panel_index;

extern void lcdinfo_notify(unsigned long val, void *v);

/* whether enter hbm brightness level or not */
static bool hbm_brightness_flag = false;
static int esd_brightness;
extern unsigned int oplus_max_normal_brightness;
static bool aod_state = false;
static unsigned int last_backlight = 0;

__attribute__((weak)) void lcd_tp_refresh_switch(int fps)
{
    return;
}

#define LCM_DSI_CMD_MODE 1

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   2047
#define LCM_BRIGHTNESS_TYPE 2

/*#ifdef OPLUS_BUG_STABILITY*/
/*
#include "../oplus/oplus_display_panel_power.h"
extern int oplus_export_drm_panel(struct drm_panel *panel_node);
extern int fan53870_ldo1_regmap_read(void);
extern int dsi_panel_parse_panel_power_cfg(struct panel_voltage_bak *panel_vol);
*/
/*
static PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {
						{0, 1800000, 1800000, 1800000, "vddi"},
						{1, 1056000, 1104000, 1152000, "vddr"},
						{2, 0, 1, 2, ""}};
*/
/*#endif*/ /*OPLUS_BUG_STABILITY*/

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *lcm3v0_gpio;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;

	int error;

	bool cv_state;
};

static int panel_osc_freq_change(void *dsi, dcs_write_gce cb, void *handle, bool en)
{
	int i = 0;
	if (!cb)
		return -EINVAL;

	DISP_INFO("en = %d\n", en);
	if (en == 0) {
		for (i = 0; i< sizeof(osc_167M_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, osc_167M_cmd[i].para_list, osc_167M_cmd[i].count);
		}
	} else if (en == 1) {
		for (i = 0; i< sizeof(osc_167M_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, osc_169M_cmd[i].para_list, osc_169M_cmd[i].count);
		}
	}
	return 0;
}

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128, "DCS sequence too big for stack");\
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
		DISP_ERR("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
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
		DISP_ERR("error %d reading dcs seq:(%#x)\n", ret, cmd);
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
		DISP_DEBUG("return %d data(0x%08x) to dsi engine\n",
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
		DISP_ERR("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		DISP_ERR("get dsv_neg fail, error: %d\n", ret);
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
		DISP_ERR("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		DISP_ERR("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		DISP_ERR("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		DISP_ERR("enable regulator disp_bias_neg fail, ret = %d\n", ret);
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
		DISP_ERR("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		DISP_ERR("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
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
    	ret = FHD_SDC60;
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
		if (seed_mode == 101) {
			pr_info("dsi_on_cmd_sdc60_seed_sRGB\n");
			push_table(ctx, lcm_seed_mode_sRGB, sizeof(lcm_seed_mode_sRGB)/sizeof(struct LCM_setting_table));
		} else {
			pr_info("dsi_on_cmd_sdc60_seed_dci-p3\n");
			push_table(ctx, lcm_seed_mode_dcip3, sizeof(lcm_seed_mode_dcip3)/sizeof(struct LCM_setting_table));
		}
		usleep_range(20*1000, 20*1000+100);
		/* Display On*/
		lcm_dcs_write_seq_static(ctx,0x29);
		break;
	case FHD_SDC90:
		push_table(ctx, dsi_on_cmd_sdc90, sizeof(dsi_on_cmd_sdc90)/sizeof(struct LCM_setting_table));
		if (seed_mode == 101) {
			pr_info("dsi_on_cmd_sdc90_seed_sRGB\n");
			push_table(ctx, lcm_seed_mode_sRGB, sizeof(lcm_seed_mode_sRGB)/sizeof(struct LCM_setting_table));
		} else {
			pr_info("dsi_on_cmd_sdc90_seed_dci-p3\n");
			push_table(ctx, lcm_seed_mode_dcip3, sizeof(lcm_seed_mode_dcip3)/sizeof(struct LCM_setting_table));
		}
		usleep_range(20*1000, 20*1000+100);
		/* Display On*/
		lcm_dcs_write_seq_static(ctx,0x29);
		break;
	default:
		push_table(ctx, dsi_on_cmd_sdc60, sizeof(dsi_on_cmd_sdc60)/sizeof(struct LCM_setting_table));
		if (seed_mode == 101) {
			pr_info("dsi_on_cmd_sdc60_seed_sRGB\n");
			push_table(ctx, lcm_seed_mode_sRGB, sizeof(lcm_seed_mode_sRGB)/sizeof(struct LCM_setting_table));
		} else {
			pr_info("dsi_on_cmd_sdc60_seed_dci-p3\n");
			push_table(ctx, lcm_seed_mode_dcip3, sizeof(lcm_seed_mode_dcip3)/sizeof(struct LCM_setting_table));
		}
		usleep_range(20*1000, 20*1000+100);
		/* Display On*/
		lcm_dcs_write_seq_static(ctx,0x29);
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
	DISP_INFO("Successful\n");
	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(20*1000, 20*1000+100);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(150*1000, 150*1000+100);

	ctx->error = 0;
	ctx->prepared = false;
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#endif
	DISP_INFO("Successful\n");
	return 0;
}

//extern int power_mode;
static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	aod_state = false;
	lcm_panel_init(ctx);
/*
	if(power_mode == 2)
	{
		DDPINFO("%s + lcm_panel_init,resume status\n", __func__);
		lcm_panel_init(ctx);
	}
*/
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	DISP_INFO("Successful\n");
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
	DISP_INFO("Successful\n");
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
	//.vrefresh = 60,
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
	//.vrefresh = 90,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x99, .mask_list[0] = 0xF9,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0E, .count = 1, .para_list[0] = 0x80,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	},
	.lcm_esd_check_table[3] = {
		.cmd = 0x03, .count = 1, .para_list[0] = 0x01,
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
	.data_rate = 598,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 67,
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 16,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 9,
	.oplus_serial_para0 = 0xD8,
	.dyn_fps = {
		.switch_en = 0, .vact_timing_fps = 60,
	},
	.vendor = "AMS643AG01",
	.manufacture = "samsung2048",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_90hz = {
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x99, .mask_list[0] = 0xF9,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0E, .count = 1, .para_list[0] = 0x80,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	},
	.lcm_esd_check_table[3] = {
		.cmd = 0x03, .count = 1, .para_list[0] = 0x01,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.skip_unnecessary_switch = true,
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
	.data_rate = 598,
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 67,
	.oplus_ofp_hbm_on_delay = 12,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 12,
	.oplus_serial_para0 = 0xD8,
	.dyn_fps = {
		.switch_en = 0, .vact_timing_fps = 90,
	},
	.vendor = "AMS643AG01",
	.manufacture = "samsung2048",
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
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

	if (!m) {
		pr_err("%s:%d invalid display_mode\n", __func__, __LINE__);
		return ret;
	}

	printk(" mtk_panel_ext_param_set enter \n");
	if (drm_mode_vrefresh(m) == 60)
		ext->params = &ext_params;
	else if (drm_mode_vrefresh(m) == 90)
		ext->params = &ext_params_90hz;
	else
		ret = 1;
	return ret;
}
static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int seed_mode)
{
	int i = 0;

	if (seed_mode == 102) {
		for (i = 0; i < sizeof(lcm_seed_mode_dcip3)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode_dcip3[i].para_list, lcm_seed_mode_dcip3[i].count);
		}
	} else if (seed_mode == 101){
		for (i = 0; i < sizeof(lcm_seed_mode_sRGB)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode_sRGB[i].para_list, lcm_seed_mode_sRGB[i].count);
		}
	}
	DISP_INFO("seed_mode=%d successful \n", seed_mode);
	//memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	return 0;
}

static void mode_switch_60_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {

	} else if (stage == AFTER_DSI_POWERON) {
		/* display on switch to 90hz */
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
		/* display on switch to 60hz */
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

	if (!m) {
		pr_err("%s invalid drm_display_mode\n", __func__);
		return 1;
	}

	if (cur_mode == dst_mode)
		return ret;

	if (stage == BEFORE_DSI_POWERDOWN) {
		if (cur_mode == 0 && dst_mode == 1) { /* 60 switch to 90 */
			DISP_INFO("[TP] ready charge to 90 hz");
			lcd_tp_refresh_switch(90);
		} else if (cur_mode == 1 && dst_mode == 0) { /* 90 switch to 60 */
			DISP_INFO("[TP] ready charge to 60 hz");
			lcd_tp_refresh_switch(60);
		}
	}

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

#if 0
static int panel_doze_area_set(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

#if 0
	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	usleep_range(12*1000, 12*1000+100);
#endif
	cb(dsi, handle, send_cmd, ARRAY_SIZE(send_cmd));

	//memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	DISP_INFO("Successful\n");

	return 0;
}
#endif

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned int notify_level = 0;
	char bl_tb0[] = {0x51, 0x03, 0xFF};
	char bl_tb1[] = {0x53, 0x20};

	char post_backlight_on[] = {0x29};

	if (!cb)
		return -1;
	if (level > 2047)
		level = level + 1679;

	if (level > 4095)
		level = 4095;
	//mapped_level = oplus_lcm_dc_backlight(dsi,cb,handle, level, 0);
	mapped_level = level;
	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;

	esd_brightness = mapped_level;
	if (level > 2)
		last_backlight = level;

	DISP_INFO("display panel backlight value,level :=%d, mapped_level := %d\n", level, mapped_level);
	if (mapped_level > 1) {
		notify_level = mapped_level * 2047 / MAX_NORMAL_BRIGHTNESS;
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &notify_level);
	}

	if (level == 1 || mapped_level == 1) {
	    DISP_INFO("enter aod mode, ignore set backlight to 1\n");
	    if (aod_state == 1) {
			cb(dsi, handle, post_backlight_on, ARRAY_SIZE(post_backlight_on));
		}
	} else if (level <= BRIGHTNESS_HALF) {
	    if(hbm_brightness_flag == true) {
			bl_tb1[1] = 0x20;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			hbm_brightness_flag = false;
		}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if (level > BRIGHTNESS_HALF && level <= BRIGHTNESS_MAX) {
	    if (hbm_brightness_flag == false) {
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			hbm_brightness_flag = true;
	    }
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	}

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

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	DISP_INFO("bl_tb0[1]=%x, bl_tb0[2]=%x\n", bl_tb0[1], bl_tb0[2]);

	return 1;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_gpio)) {
		DISP_ERR("%s: cannot get bias_gpio %ld\n",
			__func__, PTR_ERR(ctx->bias_gpio));
		return PTR_ERR(ctx->bias_gpio);
	}
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	usleep_range(5*1000, 5*1000+100);
	ctx->lcm3v0_gpio = devm_gpiod_get(ctx->dev, "lcm3v0", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->lcm3v0_gpio)) {
		DISP_ERR("%s: cannot get bias_gpio %ld\n",
			__func__, PTR_ERR(ctx->lcm3v0_gpio));
		return PTR_ERR(ctx->lcm3v0_gpio);
	}
	gpiod_set_value(ctx->lcm3v0_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->lcm3v0_gpio);
	//usleep_range(2*1000, 2*1000+100);
	/*pmic_ldo_2_set_voltage_uv(1120000);*/
	usleep_range(10*1000, 10*1000+100);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5*1000, 5*1000+100);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5*1000, 5*1000+100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5*1000, 5*1000+100);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(12*1000, 12*1000+100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(5000, 5100);
	DISP_INFO("Successful\n");
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	usleep_range(5000, 5100);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(10*1000, 10*1000+100);
	/*pmic_ldo_2_set_disable();*/
	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	usleep_range(5*1000, 5*1000+100);
	ctx->lcm3v0_gpio = devm_gpiod_get(ctx->dev, "lcm3v0", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->lcm3v0_gpio)) {
		DISP_ERR("%s: cannot get bias_gpio %ld\n",
			__func__, PTR_ERR(ctx->lcm3v0_gpio));
		return PTR_ERR(ctx->lcm3v0_gpio);
	}
	gpiod_set_value(ctx->lcm3v0_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->lcm3v0_gpio);
	usleep_range(10*1000, 10*1000+100);
	hbm_brightness_flag = false;
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(70*1000, 70*1000+100);
	DISP_INFO("Successful\n");
	return 0;
}

static int oplus_send_cmd_before_dsi_read(struct drm_panel *panel,void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0xF0, 0x5A, 0x5A};
	char bl_tb1[] = {0xB0, 0x07, 0xD8};

	DISP_INFO("oplus_send_cmd_before_dsi_read");
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	return 0;
}

static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;

	if (!cb)
		return -EINVAL;

	DISP_INFO("oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_on_cmd[i].para_list, hbm_on_cmd[i].count);
		}
		hbm_brightness_flag = true;
	} else if (hbm_mode == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_off_cmd[i].para_list, hbm_off_cmd[i].count);
		}
		hbm_brightness_flag = false;
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
		DISP_INFO("hbm_mode : %d ! backlight %d !\n", hbm_mode, oplus_display_brightness);
	}

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	int i = 0;

	if (!cb) {
		DISP_ERR("Invalid params\n");
		return -EINVAL;
	}

	DISP_INFO("last_backlight= %ld, oplus_display_brightness = %ld, en=%u\n", last_backlight, oplus_display_brightness, en);

	if(en == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_on_cmd[i].para_list, hbm_on_cmd[i].count);
		}
		hbm_brightness_flag = true;
	} else if (en == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_off_cmd[i].para_list, hbm_off_cmd[i].count);
		}
		hbm_brightness_flag = false;
		if (last_backlight > oplus_display_brightness) {
			if (oplus_display_brightness > 3)
			{
				lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
			} else {
				lcm_setbacklight_cmdq(dsi, cb, handle, last_backlight);
			}

		} else {
			lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
		}

		/* return to the current loading effect configuration */
		panel_set_seed(dsi, cb, handle, seed_mode);
	}

	return 0;
}
static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;

	for (i = 0; i < (sizeof(aod_off_cmd) / sizeof(struct LCM_setting_table)); i++) {
		unsigned int cmd;
		cmd = aod_off_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count * 1000, aod_off_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count * 1000), CMDQ_GPR_R12);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count, aod_off_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count), CMDQ_GPR_R12);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_off_cmd[i].para_list, aod_off_cmd[i].count);
		}
	}
	/* return to the current loading effect configuration */
	panel_set_seed(dsi, cb, handle, seed_mode);
	aod_state = false;
	DISP_INFO("send aod off cmd\n");

	return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	aod_state = true;

	for (i = 0; i < (sizeof(aod_on_cmd)/sizeof(struct LCM_setting_table)); i++) {
		unsigned int cmd;
		cmd = aod_on_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(aod_on_cmd[i].count * 1000, aod_on_cmd[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(aod_on_cmd[i].count, aod_on_cmd[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_on_cmd[i].para_list, aod_on_cmd[i].count);
		}
	}

	DISP_INFO("send aod on cmd\n");

	return 0;
}

static int panel_doze_post_disp_on(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	char post_backlight_on1[] = {0x29};

	cb(dsi, handle, post_backlight_on1, ARRAY_SIZE(post_backlight_on1));
	DISP_INFO("panel_doze_post_disp_on cmd\n");
	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_mode[i].para_list, aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_mode[i].para_list, aod_low_mode[i].count);
		}
	}
	DISP_INFO("level = %d\n", level);

	return 0;
}
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

#if 0
static bool panel_no_video_cmd_switch_state(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	ctx->cv_state = true;

    return ctx->cv_state;
}
#endif


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
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);
	connector->display_info.width_mm = 67;
	connector->display_info.height_mm = 149;

	return 1;
}
/*#ifdef OPLUS_BUG_STABILITY*/
#if 0
static int lcm_set_power(unsigned int voltage_id, unsigned int voltage_value)
{
	int ret = 0;
	DISP_INFO("voltage_id = %u, voltage_value = %u\n",
			voltage_id, voltage_value);

	switch(voltage_id) {
	/*0 represent the vddi*/
	case 0: {
			DISP_ERR("vddi not need to update\n");
			ret = -1;
		}
		break;
	/*case 1 represent vddr*/
	case 1: {
			ret = pmic_ldo_2_set_voltage_uv(voltage_value);
			if (ret < 0) {
				DISP_ERR("error ret = %d", ret);
			}
		}
		break;
	/*case 2 represent vgh*/
	case 2:
		break;
	default:
		DISP_ERR("error voltage id %d", voltage_id);
		break;
	}

	return ret;
}

/*voltage_id represent the PANEL_VOLTAGE_ENUM
  return the voltage_value of the voltage_id*/
static int lcm_update_power(uint32_t voltage_id)
{
	int ret = 0;
	DISP_INFO("voltage_id = %d\n", voltage_id);

	switch(voltage_id) {
	/*0 represent the vddi*/
	case 0: {
			DISP_ERR("vddi not need to update\n");
			ret = -1;
		}
		break;
	/*case 1 represent vddr*/
	case 1: {
			ret = fan53870_ldo1_regmap_read();
			if (ret < 0) {
				DISP_ERR("error ret = %d", ret);
			}
		}
		break;
	/*case 2 represent vgh*/
	case 2:
		break;
	default:
		DISP_ERR("error voltage id %d", voltage_id);
		break;
	}
	return ret;
}
/*#endif*/ /*OPLUS_BUG_STABILITY*/
#endif
static struct mtk_panel_funcs ext_funcs = {
//	.aod = enter_aod,
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	//.panel_disp_off = lcm_panel_disp_off,
	//.panel_no_cv_switch = panel_no_video_cmd_switch_state,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	.set_hbm = lcm_set_hbm,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.doze_post_disp_on = panel_doze_post_disp_on,
	.set_aod_light_mode = panel_set_aod_light_mode,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
	.send_cmd_before_dsi_read = oplus_send_cmd_before_dsi_read,
	.set_seed = panel_set_seed,
	.lcm_osc_change = panel_osc_freq_change,
	/*#ifdef OPLUS_BUG_STABILITY*/
	//.oplus_set_power =	lcm_set_power,
	//.oplus_update_power_value = lcm_update_power,
	/*#endif*/ /*OPLUS_BUG_STABILITY*/
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
	struct lcm *ctx;
	struct device_node *backlight;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	int ret;
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	unsigned int fp_type = 0x08;
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	dsi_node = of_get_parent(dev->of_node);
	DISP_INFO("[soso]22281 samsung probe\n");
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			DISP_INFO("device_node name:%s\n", remote_node->name);
        }
	}
	if (remote_node != dev->of_node) {
		DISP_INFO("skip probe due to not current lcm, of_node is %s\n", dev->of_node->name);
		return 0;
	}
	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;

#if (LCM_DSI_CMD_MODE)
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;
#else
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

#endif
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}
	ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_gpio)) {
		DISP_ERR("cannot get bias_gpio %ld\n",
			PTR_ERR(ctx->bias_gpio));
		return PTR_ERR(ctx->bias_gpio);
	}
	gpiod_set_value(ctx->bias_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	usleep_range(5*1000, 5*1000+100);
	ctx->lcm3v0_gpio = devm_gpiod_get(ctx->dev, "lcm3v0", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->lcm3v0_gpio)) {
		DISP_ERR("%s: cannot get bias_gpio %ld\n",
			__func__, PTR_ERR(ctx->lcm3v0_gpio));
		return PTR_ERR(ctx->lcm3v0_gpio);
	}
	gpiod_set_value(ctx->lcm3v0_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->lcm3v0_gpio);
	//usleep_range(5*1000, 5*1000+100);
    //pmic_ldo_2_set_voltage_uv(1120000);
	usleep_range(5*1000, 5*1000+100);
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DISP_ERR("cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
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
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	/*#ifdef OPLUS_BUG_STABILITY*/
	//oplus_export_drm_panel(&ctx->panel);
	//dsi_panel_parse_panel_power_cfg(panel_vol_bak);
	/*#endif*/ /*OPLUS_BUG_STABILITY*/

	register_device_proc("lcd", "AMS643AG01_22281", "samsung2048_22281");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	oplus_panel_index = 2228101;

	//register_device_proc("lcd", "AMS644VA04_MTK04", "samsung1024");

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	oplus_ofp_set_fp_type(&fp_type);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	DISP_INFO("Successful\n");

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "oplus22281_samsung_ams643ag01_1080p_dsi_cmd,lcm", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus22281_samsung_ams643ag01_1080p_dsi_cmd,lcm",
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

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("oplus22281_samsung_ams643ag01_1080p_dsi_cmd CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
