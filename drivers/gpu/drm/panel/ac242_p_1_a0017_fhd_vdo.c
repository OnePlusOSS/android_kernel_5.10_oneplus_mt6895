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
#include <soc/oplus/device_info.h>
#include <linux/regulator/consumer.h>
#include <mt-plat/mtk_boot_common.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "../oplus/oplus_display_panel_power.h"
#include "../mediatek/mediatek_v2/mtk_log.h"
#include "../mediatek/mediatek_v2/mtk_dsi.h"
#include "../mediatek/mediatek_v2/mtk_drm_ddp_comp.h"


#include "include/ac242_p_1_a0017_fhd_vdo.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/ac242_p_1_data_hw_roundcornerpattern.h"
#include "oplus_bl.h"
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "../oplus/oplus_adfr_ext.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR  */
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* add for cmdq_pkt_sleep */
#include "../mediatek/mediatek_v2/mtk-cmdq-ext.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#include "../oplus/oplus_drm_disp_panel.h"

static u32 flag_hbm = 0;
static int mode_id = -1;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
static int esd_brightness;
extern unsigned int last_backlight;

static unsigned int temp_seed_mode = 0;
/* whether enter hbm brightness level or not */

extern void lcdinfo_notify(unsigned long val, void *v);
struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr_enable_gpio;
	struct gpio_desc *vci_enable_gpio;
	struct drm_display_mode *m;
	struct gpio_desc *lcm_esd_err_fg_gpio;
	bool prepared;
	bool enabled;
	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)				\
	({												\
		const u8 d[] = { seq };						\
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,		\
				 "DCS sequence too big for stack");	\
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));		\
	})

#define lcm_dcs_write_seq_static(ctx, seq...)		\
	({												\
		static const u8 d[] = { seq };				\
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));		\
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

static void push_table(struct lcm *ctx, struct LCM_setting_table *table,
		unsigned int count)
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
			lcm_dcs_write(ctx, table[i].para_list,
				table[i].count);
			break;
		}
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

	DISP_INFO("%s+\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		DISP_DEBUG("return %d data(0x%08x) to dsi engine\n",ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
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
	} else if (m_vrefresh == 120) {
		ret = FHD_SDC120;
	} else {
		ret = FHD_SDC60;
	}
	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	switch (mode_id) {
		case FHD_SDC120:
			DISP_INFO("fhd_dsi_on_cmd_sdc120\n");
			push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120) / sizeof(struct LCM_setting_table));
		break;
		case FHD_SDC60:
			DISP_INFO("fhd_dsi_on_cmd_sdc60\n");
			push_table(ctx, dsi_on_cmd_sdc60, sizeof(dsi_on_cmd_sdc60) / sizeof(struct LCM_setting_table));
		break;
		default:
			DISP_INFO("%default fhd_dsi_on_cmd_sdc120\n");
			push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120) / sizeof(struct LCM_setting_table));
		break;
	}

	DISP_INFO("%s, restore seed_mode:%d\n", __func__, temp_seed_mode);
	if (temp_seed_mode == LCM_SEED_NATURAL){
		push_table(ctx, lcm_seed_natural, sizeof(lcm_seed_natural) / sizeof(struct LCM_setting_table));
	} else if (temp_seed_mode == LCM_SEED_VIVID){
		push_table(ctx, lcm_seed_vivid, sizeof(lcm_seed_vivid) / sizeof(struct LCM_setting_table));
	}
	DISP_INFO("successful\n");
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
	DISP_INFO("prepared=%d\n",ctx->prepared);

	if (!ctx->prepared) {
		return 0;
	}

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(25000, 25100);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(125000, 125100);
	ctx->error = 0;
	ctx->prepared = false;
	DISP_INFO("Successful\n");
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
	usleep_range(2000, 2100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	DISP_INFO("lcm_prepare Successful\n");
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

static const struct drm_display_mode display_mode[MODE_NUM] = {
	/* sdc_120_mode */
	{
		.clock = 353710, // ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP + VBP + VSA) * 120) / 1000
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP,
		.hsync_end = FRAME_WIDTH + HFP + HSA,
		.htotal = FRAME_WIDTH + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_120HZ,
		.vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
		//.hskew = SDC_ADFR,
	},
	/* sdc_60_mode */
	{
		.clock = 353710, // ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP + VBP + VSA) * 60) / 1000
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP,
		.hsync_end = FRAME_WIDTH + HFP + HSA,
		.htotal = FRAME_WIDTH + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_60HZ,
		.vsync_end = FRAME_HEIGHT + VFP_60HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_60HZ + VSA + VBP,
		//.hskew = SDC_MFR,
	},
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params[MODE_NUM] = {
	/* fhd_sdc_120_mode */
	{
	.pll_clk = 499,
	.phy_timcon = {
		.hs_trail = 10,
		.clk_trail = 10,
	},
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.skip_unnecessary_switch = true,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	.esd_check_multi = 0,
	.oplus_serial_para0 = 0xD8,
	.vendor = "AC242_A0017",
	.manufacture = "P_1",
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
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 40,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 989,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 631,
		.slice_bpg_offset = 651,
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
	.data_rate = 998,
//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 0,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 0,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 3 , {0xF0, 0x5A, 0x5A}},
		.dfps_cmd_table[1] = {0, 2 , {0x60, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0xF7, 0x0B}},
		.dfps_cmd_table[3] = {0, 3 , {0xF0, 0xA5, 0xA5}},
		},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 494,
		.data_rate = 988,
		.hfp = 55,
		.vfp = 16,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.panel_bpp = 8,
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	},
	/* fhd_sdc_60_mode */
	{
	.pll_clk = 499,
	.phy_timcon = {
		.hs_trail = 10,
		.clk_trail = 10,
	},
	.change_fps_by_vfp_send_cmd_need_delay = 1,
	.skip_unnecessary_switch = true,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	.esd_check_multi = 0,
	.oplus_serial_para0 = 0xD8,
	.vendor = "AC242_A0017",
	.manufacture = "P_1",
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
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 40,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 989,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 631,
		.slice_bpg_offset = 651,
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
	.data_rate = 998,
//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 0,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 0,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 3 , {0xF0, 0x5A, 0x5A}},
		.dfps_cmd_table[1] = {0, 2 , {0x60, 0x21}},
		.dfps_cmd_table[2] = {0, 2 , {0xF7, 0x0B}},
		.dfps_cmd_table[3] = {0, 3 , {0xF0, 0xA5, 0xA5}},
		},
	.dyn = {
		.switch_en = 1,
		.pll_clk = 494,
		.data_rate = 988,
		.hfp = 55,
		.vfp = 2448,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.panel_bpp = 8,
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	},
};

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);
	if (!m) {
		pr_err("invalid drm_display_mode\n");
		return 1;
	}

	mode_id = get_mode_enum(m);
	DISP_INFO("mode:%d,mode_id:%d\n", id, mode_id);

	if (mode_id == FHD_SDC120) {
		*ext_param = &ext_params[0];
	} else if (mode_id == FHD_SDC60) {
		*ext_param = &ext_params[1];
	} else {
		*ext_param = &ext_params[0];
	}

	if (*ext_param)
		DISP_DEBUG("data_rate:%d\n", (*ext_param)->data_rate);
	else
		DISP_ERR("ext_param is NULL;\n");

	return ret;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	if (!m) {
		pr_err("invalid drm_display_mode\n");
		return 1;
	}
	mode_id = get_mode_enum(m);
	DISP_INFO("mode:%d,mode_id:%d\n", mode, mode_id);

	if (mode_id == FHD_SDC120) {
		ext->params = &ext_params[0];
	} else if (mode_id == FHD_SDC60) {
		ext->params = &ext_params[1];
	} else {
		ext->params = &ext_params[0];
	}

	return ret;
}

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	unsigned char bl_tb0[] = {0x51, 0x07, 0xFF};
	unsigned char bl_tb1[] = {0x53, 0x20};
	unsigned int mapped_level = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 0) {
		DISP_INFO("[INFO][%s:%d]backlight lvl:%u, flag_hbm = %d\n", __func__, __LINE__, level, flag_hbm);
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	} else {
		DISP_INFO("[INFO][%s:%d]backlight lvl:%u, flag_hbm = %d\n", __func__, __LINE__, level, flag_hbm);
	}

	last_backlight = level;
	mapped_level = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		level = 1023;
	}

	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	if (level == 1) {
		DISP_INFO("[INFO][%s:%d]filter backlight %d setting\n", __func__, __LINE__, level);
		return 0;
	} else if (level <= 2047) {
		DISP_INFO("[%s:%d]backlight =  %d setting, flag_hbm = %d\n", __func__, __LINE__, level, flag_hbm);
		if (flag_hbm == 1) {
			bl_tb1[1] = 0x20;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 0;
		}
	}  else if (level > 2047 && level <= 4095) {
		DISP_INFO("[%s:%d]backlight =  %d setting, flag_hbm = %d\n", __func__, __LINE__, level, flag_hbm);
		if (flag_hbm == 0) {
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 1;
		}
	}

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	esd_brightness = level;
	oplus_display_brightness = level;
	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned char bl_tb0[] = {0x51, 0x03, 0xff};
	unsigned char bl_tb1[] = {0x53, 0x20};

	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &esd_brightness);

	if (esd_brightness == 1) {
		DISP_INFO("[INFO][%s:%d]filter backlight %d setting\n", __func__, __LINE__, esd_brightness);
		return 0;
	}  else if (esd_brightness <= 2047) {
		DISP_INFO("[%s:%d]esd_brightness =  %d setting\n", __func__, __LINE__, esd_brightness);
		if (flag_hbm == 1) {
			bl_tb1[1] = 0x20;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 0;
		}
	}  else if (esd_brightness > 2047 && esd_brightness <= 4095) {
		DISP_INFO("[%s:%d]esd_brightness =  %d setting\n", __func__, __LINE__, esd_brightness);
		if (flag_hbm == 0) {
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 1;
		}
	}

	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	DISP_INFO("[INFO][%s:%d]esd_brightness=%d\n", __func__, __LINE__, esd_brightness);
	return 0;
}

/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	unsigned int i = 0;

	OFP_DEBUG("start\n");

	if (!dsi || !cb) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("hbm_mode:%u,bl_lvl:%u\n", hbm_mode, oplus_display_brightness);

	if (hbm_mode == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd_60hz)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_on_cmd_60hz[i].para_list, hbm_on_cmd_60hz[i].count);
		}
	} else if (hbm_mode == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd_60hz)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_off_cmd_60hz[i].para_list, hbm_off_cmd_60hz[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	unsigned int vrefresh_rate = 0;
	unsigned int level = oplus_display_brightness;
	struct lcm *ctx = NULL;
	struct mtk_dsi *mtk_dsi = dsi;
	struct drm_crtc *crtc = NULL;
	struct drm_display_mode *mode = NULL;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct LCM_setting_table *hbm_cmd = NULL;
	struct mtk_ddp_comp comp = {0};

	OFP_DEBUG("start\n");

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	comp = ((struct mtk_dsi*)dsi)->ddp_comp;

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	crtc = mtk_dsi->encoder.crtc;
	if (!crtc) {
		OFP_ERR("Invalid crtc param\n");
		return -EINVAL;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc) {
		OFP_ERR("Invalid mtk_crtc param\n");
		return -EINVAL;
	}

	mode = &mtk_crtc->base.state->adjusted_mode;
	if (!mode) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(mode);
	}

	OFP_INFO("hbm_en:%u,bl_lvl:%u,refresh_rate:%u\n", en, oplus_display_brightness, vrefresh_rate);

	if (vrefresh_rate == 60) {
		if (en) {
			hbm_cmd = hbm_on_cmd_60hz;
			reg_count = sizeof(hbm_on_cmd_60hz) / sizeof(struct LCM_setting_table);
		} else {
			hbm_cmd = hbm_off_cmd_60hz;
			reg_count = sizeof(hbm_off_cmd_60hz) / sizeof(struct LCM_setting_table);
		}
	} else if (vrefresh_rate == 90) {
		if (en) {
			hbm_cmd = hbm_on_cmd_90hz;
			reg_count = sizeof(hbm_on_cmd_90hz) / sizeof(struct LCM_setting_table);
		} else {
			hbm_cmd = hbm_off_cmd_90hz;
			reg_count = sizeof(hbm_off_cmd_90hz) / sizeof(struct LCM_setting_table);
		}
	} else if (vrefresh_rate == 120) {
		if (en) {
			hbm_cmd = hbm_on_cmd_120hz;
			reg_count = sizeof(hbm_on_cmd_120hz) / sizeof(struct LCM_setting_table);
		} else {
			hbm_cmd = hbm_off_cmd_120hz;
			reg_count = sizeof(hbm_off_cmd_120hz) / sizeof(struct LCM_setting_table);
		}
	}

	if (!en) {
		if (level <= 2047) {
			flag_hbm = 0;
			hbm_cmd[5].para_list[1] = 0x20;
		} else if (level > 2047 && level <= 4095) {
			flag_hbm = 1;
			hbm_cmd[5].para_list[1] = 0xE0;
		}
		hbm_cmd[6].para_list[1] = level >> 8;
		hbm_cmd[6].para_list[2] = level & 0xFF;
	}

	for (i = 0; i < reg_count; i++) {
		unsigned int cmd;
		cmd = hbm_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (!handle) {
					usleep_range(hbm_cmd[i].count * 1000, hbm_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(hbm_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (!handle) {
					usleep_range(hbm_cmd[i].count, hbm_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(hbm_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, hbm_cmd[i].para_list, hbm_cmd[i].count);
		}
	}

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct mtk_dsi *mtk_dsi = dsi;
	struct drm_crtc *crtc = NULL;
	struct drm_display_mode *mode = NULL;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct LCM_setting_table *aod_off_cmd = NULL;
	struct mtk_ddp_comp comp = {0};

	if (!panel || !dsi) {
		OFP_ERR("Invalid dsi params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	comp = ((struct mtk_dsi*)dsi)->ddp_comp;

	crtc = mtk_dsi->encoder.crtc;
	if (!crtc) {
		OFP_ERR("Invalid crtc param\n");
		return -EINVAL;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc) {
		OFP_ERR("Invalid mtk_crtc param\n");
		return -EINVAL;
	}

	mode = &mtk_crtc->base.state->adjusted_mode;
	if (!mode) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(mode);
	}

	if (vrefresh_rate == 60) {
		aod_off_cmd = aod_off_cmd_60hz;
		reg_count = sizeof(aod_off_cmd_60hz) / sizeof(struct LCM_setting_table);
	} else if (vrefresh_rate == 120) {
		aod_off_cmd = aod_off_cmd_120hz;
		reg_count = sizeof(aod_off_cmd_120hz) / sizeof(struct LCM_setting_table);
	} else {
		aod_off_cmd = aod_off_cmd_90hz;
		reg_count = sizeof(aod_off_cmd_90hz) / sizeof(struct LCM_setting_table);
	}

	for (i = 0; i < reg_count; i++) {
		unsigned int cmd;
		cmd = aod_off_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count * 1000, aod_off_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count, aod_off_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count),CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_off_cmd[i].para_list, aod_off_cmd[i].count);
		}
	}

	lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	OFP_INFO("send aod off cmd\n");
	return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;

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

	OFP_INFO("send aod on cmd\n");
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
	OFP_INFO("level = %d\n", level);
	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);
	DISP_INFO("reset on=%d\n",on);
	gpiod_set_value(ctx->reset_gpio, on);
	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared) {
		DISP_DEBUG("ctx->prepared:%d return! \n",ctx->prepared);
		return 0;
	}
	usleep_range(11000,11100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(2000,2100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(2000,2100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(25000,25100);
	DISP_INFO("Successful\n");

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared){
		return 0;
	}
	DISP_INFO("begin+\n");

	/* vddi 1.8v Alawys Enable */
	usleep_range(5000, 5100);

	/* vci 3.0v */
	ctx->vci_enable_gpio = devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_enable_gpio)) {
		DISP_ERR("%s: cannot get vci_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci_enable_gpio));
		return PTR_ERR(ctx->vci_enable_gpio);
	}
	gpiod_set_value(ctx->vci_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci_enable_gpio);
	usleep_range(11000, 11100);

	/* vddr 1.2v */
	ctx->vddr_enable_gpio = devm_gpiod_get(ctx->dev, "vddr", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr_enable_gpio)) {
		DISP_ERR("%s: cannot get bias_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr_enable_gpio));
		return PTR_ERR(ctx->vddr_enable_gpio);
	}
	gpiod_set_value(ctx->vddr_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr_enable_gpio);
	usleep_range(5000, 5100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	DISP_INFO("%s-\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	DISP_INFO("begin+\n");
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);
	/* vddr 1.2v */
	ctx->vddr_enable_gpio = devm_gpiod_get(ctx->dev, "vddr", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr_enable_gpio);
	usleep_range(5000, 5100);
	/* vci 3.0v */
	ctx->vci_enable_gpio = devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vci_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vci_enable_gpio);
	usleep_range(11000, 11100);
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	usleep_range(70000, 70100);
	flag_hbm = 0;
	DISP_INFO("%s-\n", __func__);
	return 0;
}

static int lcm_get_modes(struct drm_panel *panel,
                    struct drm_connector *connector) {
    struct drm_display_mode *mode[MODE_NUM];
    int i = 0;

    mode[0] = drm_mode_duplicate(connector->dev, &display_mode[0]);
    if (!mode[0]) {
        DISP_ERR("failed to add mode %ux%ux@%u\n",
            display_mode[0].hdisplay, display_mode[0].vdisplay,
             drm_mode_vrefresh(&display_mode[0]));
        return -ENOMEM;
    }

    drm_mode_set_name(mode[0]);
    mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
    drm_mode_probed_add(connector, mode[0]);
    DISP_DEBUG("en=%u, clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n",mode[0], mode[0]->clock, mode[0]->htotal,
        mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

    for (i = 1; i < MODE_NUM; i++) {
        mode[i] = drm_mode_duplicate(connector->dev, &display_mode[i]);
        DISP_DEBUG(" en=%u\n",mode[i]);
        if (!mode[i]) {
            DISP_ERR("not enough memory\n");
            return -ENOMEM;
        }

        drm_mode_set_name(mode[i]);
        mode[i]->type = DRM_MODE_TYPE_DRIVER;
        drm_mode_probed_add(connector, mode[i]);
    }
    connector->display_info.width_mm = PHYSICAL_WIDTH / 1000;
    connector->display_info.height_mm = PHYSICAL_HEIGHT / 1000;

    return 1;
}


/* static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int seed_mode)
{
	int i = 0;

	DISP_INFO("debug for lcm %s, seed_mode=%d\n", __func__, seed_mode);
	temp_seed_mode = seed_mode;
	if (seed_mode == LCM_SEED_NATURAL){
		for (i = 0; i < sizeof(lcm_seed_vivid)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_vivid[i].para_list, lcm_seed_vivid[i].count);
		}
	} else if (seed_mode == LCM_SEED_VIVID){
		for (i = 0; i < sizeof(lcm_seed_natural)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_natural[i].para_list, lcm_seed_natural[i].count);
		}
	}

	return 0;
} */

static int lcm_esd_gpio_read(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int lcm_esd_err_fg_gpio = 0;
	int ret = 0;

	lcm_esd_err_fg_gpio = gpiod_get_value(ctx->lcm_esd_err_fg_gpio);

	pr_err("AC242 debug for %s frist time esd_err_fg = %d\n", __func__, lcm_esd_err_fg_gpio);

	if (!lcm_esd_err_fg_gpio) {
		msleep(100);
		lcm_esd_err_fg_gpio = gpiod_get_value(ctx->lcm_esd_err_fg_gpio);

		pr_err("debug for %s second time esd_err_fg = %d\n", __func__, lcm_esd_err_fg_gpio);
		if(!lcm_esd_err_fg_gpio) {
			pr_err("AC242 debug for %s triger esd to recovery\n", __func__);
			ret = 1;
		}
	} else {
		ret = 0;
	}

	if (1 == ret) {
		char payload[200] = "";
		int cnt = 0;

		cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, "DisplayDriverID@@507$$");
		cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, "ESD:");
		cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, "master_0x%x", lcm_esd_err_fg_gpio);
		pr_err("AC242 debug for %s ESD check failed: %s\n", __func__, payload);
		mm_fb_display_kevent(payload, MM_FB_KEY_RATELIMIT_1H, "ESD check failed");
	}

	return ret;
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
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	//.set_seed = panel_set_seed,
	.esd_read_gpio = lcm_esd_gpio_read,
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.set_hbm = lcm_set_hbm,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
};
#endif

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
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	unsigned int fp_type = 0x200;
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	DISP_INFO("ac242_p_1_a0017_fhd_vdo lcm_probe+\n");

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				DISP_ERR("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			DISP_ERR("device node name:%s, dev->of_node name:%s\n", remote_node->name, dev->of_node->name);
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
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->vci_enable_gpio = devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_enable_gpio)) {
		DISP_ERR("cannot get vci_enable_gpio %ld\n",
			PTR_ERR(ctx->vci_enable_gpio));
		return PTR_ERR(ctx->vci_enable_gpio);
	}
	gpiod_set_value(ctx->vci_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci_enable_gpio);
	usleep_range(10000, 10100);

	ctx->vddr_enable_gpio = devm_gpiod_get(ctx->dev, "vddr", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr_enable_gpio)) {
		DISP_ERR("%s: cannot get vddr_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr_enable_gpio));
		return PTR_ERR(ctx->vddr_enable_gpio);
	}
	gpiod_set_value(ctx->vddr_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr_enable_gpio);
	usleep_range(5000, 5100);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DISP_ERR("cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
	usleep_range(11000, 11100);

	ctx->lcm_esd_err_fg_gpio = devm_gpiod_get_optional(ctx->dev, "lcm-esd-err-fg", GPIOD_IN);
	if (IS_ERR(ctx->lcm_esd_err_fg_gpio)) {
		pr_err("[esd]cannot get lcm_esd_err_fg_gpio %ld\n",
			PTR_ERR(ctx->lcm_esd_err_fg_gpio));
		return PTR_ERR(ctx->lcm_esd_err_fg_gpio);
	} else {
		gpiod_direction_input(ctx->lcm_esd_err_fg_gpio);
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
	ret = mtk_panel_ext_create(dev, &ext_params[0], &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif

	register_device_proc("lcd", "AC242_A0017", "P_1");
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	 oplus_ofp_set_fp_type(&fp_type);
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	DISP_INFO("ac242_p_1_a0017_fhd_vdo-\n");
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
		.compatible = "ac242,p_1,a0017,fhd,vdo",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "ac242_p_1_a0017_fhd_vdo",
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
		pr_notice("%s, Failed to register lcm driver: %d\n",
			__func__, ret);

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

MODULE_AUTHOR("like");
MODULE_DESCRIPTION("ac242_p_1_a0017_fhd_vdo Panel Driver");
MODULE_LICENSE("GPL v2");
