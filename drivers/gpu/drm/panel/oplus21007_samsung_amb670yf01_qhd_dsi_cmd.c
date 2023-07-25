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
#include <soc/oplus/device_info.h>
#include <linux/regulator/consumer.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

/*#ifdef OPLUS_BUG_STABILITY*/
#include "../oplus/oplus_display_panel_power.h"

#include "oplus21007_samsung_amb670yf01_qhd_dsi_cmd.h"
#include "oplus_bl.h"
//#ifdef OPLUS_ADFR
#include "../oplus/oplus_adfr_ext.h"
//#endif
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* add for cmdq_pkt_sleep */
#include "../mediatek/mediatek_v2/mtk-cmdq-ext.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#include "../oplus/oplus_drm_disp_panel.h"
#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   2047
#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
#define LCM_BRIGHTNESS_TYPE 2

#define WQHD_LCM_WIDTH 1440
#define WQHD_LCM_HEIGHT 3216
#define FHD_LCM_WIDTH  1080
#define FHD_LCM_HEIGHT 2412

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

extern int dsi_panel_parse_panel_power_cfg(struct panel_voltage_bak *panel_vol);
static PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {
						{0, 1800000, 1800000, 1800000, "vddi"},
						{1, 1800000, 1104000, 1800000, "vddr"},
						{2, 3000000, 1104000, 3000000, "vci"},
						{3, 0, 1, 2, ""}};

extern unsigned int seed_mode;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;

static int esd_brightness;
/* whether enter hbm brightness level or not */
static bool hbm_brightness_flag = false;

extern void lcdinfo_notify(unsigned long val, void *v);

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
	struct gpio_desc *aod1p3_enable_gpio;
//#ifdef OPLUS_ADFR
	/* add for mux switch control */
	struct gpio_desc *te_switch_gpio;
	struct drm_display_mode *m;
//#endif

	bool prepared;
	bool enabled;

	int error;
};

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write_ext(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return;


	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);

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
			msleep(table[i].count);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 1000);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write_ext(ctx, table[i].para_list, 
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
static struct lcm_pmic_info *g_pmic;
static unsigned int lcm_get_reg_vufs18(void)
{
	unsigned int volt = 0;

	if (regulator_is_enabled(g_pmic->reg_vufs18))
		/* regulator_get_voltage return volt with uV */
		volt = regulator_get_voltage(g_pmic->reg_vufs18) ;

	return volt;
}

static unsigned int lcm_get_reg_vmch3p0(void)
{
	unsigned int volt = 0;

	if (regulator_is_enabled(g_pmic->reg_vmch3p0))
		/* regulator_get_voltage return volt with uV */
		volt = regulator_get_voltage(g_pmic->reg_vmch3p0);

	return volt;
}
static unsigned int lcm_enable_reg_vufs18(int en)
{
	unsigned int ret = 0,volt = 0;
	if(en) {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vufs18)) {
			ret = regulator_enable(g_pmic->reg_vufs18);
			DISP_DEBUG("Enable the Regulator vufs1p8ret=%d.\n",ret);
			volt = lcm_get_reg_vufs18();
			DISP_DEBUG("get the Regulator vufs1p8 =%d.\n",volt);
		}
	}else {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vufs18)) {
			ret = regulator_disable(g_pmic->reg_vufs18);
			volt = lcm_get_reg_vufs18();
			DISP_DEBUG("disable the Regulator vufs1p8 ret=%d,volt=%d.\n",ret,volt);
		}

	}
	return ret;

}

static unsigned int lcm_enable_reg_vmch3p0(int en)
{
	unsigned int ret=0,volt = 0;
	if(en) {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vmch3p0)) {
			ret = regulator_enable(g_pmic->reg_vmch3p0);
			DISP_DEBUG("Enable the Regulator vmch3p0 ret=%d.\n",ret);
			volt = lcm_get_reg_vmch3p0();
			DISP_DEBUG("get the Regulator vmch3p0 =%d.\n",volt);
		}
	}else {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vmch3p0)) {
			ret = regulator_disable(g_pmic->reg_vmch3p0);
			volt = lcm_get_reg_vmch3p0();
			DISP_DEBUG("disable the Regulator vmch3p0 ret=%d,volt=%d.\n",ret,volt);
		}
	}
	return ret;

}
#endif

//#ifdef OPLUS_ADFR
static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		return -EINVAL;
	}

	m_vrefresh = drm_mode_vrefresh(m);

	if (m->hdisplay == FHD_LCM_WIDTH && m->vdisplay == FHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
			ret = FHD_SDC60;
		} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
			ret = FHD_SDC90;
		} else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
			ret = FHD_SDC120;
		} else if (m_vrefresh == 118 && m->hskew == OPLUS_ADFR) {
			ret = FHD_OPLUS120;
		}
	} else if (m->hdisplay == WQHD_LCM_WIDTH && m->vdisplay == WQHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
			ret = WQHD_SDC60;
		} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
			ret = WQHD_SDC90;
		} else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
			ret = WQHD_SDC120;
		} else if (m_vrefresh == 118 && m->hskew == OPLUS_ADFR) {
			ret = WQHD_OPLUS120;
		}
	}

	return ret;
}
//#endif

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	struct drm_display_mode *m = ctx->m;

	mode_id = get_mode_enum(m);

	switch (mode_id) {
	case FHD_SDC60:
		DISP_DEBUG("fhd_dsi_on_cmd_sdc60\n");
		push_table(ctx, fhd_dsi_on_cmd_sdc60, sizeof(fhd_dsi_on_cmd_sdc60)/sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC90:
		DISP_DEBUG("fhd_dsi_on_cmd_sdc90\n");
		push_table(ctx, fhd_dsi_on_cmd_sdc90, sizeof(fhd_dsi_on_cmd_sdc90)/sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC120:
		DISP_DEBUG("fhd_dsi_on_cmd_sdc120\n");
		push_table(ctx, fhd_dsi_on_cmd_sdc120, sizeof(fhd_dsi_on_cmd_sdc120)/sizeof(struct LCM_setting_table));
		break;
	case WQHD_SDC60:
		DISP_DEBUG("wqhd_dsi_on_cmd_sdc60\n");
		push_table(ctx, wqhd_dsi_on_cmd_sdc60, sizeof(wqhd_dsi_on_cmd_sdc60)/sizeof(struct LCM_setting_table));
		break;
	case WQHD_SDC90:
		DISP_DEBUG("wqhd_dsi_on_cmd_sdc90\n");
		push_table(ctx, wqhd_dsi_on_cmd_sdc90, sizeof(wqhd_dsi_on_cmd_sdc90)/sizeof(struct LCM_setting_table));
		break;
	case WQHD_SDC120:
		DISP_DEBUG("wqhd_dsi_on_cmd_sdc120\n");
		push_table(ctx, wqhd_dsi_on_cmd_sdc120, sizeof(wqhd_dsi_on_cmd_sdc120)/sizeof(struct LCM_setting_table));
		break;
	default:
		DISP_DEBUG(" default mode_id\n");
		push_table(ctx, fhd_dsi_on_cmd_sdc120, sizeof(fhd_dsi_on_cmd_sdc120)/sizeof(struct LCM_setting_table));
		break;
	}

//#ifdef OPLUS_ADFR
	/* add for adfr status reset */
	if (oplus_adfr_is_support()) {
		// reset adfr auto mode status as auto mode will be change after power on
		oplus_adfr_status_reset(NULL, m);
	}
//endif

	//send 1024 for 0x51 on backlight resume flow
	//lcm_dcs_write_seq_static(ctx,0x51,0x03,0xff);

	DISP_INFO("successful! mode_id=%d\n", mode_id);
}

static void lcm_panel_seed(struct lcm *ctx){


	if (seed_mode == 100) {
			push_table(ctx, lcm_seed_mode0, sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table));
	} else if (seed_mode == 101){
			push_table(ctx, lcm_seed_mode1, sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table));
	} else if (seed_mode == 102){
			push_table(ctx, lcm_seed_mode2, sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table));
	}
	DISP_INFO("seed_mode=%d successful!\n", seed_mode);

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


	if (!ctx->prepared) {
		return 0;
	}
	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(20000, 20100);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(110000, 110100);
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
	lcm_panel_seed(ctx);

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

	return 0;
}

#define HFP (40)
#define HSA (10)
#define HBP (20)
#define VFP (8)
#define VSA (8)
#define VBP (8)
#define VAC_WQHD (3216)
#define HAC_WQHD (1440)

#define VAC_FHD (2412)
#define HAC_FHD (1080)

//static u32 fake_heigh = 3216;
//static u32 fake_width = 1440;
//static bool need_fake_resolution;
static const struct drm_display_mode display_mode[MODE_NUM] = {
	//fhd_sdc_60_mode
	{
		.clock = 168084,
		.hdisplay = HAC_FHD,
		.hsync_start = HAC_FHD + HFP,
		.hsync_end = HAC_FHD + HFP + HSA,
		.htotal = HAC_FHD + HFP + HSA + HBP,
		.vdisplay = VAC_FHD,
		.vsync_start = VAC_FHD + VFP,
		.vsync_end = VAC_FHD + VFP + VSA,
		.vtotal = VAC_FHD + VFP + VSA + VBP,
		.hskew = 1,
	},
	//fhd_sdc_120_mode
	{
		.clock = 336168,
		.hdisplay = HAC_FHD,
		.hsync_start = HAC_FHD + HFP,
		.hsync_end = HAC_FHD + HFP + HSA,
		.htotal = HAC_FHD + HFP + HSA + HBP,
		.vdisplay = VAC_FHD,
		.vsync_start = VAC_FHD + VFP,
		.vsync_end = VAC_FHD + VFP + VSA,
		.vtotal = VAC_FHD + VFP + VSA + VBP,
		.hskew = 0,
	},
	//fhd_sdc_90_mode
	{
		.clock = 252126,
		.hdisplay = HAC_FHD,
		.hsync_start = HAC_FHD + HFP,
		.hsync_end = HAC_FHD + HFP + HSA,
		.htotal = HAC_FHD + HFP + HSA + HBP + 2,
		.vdisplay = VAC_FHD,
		.vsync_start = VAC_FHD + VFP,
		.vsync_end = VAC_FHD + VFP + VSA,
		.vtotal = VAC_FHD + VFP + VSA + VBP,
		.hskew = 0,
	},
	//fhd_oplus_120_mode
	{
		.clock = 331428,
		.hdisplay = HAC_FHD,
		.hsync_start = HAC_FHD + HFP,
		.hsync_end = HAC_FHD + HFP + HSA,
		.htotal = HAC_FHD + HFP + HSA + HBP + 3,
		.vdisplay = VAC_FHD,
		.vsync_start = VAC_FHD + VFP,
		.vsync_end = VAC_FHD + VFP + VSA,
		.vtotal = VAC_FHD + VFP + VSA + VBP,
		.hskew = 2,
	},
	//wqhd_sdc_60_mode
	{
		.clock = 292321,
		.hdisplay = HAC_WQHD,
		.hsync_start = HAC_WQHD + HFP,
		.hsync_end = HAC_WQHD + HFP + HSA,
		.htotal = HAC_WQHD + HFP + HSA + HBP,
		.vdisplay = VAC_WQHD,
		.vsync_start = VAC_WQHD + VFP,
		.vsync_end = VAC_WQHD + VFP + VSA,
		.vtotal = VAC_WQHD + VFP + VSA + VBP,
		.hskew = 1,
	},
	//wqhd_sdc_120_mode
	{
		.clock = 584646,
		.hdisplay = HAC_WQHD,
		.hsync_start = HAC_WQHD + HFP,
		.hsync_end = HAC_WQHD + HFP + HSA,
		.htotal = HAC_WQHD + HFP + HSA + HBP,
		.vdisplay = VAC_WQHD,
		.vsync_start = VAC_WQHD + VFP,
		.vsync_end = VAC_WQHD + VFP + VSA,
		.vtotal = VAC_WQHD + VFP + VSA + VBP,
		.hskew = 0,
	},
	//wqhd_sdc_90_mode
	{
		.clock = 438460,
		.hdisplay = HAC_WQHD,
		.hsync_start = HAC_WQHD + HFP,
		.hsync_end = HAC_WQHD + HFP + HSA,
		.htotal = HAC_WQHD + HFP + HSA + HBP + 2,
		.vdisplay = VAC_WQHD,
		.vsync_start = VAC_WQHD + VFP,
		.vsync_end = VAC_WQHD + VFP + VSA,
		.vtotal = VAC_WQHD + VFP + VSA + VBP,
		.hskew = 0,
	},
	//wqhd_oplus_120_mode
	{
		.clock = 576000,
		.hdisplay = HAC_WQHD,
		.hsync_start = HAC_WQHD + HFP,
		.hsync_end = HAC_WQHD + HFP + HSA,
		.htotal = HAC_WQHD + HFP + HSA + HBP + 3,
		.vdisplay = VAC_WQHD,
		.vsync_start = VAC_WQHD + VFP,
		.vsync_end = VAC_WQHD + VFP + VSA,
		.vtotal = VAC_WQHD + VFP + VSA + VBP,
		.hskew = 2,
	},
};

/*static const struct drm_display_mode default_mode = {
	.clock = 307152,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,
};*/

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params[MODE_NUM] = {
	//fhd_sdc_60_mode
		{
			.data_rate = 826,
			.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
			.cmd_null_pkt_en = 1,
			.cmd_null_pkt_len = 105,
			.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
			.dsc_params = {
				.enable = 1,
				.ver = 18,
				.slice_mode = 1,
				.rgb_swap = 0,
				.dsc_cfg = 2088,
				.rct_on = 1,
				.bit_per_channel = 10,
				.dsc_line_buf_depth = 11,
				.bp_enable = 1,
				.bit_per_pixel = 128,
				.pic_height = 2412,
				.pic_width = 1080,
				.slice_height = 36,
				.slice_width = 540,
				.chunk_size = 540,
				.xmit_delay = 512,
				.dec_delay = 571,
				.scale_value = 32,
				.increment_interval = 821,
				.decrement_interval = 7,
				.line_bpg_offset = 14,
				.nfl_bpg_offset = 820,
				.slice_bpg_offset = 724,
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
			.oplus_mode_switch_hs = 1,
			.oplus_fakeframe_cfg = 0,
			.oplus_fakeframe_deferred_time = 0, // period*0.7
			.oplus_autoon_cfg = 0,
			.oplus_autooff_cfg = 1,
			.oplus_minfps0_cfg = 1,
			.oplus_minfps1_cfg = 0,
			.dyn_fps = {
				.switch_en = 1, .vact_timing_fps = 60,
			},
			.vendor = "AMB670_SAMSUNG",
			.manufacture = "Samsung4095",
			.color_vivid_status = true,
			.color_srgb_status = true,
			.color_softiris_status = true,
			.color_dual_panel_status = false,
			.color_dual_brightness_status = true,
			.cust_esd_check = 1,
			.esd_check_enable = 1,
			.esd_check_multi = 1,
			.lcm_esd_check_table[0] = {
				.cmd = 0x0a,
				.count = 1,
				.para_list[0] = 0x9F,
				.mask_list[0] = 0x9D,
			},
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
			.oplus_ofp_need_keep_apart_backlight = false,
			.oplus_ofp_hbm_on_delay = 17,
			.oplus_ofp_pre_hbm_off_delay = 2,
			.oplus_ofp_hbm_off_delay = 9,
			.oplus_cmdq_pkt_set_event = 1,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
			.round_corner_en = 1,
			.corner_pattern_height = ROUND_CORNER_1K_H_TOP,
			.corner_pattern_height_bot = ROUND_CORNER_1K_H_BOT,
			.corner_pattern_tp_size_l = sizeof(top_rc_1k_pattern_l),
			.corner_pattern_lt_addr_l = (void *)top_rc_1k_pattern_l,
			.corner_pattern_tp_size_r =  sizeof(top_rc_1k_pattern_r),
			.corner_pattern_lt_addr_r = (void *)top_rc_1k_pattern_r,
#endif
		.phy_timcon = {
			.clk_trail = 14,
			.hs_trail = 11,
		},

		},
		//fhd_sdc_120_mode
		{
			.data_rate = 826,
			.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
			.cmd_null_pkt_en = 1,
			.cmd_null_pkt_len = 105,
			.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
			.dsc_params = {
				.enable = 1,
				.ver = 18,
				.slice_mode = 1,
				.rgb_swap = 0,
				.dsc_cfg = 2088,
				.rct_on = 1,
				.bit_per_channel = 10,
				.dsc_line_buf_depth = 11,
				.bp_enable = 1,
				.bit_per_pixel = 128,
				.pic_height = 2412,
				.pic_width = 1080,
				.slice_height = 36,
				.slice_width = 540,
				.chunk_size = 540,
				.xmit_delay = 512,
				.dec_delay = 571,
				.scale_value = 32,
				.increment_interval = 821,
				.decrement_interval = 7,
				.line_bpg_offset = 14,
				.nfl_bpg_offset = 820,
				.slice_bpg_offset = 724,
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
			.oplus_mode_switch_hs = 1,
			.oplus_fakeframe_cfg = 3,
			.oplus_fakeframe_deferred_time = 6, // period*0.7
			.oplus_autoon_cfg = 0,
			.oplus_autooff_cfg = 1,
			.oplus_minfps0_cfg = 1,
			.oplus_minfps1_cfg = 0,
			.dyn_fps = {
				.switch_en = 1, .vact_timing_fps = 120,
			},
			.vendor = "AMB670_SAMSUNG",
			.manufacture = "Samsung4095",
			.color_vivid_status = true,
			.color_srgb_status = true,
			.color_softiris_status = true,
			.color_dual_panel_status = false,
			.color_dual_brightness_status = true,
			.cust_esd_check = 1,
			.esd_check_enable = 1,
			.esd_check_multi = 1,
			.lcm_esd_check_table[0] = {
				.cmd = 0x0a,
				.count = 1,
				.para_list[0] = 0x9F,
				.mask_list[0] = 0x9D,
			},
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
			.oplus_ofp_need_keep_apart_backlight = true,
			.oplus_ofp_hbm_on_delay = 9,
			.oplus_ofp_pre_hbm_off_delay = 2,
			.oplus_ofp_hbm_off_delay = 9,
			.oplus_cmdq_pkt_set_event = 1,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
			.round_corner_en = 1,
			.corner_pattern_height = ROUND_CORNER_1K_H_TOP,
			.corner_pattern_height_bot = ROUND_CORNER_1K_H_BOT,
			.corner_pattern_tp_size_l = sizeof(top_rc_1k_pattern_l),
			.corner_pattern_lt_addr_l = (void *)top_rc_1k_pattern_l,
			.corner_pattern_tp_size_r =  sizeof(top_rc_1k_pattern_r),
			.corner_pattern_lt_addr_r = (void *)top_rc_1k_pattern_r,
#endif
			.phy_timcon = {
				.clk_trail = 14,
				.hs_trail = 11,
			},

		},
		//fhd_sdc_90_mode
		{
			.data_rate = 826,
			.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
			.cmd_null_pkt_en = 1,
			.cmd_null_pkt_len = 105,
			.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
			.dsc_params = {
				.enable = 1,
				.ver = 18,
				.slice_mode = 1,
				.rgb_swap = 0,
				.dsc_cfg = 2088,
				.rct_on = 1,
				.bit_per_channel = 10,
				.dsc_line_buf_depth = 11,
				.bp_enable = 1,
				.bit_per_pixel = 128,
				.pic_height = 2412,
				.pic_width = 1080,
				.slice_height = 36,
				.slice_width = 540,
				.chunk_size = 540,
				.xmit_delay = 512,
				.dec_delay = 571,
				.scale_value = 32,
				.increment_interval = 821,
				.decrement_interval = 7,
				.line_bpg_offset = 14,
				.nfl_bpg_offset = 820,
				.slice_bpg_offset = 724,
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
			.oplus_mode_switch_hs = 1,
			.oplus_fakeframe_cfg = 3,
			.oplus_fakeframe_deferred_time = 8, // period*0.7
			.oplus_autoon_cfg = 0,
			.oplus_autooff_cfg = 0,
			.oplus_minfps0_cfg = 0,
			.oplus_minfps1_cfg = 0,
			.dyn_fps = {
				.switch_en = 1, .vact_timing_fps = 90,
			},
			.vendor = "AMB670_SAMSUNG",
			.manufacture = "Samsung4095",
			.color_vivid_status = true,
			.color_srgb_status = true,
			.color_softiris_status = true,
			.color_dual_panel_status = false,
			.color_dual_brightness_status = true,
			.cust_esd_check = 1,
			.esd_check_enable = 1,
			.esd_check_multi = 1,
			.lcm_esd_check_table[0] = {
				.cmd = 0x0a,
				.count = 1,
				.para_list[0] = 0x9F,
				.mask_list[0] = 0x9D,
			},
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
			.oplus_ofp_need_keep_apart_backlight = true,
			.oplus_ofp_hbm_on_delay = 12,
			.oplus_ofp_pre_hbm_off_delay = 2,
			.oplus_ofp_hbm_off_delay = 12,
			.oplus_cmdq_pkt_set_event = 1,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
			.round_corner_en = 1,
			.corner_pattern_height = ROUND_CORNER_1K_H_TOP,
			.corner_pattern_height_bot = ROUND_CORNER_1K_H_BOT,
			.corner_pattern_tp_size_l = sizeof(top_rc_1k_pattern_l),
			.corner_pattern_lt_addr_l = (void *)top_rc_1k_pattern_l,
			.corner_pattern_tp_size_r =  sizeof(top_rc_1k_pattern_r),
			.corner_pattern_lt_addr_r = (void *)top_rc_1k_pattern_r,
#endif
		.phy_timcon = {
			.clk_trail = 14,
			.hs_trail = 11,
		},

		},
		//fhd_oa_120_mode
		{
			.data_rate = 826,
			.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
			.cmd_null_pkt_en = 1,
			.cmd_null_pkt_len = 105,
			.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
			.dsc_params = {
				.enable = 1,
				.ver = 18,
				.slice_mode = 1,
				.rgb_swap = 0,
				.dsc_cfg = 2088,
				.rct_on = 1,
				.bit_per_channel = 10,
				.dsc_line_buf_depth = 11,
				.bp_enable = 1,
				.bit_per_pixel = 128,
				.pic_height = 2412,
				.pic_width = 1080,
				.slice_height = 36,
				.slice_width = 540,
				.chunk_size = 540,
				.xmit_delay = 512,
				.dec_delay = 571,
				.scale_value = 32,
				.increment_interval = 821,
				.decrement_interval = 7,
				.line_bpg_offset = 14,
				.nfl_bpg_offset = 820,
				.slice_bpg_offset = 724,
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
			.dyn_fps = {
				.switch_en = 1, .vact_timing_fps = 120,
			},
			.vendor = "AMB670_SAMSUNG",
			.manufacture = "Samsung4095",
			.color_vivid_status = true,
			.color_srgb_status = true,
			.color_softiris_status = true,
			.color_dual_panel_status = false,
			.color_dual_brightness_status = true,
			.cust_esd_check = 1,
			.esd_check_enable = 1,
			.esd_check_multi = 1,
			.lcm_esd_check_table[0] = {
				.cmd = 0x0a,
				.count = 1,
				.para_list[0] = 0x9F,
				.mask_list[0] = 0x9D,
			},
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
			.oplus_ofp_need_keep_apart_backlight = true,
			.oplus_ofp_hbm_on_delay = 9,
			.oplus_ofp_pre_hbm_off_delay = 2,
			.oplus_ofp_hbm_off_delay = 9,
			.oplus_cmdq_pkt_set_event = 1,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
			.round_corner_en = 1,
			.corner_pattern_height = ROUND_CORNER_1K_H_TOP,
			.corner_pattern_height_bot = ROUND_CORNER_1K_H_BOT,
			.corner_pattern_tp_size_l = sizeof(top_rc_1k_pattern_l),
			.corner_pattern_lt_addr_l = (void *)top_rc_1k_pattern_l,
			.corner_pattern_tp_size_r =  sizeof(top_rc_1k_pattern_r),
			.corner_pattern_lt_addr_r = (void *)top_rc_1k_pattern_r,
#endif
		.phy_timcon = {
			.clk_trail = 14,
			.hs_trail = 11,
		},

		},

	//wqhd_sdc_60_mode
	{
		.data_rate = 1372,
		.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
		.cmd_null_pkt_en = 1,
		.cmd_null_pkt_len = 105,
		.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
		.dsc_params = {
			.enable = 1,
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,//PPS4[4]0x30 convert_rgb bit 4
			.dsc_cfg = 2088,
			.rct_on = 1,
			.bit_per_channel = 10,//pps3[7:4]bits_per_component 0xAB
			.dsc_line_buf_depth = 11,//PPS3[3:0] linebuf_depth 0xAB
			.bp_enable = 1,//PPS4[5] 0x30 block_pred_enable bit 4
			.bit_per_pixel = 128,//PPS4[1:0],PPS5[7:0]0x080
			.pic_height = 3216,//PPS6[7:0],PPS7[7:0]0xc90
			.pic_width = 1440,//PPS8[7:0],PPS9[7:0]0x5a0
			.slice_height = 67,//PPS10[7:0],PPS11[7:0]0x043
			.slice_width = 720,//PPS12[7:0],PPS13[7:0]0x2d0
			.chunk_size = 720,//PPS14[7:0],PPS15[7:0]0x2d0
			.xmit_delay = 512,//PPS16[1:0],PPS17[7:0]0x200
			.dec_delay = 706,//PPS18[7:0],PPS19[7:0]0x2c2
			.scale_value = 32,//PPS21[5:0]0x20
			.increment_interval = 1673,//PPS22[7:0],PPS23[7:0]0x689
			.decrement_interval = 10,//PPS24[3:0],PPS25[7:0]0x0a
			.line_bpg_offset = 15,//PPS27[4:0]0x0F
			.nfl_bpg_offset = 466,//PPS28[7:0],PPS29[7:0]0x1d2
			.slice_bpg_offset = 292,//PPS30[7:0],PPS31[7:0]0x124
			.initial_offset = 6144,//PPS32[7:0],PPS33[7:0]0x1800
			.final_offset = 4336,//PPS34[7:0],PPS35[7:0]0x10f0
			.flatness_minqp = 7,//PPS36[4:0]0x07
			.flatness_maxqp = 16,//PPS37[4:0]0x10
			.rc_model_size = 8192,//PPS38[7:0],PPS39[7:0]0x2000
			.rc_edge_factor = 6,//PPS40[3:0]0x06
			.rc_quant_incr_limit0 = 15,//PPS41[4:0]0x0F
			.rc_quant_incr_limit1 = 15,//PPS42[4:0]0x0F
			.rc_tgt_offset_hi = 3,//PPS43[7:4] 0x03
			.rc_tgt_offset_lo = 3,//PPS43[3:0] 0x03

		},
		.oplus_mode_switch_hs = 1,
		.oplus_fakeframe_cfg = 0,
		.oplus_fakeframe_deferred_time = 0, // period*0.7
		.oplus_autoon_cfg = 0,
		.oplus_autooff_cfg = 1,
		.oplus_minfps0_cfg = 1,
		.oplus_minfps1_cfg = 0,
		.dyn_fps = {
			.switch_en = 1, .vact_timing_fps = 60,
		},
		.vendor = "AMB670_SAMSUNG",
		.manufacture = "Samsung4095",
		.color_vivid_status = true,
		.color_srgb_status = true,
		.color_softiris_status = true,
		.color_dual_panel_status = false,
		.color_dual_brightness_status = true,
		.cust_esd_check = 1,
		.esd_check_enable = 1,
		.esd_check_multi = 1,
		.lcm_esd_check_table[0] = {
			.cmd = 0x0a,
			.count = 1,
			.para_list[0] = 0x9F,
			.mask_list[0] = 0x9D,
		},
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
		.oplus_ofp_need_keep_apart_backlight = false,
		.oplus_ofp_hbm_on_delay = 17,
		.oplus_ofp_pre_hbm_off_delay = 2,
		.oplus_ofp_hbm_off_delay = 9,
		.oplus_cmdq_pkt_set_event = 1,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		.round_corner_en = 1,
		.corner_pattern_height = ROUND_CORNER_2K_H_TOP,
		.corner_pattern_height_bot = ROUND_CORNER_2K_H_BOT,
		.corner_pattern_tp_size_l = sizeof(top_rc_2k_pattern_l),
		.corner_pattern_lt_addr_l = (void *)top_rc_2k_pattern_l,
		.corner_pattern_tp_size_r =  sizeof(top_rc_2k_pattern_r),
		.corner_pattern_lt_addr_r = (void *)top_rc_2k_pattern_r,
#endif
	},
	//wqhd_sdc_120_mode
	{
		.data_rate = 1372,
		.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
		.cmd_null_pkt_en = 1,
		.cmd_null_pkt_len = 105,
		.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
#if 1
		.dsc_params = {
	.enable = 1,
	.ver = 18,
	.slice_mode = 1,
	.rgb_swap = 0,//PPS4[4]0x30 convert_rgb bit 4
			.dsc_cfg = 2088,
	.rct_on = 1,
	.bit_per_channel = 10,//pps3[7:4]bits_per_component 0xAB
	.dsc_line_buf_depth = 11,//PPS3[3:0] linebuf_depth 0xAB
	.bp_enable = 1,//PPS4[5] 0x30 block_pred_enable bit 4
	.bit_per_pixel = 128,//PPS4[1:0],PPS5[7:0]0x080
	.pic_height = 3216,//PPS6[7:0],PPS7[7:0]0xc90
	.pic_width = 1440,//PPS8[7:0],PPS9[7:0]0x5a0
	.slice_height = 67,//PPS10[7:0],PPS11[7:0]0x043
	.slice_width = 720,//PPS12[7:0],PPS13[7:0]0x2d0
	.chunk_size = 720,//PPS14[7:0],PPS15[7:0]0x2d0
	.xmit_delay = 512,//PPS16[1:0],PPS17[7:0]0x200
	.dec_delay = 706,//PPS18[7:0],PPS19[7:0]0x2c2
	.scale_value = 32,//PPS21[5:0]0x20
	.increment_interval = 1673,//PPS22[7:0],PPS23[7:0]0x689
	.decrement_interval = 10,//PPS24[3:0],PPS25[7:0]0x0a
	.line_bpg_offset = 15,//PPS27[4:0]0x0F
	.nfl_bpg_offset = 466,//PPS28[7:0],PPS29[7:0]0x1d2
	.slice_bpg_offset = 292,//PPS30[7:0],PPS31[7:0]0x124
	.initial_offset = 6144,//PPS32[7:0],PPS33[7:0]0x1800
	.final_offset = 4336,//PPS34[7:0],PPS35[7:0]0x10f0
	.flatness_minqp = 7,//PPS36[4:0]0x07
	.flatness_maxqp = 16,//PPS37[4:0]0x10
	.rc_model_size = 8192,//PPS38[7:0],PPS39[7:0]0x2000
	.rc_edge_factor = 6,//PPS40[3:0]0x06
	.rc_quant_incr_limit0 = 15,//PPS41[4:0]0x0F
	.rc_quant_incr_limit1 = 15,//PPS42[4:0]0x0F
	.rc_tgt_offset_hi = 3,//PPS43[7:4] 0x03
	.rc_tgt_offset_lo = 3,//PPS43[3:0] 0x03

		},
#endif
		.oplus_mode_switch_hs = 1,
		.oplus_fakeframe_cfg = 3,
		.oplus_fakeframe_deferred_time = 6, // period*0.7
		.oplus_autoon_cfg = 0,
		.oplus_autooff_cfg = 1,
		.oplus_minfps0_cfg = 1,
		.oplus_minfps1_cfg = 0,
		.dyn_fps = {
			.switch_en = 1, .vact_timing_fps = 120,
		},
		.vendor = "AMB670_SAMSUNG",
		.manufacture = "Samsung4095",
		.color_vivid_status = true,
		.color_srgb_status = true,
		.color_softiris_status = true,
		.color_dual_panel_status = false,
		.color_dual_brightness_status = true,
		.cust_esd_check = 1,
		.esd_check_enable = 1,
		.esd_check_multi = 1,
		.lcm_esd_check_table[0] = {
			.cmd = 0x0a,
			.count = 1,
			.para_list[0] = 0x9F,
			.mask_list[0] = 0x9D,
		},
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
		.oplus_ofp_need_keep_apart_backlight = true,
		.oplus_ofp_hbm_on_delay = 9,
		.oplus_ofp_pre_hbm_off_delay = 2,
		.oplus_ofp_hbm_off_delay = 9,
		.oplus_cmdq_pkt_set_event = 1,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		.round_corner_en = 1,
		.corner_pattern_height = ROUND_CORNER_2K_H_TOP,
		.corner_pattern_height_bot = ROUND_CORNER_2K_H_BOT,
		.corner_pattern_tp_size_l = sizeof(top_rc_2k_pattern_l),
		.corner_pattern_lt_addr_l = (void *)top_rc_2k_pattern_l,
		.corner_pattern_tp_size_r =  sizeof(top_rc_2k_pattern_r),
		.corner_pattern_lt_addr_r = (void *)top_rc_2k_pattern_r,
#endif
	},
	//wqhd_sdc_90_mode
	{
		.data_rate = 1372,
		.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
		.cmd_null_pkt_en = 1,
		.cmd_null_pkt_len = 105,
		.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
#if 1
		.dsc_params = {
	.enable = 1,
	.ver = 18,
	.slice_mode = 1,
	.rgb_swap = 0,//PPS4[4]0x30 convert_rgb bit 4
			.dsc_cfg = 2088,
	.rct_on = 1,
	.bit_per_channel = 10,//pps3[7:4]bits_per_component 0xAB
	.dsc_line_buf_depth = 11,//PPS3[3:0] linebuf_depth 0xAB
	.bp_enable = 1,//PPS4[5] 0x30 block_pred_enable bit 4
	.bit_per_pixel = 128,//PPS4[1:0],PPS5[7:0]0x080
	.pic_height = 3216,//PPS6[7:0],PPS7[7:0]0xc90
	.pic_width = 1440,//PPS8[7:0],PPS9[7:0]0x5a0
	.slice_height = 67,//PPS10[7:0],PPS11[7:0]0x043
	.slice_width = 720,//PPS12[7:0],PPS13[7:0]0x2d0
	.chunk_size = 720,//PPS14[7:0],PPS15[7:0]0x2d0
	.xmit_delay = 512,//PPS16[1:0],PPS17[7:0]0x200
	.dec_delay = 706,//PPS18[7:0],PPS19[7:0]0x2c2
	.scale_value = 32,//PPS21[5:0]0x20
	.increment_interval = 1673,//PPS22[7:0],PPS23[7:0]0x689
	.decrement_interval = 10,//PPS24[3:0],PPS25[7:0]0x0a
	.line_bpg_offset = 15,//PPS27[4:0]0x0F
	.nfl_bpg_offset = 466,//PPS28[7:0],PPS29[7:0]0x1d2
	.slice_bpg_offset = 292,//PPS30[7:0],PPS31[7:0]0x124
	.initial_offset = 6144,//PPS32[7:0],PPS33[7:0]0x1800
	.final_offset = 4336,//PPS34[7:0],PPS35[7:0]0x10f0
	.flatness_minqp = 7,//PPS36[4:0]0x07
	.flatness_maxqp = 16,//PPS37[4:0]0x10
	.rc_model_size = 8192,//PPS38[7:0],PPS39[7:0]0x2000
	.rc_edge_factor = 6,//PPS40[3:0]0x06
	.rc_quant_incr_limit0 = 15,//PPS41[4:0]0x0F
	.rc_quant_incr_limit1 = 15,//PPS42[4:0]0x0F
	.rc_tgt_offset_hi = 3,//PPS43[7:4] 0x03
	.rc_tgt_offset_lo = 3,//PPS43[3:0] 0x03
		},
#endif
		.oplus_mode_switch_hs = 1,
		.oplus_fakeframe_cfg = 3,
		.oplus_fakeframe_deferred_time = 8, // period*0.7
		.oplus_autoon_cfg = 0,
		.oplus_autooff_cfg = 0,
		.oplus_minfps0_cfg = 0,
		.oplus_minfps1_cfg = 0,
		.dyn_fps = {
			.switch_en = 1, .vact_timing_fps = 90,
		},
		.vendor = "AMB670_SAMSUNG",
		.manufacture = "Samsung4095",
		.color_vivid_status = true,
		.color_srgb_status = true,
		.color_softiris_status = true,
		.color_dual_panel_status = false,
		.color_dual_brightness_status = true,
		.cust_esd_check = 1,
		.esd_check_enable = 1,
		.esd_check_multi = 1,
		.lcm_esd_check_table[0] = {
			.cmd = 0x0a,
			.count = 1,
			.para_list[0] = 0x9F,
			.mask_list[0] = 0x9D,
		},
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
		.oplus_ofp_need_keep_apart_backlight = true,
		.oplus_ofp_hbm_on_delay = 12,
		.oplus_ofp_pre_hbm_off_delay = 2,
		.oplus_ofp_hbm_off_delay = 12,
		.oplus_cmdq_pkt_set_event = 1,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		.round_corner_en = 1,
		.corner_pattern_height = ROUND_CORNER_2K_H_TOP,
		.corner_pattern_height_bot = ROUND_CORNER_2K_H_BOT,
		.corner_pattern_tp_size_l = sizeof(top_rc_2k_pattern_l),
		.corner_pattern_lt_addr_l = (void *)top_rc_2k_pattern_l,
		.corner_pattern_tp_size_r =  sizeof(top_rc_2k_pattern_r),
		.corner_pattern_lt_addr_r = (void *)top_rc_2k_pattern_r,
#endif
	},
	//wqhd_oa_120_mode
	{
		.data_rate = 1372,
		.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
		//.vfp_low_power = 4180,
		.cmd_null_pkt_en = 1,
		.cmd_null_pkt_len = 105,
		.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
#if 1
		.dsc_params = {
			.enable = 1,
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,//PPS4[4]0x30 convert_rgb bit 4
			.dsc_cfg = 2088,
			.rct_on = 1,
			.bit_per_channel = 10,//pps3[7:4]bits_per_component 0xAB
			.dsc_line_buf_depth = 11,//PPS3[3:0] linebuf_depth 0xAB
			.bp_enable = 1,//PPS4[5] 0x30 block_pred_enable bit 4
			.bit_per_pixel = 128,//PPS4[1:0],PPS5[7:0]0x080
			.pic_height = 3216,//PPS6[7:0],PPS7[7:0]0xc90
			.pic_width = 1440,//PPS8[7:0],PPS9[7:0]0x5a0
			.slice_height = 67,//PPS10[7:0],PPS11[7:0]0x043
			.slice_width = 720,//PPS12[7:0],PPS13[7:0]0x2d0
			.chunk_size = 720,//PPS14[7:0],PPS15[7:0]0x2d0
			.xmit_delay = 512,//PPS16[1:0],PPS17[7:0]0x200
			.dec_delay = 706,//PPS18[7:0],PPS19[7:0]0x2c2
			.scale_value = 32,//PPS21[5:0]0x20
			.increment_interval = 1673,//PPS22[7:0],PPS23[7:0]0x689
			.decrement_interval = 10,//PPS24[3:0],PPS25[7:0]0x0a
			.line_bpg_offset = 15,//PPS27[4:0]0x0F
			.nfl_bpg_offset = 466,//PPS28[7:0],PPS29[7:0]0x1d2
			.slice_bpg_offset = 292,//PPS30[7:0],PPS31[7:0]0x124
			.initial_offset = 6144,//PPS32[7:0],PPS33[7:0]0x1800
			.final_offset = 4336,//PPS34[7:0],PPS35[7:0]0x10f0
			.flatness_minqp = 7,//PPS36[4:0]0x07
			.flatness_maxqp = 16,//PPS37[4:0]0x10
			.rc_model_size = 8192,//PPS38[7:0],PPS39[7:0]0x2000
			.rc_edge_factor = 6,//PPS40[3:0]0x06
			.rc_quant_incr_limit0 = 15,//PPS41[4:0]0x0F
			.rc_quant_incr_limit1 = 15,//PPS42[4:0]0x0F
			.rc_tgt_offset_hi = 3,//PPS43[7:4] 0x03
			.rc_tgt_offset_lo = 3,//PPS43[3:0] 0x03
		},
#endif
		.dyn_fps = {
			.switch_en = 1, .vact_timing_fps = 120,
		},
		.vendor = "AMB670_SAMSUNG",
		.manufacture = "Samsung4095",
		.color_vivid_status = true,
		.color_srgb_status = true,
		.color_softiris_status = true,
		.color_dual_panel_status = false,
		.color_dual_brightness_status = true,
		.cust_esd_check = 1,
		.esd_check_enable = 1,
		.esd_check_multi = 1,
		.lcm_esd_check_table[0] = {
			.cmd = 0x0a,
			.count = 1,
			.para_list[0] = 0x9F,
			.mask_list[0] = 0x9D,
		},
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
		.oplus_ofp_need_keep_apart_backlight = true,
		.oplus_ofp_hbm_on_delay = 9,
		.oplus_ofp_pre_hbm_off_delay = 2,
		.oplus_ofp_hbm_off_delay = 9,
		.oplus_cmdq_pkt_set_event = 1,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		.round_corner_en = 1,
		.corner_pattern_height = ROUND_CORNER_2K_H_TOP,
		.corner_pattern_height_bot = ROUND_CORNER_2K_H_BOT,
		.corner_pattern_tp_size_l = sizeof(top_rc_2k_pattern_l),
		.corner_pattern_lt_addr_l = (void *)top_rc_2k_pattern_l,
		.corner_pattern_tp_size_r =  sizeof(top_rc_2k_pattern_r),
		.corner_pattern_lt_addr_r = (void *)top_rc_2k_pattern_r,
#endif
	},
};

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

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);

	m_vrefresh = drm_mode_vrefresh(m);
	DISP_DEBUG("mode:%d,m_vrefresh:%d\n",id,m_vrefresh);

	if (m->hdisplay == FHD_LCM_WIDTH && m->vdisplay == FHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
			*ext_param = &ext_params[0];
		}
		else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
			*ext_param = &ext_params[1];
		} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
			*ext_param = &ext_params[2];
		} else if (m_vrefresh == 118 && m->hskew == OPLUS_ADFR) {
			*ext_param = &ext_params[3];
		} else {
			*ext_param = &ext_params[0];
		}
	} else if (m->hdisplay == WQHD_LCM_WIDTH && m->vdisplay == WQHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
			*ext_param = &ext_params[4];
		}
		else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
			*ext_param = &ext_params[5];
		} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
			*ext_param = &ext_params[6];
		} else if (m_vrefresh == 118 && m->hskew == OPLUS_ADFR) {
			*ext_param = &ext_params[7];
		} else {
			*ext_param = &ext_params[4];
		}
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
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);

	DISP_DEBUG("mode:%d,m_vrefresh:%d\n",mode,m_vrefresh);

	if (m->hdisplay == FHD_LCM_WIDTH && m->vdisplay == FHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
			ext->params = &ext_params[0];
		}
		else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
			ext->params = &ext_params[1];
		} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
			ext->params = &ext_params[2];
		} else if (m_vrefresh == 118 && m->hskew == OPLUS_ADFR) {
			ext->params = &ext_params[3];
		} else {
			ext->params = &ext_params[0];
		}
	} else if (m->hdisplay == WQHD_LCM_WIDTH && m->vdisplay == WQHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
			ext->params = &ext_params[4];
		}
		else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
			ext->params = &ext_params[5];
		} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
			ext->params = &ext_params[6];
		} else if (m_vrefresh == 118 && m->hskew == OPLUS_ADFR) {
			ext->params = &ext_params[7];
		} else {
			ext->params = &ext_params[4];
		}
	}
	return ret;
}

//#ifdef OPLUS_ADFR
static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle)
{
	unsigned int i = 0;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		DISP_ERR("out of mtk_ddic_dsi_cmd \n");
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

static int panel_send_fake_frame(void *dsi, dcs_write_gce_pack cb, void *handle)
{
	unsigned int lcm_cmd_count = 0;

	lcm_cmd_count = sizeof(fakeframe_cmd) / sizeof(struct LCM_setting_table);
	panel_send_pack_hs_cmd(dsi, fakeframe_cmd, lcm_cmd_count, cb, handle);
	DISP_DEBUG("successful\n");
	return 0;
}

static int panel_set_auto_mode(void *dsi, struct drm_panel *panel, dcs_write_gce_pack cb, void *handle, 
	bool auto_en, struct drm_display_mode *m)
{
	unsigned int lcm_cmd_count = 0;
	int mode_id = 0;

	if (m == NULL) {
		ADFR_WARN("panel_set_auto_mode, mode=NULL\n");
		return -EINVAL;
	}

	mode_id = get_mode_enum(m);
	ADFR_INFO("auto_mode:%d,mode_id:%d\n", auto_en, mode_id);

	if (auto_en) {
		if (mode_id == FHD_SDC120 || mode_id == WQHD_SDC120) {
			/* send auto on cmd */
			lcm_cmd_count = sizeof(auto_on_cmd) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_on_cmd\n");
			panel_send_pack_hs_cmd(dsi, auto_on_cmd, lcm_cmd_count, cb, handle);
		}
	} else {
		if (mode_id == FHD_SDC60 || mode_id == FHD_SDC120 || mode_id == WQHD_SDC60 || mode_id == WQHD_SDC120) {
			/* send auto off cmd */
			lcm_cmd_count = sizeof(auto_off_cmd) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_off_cmd\n");
			panel_send_pack_hs_cmd(dsi, auto_off_cmd, lcm_cmd_count, cb, handle);
		}
	}

	return 0;
}

static int panel_set_minfps(void *dsi, struct drm_panel *panel, dcs_write_gce_pack cb, void *handle, 
	void *minfps, struct drm_display_mode *m)
{
	struct oplus_minfps *min_fps = (struct oplus_minfps *)minfps;
	unsigned int lcm_cmd_count = 0;
	int mode_id = 0;

	if ((m == NULL) || (minfps == NULL)) {
		ADFR_WARN("panel_set_minfps, mode or minfps is NULL\n");
		return -EINVAL;
	}

	mode_id = get_mode_enum(m);
	ADFR_INFO("minfps_flag:%d,extern_frame:%d,mode_id:%d\n",
		min_fps->minfps_flag, min_fps->extend_frame, mode_id);

	/*update the sdc min fps cmds */
	if (min_fps->minfps_flag == 0) {
		if (mode_id == FHD_SDC60 || mode_id == FHD_SDC120 || mode_id == WQHD_SDC60 || mode_id == WQHD_SDC120) {
			/* update manual min fps */
			auto_off_minfps_cmd[SDC_MANUAL_MIN_FPS_CMD_OFFSET].para_list[1] = min_fps->extend_frame;
			lcm_cmd_count = sizeof(auto_off_minfps_cmd) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_off_minfps_cmd\n");
			panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd, lcm_cmd_count, cb, handle);
		}
	} else if (min_fps->minfps_flag == 1) {
		if (mode_id == FHD_SDC120 || mode_id == WQHD_SDC120) {
			/* update auto min fps */
			auto_on_minfps_cmd[SDC_AUTO_MIN_FPS_CMD_OFFSET].para_list[1] = min_fps->extend_frame;
			lcm_cmd_count = sizeof(auto_on_minfps_cmd) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_on_minfps_cmd\n");
			panel_send_pack_hs_cmd(dsi, auto_on_minfps_cmd, lcm_cmd_count, cb, handle);
		}
	}

	return 0;
}

/* add for mux switch control */
static int panel_set_vsync_switch_gpio(struct drm_panel *panel, int vsync_mode)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (IS_ERR(ctx->te_switch_gpio)) {
		ADFR_ERR("invalid te_switch_gpio %ld\n",
			PTR_ERR(ctx->te_switch_gpio));
		return PTR_ERR(ctx->te_switch_gpio);
	}

	if (vsync_mode == OPLUS_VSYNC_SWITCH_TP) {
		gpiod_set_value(ctx->te_switch_gpio, OPLUS_VSYNC_SWITCH_TP);
	} else {
		gpiod_set_value(ctx->te_switch_gpio, OPLUS_VSYNC_SWITCH_TE);
	}

	ADFR_INFO("set te_switch_gpio to %d successful\n", vsync_mode);

	return 0;
}

static int mode_switch_hs(struct drm_panel *panel, struct drm_connector *connector,
		void *dsi_drv, unsigned int cur_mode, unsigned int dst_mode,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage, dcs_write_gce_pack cb)
{
	int ret = 0;
	int m_vrefresh = 0;
	int src_vrefresh = 0;
	unsigned int lcm_cmd_count = 0;
	/* lk mipi setting is 846 */
	static int last_data_rate = 826;
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct drm_display_mode *src_m = get_mode_by_id(connector, cur_mode);
	struct lcm *ctx = panel_to_lcm(panel);

	//pr_info("[lh]debug for samsung_amb670 lcm, mode_switch_hs:cur_mode=%d, dst_mode=%d,m->hskew=%d\n", cur_mode, dst_mode, m->hskew);
	if (cur_mode == dst_mode)
		return ret;

	if (stage == BEFORE_DSI_POWERDOWN) {
		m_vrefresh = drm_mode_vrefresh(m);
		src_vrefresh = drm_mode_vrefresh(src_m);
		DISP_INFO("mode_switch_hs,cur_mode:%d,dst_mode:%d,hdisplay:%d->%d,vrefresh:%d->%d,hskew:%d->%d\n",
			cur_mode, dst_mode, src_m->hdisplay, m->hdisplay, src_vrefresh, m_vrefresh, src_m->hskew, m->hskew);

		if (oplus_adfr_is_support()) {
			/* 1.send pre-switch cmd */
			ADFR_INFO("kVRR send pre-switch cmd\n");
			lcm_cmd_count = sizeof(pre_switch_cmd)/sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi_drv, pre_switch_cmd, lcm_cmd_count, cb, NULL);
			/* 2.set fakeframe off */
			/* 3.switch TE signal to TPV/TE according mode */
			oplus_adfr_vsync_switch(m, false);
		}

		/* 4.send timing switch cmd */
		/* if resolution changed, resend dsc params */
		if (m->hdisplay == WQHD_LCM_WIDTH && src_m->hdisplay == FHD_LCM_WIDTH) {
			DISP_INFO("send wqhd dsc param\n");
			push_table(ctx, wqhd_dsc_cmd, sizeof(wqhd_dsc_cmd)/sizeof(struct LCM_setting_table));
		} else if (m->hdisplay == FHD_LCM_WIDTH && src_m->hdisplay == WQHD_LCM_WIDTH) {
			DISP_INFO("send fhd dsc param\n");
			push_table(ctx, fhd_dsc_cmd, sizeof(fhd_dsc_cmd)/sizeof(struct LCM_setting_table));
		}

		if (m->hdisplay == WQHD_LCM_WIDTH && m->vdisplay == WQHD_LCM_HEIGHT) {
			if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
				if (m_vrefresh == 60) {
					//pr_info("timing switch to wqhdsdc60\n");
					lcm_cmd_count = sizeof(wqhd_timing_switch_1_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, wqhd_timing_switch_1_cmd_sdc60, lcm_cmd_count, cb, NULL);
					lcm_cmd_count = sizeof(wqhd_timing_switch_2_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, wqhd_timing_switch_2_cmd_sdc60, lcm_cmd_count, cb, NULL);
				} else if (m_vrefresh == 90) {
					//pr_info("timing switch to wqhdsdc90\n");
					lcm_cmd_count = sizeof(wqhd_timing_switch_1_cmd_sdc90) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, wqhd_timing_switch_1_cmd_sdc90, lcm_cmd_count, cb, NULL);
					lcm_cmd_count = sizeof(wqhd_timing_switch_2_cmd_sdc90) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, wqhd_timing_switch_2_cmd_sdc90, lcm_cmd_count, cb, NULL);
				} else if (m_vrefresh == 120) {
					//pr_info("timing switch to wqhdsdc120\n");
					lcm_cmd_count = sizeof(wqhd_timing_switch_1_cmd_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, wqhd_timing_switch_1_cmd_sdc120, lcm_cmd_count, cb, NULL);
					lcm_cmd_count = sizeof(wqhd_timing_switch_2_cmd_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, wqhd_timing_switch_2_cmd_sdc120, lcm_cmd_count, cb, NULL);
				}
			} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
				if (m_vrefresh == 118) {
					/* send OA120 timing-switch cmd */
				}
			}
		} else if (m->hdisplay == FHD_LCM_WIDTH && m->vdisplay == FHD_LCM_HEIGHT) {
			if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
				if (m_vrefresh == 60) {
					//pr_info("timing switch to fhdsdc60\n");
					lcm_cmd_count = sizeof(fhd_timing_switch_1_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_1_cmd_sdc60, lcm_cmd_count, cb, NULL);
					lcm_cmd_count = sizeof(fhd_timing_switch_2_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_2_cmd_sdc60, lcm_cmd_count, cb, NULL);
				} else if (m_vrefresh == 90) {
					//pr_info("timing switch to fhdsdc90\n");
					lcm_cmd_count = sizeof(fhd_timing_switch_1_cmd_sdc90) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_1_cmd_sdc90, lcm_cmd_count, cb, NULL);
					lcm_cmd_count = sizeof(fhd_timing_switch_2_cmd_sdc90) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_2_cmd_sdc90, lcm_cmd_count, cb, NULL);
				} else if (m_vrefresh == 120) {
					//pr_info("timing switch to fhdsdc120\n");
					lcm_cmd_count = sizeof(fhd_timing_switch_1_cmd_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_1_cmd_sdc120, lcm_cmd_count, cb, NULL);
					lcm_cmd_count = sizeof(fhd_timing_switch_2_cmd_sdc120) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_2_cmd_sdc120, lcm_cmd_count, cb, NULL);
				}
			} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
				if (m_vrefresh == 118) {
					/* send OA120 timing-switch cmd */
				}
			}
		}

		/* add for adfr status reset and fix garbage issue after resolution switching */
		if (oplus_adfr_is_support()) {
			// reset adfr auto mode status as panel mode will be change after timing switch
			oplus_adfr_status_reset(src_m, m);
		}
	} else if (stage == AFTER_DSI_POWERON) {
		ctx->m = m;
	}

	if (ext->params->data_rate != last_data_rate) {
		ret = 1;
		DISP_INFO("need to change mipi clk, data_rate=%d,last_data_rate=%d\n",ext->params->data_rate,last_data_rate);
		last_data_rate = ext->params->data_rate;
	}

	return ret;
}
//#endif

static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int seed_mode)
{
	int i = 0;

	if (seed_mode == 100) {
		for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode0[i].para_list, lcm_seed_mode0[i].count);
		}
	} else if (seed_mode == 101){
		for (i = 0; i < sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode1[i].para_list, lcm_seed_mode1[i].count);
		}
	} else if (seed_mode == 102){
		for (i = 0; i < sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode2[i].para_list, lcm_seed_mode2[i].count);
		}

	}
	DISP_INFO("seed_mode=%d successful \n", seed_mode);

	return 0;
}
static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
	unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned int notify_level = 0;
	char bl_tb0[] = {0x51, 0x03, 0xFF};
	char bl_tb1[] = {0x53, 0x28};

	if (!cb)
		return -EINVAL;

	mapped_level = level;
	esd_brightness = mapped_level;

	if (mapped_level > 1) {
		notify_level = mapped_level * 2047 / MAX_NORMAL_BRIGHTNESS;
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &notify_level);
	}

	if(mapped_level <= PANEL_MAX_NOMAL_BRIGHTNESS) { //2047
		if(hbm_brightness_flag == true) {
			bl_tb1[1] = 0x28;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			hbm_brightness_flag = false;
		}
		mapped_level = backlight_buf[mapped_level];
	} else if(mapped_level > HBM_BASE_600NIT) { //2749
		if(hbm_brightness_flag == false) {
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			hbm_brightness_flag = true;
		}
		mapped_level = backlight_600_800nit_buf[mapped_level - HBM_BASE_600NIT];
	}else if (mapped_level > PANEL_MAX_NOMAL_BRIGHTNESS) { //2047
		if(hbm_brightness_flag == true) {
			bl_tb1[1] = 0x28;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			hbm_brightness_flag = false;
		}
		mapped_level = backlight_500_600nit_buf[mapped_level - PANEL_MAX_NOMAL_BRIGHTNESS];
	}

	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;

	DISP_INFO("level:%d, backlight = %d,bl_tb0[1]=0x%x,bl_tb0[2]=0x%x\n", level, mapped_level,bl_tb0[1],bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

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
	DISP_INFO("esd_brightness=%d ,bl_tb0[1]=%x, bl_tb0[2]=%x\n", esd_brightness,bl_tb0[1], bl_tb0[2]);
	return 1;
}

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;

	if (!cb)
		return -EINVAL;

	OFP_INFO("oplus_display_brightness=%d, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

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
	}

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	int i = 0;

	if (!cb) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("oplus_display_brightness=%d, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		/* turn on loading effect */
		panel_set_seed(dsi, cb, handle, 100);

		for (i = 0; i < sizeof(hbm_on_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_on_cmd[i].para_list, hbm_on_cmd[i].count);
		}
		hbm_brightness_flag = true;
	} else if (en == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_off_cmd[i].para_list, hbm_off_cmd[i].count);
		}
		hbm_brightness_flag = false;
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);

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
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count, aod_off_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_off_cmd[i].para_list, aod_off_cmd[i].count);
		}
	}

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
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DISP_INFO("on=%d\n",on);

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
	usleep_range(3000,3100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(10000,10100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000,5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000,10100);
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
	//set vddi 1.8v
	ret = regulator_set_voltage(g_pmic->reg_vufs18,
				1800000, 1800000);
	ret=lcm_enable_reg_vufs18(1);

	usleep_range(5000, 5100);
	//enable aod_en gpio 3 to 1.3V
	gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	//enable 1.5V gpio148
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	usleep_range(1000, 1100);

	//set vddi 3.0v
	ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
				3000000, 3000000);
	ret=lcm_enable_reg_vmch3p0(1);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	DISP_INFO("Successful\n");
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
	usleep_range(2000, 2100);

	ret = regulator_set_voltage(g_pmic->reg_vufs18,
				0, 0);
	//disable vddi 1.5V
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	usleep_range(2000, 2100);
	gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	usleep_range(5000, 5100);
	//disable 1.8V
	ret=lcm_enable_reg_vufs18(0);

	usleep_range(5000, 5100);
	//set vddi 3.0v
	ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
				0, 0);
	ret=lcm_enable_reg_vmch3p0(0);

	hbm_brightness_flag = false;
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(70000, 70100);
	DISP_INFO("Successful\n");
	return 0;
}
static int lcm_set_power(unsigned int voltage_id, unsigned int voltage_value)
{
	int ret = 0;
	DISP_INFO(" voltage_id = %u, voltage_value = %u\n",
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
			if (voltage_value >= 1860000){
				voltage_value = 1800000;
			}
			ret = regulator_set_voltage(g_pmic->reg_vufs18,
				voltage_value, voltage_value);
			ret=lcm_enable_reg_vufs18(1);
			if (ret < 0) {
				DISP_ERR("error ret = %d", ret);
			}
		}
		break;
	/*case 2 represent vci*/
	case 2:
		    //set vcore 3.0v
		    if (voltage_value >= 3500000){

				voltage_value = 3000000;
			}
			ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
						voltage_value, voltage_value);
			ret=lcm_enable_reg_vmch3p0(1);

			if (ret < 0) {
				DISP_ERR("error ret = %d",ret);
			}
		break;
	default:
		DISP_ERR("error voltage id %d",voltage_id);
		break;
	}

	return ret;
}

/*voltage_id represent the PANEL_VOLTAGE_ENUM
  return the voltage_value of the voltage_id*/
static int lcm_update_power(uint32_t voltage_id)
{
	unsigned int voltage_max = 0;
	unsigned int voltage_current = 0;
	int ret = 0;


	switch(voltage_id) {
	/*0 represent the vddi*/
	case 0: {
			DISP_ERR("vddi not need to update\n");
			ret = -1;
		}
		break;
	/*case 1 represent vddr*/
	case 1: {
			voltage_max = lcm_get_reg_vufs18();
			if (voltage_max > 0){

				ret = 0;
			}else{
				ret = -1;
			}
		}
		break;
	/*case 2 represent vcore3.0*/
	case 2:{
			voltage_max = lcm_get_reg_vmch3p0();
			if (voltage_max > 0){
				ret = 0;
			}else{
				ret = -1;
			}
		}
		break;
	default:
		DISP_DEBUG("error voltage id %d", voltage_id);
		break;
	}
	panel_vol_bak[voltage_id].voltage_max = voltage_max;
	panel_vol_bak[voltage_id].voltage_min = voltage_max;
	panel_vol_bak[voltage_id].voltage_current = voltage_current;

	DISP_DEBUG("voltage_id = %d voltage_max=%d,voltage_current=%d \n", voltage_id,voltage_max,voltage_current);
	return ret;
}
/*#endif*/ /*OPLUS_BUG_STABILITY*/


static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_reset = lcm_panel_reset,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.oplus_set_power = lcm_set_power,
	.oplus_update_power_value = lcm_update_power,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
//#ifdef OPLUS_ADFR
	.mode_switch_hs = mode_switch_hs,
	.send_fake_fakeframe = panel_send_fake_frame,
	.set_auto_mode = panel_set_auto_mode,
	.set_minfps = panel_set_minfps,
	/* add for mux switch control */
	.set_vsync_switch =panel_set_vsync_switch_gpio,
//#endif
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	.set_hbm = lcm_set_hbm,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
	.set_seed = panel_set_seed,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
};
#endif

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
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
	connector->display_info.width_mm = 70;   //align x3 panel physical w/h
	connector->display_info.height_mm = 156;

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
	int rc = 0;
	u32 config = 0;
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	unsigned int fp_type = 0x08;
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				DISP_ERR("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			DISP_INFO("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		DISP_ERR("skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	g_pmic = kzalloc(sizeof(struct lcm_pmic_info), GFP_KERNEL);
	if (!g_pmic) {
		DISP_ERR("fail to alloc lcm_pmic_info (ENOMEM)\n");
		return -ENOMEM;
	}
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
	/*devm_gpiod_put(dev, ctx->reset_gpio);*/

//#ifdef OPLUS_ADFR
	ctx->te_switch_gpio = devm_gpiod_get(dev, "te_switch", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->te_switch_gpio)) {
		DISP_ERR("cannot get te_switch_gpio %ld\n",
			PTR_ERR(ctx->te_switch_gpio));
		return PTR_ERR(ctx->te_switch_gpio);
	}
	gpiod_set_value(ctx->te_switch_gpio, 0);
	//devm_gpiod_put(dev, ctx->te_switch_gpio);
//#endif

	//enable aod_en 1.3V
	ctx->aod1p3_enable_gpio = devm_gpiod_get(ctx->dev, "aod_en", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->aod1p3_enable_gpio)) {
		DISP_ERR("cannot get aod_en_enable_gpio %ld\n",
			PTR_ERR(ctx->aod1p3_enable_gpio));
		return PTR_ERR(ctx->aod1p3_enable_gpio);
	}

	gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	/*devm_gpiod_put(ctx->dev, ctx->aod1p3_enable_gpio);*/

	ctx->vddr1p5_enable_gpio = devm_gpiod_get(dev, "vddr1p5-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p5_enable_gpio)) {
		DISP_ERR(" cannot get vddr1p5_enable_gpio %ld\n",
			PTR_ERR(ctx->vddr1p5_enable_gpio));
		return PTR_ERR(ctx->vddr1p5_enable_gpio);
	}
	/*devm_gpiod_put(dev, ctx->vddr1p5_enable_gpio);*/
	g_pmic->reg_vufs18= regulator_get(dev, "1p8");
	if (IS_ERR(g_pmic->reg_vufs18)) {
		DISP_ERR("cannot get reg_vufs18 %ld\n",
			PTR_ERR(g_pmic->reg_vufs18));
		//return PTR_ERR(g_pmic->reg_vufs18);
	}
	ret=lcm_enable_reg_vufs18(1);

	g_pmic->reg_vmch3p0= regulator_get(dev, "3p0");
	if (IS_ERR(g_pmic->reg_vmch3p0)) {
		DISP_ERR("cannot get reg_vmch3p0 %ld\n",
			PTR_ERR(g_pmic->reg_vmch3p0));
		//return PTR_ERR(g_pmic->reg_vmch3p0);
	}
	msleep(10);
	ret=lcm_enable_reg_vmch3p0(1);

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
	dsi_panel_parse_panel_power_cfg(panel_vol_bak);

	rc = of_property_read_u32(dev->of_node, "oplus-adfr-config", &config);
	if (rc == 0) {
		oplus_adfr_config = config;
		DISP_INFO("config=%d, adfrconfig=%d\n", config, oplus_adfr_config);
	} else {
		oplus_adfr_config = 0;
		DISP_INFO("adfrconfig=%d\n", oplus_adfr_config);
	}
	register_device_proc("lcd", "ANB670_SAMSUNG", "Samsung4095");
	oplus_max_normal_brightness = SILKY_MAX_NORMAL_BRIGHTNESS;

	DISP_INFO("- lcm,amb670yf01,vdo,60hz, adfrconfig=%d\n", oplus_adfr_config);

	flag_silky_panel = BL_SETTING_DELAY_60HZ;

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	oplus_ofp_set_fp_type(&fp_type);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	DISP_INFO("Successful\n");

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
	    .compatible = "oplus21007_samsung_amb670yf01_qhd_dsi_cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus21007_samsung_amb670yf01_qhd_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("lianghao");
MODULE_DESCRIPTION("lcm amb670 yf01 Panel Driver");
MODULE_LICENSE("GPL v2");
