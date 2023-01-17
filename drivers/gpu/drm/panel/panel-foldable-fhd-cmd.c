// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
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

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_log.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
#define _DSC_ENABLE_      0

#if _DSC_ENABLE_
#define pll_lcm0 (260) //panel 1080*2400
#else
#define pll_lcm0 (260) //panel 1080*2400
#endif
#define pll_lcm1 (423) ////panel 2250*2088

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *avdden_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct regulator *vddio_1v8;                      /*power vddio 1.8v*/

	bool prepared;
	bool enabled;

	int pmode_id;
	struct list_head probed_modes;

	int error;
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
	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
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
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
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

static int lcm_panel_power_on(struct lcm *ctx)
{
	int ret = 0;

	dev_err(ctx->dev, "%s %d\n", __func__, __LINE__);

	if (IS_ERR_OR_NULL(ctx->vddio_1v8)) {
		dev_err(ctx->dev, "Regulator get failed vddio_1v8 \n");
	} else {
		ret = regulator_set_voltage(ctx->vddio_1v8, 1804000, 1804000);
		if (ret) {
			dev_err(ctx->dev, "Regulator set_vtg failed vddio_i2c rc = %d\n", ret);
			return -EPROBE_DEFER;
		}

		ret = regulator_set_load(ctx->vddio_1v8, 200000);
		if (ret < 0) {
			dev_err(ctx->dev, "Failed to set vddio_1v8 mode(rc:%d)\n", ret);
			return -EPROBE_DEFER;
		}
		ret = regulator_enable(ctx->vddio_1v8);
		if (ret) {
				dev_err(ctx->dev, "Regulator vddi_i2c enable failed ret = %d\n", ret);
				return -EPROBE_DEFER;
		}
	}
	dev_err(ctx->dev, "%s %d\n", __func__, __LINE__);
	gpiod_set_value(ctx->avdden_gpio, 1);
	msleep(12);
	return ret;
}

static int lcm_panel_power_off(struct lcm *ctx)
{
	int ret = 0;
	gpiod_set_value(ctx->avdden_gpio, 0);
	gpiod_set_value(ctx->reset_gpio, 0);
	dev_err(ctx->dev, "%s %d\n", __func__, __LINE__);
	if (IS_ERR_OR_NULL(ctx->vddio_1v8)) {
		dev_err(ctx->dev, "Regulator get failed vddio_1v8 \n");
	} else {
		dev_err(ctx->dev, "%s %d\n", __func__, __LINE__);
		ret = regulator_set_voltage(ctx->vddio_1v8, 1804000, 1804000);
		if (ret) {
			dev_err(ctx->dev, "Regulator set_vtg failed vddio_i2c rc = %d\n", ret);
			return -EPROBE_DEFER;
		}

		ret = regulator_set_load(ctx->vddio_1v8, 200000);
		if (ret < 0) {
			dev_err(ctx->dev, "Failed to set vddio_1v8 mode(rc:%d)\n", ret);
			return -EPROBE_DEFER;
		}
		dev_err(ctx->dev, "%s %d\n", __func__, __LINE__);
		ret = regulator_disable(ctx->vddio_1v8);
		if (ret) {
				dev_err(ctx->dev, "Regulator vddi_i2c enable failed ret = %d\n", ret);
				return -EPROBE_DEFER;
		}
	}

	dev_err(ctx->dev, "%s %d\n", __func__, __LINE__);

	msleep(12);
	return ret;
}


static void transsion_lcm0_panel_init(struct lcm *ctx)
{
	pr_info("%s\n", __func__);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	//msleep(30);
	//gpiod_set_value(ctx->reset_gpio, 1);
	msleep(1);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(12);
	gpiod_set_value(ctx->reset_gpio, 1);
//	udelay(5 * 1000);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(12);

	lcm_dcs_write_seq_static(ctx,0xFE,0x00);
	lcm_dcs_write_seq_static(ctx,0x53,0x20);
	lcm_dcs_write_seq_static(ctx,0x2A,0x00,0x08,0x01,0x85);
	lcm_dcs_write_seq_static(ctx,0x2B,0x00,0x00,0x02,0xCF);

	lcm_dcs_write_seq_static(ctx,0x35,0x00);
	lcm_dcs_write_seq_static(ctx,0x51,0xFF);

	lcm_dcs_write_seq_static(ctx,0x11);

	msleep(120);
	lcm_dcs_write_seq_static(ctx,0x29);	//display on
	msleep(10);
	lcm_dcs_write_seq_static(ctx,0x51,0xF0);
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

	pr_info("%s, panel_0 unprepare\n", __func__);

	lcm_dcs_write_seq_static(ctx, 0x28);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(120);
	lcm_dcs_write_seq_static(ctx, 0x4f, 0x01);
	msleep(120);

	lcm_panel_power_off(ctx);
	ctx->prepared = false;
	ctx->error = 0;
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret = 0;

	pr_info("%s\n", __func__);
	pr_info("ctx->prepared=%d\n", ctx->prepared);
	if (ctx->prepared)
		return 0;

	lcm_panel_power_on(ctx);

	transsion_lcm0_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
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

#define HFP (40)
#define HSA (2)
#define HBP (20)
#define VFP (22)
#define VSA (2)
#define VBP (24)

#define VAC_FHD (720)
#define HAC_FHD (382)

static struct drm_display_mode default_mode = {
	.clock		= 167670, //60Hz
	.hdisplay	= HAC_FHD,
	.hsync_start	= HAC_FHD + HFP,
	.hsync_end	= HAC_FHD + HFP + HSA,
	.htotal		= HAC_FHD + HFP + HSA + HBP,
	.vdisplay	= VAC_FHD,
	.vsync_start	= VAC_FHD + VFP,
	.vsync_end	= VAC_FHD + VFP + VSA,
	.vtotal		= VAC_FHD + VFP + VSA + VBP,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3];
	unsigned char id[3] = {0x00, 0x80, 0x00};
	ssize_t ret;

	pr_info("%s success\n", __func__);

	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0)
		pr_info("%s error\n", __func__);

	DDPINFO("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	DDPINFO("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);

	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xF9};

	if (!cb)
		return -1;

	level = level * 255 / 4095;
	if (level > 1 && level < 8)
		level = 8;
	DDPINFO("foldable lcm backlight func: %s level is %u\n", __func__, level);
	bl_tb0[1] = level & 0xFF;
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static struct mtk_panel_params ext_params = {
	.lcm_index = 0,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
			.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
		},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 388494,
	.physical_height_um = 732240,
#if _DSC_ENABLE_
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable                =  0,
		.ver                   =  17,
		.slice_mode            =  1,
		.rgb_swap              =  0,
		.dsc_cfg               =  34,
		.rct_on                =  1,
		.bit_per_channel       =  8,
		.dsc_line_buf_depth    =  9,
		.bp_enable             =  1,
		.bit_per_pixel         =  128,
		.pic_height            =  2400,
		.pic_width             =  1080,
		.slice_height          =  40,
		.slice_width           =  540,
		.chunk_size            =  540,
		.xmit_delay            =  512,
		.dec_delay             =  526,//796,
		.scale_value           =  32,
		.increment_interval    =  989,//1325,
		.decrement_interval    =  7,//15,
		.line_bpg_offset       =  12,
		.nfl_bpg_offset        =  631,
		.slice_bpg_offset      =  651,//326,
		.initial_offset        =  6144,
		.final_offset          =  4336,
		.flatness_minqp        =  3,
		.flatness_maxqp        =  12,
		.rc_model_size         =  8192,
		.rc_edge_factor        =  6,
		.rc_quant_incr_limit0  =  11,
		.rc_quant_incr_limit1  =  11,
		.rc_tgt_offset_hi      =  3,
		.rc_tgt_offset_lo      =  3,
	},
#endif
	.dyn_fps = {
		.switch_en = 0,
	},
	.data_rate = 520,
	.pll_clk = 260,
};

int convert_mode_id_to_pmode_id(struct drm_panel *panel,
	struct drm_connector *connector, unsigned int mode)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct drm_display_mode *m, *pmode;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			break;
		i++;
	}

	if (list_empty(&ctx->probed_modes)) {
		pr_info("ctx->probed_modes is empty\n");
		return -1;
	}

	i = 0;
	list_for_each_entry(pmode, &ctx->probed_modes, head) {
		if (drm_mode_equal(pmode, m))
			return i;
		i++;
	}

	return -1;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int pmode_id = convert_mode_id_to_pmode_id(panel, connector, mode);

	DDPMSG("%s mode:%d, pmode_id:%d\n", __func__, mode, pmode_id);

	switch (pmode_id) {
	case 0:
		ext->params = &ext_params;
		break;
	case 1:
		ext->params = &ext_params;
		break;
	case 2:
		ext->params = &ext_params;
		break;
	case 3:
		ext->params = &ext_params;
		break;
	default:
		ret = 1;
	}

	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
	struct drm_connector *connector,
	struct mtk_panel_params **ext_param,
	unsigned int mode)
{
	int ret = 0;
	int pmode_id = convert_mode_id_to_pmode_id(panel, connector, mode);

	DDPMSG("%s mode:%d, pmode_id:%d\n", __func__, mode, pmode_id);

	switch (pmode_id) {
	case 0:
		*ext_param = &ext_params;
		break;
	case 1:
		*ext_param = &ext_params;
		break;
	case 2:
		*ext_param = &ext_params;
		break;
	case 3:
		*ext_param = &ext_params;
		break;
	default:
		ret = 1;
	}

	if (*ext_param)
		pr_info("data_rate:%d\n", (*ext_param)->data_rate);
	else
		pr_info("ext_param is NULL;\n");

	return ret;
}

static void mode_switch_to_wqhd_60(struct drm_panel *panel,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
}

static void mode_switch_to_fhd120(struct drm_panel *panel,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
}

static void mode_switch_to_fhd90(struct drm_panel *panel,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
}

static void mode_switch_to_fhd60(struct drm_panel *panel,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct lcm *ctx = panel_to_lcm(panel);
	int pmode_id = 0;

	if (cur_mode == dst_mode)
		return ret;

	pmode_id = convert_mode_id_to_pmode_id(panel, connector, dst_mode);
	DDPMSG("%s cur_mode:%d, dst_mode:%d, pmode_id:%d\n", __func__,
		cur_mode, dst_mode, pmode_id);

	switch (pmode_id) {
	case 0:
		mode_switch_to_wqhd_60(panel, stage);
		break;
	case 1:
		mode_switch_to_fhd60(panel, stage);
		break;
	case 2:
		mode_switch_to_fhd90(panel, stage);
		break;
	case 3:
		mode_switch_to_fhd120(panel, stage);
		break;
	default:
		ret = 1;
	}

	if (stage == AFTER_DSI_POWERON)
		ctx->pmode_id = pmode_id;

	return ret;
}

#if 0
static bool update_lcm_id_by_mode(unsigned int dst_mode)
{
	unsigned int last_lcm_id;

	pr_info("%s\n", __func__);

	last_lcm_id = LCM_ID;
	switch (dst_mode) {
		case 0:
			LCM_ID = 0;
			break;
		case 1:
			LCM_ID = 1;
			break;
		default :
			pr_info("%s, map lcm id err, mode = %d\n", __func__, dst_mode);
			break;
	}
	panel_changed = last_lcm_id ^ LCM_ID;
	pr_info("%s, dst_mode = %d, LCM_ID = %d >> %d \n",
		__func__, dst_mode, last_lcm_id, LCM_ID);
	return panel_changed;
}
#endif

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.mode_switch = mode_switch,
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

static int lcm_get_modes(struct drm_panel *panel,
			struct drm_connector *connector)
{
	struct drm_display_mode *mode0;
	//struct drm_display_mode *mode1;
	//struct lcm *ctx = panel_to_lcm(panel);

	mode0 = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode0) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode0);
	mode0->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode0);

	connector->display_info.width_mm = 39;
	connector->display_info.height_mm = 73;

	return 2;
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
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	pr_info("%s-\n", __func__);

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
	dsi->lanes = 1;
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

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);


	ctx->vddio_1v8 = regulator_get(ctx->dev, "vddio_1v8");
	if (IS_ERR_OR_NULL(ctx->vddio_1v8)) {
		dev_err(dev, "Regulator get failed vddio_1v8 \n");
	}
	else {
		ret = regulator_set_voltage(ctx->vddio_1v8, 1804000, 1804000);
		if (ret) {
			dev_err(dev, "Regulator set_vtg failed vddio_i2c rc = %d\n", ret);
			return -EPROBE_DEFER;
		}

		ret = regulator_set_load(ctx->vddio_1v8, 200000);
		if (ret < 0) {
			dev_err(dev, "Failed to set vddio_1v8 mode(rc:%d)\n", ret);
			return -EPROBE_DEFER;
		}
		/*
		ret = regulator_disable(ctx->vddio_1v8);
		if (ret) {
				dev_err(ctx->dev, "Regulator vddi_i2c enable failed ret = %d\n", ret);
				return -EPROBE_DEFER;
		}
		*/
	}

	ctx->avdden_gpio = devm_gpiod_get(dev, "avdden", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->avdden_gpio)) {
		dev_err(dev, "%s: cannot get avdden-gpios %ld\n",
			__func__, PTR_ERR(ctx->avdden_gpio));
		return PTR_ERR(ctx->avdden_gpio);
	}
	devm_gpiod_put(dev, ctx->avdden_gpio);

#ifndef CONFIG_MTK_DISP_NO_LK
	//ctx->prepared = true;
	//ctx->enabled = true;
#endif

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0){
		drm_panel_remove(&ctx->panel);
		pr_info("%s, L-%d\n", __func__, __LINE__);
	}

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	INIT_LIST_HEAD(&ctx->probed_modes);
	register_device_proc("lcd1", "BF130_RM690C0", "BF130_255");

	pr_info("%s+\n", __func__);
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
	{ .compatible = "foldable,fhd,cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "foldable_fhd_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("MEDIATEK");
MODULE_DESCRIPTION("Alpha Foldable AMOLED FHD CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
