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
//#include "../mediatek/mediatek_v2/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#include "oplus22823_tm_nt37705_fhd_dsi_cmd.h"
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

#include "../mediatek/mediatek_v2/mtk_corner_pattern/data_hw_roundedpattern_r_boe.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/data_hw_roundedpattern_l_boe.h"
/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#include "../oplus/oplus_drm_disp_panel.h"
#define REGFLAG_CMD       0xFFFA
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2
#define FHD_LCM_WIDTH 1240
#define FHD_LCM_HEIGHT 2772
static unsigned int esd_brightness = 1023;
#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
extern unsigned int oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;

static unsigned int CurrentFrameRate = 0;
#define DRM_PANEL_EVENT_PWM_TURBO  0x14
DEFINE_MUTEX(oplus_pwm_lock);
extern unsigned int hpwm_90nit_set_temp;

static struct regulator *vmc_ldo;
static struct regulator *vrfio18_aif;

static unsigned int oplus_init_fps = 0;
static bool g_pwm_en = 0;
unsigned int oplus_bl_record = 0;
extern unsigned int last_backlight;
static unsigned int bl_record = 0;
extern bool pwm_power_on;
extern unsigned int hpwm_90nit_set_temp;
static bool high_pwm_status = false;

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle);
static int oplus_panel_osc_change(void *dsi, dcs_write_gce cb, void *handle, bool en);
extern void lcdinfo_notify(unsigned long val, void *v);
extern unsigned int oplus_enhance_mipi_strength;
extern inline bool pwm_turbo_support(void);
extern int lcm_id1;
extern int lcm_id2;

enum oplus_adfr_manual_tianma_min_fps_value {
  	OPLUS_ADFR_MANAUL_MIN_FPS_MAX = 0x00,
  	OPLUS_ADFR_MANAUL_MIN_FPS_60HZ = 0x01,
  	OPLUS_ADFR_MANAUL_MIN_FPS_40HZ = 0x02,
  	OPLUS_ADFR_MANAUL_MIN_FPS_30HZ = 0x03,
  	OPLUS_ADFR_MANAUL_MIN_FPS90_45HZ = 0x01,
  	OPLUS_ADFR_MANAUL_MIN_FPS90_30HZ = 0x02,
};


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
	struct drm_display_mode *m;
	struct gpio_desc *te_switch_gpio,*te_out_gpio;
	bool prepared;
	bool enabled;
	int error;
};
//extern bool g_is_silky_panel;

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

static int get_panel_es_ver(void)
{
	int ret = 0;
	DISP_DEBUG("lcm_id1=0x%x:lcm_id2=0x%x\n",lcm_id1,lcm_id2);

	if (lcm_id1 == 0x76 && lcm_id2 == 0x01) {
		ret = ES_T0;
	} else if (lcm_id1 == 0x76 && lcm_id2 == 0x02) {
		ret = ES1;
	} else if (lcm_id1 == 0x76 && lcm_id2 == 0x03) {
		ret = ES2;
	} else if (lcm_id1 == 0x76 && lcm_id2 == 0x04) {
		ret = ES3;
	} else if (lcm_id1 == 0x76 && lcm_id2 == 0x05) {
		ret = ES4;
	} else if (lcm_id1 == 0x76 && lcm_id2 == 0x06) {
		ret = ES5;
	}

	return ret;
}

static bool get_pwm_status(bool pwm_en)
{
	DISP_INFO("get_panel_es_ver=%d:pwm_en=%d\n",get_panel_es_ver(),pwm_en);

	if (( get_panel_es_ver() >= ES1) && pwm_en) {
		DISP_DEBUG("pwm_turbo get_pwm_status: true\n");
		return true;
	} else {
		DISP_DEBUG("pwm_turbo get_pwm_status: false\n");
		return false;
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

	DISP_DEBUG("+\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		//DISP_INFO("0x%08x\n",buffer[0] | (buffer[1] << 8));
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
static struct regulator *vrfio18_aif;
static int lcm_panel_vrfio18_aif_regulator_init(struct device *dev)
{
        static int regulator_vufs_inited;
        int ret = 0;

        if (regulator_vufs_inited)
                return ret;
        DISP_DEBUG("get lcm_panel_vrfio18_aif_regulator_init\n");

        /* please only get regulator once in a driver */
        vrfio18_aif = devm_regulator_get(dev, "1p8");
        if (IS_ERR(vrfio18_aif)) { /* handle return value */
                ret = PTR_ERR(vrfio18_aif);
                DISP_INFO("get vrfio18_aif fail, error: %d\n", ret);
                //return ret;
        }
        regulator_vufs_inited = 1;
        return ret; /* must be 0 */

}

static int lcm_panel_vrfio18_aif_enable(struct device *dev)
{
        int ret = 0;
        int retval = 0;
        int volt = 0 ;
        lcm_panel_vrfio18_aif_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vrfio18_aif)) {
		ret = regulator_set_voltage(vrfio18_aif, 1800000, 1800000);
		if (ret < 0)
			DISP_INFO("set voltage vrfio18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vrfio18_aif)) {
		//if(regulator_is_enabled(vrfio18_aif)) {
		//	return 0;
		//} else {
			regulator_enable(vrfio18_aif);
		//}
	}
	volt = regulator_get_voltage(vrfio18_aif);
	DISP_DEBUG("get lcm_panel_vrf18_enable,volt:%d\n",volt);

        return retval;
}

static int lcm_panel_vrfio18_aif_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;
	int volt = 0 ;

	lcm_panel_vrfio18_aif_regulator_init(dev);

	if (!IS_ERR_OR_NULL(vrfio18_aif)) {
		ret = regulator_set_voltage(vrfio18_aif, 0, 0);
		ret = regulator_disable(vrfio18_aif);
		if (ret < 0)
			DISP_INFO("disable regulator vrfio18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
	volt = regulator_get_voltage(vrfio18_aif);
	DISP_DEBUG("get lcm_panel_vrf18_disable\n");
	return ret;
}

static struct regulator *vmc_ldo;
static int lcm_panel_vmc_ldo_regulator_init(struct device *dev)
{
        static int regulator_vmc_inited;
        int ret = 0;

        if (regulator_vmc_inited)
                return ret;
    DISP_DEBUG("get lcm_panel_vmc_ldo_regulator_init\n");

        /* please only get regulator once in a driver */
        vmc_ldo = devm_regulator_get(dev, "3p0");
        if (IS_ERR(vmc_ldo)) { /* handle return value */
                ret = PTR_ERR(vmc_ldo);
                DISP_INFO("get vmc_ldo fail, error: %d\n", ret);
                //return ret;
        }
        regulator_vmc_inited = 1;
        return ret; /* must be 0 */

}

static int lcm_panel_vmc_ldo_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;
	int volt = 0;

	lcm_panel_vmc_ldo_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vmc_ldo)) {
		ret = regulator_set_voltage(vmc_ldo, 3000000, 3000000);
		if (ret < 0)
			DISP_INFO("set voltage vmc_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
		if (!IS_ERR_OR_NULL(vmc_ldo)) {
			//if(regulator_is_enabled(vmc_ldo)) {
				//return 0;
			//} else {
				regulator_enable(vmc_ldo);
			//}
		}
	volt = regulator_get_voltage(vmc_ldo);
	DISP_DEBUG("get lcm_panel_vmc_ldo_enable,volt: %d\n",volt);

        return retval;
}

static int lcm_panel_vmc_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;
	int volt = 0;

	lcm_panel_vmc_ldo_regulator_init(dev);

	if (!IS_ERR_OR_NULL(vmc_ldo)) {
		ret = regulator_set_voltage(vmc_ldo, 0, 0);
		ret = regulator_disable(vmc_ldo);

		if (ret < 0)
			DISP_INFO("disable regulator vmc_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	volt = regulator_get_voltage(vmc_ldo);
	DISP_DEBUG("get lcm_panel_vmc_ldo_disable\n");
	return ret;
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
			lcm_dcs_write(ctx, table[i].para_list, 
				table[i].count);
			break;
		}
	}
}

int oplus_set_dbv_frame(void *dsi, dcs_write_gce_pack cb, void *handle, bool enable)
{
	u8 avdd_base=72;
	u8 avdd_out=0;
	u8 avdd_shift=0;
	u8 elvss_target=0;
	u32 bl_lvl=0;

	mutex_lock(&oplus_pwm_lock);
	bl_lvl = oplus_display_brightness;
	if (enable == true){
		avdd_base=78;
	}
	if(bl_lvl<0x731)
		elvss_target=20;
	else if(bl_lvl<0x81E)
		elvss_target=21;
	else if(bl_lvl<0x90B)
		elvss_target=22;
	else if(bl_lvl==0x90B)
		elvss_target=23;
	else if(bl_lvl<0xA9B)
		elvss_target=24;
	else if(bl_lvl<0xB63)
		elvss_target=25;
	else if(bl_lvl<0xC2B)
		elvss_target=26;
	else if(bl_lvl<0xCF3)
		elvss_target=27;
	else if(bl_lvl<0xDBB)
		elvss_target=28;
	else if(bl_lvl==0xDBB)
		elvss_target=29;
	else if(bl_lvl<0xE56)
		elvss_target=33;
	else if(bl_lvl<0xE75)
		elvss_target=35;
	else if(bl_lvl<0xEB5)
		elvss_target=38;
	else if(bl_lvl<0xF15)
		elvss_target=42;
	else if(bl_lvl <= 0xFFF)
		elvss_target=49;

	avdd_shift = elvss_target-20;
	avdd_out = avdd_base + avdd_shift;
	dsi_switch_avdd[2].para_list[3] = avdd_out;
	DISP_INFO("pwm_turbo oplus_set_dbv_frame enable=%d, dsi_switch_avdd[2].para_list[3]=%d\n", enable, avdd_out);
	panel_send_pack_hs_cmd(dsi, dsi_switch_avdd, sizeof(dsi_switch_avdd) / sizeof(struct LCM_setting_table), cb, handle);


	mutex_unlock(&oplus_pwm_lock);
	return 0;
}

int oplus_display_panel_set_elvss(void *dsi, dcs_write_gce_pack cb, void *handle, bool en_h_pwm)
{
	mutex_lock(&oplus_pwm_lock);

	if(en_h_pwm == true)
		panel_send_pack_hs_cmd(dsi, dsi_high_f_switch_120Hz, sizeof(dsi_high_f_switch_120Hz) / sizeof(struct LCM_setting_table), cb, handle);
	else
		panel_send_pack_hs_cmd(dsi, dsi_low_f_switch_120Hz, sizeof(dsi_low_f_switch_120Hz) / sizeof(struct LCM_setting_table), cb, handle);
	usleep_range(8200, 8300);

	if (get_pwm_status(en_h_pwm)) {
		if (oplus_display_brightness <= 0x643) {
			DISP_INFO("pwm_turbo dsi_high_12plus backlight level:%d\n",oplus_display_brightness);
			dsi_switch_elvss[5].para_list[1] = 0x4B;
			dsi_switch_elvss[8].para_list[1] = 0xD2;
		} else if (oplus_display_brightness > 0x643) {
			DISP_INFO("pwm_turbo dsi_low_3plus backlight level:%d\n",oplus_display_brightness);
			dsi_switch_elvss[5].para_list[1] = 0x42;
			dsi_switch_elvss[8].para_list[1] = 0xB2;
		}
	} else {
		DISP_INFO("pwm_turbo closed, set dsi_low_3plus backlight level:%d\n", oplus_display_brightness);
		dsi_switch_elvss[5].para_list[1] = 0x42;
		dsi_switch_elvss[8].para_list[1] = 0xB2;
	}

	dsi_switch_elvss[9].para_list[1] = oplus_display_brightness >> 8;
	dsi_switch_elvss[9].para_list[2] = oplus_display_brightness & 0xFF;
	panel_send_pack_hs_cmd(dsi, dsi_switch_elvss, sizeof(dsi_switch_elvss) / sizeof(struct LCM_setting_table), cb, handle);

	DISP_DEBUG("pwm_turbo lcdinfo_notify 0x14 en_h_pwm=%d\n",en_h_pwm);
	lcdinfo_notify(DRM_PANEL_EVENT_PWM_TURBO, &en_h_pwm);
	g_pwm_en = en_h_pwm;
	mutex_unlock(&oplus_pwm_lock);

	return 0;
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		return -EINVAL;
	}

	m_vrefresh = drm_mode_vrefresh(m);

    if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
    	ret = FHD_SDC60;
    } else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
    	ret = FHD_SDC90;
    } else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
    	ret = FHD_SDC120;
    } else if (m_vrefresh == 120 && m->hskew == OPLUS_ADFR) {
    	ret = FHD_OPLUS120;
    }

	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	struct drm_display_mode *m = ctx->m;

	mode_id = get_mode_enum(m);

	switch (mode_id) {
	case FHD_SDC60:

		if (get_pwm_status(g_pwm_en)) {
			DISP_DEBUG("fhd_dsi_on_cmd_high_pwm_sdc60\n");
			push_table(ctx, dsi_on_cmd_high_pwm_sdc60, sizeof(dsi_on_cmd_high_pwm_sdc60) / sizeof(struct LCM_setting_table));
		} else {
			DISP_DEBUG("fhd_dsi_on_cmd_sdc60\n");
			if (lcm_id2 <= 1) {
				push_table(ctx, dsi_on_cmd_sdc60_id02, sizeof(dsi_on_cmd_sdc60_id02) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc60_id03, sizeof(dsi_on_cmd_sdc60_id03) / sizeof(struct LCM_setting_table));
			}
		}
		break;
	case FHD_SDC90:
		DISP_DEBUG("fhd_dsi_on_cmd_sdc90\n");
		if (lcm_id2 <= 1) {
			push_table(ctx, dsi_on_cmd_sdc90_id02, sizeof(dsi_on_cmd_sdc90_id02) / sizeof(struct LCM_setting_table));
		} else {
			push_table(ctx, dsi_on_cmd_sdc90_id03, sizeof(dsi_on_cmd_sdc90_id03) / sizeof(struct LCM_setting_table));
		}
		break;
	case FHD_SDC120:
		if (get_pwm_status(g_pwm_en)) {
			DISP_DEBUG("fhd_dsi_on_cmd_high_pwm_sdc120\n");
			push_table(ctx, dsi_on_cmd_high_pwm_sdc120, sizeof(dsi_on_cmd_high_pwm_sdc120) / sizeof(struct LCM_setting_table));
		} else {
			DISP_DEBUG("fhd_dsi_on_cmd_sdc120\n");
			if (lcm_id2 <= 1) {
				push_table(ctx, dsi_on_cmd_sdc120_id02, sizeof(dsi_on_cmd_sdc120_id02) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc120_id03, sizeof(dsi_on_cmd_sdc120_id03) / sizeof(struct LCM_setting_table));
			}
		}
		break;
	case FHD_OPLUS120:
		if (get_pwm_status(g_pwm_en)) {
			DISP_DEBUG("default mode_id dsi_on_cmd_high_pwm_oa120\n");
			push_table(ctx, dsi_on_cmd_high_pwm_oa120, sizeof(dsi_on_cmd_high_pwm_oa120) / sizeof(struct LCM_setting_table));
		} else {
			DISP_DEBUG("fhd_dsi_on_cmd_oplus120\n");
			if (lcm_id2 <= 1) {
				push_table(ctx, dsi_on_cmd_oa120_id02, sizeof(dsi_on_cmd_oa120_id02) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_oa120_id03, sizeof(dsi_on_cmd_oa120_id03) / sizeof(struct LCM_setting_table));
			}
		}
		break;
	default:
		if (get_pwm_status(g_pwm_en)) {
			DISP_DEBUG("default mode_id dsi_on_cmd_high_pwm_sdc120\n");
			push_table(ctx, dsi_on_cmd_high_pwm_sdc120, sizeof(dsi_on_cmd_high_pwm_sdc120) / sizeof(struct LCM_setting_table));
		} else {
			DISP_DEBUG(" default mode_id\n");
			if (lcm_id2 <= 1) {
				push_table(ctx, dsi_on_cmd_sdc120_id02, sizeof(dsi_on_cmd_sdc120_id02) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc120_id03, sizeof(dsi_on_cmd_sdc120_id03) / sizeof(struct LCM_setting_table));
			}
		}
		break;
	}

	/* add for adfr status reset */
	if (oplus_adfr_is_support()) {
		// reset adfr auto mode status as auto mode will be change after power on
		oplus_adfr_status_reset(NULL, m);
	}

	/* init pwm backlight */
	if (get_pwm_status(g_pwm_en)) {
		oplus_init_fps = mode_id;
		oplus_bl_record = 0;
		pwm_power_on = true;
	}
	DISP_INFO("successful! mode_id=%d\n", oplus_init_fps);
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
	int vrefresh_rate = 0;
	DISP_ERR("prepared=%d\n",ctx->prepared);

	if (!ctx->prepared)
		return 0;

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	if (oplus_ofp_get_aod_state() == true) {
		if (get_pwm_status(g_pwm_en)) {
			if (vrefresh_rate == 60) {
				push_table(ctx, aod_off_cmd_hpwm_60hz, sizeof(aod_off_cmd_hpwm_60hz) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, aod_off_cmd_hpwm_120hz, sizeof(aod_off_cmd_hpwm_120hz) / sizeof(struct LCM_setting_table));
			}
			if (oplus_display_brightness <= 0x643) {
				DISP_INFO("pwm_turbo dsi_high_12plus backlight level=%d\n", oplus_display_brightness);
				push_table(ctx, dsi_high_12plus, sizeof(dsi_high_12plus) / sizeof(struct LCM_setting_table));
			} else if (oplus_display_brightness > 0x643) {
				DISP_INFO("pwm_turbo dsi_low_3plus backlight level=%d\n", oplus_display_brightness);
				push_table(ctx, dsi_low_3plus, sizeof(dsi_low_3plus) / sizeof(struct LCM_setting_table));
			}
		} else {
			if (vrefresh_rate == 60) {
				push_table(ctx, aod_off_cmd_lpwm_60hz, sizeof(aod_off_cmd_lpwm_60hz) / sizeof(struct LCM_setting_table));
			} else if (vrefresh_rate == 120) {
				push_table(ctx, aod_off_cmd_lpwm_120hz, sizeof(aod_off_cmd_lpwm_120hz) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, aod_off_cmd_lpwm_90hz, sizeof(aod_off_cmd_lpwm_90hz) / sizeof(struct LCM_setting_table));
			}
		}
		usleep_range(9000, 9100);
		DISP_INFO("send aod off cmd\n");
	}

	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(20000, 20100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
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

	DISP_DEBUG("prepared=%d\n",ctx->prepared);
	if (ctx->prepared)
		return 0;

	// lcd reset H -> L -> H
	usleep_range(10000, 10100);
	//ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if(IS_ERR(ctx->reset_gpio)){
		DISP_ERR("cannot get reset-gpios %ld\n",PTR_ERR(ctx->reset_gpio));
	}
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->reset_gpio, 1);
	//msleep(10);
	//devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(20000, 20100);

	lcm_panel_init(ctx);
	usleep_range(2000, 2100);

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

static const struct drm_display_mode display_mode[MODE_NUM] = {
	//sdc_120_mode
	{
		.clock = 436550,
		.hdisplay = 1240,
		.hsync_start = 1240 + 9,//HFP
		.hsync_end = 1240 + 9 + 2,//HSA
		.htotal = 1240 + 9 + 2 + 21,//HBP
		.vdisplay = 2772,
		.vsync_start = 2772 + 52,//VFP
		.vsync_end = 2772 + 52 + 14,//VSA
		.vtotal = 2772 + 52+ 14 + 22,//VBP
		.hskew = SDC_ADFR,
	},
	//sdc_60_mode
	{
		.clock = 218275,
		.hdisplay = 1240,
		.hsync_start = 1240 + 9,//HFP
		.hsync_end = 1240 + 9 + 2,//HSA
		.htotal = 1240 + 9 + 2 + 21,//HBP
		.vdisplay = 2772,
		.vsync_start = 2772 + 52,//VFP
		.vsync_end = 2772 + 52 + 14,//VSA
		.vtotal = 2772 + 52+ 14 + 22,//VBP
		.hskew = SDC_MFR,
	},
	//sdc_90_mode
	{
		.clock = 327412,
		.hdisplay = 1240,
		.hsync_start = 1240 + 9,//HFP
		.hsync_end = 1240 + 9 + 2,//HSA
		.htotal = 1240 + 9 + 2 + 21,//HBP
		.vdisplay = 2772,
		.vsync_start = 2772 + 52,//VFP
		.vsync_end = 2772 + 52 + 14,//VSA
		.vtotal = 2772 + 52+ 14 + 22,//VBP
		.hskew = SDC_ADFR,
	},
	//oa_120_mode
	{
		.clock = 436550,
		.hdisplay = 1240,
		.hsync_start = 1240 + 9,//HFP
		.hsync_end = 1240 + 9 + 2,//HSA
		.htotal = 1240 + 9 + 2 + 21,//HBP
		.vdisplay = 2772,
		.vsync_start = 2772 + 52,//VFP
		.vsync_end = 2772 + 52 + 14,//VSA
		.vtotal = 2772 + 52+ 14 + 22,//VBP
		.hskew = OPLUS_ADFR,
	},
};

#if defined(CONFIG_MTK_PANEL_EXT)

static struct mtk_panel_params ext_params[MODE_NUM] = {
	//fhd_sdc_120_mode
{
	.pll_clk = 556,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
		.cmd = 0x91, .count = 1, .para_list[0] = 0xAB,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
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
	.vendor = "22823_Tianma_NT37705",
	.manufacture = "Tianma4095",
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
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,
			.dsc_cfg = 40,
			.rct_on = 1,
			.bit_per_channel = 10,
			.dsc_line_buf_depth = 11,
			.bp_enable = 1,
			.bit_per_pixel = 128,
			.pic_height = 2772,
			.pic_width = 1240,
			.slice_height = 12,
			.slice_width = 620,
			.chunk_size = 620,
			.xmit_delay = 512,
			.dec_delay = 593,
			.scale_value = 32,
			.increment_interval = 294,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 2421,
			.slice_bpg_offset = 1887,
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
			.dsc_cfg_change = 1,
		},
	.data_rate = 1112,
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 8,
	.oplus_ofp_pre_hbm_off_delay = 3,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
	},
	.skip_unnecessary_switch = true,
	.prete_offset = 233,
	},
	//fhd_sdc_60_mode
	{
	.pll_clk = 556,
	.phy_timcon = {
            .hs_trail = 14,
            .clk_trail = 15,
        },
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
		.cmd = 0x91, .count = 1, .para_list[0] = 0xAB,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
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
	.cmd_null_pkt_len = 400,
	.vendor = "22823_Tianma_NT37705",
	.manufacture = "Tianma4095",
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
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,
			.dsc_cfg = 40,
			.rct_on = 1,
			.bit_per_channel = 10,
			.dsc_line_buf_depth = 11,
			.bp_enable = 1,
			.bit_per_pixel = 128,
			.pic_height = 2772,
			.pic_width = 1240,
			.slice_height = 12,
			.slice_width = 620,
			.chunk_size = 620,
			.xmit_delay = 512,
			.dec_delay = 593,
			.scale_value = 32,
			.increment_interval = 294,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 2421,
			.slice_bpg_offset = 1887,
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
			.dsc_cfg_change = 1,
		},
	.data_rate = 1112,
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 2,
	.oplus_ofp_aod_off_black_frame_total_time = 59,
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
                .switch_en = 1, .vact_timing_fps = 60,
        },
	.skip_unnecessary_switch = true,
	.prete_offset = 466,
	.first_prete_delay_time = 14000,
	},
	//fhd 90hz
{
	.pll_clk = 414,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
		.cmd = 0x91, .count = 1, .para_list[0] = 0xAB,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
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
	.vendor = "22823_Tianma_NT37705",
	.manufacture = "Tianma4095",
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
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,
			.dsc_cfg = 40,
			.rct_on = 1,
			.bit_per_channel = 10,
			.dsc_line_buf_depth = 11,
			.bp_enable = 1,
			.bit_per_pixel = 128,
			.pic_height = 2772,
			.pic_width = 1240,
			.slice_height = 12,
			.slice_width = 620,
			.chunk_size = 620,
			.xmit_delay = 512,
			.dec_delay = 593,
			.scale_value = 32,
			.increment_interval = 294,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 2421,
			.slice_bpg_offset = 1887,
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
			.dsc_cfg_change = 1,
		},
	.data_rate = 828,
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 5,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 45,
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
	},
	.skip_unnecessary_switch = true,
	.prete_offset = 211,
	},
	//fhd_oa_120_mode
	{
	.pll_clk = 556,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_multi = 1,
	.lcm_esd_check_table[0] = {
				.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .mask_list[0] = 0xDC,
				.cmd = 0x91, .count = 1, .para_list[0] = 0xAB,
        },
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
			.round_corner_en = 0,
			.corner_pattern_height = ROUND_CORNER_1K_H_TOP,
			.corner_pattern_height_bot = ROUND_CORNER_1K_H_BOT,
			.corner_pattern_tp_size_l = sizeof(top_rc_1k_pattern_l),
			.corner_pattern_lt_addr_l = (void *)top_rc_1k_pattern_l,
			.corner_pattern_tp_size_r =  sizeof(top_rc_1k_pattern_r),
			.corner_pattern_lt_addr_r = (void *)top_rc_1k_pattern_r,
#endif
	.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
//	.color_oplus_calibrate_status = true,
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
	.vendor = "22823_Tianma_NT37705",
	.manufacture = "Tianma4095",
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
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,
			.dsc_cfg = 40,
			.rct_on = 1,
			.bit_per_channel = 10,
			.dsc_line_buf_depth = 11,
			.bp_enable = 1,
			.bit_per_pixel = 128,
			.pic_height = 2772,
			.pic_width = 1240,
			.slice_height = 12,
			.slice_width = 620,
			.chunk_size = 620,
			.xmit_delay = 512,
			.dec_delay = 593,
			.scale_value = 32,
			.increment_interval = 294,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 2421,
			.slice_bpg_offset = 1887,
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
			.dsc_cfg_change = 1,
		},
	.data_rate = 1112,
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 8,
	.oplus_ofp_pre_hbm_off_delay = 3,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
	},
	.skip_unnecessary_switch = true,
	},
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}
#if 0
static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	unsigned int mapped_level = 0;
	char bl_tb0[] = {0x51, 0x07, 0xFF};

	if (level > 4095)
		level = 4095;

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
#endif
#if 1

void oplus_display_panel_set_frequency_pwm(void *dsi, dcs_write_gce cb, void *handle, unsigned int bl_lvl)
{
	unsigned int i = 0;
	if ((bl_lvl <= 0x643 && bl_record > 0x643) || (pwm_power_on == true && bl_lvl <= 0x643)) {
		DISP_INFO("pwm_turbo dsi_high_12plus backlight level=%d, bl_record=%d\n", bl_lvl, bl_record);
		for (i = 0; i < sizeof(dsi_high_12plus)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, dsi_high_12plus[i].para_list, dsi_high_12plus[i].count);
		}
		bl_record = bl_lvl;
		pwm_power_on = false;
		hpwm_90nit_set_temp = 1;
	} else if ((bl_lvl > 0x643 && bl_record <= 0x643) || (pwm_power_on == true && bl_lvl > 0x643)) {
		DISP_INFO("pwm_turbo dsi_low_3plus backlight level=%d, bl_record=%d\n", bl_lvl, bl_record);
		for (i = 0; i < sizeof(dsi_low_3plus)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, dsi_low_3plus[i].para_list, dsi_low_3plus[i].count);
		}
		bl_record = bl_lvl;
		pwm_power_on = false;
		hpwm_90nit_set_temp = 2;
	}
}

static int oplus_display_panel_set_pwm_turbo(struct drm_panel *panel, void *dsi, dcs_write_gce_pack cb, void *handle, bool en_h_pwm)
{
        unsigned int dbv = 0;
        struct mtk_panel_ext *ext = find_panel_ext(panel);
        dbv = oplus_display_brightness;
        DISP_INFO("pwm_turbo backlight level=%d, pwm_en=%d, CurrentFrameRate=%d params_pwm_set=%d\n",
			dbv, en_h_pwm, CurrentFrameRate,ext->params->f_high_pwm_en);
	oplus_set_dbv_frame(dsi, cb, handle, en_h_pwm);
        return 0;
}

static int oplus_display_panel_set_pwm_fps(void *dsi, dcs_write_gce_pack cb, void *handle, int fps, bool en_h_pwm)
{
	DISP_INFO("pwm_turbo oplus_display_panel_set_pwm_fps fps=%d, pwm_en=%d\n", fps, en_h_pwm);

	if (get_pwm_status(en_h_pwm)) {
		if (fps == 60) {
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(300), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_high_pwm_sdc60_part1, sizeof(timing_switch_cmd_high_pwm_sdc60_part1) / sizeof(struct LCM_setting_table), cb, handle);
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(8500), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_high_pwm_sdc60, sizeof(timing_switch_cmd_high_pwm_sdc60) / sizeof(struct LCM_setting_table), cb, handle);
		} else {
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_high_pwm_sdc120_part1, sizeof(timing_switch_cmd_high_pwm_sdc120_part1) / sizeof(struct LCM_setting_table), cb, handle);
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(8500), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_high_pwm_sdc120, sizeof(timing_switch_cmd_high_pwm_sdc120) / sizeof(struct LCM_setting_table), cb, handle);
		}
	} else {
		if (fps == 60) {
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(300), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_sdc60_part1, sizeof(timing_switch_cmd_sdc60_part1) / sizeof(struct LCM_setting_table), cb, handle);
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(8500), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_sdc60, sizeof(timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table), cb, handle);
		} else {
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_sdc120_part1, sizeof(timing_switch_cmd_sdc120_part1) / sizeof(struct LCM_setting_table), cb, handle);
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(8500), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_sdc120, sizeof(timing_switch_cmd_sdc120) / sizeof(struct LCM_setting_table), cb, handle);
		}
	}
	return 0;
}


static int oplus_display_panel_set_pwm_plus_bl(void *dsi, dcs_write_gce_pack cb, void *handle, unsigned int bl_lvl)
{
	if (bl_lvl <= 0x643) {
		dsi_high_12plus_bl[6].para_list[1] = bl_lvl >> 8;
		dsi_high_12plus_bl[6].para_list[2] = bl_lvl & 0xFF;
		DISP_INFO("pwm_turbo dsi_high_12plus_bl backlight level=%d, bl_record=%d\n", bl_lvl, bl_record);
		panel_send_pack_hs_cmd(dsi, dsi_high_12plus_bl, sizeof(dsi_high_12plus_bl) / sizeof(struct LCM_setting_table), cb, handle);
	} else if (bl_lvl > 0x643) {
		dsi_low_3plus_bl[6].para_list[1]  = bl_lvl >> 8;
		dsi_low_3plus_bl[6].para_list[2]  = bl_lvl & 0xFF;
		DISP_INFO("pwm_turbo dsi_low_3plus_bl backlight level=%d, bl_record=%d\n", bl_lvl, bl_record);
		panel_send_pack_hs_cmd(dsi, dsi_low_3plus_bl, sizeof(dsi_low_3plus_bl) / sizeof(struct LCM_setting_table), cb, handle);
	}
	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};
	int i = 0;

	DISP_DEBUG("[DISP][DEBUG]start\n");

	if (!dsi || !cb) {
		return -EINVAL;
	}

	last_backlight = level;

	if (level == 1) {
		DISP_INFO("[DISP][INFO]filter backlight %u setting\n", level);
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	if (lcm_id2 == 1) {
			if (level <= 1603) {
				for (i = 0; i < sizeof(bl_level_low_1603)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, bl_level_low_1603[i].para_list, bl_level_low_1603[i].count);
				}
			} else {
				for (i = 0; i < sizeof(bl_level_high_1603)/sizeof(struct LCM_setting_table); i++) {
					cb(dsi, handle, bl_level_high_1603[i].para_list, bl_level_high_1603[i].count);
				}
			}
		}



	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	DISP_INFO("backlight_level=%d,bl_level[1]=0x%x,bl_level[2]=0x%x,mapped_level=%d \n", level,bl_level[1],bl_level[2] ,mapped_level);
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	esd_brightness = level;
	oplus_display_brightness = level;

	DISP_DEBUG("end\n");

	return 0;
}


static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	DISP_DEBUG("start\n");

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 1) {
		DISP_INFO("filter backlight %u setting\n",  level);
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	DISP_INFO("backlight lvl:%u\n",  level);

	if (0) {
		oplus_display_panel_set_frequency_pwm(dsi, cb, handle, level);
	}
	bl_level[1] = oplus_display_brightness >> 8;
	bl_level[2] = oplus_display_brightness & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));

	DISP_DEBUG("end\n");

	return 0;
}

#endif
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

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd_60hz)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_on_cmd_60hz[i].para_list, hbm_on_cmd_60hz[i].count);
		}
	} else if (hbm_mode == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd_60hz)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_off_cmd_60hz[i].para_list, hbm_off_cmd_60hz[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);

		if ((get_pwm_status(g_pwm_en)) && (oplus_display_brightness != 1)) {
			pwm_power_on = true;
			oplus_display_panel_set_frequency_pwm(dsi, cb, handle, oplus_display_brightness);
		}
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
	struct lcm *ctx = NULL;
	struct LCM_setting_table *hbm_cmd = NULL;

	OFP_DEBUG("start\n");

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
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

	if (!en) {
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);

		if ((get_pwm_status(g_pwm_en)) && (oplus_display_brightness != 1)) {
			pwm_power_on = true;
			oplus_display_panel_set_frequency_pwm(dsi, cb, handle, oplus_display_brightness);
		}
	}

	lcdinfo_notify(1, &en);

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *aod_off_cmd = NULL;

	if (!panel || !dsi) {
		OFP_ERR("Invalid dsi params\n");
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	if (get_pwm_status(g_pwm_en)) {
		if (vrefresh_rate == 60) {
			aod_off_cmd = aod_off_cmd_hpwm_60hz;
			reg_count = sizeof(aod_off_cmd_hpwm_60hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_cmd_hpwm_120hz;
			reg_count = sizeof(aod_off_cmd_hpwm_120hz) / sizeof(struct LCM_setting_table);
		}
	} else {
		if (vrefresh_rate == 60) {
			aod_off_cmd = aod_off_cmd_lpwm_60hz;
			reg_count = sizeof(aod_off_cmd_lpwm_60hz) / sizeof(struct LCM_setting_table);
		} else if (vrefresh_rate == 120) {
			aod_off_cmd = aod_off_cmd_lpwm_120hz;
			reg_count = sizeof(aod_off_cmd_lpwm_120hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_cmd_lpwm_90hz;
			reg_count = sizeof(aod_off_cmd_lpwm_90hz) / sizeof(struct LCM_setting_table);
		}
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
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_off_cmd[i].para_list, aod_off_cmd[i].count);
		}
	}

	lcm_setbacklight_cmdq(dsi, cb, handle, last_backlight);
	if (get_pwm_status(g_pwm_en)) {
		pwm_power_on = true;
		oplus_display_panel_set_frequency_pwm(dsi, cb, handle, oplus_display_brightness);
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
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DISP_DEBUG("on=%d\n",on);

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
	usleep_range(5000, 5001);
	lcm_panel_vrfio18_aif_enable(ctx->dev);
	usleep_range(1000, 1100);
//	ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if(IS_ERR(ctx->vddr1p5_enable_gpio)){
		DISP_ERR("cannot get 1p5-gpios %ld\n",PTR_ERR(ctx->vddr1p5_enable_gpio));
	}
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	//devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);

	usleep_range(1000, 1100);

	//set vddi 3.0v
	lcm_panel_vmc_ldo_enable(ctx->dev);

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

	DISP_DEBUG("lcm ctx->prepared %d \n",ctx->prepared);

	if (ctx->prepared)
		return 0;

//	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if(IS_ERR(ctx->reset_gpio)){
		DISP_ERR("cannot get reset-gpios %ld\n",PTR_ERR(ctx->reset_gpio));
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	//devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);
	//set vddi 3.0v
	lcm_panel_vmc_ldo_disable(ctx->dev);
	usleep_range(5000, 5100);
	//enable 1.5V
//	ctx->vddr1p5_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	//devm_gpiod_put(ctx->dev, ctx->vddr1p5_enable_gpio);
	if(IS_ERR(ctx->vddr1p5_enable_gpio)){
		DISP_ERR("cannot get 1p5-gpios %ld\n",PTR_ERR(ctx->vddr1p5_enable_gpio));
	}
	usleep_range(55000, 55100);
	//set vddi 3.0v
	lcm_panel_vrfio18_aif_disable(ctx->dev);



	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(20000, 20100);
	DISP_INFO("Successful\n");
	return 0;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
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

	if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
		*ext_param = &ext_params[1];
	}
	else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
		*ext_param = &ext_params[0];
	} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
		*ext_param = &ext_params[2];
	} else if (m_vrefresh == 120 && m->hskew == OPLUS_ADFR) {
		*ext_param = &ext_params[3];
	} else {
		*ext_param = &ext_params[0];
	}

	if (*ext_param)
		DISP_DEBUG("data_rate:%d,mode:%d,m_vrefresh:%d\n", (*ext_param)->data_rate,id,m_vrefresh);
	else
		DISP_INFO("ext_param is NULL;\n");

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
	DISP_INFO(" mode=%d, vrefresh=%d, hskew=%d\n", mode, drm_mode_vrefresh(m), m->hskew);

	if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
		ext->params = &ext_params[1];
	} else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
		ext->params = &ext_params[0];
	} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
		ext->params = &ext_params[2];
	} else if (m_vrefresh == 120 && m->hskew == OPLUS_ADFR) {
		ext->params = &ext_params[3];
	} else {
		ext->params = &ext_params[0];
	}

	return ret;
}

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle)
{
	unsigned int i = 0;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		DISP_INFO("out of mtk_ddic_dsi_cmd \n");
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

static int panel_minfps_check(int mode_id, int extend_frame)
{
	if (mode_id == FHD_SDC90) {
		if (extend_frame < OPLUS_ADFR_MANAUL_MIN_FPS_MAX || extend_frame > OPLUS_ADFR_MANAUL_MIN_FPS90_30HZ)
			extend_frame = OPLUS_ADFR_MANAUL_MIN_FPS_MAX;
	} else if (mode_id == FHD_SDC60) {
		if (extend_frame < OPLUS_ADFR_MANAUL_MIN_FPS_60HZ || extend_frame > OPLUS_ADFR_MANAUL_MIN_FPS_30HZ)
			extend_frame = OPLUS_ADFR_MANAUL_MIN_FPS_60HZ;
	} else {
		if (extend_frame < OPLUS_ADFR_MANAUL_MIN_FPS_MAX || extend_frame > OPLUS_ADFR_MANAUL_MIN_FPS_30HZ)
			extend_frame = OPLUS_ADFR_MANAUL_MIN_FPS_MAX;
	}
	return extend_frame;
}

static int panel_set_minfps(void *dsi, struct drm_panel *panel, dcs_write_gce_pack cb, void *handle,
	void *minfps, struct drm_display_mode *m)
{
	unsigned int mode_id = 0;
	unsigned int vrefresh_rate = 0;
	unsigned int ext_frame = 0;
	unsigned int lcm_cmd_count = 0;
	struct oplus_minfps *min_fps = (struct oplus_minfps *)minfps;

	if (!dsi || !cb || !minfps || !m) {
		ADFR_ERR("Invalid params\n");
		return -EINVAL;
	}

	mode_id = get_mode_enum(m);
	vrefresh_rate = drm_mode_vrefresh(m);
	ADFR_INFO("mode_id:%u,refresh_rate:%u,minfps_flag:%u,extern_frame:%u\n",
				mode_id, vrefresh_rate, min_fps->minfps_flag, min_fps->extend_frame);

	/* update min fps cmd */
	if (!min_fps->minfps_flag) {
		/* update manual min fps */
		ext_frame = panel_minfps_check(mode_id, min_fps->extend_frame);
		if (get_pwm_status(g_pwm_en)) {
			if (vrefresh_rate != 90) {
				auto_off_minfps_cmd_hpwm_120hz[SDC_MANUAL_MIN_FPS_CMD_OFFSET].para_list[1] = ext_frame;
				lcm_cmd_count = sizeof(auto_off_minfps_cmd_hpwm_120hz) / sizeof(struct LCM_setting_table);
				ADFR_INFO("auto_off_minfps_cmd_hpwm_120hz,ext_frame:%u\n", ext_frame);
				panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_hpwm_120hz, lcm_cmd_count, cb, handle);
			}
		} else {
			if (vrefresh_rate != 90) {
				auto_off_minfps_cmd_lpwm_120hz[SDC_MANUAL_MIN_FPS_CMD_OFFSET].para_list[1] = ext_frame;
				lcm_cmd_count = sizeof(auto_off_minfps_cmd_lpwm_120hz) / sizeof(struct LCM_setting_table);
				ADFR_INFO("auto_off_minfps_cmd_lpwm_120hz,ext_frame:%u\n", ext_frame);
				panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_lpwm_120hz, lcm_cmd_count, cb, handle);
			} else {
				auto_off_minfps_cmd_lpwm_90hz[SDC_MANUAL_MIN_FPS_CMD_OFFSET].para_list[1] = ext_frame;
				lcm_cmd_count = sizeof(auto_off_minfps_cmd_lpwm_90hz) / sizeof(struct LCM_setting_table);
				ADFR_INFO("auto_off_minfps_cmd_lpwm_90hz,ext_frame:%u\n", ext_frame);
				panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd_lpwm_90hz, lcm_cmd_count, cb, handle);
			}
		}
	}

	return 0;
}

static int panel_set_multite(void *dsi, struct drm_panel *panel, dcs_write_gce_pack cb, void *handle, bool enable)
{
	unsigned int lcm_cmd_count = 0;

	/*enable or disable multi-te cmds */
	if (enable) {
		ADFR_INFO("multite enabled\n");
		/* enable multi TE */
		lcm_cmd_count = sizeof(multi_te_enable) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, multi_te_enable, lcm_cmd_count, cb, handle);
	} else {
		ADFR_INFO("multite disabled\n");
		/* disable multi TE */
		lcm_cmd_count = sizeof(multi_te_disable) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, multi_te_disable, lcm_cmd_count, cb, handle);
	}

	return 0;
}

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
static int oplus_temp_compensation_set(void *dsi, void *gce_cb, void *handle, unsigned int setting_mode)
{
	int rc = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!dsi || !gce_cb) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	rc = oplus_temp_compensation_cmd_set(dsi, gce_cb, handle, setting_mode);
	if (rc) {
		TEMP_COMPENSATION_ERR("failed to set temp compensation cmd, rc=%d\n", rc);
	}

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;
}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

static int mode_switch_hs(struct drm_panel *panel, struct drm_connector *connector,
		void *dsi_drv, unsigned int cur_mode, unsigned int dst_mode,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage, dcs_write_gce_pack cb)
{
	int ret = 0;
	int m_vrefresh = 0;
	int src_vrefresh = 0;
	unsigned int lcm_cmd_count = 0;
	/* lk mipi setting is 830 */
	static int last_data_rate = 900;
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct drm_display_mode *src_m = get_mode_by_id(connector, cur_mode);
	struct lcm *ctx = panel_to_lcm(panel);

	DISP_INFO("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	if (stage == BEFORE_DSI_POWERDOWN) {
		m_vrefresh = drm_mode_vrefresh(m);
		src_vrefresh = drm_mode_vrefresh(src_m);
		DISP_INFO("mode_switch_hs,cur_mode:%d,dst_mode:%d,hdisplay:%d->%d,vrefresh:%d->%d,hskew:%d->%d\n",
			cur_mode, dst_mode, src_m->hdisplay, m->hdisplay, src_vrefresh, m_vrefresh, src_m->hskew, m->hskew);

		if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
			if (m_vrefresh == 60) {
				DISP_INFO("timing switch to sdc60\n");
				if (get_pwm_status(g_pwm_en)) {
					if (src_vrefresh == 120) {
						usleep_range(200, 300);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_sdc60_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_sdc60_part1, lcm_cmd_count, cb, NULL);
					if(src_vrefresh == 90) {
						usleep_range(3500, 3600);
					} else if (src_vrefresh == 120) {
						usleep_range(8400, 8500);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_sdc60, lcm_cmd_count, cb, NULL);
				} else {
					if (src_vrefresh == 120) {
						usleep_range(200, 300);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60_part1, lcm_cmd_count, cb, NULL);
					if(src_vrefresh == 90) {
						usleep_range(3500, 3600);
					} else if (src_vrefresh == 120) {
						usleep_range(8400, 8500);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60, lcm_cmd_count, cb, NULL);
				}
					ret = 1;
			} else if((src_vrefresh == 60)&&(m_vrefresh == 90)) {
				DISP_INFO("timing switch sdc60 to sdc90 to return1\n");
				ret = 1;
			} else if ((src_vrefresh == 120) && (m_vrefresh == 90)) {
				DISP_INFO("timing switch 120 to sdc90\n");
				if (src_vrefresh == 120) {
					usleep_range(200, 300);
				}
				lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_part1) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_part1, lcm_cmd_count, cb, NULL);
				usleep_range(8400, 8500);
				lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
				ret = 1;
			} else if (m_vrefresh == 120) {
				DISP_INFO("timing switch to sdc120 to return1\n");
				ret = 1;
			}
			//oplus_adfr_set_multite_state(false);
		} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
			if (m_vrefresh == 120) {
				ret = 1;
			}
		}
	} else if (stage == AFTER_DSI_POWERON) {
		ctx->m = m;
		m_vrefresh = drm_mode_vrefresh(m);
		src_vrefresh = drm_mode_vrefresh(src_m);
		DISP_INFO("mode_switch_hs,cur_mode:%d,dst_mode:%d,hdisplay:%d->%d,vrefresh:%d->%d,hskew:%d->%d\n",
			cur_mode, dst_mode, src_m->hdisplay, m->hdisplay, src_vrefresh, m_vrefresh, src_m->hskew, m->hskew);

		if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
			if ((src_vrefresh == 60) && (m_vrefresh == 90)) {
				DISP_INFO("timing switch sdc60 to sdc90\n");
				lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_part1) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_part1, lcm_cmd_count, cb, NULL);
				usleep_range(8400, 8500);
				lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
			} else if (m_vrefresh == 120) {
				DISP_INFO("timing switch to sdc120\n");
				if (src_vrefresh == 90) {
						usleep_range(2500, 2600);
				}
				if (get_pwm_status(g_pwm_en)) {
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_sdc120_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_sdc120_part1, lcm_cmd_count, cb, NULL);
					if((src_vrefresh == 60) || (src_vrefresh == 120)) {
						usleep_range(8400, 8500);
					} else if (src_vrefresh == 90) {
						//usleep_range(3500, 3600);
						usleep_range(10000, 10100);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_sdc120, lcm_cmd_count, cb, NULL);
				} else {
					DISP_INFO("src_vrefresh=%d,m_vrefresh=%d\n",src_vrefresh,m_vrefresh);
					if (src_vrefresh == 90) {
						usleep_range(2500, 2600);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120_part1, lcm_cmd_count, cb, NULL);
					if((src_vrefresh == 60) || (src_vrefresh == 120)) {
						usleep_range(8400, 8500);
					} else if (src_vrefresh == 90) {
						//usleep_range(3500, 3600);
						usleep_range(10000, 10100);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120, lcm_cmd_count, cb, NULL);

				}
			}
			DISP_DEBUG("SET multite\n");
			oplus_adfr_set_multite_state(false);
		} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
			if (m_vrefresh == 120) {
				if (get_pwm_status(g_pwm_en)) {
					DISP_INFO("timing switch to high pwm oa120\n");
					if (src_vrefresh == 120) {
						usleep_range(200, 300);
					}
					if (src_vrefresh == 90) {
						usleep_range(2500, 2600);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_oa120_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_oa120_part1, lcm_cmd_count, cb, NULL);
					if((src_vrefresh == 60) || (src_vrefresh == 120)) {
						usleep_range(8400, 8500);
					} else if (src_vrefresh == 90) {
						usleep_range(10000, 10100);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_oa120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_oa120, lcm_cmd_count, cb, NULL);
				} else {
					/* send OA120 timing-switch cmd */
					DISP_INFO("timing switch to oa120\n");
					if (src_vrefresh == 120) {
						usleep_range(200, 300);
					}
					if (src_vrefresh == 90) {
						usleep_range(2500, 2600);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120_part1, lcm_cmd_count, cb, NULL);
					if((src_vrefresh == 60) || (src_vrefresh == 120)) {
						usleep_range(8400, 8500);
					} else if (src_vrefresh == 90) {
						usleep_range(10000, 10100);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120, lcm_cmd_count, cb, NULL);
				}
			}
		}
		if (oplus_adfr_is_support()) {
			// reset adfr auto mode status as panel mode will be change after timing switch
			DISP_DEBUG("RESET ADFR\n");
			oplus_adfr_status_reset(src_m, m);
		}

	}
	if (get_pwm_status(g_pwm_en) && (high_pwm_status == true)) {
		if (oplus_display_brightness <= 0x643 ) {
			dsi_high_12plus_bl[6].para_list[1] = oplus_display_brightness >> 8;
			dsi_high_12plus_bl[6].para_list[2] = oplus_display_brightness & 0xFF;
			DISP_INFO("pwm_turbo dsi_high_12plus_bl backlight odb=%d\n", oplus_display_brightness);
			panel_send_pack_hs_cmd(dsi_drv, dsi_high_12plus_bl, sizeof(dsi_high_12plus_bl) / sizeof(struct LCM_setting_table), cb, NULL);
		} else if (oplus_display_brightness > 0x643) {
			dsi_low_3plus_bl[6].para_list[1]  = oplus_display_brightness >> 8;
			dsi_low_3plus_bl[6].para_list[2]  = oplus_display_brightness & 0xFF;
			DISP_INFO("pwm_turbo dsi_low_3plus_bl backlight odb=%d\n", oplus_display_brightness);
			panel_send_pack_hs_cmd(dsi_drv, dsi_low_3plus_bl, sizeof(dsi_low_3plus_bl) / sizeof(struct LCM_setting_table), cb, NULL);
		}
		high_pwm_status = false;
	}
	if (ext->params->data_rate != last_data_rate) {
		ret = 1;
		DISP_INFO("need to change mipi clk, data_rate=%d,last_data_rate=%d\n",ext->params->data_rate,last_data_rate);
		last_data_rate = ext->params->data_rate;
	}

	return ret;
}

static int oplus_panel_osc_change(void *dsi, dcs_write_gce cb, void *handle, bool en)
{
	char page_reg[] = {0xF0,0x55,0xAA,0x52,0x08,0x01};
	char osc_mode1[] = {0xC3,0xDD,0x06,0x20,0x11,0xF1,0x00,0x06,0x20,0x11,0xFF,0x00,0x03,\
					0xE3,0x12,0x05,0x2E,0x18,0x03,0xC5,0x0D,0x05,0x38,0x12,0x03,0xE3,0x12,\
					0x05,0x2E,0x18,0x03,0xE3,0x12,0x05,0x2E,0x18,0x03,0xE3,0x12,0x05,0x2E,0x18};
	char osc_mode2[] = {0xC3,0xDD,0x06,0x20,0x09,0xF8,0x00,0x06,0x20,0x09,0xFF,0x00,0x03,\
					0xE1,0x09,0x05,0x2C,0x0C,0x04,0x0E,0x07,0x05,0x36,0x09,0x03,0xE1,0x09,\
					0x05,0x2C,0x0C,0x03,0xE1,0x09,0x05,0x2C,0x0C,0x03,0xE1,0x09,0x05,0x2C,0x0C};

	DISP_DEBUG("en=%d\n",en);

	cb(dsi, handle, page_reg, ARRAY_SIZE(page_reg));

	if (en) {
		cb(dsi, handle, osc_mode1, ARRAY_SIZE(osc_mode1));
	} else {
		cb(dsi, handle, osc_mode2, ARRAY_SIZE(osc_mode2));
	}
	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_reset = lcm_panel_reset,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_get = mtk_panel_ext_param_get,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch_hs = mode_switch_hs,
	.set_minfps = panel_set_minfps,
	.set_multite = panel_set_multite,
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
	.oplus_temp_compensation_set = oplus_temp_compensation_set,
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */
#if 1
	.set_hbm = lcm_set_hbm,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.lcm_osc_change = oplus_panel_osc_change,
	.lcm_high_pwm_set = oplus_display_panel_set_pwm_turbo,
	.lcm_high_pwm_elvss = oplus_display_panel_set_elvss,
	.lcm_high_pwm_set_fps = oplus_display_panel_set_pwm_fps,
	.lcm_high_pwm_set_plus_bl = oplus_display_panel_set_pwm_plus_bl,
	/* .esd_check_precondition = lcm_esd_check_precondition, */
};


static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode[MODE_NUM];
	int i = 0;

	mode[0] = drm_mode_duplicate(connector->dev, &display_mode[0]);
	if (!mode[0]) {
		DISP_INFO("failed to add mode %ux%ux@%u\n",
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
		DISP_DEBUG(" en=%u,clock=%d\n",mode[i], mode[i]->clock);
		if (!mode[i]) {
			DISP_ERR("not enough memory\n");
			return -ENOMEM;
		}

		drm_mode_set_name(mode[i]);
		mode[i]->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode[i]);
	}

	connector->display_info.width_mm = 70;
	connector->display_info.height_mm = 157;

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

	DISP_INFO("TM nt37705 +\n");

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

	usleep_range(1000000, 1000100);

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
	rc = oplus_temp_compensation_register_ntc_channel(dev);
	if (rc) {
		TEMP_COMPENSATION_ERR("failed to register ntc channel\n");
		return rc;
	}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

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
	if (IS_ERR(ctx->reset_gpio)) {
		DISP_ERR("cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 1);
	//devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	lcm_panel_vrfio18_aif_enable(ctx->dev);


	usleep_range(5000, 5100);
	ctx->vddr1p5_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p5_enable_gpio)) {
		DISP_ERR(" cannot get vddr1p5_enable_gpio %ld\n",PTR_ERR(ctx->vddr1p5_enable_gpio));
		return PTR_ERR(ctx->vddr1p5_enable_gpio);
	}

	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	ctx->vddr_aod_enable_gpio = devm_gpiod_get(ctx->dev, "vddr-aod-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr_aod_enable_gpio)) {
		DISP_ERR(" cannot get vddr_aod_enable_gpio %ld\n",PTR_ERR(ctx->vddr_aod_enable_gpio));
		//return PTR_ERR(ctx->vddr_aod_enable_gpio);
	}

	usleep_range(5000, 5100);

	lcm_panel_vmc_ldo_enable(ctx->dev);

	DISP_INFO("prepared=%d,enabled=%d\n",ctx->prepared,ctx->enabled);

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

	rc = of_property_read_u32(dev->of_node, "oplus-adfr-config", &config);
	if (rc == 0) {
		oplus_adfr_config = config;
		DISP_INFO("kVRR config=%d, adfrconfig=%d\n", config, oplus_adfr_config);
	} else {
		oplus_adfr_config = 0;
		DISP_INFO("kVRR adfrconfig=%d\n", oplus_adfr_config);
	}
	register_device_proc("lcd", "22823_Tianma_NT37705", "Tianma4095");
	//oplus_enhance_mipi_strength = 1;
	flag_silky_panel = BL_SETTING_DELAY_60HZ;
	bl_record = MAX_NORMAL_BRIGHTNESS;

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	oplus_ofp_set_fp_type(&fp_type);
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
	DISP_DEBUG("prepared=%d,enabled=%d\n",ctx->prepared,ctx->enabled);

	DISP_INFO("success\n");
	//g_is_silky_panel = true;
	pwm_power_on = true;
	high_pwm_status = true;
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
	    .compatible = "oplus22823,tm,nt37705,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus22823_tm_nt37705_fhd_dsi_cmd",
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


MODULE_AUTHOR("lianghao");
MODULE_DESCRIPTION("oplus22823,tm,nt37705,OLED Driver");
MODULE_LICENSE("GPL v2");
