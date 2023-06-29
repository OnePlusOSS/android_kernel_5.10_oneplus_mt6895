/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_dc.c
** Description : oplus dc feature
** Version : 1.0
** Date : 2020/07/1
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  JianBin.Zhang   2020/07/01        1.0           Build this moudle
**  Xiaolei.Gao     2021/08/14        1.1           Build this moudle
***************************************************************/
#include <oplus_display_common.h>
#include "oplus_display_panel.h"
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "oplus_display_onscreenfingerprint.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

#define PANEL_SERIAL_NUM_REG 0xA1
#define PANEL_REG_READ_LEN   10
#define BOE_PANEL_SERIAL_NUM_REG 0xA3
#define PANEL_SERIAL_NUM_REG_TIANMA 0xD6

extern unsigned int cabc_mode;
extern unsigned int cabc_true_mode;
extern unsigned int cabc_sun_flag;
extern unsigned int cabc_back_flag;
extern void disp_aal_set_dre_en(int enable);
extern unsigned int silence_mode;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern uint64_t serial_number;
extern unsigned int esd_mode;
extern unsigned int seed_mode;
extern unsigned int m_da;
extern unsigned int m_db;
extern unsigned int m_dc;
extern bool g_dp_support;
extern bool pq_trigger;
extern unsigned int hpwm_mode;
extern unsigned int hpwm_mode_90hz;


extern struct drm_device* get_drm_device(void);
extern int mtk_drm_setbacklight(struct drm_crtc *crtc, unsigned int level);

//extern int panel_serial_number_read(char cmd, int num);
//extern int oplus_mtk_drm_setcabc(struct drm_crtc *crtc, unsigned int hbm_mode);
/*extern int oplus_mtk_drm_setseed(struct drm_crtc *crtc, unsigned int seed_mode);*/

extern int panel_serial_number_read(char cmd, int num);
extern int oplus_mtk_drm_setcabc(struct drm_crtc *crtc, unsigned int hbm_mode);
extern int oplus_mtk_drm_setseed(struct drm_crtc *crtc, unsigned int seed_mode);

enum {
	CABC_LEVEL_0,
	CABC_LEVEL_1,
	CABC_LEVEL_2 = 3,
	CABC_EXIT_SPECIAL = 8,
	CABC_ENTER_SPECIAL = 9,
	GLOBAL_DRE_OPEN = 10,
	GLOBAL_DRE_CLOSE = 11,
};

int oplus_display_set_brightness(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *set_brightness = buf;
	unsigned int oplus_set_brightness = (*set_brightness);

	printk("%s %d\n", __func__, oplus_set_brightness);

	if (oplus_set_brightness > OPLUS_MAX_BRIGHTNESS || oplus_set_brightness < OPLUS_MIN_BRIGHTNESS) {
		printk(KERN_ERR "%s, brightness:%d out of scope\n", __func__, oplus_set_brightness);
		return -1;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_drm_setbacklight(crtc, oplus_set_brightness);

	return 0;
}

int oplus_display_get_brightness(void *buf)
{
	unsigned int *brightness = buf;

	(*brightness) = oplus_display_brightness;

	return 0;
}

int oplus_display_panel_get_max_brightness(void *buf)
{
	unsigned int *brightness = buf;

	(*brightness) = oplus_max_normal_brightness;

	return 0;
}

int oplus_display_panel_get_serial_number(void *buf)
{
	struct panel_serial_number *p_snumber = buf;
	int ret = 0;

	printk("%s read serial number 0x%x\n", __func__, serial_number);
	ret = scnprintf(p_snumber->serial_number, PAGE_SIZE, "Get panel serial number: %llx\n", serial_number);
	return ret;
}

int oplus_display_panel_get_cabc(void *buf)
{
	unsigned int *c_mode = buf;

	printk("%s CABC_mode=%d\n", __func__, cabc_true_mode);
	*c_mode = cabc_true_mode;

	return 0;
}
# if 0
int oplus_display_panel_set_cabc(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *c_mode = buf;
	cabc_mode = (unsigned int)(*c_mode);

	cabc_true_mode = cabc_mode;
	printk("%s,cabc mode is %d, cabc_back_flag is %d\n", __func__, cabc_mode, cabc_back_flag);
	if (cabc_mode < 4) {
		cabc_back_flag = cabc_mode;
	}

	if (cabc_mode == CABC_ENTER_SPECIAL) {
		cabc_sun_flag = 1;
		cabc_true_mode = 0;
	} else if (cabc_mode == CABC_EXIT_SPECIAL) {
		cabc_sun_flag = 0;
		cabc_true_mode = cabc_back_flag;
	} else if (cabc_sun_flag == 1) {
		if (cabc_back_flag == CABC_LEVEL_0) {
			disp_aal_set_dre_en(1);
			printk("%s sun enable dre\n", __func__);
		} else {
			disp_aal_set_dre_en(0);
			printk("%s sun disable dre\n", __func__);
		}
		return 0;
	}

	printk("%s,cabc mode is %d\n", __func__, cabc_true_mode);

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	if (cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0) {
		disp_aal_set_dre_en(1);
		printk("%s enable dre\n", __func__);
	} else {
		disp_aal_set_dre_en(0);
		printk("%s disable dre\n", __func__);
	}
	/*oplus_mtk_drm_setcabc(crtc, cabc_true_mode);*/
	if (cabc_true_mode != cabc_back_flag) {
		cabc_true_mode = cabc_back_flag;
	}

	return 0;
}
#endif
int oplus_display_panel_set_cabc(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	struct mtk_drm_crtc *mtk_crtc;
	uint32_t *cabc_mode_temp = buf;
	cabc_mode = *cabc_mode_temp;
	cabc_true_mode = cabc_mode;
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}
	printk("%s,cabc mode is %d, cabc_back_flag is %d,oplus_display_global_dre = %d\n", __func__,
		cabc_mode, cabc_back_flag, mtk_crtc->panel_ext->params->oplus_display_global_dre);
	if (cabc_mode < 4) {
		cabc_back_flag = cabc_mode;
	}

	if (mtk_crtc->panel_ext->params->oplus_display_global_dre) {
		if (cabc_mode == GLOBAL_DRE_OPEN) {
			disp_aal_set_dre_en(1);
			printk("%s adb cmd enable dre\n", __func__);
			return 0;
		} else if (cabc_mode == GLOBAL_DRE_CLOSE) {
			disp_aal_set_dre_en(0);
			printk("%s adb cmd disable dre\n", __func__);
			return 0;
		}
	}

	if (cabc_mode == CABC_ENTER_SPECIAL) {
		cabc_sun_flag = 1;
		cabc_true_mode = 0;
	} else if (cabc_mode == CABC_EXIT_SPECIAL) {
		cabc_sun_flag = 0;
		cabc_true_mode = cabc_back_flag;
	} else if (cabc_sun_flag == 1) {
		if (cabc_back_flag == CABC_LEVEL_0 || mtk_crtc->panel_ext->params->oplus_display_global_dre) {
			disp_aal_set_dre_en(1);
			printk("%s sun enable dre\n", __func__);
		} else {
			disp_aal_set_dre_en(0);
			printk("%s sun disable dre\n", __func__);
		}
		return 0;
	}

	printk("%s,cabc mode is %d\n", __func__, cabc_true_mode);

	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	if ((cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0) || mtk_crtc->panel_ext->params->oplus_display_global_dre) {
		disp_aal_set_dre_en(1);
		printk("%s enable dre\n", __func__);
	} else {
		disp_aal_set_dre_en(0);
		printk("%s disable dre\n", __func__);
	}
	oplus_mtk_drm_setcabc(crtc, cabc_true_mode);
	if (cabc_true_mode != cabc_back_flag) {
		cabc_true_mode = cabc_back_flag;
	}
	return 0;
}



int oplus_display_panel_get_closebl_flag(void *buf)
{
	unsigned int *closebl_flag = buf;

	printk("%s silence_mode=%d\n", __func__, silence_mode);
	(*closebl_flag) = silence_mode;

	return 0;
}

int oplus_display_panel_set_closebl_flag(void *buf)
{
	unsigned int *closebl_flag = buf;

	msleep(1000);
	silence_mode = (*closebl_flag);
	printk("%s silence_mode=%d\n", __func__, silence_mode);

	return 0;
}

int oplus_display_panel_get_esd(void *buf)
{
	unsigned int *p_esd = buf;

	printk("%s esd=%d\n", __func__, esd_mode);
	(*p_esd) = esd_mode;

	return 0;
}

int oplus_display_panel_set_esd(void *buf)
{
	unsigned int *p_esd = buf;

	esd_mode = (*p_esd);
	printk("%s,esd mode is %d\n", __func__, esd_mode);

	return 0;
}

int oplus_display_panel_get_vendor(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct panel_info *p_info = buf;
	struct drm_device *ddev = get_drm_device();

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail,p_info=%p\n", p_info);
		return -1;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}

	memcpy(p_info->version, mtk_crtc->panel_ext->params->vendor,
               sizeof(mtk_crtc->panel_ext->params->vendor) > 31?31:(sizeof(mtk_crtc->panel_ext->params->vendor)+1));
	memcpy(p_info->manufacture, mtk_crtc->panel_ext->params->manufacture,
               sizeof(mtk_crtc->panel_ext->params->vendor) > 31?31:(sizeof(mtk_crtc->panel_ext->params->vendor)+1));

	return 0;
}
int oplus_display_get_softiris_color_status(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	struct softiris_color *iris_color_status = buf;

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}

	iris_color_status->color_vivid_status = mtk_crtc->panel_ext->params->color_vivid_status;
	iris_color_status->color_srgb_status = mtk_crtc->panel_ext->params->color_srgb_status;
	iris_color_status->color_softiris_status = mtk_crtc->panel_ext->params->color_softiris_status;
	iris_color_status->color_dual_panel_status = mtk_crtc->panel_ext->params->color_dual_panel_status;
	iris_color_status->color_dual_brightness_status = mtk_crtc->panel_ext->params->color_dual_brightness_status;
	iris_color_status->color_oplus_calibrate_status = mtk_crtc->panel_ext->params->color_oplus_calibrate_status;
	pr_err("oplus_color_vivid_status: %s", iris_color_status->color_vivid_status ? "true" : "false");
	pr_err("oplus_color_srgb_status: %s", iris_color_status->color_srgb_status ? "true" : "false");
	pr_err("oplus_color_softiris_status: %s", iris_color_status->color_softiris_status ? "true" : "false");
	pr_err("color_dual_panel_status: %s", iris_color_status->color_dual_panel_status ? "true" : "false");
	pr_err("color_dual_brightness_status: %s", iris_color_status->color_dual_brightness_status ? "true" : "false");
	return 0;
}


int oplus_display_panel_get_seed(void *buf)
{
	unsigned int *seed = buf;

	printk("%s seed_mode=%d\n", __func__, seed_mode);
	(*seed) = seed_mode;

	return 0;
}

int oplus_display_panel_set_seed(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *seed_mode_tmp = buf;

	printk("%s, %d to be %d\n", __func__, seed_mode, *seed_mode_tmp);

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	oplus_mtk_drm_setseed(crtc, *seed_mode_tmp);
	seed_mode = (*seed_mode_tmp);

	return 0;
}

int oplus_display_panel_get_id(void *buf)
{
	struct panel_id *panel_rid = buf;

	pr_err("%s: 0xDA= 0x%x, 0xDB=0x%x, 0xDC=0x%x\n", __func__, m_da, m_db, m_dc);

	panel_rid->DA = (uint32_t)m_da;
	panel_rid->DB = (uint32_t)m_db;
	panel_rid->DC = (uint32_t)m_dc;

	return 0;
}

int oplus_display_get_dp_support(void *buf)
{
	uint32_t *dp_support = buf;

	pr_info("%s: dp_support = %s\n", __func__, g_dp_support ? "true" : "false");

	*dp_support = g_dp_support;

	return 0;
}
int oplus_display_panel_get_pq_trigger(void *buf)
{
        unsigned int *pq_trigger_flag = buf;

        printk("%s pq_trigger=%d\n", __func__, pq_trigger);
        (*pq_trigger_flag) = pq_trigger;

        return 0;
}

int oplus_display_panel_set_pq_trigger(void *buf)
{
        unsigned int *pq_trigger_flag = buf;

        pq_trigger = (*pq_trigger_flag);
        printk("%s pq_trigger=%d\n", __func__, pq_trigger);

        return 0;
}

inline bool pwm_turbo_support(void)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}
	pr_err("pwm_turbo_support info %s\n", mtk_crtc->panel_ext->params->manufacture);
	if(!strcmp(mtk_crtc->panel_ext->params->manufacture, "22823_Tianma_NT37705")) {
		printk(KERN_ERR "support pwm turbo\n");
		return true;
	}
	return false;
}
EXPORT_SYMBOL(pwm_turbo_support);

int oplus_display_panel_set_pwm_status(void *data)
{
	int rc = 0;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *pwm_status = data;

	if (!data) {
		pr_err("%s: set pwm status data is null\n", __func__);
		return -EINVAL;
	}

	printk(KERN_INFO "oplus high pwm mode = %d\n", *pwm_status);

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}

	if ((!strcmp(mtk_crtc->panel_ext->params->vendor, "22823_Tianma_NT37705"))) {
		rc = mtk_crtc_set_high_pwm_switch(crtc, *pwm_status);
		hpwm_mode = (long)*pwm_status;
	}
	return rc;
}

int oplus_display_panel_get_pwm_status(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *pwm_status = buf;

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}

	if (!strcmp(mtk_crtc->panel_ext->params->vendor, "22823_Tianma_NT37705")) {
		*pwm_status = hpwm_mode;
		pr_info("%s: high pwm mode = %d\n", __func__, hpwm_mode);
	} else {
		*pwm_status = 0;
	}

	return 0;
}

int oplus_display_panel_get_pwm_status_for_90hz(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *pwm_status = buf;

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}

	if (!strcmp(mtk_crtc->panel_ext->params->vendor, "22823_Tianma_NT37705")) {
		*pwm_status = hpwm_mode_90hz;
		pr_info("%s: high hpwm_mode_90hz = %d\n", __func__, hpwm_mode_90hz);
	} else {
		*pwm_status = 10;
	}

	return 0;
}


MODULE_AUTHOR("Xiaolei Gao");
MODULE_DESCRIPTION("OPLUS common device");
MODULE_LICENSE("GPL v2");

