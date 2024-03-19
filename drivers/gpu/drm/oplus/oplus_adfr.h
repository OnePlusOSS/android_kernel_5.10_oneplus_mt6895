/***************************************************************
** Copyright (C),  2022,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_adfr.h
** Description : oplus_adfr header
** Version : 2.0
** Date : 2022/07/22
**
** --------------------- Revision History: ---------------------
**  <author>        <data>        <version>        <desc>
**  Gaoxiaolei    2021/07/09         1.0       Build this moudle
**  Zhongliuhe    2022/07/22         2.0       Add ADFR2.0 function
***************************************************************/

#ifndef _OPLUS_ADFR_H_
#define _OPLUS_ADFR_H_

/* please just only include linux common head file to keep me pure */
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <drm/drm_device.h>
#include <drm/drm_modes.h>
#include "oplus_adfr_ext.h"

/* --------------- adfr misc ---------------*/
void oplus_adfr_init(struct drm_device *dev, struct mtk_drm_private *priv);
inline bool oplus_adfr_is_support(void);
inline bool oplus_adfr_fakeframe_is_enable(void);
ssize_t oplus_adfr_get_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_adfr_set_debug(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_adfr_get_params(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_adfr_set_params(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);

/* --------------- fake frame --------------- */
inline bool oplus_adfr_fakeframe_is_enable(void);
void oplus_adfr_send_fake_frame(struct drm_crtc *crtc);
void oplus_fakeframe_work_handler(struct work_struct *work_item);
enum hrtimer_restart oplus_fakeframe_timer_handler(struct hrtimer *timer);
int oplus_adfr_fakeframe_timer_start(int deferred_ms);
int oplus_adfr_cancel_fakeframe(void);

/* --------------- vsync switch --------------- */
ssize_t oplus_get_vsync_switch(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_set_vsync_switch(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
bool oplus_adfr_vsync_switch_is_enable(void);
enum oplus_vsync_mode oplus_adfr_get_vsync_mode(void);
void oplus_adfr_vsync_switch(struct drm_display_mode *m, bool force_te_vsync);
/* vsync switch in resolution switch and aod scene */
void oplus_adfr_vsync_switch_work_handler(struct work_struct *work_item);
void oplus_adfr_frame_done_vsync_switch(struct mtk_drm_crtc *mtk_crtc);
void oplus_adfr_resolution_vsync_switch(struct mtk_drm_crtc *mtk_crtc, struct drm_connector *connector,
	unsigned int cur_mode, unsigned int dst_mode);
void oplus_adfr_aod_fod_vsync_switch(struct mtk_drm_crtc *mtk_crtc, bool force_te_vsync);
void oplus_adfr_vsync_switch_reset(struct mtk_drm_crtc *mtk_crtc);

/* --------------- auto mode --------------- */
/* add for auto on cmd filter */
bool oplus_adfr_auto_on_cmd_filter_set(bool enable);
bool oplus_adfr_auto_on_cmd_filter_get(void);
void oplus_adfr_handle_auto_mode(int prop_id, unsigned int propval);
void oplus_adfr_dsi_display_auto_mode_update(struct drm_device *drm);
int oplus_adfr_temperature_detection_handle(void *mtk_ddp_comp, void *cmdq_pkt, int ntc_temp, int shell_temp);
void oplus_adfr_status_reset(struct drm_display_mode *src_m, struct drm_display_mode *dst_m);

/* add a timer for mutex multi te and set backlight*/
enum hrtimer_restart oplus_adfr_osync_mode_timer_handler(struct hrtimer *timer);
int oplus_adfr_osync_mode_timer_start(int deferred_ms);
void oplus_adfr_oplus_mode_multi_te_update(struct drm_device *drm);
void oplus_adfr_osync_mode_work_handler(struct work_struct *work_item);

/* --------------- idle mode -------------- */
/* ADFR:Add for idle mode control */
/* idle mode handle */
void oplus_adfr_handle_idle_mode(void *drm_crtc, int enter_idle);

#endif /* _OPLUS_ADFR_H_ */
