/***************************************************************
** Copyright (C),  2022,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_adfr_ext.h
** Description : oplus_adfr_ext header
** Version : 2.0
** Date : 2022/07/22
**
** --------------------- Revision History: ---------------------
**  <author>        <data>        <version>        <desc>
**  Gaoxiaolei    2021/07/09         1.0       Build this moudle
**  Zhongliuhe    2022/07/22         2.0       Add ADFR2.0 function
***************************************************************/

#ifndef _OPLUS_ADFR_EXT_H_
#define _OPLUS_ADFR_EXT_H_

#include <drm/drm_modes.h>

enum oplus_adfr_log_level {
	OPLUS_ADFR_LOG_LEVEL_ERR = 0,
	OPLUS_ADFR_LOG_LEVEL_WARN = 1,
	OPLUS_ADFR_LOG_LEVEL_INFO = 2,
	OPLUS_ADFR_LOG_LEVEL_DEBUG = 3,
};

enum oplus_adfr_display_id {
	OPLUS_ADFR_PRIMARY_DISPLAY = 0,
	OPLUS_ADFR_SECONDARY_DISPLAY = 1,
};

/* remember to initialize params */
struct oplus_adfr_params {
	unsigned int config;							/* */
};

/* log level config */
extern unsigned int oplus_adfr_log_level;
/* dual display id */
extern unsigned int oplus_adfr_display_id;
/* debug log switch */
extern int oplus_dsi_log_type;

/* just use for adfr */
#define ADFR_ERR(fmt, arg...)	\
	do {	\
		if (oplus_adfr_log_level >= OPLUS_ADFR_LOG_LEVEL_ERR)	\
			pr_err("[KVRR][%d][ERR][%s:%d]"pr_fmt(fmt), oplus_adfr_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define ADFR_WARN(fmt, arg...)	\
	do {	\
		if (oplus_adfr_log_level >= OPLUS_ADFR_LOG_LEVEL_WARN)	\
			pr_warn("[KVRR][%d][WARN][%s:%d]"pr_fmt(fmt), oplus_adfr_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define ADFR_INFO(fmt, arg...)	\
	do {	\
		if (oplus_adfr_log_level >= OPLUS_ADFR_LOG_LEVEL_INFO)	\
			pr_info("[KVRR][%d][INFO][%s:%d]"pr_fmt(fmt), oplus_adfr_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define ADFR_DEBUG(fmt, arg...)	\
	do {	\
		if ((oplus_adfr_log_level >= OPLUS_ADFR_LOG_LEVEL_DEBUG) && (oplus_dsi_log_type & OPLUS_DEBUG_LOG_ADFR))	\
			pr_info("[KVRR][%d][DEBUG][%s:%d]"pr_fmt(fmt), oplus_adfr_display_id, __func__, __LINE__, ##arg);	\
	} while (0)



#define VREFRESH_120_NS	(8333333)
#define VREFRESH_90_NS	(11111111)

#define SDC_AUTO_MIN_FPS_CMD_OFFSET 2
#define SDC_MANUAL_MIN_FPS_CMD_OFFSET 2
#define SDC_MIN_FPS_CMD_SIZE 2

enum oplus_vsync_mode {
	OPLUS_DOUBLE_TE_VSYNC = 0,
	OPLUS_EXTERNAL_TE_TP_VSYNC = 8,
	OPLUS_INVALID_VSYNC,
};

enum oplus_vsync_switch {
	OPLUS_VSYNC_SWITCH_TP = 0,	/* TP VSYNC */
	OPLUS_VSYNC_SWITCH_TE = 1,	/* TE VSYNC */
};

enum h_skew_type {
	SDC_ADFR = 0,				/* SA */
	SDC_MFR = 1,				/* SM */
	OPLUS_ADFR = 2,				/* OA */
	OPLUS_MFR = 3,				/* OM */
};

enum oplus_adfr_auto_mode_value {
	OPLUS_ADFR_AUTO_OFF = 0,
	OPLUS_ADFR_AUTO_ON = 1,
};

enum oplus_adfr_auto_fakeframe_value {
	OPLUS_ADFR_FAKEFRAME_OFF = 0,
	OPLUS_ADFR_FAKEFRAME_ON = 1,
};

enum oplus_adfr_auto_min_fps_value {
	OPLUS_ADFR_AUTO_MIN_FPS_MAX = 0x00,
	OPLUS_ADFR_AUTO_MIN_FPS_60HZ = 0x01,
	OPLUS_ADFR_AUTO_MIN_FPS_30HZ = 0x03,
	OPLUS_ADFR_AUTO_MIN_FPS_20HZ = 0x05,
	OPLUS_ADFR_AUTO_MIN_FPS_10HZ = 0x0B,
	OPLUS_ADFR_AUTO_MIN_FPS_1HZ = 0x77,
};

enum oplus_adfr_idle_mode_value {
	OPLUS_ADFR_IDLE_OFF = 0,
	OPLUS_ADFR_IDLE_ON = 1,
};

struct oplus_minfps {
	int minfps_flag;
	u32 extend_frame;
};

/* External variable/function declaration */
extern bool g_is_silky_panel;
extern unsigned long long last_te_time;
extern unsigned long long last_rdma_start_time;
extern struct drm_device *get_drm_device(void);
/* for gki */
extern u32 oplus_adfr_config;
extern bool enable_multite;
extern inline bool oplus_adfr_is_support(void);
extern void oplus_adfr_vsync_switch(struct drm_display_mode *m, bool force_te_vsync);
extern int oplus_adfr_temperature_detection_handle(void *mtk_ddp_comp, void *cmdq_pkt, int ntc_temp, int shell_temp);
extern void oplus_adfr_status_reset(struct drm_display_mode *src_m, struct drm_display_mode *dst_m);
extern void oplus_adfr_set_multite_state(bool state);

#endif /* _OPLUS_ADFR_EXT_H_ */
