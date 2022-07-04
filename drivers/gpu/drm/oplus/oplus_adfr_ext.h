/***************************************************************
** Copyright (C),  2021,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_adfr_ext.h
** Description : ADFR kernel module
** Version : 1.0
** Date : 2021/07/09
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Gaoxiaolei      2021/07/09        1.0         Build this moudle
******************************************************************/

#ifndef _OPLUS_ADFR_EXT_H_
#define _OPLUS_ADFR_EXT_H_

extern bool g_adfr_log;

#define ADFRINFO(fmt, arg...)	\
	do {	\
		if (g_adfr_log)	\
			pr_info("[KVRR]"pr_fmt(fmt), ##arg);	\
	} while (0)

#define ADFRFUNC(fmt, arg...)	\
	pr_info("[KVRR][%s %d]"pr_fmt(fmt), __func__, __LINE__, ##arg)

#define ADFRERR(fmt, arg...)	\
	pr_err("[KVRR]"pr_fmt(fmt), ##arg)

#define ADFRWARN(fmt, arg...)	\
	pr_warn("[KVRR]"pr_fmt(fmt), ##arg)

#define VREFRESH_120_NS	(8333333)
#define VREFRESH_90_NS	(11111111)

#define SDC_AUTO_MIN_FPS_CMD_OFFSET 2
#define SDC_MANUAL_MIN_FPS_CMD_OFFSET 1
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
	OPLUS_ADFR_AUTO_MIN_FPS_MAX = 0,
	OPLUS_ADFR_AUTO_MIN_FPS_60HZ = 1,
};

struct oplus_minfps {
	int minfps_flag;
	u32 extend_frame;
};

/* External variable/function declaration */
extern unsigned long long last_te_time;
extern unsigned long long last_rdma_start_time;
extern struct drm_device *get_drm_device(void);
/* for gki */
extern u32 oplus_adfr_config;
extern inline bool oplus_adfr_is_support(void);
extern void oplus_adfr_vsync_switch(struct drm_display_mode *m, bool force_te_vsync);
extern void oplus_adfr_status_reset(struct drm_display_mode *src_m, struct drm_display_mode *dst_m);

#endif /* _OPLUS_ADFR_EXT_H_ */
