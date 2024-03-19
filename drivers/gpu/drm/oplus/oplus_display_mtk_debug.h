/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_mtk_debug.h
** Description : oplus display mtk debug
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Xiaolei.Gao       2020/12/06        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_MTK_DEBUG_H_
#define _OPLUS_DISPLAY_MTK_DEBUG_H_

#include <linux/fs.h>
#include <linux/printk.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ktime.h>

#define BACKLIGHT_CACHE_MAX 50

enum oplus_display_id {
	DISPLAY_PRIMARY = 0,
	DISPLAY_SECONDARY = 1,
	DISPLAY_MAX,
};
enum {
	MTK_LOG_LEVEL_MOBILE_LOG = 0x1,
	MTK_LOG_LEVEL_DETAIL_LOG = 0x2,
	MTK_LOG_LEVEL_FENCE_LOG = 0x4,
	MTK_LOG_LEVEL_IRQ_LOG = 0x8,
	MTK_LOG_LEVEL_TRACE_LOG = 0x10,
	MTK_LOG_LEVEL_DUMP_REGS = 0x20,
};

enum oplus_disp_drv_log_level {
	OPLUS_DISP_DRV_LOG_LEVEL_ERR = 0,
	OPLUS_DISP_DRV_LOG_LEVEL_WARN = 1,
	OPLUS_DISP_DRV_LOG_LEVEL_INFO = 2,
	OPLUS_DISP_DRV_LOG_LEVEL_DEBUG = 3,
};

static struct backlight_log {
	u32 bl_count;
	unsigned int backlight[BACKLIGHT_CACHE_MAX];
	struct timespec64 past_times[BACKLIGHT_CACHE_MAX];
}oplus_bl_log[DISPLAY_MAX];

/* log level config */
extern int oplus_disp_drv_log_level;
extern bool g_mobile_log;
extern bool g_detail_log;

/* just use for ofp */
#define DISP_ERR(fmt, arg...)	\
	do {	\
		if (oplus_disp_drv_log_level >= OPLUS_DISP_DRV_LOG_LEVEL_ERR)	\
			pr_err("[DISP_DRV][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define DISP_WARN(fmt, arg...)	\
	do {	\
		if (oplus_disp_drv_log_level >= OPLUS_DISP_DRV_LOG_LEVEL_WARN)	\
			pr_warn("[DISP_DRV][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define DISP_INFO(fmt, arg...)	\
	do {	\
		if ((oplus_disp_drv_log_level >= OPLUS_DISP_DRV_LOG_LEVEL_INFO)) \
			pr_info("[DISP_DRV][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define DISP_DEBUG(fmt, arg...)	\
	do {	\
		if ((oplus_disp_drv_log_level >= OPLUS_DISP_DRV_LOG_LEVEL_DEBUG) || (g_mobile_log)) \
			pr_info("[DISP_DRV][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#endif /*_OPLUS_DISPLAY_MTK_DEBUG_H_*/
