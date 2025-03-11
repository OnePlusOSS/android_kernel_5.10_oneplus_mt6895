/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_private_api.h
** Description : oplus display private api implement
** Version : 1.0
** Date : 2018/03/20
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Hu.Jie          2018/03/20        1.0           Build this moudle
**   Guo.Ling        2018/10/11        1.1           Modify for SDM660
**   Guo.Ling        2018/11/27        1.2           Modify for mt6779
******************************************************************/
#ifndef _OPLUS_DISPLAY_PRIVATE_API_H_
#define _OPLUS_DISPLAY_PRIVATE_API_H_

#include <linux/err.h>
#include <linux/of.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <drm/drm_device.h>
#include <linux/delay.h>

//#include <drm/drmP.h>
//#include <linux/soc/mediatek/mtk-cmdq.h>
//#include <linux/mailbox/mtk-cmdq-mailbox-ext.h>
#include <drm/drm_mipi_dsi.h>
#include <video/videomode.h>

#include "mtk_drm_drv.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_ddp.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_gem.h"
#include "mtk_drm_plane.h"
//#include "mtk_writeback.h"
//#include "mtk_fence.h"
//#include "mtk_sync.h"
#include "mtk_drm_session.h"
#include "mtk_dump.h"
#include "mtk_drm_fb.h"
#include "mtk_rect.h"
#include "mtk_drm_ddp_addon.h"
#include "mtk_drm_helper.h"
#include "mtk_drm_lowpower.h"
//#include "mtk_drm_fbdev.h"
#include "mtk_drm_assert.h"
#include "mtk_drm_mmp.h"
#include "mtk_disp_recovery.h"
#include "mtk_drm_arr.h"
#include "mtk_log.h"
#include "mtk_drm_trace.h"
#include "mtk_dsi.h"
#define OPLUS_MAX_BRIGHTNESS 4095
#define OPLUS_NORMAL_MAX_BRIGHTNESS 2047
#define OPLUS_MIN_BRIGHTNESS 0

/**
 * enum oplus_debug_log --       flags to control debug log; 1->enbale  0->disable
 * @OPLUS_DEBUG_LOG_DISABLED:    disable all debug log
 * @OPLUS_DEBUG_LOG_CMD:         dump register log
 * @OPLUS_DEBUG_LOG_BACKLIGHT:   backlight log
 * @OPLUS_DEBUG_LOG_COMMON:      common log
 * @OPLUS_DEBUG_LOG_OFP:         OFP log
 * @OPLUS_DEBUG_LOG_ADFR:        ADFR log
 * @OPLUS_DEBUG_LOG_TEMP_COMPENSATION:temp compensation log
 */
enum oplus_debug_log {
	OPLUS_DEBUG_LOG_DISABLED = 0,
	OPLUS_DEBUG_LOG_CMD = BIT(0),
	OPLUS_DEBUG_LOG_BACKLIGHT = BIT(1),
	OPLUS_DEBUG_LOG_COMMON = BIT(2),
	OPLUS_DEBUG_LOG_OFP = BIT(3),
	OPLUS_DEBUG_LOG_ADFR = BIT(4),
	OPLUS_DEBUG_LOG_TEMP_COMPENSATION = BIT(6),
};

enum oplus_display_trace_enable {
	OPLUS_DISPLAY_DISABLE_TRACE = 0,
	OPLUS_DISPLAY_OFP_TRACE_ENABLE = BIT(0),
	OPLUS_DISPLAY_ADFR_TRACE_ENABLE = BIT(1),
	OPLUS_DISPLAY_TEMP_COMPENSATION_TRACE_ENABLE = BIT(3),
};

/* aod_area begin */
struct aod_area {
	bool enable;
	int x;
	int y;
	int w;
	int h;
	int color;
	int bitdepth;
	int mono;
	int gray;
};

#define RAMLESS_AOD_AREA_NUM		6
#define RAMLESS_AOD_PAYLOAD_SIZE	100

int oplus_display_private_api_init(void);
void oplus_display_private_api_exit(void);
#endif /* _OPLUS_DISPLAY_PRIVATE_API_H_ */
