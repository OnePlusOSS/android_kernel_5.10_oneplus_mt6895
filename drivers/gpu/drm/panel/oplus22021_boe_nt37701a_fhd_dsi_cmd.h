#ifndef PANEL_SAMSUNG_AMB670YF07_H
#define PANEL_SAMSUNG_AMB670YF07_H

#include <linux/trace_events.h>
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/mtk_data_hw_roundedpattern_2k.h"
#endif

#define REGFLAG_CMD				0xFFFA
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

#define MODE_NUM	1

#define mtk_drm_trace_begin(fmt, args...) do { \
	if (g_trace_log) { \
		preempt_disable(); \
		event_trace_printk(mtk_drm_get_tracing_mark(), \
			"B|%d|"fmt"\n", current->tgid, ##args); \
		preempt_enable();\
	} \
} while (0)

#define mtk_drm_trace_end() do { \
	if (g_trace_log) { \
		preempt_disable(); \
		event_trace_printk(mtk_drm_get_tracing_mark(), "E\n"); \
		preempt_enable(); \
	} \
} while (0)

#define mtk_drm_trace_c(fmt, args...) do { \
	if (g_trace_log) { \
		preempt_disable(); \
		event_trace_printk(mtk_drm_get_tracing_mark(), \
			"C|"fmt"\n", ##args); \
		preempt_enable();\
	} \
} while (0)

//#ifdef OPLUS_ADFR
enum MODE_ID {
	FHD_SDC60 = 0,
};
//#endif

extern bool g_trace_log;
extern unsigned long mtk_drm_get_tracing_mark(void);

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

/* ------------------------- timming parameters ------------------------- */

/* --------------- DSC Setting --------------- */
/* FHD DSC1.2 10bit */
struct LCM_setting_table fhd_dsc_cmd[] = {

};

/* WQHD DSC1.2 10bit */
struct LCM_setting_table wqhd_dsc_cmd[] = {

};

/* --------------- timing@fhd_sdc_60 --------------- */
/* timing-switch cmd */
struct LCM_setting_table fhd_timing_switch_1_cmd_sdc60[] = {

};

struct LCM_setting_table fhd_timing_switch_2_cmd_sdc60[] = {

};

/* dsi-on cmd */
struct LCM_setting_table fhd_dsi_on_cmd_sdc60[] = {
	{REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x04}},
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x1D}},
    {REGFLAG_CMD, 2, {0xF2,0x05}},
    {REGFLAG_CMD, 2, {0x6F,0x20}},
    {REGFLAG_CMD, 2, {0xF7,0x32}},
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x0F}},
    {REGFLAG_CMD, 2, {0xFD,0x01}},
    {REGFLAG_CMD, 2, {0x6F,0x10}},
    {REGFLAG_CMD, 2, {0xFD,0x80}},
    {REGFLAG_CMD, 2, {0x35,0x00}},
    {REGFLAG_CMD, 2, {0x53,0x20}},
    {REGFLAG_CMD, 5, {0x51,0x0D,0xBB,0x0F,0xFE}},
    {REGFLAG_CMD, 5, {0x2A,0x00,0x00,0x04,0x37}},
    {REGFLAG_CMD, 5, {0x2B,0x00,0x00,0x09,0x6B}},
    {REGFLAG_CMD, 2, {0x82,0xAE}},
    {REGFLAG_CMD, 19,{0x91,0x89,0x28,0x00,0x0C,0xC2,0x00,0x02,0x0E,0x01,0x1F,0x00,0x07,0x08,0xBB,0x08,0x7A,0x10,0xF0}},
    {REGFLAG_CMD, 2, {0x03,0x01}},
    {REGFLAG_CMD, 2, {0x90,0x11}},
    {REGFLAG_CMD, 2, {0x2C,0x00}},
    {REGFLAG_CMD, 2, {0x2F,0x01}},
    {REGFLAG_CMD, 2, {0x11,0x00}},
    {REGFLAG_CMD, 120, {}},
    {REGFLAG_CMD, 1, {0x29}},
    {REGFLAG_CMD, 30, {}},
};

/* --------------- timing@wqhd_oplus_120 --------------- */
/* timing-switch cmd */
struct LCM_setting_table wqhd_timing_switch_cmd_oplus120[] = {
};

/* dsi-on cmd */
struct LCM_setting_table wqhd_dsi_on_cmd_oplus120[] = {
};

/* ------------------------- common parameters ------------------------- */

/* --------------- panel common --------------- */
/* aod/fod command */
struct LCM_setting_table aod_on_cmd[] = {

};

struct LCM_setting_table aod_off_cmd[] = {

};

struct LCM_setting_table aod_high_mode[] = {

};

struct LCM_setting_table aod_low_mode[] = {

};

struct LCM_setting_table hbm_on_cmd[] = {

};

struct LCM_setting_table hbm_off_cmd[] = {

};

/* loading effect */
struct LCM_setting_table lcm_seed_mode0[] = {

};

struct LCM_setting_table lcm_seed_mode1[] = {

};

struct LCM_setting_table lcm_seed_mode2[] = {

};

/* --------------- adfr common --------------- */
/* pre-switch cmd */
struct LCM_setting_table pre_switch_cmd[] = {

};

/* fake frame cmd, used for sdc 90/120 */
struct LCM_setting_table fakeframe_cmd[] = {

};

/* SDC Auto On */
struct LCM_setting_table auto_on_cmd[] = {

};

/* SDC Auto Off */
struct LCM_setting_table auto_off_cmd[] = {

};

/* SDC Auto Off Min Fps */
struct LCM_setting_table auto_off_minfps_cmd[] = {

};

/* SDC Auto On Min Fps */
struct LCM_setting_table auto_on_minfps_cmd[] = {

};

#endif
