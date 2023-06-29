#include <linux/ktime.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OLC)
#include <soc/oplus/system/olc.h>
#endif
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
#include <soc/oplus/dft/kernel_fb.h>
#endif
#include "oplus_cam_olc_exception.h"

static struct cam_fb_conf g_kernel_fb_conf[] = {
    {EXCEP_PROBE,   	"CAM_EXCEP_PROBE", 				"EXP_HARDWARE"},
    {EXCEP_CLOCK,   	"CAM_EXCEP_CLOCK",				"EXP_HARDWARE"},
    {EXCEP_VOLTAGE, 	"CAM_EXCEP_VOLTAGE",			"EXP_HARDWARE"},
    {EXCEP_GPIO,    	"CAM_EXCEP_GPIO", 				"EXP_HARDWARE"},
    {EXCEP_I2C,     	"CAM_EXCEP_I2C", 				"EXP_HARDWARE"},
    {EXCEP_SOF_TIMEOUT,	"CAM_EXCEP_SOF_TIMEOUT",		"EXP_HARDWARE"}
};

static int find_event_id_index(int excep_id)
{
	int len = sizeof(g_kernel_fb_conf) / sizeof(g_kernel_fb_conf[0]);
	int ret = -1;
	int index = 0;

	for (index = 0; index < len; index++) {
		if (g_kernel_fb_conf[index].excepId == excep_id) {
			return index;
		}
	}
	return ret;
}

const unsigned char* acquireEventField(int excepId)
{
	int len = sizeof(g_kernel_fb_conf) / sizeof(g_kernel_fb_conf[0]);
	int index = 0;
	for (index = 0; index < len; index++) {
		if (g_kernel_fb_conf[index].excepId == excepId) {
			return g_kernel_fb_conf[index].fb_field;
		}
	}
	return NULL;
}
EXPORT_SYMBOL(acquireEventField);

int cam_olc_raise_exception(int excep_tpye, unsigned char* pay_load)
{
	#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OLC)
	struct exception_info exp_info = {};
	#endif
	int ret = -1;
	struct timespec64 time = {};
	pr_info("%s: enter, type:%d\n", __func__, excep_tpye);

	if (excep_tpye > 0xf) {
		pr_err("%s: excep_tpye:%d is beyond 0xf\n", __func__ , excep_tpye);
		goto free_exp;
	}

	ktime_get_real_ts64(&time);
	#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OLC)
	exp_info.time = time.tv_sec;
	exp_info.exceptionId = CAM_RESERVED_ID << 20 | CAM_MODULE_ID << 12 | excep_tpye;
	exp_info.exceptionType = EXCEPTION_KERNEL;
	exp_info.level = EXP_LEVEL_CRITICAL;
	exp_info.atomicLogs = LOG_KERNEL | LOG_ANDROID | LOG_CAMERA_EXPLORER;
	pr_err("camera exception:id=0x%x,time=%ld,level=%d,atomicLogs=0x%lx,logParams=%s\n",
				exp_info.exceptionId, exp_info.time, exp_info.level, exp_info.atomicLogs, exp_info.logParams);
	ret = olc_raise_exception(&exp_info);
	#endif
	if (ret) {
		pr_err("err %s: raise fail, ret:%d\n", __func__ , ret);
	}

	// camera excep feedback
	if (pay_load) {
		int index = find_event_id_index(excep_tpye);
		// at least 30s between two times fb
		if (index == -1 || (time.tv_sec - g_kernel_fb_conf[index].record_time < 30)) {
			pr_err("not find event_id = %d\n", excep_tpye);
			goto free_exp;
		}
		g_kernel_fb_conf[index].record_time = time.tv_sec;
		#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
		oplus_kevent_fb(FB_CAMERA, g_kernel_fb_conf[index].fb_event_id, pay_load);
		#endif
	}
free_exp:
	return ret;
}

EXPORT_SYMBOL(cam_olc_raise_exception);
