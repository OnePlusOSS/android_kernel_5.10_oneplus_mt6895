#include <soc/oplus/system/olc.h>
#include <linux/ktime.h>

#include "oplus_cam_olc_exception.h"

int cam_olc_raise_exception(int excep_tpye)
{
	struct exception_info exp_info = {};
	int ret = -1;
	struct timespec64 time = {};

	pr_info("%s: enter, type:%d\n", __func__, excep_tpye);

	if (excep_tpye > 0xf) {
		pr_err("%s: excep_tpye:%d is beyond 0xf\n", __func__ , excep_tpye);
		goto free_exp;
	}

	ktime_get_real_ts64(&time);

	exp_info.time = time.tv_sec;
	exp_info.exceptionId = CAM_RESERVED_ID << 20 | CAM_MODULE_ID << 12 | excep_tpye;
	exp_info.exceptionType = EXCEPTION_KERNEL;
	exp_info.level = EXP_LEVEL_CRITICAL;
	exp_info.atomicLogs = LOG_KERNEL | LOG_MAIN | LOG_CAMERA_EXPLORER;
	pr_err("camera exception:id=0x%x,time=%ld,level=%d,atomicLogs=0x%lx,logParams=%s\n",
				exp_info.exceptionId, exp_info.time, exp_info.level, exp_info.atomicLogs, exp_info.logParams);
	ret = olc_raise_exception(&exp_info);
	if (ret) {
		pr_err("err %s: raise fail, ret:%d\n", __func__ , ret);
	}

free_exp:
	return ret;
}

EXPORT_SYMBOL(cam_olc_raise_exception);
