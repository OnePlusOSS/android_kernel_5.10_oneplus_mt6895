/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/err.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <asm/current.h>
#include <linux/version.h>
#include <linux/unistd.h>
#include <linux/timekeeping.h>

#include "include/fp_health.h"

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOG_CORE)
#include <soc/oplus/system/olc.h>
#endif /* CONFIG_OPLUS_FEATURE_LOG_CORE */

#define FP_MODULE_NAME "fingerprint.driver"
#define FP_EXC_MODULE_ID 2
#define FP_EXC_RESERVED_ID 256

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOG_CORE)
int fp_olc_raise_exception(fp_exception_info_t *fp_exp_info) {
	int ret = -1;
	struct exception_info exp_info = {
		.time		  = fp_exp_info->time,
		.id			= fp_exp_info->id,
		.pid		   = fp_exp_info->pid,
		.exceptionType = fp_exp_info->exceptionType,
		.logOption	 = fp_exp_info->logOption,
		.faultLevel	= fp_exp_info->faultLevel,
		.logPath	   = "",
	};
	memcpy(exp_info.module, fp_exp_info->module, strlen(fp_exp_info->module));

	pr_info("%s enter to raise log exception.\n", __func__);
	ret = olc_raise_exception(&exp_info);

	return ret;
}
#else
int fp_olc_raise_exception(fp_exception_info_t *exp_info) {
	return 0;
}
#endif /* CONFIG_OPLUS_FEATURE_LOG_CORE */

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOG_CORE)
int fp_exception_report_drv(fp_scene_id_t scene_id) {
	int ret = -1;
	struct timespec64 _time = {};
	fp_exception_info_t fp_exp_info = {
		.time = _time.tv_sec,
		.id  = (FP_EXC_RESERVED_ID << 20) | (FP_EXC_MODULE_ID << 12) | scene_id,
		.pid = 0,
		.exceptionType = EXCEPTION_KERNEL,
		.logOption	 = LOG_MAIN | LOG_KERNEL,
		.module		= FP_MODULE_NAME,
		.faultLevel	= EXP_LEVEL_INFO,
		.logPath	   = "",
	};

	pr_info("%s enter to report log exception.\n", __func__);

	ktime_get_real_ts64(&_time);
	fp_exp_info.time = _time.tv_sec;

	ret = fp_olc_raise_exception(&fp_exp_info);

	return ret;
}
#else
int fp_exception_report_drv(fp_scene_id_t scene_id) {
	return 0;
}
#endif /* CONFIG_OPLUS_FEATURE_LOG_CORE */
