// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/kobject.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/gpio.h>
#include <video/mipi_display.h>

#include "iris_api.h"
#include "iris_lightup.h"
#include "iris_lightup_ocp.h"
#include "iris_pq.h"
#include "iris_ioctl.h"
#include "iris_lut.h"
#include "iris_mode_switch.h"
#include "iris_log.h"

#define IRIS_DBG_TOP_DIR "iris"
#define IRIS_DBG_FUNCSTATUS_FILE "iris_func_status"
static struct kobject *iris_display_kobj;

int iris_dbg_fstatus_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;
	return 0;
}

/**
 * read module's status
 */
static ssize_t iris_dbg_fstatus_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char *kbuf = NULL;
	int size = count < PAGE_SIZE ? PAGE_SIZE : (int)count;
	int len = 0;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (*ppos)
		return 0;

	kbuf = vzalloc(size);
	if (kbuf == NULL) {
		IRIS_LOGE("Fatal erorr: No mem!\n");
		return -ENOMEM;
	}

	len += snprintf(kbuf, size,
			"iris function parameter info\n"
			"***peaking_setting***\n"
			"%-20s:\t%d\n",
			"peaking", pqlt_cur_setting->pq_setting.peaking);

	len += snprintf(kbuf+len, size - len,
			"***system_brightness***\n"
			"%-20s:\t%d\n",
			"system_brightness",
			pqlt_cur_setting->system_brightness);

	len += snprintf(kbuf+len, size - len,
			"***dspp_dirty***\n"
			"%-20s:\t%d\n",
			"dspp_dirty", pqlt_cur_setting->dspp_dirty);

	len += snprintf(kbuf+len, size - len,
			"***dbc_setting***\n"
			"%-20s:\t%d\n",
			"dbc", pqlt_cur_setting->pq_setting.dbc);

	len += snprintf(kbuf+len, size - len,
			"***lce_setting***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"mode", pqlt_cur_setting->pq_setting.lcemode,
			"level", pqlt_cur_setting->pq_setting.lcelevel,
			"graphics_detection",
			pqlt_cur_setting->pq_setting.graphicdet);

	len += snprintf(kbuf+len, size - len,
			"***cm_setting***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"cm6axis", pqlt_cur_setting->pq_setting.cm6axis,
			"cmftc", pqlt_cur_setting->cmftc,
			"cmcolortempmode",
			pqlt_cur_setting->pq_setting.cmcolortempmode,
			"colortempvalue", pqlt_cur_setting->colortempvalue,
			"min_colortempvalue",
			pqlt_cur_setting->min_colortempvalue,
			"max_colortempvalue",
			pqlt_cur_setting->max_colortempvalue,
			"cmcolorgamut",
			pqlt_cur_setting->pq_setting.cmcolorgamut,
			"demomode", pqlt_cur_setting->pq_setting.demomode,
			"source_switch", pqlt_cur_setting->source_switch);

	len += snprintf(kbuf+len, size - len,
			"***lux_value***\n"
			"%-20s:\t%d\n",
			"luxvalue", pqlt_cur_setting->luxvalue);

	len += snprintf(kbuf+len, size - len,
			"***cct_value***\n"
			"%-20s:\t%d\n",
			"cctvalue", pqlt_cur_setting->cctvalue);

	len += snprintf(kbuf+len, size - len,
			"***reading_mode***\n"
			"%-20s:\t%d\n",
			"readingmode", pqlt_cur_setting->pq_setting.readingmode);

	len += snprintf(kbuf+len, size - len,
			"***ambient_lut***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"al_en", pqlt_cur_setting->pq_setting.alenable,
			"al_luxvalue", pqlt_cur_setting->luxvalue,
			"al_bl_ratio", pqlt_cur_setting->al_bl_ratio);

	len += snprintf(kbuf+len, size - len,
			"***sdr2hdr***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"sdr2hdr", pqlt_cur_setting->pq_setting.sdr2hdr,
			"maxcll", pqlt_cur_setting->maxcll);

	len += snprintf(kbuf+len, size - len,
			"***frc_setting***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"memc_level", pcfg->frc_setting.memc_level,
			"memc_osd", pcfg->frc_setting.memc_osd,
			"in_fps", pcfg->frc_setting.in_fps,
			"out_fps", pcfg->frc_setting.out_fps,
			"in_fps_configured", pcfg->frc_setting.in_fps_configured,
			"low_latency", pcfg->frc_low_latency);

	len += snprintf(kbuf+len, size - len,
			"***osd***\n"
			"%-20s:\t%d\n",
			"osd_en", pcfg->osd_enable);

	len += snprintf(kbuf+len, size - len,
			"***analog_abypass***\n"
			"%-20s:\t%d\n",
			"abyp_mode", pcfg->abypss_ctrl.abypass_mode);

	len += snprintf(kbuf+len, size - len,
			"***n2m***\n"
			"%-20s:\t%d\n",
			"n2m_en", pcfg->n2m_enable);

	len += snprintf(kbuf+len, size - len,
			"***osd protect window***\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n",
			"osd0_tl", pcfg->frc_setting.iris_osd0_tl,
			"osd0_br", pcfg->frc_setting.iris_osd0_br,
			"osd1_tl", pcfg->frc_setting.iris_osd1_tl,
			"osd1_br", pcfg->frc_setting.iris_osd1_br,
			"osd2_tl", pcfg->frc_setting.iris_osd2_tl,
			"osd2_br", pcfg->frc_setting.iris_osd2_br,
			"osd3_tl", pcfg->frc_setting.iris_osd3_tl,
			"osd3_br", pcfg->frc_setting.iris_osd3_br,
			"osd4_tl", pcfg->frc_setting.iris_osd4_tl,
			"osd4_br", pcfg->frc_setting.iris_osd4_br,
			"osd_win_ctrl", pcfg->frc_setting.iris_osd_window_ctrl,
			"osd_win_ctrl", pcfg->frc_setting.iris_osd_win_dynCompensate);

	len += snprintf(kbuf+len, size - len,
			"***firmware_version***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d%d/%d/%d\n",
			"version", pcfg->app_version,
			"date", pcfg->app_date[3], pcfg->app_date[2], pcfg->app_date[1], pcfg->app_date[0]);

	size = len;
	if (len >= count)
		size = count - 1;

	if (copy_to_user(ubuf, kbuf, size)) {
		vfree(kbuf);
		return -EFAULT;
	}

	vfree(kbuf);

	*ppos += size;

	return size;
}


static const struct file_operations iris_dbg_fstatus_fops = {
	.open = iris_dbg_fstatus_open,
	.read = iris_dbg_fstatus_read,
};

void iris_display_mode_name_update(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg == NULL)
		return;

	strlcpy(pcfg->display_mode_name, "Not Set", sizeof(pcfg->display_mode_name));

	if (!iris_is_pt_mode()) {
		strlcpy(pcfg->display_mode_name, "ABYPASS", sizeof(pcfg->display_mode_name));
	} else {
		if (pcfg->osd_enable) {
			if (pcfg->pwil_mode == FRC_MODE)
				strlcpy(pcfg->display_mode_name, "DUAL-MEMC", sizeof(pcfg->display_mode_name));
			else
				strlcpy(pcfg->display_mode_name, "DUAL-PT", sizeof(pcfg->display_mode_name));
			return;
		}
		strlcpy(pcfg->display_mode_name, "PT", sizeof(pcfg->display_mode_name));
		if (pcfg->pwil_mode == FRC_MODE) {
			strlcpy(pcfg->display_mode_name, "MEMC", sizeof(pcfg->display_mode_name));
		} else if (pcfg->pwil_mode == RFB_MODE) {
			strlcpy(pcfg->display_mode_name, "RFB", sizeof(pcfg->display_mode_name));
		}
	}
}

static ssize_t iris_dbg_display_mode_show(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char *kbuf = NULL;
	int size = count < PAGE_SIZE ? PAGE_SIZE : count;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (*ppos)
		return 0;

	kbuf = vzalloc(size);
	if (kbuf == NULL) {
		IRIS_LOGE("Fatal erorr: No mem!\n");
		return -ENOMEM;
	}

	iris_display_mode_name_update();

	snprintf(kbuf, size,
			"%s\n", pcfg->display_mode_name);

	size = strlen(kbuf);
	if (size >= count)
		size = count - 1;

	if (copy_to_user(ubuf, kbuf, size)) {
		vfree(kbuf);
		return -EFAULT;
	}

	vfree(kbuf);

	*ppos += size;

	return size;
}

static ssize_t display_mode_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_display_mode_name_update();

	return snprintf(buf, PAGE_SIZE,
			"%s\n", pcfg->display_mode_name);
}

#define IRIS_ATTR(_name, _mode, _show, _store) \
struct kobj_attribute iris_attr_##_name = __ATTR(_name, _mode, _show, _store)
static IRIS_ATTR(display_mode, S_IRUGO|S_IWUSR, display_mode_show, NULL);

static struct attribute *iris_dev_attrs[] = {
    &iris_attr_display_mode.attr,
    NULL};

static const struct attribute_group iris_attr_group = {
    .attrs = iris_dev_attrs,
};

static const struct file_operations iris_dbg_dislay_mode_fops = {
	.open = simple_open,
	.read = iris_dbg_display_mode_show,
};

extern void iris_set_dsi_cmd_log(uint32_t);
static ssize_t _iris_dsi_cmd_log_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint_from_user(buff, count, 0, &val))
		return -EFAULT;

	iris_set_dsi_cmd_log(val);

	return count;
}

extern uint32_t iris_get_dsi_cmd_log(void);
static ssize_t _iris_dsi_cmd_log_read(
		struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;
	uint32_t len = 0;
	char bf[SZ_32];

	if (*ppos)
		return 0;

	val = iris_get_dsi_cmd_log();
	len += scnprintf(bf, SZ_32, "%u\n", val);

	len = min_t(size_t, count, len);
	if (copy_to_user(buff, bf, len))
		return -EFAULT;

	*ppos += len;

	return len;
}

static const struct file_operations iris_dsi_cmd_log_fops = {
	.open = simple_open,
	.write = _iris_dsi_cmd_log_write,
	.read = _iris_dsi_cmd_log_read,
};

int iris_dbgfs_status_init(void)
{
	struct iris_cfg *pcfg = NULL;
	int retval;
	pcfg = iris_get_cfg();

	iris_display_kobj = kobject_create_and_add(IRIS_DBG_TOP_DIR, kernel_kobj);
	if (!iris_display_kobj){
		IRIS_LOGW("sysfs create group iris dir error");
	}

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(iris_display_kobj, &iris_attr_group);
	if (retval) {
		kobject_put(iris_display_kobj);
		IRIS_LOGW("sysfs create group iris_dbg_display_mode_show error");
	}

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir(IRIS_DBG_TOP_DIR, NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("create dir for iris failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	if (debugfs_create_file(IRIS_DBG_FUNCSTATUS_FILE, 0644,
				pcfg->dbg_root, NULL,
				&iris_dbg_fstatus_fops) == NULL)
		IRIS_LOGE("create file func_status failed");

	if (debugfs_create_file("display_mode", 0644,
				pcfg->dbg_root, NULL,
				&iris_dbg_dislay_mode_fops) == NULL)
		IRIS_LOGE("create file display_mode failed");

	if (debugfs_create_file("dsi_cmd_log", 0644,
				pcfg->dbg_root, NULL,
				&iris_dsi_cmd_log_fops) == NULL)
		IRIS_LOGE("create file dsi_cmd_log failed");

	return 0;
}
void iris_dbgfs_status_deinit(void)
{
	kobject_put(iris_display_kobj);
}

