// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2022 Oplus. All rights reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <linux/kfifo.h>

#include "oplus_cam_event_report.h"
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
#include <soc/oplus/dft/kernel_fb.h>
#endif

#define CAM_EVENT_NAME "cam-event"
#define DTS_COMP_CAM_EVENT_NAME "oplus,cam-event"
#define DEFAULT_REPORT_EVENT_TIME 30

static dev_t dev_no = 0;

#define BUFF_SIZE 1024
struct kfifo gkfifo;
static char cam_probe_fail_list[BUFF_SIZE];

struct cam_event_interf
{
	int id;
	int event;
	char  *name;
	struct device   *dev;
	char time[32];
	struct mutex lock;
	u64 expire;
};

struct cam_event_interf *cam_event_data;
static struct class *cam_class;

static void get_report_time(struct rtc_time *time)
{
	struct rtc_time tm;
	struct timespec64 tv = { 0 };
	/* android time */
	struct rtc_time *tm_android;
	struct timespec64 tv_android = { 0 };
	if(time) {
		tm_android = time;
	} else {
		pr_err("time is NLL");
		return;
	}

	ktime_get_real_ts64(&tv);
	tv_android = tv;
	rtc_time64_to_tm(tv.tv_sec, &tm);
	tv_android.tv_sec -= (uint64_t)sys_tz.tz_minuteswest * 60;
	rtc_time64_to_tm(tv_android.tv_sec, tm_android);
	pr_err("cam event date: %d-%02d-%02d %02d:%02d:%02d",
		tm_android->tm_year + 1900, tm_android->tm_mon + 1,
		tm_android->tm_mday, tm_android->tm_hour,
		tm_android->tm_min, tm_android->tm_sec);
}

void cam_event_report(int id, int event) {
	struct rtc_time time = { 0 };
	pr_err("cam %d report error event 0x%x", id, event);

	if(cam_event_data == NULL) {
		pr_err("cam_event_data is NULL.");
		return;
	}
	/* Block normal reporting, such as sensor i2c probe when sensor has secend supply. */
	if (cam_event_data->expire > jiffies) {
		pr_info("System startup do not report event.");
		return;
	}
	mutex_lock(&cam_event_data->lock);
	get_report_time(&time);
	snprintf(cam_event_data->time, 32, "%d%02d%02d%02d%02d%02d",
			time.tm_year + 1900, time.tm_mon + 1,
			time.tm_mday, time.tm_hour,
			time.tm_min, time.tm_sec);
	cam_event_data->id = id;
	cam_event_data->event = CAM_HARDWARE_EVENT_BASE + event;
	mutex_unlock(&cam_event_data->lock);
}
EXPORT_SYMBOL(cam_event_report);

static ssize_t hardware_exception_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t hardware_exception_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int id = 0;
	int event = 0;
	char time[32] = { 0 };
	strlcpy(time, cam_event_data->time, sizeof(cam_event_data->time));
	id = cam_event_data->id;
	event = cam_event_data->event;

	cam_event_data->id = 0;
	cam_event_data->event = 0;
	memset(cam_event_data->time, 0, sizeof(cam_event_data->time));

	return snprintf(buf, 32, "%d_%d_%s\n", event, id, time);
}

static DEVICE_ATTR(hardware_exception, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH,
		hardware_exception_show, hardware_exception_store);

void push_sensor_name(const char *sensorName)
{
	kfifo_in(&gkfifo, sensorName, strlen(sensorName));
	kfifo_in(&gkfifo, "-", 1);
}
EXPORT_SYMBOL(push_sensor_name);

static ssize_t kernel_fb_write(struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *lo)
{
	char *r_buf;
	char event_id[MAX_ID] = {0};
	int idx = 0;
	int len;

	pr_info("cam kernel_fb_write in");

	r_buf = (char *)kzalloc(MAX_BUF_LEN, GFP_KERNEL);
	if (!r_buf) {
		return count;
	}

	if (copy_from_user(r_buf, buf, MAX_BUF_LEN > count ? count : MAX_BUF_LEN)) {
		goto exit;
	}

	r_buf[MAX_BUF_LEN - 1] = '\0'; /*make sure last bype is eof*/
	len = strlen(r_buf);
	pr_info("cam kernel_fb_write r_buf=%s, len=%d", r_buf, len);

	while (idx < len) {
		if (r_buf[idx++] == ':') {
			break;
		}
	}
	if (idx == len) {
		pr_err("not fount event id");
		goto exit;
	}

	memcpy(event_id, &r_buf[0], idx > MAX_ID ? MAX_ID : idx - 1);
	pr_info("kernel_fb_write:event_id = %s, payload=%s", event_id, r_buf + idx);

	#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	oplus_kevent_fb(FB_CAMERA, event_id, r_buf + idx);
	#endif
exit:
	kfree(r_buf);
	return count;
}

static ssize_t kernel_fb_read(struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char rcv_data[BUFF_SIZE]={0};
	kfifo_out(&gkfifo, rcv_data, sizeof(rcv_data));
	pr_info("push_sensor_name rcv_data = %s, len = %d", rcv_data, strlen(rcv_data));
	if (strlen(rcv_data) == 0 || copy_to_user(buf, rcv_data, BUFF_SIZE)) {
		return 0;
	}
	return BUFF_SIZE;
}

static const struct proc_ops cam_kern_fb_fops = {
	.proc_write = kernel_fb_write,
	.proc_read  = kernel_fb_read,
	.proc_open  = simple_open,
};

static int cam_event_probe(struct platform_device *pdev)
{
	ssize_t ret = -EINVAL;
	struct device_node *np = NULL;
	struct proc_dir_entry *d_entry = NULL;

	pr_info("cam event report probe");
	np = of_find_compatible_node(NULL, NULL, DTS_COMP_CAM_EVENT_NAME);
	if (!np) {
		dev_err(&pdev->dev, "%s: compatible(%s) match failed.\n", __func__, DTS_COMP_CAM_EVENT_NAME);
		return PTR_ERR(np);
	}

	cam_class = class_create(THIS_MODULE, "oplus_cam");
	if (IS_ERR(cam_class)) {
		dev_err(&pdev->dev, "%s: Fail to creat cam class.\n", __func__);
		return PTR_ERR(cam_class);
	}

	cam_event_data = devm_kzalloc(&pdev->dev, sizeof(struct cam_event_interf), GFP_KERNEL);
	if (!cam_event_data) {
		dev_err(&pdev->dev, "Fail to kzalloc mem.\n");
		ret = -ENOMEM;
		goto err_alloc_data;
	}

	ret = alloc_chrdev_region(&dev_no, 0, 1, "cam_event");
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to allocate cam_event char device region\n");
		goto err_alloc_chrdev;
	}

	cam_event_data->name = "cam_event";
	cam_event_data->dev = device_create(cam_class, NULL, dev_no, cam_event_data, "%s", cam_event_data->name);
	if (IS_ERR(cam_event_data->dev)) {
		dev_err(&pdev->dev, "Fail to register cam_event device.\n");
		ret = -PTR_ERR(cam_event_data->dev);
		goto err_device_create;
	}

	ret = device_create_file(cam_event_data->dev, &dev_attr_hardware_exception);
	if (ret) {
		dev_err(&pdev->dev, "Fail to create camera event file.");
		goto err_device_file;
	}
	platform_set_drvdata(pdev, cam_event_data);

	cam_event_data->id = 0;
	cam_event_data->event = 0;
	cam_event_data->expire = jiffies + DEFAULT_REPORT_EVENT_TIME * HZ;
	mutex_init(&cam_event_data->lock);

	d_entry = proc_create_data("cam_kern_fb", 0666, NULL, &cam_kern_fb_fops, NULL);
	if (!d_entry) {
		pr_err("failed to create kern_fb node\n");
		return -ENODEV;
	}
	kfifo_init(&gkfifo, cam_probe_fail_list, BUFF_SIZE);
	return 0;

err_device_file:
	device_destroy(cam_class, dev_no);
err_device_create:
	unregister_chrdev_region(dev_no, 1);
err_alloc_chrdev:
	devm_kfree(&pdev->dev, cam_event_data);
err_alloc_data:
	class_destroy(cam_class);

	return ret;
}

static int cam_event_remove(struct platform_device *pdev)
{
	unregister_chrdev_region(dev_no, 1);
	device_destroy(cam_class, dev_no);
	class_destroy(cam_class);
	cam_event_data = NULL;
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id cam_event_of_match[] = {
	{
		.compatible = DTS_COMP_CAM_EVENT_NAME,
		.data = NULL,
	},
	{},
};
MODULE_DEVICE_TABLE(of, cam_event_of_match);
#endif

static struct platform_driver cam_event_platform_driver = {
	.probe  = cam_event_probe,
	.remove = cam_event_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = CAM_EVENT_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(cam_event_of_match),
#endif
	},
};

static int __init cam_event_report_init(void)
{
	int ret;

	pr_info("camera debug Init start.");

	ret = platform_driver_register(&cam_event_platform_driver);
	if (ret) {
		class_destroy(cam_class);
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_info("camera debug Init done.");

	return 0;
}

static void __exit cam_event_report_exit(void)
{
	platform_driver_unregister(&cam_event_platform_driver);
}


late_initcall(cam_event_report_init);
module_exit(cam_event_report_exit);

MODULE_DESCRIPTION("camera report hareware error to HAL");
MODULE_LICENSE("GPL v2");
