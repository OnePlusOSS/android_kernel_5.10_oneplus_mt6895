// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/notifier.h>
#include <linux/ktime.h>
#include <soc/oplus/system/kernel_fb.h>

#include "hf_manager.h"
#include "oplus_mha100kn.h"
#define OPLUS_HALL_TRACK_PAYLOAD_LEN		1024
#define OPLUS_HALL_TRACK_MSG_LEN		(OPLUS_HALL_TRACK_PAYLOAD_LEN * 2)
#define OPLUS_HALL_TRACK_LOG_TAG		"PSW_BSP_SENSOR"
#define OPLUS_HALL_TRACK_EVENT_ID		"20002"
#define OPLUS_HALL_TRACK_LOG_LEN		32
#define OPLUS_HALL_TRACK_EVENT_LEN		20

static struct sensor_info support_sensors[] = {
	{
		.sensor_type = SENSOR_TYPE_FOLD_HALL,
		.gain = 1,
		.name = "leather fhall",
		.vendor = "haechitech",
	}
};

static int oplus_hall_upload_kevent_data(int cnt, int32_t status)
{
	int ret = 0;
	int offset = 0;
	struct kernel_packet_info *dcs_info;
	char *log_tag = OPLUS_HALL_TRACK_LOG_TAG;
	char *event_id = OPLUS_HALL_TRACK_EVENT_ID;

	dcs_info = (struct kernel_packet_info *)kmalloc(sizeof(char) * OPLUS_HALL_TRACK_MSG_LEN, GFP_KERNEL);
	if (!dcs_info) {
		MHALL_ERR("%s: malloc err\n", __func__);
		return -ENOMEM;
	}

	memset(dcs_info, 0x0, sizeof(char) * OPLUS_HALL_TRACK_MSG_LEN);
	offset += snprintf(&dcs_info->payload[offset], OPLUS_HALL_TRACK_PAYLOAD_LEN - offset,
		"$$ReportCnt@@device_hall_status$$detailData@@%d %d", cnt, status);
	dcs_info->type = 1;
	memcpy(dcs_info->log_tag, log_tag, strlen(log_tag));
	dcs_info->log_tag[OPLUS_HALL_TRACK_LOG_LEN - 1] = '\0';
	memcpy(dcs_info->event_id, event_id, strlen(event_id));
	dcs_info->event_id[OPLUS_HALL_TRACK_EVENT_LEN - 1] = '\0';
	dcs_info->payload_length = offset + 1;
	MHALL_LOG("%s\n", dcs_info->payload);

	ret = fb_kevent_send_to_user(dcs_info);
	if (ret) {
		MHALL_LOG("fb_kevent_send_to_user failed\n");
		return ret;
	}
	kfree(dcs_info);

	return ret;
}

static int mha100kn_batch(struct hf_device *hfdev, int sensor_type,
	int64_t delay, int64_t latency)
{
	return 0;
}

static int mha100kn_sample(struct hf_device *hfdev)
{
	return 0;
}

static void hall_handle_report(struct hall_mha100kn_data *hall_data,
	uint8_t action, int32_t value)
{
	struct hf_device *hf_dev = hall_data->hf_dev;
	struct hf_manager *manager = hf_dev->manager;
	struct hf_manager_event event;

	memset(&event, 0, sizeof(struct hf_manager_event));
	event.timestamp = ktime_get_boottime_ns();
	event.sensor_type = SENSOR_TYPE_FOLD_HALL;
	event.action = action;
	event.word[0] = value;
	manager->report(manager, &event);
	manager->complete(manager);
}

static int mha100kn_flush(struct hf_device *hfdev, int sensor_type)
{
	int32_t value;
	struct hall_mha100kn_data *hall_data = NULL;
	struct platform_device *dev = hf_device_get_private_data(hfdev);

	if (dev == NULL) {
		MHALL_ERR("%s dev is null !\n", __func__);
		return 0;
	}
	hall_data = dev_get_drvdata(&dev->dev);
	if (hall_data == NULL) {
		MHALL_ERR("%s hall_data is null !\n", __func__);
		return 0;
	}

	value = gpio_get_value(hall_data->irq_gpio);

	hall_handle_report(hall_data, FLUSH_ACTION, value);

	return 0;
}

static int mha100kn_enable(struct hf_device *hfdev, int sensor_type, int en)
{
	int32_t value;
	struct hall_mha100kn_data *hall_data = NULL;
	struct platform_device *dev = hf_device_get_private_data(hfdev);

	if (dev == NULL) {
		MHALL_ERR("%s dev is null !\n", __func__);
		return 0;
	}
	hall_data = dev_get_drvdata(&dev->dev);
	if (hall_data == NULL) {
		MHALL_ERR("%s hall_data is null !\n", __func__);
		return 0;
	}

	value = gpio_get_value(hall_data->irq_gpio);

	hall_handle_report(hall_data, DATA_ACTION, value);

	return 0;
}

static void hall_optional_handle(struct hall_mha100kn_data *hall_data, hall_status status)
{
	static struct hall_mha100kn_data *last_hall = NULL;
	bool  need_report = false;
	int ret = 0;
	if (!hall_data) {
		return;
	}
	if (last_hall != NULL && last_hall != hall_data) {
		MHALL_ERR("%d--anather hall near now ,return , status = %d" ,hall_data->id, TYPE_HALL_NEAR == status ? 1 : 0);
		return;
	} else if (last_hall == NULL && status == TYPE_HALL_NEAR) {
		last_hall = hall_data;
		need_report = true;
	} else if (last_hall != NULL && status == TYPE_HALL_FAR && last_hall == hall_data) {
		last_hall = NULL;
		need_report = true;
	}
	if (need_report) {
		hall_handle_report(hall_data, DATA_ACTION, TYPE_HALL_NEAR == status ? 0 : 1);
		MHALL_LOG("id %d--status %d" ,hall_data->id, TYPE_HALL_NEAR == status ? 0 : 1);
		hall_data->fb_report_cnt++;
		ret = oplus_hall_upload_kevent_data(hall_data->fb_report_cnt, TYPE_HALL_NEAR == status ? 0 : 1);
		if (ret) {
			MHALL_ERR("upload_kevent_data err\n");
			return;
		}
	}

	return;
}

static irqreturn_t hall_interrupt_handler_func(int irq, void *dev)
{
	int val = 0;
	struct hall_mha100kn_data *hall_data = dev;

	mutex_lock(&hall_data->report_mutex);

	val = gpio_get_value(hall_data->irq_gpio);
	if (hall_data->active_low) {
		if ((TYPE_HALL_NEAR == hall_data->hall_status) && val) {
			hall_data->hall_status = TYPE_HALL_FAR;
			hall_optional_handle(hall_data, TYPE_HALL_FAR);
		} else if ((TYPE_HALL_FAR == hall_data->hall_status) && !val) {
			hall_data->hall_status = TYPE_HALL_NEAR;
			hall_optional_handle(hall_data, TYPE_HALL_NEAR);
		} else {
			hall_data->hall_status = (0 == val ? TYPE_HALL_NEAR : TYPE_HALL_FAR);
			MHALL_ERR("[logic wrong]%s hall_status:%d, gpio_val:%d.",hall_data->DEV_NAME, hall_data->hall_status, val);
			if (hall_data->hall_status == TYPE_HALL_NEAR) {
				hall_optional_handle(hall_data, hall_data->hall_status);
			}
		}
	} else {
		MHALL_ERR("[%s]not support non active low handle yet.", hall_data->DEV_NAME);
	}

	mutex_unlock(&hall_data->report_mutex);

	return IRQ_HANDLED;
}

static int hall_int_gpio_init(struct device *dev, struct hall_mha100kn_data *hall_data)
{
	char pinctrl_status[64] = {0};

	hall_data->hall_pinctrl = devm_pinctrl_get(dev);
	if (hall_data->hall_pinctrl == NULL) {
		MHALL_ERR("get pinctrl fail\n");
		return -EINVAL;
	}

	sprintf(pinctrl_status , "mha100kn_hall%d_active" , hall_data->id);
	MHALL_LOG("pinctrl_status:%s\n" , pinctrl_status);

	hall_data->hall_int_active =
				pinctrl_lookup_state(hall_data->hall_pinctrl, pinctrl_status);
	if (hall_data->hall_int_active == NULL) {
		MHALL_ERR("get hall_int_active fail\n");
		return -EINVAL;
	}
	MHALL_LOG("get %s success!\n" , pinctrl_status);
	sprintf(pinctrl_status , "mha100kn_hall%d_sleep" , hall_data->id);
	MHALL_LOG("pinctrl_status:%s\n" , pinctrl_status);
	hall_data->hall_int_sleep =
				pinctrl_lookup_state(hall_data->hall_pinctrl, pinctrl_status);
	if (hall_data->hall_int_sleep == NULL) {
		MHALL_ERR("get hall_int_sleep fail\n");
		return -EINVAL;
	}
	MHALL_LOG("get %s success!\n" , pinctrl_status);

	if (hall_data->irq_gpio > 0) {
		gpio_direction_input(hall_data->irq_gpio);
	}

	MHALL_LOG("pinctrl_select_state mha100kn_hall !\n");
	pinctrl_select_state(hall_data->hall_pinctrl, hall_data->hall_int_active);

	return 0;
}

static int hall_parse_dt(struct device *dev, struct hall_mha100kn_data *hall_data)
{
	int rc = 0;
	struct device_node *np = dev->of_node;

	hall_data->irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0, &(hall_data->irq_flags));

	hall_data->active_low = of_property_read_bool(np, "irq_active_low");
	rc = of_property_read_u32(np, "hall-id", &hall_data->id);
	if (rc) {
		MHALL_ERR("handle id not specified.\n");
		return rc;
	}
	sprintf(hall_data->DEV_NAME , "%s_%d" , HALL_DEV_NAME , hall_data->id);
	rc = hall_int_gpio_init(dev, hall_data);
	if (rc) {
		MHALL_ERR("gpio init fail.\n");
		return rc;
	}
	hall_data->irq_number = gpio_to_irq(hall_data->irq_gpio);
	MHALL_LOG("gpio %d hall_irq %d, flags = %d\n",hall_data->irq_gpio , hall_data->irq_number, hall_data->irq_flags);
	return 0;
}

static int mha100kn_probe(struct platform_device *dev)
{
	int err = 0;
	struct hf_device *hf_dev = NULL;
	struct hall_mha100kn_data *hall_data = NULL;

	hall_data = devm_kzalloc(&dev->dev, sizeof(*hall_data), GFP_KERNEL);
	if (hall_data == NULL) {
		MHALL_ERR("failed to allocate memory\n");
		err = -ENOMEM;
		goto return_err;
	}
	dev_set_drvdata(&dev->dev, hall_data);

	if (dev->dev.of_node) {
		err = hall_parse_dt(&dev->dev, hall_data);
		if (err) {
			dev_err(&dev->dev, "Failed to parse device tree\n");
			goto return_err;
		}
	} else if (dev->dev.platform_data != NULL) {
		memcpy(hall_data, dev->dev.platform_data, sizeof(*hall_data));
	} else {
		MHALL_ERR("No valid platform data, probe failed.\n");
		err = -ENODEV;
		goto return_err;
	}

	hf_dev = devm_kzalloc(&dev->dev, sizeof(struct hf_device), GFP_KERNEL);
	if (!hf_dev) {
		MHALL_ERR("failed to allocate hf_dev memory %d\n", err);
		err = -ENOMEM;
		goto return_err;
	}

	hf_dev->dev_name = HALL_DEV_NAME;
	hf_dev->device_poll = HF_DEVICE_IO_INTERRUPT;
	hf_dev->device_bus = HF_DEVICE_IO_ASYNC;
	hf_dev->support_list = support_sensors;
	hf_dev->support_size = ARRAY_SIZE(support_sensors);
	hf_dev->enable = mha100kn_enable;
	hf_dev->batch = mha100kn_batch;
	hf_dev->sample = mha100kn_sample;
	hf_dev->flush = mha100kn_flush;
	hf_device_set_private_data(hf_dev, dev);
	err = hf_device_register_manager_create(hf_dev);
	if (err < 0) {
		pr_err("%s hf_manager_create fail\n", __func__);
		goto return_err;
	}

	hall_data->hall_status = TYPE_HALL_FAR;
	hall_data->fb_report_cnt = 0;
	hall_data->hf_dev = hf_dev;
	mutex_init(&hall_data->report_mutex);

	/*** register irq handler ***/
	err = request_threaded_irq(hall_data->irq_number, NULL, hall_interrupt_handler_func, hall_data->irq_flags, hall_data->DEV_NAME, hall_data);
	if (err < 0) {
		MHALL_ERR("request irq handler failed.\n");
		goto distory_hf_dev;
	}
	enable_irq_wake(hall_data->irq_number);

	MHALL_LOG("mha100kn mhall probe ok\n");
	hall_interrupt_handler_func(hall_data->irq_number , hall_data);
	MHALL_LOG("mha100kn mhall_%d update first status ok\n" , hall_data->id);

	return 0;

distory_hf_dev:
	hf_device_unregister_manager_destroy(hf_dev);
return_err:
	return err;
}

static int mha100kn_remove(struct platform_device *dev)
{
	struct hall_mha100kn_data *hall_data = dev_get_drvdata(&dev->dev);

	hf_manager_destroy(hall_data->hf_dev->manager);
	kfree(dev);
	return 0;
}

static struct platform_device_id hall_mha100kn_id[] = {
	{HALL_DEV_NAME, 0 },
	{ },
};

static const struct of_device_id mha100kn_ids[] = {
	{.compatible = HALL_DEV_NAME},
	{},
};

static struct platform_driver mha100kn_driver = {
	.driver = {
		.name = HALL_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mha100kn_ids,
	},
	.probe = mha100kn_probe,
	.remove = mha100kn_remove,
	.id_table = hall_mha100kn_id,
};

static int __init mha100kn_hall_init(void)
{
	MHALL_LOG("mha100kn hall sensor start init.\n");
	platform_driver_register(&mha100kn_driver);
	return 0;
}

static void __exit mha100kn_hall_exit(void)
{
	platform_driver_unregister(&mha100kn_driver);
}


late_initcall(mha100kn_hall_init);
module_exit(mha100kn_hall_exit);
MODULE_AUTHOR("Mediatek");
MODULE_DESCRIPTION("mha100kn driver");
MODULE_LICENSE("GPL");

