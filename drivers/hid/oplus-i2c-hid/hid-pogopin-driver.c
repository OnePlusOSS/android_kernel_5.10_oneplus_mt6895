// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID support for pogopin keyboard
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/hid.h>

extern bool kb_connected;

static bool hid_pogopin_match(struct hid_device *hdev, bool ignore_special_driver)
{
	return true;
}

static int hid_pogopin_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret = 0;

	hdev->quirks |= HID_QUIRK_INPUT_PER_APP;

	ret = hid_parse(hdev);
	if (ret) {
		dev_err(&hdev->dev, "%s: hid_parse fail:ret = %d\n", __func__, ret);
		return ret;
	}
	if (kb_connected) {
		ret = hid_hw_start(hdev, HID_CONNECT_HIDINPUT);
		if(ret)
			dev_err(&hdev->dev, "%s: hid_hw_start fail:ret = %d\n", __func__, ret);
	}

	dev_err(&hdev->dev, "%s: kb_connected[%d]. end\n", __func__, kb_connected);
	return ret;
}

static const struct hid_device_id pogopin_hid_table[] = {
	{ HID_DEVICE(HID_BUS_ANY, HID_GROUP_ANY, 0x17EF, 0x610B) },
	{ HID_DEVICE(HID_BUS_ANY, HID_GROUP_ANY, 0x22D9, 0x3868) },
	{ }
};
MODULE_DEVICE_TABLE(hid, pogopin_hid_table);

static struct hid_driver hid_pogopin = {
	.name = "hid-pogopin",
	.id_table = pogopin_hid_table,
	.match = hid_pogopin_match,
	.probe = hid_pogopin_probe,
};
module_hid_driver(hid_pogopin);

MODULE_DESCRIPTION("HID pogopin driver");
MODULE_LICENSE("GPL");
