/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_display_temp_compensation.c
** Description : oplus_display_temp_compensation implement
** Version : 1.0
** Date : 2022/11/20
** Author : Display
***************************************************************/

#include <linux/thermal.h>
#include "mtk_panel_ext.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_drv.h"
#include "mtk_dsi.h"
#include "oplus_display_temp_compensation.h"
#include "oplus_display_onscreenfingerprint.h"
#include "oplus_adfr_ext.h"

/* -------------------- macro -------------------- */
/* config bit setting */
#define OPLUS_TEMP_COMPENSAITON_CONFIG_ENABLE				(BIT(0))
#define OPLUS_TEMP_COMPENSAITON_CONFIG_DRY_RUN				(BIT(1))
#define OPLUS_TEMP_COMPENSAITON_CONFIG_DATA_UPDATE			(BIT(2))
/* get config value */
#define OPLUS_TEMP_COMPENSATION_GET_ENABLE_CONFIG(config)	((config) & OPLUS_TEMP_COMPENSAITON_CONFIG_ENABLE)
#define OPLUS_TEMP_COMPENSATION_GET_DRY_RUN_CONFIG(config)	((config) & OPLUS_TEMP_COMPENSAITON_CONFIG_DRY_RUN)
#define OPLUS_TEMP_COMPENSATION_GET_DATA_UPDATE_CONFIG(config)	((config) & OPLUS_TEMP_COMPENSAITON_CONFIG_DATA_UPDATE)

#define OPLUS_TEMP_COMPENSAITON_MAX_INIT_RETRY_TIME			100
#define REGFLAG_CMD											0xFFFA

/* -------------------- parameters -------------------- */
struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

/* log level config */
unsigned int oplus_temp_compensation_log_level = OPLUS_TEMP_COMPENSATION_LOG_LEVEL_DEBUG;
EXPORT_SYMBOL(oplus_temp_compensation_log_level);
/* temp compensation global structure */
static struct oplus_temp_compensation_params g_oplus_temp_compensation_params = {0};

/* ntc_resistance:100k internal_pull_up:100k voltage:1.84v */
static int con_temp_ntc_100k_1840mv[] = {
	-40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22,
	-21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
	26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
	50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
	74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
	98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
	117, 118, 119, 120, 121, 122, 123, 124, 125
};

static int con_volt_ntc_100k_1840mv[] = {
	1799, 1796, 1793, 1790, 1786, 1782, 1778, 1774, 1770, 1765, 1760, 1755, 1749, 1743, 1737, 1731,
	1724, 1717, 1709, 1701, 1693, 1684, 1675, 1666, 1656, 1646, 1635, 1624, 1612, 1600, 1588, 1575,
	1561, 1547, 1533, 1518, 1503, 1478, 1471, 1454, 1437, 1420, 1402, 1384, 1365, 1346, 1327, 1307,
	1287, 1267, 1246, 1225, 1204, 1183, 1161, 1139, 1118, 1096, 1074, 1052, 1030, 1008, 986, 964,
	942, 920, 898, 877, 855, 834, 813, 793, 772, 752, 732, 712, 693, 674, 655, 637, 619, 601, 584,
	567, 550, 534, 518, 503, 488, 473, 459, 445, 431, 418, 405, 392, 380, 368, 357, 345, 335, 324,
	314, 304, 294, 285, 276, 267, 259, 251, 243, 235, 227, 220, 213, 206, 200, 194, 187, 182, 176,
	170, 165, 160, 155, 150, 145, 140, 136, 132, 128, 124, 120, 117, 113, 110, 106, 103, 100, 97,
	94, 91, 88, 86, 83, 81, 78, 76, 74, 72, 70, 68, 66, 64, 62, 60, 58, 57, 55, 54, 52, 51, 49, 48,
	47, 45
};

/* -------------------- extern -------------------- */
/* extern params */
extern unsigned int hpwm_mode;
extern unsigned int oplus_display_brightness;

/* extern functions */
extern void ddic_dsi_send_cmd(unsigned int cmd_num, char val[20]);
extern void mtk_read_ddic_v2(u8 ddic_reg, int ret_num, char ret_val[10]);
extern void mtk_crtc_cmdq_timeout_cb(struct cmdq_cb_data data);

/* -------------------- function implementation -------------------- */
static struct oplus_temp_compensation_params *oplus_temp_compensation_get_params(void)
{
	return &g_oplus_temp_compensation_params;
}

bool oplus_temp_compensation_is_supported(void)
{
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return false;
	}

	/* global config */
	return (bool)(OPLUS_TEMP_COMPENSATION_GET_ENABLE_CONFIG(p_oplus_temp_compensation_params->config));
}

static bool oplus_temp_compensation_dry_run_is_enabled(void)
{
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_temp_compensation_is_supported()) {
		TEMP_COMPENSATION_DEBUG("temp compensation is not support, dry run is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_TEMP_COMPENSATION_GET_DRY_RUN_CONFIG(p_oplus_temp_compensation_params->config));
}

static bool oplus_temp_compensation_data_update_is_enabled(void)
{
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_temp_compensation_is_supported()) {
		TEMP_COMPENSATION_DEBUG("temp compensation is not support, data updating is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_TEMP_COMPENSATION_GET_DATA_UPDATE_CONFIG(p_oplus_temp_compensation_params->config));
}

int oplus_temp_compensation_init(void *device)
{
	static bool registered = false;
	const char *data1 = NULL;
	unsigned char *data2 = NULL;
	int rc = 0;
	int length = 0;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int k = 0;
	unsigned int value = 0;
	static unsigned int failure_count = 0;
	struct device *dev = device;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (registered) {
		TEMP_COMPENSATION_DEBUG("temp compensation has been already initialized\n");
		return rc;
	}

	if (IS_ERR_OR_NULL(dev) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid dev or p_oplus_temp_compensation_params params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_init");

	/* oplus,temp-compensation-config */
	rc = of_property_read_u32(dev->of_node, "oplus,temp-compensation-config", &value);
	if (rc) {
		TEMP_COMPENSATION_INFO("failed to read oplus,temp-compensation-config, rc=%d\n", rc);
	} else {
		p_oplus_temp_compensation_params->config = value;
	}
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_config:0x%x\n", p_oplus_temp_compensation_params->config);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_config", p_oplus_temp_compensation_params->config);

	if (oplus_temp_compensation_is_supported()) {
		/* oplus,temp-compensation-reg-repeat */
		rc = of_property_read_u32(dev->of_node, "oplus,temp-compensation-reg-repeat", &value);
		if (rc) {
			TEMP_COMPENSATION_INFO("failed to read oplus,temp-compensation-reg-repeat, rc=%d, set reg_repeat to default value 4\n", rc);
			p_oplus_temp_compensation_params->reg_repeat = 4;
		} else {
			p_oplus_temp_compensation_params->reg_repeat = value;
		}
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_reg_repeat:%u\n", p_oplus_temp_compensation_params->reg_repeat);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_reg_repeat", p_oplus_temp_compensation_params->reg_repeat);

		/* oplus,temp-compensation-dbv-group */
		length = of_property_count_elems_of_size(dev->of_node, "oplus,temp-compensation-dbv-group", sizeof(unsigned int));
		if (length < 0) {
			TEMP_COMPENSATION_ERR("failed to get the count of oplus,temp-compensation-dbv-group\n");
			rc = -EINVAL;
			goto error;
		}
		p_oplus_temp_compensation_params->dbv_group = kzalloc(length * sizeof(unsigned int), GFP_KERNEL);
		if (!p_oplus_temp_compensation_params->dbv_group) {
			TEMP_COMPENSATION_ERR("failed to kzalloc dbv_group\n");
			rc = -ENOMEM;
			goto error;
		}
		rc = of_property_read_u32_array(dev->of_node, "oplus,temp-compensation-dbv-group", p_oplus_temp_compensation_params->dbv_group, length);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to read oplus,temp-compensation-dbv-group\n");
			rc = -EINVAL;
			goto error_free_dbv_group;
		}
		TEMP_COMPENSATION_INFO("property:oplus,temp-compensation-dbv-group,length:%u\n", length);
		for (i = 0; i < length; i++) {
			TEMP_COMPENSATION_INFO("dbv_group[%u]=%u\n", i, p_oplus_temp_compensation_params->dbv_group[i]);
		}
		p_oplus_temp_compensation_params->dbv_group_count = length + 1;
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_dbv_group_count:%u\n", p_oplus_temp_compensation_params->dbv_group_count);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_dbv_group_count", p_oplus_temp_compensation_params->dbv_group_count);

		/* oplus,temp-compensation-temp-group */
		length = of_property_count_elems_of_size(dev->of_node, "oplus,temp-compensation-temp-group", sizeof(int));
		if (length < 0) {
			TEMP_COMPENSATION_ERR("failed to get the count of oplus,temp-compensation-temp-group\n");
			rc = -EINVAL;
			goto error_free_dbv_group;
		}
		p_oplus_temp_compensation_params->temp_group = kzalloc(length * sizeof(int), GFP_KERNEL);
		if (!p_oplus_temp_compensation_params->temp_group) {
			TEMP_COMPENSATION_ERR("failed to kzalloc temp_group\n");
			rc = -ENOMEM;
			goto error_free_dbv_group;
		}
		rc = of_property_read_u32_array(dev->of_node, "oplus,temp-compensation-temp-group", (unsigned int *)p_oplus_temp_compensation_params->temp_group, length);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to read oplus,temp-compensation-temp-group\n");
			rc = -EINVAL;
			goto error_free_temp_group;
		}
		TEMP_COMPENSATION_INFO("property:oplus,temp-compensation-temp-group,length:%u\n", length);
		for (i = 0; i < length; i++) {
			TEMP_COMPENSATION_INFO("temp_group[%u]=%d\n", i, p_oplus_temp_compensation_params->temp_group[i]);
		}
		p_oplus_temp_compensation_params->temp_group_count = length + 1;
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_temp_group_count:%u\n", p_oplus_temp_compensation_params->temp_group_count);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_temp_group_count", p_oplus_temp_compensation_params->temp_group_count);

		/* oplus,temp-compensation-data */
		data1 = of_get_property(dev->of_node, "oplus,temp-compensation-data", &length);
		if (!data1) {
			TEMP_COMPENSATION_ERR("failed to get oplus,temp-compensation-data\n");
			rc = -ENOTSUPP;
			goto error_free_temp_group;
		}
		TEMP_COMPENSATION_INFO("property:oplus,temp-compensation-data,length:%u\n", length);
		if (length % (p_oplus_temp_compensation_params->dbv_group_count * p_oplus_temp_compensation_params->temp_group_count)) {
			TEMP_COMPENSATION_ERR("format error\n");
			rc = -EINVAL;
			goto error_free_temp_group;
		}
		p_oplus_temp_compensation_params->voltage_group_count =
			length / (p_oplus_temp_compensation_params->dbv_group_count * p_oplus_temp_compensation_params->temp_group_count);
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_voltage_group_count:%u\n", p_oplus_temp_compensation_params->voltage_group_count);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_voltage_group_count", p_oplus_temp_compensation_params->voltage_group_count);
		data2 = kzalloc(length * sizeof(unsigned char), GFP_KERNEL);
		if (!data2) {
			TEMP_COMPENSATION_ERR("failed to kzalloc data2\n");
			rc = -ENOMEM;
			goto error_free_temp_group;
		}
		memcpy(data2, data1, length * sizeof(unsigned char));
		p_oplus_temp_compensation_params->data = kzalloc(p_oplus_temp_compensation_params->dbv_group_count * sizeof(unsigned char**), GFP_KERNEL);
		if (!p_oplus_temp_compensation_params->data) {
			TEMP_COMPENSATION_ERR("failed to kzalloc data\n");
			rc = -ENOMEM;
			goto error_free_data2;
		}
		for (i = 0; i < p_oplus_temp_compensation_params->dbv_group_count; i++) {
			p_oplus_temp_compensation_params->data[i] = kzalloc(p_oplus_temp_compensation_params->temp_group_count * sizeof(unsigned char*), GFP_KERNEL);
			if (!p_oplus_temp_compensation_params->data[i]) {
				TEMP_COMPENSATION_ERR("failed to kzalloc data[%u]\n", i);
				rc = -ENOMEM;
				goto error_free_data;
			}
			for (j = 0; j < p_oplus_temp_compensation_params->temp_group_count; j++) {
				p_oplus_temp_compensation_params->data[i][j] =
					&data2[i * p_oplus_temp_compensation_params->temp_group_count * p_oplus_temp_compensation_params->voltage_group_count +
							j * p_oplus_temp_compensation_params->voltage_group_count + 0];
				for (k = 0; k < (p_oplus_temp_compensation_params->voltage_group_count); k++) {
					TEMP_COMPENSATION_DEBUG("data[%u][%u][%u]=0x%02X\n", i, j, k, p_oplus_temp_compensation_params->data[i][j][k]);
				}
			}
		}

		/* register ntc channel */
		rc = oplus_temp_compensation_register_ntc_channel(dev);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to register ntc channel\n");
			rc = -EINVAL;
			goto error_free_data;
		}

		registered = true;
		TEMP_COMPENSATION_INFO("init temp compensation successfully\n");
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_init");

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;

error_free_data:
	for (i = i - 1; i >= 0; i--) {
		kfree(p_oplus_temp_compensation_params->data[i]);
		p_oplus_temp_compensation_params->data[i] = NULL;
	}
	kfree(p_oplus_temp_compensation_params->data);
	p_oplus_temp_compensation_params->data = NULL;
error_free_data2:
	kfree(data2);
	data2 = NULL;
error_free_temp_group:
	kfree(p_oplus_temp_compensation_params->temp_group);
	p_oplus_temp_compensation_params->temp_group = NULL;
error_free_dbv_group:
	kfree(p_oplus_temp_compensation_params->dbv_group);
	p_oplus_temp_compensation_params->dbv_group = NULL;
error:
	p_oplus_temp_compensation_params->config = 0;
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_config:0x%x\n", p_oplus_temp_compensation_params->config);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_config", p_oplus_temp_compensation_params->config);

	/* set max retry time to OPLUS_TEMP_COMPENSAITON_MAX_INIT_RETRY_TIME */
	if (!registered) {
		failure_count++;
		TEMP_COMPENSATION_INFO("failure_count:%u\n", failure_count);
		if (failure_count == OPLUS_TEMP_COMPENSAITON_MAX_INIT_RETRY_TIME) {
			registered = true;
		}
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_init");

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}
EXPORT_SYMBOL(oplus_temp_compensation_init);

int oplus_temp_compensation_register_ntc_channel(void *device)
{
	int rc = 0;
	unsigned int i = 0;
	struct device *dev = device;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(dev) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid dev or p_oplus_temp_compensation_params params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_register_ntc_channel");

	for (i = 0; i < 5; i++) {
		p_oplus_temp_compensation_params->ntc_temp_chan = devm_iio_channel_get(dev, "panel-channel");
		rc = IS_ERR_OR_NULL(p_oplus_temp_compensation_params->ntc_temp_chan);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to get panel channel\n");
			p_oplus_temp_compensation_params->ntc_temp_chan = NULL;
		} else {
			p_oplus_temp_compensation_params->ntc_temp = 29;
			TEMP_COMPENSATION_INFO("oplus_temp_compensation_ntc_temp:%d\n", p_oplus_temp_compensation_params->ntc_temp);
			OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_ntc_temp", p_oplus_temp_compensation_params->ntc_temp);
			p_oplus_temp_compensation_params->shell_temp = 29;
			TEMP_COMPENSATION_INFO("oplus_temp_compensation_shell_temp:%d\n", p_oplus_temp_compensation_params->shell_temp);
			OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_shell_temp", p_oplus_temp_compensation_params->shell_temp);
			TEMP_COMPENSATION_INFO("register ntc channel successfully\n");
			break;
		}
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_register_ntc_channel");

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}

static int oplus_temp_compensation_volt_to_temp(int volt)
{
	int volt_avg = 0;
	unsigned int i = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_volt_to_temp");

	for (i = 0; i < ARRAY_SIZE(con_temp_ntc_100k_1840mv) - 1; i++) {
		if ((volt >= con_volt_ntc_100k_1840mv[i + 1])
				&& (volt <= con_volt_ntc_100k_1840mv[i])) {
			volt_avg = (con_volt_ntc_100k_1840mv[i + 1] + con_volt_ntc_100k_1840mv[i]) / 2;
			if(volt <= volt_avg) {
				i++;
			}
			break;
		}
	}

	TEMP_COMPENSATION_DEBUG("i=%u,volt=%d,temp=%d\n", i, con_volt_ntc_100k_1840mv[i], con_temp_ntc_100k_1840mv[i]);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_volt_to_temp");

	TEMP_COMPENSATION_DEBUG("end\n");

	return con_temp_ntc_100k_1840mv[i];
}

int oplus_temp_compensation_get_ntc_temp(void)
{
	int rc = 0;
	int val_avg = 0;
	int val[3] = {0, 0, 0};
	unsigned int i = 0;
	unsigned int count = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid p_oplus_temp_compensation_params params\n");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params->ntc_temp_chan)) {
		TEMP_COMPENSATION_ERR("Invalid ntc_temp_chan params\n");
		p_oplus_temp_compensation_params->ntc_temp = 29;
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_ntc_temp:%d\n", p_oplus_temp_compensation_params->ntc_temp);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_ntc_temp", p_oplus_temp_compensation_params->ntc_temp);
		return -EINVAL;
	}

	if (p_oplus_temp_compensation_params->fake_ntc_temp) {
		TEMP_COMPENSATION_DEBUG("fake panel ntc temp is %d\n", p_oplus_temp_compensation_params->ntc_temp);
		return p_oplus_temp_compensation_params->ntc_temp;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_ntc_temp");

	do {
		for (i = 0; i < 3; i++) {
			rc = iio_read_channel_processed(p_oplus_temp_compensation_params->ntc_temp_chan, &val[i]);
			TEMP_COMPENSATION_DEBUG("ntc_volt:%d\n", val[i]);
			if (rc < 0) {
				TEMP_COMPENSATION_ERR("read ntc_temp_chan volt failed, rc=%d\n", rc);
			} else {
				val[i] = oplus_temp_compensation_volt_to_temp(val[i]);
				TEMP_COMPENSATION_DEBUG("ntc_temp:%d\n", val[i]);
			}
		}

		if (count) {
			TEMP_COMPENSATION_ERR("retry %u\n", count);
		}
		count++;
	} while (((abs(val[0] - val[1]) >= 2) || (abs(val[0] - val[2]) >= 2)
				|| (abs(val[1] - val[2]) >= 2) || (rc < 0)) && (count < 5));

	if (count == 5) {
		TEMP_COMPENSATION_ERR("use last panel ntc temp %d\n", p_oplus_temp_compensation_params->ntc_temp);
		return p_oplus_temp_compensation_params->ntc_temp;
	} else {
		val_avg = (val[0] + val[1] + val[2]) / 3;
	}

	p_oplus_temp_compensation_params->ntc_temp = val_avg;
	TEMP_COMPENSATION_DEBUG("oplus_temp_compensation_ntc_temp:%d\n", p_oplus_temp_compensation_params->ntc_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_ntc_temp", p_oplus_temp_compensation_params->ntc_temp);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_ntc_temp");

	TEMP_COMPENSATION_DEBUG("end\n");

	return p_oplus_temp_compensation_params->ntc_temp;
}

static int oplus_temp_compensation_get_shell_temp(void)
{
	int temp = -127000;
	int max_temp = -127000;
	unsigned int i = 0;
	const char *shell_tz[] = {"shell_front", "shell_frame", "shell_back"};
	struct thermal_zone_device *tz = NULL;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_temp_compensation_params->fake_shell_temp) {
		TEMP_COMPENSATION_DEBUG("fake shell temp is %d\n", p_oplus_temp_compensation_params->shell_temp);
		return p_oplus_temp_compensation_params->shell_temp;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_shell_temp");

	for (i = 0; i < ARRAY_SIZE(shell_tz); i++) {
		tz = thermal_zone_get_zone_by_name(shell_tz[i]);
		thermal_zone_get_temp(tz, &temp);
		if (max_temp < temp) {
			max_temp = temp;
		}
	}

	p_oplus_temp_compensation_params->shell_temp = max_temp / 1000;
	TEMP_COMPENSATION_DEBUG("oplus_temp_compensation_shell_temp:%d\n", p_oplus_temp_compensation_params->shell_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_shell_temp", p_oplus_temp_compensation_params->shell_temp);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_shell_temp");

	TEMP_COMPENSATION_DEBUG("end\n");

	return p_oplus_temp_compensation_params->shell_temp;
}

int oplus_temp_compensation_data_update(void)
{
	static bool calibrated = false;
	int delta1 = 0;
	int delta2 = 0;
	unsigned char page_3[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x03};
	unsigned char offset_33[2] = {0x6F, 0x21};
	unsigned char rx_buf[7] = {0};
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int k = 0;
	static unsigned int failure_count = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!oplus_temp_compensation_data_update_is_enabled()) {
		TEMP_COMPENSATION_DEBUG("data updating is not enabled\n");
		return 0;
	}

	if (calibrated || failure_count > 10) {
		TEMP_COMPENSATION_DEBUG("calibrated:%u,failure_count:%u, no need to update temp compensation data\n", calibrated, failure_count);
		return 0;
	}

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_data_update");

	ddic_dsi_send_cmd(6, page_3);
	ddic_dsi_send_cmd(2, offset_33);
	mtk_read_ddic_v2(0xE0, 7, rx_buf);

	TEMP_COMPENSATION_INFO("rx_buf[0]=%u,rx_buf[6]=%u\n", rx_buf[0], rx_buf[6]);
	if ((rx_buf[0] < 25) || (rx_buf[0] > 29) || (rx_buf[6] < 26) || (rx_buf[6] > 30)) {
		failure_count++;
		TEMP_COMPENSATION_ERR("invalid rx_buf,failure_count:%u\n", failure_count);
		OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_data_update");
		return -EINVAL;
	}

	delta1 = rx_buf[0] - 0x1B;
	delta2 = rx_buf[6] - 0x1C;
	TEMP_COMPENSATION_DEBUG("delta1=%d,delta2=%d\n", delta1, delta2);

	for (i = OPLUS_TEMP_COMPENSATION_1511_1604_DBV_INDEX; i <= OPLUS_TEMP_COMPENSATION_1212_1328_DBV_INDEX; i++) {
		for (j = OPLUS_TEMP_COMPENSATION_LESS_THAN_MINUS10_TEMP_INDEX; j <= OPLUS_TEMP_COMPENSATION_GREATER_THAN_50_TEMP_INDEX; j++) {
			for (k = 11; k <= 13; k++) {
				p_oplus_temp_compensation_params->data[i][j][k] += delta2;
				TEMP_COMPENSATION_DEBUG("data[%u][%u][%u]=0x%02X\n", i, j, k, p_oplus_temp_compensation_params->data[i][j][k]);
			}
			for (k = 14; k <= 16; k++) {
				p_oplus_temp_compensation_params->data[i][j][k] += delta1;
				TEMP_COMPENSATION_DEBUG("data[%u][%u][%u]=0x%02X\n", i, j, k, p_oplus_temp_compensation_params->data[i][j][k]);
			}
		}
	}

	i = OPLUS_TEMP_COMPENSATION_1604_3515_DBV_INDEX;
	for (j = OPLUS_TEMP_COMPENSATION_LESS_THAN_MINUS10_TEMP_INDEX; j <= OPLUS_TEMP_COMPENSATION_GREATER_THAN_50_TEMP_INDEX; j++) {
		for (k = 8; k <= 10; k++) {
			p_oplus_temp_compensation_params->data[i][j][k] += delta1;
			TEMP_COMPENSATION_DEBUG("data[%u][%u][%u]=0x%02X\n", i, j, k, p_oplus_temp_compensation_params->data[i][j][k]);
		}
	}

	calibrated = true;
	TEMP_COMPENSATION_INFO("update temp compensation data successfully\n");

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_data_update");

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;
}

void oplus_temp_compensation_vpark_set(unsigned char voltage, struct LCM_setting_table *temp_compensation_cmd)
{
	unsigned char voltage1, voltage2, voltage3, voltage4;
	unsigned short vpark = (69 - voltage) * 1024 / (69 - 10);

	TEMP_COMPENSATION_DEBUG("start\n");

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_vpark_set");

	voltage1 = ((vpark & 0xFF00) >> 8) + ((vpark & 0xFF00) >> 6) + ((vpark & 0xFF00) >> 4);
	voltage2 = vpark & 0xFF;
	voltage3 = vpark & 0xFF;
	voltage4 = vpark & 0xFF;
	temp_compensation_cmd[17].para_list[0+1] = voltage1;
	temp_compensation_cmd[17].para_list[1+1] = voltage2;
	temp_compensation_cmd[17].para_list[2+1] = voltage3;
	temp_compensation_cmd[17].para_list[3+1] = voltage4;

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_vpark_set");

	TEMP_COMPENSATION_DEBUG("end\n");
}

static unsigned int oplus_temp_compensation_get_dbv_index(unsigned int bl_lvl)
{
	unsigned int i = 0;
	unsigned int dbv_index = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		/* set default dbv index to 2 */
		return 2;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_dbv_index");

	for (i = 0; i < (p_oplus_temp_compensation_params->dbv_group_count - 1); i++) {
		if (bl_lvl >= p_oplus_temp_compensation_params->dbv_group[i]) {
			break;
		}
	}
	dbv_index = i;
	TEMP_COMPENSATION_DEBUG("dbv_index:%u\n", dbv_index);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_dbv_index");

	TEMP_COMPENSATION_DEBUG("end\n");

	return dbv_index;
}

static unsigned int oplus_temp_compensation_get_temp_index(int temp)
{
	unsigned int i = 0;
	unsigned int temp_index = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		/* set default temp index to 6 */
		return 6;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_temp_index");

	for (i = 0; i < (p_oplus_temp_compensation_params->temp_group_count - 1); i++) {
		if (temp < p_oplus_temp_compensation_params->temp_group[i]) {
			break;
		}
	}
	temp_index = i;
	TEMP_COMPENSATION_DEBUG("temp_index:%u\n", temp_index);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_temp_index");

	TEMP_COMPENSATION_DEBUG("end\n");

	return temp_index;
}

static int oplus_temp_compensation_send_pack_hs_cmd(void *dsi, void *LCM_setting_table, unsigned int lcm_cmd_count, void *p_dcs_write_gce_pack, void *handle)
{
	unsigned int i = 0;
	struct LCM_setting_table *table = LCM_setting_table;
	dcs_write_gce_pack cb = p_dcs_write_gce_pack;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(dsi) || IS_ERR_OR_NULL(table) || !lcm_cmd_count
			|| IS_ERR_OR_NULL(cb) || IS_ERR_OR_NULL(handle)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		TEMP_COMPENSATION_ERR("out of mtk_ddic_dsi_cmd\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_send_pack_hs_cmd");

	for (i = 0; i < lcm_cmd_count; i++) {
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].cmd_num = table[i].count;
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].para_list = table[i].para_list;
	}
	send_cmd_to_ddic.is_hs = 1;
	send_cmd_to_ddic.is_package = 1;
	send_cmd_to_ddic.cmd_count = lcm_cmd_count;

	cb(dsi, handle, &send_cmd_to_ddic);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_send_pack_hs_cmd");

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;
}

int oplus_temp_compensation_cmd_set(void *mtk_dsi, void *gce_cb, void *handle, unsigned int setting_mode)
{
	int rc = 0;
	int ntc_temp = 0;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int lcm_cmd_count = 0;
	unsigned int refresh_rate = 120;
	unsigned int dbv_index = 0;
	unsigned int temp_index = 0;
	unsigned int bl_lvl = 0;
	static unsigned int last_dbv_index = 0;
	static unsigned int last_temp_index = 0;
	static unsigned int last_bl_lvl = 0;		/* Force sending temp compensation cmd when booting up */
	struct mtk_dsi *dsi = mtk_dsi;
	dcs_write_gce cb = gce_cb;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();
	struct LCM_setting_table temp_compensation_cmd[] = {
		{REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01}},
		{REGFLAG_CMD, 2, {0x6F, 0x06}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x0A}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x0E}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x28}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x2C}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x4A}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x4E}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x52}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x03}},
		{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x0C}},
		{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x15}},
		{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05}},
		{REGFLAG_CMD, 2, {0x6F, 0x12}},
		{REGFLAG_CMD, 5, {0xEC, 0x00, 0x00, 0x00, 0x00}},
	};

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(dsi) || IS_ERR_OR_NULL(gce_cb) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_cmd_set");

	bl_lvl = oplus_display_brightness;

	TEMP_COMPENSATION_DEBUG("setting_mode:%u\n", setting_mode);

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	if (oplus_ofp_is_support()) {
		if ((setting_mode == OPLUS_TEMP_COMPENSATION_FOD_ON_SETTING)
				|| (oplus_ofp_get_hbm_state() && (setting_mode != OPLUS_TEMP_COMPENSATION_FOD_OFF_SETTING))) {
			bl_lvl = 3840;
		}
	}
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	if (IS_ERR_OR_NULL(dsi->ext) || IS_ERR_OR_NULL(dsi->ext->params)) {
		/* default refresh_rate is 120 */
		refresh_rate = 120;
	} else {
		refresh_rate = dsi->ext->params->dyn_fps.vact_timing_fps;
	}
	TEMP_COMPENSATION_DEBUG("refresh_rate:%u\n", refresh_rate);

	dbv_index = oplus_temp_compensation_get_dbv_index(bl_lvl);

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params->ntc_temp_chan)) {
		TEMP_COMPENSATION_DEBUG("panel ntc is not exist, use default ntc temp value\n");
		ntc_temp = 29;
	} else {
		if (!last_bl_lvl && bl_lvl) {
			/* update ntc temp immediately when power on */
			ntc_temp = oplus_temp_compensation_get_ntc_temp();
		} else {
			ntc_temp = p_oplus_temp_compensation_params->ntc_temp;
		}
	}

	temp_index = oplus_temp_compensation_get_temp_index(ntc_temp);

	TEMP_COMPENSATION_DEBUG("last_bl_lvl:%u,bl_lvl:%u,last_dbv_index:%u,dbv_index:%u,shell_temp:%d,ntc_temp:%d,last_temp_index:%u,temp_index:%u\n",
			last_bl_lvl, bl_lvl, last_dbv_index, dbv_index, oplus_temp_compensation_get_shell_temp(), ntc_temp,
				last_temp_index, temp_index);

	if ((last_dbv_index != dbv_index) || (last_temp_index != temp_index) || (!last_bl_lvl && bl_lvl)
			|| (setting_mode == OPLUS_TEMP_COMPENSATION_ESD_SETTING) || (setting_mode == OPLUS_TEMP_COMPENSATION_FIRST_HALF_FRAME_SETTING)) {
		if ((refresh_rate == 60) && (setting_mode == OPLUS_TEMP_COMPENSATION_BACKLIGHT_SETTING)
			&& (bl_lvl != 0) && (bl_lvl != 1)) {
			p_oplus_temp_compensation_params->need_to_set_in_first_half_frame = true;
			TEMP_COMPENSATION_DEBUG("oplus_temp_compensation_need_to_set_in_first_half_frame:%d\n", p_oplus_temp_compensation_params->need_to_set_in_first_half_frame);
			OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_need_to_set_in_first_half_frame",
												p_oplus_temp_compensation_params->need_to_set_in_first_half_frame);
		} else {
			for (i = 0; i < p_oplus_temp_compensation_params->reg_repeat; i++) {
				temp_compensation_cmd[2].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][0];
				temp_compensation_cmd[4].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][1];
				temp_compensation_cmd[6].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][2];
				temp_compensation_cmd[12].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][5];
				temp_compensation_cmd[14].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][6];
				temp_compensation_cmd[16].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][7];
			}

			for (i = 0; i < 4; i++) {
				if (hpwm_mode) {
					temp_compensation_cmd[25].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][21+i];
				} else {
					temp_compensation_cmd[25].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][17+i];
				}
			}

			for (i = 0; i < 3; i++) {
				temp_compensation_cmd[8].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][3];
				temp_compensation_cmd[10].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][4];
				temp_compensation_cmd[18].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][8+i];
				temp_compensation_cmd[20].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][11+i];
				temp_compensation_cmd[22].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][14+i];
			}

			TEMP_COMPENSATION_INFO("refresh_rate:%u,setting_mode:%u\n", refresh_rate, setting_mode);
			TEMP_COMPENSATION_INFO("last_bl_lvl:%u,bl_lvl:%u,last_dbv_index:%u,dbv_index:%u,ntc_temp:%d,last_temp_index:%u,temp_index:%u,set temp compensation cmd\n",
				last_bl_lvl, bl_lvl, last_dbv_index, dbv_index, ntc_temp, last_temp_index, temp_index);

			lcm_cmd_count = sizeof(temp_compensation_cmd) / sizeof(struct LCM_setting_table);
			if (!oplus_temp_compensation_dry_run_is_enabled()) {
				if (!handle) {
					for (i = 0; i < lcm_cmd_count; i++) {
						cb(dsi, handle, temp_compensation_cmd[i].para_list, temp_compensation_cmd[i].count);
					}
				} else {
					rc = oplus_temp_compensation_send_pack_hs_cmd(dsi, temp_compensation_cmd, lcm_cmd_count, gce_cb, handle);
					if (rc) {
						TEMP_COMPENSATION_ERR("failed to send pack hs cmd\n");
					}
				}
			} else {
				TEMP_COMPENSATION_INFO("dry running\n");
			}

			for (i = 0; i < lcm_cmd_count; i++) {
				for (j = 0; j < temp_compensation_cmd[i].count; j++) {
					TEMP_COMPENSATION_DEBUG("temp_compensation_cmd[%u][%u]=0x%02X\n", i, j, temp_compensation_cmd[i].para_list[j]);
				}
			}
		}
	}

	last_dbv_index = dbv_index;
	last_temp_index = temp_index;
	last_bl_lvl = bl_lvl;

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_cmd_set");

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}
EXPORT_SYMBOL(oplus_temp_compensation_cmd_set);

static void oplus_temp_compensation_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(cb_data)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_cmdq_cb");

	TEMP_COMPENSATION_INFO("set temp compensation cmd in the first half frame cb done\n");
	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_cmdq_cb");

	TEMP_COMPENSATION_DEBUG("end\n");

	return;
}

int oplus_temp_compensation_first_half_frame_cmd_set(void *drm_crtc)
{
	bool is_frame_mode = true;
	struct drm_crtc *crtc = drm_crtc;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct mtk_ddp_comp *comp = NULL;
	struct mtk_cmdq_cb_data *cb_data = NULL;
	struct cmdq_client *client = NULL;
	struct cmdq_pkt *cmdq_handle = NULL;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid p_oplus_temp_compensation_params params\n");
		return -EINVAL;
	}

	/*
	 temp compensation cmd should be sent in the first half frame in 60hz timing.
	 otherwise,there would be a low probability backlight flash issue.
	*/
	if (p_oplus_temp_compensation_params->need_to_set_in_first_half_frame) {
		TEMP_COMPENSATION_DEBUG("need to set temp compensation cmd in the first half frame\n");

		if (IS_ERR_OR_NULL(crtc)) {
			TEMP_COMPENSATION_ERR("Invalid input params\n");
			return -EINVAL;
		}

		mtk_crtc = to_mtk_crtc(crtc);
		if (IS_ERR_OR_NULL(mtk_crtc)) {
			TEMP_COMPENSATION_ERR("Invalid mtk_crtc params\n");
			return -EINVAL;
		}

		comp = mtk_ddp_comp_request_output(mtk_crtc);
		if (IS_ERR_OR_NULL(comp)) {
			TEMP_COMPENSATION_ERR("Invalid comp params\n");
			return -EINVAL;
		}

		if (!(mtk_crtc->enabled)) {
			TEMP_COMPENSATION_ERR("mtk_crtc is not enabled\n");
			return -EINVAL;
		}

		cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
		if (IS_ERR_OR_NULL(cb_data)) {
			TEMP_COMPENSATION_ERR("failed to kmalloc cb_data\n");
			return -EINVAL;
		}

		OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_first_half_frame_cmd_set");

		mtk_drm_idlemgr_kick(__func__, crtc, 0);

		is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);

		client = (is_frame_mode) ? mtk_crtc->gce_obj.client[CLIENT_CFG] :
					mtk_crtc->gce_obj.client[CLIENT_DSI_CFG];

#ifndef OPLUS_FEATURE_DISPLAY
		cmdq_handle = cmdq_pkt_create(client);
#else
		mtk_crtc_pkt_create(&cmdq_handle, crtc, client);
#endif /* OPLUS_FEATURE_DISPLAY */
		if (IS_ERR_OR_NULL(cmdq_handle)) {
			TEMP_COMPENSATION_ERR("Invalid cmdq_handle params\n");
			kfree(cb_data);
			OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_first_half_frame_cmd_set");
			return -EINVAL;
		}

		/* wait one TE */
		if (mtk_crtc_with_event_loop(crtc)) {
			cmdq_pkt_clear_event(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_TE]);
			if (mtk_drm_lcm_is_connect(mtk_crtc)) {
				cmdq_pkt_wait_no_clear(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_TE]);
			}
		} else {
			cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			if (mtk_drm_lcm_is_connect(mtk_crtc)) {
				cmdq_pkt_wait_no_clear(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			}
		}

		if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode)) {
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_SECOND_PATH, 0);
		} else {
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_FIRST_PATH, 0);
		}

		if (is_frame_mode) {
			cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
			cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		}

		cmdq_handle->err_cb.cb = mtk_crtc_cmdq_timeout_cb;
		cmdq_handle->err_cb.data = crtc;

		TEMP_COMPENSATION_DEBUG("OPLUS_TEMP_COMPENSATION_FIRST_HALF_FRAME_SETTING\n");
		oplus_temp_compensation_io_cmd_set(comp, cmdq_handle, OPLUS_TEMP_COMPENSATION_FIRST_HALF_FRAME_SETTING);

		if (is_frame_mode) {
			cmdq_pkt_set_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
			cmdq_pkt_set_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		}

		cb_data->crtc = crtc;
		cb_data->cmdq_handle = cmdq_handle;

		if (cmdq_pkt_flush_threaded(cmdq_handle, oplus_temp_compensation_cmdq_cb, cb_data) < 0) {
			TEMP_COMPENSATION_ERR("failed to flush oplus_temp_compensation_cmdq_cb\n");
			OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_first_half_frame_cmd_set");
			return -EINVAL;
		}

		p_oplus_temp_compensation_params->need_to_set_in_first_half_frame = false;
		TEMP_COMPENSATION_DEBUG("oplus_temp_compensation_need_to_set_in_first_half_frame:%d\n", p_oplus_temp_compensation_params->need_to_set_in_first_half_frame);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_need_to_set_in_first_half_frame",
											p_oplus_temp_compensation_params->need_to_set_in_first_half_frame);
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_first_half_frame_cmd_set");

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;
}

int oplus_temp_compensation_io_cmd_set(void *mtk_ddp_comp, void *cmdq_pkt, unsigned int setting_mode)
{
	struct mtk_ddp_comp *comp = mtk_ddp_comp;
	struct cmdq_pkt *cmdq_handle = cmdq_pkt;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(comp)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_io_cmd_set");

	TEMP_COMPENSATION_DEBUG("OPLUS_TEMP_COMPENSATION_SET\n");
	mtk_ddp_comp_io_cmd(comp, cmdq_handle, OPLUS_TEMP_COMPENSATION_SET, &setting_mode);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_io_cmd_set");

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;
}

int oplus_temp_compensation_temp_check(void *mtk_ddp_comp, void *cmdq_pkt)
{
	int rc = 0;
	int ntc_temp = 0;
	int shell_temp = 0;
	static int last_ntc_temp = 0;
	struct mtk_ddp_comp *comp = mtk_ddp_comp;
	struct cmdq_pkt *cmdq_handle = cmdq_pkt;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(comp) || IS_ERR_OR_NULL(cmdq_handle) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_temp_check");

	ntc_temp = p_oplus_temp_compensation_params->ntc_temp;
	shell_temp = oplus_temp_compensation_get_shell_temp();

	if (oplus_temp_compensation_get_temp_index(last_ntc_temp) != oplus_temp_compensation_get_temp_index(ntc_temp)) {
		oplus_temp_compensation_io_cmd_set(comp, cmdq_handle, OPLUS_TEMP_COMPENSATION_TEMPERATURE_SETTING);
		TEMP_COMPENSATION_INFO("last_ntc_temp:%d,current_ntc_temp:%d,bl_lvl:%u,update temp compensation cmd\n", last_ntc_temp, ntc_temp, oplus_display_brightness);
	}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	if (oplus_adfr_is_support()) {
		rc = oplus_adfr_temperature_detection_handle(comp, cmdq_handle, ntc_temp, shell_temp);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to handle temperature detection\n");
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

	last_ntc_temp = p_oplus_temp_compensation_params->ntc_temp;

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_temp_check");

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}

/* -------------------- node -------------------- */
/* config */
ssize_t oplus_temp_compensation_set_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int config = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(buf) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return count;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_set_config_attr");

	sscanf(buf, "%u", &config);

	p_oplus_temp_compensation_params->config = config;
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_config:0x%x\n", p_oplus_temp_compensation_params->config);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_config", p_oplus_temp_compensation_params->config);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_set_config_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_temp_compensation_get_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(buf) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_config_attr");

	TEMP_COMPENSATION_INFO("config:0x%x\n", p_oplus_temp_compensation_params->config);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_config_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_temp_compensation_params->config);
}

/* ntc temp */
ssize_t oplus_temp_compensation_set_ntc_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ntc_temp = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!oplus_temp_compensation_is_supported()) {
		TEMP_COMPENSATION_ERR("temp compensation is not supported\n");
		return count;
	}

	if (IS_ERR_OR_NULL(buf) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return count;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_set_ntc_temp_attr");

	sscanf(buf, "%d", &ntc_temp);
	TEMP_COMPENSATION_INFO("set ntc temp to %d\n", ntc_temp);

	if (ntc_temp == -1) {
		p_oplus_temp_compensation_params->fake_ntc_temp = false;
	} else {
		p_oplus_temp_compensation_params->ntc_temp = ntc_temp;
		p_oplus_temp_compensation_params->fake_ntc_temp = true;
	}

	TEMP_COMPENSATION_INFO("oplus_temp_compensation_ntc_temp:%d\n", p_oplus_temp_compensation_params->ntc_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_ntc_temp", p_oplus_temp_compensation_params->ntc_temp);
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_fake_ntc_temp:%d\n", p_oplus_temp_compensation_params->fake_ntc_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_fake_ntc_temp", p_oplus_temp_compensation_params->fake_ntc_temp);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_set_ntc_temp_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return count;
}

ssize_t oplus_temp_compensation_get_ntc_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int ntc_temp = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(buf)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_ntc_temp_attr");

	if (!oplus_temp_compensation_is_supported()) {
		ntc_temp = -EINVAL;
		TEMP_COMPENSATION_ERR("temp compensation is not supported\n");
	} else {
		ntc_temp = oplus_temp_compensation_get_ntc_temp();
		TEMP_COMPENSATION_INFO("ntc_temp:%d\n", ntc_temp);
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_ntc_temp_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return sprintf(buf, "%d\n", ntc_temp);
}

/* shell temp */
ssize_t oplus_temp_compensation_set_shell_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int shell_temp = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!oplus_temp_compensation_is_supported()) {
		TEMP_COMPENSATION_ERR("temp compensation is not supported\n");
		return count;
	}

	if (IS_ERR_OR_NULL(buf) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return count;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_set_shell_temp_attr");

	sscanf(buf, "%d", &shell_temp);
	TEMP_COMPENSATION_INFO("set shell temp to %d\n", shell_temp);

	if (shell_temp == -1) {
		p_oplus_temp_compensation_params->fake_shell_temp = false;
	} else {
		p_oplus_temp_compensation_params->shell_temp = shell_temp;
		p_oplus_temp_compensation_params->fake_shell_temp = true;
	}

	TEMP_COMPENSATION_INFO("oplus_temp_compensation_shell_temp:%d\n", p_oplus_temp_compensation_params->shell_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_shell_temp", p_oplus_temp_compensation_params->shell_temp);
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_fake_shell_temp:%d\n", p_oplus_temp_compensation_params->fake_shell_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_fake_shell_temp", p_oplus_temp_compensation_params->fake_shell_temp);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_set_shell_temp_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return count;
}

ssize_t oplus_temp_compensation_get_shell_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int shell_temp = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(buf)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_shell_temp_attr");

	if (!oplus_temp_compensation_is_supported()) {
		shell_temp = -EINVAL;
		TEMP_COMPENSATION_ERR("temp compensation is not supported\n");
	} else {
		shell_temp = oplus_temp_compensation_get_shell_temp();
		TEMP_COMPENSATION_INFO("shell_temp:%d\n", shell_temp);
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_shell_temp_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return sprintf(buf, "%d\n", shell_temp);
}
