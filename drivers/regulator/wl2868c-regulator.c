// SPDX-License-Identifier: GPL-2.0+
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

struct wl2868c_platform_data {
	struct device *dev;
	struct regmap *regmap;
};

enum wl2868c_regulator_ids {
	WL2868C_LDO1,
	WL2868C_LDO2,
	WL2868C_LDO3,
	WL2868C_LDO4,
	WL2868C_LDO5,
	WL2868C_LDO6,
	WL2868C_LDO7,
};

enum wl2868c_registers {
	WL2868C_PRODUCT_ID = 0x00,
	WL2868C_SILICON_REV,
	WL2868C_IOUT,
	WL2868C_LDO1VOUT = 0x03,
	WL2868C_LDO2VOUT,
	WL2868C_LDO3VOUT,
	WL2868C_LDO4VOUT,
	WL2868C_LDO5VOUT,
	WL2868C_LDO6VOUT,
	WL2868C_LDO7VOUT,
	WL2868C_ENABLE = 0x0e,
	WL2868C_REG_MAX = 0x25,
};

#define WL2868C_ID	0x82

static const struct regulator_ops wl2868c_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

#define WL2868C_NLDO(_num, _supply, _default)				\
	[WL2868C_LDO ## _num] = {					\
		.name =		   "ONLDO"#_num,			\
		.of_match =	   of_match_ptr("ONLDO"#_num),		\
		.regulators_node = of_match_ptr("regulators"),		\
		.type =		   REGULATOR_VOLTAGE,			\
		.owner =	   THIS_MODULE,				\
		.linear_ranges =   (struct linear_range[]) {		\
		      REGULATOR_LINEAR_RANGE(496000, 0x0, 0xff, 8000),	\
		},							\
		.n_linear_ranges = 1,					\
		.vsel_reg =	   WL2868C_LDO ## _num ## VOUT,	\
		.vsel_mask =	   0xff,				\
		.enable_reg =	   WL2868C_ENABLE,			\
		.enable_mask =	   BIT(_num - 1),			\
		.enable_time =	   150,					\
		.supply_name =	   _supply,				\
		.ops =		   &wl2868c_ops,			\
	}

#define WL2868C_PLDO(_num, _supply, _default)				\
	[WL2868C_LDO ## _num] = {					\
		.name =		   "ONLDO"#_num,				\
		.of_match =	   of_match_ptr("ONLDO"#_num),		\
		.regulators_node = of_match_ptr("regulators"),		\
		.type =		   REGULATOR_VOLTAGE,			\
		.owner =	   THIS_MODULE,				\
		.linear_ranges =   (struct linear_range[]) {		\
		      REGULATOR_LINEAR_RANGE(1504000, 0x0, 0xff, 8000),	\
		},							\
		.n_linear_ranges = 1,					\
		.vsel_reg =	   WL2868C_LDO ## _num ## VOUT,	\
		.vsel_mask =	   0xff,				\
		.enable_reg =	   WL2868C_ENABLE,			\
		.enable_mask =	   BIT(_num - 1),			\
		.enable_time =	   150,					\
		.supply_name =	   _supply,				\
		.ops =		   &wl2868c_ops,			\
	}

static const struct regulator_desc wl2868c_regulators[] = {
	WL2868C_NLDO(1, "vin12", 1200000),
	WL2868C_NLDO(2, "vin12",  808000),
	WL2868C_PLDO(3, "vin34", 2800000),
	WL2868C_PLDO(4, "vin34", 2904000),
	WL2868C_PLDO(5, "vin5", 2704000),
	WL2868C_PLDO(6, "vin6", 2800000),
	WL2868C_PLDO(7, "vin7", 2800000),
};

static const struct regmap_config wl2868c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL2868C_REG_MAX,
};

static int wl2868c_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct wl2868c_platform_data *pdata;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	//struct regmap *regmap;
	int i, ret;
	unsigned int data;
	struct pinctrl *pinctrl;
	struct pinctrl_state *reset_output_high;
	int access_time = 3;

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev,
				sizeof(struct wl2868c_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
	} else {
		pdata = i2c->dev.platform_data;
	}

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	if (pdata == NULL) {
		dev_err(&i2c->dev, "fail : no platform data.\n");
		return -ENODATA;
	}

	pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_err(&i2c->dev, "Cannot find pinctrl!\n");
		//return ret;
	} else {
		reset_output_high = pinctrl_lookup_state(pinctrl, "reset_high");
		if (IS_ERR(reset_output_high)) {
			ret = PTR_ERR(reset_output_high);
			dev_err(&i2c->dev, "Cannot find pinctrl reset_output_high!\n");
		} else {
			pinctrl_select_state(pinctrl, reset_output_high);
		}
	}
	pdata->regmap = devm_regmap_init_i2c(i2c, &wl2868c_regmap);
	if (IS_ERR(pdata->regmap)) {
		ret = PTR_ERR(pdata->regmap);
		dev_err(&i2c->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	ret = regmap_read(pdata->regmap, WL2868C_PRODUCT_ID, &data);

	while(ret<0 && --access_time) {
		mdelay(2);
		ret = regmap_read(pdata->regmap, WL2868C_PRODUCT_ID, &data);
	}
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to read PRODUCT_ID: %d\n", ret);
		return ret;
	}
	if (data != WL2868C_ID) {
		dev_err(&i2c->dev, "Unsupported device id: 0x%x.\n", data);
		return -ENODEV;
	}
		ret = regmap_read(pdata->regmap, 0x0C, &data);
		dev_err(&i2c->dev, "read reg[0x0c] =0x%02x.\n", data);

	config.dev = &i2c->dev;
	config.regmap = pdata->regmap;
	config.init_data = NULL;

	for (i = 0; i < ARRAY_SIZE(wl2868c_regulators); i++) {
		rdev = devm_regulator_register(&i2c->dev,
					       &wl2868c_regulators[i],
					       &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(&i2c->dev, "Failed to register %s: %d\n",
				wl2868c_regulators[i].name, ret);
			return ret;
		}

		if (i < 2) {
			config.ena_gpiod = gpiod_get_optional(&i2c->dev, "vin12_ena",
				 GPIOD_OUT_LOW| GPIOD_FLAGS_BIT_NONEXCLUSIVE);
			if (IS_ERR(config.ena_gpiod)) {
				dev_err(&i2c->dev, "no vin12_ena config\n");
			}
		} else {
			config.ena_gpiod = NULL;
		}
		dev_err(&i2c->dev, "regulator register %s: %d\n",
			wl2868c_regulators[i].name, ret);
	}
	dev_err(&i2c->dev, "regulator probe end\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id wl2868c_dt_ids[] = {
	{ .compatible = "wl2868c", },
	{}
};
MODULE_DEVICE_TABLE(of, wl2868c_dt_ids);
#endif

static const struct i2c_device_id wl2868c_i2c_id[] = {
	{ "wl2868c-pmic", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, wl2868c_i2c_id);

static struct i2c_driver wl2868c_regulator_driver = {
	.driver = {
		.name = "wl2868c-pmic",
		.owner = THIS_MODULE
		//.of_match_table	= of_match_ptr(wl2868c_dt_ids),
	},
	.probe = wl2868c_i2c_probe,
	.id_table = wl2868c_i2c_id,
};

module_i2c_driver(wl2868c_regulator_driver);

MODULE_DESCRIPTION("WL2868C PMIC voltage regulator driver");
MODULE_AUTHOR("XXX");
MODULE_LICENSE("GPL");
