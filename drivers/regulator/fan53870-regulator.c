// SPDX-License-Identifier: GPL-2.0+
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

struct fan53870_platform_data {
	struct device *dev;
	struct regmap *regmap;
};

enum fan53870_regulator_ids {
	FAN53870_LDO1,
	FAN53870_LDO2,
	FAN53870_LDO3,
	FAN53870_LDO4,
	FAN53870_LDO5,
	FAN53870_LDO6,
	FAN53870_LDO7,
};

enum fan53870_registers {
	FAN53870_PRODUCT_ID = 0x00,
	FAN53870_SILICON_REV,
	FAN53870_IOUT,
	FAN53870_ENABLE,
	FAN53870_LDO1VOUT,
	FAN53870_LDO2VOUT,
	FAN53870_LDO3VOUT,
	FAN53870_LDO4VOUT,
	FAN53870_LDO5VOUT,
	FAN53870_LDO6VOUT,
	FAN53870_LDO7VOUT,
	FAN53870_REG_MAX = 0x0d,
};

#define FAN53870_ID	0x01

static const struct regulator_ops fan53870_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

#define FAN53870_NLDO(_num, _supply, _default)				\
	[FAN53870_LDO ## _num] = {					\
		.name =		   "ONLDO"#_num,			\
		.of_match =	   of_match_ptr("ONLDO"#_num),		\
		.regulators_node = of_match_ptr("regulators"),		\
		.type =		   REGULATOR_VOLTAGE,			\
		.owner =	   THIS_MODULE,				\
		.linear_ranges =   (struct linear_range[]) {		\
		      REGULATOR_LINEAR_RANGE(_default, 0, 0x62, 0),	\
		      REGULATOR_LINEAR_RANGE(800000, 0x63, 0xba, 8000),	\
		},							\
		.n_linear_ranges = 2,					\
		.vsel_reg =	   FAN53870_LDO ## _num ## VOUT,	\
		.vsel_mask =	   0xff,				\
		.enable_reg =	   FAN53870_ENABLE,			\
		.enable_mask =	   BIT(_num - 1),			\
		.enable_time =	   150,					\
		.supply_name =	   _supply,				\
		.ops =		   &fan53870_ops,			\
	}

#define FAN53870_PLDO(_num, _supply, _default)				\
	[FAN53870_LDO ## _num] = {					\
		.name =		   "ONLDO"#_num,				\
		.of_match =	   of_match_ptr("ONLDO"#_num),		\
		.regulators_node = of_match_ptr("regulators"),		\
		.type =		   REGULATOR_VOLTAGE,			\
		.owner =	   THIS_MODULE,				\
		.linear_ranges =   (struct linear_range[]) {		\
		      REGULATOR_LINEAR_RANGE(_default, 0, 0x0f, 0),	\
		      REGULATOR_LINEAR_RANGE(1500000, 0x10, 0xff, 8000),	\
		},							\
		.n_linear_ranges = 2,					\
		.vsel_reg =	   FAN53870_LDO ## _num ## VOUT,	\
		.vsel_mask =	   0xff,				\
		.enable_reg =	   FAN53870_ENABLE,			\
		.enable_mask =	   BIT(_num - 1),			\
		.enable_time =	   150,					\
		.supply_name =	   _supply,				\
		.ops =		   &fan53870_ops,			\
	}

static const struct regulator_desc fan53870_regulators[] = {
	FAN53870_NLDO(1, "vin12", 809000),
	FAN53870_NLDO(2, "vin12", 1049000),
	FAN53870_PLDO(3, "vin34", 2799000),
	FAN53870_PLDO(4, "vin34", 1799000),
	FAN53870_PLDO(5, "vin5", 2799000),
	FAN53870_PLDO(6, "vin6", 2799000),
	FAN53870_PLDO(7, "vin7", 1799000),
};

static const struct regmap_config fan53870_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FAN53870_REG_MAX,
};

static int fan53870_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct fan53870_platform_data *pdata;
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
				sizeof(struct fan53870_platform_data), GFP_KERNEL);
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
	pdata->regmap = devm_regmap_init_i2c(i2c, &fan53870_regmap);
	if (IS_ERR(pdata->regmap)) {
		ret = PTR_ERR(pdata->regmap);
		dev_err(&i2c->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	ret = regmap_read(pdata->regmap, FAN53870_PRODUCT_ID, &data);

	while(ret<0 && --access_time) {
		mdelay(2);
		ret = regmap_read(pdata->regmap, FAN53870_PRODUCT_ID, &data);
	}
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to read PRODUCT_ID: %d\n", ret);
		return ret;
	}
	if (data != FAN53870_ID) {
		dev_err(&i2c->dev, "Unsupported device id: 0x%x.\n", data);
		return -ENODEV;
	}
		ret = regmap_read(pdata->regmap, 0x0C, &data);
		dev_err(&i2c->dev, "read reg[0x0c] =0x%02x.\n", data);

	config.dev = &i2c->dev;
	config.regmap = pdata->regmap;
	config.init_data = NULL;

	for (i = 0; i < ARRAY_SIZE(fan53870_regulators); i++) {
		rdev = devm_regulator_register(&i2c->dev,
					       &fan53870_regulators[i],
					       &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(&i2c->dev, "Failed to register %s: %d\n",
				fan53870_regulators[i].name, ret);
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
			fan53870_regulators[i].name, ret);
	}
	dev_err(&i2c->dev, "regulator probe end\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id fan53870_dt_ids[] = {
	{ .compatible = "fan53870", },
	{}
};
MODULE_DEVICE_TABLE(of, fan53870_dt_ids);
#endif

static const struct i2c_device_id fan53870_i2c_id[] = {
	{ "fan53870-pmic", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fan53870_i2c_id);

static struct i2c_driver fan53870_regulator_driver = {
	.driver = {
		.name = "fan53870-pmic",
		.owner = THIS_MODULE
		//.of_match_table	= of_match_ptr(fan53870_dt_ids),
	},
	.probe = fan53870_i2c_probe,
	.id_table = fan53870_i2c_id,
};

module_i2c_driver(fan53870_regulator_driver);

MODULE_DESCRIPTION("FAN53870 PMIC voltage regulator driver");
MODULE_AUTHOR("XXX");
MODULE_LICENSE("GPL");
