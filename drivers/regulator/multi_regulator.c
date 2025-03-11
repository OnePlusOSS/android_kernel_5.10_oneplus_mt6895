// SPDX-License-Identifier: GPL-2.0+
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

struct multi_reg_platform_data {
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
enum wl2868c_regulator_ids {
    WL2868C_LDO1,
    WL2868C_LDO2,
    WL2868C_LDO3,
    WL2868C_LDO4,
    WL2868C_LDO5,
    WL2868C_LDO6,
    WL2868C_LDO7,
};

enum wl28681c_regulator_ids {
    WL28681C_LDO1,
    WL28681C_LDO2,
    WL28681C_LDO3,
    WL28681C_LDO4,
    WL28681C_LDO5,
    WL28681C_LDO6,
    WL28681C_LDO7,
};

typedef enum {
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
} multi_reg_fan53870_registers_t;

typedef enum {
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
} multi_reg_wl2868_registers_t;

typedef enum {
    WL28681C_PRODUCT_ID = 0x00,
    WL28681C_SILICON_REV,
    WL28681C_IOUT,
    WL28681C_ENABLE,
    WL28681C_LDO1VOUT,
    WL28681C_LDO2VOUT,
    WL28681C_LDO3VOUT,
    WL28681C_LDO4VOUT,
    WL28681C_LDO5VOUT,
    WL28681C_LDO6VOUT,
    WL28681C_LDO7VOUT,
    WL28681C_REG_MAX = 0x0d,
} multi_reg_wl28681c_registers_t;

#define MULTI_REG_MAX_REG (WL2868C_REG_MAX)

static const struct regulator_ops multi_reg_ops = {
    .enable = regulator_enable_regmap,
    .disable = regulator_disable_regmap,
    .is_enabled = regulator_is_enabled_regmap,
    .list_voltage = regulator_list_voltage_linear_range,
    .map_voltage = regulator_map_voltage_linear_range,
    .set_voltage_sel = regulator_set_voltage_sel_regmap,
    .get_voltage_sel = regulator_get_voltage_sel_regmap,
};

#define FAN53870_NLDO(_num, _supply, _default)                \
    [FAN53870_LDO ## _num] = {                    \
        .name =           "ONLDO"#_num,            \
        .of_match =       of_match_ptr("ONLDO"#_num),        \
        .regulators_node = of_match_ptr("regulators"),        \
        .type =           REGULATOR_VOLTAGE,            \
        .owner =       THIS_MODULE,                \
        .linear_ranges =   (struct linear_range[]) {        \
              REGULATOR_LINEAR_RANGE(_default, 0, 0x62, 0),    \
              REGULATOR_LINEAR_RANGE(800000, 0x63, 0xba, 8000),    \
        },                            \
        .n_linear_ranges = 2,                    \
        .vsel_reg =       FAN53870_LDO ## _num ## VOUT,    \
        .vsel_mask =       0xff,                \
        .enable_reg =       FAN53870_ENABLE,            \
        .enable_mask =       BIT(_num - 1),            \
        .enable_time =       150,                    \
        .supply_name =       _supply,                \
        .ops =           &multi_reg_ops,            \
    }

#define FAN53870_PLDO(_num, _supply, _default)                \
    [FAN53870_LDO ## _num] = {                    \
        .name =           "ONLDO"#_num,                \
        .of_match =       of_match_ptr("ONLDO"#_num),        \
        .regulators_node = of_match_ptr("regulators"),        \
        .type =           REGULATOR_VOLTAGE,            \
        .owner =       THIS_MODULE,                \
        .linear_ranges =   (struct linear_range[]) {        \
              REGULATOR_LINEAR_RANGE(_default, 0, 0x0f, 0),    \
              REGULATOR_LINEAR_RANGE(1500000, 0x10, 0xff, 8000),    \
        },                            \
        .n_linear_ranges = 2,                    \
        .vsel_reg =       FAN53870_LDO ## _num ## VOUT,    \
        .vsel_mask =       0xff,                \
        .enable_reg =       FAN53870_ENABLE,            \
        .enable_mask =       BIT(_num - 1),            \
        .enable_time =       150,                    \
        .supply_name =       _supply,                \
        .ops =           &multi_reg_ops,            \
    }

/*wl2868 data*/
#define WL2868C_NLDO(_num, _supply, _default)                \
    [WL2868C_LDO ## _num] = {                    \
        .name =           "ONLDO"#_num,            \
        .of_match =       of_match_ptr("ONLDO"#_num),        \
        .regulators_node = of_match_ptr("regulators"),        \
        .type =           REGULATOR_VOLTAGE,            \
        .owner =       THIS_MODULE,                \
        .linear_ranges =   (struct linear_range[]) {        \
              REGULATOR_LINEAR_RANGE(496000, 0x0, 0xff, 8000),    \
        },                            \
        .n_linear_ranges = 1,                    \
        .vsel_reg =       WL2868C_LDO ## _num ## VOUT,    \
        .vsel_mask =       0xff,                \
        .enable_reg =       WL2868C_ENABLE,            \
        .enable_mask =       BIT(_num - 1),            \
        .enable_time =       150,                    \
        .supply_name =       _supply,                \
        .ops =           &multi_reg_ops,            \
    }

#define WL2868C_PLDO(_num, _supply, _default)                \
    [WL2868C_LDO ## _num] = {                    \
        .name =           "ONLDO"#_num,                \
        .of_match =       of_match_ptr("ONLDO"#_num),        \
        .regulators_node = of_match_ptr("regulators"),        \
        .type =           REGULATOR_VOLTAGE,            \
        .owner =       THIS_MODULE,                \
        .linear_ranges =   (struct linear_range[]) {        \
              REGULATOR_LINEAR_RANGE(1504000, 0x0, 0xff, 8000),    \
        },                            \
        .n_linear_ranges = 1,                    \
        .vsel_reg =       WL2868C_LDO ## _num ## VOUT,    \
        .vsel_mask =       0xff,                \
        .enable_reg =       WL2868C_ENABLE,            \
        .enable_mask =       BIT(_num - 1),            \
        .enable_time =       150,                    \
        .supply_name =       _supply,                \
        .ops =           &multi_reg_ops,            \
    }

#define WL28681C_NLDO(_num, _supply, _default)                \
    [WL28681C_LDO ## _num] = {                    \
        .name =           "ONLDO"#_num,            \
        .of_match =       of_match_ptr("ONLDO"#_num),        \
        .regulators_node = of_match_ptr("regulators"),        \
        .type =           REGULATOR_VOLTAGE,            \
        .owner =       THIS_MODULE,                \
        .linear_ranges =   (struct linear_range[]) {        \
              REGULATOR_LINEAR_RANGE(_default, 0, 0x3C, 0),    \
              REGULATOR_LINEAR_RANGE(496000, 0x3D, 0xFF, 8000),    \
        },                            \
        .n_linear_ranges = 2,                    \
        .vsel_reg =       WL28681C_LDO ## _num ## VOUT,    \
        .vsel_mask =       0xff,                \
        .enable_reg =       WL28681C_ENABLE,            \
        .enable_mask =       BIT(_num - 1),            \
        .enable_time =       150,                    \
        .supply_name =       _supply,                \
        .ops =           &multi_reg_ops,            \
    }

#define WL28681C_PLDO(_num, _supply, _default)                \
    [WL28681C_LDO ## _num] = {                    \
        .name =           "ONLDO"#_num,                \
        .of_match =       of_match_ptr("ONLDO"#_num),        \
        .regulators_node = of_match_ptr("regulators"),        \
        .type =           REGULATOR_VOLTAGE,            \
        .owner =       THIS_MODULE,                \
        .linear_ranges =   (struct linear_range[]) {        \
              REGULATOR_LINEAR_RANGE(1372000, 0x00, 0xff, 8000),    \
        },                            \
        .n_linear_ranges = 1,                    \
        .vsel_reg =       WL28681C_LDO ## _num ## VOUT,    \
        .vsel_mask =       0xff,                \
        .enable_reg =       WL28681C_ENABLE,            \
        .enable_mask =       BIT(_num - 1),            \
        .enable_time =       150,                    \
        .supply_name =       _supply,                \
        .ops =           &multi_reg_ops,            \
    }

static struct regulator_desc multi_reg_fan53870_regulators[] = {
    FAN53870_NLDO(1, "vin12", 809000),
    FAN53870_NLDO(2, "vin12", 1049000),
    FAN53870_PLDO(3, "vin34", 2799000),
    FAN53870_PLDO(4, "vin34", 1799000),
    FAN53870_PLDO(5, "vin5", 2799000),
    FAN53870_PLDO(6, "vin6", 2799000),
    FAN53870_PLDO(7, "vin7", 1799000),
};

static struct regulator_desc multi_reg_wl2868_regulators[] = {
    WL2868C_NLDO(1, "vin12", 1200000),
    WL2868C_NLDO(2, "vin12",  808000),
    WL2868C_PLDO(3, "vin34", 2800000),
    WL2868C_PLDO(4, "vin34", 2904000),
    WL2868C_PLDO(5, "vin5", 2704000),
    WL2868C_PLDO(6, "vin6", 2800000),
    WL2868C_PLDO(7, "vin7", 2800000),
};

static struct regulator_desc multi_reg_wl28681c_regulators[] = {
    WL28681C_NLDO(1, "vin12", 1000000),
    WL28681C_NLDO(2, "vin12", 1000000),
    WL28681C_PLDO(3, "vin34", 1700000),
    WL28681C_PLDO(4, "vin34", 1700000),
    WL28681C_PLDO(5, "vin5", 1700000),
    WL28681C_PLDO(6, "vin6", 1700000),
    WL28681C_PLDO(7, "vin7", 1700000),
};
typedef struct {
    int mult_reg_slave_id;
    int product_id_reg;
    int product_id;
    struct regulator_desc *reg_desc;
    int reg_desc_size;
} mult_reg_dev_info_t;

/*SlaveId is 7bit address, 0xFF is a invalid address.*/
static mult_reg_dev_info_t dev_info[] = {
    /*wl2868*/
    {.mult_reg_slave_id = 0x2F,
     .product_id_reg = WL2868C_PRODUCT_ID,
     .product_id = 0x82,
     .reg_desc = &multi_reg_wl2868_regulators[0],
     .reg_desc_size = ARRAY_SIZE(multi_reg_wl2868_regulators),
    },
    /*Fan53870*/
    {.mult_reg_slave_id = 0x35,
     .product_id_reg = FAN53870_PRODUCT_ID,
     .product_id = 0x01,
     .reg_desc = &multi_reg_fan53870_regulators[0],
     .reg_desc_size = ARRAY_SIZE(multi_reg_fan53870_regulators),
    },
    /*wl28681c*/
    {.mult_reg_slave_id = 0x35,
     .product_id_reg = WL28681C_PRODUCT_ID,
     .product_id = 0x0D,
     .reg_desc = &multi_reg_wl28681c_regulators[0],
     .reg_desc_size = ARRAY_SIZE(multi_reg_wl28681c_regulators),
    },
    /*Ivalid*/
    {0xff},
};

static const struct regmap_config multi_reg_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = MULTI_REG_MAX_REG,
};

static int multi_reg_i2c_probe(struct i2c_client *i2c,
                 const struct i2c_device_id *id)
{
    struct multi_reg_platform_data *pdata;
    struct regulator_config config = { };
    struct regulator_dev *rdev;
    int i, ret, index=0;
    unsigned int data;
    struct pinctrl *pinctrl;
    struct pinctrl_state *reset_output_high;
    int access_time = 3;
    struct regulator_desc *multi_reg_regulators = NULL;
    if (i2c->dev.of_node) {
        pdata = devm_kzalloc(&i2c->dev,
                sizeof(struct multi_reg_platform_data), GFP_KERNEL);
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

    /*process the fan53870 wl2868 wl28681c IC*/
    pdata->regmap = devm_regmap_init_i2c(i2c, &multi_reg_regmap);
    if (IS_ERR(pdata->regmap)) {
        ret = PTR_ERR(pdata->regmap);
        dev_err(&i2c->dev, "Failed to create regmap: %d\n", ret);
        return ret;
    }

    while(dev_info[index].mult_reg_slave_id != 0xFF){
        dev_info(&i2c->dev, "get product id of regulator(slaveId:0x%x)\n", dev_info[index].mult_reg_slave_id);
        i2c->addr = dev_info[index].mult_reg_slave_id;
        ret = regmap_read(pdata->regmap, dev_info[index].product_id_reg, &data);
        while(ret<0 && --access_time) {
            mdelay(2);
            ret = regmap_read(pdata->regmap, dev_info[index].product_id_reg, &data);
        }
        if (ret < 0) {
            dev_err(&i2c->dev, "Failed to read PRODUCT_ID: %d\n", ret);
            index += 1;
            continue;
        }
        if (data == dev_info[index].product_id) {
            break;
        }
        index += 1;
    }
    if(0xFF == dev_info[index].mult_reg_slave_id){
        dev_err(&i2c->dev, "No valid regulator IC.\n");
        return -ENODEV;
    }
    dev_info(&i2c->dev, "find the regulator ic, slaveId:0x%x.\n", dev_info[index].mult_reg_slave_id);

    /*Common flow*/
    config.dev = &i2c->dev;
    config.regmap = pdata->regmap;
    config.init_data = NULL;
    config.ena_gpiod = NULL;
    multi_reg_regulators = dev_info[index].reg_desc;
    for (i = 0; i < dev_info[index].reg_desc_size; i++) {
        rdev = devm_regulator_register(&i2c->dev,
                           &multi_reg_regulators[i],
                           &config);
        if (IS_ERR(rdev)) {
            ret = PTR_ERR(rdev);
            dev_err(&i2c->dev, "Failed to register %s: %d\n",
                multi_reg_regulators[i].name, ret);
            return ret;
        }
        dev_info(&i2c->dev, "register regulator ldo %s ok\n", multi_reg_regulators[i].name);
    }
    dev_info(&i2c->dev, "regulator probe end\n");
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id multi_reg_dt_ids[] = {
    { .compatible = "multi_reg-pmic", },
    {}
};
MODULE_DEVICE_TABLE(of, multi_reg_dt_ids);
#endif

static const struct i2c_device_id multi_reg_i2c_id[] = {
    { "multi_reg-pmic", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, multi_reg_i2c_id);

static struct i2c_driver multi_reg_regulator_driver = {
    .driver = {
        .name = "multi_reg-pmic",
        .owner = THIS_MODULE
        //.of_match_table    = of_match_ptr(multi_reg_dt_ids),
    },
    .probe = multi_reg_i2c_probe,
    .id_table = multi_reg_i2c_id,
};

module_i2c_driver(multi_reg_regulator_driver);

MODULE_DESCRIPTION("multi PMIC voltage regulator driver");
MODULE_AUTHOR("XXX");
MODULE_LICENSE("GPL");
