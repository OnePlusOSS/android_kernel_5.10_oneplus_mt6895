#include <dt-bindings/iio/mt635x-auxadc.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/types.h>

#define PFX "OPLUS_CAM_NTC"
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)


struct pmic_tz_data {
    struct iio_channel *iio_chan;
    char *iio_chan_name;
};

struct cam_ntc_info {
    struct device *dev;
    int sersor_num;
    int *lookup_table;
    int lookup_table_num;
    int pullup_voltage;
    int pullup_resistor;
    int sensor_num;
    struct pmic_tz_data *pmic_data;
};

struct pmic_temp_tz {
    int tz_id;
    struct cam_ntc_info *pmic_tz_info;
};

static int pmic_temp_parse_iio_channel(struct device *dev,
                    struct cam_ntc_info *pmic_info)
{
    int ret;
    struct pmic_tz_data *tz_data = pmic_info->pmic_data;
    int i = 0, channel_num;
    if (of_property_read_u32(dev->of_node, "io-channels-num", &channel_num) == 0) {
        pmic_info->sensor_num = channel_num;
    }
    LOG_INF("pmic_temp_parse_iio_channel in, channel_num:%d\n", pmic_info->sensor_num);
    for (i = 0; i < pmic_info->sensor_num; i++) {
        LOG_INF("In %s: get i2o chan name %s", __func__, tz_data[i].iio_chan_name);
        tz_data[i].iio_chan = devm_iio_channel_get(dev, tz_data[i].iio_chan_name);
        ret = IS_ERR(tz_data[i].iio_chan);
        if (ret) {
            pr_info("pmic_chip_temp auxadc get fail, ret=%d\n", ret);
            return ret;
        }
    }

    LOG_INF("pmic_temp_parse_iio_channel out\n");
    return 0;
}
static int cam_ntc_parse_lookup_table(struct device *dev,
                    struct cam_ntc_info *ntc_info)
{
    struct device_node *np = dev->of_node;
    int num, ret;
    LOG_INF("cam_ntc_parse_lookup_table in\n");

    num = of_property_count_elems_of_size(np, "temperature-lookup-table", sizeof(unsigned int));
    if (num < 0) {
        dev_err(dev, "lookup table is not found\n");
        return num;
    }
    LOG_INF("cam_ntc_parse_lookup_table num = %d\n",num);
    if (num % 2) {
        dev_err(dev, "temp vs ADC value in table are unpaired\n");
        return -EINVAL;
    }

    ntc_info->lookup_table = devm_kcalloc(dev, num,
                     sizeof(*ntc_info->lookup_table),
                     GFP_KERNEL);
    if (!ntc_info->lookup_table)
        return -ENOMEM;

    ret = of_property_read_u32_array(np, "temperature-lookup-table",
                    (unsigned int *)ntc_info->lookup_table,
                    num);
    if (ret < 0) {
        dev_err(dev, "Failed to read temperature lookup table: %d\n",
            ret);
        return ret;
    }

    ntc_info->lookup_table_num = num / 2;
    LOG_INF("cam_ntc_parse_lookup_table lookup_table_num = %d\n",ntc_info->lookup_table_num);
    return 0;
}

struct chtemperature {
    __s32 bts_temp;
    __s32 temperature_r;
};

static struct chtemperature vol_temp_table[] = {
    {-40, 4397119}, {-39, 4092874}, {-38, 3811717}, {-37, 3551749}, {-36, 3311236}, {-35, 3088599}, {-34, 2882396}, {-33, 2691310},
    {-32, 2514137}, {-31, 2349778}, {-30, 2197225}, {-29, 2055558}, {-28, 1923932}, {-27, 1801573}, {-26, 1687773}, {-25, 1581881},
    {-24, 1483100}, {-23, 1391113}, {-22, 1305413}, {-21, 1225531}, {-20, 1151037}, {-19, 1081535}, {-18, 1016661}, {-17,  956080},
    {-16,  899481}, {-15,  846579}, {-14,  797111}, {-13,  750834}, {-12,  707524}, {-11,  666972}, {-10,  628988}, {-9,   593342},
    {-8,   559931}, {-7,   528602}, {-6,   499212}, {-5,   471632}, {-4,   445772}, {-3,   421480}, {-2,   398652}, {-1,   377193},
    {0,    357012}, {1,    338006}, {2,    320122}, {3,    303287}, {4,    287434}, {5,    272500}, {6,    258426}, {7,    245160},
    {8,    232649}, {9,    220847}, {10,   209710}, {11,   199196}, {12,   189268}, {13,   179890}, {14,   171028}, {15,   162651},
    {16,   154726}, {17,   147232}, {18,   140142}, {19,   133432}, {20,   127080}, {21,   121066}, {22,   115368}, {23,   109970},
    {24,   104852}, {25,   100000}, {26,   95398 }, {27,   91032 }, {28,   86889 }, {29,   82956 }, {30,   79222 }, {31,   75675 },
    {32,   72306 }, {33,   69104 }, {34,   66061 }, {35,   63167 }, {36,   60415 }, {37,   57797 }, {38,   55306 }, {39,   52934 },
    {40,   50677 }, {41,   48528 }, {42,   46482 }, {43,   44533 }, {44,   42675 }, {45,   40904 }, {46,   39213 }, {47,   37601 },
    {48,   36063 }, {49,   34595 }, {50,   33195 }, {51,   31859 }, {52,   30584 }, {53,   29366 }, {54,   28203 }, {55,   27091 },
    {56,   26028 }, {57,   25013 }, {58,   24042 }, {59,   23113 }, {60,   22224 }, {61,   21374 }, {62,   20561 }, {63,   19782 },
    {64,   19036 }, {65,   18323 }, {66,   17640 }, {67,   16986 }, {68,   16360 }, {69,   15760 }, {70,   15184 }, {71,   14631 },
    {72,   14101 }, {73,   13592 }, {74,   13104 }, {75,   12635 }, {76,   12187 }, {77,   11757 }, {78,   11344 }, {79,   10947 },
    {80,   10566 }, {81,   10200 }, {82,     9848}, {83,     9510}, {84,     9185}, {85,     8873}, {86,     8572}, {87,     8283},
    {88,     8006}, {89,     7738}, {90,     7481}, {91,     7234}, {92,     6997}, {93,     6769}, {94,     6548}, {95,     6337},
    {96,     6132}, {97,     5934}, {98,     5744}, {99,     5561}, {100,    5384}, {101,    5214}, {102,    5051}, {103,    4893},
    {104,    4741}, {105,    4594}, {106,    4453}, {107,    4316}, {108,    4184}, {109,    4057}, {110,    3934}, {111,    3816},
    {112,    3701}, {113,    3591}, {114,    3484}, {115,    3380}, {116,    3281}, {117,    3185}, {118,    3093}, {119,    3003},
    {120,    2916}, {121,    2832}, {122,    2751}, {123,    2672}, {124,    2596}, {125,    2522}
};

static __s16 oplus_cam_thermistor_conver_temp(__s32 res)
{
    int i = 0;
    int asize = 0;
    __s32 res1 = 0, res2 = 0;
    __s32 tap_value = -2000, tmp1 = 0, tmp2 = 0;

    asize = (sizeof(vol_temp_table) / sizeof(struct chtemperature));

    if (res >= vol_temp_table[0].temperature_r) {
        tap_value = -400;    /* min */
    } else if (res <= vol_temp_table[asize - 1].temperature_r) {
        tap_value = 1250;    /* max */
    } else {
        res1 = vol_temp_table[0].temperature_r;
        tmp1 = vol_temp_table[0].bts_temp;

        for (i = 0; i < asize; i++) {
            if (res >= vol_temp_table[i].temperature_r) {
                res2 = vol_temp_table[i].temperature_r;
                tmp2 = vol_temp_table[i].bts_temp;
                break;
            }
            res1 = vol_temp_table[i].temperature_r;
            tmp1 = vol_temp_table[i].bts_temp;
        }

        tap_value = (((res - res2) * tmp1) + ((res1 - res) * tmp2)) * 10 / (res1 - res2);
    }

    return tap_value;
}

static __s16 oplus_ts_cam_volt_to_temp(__u32 dwvolt,struct cam_ntc_info *pmic_info)
{
    __s32 tres;
    __u64 dwvcrich = 0;
    __s32 cam_tmp = -100;
    __u64 dwvcrich2 = 0;
    const int g_tap_over_critical_low = 4397119;
    const int g_rap_pull_up_r = pmic_info->pullup_resistor; /* pull up resister */
    const int g_rap_pull_up_voltage = pmic_info->pullup_voltage;

    dwvcrich = ((__u64)g_tap_over_critical_low * (__u64)g_rap_pull_up_voltage);
    dwvcrich2 = (g_tap_over_critical_low + g_rap_pull_up_r);
    do_div(dwvcrich, dwvcrich2);

    if (dwvolt > ((__u32)dwvcrich)) {
        tres = g_tap_over_critical_low;
    } else {
        tres = (g_rap_pull_up_r * dwvolt) / (g_rap_pull_up_voltage - dwvolt);
    }

    /* convert register to temperature */
    cam_tmp = oplus_cam_thermistor_conver_temp(tres);

    return cam_tmp;
}

static int pmic_get_temp(void *data, int *temp)
{
    int val = 0;
    int ret = 0, output;
    struct pmic_temp_tz *pmic_tz = (struct pmic_temp_tz *)data;
    struct cam_ntc_info *temp_info = pmic_tz->pmic_tz_info;
    struct pmic_tz_data *tz_data = temp_info->pmic_data;

    LOG_INF("tz_id = %d\n",pmic_tz->tz_id);
    ret = iio_read_channel_processed(tz_data[pmic_tz->tz_id].iio_chan, &val);
    if (ret < 0) {
        pr_notice("pmic_chip_temp read fail, ret=%d\n", ret);
        *temp = THERMAL_TEMP_INVALID;
        dev_info(temp_info->dev, "temp = %d\n", *temp);
        return -EINVAL;
    }
    LOG_INF("pmic_get_temp() val = %d\n",val);
    ret = val;
    output = oplus_ts_cam_volt_to_temp(ret,temp_info);

    *temp = output * 100;
    LOG_INF("*temp = %d\n",*temp);

    return 0;
}

static const struct thermal_zone_of_device_ops pmic_temp_ops = {
    .get_temp = pmic_get_temp,
};

static int pmic_register_thermal_zones(struct cam_ntc_info *pmic_info)
{
    //struct pmic_tz_data *tz_data = pmic_info->pmic_data;
    struct thermal_zone_device *tzdev;
    struct device *dev = pmic_info->dev;
    struct pmic_temp_tz *pmic_tz;
    int i, ret;
    LOG_INF("pmic_register_thermal_zones in\n");

    for (i = 0; i < pmic_info->sensor_num; i++) {
        pmic_tz = devm_kzalloc(dev, sizeof(*pmic_tz), GFP_KERNEL);
        if (!pmic_tz)
            return -ENOMEM;

        pmic_tz->tz_id = i;
        pmic_tz->pmic_tz_info = pmic_info;
        tzdev = devm_thermal_zone_of_sensor_register(dev, pmic_tz->tz_id,
                pmic_tz, &pmic_temp_ops);

        if (IS_ERR(tzdev)) {
            ret = PTR_ERR(tzdev);
            dev_info(dev,
                "Error: Failed to register pmic tz %d, ret = %d\n",
                pmic_tz->tz_id, ret);
            return ret;
        }
    }

    LOG_INF("pmic_register_thermal_zones out\n");
    return 0;
}
static int cam_temp_probe(struct platform_device *pdev)
{
    struct cam_ntc_info *pmic_info;
    int ret;
    LOG_INF("Enter into %s\n", __func__);

    if (!pdev->dev.of_node) {
        dev_info(&pdev->dev, "Only DT based supported\n");
        return -ENODEV;
    }

    pmic_info = devm_kzalloc(&pdev->dev, sizeof(*pmic_info), GFP_KERNEL);
    if (!pmic_info)
        return -ENOMEM;
    //初始化基本配置
    pmic_info->dev = &pdev->dev;
    pmic_info->sensor_num = 2;
    pmic_info->pmic_data = (struct pmic_tz_data *)of_device_get_match_data(&pdev->dev);

    if(of_property_read_u32(pdev->dev.of_node, "ntc_pullup_voltage", &pmic_info->pullup_voltage)!=0){
        pmic_info->pullup_voltage = 1800; //1.8V上拉电压
    }
    if(of_property_read_u32(pdev->dev.of_node, "ntc_pullup_resistor", &pmic_info->pullup_resistor)!=0){
        pmic_info->pullup_resistor = 100000; //100k上拉电阻
    }
    ret = cam_ntc_parse_lookup_table(&pdev->dev, pmic_info);
    if (ret < 0)
        return ret;

    ret = pmic_temp_parse_iio_channel(&pdev->dev, pmic_info);
    if (ret < 0)
        return ret;
    LOG_INF("cam_temp_probe pmic_info pullup_voltage=%d,pullup_resistor=%d",pmic_info->pullup_voltage,pmic_info->pullup_resistor);
    platform_set_drvdata(pdev, pmic_info);

    ret = pmic_register_thermal_zones(pmic_info);
    if (ret)
        return ret;
    LOG_INF("cam_temp_probe sucess \n");

    return 0;
}
static struct pmic_tz_data cam_tz_data[] = {
    [0] = {
        .iio_chan_name = "pmic_adc_channel6",
    },
    [1] = {
        .iio_chan_name = "pmic_adc_channel7",
    },
};
static struct pmic_tz_data cam_tz_data_dragonfly[] = {
    [0] = {
        .iio_chan_name = "pmic_adc_channel1",
    },
    [1] = {
        .iio_chan_name = "pmic_adc_channel3",
    },
};
static struct pmic_tz_data cam_tz_data_vitamin[] = {
    [0] = {
        .iio_chan_name = "pmic_adc_channel3",
    },
};
static struct pmic_tz_data cam_tz_data_aries[] = {
    [0] = {
        .iio_chan_name = "pmic_adc_channel3",
    },
};
static const struct of_device_id cam_temp_of_match[] = {
    {
        .compatible = "mediatek,cam-temp",
        .data = (void *)&cam_tz_data,
    },
    {
        .compatible = "mediatek,cam-temp-dragonfly",
        .data = (void *)&cam_tz_data_dragonfly,
    },
    {
        .compatible = "mediatek,cam-temp-aries",
        .data = (void *)&cam_tz_data_aries,
    },
    {
        .compatible = "mediatek,cam-temp-vitamin",
        .data = (void *)&cam_tz_data_vitamin,
    },
    {},
};
MODULE_DEVICE_TABLE(of, cam_temp_of_match);
static struct platform_driver oplus_temp_driver = {
    .probe = cam_temp_probe,
    .driver = {
        .name = "cam-temp",
        .of_match_table = cam_temp_of_match,
    },
};

module_platform_driver(oplus_temp_driver);

MODULE_AUTHOR("Henry Huang <henry.huang@mediatek.com>");
MODULE_DESCRIPTION("Mediatek pmic temp camera driver");
MODULE_LICENSE("GPL v2");
