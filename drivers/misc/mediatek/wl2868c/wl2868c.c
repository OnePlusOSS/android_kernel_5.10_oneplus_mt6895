/*
 * Copyright (C) 2015 HUAQIN Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*

Here is WL2868C Driver,
WL2868C Interface is in vnd\kernel-5.10\drivers\misc\mediatek\imgsensor\src\common\v1_1\camera_hw\wl2868c_hw

*/

#include "wl2868c.h"
#include <soc/oplus/system/oplus_project.h>

/*****************************************************************************
 * Static Var
 *****************************************************************************/
static int ldo_id = 0;
static int power_reference_counts[] = {0,0,1,0,0,0,0,0};
static struct i2c_device which_ldo_chip[] = {
    //ldo i2c addr,         chip id addr,           chip id,             enable addr,
    {WL2868C_LDO_I2C_ADDR,  WL2868C_CHIP_REV_ADDR,  CAMERA_LDO_WL2868C,  WL2868C_LDO_EN_ADDR},
    {ET5907_LDO_I2C_ADDR,   ET5907_CHIP_REV_ADDR,   CAMERA_LDO_ET5907,   ET5907_LDO_EN_ADDR},
    {FAN53870_LDO_I2C_ADDR, FAN53870_CHIP_REV_ADDR, CAMERA_LDO_FAN53870, FAN53870_LDO_EN_ADDR},
};
static struct i2c_client *wl2868c_i2c_client;
static struct pinctrl *wl2868c_pctrl; /* static pinctrl instance */
struct mutex i2c_control_mutex;
static const char *wl2868c_state_name[WL2868C_GPIO_STATE_MAX] = {
    "wl2868c_gpio_rst0",
    "wl2868c_gpio_rst1",
    "wl2868c_gpio_ext_buck_en0",
    "wl2868c_gpio_ext_buck_en1",
    "wl2836d18_gpio_en0",
    "wl2836d18_gpio_en1"
};/* GPIO state mapping name */
static const struct of_device_id gpio_of_match[] = {
    { .compatible = "mediatek,gpio_wl2868c", },
    {},
};
static const struct of_device_id i2c_of_match[] = {
    { .compatible = "mediatek,i2c_wl2868c", },
    {},
};
static const struct i2c_device_id wl2868c_i2c_id[] = {
    {"WL2868C_I2C", 0},
    {},
};

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int wl2868c_gpio_probe(struct platform_device *pdev);
static int wl2868c_gpio_remove(struct platform_device *pdev);
static int wl2868c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int wl2868c_i2c_remove(struct i2c_client *client);

/*****************************************************************************
 * Sub Function
 *****************************************************************************/
static int wl2868c_write(unsigned short addr, unsigned short value){
    return i2c_smbus_write_byte_data(wl2868c_i2c_client, addr, value);
}

static int wl2868c_read(unsigned short addr){
    return i2c_smbus_read_byte_data(wl2868c_i2c_client, addr);
}

static long wl2868c_set_state(const char *name)
{
    int ret = 0;
    struct pinctrl_state *pState = 0;

    BUG_ON(!wl2868c_pctrl);

    pState = pinctrl_lookup_state(wl2868c_pctrl, name);
    if (IS_ERR(pState)) {
        pr_err("set state '%s' failed\n", name);
        ret = PTR_ERR(pState);
        goto exit;
    }

    /* select state! */
    pinctrl_select_state(wl2868c_pctrl, pState);

exit:
    return ret; /* Good! */
}

static void wl2868c_gpio_select_state(enum WL2868C_GPIO_STATE s)
{
    WL2868C_PRINT("[wl2868c]%s,%d\n",__FUNCTION__,s);

    BUG_ON(!((unsigned int)(s) < (unsigned int)(WL2868C_GPIO_STATE_MAX)));
    wl2868c_set_state(wl2868c_state_name[s]);
}

static void wl2868c_set_ext_buck_en(enum WL2868C_SELECT ldonum, unsigned int en){
    if (ldonum == LDO1 || ldonum == LDO2) {
        switch (en){
            case 0:
                if (power_reference_counts[0] | power_reference_counts[1]);
                else wl2868c_gpio_select_state(WL2868C_GPIO_EXT_BUCK_EN0);
                WL2868C_PRINT("[wl2868c] %s close WL2868C_GPIO_EXT_BUCK_EN for LDO1 & LDO2\n",__FUNCTION__);
                break;
            case 1:
                wl2868c_gpio_select_state(WL2868C_GPIO_EXT_BUCK_EN1);
                WL2868C_PRINT("[wl2868c] %s open WL2868C_GPIO_EXT_BUCK_EN for LDO1 & LDO2\n",__FUNCTION__);
                break;
            default:
                break;
        }
    }
}

static void wl2868c_set_en_ldo(enum WL2868C_SELECT ldonum, unsigned int en)
{
    int ret = 0;
    unsigned int value =0;

    if (NULL == wl2868c_i2c_client) {
            WL2868C_PRINT("[wl2868c] wl2868c_i2c_client is null!!\n");
            return ;
    }

    if(ldonum < LDO1 || ldonum > LDO8) {
        WL2868C_PRINT("[wl2868c] %s error ldonum not support!!!\n",__FUNCTION__);
        return;
    }

    ret = wl2868c_read(which_ldo_chip[ldo_id].enable_addr);
    if (ret < 0)
    {
        WL2868C_PRINT("[wl2868c] wl2868c_set_en_ldo read error!\n");
        return;
    }

    if(en == 0)
    {
        value = (ret & (~(0x01<<ldonum)));
    }
    else
    {
        if(which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2868C) {
            value = (ret|(0x01<<ldonum))|0x80;
        } else if ((which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5907)
                || (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_FAN53870)) {
            value = (ret|(0x01<<ldonum));
        }
    }

    ret = wl2868c_write(which_ldo_chip[ldo_id].enable_addr, value);
    if (ret < 0)
    {
        WL2868C_PRINT("[wl2868c] %s write error.\n", __FUNCTION__);
        return;
    }
    WL2868C_PRINT("[wl2868c] wl2868c_set_en_ldo enable before:%x after set :%x\n", ret, value);
    return;

}

static void wl2836d18_set_en_ldo(enum WL2868C_GPIO_STATE en){
    struct pinctrl_state *pState = 0;

    BUG_ON(!wl2868c_pctrl);
    BUG_ON(!((unsigned int)(en) < (unsigned int)(WL2868C_GPIO_STATE_MAX)));

    pState = pinctrl_lookup_state(wl2868c_pctrl, wl2868c_state_name[en]);
    if (IS_ERR(pState)) {
        pr_err("set state '%s' failed\n", wl2868c_state_name[en]);
        PTR_ERR(pState);
        goto exit;
    }

    /* select state! */
    pinctrl_select_state(wl2868c_pctrl, pState);

exit:
    return;
}

/** wl2868c
   Voutx=0.496v+LDOX_OUT[6:0]*0.008V LDO1/LDO2
   Voutx=1.504v+LDOX_OUT[7:0]*0.008V LDO3~LDO7
===FAN53870
   Voutx=0.800v+(LDOX_OUT[6:0]-99)*0.008V LDO1/LDO2
   Voutx=1.500v+()LDOX_OUT[7:0]-16)*0.008V LDO3~LDO7
===ET5907
   Voutx=0.600v+LDOX_OUT[7:0]*0.006V LDO1/LDO2
   Voutx=1.200v+LDOX_OUT[7:0]*0.010V LDO3~LDO7
*/
static void wl2868c_set_ldo_value(enum WL2868C_SELECT ldonum,unsigned int value)
{
    unsigned int  Ldo_out = 0;
    unsigned short regaddr = 0;
    int ret = 0;

    WL2868C_PRINT("[wl2868c] %s begin\n",__FUNCTION__);

    if (NULL == wl2868c_i2c_client) {
        WL2868C_PRINT("[wl2868c] wl2868c_i2c_client is null!!\n");
        return ;
    }
    if(ldonum >= LDO8 || ldonum < LDO1)
    {
        WL2868C_PRINT("[wl2868c] error ldonum not support!!!\n");
        return;
    }

    switch(ldonum)
    {
        case LDO1:
        case LDO2:
            if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2868C) {//WL2868C
                if (value < 496) {
                    WL2868C_PRINT("[WL2868C] error vol!!!\n");
                    goto exit;
                } else {
                    Ldo_out = (value - 496)/8;
                }
            } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_FAN53870) {//FAN53870
                if (value < 800) {
                    WL2868C_PRINT("[FAN53870] error vol!!!\n");
                    goto exit;
                } else {
                    Ldo_out = (value - 800)/8 + 99;
                }
            } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5907) {//ET5907
                if (value < 600) {
                    WL2868C_PRINT("[ET5907] error vol!!!\n");
                    goto exit;
                } else {
                    Ldo_out = (value - 600)/6;
                }
            }
            break;
        case LDO3:
        case LDO4:
        case LDO5:
        case LDO6:
        case LDO7:
            if(which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2868C) {//WL2868C
                if(value < 1504)
                {
                    WL2868C_PRINT("[wl2868c] error vol!!!\n");
                    goto exit;
                }
                else
                {
                    Ldo_out = (value - 1504)/8;
                }
            } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_FAN53870) {//FAN53870
                if(value < 1500)
                {
                    WL2868C_PRINT("[wl2868c-FAN53870] error vol!!!\n");
                    goto exit;
                }
                else
                {
                    Ldo_out = (value - 1500)/8 + 16;
                }
            }else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5907) {//ET5907
                if(value < 1200)
                {
                    WL2868C_PRINT("[wl2868c-ET5907] error vol!!!\n");
                    goto exit;
                }
                else
                {
                    Ldo_out = (value - 1200)/10;
                }
            }
            break;
        default:
            goto exit;
        break;
    }

    if(which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2868C) {
        WL2868C_PRINT("[wl2868c] CAMERA_LDO_WL2868C");
        regaddr = ldonum + WL2868C_LDO1_OUT_ADDR;
    } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5907) {
        WL2868C_PRINT("[wl2868c] CAMERA_LDO_ET5907");
        regaddr = ldonum + LDO1_OUT_ADDR;
    } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_FAN53870) {
        WL2868C_PRINT("[wl2868c] CAMERA_LDO_FAN53870");
        regaddr = ldonum + LDO1_OUT_ADDR;
    }
    ret = wl2868c_write(regaddr, Ldo_out);
    if(ret < 0){
        WL2868C_PRINT("[wl2868c] %s write error.\n", __FUNCTION__);
        return;
    }
    WL2868C_PRINT("[wl2868c] %s write regaddr = 0x%x, ldo_out = %d\n", __FUNCTION__, regaddr, Ldo_out);
    ret = wl2868c_read(regaddr);
    WL2868C_PRINT("[wl2868c] %s read validate ret = %d\n", __FUNCTION__, ret);
    return;

exit:
    WL2868C_PRINT("[wl2868c] %s exit err\n",__FUNCTION__);

}

/*****************************************************************************
 * Extern Area
 *****************************************************************************/
int wl2868c_set_ldo_enable(enum WL2868C_SELECT ldonum, uint32_t voltage)
{
    unsigned int ldo_vol_value = 0;

    mutex_lock(&i2c_control_mutex);

    if(ldonum < LDO1 || ldonum > LDO8) {
        WL2868C_PRINT("[wl2868c] %s error ldonum %d not support!!!\n",__FUNCTION__, ldonum);
        return -2;
    }

    ldo_vol_value = voltage;
    power_reference_counts[ldonum] += 1;
    if(power_reference_counts[ldonum] > 1){
        WL2868C_PRINT("[wl2868c]: LDO_%d poweron already!\n", ldonum + 1);
    } else {
        if(ldonum == LDO8) wl2836d18_set_en_ldo(WL2836D18_GPIO_STATE_RST1); //this is for camera
        else {
            wl2868c_set_ext_buck_en(ldonum, 1);
            wl2868c_set_ldo_value(ldonum, ldo_vol_value);
            wl2868c_set_en_ldo(ldonum, 1);
        }
        WL2868C_PRINT("[wl2868c]: LDO_%d %dmV poweron.\n", ldonum + 1, ldo_vol_value);
    }
    WL2868C_PRINT("[wl2868c] power counts array:[%d, %d, %d, %d, %d, %d, %d, %d]\n",
                                                power_reference_counts[0],power_reference_counts[1],
                                                power_reference_counts[2],power_reference_counts[3],
                                                power_reference_counts[4],power_reference_counts[5],
                                                power_reference_counts[6],power_reference_counts[7]);
    mutex_unlock(&i2c_control_mutex);
    return 0;
}
EXPORT_SYMBOL(wl2868c_set_ldo_enable);

int wl2868c_set_ldo_disable(enum WL2868C_SELECT ldonum)
{
    if(ldonum < LDO1 || ldonum > LDO8) {
        WL2868C_PRINT("[wl2868c] %s ldo %d setting not found in ldolist!!!\n",__FUNCTION__, ldonum);
        return -2;
    }

    mutex_lock(&i2c_control_mutex);

    power_reference_counts[ldonum] -= 1;
    if(power_reference_counts[ldonum] == 0){
        if(ldonum == LDO8) wl2836d18_set_en_ldo(WL2836D18_GPIO_STATE_RST0); //this is for camera
        else {
            wl2868c_set_en_ldo(ldonum, 0);
            wl2868c_set_ext_buck_en(ldonum, 0);
        }
        WL2868C_PRINT("[wl2868c]: LDO_%d poweroff.\n", ldonum + 1);
    }else{
        WL2868C_PRINT("[wl2868c]: LDO_%d will not poweroff, still in use!\n", ldonum + 1);
    }
    WL2868C_PRINT("[wl2868c] power counts array:[%d, %d, %d, %d, %d, %d, %d, %d]\n",
                                                power_reference_counts[0],power_reference_counts[1],
                                                power_reference_counts[2],power_reference_counts[3],
                                                power_reference_counts[4],power_reference_counts[5],
                                                power_reference_counts[6],power_reference_counts[7]);
    mutex_unlock(&i2c_control_mutex);
    return 0;
}
EXPORT_SYMBOL(wl2868c_set_ldo_disable);

#if (defined CONFIG_REGULATOR_OPLUS_WL2868C_FP_LDO) || (defined CONFIG_FP_SUPPLY_MODE_LDO)
int fingerprint_ldo_enable(unsigned int ldo_num, unsigned int mv)
{
    int ret = 0;
    switch(ldo_num) {
        case 1:
            ret = wl2868c_set_ldo_enable(LDO1, mv);
            break;
        case 2:
            ret = wl2868c_set_ldo_enable(LDO2, mv);
            break;
        case 3:
            ret = wl2868c_set_ldo_enable(LDO3, mv);
            break;
        case 4:
            ret = wl2868c_set_ldo_enable(LDO4, mv);
            break;
        case 5:
            ret = wl2868c_set_ldo_enable(LDO5, mv);
            break;
        case 6:
            ret = wl2868c_set_ldo_enable(LDO6, mv);
            break;
        case 7:
            ret = wl2868c_set_ldo_enable(LDO7, mv);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    WL2868C_PRINT("[wl2868c] %s ,ret = %d\n",__FUNCTION__, ret);
    return ret;
}
EXPORT_SYMBOL(fingerprint_ldo_enable);

int fingerprint_ldo_disable(unsigned int ldo_num, unsigned int mv)
{
    int ret = 0;
    switch(ldo_num) {
        case 1:
            ret = wl2868c_set_ldo_disable(LDO1);
            break;
        case 2:
            ret = wl2868c_set_ldo_disable(LDO2);
            break;
        case 3:
            ret = wl2868c_set_ldo_disable(LDO3);
            break;
        case 4:
            ret = wl2868c_set_ldo_disable(LDO4);
            break;
        case 5:
            ret = wl2868c_set_ldo_disable(LDO5);
            break;
        case 6:
            ret = wl2868c_set_ldo_disable(LDO6);
            break;
        case 7:
            ret = wl2868c_set_ldo_disable(LDO7);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    WL2868C_PRINT("[wl2868c] %s ,ret = %d\n",__FUNCTION__, ret);
    return ret;
}
EXPORT_SYMBOL(fingerprint_ldo_disable);
#endif /* CONFIG_REGULATOR_OPLUS_WL2868C_FP_LDO */

/*****************************************************************************
 * Driver Structure
 *****************************************************************************/
static struct platform_driver wl2868c_platform_driver = {
    .probe = wl2868c_gpio_probe,
    .remove = wl2868c_gpio_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "WL2868C_GPIO",
        .of_match_table = gpio_of_match,
    },
};

static struct i2c_driver wl2868c_i2c_driver = {
/************************************************************
Attention:
Althouh i2c_bus do not use .id_table to match, but it must be defined,
otherwise the probe function will not be executed!
************************************************************/
    .id_table = wl2868c_i2c_id,
    .probe = wl2868c_i2c_probe,
    .remove = wl2868c_i2c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "WL2868C_I2C",
        .of_match_table = i2c_of_match,
    },
};

/*****************************************************************************
 * Driver Member Function
 *****************************************************************************/
static long wl2868c_gpio_init(struct platform_device *pdev)
{
    int ret = 0;
    struct pinctrl *pctrl;

    /* retrieve */
    pctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pctrl)) {
        dev_err(&pdev->dev, "Cannot find disp pinctrl!");
        ret = PTR_ERR(pctrl);
        goto exit;
    }

    wl2868c_pctrl = pctrl;

exit:
    return ret;
}

static int wl2868c_gpio_probe(struct platform_device *pdev)
{
    int ret = 0;

    ret = wl2868c_gpio_init(pdev);
    if (ret) {
        WL2868C_PRINT("[wl2868c]wl2868c_gpio_probe failed\n");
        return ret;
    }
    WL2868C_PRINT("[wl2868c] wl2868c_gpio_probe success\n");

    return 0;
}

static int wl2868c_gpio_remove(struct platform_device *pdev)
{
    platform_driver_unregister(&wl2868c_platform_driver);

    return 0;
}

static int wl2868c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i = 0;
    int chipid = 0;

    if (NULL == client) {
        WL2868C_PRINT("[wl2868c] i2c_client is NULL\n");
        return -1;
    }

    if (is_project(22281) || is_project(22282) || is_project(22283)) {
        power_reference_counts[2] = 0;
        WL2868C_PRINT("[wl2868c]match project changzheng\n");
    }

    wl2868c_gpio_select_state(WL2868C_GPIO_STATE_RST1);
    //wl2868c_gpio_select_state(WL2868C_GPIO_EXT_BUCK_EN1);
    msleep(1);
    for(i = 0; i < (sizeof(which_ldo_chip) / sizeof(which_ldo_chip[0])); i++) {
        client->addr = which_ldo_chip[i].i2c_addr;
        wl2868c_i2c_client = client;
        chipid = wl2868c_read(which_ldo_chip[i].chip_addr) & 0xff;
        WL2868C_PRINT("[wl2868c]camera_ldo_i2c_probe addr = 0x%x,chipid = 0x%x\n", client->addr, chipid);

        if (chipid == which_ldo_chip[i].ldoId) {
             ldo_id = i;
             WL2868C_PRINT("[wl2868c]camera_ldo_i2c_probe, this is %d\n", i);
             break;
        }
    }
    if(i == (sizeof(which_ldo_chip) / sizeof(which_ldo_chip[0]))){
        WL2868C_PRINT("[wl2868c]wl2868c_i2c_probe failed, because of i2c problems, now try to pull ldo_rst down.");
        wl2868c_gpio_select_state(WL2868C_GPIO_STATE_RST0);
    } else WL2868C_PRINT("[wl2868c]wl2868c_i2c_probe success addr = 0x%x\n", client->addr);

    mutex_init(&i2c_control_mutex);
    return 0;
}

static int wl2868c_i2c_remove(struct i2c_client *client)
{
    wl2868c_i2c_client = NULL;
    i2c_unregister_device(client);

    return 0;
}

/*****************************************************************************
 * wl286c_char_dev
 *****************************************************************************/
int wl2868c_init_module(void)
{
    // char driver init
    if (platform_driver_register(&wl2868c_platform_driver)) {
        WL2868C_PRINT("[wl2868c]Failed to register wl2868c_platform_driver!\n");
        i2c_del_driver(&wl2868c_i2c_driver);
        return -1;
    }
    WL2868C_PRINT("begin wl2868c initialization");
    if (i2c_add_driver(&wl2868c_i2c_driver)) {
        WL2868C_PRINT("[wl2868c]Failed to register wl2868c_i2c_driver!\n");
        return -1;
    }

    return 0;
}

void wl2868c_exit_module(void)
{
    platform_driver_unregister(&wl2868c_platform_driver);
    i2c_del_driver(&wl2868c_i2c_driver);
}

module_init(wl2868c_init_module);
module_exit(wl2868c_exit_module);

MODULE_AUTHOR("Jack Lee <lixiaolong6@huaqin.com>");
MODULE_DESCRIPTION("CAMERA LDO Driver");
MODULE_LICENSE("GPL");