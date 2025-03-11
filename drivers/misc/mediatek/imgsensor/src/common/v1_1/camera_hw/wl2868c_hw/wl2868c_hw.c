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

Here is WL2868C Interface,
WL2868C Driver is in vnd\kernel-5.10\drivers\misc\mediatek\wl2868c

*/

#include "wl2868c_hw.h"
#include <soc/oplus/system/oplus_project.h>
extern int wl2868c_set_ldo_enable(enum WL2868C_SELECT, uint32_t);
extern int wl2868c_set_ldo_disable(enum WL2868C_SELECT);

/*****************************************************************************
 * Static Var
 *****************************************************************************/
static const uint32_t pin_state_vol_table[] = {
    EXTLDO_REGULATOR_VOLTAGE_0,
    EXTLDO_REGULATOR_VOLTAGE_1000,
    EXTLDO_REGULATOR_VOLTAGE_1050,
    EXTLDO_REGULATOR_VOLTAGE_1100,
    EXTLDO_REGULATOR_VOLTAGE_1150,
    EXTLDO_REGULATOR_VOLTAGE_1200,
    EXTLDO_REGULATOR_VOLTAGE_1210,
    EXTLDO_REGULATOR_VOLTAGE_1220,
    EXTLDO_REGULATOR_VOLTAGE_1500,
    EXTLDO_REGULATOR_VOLTAGE_1800,
    EXTLDO_REGULATOR_VOLTAGE_2200,
    EXTLDO_REGULATOR_VOLTAGE_2500,
    EXTLDO_REGULATOR_VOLTAGE_2800,
    EXTLDO_REGULATOR_VOLTAGE_2900,
    EXTLDO_REGULATOR_VOLTAGE_HIGH
};
static struct wl2868c_ldomap ldolist[] = {
    //main 50M
    {IMGSENSOR_SENSOR_IDX_MAIN, AVDD, LDO6}, //BackMain AVDD
    {IMGSENSOR_SENSOR_IDX_MAIN, DVDD, LDO2}, //BackMain DVDD
    {IMGSENSOR_SENSOR_IDX_MAIN, DOVDD, LDO8}, //BackMain IOVDD, Modified in Doki, is placed by WL2836D18
    {IMGSENSOR_SENSOR_IDX_MAIN, AFVDD, LDO7}, //BackMain AFVDD

    //front 16M
    {IMGSENSOR_SENSOR_IDX_SUB, AVDD, LDO5}, //FrontMain AVDD
    {IMGSENSOR_SENSOR_IDX_SUB, DVDD, LDO1}, //FrontMain DVDD
    {IMGSENSOR_SENSOR_IDX_SUB, DOVDD, LDO8}, //FrontMain IOVDD, Modified in Doki, is placed by WL2836D18

    //mono 2M
    {IMGSENSOR_SENSOR_IDX_MAIN2, AVDD, LDO5}, //BackMono AVDD
    {IMGSENSOR_SENSOR_IDX_MAIN2, DOVDD, LDO8}, //BackMono IOVDD, Modified in Doki, is placed by WL2836D18
};
static struct wl2868c_ldomap ldolist_changzheng[] = {
    //main 50M
    {IMGSENSOR_SENSOR_IDX_MAIN, AVDD, LDO6}, //BackMain AVDD
    {IMGSENSOR_SENSOR_IDX_MAIN, DVDD, LDO2}, //BackMain DVDD
    {IMGSENSOR_SENSOR_IDX_MAIN, DOVDD, LDO3}, //BackMain DOVDD
    {IMGSENSOR_SENSOR_IDX_MAIN, AFVDD, LDO7}, //BackMain AFVDD

    //front 16M
    {IMGSENSOR_SENSOR_IDX_SUB, AVDD, LDO4}, //FrontMain AVDD
    {IMGSENSOR_SENSOR_IDX_SUB, DVDD, LDO1}, //FrontMain DVDD
    {IMGSENSOR_SENSOR_IDX_SUB, DOVDD, LDO3}, //FrontMain DOVDD

    //macro 2M
    {IMGSENSOR_SENSOR_IDX_MAIN2, AVDD, LDO4}, //BackMono AVDD
    {IMGSENSOR_SENSOR_IDX_MAIN2, DOVDD, LDO3}, //BackMono DOVDD

    //mono 2M
    {IMGSENSOR_SENSOR_IDX_SUB2, AVDD, LDO4}, //BackMacro AVDD
    {IMGSENSOR_SENSOR_IDX_SUB2, DOVDD, LDO3}, //BackMacro DOVDD
};

static char* camera_power_pin_name[IMGSENSOR_HW_PIN_MAX_NUM];
/*****************************************************************************
 * Static Fun
 *****************************************************************************/

static enum IMGSENSOR_RETURN wl2868c_hw_init(void *pinstance, struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
    WL2868C_PRINT("[wl2868c_hw] %s in.\n", __FUNCTION__);
    return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN wl2868c_hw_set(
    void *pinstance,
    enum IMGSENSOR_SENSOR_IDX   sensor_idx,
    enum IMGSENSOR_HW_PIN       pin,
    enum IMGSENSOR_HW_PIN_STATE pin_state)
{
    int ret = IMGSENSOR_RETURN_SUCCESS;
    enum WL2868C_SELECT ldonum = LDO_NONE;
    unsigned int i = 0;

    // var init
    camera_power_pin_name[AVDD] = "AVDD";
    camera_power_pin_name[DVDD] = "DVDD";
    camera_power_pin_name[DOVDD] = "DOVDD";
    camera_power_pin_name[AFVDD] = "AFVDD";

    WL2868C_PRINT("[wl2868c_hw] %s stoneadd pin=%s sensor_idx=%d pin_state=%d\n", __FUNCTION__, camera_power_pin_name[pin], sensor_idx, pin_state);

    if (sensor_idx < IMGSENSOR_SENSOR_IDX_MIN_NUM ||
            sensor_idx >= IMGSENSOR_SENSOR_IDX_MAX_NUM){
        WL2868C_PRINT("[wl2868c_hw] error sensor_idx:%d (%d ~ d%)", sensor_idx, IMGSENSOR_SENSOR_IDX_MIN_NUM,IMGSENSOR_SENSOR_IDX_MAX_NUM);
        return IMGSENSOR_RETURN_ERROR;
    } else if (pin < IMGSENSOR_HW_PIN_AVDD ||
            pin > IMGSENSOR_HW_PIN_DOVDD) {
        WL2868C_PRINT("[wl2868c_hw] error pin:%d (%d ~ d%)", pin, IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_PIN_DOVDD);
        return IMGSENSOR_RETURN_ERROR;
    } else if ( pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
            pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH) {
        WL2868C_PRINT("[wl2868c_hw] error pin_state:%d (%d ~ d%)", pin_state, IMGSENSOR_HW_PIN_STATE_LEVEL_0,IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH);
        return IMGSENSOR_RETURN_ERROR;
    }

    if( is_project(22281) || is_project(22282) || is_project(22283)){
        for(i = 0; i < (sizeof(ldolist_changzheng) / sizeof(ldolist_changzheng[0])); i++) {
            if(sensor_idx == ldolist_changzheng[i].sensor_index && pin == ldolist_changzheng[i].seq_type) {
                ldonum = ldolist_changzheng[i].ldo_selected;
                WL2868C_PRINT("[wl2868c_hw] %s sensor %d, seq_type = %d matched ldo %d\n", __FUNCTION__, sensor_idx, pin, ldonum + 1);
                break;
            }
        }
    }else{
        for(i = 0; i < (sizeof(ldolist) / sizeof(ldolist[0])); i++) {
            if(sensor_idx == ldolist[i].sensor_index && pin == ldolist[i].seq_type) {
                ldonum = ldolist[i].ldo_selected;
                WL2868C_PRINT("[wl2868c_hw] %s sensor %d, seq_type = %d matched ldo %d\n", __FUNCTION__, sensor_idx, pin, ldonum + 1);
                break;
            }
        }
    }

    if((ldonum < LDO1) || (ldonum > LDO8)) {
        WL2868C_PRINT("[wl2868c_hw] %s ldo setting not found in ldolist!!!\n",__FUNCTION__);
        return IMGSENSOR_RETURN_ERROR;
    }

    if(pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
        wl2868c_set_ldo_enable(ldonum, pin_state_vol_table[pin_state]);
        WL2868C_PRINT("[wl2868c_hw] cameraid%d poweron %s, LDO%d\n", sensor_idx,
                camera_power_pin_name[pin], ldonum + 1);
    } else {
        wl2868c_set_ldo_disable(ldonum);
        WL2868C_PRINT("[wl2868c] cameraid%d poweroff %s, LDO%d.\n", sensor_idx,
                camera_power_pin_name[pin], ldonum + 1);
    }

    return ret;
}

static enum IMGSENSOR_RETURN wl2868c_hw_release(void *instance)
{
    return IMGSENSOR_RETURN_SUCCESS;
}

/*
 * IMGSENSOR_HW_DEVICE obj will be called by imgsensor_hw.c
*/
static struct IMGSENSOR_HW_DEVICE device = {
    .init      = wl2868c_hw_init,
    .set       = wl2868c_hw_set,
    .release   = wl2868c_hw_release,
    .id        = IMGSENSOR_HW_ID_WL2868C
};

enum IMGSENSOR_RETURN imgsensor_hw_wl2868c_open(
    struct IMGSENSOR_HW_DEVICE **pdevice)
{
    *pdevice = &device;
    return IMGSENSOR_RETURN_SUCCESS;
}
