#ifndef _IMGSENSOR_WL2868C_h_
#define _IMGSENSOR_WL2868C_h_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/init.h>

//ldo i2c 8-bit write addr to 7-bit salve addr
#define WL2868C_LDO_I2C_ADDR     0x2f
#define ET5907_LDO_I2C_ADDR      0x6a >> 1
#define FAN53870_LDO_I2C_ADDR    0x6a >> 1

//chip id addr
#define WL2868C_CHIP_REV_ADDR    0x00
#define ET5907_CHIP_REV_ADDR     0x01
#define FAN53870_CHIP_REV_ADDR   0x01

//chip id
#define CAMERA_LDO_WL2868C  0x82
#define CAMERA_LDO_ET5907   0x00
#define CAMERA_LDO_FAN53870 0x01

//enable addr
#define WL2868C_LDO_EN_ADDR          0x0E  //bit0:LDO1 ~ bit6:LDO7
#define ET5907_LDO_EN_ADDR           0x03  //bit0:LDO1 ~ bit6:LDO7
#define FAN53870_LDO_EN_ADDR         0x03  //bit0:LDO1 ~ bit6:LDO7

//ldo 1 addr, if addr(ldo 1) = 0x03, then addr(ldo 2) = 0x04, addr(ldo 3) = 0x05...
#define WL2868C_LDO1_OUT_ADDR    0x03
#define LDO1_OUT_ADDR    0x04

//printk
#define WL2868C_PRINT pr_info

enum WL2868C_SELECT{
	LDO_NONE=-1,
	LDO1,
	LDO2,
	LDO3,
	LDO4,
	LDO5,
	LDO6,
	LDO7,
	LDO8, // this is for camera IOVDD
	LDO_MAX
};

/* GPIO state */
enum WL2868C_GPIO_STATE{
	WL2868C_GPIO_STATE_RST0, //rst low
	WL2868C_GPIO_STATE_RST1, //rst high
	WL2868C_GPIO_EXT_BUCK_EN0, //buck en low
	WL2868C_GPIO_EXT_BUCK_EN1, //buck en high
	WL2836D18_GPIO_STATE_RST0, //en low
	WL2836D18_GPIO_STATE_RST1, //en high
	WL2868C_GPIO_STATE_MAX /* for array size */
};

struct i2c_device{
    unsigned short i2c_addr;
    unsigned short chip_addr;
    unsigned short ldoId;
    unsigned short enable_addr;
};


#if (defined CONFIG_REGULATOR_OPLUS_WL2868C_FP_LDO) || (defined CONFIG_FP_SUPPLY_MODE_LDO)
int fingerprint_ldo_enable(unsigned int ldo_num, unsigned int mv);
int fingerprint_ldo_disable(unsigned int ldo_num, unsigned int mv);
#endif /* CONFIG_REGULATOR_OPLUS_WL2868C_FP_LDO */

#endif