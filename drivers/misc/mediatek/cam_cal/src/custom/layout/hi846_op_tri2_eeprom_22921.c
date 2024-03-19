// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"

#define pr_debug_if(cond, ...)      do { if ((cond)) pr_debug(__VA_ARGS__); } while (0)
#define pr_debug_err(...)    pr_debug("error: " __VA_ARGS__)

static unsigned int do_single_lsc_hi846(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_2a_gain_hi846(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_lens_id_hi846(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);

unsigned char read_otp_8bit_hi846_tri2(struct i2c_client *client, u16 reg)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msg[2];

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		must_log("I2C read data failed!!\n");
	}

	return buf[0];
}

static void write_otp_8bit_hi846_tri2(struct i2c_client *client, u16 reg, u8 val)
{
	int i4RetValue = 0;
	u8 buf[3];
	struct i2c_msg msg;

	buf[0] = (u8)(reg >> 8);
	buf[1] = (u8)(reg & 0xFF);
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.buf = buf;
	msg.len = sizeof(buf);

	i4RetValue = i2c_transfer(client->adapter, &msg, 1);
	if (i4RetValue != 1) {
		must_log("write data failed!!\n");
	}
}

static void write_otp_16bit_hi846_tri2(struct i2c_client *client, u16 reg, u16 val)
{
	int ret;
	u8 buf[4];
	struct i2c_msg msg;

 	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8;
	buf[3] = val & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		must_log("write data failed!!\n");
	}
}

unsigned int Otp_read_region_hi846_tri2(struct i2c_client *client,
	unsigned int addr, unsigned char *data, unsigned int size)
{
	u16 ret = 0;
	u16 OTP_addr = addr & 0xFFFF;
	u32 readsize = size;
	u8 *read_data = data;

	pr_debug("readsize: 0x%x\n", size);
	pr_debug("addr: 0x%04x\n", addr);

	/* 1. sensor init */
	write_otp_16bit_hi846_tri2(client, 0x0A00, 0x0000); //tg_pmem_sckpw/sdly
	write_otp_16bit_hi846_tri2(client, 0x2000, 0x0000); //tg_pmem_sckpw/sdly
	write_otp_16bit_hi846_tri2(client, 0x2002, 0x00FF); //tg_pmem_sckpw/sdly
	write_otp_16bit_hi846_tri2(client, 0x2004, 0x0000); //tg_pmem_rom_dly
	write_otp_16bit_hi846_tri2(client, 0x2008, 0x3FFF); // firmware start address-ROM
	write_otp_16bit_hi846_tri2(client, 0x23FE, 0xC056); // BGR enable
	write_otp_16bit_hi846_tri2(client, 0x0A00, 0x0000); // STRB(OTP Busy) output enable
	write_otp_16bit_hi846_tri2(client, 0x0E04, 0x0012); // STRB(OTP Busy) output drivability
	write_otp_16bit_hi846_tri2(client, 0x0F08, 0x2F04); // Analog PLL setting
	write_otp_16bit_hi846_tri2(client, 0x0F30, 0x001F); // Analog CLKGEN setting
	write_otp_16bit_hi846_tri2(client, 0x0F36, 0x001F); // Analog CLKGEN setting
	write_otp_16bit_hi846_tri2(client, 0x0F04, 0x3A00); // PLL enable
	write_otp_16bit_hi846_tri2(client, 0x0F32, 0x025A); // mipi disable
	write_otp_16bit_hi846_tri2(client, 0x0F38, 0x0256); // TG PMEM CEN anable
	write_otp_16bit_hi846_tri2(client, 0x0F2A, 0x4124); // TG MCU enable
	write_otp_16bit_hi846_tri2(client, 0x006A, 0x0100); // ROM OTP Continuous W/R mode enable
	write_otp_16bit_hi846_tri2(client, 0x004C, 0x0100); // Stream ON

	/* 1. init OTP setting*/
	write_otp_8bit_hi846_tri2(client, 0x0A02, 0x01); //Fast sleep on
	write_otp_8bit_hi846_tri2(client, 0x0A00, 0x00);//stand by on
	mdelay(10);
	write_otp_8bit_hi846_tri2(client, 0x0F02, 0x00);//pll disable
	write_otp_8bit_hi846_tri2(client, 0x071A, 0x01);//CP TRIM_H
	write_otp_8bit_hi846_tri2(client, 0x071B, 0x09);//IPGM TRIM_H
	write_otp_8bit_hi846_tri2(client, 0x0D04, 0x00);//Fsync(OTP busy)Output Enable
	write_otp_8bit_hi846_tri2(client, 0x0D00, 0x07);//Fsync(OTP busy)Output Drivability
	write_otp_8bit_hi846_tri2(client, 0x003E, 0x10);//OTP r/w mode
	write_otp_8bit_hi846_tri2(client, 0x0A00, 0x01);//standby off
	mdelay(1);

	/* 2. read otp data function */
	write_otp_8bit_hi846_tri2(client, 0x070A, (unsigned char)((OTP_addr >> 8) & 0xFF));
	write_otp_8bit_hi846_tri2(client, 0x070B, (unsigned char)(OTP_addr & 0xFF));
	write_otp_8bit_hi846_tri2(client, 0x0702, 0x01);
	for (ret=0; ret < readsize; ret++) {
		//OTP data read
		*(read_data + ret) = read_otp_8bit_hi846_tri2(client, 0x0708);
	}

	/* 3. disable otp function */
	write_otp_8bit_hi846_tri2(client, 0x0a00, 0x00);
	mdelay(10);
	write_otp_8bit_hi846_tri2(client, 0x003e, 0x00);
	write_otp_8bit_hi846_tri2(client, 0x0a00, 0x01);
	mdelay(1);

	pr_debug("OTP read done\n");
	return readsize;
}

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x0000097C, 0x01051755, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000001, 0x0000097D, 0x00000001, do_module_version},
		{0x00000001, 0x0000097D, 0x00000001, do_part_number},
		{0x00000001, 0x000009A9, 0x0000074C, do_single_lsc_hi846},
		{0x00000001, 0x00000007, 0x0000000E, do_2a_gain_hi846},  //Start address, block size is useless
		{0x00000000, 0x00000785, 0x000005E1, do_pdaf},
		{0x00000000, 0x00000000, 0x00000000, do_stereo_data},
		{0x00000001, 0x00000000, 0x00002000, do_dump_all},
		{0x00000001, 0x0000097F, 0x00000001, do_lens_id_hi846}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT hi846_op_tri2_eeprom_22921 = {
	.name = "hi846_op_tri2_eeprom_22921",
	.check_layout_function = layout_check,
	.read_function = Otp_read_region_hi846_tri2,
	.layout = &cal_layout_table,
	.sensor_id = HI846_SENSOR_ID_22921,
	.i2c_write_id = 0x40,
	.max_size = 0x2000,
	.enable_preload = 1,
	.preload_size = 0x2000,
};

static unsigned int do_single_lsc_hi846(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
		(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	unsigned short table_size;

	debug_log("do_single_lsc_hi846 E\n");
#ifdef MTK_LOAD_DEBUG
	dump_enable = 1;
#else
	dump_enable = 0;
#endif

	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		pr_debug_err("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}
	if (block_size != CAM_CAL_SINGLE_LSC_SIZE)
		pr_debug_err("block_size(%d) is not match (%d)\n",
			block_size, CAM_CAL_SINGLE_LSC_SIZE);

	pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType = 2;//mtk type
	pCamCalData->SingleLsc.LscTable.MtkLcsData.PixId = 8;

	table_size = 1868;

	pr_debug("lsc table_size %d\n", table_size);
	pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize = table_size;
	if (table_size > 0) {
		pCamCalData->SingleLsc.TableRotation = 0;
		pr_debug_if(dump_enable, "u4Offset=%d u4Length=%d", start_addr, table_size);
		read_data_size = read_data(pdata,
			pCamCalData->sensorID, pCamCalData->deviceID,
			start_addr, table_size, (unsigned char *)
			&pCamCalData->SingleLsc.LscTable.MtkLcsData.SlimLscType);
		if (table_size == read_data_size)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pr_debug_err("Read Failed\n");
			err = CamCalReturnErr[pCamCalData->Command];
			show_cmd_error_log(pCamCalData->Command);
		}
	}
	//#ifdef DEBUG_CALIBRATION_LOAD
	pr_debug("======================SingleLsc Data==================\n");
	pr_debug("[1st] = %x, %x, %x, %x\n",
		pCamCalData->SingleLsc.LscTable.Data[0],
		pCamCalData->SingleLsc.LscTable.Data[1],
		pCamCalData->SingleLsc.LscTable.Data[2],
		pCamCalData->SingleLsc.LscTable.Data[3]);
	pr_debug("[1st] = SensorLSC(1)?MTKLSC(2)?  %x\n",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType);
	pr_debug("CapIspReg =0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[0],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[1],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[2],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[3],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[4]);
	pr_debug("RETURN = 0x%x\n", err);
	pr_debug("======================SingleLsc Data==================\n");
	//#endif

	return err;
}

static unsigned int do_2a_gain_hi846(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
		(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	long long CalGain, FacGain;
	unsigned char AWBAFConfig = 0xf;

	int tempMax = 0;
	int CalR = 1, CalGr = 1, CalGb = 1, CalG = 1, CalB = 1;
	int FacR = 1, FacGr = 1, FacGb = 1, FacG = 1, FacB = 1;
	unsigned int awb_offset;

	(void) start_addr;
	(void) block_size;

#ifdef MTK_LOAD_DEBUG
	dump_enable = 1;
#else
	dump_enable = 0;
#endif

	pr_debug("block_size=%d sensor_id=%x\n", block_size, pCamCalData->sensorID);
	memset((void *)&pCamCalData->Single2A, 0, sizeof(struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT));
	/* Check rule */
	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		pr_debug_err("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}
	/* Check AWB & AF enable bit */
	pCamCalData->Single2A.S2aVer = 0x01;
	pCamCalData->Single2A.S2aBitEn = (0x03 & AWBAFConfig);
	pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & AWBAFConfig);
	pr_debug_if(dump_enable, "S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);
	/* AWB Calibration Data*/
	if (0x1 & AWBAFConfig) {
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 0x02;
		/* AWB Unit Gain (5100K) */
		pr_debug_if(dump_enable, "5100K AWB\n");
		awb_offset = 0x0998;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			awb_offset, 8, (unsigned char *)&CalGain);
		if (read_data_size > 0)	{
			pr_debug_if(dump_enable, "Read CalGain OK %x\n", read_data_size);
			CalR = CalGain & 0xFFFF;
			CalB = (CalGain >> 16) & 0xFFFF;
			CalGr = (CalGain >> 32) & 0xFFFF;
			CalGb = (CalGain >> 48) & 0xFFFF;
			CalG  = ((CalGr + CalGb) + 1) >> 1;
			if (CalR > CalG)
				/* R > G */
				if (CalR > CalB)
					tempMax = CalR;
				else
					tempMax = CalB;
			else
				/* G > R */
				if (CalG > CalB)
					tempMax = CalG;
				else
					tempMax = CalB;
			pr_debug_if(dump_enable,
					"UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
					CalR, CalG, CalB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			pr_debug_err("Read CalGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (CalGain != 0x0000000000000000 &&
			CalGain != 0xFFFFFFFFFFFFFFFF &&
			CalR    != 0x00000000 &&
			CalG    != 0x00000000 &&
			CalB    != 0x00000000) {
			pCamCalData->Single2A.S2aAwb.rGainSetNum = 1;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R =
					(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G =
					(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B =
					(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
		} else {
			pr_debug("There are something wrong on EEPROM, plz contact module vendor!!\n");
			pr_debug("Unit R=%d G=%d B=%d!!\n", CalR, CalG, CalB);
		}
		/* AWB Golden Gain (5100K) */
		awb_offset = 0x09A0;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&FacGain);
		if (read_data_size > 0)	{
			pr_debug_if(dump_enable, "Read FacGain OK\n");
			FacR  = FacGain & 0xFFFF;
			FacB  = (FacGain >> 16) & 0xFFFF;
			FacGr = (FacGain >> 32) & 0xFFFF;
			FacGb = (FacGain >> 48) & 0xFFFF;
			FacG  = ((FacGr + FacGb) + 1) >> 1;
			if (FacR > FacG)
				if (FacR > FacB)
					tempMax = FacR;
				else
					tempMax = FacB;
			else
				if (FacG > FacB)
					tempMax = FacG;
				else
					tempMax = FacB;
			pr_debug_if(dump_enable,
				"GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
				FacR, FacG, FacB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			pr_debug_err("Read FacGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (FacGain != 0x0000000000000000 &&
			FacGain != 0xFFFFFFFFFFFFFFFF &&
			FacR    != 0x00000000 &&
			FacG    != 0x00000000 &&
			FacB    != 0x00000000)	{
			pCamCalData->Single2A.S2aAwb.rGoldGainu4R =
				(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G =
				(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B =
					(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
		} else {
			pr_debug("There are something wrong on EEPROM, plz contact module vendor!!");
			pr_debug("Golden R=%d G=%d B=%d\n", FacR, FacG, FacB);
		}
		/* Set AWB to 3A Layer */
		pCamCalData->Single2A.S2aAwb.rValueR   = CalR;
		pCamCalData->Single2A.S2aAwb.rValueGr  = CalGr;
		pCamCalData->Single2A.S2aAwb.rValueGb  = CalGb;
		pCamCalData->Single2A.S2aAwb.rValueB   = CalB;
		pCamCalData->Single2A.S2aAwb.rGoldenR  = FacR;
		pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr;
		pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb;
		pCamCalData->Single2A.S2aAwb.rGoldenB  = FacB;
		//#ifdef DEBUG_CALIBRATION_LOAD
		pr_debug("======================AWB CAM_CAL==================\n");
		pr_debug("AWB Calibration @5100K\n");
		pr_debug("[CalGain] = 0x%x\n", CalGain);
		pr_debug("[FacGain] = 0x%x\n", FacGain);
		pr_debug("[rCalGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
		pr_debug("[rCalGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
		pr_debug("[rCalGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
		pr_debug("[rFacGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
		pr_debug("[rFacGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
		pr_debug("[rFacGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
		//#endif
	}
	return err;
}

static unsigned int do_lens_id_hi846(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	pr_debug("do_lens_id_hi846 E\n");
	return do_lens_id_base(pdata, start_addr, block_size, pGetSensorCalData);
}
