/*****************************************************************************
 *
 * Filename:
 * ---------
 *     gc32e2mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 * Driver Version:
 * ------------
 *     V0.6040.204
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc32e2mipi_otp.h"

/****************************Modify Following Strings for Debug**************************/
#define PFX "GC32E2MIPI_OTP"
/****************************   Modify end    *******************************************/
#define GC32E2_OTP_DEBUG 1
#if GC32E2_OTP_DEBUG
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

/*for debug*/
#define CT_DEBUG            1


static kal_uint16 read_eeprom(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xff) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, EEPROM_WRITE_ID);
	return get_byte;
}

static kal_uint8 ct_read_data[CROSSTALK_BUF_SIZE];

void gc32e2_read_crosstalk_data(void)
{
	kal_uint16 i = 0;
	kal_uint16 ct_flag = 0;
	kal_uint32 checksum_crosstalk = 0;
	kal_uint32 checksum_pd_crosstalk = 0;
    LOG_INF("start hyper crosstalk");
	memset(&ct_read_data, 0, CROSSTALK_BUF_SIZE * sizeof(kal_uint8));

	ct_flag = read_eeprom(CROSSTALK_FLAG_OFFSET);
	LOG_INF("ct_flag = %d\n", ct_flag);
//	ct_flag = 1;
	if (ct_flag == OTP_FLAG_VALID) {
		for (i = 0; i < CROSSTALK_BUF_SIZE; i++)
			ct_read_data[i] = read_eeprom(CROSSTALK_START_ADDR + i);

        //check_crosstalk
		for (i = 2; i < 786; i++)
			checksum_crosstalk += ct_read_data[i];

		checksum_crosstalk++;
/*
		if ((checksum_crosstalk % 255) == (checksum_crosstalk % 255))
			LOG_INF("check success! calc_checksum_crosstalk = %d, read_checksum_crosstalk = %d\n",
			(checksum_crosstalk % 255), ct_read_data[1374]);
		else {
			//memset(ct_read_data, 0, 787);
			LOG_INF("check error! calc_checksum_crosstalk = %d, read_checksum_crosstalk = %d\n",
			(checksum_crosstalk % 255), ct_read_data[786]);
		}
*/
		//check_pd_crosstalk
		for (i = 786; i < 1374; i++)
			checksum_pd_crosstalk += ct_read_data[i];

		checksum_pd_crosstalk++;
/*
		if ((checksum_pd_crosstalk % 255) == (checksum_pd_crosstalk % 255))
			LOG_INF("check success! calc_checksum_pd_crosstalk = %d, read_checksum_pd_crosstalk = %d\n",
			(checksum_pd_crosstalk % 255), ct_read_data[1375]);
		else {
			LOG_INF("check error! calc_checksum_pd_crosstalk = %d, read_checksum_pd_crosstalk = %d\n",
			(checksum_pd_crosstalk % 255), ct_read_data[1375]);
			memset(ct_read_data, 0, 1376);
		}
*/
#if CT_DEBUG
		for (i = 0; i < CROSSTALK_BUF_SIZE; i++)
			LOG_INF("ct_data[%d] = %x\n", i, ct_read_data[i]);
#endif
	} else
		LOG_INF("crosstalk data is zero!\n");
}

void gc32e2_copy_crosstalk_data(kal_uint8 *ct_data)
{
	LOG_INF("start copy crosstalk_data!\n");
	memcpy(ct_data, (void *)&ct_read_data[0], GC_SXTC_DATA_SIZE);
	memcpy(ct_data + GC_SXTC_DATA_SIZE, (void *)&ct_read_data[GC_SXTC_DATA_SIZE + FLAG_AND_CHECK_SUM_OF_SXTC_SIZE + SUNNY_RESERVED_SIZE], GC_PDXTC_DATA_SIZE);
}



void gc32e2_otp_identify(void)
{
	LOG_INF("start hyper otp_identify");
	gc32e2_read_crosstalk_data();

}
