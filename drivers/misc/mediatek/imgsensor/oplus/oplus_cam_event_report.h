// SPDX-License-Identifier: GPL-2.0
#ifndef __OPLUS_CAM_EVENT_REPORT__
#define __OPLUS_CAM_EVENT_REPORT__

void cam_event_report(int id, int event);
void push_sensor_name(const char* sensorName);

#define CAM_HARDWARE_EVENT_BASE 0x40010000
#define SENSOR_POWER_FAIL 1 << 0 //0x40010001
#define SENSOR_I2C_TRANSF_FAIL 1 << 1 //0x40010002
//#define FLASH_HARDWARE_ERROR 1 << 2 //0x40010004
//#define MOTOR_HARDWARE_ERROR 1 << 3 //0x40010008
//#define OIS_HARDWARE_ERROR 1 << 4 //0x40010010
#define MAX_ID			19
#define MAX_BUF_LEN		2048


#endif
