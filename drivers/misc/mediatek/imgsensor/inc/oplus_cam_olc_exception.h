#ifndef	__OPLUS_CAM_EXCEPTION__
#define __OPLUS_CAM_EXCEPTION__

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#define CAM_RESERVED_ID 	0x100
#define CAM_MODULE_ID 		0x25
#define PAYLOAD_LENGTH		1024
typedef enum {
	EXCEP_CLOCK,
	EXCEP_VOLTAGE,
	EXCEP_GPIO,
	EXCEP_I2C,
	EXCEP_SOF_TIMEOUT,
	EXCEP_PROBE,
} cam_excep_type;

struct cam_fb_conf {
	int excepId;
	unsigned char* fb_field;
	unsigned char* fb_event_id;
	long long	record_time;
};

int cam_olc_raise_exception(int excep_tpye, unsigned char* pay_load);
const unsigned char* acquireEventField(int excepId);

#endif
