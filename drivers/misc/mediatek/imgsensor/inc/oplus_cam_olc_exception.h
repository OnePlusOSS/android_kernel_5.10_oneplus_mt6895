#ifndef	__OPLUS_CAM_EXCEPTION__
#define __OPLUS_CAM_EXCEPTION__

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#define CAM_RESERVED_ID 0x100
#define CAM_MODULE_ID 0x25

typedef enum {
	EXCEP_CLOCK,
	EXCEP_VOLTAGE,
	EXCEP_GPIO,
	EXCEP_I2C,
	EXCEP_WATCHDOG,
	EXCEP_FLASHLIGHT,
	EXCEP_VCM,

} cam_excep_type;

int cam_olc_raise_exception(int excep_tpye);
#endif
