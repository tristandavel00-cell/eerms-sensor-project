#ifndef SENSOR_ACCEL_H
#define SENSOR_ACCEL_H

#include <stdint.h>

struct sensor_accel_sample {
	int32_t x_mg;
	int32_t y_mg;
	int32_t z_mg;
};

enum sensor_accel_profile {
	SENSOR_ACCEL_PROFILE_OFF = 0,
	SENSOR_ACCEL_PROFILE_MOTION,
	SENSOR_ACCEL_PROFILE_MEASUREMENT,
};

typedef void (*sensor_accel_motion_callback_t)(void);

int sensor_accel_init(void);
int sensor_accel_read(struct sensor_accel_sample *sample);
int sensor_accel_profile_set(enum sensor_accel_profile profile);

int sensor_accel_motion_init(sensor_accel_motion_callback_t callback,
			     uint16_t threshold_mg);
int sensor_accel_motion_arm(void);
int sensor_accel_motion_disarm(void);

#endif
