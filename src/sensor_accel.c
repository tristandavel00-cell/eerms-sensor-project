#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "sensor_accel.h"

LOG_MODULE_REGISTER(sensor_accel, LOG_LEVEL_DBG);

#define ACCEL_NODE DT_ALIAS(accel0)

#define LIS2DW12_WHO_AM_I_REG   0x0f
#define LIS2DW12_WHO_AM_I_VALUE 0x44

#define SENSOR_ACCEL_ODR_OFF         0U
#define SENSOR_ACCEL_ODR_MOTION      12U
#define SENSOR_ACCEL_ODR_MEASUREMENT 100U

#if !DT_NODE_HAS_STATUS(ACCEL_NODE, okay)
#error "accel0 alias is not defined or the accelerometer is disabled"
#endif

static const struct device *const accel_dev = DEVICE_DT_GET(ACCEL_NODE);
static const struct i2c_dt_spec accel_i2c = I2C_DT_SPEC_GET(ACCEL_NODE);

static sensor_accel_motion_callback_t motion_callback;
static bool motion_configured;
static bool motion_armed;
static bool current_profile_valid;
static enum sensor_accel_profile current_profile;

static const struct sensor_trigger motion_trigger = {
	.type = SENSOR_TRIG_MOTION,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

static const char *sensor_accel_profile_name(enum sensor_accel_profile profile)
{
	switch (profile) {
	case SENSOR_ACCEL_PROFILE_OFF:
		return "OFF";

	case SENSOR_ACCEL_PROFILE_MOTION:
		return "MOTION";

	case SENSOR_ACCEL_PROFILE_MEASUREMENT:
		return "MEASUREMENT";

	default:
		return "UNKNOWN";
	}
}

static const char *sensor_accel_profile_rate_name(
	enum sensor_accel_profile profile)
{
	switch (profile) {
	case SENSOR_ACCEL_PROFILE_OFF:
		return "power-down";

	case SENSOR_ACCEL_PROFILE_MOTION:
		return "12.5 Hz";

	case SENSOR_ACCEL_PROFILE_MEASUREMENT:
		return "100 Hz";

	default:
		return "unknown";
	}
}

static void sensor_accel_motion_handler(
	const struct device *dev,
	const struct sensor_trigger *trigger)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trigger);

	if (motion_callback != NULL) {
		motion_callback();
	}
}

int sensor_accel_init(void)
{
	uint8_t who_am_i;
	int ret;

	if (!device_is_ready(accel_dev)) {
		LOG_ERR("LIS2DW12 device is not ready");
		return -ENODEV;
	}

	if (!i2c_is_ready_dt(&accel_i2c)) {
		LOG_ERR("LIS2DW12 I2C bus is not ready");
		return -ENODEV;
	}

	ret = i2c_reg_read_byte_dt(&accel_i2c,
				   LIS2DW12_WHO_AM_I_REG,
				   &who_am_i);
	if (ret < 0) {
		LOG_ERR("Failed to read LIS2DW12 WHO_AM_I: %d", ret);
		return ret;
	}

	if (who_am_i != LIS2DW12_WHO_AM_I_VALUE) {
		LOG_ERR("Unexpected LIS2DW12 WHO_AM_I: 0x%02x", who_am_i);
		return -ENODEV;
	}

	LOG_INF("LIS2DW12 detected: WHO_AM_I=0x%02x", who_am_i);
	return 0;
}

int sensor_accel_read(struct sensor_accel_sample *sample)
{
	struct sensor_value xyz[3];
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	ret = sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_ACCEL_XYZ);
	if (ret < 0) {
		LOG_ERR("Failed to fetch LIS2DW12 sample: %d", ret);
		return ret;
	}

	ret = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, xyz);
	if (ret < 0) {
		LOG_ERR("Failed to get LIS2DW12 XYZ channels: %d", ret);
		return ret;
	}

	sample->x_mg = sensor_ms2_to_mg(&xyz[0]);
	sample->y_mg = sensor_ms2_to_mg(&xyz[1]);
	sample->z_mg = sensor_ms2_to_mg(&xyz[2]);

	return 0;
}

int sensor_accel_profile_set(enum sensor_accel_profile profile)
{
	struct sensor_value odr = {0};
	uint16_t odr_hz;
	int ret;

	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Cannot set accelerometer profile: device is not ready");
		return -ENODEV;
	}

	switch (profile) {
	case SENSOR_ACCEL_PROFILE_OFF:
		odr_hz = SENSOR_ACCEL_ODR_OFF;
		break;

	case SENSOR_ACCEL_PROFILE_MOTION:
		odr_hz = SENSOR_ACCEL_ODR_MOTION;
		break;

	case SENSOR_ACCEL_PROFILE_MEASUREMENT:
		odr_hz = SENSOR_ACCEL_ODR_MEASUREMENT;
		break;

	default:
		LOG_ERR("Invalid accelerometer profile: %d", profile);
		return -EINVAL;
	}

	if (current_profile_valid && (profile == current_profile)) {
		LOG_DBG("Accelerometer profile already set to %s",
			sensor_accel_profile_name(profile));
		return 0;
	}

	odr.val1 = (int32_t)odr_hz;

	ret = sensor_attr_set(accel_dev,
			      SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY,
			      &odr);
	if (ret < 0) {
		LOG_ERR("Failed to set accelerometer profile %s: %d",
			sensor_accel_profile_name(profile), ret);
		return ret;
	}

	current_profile = profile;
	current_profile_valid = true;

	LOG_INF("Accelerometer profile changed to %s (%s)",
		sensor_accel_profile_name(profile),
		sensor_accel_profile_rate_name(profile));
	return 0;
}

int sensor_accel_motion_init(sensor_accel_motion_callback_t callback,
			     uint16_t threshold_mg)
{
	struct sensor_value threshold;
	int ret;

	if ((callback == NULL) || (threshold_mg == 0U)) {
		return -EINVAL;
	}

	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Cannot configure motion detection: LIS2DW12 is not ready");
		return -ENODEV;
	}

	sensor_ug_to_ms2((int32_t)threshold_mg * 1000, &threshold);

	ret = sensor_attr_set(accel_dev,
			      SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_UPPER_THRESH,
			      &threshold);
	if (ret < 0) {
		LOG_ERR("Failed to configure LIS2DW12 motion threshold: %d", ret);
		return ret;
	}

	motion_callback = callback;
	motion_configured = true;
	motion_armed = false;

	LOG_INF("LIS2DW12 motion threshold configured: %u mg", threshold_mg);
	return 0;
}

int sensor_accel_motion_arm(void)
{
	int ret;

	if (!motion_configured) {
		LOG_ERR("Motion detection has not been configured");
		return -EACCES;
	}

	if (motion_armed) {
		return 0;
	}

	ret = sensor_trigger_set(accel_dev,
				 &motion_trigger,
				 sensor_accel_motion_handler);
	if (ret < 0) {
		LOG_ERR("Failed to arm LIS2DW12 motion trigger: %d", ret);
		return ret;
	}

	motion_armed = true;
	LOG_INF("LIS2DW12 motion trigger armed");
	return 0;
}

int sensor_accel_motion_disarm(void)
{
	int ret;

	if (!motion_configured || !motion_armed) {
		return 0;
	}

	ret = sensor_trigger_set(accel_dev, &motion_trigger, NULL);
	if (ret < 0) {
		LOG_ERR("Failed to disarm LIS2DW12 motion trigger: %d", ret);
		return ret;
	}

	motion_armed = false;
	LOG_INF("LIS2DW12 motion trigger disarmed");
	return 0;
}
