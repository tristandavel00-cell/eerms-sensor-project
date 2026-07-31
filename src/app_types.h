#ifndef APP_TYPES_H
#define APP_TYPES_H

/*
 * Application modes.
 *
 * These values are part of the BLE protocol and persistent settings.
 * Do not change an existing numeric value after release.
 */
enum app_mode {
	APP_MODE_NORMAL = 0,
	APP_MODE_CONFIG = 1,
	APP_MODE_DIAGNOSTIC = 2,
};

/*
 * Runtime application states.
 *
 * These values will be transmitted in the BLE system-status packet.
 */
enum app_runtime_state {
	APP_STATE_BOOT = 0,
	APP_STATE_IDLE = 1,
	APP_STATE_MEASURING = 2,
	APP_STATE_REPORTING = 3,
	APP_STATE_ERROR = 4,
};

/*
 * Reason for the most recent measurement or wake operation.
 */
enum app_wake_reason {
	APP_WAKE_REASON_NONE = 0,
	APP_WAKE_REASON_BOOT = 1,
	APP_WAKE_REASON_SHAKE = 2,
	APP_WAKE_REASON_PERIODIC = 3,
	APP_WAKE_REASON_NFC = 4,
	APP_WAKE_REASON_BLE_COMMAND = 5,
};

/*
 * Overall product health.
 *
 * OK:
 *     All required functions are operating.
 *
 * WARNING:
 *     An unusual condition occurred, but monitoring is still complete.
 *
 * DEGRADED:
 *     One subsystem has failed, but partial monitoring can continue.
 *
 * FAULTED:
 *     The application cannot perform useful monitoring.
 */
enum app_health_state {
	APP_HEALTH_OK = 0,
	APP_HEALTH_WARNING = 1,
	APP_HEALTH_DEGRADED = 2,
	APP_HEALTH_FAULTED = 3,
};

/*
 * Stable product error codes.
 *
 * Error-code ranges:
 *
 * 0x01xx - accelerometer
 * 0x02xx - temperature measurement
 * 0x03xx - battery measurement
 * 0x04xx - application and measurement processing
 * 0x05xx - Bluetooth
 * 0x06xx - persistent settings
 * 0x07xx - NFC
 */
enum app_error_code {
	APP_ERROR_NONE = 0x0000,

	APP_ERROR_ACCEL_INIT = 0x0101,
	APP_ERROR_ACCEL_MOTION_INIT = 0x0102,
	APP_ERROR_ACCEL_PROFILE = 0x0103,
	APP_ERROR_ACCEL_MOTION_ARM = 0x0104,
	APP_ERROR_ACCEL_MOTION_DISARM = 0x0105,
	APP_ERROR_ACCEL_SAMPLE_READ = 0x0106,

	APP_ERROR_TEMPERATURE_INIT = 0x0201,
	APP_ERROR_TEMPERATURE_ADC_READ = 0x0202,
	APP_ERROR_TEMPERATURE_OPEN_CIRCUIT = 0x0203,
	APP_ERROR_TEMPERATURE_SHORT_CIRCUIT = 0x0204,

	APP_ERROR_BATTERY_INIT = 0x0301,
	APP_ERROR_BATTERY_ADC_READ = 0x0302,

	APP_ERROR_EVENT_QUEUE_FULL = 0x0401,
	APP_ERROR_MEASUREMENT_PROCESS = 0x0402,
	APP_ERROR_REPORT_WINDOW = 0x0403,

	APP_ERROR_BLE_INIT = 0x0501,
	APP_ERROR_BLE_ADVERTISING = 0x0502,

	APP_ERROR_SETTINGS_LOAD = 0x0601,
	APP_ERROR_SETTINGS_SAVE = 0x0602,

	APP_ERROR_NFC_INIT = 0x0701,
};

/*
 * Active fault bits.
 *
 * More than one bit may be set at the same time. For example, the
 * temperature sensor and NFC subsystem could both be unavailable while
 * accelerometer monitoring continues.
 */
enum app_fault_flag {
	APP_FAULT_NONE = 0x0000,
	APP_FAULT_ACCEL = 0x0001,
	APP_FAULT_TEMPERATURE = 0x0002,
	APP_FAULT_BATTERY = 0x0004,
	APP_FAULT_APPLICATION = 0x0008,
	APP_FAULT_BLE = 0x0010,
	APP_FAULT_SETTINGS = 0x0020,
	APP_FAULT_NFC = 0x0040,
};

#endif