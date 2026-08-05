#ifndef APP_BLE_H
#define APP_BLE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"

enum app_ble_command {
	APP_BLE_COMMAND_STATUS_REPORT = 0x02,
};

/*
 * Validity bits for the versioned main measurement packet.
 */
enum app_ble_measurement_valid_flag {
	APP_BLE_MEASUREMENT_VALID_VIBRATION = 0x01,
	APP_BLE_MEASUREMENT_VALID_BATTERY = 0x02,
	APP_BLE_MEASUREMENT_VALID_TEMPERATURE = 0x04,
};

/*
 * Values used to build the 20-byte main measurement packet.
 *
 * The BLE implementation performs the actual byte packing so this
 * structure does not need to have a packed memory layout.
 */
struct app_ble_measurement {
	uint8_t valid_flags;

	enum app_battery_state battery_state;
	enum app_health_state health;

	uint16_t battery_mv;
	int16_t temperature_centi_c;

	int16_t mean_x_mg;
	int16_t mean_y_mg;
	int16_t mean_z_mg;

	uint16_t rms_mg;
	uint16_t peak_mg;
	uint16_t sequence;
};

struct app_ble_callbacks {
	void (*mode_requested)(enum app_mode mode);
	void (*command_received)(enum app_ble_command command);

	enum app_mode (*mode_get)(void);
	enum app_runtime_state (*runtime_state_get)(void);
	enum app_wake_reason (*wake_reason_get)(void);
	enum app_health_state (*health_get)(void);

	uint16_t (*error_code_get)(void);
	uint16_t (*fault_flags_get)(void);
	uint16_t (*fault_occurrence_count_get)(void);

	uint32_t (*state_transition_count_get)(void);
	uint32_t (*uptime_seconds_get)(void);
};

int app_ble_start(
	const struct app_ble_callbacks *callbacks);

int app_ble_notify_mode(
	enum app_mode mode);

int app_ble_update_accel(
	int16_t x_mg,
	int16_t y_mg,
	int16_t z_mg);

int app_ble_update_measurement(
	const struct app_ble_measurement *measurement);

int app_ble_set_advertising(
	bool enabled);

int app_ble_disconnect(void);

bool app_ble_is_connected(void);

#endif