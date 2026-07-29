#ifndef APP_BLE_H
#define APP_BLE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"

enum app_ble_command {
	APP_BLE_COMMAND_RESET_COUNTERS = 0x01,
	APP_BLE_COMMAND_STATUS_REPORT = 0x02,
};

struct app_ble_callbacks {
	void (*mode_requested)(enum app_mode mode);
	void (*command_received)(enum app_ble_command command);

	uint32_t (*click_count_get)(void);
	enum app_mode (*mode_get)(void);
	bool (*button_pressed_get)(void);
	uint32_t (*uptime_seconds_get)(void);
	uint32_t (*accel_window_count_get)(void);
};

int app_ble_start(const struct app_ble_callbacks *callbacks);
int app_ble_notify_click_count(uint32_t click_count);
int app_ble_notify_mode(enum app_mode mode);
int app_ble_update_accel(int16_t x_mg,
				int16_t y_mg,
				int16_t z_mg);
int app_ble_update_vibration(int16_t mean_x_mg,
			     int16_t mean_y_mg,
			     int16_t mean_z_mg,
			     uint16_t rms_mg,
			     uint16_t peak_mg,
			     uint16_t window_count);
bool app_ble_is_connected(void);

#endif