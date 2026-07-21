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
};

int app_ble_start(const struct app_ble_callbacks *callbacks);
int app_ble_notify_click_count(uint32_t click_count);
int app_ble_notify_mode(enum app_mode mode);
bool app_ble_is_connected(void);

#endif