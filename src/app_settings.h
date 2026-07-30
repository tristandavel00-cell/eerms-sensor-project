#ifndef APP_SETTINGS_H
#define APP_SETTINGS_H

#include "app_types.h"

/*
 * Loads all registered Zephyr settings.
 *
 * This must be called after Bluetooth initialization because
 * Bluetooth bonding information also uses the settings subsystem.
 */
int app_settings_load(void);

/*
 * Returns the application mode restored from persistent settings.
 * APP_MODE_NORMAL is returned when no mode has previously been saved.
 */
enum app_mode app_settings_mode_get(void);

/*
 * Requests an asynchronous write of the selected application mode.
 */
int app_settings_mode_save_request(enum app_mode mode);

#endif