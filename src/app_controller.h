#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

/*
 * Initializes the complete product application:
 *
 * - accelerometer
 * - motion detection
 * - Bluetooth
 * - persistent settings
 * - NFC
 * - restored operating mode
 * - runtime-state policy
 */
int app_controller_init(void);

/*
 * Runs the application event loop.
 *
 * This function normally does not return.
 */
void app_controller_run(void);

#endif