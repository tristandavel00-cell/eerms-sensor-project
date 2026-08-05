#ifndef SENSOR_BATTERY_H
#define SENSOR_BATTERY_H

int sensor_battery_init(void);

int sensor_battery_read_mv(uint16_t *battery_mv);

#endif