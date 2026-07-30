#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <stdint.h>

#define MEASUREMENT_WINDOW_SAMPLE_COUNT 100U

enum measurement_add_status {
	MEASUREMENT_SAMPLE_STORED = 0,
	MEASUREMENT_WINDOW_COMPLETE = 1,
};

struct vibration_result {
	int16_t mean_x_mg;
	int16_t mean_y_mg;
	int16_t mean_z_mg;

	uint16_t rms_mg;
	uint16_t peak_mg;

	uint32_t window_count;
};

struct measurement_context {
	int16_t x_mg[MEASUREMENT_WINDOW_SAMPLE_COUNT];
	int16_t y_mg[MEASUREMENT_WINDOW_SAMPLE_COUNT];
	int16_t z_mg[MEASUREMENT_WINDOW_SAMPLE_COUNT];

	uint16_t sample_index;
	uint32_t completed_window_count;
};

/*
 * Initializes the complete measurement context, including the
 * lifetime completed-window counter.
 */
void measurement_init(
	struct measurement_context *context);

/*
 * Discards only a partially collected window.
 *
 * The completed-window counter is intentionally preserved.
 */
void measurement_window_reset(
	struct measurement_context *context);

/*
 * Adds one XYZ sample to the current measurement window.
 *
 * Returns:
 *   MEASUREMENT_SAMPLE_STORED
 *       The window is still being filled.
 *
 *   MEASUREMENT_WINDOW_COMPLETE
 *       A complete result has been written to result.
 *
 *   Negative errno
 *       Invalid parameter, out-of-range sample, or corrupt index.
 */
int measurement_sample_add(
	struct measurement_context *context,
	int32_t x_mg,
	int32_t y_mg,
	int32_t z_mg,
	struct vibration_result *result);

uint32_t measurement_window_count_get(
	const struct measurement_context *context);

#endif