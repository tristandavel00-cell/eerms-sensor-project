#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "measurement.h"

static uint32_t measurement_integer_sqrt(uint64_t value)
{
	uint64_t result = 0U;
	uint64_t bit = 1ULL << 62;

	while (bit > value) {
		bit >>= 2;
	}

	while (bit != 0U) {
		if (value >= result + bit) {
			value -= result + bit;
			result = (result >> 1) + bit;
		} else {
			result >>= 1;
		}

		bit >>= 2;
	}

	return (uint32_t)result;
}

static void measurement_result_calculate(
	const struct measurement_context *context,
	struct vibration_result *result)
{
	int64_t sum_x = 0;
	int64_t sum_y = 0;
	int64_t sum_z = 0;

	for (uint16_t i = 0U;
	     i < MEASUREMENT_WINDOW_SAMPLE_COUNT;
	     i++) {

		sum_x += context->x_mg[i];
		sum_y += context->y_mg[i];
		sum_z += context->z_mg[i];
	}

	int32_t mean_x =
		(int32_t)(sum_x /
			  MEASUREMENT_WINDOW_SAMPLE_COUNT);

	int32_t mean_y =
		(int32_t)(sum_y /
			  MEASUREMENT_WINDOW_SAMPLE_COUNT);

	int32_t mean_z =
		(int32_t)(sum_z /
			  MEASUREMENT_WINDOW_SAMPLE_COUNT);

	uint64_t sum_magnitude_squared = 0U;
	uint64_t peak_magnitude_squared = 0U;

	for (uint16_t i = 0U;
	     i < MEASUREMENT_WINDOW_SAMPLE_COUNT;
	     i++) {

		int32_t dynamic_x =
			(int32_t)context->x_mg[i] - mean_x;

		int32_t dynamic_y =
			(int32_t)context->y_mg[i] - mean_y;

		int32_t dynamic_z =
			(int32_t)context->z_mg[i] - mean_z;

		uint64_t magnitude_squared =
			(uint64_t)((int64_t)dynamic_x *
				   dynamic_x) +
			(uint64_t)((int64_t)dynamic_y *
				   dynamic_y) +
			(uint64_t)((int64_t)dynamic_z *
				   dynamic_z);

		sum_magnitude_squared +=
			magnitude_squared;

		if (magnitude_squared >
		    peak_magnitude_squared) {

			peak_magnitude_squared =
				magnitude_squared;
		}
	}

	uint64_t mean_magnitude_squared =
		sum_magnitude_squared /
		MEASUREMENT_WINDOW_SAMPLE_COUNT;

	result->mean_x_mg = (int16_t)mean_x;
	result->mean_y_mg = (int16_t)mean_y;
	result->mean_z_mg = (int16_t)mean_z;

	result->rms_mg =
		(uint16_t)measurement_integer_sqrt(
			mean_magnitude_squared);

	result->peak_mg =
		(uint16_t)measurement_integer_sqrt(
			peak_magnitude_squared);
}

void measurement_init(
	struct measurement_context *context)
{
	if (context == NULL) {
		return;
	}

	memset(context, 0, sizeof(*context));
}

void measurement_window_reset(
	struct measurement_context *context)
{
	if (context == NULL) {
		return;
	}

	/*
	 * The old array contents do not need to be cleared. They will
	 * be overwritten before the next calculation is performed.
	 */
	context->sample_index = 0U;
}

int measurement_sample_add(
	struct measurement_context *context,
	int32_t x_mg,
	int32_t y_mg,
	int32_t z_mg,
	struct vibration_result *result)
{
	if ((context == NULL) || (result == NULL)) {
		return -EINVAL;
	}

	if ((x_mg < INT16_MIN) ||
	    (x_mg > INT16_MAX) ||
	    (y_mg < INT16_MIN) ||
	    (y_mg > INT16_MAX) ||
	    (z_mg < INT16_MIN) ||
	    (z_mg > INT16_MAX)) {

		return -ERANGE;
	}

	if (context->sample_index >=
	    MEASUREMENT_WINDOW_SAMPLE_COUNT) {

		context->sample_index = 0U;
		return -EOVERFLOW;
	}

	uint16_t index = context->sample_index;

	context->x_mg[index] = (int16_t)x_mg;
	context->y_mg[index] = (int16_t)y_mg;
	context->z_mg[index] = (int16_t)z_mg;

	context->sample_index++;

	if (context->sample_index <
	    MEASUREMENT_WINDOW_SAMPLE_COUNT) {

		return MEASUREMENT_SAMPLE_STORED;
	}

	measurement_result_calculate(
		context,
		result);

	context->completed_window_count++;

	result->window_count =
		context->completed_window_count;

	context->sample_index = 0U;

	return MEASUREMENT_WINDOW_COMPLETE;
}

uint32_t measurement_window_count_get(
	const struct measurement_context *context)
{
	if (context == NULL) {
		return 0U;
	}

	return context->completed_window_count;
}