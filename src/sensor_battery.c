#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sensor_battery.h"

LOG_MODULE_REGISTER(sensor_battery, LOG_LEVEL_DBG);

#define APP_USER_NODE DT_PATH(zephyr_user)

/*
 * Allow time for the switched divider and ADC input node to settle.
 *
 * The measurement is infrequent, so a conservative 5 ms delay has
 * negligible effect on overall battery life.
 */
#define BATTERY_DIVIDER_SETTLE_MS 5U

/*
 * Battery-divider values from the PCB:
 *
 * Battery positive -- 1 MΩ -- ADC -- 1 MΩ -- switched ground
 */
#define BATTERY_DIVIDER_TOP_OHMS    1000000ULL
#define BATTERY_DIVIDER_BOTTOM_OHMS 1000000ULL

/*
 * Retrieves the ADC channel named "battery" from:
 *
 *     io-channels = <&adc 0>;
 *     io-channel-names = "battery";
 */
static const struct adc_dt_spec battery_adc =
	ADC_DT_SPEC_GET_BY_NAME(APP_USER_NODE, battery);

/*
 * Retrieves P0.08 from:
 *
 *     battery-enable-gpios =
 *         <&gpio0 8 GPIO_ACTIVE_HIGH>;
 */
static const struct gpio_dt_spec battery_enable =
	GPIO_DT_SPEC_GET(APP_USER_NODE,
			 battery_enable_gpios);

static bool battery_initialized;

int sensor_battery_init(void)
{
	int ret;

	if (battery_initialized) {
		return 0;
	}

	/*
	 * Configure P0.08 first and keep the battery divider disabled.
	 */
	if (!gpio_is_ready_dt(&battery_enable)) {
		LOG_ERR("Battery-divider enable GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(
		&battery_enable,
		GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		LOG_ERR("Failed to configure battery-divider enable "
			"GPIO: %d",
			ret);
		return ret;
	}

	/*
	 * Check and configure the SAADC channel.
	 */
	if (!adc_is_ready_dt(&battery_adc)) {
		LOG_ERR("Battery ADC device is not ready");
		return -ENODEV;
	}

	ret = adc_channel_setup_dt(&battery_adc);
	if (ret < 0) {
		LOG_ERR("Failed to configure battery ADC channel: %d",
			ret);
		return ret;
	}

	battery_initialized = true;

	LOG_INF("Battery measurement hardware initialized; "
		"divider disabled");

	return 0;
}

int sensor_battery_read_mv(uint16_t *battery_mv)
{
	int16_t raw_sample = 0;
	int32_t adc_input_mv;
	int64_t calculated_battery_mv = 0;
	int disable_ret;
	int ret;

	struct adc_sequence sequence = {
		.buffer = &raw_sample,
		.buffer_size = sizeof(raw_sample),
	};

	if (battery_mv == NULL) {
		return -EINVAL;
	}

	if (!battery_initialized) {
		LOG_ERR("Battery measurement requested before initialization");
		return -EACCES;
	}

	/*
	 * Fill the channel mask, resolution, and oversampling fields from
	 * the battery ADC Devicetree specification.
	 */
	ret = adc_sequence_init_dt(&battery_adc, &sequence);
	if (ret < 0) {
		LOG_ERR("Failed to initialize battery ADC sequence: %d",
			ret);
		return ret;
	}

	/*
	 * A logical 1 drives the active-high P0.08 enable signal high,
	 * turning on the BSS138 and completing the divider.
	 */
	ret = gpio_pin_set_dt(&battery_enable, 1);
	if (ret < 0) {
		LOG_ERR("Failed to enable battery divider: %d",
			ret);
		return ret;
	}

	k_sleep(K_MSEC(BATTERY_DIVIDER_SETTLE_MS));

	ret = adc_read_dt(&battery_adc, &sequence);
	if (ret < 0) {
		LOG_ERR("Failed to read battery ADC: %d", ret);
		goto disable_divider;
	}

	/*
	 * A single-ended battery reading should not be negative.
	 * A small negative result can occur from ADC offset, but it does
	 * not represent a valid battery measurement.
	 */
	if (raw_sample < 0) {
		LOG_ERR("Battery ADC returned a negative raw value: %d",
			raw_sample);
		ret = -ERANGE;
		goto disable_divider;
	}

	adc_input_mv = raw_sample;

	ret = adc_raw_to_millivolts_dt(
		&battery_adc,
		&adc_input_mv);

	if (ret < 0) {
		LOG_ERR("Failed to convert battery ADC value to "
			"millivolts: %d",
			ret);
		goto disable_divider;
	}

	if (adc_input_mv < 0) {
		LOG_ERR("Battery ADC conversion returned a negative "
			"voltage: %d mV",
			adc_input_mv);
		ret = -ERANGE;
		goto disable_divider;
	}

	/*
	 * Reconstruct the battery voltage from the divider:
	 *
	 * VBAT = VADC × (Rtop + Rbottom) / Rbottom
	 *
	 * Half of Rbottom is added before division to round to the
	 * nearest millivolt rather than always rounding down.
	 */
	calculated_battery_mv =
		((int64_t)adc_input_mv *
		 (BATTERY_DIVIDER_TOP_OHMS +
		  BATTERY_DIVIDER_BOTTOM_OHMS) +
		 (BATTERY_DIVIDER_BOTTOM_OHMS / 2ULL)) /
		BATTERY_DIVIDER_BOTTOM_OHMS;

	if ((calculated_battery_mv < 0) ||
	    (calculated_battery_mv > UINT16_MAX)) {

		LOG_ERR("Calculated battery voltage is out of range");
		ret = -ERANGE;
		goto disable_divider;
	}

	ret = 0;

disable_divider:
	/*
	 * Always switch the divider off after it has been enabled,
	 * including after an ADC or conversion failure.
	 */
	disable_ret = gpio_pin_set_dt(&battery_enable, 0);
	if (disable_ret < 0) {
		LOG_ERR("Failed to disable battery divider: %d",
			disable_ret);

		if (ret == 0) {
			ret = disable_ret;
		}
	}

	if (ret < 0) {
		return ret;
	}

	*battery_mv = (uint16_t)calculated_battery_mv;

	LOG_DBG("Battery reading: raw=%d, ADC=%d mV, "
		"battery=%u mV",
		raw_sample,
		adc_input_mv,
		(unsigned int)*battery_mv);

	return 0;
}