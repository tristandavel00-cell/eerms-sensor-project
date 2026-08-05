#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nfc_t4t_lib.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "app_ble.h"
#include "app_controller.h"
#include "app_nfc.h"
#include "app_settings.h"
#include "app_types.h"
#include "measurement.h"
#include "sensor_accel.h"
#include "sensor_battery.h"

LOG_MODULE_REGISTER(app_controller, LOG_LEVEL_DBG);

/* Measurement and reporting configuration. */
#define ACCEL_SAMPLE_INTERVAL_MS       10U
#define ACCEL_BLE_REPORT_INTERVAL_MS   500U
#define ACCEL_WAKE_THRESHOLD_MG        250U
#define ACCEL_SETTLING_SAMPLE_COUNT    4U
#define NORMAL_REPORT_WINDOW_MS        30000U
#define NORMAL_PERIODIC_INTERVAL_SECONDS \
    (3U * 60U * 60U)

#define ACCEL_BLE_REPORT_SAMPLE_COUNT \
	(ACCEL_BLE_REPORT_INTERVAL_MS / ACCEL_SAMPLE_INTERVAL_MS)

/* Event queue configuration. */
#define APP_EVENT_QUEUE_SIZE   32
#define APP_EVENT_QUEUE_ALIGN  4
#define APP_EVENT_VALUE_UNUSED 0U

/*
 * Provisional CR2477 battery-condition thresholds.
 *
 * These are first bench-test values, not final production limits.
 * They must be checked using measured PCB voltage, radio activity,
 * temperature, and real battery discharge behaviour.
 */
#define BATTERY_GOOD_MIN_MV     2900U
#define BATTERY_DEGRADED_MIN_MV 2700U
#define BATTERY_LOW_MIN_MV      2500U

BUILD_ASSERT(
	BATTERY_GOOD_MIN_MV >
		BATTERY_DEGRADED_MIN_MV,
	"GOOD threshold must exceed DEGRADED threshold");

BUILD_ASSERT(
	BATTERY_DEGRADED_MIN_MV >
		BATTERY_LOW_MIN_MV,
	"DEGRADED threshold must exceed LOW threshold");

enum app_event_type {
	APP_EVENT_ACCEL_SAMPLE,
	APP_EVENT_SHAKE_DETECTED,
    APP_EVENT_PERIODIC_WAKE,
	APP_EVENT_REPORT_TIMEOUT,
	APP_EVENT_STATUS_REPORT,
	APP_EVENT_SET_MODE,
	APP_EVENT_NFC_FIELD_ON,
	APP_EVENT_NFC_FIELD_OFF,
	APP_EVENT_NFC_DATA_READ,
	APP_EVENT_NFC_DATA_UPDATED,
};

struct app_event {
	enum app_event_type type;
	uint32_t value;
};

struct app_state {
	uint16_t accel_ble_sample_counter;
	uint8_t accel_settling_samples_remaining;

    bool periodic_measurement_pending;
	bool accel_available;
	bool battery_available;
	bool nfc_available;

	bool battery_valid;
	uint16_t battery_mv;
	enum app_battery_state battery_state;

	struct measurement_context measurement;
	struct vibration_result vibration;

	enum app_mode mode;
	enum app_runtime_state runtime_state;
	enum app_wake_reason last_wake_reason;
	enum app_health_state health;
	enum app_error_code current_error;

	uint16_t fault_flags;
	uint16_t fault_occurrence_count;
	uint32_t state_transition_count;
};

static struct app_state app;

/* Forward declarations: kernel callbacks. */
static void accel_sample_timer_handler(struct k_timer *timer);
static void report_timer_handler(struct k_timer *timer);
static void periodic_timer_handler(struct k_timer *timer);

/* Forward declarations: application control. */
static int app_init(void);
static void app_event_loop(void);
static void app_log_startup_config(void);

static void app_accel_sampling_start(void);
static void app_accel_sampling_stop(void);
static void app_report_window_cancel(void);
static int app_report_window_start(void);
static void app_handle_report_timeout(void);

static void app_enter_idle(void);
static void app_start_measurement(enum app_wake_reason reason);
static void app_finish_measurement(void);
static int app_apply_ble_policy(void);
static void app_apply_mode_runtime(enum app_wake_reason reason);

static void app_periodic_timer_start(void);
static void app_periodic_timer_stop(void);
static void app_handle_periodic_wake(void);

static enum app_battery_state app_battery_state_classify(
	uint16_t battery_mv);

static int app_battery_refresh(void);

static int app_publish_measurement(
	bool vibration_valid);

/* Kernel objects. */
K_MSGQ_DEFINE(app_event_msgq,
	      sizeof(struct app_event),
	      APP_EVENT_QUEUE_SIZE,
	      APP_EVENT_QUEUE_ALIGN);

K_TIMER_DEFINE(accel_sample_timer, accel_sample_timer_handler, NULL);
K_TIMER_DEFINE(report_timer, report_timer_handler, NULL);
K_TIMER_DEFINE(periodic_timer, periodic_timer_handler, NULL);

BUILD_ASSERT(
	ACCEL_BLE_REPORT_INTERVAL_MS % ACCEL_SAMPLE_INTERVAL_MS == 0U,
	"BLE report interval must be divisible by sample interval");

BUILD_ASSERT(
	ACCEL_BLE_REPORT_SAMPLE_COUNT > 0U,
	"BLE report interval must be at least one sample interval");

BUILD_ASSERT(
	MEASUREMENT_WINDOW_SAMPLE_COUNT * ACCEL_SAMPLE_INTERVAL_MS == 1000U,
	"Accelerometer window must currently equal one second");

static const char *app_event_type_name(enum app_event_type type)
{
	switch (type) {
	case APP_EVENT_ACCEL_SAMPLE:
		return "APP_EVENT_ACCEL_SAMPLE";
	case APP_EVENT_SHAKE_DETECTED:
		return "APP_EVENT_SHAKE_DETECTED";
	case APP_EVENT_REPORT_TIMEOUT:
		return "APP_EVENT_REPORT_TIMEOUT";
	case APP_EVENT_STATUS_REPORT:
		return "APP_EVENT_STATUS_REPORT";
	case APP_EVENT_SET_MODE:
		return "APP_EVENT_SET_MODE";
	case APP_EVENT_NFC_FIELD_ON:
		return "APP_EVENT_NFC_FIELD_ON";
	case APP_EVENT_NFC_FIELD_OFF:
		return "APP_EVENT_NFC_FIELD_OFF";
	case APP_EVENT_NFC_DATA_READ:
		return "APP_EVENT_NFC_DATA_READ";
	case APP_EVENT_NFC_DATA_UPDATED:
		return "APP_EVENT_NFC_DATA_UPDATED";
    case APP_EVENT_PERIODIC_WAKE:
        return "APP_EVENT_PERIODIC_WAKE";
	default:
		return "APP_EVENT_UNKNOWN";
	}
}

static const char *app_mode_name(enum app_mode mode)
{
	switch (mode) {
	case APP_MODE_NORMAL:
		return "NORMAL";
	case APP_MODE_CONFIG:
		return "CONFIG";
	case APP_MODE_DIAGNOSTIC:
		return "DIAGNOSTIC";
	default:
		return "UNKNOWN";
	}
}

static const char *app_runtime_state_name(enum app_runtime_state state)
{
	switch (state) {
	case APP_STATE_BOOT:
		return "BOOT";
	case APP_STATE_IDLE:
		return "IDLE";
	case APP_STATE_MEASURING:
		return "MEASURING";
	case APP_STATE_REPORTING:
		return "REPORTING";
	case APP_STATE_ERROR:
		return "ERROR";
	default:
		return "UNKNOWN";
	}
}

static const char *app_wake_reason_name(enum app_wake_reason reason)
{
	switch (reason) {
	case APP_WAKE_REASON_NONE:
		return "NONE";
	case APP_WAKE_REASON_BOOT:
		return "BOOT";
	case APP_WAKE_REASON_SHAKE:
		return "SHAKE";
	case APP_WAKE_REASON_PERIODIC:
		return "PERIODIC";
	case APP_WAKE_REASON_NFC:
		return "NFC";
	case APP_WAKE_REASON_BLE_COMMAND:
		return "BLE_COMMAND";
	default:
		return "UNKNOWN";
	}
}

static bool app_runtime_state_is_valid(enum app_runtime_state state)
{
	switch (state) {
	case APP_STATE_BOOT:
	case APP_STATE_IDLE:
	case APP_STATE_MEASURING:
	case APP_STATE_REPORTING:
	case APP_STATE_ERROR:
		return true;
	default:
		return false;
	}
}

static void app_set_runtime_state(enum app_runtime_state new_state)
{
	enum app_runtime_state previous_state;

	if (!app_runtime_state_is_valid(new_state)) {
		LOG_ERR("Invalid runtime state requested: %d", new_state);
		new_state = APP_STATE_ERROR;
	}

	if (new_state == app.runtime_state) {
		return;
	}

	previous_state = app.runtime_state;
	app.runtime_state = new_state;
	app.state_transition_count++;

	LOG_INF("Runtime state changed: %s -> %s",
		app_runtime_state_name(previous_state),
		app_runtime_state_name(new_state));
}

static void app_fault_record(enum app_error_code error_code,
			     uint16_t fault_mask,
			     enum app_health_state health)
{
	if ((error_code == APP_ERROR_NONE) ||
	    (fault_mask == APP_FAULT_NONE) ||
	    (health <= APP_HEALTH_OK) ||
	    (health > APP_HEALTH_FAULTED)) {

		LOG_ERR("Invalid fault record request: error=0x%04x, "
			"mask=0x%04x, health=%u",
			(unsigned int)(uint16_t)error_code,
			(unsigned int)fault_mask,
			(unsigned int)health);

		return;
	}

	/*
	 * Preserve the most serious health level.
	 *
	 * An error at the same severity may replace the previous error,
	 * allowing current_error to represent the latest failure at that
	 * severity.
	 */
	if (health >= app.health) {
		app.current_error = error_code;
	}

	/*
	 * Latch the affected subsystem without removing any previously
	 * recorded subsystem faults.
	 */
	app.fault_flags |= fault_mask;

	/*
	 * Saturate the counter instead of allowing it to wrap to zero.
	 */
	if (app.fault_occurrence_count < UINT16_MAX) {
		app.fault_occurrence_count++;
	}

	if (health > app.health) {
		app.health = health;
	}

	LOG_ERR("Fault recorded: error=0x%04x, flags=0x%04x, "
		"health=%u, count=%u",
		(unsigned int)(uint16_t)app.current_error,
		(unsigned int)app.fault_flags,
		(unsigned int)app.health,
		(unsigned int)app.fault_occurrence_count);
}

static enum app_battery_state app_battery_state_classify(
	uint16_t battery_mv)
{
	if (battery_mv >= BATTERY_GOOD_MIN_MV) {
		return APP_BATTERY_STATE_GOOD;
	}

	if (battery_mv >= BATTERY_DEGRADED_MIN_MV) {
		return APP_BATTERY_STATE_DEGRADED;
	}

	if (battery_mv >= BATTERY_LOW_MIN_MV) {
		return APP_BATTERY_STATE_LOW;
	}

	return APP_BATTERY_STATE_EXTREMELY_LOW;
}

static int app_battery_refresh(void)
{
	uint16_t battery_mv;
	int ret;

	if (!app.battery_available) {
		app.battery_valid = false;
		app.battery_mv = 0U;
		app.battery_state =
			APP_BATTERY_STATE_UNKNOWN;

		return -ENODEV;
	}

	ret = sensor_battery_read_mv(&battery_mv);
	if (ret < 0) {
		LOG_ERR("Failed to read battery voltage: %d",
			ret);

		app.battery_valid = false;
		app.battery_mv = 0U;
		app.battery_state =
			APP_BATTERY_STATE_UNKNOWN;

		app_fault_record(
			APP_ERROR_BATTERY_ADC_READ,
			APP_FAULT_BATTERY,
			APP_HEALTH_DEGRADED);

		return ret;
	}

	app.battery_mv = battery_mv;
	app.battery_state =
		app_battery_state_classify(battery_mv);
	app.battery_valid = true;

	LOG_INF("Battery measurement: %u mV, state=%u",
		(unsigned int)app.battery_mv,
		(unsigned int)app.battery_state);

	return 0;
}

static int app_publish_measurement(
	bool vibration_valid)
{
	struct app_ble_measurement report = {
		.valid_flags = 0U,

		.battery_state =
			app.battery_state,

		.health =
			app.health,

		.battery_mv =
			app.battery_valid ?
				app.battery_mv :
				0U,

		/*
		 * Temperature is reserved in protocol version 1 but
		 * remains invalid until the PCB connection is corrected.
		 */
		.temperature_centi_c = 0,

		.mean_x_mg = 0,
		.mean_y_mg = 0,
		.mean_z_mg = 0,

		.rms_mg = 0U,
		.peak_mg = 0U,
		.sequence = 0U,
	};

	if (app.battery_valid) {
		report.valid_flags |=
			APP_BLE_MEASUREMENT_VALID_BATTERY;
	}

	if (vibration_valid) {
		report.valid_flags |=
			APP_BLE_MEASUREMENT_VALID_VIBRATION;

		report.mean_x_mg =
			app.vibration.mean_x_mg;
		report.mean_y_mg =
			app.vibration.mean_y_mg;
		report.mean_z_mg =
			app.vibration.mean_z_mg;

		report.rms_mg =
			app.vibration.rms_mg;
		report.peak_mg =
			app.vibration.peak_mg;

		report.sequence =
			(uint16_t)app.vibration.window_count;
	}

	return app_ble_update_measurement(&report);
}

static uint32_t app_uptime_seconds(void)
{
	return k_uptime_get_32() / 1000U;
}

static void app_post_event(enum app_event_type type, uint32_t value)
{
	const struct app_event event = {
		.type = type,
		.value = value,
	};
	int ret;

	ret = k_msgq_put(&app_event_msgq, &event, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to post %s: %d",
			app_event_type_name(type),
			ret);
		return;
	}

	if (type != APP_EVENT_ACCEL_SAMPLE) {
		LOG_DBG("Event posted: %s", app_event_type_name(type));
	}
}

/* BLE callback adapters. */
static void app_ble_mode_requested(enum app_mode mode)
{
	app_post_event(APP_EVENT_SET_MODE, (uint32_t)mode);
}

static void app_ble_command_received(enum app_ble_command command)
{
	switch (command) {
	case APP_BLE_COMMAND_STATUS_REPORT:
		app_post_event(APP_EVENT_STATUS_REPORT, APP_EVENT_VALUE_UNUSED);
		break;
	default:
		LOG_WRN("Unknown BLE command: %d", command);
		break;
	}
}

static enum app_mode app_ble_mode_get(void)
{
	return app.mode;
}

static enum app_runtime_state app_ble_runtime_state_get(void)
{
	return app.runtime_state;
}

static enum app_wake_reason app_ble_wake_reason_get(void)
{
	return app.last_wake_reason;
}

static enum app_health_state app_ble_health_get(void)
{
	return app.health;
}

static uint16_t app_ble_error_code_get(void)
{
	return (uint16_t)app.current_error;
}

static uint16_t app_ble_fault_flags_get(void)
{
	return app.fault_flags;
}

static uint16_t app_ble_fault_occurrence_count_get(void)
{
	return app.fault_occurrence_count;
}

static uint32_t app_ble_state_transition_count_get(void)
{
	return app.state_transition_count;
}

static uint32_t app_ble_uptime_seconds_get(void)
{
	return app_uptime_seconds();
}

static const struct app_ble_callbacks ble_callbacks = {
	.mode_requested = app_ble_mode_requested,
	.command_received = app_ble_command_received,

	.mode_get = app_ble_mode_get,
	.runtime_state_get = app_ble_runtime_state_get,
	.wake_reason_get = app_ble_wake_reason_get,
	.health_get = app_ble_health_get,

	.error_code_get = app_ble_error_code_get,
	.fault_flags_get = app_ble_fault_flags_get,
	.fault_occurrence_count_get =
		app_ble_fault_occurrence_count_get,

	.state_transition_count_get =
		app_ble_state_transition_count_get,
	.uptime_seconds_get = app_ble_uptime_seconds_get,
};

/* NFC callback adapters. */
static void app_nfc_mode_command_received(enum app_mode mode)
{
	app_post_event(APP_EVENT_SET_MODE, (uint32_t)mode);
}

static void app_nfc_callback(void *context,
			     nfc_t4t_event_t event,
			     const uint8_t *data,
			     size_t data_length,
			     uint32_t flags)
{
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(flags);

	switch (event) {
	case NFC_T4T_EVENT_FIELD_ON:
		if (app_nfc_field_on()) {
			app_post_event(APP_EVENT_NFC_FIELD_ON,
				       APP_EVENT_VALUE_UNUSED);
		}
		break;

	case NFC_T4T_EVENT_FIELD_OFF:
		if (app_nfc_field_off()) {
			app_post_event(APP_EVENT_NFC_FIELD_OFF,
				       APP_EVENT_VALUE_UNUSED);
		}
		break;

	case NFC_T4T_EVENT_NDEF_READ:
		app_post_event(APP_EVENT_NFC_DATA_READ,
			       APP_EVENT_VALUE_UNUSED);
		break;

	case NFC_T4T_EVENT_NDEF_UPDATED:
		app_nfc_write_received(data_length);
		app_post_event(APP_EVENT_NFC_DATA_UPDATED,
			       (uint32_t)data_length);
		break;

	default:
		break;
	}
}

static int app_nfc_init(void)
{
	int ret;

	ret = nfc_t4t_setup(app_nfc_callback, NULL);
	if (ret < 0) {
		LOG_ERR("Failed to set up NFC Type 4 Tag: %d", ret);
		return ret;
	}

	ret = app_nfc_start(app.mode,
			    app.current_error,
			    app_nfc_mode_command_received);
	if (ret < 0) {
		LOG_ERR("Failed to start NFC payload: %d", ret);
		return ret;
	}

	LOG_INF("NFC initialized");
	return 0;
}


/* Sensor and timer callbacks. */
static void app_accel_motion_detected(void)
{
	app_post_event(APP_EVENT_SHAKE_DETECTED, APP_EVENT_VALUE_UNUSED);
}

static void accel_sample_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	app_post_event(APP_EVENT_ACCEL_SAMPLE, APP_EVENT_VALUE_UNUSED);
}

static void report_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	app_post_event(APP_EVENT_REPORT_TIMEOUT, APP_EVENT_VALUE_UNUSED);
}

static void periodic_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    app_post_event(APP_EVENT_PERIODIC_WAKE, APP_EVENT_VALUE_UNUSED);
}

static void app_accel_sampling_start(void)
{
	k_timer_start(&accel_sample_timer,
		      K_MSEC(ACCEL_SAMPLE_INTERVAL_MS),
		      K_MSEC(ACCEL_SAMPLE_INTERVAL_MS));
}

static void app_accel_sampling_stop(void)
{
	k_timer_stop(&accel_sample_timer);
}

static void app_periodic_timer_start(void)
{
    k_timer_start(
        &periodic_timer,
        K_SECONDS(
            NORMAL_PERIODIC_INTERVAL_SECONDS),
        K_SECONDS(
            NORMAL_PERIODIC_INTERVAL_SECONDS));

    LOG_INF("Periodic measurement timer start: %u seconds",
        NORMAL_PERIODIC_INTERVAL_SECONDS);
}

static void app_periodic_timer_stop(void)
{
    k_timer_stop(&periodic_timer);

    app.periodic_measurement_pending = false;

    LOG_DBG("Periodic measurement timer stopped");
}

static void app_handle_periodic_wake(void)
{
    if(app.mode != APP_MODE_NORMAL) {
        app.periodic_measurement_pending = false;

        LOG_DBG("Periodic wake ignored in application mode %s",
            app_mode_name(app.mode));

        return;
    }

    switch(app.runtime_state) {
        case APP_STATE_IDLE:
            app.periodic_measurement_pending = false;

            LOG_INF("Periodic measurement requested");

            app_start_measurement(APP_WAKE_REASON_PERIODIC);
            break;

        case APP_STATE_MEASURING:
        case APP_STATE_REPORTING:

            app.periodic_measurement_pending = true;

            LOG_INF("Periodic measurement deferred while in state %s",
                app_runtime_state_name(
                    app.runtime_state));

                break;

        case APP_STATE_BOOT:
            LOG_DBG("Periodic wake ignored during boot");
            break;

        case APP_STATE_ERROR:
            LOG_WRN("Periodic wake ignored while in ERROR state");
            break;
        default:
            LOG_WRN("Periodic wake ignored while in UNKNOWN state: %d",
                app.runtime_state);
            break;
    }
}

static void app_report_window_cancel(void)
{
	k_timer_stop(&report_timer);
}

static void app_enter_idle(void)
{
	int ret;

	app_report_window_cancel();
	app_accel_sampling_stop();
	measurement_window_reset(&app.measurement);

	app.accel_ble_sample_counter = 0U;
	app.accel_settling_samples_remaining = 0U;

	ret = sensor_accel_motion_disarm();
	if (ret < 0) {
		LOG_ERR("Failed to disarm motion detection while entering idle: %d",
			ret);

		app_fault_record(APP_ERROR_ACCEL_MOTION_DISARM,
				 APP_FAULT_ACCEL,
				 APP_HEALTH_FAULTED);

		app_set_runtime_state(APP_STATE_ERROR);
		return;
	}

	switch (app.mode) {
	case APP_MODE_NORMAL:
        ret = sensor_accel_profile_set(
			SENSOR_ACCEL_PROFILE_MOTION);

			if (ret < 0) {
				LOG_ERR("Failed to apply motion profile: %d",
					ret);

				app_fault_record(APP_ERROR_ACCEL_PROFILE,
				 		APP_FAULT_ACCEL,
				 		APP_HEALTH_FAULTED);

				app_set_runtime_state(APP_STATE_ERROR);
				return;
			}

        app_set_runtime_state(
	        APP_STATE_IDLE);

/*
 * A scheduled wake may have occurred while the previous
 * measurement or report window was still active.
 *
 * Run it now instead of arming motion detection and waiting
 * until the next three-hour interval.
 */
        if (app.periodic_measurement_pending) {
	        app.periodic_measurement_pending = false;

	        LOG_INF("Starting deferred periodic measurement");

	        app_start_measurement(
		        APP_WAKE_REASON_PERIODIC);

	        return;
        }

       	ret = sensor_accel_motion_arm();

			if (ret < 0) {
				LOG_ERR("Failed to arm shake detection: %d",
					ret);

				app_fault_record(APP_ERROR_ACCEL_MOTION_ARM,
				 		APP_FAULT_ACCEL,
						 APP_HEALTH_FAULTED);

				app_set_runtime_state(APP_STATE_ERROR);
		return;
	}
		break;

	case APP_MODE_CONFIG:
		ret = sensor_accel_profile_set(SENSOR_ACCEL_PROFILE_OFF);
		if (ret < 0) {
			LOG_ERR("Failed to power down accelerometer: %d",
				ret);

			app_fault_record(APP_ERROR_ACCEL_PROFILE,
					 APP_FAULT_ACCEL,
					 APP_HEALTH_FAULTED);

			app_set_runtime_state(APP_STATE_ERROR);
			return;
		}

		app_set_runtime_state(APP_STATE_IDLE);
		break;

	default:
		LOG_ERR("Cannot enter idle from application mode %d", app.mode);
		app_set_runtime_state(APP_STATE_ERROR);
		break;
	}
}

static void app_start_measurement(enum app_wake_reason reason)
{
	int ret;

	if (app.runtime_state == APP_STATE_ERROR) {
		LOG_WRN("Measurement start rejected while in ERROR state");
		return;
	}

	if (app.runtime_state == APP_STATE_MEASURING) {
		LOG_DBG("Measurement already active");
		return;
	}

	app_report_window_cancel();

	ret = sensor_accel_motion_disarm();
	if (ret < 0) {
		LOG_ERR("Failed to disarm motion detection before "
			"measurement: %d",
			ret);

		app_fault_record(APP_ERROR_ACCEL_MOTION_DISARM,
				 APP_FAULT_ACCEL,
				 APP_HEALTH_FAULTED);

		app_set_runtime_state(APP_STATE_ERROR);
		return;
	}

	ret = sensor_accel_profile_set(
		SENSOR_ACCEL_PROFILE_MEASUREMENT);

	if (ret < 0) {
		LOG_ERR("Failed to apply accelerometer measurement "
			"profile: %d",
			ret);

		app_fault_record(APP_ERROR_ACCEL_PROFILE,
				 APP_FAULT_ACCEL,
				 APP_HEALTH_FAULTED);

		app_set_runtime_state(APP_STATE_ERROR);
		return;
	}

	if (reason != APP_WAKE_REASON_NONE) {
		app.last_wake_reason = reason;
	}

	measurement_window_reset(&app.measurement);

	app.accel_ble_sample_counter = 0U;
	app.accel_settling_samples_remaining =
		ACCEL_SETTLING_SAMPLE_COUNT;

	app_set_runtime_state(APP_STATE_MEASURING);
	app_accel_sampling_start();

	LOG_INF("Accelerometer measurement started. Reason: %s",
		app_wake_reason_name(app.last_wake_reason));
}

static int app_report_window_start(void)
{
	int ret;

	if (app.mode != APP_MODE_NORMAL) {
		LOG_ERR("Report window requested outside NORMAL mode");
		return -EINVAL;
	}

	if (app.runtime_state != APP_STATE_REPORTING) {
		LOG_ERR("Report window requested in runtime state %s",
			app_runtime_state_name(app.runtime_state));
		return -EINVAL;
	}

	app_accel_sampling_stop();

	ret = sensor_accel_motion_disarm();
	if (ret < 0) {
		LOG_ERR("Failed to disarm motion detection for reporting: %d", ret);
		return ret;
	}

	ret = sensor_accel_profile_set(SENSOR_ACCEL_PROFILE_OFF);
	if (ret < 0) {
		LOG_ERR("Failed to power down accelerometer for reporting: %d", ret);
		return ret;
	}

	ret = app_ble_set_advertising(true);
	if (ret < 0) {
		LOG_ERR("Failed to enable Bluetooth reporting advertising: %d",
			ret);
		return ret;
	}

	app_report_window_cancel();
	k_timer_start(&report_timer,
		      K_MSEC(NORMAL_REPORT_WINDOW_MS),
		      K_NO_WAIT);

	LOG_INF("NORMAL-mode Bluetooth report window started: %u ms",
		NORMAL_REPORT_WINDOW_MS);
	return 0;
}

static void app_handle_report_timeout(void)
{
	int ret;

	if ((app.mode != APP_MODE_NORMAL) ||
	    (app.runtime_state != APP_STATE_REPORTING)) {

		LOG_DBG("Ignoring stale report-timeout event");
		return;
	}

	app_report_window_cancel();

	/*
	 * A healthy NORMAL-mode device stops advertising here.
	 *
	 * A device with a recorded fault keeps advertising so its
	 * diagnostic status remains accessible.
	 */
	ret = app_apply_ble_policy();
	if (ret < 0) {
		LOG_ERR("Failed to restore Bluetooth policy after "
			"report window: %d",
			ret);

		app_fault_record(APP_ERROR_BLE_ADVERTISING,
				 APP_FAULT_BLE,
				 APP_HEALTH_FAULTED);

		(void)app_ble_set_advertising(true);
		app_set_runtime_state(APP_STATE_ERROR);
		return;
	}

	ret = app_ble_disconnect();
	if (ret < 0) {
		LOG_ERR("Failed to end reporting connection: %d",
			ret);
	}

	LOG_INF("NORMAL-mode Bluetooth report window ended");

	app_enter_idle();
}

static void app_finish_measurement(void)
{
	const bool continuous_measurement =
		(app.mode == APP_MODE_DIAGNOSTIC);
	int ret;

	if (!continuous_measurement) {
		app_accel_sampling_stop();
	}

	app_set_runtime_state(APP_STATE_REPORTING);

		/*
	 * NORMAL mode sends a fresh battery result with each completed
	 * report. DIAGNOSTIC mode reuses the boot measurement instead of
	 * sampling the battery every one-second vibration window.
	 */
	if ((app.mode == APP_MODE_NORMAL) &&
	    app.battery_available) {

		(void)app_battery_refresh();
	}

	app_set_runtime_state(APP_STATE_REPORTING);

	ret = app_publish_measurement(true);

	if ((ret < 0) && (ret != -ENOTCONN) && (ret != -EACCES)) {
		LOG_WRN("Failed to update BLE measurement result: %d", ret);
	}

	if (continuous_measurement) {
		app_set_runtime_state(APP_STATE_MEASURING);
		return;
	}

	if (app.mode == APP_MODE_NORMAL) {
		ret = app_report_window_start();
		if (ret < 0) {
			LOG_ERR("Failed to start NORMAL report window: %d", ret);
			(void)app_ble_set_advertising(false);
			app_set_runtime_state(APP_STATE_ERROR);
		}
		return;
	}

	/* Safely handle a mode change that completed during measurement. */
	app_enter_idle();
}

static int app_apply_ble_policy(void)
{
	/*
	 * Fault visibility takes priority over the normal low-power
	 * advertising policy.
	 *
	 * A device with any recorded fault must remain discoverable so
	 * its version 2 status packet can be read.
	 */
	if (app.fault_flags != APP_FAULT_NONE) {
		return app_ble_set_advertising(true);
	}

	switch (app.mode) {
	case APP_MODE_NORMAL:
		return app_ble_set_advertising(false);

	case APP_MODE_CONFIG:
	case APP_MODE_DIAGNOSTIC:
		return app_ble_set_advertising(true);

	default:
		return -EINVAL;
	}
}

static void app_apply_mode_runtime(enum app_wake_reason reason)
{
	int ret;

	app_report_window_cancel();

	ret = app_apply_ble_policy();
	if (ret < 0) {
		LOG_ERR("Failed to apply Bluetooth policy for mode %s: %d",
			app_mode_name(app.mode),
			ret);

		app_fault_record(APP_ERROR_BLE_ADVERTISING,
				 APP_FAULT_BLE,
				 APP_HEALTH_FAULTED);

		/*
		 * Make one recovery attempt so the recorded fault may still
		 * become visible.
		 */
		(void)app_ble_set_advertising(true);

		app_set_runtime_state(APP_STATE_ERROR);
		return;
	}

	switch (app.mode) {
	case APP_MODE_NORMAL:
		if (!app.accel_available) {
			app_periodic_timer_stop();

			LOG_ERR("NORMAL mode unavailable: "
				"accelerometer is not operational");

			app_set_runtime_state(APP_STATE_ERROR);
			return;
		}

		app_periodic_timer_start();
		app_enter_idle();
		break;

	case APP_MODE_CONFIG:
		app_periodic_timer_stop();

		/*
		 * CONFIG mode remains useful for BLE diagnostics and mode
		 * control even when the accelerometer is unavailable.
		 */
		if (!app.accel_available) {
			app_accel_sampling_stop();
			app_set_runtime_state(APP_STATE_IDLE);
			return;
		}

		app_enter_idle();
		break;

	case APP_MODE_DIAGNOSTIC:
		app_periodic_timer_stop();

		if (!app.accel_available) {
			LOG_ERR("DIAGNOSTIC measurement unavailable: "
				"accelerometer is not operational");

			app_set_runtime_state(APP_STATE_ERROR);
			return;
		}

		app_start_measurement(reason);
		break;

	default:
		LOG_ERR("Cannot apply unknown application mode: %d",
			app.mode);

		app_periodic_timer_stop();
		app_set_runtime_state(APP_STATE_ERROR);
		break;
	}
}

static void app_handle_accel_sample(void)
{
	struct sensor_accel_sample sample;
	int measurement_status;
	int ret;

	if (app.runtime_state != APP_STATE_MEASURING) {
		LOG_DBG("Ignoring accelerometer sample event in state %s",
			app_runtime_state_name(app.runtime_state));
		return;
	}

	ret = sensor_accel_read(&sample);
	if (ret < 0) {
		LOG_ERR("Failed to read accelerometer: %d", ret);
		return;
	}

	if (app.accel_settling_samples_remaining > 0U) {
		app.accel_settling_samples_remaining--;
		return;
	}

	measurement_status = measurement_sample_add(&app.measurement,
					    sample.x_mg,
					    sample.y_mg,
					    sample.z_mg,
					    &app.vibration);
	if (measurement_status < 0) {
		LOG_ERR("Failed to process accelerometer sample: %d",
			measurement_status);

		app_fault_record(APP_ERROR_MEASUREMENT_PROCESS,
			APP_FAULT_APPLICATION,
			APP_HEALTH_FAULTED);

		app_accel_sampling_stop();
		app_set_runtime_state(APP_STATE_ERROR);
		return;
	}

	app.accel_ble_sample_counter++;
	if (app.accel_ble_sample_counter >= ACCEL_BLE_REPORT_SAMPLE_COUNT) {
		app.accel_ble_sample_counter = 0U;
		(void)app_ble_update_accel((int16_t)sample.x_mg,
					   (int16_t)sample.y_mg,
					   (int16_t)sample.z_mg);
	}

	if (measurement_status != MEASUREMENT_WINDOW_COMPLETE) {
		return;
	}

	LOG_INF("Window %u: mean=(%d, %d, %d) mg, RMS=%u mg, peak=%u mg",
		app.vibration.window_count,
		app.vibration.mean_x_mg,
		app.vibration.mean_y_mg,
		app.vibration.mean_z_mg,
		app.vibration.rms_mg,
		app.vibration.peak_mg);

	app_finish_measurement();
}

static void app_handle_mode_request(enum app_mode requested_mode)
{
	int ret;

	switch (requested_mode) {
	case APP_MODE_NORMAL:
	case APP_MODE_CONFIG:
	case APP_MODE_DIAGNOSTIC:
		break;
	default:
		LOG_WRN("Invalid application mode event value: %u",
			(uint32_t)requested_mode);
		return;
	}

	if (requested_mode == app.mode) {
		LOG_INF("Application mode already set to: %s",
			app_mode_name(app.mode));
		return;
	}

	app.mode = requested_mode;
	LOG_INF("Application mode changed to: %s", app_mode_name(app.mode));

	app_apply_mode_runtime(APP_WAKE_REASON_NONE);

	ret = app_settings_mode_save_request(
	app.mode);

if (ret < 0) {
	LOG_ERR("Failed to request application mode save: %d",
		ret);
}

		(void)app_ble_notify_mode(app.mode);

	if (app.nfc_available) {
		app_nfc_update_request(app.mode,
				       app.current_error);
	}
}


static void app_handle_shake_detected(void)
{
	if (app.mode != APP_MODE_NORMAL) {
		LOG_DBG("Shake ignored in application mode %s",
			app_mode_name(app.mode));
		return;
	}

	if (app.runtime_state != APP_STATE_IDLE) {
		LOG_DBG("Shake ignored in runtime state %s",
			app_runtime_state_name(app.runtime_state));
		return;
	}

	LOG_INF("Shake detected");
	app_start_measurement(APP_WAKE_REASON_SHAKE);
}

static void app_log_status(void)
{
	LOG_INF("Status: mode=%s, state=%s, wake=%s, error=0x%04x, "
		"transitions=%u, uptime=%u s, windows=%u",
		app_mode_name(app.mode),
		app_runtime_state_name(app.runtime_state),
		app_wake_reason_name(app.last_wake_reason),
		(unsigned int)(uint16_t)app.current_error,
		app.state_transition_count,
		app_uptime_seconds(),
		measurement_window_count_get(&app.measurement));
}

static void app_event_handler(const struct app_event *event)
{
	switch (event->type) {
	case APP_EVENT_ACCEL_SAMPLE:
		app_handle_accel_sample();
		break;

    case APP_EVENT_PERIODIC_WAKE:
	    app_handle_periodic_wake();
	    break;

	case APP_EVENT_SHAKE_DETECTED:
		app_handle_shake_detected();
		break;

	case APP_EVENT_REPORT_TIMEOUT:
		app_handle_report_timeout();
		break;

	case APP_EVENT_STATUS_REPORT:
		app_log_status();
		break;

	case APP_EVENT_SET_MODE:
		app_handle_mode_request((enum app_mode)event->value);
		break;

	case APP_EVENT_NFC_FIELD_ON:
		LOG_INF("NFC field detected");
		break;

	case APP_EVENT_NFC_FIELD_OFF:
		LOG_INF("NFC field removed");
		break;

	case APP_EVENT_NFC_DATA_READ:
		LOG_INF("NFC message read by phone");
		break;

	case APP_EVENT_NFC_DATA_UPDATED:
		LOG_INF("NFC write received: %u-byte NDEF message", event->value);
		break;

	default:
		LOG_WRN("Unknown event type: %d (%s)",
			event->type,
			app_event_type_name(event->type));
		break;
	}
}

static int app_init(void)
{
	int ret;

	app.mode = APP_MODE_NORMAL;
	app.runtime_state = APP_STATE_BOOT;
	app.last_wake_reason = APP_WAKE_REASON_BOOT;
	app.health = APP_HEALTH_OK;
	app.current_error = APP_ERROR_NONE;

	app.fault_flags = APP_FAULT_NONE;
	app.fault_occurrence_count = 0U;
	app.state_transition_count = 0U;

	app.periodic_measurement_pending = false;
	app.accel_available = false;
	app.battery_available = false;
	app.nfc_available = false;

	app.battery_valid = false;
	app.battery_mv = 0U;
	app.battery_state =
		APP_BATTERY_STATE_UNKNOWN;

	measurement_init(&app.measurement);

	/*
	 * Bluetooth is initialized first because it is the primary
	 * diagnostic interface on the production PCB.
	 *
	 * A Bluetooth initialization failure cannot be reported through
	 * Bluetooth, so this remains a fatal initialization failure.
	 */
	ret = app_ble_start(&ble_callbacks);
	if (ret < 0) {
		LOG_ERR("Failed to initialize Bluetooth module: %d",
			ret);
		return ret;
	}

	/*
	 * Settings must still be loaded after Bluetooth initialization,
	 * because the Bluetooth stack registers settings handlers for
	 * bonding information.
	 *
	 * A settings failure does not prevent measurement or BLE status
	 * reporting. Fall back to NORMAL mode and record a warning.
	 */
	ret = app_settings_load();
	if (ret < 0) {
		LOG_ERR("Failed to load persistent settings: %d",
			ret);

		app_fault_record(APP_ERROR_SETTINGS_LOAD,
				 APP_FAULT_SETTINGS,
				 APP_HEALTH_WARNING);

		app.mode = APP_MODE_NORMAL;
	} else {
		app.mode = app_settings_mode_get();
	}

	LOG_INF("Restored application mode: %s",
		app_mode_name(app.mode));

	ret = sensor_battery_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize battery sensor: %d",
			ret);

		app_fault_record(APP_ERROR_BATTERY_INIT,
				 APP_FAULT_BATTERY,
				 APP_HEALTH_DEGRADED);

	} else {
		app.battery_available = true;
		(void)app_battery_refresh();
	}

	/*
	 * Accelerometer initialization is recoverable from the firmware's
	 * perspective: BLE status reporting must remain operational even
	 * when vibration measurement cannot run.
	 */
	ret = sensor_accel_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize accelerometer: %d",
			ret);

		app_fault_record(APP_ERROR_ACCEL_INIT,
				 APP_FAULT_ACCEL,
				 APP_HEALTH_FAULTED);
	} else {
		ret = sensor_accel_motion_init(
			app_accel_motion_detected,
			ACCEL_WAKE_THRESHOLD_MG);

		if (ret < 0) {
			LOG_ERR("Failed to initialize accelerometer "
				"motion detection: %d",
				ret);

			app_fault_record(APP_ERROR_ACCEL_MOTION_INIT,
					 APP_FAULT_ACCEL,
					 APP_HEALTH_FAULTED);
		} else {
			app.accel_available = true;
		}
	}

	/*
	 * NFC is an optional configuration interface. A failure is
	 * reported as a warning but must not stop BLE or measurement.
	 */
	ret = app_nfc_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize NFC: %d",
			ret);

		app_fault_record(APP_ERROR_NFC_INIT,
				 APP_FAULT_NFC,
				 APP_HEALTH_WARNING);
	} else {
		app.nfc_available = true;
	}

		/*
	 * Publish the boot-time battery result immediately.
	 *
	 * Vibration and temperature remain invalid until their
	 * respective measurements have completed.
	 */
	(void)app_publish_measurement(false);

	/*
	 * This applies either the normal operating policy or the
	 * BLE-visible degraded/fault policy.
	 */
	app_apply_mode_runtime(APP_WAKE_REASON_BOOT);

	/*
	 * Recoverable subsystem failures must not prevent the controller
	 * event loop from running.
	 */
	return 0;
}

static void app_log_startup_config(void)
{
	LOG_INF("Event queue size: %u", APP_EVENT_QUEUE_SIZE);
	LOG_INF("Accelerometer: sample interval=%u ms, window=%u samples, "
		"settling=%u samples, wake threshold=%u mg",
		ACCEL_SAMPLE_INTERVAL_MS,
		MEASUREMENT_WINDOW_SAMPLE_COUNT,
		ACCEL_SETTLING_SAMPLE_COUNT,
		ACCEL_WAKE_THRESHOLD_MG);
	LOG_INF("NORMAL report window: %u ms", NORMAL_REPORT_WINDOW_MS);
	LOG_INF("Application mode: %s", app_mode_name(app.mode));
	LOG_INF("Runtime state: %s",
		app_runtime_state_name(app.runtime_state));
	LOG_INF("Last wake reason: %s",
		app_wake_reason_name(app.last_wake_reason));
	LOG_INF("Current error: 0x%04x",
		(unsigned int)(uint16_t)app.current_error);
    LOG_INF("Periodic measurement interval: %u seconds",
	    NORMAL_PERIODIC_INTERVAL_SECONDS);
}

static void app_event_loop(void)
{
	while (true) {
		struct app_event event;
		int ret;

		ret = k_msgq_get(&app_event_msgq, &event, K_FOREVER);
		if (ret < 0) {
			LOG_ERR("Failed to get event from message queue: %d", ret);
			continue;
		}

		app_event_handler(&event);
	}
}

int app_controller_init(void)
{
	int ret;

	ret = app_init();

	if (ret < 0) {
		return ret;
	}

	app_log_startup_config();

	return 0;
}

void app_controller_run(void)
{
	app_event_loop();
}