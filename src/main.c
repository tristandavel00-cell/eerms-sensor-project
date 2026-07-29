#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>
#include <nfc_t4t_lib.h>

#include "app_ble.h"
#include "app_nfc.h"
#include "app_types.h"
#include "sensor_accel.h"

LOG_MODULE_REGISTER(button_app, LOG_LEVEL_DBG);

/* Devicetree aliases */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define SW0_NODE  DT_ALIAS(sw0)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(LED1_NODE, okay)
#error "Unsupported board: led1 alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 alias is not defined"
#endif

/* Timing configuration */
#define DEBOUNCE_TIME_MS               30
#define HEARTBEAT_INTERVAL_MS          1000
#define HEARTBEAT_LOG_INTERVAL         10
#define LONG_PRESS_TIME_MS             2000
#define DOUBLE_CLICK_TIME_MS           400
#define DOUBLE_CLICK_BLINK_DELAY_MS    100
#define DOUBLE_CLICK_BLINK_COUNT       2
#define HEARTBEAT_ENABLED              0
#define ACCEL_SAMPLE_INTERVAL_MS       10
#define ACCEL_BLE_REPORT_INTERVAL_MS   500
#define ACCEL_WAKE_THRESHOLD_MG        250U
#define ACCEL_SETTLING_SAMPLE_COUNT    4U
#define ACCEL_WINDOW_SAMPLE_COUNT      100
#define ACCEL_BLE_REPORT_SAMPLE_COUNT \
	(ACCEL_BLE_REPORT_INTERVAL_MS / ACCEL_SAMPLE_INTERVAL_MS)

/* Event queue configuration */
#define APP_EVENT_QUEUE_SIZE   32
#define APP_EVENT_QUEUE_ALIGN  4
#define APP_EVENT_VALUE_UNUSED 0

/* Persistent settings */
#define APP_SETTINGS_SUBTREE "app"
#define APP_SETTINGS_MODE_KEY "mode"

enum app_event_type {
	APP_EVENT_BUTTON_PRESSED,
	APP_EVENT_BUTTON_RELEASED,
	APP_EVENT_BUTTON_SHORT_PRESS,
	APP_EVENT_BUTTON_SINGLE_CLICK,
	APP_EVENT_BUTTON_DOUBLE_CLICK,
	APP_EVENT_BUTTON_LONG_PRESS,
	APP_EVENT_HEARTBEAT,
	APP_EVENT_ACCEL_SAMPLE,
	APP_EVENT_SHAKE_DETECTED,
	APP_EVENT_STATUS_REPORT,
	APP_EVENT_RESET_COUNTERS,
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

struct accel_window {
	int16_t x_mg[ACCEL_WINDOW_SAMPLE_COUNT];
	int16_t y_mg[ACCEL_WINDOW_SAMPLE_COUNT];
	int16_t z_mg[ACCEL_WINDOW_SAMPLE_COUNT];
	uint16_t sample_index;
	uint32_t completed_window_count;
};

struct vibration_result {
	int16_t mean_x_mg;
	int16_t mean_y_mg;
	int16_t mean_z_mg;
	uint16_t rms_mg;
	uint16_t peak_mg;
};

struct app_state {
	uint32_t single_click_count;
	uint32_t heartbeat_count;
	uint32_t last_status_uptime_seconds;

	uint32_t button_press_start_ms;
	int last_button_state;
	bool button_is_pressed;

	bool short_press_pending;
	uint32_t first_short_press_time_ms;
	bool second_click_in_progress;
	uint32_t double_click_blink_remaining;

	uint16_t accel_ble_sample_counter;
	uint8_t accel_settling_samples_remaining;
	struct accel_window accel_window;
	struct vibration_result vibration;

	enum app_runtime_state runtime_state;
	enum app_wake_reason last_wake_reason;
	uint32_t state_transition_count;
	enum app_mode mode;
};

/* Hardware */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback sw0_cb_data;

/* Application state */
static struct app_state app;

static const char *app_mode_name(enum app_mode mode);
static const char *app_runtime_state_name(enum app_runtime_state state);
static const char *app_wake_reason_name(enum app_wake_reason reason);
static void app_set_runtime_state(enum app_runtime_state new_state);
static uint32_t app_uptime_seconds(void);

static int app_settings_set(const char *name,
				size_t len,
				settings_read_cb read_cb,
				void *cb_arg)
{
	int ret;
	uint8_t saved_mode;

	if (strcmp(name, APP_SETTINGS_MODE_KEY) != 0) {
		return -ENOENT;
	}

	if (len != sizeof(saved_mode)) {
		LOG_WRN("Invalid saved application mode length: %u",
			(unsigned int)len);
		return -EINVAL;
	}

	ret = read_cb(cb_arg, &saved_mode, sizeof(saved_mode));

	if (ret < 0) {
		LOG_ERR("Failed to read saved application mode: %d", ret);
		return ret;
	}

	if (ret != sizeof(saved_mode)) {
		LOG_WRN("Incomplete application mode setting: %d", ret);
		return -EINVAL;
	}

	if (saved_mode > APP_MODE_DIAGNOSTIC) {
		LOG_WRN("Invalid saved application mode: %u", saved_mode);
		return -EINVAL;
	}

	app.mode = (enum app_mode)saved_mode;

	LOG_INF("Application mode restored: %s",
		app_mode_name(app.mode));

	return 0;
}

/* Forward declarations */
static void heartbeat_timer_handler(struct k_timer *timer);
static void accel_sample_timer_handler(struct k_timer *timer);
static void single_click_timer_handler(struct k_timer *timer);
static void button_debounce_handler(struct k_work *work);
static void double_click_blink_handler(struct k_work *work);
static void app_save_mode_work_handler(struct k_work *work);

static void led0_toggle(void);
static void led1_toggle(void);
static int app_save_mode(void);

static void app_accel_sampling_start(void);
static void app_accel_sampling_stop(void);
static void app_accel_window_reset(void);
static void app_accel_motion_detected(void);
static void app_enter_idle(void);
static void app_start_measurement(enum app_wake_reason reason);
static void app_finish_measurement(void);
static void app_apply_mode_runtime(enum app_wake_reason reason);
static void app_handle_mode_request(enum app_mode requested_mode);

SETTINGS_STATIC_HANDLER_DEFINE(app_settings,
			       APP_SETTINGS_SUBTREE,
			       NULL,
			       app_settings_set,
			       NULL,
			       NULL);

/* Kernel objects */
K_MSGQ_DEFINE(app_event_msgq,
	      sizeof(struct app_event),
	      APP_EVENT_QUEUE_SIZE,
	      APP_EVENT_QUEUE_ALIGN);

K_TIMER_DEFINE(heartbeat_timer, heartbeat_timer_handler, NULL);
K_TIMER_DEFINE(single_click_timer, single_click_timer_handler, NULL);
K_TIMER_DEFINE(accel_sample_timer,
	       accel_sample_timer_handler,
	       NULL);
K_WORK_DEFINE(app_save_mode_work, app_save_mode_work_handler);

K_WORK_DELAYABLE_DEFINE(button_debounce_work, button_debounce_handler);
K_WORK_DELAYABLE_DEFINE(double_click_blink_work, double_click_blink_handler);

BUILD_ASSERT(
	ACCEL_BLE_REPORT_INTERVAL_MS %
	ACCEL_SAMPLE_INTERVAL_MS == 0,
	"BLE report interval must be divisible by sample interval");

BUILD_ASSERT(
	ACCEL_WINDOW_SAMPLE_COUNT *
	ACCEL_SAMPLE_INTERVAL_MS == 1000,
	"Accelerometer window must currently equal one second");


static const char *app_event_type_name(enum app_event_type type)
{
	switch (type) {
	case APP_EVENT_BUTTON_PRESSED:
		return "APP_EVENT_BUTTON_PRESSED";

	case APP_EVENT_BUTTON_RELEASED:
		return "APP_EVENT_BUTTON_RELEASED";

	case APP_EVENT_BUTTON_SHORT_PRESS:
		return "APP_EVENT_BUTTON_SHORT_PRESS";

	case APP_EVENT_BUTTON_SINGLE_CLICK:
		return "APP_EVENT_BUTTON_SINGLE_CLICK";

	case APP_EVENT_BUTTON_DOUBLE_CLICK:
		return "APP_EVENT_BUTTON_DOUBLE_CLICK";

	case APP_EVENT_BUTTON_LONG_PRESS:
		return "APP_EVENT_BUTTON_LONG_PRESS";

	case APP_EVENT_HEARTBEAT:
		return "APP_EVENT_HEARTBEAT";

	case APP_EVENT_ACCEL_SAMPLE:
		return "APP_EVENT_ACCEL_SAMPLE";

	case APP_EVENT_STATUS_REPORT:
		return "APP_EVENT_STATUS_REPORT";

	case APP_EVENT_RESET_COUNTERS:
		return "APP_EVENT_RESET_COUNTERS";

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

	case APP_EVENT_SHAKE_DETECTED:
		return "APP_EVENT_SHAKE_DETECTED";

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

static void app_set_runtime_state(enum app_runtime_state new_state)
{
	enum app_runtime_state previous_state;

	if ((new_state < APP_STATE_BOOT) ||
	    (new_state > APP_STATE_ERROR)) {
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

	int ret = k_msgq_put(&app_event_msgq, &event, K_NO_WAIT);

	if (ret < 0) {
		LOG_ERR("Failed to post %s: %d",
			app_event_type_name(type),
			ret);
		return;
	}

	if ((type != APP_EVENT_HEARTBEAT) &&
	    (type != APP_EVENT_ACCEL_SAMPLE)) {
		LOG_DBG("Event posted: %s", app_event_type_name(type));
	}
}

static void app_post_button_pressed_event(uint32_t button_state)
{
	app_post_event(APP_EVENT_BUTTON_PRESSED, button_state);
}

static void app_post_button_released_event(uint32_t button_state)
{
	app_post_event(APP_EVENT_BUTTON_RELEASED, button_state);
}

static void app_post_button_short_press_event(uint32_t held_time_ms)
{
	app_post_event(APP_EVENT_BUTTON_SHORT_PRESS, held_time_ms);
}

static void app_post_button_single_click_event(uint32_t delay_ms)
{
	app_post_event(APP_EVENT_BUTTON_SINGLE_CLICK, delay_ms);
}

static void app_post_button_double_click_event(uint32_t interval_ms)
{
	app_post_event(APP_EVENT_BUTTON_DOUBLE_CLICK, interval_ms);
}

static void app_post_button_long_press_event(uint32_t held_time_ms)
{
	app_post_event(APP_EVENT_BUTTON_LONG_PRESS, held_time_ms);
}

static void app_post_heartbeat_event(void)
{
	app_post_event(APP_EVENT_HEARTBEAT, APP_EVENT_VALUE_UNUSED);
}

static void app_post_accel_sample_event(void)
{
	app_post_event(APP_EVENT_ACCEL_SAMPLE, APP_EVENT_VALUE_UNUSED);
}

static void app_post_status_report_event(uint32_t uptime_seconds)
{
	app_post_event(APP_EVENT_STATUS_REPORT, uptime_seconds);
}

static void app_post_reset_counters_event(void)
{
	app_post_event(APP_EVENT_RESET_COUNTERS, APP_EVENT_VALUE_UNUSED);
}

static void app_nfc_mode_command_received(enum app_mode mode)
{
	app_post_event(APP_EVENT_SET_MODE, (uint32_t)mode);
}

static void app_ble_mode_requested(enum app_mode mode)
{
	app_post_event(APP_EVENT_SET_MODE, (uint32_t)mode);
}

static void app_ble_command_received(enum app_ble_command command)
{
	switch (command) {
	case APP_BLE_COMMAND_STATUS_REPORT:
		app_post_event(APP_EVENT_STATUS_REPORT, app_uptime_seconds());
		break;

	case APP_BLE_COMMAND_RESET_COUNTERS:
		app_post_event(APP_EVENT_RESET_COUNTERS, APP_EVENT_VALUE_UNUSED);
		break;

	default:
		LOG_WRN("Unknown BLE command: %d", command);
		break;
	}
}

static uint32_t app_ble_click_count_get(void)
{
	return app.single_click_count;
}

static enum app_mode app_ble_mode_get(void)
{
	return app.mode;
}

static bool app_ble_button_pressed_get(void)
{
	return app.button_is_pressed;
}

static uint32_t app_ble_uptime_seconds_get(void)
{
	return app_uptime_seconds();
}

static const struct app_ble_callbacks ble_callbacks = {
	.mode_requested = app_ble_mode_requested,
	.command_received = app_ble_command_received,
	.click_count_get = app_ble_click_count_get,
	.mode_get = app_ble_mode_get,
	.button_pressed_get = app_ble_button_pressed_get,
	.uptime_seconds_get = app_ble_uptime_seconds_get,
};

static void app_post_nfc_data_read_event(void)
{
	app_post_event(APP_EVENT_NFC_DATA_READ,
				APP_EVENT_VALUE_UNUSED);
}

static void led0_toggle(void)
{
	int ret = gpio_pin_toggle_dt(&led0);

	if (ret < 0) {
		LOG_ERR("Failed to toggle LED0: %d", ret);
	}
}

static void led1_toggle(void)
{
	int ret = gpio_pin_toggle_dt(&led1);

	if (ret < 0) {
		LOG_ERR("Failed to toggle LED1: %d", ret);
	}
}

static void led0_start_double_click_blink(void)
{
	app.double_click_blink_remaining = DOUBLE_CLICK_BLINK_COUNT;

	int ret = k_work_schedule(&double_click_blink_work, K_NO_WAIT);

	if (ret < 0) {
		LOG_ERR("Failed to schedule double-click blink work: %d", ret);
	}
}

static void app_save_mode_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	(void)app_save_mode();
}

static void app_accel_motion_detected(void)
{
	app_post_event(APP_EVENT_SHAKE_DETECTED, APP_EVENT_VALUE_UNUSED);
}

static int led_init(void)
{
	int ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		LOG_ERR("Failed to configure LED0: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		LOG_ERR("Failed to configure LED1: %d", ret);
		return ret;
	}

	return 0;
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
		app_post_nfc_data_read_event();
		break;

	case NFC_T4T_EVENT_NDEF_UPDATED:
		LOG_INF("NFC NDEF message updated: %u bytes",
			(unsigned int)data_length);

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
	int err = nfc_t4t_setup(app_nfc_callback, NULL);

	if (err < 0) {
		LOG_ERR("Failed to set up NFC Type 4 Tag: %d", err);
		return err;
	}

	err = app_nfc_start(app.mode,
		    app.single_click_count,
		    app_nfc_mode_command_received);

	if (err < 0) {
		LOG_ERR("Failed to start NFC payload: %d", err);
		return err;
	}

	LOG_INF("NFC initialized.");

	return 0;
}


static int app_save_mode(void)
{
	uint8_t saved_mode = (uint8_t)app.mode;
	int ret;

	ret = settings_save_one("app/mode",
				&saved_mode,
				sizeof(saved_mode));

	if (ret < 0) {
		LOG_ERR("Failed to save application mode: %d", ret);
		return ret;
	}

	LOG_INF("Application mode saved: %s",
		app_mode_name(app.mode));

	return 0;
}


static void heartbeat_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	app_post_heartbeat_event();
}

static void accel_sample_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	app_post_accel_sample_event();
}

static void single_click_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	if (!app.short_press_pending) {
		return;
	}

	if (app.second_click_in_progress) {
		LOG_DBG("Single-click timer expired while second click was in "
			"progress; "
			"waiting for release.");
		return;
	}

	app.short_press_pending = false;
	app_post_button_single_click_event(DOUBLE_CLICK_TIME_MS);
}

static void double_click_blink_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (app.double_click_blink_remaining == 0U) {
		return;
	}

	led0_toggle();
	app.double_click_blink_remaining--;

	if (app.double_click_blink_remaining > 0U) {
		int ret = k_work_schedule(&double_click_blink_work,
					  K_MSEC(DOUBLE_CLICK_BLINK_DELAY_MS));

		if (ret < 0) {
			LOG_ERR("Failed to reschedule double-click blink work: "
				"%d",
				ret);
		}
	}
}

static void button_debounce_handler(struct k_work *work)
{
	int button_state;
	int ret;

	ARG_UNUSED(work);

	button_state = gpio_pin_get_dt(&sw0);
	if (button_state < 0) {
		LOG_ERR("Failed to read SW0 during debounce: %d", button_state);
		goto reenable_interrupt;
	}

	LOG_DBG("Debounced button state: %d", button_state);

	if (button_state == app.last_button_state) {
		LOG_DBG("Ignoring duplicate debounced state: %d", button_state);
		goto reenable_interrupt;
	}

	app.last_button_state = button_state;

	if (button_state > 0) {
		app_post_button_pressed_event((uint32_t)button_state);
	} else {
		app_post_button_released_event((uint32_t)button_state);
	}

reenable_interrupt:
	ret = gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to re-enable SW0 interrupt: %d", ret);
		return;
	}

	LOG_DBG("Button interrupt re-enabled.");
}

static void sw0_pressed_callback(const struct device *dev,
				 struct gpio_callback *cb,
				 uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	int ret = gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_DISABLE);

	if (ret < 0) {
		LOG_ERR("Failed to disable SW0 interrupt: %d", ret);
		return;
	}

	LOG_DBG("Button interrupt fired; starting debounce work.");

	ret = k_work_schedule(&button_debounce_work, K_MSEC(DEBOUNCE_TIME_MS));

	if (ret < 0) {
		LOG_ERR("Failed to schedule debounce work: %d", ret);
	}
}

static int app_gpio_init(void)
{
	if (!gpio_is_ready_dt(&led0)) {
		LOG_ERR("LED0 GPIO device is not ready.");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&led1)) {
		LOG_ERR("LED1 GPIO device is not ready.");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&sw0)) {
		LOG_ERR("SW0 GPIO device is not ready.");
		return -ENODEV;
	}

	int ret = led_init();

	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&sw0, GPIO_INPUT);

	if (ret < 0) {
		LOG_ERR("Failed to configure SW0: %d", ret);
		return ret;
	}

	gpio_init_callback(&sw0_cb_data, sw0_pressed_callback, BIT(sw0.pin));

	ret = gpio_add_callback(sw0.port, &sw0_cb_data);

	if (ret < 0) {
		LOG_ERR("Failed to add SW0 callback: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_BOTH);

	if (ret < 0) {
		LOG_ERR("Failed to configure SW0 interrupt: %d", ret);
		return ret;
	}

	return 0;
}

static void app_timers_start(void)
{
	if (HEARTBEAT_ENABLED) {
		k_timer_start(&heartbeat_timer,
			      K_MSEC(HEARTBEAT_INTERVAL_MS),
			      K_MSEC(HEARTBEAT_INTERVAL_MS));
	}
}

static int app_init(void)
{
	int ret;

	app.mode = APP_MODE_NORMAL;
	app.runtime_state = APP_STATE_BOOT;
	app.last_wake_reason = APP_WAKE_REASON_BOOT;
	app.state_transition_count = 0U;

	ret = app_gpio_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize GPIOs: %d", ret);
		return ret;
	}

	ret = gpio_pin_get_dt(&sw0);
	if (ret < 0) {
		LOG_ERR("Failed to read initial SW0 state: %d", ret);
		return ret;
	}

	app.last_button_state = ret;
	LOG_INF("Initial button state: %d", app.last_button_state);

	ret = sensor_accel_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize accelerometer: %d", ret);
		return ret;
	}

	ret = sensor_accel_motion_init(app_accel_motion_detected,
				       ACCEL_WAKE_THRESHOLD_MG);
	if (ret < 0) {
		LOG_ERR("Failed to initialize accelerometer motion detection: %d",
			ret);
		return ret;
	}

	/* app_ble_start() loads Bluetooth and application settings. */
	ret = app_ble_start(&ble_callbacks);
	if (ret < 0) {
		LOG_ERR("Failed to initialize Bluetooth module: %d", ret);
		return ret;
	}

	ret = app_nfc_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize NFC: %d", ret);
		return ret;
	}

	/* Apply the mode restored by settings_load(). */
	app_apply_mode_runtime(APP_WAKE_REASON_BOOT);
	if (app.runtime_state == APP_STATE_ERROR) {
		return -EIO;
	}

	app_timers_start();
	return 0;
}

static void app_log_startup_config(void)
{
	LOG_INF("Heartbeat: %d ms. Debounce: %d ms. Queue size: %d. "
		"Status interval: %d heartbeats. Long press: %d ms. "
		"Double-click window: %d ms.",
		HEARTBEAT_INTERVAL_MS,
		DEBOUNCE_TIME_MS,
		APP_EVENT_QUEUE_SIZE,
		HEARTBEAT_LOG_INTERVAL,
		LONG_PRESS_TIME_MS,
		DOUBLE_CLICK_TIME_MS);

	LOG_INF("Application mode: %s", app_mode_name(app.mode));
	LOG_INF("Heartbeat enabled: %s", HEARTBEAT_ENABLED ? "yes" : "no");
	LOG_INF("Accelerometer sample interval: %d ms",
		ACCEL_SAMPLE_INTERVAL_MS);
	LOG_INF("Runtime state: %s",
		app_runtime_state_name(app.runtime_state));

	LOG_INF("Last wake reason: %s",
		app_wake_reason_name(app.last_wake_reason));
}

static void app_mode_next(void)
{
	enum app_mode next_mode;

	switch (app.mode) {
	case APP_MODE_NORMAL:
		next_mode = APP_MODE_CONFIG;
		break;

	case APP_MODE_CONFIG:
		next_mode = APP_MODE_DIAGNOSTIC;
		break;

	case APP_MODE_DIAGNOSTIC:
	default:
		next_mode = APP_MODE_NORMAL;
		break;
	}

	app_handle_mode_request(next_mode);
}

static void app_handle_single_click(void)
{
	switch (app.mode) {
	case APP_MODE_NORMAL:
		app.single_click_count++;
		led0_toggle();

		(void)app_ble_notify_click_count(app.single_click_count);
		app_nfc_update_request(app.mode, app.single_click_count);

		LOG_INF("Single click handled. Count: %u",
			app.single_click_count);
		break;

	case APP_MODE_CONFIG:
		LOG_INF("Single click handled in CONFIG mode. Count: %u",
			app.single_click_count);
		break;

	case APP_MODE_DIAGNOSTIC:
		LOG_INF("Single click handled in DIAGNOSTIC mode. Count: %u",
			app.single_click_count);
		break;

	default:
		LOG_WRN("Single click handled in unknown mode. Count: %u",
			app.single_click_count);
		break;
	}
}

static void app_handle_button_release(void)
{
	if (!app.button_is_pressed) {
		LOG_WRN("Release event received while button was not marked "
			"pressed.");
		return;
	}

	uint32_t held_time_ms = k_uptime_get_32() - app.button_press_start_ms;

	app.button_is_pressed = false;

	if (held_time_ms >= LONG_PRESS_TIME_MS) {
		LOG_INF("Button released after long press: %u ms",
			held_time_ms);
		app_post_button_long_press_event(held_time_ms);
	} else {
		LOG_INF("Button released after short press: %u ms",
			held_time_ms);
		app_post_button_short_press_event(held_time_ms);
	}
}

static void app_handle_short_press(const struct app_event *event)
{
	if (app.short_press_pending) {
		uint32_t interval_ms =
			k_uptime_get_32() - app.first_short_press_time_ms;

		app.short_press_pending = false;
		app.second_click_in_progress = false;

		k_timer_stop(&single_click_timer);
		app_post_button_double_click_event(interval_ms);
		return;
	}

	app.short_press_pending = true;
	app.second_click_in_progress = false;
	app.first_short_press_time_ms = k_uptime_get_32();

	k_timer_start(
		&single_click_timer, K_MSEC(DOUBLE_CLICK_TIME_MS), K_NO_WAIT);

	LOG_INF("Short press handled. Held: %u ms. "
		"Waiting for possible double click.",
		event->value);
}

static void app_accel_window_reset(void)
{
	app.accel_window.sample_index = 0U;
	app.accel_ble_sample_counter = 0U;
	app.accel_settling_samples_remaining = 0U;
}

static void app_accel_sampling_start(void)
{
	k_timer_start(
		&accel_sample_timer,
		K_MSEC(ACCEL_SAMPLE_INTERVAL_MS),
		K_MSEC(ACCEL_SAMPLE_INTERVAL_MS));
}

static void app_accel_sampling_stop(void)
{
	k_timer_stop(&accel_sample_timer);
}

static void app_enter_idle(void)
{
	int ret;

	app_accel_sampling_stop();

	/* Every future wake begins with a completely new window. */
	app_accel_window_reset();

	/*
	 * Ensure the interrupt route is disabled before changing
	 * the accelerometer operating profile.
	 */
	ret = sensor_accel_motion_disarm();

	if (ret < 0) {
		LOG_ERR("Failed to disarm motion detection "
			"while entering idle: %d",
			ret);

		app_set_runtime_state(APP_STATE_ERROR);
		return;
	}

	switch (app.mode) {
	case APP_MODE_NORMAL:
		/*
		 * NORMAL idle:
		 * low-rate accelerometer operation with motion wake.
		 */
		ret = sensor_accel_profile_set(
			SENSOR_ACCEL_PROFILE_MOTION);

		if (ret < 0) {
			LOG_ERR("Failed to apply motion profile: %d",
				ret);

			app_set_runtime_state(APP_STATE_ERROR);
			return;
		}

		/*
		 * Set IDLE before arming the interrupt. If movement is
		 * already present and the trigger fires immediately,
		 * the queued shake event will be accepted.
		 */
		app_set_runtime_state(APP_STATE_IDLE);

		ret = sensor_accel_motion_arm();

		if (ret < 0) {
			LOG_ERR("Failed to arm shake detection: %d",
				ret);

			app_set_runtime_state(APP_STATE_ERROR);
			return;
		}

		break;

	case APP_MODE_CONFIG:
		/*
		 * CONFIG does not require shake detection, so the
		 * accelerometer can enter power-down.
		 */
		ret = sensor_accel_profile_set(
			SENSOR_ACCEL_PROFILE_OFF);

		if (ret < 0) {
			LOG_ERR("Failed to power down accelerometer: %d",
				ret);

			app_set_runtime_state(APP_STATE_ERROR);
			return;
		}

		app_set_runtime_state(APP_STATE_IDLE);
		break;

	default:
		LOG_ERR("Cannot enter idle from application mode %d",
			app.mode);

		app_set_runtime_state(APP_STATE_ERROR);
		break;
	}
}

static void app_start_measurement(enum app_wake_reason reason)
{
	int ret;

	if (app.runtime_state == APP_STATE_ERROR) {
		LOG_WRN("Measurement start rejected while "
			"in ERROR state");
		return;
	}

	if (app.runtime_state == APP_STATE_MEASURING) {
		LOG_DBG("Measurement already active");
		return;
	}

	/*
	 * Prevent repeated wake interrupts while the measurement
	 * window is being collected.
	 */
	ret = sensor_accel_motion_disarm();

	if (ret < 0) {
		LOG_ERR("Failed to disarm motion detection "
			"before measurement: %d",
			ret);

		app_set_runtime_state(APP_STATE_ERROR);
		return;
	}

	ret = sensor_accel_profile_set(
		SENSOR_ACCEL_PROFILE_MEASUREMENT);

	if (ret < 0) {
		LOG_ERR("Failed to apply accelerometer "
			"measurement profile: %d",
			ret);

		app_set_runtime_state(APP_STATE_ERROR);
		return;
	}

	if (reason != APP_WAKE_REASON_NONE) {
		app.last_wake_reason = reason;
	}

	app_accel_window_reset();
	app.accel_settling_samples_remaining =
		ACCEL_SETTLING_SAMPLE_COUNT;

	app_set_runtime_state(APP_STATE_MEASURING);

	app_accel_sampling_start();

	LOG_INF("Accelerometer measurement started. Reason: %s",
		app_wake_reason_name(app.last_wake_reason));
}

static void app_finish_measurement(void)
{
	bool continuous_measurement =
		(app.mode == APP_MODE_DIAGNOSTIC);
	int ret;

	if (!continuous_measurement) {
		app_accel_sampling_stop();
	}

	app_set_runtime_state(APP_STATE_REPORTING);

	ret = app_ble_update_vibration(
		app.vibration.mean_x_mg,
		app.vibration.mean_y_mg,
		app.vibration.mean_z_mg,
		app.vibration.rms_mg,
		app.vibration.peak_mg,
		(uint16_t)app.accel_window.completed_window_count);

	if ((ret < 0) && (ret != -ENOTCONN) && (ret != -EACCES)) {
		LOG_WRN("Failed to update BLE vibration result: %d", ret);
	}

	if (continuous_measurement) {
		app_set_runtime_state(APP_STATE_MEASURING);
		return;
	}

	app_enter_idle();
}

static bool app_accel_window_add_sample(
	const struct sensor_accel_sample *sample)
{
	uint16_t index;

	if (sample == NULL) {
		return false;
	}

	index = app.accel_window.sample_index;
	if (index >= ACCEL_WINDOW_SAMPLE_COUNT) {
		LOG_ERR("Accelerometer window index out of range: %u", index);
		app.accel_window.sample_index = 0U;
		index = 0U;
	}

	app.accel_window.x_mg[index] = (int16_t)sample->x_mg;
	app.accel_window.y_mg[index] = (int16_t)sample->y_mg;
	app.accel_window.z_mg[index] = (int16_t)sample->z_mg;

	app.accel_window.sample_index++;
	if (app.accel_window.sample_index < ACCEL_WINDOW_SAMPLE_COUNT) {
		return false;
	}

	app.accel_window.sample_index = 0U;
	app.accel_window.completed_window_count++;
	return true;
}

static uint32_t app_integer_sqrt(uint64_t value)
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

static void app_accel_window_calculate(void)
{
	int64_t sum_x = 0;
	int64_t sum_y = 0;
	int64_t sum_z = 0;

	for (uint16_t i = 0U;
	     i < ACCEL_WINDOW_SAMPLE_COUNT;
	     i++) {

		sum_x += app.accel_window.x_mg[i];
		sum_y += app.accel_window.y_mg[i];
		sum_z += app.accel_window.z_mg[i];
	}

	int32_t mean_x =
		(int32_t)(sum_x / ACCEL_WINDOW_SAMPLE_COUNT);

	int32_t mean_y =
		(int32_t)(sum_y / ACCEL_WINDOW_SAMPLE_COUNT);

	int32_t mean_z =
		(int32_t)(sum_z / ACCEL_WINDOW_SAMPLE_COUNT);

	uint64_t sum_magnitude_squared = 0U;
	uint64_t peak_magnitude_squared = 0U;

	for (uint16_t i = 0U;
	     i < ACCEL_WINDOW_SAMPLE_COUNT;
	     i++) {

		int32_t dynamic_x =
			(int32_t)app.accel_window.x_mg[i] - mean_x;

		int32_t dynamic_y =
			(int32_t)app.accel_window.y_mg[i] - mean_y;

		int32_t dynamic_z =
			(int32_t)app.accel_window.z_mg[i] - mean_z;

		uint64_t magnitude_squared =
			(uint64_t)((int64_t)dynamic_x * dynamic_x) +
			(uint64_t)((int64_t)dynamic_y * dynamic_y) +
			(uint64_t)((int64_t)dynamic_z * dynamic_z);

		sum_magnitude_squared += magnitude_squared;

		if (magnitude_squared >
		    peak_magnitude_squared) {
			peak_magnitude_squared =
				magnitude_squared;
		}
	}

	uint64_t mean_magnitude_squared =
		sum_magnitude_squared /
		ACCEL_WINDOW_SAMPLE_COUNT;

	app.vibration.mean_x_mg = (int16_t)mean_x;
	app.vibration.mean_y_mg = (int16_t)mean_y;
	app.vibration.mean_z_mg = (int16_t)mean_z;

	app.vibration.rms_mg =
		(uint16_t)app_integer_sqrt(
			mean_magnitude_squared);

	app.vibration.peak_mg =
		(uint16_t)app_integer_sqrt(
			peak_magnitude_squared);
}

static void app_apply_mode_runtime(enum app_wake_reason reason)
{
	switch (app.mode) {
	case APP_MODE_DIAGNOSTIC:
		app_start_measurement(reason);
		break;

	case APP_MODE_NORMAL:
	case APP_MODE_CONFIG:
		app_enter_idle();
		break;

	default:
		LOG_ERR("Cannot apply unknown application mode: %d", app.mode);
		app_set_runtime_state(APP_STATE_ERROR);
		break;
	}
}


static void app_handle_accel_sample(void)
{
	struct sensor_accel_sample sample;
	bool window_complete;
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

	/*
	 * Discard the first samples after changing from the motion profile
	 * to the 100 Hz measurement profile. This keeps profile-transition
	 * data out of the vibration window.
	 */
	if (app.accel_settling_samples_remaining > 0U) {
		app.accel_settling_samples_remaining--;
		return;
	}

	window_complete = app_accel_window_add_sample(&sample);
	app.accel_ble_sample_counter++;

	if (app.accel_ble_sample_counter >=
	    ACCEL_BLE_REPORT_SAMPLE_COUNT) {
		app.accel_ble_sample_counter = 0U;

		(void)app_ble_update_accel((int16_t)sample.x_mg,
					   (int16_t)sample.y_mg,
					   (int16_t)sample.z_mg);
	}

	if (!window_complete) {
		return;
	}

	app_accel_window_calculate();

	LOG_INF("Window %u: mean=(%d, %d, %d) mg, "
		"RMS=%u mg, peak=%u mg",
		app.accel_window.completed_window_count,
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

	if ((requested_mode < APP_MODE_NORMAL) ||
	    (requested_mode > APP_MODE_DIAGNOSTIC)) {
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

	ret = k_work_submit(&app_save_mode_work);
	if (ret < 0) {
		LOG_ERR("Failed to submit application mode save work: %d", ret);
	}

	(void)app_ble_notify_mode(app.mode);
	app_nfc_update_request(app.mode, app.single_click_count);
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

static void app_event_handler(const struct app_event *event)
{
	switch (event->type) {
	case APP_EVENT_BUTTON_PRESSED:
		app.button_press_start_ms = k_uptime_get_32();
		app.button_is_pressed = true;

		if (app.short_press_pending) {
			app.second_click_in_progress = true;
			LOG_INF("Second click started within double-click "
				"window.");
		}
		break;

	case APP_EVENT_BUTTON_RELEASED:
		app_handle_button_release();
		break;

	case APP_EVENT_BUTTON_SHORT_PRESS:
		app_handle_short_press(event);
		break;

	case APP_EVENT_BUTTON_SINGLE_CLICK:
		app_handle_single_click();
		break;

	case APP_EVENT_BUTTON_DOUBLE_CLICK:
		LOG_INF("Double click handled. Interval: %u ms", event->value);

		app_mode_next();
		led0_start_double_click_blink();
		break;

	case APP_EVENT_BUTTON_LONG_PRESS:
		LOG_INF("Long press handled. Held: %u ms", event->value);
		app_post_reset_counters_event();
		break;

	case APP_EVENT_HEARTBEAT: {
		app.heartbeat_count++;

		uint32_t uptime_seconds = app_uptime_seconds();

		led1_toggle();

		if ((app.heartbeat_count % HEARTBEAT_LOG_INTERVAL) == 0U) {
			app_post_status_report_event(uptime_seconds);
		}
		break;
	}

	case APP_EVENT_ACCEL_SAMPLE:
		app_handle_accel_sample();
		break;

	case APP_EVENT_STATUS_REPORT: {
		app.last_status_uptime_seconds = event->value;

		uint32_t current_uptime_seconds = app_uptime_seconds();

		LOG_INF("Status report: mode: %s, single click count: %u, "
			"heartbeat count: %u, uptime at request: %u s, "
			"current uptime: %u s",
			app_mode_name(app.mode),
			app.single_click_count,
			app.heartbeat_count,
			app.last_status_uptime_seconds,
			current_uptime_seconds);
		break;
	}

	case APP_EVENT_RESET_COUNTERS: {
		uint32_t current_uptime_seconds;

		app.single_click_count = 0U;
		app.heartbeat_count = 0U;

		app_nfc_update_request(app.mode, app.single_click_count);

		current_uptime_seconds = app_uptime_seconds();

		LOG_INF("Resettable counters cleared. Uptime remains: %u s",
			current_uptime_seconds);
		break;
	}

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
		LOG_INF("NFC write received: %u-byte NDEF message",
			event->value);
		break;

	case APP_EVENT_SHAKE_DETECTED:
		app_handle_shake_detected();
		break;

	default:
		LOG_WRN("Unknown event type: %d (%s)",
			event->type,
			app_event_type_name(event->type));
		break;
	}
}

static void app_event_loop(void)
{
	while (true) {
		struct app_event event;

		int ret = k_msgq_get(&app_event_msgq, &event, K_FOREVER);

		if (ret < 0) {
			LOG_ERR("Failed to get event from message queue: %d",
				ret);
			continue;
		}

		app_event_handler(&event);
	}
}

int main(void)
{
	LOG_INF("Application started.");

	int ret = app_init();

	if (ret < 0) {
		LOG_ERR("Failed to initialize application: %d", ret);
		return ret;
	}

	app_log_startup_config();

	LOG_INF("Setup complete. Waiting for events.");

	app_event_loop();

	return 0;
}
