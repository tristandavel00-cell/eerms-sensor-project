#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "app_ble.h"

LOG_MODULE_REGISTER(app_ble, LOG_LEVEL_DBG);

#define APP_STATUS_PACKET_SIZE	      12
#define APP_STATUS_MODE_OFFSET	      0
#define APP_STATUS_BUTTON_OFFSET      1
#define APP_STATUS_CLICK_COUNT_OFFSET 4
#define APP_STATUS_UPTIME_OFFSET      8

#define BT_UUID_BUTTON_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_SINGLE_CLICK_COUNT_VAL \
	BT_UUID_128_ENCODE(0x12345679, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_APP_MODE_VAL \
	BT_UUID_128_ENCODE(0x1234567a, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_APP_STATUS_VAL \
	BT_UUID_128_ENCODE(0x1234567b, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_APP_COMMAND_VAL \
	BT_UUID_128_ENCODE(0x1234567c, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static const struct app_ble_callbacks *app_callbacks;
static struct bt_conn *current_conn;
static bool single_click_notifications_enabled;
static bool app_mode_notifications_enabled;

static struct bt_uuid_128 button_service_uuid =
	BT_UUID_INIT_128(BT_UUID_BUTTON_SERVICE_VAL);
static struct bt_uuid_128 single_click_count_uuid =
	BT_UUID_INIT_128(BT_UUID_SINGLE_CLICK_COUNT_VAL);
static struct bt_uuid_128 app_mode_uuid =
	BT_UUID_INIT_128(BT_UUID_APP_MODE_VAL);
static struct bt_uuid_128 app_status_uuid =
	BT_UUID_INIT_128(BT_UUID_APP_STATUS_VAL);
static struct bt_uuid_128 app_command_uuid =
	BT_UUID_INIT_128(BT_UUID_APP_COMMAND_VAL);

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BUTTON_SERVICE_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param advertising_params =
	BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN,
			     BT_GAP_ADV_FAST_INT_MIN_2,
			     BT_GAP_ADV_FAST_INT_MAX_2,
			     NULL);

static const struct bt_le_conn_param preferred_conn_params = {
	.interval_min = 80,
	.interval_max = 120,
	.latency = 4,
	.timeout = 400,
};

static ssize_t read_single_click_count(struct bt_conn *conn,
				       const struct bt_gatt_attr *attr,
				       void *buf,
				       uint16_t len,
				       uint16_t offset);
static ssize_t write_app_mode(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr,
			      const void *buf,
			      uint16_t len,
			      uint16_t offset,
			      uint8_t flags);
static ssize_t read_app_mode(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf,
			     uint16_t len,
			     uint16_t offset);
static ssize_t read_app_status(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       void *buf,
			       uint16_t len,
			       uint16_t offset);
static ssize_t write_app_command(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 const void *buf,
				 uint16_t len,
				 uint16_t offset,
				 uint8_t flags);

static void single_click_ccc_changed(const struct bt_gatt_attr *attr,
				     uint16_t value)
{
	ARG_UNUSED(attr);
	single_click_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Single-click notifications %s",
		single_click_notifications_enabled ? "enabled" : "disabled");
}

static void app_mode_ccc_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	ARG_UNUSED(attr);
	app_mode_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Application mode notifications %s",
		app_mode_notifications_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(
	button_service,
	BT_GATT_PRIMARY_SERVICE(&button_service_uuid.uuid),
	BT_GATT_CHARACTERISTIC(&single_click_count_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_single_click_count, NULL, NULL),
	BT_GATT_CCC(single_click_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&app_mode_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
				       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_app_mode, write_app_mode, NULL),
	BT_GATT_CCC(app_mode_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&app_status_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_app_status, NULL, NULL),
	BT_GATT_CHARACTERISTIC(&app_command_uuid.uuid,
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_app_command, NULL)
);

static int advertising_start(void)
{
	int err = bt_le_adv_start(&advertising_params,
				  advertising_data,
				  ARRAY_SIZE(advertising_data),
				  NULL, 0);
	if (err < 0) {
		LOG_ERR("Bluetooth advertising failed to start: %d", err);
		return err;
	}

	LOG_INF("Bluetooth advertising started as \"%s\"",
		CONFIG_BT_DEVICE_NAME);
	return 0;
}

static void advertising_restart_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int err = advertising_start();
	if (err < 0) {
		LOG_ERR("Failed to restart Bluetooth advertising: %d", err);
		return;
	}

	LOG_INF("Bluetooth advertising restarted");
}

K_WORK_DEFINE(advertising_restart_work, advertising_restart_handler);

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0U) {
		LOG_ERR("Bluetooth connection failed: %u", err);
		return;
	}

	if (current_conn != NULL) {
		bt_conn_unref(current_conn);
	}
	current_conn = bt_conn_ref(conn);

	int ret = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (ret < 0) {
		LOG_ERR("Failed to request Bluetooth security: %d", ret);
	} else {
		LOG_INF("Bluetooth security requested");
	}

	ret = bt_conn_le_param_update(conn, &preferred_conn_params);
	if (ret < 0) {
		LOG_ERR("Failed to request connection parameter update: %d", ret);
	} else {
		LOG_INF("Connection parameter update requested");
	}

	LOG_INF("Bluetooth device connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Bluetooth device disconnected. Reason: %u", reason);

	if (current_conn == conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	int ret = k_work_submit(&advertising_restart_work);
	if (ret < 0) {
		LOG_ERR("Failed to submit advertising restart work: %d", ret);
	}
}

static void le_param_updated(struct bt_conn *conn,
			     uint16_t interval,
			     uint16_t latency,
			     uint16_t timeout)
{
	ARG_UNUSED(conn);

	uint32_t interval_ms_x100 = (uint32_t)interval * 125U;
	uint32_t timeout_ms = (uint32_t)timeout * 10U;

	LOG_INF("Connection parameters updated: interval=%u.%02u ms, "
		"latency=%u, timeout=%u ms",
		interval_ms_x100 / 100U,
		interval_ms_x100 % 100U,
		latency,
		timeout_ms);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	ARG_UNUSED(conn);
	LOG_INF("Bluetooth pairing complete. Bonded: %s",
		bonded ? "yes" : "no");
}

static void pairing_failed(struct bt_conn *conn,
			   enum bt_security_err reason)
{
	ARG_UNUSED(conn);
	LOG_ERR("Bluetooth pairing failed. Reason: %d", reason);
}

static void security_changed(struct bt_conn *conn,
			     bt_security_t level,
			     enum bt_security_err err)
{
	ARG_UNUSED(conn);

	if (err != BT_SECURITY_ERR_SUCCESS) {
		LOG_ERR("Bluetooth security failed: level=%u, error=%d",
			level, err);
		return;
	}

	LOG_INF("Bluetooth security established at level %u", level);
}

static struct bt_conn_auth_info_cb auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

BT_CONN_CB_DEFINE(connection_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
	.security_changed = security_changed,
};

static ssize_t read_single_click_count(struct bt_conn *conn,
				       const struct bt_gatt_attr *attr,
				       void *buf,
				       uint16_t len,
				       uint16_t offset)
{
	uint32_t value = app_callbacks->click_count_get();
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &value, sizeof(value));
}

static ssize_t write_app_mode(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr,
			      const void *buf,
			      uint16_t len,
			      uint16_t offset,
			      uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t requested_mode = *((const uint8_t *)buf);
	if (requested_mode > APP_MODE_DIAGNOSTIC) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	app_callbacks->mode_requested((enum app_mode)requested_mode);
	LOG_INF("Application mode change requested over Bluetooth: %u",
		requested_mode);
	return len;
}

static ssize_t read_app_mode(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf,
			     uint16_t len,
			     uint16_t offset)
{
	uint8_t value = (uint8_t)app_callbacks->mode_get();
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &value, sizeof(value));
}

static ssize_t read_app_status(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       void *buf,
			       uint16_t len,
			       uint16_t offset)
{
	uint8_t packet[APP_STATUS_PACKET_SIZE] = {0};

	packet[APP_STATUS_MODE_OFFSET] = (uint8_t)app_callbacks->mode_get();
	packet[APP_STATUS_BUTTON_OFFSET] =
		app_callbacks->button_pressed_get() ? 1U : 0U;

	sys_put_le32(app_callbacks->click_count_get(),
		     &packet[APP_STATUS_CLICK_COUNT_OFFSET]);
	sys_put_le32(app_callbacks->uptime_seconds_get(),
		     &packet[APP_STATUS_UPTIME_OFFSET]);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 packet, sizeof(packet));
}

static ssize_t write_app_command(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 const void *buf,
				 uint16_t len,
				 uint16_t offset,
				 uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t value = *((const uint8_t *)buf);
	switch (value) {
	case APP_BLE_COMMAND_RESET_COUNTERS:
	case APP_BLE_COMMAND_STATUS_REPORT:
		app_callbacks->command_received((enum app_ble_command)value);
		LOG_INF("Application command received over Bluetooth: 0x%02x",
			value);
		return len;
	default:
		LOG_WRN("Unknown Bluetooth command: 0x%02x", value);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
}

int app_ble_start(const struct app_ble_callbacks *callbacks)
{
	if ((callbacks == NULL) ||
	    (callbacks->mode_requested == NULL) ||
	    (callbacks->command_received == NULL) ||
	    (callbacks->click_count_get == NULL) ||
	    (callbacks->mode_get == NULL) ||
	    (callbacks->button_pressed_get == NULL) ||
	    (callbacks->uptime_seconds_get == NULL)) {
		return -EINVAL;
	}

	app_callbacks = callbacks;

	int err = bt_enable(NULL);
	if (err < 0) {
		LOG_ERR("Bluetooth initialization failed: %d", err);
		return err;
	}
	LOG_INF("Bluetooth initialized");

	err = settings_load();
	if (err < 0) {
		LOG_ERR("Failed to load settings: %d", err);
		return err;
	}
	LOG_INF("Settings loaded");

	err = bt_conn_auth_info_cb_register(&auth_info_callbacks);
	if ((err < 0) && (err != -EALREADY)) {
		LOG_ERR("Failed to register authentication callbacks: %d", err);
		return err;
	}
	LOG_INF("Bluetooth authentication callbacks registered");

	return advertising_start();
}

int app_ble_notify_click_count(uint32_t click_count)
{
	if (current_conn == NULL) {
		LOG_DBG("Click notification skipped: no active connection");
		return -ENOTCONN;
	}
	if (!single_click_notifications_enabled) {
		LOG_DBG("Click notification skipped: notifications disabled");
		return -EACCES;
	}

	int err = bt_gatt_notify(current_conn, &button_service.attrs[2],
				 &click_count, sizeof(click_count));
	if (err < 0) {
		LOG_ERR("Failed to send click notification: %d", err);
		return err;
	}

	LOG_DBG("Click notification sent: %u", click_count);
	return 0;
}

int app_ble_notify_mode(enum app_mode mode)
{
	if (mode > APP_MODE_DIAGNOSTIC) {
		return -EINVAL;
	}
	if (current_conn == NULL) {
		LOG_DBG("Mode notification skipped: no active connection");
		return -ENOTCONN;
	}
	if (!app_mode_notifications_enabled) {
		LOG_DBG("Mode notification skipped: notifications disabled");
		return -EACCES;
	}

	uint8_t value = (uint8_t)mode;
	int err = bt_gatt_notify(current_conn, &button_service.attrs[5],
				 &value, sizeof(value));
	if (err < 0) {
		LOG_ERR("Failed to send mode notification: %d", err);
		return err;
	}

	LOG_DBG("Mode notification sent: %u", value);
	return 0;
}

bool app_ble_is_connected(void)
{
	return current_conn != NULL;
}