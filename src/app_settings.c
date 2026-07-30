#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>

#include "app_settings.h"

LOG_MODULE_REGISTER(app_settings, LOG_LEVEL_DBG);

#define APP_SETTINGS_SUBTREE  "app"
#define APP_SETTINGS_MODE_KEY "mode"

#define APP_SETTINGS_MODE_PATH \
	APP_SETTINGS_SUBTREE "/" APP_SETTINGS_MODE_KEY

/*
 * restored_mode contains the most recent valid mode loaded from flash.
 * It defaults to NORMAL when no setting exists.
 */
static atomic_t restored_mode =
	ATOMIC_INIT(APP_MODE_NORMAL);

/*
 * pending_mode is the latest mode requested for saving.
 */
static atomic_t pending_mode =
	ATOMIC_INIT(APP_MODE_NORMAL);

/*
 * A generation counter prevents a mode change from being lost if another
 * save request arrives while the flash write work item is executing.
 */
static atomic_t save_generation = ATOMIC_INIT(0);

static void app_settings_save_work_handler(
	struct k_work *work);

K_WORK_DELAYABLE_DEFINE(
	app_settings_save_work,
	app_settings_save_work_handler);

static int app_settings_set(
	const char *name,
	size_t len,
	settings_read_cb read_cb,
	void *cb_arg)
{
	uint8_t saved_mode;
	int ret;

	if (strcmp(name, APP_SETTINGS_MODE_KEY) != 0) {
		return -ENOENT;
	}

	if (len != sizeof(saved_mode)) {
		LOG_WRN("Invalid saved application mode length: %u",
			(unsigned int)len);

		return -EINVAL;
	}

	ret = read_cb(
		cb_arg,
		&saved_mode,
		sizeof(saved_mode));

	if (ret < 0) {
		LOG_ERR("Failed to read saved application mode: %d",
			ret);

		return ret;
	}

	if (ret != sizeof(saved_mode)) {
		LOG_WRN("Incomplete application mode setting: %d",
			ret);

		return -EINVAL;
	}

	if (saved_mode > APP_MODE_DIAGNOSTIC) {
		LOG_WRN("Invalid saved application mode: %u",
			saved_mode);

		return -EINVAL;
	}

	atomic_set(
		&restored_mode,
		(atomic_val_t)saved_mode);

	atomic_set(
		&pending_mode,
		(atomic_val_t)saved_mode);

	LOG_INF("Application mode restored: %u",
		saved_mode);

	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(
	app_settings,
	APP_SETTINGS_SUBTREE,
	NULL,
	app_settings_set,
	NULL,
	NULL);

static void app_settings_save_work_handler(
	struct k_work *work)
{
	atomic_val_t generation_before;
	enum app_mode mode;
	uint8_t saved_mode;
	int ret;

	ARG_UNUSED(work);

	generation_before =
		atomic_get(&save_generation);

	mode = (enum app_mode)atomic_get(
		&pending_mode);

	if ((mode < APP_MODE_NORMAL) ||
	    (mode > APP_MODE_DIAGNOSTIC)) {

		LOG_ERR("Refusing to save invalid application mode: %d",
			mode);

		return;
	}

	saved_mode = (uint8_t)mode;

	ret = settings_save_one(
		APP_SETTINGS_MODE_PATH,
		&saved_mode,
		sizeof(saved_mode));

	if (ret < 0) {
		LOG_ERR("Failed to save application mode %u: %d",
			saved_mode,
			ret);
	} else {
		atomic_set(
			&restored_mode,
			(atomic_val_t)mode);

		LOG_INF("Application mode saved: %u",
			saved_mode);
	}

	/*
	 * Another mode may have been requested while the flash write
	 * was taking place. Schedule another write for the newest value.
	 */
	if (atomic_get(&save_generation) !=
	    generation_before) {

		ret = k_work_reschedule(
			&app_settings_save_work,
			K_NO_WAIT);

		if (ret < 0) {
			LOG_ERR("Failed to reschedule application "
				"mode save: %d",
				ret);
		}
	}
}

int app_settings_load(void)
{
	int ret = settings_load();

	if (ret < 0) {
		LOG_ERR("Failed to load settings: %d",
			ret);

		return ret;
	}

	LOG_INF("Settings loaded");

	return 0;
}

enum app_mode app_settings_mode_get(void)
{
	enum app_mode mode =
		(enum app_mode)atomic_get(
			&restored_mode);

	if ((mode < APP_MODE_NORMAL) ||
	    (mode > APP_MODE_DIAGNOSTIC)) {

		return APP_MODE_NORMAL;
	}

	return mode;
}

int app_settings_mode_save_request(
	enum app_mode mode)
{
	int ret;

	if ((mode < APP_MODE_NORMAL) ||
	    (mode > APP_MODE_DIAGNOSTIC)) {

		return -EINVAL;
	}

	atomic_set(
		&pending_mode,
		(atomic_val_t)mode);

	atomic_inc(&save_generation);

	ret = k_work_reschedule(
		&app_settings_save_work,
		K_NO_WAIT);

	if (ret < 0) {
		LOG_ERR("Failed to schedule application mode save: %d",
			ret);

		return ret;
	}

	return 0;
}