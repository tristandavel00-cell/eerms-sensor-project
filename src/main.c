#include <zephyr/logging/log.h>

#include "app_controller.h"

LOG_MODULE_REGISTER(eerms_main, LOG_LEVEL_INF);

int main(void)
{
	int ret;

	LOG_INF("Application started");

	ret = app_controller_init();

	if (ret < 0) {
		LOG_ERR("Application initialization failed: %d",
			ret);

		return ret;
	}

	LOG_INF("Setup complete; entering application event loop");

	app_controller_run();

	/*
	 * The controller event loop normally never returns.
	 */
	return 0;
}