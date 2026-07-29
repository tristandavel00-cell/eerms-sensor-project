#ifndef APP_NFC_H
#define APP_NFC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_types.h"

typedef void (*app_nfc_mode_command_cb_t)(enum app_mode mode);

int app_nfc_start(enum app_mode mode,
		  uint32_t single_click_count,
		  app_nfc_mode_command_cb_t mode_command_cb);

void app_nfc_update_request(enum app_mode mode,
			    uint32_t single_click_count);

bool app_nfc_field_on(void);
bool app_nfc_field_off(void);
void app_nfc_write_received(size_t data_length);

#endif
