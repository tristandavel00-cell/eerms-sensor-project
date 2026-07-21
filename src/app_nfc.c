#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>
#include <nfc_t4t_lib.h>
#include <nfc/t4t/ndef_file.h>

#include "app_nfc.h"

LOG_MODULE_REGISTER(app_nfc, LOG_LEVEL_DBG);

#define NFC_NDEF_BUFFER_SIZE 256
#define NFC_TEXT_BUFFER_SIZE 96
#define NFC_LANGUAGE_CODE "en"
#define NFC_UPDATE_DELAY_MS 50

static uint8_t nfc_ndef_buffer[NFC_NDEF_BUFFER_SIZE];
static uint32_t nfc_ndef_length;
static uint8_t nfc_text_buffer[NFC_TEXT_BUFFER_SIZE];
static bool nfc_emulation_running;
static atomic_t nfc_update_generation = ATOMIC_INIT(0);
static atomic_t pending_mode = ATOMIC_INIT(APP_MODE_NORMAL);
static atomic_t pending_click_count = ATOMIC_INIT(0);
static atomic_t nfc_field_present = ATOMIC_INIT(0);
static atomic_t nfc_update_pending = ATOMIC_INIT(0);

static uint8_t nfc_write_buffer[NFC_NDEF_BUFFER_SIZE];
static size_t nfc_write_length;
static atomic_t nfc_write_pending = ATOMIC_INIT(0);
static app_nfc_mode_command_cb_t nfc_mode_command_cb;


static void app_nfc_update_work_handler(struct k_work *work);
static void app_nfc_write_work_handler(struct k_work *work);

static int app_nfc_parse_mode_command(const uint8_t *text,
				      size_t text_length,
				      enum app_mode *mode);
					  
K_WORK_DELAYABLE_DEFINE(app_nfc_update_work, app_nfc_update_work_handler);
K_WORK_DEFINE(app_nfc_write_work, app_nfc_write_work_handler);

static const char *app_nfc_mode_name(enum app_mode mode)
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

bool app_nfc_field_on(void)
{
	if (!atomic_cas(&nfc_field_present, 0, 1)) {
		return false;
	}

	LOG_DBG("NFC field state changed to present");
	return true;
}

bool app_nfc_field_off(void)
{
	if (!atomic_cas(&nfc_field_present, 1, 0)) {
		return false;
	}

	LOG_DBG("NFC field state changed to absent");

	if (atomic_cas(&nfc_update_pending, 1, 0)) {
		int ret = k_work_reschedule(
			&app_nfc_update_work,
			K_MSEC(NFC_UPDATE_DELAY_MS));

		if (ret < 0) {
			LOG_ERR("Failed to schedule pending NFC update: %d", ret);
		}
	}

	return true;
}

static int app_nfc_update_payload(enum app_mode mode,
				  uint32_t single_click_count)
{
	int err;
	int text_length;

	text_length = snprintk(
		nfc_text_buffer,
		sizeof(nfc_text_buffer),
		"Zephyr Button App\nMode: %s\nClicks: %u",
		app_nfc_mode_name(mode),
		single_click_count);

	if (text_length < 0) {
		LOG_ERR("Failed to format NFC text: %d", text_length);
		return text_length;
	}

	if ((size_t)text_length >= sizeof(nfc_text_buffer)) {
		LOG_ERR("NFC text buffer is too small");
		return -ENOMEM;
	}

	static const uint8_t language_code[] = NFC_LANGUAGE_CODE;

	NFC_NDEF_TEXT_RECORD_DESC_DEF(text_record,
				      UTF_8,
				      language_code,
				      sizeof(language_code) - 1U,
				      nfc_text_buffer,
				      (uint32_t)text_length);

	NFC_NDEF_MSG_DEF(nfc_message, 1);

	err = nfc_ndef_msg_record_add(
		&NFC_NDEF_MSG(nfc_message),
		&NFC_NDEF_TEXT_RECORD_DESC(text_record));

	if (err < 0) {
		LOG_ERR("Failed to add NFC text record: %d", err);
		return err;
	}

	nfc_ndef_length =
		nfc_t4t_ndef_file_msg_size_get(sizeof(nfc_ndef_buffer));

	err = nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_message),
				  nfc_t4t_ndef_file_msg_get(nfc_ndef_buffer),
				  &nfc_ndef_length);

	if (err < 0) {
		LOG_ERR("Failed to encode NFC message: %d", err);
		return err;
	}

	err = nfc_t4t_ndef_file_encode(
		nfc_ndef_buffer,
		&nfc_ndef_length);

	if (err < 0) {
		LOG_ERR("Failed to encode NFC type 4 file: %d", err);
		return err;
	}

	if (nfc_emulation_running) {
		err = nfc_t4t_emulation_stop();

		if (err < 0) {
			LOG_ERR("Failed to stop NFC emulation before update: %d",
				err);
			return err;
		}

		nfc_emulation_running = false;
	}

	err = nfc_t4t_ndef_rwpayload_set(nfc_ndef_buffer,
				       sizeof(nfc_ndef_buffer));

	if (err < 0) {
		LOG_ERR("Failed to update NFC payload: %d", err);
		return err;
	}

	err = nfc_t4t_emulation_start();

	if (err < 0) {
		LOG_ERR("Failed to restart NFC emulation after update: %d", err);
		return err;
	}

	nfc_emulation_running = true;

	LOG_INF("NFC payload updated: mode=%s, clicks=%u",
	app_nfc_mode_name(mode),
	single_click_count);

	return 0;
}

static void app_nfc_update_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	atomic_val_t generation_before =
		atomic_get(&nfc_update_generation);

	enum app_mode mode =
		(enum app_mode)atomic_get(&pending_mode);

	uint32_t single_click_count =
		(uint32_t)atomic_get(&pending_click_count);

	int err = app_nfc_update_payload(mode, single_click_count);

	if (err < 0) {
		LOG_ERR("Failed to update NFC payload from work item: %d", err);
		return;
	}

	atomic_val_t generation_after =
		atomic_get(&nfc_update_generation);

	if (generation_after != generation_before) {
		LOG_DBG("NFC state changed during update; scheduling another update.");

		int ret = k_work_reschedule(
			&app_nfc_update_work,
			K_MSEC(NFC_UPDATE_DELAY_MS));

		if (ret < 0) {
			LOG_ERR("Failed to reschedule NFC update work: %d", ret);
		}
	}
}

void app_nfc_update_request(enum app_mode mode,
			    uint32_t single_click_count)
{
	atomic_set(&pending_mode, (atomic_val_t)mode);
	atomic_set(&pending_click_count,
		   (atomic_val_t)single_click_count);

	atomic_inc(&nfc_update_generation);

	if (atomic_get(&nfc_field_present)) {
		atomic_set(&nfc_update_pending, 1);
		LOG_DBG("NFC update deferred until field is removed");
		return;
	}

	int ret = k_work_reschedule(
		&app_nfc_update_work,
		K_MSEC(NFC_UPDATE_DELAY_MS));

	if (ret < 0) {
		LOG_ERR("Failed to schedule NFC update work: %d", ret);
	}
}

void app_nfc_write_received(size_t data_length)
{
	size_t file_length =
		data_length + NFC_NDEF_FILE_NLEN_FIELD_SIZE;

	if (file_length > sizeof(nfc_write_buffer)) {
		LOG_ERR("Written NFC message is too large: %u bytes",
			(unsigned int)file_length);
		return;
	}

	if (!atomic_cas(&nfc_write_pending, 0, 1)) {
		LOG_WRN("Previous NFC write is still being processed");
		return;
	}

	memcpy(nfc_write_buffer,
		nfc_ndef_buffer,
		file_length);

	nfc_write_length = file_length;

	int ret = k_work_submit(&app_nfc_write_work);

	if (ret < 0) {
		atomic_set(&nfc_write_pending, 0);
		LOG_ERR("Failed to submit NFC write work: %d", ret);
	}
}

static void app_nfc_write_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_INF("Copied NFC Type 4 file: %u bytes",
		(unsigned int)nfc_write_length);

	LOG_HEXDUMP_DBG(nfc_write_buffer,
			nfc_write_length,
			"Written NFC data");

	/*
	 * Minimum structure:
	 * 2-byte NLEN
	 * NDEF header
	 * type length
	 * payload length
	 * type
	 * text status byte
	 */
	if (nfc_write_length < 8U) {
		LOG_WRN("NFC file is too short");
		goto finished;
	}

	uint16_t ndef_length =
		((uint16_t)nfc_write_buffer[0] << 8) |
		nfc_write_buffer[1];

	if ((size_t)ndef_length + NFC_NDEF_FILE_NLEN_FIELD_SIZE >
	    nfc_write_length) {
		LOG_WRN("Invalid NFC NDEF length: %u",
			(unsigned int)ndef_length);
		goto finished;
	}

	const uint8_t *record = &nfc_write_buffer[2];

	uint8_t header = record[0];
	uint8_t type_length = record[1];
	uint8_t payload_length = record[2];

	/*
	 * 0xD1 means:
	 * MB=1, ME=1, SR=1, TNF=well-known.
	 */
	if (header != 0xD1U) {
		LOG_WRN("Unsupported NDEF header: 0x%02x", header);
		goto finished;
	}

	if ((type_length != 1U) || (record[3] != 'T')) {
		LOG_WRN("NDEF record is not a text record");
		goto finished;
	}

	size_t record_length =
		3U + type_length + payload_length;

	if (record_length > ndef_length) {
		LOG_WRN("NDEF record exceeds message length");
		goto finished;
	}

	const uint8_t *payload = &record[3U + type_length];

	if (payload_length < 1U) {
		LOG_WRN("Text-record payload is empty");
		goto finished;
	}

	uint8_t status = payload[0];

	if ((status & 0x80U) != 0U) {
		LOG_WRN("UTF-16 NFC text is not supported");
		goto finished;
	}

	size_t language_length = status & 0x3FU;

	if ((1U + language_length) > payload_length) {
		LOG_WRN("Invalid NFC language-code length");
		goto finished;
	}

	const uint8_t *text =
		&payload[1U + language_length];

	size_t text_length =
		payload_length - 1U - language_length;

	enum app_mode requested_mode;
	int err = app_nfc_parse_mode_command(
		text,
		text_length,
		&requested_mode);

	if (err < 0) {
		LOG_WRN("Unsupported NFC command");
		LOG_HEXDUMP_WRN(text,
				text_length,
				"Received NFC text");
		goto finished;
	}

	LOG_INF("Valid NFC mode command received: %s",
		app_nfc_mode_name(requested_mode));

	if (nfc_mode_command_cb != NULL) {
		nfc_mode_command_cb(requested_mode);
	}

finished:
	atomic_set(&nfc_write_pending, 0);
	atomic_set(&nfc_update_pending, 1);

	if (!atomic_get(&nfc_field_present)) {
		(void)app_nfc_field_off();
		int ret = k_work_reschedule(&app_nfc_update_work,
					    K_MSEC(NFC_UPDATE_DELAY_MS));
		if (ret < 0) {
			LOG_ERR("Failed to restore NFC status payload: %d", ret);
		}
	}
}

static int app_nfc_parse_mode_command(const uint8_t *text,
						size_t text_length,
						enum app_mode *mode)
{
	if ((text == NULL) || (mode == NULL)) {
		return -EINVAL;
	}

	if ((text_length == strlen("MODE=NORMAL")) && 
		(memcmp(text, "MODE=NORMAL", text_length) == 0)) {
			*mode = APP_MODE_NORMAL;
			return 0;
		} 

	if ((text_length == strlen("MODE=CONFIG")) && 
		(memcmp(text, "MODE=CONFIG", text_length) == 0)) {
			*mode = APP_MODE_CONFIG;
			return 0;
		} 

	if ((text_length == strlen("MODE=DIAGNOSTIC")) && 
		(memcmp(text, "MODE=DIAGNOSTIC", text_length) == 0)) {
			*mode = APP_MODE_DIAGNOSTIC;
			return 0;
		} 

		return -EINVAL;
	
}

int app_nfc_start(enum app_mode mode, 
			uint32_t single_click_count,
			app_nfc_mode_command_cb_t mode_command_cb)
{
	nfc_mode_command_cb = mode_command_cb;

	atomic_set(&pending_mode, (atomic_val_t)mode);
	atomic_set(&pending_click_count,
		   (atomic_val_t)single_click_count);

	return app_nfc_update_payload(mode, single_click_count);
}