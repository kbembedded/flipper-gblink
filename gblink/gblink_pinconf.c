// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2023 KBEmbedded

#include <furi.h>
#include <storage/storage.h>
#include <lib/toolbox/stream/stream.h>
#include <lib/flipper_format/flipper_format.h>

#include <stdint.h>

#include <gblink/include/gblink.h>
#include <gblink/include/gblink_pinconf.h>
#include "gblink_i.h"

#define PINCONF_FILE_TYPE	"Flipper GB Link Pinconf"
#define PINCONF_FILE_VER	1

/* XXX: This must match gblink_pinouts order */
const struct gblink_gpio_pinouts gblink_gpio_pinouts[] = {
	{ "Original",	{ &gpio_ext_pc3, &gpio_ext_pb3, &gpio_ext_pb2/*, &gpio_ext_pa4*/ }, },
	{ "MLVK2.5",	{ &gpio_ext_pa6, &gpio_ext_pa7, &gpio_ext_pb3/*, &gpio_ext_pa4*/ }, },
	{ "Custom",	{ NULL, NULL, NULL/*, NULL*/ }, },
};

/* XXX: This must match gblink_bus_pins order */
const char* gblink_gpio_pinnames[] = {
	"SI",
	"SO",
	"CLK",
};

int gblink_pin_set_by_gpiopin(void *handle, gblink_bus_pins pin, const GpioPin *gpio)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk)
		return -1;

	/* XXX: This doesn't do any bounds checking */
	gblink->gpio[pin] = gpio;

	furi_mutex_release(gblink->start_mutex);

	return 0;
}

const GpioPin *gblink_pin_get_by_gpiopin(void *handle, gblink_bus_pins pin)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	/* XXX: This doesn't do any bounds checking */
	return gblink->gpio[pin];
}

int gblink_pin_set_default(void *handle, gblink_pinouts pinout)
{
	furi_assert(handle);
	struct gblink *gblink = handle;
	gblink_bus_pins pin;


	if (pinout == PINOUT_CUSTOM || pinout >= PINOUT_COUNT)
		return -1;

	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk)
		return -1;

	for (pin = PIN_START; pin < PIN_COUNT; pin++)
		gblink->gpio[pin] = gblink_gpio_pinouts[pinout].pin[pin];

	furi_mutex_release(gblink->start_mutex);

	return 0;
}

gblink_pinouts gblink_pin_get_default(void *handle)
{
	furi_assert(handle);
	struct gblink *gblink = handle;
	gblink_pinouts pinout;
	gblink_bus_pins pin;

	for (pinout = PINOUT_START; pinout < PINOUT_CUSTOM; pinout++) {
		for (pin = PIN_START; pin < PIN_COUNT; pin++) {
			if (gblink->gpio[pin] != gblink_gpio_pinouts[pinout].pin[pin])
				break;
		}
		/* If we broke out early, that means its not a match */
		if (pin != PIN_COUNT)
			continue;

		/* If all pins match, then the current pinout matches a known
		 * default pinout.
		 */
		break;
	}

	return pinout;
}

int gblink_pin_set(void *handle, gblink_bus_pins pin, unsigned int pinnum)
{
	furi_assert(handle);
	furi_assert(pinnum < gpio_pins_count);

	struct gblink *gblink = handle;

	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk)
		return -1;

	/* XXX: No bounds checking done at this time */
	gblink->gpio[pin] = gpio_pins[pinnum].pin;

	furi_mutex_release(gblink->start_mutex);

	return 0;
}

int gblink_pin_get(void *handle, gblink_bus_pins pin)
{
	furi_assert(handle);
	struct gblink *gblink = handle;
	unsigned int i;

	/* XXX: This doesn't do bounds checking */
	for (i = 0; i < gpio_pins_count; i++) {
		if (gpio_pins[i].pin == gblink->gpio[pin])
			return i;
	}

	return -1;
}

int gblink_pin_count_max(void)
{
	unsigned int i;
	int count = 0;

	for (i = 0; i < gpio_pins_count; i++)
		if (!gpio_pins[i].debug)
			count = i;

	return count;
}

int gblink_pin_get_next(unsigned int pinnum)
{
	unsigned int i;

	/* The check could be eliminated and just rely on the return -1 at the
	 * end of the function, but this is a shortcut so we don't have to walk
	 * through the whole table just to find out its an out-of-bounds pin.
	 */
	if (pinnum >= gpio_pins_count)
		return -1;

	for (i = pinnum; i < gpio_pins_count; i++)
		if (!gpio_pins[i].debug)
			return i;

	return -1;
}

int gblink_pin_get_prev(unsigned int pinnum)
{
	int i;

	/* The check could be eliminated and just rely on the return -1 at the
	 * end of the function, but this is a shortcut so we don't have to walk
	 * through the whole table just to find out its an out-of-bounds pin.
	 */
	if (pinnum >= gpio_pins_count)
		return -1;

	for (i = pinnum; i >= 0; i--)
		if (!gpio_pins[i].debug)
			return i;

	return -1;
}

bool gblink_pinconf_load(void *gblink)
{

	Storage *storage = NULL;;
	FlipperFormat *data_file = NULL;
	FuriString *string = NULL;
	uint32_t val;
	bool ret = false;
	gblink_pinouts pinout;
	gblink_bus_pins pin;

	storage = furi_record_open(RECORD_STORAGE);

	string = furi_string_alloc_set(APP_DATA_PATH(""));
	storage_common_resolve_path_and_ensure_app_directory(storage, string);
	furi_string_cat_str(string, ".gblink_pinconf");

	data_file = flipper_format_file_alloc(storage);

	if (!flipper_format_file_open_existing(data_file, furi_string_get_cstr(string))) {
		FURI_LOG_E("pinconf", "Error opening file %s", furi_string_get_cstr(string));
		goto out;
	}

	if (!flipper_format_read_header(data_file, string, &val)) {
		FURI_LOG_E("pinconf", "Missing or incorrect header");
		goto out;
	}

	if (strncmp(furi_string_get_cstr(string), PINCONF_FILE_TYPE, strlen(PINCONF_FILE_TYPE)) ||
	    val != PINCONF_FILE_VER) {
		FURI_LOG_E("pinconf", "Type or version mismatch");
		goto out;
	}

	if (!flipper_format_read_string(data_file, "Mode", string)) {
		FURI_LOG_E("pinconf", "Missing Mode");
		goto out;
	}

	for (pinout = PINOUT_START; pinout < PINOUT_COUNT; pinout++) {
		if (!strncmp(furi_string_get_cstr(string),
			     gblink_gpio_pinouts[pinout].name,
			     strlen(gblink_gpio_pinouts[pinout].name))) {
			FURI_LOG_I("pinconf", "Setting %s pinout",
				   gblink_gpio_pinouts[pinout].name);
			gblink_pin_set_default(gblink, pinout);
			break;
		}
	}
	/* XXX: This should update ret */
	/* A normal pinout was set, move on */
	if (pinout != PINOUT_CUSTOM)
		goto out;

	/* If a custom pinout was set */
	for (pin = PIN_START; pin < PIN_COUNT; pin++) {
		if (!flipper_format_read_uint32(data_file, gblink_gpio_pinnames[pin], &val, 1)) {
			FURI_LOG_E("pinconf", "Missing %s", gblink_gpio_pinnames[pin]);
			goto out;
		} else {
			gblink_pin_set(gblink, pin, val);
		}
	}

	ret = true;

out:
	flipper_format_file_close(data_file);
	furi_string_free(string);
	furi_record_close(RECORD_STORAGE);
	return ret;
}

/* XXX: TODO: I think there is a way to build this ahead of time and
 * write it in one go to be a bit more time efficient.
 */
bool gblink_pinconf_save(void *gblink)
{

	Storage *storage = NULL;;
	FlipperFormat *data_file = NULL;
	FuriString *string = NULL;
	uint32_t pinnum;
	bool ret = false;
	gblink_pinouts pinout;
	gblink_bus_pins pin;

	storage = furi_record_open(RECORD_STORAGE);

	string = furi_string_alloc_set(APP_DATA_PATH(""));
	storage_common_resolve_path_and_ensure_app_directory(storage, string);
	furi_string_cat_str(string, ".gblink_pinconf");

	data_file = flipper_format_file_alloc(storage);

	if (!flipper_format_file_open_always(data_file, furi_string_get_cstr(string))) {
		FURI_LOG_E("pinconf", "Error opening file %s", furi_string_get_cstr(string));
		goto out;
	}

	if (!flipper_format_write_header_cstr(data_file, PINCONF_FILE_TYPE, PINCONF_FILE_VER)) {
		FURI_LOG_E("pinconf", "Error writing header to file");
		goto out;
	}

	pinout = gblink_pin_get_default(gblink);
	if (!flipper_format_write_string_cstr(data_file, "Mode", gblink_gpio_pinouts[pinout].name))
		FURI_LOG_E("pinconf", "Error writing mode to file");

	if (pinout != PINOUT_CUSTOM)
		goto out;

	/* Set each pin for a custom pinout */
	for (pin = PIN_START; pin < PIN_COUNT; pin++) {
		pinnum = gblink_pin_get(gblink, pin);
		if (!flipper_format_write_uint32(data_file,
						 gblink_gpio_pinnames[pin],
						 &pinnum,
						 1)) {
			FURI_LOG_E("pinconf", "Error writing %s to file",
					      gblink_gpio_pinnames[pin]);
			goto out;
		}
	}

	ret = true;

out:
	flipper_format_file_close(data_file);
	furi_string_free(string);
	furi_record_close(RECORD_STORAGE);
	return ret;
}

