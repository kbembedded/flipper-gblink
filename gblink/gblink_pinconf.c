// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2023 KBEmbedded

#include <furi.h>
#include <furi_hal.h>

#include <stdint.h>

#include <gblink/include/gblink.h>
#include "gblink_i.h"

struct gblink_pins {
        const GpioPin *serin;
        const GpioPin *serout;
        const GpioPin *clk;
        const GpioPin *sd;
};

const struct gblink_pins common_pinouts[PINOUT_COUNT] = {
	/* Original */
	{
		&gpio_ext_pc3,
		&gpio_ext_pb3,
		&gpio_ext_pb2,
		&gpio_ext_pa4,
	},
	/* MALVEKE EXT1 */
	{
		&gpio_ext_pa6,
		&gpio_ext_pa7,
		&gpio_ext_pb3,
		&gpio_ext_pa4,
	},
};

int gblink_pin_set(void *handle, gblink_bus_pins pin, const GpioPin *gpio)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk)
		return 1;

	switch (pin) {
	case PIN_SERIN:
		gblink->serin = gpio;
		break;
	case PIN_SEROUT:
		gblink->serout = gpio;
		break;
	case PIN_CLK:
		gblink->clk = gpio;
		break;
	case PIN_SD:
		gblink->sd = gpio;
		break;
	default:
		furi_crash();
		break;
	}

	furi_mutex_release(gblink->start_mutex);

	return 0;
}

const GpioPin *gblink_pin_get(void *handle, gblink_bus_pins pin)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	switch (pin) {
	case PIN_SERIN:
		return gblink->serin;
	case PIN_SEROUT:
		return gblink->serout;
	case PIN_CLK:
		return gblink->clk;
	case PIN_SD:
		return gblink->sd;
	default:
		furi_crash();
		break;
	}

	return NULL;
}

int gblink_pin_set_default(void *handle, gblink_pinouts pinout)
{
	furi_assert(handle);
	struct gblink *gblink = handle;

	if (furi_mutex_acquire(gblink->start_mutex, 0) != FuriStatusOk)
		return 1;

	gblink->serin = common_pinouts[pinout].serin;
	gblink->serout = common_pinouts[pinout].serout;
	gblink->clk = common_pinouts[pinout].clk;
	gblink->sd = common_pinouts[pinout].sd;

	furi_mutex_release(gblink->start_mutex);

	return 0;
}

int gblink_pin_get_default(void *handle)
{
	furi_assert(handle);
	struct gblink *gblink = handle;
	int i;

	for (i = 0; i < PINOUT_COUNT; i++) {
		if (gblink->serin != common_pinouts[i].serin)
			continue;
		if (gblink->serout != common_pinouts[i].serout)
			continue;
		if (gblink->clk != common_pinouts[i].clk)
			continue;
		/* XXX: Currently not checked or used! */
		//if (gblink->sd != common_pinouts[pinout].sd;
		break;
	}

	if (i == PINOUT_COUNT)
		i = -1;

	return i;
}
