// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2023 KBEmbedded

#ifndef __GBLINK_H__
#define __GBLINK_H__

#pragma once

#include <furi.h>
#include <furi_hal.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	/* Flipper drives the clock line */
	/* Unsupported at this time */
	GBLINK_CLK_INT,
	/* Game Boy drives the clock line */
	GBLINK_CLK_EXT,
} gblink_clk_source;

/* Currently unused */
typedef enum {
	GBLINK_MODE_GBC,
	GBLINK_MODE_GBA,
} gblink_mode;

/* Should this just be a macro? */
/* This pretty much only applies to GBC, OG GB is 8192 Hz only */
/* This is only for TX */
typedef enum {
	GBLINK_SPD_8192HZ = 4096,
	GBLINK_SPD_16384HZ = 8192,
	GBLINK_SPD_262144HZ = 16384,
	GBLINK_SPD_524288HZ = 262144,
} gblink_speed;

struct gblink_pins {
        const GpioPin *serin;
        const GpioPin *serout;
        const GpioPin *clk;
        const GpioPin *sd;
};

typedef enum {
	PINOUT_ORIGINAL,
	PINOUT_MALVEKE_EXT1,
	PINOUT_COUNT,
} gblink_pinouts;

extern const struct gblink_pins common_pinouts[PINOUT_COUNT];

/* 
 * All of these are things that are basically set once and would never need
 * to change during the lifetime of the instance.
 *
 * Callback can indeed be NULL if unneeded, however a call to the blocking
 * gblink_transfer_tx_wait_complete() must be used in order to get a notification
 * of the transfer being complete.
 */
struct gblink_spec {
	struct gblink_pins *pins;
	gblink_mode mode;
	void (*callback)(void* cb_context, uint8_t in);
	void *cb_context;
};

/*
 * NOTE:
 * This can be called at any time, it resets the current byte transfer information
 */
void gblink_clk_source_set(void *handle, gblink_clk_source clk_source);

void gblink_speed_set(void *handle, gblink_speed speed);

void gblink_timeout_set(void *handle, uint32_t us);

bool gblink_transfer(void *handle, uint8_t val);

/* 
 * This can be used for INT or EXT clock modes. After a call to
 * gblink_transfer this can be called at any time and will return only after
 * a full byte is transferred and it will return the byte that was last shifted
 * in from the link partner.
 */
uint8_t gblink_transfer_tx_wait_complete(void *handle);

void gblink_nobyte_set(void *handle, uint8_t val);

void gblink_int_enable(void *handle);

void gblink_int_disable(void *handle);

void *gblink_alloc(struct gblink_spec *gblink_spec);

void gblink_free(void *handle);

#ifdef __cplusplus
}
#endif

#endif // __GBLINK_H__
