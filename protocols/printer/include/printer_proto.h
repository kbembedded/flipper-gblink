// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2024 KBEmbedded

#ifndef PRINTER_PROTO_H
#define PRINTER_PROTO_H

#include <gblink/include/gblink.h>

#pragma once

#define STATUS_LOWBATT		(1 << 7)
#define STATUS_ERR		(1 << 6)
#define STATUS_JAM		(1 << 5)
#define STATUS_PKT_ERR		(1 << 4)
#define STATUS_READY		(1 << 3)
#define STATUS_FULL		(1 << 2)
#define STATUS_PRINTING		(1 << 1)
#define STATUS_CKSUM_ERR	(1 << 0)

/* emulate printer's internal print receive buffer */
#define TILE_SIZE		16 // 8x8 tile, 2bpp color
#define WIDTH			20 // 20 tiles wide
#define HEIGHT			18 // 18 tiles tall
#define PRINT_FULL_SZ		5760 // (LINE_BUF_SZ * HEIGHT / 2)

enum cb_reason {
	reason_line_xfer,
	reason_print,
	reason_complete,
};

/* Dual purpose struct used for both receiving image data from game boy, and
 * sending it to printer.
 */
struct gb_image {
	/* NOTE: Do not change the order of these 4 bytes!
	 * TODO: Maybe make this a struct, or a union, or something to help
	 * enforce their ordering to allow for a memcpy to and from printer.
	 */
	/* TODO: Need to understand this more */
	uint8_t num_sheets;
	uint8_t margins;
	/* TODO: Does this actually matter? */
	uint8_t palette;
	/* TODO: Need to play with this more */
	uint8_t exposure;

	size_t data_sz;
	uint8_t data[PRINT_FULL_SZ];
};

/**
 * Allocate a printer instance
 *
 * This will manage a gblink instance under the hood, top level applications
 * needing to send/receive Game Boy Printer packets only need to work with
 * the printer specific functions.
 *
 * @returns Pointer to a printer instance.
 */
void *printer_alloc(void);

/**
 * Free a printer instance
 *
 * @param printer_handle Printer instance handle
 */
void printer_free(void *printer_handle);

/**
 * Set context for registered callback
 *
 * @param printer_handle Printer instance handle
 * @param context Pointer to context
 */
void printer_callback_context_set(void *printer_handle, void *context);

/**
 * Register a callback
 *
 * The callback can be called multiple times and for different reasons
 *
 * The callback is called with the arguments of the user specified context,
 * a pointer to a struct gb_image, and a reason for the callback.
 *
 * @note The struct gb_image pointer is valid until the the print is marked
 * as completed.
 *
 * @param printer_handle Printer instance handle
 * @param callback Pointer to callback
 */
void printer_callback_set(void *printer_handle, void (*callback)(void *context, struct gb_image *image, enum cb_reason reason));

/* Can only be run after alloc, before start */
/**
 * Set one of the pre-configured pinouts
 *
 * @param printer_handle Printer instance handle
 * @param pinout Which pinout to use
 *
 * @note The printer instance must not be actively sending or receiving
 *
 * @returns 0 on success, 1 if gblink instance is not gblink_stop()'ed.
 */
int printer_pin_set_default(void *printer_handle, gblink_pinouts pinout);

/**
 * Set a gpio pin to a specific pin mode
 *
 * @param printer_handle Printer instance handle
 * @param pin Pin mode to assign to the gpio pin
 * @param gpio Which gpio pin to assign the pin mode
 *
 * @note The printer instance must not be actively sending or receiving
 *
 * @returns 0 on success, 1 if gblink instance is not gblink_stop()'ed.
 */
int printer_pin_set(void *printer_handle, gblink_bus_pins pin, const GpioPin *gpio);

/**
 * Get the gpio pin associated with the requested pin mode
 *
 * @param printer_handle Printer instance handle
 * @param pin Pin mode to inquire about
 *
 * @returns GpioPin pointer
 */
const GpioPin *printer_pin_get(void *printer_handle, gblink_bus_pins pin);

/**
 * Stop a printer instance
 *
 * Disables interrupts, stops any pending timers, and enters back to an idle
 * state. Once called, re-allows changes to be made.
 *
 * @param printer_handle Printer instance handle
 */
void printer_stop(void *printer_handle);

#endif // PRINTER_PROTO_H
