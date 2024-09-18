#ifndef PRINTER_PROTO_H
#define PRINTER_PROTO_H

#include <gblink/include/gblink.h>

#pragma once

enum cb_reason {
	reason_data,
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

	/* Always expected to be 160 px wide */
	size_t data_sz;
	uint8_t data[];
};

void *printer_alloc(void);

void printer_free(void *printer_handle);

void printer_callback_context_set(void *printer_handle, void *context);

void printer_callback_set(void *printer_handle, void (*callback)(void *context, struct gb_image *image, enum cb_reason reason));

/* Can only be run after alloc, before start */
int printer_pin_set_default(void *printer_handle, gblink_pinouts pinout);

int printer_pin_set(void *printer_handle, gblink_bus_pins pin, const GpioPin *gpio);

const GpioPin *printer_pin_get(void *printer_handle, gblink_bus_pins pin);

void printer_stop(void *printer_handle);

/* Allocates a buffer of the maximum size that the printer can handle, must be
 * freed manually. Provided as a convenience function so the application doesn't
 * need to know how big of a buffer to create.
 */
struct gb_image *printer_image_buffer_alloc(void);

void printer_image_buffer_free(struct gb_image *image);

#endif // PRINTER_PROTO_H
