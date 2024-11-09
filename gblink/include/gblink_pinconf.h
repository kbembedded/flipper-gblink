// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2023 KBEmbedded

#ifndef __GBLINK_PINCONF_H__
#define __GBLINK_PINCONF_H__

#pragma once

#include <furi.h>
#include <furi_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Set one of the pre-configured pinouts
 *
 * @param handle Pointer to gblink handle
 * @param pinout Which pinout to use
 *
 * @note The gblink instance must not be gblink_start()'ed!
 *
 * @returns 0 on success, 1 if gblink instance is not gblink_stop()'ed.
 */
int gblink_pin_set_default(void *handle, gblink_pinouts pinout);

/**
 * Check if the pinout set matches a pre-configured one
 *
 * @param handle Pointer to gblink handle
 *
 * @returns The index of the pre-configured pinout or -1 on error
 */
int gblink_pin_get_default(void *handle);

/**
 * Set a gpio pin to a specific pin mode
 *
 * @param handle Pointer to gblink handle
 * @param pin Pin mode to assign to the gpio pin
 * @param gpio Which gpio pin to assign the pin mode
 *
 * @note The gblink instance must not be gblink_start()'ed!
 *
 * @returns 0 on success, 1 if gblink instance is not gblink_stop()'ed.
 */
int gblink_pin_set(void *handle, gblink_bus_pins pin, const GpioPin *gpio);

/**
 * Get the gpio pin associated with the requested pin mode
 *
 * @param handle Pointer to gblink handle
 * @param pin Pin mode to inquire about
 *
 * @returns GpioPin pointer
 */
const GpioPin *gblink_pin_get(void *handle, gblink_bus_pins pin);

#ifdef __cplusplus
}
#endif

#endif // __GBLINK_PINCONF_H__
