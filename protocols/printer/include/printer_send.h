// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2024 KBEmbedded

#ifndef PRINTER_SEND_H
#define PRINTER_SEND_H

#pragma once

#include <protocols/printer/include/printer_proto.h>

/**
 * Start a printer instance in send mode
 *
 * Allows devices to send image data to the flipper
 *
 * @note Once printer_send_start() is called, some printer instance settings
 * can no longer be modified, e.g. the Game Boy Link pinout.
 *
 * @param printer_handle Printer instance handle
 * @param gb_image Pointer to a struct gb_image to print
 */
void printer_send_start(void *printer_handle, struct gb_image *gb_image);

#endif // PRINTER_SEND_H
