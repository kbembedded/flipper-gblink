// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2023 KBEmbedded

#ifndef __EXTI_WORKAROUND_H__
#define __EXTI_WORKAROUND_H__

#include <furi.h>
#include <furi_hal.h>

void *exti_workaround(const GpioPin *clk, void (*isr_callback)(void *context), void *context);

void exti_workaround_undo(void *handle);

#endif // __EXTI_WORKAROUND_H__
