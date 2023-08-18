#pragma once

#include <stdlib.h>

// Initialize badge display. An error will be reported if the initialization
// failed.
//
// Must be called exactly once from a task and cannot be called cocurrently with
// any other flow3r_bsp_display_* functions.
//
// Side effects: initializes singleton flow3r display object. All other
// flow3r_bsp_display_* functions operate on same object.
void flow3r_bsp_display_init(void);

// Set display backlight, as integer percent value (from 0 to 100, clamped).
// No-op if display hasn't been succesfully initialized.
void flow3r_bsp_display_set_backlight(uint8_t percent);