#pragma once

#include "breezy_settings.h"

/*
 * Write display-distance and follow-threshold control signals to
 * /dev/shm/xr_driver_control whenever the relevant GSettings change.
 *
 * The effective distance sent to the driver is display_distance when
 * zoom-on-focus is inactive (display_distance == display_distance_default),
 * or display_distance_default otherwise — mirroring _change_distance() in
 * breezy-desktop, which toggles between toggle_display_distance_start/end.
 *
 * Call this whenever breezy_settings_consume_if_changed returns true, or
 * once at startup with the initial settings.
 */
void breezy_driver_control_update(const struct breezy_display_settings *s);
