#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BREEZY_GSETTINGS_SCHEMA "com.xronlinux.BreezyDesktop"
#define BREEZY_GSETTINGS_PATH   "/com/xronlinux/BreezyDesktop/"

/*
 * Display-related settings read from the com.xronlinux.BreezyDesktop GSettings
 * schema.  Only keys that affect rendering are included; UI-only keys (shortcuts,
 * monitor management, etc.) are omitted.
 */
struct breezy_display_settings {
    /* Monitor placement */
    char    monitor_wrapping_scheme[32]; /* "automatic"|"horizontal"|"vertical"|"flat" */
    int     monitor_spacing;             /* 0-100; visual units of 0.001 * viewport width */
    bool    curved_display;
    bool    headset_display_as_viewport_center;
    double  viewport_offset_x;          /* fraction of viewport width  */
    double  viewport_offset_y;          /* fraction of viewport height */

    /* Size and distance */
    double  display_distance;
    double  display_size;
    double  toggle_display_distance_start;
    double  toggle_display_distance_end;

    /* Rendering */
    double  framerate_cap;              /* 0 = uncapped */
    int     look_ahead_override;        /* -1 = use device default */
    bool    disable_anti_aliasing;

    /* Follow */
    double  follow_threshold;           /* degrees */
};

/*
 * Opaque handle returned by breezy_settings_start().
 * Owns the GLib main loop thread and GSettings change subscription.
 */
typedef struct breezy_settings_handle breezy_settings_handle;

/*
 * Start the background GLib thread and subscribe to GSettings changes.
 * Populates *initial with the current values before returning.
 * Returns NULL on failure.
 */
breezy_settings_handle *breezy_settings_start(struct breezy_display_settings *initial);

/*
 * If any GSettings key changed since the last call, copy the latest values
 * into *out and return true.  Otherwise return false and leave *out untouched.
 */
bool breezy_settings_consume_if_changed(breezy_settings_handle *h,
                                        struct breezy_display_settings *out);

/*
 * Stop the background thread and free all resources.
 * Safe to call with NULL.
 */
void breezy_settings_stop(breezy_settings_handle *h);

/* max(display_distance, toggle_distance_start, toggle_distance_end) */
double breezy_settings_display_distance_default(const struct breezy_display_settings *s);

/*
 * (display_distance_default - lens_distance_ratio) * display_size
 * Matches VirtualDisplaysActor._distance_adjusted_size.
 */
float breezy_settings_distance_adjusted_size(const struct breezy_display_settings *s,
                                              float lens_distance_ratio);
