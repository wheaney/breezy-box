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

    uint64_t last_poll_ms;
};

/* Fill *s with the gsettings schema defaults. */
void breezy_settings_init_defaults(struct breezy_display_settings *s);

/* Read all relevant keys from GSettings into *s.  Silently skips on error. */
void breezy_settings_read(struct breezy_display_settings *s);

/*
 * Re-read settings at most once per second.  Pass the current realtime
 * millisecond timestamp obtained from the same clock used by the caller.
 */
void breezy_settings_poll(struct breezy_display_settings *s, uint64_t now_ms);

/* max(display_distance, toggle_distance_start, toggle_distance_end) */
double breezy_settings_display_distance_default(const struct breezy_display_settings *s);

/*
 * (display_distance_default - lens_distance_ratio) * display_size
 * Matches VirtualDisplaysActor._distance_adjusted_size.
 */
float breezy_settings_distance_adjusted_size(const struct breezy_display_settings *s,
                                              float lens_distance_ratio);
