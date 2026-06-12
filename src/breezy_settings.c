#define _POSIX_C_SOURCE 200809L

#include "breezy_settings.h"

#include <gio/gio.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define SETTINGS_POLL_INTERVAL_MS 1000u

void breezy_settings_init_defaults(struct breezy_display_settings *s)
{
    memset(s, 0, sizeof(*s));
    strncpy(s->monitor_wrapping_scheme, "automatic",
            sizeof(s->monitor_wrapping_scheme) - 1);
    s->display_distance              = 1.05;
    s->display_size                  = 1.0;
    s->toggle_display_distance_start = 0.85;
    s->toggle_display_distance_end   = 1.05;
    s->look_ahead_override           = -1;
    s->follow_threshold              = 15.0;
}

void breezy_settings_read(struct breezy_display_settings *s)
{
    GSettings *gs = g_settings_new_with_path(BREEZY_GSETTINGS_SCHEMA,
                                              BREEZY_GSETTINGS_PATH);
    if (!gs) {
        fprintf(stderr, "breezy_settings: failed to open schema %s\n",
                BREEZY_GSETTINGS_SCHEMA);
        return;
    }

    gchar *scheme = g_settings_get_string(gs, "monitor-wrapping-scheme");
    if (scheme) {
        strncpy(s->monitor_wrapping_scheme, scheme,
                sizeof(s->monitor_wrapping_scheme) - 1);
        s->monitor_wrapping_scheme[sizeof(s->monitor_wrapping_scheme) - 1] = '\0';
        g_free(scheme);
    }

    s->monitor_spacing                    = g_settings_get_int    (gs, "monitor-spacing");
    s->curved_display                     = g_settings_get_boolean(gs, "curved-display");
    s->headset_display_as_viewport_center = g_settings_get_boolean(gs, "headset-display-as-viewport-center");
    s->viewport_offset_x                  = g_settings_get_double (gs, "viewport-offset-x");
    s->viewport_offset_y                  = g_settings_get_double (gs, "viewport-offset-y");
    s->display_distance                   = g_settings_get_double (gs, "display-distance");
    s->display_size                       = g_settings_get_double (gs, "display-size");
    s->toggle_display_distance_start      = g_settings_get_double (gs, "toggle-display-distance-start");
    s->toggle_display_distance_end        = g_settings_get_double (gs, "toggle-display-distance-end");
    s->framerate_cap                      = g_settings_get_double (gs, "framerate-cap");
    s->look_ahead_override                = g_settings_get_int    (gs, "look-ahead-override");
    s->disable_anti_aliasing              = g_settings_get_boolean(gs, "disable-anti-aliasing");
    s->follow_threshold                   = g_settings_get_double (gs, "follow-threshold");

    g_object_unref(gs);
}

void breezy_settings_poll(struct breezy_display_settings *s, uint64_t now_ms)
{
    if (now_ms - s->last_poll_ms < SETTINGS_POLL_INTERVAL_MS)
        return;
    s->last_poll_ms = now_ms;
    breezy_settings_read(s);
}

double breezy_settings_display_distance_default(const struct breezy_display_settings *s)
{
    double d = s->display_distance;
    if (s->toggle_display_distance_start > d) d = s->toggle_display_distance_start;
    if (s->toggle_display_distance_end   > d) d = s->toggle_display_distance_end;
    return d;
}

float breezy_settings_distance_adjusted_size(const struct breezy_display_settings *s,
                                               float lens_distance_ratio)
{
    float def = (float)breezy_settings_display_distance_default(s);
    return (def - lens_distance_ratio) * (float)s->display_size;
}
