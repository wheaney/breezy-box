#define _POSIX_C_SOURCE 200809L

#include "breezy_driver_control.h"

#include <stdbool.h>
#include <stdio.h>

#define CONTROL_PATH "/dev/shm/xr_driver_control"

static void write_control(const char *key, double value)
{
    FILE *f = fopen(CONTROL_PATH, "w");
    if (!f) {
        perror("breezy_driver_control: fopen " CONTROL_PATH);
        return;
    }
    fprintf(f, "%s=%g\n", key, value);
    fclose(f);
}

void breezy_driver_control_update(const struct breezy_display_settings *s)
{
    /*
     * Mirrors breezy-desktop's _update_display_distance / _change_distance:
     *
     *   defaultDistance = max(display_distance, toggle_start, toggle_end)
     *   zoom_on_focus   = display_distance < defaultDistance
     *   effective       = zoom_on_focus ? display_distance : defaultDistance
     *   driver_value    = effective / (display_size * defaultDistance)
     *
     * When zoom-on-focus is off, effective == defaultDistance and the ratio
     * simplifies to 1/display_size.  When on, effective is the nearer value.
     */
    double default_distance = breezy_settings_display_distance_default(s);
    double size_adjustment  = s->display_size * default_distance;
    if (size_adjustment != 0.0) {
        bool zoom_on_focus = s->display_distance < default_distance;
        double effective   = zoom_on_focus ? s->display_distance : default_distance;
        write_control("breezy_desktop_display_distance",
                      effective / size_adjustment);
    }

    write_control("breezy_desktop_follow_threshold", s->follow_threshold);
}
