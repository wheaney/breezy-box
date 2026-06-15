#define _POSIX_C_SOURCE 200809L

#include "breezy_settings.h"

#include <gio/gio.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

struct breezy_settings_handle {
    GMainLoop  *loop;
    GSettings  *gs;
    pthread_t   thread;

    pthread_mutex_t mu;
    bool            dirty;
    struct breezy_display_settings current;
};

/* ---- helpers ---- */

static void read_into(GSettings *gs, struct breezy_display_settings *s)
{
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
}

static void on_changed(GSettings *gs, const gchar *key, gpointer user_data)
{
    (void)key;
    struct breezy_settings_handle *h = user_data;

    struct breezy_display_settings tmp = h->current;
    read_into(gs, &tmp);

    pthread_mutex_lock(&h->mu);
    h->current = tmp;
    h->dirty   = true;
    pthread_mutex_unlock(&h->mu);
}

static void *glib_thread_main(void *arg)
{
    struct breezy_settings_handle *h = arg;
    g_main_loop_run(h->loop);
    return NULL;
}

/* ---- public API ---- */

breezy_settings_handle *breezy_settings_start(struct breezy_display_settings *initial)
{
    struct breezy_settings_handle *h = calloc(1, sizeof(*h));
    if (!h)
        return NULL;

    pthread_mutex_init(&h->mu, NULL);

    /* Schema defaults in case read_into partially fails. */
    memset(&h->current, 0, sizeof(h->current));
    strncpy(h->current.monitor_wrapping_scheme, "automatic",
            sizeof(h->current.monitor_wrapping_scheme) - 1);
    h->current.display_distance              = 1.05;
    h->current.display_size                  = 1.0;
    h->current.toggle_display_distance_start = 0.85;
    h->current.toggle_display_distance_end   = 1.05;
    h->current.look_ahead_override           = -1;
    h->current.follow_threshold              = 15.0;

    GMainContext *ctx = g_main_context_new();
    h->loop = g_main_loop_new(ctx, FALSE);

    /* Push the context so GSettings binds its signal delivery to our loop. */
    g_main_context_push_thread_default(ctx);

    h->gs = g_settings_new_with_path(BREEZY_GSETTINGS_SCHEMA, BREEZY_GSETTINGS_PATH);
    if (!h->gs) {
        fprintf(stderr, "breezy_settings: failed to open schema %s\n",
                BREEZY_GSETTINGS_SCHEMA);
        g_main_context_pop_thread_default(ctx);
        g_main_context_unref(ctx);
        g_main_loop_unref(h->loop);
        pthread_mutex_destroy(&h->mu);
        free(h);
        return NULL;
    }

    /* Read initial values before starting the thread. */
    read_into(h->gs, &h->current);
    *initial = h->current;

    g_signal_connect(h->gs, "changed", G_CALLBACK(on_changed), h);

    g_main_context_pop_thread_default(ctx);
    g_main_context_unref(ctx); /* loop holds the remaining ref */

    if (pthread_create(&h->thread, NULL, glib_thread_main, h) != 0) {
        perror("breezy_settings: pthread_create");
        g_object_unref(h->gs);
        g_main_loop_unref(h->loop);
        pthread_mutex_destroy(&h->mu);
        free(h);
        return NULL;
    }

    return h;
}

bool breezy_settings_consume_if_changed(breezy_settings_handle *h,
                                        struct breezy_display_settings *out)
{
    pthread_mutex_lock(&h->mu);
    bool changed = h->dirty;
    if (changed) {
        *out     = h->current;
        h->dirty = false;
    }
    pthread_mutex_unlock(&h->mu);
    return changed;
}

void breezy_settings_stop(breezy_settings_handle *h)
{
    if (!h)
        return;
    g_main_loop_quit(h->loop);
    pthread_join(h->thread, NULL);
    g_object_unref(h->gs);
    g_main_loop_unref(h->loop);
    pthread_mutex_destroy(&h->mu);
    free(h);
}

/* ---- pure helpers (unchanged) ---- */

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
