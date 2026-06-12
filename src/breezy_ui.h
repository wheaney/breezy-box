#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>

/*
 * breezy_ui — launch and track the breezydesktop UI process tied to the
 * lifetime of a rendering app.
 *
 * The rendering app starts the UI before entering its main loop and stops it
 * on the way out.  The UI is an independent process (fork+exec) so it doesn't
 * block the render loop, and its pid is tracked so it can be reaped cleanly
 * on shutdown.
 *
 * Lifecycle:
 *   breezy_ui_start()  — fork+exec the UI; records its pid
 *   breezy_ui_stop()   — SIGTERM → waitpid with timeout → SIGKILL fallback
 *   breezy_ui_reap()   — non-blocking reap; call after the render loop exits
 *                        to collect the child if it exited early
 *
 * Renderer-agnostic: any rendering binary (KMS, Wayland, ...) instantiates
 * one of these and wires start/stop around its own main loop.
 */

#define BREEZY_UI_PATH_MAX   256
#define BREEZY_UI_ENV_MAX    16   /* max extra environment variables */

struct breezy_ui_config {
    char path[BREEZY_UI_PATH_MAX];                      /* executable path */
    char env[BREEZY_UI_ENV_MAX][BREEZY_UI_PATH_MAX];    /* "KEY=VALUE" pairs prepended to child env */
    size_t env_count;
    bool enabled;
};

struct breezy_ui_state {
    pid_t pid;     /* 0 = not running */
};

/* Fill cfg with defaults for the breezydesktop UI. */
void breezy_ui_config_defaults(struct breezy_ui_config *cfg);

/*
 * Fork and exec the UI.  Best-effort: if the binary is absent or exec fails,
 * logs a warning and returns 0 so startup continues.  Returns -1 only on
 * invalid arguments.
 */
int breezy_ui_start(const struct breezy_ui_config *cfg,
                    struct breezy_ui_state *state);

/* Terminate the UI started by breezy_ui_start().  Idempotent. */
void breezy_ui_stop(struct breezy_ui_state *state);

/*
 * Non-blocking reap: if the UI has already exited, collect its status and
 * reset state->pid to 0.  Call after the render loop exits to avoid leaving
 * a zombie when the child exits before the parent does.
 */
void breezy_ui_reap(struct breezy_ui_state *state);
