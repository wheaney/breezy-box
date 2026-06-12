#define _XOPEN_SOURCE 700

#include "breezy_ui.h"

#include <errno.h>
#include <pwd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define BREEZY_UI_REL_PATH "/.local/bin/breezydesktop"

void breezy_ui_config_defaults(struct breezy_ui_config *cfg)
{
    struct passwd *pw;

    if (!cfg)
        return;
    memset(cfg, 0, sizeof(*cfg));

    pw = getpwuid(getuid());
    if (pw && pw->pw_dir && pw->pw_dir[0])
        snprintf(cfg->path, sizeof(cfg->path), "%s%s", pw->pw_dir, BREEZY_UI_REL_PATH);
    else
        snprintf(cfg->path, sizeof(cfg->path), "%s", BREEZY_UI_REL_PATH);
    snprintf(cfg->env[0], sizeof(cfg->env[0]), "GDK_BACKEND=broadway");
    snprintf(cfg->env[1], sizeof(cfg->env[1]), "BROADWAY_DISPLAY=:0");
    cfg->env_count = 2;
    cfg->enabled = true;
}

int breezy_ui_start(const struct breezy_ui_config *cfg,
                    struct breezy_ui_state *state)
{
    struct stat st;
    pid_t pid;
    size_t i;

    if (!cfg || !state)
        return -1;

    state->pid = 0;

    if (!cfg->enabled)
        return 0;

    if (cfg->path[0] == '\0') {
        fprintf(stderr, "breezy_ui: no path configured\n");
        return 0;
    }

    if (stat(cfg->path, &st) != 0) {
        fprintf(stderr, "breezy_ui: %s not found, skipping (%s)\n",
                cfg->path, strerror(errno));
        return 0;
    }

    pid = fork();
    if (pid < 0) {
        perror("breezy_ui: fork");
        return 0; /* best-effort */
    }

    if (pid == 0) {
        /* Child: inject env vars then exec */
        for (i = 0; i < cfg->env_count && i < BREEZY_UI_ENV_MAX; i++) {
            if (cfg->env[i][0])
                putenv((char *)cfg->env[i]);
        }
        char *const argv[] = {(char *)cfg->path, NULL};
        execv(cfg->path, argv);
        fprintf(stderr, "breezy_ui: execv %s failed: %s\n",
                cfg->path, strerror(errno));
        _exit(127);
    }

    state->pid = pid;
    printf("breezy_ui: started %s (pid %ld)\n", cfg->path, (long)pid);
    return 0;
}

void breezy_ui_stop(struct breezy_ui_state *state)
{
    int i;
    struct timespec ts = {0, 50000000L}; /* 50 ms */

    if (!state || state->pid == 0)
        return;

    kill(state->pid, SIGTERM);

    /* Wait up to ~2 s for a clean exit before escalating. */
    for (i = 0; i < 40; i++) {
        int status;
        pid_t rc = waitpid(state->pid, &status, WNOHANG);

        if (rc == state->pid) {
            state->pid = 0;
            return;
        }
        if (rc < 0) {
            state->pid = 0;
            return;
        }
        nanosleep(&ts, NULL);
    }

    /* Still running after timeout — escalate. */
    fprintf(stderr, "breezy_ui: pid %ld did not exit cleanly, sending SIGKILL\n",
            (long)state->pid);
    kill(state->pid, SIGKILL);
    waitpid(state->pid, NULL, 0);
    state->pid = 0;
}

void breezy_ui_reap(struct breezy_ui_state *state)
{
    int status;
    pid_t rc;

    if (!state || state->pid == 0)
        return;

    rc = waitpid(state->pid, &status, WNOHANG);
    if (rc == state->pid) {
        if (WIFEXITED(status))
            fprintf(stderr, "breezy_ui: pid %ld exited with status %d\n",
                    (long)state->pid, WEXITSTATUS(status));
        else if (WIFSIGNALED(status))
            fprintf(stderr, "breezy_ui: pid %ld killed by signal %d\n",
                    (long)state->pid, WTERMSIG(status));
        state->pid = 0;
    }
}
