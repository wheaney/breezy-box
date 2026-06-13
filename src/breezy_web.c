/*
 * breezy_web — landing page server for Breezy Box.
 *
 * Listens on :80 (requires CAP_NET_BIND_SERVICE).
 *
 * Routes:
 *   GET  /             → serve web/index.html
 *   GET  /status       → JSON: VNC reachability + renderer state
 *   POST /restart/:svc → restart a named user service via systemctl
 *
 * Build: see Makefile (breezy_web target).
 * Run:   breezy_web [--port 80] [--web-root /path/to/web/]
 */

#define MG_ENABLE_PACKED_FS 0

#include "vendor/mongoose.h"

#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#define DEFAULT_LISTEN_ADDR  "http://0.0.0.0:80"
#define DEFAULT_WEB_ROOT     "web"
#define MAX_WEB_ROOT         256

/* ------------------------------------------------------------------ */

static volatile int g_stop = 0;
static char g_web_root[MAX_WEB_ROOT] = DEFAULT_WEB_ROOT;

static void on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

/*
 * Allowed service names for the /restart route.
 * Prevents arbitrary systemctl calls from an open endpoint.
 */
static const char *const RESTARTABLE_SERVICES[] = {
	"xr-driver",
	"breezy-renderer",
	"breezy-desktop",
	"breezy-x11vnc",
	"breezy-novnc",
	NULL,
};

static bool service_name_allowed(const char *name)
{
	for (size_t i = 0; RESTARTABLE_SERVICES[i]; i++) {
		if (strcmp(name, RESTARTABLE_SERVICES[i]) == 0)
			return true;
	}
	return false;
}

static int restart_user_service(const char *service)
{
	pid_t pid = fork();
	if (pid < 0)
		return -1;
	if (pid == 0) {
		/* Suppress output in the child. */
		int devnull = open("/dev/null", O_WRONLY);
		if (devnull >= 0) {
			dup2(devnull, STDOUT_FILENO);
			dup2(devnull, STDERR_FILENO);
			close(devnull);
		}
		execlp("systemctl", "systemctl", "--user", "restart", service,
		       (char *)NULL);
		_exit(127);
	}
	int status;
	waitpid(pid, &status, 0);
	return (WIFEXITED(status) && WEXITSTATUS(status) == 0) ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/* Route handlers                                                       */
/* ------------------------------------------------------------------ */

static void handle_index(struct mg_connection *c)
{
	char path[MAX_WEB_ROOT + 32];
	snprintf(path, sizeof(path), "%s/index.html", g_web_root);

	FILE *f = fopen(path, "r");
	if (!f) {
		mg_http_reply(c, 404, "", "Not found\n");
		return;
	}
	fseek(f, 0, SEEK_END);
	long len = ftell(f);
	rewind(f);

	char *buf = malloc((size_t)len + 1);
	if (!buf) {
		fclose(f);
		mg_http_reply(c, 500, "", "Out of memory\n");
		return;
	}
	fread(buf, 1, (size_t)len, f);
	buf[len] = '\0';
	fclose(f);

	mg_http_reply(c, 200, "Content-Type: text/html\r\n", "%s", buf);
	free(buf);
}

static void handle_status(struct mg_connection *c)
{
	/*
	 * TODO: query real renderer state (e.g. via a Unix socket or shared
	 * state file written by displaylink_kms_renderer).  For now, report
	 * static VNC reachability.
	 */
	mg_http_reply(c, 200,
	              "Content-Type: application/json\r\n",
	              "{\"vnc_port\":5900,\"novnc_port\":8080,\"status\":\"ok\"}\n");
}

static void handle_restart(struct mg_connection *c, struct mg_http_message *hm)
{
	/* Extract the service name from the URI: /restart/<name> */
	struct mg_str uri = hm->uri;
	const char *prefix = "/restart/";
	size_t prefix_len = strlen(prefix);

	if (uri.len <= prefix_len) {
		mg_http_reply(c, 400, "", "Missing service name\n");
		return;
	}

	char service[64] = {0};
	size_t name_len = uri.len - prefix_len;
	if (name_len >= sizeof(service)) {
		mg_http_reply(c, 400, "", "Service name too long\n");
		return;
	}
	memcpy(service, uri.buf + prefix_len, name_len);

	if (!service_name_allowed(service)) {
		mg_http_reply(c, 403, "", "Service not in allowlist\n");
		return;
	}

	int rc = restart_user_service(service);
	if (rc == 0) {
		mg_http_reply(c, 200,
		              "Content-Type: application/json\r\n",
		              "{\"restarted\":\"%s\"}\n", service);
	} else {
		mg_http_reply(c, 500,
		              "Content-Type: application/json\r\n",
		              "{\"error\":\"restart failed\",\"service\":\"%s\"}\n", service);
	}
}

/* ------------------------------------------------------------------ */
/* Mongoose event handler                                               */
/* ------------------------------------------------------------------ */

static void ev_handler(struct mg_connection *c, int ev, void *ev_data)
{
	if (ev != MG_EV_HTTP_MSG)
		return;

	struct mg_http_message *hm = (struct mg_http_message *)ev_data;

	if (mg_match(hm->uri, mg_str("/"), NULL)) {
		handle_index(c);
	} else if (mg_match(hm->uri, mg_str("/status"), NULL)) {
		handle_status(c);
	} else if (mg_match(hm->uri, mg_str("/restart/*"), NULL) &&
	           mg_match(hm->method, mg_str("POST"), NULL)) {
		handle_restart(c, hm);
	} else {
		mg_http_reply(c, 404, "", "Not found\n");
	}
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

static void usage(const char *prog)
{
	fprintf(stderr,
	        "Usage: %s [--port PORT] [--web-root DIR]\n"
	        "  --port PORT      TCP port to listen on (default: 80)\n"
	        "  --web-root DIR   Directory containing index.html (default: web)\n",
	        prog);
}

int main(int argc, char *argv[])
{
	char listen_addr[64];
	snprintf(listen_addr, sizeof(listen_addr), "%s", DEFAULT_LISTEN_ADDR);

	static const struct option longopts[] = {
		{"port",     required_argument, NULL, 'p'},
		{"web-root", required_argument, NULL, 'w'},
		{"help",     no_argument,       NULL, 'h'},
		{NULL, 0, NULL, 0},
	};

	int opt;
	while ((opt = getopt_long(argc, argv, "p:w:h", longopts, NULL)) != -1) {
		switch (opt) {
		case 'p':
			snprintf(listen_addr, sizeof(listen_addr),
			         "http://0.0.0.0:%s", optarg);
			break;
		case 'w':
			snprintf(g_web_root, sizeof(g_web_root), "%s", optarg);
			break;
		case 'h':
			usage(argv[0]);
			return 0;
		default:
			usage(argv[0]);
			return 1;
		}
	}

	signal(SIGINT,  on_signal);
	signal(SIGTERM, on_signal);

	struct mg_mgr mgr;
	mg_mgr_init(&mgr);

	struct mg_connection *c = mg_http_listen(&mgr, listen_addr, ev_handler, NULL);
	if (!c) {
		fprintf(stderr, "breezy_web: failed to listen on %s\n", listen_addr);
		mg_mgr_free(&mgr);
		return 1;
	}

	printf("breezy_web: listening on %s, web root: %s\n", listen_addr, g_web_root);

	while (!g_stop)
		mg_mgr_poll(&mgr, 100);

	mg_mgr_free(&mgr);
	return 0;
}
