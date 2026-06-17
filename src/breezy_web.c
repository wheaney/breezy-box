/*
 * breezy_web — landing page server for Breezy Box.
 *
 * Listens on :443 (HTTPS) or :80 (HTTP fallback when no cert is provided).
 * Requires CAP_NET_BIND_SERVICE, or run on an unprivileged port with an
 * nftables DNAT redirect (see setup_system.sh).
 *
 * Routes:
 *   GET  /                  → serve web/index.html
 *   GET  /status            → JSON: VNC reachability + renderer state
 *   POST /restart/:svc      → restart a named user service via systemctl
 *   GET  /settings/:key     → retrieve a GSettings key value (raw gsettings output)
 *   PUT  /settings/:key     → store a value; body: {"value":"..."}
 *   POST /monitors          → accept window-management monitor layout from browser;
 *                             body: array of {id, label, width, height, x, y};
 *                             updates x/y of matching devices in the config file
 *
 * Build: see Makefile (breezy_web target).
 * Run:   breezy_web [--port 443] [--web-root DIR] [--cert FILE] [--key FILE]
 */

#define MG_ENABLE_PACKED_FS 0

#include "vendor/mongoose.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include <json-c/json.h>
#include <sys/types.h>

#define DEFAULT_HTTP_LISTEN    "http://0.0.0.0:80"
#define DEFAULT_HTTPS_LISTEN   "https://0.0.0.0:443"
#define DEFAULT_WEB_ROOT       "web"
#define DEFAULT_CONFIG_RELPATH "breezy-box/config.json"
#define CONTROL_SIGNALS_PATH   "/dev/shm/xr_driver_control"
#define MAX_WEB_ROOT           256
#define MAX_PATH_ARG           4096
#define MAX_GSETTINGS_KEY      128
#define GSETTINGS_OUT_MAX      1024
#define MAX_CONTROL_SIGNAL_NAME 128

#define GSETTINGS_SCHEMA "com.xronlinux.BreezyDesktop"

static volatile int g_stop = 0;
static char g_web_root[MAX_WEB_ROOT] = DEFAULT_WEB_ROOT;
static char g_tls_cert_path[MAX_PATH_ARG] = "";
static char g_tls_key_path[MAX_PATH_ARG]  = "";

/* PEM contents loaded from the cert/key files at startup. Mongoose's built-in
 * TLS expects the certificate/key *data* in mg_tls_opts, not a file path, so we
 * read the files into these buffers once and reuse them for every connection. */
static struct mg_str g_tls_cert = {NULL, 0};
static struct mg_str g_tls_key  = {NULL, 0};

static void on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

/* If DBUS_SESSION_BUS_ADDRESS isn't inherited (e.g. breezy-web started before
 * the session bus was ready), fall back to the well-known XDG runtime socket. */
static void ensure_dbus_session_env(void)
{
	if (getenv("DBUS_SESSION_BUS_ADDRESS"))
		return;
	char path[128];
	snprintf(path, sizeof(path), "unix:path=/run/user/%d/bus", (int)getuid());
	setenv("DBUS_SESSION_BUS_ADDRESS", path, 0);
}

static int gsettings_get(const char *key, char *buf, size_t bufsz)
{
	int pipefd[2];
	if (pipe(pipefd) != 0)
		return -1;

	pid_t pid = fork();
	if (pid < 0) {
		close(pipefd[0]);
		close(pipefd[1]);
		return -1;
	}
	if (pid == 0) {
		close(pipefd[0]);
		dup2(pipefd[1], STDOUT_FILENO);
		close(pipefd[1]);
		ensure_dbus_session_env();
		execlp("gsettings", "gsettings", "get",
		       GSETTINGS_SCHEMA, key, (char *)NULL);
		_exit(127);
	}

	close(pipefd[1]);

	size_t total = 0;
	ssize_t n;
	while (total + 1 < bufsz &&
	       (n = read(pipefd[0], buf + total, bufsz - 1 - total)) > 0)
		total += (size_t)n;
	close(pipefd[0]);

	int status;
	waitpid(pid, &status, 0);

	while (total > 0 && (buf[total - 1] == '\n' || buf[total - 1] == '\r'))
		total--;
	buf[total] = '\0';

	return (WIFEXITED(status) && WEXITSTATUS(status) == 0) ? 0 : -1;
}

static int gsettings_set(const char *key, const char *value)
{
	pid_t pid = fork();
	if (pid < 0)
		return -1;
	if (pid == 0) {
		ensure_dbus_session_env();
		execlp("gsettings", "gsettings", "set",
		       GSETTINGS_SCHEMA, key, value, (char *)NULL);
		_exit(127);
	}
	int status;
	waitpid(pid, &status, 0);
	return (WIFEXITED(status) && WEXITSTATUS(status) == 0) ? 0 : -1;
}

static int control_signal_write(const char *signal, const char *value)
{
	int fd = open(CONTROL_SIGNALS_PATH, O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return -1;
	char buf[256];
	int n = snprintf(buf, sizeof(buf), "%s=%s\n", signal, value);
	ssize_t written = 0;
	while (written < n) {
		ssize_t w = write(fd, buf + written, (size_t)n - (size_t)written);
		if (w < 0) {
			close(fd);
			return -1;
		}
		written += w;
	}
	close(fd);
	return 0;
}

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
		execlp("systemctl", "systemctl", "--user", "restart", service,
		       (char *)NULL);
		_exit(127);
	}
	int status;
	waitpid(pid, &status, 0);
	return (WIFEXITED(status) && WEXITSTATUS(status) == 0) ? 0 : -1;
}

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
	mg_http_reply(c, 200,
	              "Content-Type: application/json\r\n",
	              "{\"vnc_port\":5900,\"novnc_port\":8080,\"status\":\"ok\"}\n");
}

static void handle_restart(struct mg_connection *c, struct mg_http_message *hm)
{
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

static void handle_settings_get(struct mg_connection *c, struct mg_http_message *hm)
{
	const char *prefix = "/settings/";
	size_t prefix_len = strlen(prefix);

	if (hm->uri.len <= prefix_len) {
		mg_http_reply(c, 400, "", "Missing key\n");
		return;
	}

	char key[MAX_GSETTINGS_KEY] = {0};
	size_t key_len = hm->uri.len - prefix_len;
	if (key_len >= sizeof(key)) {
		mg_http_reply(c, 400, "", "Key too long\n");
		return;
	}
	memcpy(key, hm->uri.buf + prefix_len, key_len);

	char out[GSETTINGS_OUT_MAX];
	if (gsettings_get(key, out, sizeof(out)) != 0) {
		mg_http_reply(c, 500,
		              "Content-Type: application/json\r\n",
		              "{\"error\":\"gsettings get failed\",\"key\":%m}\n",
		              MG_ESC(key));
		return;
	}

	mg_http_reply(c, 200,
	              "Content-Type: application/json\r\n",
	              "{\"key\":%m,\"value\":%m}\n",
	              MG_ESC(key), MG_ESC(out));
}

static void handle_settings_put(struct mg_connection *c, struct mg_http_message *hm)
{
	const char *prefix = "/settings/";
	size_t prefix_len = strlen(prefix);

	if (hm->uri.len <= prefix_len) {
		mg_http_reply(c, 400, "", "Missing key\n");
		return;
	}

	char key[MAX_GSETTINGS_KEY] = {0};
	size_t key_len = hm->uri.len - prefix_len;
	if (key_len >= sizeof(key)) {
		mg_http_reply(c, 400, "", "Key too long\n");
		return;
	}
	memcpy(key, hm->uri.buf + prefix_len, key_len);

	char *value = mg_json_get_str(hm->body, "$.value");
	if (!value) {
		mg_http_reply(c, 400,
		              "Content-Type: application/json\r\n",
		              "{\"error\":\"body must be JSON with a \\\"value\\\" field\"}\n");
		return;
	}

	int rc = gsettings_set(key, value);
	free(value);

	if (rc != 0) {
		mg_http_reply(c, 500,
		              "Content-Type: application/json\r\n",
		              "{\"error\":\"gsettings set failed\",\"key\":%m}\n",
		              MG_ESC(key));
		return;
	}

	mg_http_reply(c, 200,
	              "Content-Type: application/json\r\n",
	              "{\"key\":%m,\"status\":\"ok\"}\n",
	              MG_ESC(key));
}

static void handle_control_signal_put(struct mg_connection *c, struct mg_http_message *hm)
{
	const char *prefix = "/controls/";
	size_t prefix_len = strlen(prefix);

	if (hm->uri.len <= prefix_len) {
		mg_http_reply(c, 400, "", "Missing key\n");
		return;
	}

	char key[MAX_CONTROL_SIGNAL_NAME] = {0};
	size_t key_len = hm->uri.len - prefix_len;
	if (key_len >= sizeof(key)) {
		mg_http_reply(c, 400, "", "Key too long\n");
		return;
	}
	memcpy(key, hm->uri.buf + prefix_len, key_len);

	char *value = mg_json_get_str(hm->body, "$.value");
	if (!value) {
		mg_http_reply(c, 400,
		              "Content-Type: application/json\r\n",
		              "{\"error\":\"body must be JSON with a \\\"value\\\" field\"}\n");
		return;
	}

	int rc = control_signal_write(key, value);
	free(value);

	if (rc != 0) {
		mg_http_reply(c, 500,
		              "Content-Type: application/json\r\n",
		              "{\"error\":\"control signal write failed\",\"key\":%m}\n",
		              MG_ESC(key));
		return;
	}

	mg_http_reply(c, 200,
	              "Content-Type: application/json\r\n",
	              "{\"key\":%m,\"status\":\"ok\"}\n",
	              MG_ESC(key));
}

static void resolve_config_path(char *out, size_t outsz)
{
	const char *xdg  = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	if (xdg && xdg[0])
		snprintf(out, outsz, "%s/%s", xdg, DEFAULT_CONFIG_RELPATH);
	else if (home && home[0])
		snprintf(out, outsz, "%s/.config/%s", home, DEFAULT_CONFIG_RELPATH);
	else
		snprintf(out, outsz, "%s", DEFAULT_CONFIG_RELPATH);
}

static void handle_monitors_post(struct mg_connection *c, struct mg_http_message *hm)
{
	if (hm->body.len == 0) {
		mg_http_reply(c, 400, "Content-Type: application/json\r\n",
		              "{\"error\":\"empty body\"}\n");
		return;
	}

	struct mg_str body = hm->body;

	size_t i = 0;
	while (i < body.len && (body.buf[i] == ' ' || body.buf[i] == '\t' ||
	                         body.buf[i] == '\r' || body.buf[i] == '\n'))
		i++;
	if (i >= body.len || body.buf[i] != '[') {
		mg_http_reply(c, 400, "Content-Type: application/json\r\n",
		              "{\"error\":\"body must be a JSON array\"}\n");
		return;
	}

	size_t ofs = 0;
	int count = 0;
	struct mg_str mkey, mval;
	while ((ofs = mg_json_next(body, ofs, &mkey, &mval)) > 0) {
		static const char *required[] = { "$.label", "$.width", "$.height", "$.x", "$.y" };
		for (size_t r = 0; r < sizeof(required) / sizeof(required[0]); r++) {
			int toklen;
			if (mg_json_get(mval, required[r], &toklen) < 0) {
				mg_http_reply(c, 400, "Content-Type: application/json\r\n",
				              "{\"error\":\"monitor entry missing field\","
				              "\"field\":\"%s\"}\n", required[r] + 2);
				return;
			}
		}
		count++;
	}

	if (count == 0) {
		mg_http_reply(c, 400, "Content-Type: application/json\r\n",
		              "{\"error\":\"array must not be empty\"}\n");
		return;
	}

	char config_path[PATH_MAX];
	resolve_config_path(config_path, sizeof(config_path));

	struct json_object *root = json_object_from_file(config_path);
	if (!root) {
		mg_http_reply(c, 500, "Content-Type: application/json\r\n",
		              "{\"error\":\"failed to load config file\"}\n");
		return;
	}

	struct json_object *devices;
	if (!json_object_object_get_ex(root, "devices", &devices) ||
	    !json_object_is_type(devices, json_type_array)) {
		json_object_put(root);
		mg_http_reply(c, 500, "Content-Type: application/json\r\n",
		              "{\"error\":\"config has no devices array\"}\n");
		return;
	}

	int updated = 0;
	ofs = 0;
	while ((ofs = mg_json_next(body, ofs, &mkey, &mval)) > 0) {
		char *label = mg_json_get_str(mval, "$.label");
		if (!label)
			continue;
		long mx = mg_json_get_long(mval, "$.x", 0);
		long my = mg_json_get_long(mval, "$.y", 0);

		int n = json_object_array_length(devices);
		for (int d = 0; d < n; d++) {
			struct json_object *dev = json_object_array_get_idx(devices, d);
			struct json_object *name_obj;
			if (!json_object_object_get_ex(dev, "monitor_name", &name_obj))
				continue;
			const char *monitor_name = json_object_get_string(name_obj);
			if (!monitor_name || strcmp(monitor_name, label) != 0)
				continue;
			json_object_object_add(dev, "x", json_object_new_int((int)mx));
			json_object_object_add(dev, "y", json_object_new_int((int)my));
			updated++;
		}
		free(label);
	}

	char tmp[PATH_MAX + 8];
	snprintf(tmp, sizeof(tmp), "%s.tmp", config_path);

	if (json_object_to_file_ext(tmp, root,
	                            JSON_C_TO_STRING_PRETTY |
	                            JSON_C_TO_STRING_SPACED) != 0) {
		json_object_put(root);
		mg_http_reply(c, 500, "Content-Type: application/json\r\n",
		              "{\"error\":\"failed to write config\"}\n");
		return;
	}
	json_object_put(root);

	if (rename(tmp, config_path) != 0) {
		mg_http_reply(c, 500, "Content-Type: application/json\r\n",
		              "{\"error\":\"failed to replace config file\"}\n");
		return;
	}

	mg_http_reply(c, 200, "Content-Type: application/json\r\n",
	              "{\"status\":\"ok\",\"matched\":%d}\n", updated);
}

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
	} else if (mg_match(hm->uri, mg_str("/settings/*"), NULL) &&
	           mg_match(hm->method, mg_str("GET"), NULL)) {
		handle_settings_get(c, hm);
	} else if (mg_match(hm->uri, mg_str("/settings/*"), NULL) &&
	           mg_match(hm->method, mg_str("PUT"), NULL)) {
		handle_settings_put(c, hm);
	} else if (mg_match(hm->uri, mg_str("/monitors"), NULL) &&
	           mg_match(hm->method, mg_str("POST"), NULL)) {
		handle_monitors_post(c, hm);
	} else if (mg_match(hm->uri, mg_str("/controls/*"), NULL) &&
			   mg_match(hm->method, mg_str("PUT"), NULL)) {
		handle_control_signal_put(c, hm);
	} else {
		mg_http_reply(c, 404, "", "Not found\n");
	}
}

static void usage(const char *prog)
{
	fprintf(stderr,
	        "Usage: %s [--port PORT] [--web-root DIR] [--cert FILE] [--key FILE]\n"
	        "  --port PORT      TCP port to listen on (default: 443 with cert, 80 without)\n"
	        "  --web-root DIR   Directory containing index.html (default: web)\n"
	        "  --cert FILE      PEM certificate file for HTTPS\n"
	        "  --key  FILE      PEM private key file for HTTPS\n",
	        prog);
}

static void tls_ev_handler(struct mg_connection *c, int ev, void *ev_data)
{
	if (ev == MG_EV_ACCEPT) {
		struct mg_tls_opts opts = {
			.cert = g_tls_cert,
			.key  = g_tls_key,
		};
		mg_tls_init(c, &opts);
	}
	ev_handler(c, ev, ev_data);
}

int main(int argc, char *argv[])
{
	static const struct option longopts[] = {
		{"port",     required_argument, NULL, 'p'},
		{"web-root", required_argument, NULL, 'w'},
		{"cert",     required_argument, NULL, 'c'},
		{"key",      required_argument, NULL, 'k'},
		{"help",     no_argument,       NULL, 'h'},
		{NULL, 0, NULL, 0},
	};

	char port_override[16] = "";

	int opt;
	while ((opt = getopt_long(argc, argv, "p:w:c:k:h", longopts, NULL)) != -1) {
		switch (opt) {
		case 'p':
			snprintf(port_override, sizeof(port_override), "%s", optarg);
			break;
		case 'w':
			snprintf(g_web_root, sizeof(g_web_root), "%s", optarg);
			break;
		case 'c':
			snprintf(g_tls_cert_path, sizeof(g_tls_cert_path), "%s", optarg);
			break;
		case 'k':
			snprintf(g_tls_key_path, sizeof(g_tls_key_path), "%s", optarg);
			break;
		case 'h':
			usage(argv[0]);
			return 0;
		default:
			usage(argv[0]);
			return 1;
		}
	}

	bool use_tls = (g_tls_cert_path[0] != '\0' && g_tls_key_path[0] != '\0');

	if (use_tls) {
		g_tls_cert = mg_file_read(&mg_fs_posix, g_tls_cert_path);
		g_tls_key  = mg_file_read(&mg_fs_posix, g_tls_key_path);
		if (g_tls_cert.buf == NULL || g_tls_key.buf == NULL) {
			fprintf(stderr,
			        "breezy_web: failed to read TLS cert/key (%s, %s)\n",
			        g_tls_cert_path, g_tls_key_path);
			return 1;
		}
	}

	char listen_addr[64];
	if (port_override[0]) {
		snprintf(listen_addr, sizeof(listen_addr),
		         "%s://0.0.0.0:%s",
		         use_tls ? "https" : "http", port_override);
	} else {
		snprintf(listen_addr, sizeof(listen_addr), "%s",
		         use_tls ? DEFAULT_HTTPS_LISTEN : DEFAULT_HTTP_LISTEN);
	}

	signal(SIGINT,  on_signal);
	signal(SIGTERM, on_signal);

	struct mg_mgr mgr;
	mg_mgr_init(&mgr);

	mg_event_handler_t handler = use_tls ? tls_ev_handler : ev_handler;
	struct mg_connection *c = mg_http_listen(&mgr, listen_addr, handler, NULL);
	if (!c) {
		fprintf(stderr, "breezy_web: failed to listen on %s\n", listen_addr);
		mg_mgr_free(&mgr);
		return 1;
	}

	printf("breezy_web: listening on %s (%s), web root: %s\n",
	       listen_addr, use_tls ? "HTTPS" : "HTTP", g_web_root);

	while (!g_stop)
		mg_mgr_poll(&mgr, 100);

	mg_mgr_free(&mgr);
	mg_free((void *) g_tls_cert.buf);
	mg_free((void *) g_tls_key.buf);
	return 0;
}
