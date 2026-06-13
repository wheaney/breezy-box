#pragma once

#include "usbip.h"
#include "udl_device.h"
#include "udl_runtime.h"

#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <pthread.h>

/* Default server configuration values */
#define DEFAULT_LISTEN_HOST          "0.0.0.0"
#define DEFAULT_CONFIG_RELPATH       "breezy-box/config.json"
#define DEFAULT_BUSID                "breezy-box-0"
#define DEFAULT_DEVICE_PATH          "/devices/platform/breezy-box/displaylink0"
#define DEFAULT_MANUFACTURER_STRING  "DisplayLink"
#define DEFAULT_PRODUCT_STRING       "DisplayLink Adapter"
#define DEFAULT_SERIAL_STRING        "DEADBEEF0001"
#define DEFAULT_MONITOR_NAME         "Breezy Box"
#define DEFAULT_REFRESH_HZ           30u
#define DEFAULT_STRICT_NATIVE_MODE   true
#define DEFAULT_ALLOW_30HZ_FALLBACK  false
#define DEFAULT_DISPLAY_WIDTH        1920u
#define DEFAULT_DISPLAY_HEIGHT       1080u

#define MAX_USBIP_DEVICES 8u
#define STRING_FIELD_MAX  160u

#define USBIP_DEFAULT_PORT 3240u

/* Per-device configuration (from JSON config or CLI) */
struct configured_device {
	char name[STRING_FIELD_MAX];
	char busid[SYSFS_BUS_ID_SIZE];
	char device_path[SYSFS_PATH_MAX];
	char edid_path[STRING_FIELD_MAX];
	char dump_image_path[STRING_FIELD_MAX];
	char manufacturer_string[STRING_FIELD_MAX];
	char product_string[STRING_FIELD_MAX];
	char serial_string[STRING_FIELD_MAX];
	char monitor_name[STRING_FIELD_MAX];
	uint16_t vendor_id;
	uint16_t product_id;
	enum usb_device_speed usb_speed;
	uint32_t decode_width;
	uint32_t decode_height;
	uint32_t refresh_hz;
	int32_t x;
	int32_t y;
	bool strict_native_mode;
	bool allow_30hz_fallback;
	bool decode_stream;
	bool verbose;
};

/* Top-level server configuration */
struct server_options {
	char listen_host[STRING_FIELD_MAX];
	uint16_t listen_port;
	bool show_window;
	uint32_t window_scale;
	bool verbose;
	size_t device_count;
	struct configured_device devices[MAX_USBIP_DEVICES];
};

/* Server runtime: holds options and per-device runtimes */
struct server_runtime {
	struct server_options opts;
	struct device_runtime devices[MAX_USBIP_DEVICES];
	pthread_t viewer_thread;
	atomic_bool viewer_stop_requested;
	bool viewer_thread_created;
	int listen_fd; /* -1 until run_server opens it; shutdown() to unblock accept() */
};

/* Per-connection context passed to connection threads */
struct connection_context {
	struct server_runtime *server;
	int fd;
};

/* CLI arguments parsed before loading the full config */
struct runtime_cli_args {
	char config_path[PATH_MAX];
	bool force_verbose;
};

/* Function declarations */
void usage(const char *argv0);
int parse_hex16(const char *text, uint16_t *value);
int parse_usb_speed(const char *text, enum usb_device_speed *speed);
void default_options(struct options *opts);
int parse_runtime_cli_args(int argc, char **argv, struct runtime_cli_args *args);

void server_options_init(struct server_options *opts);
void configured_device_default(struct configured_device *device, size_t index);

int load_server_options_from_config(const char *config_path,
				    const struct runtime_cli_args *cli,
				    struct server_options *opts);

int send_devlist_reply(const struct server_runtime *server, int fd);
struct device_runtime *find_device_by_busid(struct server_runtime *server, const char *busid);
int handle_import_request(struct server_runtime *server, int fd);

int create_listener(const struct server_options *opts);
void *connection_thread_main(void *arg);
int run_server(struct server_runtime *server);

void options_from_configured_device(const struct configured_device *src,
				    const struct server_options *server_opts,
				    struct options *dst);
int server_runtime_init_devices(struct server_runtime *server);
void server_runtime_destroy(struct server_runtime *server);
