#pragma once

#include "usbip.h"
#include "udl_device.h"
#include "udl_runtime.h"

#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <pthread.h>
#include <linux/limits.h>

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

	/* Config file watcher — started after device init, stopped before destroy. */
	int    config_inotify_fd;
	pthread_t config_watcher_thread;
	bool   config_watcher_created;
	char   config_path[PATH_MAX];
	char   config_filename[NAME_MAX + 1]; /* basename of config_path */

	/* Per-slot liveness.  Replaces the old "contiguous 0..device_count" model:
	 * slots are stable, holes are allowed and reusable, and every lockless reader
	 * scans 0..MAX_USBIP_DEVICES gating on this flag (acquire load).  A slot's heavy
	 * resources (udl_runtime, bulk_recv_buf, descriptors, GL texture) exist iff the
	 * slot is active; set true only after the slot is fully built (release), cleared
	 * before any teardown.  The per-slot import_mutex, by contrast, is initialised
	 * once at startup and lives for the whole process. */
	atomic_bool slot_active[MAX_USBIP_DEVICES];

	/* Signals from watcher to consumers (KMS renderer).
	 * config_positions_changed: set when only x/y changed on any device;
	 *   consumer should re-run placement logic immediately (bypass 1 s throttle).
	 * config_textures_dirty[i]: set when device i's decode dimensions changed;
	 *   consumer must destroy and reinitialise the udl_runtime and GL texture.
	 * config_slot_add_pending[i]: watcher has built slot i's server-side resources
	 *   (still inactive); render thread must build its GL texture then activate it.
	 * config_slot_remove_pending[i]: watcher has quiesced + deactivated slot i;
	 *   render thread must tear down its GL texture then destroy server resources. */
	atomic_bool config_positions_changed;
	atomic_bool config_textures_dirty[MAX_USBIP_DEVICES];
	atomic_bool config_slot_add_pending[MAX_USBIP_DEVICES];
	atomic_bool config_slot_remove_pending[MAX_USBIP_DEVICES];
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

/*
 * Per-slot lifecycle primitives (slot index in [0, MAX_USBIP_DEVICES)).
 *
 * server_device_init_slot   builds a slot's heavy resources (udl_runtime, bulk
 *                           buffer, EDID/USB descriptors) from server->opts.devices[i].
 *                           Does NOT touch import_mutex (initialised once at startup)
 *                           and leaves the slot inactive.  Returns 0 / -1.
 * server_device_destroy_slot frees those heavy resources.  Does NOT destroy the
 *                           import_mutex.  The slot must already be inactive and
 *                           quiesced (no feeding session).
 * server_activate_slot      publish a fully-built slot (release): readers may now use it.
 * server_deactivate_slot    retract a slot (release) so lockless readers stop using it.
 * server_slot_is_active     acquire load of the slot's liveness flag.
 * server_find_free_slot     first inactive, non-add-pending slot index, or SIZE_MAX.
 * server_find_active_slot_by_busid  index of the active slot serving busid, or SIZE_MAX.
 */
int    server_device_init_slot(struct server_runtime *server, size_t i);
void   server_device_destroy_slot(struct server_runtime *server, size_t i);
void   server_activate_slot(struct server_runtime *server, size_t i);
void   server_deactivate_slot(struct server_runtime *server, size_t i);
bool   server_slot_is_active(const struct server_runtime *server, size_t i);
size_t server_find_free_slot(const struct server_runtime *server);
size_t server_find_active_slot_by_busid(const struct server_runtime *server,
					const char *busid);

/*
 * Start an inotify watcher on config_path.  When the file is modified the
 * watcher diffs the "devices" array against the live server_options and:
 *   - x/y-only change   → updates server->opts in-place, sets config_positions_changed
 *   - EDID field change → disconnects active client, rebuilds EDID/descriptors;
 *     if decode dimensions changed, also sets config_textures_dirty[i] so the
 *     KMS renderer can reinitialise the udl_runtime and GL texture.
 *
 * Must be called after server_runtime_init_devices.
 * Returns 0 on success, -1 on error (non-fatal: server continues without hot-reload).
 */
int  server_start_config_watcher(struct server_runtime *server, const char *config_path);
void server_stop_config_watcher(struct server_runtime *server);

/*
 * Clear the per-device reconfiguring guard after the KMS renderer has completed
 * a dimension-change reinit (udl_runtime + GL texture).  Until this is called the
 * device refuses USB/IP imports.  Called from the render thread; no-op for an
 * out-of-range index.
 */
void server_finish_device_reconfigure(struct server_runtime *server, size_t i);
