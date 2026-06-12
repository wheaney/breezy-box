#define _POSIX_C_SOURCE 200809L

#include "common.h"
#include "server.h"

#include <arpa/inet.h>
#include <errno.h>
#include <getopt.h>
#include <json-c/json.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>

void usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s [options]\n"
		"  --config PATH           Persistent JSON config file (default: %s)\n"
		"  --verbose               Force verbose logging for all configured devices\n"
		"  --help                  Show this message\n",
		argv0,
		DEFAULT_CONFIG_PATH);
}

int parse_hex16(const char *text, uint16_t *value)
{
	char *end = NULL;
	unsigned long parsed;

	errno = 0;
	parsed = strtoul(text, &end, 0);
	if (errno != 0 || !end || *end != '\0' || parsed > 0xffffUL)
		return -1;

	*value = (uint16_t)parsed;
	return 0;
}

int parse_usb_speed(const char *text, enum usb_device_speed *speed)
{
	if (!text || !speed)
		return -1;
	if (strcmp(text, "high") == 0) {
		*speed = USB_SPEED_HIGH;
		return 0;
	}
	if (strcmp(text, "super") == 0) {
		*speed = USB_SPEED_SUPER;
		return 0;
	}

	return -1;
}

static bool env_flag_enabled(const char *name)
{
	const char *value;

	if (!name)
		return false;
	value = getenv(name);
	return value && value[0] != '\0' && strcmp(value, "0") != 0;
}

void default_options(struct options *opts)
{
	memset(opts, 0, sizeof(*opts));
	opts->busid = DEFAULT_BUSID;
	opts->device_path = DEFAULT_DEVICE_PATH;
	opts->manufacturer_string = DEFAULT_MANUFACTURER_STRING;
	opts->product_string = DEFAULT_PRODUCT_STRING;
	opts->serial_string = DEFAULT_SERIAL_STRING;
	opts->monitor_name = DEFAULT_MONITOR_NAME;
	opts->vendor_id = DISPLAYLINK_VENDOR_ID;
	opts->product_id = DISPLAYLINK_PRODUCT_ID;
	opts->usb_speed = USB_SPEED_HIGH;
	opts->decode_width = DEFAULT_DISPLAY_WIDTH;
	opts->decode_height = DEFAULT_DISPLAY_HEIGHT;
	opts->refresh_hz = DEFAULT_REFRESH_HZ;
	opts->window_scale = 1u;
	opts->strict_native_mode = DEFAULT_STRICT_NATIVE_MODE;
	opts->allow_30hz_fallback = DEFAULT_ALLOW_30HZ_FALLBACK;
	opts->decode_stream = true;
	opts->show_window = false;
	opts->verbose = false;
}

int parse_runtime_cli_args(int argc, char **argv, struct runtime_cli_args *args)
{
	static const struct option long_options[] = {
		{ "config", required_argument, NULL, 'c' },
		{ "verbose", no_argument, NULL, 'V' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int opt;

	if (!args)
		return -1;
	args->config_path = DEFAULT_CONFIG_PATH;
	args->force_verbose = false;

	optind = 1;
	while ((opt = getopt_long(argc, argv, "c:Vh", long_options, NULL)) != -1) {
		switch (opt) {
		case 'c':
			args->config_path = optarg;
			break;
		case 'V':
			args->force_verbose = true;
			break;
		case 'h':
			usage(argv[0]);
			exit(0);
		default:
			return -1;
		}
	}

	return 0;
}

void server_options_init(struct server_options *opts)
{
	if (!opts)
		return;

	memset(opts, 0, sizeof(*opts));
	(void)snprintf(opts->listen_host, sizeof(opts->listen_host), "%s", DEFAULT_LISTEN_HOST);
	opts->listen_port = USBIP_DEFAULT_PORT;
	opts->show_window = true;
	opts->window_scale = 1u;
	opts->verbose = false;
}

void configured_device_default(struct configured_device *device, size_t index)
{
	if (!device)
		return;

	memset(device, 0, sizeof(*device));
	(void)snprintf(device->name, sizeof(device->name), "display-%zu", index + 1u);
	(void)snprintf(device->busid, sizeof(device->busid), "breezy-box-%zu", index);
	(void)snprintf(device->device_path,
		       sizeof(device->device_path),
		       "/devices/platform/breezy-box/displaylink%zu",
		       index);
	(void)snprintf(device->manufacturer_string,
		       sizeof(device->manufacturer_string),
		       "%s",
		       DEFAULT_MANUFACTURER_STRING);
	(void)snprintf(device->product_string,
		       sizeof(device->product_string),
		       "%s",
		       DEFAULT_PRODUCT_STRING);
	(void)snprintf(device->serial_string,
		       sizeof(device->serial_string),
		       "BREEZY%04zu",
		       index + 1u);
	(void)snprintf(device->monitor_name,
		       sizeof(device->monitor_name),
		       "Breezy Box %zu",
		       index + 1u);
	device->vendor_id = DISPLAYLINK_VENDOR_ID;
	device->product_id = DISPLAYLINK_PRODUCT_ID;
	device->usb_speed = USB_SPEED_HIGH;
	device->decode_width = DEFAULT_DISPLAY_WIDTH;
	device->decode_height = DEFAULT_DISPLAY_HEIGHT;
	device->refresh_hz = DEFAULT_REFRESH_HZ;
	device->x = 0;
	device->y = 0;
	device->strict_native_mode = DEFAULT_STRICT_NATIVE_MODE;
	device->allow_30hz_fallback = DEFAULT_ALLOW_30HZ_FALLBACK;
	device->decode_stream = true;
	device->verbose = false;
}

static void configured_device_apply_json(struct json_object *obj,
					  struct configured_device *dev,
					  bool is_defaults)
{
	struct json_object *v;
	const char *s;

	if (!dev || !obj)
		return;

	if (!is_defaults && json_object_object_get_ex(obj, "name", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(dev->name, sizeof(dev->name), "%s", s);
	}
	if (json_object_object_get_ex(obj, "busid", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(dev->busid, sizeof(dev->busid), "%s", s);
	}
	if (json_object_object_get_ex(obj, "device_path", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(dev->device_path, sizeof(dev->device_path), "%s", s);
	}
	if (json_object_object_get_ex(obj, "edid_path", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(dev->edid_path, sizeof(dev->edid_path), "%s", s);
	}
	if (json_object_object_get_ex(obj, "dump_image_path", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(dev->dump_image_path, sizeof(dev->dump_image_path), "%s", s);
	}
	if (json_object_object_get_ex(obj, "manufacturer_string", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(dev->manufacturer_string, sizeof(dev->manufacturer_string), "%s", s);
	}
	if (json_object_object_get_ex(obj, "product_string", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(dev->product_string, sizeof(dev->product_string), "%s", s);
	}
	if (json_object_object_get_ex(obj, "serial_string", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(dev->serial_string, sizeof(dev->serial_string), "%s", s);
	}
	if (json_object_object_get_ex(obj, "monitor_name", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(dev->monitor_name, sizeof(dev->monitor_name), "%s", s);
	}

	if (json_object_object_get_ex(obj, "vendor_id", &v)) {
		if (json_object_is_type(v, json_type_string)) {
			(void)parse_hex16(json_object_get_string(v), &dev->vendor_id);
		} else {
			int64_t n = json_object_get_int64(v);
			if (n >= 0 && n <= UINT16_MAX)
				dev->vendor_id = (uint16_t)n;
		}
	}
	if (json_object_object_get_ex(obj, "product_id", &v)) {
		if (json_object_is_type(v, json_type_string)) {
			(void)parse_hex16(json_object_get_string(v), &dev->product_id);
		} else {
			int64_t n = json_object_get_int64(v);
			if (n >= 0 && n <= UINT16_MAX)
				dev->product_id = (uint16_t)n;
		}
	}
	if (json_object_object_get_ex(obj, "usb_speed", &v)) {
		s = json_object_get_string(v);
		if (s) {
			if (parse_usb_speed(s, &dev->usb_speed) != 0)
				fprintf(stderr, "invalid usb_speed '%s' in config\n", s);
		}
	}
	if (json_object_object_get_ex(obj, "decode_width", &v))
		dev->decode_width = (uint32_t)json_object_get_int(v);
	if (json_object_object_get_ex(obj, "decode_height", &v))
		dev->decode_height = (uint32_t)json_object_get_int(v);
	if (json_object_object_get_ex(obj, "refresh_hz", &v))
		dev->refresh_hz = (uint32_t)json_object_get_int(v);
	if (json_object_object_get_ex(obj, "x", &v))
		dev->x = (int32_t)json_object_get_int(v);
	if (json_object_object_get_ex(obj, "y", &v))
		dev->y = (int32_t)json_object_get_int(v);
	if (json_object_object_get_ex(obj, "strict_native_mode", &v))
		dev->strict_native_mode = json_object_get_boolean(v);
	if (json_object_object_get_ex(obj, "allow_30hz_fallback", &v))
		dev->allow_30hz_fallback = json_object_get_boolean(v);
	if (json_object_object_get_ex(obj, "decode_stream", &v))
		dev->decode_stream = json_object_get_boolean(v);
	if (json_object_object_get_ex(obj, "verbose", &v))
		dev->verbose = json_object_get_boolean(v);
}

static int parse_config_devices_array(struct json_object *arr,
				      const struct configured_device *defaults,
				      struct server_options *opts)
{
	int i, n;

	if (!arr || !defaults || !opts || !json_object_is_type(arr, json_type_array))
		return -1;

	n = json_object_array_length(arr);
	for (i = 0; i < n; i++) {
		struct json_object *dev_obj = json_object_array_get_idx(arr, i);

		if (!dev_obj || !json_object_is_type(dev_obj, json_type_object))
			continue;
		if (opts->device_count >= MAX_USBIP_DEVICES) {
			fprintf(stderr, "config declares more than %u devices\n", MAX_USBIP_DEVICES);
			return -1;
		}
		opts->devices[opts->device_count] = *defaults;
		configured_device_apply_json(dev_obj, &opts->devices[opts->device_count], false);
		opts->device_count += 1u;
	}

	return opts->device_count == 0u ? -1 : 0;
}

static int validate_server_options(const struct server_options *opts)
{
	size_t i;
	size_t j;

	if (!opts)
		return -1;
	if (opts->device_count == 0u) {
		fprintf(stderr, "config must define at least one device\n");
		return -1;
	}
	if (opts->window_scale == 0u) {
		fprintf(stderr, "window_scale must be at least 1\n");
		return -1;
	}

	for (i = 0u; i < opts->device_count; ++i) {
		const struct configured_device *d = &opts->devices[i];

		if (d->busid[0] == '\0' || d->device_path[0] == '\0') {
			fprintf(stderr, "device %zu is missing busid or device_path\n", i + 1u);
			return -1;
		}
		if (strlen(d->busid) >= SYSFS_BUS_ID_SIZE) {
			fprintf(stderr, "device %zu busid is too long\n", i + 1u);
			return -1;
		}
		if (strlen(d->device_path) >= SYSFS_PATH_MAX) {
			fprintf(stderr, "device %zu device_path is too long\n", i + 1u);
			return -1;
		}
		if (d->decode_width == 0u || d->decode_height == 0u) {
			fprintf(stderr, "device %zu decode dimensions must be non-zero\n", i + 1u);
			return -1;
		}
		if (d->refresh_hz == 0u || d->refresh_hz > 240u) {
			fprintf(stderr, "device %zu refresh_hz must be between 1 and 240\n", i + 1u);
			return -1;
		}
		for (j = i + 1u; j < opts->device_count; ++j) {
			if (strcmp(d->busid, opts->devices[j].busid) == 0) {
				fprintf(stderr,
					"devices %zu and %zu share busid '%s'\n",
					i + 1u,
					j + 1u,
					d->busid);
				return -1;
			}
		}
	}

	return 0;
}

int load_server_options_from_config(const char *config_path,
				   const struct runtime_cli_args *cli,
				   struct server_options *opts)
{
	struct json_object *root = NULL;
	struct json_object *v = NULL;
	struct configured_device defaults;
	const char *s;
	int64_t n64;
	int ret = -1;

	if (!config_path || !opts)
		return -1;

	server_options_init(opts);
	configured_device_default(&defaults, 0u);

	root = json_object_from_file(config_path);
	if (!root) {
		fprintf(stderr, "failed to parse config %s: %s\n",
			config_path, json_util_get_last_err());
		return -1;
	}

	if (json_object_object_get_ex(root, "listen_host", &v)) {
		s = json_object_get_string(v);
		if (s) snprintf(opts->listen_host, sizeof(opts->listen_host), "%s", s);
	}
	if (json_object_object_get_ex(root, "listen_port", &v)) {
		n64 = json_object_get_int64(v);
		if (n64 < 1 || n64 > UINT16_MAX) {
			fprintf(stderr, "listen_port out of range in %s\n", config_path);
			goto done;
		}
		opts->listen_port = (uint16_t)n64;
	}
	if (json_object_object_get_ex(root, "show_window", &v))
		opts->show_window = json_object_get_boolean(v);
	if (json_object_object_get_ex(root, "window_scale", &v))
		opts->window_scale = (uint32_t)json_object_get_int(v);
	if (json_object_object_get_ex(root, "verbose", &v))
		opts->verbose = json_object_get_boolean(v);

	if (json_object_object_get_ex(root, "defaults", &v))
		configured_device_apply_json(v, &defaults, true);

	if (json_object_object_get_ex(root, "usb_speed", &v)) {
		s = json_object_get_string(v);
		if (s && parse_usb_speed(s, &defaults.usb_speed) != 0) {
			fprintf(stderr, "invalid top-level usb_speed '%s'\n", s);
			goto done;
		}
	}

	if (!json_object_object_get_ex(root, "devices", &v)) {
		fprintf(stderr, "config is missing a 'devices' array\n");
		goto done;
	}
	if (parse_config_devices_array(v, &defaults, opts) != 0) {
		fprintf(stderr, "failed to parse devices array in config\n");
		goto done;
	}

	if (cli && cli->force_verbose)
		opts->verbose = true;
	if (opts->verbose) {
		size_t i;

		for (i = 0u; i < opts->device_count; ++i)
			opts->devices[i].verbose = true;
	}

	ret = validate_server_options(opts);

done:
	json_object_put(root);
	return ret;
}

int send_devlist_reply(const struct server_runtime *server, int fd)
{
	struct op_devlist_reply reply;
	size_t i;

	if (!server)
		return -1;

	memset(&reply, 0, sizeof(reply));
	{
		uint32_t ndev = 0u;

		for (i = 0u; i < server->opts.device_count; ++i)
			if (!server->devices[i].is_gadget_device)
				ndev++;
		reply.ndev = pack_u32(1, ndev);
	}
	if (send_op_common_with_payload(fd, OP_REP_DEVLIST, ST_OK, &reply, sizeof(reply)) != 0)
		return -1;

	for (i = 0u; i < server->opts.device_count; ++i) {
		struct usbip_usb_device packed_device;
		struct iovec iov[2];

		if (server->devices[i].is_gadget_device)
			continue;
		packed_device = server->devices[i].usbip_device;
		pack_usbip_usb_device(1, &packed_device);
		iov[0].iov_base = &packed_device;
		iov[0].iov_len = sizeof(packed_device);
		iov[1].iov_base = (void *)&server->devices[i].usbip_interface;
		iov[1].iov_len = sizeof(server->devices[i].usbip_interface);
		if (send_iov_all(fd, iov, 2) !=
		    (ssize_t)(sizeof(packed_device) + sizeof(server->devices[i].usbip_interface)))
			return -1;
	}
	return 0;
}

struct device_runtime *find_device_by_busid(struct server_runtime *server, const char *busid)
{
	size_t i;

	if (!server || !busid)
		return NULL;

	for (i = 0u; i < server->opts.device_count; ++i) {
		if (server->devices[i].is_gadget_device)
			continue;
		if (strncmp(busid, server->devices[i].usbip_device.busid, SYSFS_BUS_ID_SIZE) == 0)
			return &server->devices[i];
	}

	return NULL;
}

int handle_import_request(struct server_runtime *server, int fd)
{
	struct op_import_request request;
	struct usbip_usb_device packed_device;
	uint32_t status = ST_OK;
	struct device_runtime *runtime = NULL;
	bool claimed = false;

	memset(&request, 0, sizeof(request));
	if (recv_all(fd, &request, sizeof(request)) != (ssize_t)sizeof(request))
		return -1;

	runtime = find_device_by_busid(server, request.busid);
	if (!runtime)
		status = ST_NODEV;
	else {
		pthread_mutex_lock(&runtime->import_mutex);
		if (runtime->imported) {
			status = ST_DEV_BUSY;
		} else {
			runtime->imported = true;
			claimed = true;
		}
		pthread_mutex_unlock(&runtime->import_mutex);
	}

	if (status != ST_OK)
		return send_op_common(fd, OP_REP_IMPORT, status);

	packed_device = runtime->usbip_device;
	pack_usbip_usb_device(1, &packed_device);
	if (send_op_common_with_payload(fd, OP_REP_IMPORT, status, &packed_device, sizeof(packed_device)) != 0)
		return -1;

	if (runtime->opts.verbose)
		fprintf(stderr, "USB/IP import attached for busid %s\n", runtime->usbip_device.busid);
	if (run_import_session(runtime, fd) != 0 && runtime->opts.verbose)
		fprintf(stderr, "USB/IP import session ended\n");
	if (claimed) {
		pthread_mutex_lock(&runtime->import_mutex);
		runtime->imported = false;
		pthread_mutex_unlock(&runtime->import_mutex);
	}
	runtime->current_configuration = 0u;
	return 0;
}

int create_listener(const struct server_options *opts)
{
	struct sockaddr_in address;
	int fd;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		perror("socket");
		return -1;
	}
	if (set_socket_options(fd) != 0) {
		perror("setsockopt");
		close(fd);
		return -1;
	}

	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_port = htons(opts->listen_port);
	if (inet_pton(AF_INET, opts->listen_host, &address.sin_addr) != 1) {
		fprintf(stderr, "Invalid IPv4 listen address '%s'\n", opts->listen_host);
		close(fd);
		return -1;
	}
	if (bind(fd, (const struct sockaddr *)&address, sizeof(address)) != 0) {
		perror("bind");
		close(fd);
		return -1;
	}
	if (listen(fd, SOMAXCONN) != 0) {
		perror("listen");
		close(fd);
		return -1;
	}

	return fd;
}

void *connection_thread_main(void *arg)
{
	struct connection_context *ctx = arg;
	struct server_runtime *server;
	int conn_fd;
	struct op_common op;

	if (!ctx)
		return NULL;
	server = ctx->server;
	conn_fd = ctx->fd;
	free(ctx);

	if (recv_op_common(conn_fd, &op) != 0) {
		close(conn_fd);
		return NULL;
	}

	if (server->opts.verbose)
		fprintf(stderr, "USB/IP request opcode=0x%04x\n", op.code);

	switch (op.code) {
	case OP_REQ_DEVLIST:
		(void)send_devlist_reply(server, conn_fd);
		break;
	case OP_REQ_IMPORT:
		(void)handle_import_request(server, conn_fd);
		break;
	default:
		if (server->opts.verbose)
			fprintf(stderr, "Unsupported management opcode 0x%04x\n", op.code);
		break;
	}

	close(conn_fd);
	return NULL;
}

int run_server(struct server_runtime *server)
{
	int listen_fd;
	size_t i;

	listen_fd = create_listener(&server->opts);
	if (listen_fd < 0)
		return -1;
	server->listen_fd = listen_fd;

	printf("DisplayLink USB/IP server listening on %s:%u\n",
	       server->opts.listen_host,
	       server->opts.listen_port);
	for (i = 0u; i < server->opts.device_count; ++i) {
		const struct device_runtime *runtime = &server->devices[i];

		printf("  [%zu] busid=%s speed=%s decode=%s %ux%u@%uhz",
		       i,
		       runtime->usbip_device.busid,
		       usb_speed_name(runtime->opts.usb_speed),
		       runtime->opts.decode_stream ? "on" : "off",
		       runtime->opts.decode_width,
		       runtime->opts.decode_height,
		       runtime->opts.refresh_hz);
		if (runtime->opts.decode_stream)
			printf(" backing=%ux%u", runtime->udl.backing_width, runtime->udl.backing_height);
		printf("\n");
	}

	while (!stop_requested) {
		struct sockaddr_in peer;
		socklen_t peer_len = sizeof(peer);
		int conn_fd;
		pthread_t worker;
		struct connection_context *ctx;

		conn_fd = accept(listen_fd, (struct sockaddr *)&peer, &peer_len);
		if (conn_fd < 0) {
			if (stop_requested)
				break;
			perror("accept");
			close(listen_fd);
			return -1;
		}
		(void)set_socket_options(conn_fd);

		ctx = calloc(1u, sizeof(*ctx));
		if (!ctx) {
			close(conn_fd);
			continue;
		}
		ctx->server = server;
		ctx->fd = conn_fd;
		if (pthread_create(&worker, NULL, connection_thread_main, ctx) != 0) {
			close(conn_fd);
			free(ctx);
			continue;
		}
		(void)pthread_detach(worker);
	}

	close(listen_fd);
	return 0;
}

void options_from_configured_device(const struct configured_device *src,
				   const struct server_options *server_opts,
				   struct options *dst)
{
	if (!src || !dst)
		return;

	default_options(dst);
	dst->busid = src->busid;
	dst->device_path = src->device_path;
	dst->edid_path = src->edid_path[0] == '\0' ? NULL : src->edid_path;
	dst->dump_image_path = src->dump_image_path[0] == '\0' ? NULL : src->dump_image_path;
	dst->manufacturer_string = src->manufacturer_string;
	dst->product_string = src->product_string;
	dst->serial_string = src->serial_string;
	dst->monitor_name = src->monitor_name;
	dst->vendor_id = src->vendor_id;
	dst->product_id = src->product_id;
	dst->usb_speed = src->usb_speed;
	dst->decode_width = src->decode_width;
	dst->decode_height = src->decode_height;
	dst->refresh_hz = src->refresh_hz;
	dst->strict_native_mode = src->strict_native_mode;
	dst->allow_30hz_fallback = src->allow_30hz_fallback;
	dst->decode_stream = src->decode_stream;
	dst->show_window = false;
	dst->verbose = src->verbose || (server_opts && server_opts->verbose);
	dst->window_scale = server_opts ? server_opts->window_scale : 1u;
}

int server_runtime_init_devices(struct server_runtime *server)
{
	size_t i;

	if (!server)
		return -1;

	for (i = 0u; i < server->opts.device_count; ++i) {
		struct device_runtime *runtime = &server->devices[i];

		memset(runtime, 0, sizeof(*runtime));
		options_from_configured_device(&server->opts.devices[i], &server->opts, &runtime->opts);
		runtime->current_configuration = 0u;
		runtime->imported = false;
		runtime->usbip_cadence_logging_enabled = runtime->opts.verbose ||
			env_flag_enabled("BREEZY_USBIP_CADENCE_STATS");
		udl_control_ram_seed(runtime);
		build_default_edid(runtime->edid,
				  runtime->opts.monitor_name,
				  runtime->opts.decode_width,
				  runtime->opts.decode_height,
				  runtime->opts.refresh_hz,
				  runtime->opts.strict_native_mode,
				  runtime->opts.allow_30hz_fallback);
		if (runtime->opts.edid_path && load_edid_file(runtime->opts.edid_path, runtime->edid) != 0)
			return -1;
		build_vendor_descriptor(runtime);
		build_device_descriptor(runtime);
		build_device_qualifier(runtime);
		build_bos_descriptor(runtime);
		fill_usbip_device(runtime);
		if (pthread_mutex_init(&runtime->import_mutex, NULL) != 0)
			return -1;
		runtime->import_mutex_initialized = true;
		runtime->bulk_recv_buf = malloc(MAX_TRANSFER_BUFFER_SIZE);
		if (!runtime->bulk_recv_buf) {
			fprintf(stderr, "Unable to allocate bulk receive buffer for busid '%s'\n",
				runtime->opts.busid);
			return -1;
		}
		runtime->bulk_recv_buf_size = MAX_TRANSFER_BUFFER_SIZE;
		if (udl_runtime_init(&runtime->udl, &runtime->opts) != 0) {
			fprintf(stderr,
				"Unable to initialize UDL decode runtime for busid '%s'\n",
				runtime->opts.busid);
			return -1;
		}
	}

	return 0;
}

void server_runtime_destroy(struct server_runtime *server)
{
	size_t i;

	if (!server)
		return;

	for (i = 0u; i < server->opts.device_count; ++i) {
		free(server->devices[i].bulk_recv_buf);
		server->devices[i].bulk_recv_buf = NULL;
		udl_runtime_destroy(&server->devices[i].udl);
		if (server->devices[i].import_mutex_initialized) {
			pthread_mutex_destroy(&server->devices[i].import_mutex);
			server->devices[i].import_mutex_initialized = false;
		}
	}
}
