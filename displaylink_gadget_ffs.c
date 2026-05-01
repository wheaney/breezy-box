#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/usb/ch9.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <usbg/usbg.h>

#define DISPLAYLINK_VENDOR_ID 0x17e9
#define DISPLAYLINK_PRODUCT_ID 0x0104
#define DISPLAYLINK_LANG 0x0409

#define FUNCTIONFS_DESCRIPTORS_MAGIC_V2 3U
#define FUNCTIONFS_STRINGS_MAGIC 2U
#define FUNCTIONFS_HAS_FS_DESC 1U
#define FUNCTIONFS_HAS_HS_DESC 2U
#define FUNCTIONFS_HAS_SS_DESC 4U

#define FUNCTIONFS_BIND 0
#define FUNCTIONFS_UNBIND 1
#define FUNCTIONFS_ENABLE 2
#define FUNCTIONFS_DISABLE 3
#define FUNCTIONFS_SETUP 4
#define FUNCTIONFS_SUSPEND 5
#define FUNCTIONFS_RESUME 6

struct options {
	const char *configfs_root;
	const char *gadget_name;
	const char *ffs_instance;
	const char *mount_path;
	const char *config_label;
	const char *manufacturer;
	const char *product;
	const char *serial;
	const char *udc_name;
	uint16_t vendor_id;
	uint16_t product_id;
	uint8_t max_power_units;
	bool cleanup_existing;
	bool bind_gadget;
	bool stay_alive;
	bool verbose;
};

struct __attribute__((packed)) ffs_v2_header {
	uint32_t magic;
	uint32_t length;
	uint32_t flags;
	uint32_t fs_count;
	uint32_t hs_count;
	uint32_t ss_count;
};

struct __attribute__((packed)) ffs_strings_header {
	uint32_t magic;
	uint32_t length;
	uint32_t str_count;
	uint32_t lang_count;
	uint16_t lang;
	char string_0[sizeof("DisplayLink Debug Interface")];
};

struct __attribute__((packed)) ffs_event {
	struct usb_ctrlrequest setup;
	uint8_t type;
	uint8_t reserved[3];
};

struct __attribute__((packed)) displaylink_descriptors {
	struct ffs_v2_header header;
	struct usb_interface_descriptor fs_interface;
	struct usb_endpoint_descriptor fs_out;
	struct usb_endpoint_descriptor fs_in;
	struct usb_interface_descriptor hs_interface;
	struct usb_endpoint_descriptor hs_out;
	struct usb_endpoint_descriptor hs_in;
	struct usb_interface_descriptor ss_interface;
	struct usb_endpoint_descriptor ss_out;
	struct usb_ss_ep_comp_descriptor ss_out_comp;
	struct usb_endpoint_descriptor ss_in;
	struct usb_ss_ep_comp_descriptor ss_in_comp;
};

static volatile sig_atomic_t stop_requested = 0;

static void usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s [options]\n"
		"  --cleanup-existing      Remove an existing gadget of the same name first\n"
		"  --no-bind               Create and prepare the gadget, but do not bind it\n"
		"  --no-stay-alive         Exit after registering descriptors instead of pumping ep0/ep1\n"
		"  --gadget-name NAME      Configfs gadget name (default: g1)\n"
		"  --ffs-instance NAME     FunctionFS instance name (default: displaylink)\n"
		"  --mount-path PATH       FunctionFS mount path (default: /dev/ffs-displaylink)\n"
		"  --config-label LABEL    USB config string (default: DisplayLink)\n"
		"  --manufacturer TEXT     Manufacturer string\n"
		"  --product TEXT          Product string\n"
		"  --serial TEXT           Serial string\n"
		"  --udc NAME              Bind to a specific UDC instead of the default\n"
		"  --vendor-id HEX         USB vendor ID (default: 0x17e9)\n"
		"  --product-id HEX        USB product ID (default: 0x0104)\n"
		"  --verbose               Log control and bulk traffic\n",
		argv0);
}

static void on_signal(int signo)
{
	(void)signo;
	stop_requested = 1;
}

static void install_signal_handlers(void)
{
	struct sigaction action;
	memset(&action, 0, sizeof(action));
	action.sa_handler = on_signal;
	sigaction(SIGINT, &action, NULL);
	sigaction(SIGTERM, &action, NULL);
}

static int parse_hex16(const char *text, uint16_t *value)
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

static int mkdir_p(const char *path)
{
	char *copy;
	char *cursor;
	int ret = 0;

	copy = strdup(path);
	if (!copy)
		return -1;

	for (cursor = copy + 1; *cursor; cursor++) {
		if (*cursor != '/')
			continue;
		*cursor = '\0';
		if (mkdir(copy, 0755) != 0 && errno != EEXIST) {
			ret = -1;
			goto out;
		}
		*cursor = '/';
	}

	if (mkdir(copy, 0755) != 0 && errno != EEXIST)
		ret = -1;

out:
	free(copy);
	return ret;
}

static void print_usbg_error(const char *context, int usbg_ret)
{
	fprintf(stderr, "%s: %s: %s\n", context, usbg_error_name(usbg_ret), usbg_strerror(usbg_ret));
}

static int remove_existing_gadget(usbg_state *state, const char *gadget_name)
{
	usbg_gadget *gadget;
	usbg_udc *udc;
	int usbg_ret;

	gadget = usbg_get_gadget(state, gadget_name);
	if (!gadget)
		return USBG_SUCCESS;

	udc = usbg_get_gadget_udc(gadget);
	if (udc) {
		usbg_ret = usbg_disable_gadget(gadget);
		if (usbg_ret != USBG_SUCCESS)
			return usbg_ret;
	}

	return usbg_rm_gadget(gadget, USBG_RM_RECURSE);
}

static int mount_functionfs(const struct options *opts)
{
	if (mkdir_p(opts->mount_path) != 0) {
		perror("mkdir_p mount path");
		return -1;
	}

	if (umount2(opts->mount_path, 0) != 0 && errno != EINVAL && errno != ENOENT) {
		perror("umount functionfs");
		return -1;
	}

	if (mount(opts->ffs_instance, opts->mount_path, "functionfs", 0, NULL) != 0) {
		perror("mount functionfs");
		return -1;
	}

	return 0;
}

static struct displaylink_descriptors build_descriptors(void)
{
	struct displaylink_descriptors descriptors;
	memset(&descriptors, 0, sizeof(descriptors));

	descriptors.header.magic = FUNCTIONFS_DESCRIPTORS_MAGIC_V2;
	descriptors.header.length = (uint32_t)sizeof(descriptors);
	descriptors.header.flags = FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC | FUNCTIONFS_HAS_SS_DESC;
	descriptors.header.fs_count = 3;
	descriptors.header.hs_count = 3;
	descriptors.header.ss_count = 5;

	descriptors.fs_interface.bLength = sizeof(descriptors.fs_interface);
	descriptors.fs_interface.bDescriptorType = USB_DT_INTERFACE;
	descriptors.fs_interface.bNumEndpoints = 2;
	descriptors.fs_interface.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
	descriptors.fs_interface.iInterface = 1;

	descriptors.fs_out.bLength = sizeof(descriptors.fs_out);
	descriptors.fs_out.bDescriptorType = USB_DT_ENDPOINT;
	descriptors.fs_out.bEndpointAddress = 0x01;
	descriptors.fs_out.bmAttributes = USB_ENDPOINT_XFER_BULK;
	descriptors.fs_out.wMaxPacketSize = 64;

	descriptors.fs_in.bLength = sizeof(descriptors.fs_in);
	descriptors.fs_in.bDescriptorType = USB_DT_ENDPOINT;
	descriptors.fs_in.bEndpointAddress = USB_DIR_IN | 0x02;
	descriptors.fs_in.bmAttributes = USB_ENDPOINT_XFER_BULK;
	descriptors.fs_in.wMaxPacketSize = 64;

	descriptors.hs_interface = descriptors.fs_interface;
	descriptors.hs_out = descriptors.fs_out;
	descriptors.hs_out.wMaxPacketSize = 512;
	descriptors.hs_in = descriptors.fs_in;
	descriptors.hs_in.wMaxPacketSize = 512;

	descriptors.ss_interface = descriptors.fs_interface;
	descriptors.ss_out = descriptors.fs_out;
	descriptors.ss_out.wMaxPacketSize = 1024;
	descriptors.ss_out_comp.bLength = sizeof(descriptors.ss_out_comp);
	descriptors.ss_out_comp.bDescriptorType = USB_DT_SS_ENDPOINT_COMP;
	descriptors.ss_in = descriptors.fs_in;
	descriptors.ss_in.wMaxPacketSize = 1024;
	descriptors.ss_in_comp.bLength = sizeof(descriptors.ss_in_comp);
	descriptors.ss_in_comp.bDescriptorType = USB_DT_SS_ENDPOINT_COMP;

	return descriptors;
}

static struct ffs_strings_header build_strings(void)
{
	struct ffs_strings_header strings;
	memset(&strings, 0, sizeof(strings));

	strings.magic = FUNCTIONFS_STRINGS_MAGIC;
	strings.length = (uint32_t)sizeof(strings);
	strings.str_count = 1;
	strings.lang_count = 1;
	strings.lang = DISPLAYLINK_LANG;
	memcpy(strings.string_0, "DisplayLink Debug Interface", sizeof(strings.string_0));

	return strings;
}

static int wait_for_endpoint(const char *mount_path, const char *name, int flags)
{
	char path[512];
	int fd = -1;
	int attempts;
	const struct timespec delay = {
		.tv_sec = 0,
		.tv_nsec = 50 * 1000 * 1000,
	};

	snprintf(path, sizeof(path), "%s/%s", mount_path, name);
	for (attempts = 0; attempts < 100; attempts++) {
		fd = open(path, flags, 0);
		if (fd >= 0)
			return fd;
		if (errno != ENOENT)
			break;
		nanosleep(&delay, NULL);
	}

	perror(path);
	return -1;
}

static int write_ffs_descriptors(int ep0_fd)
{
	struct displaylink_descriptors descriptors = build_descriptors();
	struct ffs_strings_header strings = build_strings();

	if (write(ep0_fd, &descriptors, sizeof(descriptors)) != (ssize_t)sizeof(descriptors)) {
		perror("write descriptors");
		return -1;
	}

	if (write(ep0_fd, &strings, sizeof(strings)) != (ssize_t)sizeof(strings)) {
		perror("write strings");
		return -1;
	}

	return 0;
}

static const char *event_name(uint8_t type)
{
	switch (type) {
	case FUNCTIONFS_BIND:
		return "bind";
	case FUNCTIONFS_UNBIND:
		return "unbind";
	case FUNCTIONFS_ENABLE:
		return "enable";
	case FUNCTIONFS_DISABLE:
		return "disable";
	case FUNCTIONFS_SETUP:
		return "setup";
	case FUNCTIONFS_SUSPEND:
		return "suspend";
	case FUNCTIONFS_RESUME:
		return "resume";
	default:
		return "unknown";
	}
}

static int handle_setup_request(int ep0_fd, const struct usb_ctrlrequest *setup, bool verbose)
{
	uint8_t buffer[4096];
	uint16_t length = setup->wLength;

	if (verbose) {
		fprintf(stderr,
			"setup bmRequestType=0x%02x bRequest=0x%02x wValue=0x%04x wIndex=0x%04x wLength=%u\n",
			setup->bRequestType,
			setup->bRequest,
			setup->wValue,
			setup->wIndex,
			length);
	}

	if (setup->bRequestType & USB_DIR_IN) {
		if (length > sizeof(buffer))
			length = sizeof(buffer);
		memset(buffer, 0, length);
		if (write(ep0_fd, buffer, length) != (ssize_t)length) {
			perror("write ep0 IN response");
			return -1;
		}
		return 0;
	}

	if (length == 0)
		return 0;

	if (length > sizeof(buffer))
		length = sizeof(buffer);
	if (read(ep0_fd, buffer, length) < 0) {
		perror("read ep0 OUT payload");
		return -1;
	}

	return 0;
}

static int run_displaylink_loop(int ep0_fd, int ep_out_fd, bool verbose)
{
	struct pollfd pollfds[2];
	uint8_t bulk_buffer[65536];

	pollfds[0].fd = ep0_fd;
	pollfds[0].events = POLLIN;
	pollfds[1].fd = ep_out_fd;
	pollfds[1].events = POLLIN;

	while (!stop_requested) {
		int poll_ret = poll(pollfds, 2, 250);
		if (poll_ret < 0) {
			if (errno == EINTR)
				continue;
			perror("poll");
			return -1;
		}

		if (poll_ret == 0)
			continue;

		if (pollfds[0].revents & POLLIN) {
			struct ffs_event event;
			ssize_t read_len = read(ep0_fd, &event, sizeof(event));
			if (read_len < 0) {
				if (errno != EAGAIN)
					perror("read ep0 event");
			} else if (read_len == (ssize_t)sizeof(event)) {
				fprintf(stderr, "FunctionFS event: %s\n", event_name(event.type));
				if (event.type == FUNCTIONFS_SETUP && handle_setup_request(ep0_fd, &event.setup, verbose) != 0)
					return -1;
			}
		}

		if (pollfds[1].revents & POLLIN) {
			ssize_t read_len = read(ep_out_fd, bulk_buffer, sizeof(bulk_buffer));
			if (read_len < 0) {
				if (errno != EAGAIN)
					perror("read ep1 bulk");
			} else if (read_len > 0) {
				fprintf(stderr, "bulk OUT %zd bytes", read_len);
				if (verbose) {
					size_t prefix = (size_t)read_len < 16 ? (size_t)read_len : 16U;
					size_t i;
					fprintf(stderr, " [");
					for (i = 0; i < prefix; i++)
						fprintf(stderr, "%s%02x", i ? " " : "", bulk_buffer[i]);
					fprintf(stderr, "]");
				}
				fprintf(stderr, "\n");
			}
		}
	}

	return 0;
}

static void default_options(struct options *opts)
{
	memset(opts, 0, sizeof(*opts));
	opts->configfs_root = "/sys/kernel/config";
	opts->gadget_name = "g1";
	opts->ffs_instance = "displaylink";
	opts->mount_path = "/dev/ffs-displaylink";
	opts->config_label = "DisplayLink";
	opts->manufacturer = "DisplayLink";
	opts->product = "DisplayLink Adapter";
	opts->serial = "DEADBEEF0001";
	opts->vendor_id = DISPLAYLINK_VENDOR_ID;
	opts->product_id = DISPLAYLINK_PRODUCT_ID;
	opts->max_power_units = 125;
	opts->cleanup_existing = true;
	opts->bind_gadget = true;
	opts->stay_alive = true;
	opts->verbose = false;
}

static int parse_args(int argc, char **argv, struct options *opts)
{
	static const struct option long_options[] = {
		{ "cleanup-existing", no_argument, NULL, 'c' },
		{ "no-bind", no_argument, NULL, 'n' },
		{ "no-stay-alive", no_argument, NULL, 'x' },
		{ "gadget-name", required_argument, NULL, 'g' },
		{ "ffs-instance", required_argument, NULL, 'f' },
		{ "mount-path", required_argument, NULL, 'm' },
		{ "config-label", required_argument, NULL, 'l' },
		{ "manufacturer", required_argument, NULL, 'M' },
		{ "product", required_argument, NULL, 'P' },
		{ "serial", required_argument, NULL, 'S' },
		{ "udc", required_argument, NULL, 'u' },
		{ "vendor-id", required_argument, NULL, 'v' },
		{ "product-id", required_argument, NULL, 'p' },
		{ "verbose", no_argument, NULL, 'V' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int opt;

	while ((opt = getopt_long(argc, argv, "cnxg:f:m:l:M:P:S:u:v:p:Vh", long_options, NULL)) != -1) {
		switch (opt) {
		case 'c':
			opts->cleanup_existing = true;
			break;
		case 'n':
			opts->bind_gadget = false;
			break;
		case 'x':
			opts->stay_alive = false;
			break;
		case 'g':
			opts->gadget_name = optarg;
			break;
		case 'f':
			opts->ffs_instance = optarg;
			break;
		case 'm':
			opts->mount_path = optarg;
			break;
		case 'l':
			opts->config_label = optarg;
			break;
		case 'M':
			opts->manufacturer = optarg;
			break;
		case 'P':
			opts->product = optarg;
			break;
		case 'S':
			opts->serial = optarg;
			break;
		case 'u':
			opts->udc_name = optarg;
			break;
		case 'v':
			if (parse_hex16(optarg, &opts->vendor_id) != 0)
				return -1;
			break;
		case 'p':
			if (parse_hex16(optarg, &opts->product_id) != 0)
				return -1;
			break;
		case 'V':
			opts->verbose = true;
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

int main(int argc, char **argv)
{
	struct options opts;
	struct usbg_gadget_attrs gadget_attrs;
	struct usbg_gadget_strs gadget_strs;
	struct usbg_config_attrs config_attrs;
	struct usbg_config_strs config_strs;
	usbg_state *state = NULL;
	usbg_gadget *gadget = NULL;
	usbg_config *config = NULL;
	usbg_function *ffs = NULL;
	usbg_udc *udc = NULL;
	int ep0_fd = -1;
	int ep_out_fd = -1;
	int ep_in_fd = -1;
	int usbg_ret;
	int ret = EXIT_FAILURE;

	default_options(&opts);
	if (parse_args(argc, argv, &opts) != 0) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	install_signal_handlers();

	memset(&gadget_attrs, 0, sizeof(gadget_attrs));
	gadget_attrs.bcdUSB = 0x0320;
	gadget_attrs.bDeviceClass = USB_CLASS_PER_INTERFACE;
	gadget_attrs.bDeviceSubClass = 0x00;
	gadget_attrs.bDeviceProtocol = 0x00;
	gadget_attrs.bMaxPacketSize0 = 64;
	gadget_attrs.idVendor = opts.vendor_id;
	gadget_attrs.idProduct = opts.product_id;
	gadget_attrs.bcdDevice = 0x0001;

	gadget_strs.serial = (char *)opts.serial;
	gadget_strs.manufacturer = (char *)opts.manufacturer;
	gadget_strs.product = (char *)opts.product;

	config_attrs.bmAttributes = USB_CONFIG_ATT_ONE;
	config_attrs.bMaxPower = opts.max_power_units;
	config_strs.configuration = (char *)opts.config_label;

	usbg_ret = usbg_init(opts.configfs_root, &state);
	if (usbg_ret != USBG_SUCCESS) {
		print_usbg_error("usbg_init", usbg_ret);
		goto out;
	}

	if (opts.cleanup_existing) {
		usbg_ret = remove_existing_gadget(state, opts.gadget_name);
		if (usbg_ret != USBG_SUCCESS) {
			print_usbg_error("remove_existing_gadget", usbg_ret);
			goto out;
		}
	}

	usbg_ret = usbg_create_gadget(state, opts.gadget_name, &gadget_attrs, &gadget_strs, &gadget);
	if (usbg_ret != USBG_SUCCESS) {
		print_usbg_error("usbg_create_gadget", usbg_ret);
		goto out;
	}

	usbg_ret = usbg_create_function(gadget, USBG_F_FFS, opts.ffs_instance, NULL, &ffs);
	if (usbg_ret != USBG_SUCCESS) {
		print_usbg_error("usbg_create_function", usbg_ret);
		goto out;
	}

	usbg_ret = usbg_create_config(gadget, 1, opts.config_label, &config_attrs, &config_strs, &config);
	if (usbg_ret != USBG_SUCCESS) {
		print_usbg_error("usbg_create_config", usbg_ret);
		goto out;
	}

	usbg_ret = usbg_add_config_function(config, opts.ffs_instance, ffs);
	if (usbg_ret != USBG_SUCCESS) {
		print_usbg_error("usbg_add_config_function", usbg_ret);
		goto out;
	}

	if (mount_functionfs(&opts) != 0)
		goto out;

	{
		char ep0_path[512];
		snprintf(ep0_path, sizeof(ep0_path), "%s/ep0", opts.mount_path);
		ep0_fd = open(ep0_path, O_RDWR);
		if (ep0_fd < 0) {
			perror(ep0_path);
			goto out;
		}
	}

	if (write_ffs_descriptors(ep0_fd) != 0)
		goto out;

	ep_out_fd = wait_for_endpoint(opts.mount_path, "ep1", O_RDONLY | O_NONBLOCK);
	if (ep_out_fd < 0)
		goto out;
	ep_in_fd = wait_for_endpoint(opts.mount_path, "ep2", O_RDWR | O_NONBLOCK);
	if (ep_in_fd < 0)
		goto out;

	if (opts.bind_gadget) {
		if (opts.udc_name) {
			udc = usbg_get_udc(state, opts.udc_name);
			if (!udc) {
				fprintf(stderr, "Unable to find UDC %s\n", opts.udc_name);
				goto out;
			}
		}

		usbg_ret = usbg_enable_gadget(gadget, udc);
		if (usbg_ret != USBG_SUCCESS) {
			print_usbg_error("usbg_enable_gadget", usbg_ret);
			goto out;
		}
	}

	fprintf(stdout,
		"DisplayLink FunctionFS gadget prepared. gadget=%s ffs=%s mount=%s bind=%s\n",
		opts.gadget_name,
		opts.ffs_instance,
		opts.mount_path,
		opts.bind_gadget ? "yes" : "no");

	if (opts.stay_alive && opts.bind_gadget) {
		fprintf(stdout, "Entering FunctionFS event loop. Press Ctrl+C to stop.\n");
		if (run_displaylink_loop(ep0_fd, ep_out_fd, opts.verbose) != 0)
			goto out;
	}

	ret = EXIT_SUCCESS;

out:
	if (ep_in_fd >= 0)
		close(ep_in_fd);
	if (ep_out_fd >= 0)
		close(ep_out_fd);
	if (ep0_fd >= 0)
		close(ep0_fd);
	if (state)
		usbg_cleanup(state);
	return ret;
}