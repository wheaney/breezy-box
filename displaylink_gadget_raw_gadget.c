#define _POSIX_C_SOURCE 200809L

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/usb/ch9.h>
#include <linux/usb/raw_gadget.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define DISPLAYLINK_VENDOR_ID 0x17e9
#define DISPLAYLINK_PRODUCT_ID 0x0104

#define USB_LANG_EN_US 0x0409u

#define DEFAULT_DISPLAY_WIDTH 1920u
#define DEFAULT_DISPLAY_HEIGHT 1080u
#define DEFAULT_BULK_OUT_ADDRESS 0x01u
#define DEFAULT_RAW_DEVICE_PATH "/dev/raw-gadget"

#define UDL_VENDOR_DESCRIPTOR_TYPE 0x5fU
#define UDL_VENDOR_DESCRIPTOR_VERSION 0x0001U
#define UDL_VENDOR_KEY_MAX_PIXELS 0x0200U
#define UDL_VENDOR_REQUEST_CHANNEL 0x12U
#define UDL_VENDOR_REQUEST_EDID 0x02U
#define UDL_EDID_INDEX 0x00a1U

#define EP0_BUFFER_SIZE 512u
#define BULK_OUT_BUFFER_SIZE 16384u

struct options {
	const char *raw_device_path;
	const char *edid_path;
	const char *udc_driver;
	const char *udc_device;
	uint16_t vendor_id;
	uint16_t product_id;
	bool verbose;
};

struct __attribute__((packed)) gadget_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};

struct __attribute__((packed)) gadget_config_block {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct gadget_endpoint_descriptor bulk_out;
};

struct usb_raw_control_event {
	struct usb_raw_event inner;
	struct usb_ctrlrequest ctrl;
};

struct usb_raw_control_io {
	struct usb_raw_ep_io inner;
	uint8_t data[EP0_BUFFER_SIZE];
};

struct usb_raw_bulk_io {
	struct usb_raw_ep_io inner;
	uint8_t data[BULK_OUT_BUFFER_SIZE];
};

struct raw_runtime {
	int fd;
	uint8_t current_configuration;
	int bulk_out_handle;
	pthread_t bulk_out_thread;
	uint8_t bulk_out_address;
	bool bulk_out_address_valid;
	bool bulk_out_thread_created;
	bool verbose;
	atomic_bool bulk_out_thread_stop;
	struct usb_device_descriptor device_descriptor;
	struct usb_qualifier_descriptor qualifier_descriptor;
	uint8_t edid[128];
	uint8_t vendor_descriptor[16];
	size_t vendor_descriptor_len;
};

enum control_action {
	CONTROL_ACTION_STALL,
	CONTROL_ACTION_WRITE,
	CONTROL_ACTION_READ,
};

static const char *const k_string_manufacturer = "DisplayLink";
static const char *const k_string_product = "DisplayLink Adapter";
static const char *const k_string_serial = "DEADBEEF0001";

static const uint8_t k_std_channel[16] = {
	0x57, 0xcd, 0xdc, 0xa7, 0x1c, 0x88, 0x5e, 0x15,
	0x60, 0xfe, 0xc6, 0x97, 0x16, 0x3d, 0x47, 0xf2,
};

static volatile sig_atomic_t stop_requested = 0;

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define host_to_le16(value) ((uint16_t)(value))
#else
#define host_to_le16(value) __builtin_bswap16((uint16_t)(value))
#endif

static void usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s [options]\n"
		"  --raw-device PATH       Raw Gadget device node (default: /dev/raw-gadget)\n"
		"  --edid-file PATH        Override the built-in 128-byte EDID with a binary blob\n"
		"  --udc-driver NAME       UDC driver name (default: auto-detect)\n"
		"  --udc-device NAME       UDC device name (default: auto-detect)\n"
		"  --vendor-id HEX         USB vendor ID (default: 0x17e9)\n"
		"  --product-id HEX        USB product ID (default: 0x0104)\n"
		"  --verbose               Log control requests and endpoint discovery\n",
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
	sigemptyset(&action.sa_mask);
	sigaction(SIGINT, &action, NULL);
	sigaction(SIGTERM, &action, NULL);
	sigaction(SIGHUP, &action, NULL);
	sigaction(SIGQUIT, &action, NULL);
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

static void default_options(struct options *opts)
{
	memset(opts, 0, sizeof(*opts));
	opts->raw_device_path = DEFAULT_RAW_DEVICE_PATH;
	opts->vendor_id = DISPLAYLINK_VENDOR_ID;
	opts->product_id = DISPLAYLINK_PRODUCT_ID;
	opts->verbose = false;
}

static int parse_args(int argc, char **argv, struct options *opts)
{
	static const struct option long_options[] = {
		{ "raw-device", required_argument, NULL, 'r' },
		{ "edid-file", required_argument, NULL, 'e' },
		{ "udc-driver", required_argument, NULL, 'u' },
		{ "udc-device", required_argument, NULL, 'd' },
		{ "vendor-id", required_argument, NULL, 'v' },
		{ "product-id", required_argument, NULL, 'p' },
		{ "verbose", no_argument, NULL, 'V' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int opt;

	while ((opt = getopt_long(argc, argv, "r:e:u:d:v:p:Vh", long_options, NULL)) != -1) {
		switch (opt) {
		case 'r':
			opts->raw_device_path = optarg;
			break;
		case 'e':
			opts->edid_path = optarg;
			break;
		case 'u':
			opts->udc_driver = optarg;
			break;
		case 'd':
			opts->udc_device = optarg;
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

static int load_edid_file(const char *path, uint8_t edid[128])
{
	int fd;
	size_t total = 0u;
	bool has_extra_bytes = false;

	fd = open(path, O_RDONLY);
	if (fd < 0) {
		perror(path);
		return -1;
	}

	while (total < 128u) {
		ssize_t read_len = read(fd, edid + total, 128u - total);

		if (read_len < 0) {
			if (errno == EINTR)
				continue;
			perror(path);
			close(fd);
			return -1;
		}
		if (read_len == 0) {
			fprintf(stderr,
				"EDID file '%s' ended after %zu bytes; expected exactly 128 bytes.\n",
				path,
				total);
			close(fd);
			return -1;
		}

		total += (size_t)read_len;
	}

	{
		uint8_t extra_byte;
		ssize_t extra_len;

		extra_len = read(fd, &extra_byte, sizeof(extra_byte));
		if (extra_len < 0 && errno == EINTR)
			extra_len = read(fd, &extra_byte, sizeof(extra_byte));
		if (extra_len < 0) {
			perror(path);
			close(fd);
			return -1;
		}
		has_extra_bytes = extra_len > 0;
	}

	close(fd);

	if (has_extra_bytes || edid[126] != 0u) {
		fprintf(stderr,
			"EDID file '%s' contains extension data. Raw Gadget currently serves only the first 128-byte base block, so extension blocks will be ignored.\n",
			path);
		edid[126] = 0u;
		{
			uint32_t sum = 0u;
			size_t index;

			for (index = 0u; index < 127u; ++index)
				sum += edid[index];
			edid[127] = (uint8_t)((256u - (sum & 0xffu)) & 0xffu);
		}
	}

	return 0;
}

static const char *event_name(uint32_t type)
{
	switch ((enum usb_raw_event_type)type) {
	case USB_RAW_EVENT_CONNECT:
		return "connect";
	case USB_RAW_EVENT_CONTROL:
		return "control";
	case USB_RAW_EVENT_SUSPEND:
		return "suspend";
	case USB_RAW_EVENT_RESUME:
		return "resume";
	case USB_RAW_EVENT_RESET:
		return "reset";
	case USB_RAW_EVENT_DISCONNECT:
		return "disconnect";
	default:
		return "unknown";
	}
}

static const char *path_basename(const char *path)
{
	const char *slash = strrchr(path, '/');

	return slash ? slash + 1 : path;
}

static int read_single_line_file(const char *path,
				 char *buffer,
				 size_t capacity)
{
	FILE *stream;

	if (capacity == 0u)
		return -1;

	stream = fopen(path, "r");
	if (!stream)
		return -1;
	if (!fgets(buffer, (int)capacity, stream)) {
		fclose(stream);
		return -1;
	}
	fclose(stream);

	buffer[strcspn(buffer, "\r\n")] = '\0';
	return 0;
}

static int read_uevent_value(const char *path,
			     const char *key,
			     char *buffer,
			     size_t capacity)
{
	FILE *stream;
	char line[512];

	if (capacity == 0u)
		return -1;

	stream = fopen(path, "r");
	if (!stream)
		return -1;

	while (fgets(line, sizeof(line), stream)) {
		char *equals;

		line[strcspn(line, "\r\n")] = '\0';
		equals = strchr(line, '=');
		if (!equals)
			continue;
		*equals++ = '\0';
		if (strcmp(line, key) != 0)
			continue;
		if (snprintf(buffer, capacity, "%s", equals) >= (int)capacity) {
			fclose(stream);
			return -1;
		}
		fclose(stream);
		return 0;
	}

	fclose(stream);
	return -1;
}

static int auto_detect_udc(char *driver_buffer,
			       size_t driver_capacity,
			       char *device_buffer,
			       size_t device_capacity)
{
	DIR *directory;
	struct dirent *entry;
	int found = 0;
	char uevent_path[512];
	char symlink_path[512];
	char driver_target[512];
	ssize_t link_len;

	directory = opendir("/sys/class/udc");
	if (!directory) {
		perror("opendir /sys/class/udc");
		return -1;
	}

	while ((entry = readdir(directory)) != NULL) {
		if (entry->d_name[0] == '.')
			continue;
		if (found++) {
			fprintf(stderr,
				"Multiple UDC instances found under /sys/class/udc; pass --udc-device and --udc-driver explicitly\n");
			closedir(directory);
			return -1;
		}
		if (snprintf(device_buffer, device_capacity, "%s", entry->d_name) >= (int)device_capacity) {
			closedir(directory);
			return -1;
		}
	}

	closedir(directory);
	if (found != 1) {
		fprintf(stderr, "Unable to auto-detect a UDC under /sys/class/udc\n");
		return -1;
	}

	if (snprintf(uevent_path,
		     sizeof(uevent_path),
		     "/sys/class/udc/%s/uevent",
		     device_buffer) >= (int)sizeof(uevent_path))
		return -1;

	if (read_uevent_value(uevent_path,
			      "USB_UDC_NAME",
			      driver_buffer,
			      driver_capacity) == 0)
		return 0;

	if (snprintf(symlink_path,
		     sizeof(symlink_path),
		     "/sys/class/udc/%s/device/driver",
		     device_buffer) >= (int)sizeof(symlink_path))
		return -1;

	link_len = readlink(symlink_path, driver_target, sizeof(driver_target) - 1u);
	if (link_len < 0) {
		if (snprintf(driver_buffer, driver_capacity, "%s", device_buffer) >= (int)driver_capacity)
			return -1;
		return 0;
	}

	driver_target[link_len] = '\0';
	if (snprintf(driver_buffer,
		     driver_capacity,
		     "%s",
		     path_basename(driver_target)) >= (int)driver_capacity)
		return -1;

	return 0;
}

static uint16_t encode_manufacturer_id(char first, char second, char third)
{
	return (uint16_t)((((uint16_t)(first - '@')) & 0x1fu) << 10 |
			 (((uint16_t)(second - '@')) & 0x1fu) << 5 |
			 (((uint16_t)(third - '@')) & 0x1fu));
}

static void write_ascii_monitor_name(uint8_t *dst, const char *name)
{
	size_t length = strlen(name);

	memset(dst, ' ', 13u);
	if (length > 13u)
		length = 13u;
	memcpy(dst, name, length);
	if (length < 13u)
		dst[length] = '\n';
}

static void build_default_edid(uint8_t edid[128])
{
	uint32_t sum = 0u;
	size_t index;

	memset(edid, 0, 128u);
	edid[0] = 0x00;
	edid[1] = 0xff;
	edid[2] = 0xff;
	edid[3] = 0xff;
	edid[4] = 0xff;
	edid[5] = 0xff;
	edid[6] = 0xff;
	edid[7] = 0x00;

	{
		const uint16_t manufacturer = encode_manufacturer_id('B', 'B', 'X');
		edid[8] = (uint8_t)(manufacturer >> 8);
		edid[9] = (uint8_t)manufacturer;
	}

	edid[10] = 0x01;
	edid[11] = 0x00;
	edid[12] = 0x01;
	edid[13] = 0x00;
	edid[14] = 0x00;
	edid[15] = 0x00;
	edid[16] = 1u;
	edid[17] = 34u;
	edid[18] = 0x01;
	edid[19] = 0x03;
	edid[20] = 0x81;
	edid[21] = 0xa0;
	edid[22] = 0x5a;
	edid[23] = 0x78;
	edid[24] = 0x0f;
	{
		static const uint8_t chromaticity[10] = {
			0xee, 0x91, 0xa3, 0x54, 0x4c,
			0x99, 0x26, 0x0f, 0x50, 0x54,
		};
		memcpy(&edid[25], chromaticity, sizeof(chromaticity));
	}
	for (index = 35u; index <= 37u; ++index)
		edid[index] = 0x00;
	for (index = 38u; index < 54u; ++index)
		edid[index] = 0x01;

	{
		static const uint8_t dtd_1080p60[18] = {
			0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40,
			0x58, 0x2c, 0x45, 0x00, 0x40, 0x84, 0x63, 0x00,
			0x00, 0x1e,
		};
		memcpy(&edid[54], dtd_1080p60, sizeof(dtd_1080p60));
	}

	edid[72] = 0x00;
	edid[73] = 0x00;
	edid[74] = 0x00;
	edid[75] = 0xfc;
	edid[76] = 0x00;
	write_ascii_monitor_name(&edid[77], "Breezy Box");

	edid[90] = 0x00;
	edid[91] = 0x00;
	edid[92] = 0x00;
	edid[93] = 0xfd;
	edid[94] = 0x00;
	edid[95] = 0x32;
	edid[96] = 0x46;
	edid[97] = 0x1e;
	edid[98] = 0x46;
	edid[99] = 0x0f;
	edid[100] = 0x00;
	edid[101] = 0x0a;
	edid[102] = 0x20;
	edid[103] = 0x20;
	edid[104] = 0x20;
	edid[105] = 0x20;
	edid[106] = 0x20;
	edid[107] = 0x20;

	edid[108] = 0x00;
	edid[109] = 0x00;
	edid[110] = 0x00;
	edid[111] = 0x10;

	for (index = 0u; index < 127u; ++index)
		sum += edid[index];
	edid[127] = (uint8_t)((256u - (sum & 0xffu)) & 0xffu);
}

static void build_vendor_descriptor(struct raw_runtime *runtime)
{
	const uint32_t pixel_limit = DEFAULT_DISPLAY_WIDTH * DEFAULT_DISPLAY_HEIGHT;

	runtime->vendor_descriptor[0] = 12u;
	runtime->vendor_descriptor[1] = UDL_VENDOR_DESCRIPTOR_TYPE;
	runtime->vendor_descriptor[2] = (uint8_t)(UDL_VENDOR_DESCRIPTOR_VERSION & 0xffu);
	runtime->vendor_descriptor[3] = (uint8_t)(UDL_VENDOR_DESCRIPTOR_VERSION >> 8);
	runtime->vendor_descriptor[4] = 10u;
	runtime->vendor_descriptor[5] = (uint8_t)(UDL_VENDOR_KEY_MAX_PIXELS & 0xffu);
	runtime->vendor_descriptor[6] = (uint8_t)(UDL_VENDOR_KEY_MAX_PIXELS >> 8);
	runtime->vendor_descriptor[7] = 4u;
	runtime->vendor_descriptor[8] = (uint8_t)(pixel_limit & 0xffu);
	runtime->vendor_descriptor[9] = (uint8_t)((pixel_limit >> 8) & 0xffu);
	runtime->vendor_descriptor[10] = (uint8_t)((pixel_limit >> 16) & 0xffu);
	runtime->vendor_descriptor[11] = (uint8_t)((pixel_limit >> 24) & 0xffu);
	runtime->vendor_descriptor_len = 12u;
}

static void build_device_descriptor(struct raw_runtime *runtime,
				    const struct options *opts)
{
	memset(&runtime->device_descriptor, 0, sizeof(runtime->device_descriptor));
	runtime->device_descriptor.bLength = USB_DT_DEVICE_SIZE;
	runtime->device_descriptor.bDescriptorType = USB_DT_DEVICE;
	runtime->device_descriptor.bcdUSB = host_to_le16(0x0200u);
	runtime->device_descriptor.bDeviceClass = USB_CLASS_PER_INTERFACE;
	runtime->device_descriptor.bDeviceSubClass = 0u;
	runtime->device_descriptor.bDeviceProtocol = 0u;
	runtime->device_descriptor.bMaxPacketSize0 = 64u;
	runtime->device_descriptor.idVendor = host_to_le16(opts->vendor_id);
	runtime->device_descriptor.idProduct = host_to_le16(opts->product_id);
	runtime->device_descriptor.bcdDevice = host_to_le16(0x0001u);
	runtime->device_descriptor.iManufacturer = 1u;
	runtime->device_descriptor.iProduct = 2u;
	runtime->device_descriptor.iSerialNumber = 3u;
	runtime->device_descriptor.bNumConfigurations = 1u;
}

static void build_device_qualifier(struct raw_runtime *runtime)
{
	memset(&runtime->qualifier_descriptor, 0, sizeof(runtime->qualifier_descriptor));
	runtime->qualifier_descriptor.bLength = sizeof(runtime->qualifier_descriptor);
	runtime->qualifier_descriptor.bDescriptorType = USB_DT_DEVICE_QUALIFIER;
	runtime->qualifier_descriptor.bcdUSB = host_to_le16(0x0200u);
	runtime->qualifier_descriptor.bDeviceClass = USB_CLASS_PER_INTERFACE;
	runtime->qualifier_descriptor.bDeviceSubClass = 0u;
	runtime->qualifier_descriptor.bDeviceProtocol = 0u;
	runtime->qualifier_descriptor.bMaxPacketSize0 = 64u;
	runtime->qualifier_descriptor.bNumConfigurations = 1u;
	runtime->qualifier_descriptor.bRESERVED = 0u;
}

static size_t build_string_descriptor(uint8_t *buffer,
				      size_t capacity,
				      uint8_t index)
{
	const char *text = NULL;
	size_t length;
	size_t i;

	if (index == 0u) {
		if (capacity < 4u)
			return 0u;
		buffer[0] = 4u;
		buffer[1] = USB_DT_STRING;
		buffer[2] = (uint8_t)(USB_LANG_EN_US & 0xffu);
		buffer[3] = (uint8_t)(USB_LANG_EN_US >> 8);
		return 4u;
	}

	switch (index) {
	case 1u:
		text = k_string_manufacturer;
		break;
	case 2u:
		text = k_string_product;
		break;
	case 3u:
		text = k_string_serial;
		break;
	default:
		return 0u;
	}

	length = strlen(text);
	if (2u + (length * 2u) > capacity)
		return 0u;

	buffer[0] = (uint8_t)(2u + (length * 2u));
	buffer[1] = USB_DT_STRING;
	for (i = 0; i < length; ++i) {
		buffer[2u + (i * 2u)] = (uint8_t)text[i];
		buffer[2u + (i * 2u) + 1u] = 0u;
	}

	return (size_t)buffer[0];
}

static size_t build_config_descriptor(uint8_t *buffer,
				      size_t capacity,
				      uint8_t bulk_out_address,
				      bool other_speed)
{
	struct gadget_config_block block;
	const uint16_t total_length = (uint16_t)sizeof(block);

	if (capacity < sizeof(block))
		return 0u;

	memset(&block, 0, sizeof(block));
	block.config.bLength = USB_DT_CONFIG_SIZE;
	block.config.bDescriptorType = other_speed ? USB_DT_OTHER_SPEED_CONFIG : USB_DT_CONFIG;
	block.config.wTotalLength = host_to_le16(total_length);
	block.config.bNumInterfaces = 1u;
	block.config.bConfigurationValue = 1u;
	block.config.iConfiguration = 0u;
	block.config.bmAttributes = USB_CONFIG_ATT_ONE;
	block.config.bMaxPower = 125u;

	block.interface.bLength = USB_DT_INTERFACE_SIZE;
	block.interface.bDescriptorType = USB_DT_INTERFACE;
	block.interface.bInterfaceNumber = 0u;
	block.interface.bAlternateSetting = 0u;
	block.interface.bNumEndpoints = 1u;
	block.interface.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
	block.interface.bInterfaceSubClass = 0u;
	block.interface.bInterfaceProtocol = 0u;
	block.interface.iInterface = 0u;

	block.bulk_out.bLength = USB_DT_ENDPOINT_SIZE;
	block.bulk_out.bDescriptorType = USB_DT_ENDPOINT;
	block.bulk_out.bEndpointAddress = bulk_out_address;
	block.bulk_out.bmAttributes = USB_ENDPOINT_XFER_BULK;
	block.bulk_out.wMaxPacketSize = host_to_le16(other_speed ? 64u : 512u);
	block.bulk_out.bInterval = 0u;

	memcpy(buffer, &block, sizeof(block));
	return sizeof(block);
}

static int raw_init(int fd,
		    enum usb_device_speed speed,
		    const char *driver_name,
		    const char *device_name)
{
	struct usb_raw_init init;

	memset(&init, 0, sizeof(init));
	if (snprintf((char *)init.driver_name,
		     sizeof(init.driver_name),
		     "%s",
		     driver_name) >= (int)sizeof(init.driver_name))
		return -1;
	if (snprintf((char *)init.device_name,
		     sizeof(init.device_name),
		     "%s",
		     device_name) >= (int)sizeof(init.device_name))
		return -1;
	init.speed = (uint8_t)speed;
	return ioctl(fd, USB_RAW_IOCTL_INIT, &init);
}

static int raw_run(int fd)
{
	return ioctl(fd, USB_RAW_IOCTL_RUN, 0);
}

static int raw_event_fetch(int fd, struct usb_raw_control_event *event)
{
	event->inner.type = 0u;
	event->inner.length = sizeof(event->ctrl);
	return ioctl(fd, USB_RAW_IOCTL_EVENT_FETCH, event);
}

static int raw_ep0_read(int fd, struct usb_raw_control_io *io)
{
	return ioctl(fd, USB_RAW_IOCTL_EP0_READ, io);
}

static int raw_ep0_write(int fd, struct usb_raw_control_io *io)
{
	return ioctl(fd, USB_RAW_IOCTL_EP0_WRITE, io);
}

static int raw_ep_read(int fd, struct usb_raw_ep_io *io)
{
	return ioctl(fd, USB_RAW_IOCTL_EP_READ, io);
}

static int raw_ep0_stall(int fd)
{
	return ioctl(fd, USB_RAW_IOCTL_EP0_STALL, 0);
}

static int raw_ep_enable(int fd, struct usb_endpoint_descriptor *desc)
{
	return ioctl(fd, USB_RAW_IOCTL_EP_ENABLE, desc);
}

static int raw_ep_disable(int fd, int handle)
{
	return ioctl(fd, USB_RAW_IOCTL_EP_DISABLE, handle);
}

static int raw_configure(int fd)
{
	return ioctl(fd, USB_RAW_IOCTL_CONFIGURE, 0);
}

static int raw_vbus_draw(int fd, uint32_t power_2ma_units)
{
	return ioctl(fd, USB_RAW_IOCTL_VBUS_DRAW, power_2ma_units);
}

static int raw_eps_info(int fd, struct usb_raw_eps_info *info)
{
	memset(info, 0, sizeof(*info));
	return ioctl(fd, USB_RAW_IOCTL_EPS_INFO, info);
}

static void *bulk_out_drain_thread_main(void *arg)
{
	struct raw_runtime *runtime = arg;
	struct usb_raw_bulk_io io;
	const uint16_t handle = (uint16_t)runtime->bulk_out_handle;
	int rc;

	rc = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	if (rc != 0 && runtime->verbose) {
		errno = rc;
		perror("pthread_setcancelstate bulk OUT drain");
	}
	rc = pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
	if (rc != 0 && runtime->verbose) {
		errno = rc;
		perror("pthread_setcanceltype bulk OUT drain");
	}

	if (runtime->verbose) {
		fprintf(stderr,
			"Raw Gadget bulk OUT drain thread started: handle=%u address=0x%02x\n",
			(unsigned int)handle,
			runtime->bulk_out_address);
	}

	while (!stop_requested && !atomic_load(&runtime->bulk_out_thread_stop)) {
		int rc;

		memset(&io, 0, sizeof(io));
		io.inner.ep = handle;
		io.inner.length = sizeof(io.data);

		rc = raw_ep_read(runtime->fd, &io.inner);
		if (rc < 0) {
			if (errno == EINTR && !stop_requested)
				continue;
			if (!stop_requested && !atomic_load(&runtime->bulk_out_thread_stop) && runtime->verbose)
				perror("ioctl USB_RAW_IOCTL_EP_READ");
			break;
		}

		if (runtime->verbose)
			fprintf(stderr, "bulk OUT transferred %d bytes\n", rc);
	}

	if (runtime->verbose)
		fprintf(stderr, "Raw Gadget bulk OUT drain thread stopped\n");

	return NULL;
}

static void report_raw_run_failure(const char *udc_driver,
				  const char *udc_device,
				  int error_code)
{
	char function_path[512];
	char function_name[256];
	char uevent_path[512];
	char gadget_name[UDC_NAME_LENGTH_MAX];

	errno = error_code;
	perror("ioctl USB_RAW_IOCTL_RUN");

	if (error_code != EBUSY)
		return;

	fprintf(stderr,
		"Raw Gadget could not bind to an available UDC. This usually means the selected UDC is already owned by another gadget driver, or the --udc-driver name did not match the kernel's UDC gadget name.\n");
	fprintf(stderr,
		"Requested Raw Gadget bind: udc_driver=%s udc_device=%s\n",
		udc_driver,
		udc_device);

	if (snprintf(function_path,
		     sizeof(function_path),
		     "/sys/class/udc/%s/function",
		     udc_device) < (int)sizeof(function_path) &&
	    read_single_line_file(function_path,
				  function_name,
				  sizeof(function_name)) == 0 &&
	    function_name[0] != '\0') {
		fprintf(stderr,
			"The UDC currently reports gadget function '%s'. Stop that owner before retrying.\n",
			function_name);
	}

	if (snprintf(uevent_path,
		     sizeof(uevent_path),
		     "/sys/class/udc/%s/uevent",
		     udc_device) < (int)sizeof(uevent_path) &&
	    read_uevent_value(uevent_path,
			      "USB_UDC_NAME",
			      gadget_name,
			      sizeof(gadget_name)) == 0 &&
	    strcmp(gadget_name, udc_driver) != 0) {
		fprintf(stderr,
			"The kernel reports USB_UDC_NAME='%s' for UDC '%s'. If the controller is otherwise idle, retry with '--udc-driver %s --udc-device %s'.\n",
			gadget_name,
			udc_device,
			gadget_name,
			udc_device);
	}

	fprintf(stderr,
		"Common competing owners here are a lingering GadgetFS process, a configfs gadget, or another legacy gadget driver.\n");
}

static void report_raw_device_open_failure(const char *path, int error_code)
{
	errno = error_code;
	perror(path);

	if (error_code != ENOENT)
		return;

	fprintf(stderr,
		"Raw Gadget device node '%s' is missing. This usually means the raw_gadget module is not loaded or this kernel was built without CONFIG_USB_RAW_GADGET.\n",
		path);
	fprintf(stderr,
		"Try 'sudo modprobe raw_gadget'. If that fails or '%s' still does not appear, Raw Gadget is not available on the current kernel.\n",
		path);
}

static int choose_bulk_out_address(struct raw_runtime *runtime)
{
	struct usb_raw_eps_info info;
	int count;
	int i;

	count = raw_eps_info(runtime->fd, &info);
	if (count < 0) {
		perror("ioctl USB_RAW_IOCTL_EPS_INFO");
		return -1;
	}

	for (i = 0; i < count; ++i) {
		const struct usb_raw_ep_info *ep = &info.eps[i];
		const uint8_t candidate_address = (ep->addr == USB_RAW_EP_ADDR_ANY)
			? DEFAULT_BULK_OUT_ADDRESS
			: (uint8_t)ep->addr;

		if (runtime->verbose) {
			fprintf(stderr,
				"Raw Gadget endpoint[%d]: name=%s addr=%u bulk=%u in=%u out=%u maxpacket=%u\n",
				i,
				(const char *)ep->name,
				ep->addr,
				ep->caps.type_bulk,
				ep->caps.dir_in,
				ep->caps.dir_out,
				ep->limits.maxpacket_limit);
		}

		if (!ep->caps.type_bulk || !ep->caps.dir_out)
			continue;
		if (candidate_address != DEFAULT_BULK_OUT_ADDRESS) {
			if (runtime->verbose) {
				fprintf(stderr,
					"Skipping bulk OUT endpoint %s at address 0x%02x because the Linux udl driver submits render URBs to endpoint 0x%02x.\n",
					(const char *)ep->name,
					candidate_address,
					DEFAULT_BULK_OUT_ADDRESS);
			}
			continue;
		}

		runtime->bulk_out_address = candidate_address;
		runtime->bulk_out_address_valid = true;
		return 0;
	}

	fprintf(stderr,
		"Unable to find a Raw Gadget bulk OUT endpoint at address 0x%02x. The Linux udl driver submits bulk writes to endpoint 0x%02x, so this UDC layout is not compatible with the current spoofing path.\n",
		DEFAULT_BULK_OUT_ADDRESS,
		DEFAULT_BULK_OUT_ADDRESS);
	return -1;
}

static int enable_bulk_out_endpoint(struct raw_runtime *runtime)
{
	struct usb_endpoint_descriptor descriptor;
	int handle;

	if (runtime->bulk_out_handle >= 0)
		return 0;
	if (!runtime->bulk_out_address_valid && choose_bulk_out_address(runtime) != 0)
		return -1;

	memset(&descriptor, 0, sizeof(descriptor));
	descriptor.bLength = USB_DT_ENDPOINT_SIZE;
	descriptor.bDescriptorType = USB_DT_ENDPOINT;
	descriptor.bEndpointAddress = runtime->bulk_out_address;
	descriptor.bmAttributes = USB_ENDPOINT_XFER_BULK;
	descriptor.wMaxPacketSize = host_to_le16(512u);

	handle = raw_ep_enable(runtime->fd, &descriptor);
	if (handle < 0) {
		perror("ioctl USB_RAW_IOCTL_EP_ENABLE");
		return -1;
	}

	runtime->bulk_out_handle = handle;
	fprintf(stderr,
		"Raw Gadget bulk OUT enabled: handle=%d address=0x%02x\n",
		runtime->bulk_out_handle,
		runtime->bulk_out_address);
	return 0;
}

static void disable_bulk_out_endpoint(struct raw_runtime *runtime)
{
	int handle;
	int rc;

	if (runtime->bulk_out_handle < 0)
		return;

	handle = runtime->bulk_out_handle;
	runtime->bulk_out_handle = -1;
	rc = raw_ep_disable(runtime->fd, handle);
	if (rc < 0 && runtime->verbose && errno != EINVAL)
		perror("ioctl USB_RAW_IOCTL_EP_DISABLE");
}

static int start_bulk_out_drain_thread(struct raw_runtime *runtime)
{
	int rc;

	if (runtime->bulk_out_thread_created)
		return 0;

	atomic_store(&runtime->bulk_out_thread_stop, false);
	rc = pthread_create(&runtime->bulk_out_thread, NULL, bulk_out_drain_thread_main, runtime);
	if (rc != 0) {
		errno = rc;
		perror("pthread_create bulk OUT drain");
		return -1;
	}

	runtime->bulk_out_thread_created = true;
	return 0;
}

static void stop_bulk_out_drain_thread(struct raw_runtime *runtime)
{
	if (runtime->bulk_out_thread_created) {
		int rc;

		atomic_store(&runtime->bulk_out_thread_stop, true);
		rc = pthread_cancel(runtime->bulk_out_thread);
		if (rc != 0 && rc != ESRCH && runtime->verbose) {
			errno = rc;
			perror("pthread_cancel bulk OUT drain");
		}
		rc = pthread_join(runtime->bulk_out_thread, NULL);
		if (rc != 0 && runtime->verbose) {
			errno = rc;
			perror("pthread_join bulk OUT drain");
		}
		runtime->bulk_out_thread_created = false;
		atomic_store(&runtime->bulk_out_thread_stop, false);
		disable_bulk_out_endpoint(runtime);
		return;
	}

	disable_bulk_out_endpoint(runtime);
}

static void reset_configuration_state(struct raw_runtime *runtime)
{
	runtime->current_configuration = 0u;
	stop_bulk_out_drain_thread(runtime);
}

static int prepare_standard_request(struct raw_runtime *runtime,
				    const struct usb_ctrlrequest *setup,
				    enum control_action *action,
				    struct usb_raw_control_io *io)
{
	const uint8_t request = setup->bRequest;
	const uint8_t request_type = setup->bRequestType;
	const uint16_t value = (uint16_t)setup->wValue;
	const uint16_t index = (uint16_t)setup->wIndex;
	const uint16_t length = (uint16_t)setup->wLength;
	size_t response_length;

	if ((request_type & USB_TYPE_MASK) != USB_TYPE_STANDARD)
		return 1;

	switch (request) {
	case USB_REQ_GET_DESCRIPTOR:
		switch ((uint8_t)(value >> 8)) {
		case USB_DT_DEVICE:
			memcpy(io->data,
			       &runtime->device_descriptor,
			       sizeof(runtime->device_descriptor));
			io->inner.length = sizeof(runtime->device_descriptor);
			*action = CONTROL_ACTION_WRITE;
			return 0;
		case USB_DT_DEVICE_QUALIFIER:
			memcpy(io->data,
			       &runtime->qualifier_descriptor,
			       sizeof(runtime->qualifier_descriptor));
			io->inner.length = sizeof(runtime->qualifier_descriptor);
			*action = CONTROL_ACTION_WRITE;
			return 0;
		case USB_DT_CONFIG:
			response_length = build_config_descriptor(io->data,
							  sizeof(io->data),
							  runtime->bulk_out_address_valid
								  ? runtime->bulk_out_address
								  : DEFAULT_BULK_OUT_ADDRESS,
							  false);
			if (response_length == 0u)
				return -1;
			io->inner.length = response_length;
			*action = CONTROL_ACTION_WRITE;
			return 0;
		case USB_DT_OTHER_SPEED_CONFIG:
			response_length = build_config_descriptor(io->data,
							  sizeof(io->data),
							  runtime->bulk_out_address_valid
								  ? runtime->bulk_out_address
								  : DEFAULT_BULK_OUT_ADDRESS,
							  true);
			if (response_length == 0u)
				return -1;
			io->inner.length = response_length;
			*action = CONTROL_ACTION_WRITE;
			return 0;
		case USB_DT_STRING:
			response_length = build_string_descriptor(io->data,
							  sizeof(io->data),
							  (uint8_t)(value & 0xffu));
			if (response_length == 0u)
				return 1;
			io->inner.length = response_length;
			*action = CONTROL_ACTION_WRITE;
			return 0;
		case UDL_VENDOR_DESCRIPTOR_TYPE:
			memcpy(io->data,
			       runtime->vendor_descriptor,
			       runtime->vendor_descriptor_len);
			io->inner.length = runtime->vendor_descriptor_len;
			*action = CONTROL_ACTION_WRITE;
			return 0;
		default:
			return 1;
		}
	case USB_REQ_GET_STATUS:
		io->data[0] = 0u;
		io->data[1] = 0u;
		io->inner.length = 2u;
		*action = CONTROL_ACTION_WRITE;
		return 0;
	case USB_REQ_GET_CONFIGURATION:
		io->data[0] = runtime->current_configuration;
		io->inner.length = 1u;
		*action = CONTROL_ACTION_WRITE;
		return 0;
	case USB_REQ_GET_INTERFACE:
		(void)index;
		io->data[0] = 0u;
		io->inner.length = 1u;
		*action = CONTROL_ACTION_WRITE;
		return 0;
	case USB_REQ_SET_CONFIGURATION:
		runtime->current_configuration = (uint8_t)(value & 0xffu);
		if (runtime->current_configuration != 0u) {
			if (enable_bulk_out_endpoint(runtime) != 0)
				return -1;
			if (raw_vbus_draw(runtime->fd, 125u) < 0) {
				perror("ioctl USB_RAW_IOCTL_VBUS_DRAW");
				return -1;
			}
			if (raw_configure(runtime->fd) < 0) {
				perror("ioctl USB_RAW_IOCTL_CONFIGURE");
				return -1;
			}
			if (start_bulk_out_drain_thread(runtime) != 0)
				return -1;
		} else {
			stop_bulk_out_drain_thread(runtime);
		}
		io->inner.length = 0u;
		*action = CONTROL_ACTION_READ;
		return 0;
	case USB_REQ_SET_INTERFACE:
		(void)index;
		io->inner.length = 0u;
		*action = CONTROL_ACTION_READ;
		return 0;
	default:
		break;
	}

	if (request_type == 0x00u && length == 0u) {
		io->inner.length = 0u;
		*action = CONTROL_ACTION_READ;
		return 0;
	}

	return 1;
}

static int prepare_vendor_request(struct raw_runtime *runtime,
				  const struct usb_ctrlrequest *setup,
				  enum control_action *action,
				  struct usb_raw_control_io *io)
{
	const uint8_t request_type = setup->bRequestType;
	const uint8_t request = setup->bRequest;
	const uint16_t value = (uint16_t)setup->wValue;
	const uint16_t index = (uint16_t)setup->wIndex;
	const uint16_t length = (uint16_t)setup->wLength;

	if (request_type == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_CHANNEL) {
		if (length != sizeof(k_std_channel)) {
			fprintf(stderr, "Unexpected channel select payload length %u\n", length);
			return -1;
		}
		io->inner.length = length;
		*action = CONTROL_ACTION_READ;
		return 0;
	}

	if (request_type == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_EDID &&
	    index == UDL_EDID_INDEX) {
		const uint16_t byte_index = (uint16_t)(value >> 8);

		io->data[0] = 0u;
		io->data[1] = (byte_index < sizeof(runtime->edid)) ? runtime->edid[byte_index] : 0u;
		io->inner.length = 2u;
		*action = CONTROL_ACTION_WRITE;
		return 0;
	}

	return 1;
}

static int prepare_control_transfer(struct raw_runtime *runtime,
				    const struct usb_ctrlrequest *setup,
				    enum control_action *action,
				    struct usb_raw_control_io *io)
{
	int rc;

	if (runtime->verbose) {
		fprintf(stderr,
			"Raw Gadget setup: bmRequestType=0x%02x bRequest=0x%02x wValue=0x%04x wIndex=0x%04x wLength=%u\n",
			setup->bRequestType,
			setup->bRequest,
			(unsigned int)((uint16_t)setup->wValue),
			(unsigned int)((uint16_t)setup->wIndex),
			(unsigned int)((uint16_t)setup->wLength));
	}

	rc = prepare_standard_request(runtime, setup, action, io);
	if (rc <= 0)
		return rc;

	rc = prepare_vendor_request(runtime, setup, action, io);
	if (rc <= 0)
		return rc;

	if (setup->bRequestType == 0x00u && setup->wLength == 0u) {
		if (runtime->verbose)
			fprintf(stderr, "Unhandled zero-length device OUT setup, acknowledging to observe subsequent traffic\n");
		io->inner.length = 0u;
		*action = CONTROL_ACTION_READ;
		return 0;
	}

	if (runtime->verbose)
		fprintf(stderr, "Unhandled setup request, stalling ep0\n");
	*action = CONTROL_ACTION_STALL;
	return 0;
}

static int postprocess_control_out(struct raw_runtime *runtime,
				   const struct usb_ctrlrequest *setup,
				   const uint8_t *payload,
				   size_t length)
{
	if (setup->bRequestType == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    setup->bRequest == UDL_VENDOR_REQUEST_CHANNEL) {
		if (length != sizeof(k_std_channel)) {
			fprintf(stderr, "Channel select transferred %zu bytes instead of %zu\n",
				length,
				sizeof(k_std_channel));
			return -1;
		}
		if (memcmp(payload, k_std_channel, sizeof(k_std_channel)) != 0 && runtime->verbose)
			fprintf(stderr, "Received non-standard channel select payload\n");
	}

	return 0;
}

static int handle_connect(struct raw_runtime *runtime)
{
	reset_configuration_state(runtime);
	if (choose_bulk_out_address(runtime) != 0)
		return -1;

	fprintf(stderr,
		"Raw Gadget selected bulk OUT endpoint address 0x%02x\n",
		runtime->bulk_out_address);
	return 0;
}

static int run_loop(struct raw_runtime *runtime)
{
	while (!stop_requested) {
		struct usb_raw_control_event event;
		int rc = raw_event_fetch(runtime->fd, &event);

		if (rc < 0) {
			if (errno == EINTR && stop_requested)
				break;
			if (errno == EINTR)
				continue;
			perror("ioctl USB_RAW_IOCTL_EVENT_FETCH");
			return -1;
		}

		switch ((enum usb_raw_event_type)event.inner.type) {
		case USB_RAW_EVENT_CONNECT:
			fprintf(stderr, "Raw Gadget event: %s\n", event_name(event.inner.type));
			if (handle_connect(runtime) != 0)
				return -1;
			break;
		case USB_RAW_EVENT_RESET:
		case USB_RAW_EVENT_DISCONNECT:
			fprintf(stderr, "Raw Gadget event: %s\n", event_name(event.inner.type));
			reset_configuration_state(runtime);
			break;
		case USB_RAW_EVENT_SUSPEND:
		case USB_RAW_EVENT_RESUME:
			fprintf(stderr, "Raw Gadget event: %s\n", event_name(event.inner.type));
			break;
		case USB_RAW_EVENT_CONTROL: {
			struct usb_raw_control_io io;
			enum control_action action = CONTROL_ACTION_STALL;
			int transfer_rc;

			memset(&io, 0, sizeof(io));
			io.inner.ep = 0u;
			io.inner.flags = 0u;
			io.inner.length = 0u;

			if (prepare_control_transfer(runtime, &event.ctrl, &action, &io) != 0)
				return -1;

			if ((uint16_t)event.ctrl.wLength < io.inner.length)
				io.inner.length = (uint16_t)event.ctrl.wLength;

			switch (action) {
			case CONTROL_ACTION_STALL:
				if (raw_ep0_stall(runtime->fd) < 0) {
					perror("ioctl USB_RAW_IOCTL_EP0_STALL");
					return -1;
				}
				break;
			case CONTROL_ACTION_WRITE:
				transfer_rc = raw_ep0_write(runtime->fd, &io);
				if (transfer_rc < 0) {
					perror("ioctl USB_RAW_IOCTL_EP0_WRITE");
					return -1;
				}
				if (runtime->verbose)
					fprintf(stderr, "ep0 IN transferred %d bytes\n", transfer_rc);
				break;
			case CONTROL_ACTION_READ:
				transfer_rc = raw_ep0_read(runtime->fd, &io);
				if (transfer_rc < 0) {
					perror("ioctl USB_RAW_IOCTL_EP0_READ");
					return -1;
				}
				if (runtime->verbose)
					fprintf(stderr, "ep0 OUT transferred %d bytes\n", transfer_rc);
				if (postprocess_control_out(runtime,
						    &event.ctrl,
						    io.data,
						    (size_t)transfer_rc) != 0)
					return -1;
				break;
			}
			break;
		}
		default:
			fprintf(stderr,
				"Raw Gadget event: %s (%u)\n",
				event_name(event.inner.type),
				event.inner.type);
			break;
		}
	}

	return 0;
}

int main(int argc, char **argv)
{
	struct options opts;
	struct raw_runtime runtime;
	char udc_driver[UDC_NAME_LENGTH_MAX] = {0};
	char udc_device[UDC_NAME_LENGTH_MAX] = {0};
	int ret = EXIT_FAILURE;

	default_options(&opts);
	if (parse_args(argc, argv, &opts) != 0) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	if ((!opts.udc_driver || !opts.udc_device) &&
	    auto_detect_udc(udc_driver, sizeof(udc_driver), udc_device, sizeof(udc_device)) != 0) {
		return EXIT_FAILURE;
	}
	if (opts.udc_driver &&
	    snprintf(udc_driver, sizeof(udc_driver), "%s", opts.udc_driver) >= (int)sizeof(udc_driver))
		return EXIT_FAILURE;
	if (opts.udc_device &&
	    snprintf(udc_device, sizeof(udc_device), "%s", opts.udc_device) >= (int)sizeof(udc_device))
		return EXIT_FAILURE;

	memset(&runtime, 0, sizeof(runtime));
	runtime.fd = -1;
	runtime.bulk_out_handle = -1;
	runtime.bulk_out_address = DEFAULT_BULK_OUT_ADDRESS;
	runtime.verbose = opts.verbose;
	atomic_init(&runtime.bulk_out_thread_stop, false);
	build_default_edid(runtime.edid);
	if (opts.edid_path && load_edid_file(opts.edid_path, runtime.edid) != 0)
		goto out;
	build_vendor_descriptor(&runtime);
	build_device_descriptor(&runtime, &opts);
	build_device_qualifier(&runtime);

	runtime.fd = open(opts.raw_device_path, O_RDWR);
	if (runtime.fd < 0) {
		report_raw_device_open_failure(opts.raw_device_path, errno);
		goto out;
	}

	if (raw_init(runtime.fd, USB_SPEED_HIGH, udc_driver, udc_device) != 0) {
		perror("ioctl USB_RAW_IOCTL_INIT");
		goto out;
	}
	if (raw_run(runtime.fd) != 0) {
		report_raw_run_failure(udc_driver, udc_device, errno);
		goto out;
	}

	printf("DisplayLink Raw Gadget prepared. raw=%s udc_driver=%s udc_device=%s\n",
	       opts.raw_device_path,
	       udc_driver,
	       udc_device);
	if (opts.edid_path)
		printf("Using EDID override from %s\n", opts.edid_path);
	printf("Entering Raw Gadget event loop. Press Ctrl+C to stop.\n");
	printf("This control-plane prototype enables the DisplayLink vendor descriptor/channel/EDID path and drains bulk OUT traffic, but it does not yet decode or render the stream.\n");

	install_signal_handlers();
	if (run_loop(&runtime) != 0)
		goto out;

	ret = EXIT_SUCCESS;

out:
	stop_bulk_out_drain_thread(&runtime);
	if (runtime.fd >= 0)
		close(runtime.fd);
	return ret;
}