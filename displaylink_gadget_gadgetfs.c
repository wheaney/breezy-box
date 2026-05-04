#define _POSIX_C_SOURCE 200809L

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadgetfs.h>
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

#include "udl_sink.h"

#define DISPLAYLINK_VENDOR_ID 0x17e9
#define DISPLAYLINK_PRODUCT_ID 0x0104

#define USB_LANG_EN_US 0x0409u

#define UDL_MSG_BULK 0xafU
#define UDL_CMD_WRITEREG 0x20U
#define UDL_CMD_WRITERAW8 0x60U
#define UDL_CMD_WRITERL8 0x61U
#define UDL_CMD_WRITECOPY8 0x62U
#define UDL_CMD_WRITERLX8 0x63U
#define UDL_CMD_WRITERAW16 0x68U
#define UDL_CMD_WRITERL16 0x69U
#define UDL_CMD_WRITECOPY16 0x6aU
#define UDL_CMD_WRITERLX16 0x6bU
#define UDL_MAX_COMMAND_PIXELS 256U
#define DEFAULT_DECODE_WIDTH 1920U
#define DEFAULT_DECODE_HEIGHT 1080U
#define DEFAULT_GADGETFS_EP_OUT_ADDRESS 0x01U
#define DEFAULT_GADGETFS_EP_IN_ADDRESS (USB_DIR_IN | DEFAULT_GADGETFS_EP_OUT_ADDRESS)

#define GADGETFS_DEVICE_CONFIG_TAG 0u
#define GADGETFS_ENDPOINT_CONFIG_TAG 1u

#define UDL_VENDOR_DESCRIPTOR_TYPE 0x5fU
#define UDL_VENDOR_DESCRIPTOR_VERSION 0x0001U
#define UDL_VENDOR_KEY_MAX_PIXELS 0x0200U
#define UDL_VENDOR_REQUEST_CHANNEL 0x12U
#define UDL_VENDOR_REQUEST_EDID 0x02U
#define UDL_EDID_INDEX 0x00a1U

struct options {
	const char *mount_path;
	const char *device_name;
	const char *dump_image_path;
	uint16_t vendor_id;
	uint16_t product_id;
	uint32_t decode_width;
	uint32_t decode_height;
	bool decode_stream;
	bool stay_alive;
	bool verbose;
};

enum udl_stream_parse_result {
	UDL_STREAM_PARSE_COMPLETE,
	UDL_STREAM_PARSE_NEED_MORE,
	UDL_STREAM_PARSE_INVALID,
};

struct udl_decode_runtime {
	struct udl_sink sink;
	uint16_t *framebuffer_rgb565;
	uint32_t *framebuffer_xrgb8888;
	uint8_t *pending;
	size_t pending_len;
	size_t pending_capacity;
	uint64_t bulk_packets;
	uint64_t bulk_bytes;
	uint64_t decoded_commands;
	uint64_t decode_errors;
	uint64_t dropped_bytes;
	uint32_t width;
	uint32_t height;
	const char *dump_image_path;
	bool enabled;
	bool verbose;
};

struct gadgetfs_runtime {
	int ep0_fd;
	int ep_in_fd;
	int ep_out_fd;
	const char *mount_path;
	char device_name[256];
	char ep_in_name[256];
	char ep_out_name[256];
	uint8_t ep_in_address;
	uint8_t ep_out_address;
	uint8_t current_configuration;
	bool verbose;
	struct udl_decode_runtime decoder;
	uint8_t edid[128];
	uint8_t vendor_descriptor[16];
	size_t vendor_descriptor_len;
};

struct __attribute__((packed)) gadgetfs_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};

struct __attribute__((packed)) gadgetfs_config_block {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct gadgetfs_endpoint_descriptor source;
	struct gadgetfs_endpoint_descriptor sink;
};

struct __attribute__((packed)) gadgetfs_descriptor_block {
	uint32_t tag;
	struct gadgetfs_config_block fs;
	struct gadgetfs_config_block hs;
	struct usb_device_descriptor device;
};

struct __attribute__((packed)) gadgetfs_endpoint_block {
	uint32_t tag;
	struct gadgetfs_endpoint_descriptor fs;
	struct gadgetfs_endpoint_descriptor hs;
};

_Static_assert(sizeof(struct gadgetfs_endpoint_descriptor) == USB_DT_ENDPOINT_SIZE,
	       "GadgetFS endpoint descriptors must be 7 bytes");
_Static_assert(sizeof(struct gadgetfs_config_block) ==
	       USB_DT_CONFIG_SIZE + USB_DT_INTERFACE_SIZE + (2u * USB_DT_ENDPOINT_SIZE),
	       "GadgetFS config block size must match descriptor lengths");
_Static_assert(sizeof(struct gadgetfs_endpoint_block) ==
	       sizeof(uint32_t) + (2u * USB_DT_ENDPOINT_SIZE),
	       "GadgetFS endpoint block size must match endpoint descriptor lengths");

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
#define le16_to_host(value) ((uint16_t)(value))
#else
#define host_to_le16(value) __builtin_bswap16((uint16_t)(value))
#define le16_to_host(value) __builtin_bswap16((uint16_t)(value))
#endif

static void usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s [options]\n"
		"  --mount-path PATH       GadgetFS mount path (default: /dev/gadget)\n"
		"  --device-name NAME      GadgetFS device file name under the mount path\n"
		"  --vendor-id HEX         USB vendor ID (default: 0x17e9)\n"
		"  --product-id HEX        USB product ID (default: 0x0104)\n"
		"  --no-stay-alive         Exit after descriptors and endpoint configuration\n"
		"  --no-decode             Leave bulk traffic undecoded and only log packet sizes\n"
		"  --decode-width PIXELS   Sink storage width for decoded traffic (default: 1920)\n"
		"  --decode-height PIXELS  Sink storage height for decoded traffic (default: 1080)\n"
		"  --dump-image PATH       Write a binary PPM snapshot of the decoded frame on exit\n"
		"  --verbose               Log control requests and bulk traffic\n",
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

static int parse_u32(const char *text, uint32_t *value)
{
	char *end = NULL;
	unsigned long parsed;

	errno = 0;
	parsed = strtoul(text, &end, 0);
	if (errno != 0 || !end || *end != '\0' || parsed > 0xffffffffUL)
		return -1;

	*value = (uint32_t)parsed;
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

static uint32_t udl_count_from_byte(uint8_t value)
{
	return value == 0u ? UDL_MAX_COMMAND_PIXELS : (uint32_t)value;
}

static enum udl_stream_parse_result udl_parse_writerlx_length(const uint8_t *command,
							      size_t length,
							      size_t bytes_per_pixel,
							      size_t *command_len_out)
{
	uint32_t produced = 0u;
	const uint32_t total_pixels = udl_count_from_byte(command[5]);
	size_t offset = 6u;

	if (length < 7u)
		return UDL_STREAM_PARSE_NEED_MORE;

	while (produced < total_pixels) {
		uint32_t raw_count;
		size_t raw_bytes;
		uint32_t repeat_count;

		if (offset >= length)
			return UDL_STREAM_PARSE_NEED_MORE;

		raw_count = udl_count_from_byte(command[offset]);
		offset += 1u;
		if (raw_count > total_pixels - produced)
			return UDL_STREAM_PARSE_INVALID;

		raw_bytes = (size_t)raw_count * bytes_per_pixel;
		if (length - offset < raw_bytes)
			return UDL_STREAM_PARSE_NEED_MORE;

		offset += raw_bytes;
		produced += raw_count;
		if (produced == total_pixels)
			break;

		if (offset >= length)
			return UDL_STREAM_PARSE_NEED_MORE;

		repeat_count = command[offset];
		offset += 1u;
		if (repeat_count == 0u || repeat_count > total_pixels - produced)
			return UDL_STREAM_PARSE_INVALID;

		produced += repeat_count;
	}

	*command_len_out = offset;
	return UDL_STREAM_PARSE_COMPLETE;
}

static enum udl_stream_parse_result udl_next_command_length(const uint8_t *command,
						    size_t length,
						    size_t *command_len_out)
{
	uint32_t pixel_count;
	size_t command_len;

	if (length < 2u)
		return UDL_STREAM_PARSE_NEED_MORE;
	if (command[0] != UDL_MSG_BULK)
		return UDL_STREAM_PARSE_INVALID;

	switch (command[1]) {
	case UDL_CMD_WRITEREG:
		if (length < 4u)
			return UDL_STREAM_PARSE_NEED_MORE;
		*command_len_out = 4u;
		return UDL_STREAM_PARSE_COMPLETE;
	case UDL_CMD_WRITERAW8:
		if (length < 6u)
			return UDL_STREAM_PARSE_NEED_MORE;
		pixel_count = udl_count_from_byte(command[5]);
		command_len = 6u + (size_t)pixel_count;
		break;
	case UDL_CMD_WRITERL8:
		command_len = 7u;
		break;
	case UDL_CMD_WRITECOPY8:
		command_len = 9u;
		break;
	case UDL_CMD_WRITERLX8:
		return udl_parse_writerlx_length(command, length, 1u, command_len_out);
	case UDL_CMD_WRITERAW16:
		if (length < 6u)
			return UDL_STREAM_PARSE_NEED_MORE;
		pixel_count = udl_count_from_byte(command[5]);
		command_len = 6u + ((size_t)pixel_count * 2u);
		break;
	case UDL_CMD_WRITERL16:
		command_len = 8u;
		break;
	case UDL_CMD_WRITECOPY16:
		command_len = 9u;
		break;
	case UDL_CMD_WRITERLX16:
		return udl_parse_writerlx_length(command, length, 2u, command_len_out);
	default:
		return UDL_STREAM_PARSE_INVALID;
	}

	if (length < command_len)
		return UDL_STREAM_PARSE_NEED_MORE;

	*command_len_out = command_len;
	return UDL_STREAM_PARSE_COMPLETE;
}

static int udl_decoder_reserve_pending(struct udl_decode_runtime *decoder, size_t additional)
{
	size_t needed;
	size_t new_capacity;
	uint8_t *new_pending;

	if (SIZE_MAX - decoder->pending_len < additional)
		return -1;

	needed = decoder->pending_len + additional;
	if (needed <= decoder->pending_capacity)
		return 0;

	new_capacity = decoder->pending_capacity ? decoder->pending_capacity : 65536u;
	while (new_capacity < needed) {
		if (new_capacity > SIZE_MAX / 2u) {
			new_capacity = needed;
			break;
		}
		new_capacity *= 2u;
	}

	new_pending = realloc(decoder->pending, new_capacity);
	if (!new_pending)
		return -1;

	decoder->pending = new_pending;
	decoder->pending_capacity = new_capacity;
	return 0;
}

static void udl_decoder_consume(struct udl_decode_runtime *decoder, size_t count)
{
	if (count >= decoder->pending_len) {
		decoder->pending_len = 0u;
		return;
	}

	memmove(decoder->pending, decoder->pending + count, decoder->pending_len - count);
	decoder->pending_len -= count;
}

static void udl_decoder_report_success(struct udl_decode_runtime *decoder,
					       size_t command_len,
					       const struct udl_sink_damage *damage)
{
	if (!decoder->verbose)
		return;

	if (damage && damage->touched) {
		fprintf(stderr,
			"decoded command #%llu len=%zu damage=(%u,%u)-(%u,%u) pixels=%u mode=%ux%u depth=%u\n",
			(unsigned long long)decoder->decoded_commands,
			command_len,
			damage->x1,
			damage->y1,
			damage->x2,
			damage->y2,
			damage->pixel_count,
			(unsigned int)udl_sink_get_hpixels(&decoder->sink),
			(unsigned int)udl_sink_get_vpixels(&decoder->sink),
			(unsigned int)udl_sink_get_color_depth(&decoder->sink));
	} else {
		fprintf(stderr,
			"decoded command #%llu len=%zu mode=%ux%u depth=%u\n",
			(unsigned long long)decoder->decoded_commands,
			command_len,
			(unsigned int)udl_sink_get_hpixels(&decoder->sink),
			(unsigned int)udl_sink_get_vpixels(&decoder->sink),
			(unsigned int)udl_sink_get_color_depth(&decoder->sink));
	}
}

static int udl_decoder_init(struct udl_decode_runtime *decoder, const struct options *opts)
{
	size_t pixel_count;

	memset(decoder, 0, sizeof(*decoder));
	decoder->enabled = opts->decode_stream;
	decoder->verbose = opts->verbose;
	decoder->width = opts->decode_width;
	decoder->height = opts->decode_height;
	decoder->dump_image_path = opts->dump_image_path;

	if (!decoder->enabled)
		return 0;

	if (decoder->width == 0u || decoder->height == 0u)
		return -1;
	if ((size_t)decoder->height > SIZE_MAX / (size_t)decoder->width)
		return -1;

	pixel_count = (size_t)decoder->width * (size_t)decoder->height;
	decoder->framebuffer_rgb565 = calloc(pixel_count, sizeof(*decoder->framebuffer_rgb565));
	if (!decoder->framebuffer_rgb565)
		return -1;
	if (decoder->dump_image_path) {
		decoder->framebuffer_xrgb8888 = calloc(pixel_count, sizeof(*decoder->framebuffer_xrgb8888));
		if (!decoder->framebuffer_xrgb8888)
			return -1;
	}

	udl_sink_init(&decoder->sink,
		      decoder->framebuffer_rgb565,
		      decoder->width,
		      decoder->height,
		      decoder->width);
	if (decoder->framebuffer_xrgb8888)
		udl_sink_attach_xrgb8888_output(&decoder->sink,
					       decoder->framebuffer_xrgb8888,
					       decoder->width);
	return 0;
}

static void udl_decoder_destroy(struct udl_decode_runtime *decoder)
{
	if (!decoder)
		return;

	udl_sink_destroy(&decoder->sink);
	free(decoder->framebuffer_rgb565);
	free(decoder->framebuffer_xrgb8888);
	free(decoder->pending);
	decoder->framebuffer_rgb565 = NULL;
	decoder->framebuffer_xrgb8888 = NULL;
	decoder->pending = NULL;
	decoder->pending_len = 0u;
	decoder->pending_capacity = 0u;
}

static uint32_t udl_decoder_visible_width(const struct udl_decode_runtime *decoder)
{
	const uint16_t signaled_width = udl_sink_get_hpixels(&decoder->sink);

	if (signaled_width != 0u && signaled_width <= decoder->width)
		return signaled_width;

	return decoder->width;
}

static uint32_t udl_decoder_visible_height(const struct udl_decode_runtime *decoder)
{
	const uint16_t signaled_height = udl_sink_get_vpixels(&decoder->sink);

	if (signaled_height != 0u && signaled_height <= decoder->height)
		return signaled_height;

	return decoder->height;
}

static int udl_decoder_dump_image(const struct udl_decode_runtime *decoder)
{
	FILE *file;
	uint8_t *row_buffer;
	const uint32_t width = udl_decoder_visible_width(decoder);
	const uint32_t height = udl_decoder_visible_height(decoder);
	uint32_t row;

	if (!decoder || !decoder->enabled || !decoder->dump_image_path || !decoder->framebuffer_xrgb8888)
		return 0;
	if (width == 0u || height == 0u)
		return 0;

	file = fopen(decoder->dump_image_path, "wb");
	if (!file) {
		perror(decoder->dump_image_path);
		return -1;
	}

	if (fprintf(file, "P6\n%u %u\n255\n", width, height) < 0) {
		perror("write PPM header");
		fclose(file);
		return -1;
	}

	row_buffer = malloc((size_t)width * 3u);
	if (!row_buffer) {
		fprintf(stderr, "Unable to allocate row buffer for image dump\n");
		fclose(file);
		return -1;
	}

	for (row = 0; row < height; ++row) {
		const uint32_t *src = decoder->framebuffer_xrgb8888 + ((size_t)row * decoder->width);
		uint32_t column;

		for (column = 0; column < width; ++column) {
			const uint32_t pixel = src[column];
			row_buffer[(size_t)column * 3u] = (uint8_t)(pixel >> 16);
			row_buffer[(size_t)column * 3u + 1u] = (uint8_t)(pixel >> 8);
			row_buffer[(size_t)column * 3u + 2u] = (uint8_t)pixel;
		}

		if (fwrite(row_buffer, 1u, (size_t)width * 3u, file) != (size_t)width * 3u) {
			perror("write PPM pixels");
			free(row_buffer);
			fclose(file);
			return -1;
		}
	}

	free(row_buffer);
	if (fclose(file) != 0) {
		perror("close PPM image");
		return -1;
	}

	fprintf(stderr,
		"Wrote decoded framebuffer snapshot to %s (%ux%u)\n",
		decoder->dump_image_path,
		width,
		height);
	return 0;
}

static void udl_decoder_print_summary(const struct udl_decode_runtime *decoder)
{
	if (!decoder || !decoder->enabled)
		return;

	fprintf(stderr,
		"UDL decode summary: bulk_packets=%llu bulk_bytes=%llu commands=%llu errors=%llu dropped=%llu configured=%ux%u signaled=%ux%u depth=%u\n",
		(unsigned long long)decoder->bulk_packets,
		(unsigned long long)decoder->bulk_bytes,
		(unsigned long long)decoder->decoded_commands,
		(unsigned long long)decoder->decode_errors,
		(unsigned long long)decoder->dropped_bytes,
		decoder->width,
		decoder->height,
		(unsigned int)udl_sink_get_hpixels(&decoder->sink),
		(unsigned int)udl_sink_get_vpixels(&decoder->sink),
		(unsigned int)udl_sink_get_color_depth(&decoder->sink));
}

static int udl_decoder_feed(struct udl_decode_runtime *decoder, const uint8_t *data, size_t length)
{
	if (!decoder || !decoder->enabled || !data || length == 0u)
		return 0;

	decoder->bulk_packets += 1u;
	decoder->bulk_bytes += length;

	if (udl_decoder_reserve_pending(decoder, length) != 0) {
		fprintf(stderr, "Unable to grow UDL pending buffer\n");
		return -1;
	}

	memcpy(decoder->pending + decoder->pending_len, data, length);
	decoder->pending_len += length;

	while (decoder->pending_len > 0u) {
		uint8_t *sync;
		size_t command_len = 0u;
		enum udl_stream_parse_result parse_result;
		struct udl_sink_damage damage;
		enum udl_sink_result decode_result;

		sync = memchr(decoder->pending, UDL_MSG_BULK, decoder->pending_len);
		if (!sync) {
			decoder->dropped_bytes += decoder->pending_len;
			decoder->pending_len = 0u;
			break;
		}
		if (sync != decoder->pending) {
			size_t skipped = (size_t)(sync - decoder->pending);
			decoder->dropped_bytes += skipped;
			udl_decoder_consume(decoder, skipped);
		}

		if (decoder->pending_len < 2u)
			break;
		if (decoder->pending[1] == UDL_MSG_BULK) {
			udl_decoder_consume(decoder, 1u);
			continue;
		}

		parse_result = udl_next_command_length(decoder->pending, decoder->pending_len, &command_len);
		if (parse_result == UDL_STREAM_PARSE_NEED_MORE)
			break;
		if (parse_result == UDL_STREAM_PARSE_INVALID) {
			decoder->decode_errors += 1u;
			if (decoder->verbose) {
				fprintf(stderr,
					"Invalid UDL command framing at opcode 0x%02x, resyncing\n",
					decoder->pending[1]);
			}
			udl_decoder_consume(decoder, 1u);
			continue;
		}

		udl_sink_clear_damage(&damage);
		decode_result = udl_sink_decode_buffer(&decoder->sink, decoder->pending, command_len, &damage);
		if (decode_result != UDL_SINK_OK) {
			decoder->decode_errors += 1u;
			fprintf(stderr,
				"UDL decode error for opcode 0x%02x: %s\n",
				decoder->pending[1],
				udl_sink_result_string(decode_result));
			udl_decoder_consume(decoder, command_len);
			continue;
		}

		decoder->decoded_commands += 1u;
		udl_decoder_report_success(decoder, command_len, &damage);
		udl_decoder_consume(decoder, command_len);
	}

	return 0;
}

static const char *gadgetfs_event_name(enum usb_gadgetfs_event_type type)
{
	switch (type) {
	case GADGETFS_NOP:
		return "nop";
	case GADGETFS_CONNECT:
		return "connect";
	case GADGETFS_DISCONNECT:
		return "disconnect";
	case GADGETFS_SETUP:
		return "setup";
	case GADGETFS_SUSPEND:
		return "suspend";
	default:
		return "unknown";
	}
}

static const char *speed_name(enum usb_device_speed speed)
{
	switch (speed) {
	case USB_SPEED_LOW:
		return "low";
	case USB_SPEED_FULL:
		return "full";
	case USB_SPEED_HIGH:
		return "high";
	case USB_SPEED_SUPER:
		return "super";
	case USB_SPEED_SUPER_PLUS:
		return "super+";
	default:
		return "unknown";
	}
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
	edid[24] = 0x0a;
	for (index = 25u; index <= 34u; ++index)
		edid[index] = 0x00;
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

static void build_vendor_descriptor(struct gadgetfs_runtime *runtime)
{
	const uint32_t pixel_limit = runtime->decoder.width * runtime->decoder.height;

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

static int mount_gadgetfs(const struct options *opts)
{
	if (mkdir_p(opts->mount_path) != 0) {
		perror("mkdir_p gadgetfs mount");
		return -1;
	}

	if (mount("gadgetfs", opts->mount_path, "gadgetfs", 0, NULL) != 0) {
		if (errno != EBUSY) {
			perror("mount gadgetfs");
			return -1;
		}
	}

	return 0;
}

static int copy_auto_detected_device_name(const char *mount_path, char *buffer, size_t capacity)
{
	DIR *directory;
	struct dirent *entry;
	int found = 0;

	directory = opendir(mount_path);
	if (!directory) {
		perror("opendir gadgetfs mount");
		return -1;
	}

	while ((entry = readdir(directory)) != NULL) {
		if (entry->d_name[0] == '.')
			continue;
		if (strncmp(entry->d_name, "ep", 2) == 0)
			continue;
		if (found++) {
			fprintf(stderr,
				"Multiple GadgetFS device nodes present under %s; use --device-name\n",
				mount_path);
			closedir(directory);
			return -1;
		}
		if (snprintf(buffer, capacity, "%s", entry->d_name) >= (int)capacity) {
			closedir(directory);
			return -1;
		}
	}

	closedir(directory);
	if (found != 1) {
		fprintf(stderr,
			"Unable to auto-detect GadgetFS device node under %s\n",
			mount_path);
		return -1;
	}

	return 0;
}

static int parse_endpoint_name(const char *name, uint8_t *address_out, bool *bulk_out)
{
	unsigned long ep_num = 0;
	const char *cursor = name;
	bool is_in;

	if (strncmp(cursor, "ep", 2) != 0)
		return -1;
	cursor += 2;
	if (!isdigit((unsigned char)*cursor))
		return -1;

	while (isdigit((unsigned char)*cursor)) {
		ep_num = (ep_num * 10u) + (unsigned long)(*cursor - '0');
		cursor += 1;
	}
	if (ep_num == 0u || ep_num > 15u)
		return -1;

	if (strstr(cursor, "in") != NULL)
		is_in = true;
	else if (strstr(cursor, "out") != NULL)
		is_in = false;
	else
		return -1;

	*bulk_out = strstr(name, "bulk") != NULL;
	*address_out = (uint8_t)ep_num | (is_in ? USB_DIR_IN : 0u);
	return 0;
}

static int choose_endpoint_name_by_address(const char *mount_path,
					    uint8_t wanted_address,
					    char *buffer,
					    size_t capacity)
{
	DIR *directory;
	struct dirent *entry;
	char first_match[256] = {0};

	directory = opendir(mount_path);
	if (!directory) {
		perror("opendir gadgetfs mount");
		return -1;
	}

	while ((entry = readdir(directory)) != NULL) {
		uint8_t address;
		bool is_bulk;

		if (parse_endpoint_name(entry->d_name, &address, &is_bulk) != 0)
			continue;
		if (address != wanted_address)
			continue;
		if (first_match[0] == '\0')
			snprintf(first_match, sizeof(first_match), "%s", entry->d_name);
		if (!is_bulk)
			continue;
		if (snprintf(buffer, capacity, "%s", entry->d_name) >= (int)capacity) {
			closedir(directory);
			return -1;
		}
		closedir(directory);
		return 0;
	}

	closedir(directory);
	if (first_match[0] == '\0') {
		fprintf(stderr,
			"Unable to find GadgetFS endpoint 0x%02x under %s\n",
			wanted_address,
			mount_path);
		return -1;
	}

	if (snprintf(buffer, capacity, "%s", first_match) >= (int)capacity)
		return -1;
	return 0;
}

static int configure_endpoint_fd(int fd, uint8_t address)
{
	struct gadgetfs_endpoint_block descriptor_block;

	memset(&descriptor_block, 0, sizeof(descriptor_block));
	descriptor_block.tag = GADGETFS_ENDPOINT_CONFIG_TAG;

	descriptor_block.fs.bLength = USB_DT_ENDPOINT_SIZE;
	descriptor_block.fs.bDescriptorType = USB_DT_ENDPOINT;
	descriptor_block.fs.bEndpointAddress = address;
	descriptor_block.fs.bmAttributes = USB_ENDPOINT_XFER_BULK;
	descriptor_block.fs.wMaxPacketSize = host_to_le16(64u);

	descriptor_block.hs = descriptor_block.fs;
	descriptor_block.hs.wMaxPacketSize = host_to_le16(512u);

	if (write(fd, &descriptor_block, sizeof(descriptor_block)) != (ssize_t)sizeof(descriptor_block)) {
		perror("write GadgetFS endpoint descriptor");
		return -1;
	}

	return 0;
}

static int discover_bulk_endpoints(struct gadgetfs_runtime *runtime, const char *mount_path)
{
	if (runtime->ep_in_address == 0u || runtime->ep_out_address == 0u) {
		fprintf(stderr, "GadgetFS endpoints were not initialized before discovery\n");
		return -1;
	}

	if (choose_endpoint_name_by_address(mount_path,
					   runtime->ep_in_address,
					   runtime->ep_in_name,
					   sizeof(runtime->ep_in_name)) != 0)
		return -1;
	if (choose_endpoint_name_by_address(mount_path,
					   runtime->ep_out_address,
					   runtime->ep_out_name,
					   sizeof(runtime->ep_out_name)) != 0)
		return -1;

	return 0;
}

static int configure_bulk_endpoints(struct gadgetfs_runtime *runtime, const char *mount_path)
{
	char path[512];

	snprintf(path, sizeof(path), "%s/%s", mount_path, runtime->ep_in_name);
	runtime->ep_in_fd = open(path, O_RDWR | O_NONBLOCK);
	if (runtime->ep_in_fd < 0) {
		perror(path);
		return -1;
	}
	if (configure_endpoint_fd(runtime->ep_in_fd, runtime->ep_in_address) != 0)
		return -1;

	snprintf(path, sizeof(path), "%s/%s", mount_path, runtime->ep_out_name);
	runtime->ep_out_fd = open(path, O_RDWR | O_NONBLOCK);
	if (runtime->ep_out_fd < 0) {
		perror(path);
		return -1;
	}
	if (configure_endpoint_fd(runtime->ep_out_fd, runtime->ep_out_address) != 0)
		return -1;

	return 0;
}

static void build_descriptor_block(struct gadgetfs_descriptor_block *block,
					 const struct options *opts,
					 const struct gadgetfs_runtime *runtime)
{
	const uint16_t total_length = (uint16_t)sizeof(struct gadgetfs_config_block);
	const uint8_t source_address = runtime->ep_in_address;
	const uint8_t sink_address = runtime->ep_out_address;

	memset(block, 0, sizeof(*block));
	block->tag = GADGETFS_DEVICE_CONFIG_TAG;

	block->device.bLength = USB_DT_DEVICE_SIZE;
	block->device.bDescriptorType = USB_DT_DEVICE;
	block->device.bcdUSB = host_to_le16(0x0200u);
	block->device.bDeviceClass = USB_CLASS_PER_INTERFACE;
	block->device.bDeviceSubClass = 0x00u;
	block->device.bDeviceProtocol = 0x00u;
	block->device.bMaxPacketSize0 = 64u;
	block->device.idVendor = host_to_le16(opts->vendor_id);
	block->device.idProduct = host_to_le16(opts->product_id);
	block->device.bcdDevice = host_to_le16(0x0001u);
	block->device.iManufacturer = 1u;
	block->device.iProduct = 2u;
	block->device.iSerialNumber = 3u;
	block->device.bNumConfigurations = 1u;

	block->fs.config.bLength = USB_DT_CONFIG_SIZE;
	block->fs.config.bDescriptorType = USB_DT_CONFIG;
	block->fs.config.wTotalLength = host_to_le16(total_length);
	block->fs.config.bNumInterfaces = 1u;
	block->fs.config.bConfigurationValue = 1u;
	block->fs.config.iConfiguration = 0u;
	block->fs.config.bmAttributes = USB_CONFIG_ATT_ONE;
	block->fs.config.bMaxPower = 125u;

	block->fs.interface.bLength = USB_DT_INTERFACE_SIZE;
	block->fs.interface.bDescriptorType = USB_DT_INTERFACE;
	block->fs.interface.bInterfaceNumber = 0u;
	block->fs.interface.bAlternateSetting = 0u;
	block->fs.interface.bNumEndpoints = 2u;
	block->fs.interface.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
	block->fs.interface.bInterfaceSubClass = 0x00u;
	block->fs.interface.bInterfaceProtocol = 0x00u;
	block->fs.interface.iInterface = 0u;

	block->fs.source.bLength = USB_DT_ENDPOINT_SIZE;
	block->fs.source.bDescriptorType = USB_DT_ENDPOINT;
	block->fs.source.bEndpointAddress = source_address;
	block->fs.source.bmAttributes = USB_ENDPOINT_XFER_BULK;
	block->fs.source.wMaxPacketSize = host_to_le16(64u);

	block->fs.sink = block->fs.source;
	block->fs.sink.bEndpointAddress = sink_address;

	block->hs.config = block->fs.config;
	block->hs.interface = block->fs.interface;
	block->hs.source = block->fs.source;
	block->hs.sink = block->fs.sink;
	block->hs.source.wMaxPacketSize = host_to_le16(512u);
	block->hs.sink.wMaxPacketSize = host_to_le16(512u);

	(void)runtime;
}

static int write_device_descriptors(int ep0_fd,
				    const struct options *opts,
				    const struct gadgetfs_runtime *runtime)
{
	struct gadgetfs_descriptor_block descriptor_block;

	build_descriptor_block(&descriptor_block, opts, runtime);
	if (write(ep0_fd, &descriptor_block, sizeof(descriptor_block)) != (ssize_t)sizeof(descriptor_block)) {
		perror("write GadgetFS descriptors");
		return -1;
	}

	return 0;
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

static int read_exact(int fd, void *buffer, size_t length)
{
	uint8_t *cursor = buffer;
	size_t remaining = length;

	while (remaining > 0u) {
		ssize_t rc = read(fd, cursor, remaining);
		if (rc < 0) {
			if (errno == EINTR)
				continue;
			return -1;
		}
		if (rc == 0)
			return -1;
		cursor += rc;
		remaining -= (size_t)rc;
	}

	return 0;
}

static int write_control_response(int ep0_fd, const void *data, uint16_t requested, size_t actual)
{
	size_t to_write = actual < (size_t)requested ? actual : (size_t)requested;

	if (to_write == 0u)
		return 0;
	if (write(ep0_fd, data, to_write) != (ssize_t)to_write) {
		perror("write ep0 response");
		return -1;
	}

	return 0;
}

static int ack_control_status_out(int ep0_fd)
{
	uint8_t dummy = 0u;

	if (read(ep0_fd, &dummy, 0) < 0) {
		perror("ack ep0 status OUT");
		return -1;
	}

	return 0;
}

static void close_bulk_endpoints(struct gadgetfs_runtime *runtime)
{
	if (runtime->ep_out_fd >= 0) {
		close(runtime->ep_out_fd);
		runtime->ep_out_fd = -1;
	}
	if (runtime->ep_in_fd >= 0) {
		close(runtime->ep_in_fd);
		runtime->ep_in_fd = -1;
	}
	runtime->ep_in_name[0] = '\0';
	runtime->ep_out_name[0] = '\0';
}

static int ensure_bulk_endpoints_configured(struct gadgetfs_runtime *runtime)
{
	if (runtime->ep_in_fd >= 0 && runtime->ep_out_fd >= 0)
		return 0;
	if (!runtime->mount_path) {
		fprintf(stderr, "GadgetFS mount path is not initialized\n");
		return -1;
	}

	close_bulk_endpoints(runtime);
	if (discover_bulk_endpoints(runtime, runtime->mount_path) != 0)
		return -1;
	if (configure_bulk_endpoints(runtime, runtime->mount_path) != 0) {
		close_bulk_endpoints(runtime);
		return -1;
	}

	fprintf(stdout,
		"Configured GadgetFS bulk endpoints. ep_in=%s ep_out=%s\n",
		runtime->ep_in_name,
		runtime->ep_out_name);
	return 0;
}

static int handle_udl_vendor_request(struct gadgetfs_runtime *runtime,
					     const struct usb_ctrlrequest *setup)
{
	const uint8_t request_type = setup->bRequestType;
	const uint8_t request = setup->bRequest;
	const uint16_t value = le16_to_host(setup->wValue);
	const uint16_t index = le16_to_host(setup->wIndex);
	const uint16_t length = le16_to_host(setup->wLength);

	if (request_type == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_CHANNEL) {
		uint8_t payload[sizeof(k_std_channel)];

		if (length != sizeof(payload)) {
			fprintf(stderr, "Unexpected channel select payload length %u\n", length);
			return -1;
		}
		if (read_exact(runtime->ep0_fd, payload, sizeof(payload)) != 0) {
			perror("read channel select payload");
			return -1;
		}
		if (memcmp(payload, k_std_channel, sizeof(payload)) != 0 && runtime->verbose)
			fprintf(stderr, "Received non-standard channel select payload\n");
		return 0;
	}

	if (request_type == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_EDID &&
	    index == UDL_EDID_INDEX) {
		uint8_t response[2] = {0u, 0u};
		const uint16_t byte_index = (uint16_t)(value >> 8);

		if (byte_index < sizeof(runtime->edid))
			response[1] = runtime->edid[byte_index];
		return write_control_response(runtime->ep0_fd, response, length, sizeof(response));
	}

	return 1;
}

static int handle_standard_request(struct gadgetfs_runtime *runtime,
				   const struct usb_ctrlrequest *setup)
{
	const uint8_t request_type = setup->bRequestType;
	const uint8_t request = setup->bRequest;
	const uint16_t value = le16_to_host(setup->wValue);
	const uint16_t index = le16_to_host(setup->wIndex);
	const uint16_t length = le16_to_host(setup->wLength);
	uint8_t buffer[512];
	size_t response_length;

	if ((request_type & USB_TYPE_MASK) != USB_TYPE_STANDARD)
		return 1;

	switch (request) {
	case USB_REQ_GET_STATUS:
		buffer[0] = 0u;
		buffer[1] = 0u;
		return write_control_response(runtime->ep0_fd, buffer, length, 2u);
	case USB_REQ_GET_CONFIGURATION:
		buffer[0] = runtime->current_configuration;
		return write_control_response(runtime->ep0_fd, buffer, length, 1u);
	case USB_REQ_SET_CONFIGURATION:
		runtime->current_configuration = (uint8_t)(value & 0xffu);
		if (runtime->current_configuration != 0u) {
			if (ensure_bulk_endpoints_configured(runtime) != 0)
				return -1;
		} else {
			close_bulk_endpoints(runtime);
		}
		return ack_control_status_out(runtime->ep0_fd);
	case USB_REQ_GET_INTERFACE:
		buffer[0] = 0u;
		return write_control_response(runtime->ep0_fd, buffer, length, 1u);
	case USB_REQ_SET_INTERFACE:
		(void)index;
		return ack_control_status_out(runtime->ep0_fd);
	case USB_REQ_GET_DESCRIPTOR:
		switch ((uint8_t)(value >> 8)) {
		case USB_DT_STRING:
			response_length = build_string_descriptor(buffer, sizeof(buffer), (uint8_t)(value & 0xffu));
			if (response_length == 0u)
				return 1;
			return write_control_response(runtime->ep0_fd, buffer, length, response_length);
		case UDL_VENDOR_DESCRIPTOR_TYPE:
			return write_control_response(runtime->ep0_fd,
						     runtime->vendor_descriptor,
						     length,
						     runtime->vendor_descriptor_len);
		default:
			return 1;
		}
	default:
		return 1;
	}
}

static int handle_setup_request(struct gadgetfs_runtime *runtime,
				 const struct usb_ctrlrequest *setup)
{
	const uint16_t value = le16_to_host(setup->wValue);
	const uint16_t index = le16_to_host(setup->wIndex);
	const uint16_t length = le16_to_host(setup->wLength);
	int rc;

	if (runtime->verbose) {
		fprintf(stderr,
			"GadgetFS setup: bmRequestType=0x%02x bRequest=0x%02x wValue=0x%04x wIndex=0x%04x wLength=%u\n",
			setup->bRequestType,
			setup->bRequest,
			value,
			index,
			length);
	}

	rc = handle_standard_request(runtime, setup);
	if (rc <= 0)
		return rc;

	rc = handle_udl_vendor_request(runtime, setup);
	if (rc <= 0)
		return rc;

	if (runtime->verbose)
		fprintf(stderr, "Unhandled setup request, leaving it to stall\n");
	return 0;
}

static int handle_ep0_events(struct gadgetfs_runtime *runtime)
{
	struct usb_gadgetfs_event events[8];
	ssize_t read_len;
	size_t i;

	read_len = read(runtime->ep0_fd, events, sizeof(events));
	if (read_len < 0) {
		if (errno != EAGAIN)
			perror("read GadgetFS ep0 events");
		return 0;
	}

	for (i = 0; i < (size_t)read_len / sizeof(events[0]); ++i) {
		switch (events[i].type) {
		case GADGETFS_SETUP:
			if (handle_setup_request(runtime, &events[i].u.setup) != 0)
				return -1;
			break;
		case GADGETFS_CONNECT:
			fprintf(stderr,
				"GadgetFS event: %s speed=%s\n",
				gadgetfs_event_name(events[i].type),
				speed_name(events[i].u.speed));
			break;
		case GADGETFS_DISCONNECT:
			runtime->current_configuration = 0u;
			close_bulk_endpoints(runtime);
			fprintf(stderr, "GadgetFS event: %s\n", gadgetfs_event_name(events[i].type));
			break;
		default:
			fprintf(stderr,
				"GadgetFS event: %s\n",
				gadgetfs_event_name(events[i].type));
			break;
		}
	}

	return 0;
}

static int run_loop(struct gadgetfs_runtime *runtime)
{
	struct pollfd pollfds[2];

	while (!stop_requested) {
		uint8_t bulk_buffer[65536];

		pollfds[0].fd = runtime->ep0_fd;
		pollfds[0].events = POLLIN;
		pollfds[1].fd = runtime->ep_out_fd;
		pollfds[1].events = runtime->ep_out_fd >= 0 ? POLLIN : 0;
		int poll_ret = poll(pollfds, 2, 250);

		if (poll_ret < 0) {
			if (errno == EINTR)
				continue;
			perror("poll GadgetFS");
			return -1;
		}
		if (poll_ret == 0)
			continue;

		if (pollfds[0].revents & POLLIN) {
			if (handle_ep0_events(runtime) != 0)
				return -1;
		}
		if (pollfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
			fprintf(stderr, "ep0 poll revents=0x%x\n", pollfds[0].revents);
		}

		if (pollfds[1].revents & POLLIN) {
			ssize_t read_len = read(runtime->ep_out_fd, bulk_buffer, sizeof(bulk_buffer));
			if (read_len < 0) {
				if (errno != EAGAIN)
					perror("read GadgetFS bulk OUT");
			} else if (read_len > 0) {
				if (runtime->verbose || !runtime->decoder.enabled) {
					fprintf(stderr, "bulk OUT %zd bytes", read_len);
					if (runtime->verbose) {
						size_t prefix = (size_t)read_len < 16u ? (size_t)read_len : 16u;
						size_t j;
						fprintf(stderr, " [");
						for (j = 0; j < prefix; ++j)
							fprintf(stderr, "%s%02x", j ? " " : "", bulk_buffer[j]);
						fprintf(stderr, "]");
					}
					fprintf(stderr, "\n");
				}
				if (udl_decoder_feed(&runtime->decoder, bulk_buffer, (size_t)read_len) != 0)
					return -1;
			}
		}
		if (pollfds[1].revents & (POLLERR | POLLHUP | POLLNVAL)) {
			if (runtime->verbose)
				fprintf(stderr, "bulk OUT poll revents=0x%x\n", pollfds[1].revents);
		}
	}

	return 0;
}

static void default_options(struct options *opts)
{
	memset(opts, 0, sizeof(*opts));
	opts->mount_path = "/dev/gadget";
	opts->vendor_id = DISPLAYLINK_VENDOR_ID;
	opts->product_id = DISPLAYLINK_PRODUCT_ID;
	opts->decode_width = DEFAULT_DECODE_WIDTH;
	opts->decode_height = DEFAULT_DECODE_HEIGHT;
	opts->decode_stream = true;
	opts->stay_alive = true;
	opts->verbose = false;
}

static int parse_args(int argc, char **argv, struct options *opts)
{
	static const struct option long_options[] = {
		{ "mount-path", required_argument, NULL, 'm' },
		{ "device-name", required_argument, NULL, 'D' },
		{ "vendor-id", required_argument, NULL, 'v' },
		{ "product-id", required_argument, NULL, 'p' },
		{ "no-stay-alive", no_argument, NULL, 'x' },
		{ "no-decode", no_argument, NULL, 'd' },
		{ "decode-width", required_argument, NULL, 'W' },
		{ "decode-height", required_argument, NULL, 'H' },
		{ "dump-image", required_argument, NULL, 'o' },
		{ "verbose", no_argument, NULL, 'V' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int opt;

	while ((opt = getopt_long(argc, argv, "m:D:v:p:xdW:H:o:Vh", long_options, NULL)) != -1) {
		switch (opt) {
		case 'm':
			opts->mount_path = optarg;
			break;
		case 'D':
			opts->device_name = optarg;
			break;
		case 'v':
			if (parse_hex16(optarg, &opts->vendor_id) != 0)
				return -1;
			break;
		case 'p':
			if (parse_hex16(optarg, &opts->product_id) != 0)
				return -1;
			break;
		case 'x':
			opts->stay_alive = false;
			break;
		case 'd':
			opts->decode_stream = false;
			break;
		case 'W':
			if (parse_u32(optarg, &opts->decode_width) != 0)
				return -1;
			break;
		case 'H':
			if (parse_u32(optarg, &opts->decode_height) != 0)
				return -1;
			break;
		case 'o':
			opts->dump_image_path = optarg;
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
	struct gadgetfs_runtime runtime;
	char path[512];
	int ret = EXIT_FAILURE;

	default_options(&opts);
	if (parse_args(argc, argv, &opts) != 0) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	if (opts.dump_image_path && !opts.decode_stream) {
		fprintf(stderr, "--dump-image requires decoding to remain enabled\n");
		return EXIT_FAILURE;
	}

	memset(&runtime, 0, sizeof(runtime));
	runtime.ep0_fd = -1;
	runtime.ep_in_fd = -1;
	runtime.ep_out_fd = -1;
	runtime.mount_path = opts.mount_path;
	runtime.ep_in_address = DEFAULT_GADGETFS_EP_IN_ADDRESS;
	runtime.ep_out_address = DEFAULT_GADGETFS_EP_OUT_ADDRESS;
	runtime.verbose = opts.verbose;
	build_default_edid(runtime.edid);

	if (udl_decoder_init(&runtime.decoder, &opts) != 0) {
		fprintf(stderr,
			"Unable to initialize UDL decoder for %ux%u storage\n",
			opts.decode_width,
			opts.decode_height);
		return EXIT_FAILURE;
	}
	build_vendor_descriptor(&runtime);

	if (mount_gadgetfs(&opts) != 0)
		goto out;

	if (opts.device_name) {
		if (snprintf(runtime.device_name, sizeof(runtime.device_name), "%s", opts.device_name) >= (int)sizeof(runtime.device_name))
			goto out;
	} else if (copy_auto_detected_device_name(opts.mount_path,
						 runtime.device_name,
						 sizeof(runtime.device_name)) != 0) {
		goto out;
	}

	if (snprintf(path, sizeof(path), "%s/%s", opts.mount_path, runtime.device_name) >= (int)sizeof(path))
		goto out;
	runtime.ep0_fd = open(path, O_RDWR);
	if (runtime.ep0_fd < 0) {
		perror(path);
		goto out;
	}

	if (write_device_descriptors(runtime.ep0_fd, &opts, &runtime) != 0)
		goto out;

	fprintf(stdout,
		"DisplayLink GadgetFS gadget prepared. device=%s mount=%s awaiting SET_CONFIGURATION\n",
		runtime.device_name,
		opts.mount_path);

	if (opts.stay_alive) {
		fprintf(stdout, "Entering GadgetFS event loop. Press Ctrl+C to stop.\n");
		install_signal_handlers();
		if (run_loop(&runtime) != 0)
			goto out;
	}

	ret = EXIT_SUCCESS;

out:
	if (udl_decoder_dump_image(&runtime.decoder) != 0)
		ret = EXIT_FAILURE;
	udl_decoder_print_summary(&runtime.decoder);
	udl_decoder_destroy(&runtime.decoder);
	close_bulk_endpoints(&runtime);
	if (runtime.ep0_fd >= 0)
		close(runtime.ep0_fd);
	return ret;
}