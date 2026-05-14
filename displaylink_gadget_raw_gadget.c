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
#include <time.h>
#include <unistd.h>

#include <SDL2/SDL.h>

#include "displaylink_compositor.h"
#include "displaylink_session.h"
#include "udl_sink.h"

#define DISPLAYLINK_VENDOR_ID 0x17e9
#define DISPLAYLINK_PRODUCT_ID 0x0104

#define USB_LANG_EN_US 0x0409u

#define DEFAULT_DISPLAY_WIDTH 1920u
#define DEFAULT_DISPLAY_HEIGHT 1080u
#define DEFAULT_BULK_OUT_ADDRESS 0x01u
#define DEFAULT_RAW_DEVICE_PATH "/dev/raw-gadget"
#define DEFAULT_MANUFACTURER_STRING "DisplayLink"
#define DEFAULT_PRODUCT_STRING "DisplayLink Adapter"
#define DEFAULT_SERIAL_STRING "DEADBEEF0001"
#define DEFAULT_MONITOR_NAME "Breezy Box"
#define UDC_SOFT_RECONNECT_SETTLE_USEC 250000u
#define UDL_CAPTURE_STREAM_MAGIC "UDLCAP01"
#define UDL_CAPTURE_STREAM_MAGIC_SIZE 8u
#define UDL_CAPTURE_STREAM_VERSION 1u
#define UDL_CAPTURE_STREAM_BUFFER_SIZE (1024u * 1024u)

#define UDL_VENDOR_DESCRIPTOR_TYPE 0x5fU
#define UDL_VENDOR_DESCRIPTOR_VERSION 0x0001U
#define UDL_VENDOR_KEY_MAX_PIXELS 0x0200U
#define UDL_VENDOR_REQUEST_CHANNEL 0x12U
#define UDL_VENDOR_REQUEST_EDID 0x02U
#define UDL_EDID_INDEX 0x00a1U

#define EP0_BUFFER_SIZE 512u
#define BULK_OUT_BUFFER_SIZE 16384u
#define UDL_PACKET_QUEUE_CAPACITY 256u
#define UDL_BACKLOG_LOG_INTERVAL_NSEC 1000000000ull

struct options {
	const char *raw_device_path;
	const char *edid_path;
	const char *capture_stream_path;
	const char *dump_image_path;
	const char *manufacturer_string;
	const char *product_string;
	const char *serial_string;
	const char *monitor_name;
	const char *udc_driver;
	const char *udc_device;
	uint16_t vendor_id;
	uint16_t product_id;
	enum usb_device_speed usb_speed;
	uint32_t decode_width;
	uint32_t decode_height;
	uint32_t window_scale;
	struct displaylink_output_surface output_surface;
	bool decode_stream;
	bool show_window;
	bool startup_soft_reconnect;
	bool verbose;
};

struct udl_bulk_packet {
	size_t length;
	uint8_t data[BULK_OUT_BUFFER_SIZE];
};

struct udl_decode_runtime {
	struct udl_sink sink;
	struct udl_transport transport;
	uint16_t *framebuffer_rgb565;
	uint32_t *framebuffer_xrgb8888;
	uint32_t *viewer_snapshot_xrgb8888;
	struct displaylink_compositor_surface viewer_surface;
	uint64_t bulk_packets;
	uint64_t bulk_bytes;
	uint32_t width;
	uint32_t height;
	uint32_t framebuffer_xrgb8888_stride_pixels;
	const char *dump_image_path;
	const char *viewer_window_title;
	pthread_t decode_thread;
	pthread_t viewer_thread;
	pthread_mutex_t packet_queue_mutex;
	pthread_mutex_t performance_report_mutex;
	pthread_mutex_t viewer_snapshot_mutex;
	pthread_cond_t packet_queue_not_empty;
	pthread_cond_t packet_queue_not_full;
	pthread_mutex_t framebuffer_mutex;
	struct udl_bulk_packet packet_queue[UDL_PACKET_QUEUE_CAPACITY];
	struct udl_sink_damage viewer_pending_damage;
	size_t packet_queue_read_index;
	size_t packet_queue_write_index;
	size_t packet_queue_count;
	size_t packet_queue_peak_count;
	uint64_t packet_queue_backlog_since_nsec;
	uint64_t packet_queue_last_log_nsec;
	uint64_t packet_queue_full_waits;
	uint64_t performance_last_report_nsec;
	atomic_ullong perf_bulk_reads;
	atomic_ullong perf_bulk_bytes;
	atomic_ullong perf_decoded_commands;
	atomic_ullong perf_no_damage_commands;
	atomic_ullong perf_writereg_commands;
	atomic_ullong perf_writereg_redundant_commands;
	atomic_ullong perf_writeraw8_commands;
	atomic_ullong perf_writerl8_commands;
	atomic_ullong perf_writecopy8_commands;
	atomic_ullong perf_writerlx8_commands;
	atomic_ullong perf_writeraw16_commands;
	atomic_ullong perf_writerl16_commands;
	atomic_ullong perf_writecopy16_commands;
	atomic_ullong perf_writerlx16_commands;
	atomic_ullong perf_writerlx16_raw_spans;
	atomic_ullong perf_writerlx16_repeat_spans;
	atomic_ullong perf_writerlx16_raw_pixels;
	atomic_ullong perf_writerlx16_repeat_pixels;
	atomic_ullong perf_writerlx16_raw_single_pixel_spans;
	atomic_ullong perf_damage_pixels;
	atomic_ullong perf_damage_rectangles;
	atomic_ullong perf_usb_read_wait_nsec;
	atomic_ullong perf_queue_wait_nsec;
	atomic_ullong perf_decode_nsec;
	atomic_ullong perf_snapshot_copy_nsec;
	atomic_ullong perf_sdl_upload_nsec;
	atomic_ullong perf_sdl_present_nsec;
	atomic_ullong perf_sdl_present_count;
	atomic_bool decode_thread_stop;
	atomic_bool viewer_thread_stop;
	bool decode_thread_created;
	bool packet_queue_initialized;
	bool performance_report_mutex_initialized;
	bool viewer_snapshot_mutex_initialized;
	bool viewer_thread_created;
	bool framebuffer_mutex_initialized;
	bool framebuffer_xrgb8888_owned;
	bool enabled;
	bool viewer_enabled;
	bool verbose;
	uint32_t window_scale;
	uint32_t viewer_visible_width;
	uint32_t viewer_visible_height;
};

struct __attribute__((packed)) gadget_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};

struct __attribute__((packed)) gadget_ss_ep_comp_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bMaxBurst;
	uint8_t bmAttributes;
	uint16_t wBytesPerInterval;
};

struct __attribute__((packed)) gadget_config_block {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct gadget_endpoint_descriptor bulk_out;
};

struct __attribute__((packed)) gadget_ss_config_block {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct gadget_endpoint_descriptor bulk_out;
	struct gadget_ss_ep_comp_descriptor bulk_out_companion;
};

struct __attribute__((packed)) gadget_bos_block {
	struct usb_bos_descriptor bos;
	struct usb_ss_cap_descriptor ss_cap;
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
	const char *udc_device;
	const char *capture_stream_path;
	const char *manufacturer_string;
	const char *product_string;
	const char *serial_string;
	const char *monitor_name;
	uint8_t current_configuration;
	int bulk_out_handle;
	FILE *capture_stream_file;
	pthread_t bulk_out_thread;
	uint8_t bulk_out_address;
	uint64_t capture_stream_packets;
	uint64_t capture_stream_bytes;
	bool bulk_out_address_valid;
	bool bulk_out_thread_created;
	bool verbose;
	atomic_bool bulk_out_thread_stop;
	enum usb_device_speed usb_speed;
	struct usb_device_descriptor device_descriptor;
	struct usb_qualifier_descriptor qualifier_descriptor;
	struct gadget_bos_block bos_descriptor;
	uint8_t edid[128];
	uint8_t vendor_descriptor[16];
	size_t vendor_descriptor_len;
	struct udl_decode_runtime decoder;
};

enum control_action {
	CONTROL_ACTION_STALL,
	CONTROL_ACTION_WRITE,
	CONTROL_ACTION_READ,
};

static const uint8_t k_std_channel[16] = {
	0x57, 0xcd, 0xdc, 0xa7, 0x1c, 0x88, 0x5e, 0x15,
	0x60, 0xfe, 0xc6, 0x97, 0x16, 0x3d, 0x47, 0xf2,
};

static volatile sig_atomic_t stop_requested = 0;

static int parse_args(int argc, char **argv, struct options *opts);
static uint32_t udl_decoder_visible_width(const struct udl_decode_runtime *decoder);
static uint32_t udl_decoder_visible_height(const struct udl_decode_runtime *decoder);
static void udl_decoder_destroy(struct udl_decode_runtime *decoder);

static uint64_t monotonic_nanoseconds(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
		return 0u;

	return ((uint64_t)ts.tv_sec * 1000000000ull) + (uint64_t)ts.tv_nsec;
}

static void udl_decoder_clear_damage(struct udl_sink_damage *damage)
{
	if (!damage)
		return;

	udl_sink_clear_damage(damage);
}

static void udl_decoder_record_counter(atomic_ullong *counter, uint64_t value)
{
	if (!counter || value == 0u)
		return;

	atomic_fetch_add(counter, (unsigned long long)value);
}

static void udl_decoder_maybe_log_performance(struct udl_decode_runtime *decoder)
{
	uint64_t now;
	uint64_t interval_nsec;
	unsigned long long bulk_reads;
	unsigned long long bulk_bytes;
	unsigned long long decoded_commands;
	unsigned long long no_damage_commands;
	unsigned long long writereg_commands;
	unsigned long long writereg_redundant_commands;
	unsigned long long writeraw8_commands;
	unsigned long long writerl8_commands;
	unsigned long long writecopy8_commands;
	unsigned long long writerlx8_commands;
	unsigned long long writeraw16_commands;
	unsigned long long writerl16_commands;
	unsigned long long writecopy16_commands;
	unsigned long long writerlx16_commands;
	unsigned long long writerlx16_raw_spans;
	unsigned long long writerlx16_repeat_spans;
	unsigned long long writerlx16_raw_pixels;
	unsigned long long writerlx16_repeat_pixels;
	unsigned long long writerlx16_raw_single_pixel_spans;
	unsigned long long damage_pixels;
	unsigned long long damage_rectangles;
	unsigned long long usb_read_wait_nsec;
	unsigned long long queue_wait_nsec;
	unsigned long long decode_nsec;
	unsigned long long snapshot_copy_nsec;
	unsigned long long sdl_upload_nsec;
	unsigned long long sdl_present_nsec;
	unsigned long long sdl_present_count;
	size_t queue_depth;

	if (!decoder || !decoder->verbose || !decoder->performance_report_mutex_initialized)
		return;

	now = monotonic_nanoseconds();
	if (now == 0u)
		return;

	pthread_mutex_lock(&decoder->performance_report_mutex);
	if (decoder->performance_last_report_nsec == 0u) {
		decoder->performance_last_report_nsec = now;
		pthread_mutex_unlock(&decoder->performance_report_mutex);
		return;
	}

	interval_nsec = now - decoder->performance_last_report_nsec;
	if (interval_nsec < 1000000000ull) {
		pthread_mutex_unlock(&decoder->performance_report_mutex);
		return;
	}
	decoder->performance_last_report_nsec = now;
	bulk_reads = atomic_exchange(&decoder->perf_bulk_reads, 0u);
	bulk_bytes = atomic_exchange(&decoder->perf_bulk_bytes, 0u);
	decoded_commands = atomic_exchange(&decoder->perf_decoded_commands, 0u);
	no_damage_commands = atomic_exchange(&decoder->perf_no_damage_commands, 0u);
	writereg_commands = atomic_exchange(&decoder->perf_writereg_commands, 0u);
	writereg_redundant_commands = atomic_exchange(&decoder->perf_writereg_redundant_commands, 0u);
	writeraw8_commands = atomic_exchange(&decoder->perf_writeraw8_commands, 0u);
	writerl8_commands = atomic_exchange(&decoder->perf_writerl8_commands, 0u);
	writecopy8_commands = atomic_exchange(&decoder->perf_writecopy8_commands, 0u);
	writerlx8_commands = atomic_exchange(&decoder->perf_writerlx8_commands, 0u);
	writeraw16_commands = atomic_exchange(&decoder->perf_writeraw16_commands, 0u);
	writerl16_commands = atomic_exchange(&decoder->perf_writerl16_commands, 0u);
	writecopy16_commands = atomic_exchange(&decoder->perf_writecopy16_commands, 0u);
	writerlx16_commands = atomic_exchange(&decoder->perf_writerlx16_commands, 0u);
	writerlx16_raw_spans = atomic_exchange(&decoder->perf_writerlx16_raw_spans, 0u);
	writerlx16_repeat_spans = atomic_exchange(&decoder->perf_writerlx16_repeat_spans, 0u);
	writerlx16_raw_pixels = atomic_exchange(&decoder->perf_writerlx16_raw_pixels, 0u);
	writerlx16_repeat_pixels = atomic_exchange(&decoder->perf_writerlx16_repeat_pixels, 0u);
	writerlx16_raw_single_pixel_spans = atomic_exchange(&decoder->perf_writerlx16_raw_single_pixel_spans, 0u);
	damage_pixels = atomic_exchange(&decoder->perf_damage_pixels, 0u);
	damage_rectangles = atomic_exchange(&decoder->perf_damage_rectangles, 0u);
	usb_read_wait_nsec = atomic_exchange(&decoder->perf_usb_read_wait_nsec, 0u);
	queue_wait_nsec = atomic_exchange(&decoder->perf_queue_wait_nsec, 0u);
	decode_nsec = atomic_exchange(&decoder->perf_decode_nsec, 0u);
	snapshot_copy_nsec = atomic_exchange(&decoder->perf_snapshot_copy_nsec, 0u);
	sdl_upload_nsec = atomic_exchange(&decoder->perf_sdl_upload_nsec, 0u);
	sdl_present_nsec = atomic_exchange(&decoder->perf_sdl_present_nsec, 0u);
	sdl_present_count = atomic_exchange(&decoder->perf_sdl_present_count, 0u);
	queue_depth = decoder->packet_queue_count;
	pthread_mutex_unlock(&decoder->performance_report_mutex);

	fprintf(stderr,
		"UDL perf: interval=%.2fs reads=%llu bytes=%llu cmds=%llu nodmg=%llu reg=%llu reg_noop=%llu raw8=%llu rl8=%llu copy8=%llu rlx8=%llu raw16=%llu rl16=%llu copy16=%llu rlx16=%llu rlx16_raw_spans=%llu rlx16_repeat_spans=%llu rlx16_raw_px=%llu rlx16_repeat_px=%llu rlx16_raw1=%llu rects=%llu pixels=%llu queue_depth=%zu usb_wait=%.2fms queue_wait=%.2fms decode=%.2fms snapshot=%.2fms sdl_upload=%.2fms sdl_present=%.2fms presents=%llu\n",
		(double)interval_nsec / 1000000000.0,
		bulk_reads,
		bulk_bytes,
		decoded_commands,
		no_damage_commands,
		writereg_commands,
		writereg_redundant_commands,
		writeraw8_commands,
		writerl8_commands,
		writecopy8_commands,
		writerlx8_commands,
		writeraw16_commands,
		writerl16_commands,
		writecopy16_commands,
		writerlx16_commands,
		writerlx16_raw_spans,
		writerlx16_repeat_spans,
		writerlx16_raw_pixels,
		writerlx16_repeat_pixels,
		writerlx16_raw_single_pixel_spans,
		damage_rectangles,
		damage_pixels,
		queue_depth,
		(double)usb_read_wait_nsec / 1000000.0,
		(double)queue_wait_nsec / 1000000.0,
		(double)decode_nsec / 1000000.0,
		(double)snapshot_copy_nsec / 1000000.0,
		(double)sdl_upload_nsec / 1000000.0,
		(double)sdl_present_nsec / 1000000.0,
		sdl_present_count);
}

static void udl_decoder_merge_damage(struct udl_sink_damage *dst,
				     const struct udl_sink_damage *src)
{
	if (!dst || !src || !src->touched)
		return;
	if (!dst->touched) {
		*dst = *src;
		return;
	}

	if (src->x1 < dst->x1)
		dst->x1 = src->x1;
	if (src->y1 < dst->y1)
		dst->y1 = src->y1;
	if (src->x2 > dst->x2)
		dst->x2 = src->x2;
	if (src->y2 > dst->y2)
		dst->y2 = src->y2;
	dst->pixel_count += src->pixel_count;
	dst->touched = true;
}

static void udl_decoder_mark_full_damage(struct udl_sink_damage *damage,
				    uint32_t width,
				    uint32_t height)
{
	if (!damage || width == 0u || height == 0u)
		return;

	damage->touched = true;
	damage->x1 = 0u;
	damage->y1 = 0u;
	damage->x2 = width - 1u;
	damage->y2 = height - 1u;
	damage->pixel_count = width * height;
}

static void udl_decoder_copy_damage_region(uint32_t *dst,
				   const uint32_t *src,
				   uint32_t stride_pixels,
				   uint32_t width,
				   uint32_t height,
				   const struct udl_sink_damage *damage)
{
	uint32_t start_x;
	uint32_t end_x;
	uint32_t start_y;
	uint32_t end_y;
	uint32_t row;
	size_t copy_width;

	if (!dst || !src || !damage || !damage->touched || width == 0u || height == 0u)
		return;

	start_x = damage->x1 < width ? damage->x1 : width - 1u;
	start_y = damage->y1 < height ? damage->y1 : height - 1u;
	end_x = damage->x2 < width ? damage->x2 : width - 1u;
	end_y = damage->y2 < height ? damage->y2 : height - 1u;
	if (end_x < start_x || end_y < start_y)
		return;

	copy_width = (size_t)(end_x - start_x + 1u);
	for (row = start_y; row <= end_y; ++row) {
		memcpy(dst + ((size_t)row * stride_pixels) + start_x,
		       src + ((size_t)row * stride_pixels) + start_x,
		       copy_width * sizeof(*dst));
	}
}

static void udl_decoder_update_backlog_state_locked(struct udl_decode_runtime *decoder)
{
	uint64_t now;
	uint64_t age_nsec;

	if (!decoder)
		return;

	now = monotonic_nanoseconds();
	if (decoder->packet_queue_count > decoder->packet_queue_peak_count)
		decoder->packet_queue_peak_count = decoder->packet_queue_count;

	if (decoder->packet_queue_count == 0u) {
		if (decoder->verbose && decoder->packet_queue_backlog_since_nsec != 0u) {
			age_nsec = now > decoder->packet_queue_backlog_since_nsec
				? now - decoder->packet_queue_backlog_since_nsec
				: 0u;
			if (age_nsec >= UDL_BACKLOG_LOG_INTERVAL_NSEC) {
				fprintf(stderr,
					"UDL packet backlog drained after %.2fs peak=%zu/%u full_waits=%llu\n",
					(double)age_nsec / 1000000000.0,
					decoder->packet_queue_peak_count,
					(unsigned int)UDL_PACKET_QUEUE_CAPACITY,
					(unsigned long long)decoder->packet_queue_full_waits);
			}
		}
		decoder->packet_queue_backlog_since_nsec = 0u;
		decoder->packet_queue_last_log_nsec = 0u;
		decoder->packet_queue_peak_count = 0u;
		return;
	}

	if (decoder->packet_queue_backlog_since_nsec == 0u)
		decoder->packet_queue_backlog_since_nsec = now;
	if (!decoder->verbose)
		return;

	age_nsec = now > decoder->packet_queue_backlog_since_nsec
		? now - decoder->packet_queue_backlog_since_nsec
		: 0u;
	if (age_nsec < UDL_BACKLOG_LOG_INTERVAL_NSEC)
		return;
	if (decoder->packet_queue_last_log_nsec != 0u &&
	    now - decoder->packet_queue_last_log_nsec < UDL_BACKLOG_LOG_INTERVAL_NSEC)
		return;

	decoder->packet_queue_last_log_nsec = now;
	fprintf(stderr,
		"UDL packet backlog persists: depth=%zu/%u peak=%zu/%u age=%.2fs full_waits=%llu\n",
		decoder->packet_queue_count,
		(unsigned int)UDL_PACKET_QUEUE_CAPACITY,
		decoder->packet_queue_peak_count,
		(unsigned int)UDL_PACKET_QUEUE_CAPACITY,
		(double)age_nsec / 1000000000.0,
		(unsigned long long)decoder->packet_queue_full_waits);
}

static bool usb_speed_is_super(enum usb_device_speed speed)
{
	return speed == USB_SPEED_SUPER || speed == USB_SPEED_SUPER_PLUS;
}

static const char *usb_speed_name(enum usb_device_speed speed)
{
	return usb_speed_is_super(speed) ? "super" : "high";
}

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
		"  --capture-stream PATH   Write raw bulk OUT packets to a replayable binary trace\n"
		"  --dump-image PATH       Write a binary PPM snapshot of the decoded frame on exit\n"
		"  --manufacturer TEXT     Override the USB manufacturer string\n"
		"  --product-string TEXT   Override the USB product string\n"
		"  --serial-string TEXT    Override the USB serial string\n"
		"  --monitor-name TEXT     Override the built-in EDID monitor name\n"
		"  --udc-driver NAME       UDC driver name (default: auto-detect)\n"
		"  --udc-device NAME       UDC device name (default: auto-detect)\n"
		"  --vendor-id HEX         USB vendor ID (default: 0x17e9)\n"
		"  --product-id HEX        USB product ID (default: 0x0104)\n"
		"  --usb-speed MODE        USB link speed: high or super (default: high)\n"
		"  --decode-width PIXELS   Sink storage width for decoded traffic (default: 1920)\n"
		"  --decode-height PIXELS  Sink storage height for decoded traffic (default: 1080)\n"
		"  --show-window           Display the decoded framebuffer in a live SDL2 window\n"
		"  --window-scale FACTOR   Integer scale factor for the SDL2 viewer window (default: 1)\n"
		"  --no-decode             Leave bulk traffic undecoded and only drain the endpoint\n"
		"  --no-startup-reconnect  Do not force a startup soft disconnect/connect pulse\n"
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

static int parse_usb_speed(const char *text, enum usb_device_speed *speed)
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

static void public_options_from_internal(struct displaylink_session_options *dst,
					 const struct options *src)
{
	if (!dst || !src)
		return;

	memset(dst, 0, sizeof(*dst));
	dst->raw_device_path = src->raw_device_path;
	dst->edid_path = src->edid_path;
	dst->capture_stream_path = src->capture_stream_path;
	dst->dump_image_path = src->dump_image_path;
	dst->manufacturer_string = src->manufacturer_string;
	dst->product_string = src->product_string;
	dst->serial_string = src->serial_string;
	dst->monitor_name = src->monitor_name;
	dst->udc_driver = src->udc_driver;
	dst->udc_device = src->udc_device;
	dst->vendor_id = src->vendor_id;
	dst->product_id = src->product_id;
	dst->usb_speed = src->usb_speed;
	dst->decode_width = src->decode_width;
	dst->decode_height = src->decode_height;
	dst->window_scale = src->window_scale;
	dst->output_surface = src->output_surface;
	dst->decode_stream = src->decode_stream;
	dst->show_window = src->show_window;
	dst->startup_soft_reconnect = src->startup_soft_reconnect;
	dst->verbose = src->verbose;
}

static void internal_options_from_public(struct options *dst,
					 const struct displaylink_session_options *src)
{
	if (!dst || !src)
		return;

	memset(dst, 0, sizeof(*dst));
	dst->raw_device_path = src->raw_device_path;
	dst->edid_path = src->edid_path;
	dst->capture_stream_path = src->capture_stream_path;
	dst->dump_image_path = src->dump_image_path;
	dst->manufacturer_string = src->manufacturer_string;
	dst->product_string = src->product_string;
	dst->serial_string = src->serial_string;
	dst->monitor_name = src->monitor_name;
	dst->udc_driver = src->udc_driver;
	dst->udc_device = src->udc_device;
	dst->vendor_id = src->vendor_id;
	dst->product_id = src->product_id;
	dst->usb_speed = src->usb_speed;
	dst->decode_width = src->decode_width;
	dst->decode_height = src->decode_height;
	dst->window_scale = src->window_scale;
	dst->output_surface = src->output_surface;
	dst->decode_stream = src->decode_stream;
	dst->show_window = src->show_window;
	dst->startup_soft_reconnect = src->startup_soft_reconnect;
	dst->verbose = src->verbose;
}

static void default_options(struct options *opts)
{
	memset(opts, 0, sizeof(*opts));
	opts->raw_device_path = DEFAULT_RAW_DEVICE_PATH;
	opts->manufacturer_string = DEFAULT_MANUFACTURER_STRING;
	opts->product_string = DEFAULT_PRODUCT_STRING;
	opts->serial_string = DEFAULT_SERIAL_STRING;
	opts->monitor_name = DEFAULT_MONITOR_NAME;
	opts->vendor_id = DISPLAYLINK_VENDOR_ID;
	opts->product_id = DISPLAYLINK_PRODUCT_ID;
	opts->usb_speed = USB_SPEED_HIGH;
	opts->decode_width = DEFAULT_DISPLAY_WIDTH;
	opts->decode_height = DEFAULT_DISPLAY_HEIGHT;
	opts->window_scale = 1u;
	opts->decode_stream = true;
	opts->startup_soft_reconnect = true;
	opts->verbose = false;
}

static int validate_options(const struct options *opts)
{
	if (!opts)
		return -1;
	if (!opts->raw_device_path) {
		fprintf(stderr, "raw_device_path is required\n");
		return -1;
	}
	if (opts->decode_width == 0u || opts->decode_height == 0u) {
		fprintf(stderr, "decode dimensions must be at least 1x1\n");
		return -1;
	}
	if (!opts->decode_stream && opts->dump_image_path) {
		fprintf(stderr, "--dump-image requires decode support; remove --no-decode\n");
		return -1;
	}
	if (!opts->decode_stream && opts->show_window) {
		fprintf(stderr, "--show-window requires decode support; remove --no-decode\n");
		return -1;
	}
	if (opts->window_scale == 0u) {
		fprintf(stderr, "--window-scale must be at least 1\n");
		return -1;
	}
	if (opts->output_surface.pixels) {
		if (!opts->decode_stream) {
			fprintf(stderr, "external output_surface requires decode support\n");
			return -1;
		}
		if (opts->output_surface.width < opts->decode_width ||
		    opts->output_surface.height < opts->decode_height ||
		    opts->output_surface.stride_pixels < opts->decode_width) {
			fprintf(stderr,
				"external output_surface must be at least %ux%u with stride >= %u\n",
				opts->decode_width,
				opts->decode_height,
				opts->decode_width);
			return -1;
		}
	}

	return 0;
}

void displaylink_session_options_init(struct displaylink_session_options *opts)
{
	struct options internal_opts;

	default_options(&internal_opts);
	public_options_from_internal(opts, &internal_opts);
}

int displaylink_session_parse_args(int argc,
				   char **argv,
				   struct displaylink_session_options *opts)
{
	struct options internal_opts;

	if (!opts)
		return -1;

	default_options(&internal_opts);
	optind = 1;
	if (parse_args(argc, argv, &internal_opts) != 0)
		return -1;
	public_options_from_internal(opts, &internal_opts);
	return 0;
}

int displaylink_session_validate_options(const struct displaylink_session_options *opts)
{
	struct options internal_opts;

	if (!opts)
		return -1;

	internal_options_from_public(&internal_opts, opts);
	return validate_options(&internal_opts);
}

void displaylink_session_request_stop(void)
{
	stop_requested = 1;
}

static int parse_args(int argc, char **argv, struct options *opts)
{
	static const struct option long_options[] = {
		{ "raw-device", required_argument, NULL, 'r' },
		{ "edid-file", required_argument, NULL, 'e' },
		{ "capture-stream", required_argument, NULL, 'c' },
		{ "dump-image", required_argument, NULL, 'o' },
		{ "manufacturer", required_argument, NULL, 1000 },
		{ "product-string", required_argument, NULL, 1001 },
		{ "serial-string", required_argument, NULL, 1002 },
		{ "monitor-name", required_argument, NULL, 1003 },
		{ "udc-driver", required_argument, NULL, 'u' },
		{ "udc-device", required_argument, NULL, 'd' },
		{ "vendor-id", required_argument, NULL, 'v' },
		{ "product-id", required_argument, NULL, 'p' },
		{ "usb-speed", required_argument, NULL, 'm' },
		{ "decode-width", required_argument, NULL, 'W' },
		{ "decode-height", required_argument, NULL, 'H' },
		{ "show-window", no_argument, NULL, 's' },
		{ "window-scale", required_argument, NULL, 'S' },
		{ "no-decode", no_argument, NULL, 'x' },
		{ "no-startup-reconnect", no_argument, NULL, 'n' },
		{ "verbose", no_argument, NULL, 'V' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int opt;

	while ((opt = getopt_long(argc, argv, "r:e:c:o:u:d:v:p:m:W:H:sS:xnVh", long_options, NULL)) != -1) {
		switch (opt) {
		case 'r':
			opts->raw_device_path = optarg;
			break;
		case 'e':
			opts->edid_path = optarg;
			break;
		case 'c':
			opts->capture_stream_path = optarg;
			break;
		case 'o':
			opts->dump_image_path = optarg;
			break;
		case 1000:
			opts->manufacturer_string = optarg;
			break;
		case 1001:
			opts->product_string = optarg;
			break;
		case 1002:
			opts->serial_string = optarg;
			break;
		case 1003:
			opts->monitor_name = optarg;
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
		case 'm':
			if (parse_usb_speed(optarg, &opts->usb_speed) != 0)
				return -1;
			break;
		case 'W':
			if (parse_u32(optarg, &opts->decode_width) != 0)
				return -1;
			break;
		case 'H':
			if (parse_u32(optarg, &opts->decode_height) != 0)
				return -1;
			break;
		case 's':
			opts->show_window = true;
			break;
		case 'S':
			if (parse_u32(optarg, &opts->window_scale) != 0)
				return -1;
			break;
		case 'x':
			opts->decode_stream = false;
			break;
		case 'n':
			opts->startup_soft_reconnect = false;
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

static int write_capture_bytes(FILE *file, const void *buffer, size_t length)
{
	if (!file || !buffer || length == 0u)
		return 0;

	if (fwrite(buffer, 1u, length, file) != length) {
		if (errno == 0)
			errno = EIO;
		return -1;
	}

	return 0;
}

static int write_capture_u32_le(FILE *file, uint32_t value)
{
	const uint8_t bytes[4] = {
		(uint8_t)(value & 0xffu),
		(uint8_t)((value >> 8) & 0xffu),
		(uint8_t)((value >> 16) & 0xffu),
		(uint8_t)((value >> 24) & 0xffu),
	};

	return write_capture_bytes(file, bytes, sizeof(bytes));
}

static int write_capture_u64_le(FILE *file, uint64_t value)
{
	const uint8_t bytes[8] = {
		(uint8_t)(value & 0xffu),
		(uint8_t)((value >> 8) & 0xffu),
		(uint8_t)((value >> 16) & 0xffu),
		(uint8_t)((value >> 24) & 0xffu),
		(uint8_t)((value >> 32) & 0xffu),
		(uint8_t)((value >> 40) & 0xffu),
		(uint8_t)((value >> 48) & 0xffu),
		(uint8_t)((value >> 56) & 0xffu),
	};

	return write_capture_bytes(file, bytes, sizeof(bytes));
}

static int open_capture_stream(struct raw_runtime *runtime)
{
	FILE *file;

	if (!runtime || !runtime->capture_stream_path)
		return 0;

	file = fopen(runtime->capture_stream_path, "wb");
	if (!file) {
		perror(runtime->capture_stream_path);
		return -1;
	}

	(void)setvbuf(file, NULL, _IOFBF, UDL_CAPTURE_STREAM_BUFFER_SIZE);
	if (write_capture_bytes(file, UDL_CAPTURE_STREAM_MAGIC, UDL_CAPTURE_STREAM_MAGIC_SIZE) != 0 ||
	    write_capture_u32_le(file, UDL_CAPTURE_STREAM_VERSION) != 0 ||
	    write_capture_u32_le(file, 0u) != 0) {
		perror(runtime->capture_stream_path);
		fclose(file);
		return -1;
	}

	runtime->capture_stream_file = file;
	runtime->capture_stream_packets = 0u;
	runtime->capture_stream_bytes = 0u;
	return 0;
}

static int capture_stream_packet(struct raw_runtime *runtime,
				       const uint8_t *data,
				       size_t length,
				       uint64_t timestamp_nsec)
{
	if (!runtime || !runtime->capture_stream_file || !data || length == 0u)
		return 0;
	if (length > UINT32_MAX) {
		errno = EOVERFLOW;
		return -1;
	}

	if (write_capture_u64_le(runtime->capture_stream_file, timestamp_nsec) != 0 ||
	    write_capture_u32_le(runtime->capture_stream_file, (uint32_t)length) != 0 ||
	    write_capture_u32_le(runtime->capture_stream_file, 0u) != 0 ||
	    write_capture_bytes(runtime->capture_stream_file, data, length) != 0) {
		if (errno == 0)
			errno = EIO;
		return -1;
	}

	runtime->capture_stream_packets += 1u;
	runtime->capture_stream_bytes += (uint64_t)length;
	return 0;
}

static void close_capture_stream(struct raw_runtime *runtime)
{
	int rc;

	if (!runtime || !runtime->capture_stream_file)
		return;

	rc = fclose(runtime->capture_stream_file);
	if (rc != 0) {
		perror(runtime->capture_stream_path ? runtime->capture_stream_path : "capture stream");
	} else {
		fprintf(stderr,
			"Captured %llu bulk packets (%llu bytes) to %s\n",
			(unsigned long long)runtime->capture_stream_packets,
			(unsigned long long)runtime->capture_stream_bytes,
			runtime->capture_stream_path);
	}

	runtime->capture_stream_file = NULL;
}

static void udl_decoder_lock(const struct udl_decode_runtime *decoder)
{
	if (decoder && decoder->framebuffer_mutex_initialized)
		pthread_mutex_lock((pthread_mutex_t *)&decoder->framebuffer_mutex);
}

static void udl_decoder_unlock(const struct udl_decode_runtime *decoder)
{
	if (decoder && decoder->framebuffer_mutex_initialized)
		pthread_mutex_unlock((pthread_mutex_t *)&decoder->framebuffer_mutex);
}

static int udl_decoder_feed(struct udl_decode_runtime *decoder, const uint8_t *data, size_t length)
{
	enum udl_transport_result transport_result;
	struct udl_sink_damage damage;
	struct udl_sink_damage snapshot_damage;
	struct udl_transport_stats before_stats = {0};
	struct udl_transport_stats after_stats = {0};
	const bool collect_performance = decoder && decoder->verbose;
	uint64_t decode_start_nsec;
	uint64_t decode_end_nsec;
	uint64_t snapshot_start_nsec;
	uint64_t snapshot_end_nsec;
	uint32_t visible_width;
	uint32_t visible_height;

	if (!decoder || !decoder->enabled || !data || length == 0u)
		return 0;

	udl_decoder_clear_damage(&damage);
	udl_decoder_clear_damage(&snapshot_damage);
	decode_start_nsec = collect_performance ? monotonic_nanoseconds() : 0u;
	udl_decoder_lock(decoder);
	if (collect_performance)
		before_stats = udl_transport_get_stats(&decoder->transport);
	transport_result = udl_transport_feed(&decoder->transport, data, length, &damage);
	if (transport_result != UDL_TRANSPORT_OK) {
		udl_decoder_unlock(decoder);
		fprintf(stderr,
			"UDL transport error: %s\n",
			udl_transport_result_string(transport_result));
		return -1;
	}
	visible_width = udl_decoder_visible_width(decoder);
	visible_height = udl_decoder_visible_height(decoder);
	snapshot_damage = damage;
	snapshot_start_nsec = collect_performance ? monotonic_nanoseconds() : 0u;
	if (decoder->viewer_enabled &&
	    decoder->framebuffer_xrgb8888 &&
	    decoder->viewer_snapshot_xrgb8888 &&
	    decoder->viewer_snapshot_mutex_initialized) {
		struct udl_sink_damage composed_damage;

		udl_decoder_clear_damage(&composed_damage);
		pthread_mutex_lock(&decoder->viewer_snapshot_mutex);
		if (decoder->viewer_visible_width != visible_width ||
		    decoder->viewer_visible_height != visible_height) {
			decoder->viewer_visible_width = visible_width;
			decoder->viewer_visible_height = visible_height;
			udl_decoder_mark_full_damage(&snapshot_damage,
						    decoder->width,
						    decoder->height);
		}
		if (snapshot_damage.touched) {
			udl_decoder_copy_damage_region(decoder->viewer_snapshot_xrgb8888,
						      decoder->framebuffer_xrgb8888,
						      decoder->width,
					      decoder->framebuffer_xrgb8888_stride_pixels,
						      decoder->height,
						      &snapshot_damage);
			if (!displaylink_compositor_surface_blit_damage(&decoder->viewer_surface,
							       decoder->viewer_snapshot_xrgb8888,
							       decoder->width,
							       decoder->width,
							       decoder->height,
							       0u,
							       0u,
							       &snapshot_damage,
							       &composed_damage)) {
				pthread_mutex_unlock(&decoder->viewer_snapshot_mutex);
				fprintf(stderr, "Viewer compositor update failed\n");
				return -1;
			}
			udl_decoder_merge_damage(&decoder->viewer_pending_damage, &composed_damage);
		}
		pthread_mutex_unlock(&decoder->viewer_snapshot_mutex);
	}
	snapshot_end_nsec = collect_performance ? monotonic_nanoseconds() : 0u;
	if (collect_performance)
		after_stats = udl_transport_get_stats(&decoder->transport);
	udl_decoder_unlock(decoder);
	decode_end_nsec = collect_performance ? monotonic_nanoseconds() : 0u;
	if (collect_performance) {
		udl_decoder_record_counter(&decoder->perf_decoded_commands,
			(uint64_t)(after_stats.decoded_commands - before_stats.decoded_commands));
		udl_decoder_record_counter(&decoder->perf_no_damage_commands,
			(uint64_t)(after_stats.no_damage_commands - before_stats.no_damage_commands));
		udl_decoder_record_counter(&decoder->perf_writereg_commands,
			(uint64_t)(after_stats.writereg_commands - before_stats.writereg_commands));
		udl_decoder_record_counter(&decoder->perf_writereg_redundant_commands,
			(uint64_t)(after_stats.writereg_redundant_commands - before_stats.writereg_redundant_commands));
		udl_decoder_record_counter(&decoder->perf_writeraw8_commands,
			(uint64_t)(after_stats.writeraw8_commands - before_stats.writeraw8_commands));
		udl_decoder_record_counter(&decoder->perf_writerl8_commands,
			(uint64_t)(after_stats.writerl8_commands - before_stats.writerl8_commands));
		udl_decoder_record_counter(&decoder->perf_writecopy8_commands,
			(uint64_t)(after_stats.writecopy8_commands - before_stats.writecopy8_commands));
		udl_decoder_record_counter(&decoder->perf_writerlx8_commands,
			(uint64_t)(after_stats.writerlx8_commands - before_stats.writerlx8_commands));
		udl_decoder_record_counter(&decoder->perf_writeraw16_commands,
			(uint64_t)(after_stats.writeraw16_commands - before_stats.writeraw16_commands));
		udl_decoder_record_counter(&decoder->perf_writerl16_commands,
			(uint64_t)(after_stats.writerl16_commands - before_stats.writerl16_commands));
		udl_decoder_record_counter(&decoder->perf_writecopy16_commands,
			(uint64_t)(after_stats.writecopy16_commands - before_stats.writecopy16_commands));
		udl_decoder_record_counter(&decoder->perf_writerlx16_commands,
			(uint64_t)(after_stats.writerlx16_commands - before_stats.writerlx16_commands));
		udl_decoder_record_counter(&decoder->perf_writerlx16_raw_spans,
			(uint64_t)(after_stats.writerlx16_raw_spans - before_stats.writerlx16_raw_spans));
		udl_decoder_record_counter(&decoder->perf_writerlx16_repeat_spans,
			(uint64_t)(after_stats.writerlx16_repeat_spans - before_stats.writerlx16_repeat_spans));
		udl_decoder_record_counter(&decoder->perf_writerlx16_raw_pixels,
			(uint64_t)(after_stats.writerlx16_raw_pixels - before_stats.writerlx16_raw_pixels));
		udl_decoder_record_counter(&decoder->perf_writerlx16_repeat_pixels,
			(uint64_t)(after_stats.writerlx16_repeat_pixels - before_stats.writerlx16_repeat_pixels));
		udl_decoder_record_counter(&decoder->perf_writerlx16_raw_single_pixel_spans,
			(uint64_t)(after_stats.writerlx16_raw_single_pixel_spans - before_stats.writerlx16_raw_single_pixel_spans));
		if (damage.touched) {
			udl_decoder_record_counter(&decoder->perf_damage_pixels, damage.pixel_count);
			udl_decoder_record_counter(&decoder->perf_damage_rectangles, 1u);
		}
		if (decode_end_nsec >= decode_start_nsec)
			udl_decoder_record_counter(&decoder->perf_decode_nsec,
				decode_end_nsec - decode_start_nsec);
		if (snapshot_end_nsec >= snapshot_start_nsec)
			udl_decoder_record_counter(&decoder->perf_snapshot_copy_nsec,
				snapshot_end_nsec - snapshot_start_nsec);
	}
	if (collect_performance && after_stats.decode_errors > before_stats.decode_errors) {
		fprintf(stderr,
			"UDL transport resync/errors while processing %zu-byte chunk: +%llu error(s) +%llu dropped byte(s)\n",
			length,
			(unsigned long long)(after_stats.decode_errors - before_stats.decode_errors),
			(unsigned long long)(after_stats.dropped_bytes - before_stats.dropped_bytes));
	}
	if (collect_performance)
		udl_decoder_maybe_log_performance(decoder);

	return 0;
}

static void *udl_decode_thread_main(void *arg)
{
	struct udl_decode_runtime *decoder = arg;
	uint8_t packet[BULK_OUT_BUFFER_SIZE];
	size_t packet_length;
	int rc;

	if (!decoder || !decoder->enabled)
		return NULL;

	rc = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	if (rc != 0 && decoder->verbose) {
		errno = rc;
		perror("pthread_setcancelstate UDL decode");
	}
	rc = pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
	if (rc != 0 && decoder->verbose) {
		errno = rc;
		perror("pthread_setcanceltype UDL decode");
	}

	for (;;) {
		pthread_mutex_lock(&decoder->packet_queue_mutex);
		while (decoder->packet_queue_count == 0u &&
		       !atomic_load(&decoder->decode_thread_stop)) {
			pthread_cond_wait(&decoder->packet_queue_not_empty,
					  &decoder->packet_queue_mutex);
		}
		if (decoder->packet_queue_count == 0u &&
		    atomic_load(&decoder->decode_thread_stop)) {
			pthread_mutex_unlock(&decoder->packet_queue_mutex);
			break;
		}

		packet_length = decoder->packet_queue[decoder->packet_queue_read_index].length;
		memcpy(packet,
		       decoder->packet_queue[decoder->packet_queue_read_index].data,
		       packet_length);
		decoder->packet_queue_read_index =
			(decoder->packet_queue_read_index + 1u) % UDL_PACKET_QUEUE_CAPACITY;
		decoder->packet_queue_count -= 1u;
		udl_decoder_update_backlog_state_locked(decoder);
		pthread_cond_signal(&decoder->packet_queue_not_full);
		pthread_mutex_unlock(&decoder->packet_queue_mutex);
		if (decoder->verbose)
			udl_decoder_maybe_log_performance(decoder);

		if (udl_decoder_feed(decoder, packet, packet_length) != 0) {
			stop_requested = 1;
			break;
		}
	}

	return NULL;
}

static int udl_decoder_start_worker(struct udl_decode_runtime *decoder)
{
	int rc;

	if (!decoder || !decoder->enabled || decoder->decode_thread_created)
		return 0;

	atomic_store(&decoder->decode_thread_stop, false);
	rc = pthread_create(&decoder->decode_thread, NULL, udl_decode_thread_main, decoder);
	if (rc != 0) {
		errno = rc;
		perror("pthread_create UDL decode");
		return -1;
	}

	decoder->decode_thread_created = true;
	return 0;
}

static void udl_decoder_stop_worker(struct udl_decode_runtime *decoder)
{
	if (!decoder || !decoder->decode_thread_created)
		return;

	atomic_store(&decoder->decode_thread_stop, true);
	if (decoder->packet_queue_initialized) {
		pthread_mutex_lock(&decoder->packet_queue_mutex);
		pthread_cond_broadcast(&decoder->packet_queue_not_empty);
		pthread_cond_broadcast(&decoder->packet_queue_not_full);
		pthread_mutex_unlock(&decoder->packet_queue_mutex);
	}
	(void)pthread_join(decoder->decode_thread, NULL);
	decoder->decode_thread_created = false;
}

static int udl_decoder_submit(struct udl_decode_runtime *decoder,
				      const uint8_t *data,
				      size_t length)
{
	struct udl_bulk_packet *packet;
	const bool collect_performance = decoder && decoder->verbose;

	if (!decoder || !decoder->enabled || !data || length == 0u)
		return 0;
	if (length > BULK_OUT_BUFFER_SIZE)
		return -1;

	decoder->bulk_packets += 1u;
	decoder->bulk_bytes += length;

	if (!decoder->packet_queue_initialized)
		return udl_decoder_feed(decoder, data, length);

	pthread_mutex_lock(&decoder->packet_queue_mutex);
	while (decoder->packet_queue_count == UDL_PACKET_QUEUE_CAPACITY &&
	       !stop_requested &&
	       !atomic_load(&decoder->decode_thread_stop)) {
		uint64_t wait_start_nsec;
		uint64_t wait_end_nsec;

		decoder->packet_queue_full_waits += 1u;
		udl_decoder_update_backlog_state_locked(decoder);
		wait_start_nsec = collect_performance ? monotonic_nanoseconds() : 0u;
		pthread_cond_wait(&decoder->packet_queue_not_full, &decoder->packet_queue_mutex);
		wait_end_nsec = collect_performance ? monotonic_nanoseconds() : 0u;
		if (collect_performance && wait_end_nsec >= wait_start_nsec)
			udl_decoder_record_counter(&decoder->perf_queue_wait_nsec,
				wait_end_nsec - wait_start_nsec);
	}
	if (stop_requested || atomic_load(&decoder->decode_thread_stop)) {
		pthread_mutex_unlock(&decoder->packet_queue_mutex);
		return -1;
	}

	packet = &decoder->packet_queue[decoder->packet_queue_write_index];
	packet->length = length;
	memcpy(packet->data, data, length);
	decoder->packet_queue_write_index =
		(decoder->packet_queue_write_index + 1u) % UDL_PACKET_QUEUE_CAPACITY;
	decoder->packet_queue_count += 1u;
	udl_decoder_update_backlog_state_locked(decoder);
	pthread_cond_signal(&decoder->packet_queue_not_empty);
	pthread_mutex_unlock(&decoder->packet_queue_mutex);
	if (collect_performance) {
		udl_decoder_record_counter(&decoder->perf_bulk_reads, 1u);
		udl_decoder_record_counter(&decoder->perf_bulk_bytes, length);
		udl_decoder_maybe_log_performance(decoder);
	}
	return 0;
}

static void *udl_viewer_thread_main(void *arg)
{
	struct udl_decode_runtime *decoder = arg;
	SDL_Window *window = NULL;
	SDL_Renderer *renderer = NULL;
	SDL_Texture *texture = NULL;
	int last_window_width = 0;
	int last_window_height = 0;
	bool needs_present = true;
	int rc;

	if (!decoder || !decoder->viewer_enabled || !decoder->viewer_surface.pixels)
		return NULL;

	rc = SDL_Init(SDL_INIT_VIDEO);
	if (rc != 0) {
		fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
		stop_requested = 1;
		return NULL;
	}

	window = SDL_CreateWindow(decoder->viewer_window_title
				 ? decoder->viewer_window_title
				 : DEFAULT_MONITOR_NAME,
					 SDL_WINDOWPOS_CENTERED,
					 SDL_WINDOWPOS_CENTERED,
					 (int)decoder->width,
					 (int)decoder->height,
					 SDL_WINDOW_RESIZABLE);
	if (!window) {
		fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
		SDL_Quit();
		stop_requested = 1;
		return NULL;
	}

	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (!renderer)
		renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
	if (!renderer) {
		fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
		SDL_DestroyWindow(window);
		SDL_Quit();
		stop_requested = 1;
		return NULL;
	}

	texture = SDL_CreateTexture(renderer,
					   SDL_PIXELFORMAT_ARGB8888,
					   SDL_TEXTUREACCESS_STREAMING,
					   (int)decoder->width,
					   (int)decoder->height);
	if (!texture) {
		fprintf(stderr, "SDL_CreateTexture failed: %s\n", SDL_GetError());
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
		stop_requested = 1;
		return NULL;
	}

	while (!stop_requested && !atomic_load(&decoder->viewer_thread_stop)) {
		SDL_Event event;
		uint32_t visible_width = decoder->width;
		uint32_t visible_height = decoder->height;
		struct udl_sink_damage damage;
		SDL_Rect src_rect;
		SDL_Rect dst_rect;
		bool have_damage = false;
		bool size_changed = false;
		uint64_t upload_start_nsec;
		uint64_t upload_end_nsec;
		uint64_t present_start_nsec;
		uint64_t present_end_nsec;

		udl_decoder_clear_damage(&damage);

		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT) {
				stop_requested = 1;
				break;
			}
			if (event.type == SDL_WINDOWEVENT &&
			    (event.window.event == SDL_WINDOWEVENT_EXPOSED ||
			     event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)) {
				needs_present = true;
			}
		}

		if (!decoder->viewer_snapshot_mutex_initialized) {
			SDL_Delay(16u);
			continue;
		}

		pthread_mutex_lock(&decoder->viewer_snapshot_mutex);
		visible_width = decoder->viewer_visible_width != 0u
			? decoder->viewer_visible_width
			: decoder->width;
		visible_height = decoder->viewer_visible_height != 0u
			? decoder->viewer_visible_height
			: decoder->height;
		damage = decoder->viewer_pending_damage;
		have_damage = damage.touched;

		if (visible_width == 0u)
			visible_width = decoder->width;
		if (visible_height == 0u)
			visible_height = decoder->height;

		if (last_window_width != (int)(visible_width * decoder->window_scale) ||
		    last_window_height != (int)(visible_height * decoder->window_scale)) {
			last_window_width = (int)(visible_width * decoder->window_scale);
			last_window_height = (int)(visible_height * decoder->window_scale);
			SDL_SetWindowSize(window, last_window_width, last_window_height);
			size_changed = true;
			needs_present = true;
		}

		if (have_damage) {
			SDL_Rect update_rect;
			const uint8_t *pixels = (const uint8_t *)(decoder->viewer_surface.pixels +
				((size_t)damage.y1 * decoder->width) + damage.x1);

			update_rect.x = (int)damage.x1;
			update_rect.y = (int)damage.y1;
			update_rect.w = (int)(damage.x2 - damage.x1 + 1u);
			update_rect.h = (int)(damage.y2 - damage.y1 + 1u);
			upload_start_nsec = monotonic_nanoseconds();
			if (SDL_UpdateTexture(texture,
					     &update_rect,
					     pixels,
					     (int)(decoder->width * sizeof(*decoder->viewer_surface.pixels))) != 0) {
				pthread_mutex_unlock(&decoder->viewer_snapshot_mutex);
				fprintf(stderr, "SDL_UpdateTexture failed: %s\n", SDL_GetError());
				stop_requested = 1;
				break;
			}
			upload_end_nsec = monotonic_nanoseconds();
			if (upload_end_nsec >= upload_start_nsec)
				udl_decoder_record_counter(&decoder->perf_sdl_upload_nsec,
					upload_end_nsec - upload_start_nsec);
			udl_decoder_clear_damage(&decoder->viewer_pending_damage);
			needs_present = true;
		}
		pthread_mutex_unlock(&decoder->viewer_snapshot_mutex);

		if (!needs_present && !size_changed) {
			SDL_Delay(16u);
			continue;
		}

		src_rect.x = 0;
		src_rect.y = 0;
		src_rect.w = (int)visible_width;
		src_rect.h = (int)visible_height;
		dst_rect.x = 0;
		dst_rect.y = 0;
		dst_rect.w = last_window_width;
		dst_rect.h = last_window_height;

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		present_start_nsec = monotonic_nanoseconds();
		SDL_RenderClear(renderer);
		SDL_RenderCopy(renderer, texture, &src_rect, &dst_rect);
		SDL_RenderPresent(renderer);
		present_end_nsec = monotonic_nanoseconds();
		if (present_end_nsec >= present_start_nsec)
			udl_decoder_record_counter(&decoder->perf_sdl_present_nsec,
				present_end_nsec - present_start_nsec);
		udl_decoder_record_counter(&decoder->perf_sdl_present_count, 1u);
		needs_present = false;
		udl_decoder_maybe_log_performance(decoder);
		SDL_Delay(16u);
	}

	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
	return NULL;
}

static int udl_decoder_start_viewer(struct udl_decode_runtime *decoder)
{
	int rc;

	if (!decoder || !decoder->viewer_enabled || decoder->viewer_thread_created)
		return 0;

	atomic_store(&decoder->viewer_thread_stop, false);
	rc = pthread_create(&decoder->viewer_thread, NULL, udl_viewer_thread_main, decoder);
	if (rc != 0) {
		errno = rc;
		perror("pthread_create SDL viewer");
		return -1;
	}

	decoder->viewer_thread_created = true;
	return 0;
}

static void udl_decoder_stop_viewer(struct udl_decode_runtime *decoder)
{
	if (!decoder || !decoder->viewer_thread_created)
		return;

	atomic_store(&decoder->viewer_thread_stop, true);
	SDL_PushEvent(&(SDL_Event){ .type = SDL_QUIT });
	(void)pthread_join(decoder->viewer_thread, NULL);
	decoder->viewer_thread_created = false;
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
	decoder->viewer_window_title = opts->monitor_name;
	decoder->viewer_enabled = opts->show_window;
	decoder->window_scale = opts->window_scale;
	decoder->framebuffer_xrgb8888_stride_pixels = opts->decode_width;

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
	if (opts->output_surface.pixels) {
		decoder->framebuffer_xrgb8888 = opts->output_surface.pixels;
		decoder->framebuffer_xrgb8888_stride_pixels = opts->output_surface.stride_pixels;
		decoder->framebuffer_xrgb8888_owned = false;
	} else if (decoder->dump_image_path || decoder->viewer_enabled) {
		decoder->framebuffer_xrgb8888 = calloc(pixel_count, sizeof(*decoder->framebuffer_xrgb8888));
		if (!decoder->framebuffer_xrgb8888)
			goto fail;
		decoder->framebuffer_xrgb8888_stride_pixels = decoder->width;
		decoder->framebuffer_xrgb8888_owned = true;
	}
	if (decoder->viewer_enabled) {
		decoder->viewer_snapshot_xrgb8888 = calloc(pixel_count, sizeof(*decoder->viewer_snapshot_xrgb8888));
		if (!decoder->viewer_snapshot_xrgb8888)
			goto fail;
		if (!displaylink_compositor_surface_init(&decoder->viewer_surface,
							       decoder->width,
							       decoder->height))
			goto fail;
	}
	if (pthread_mutex_init(&decoder->packet_queue_mutex, NULL) != 0)
		goto fail;
	if (pthread_cond_init(&decoder->packet_queue_not_empty, NULL) != 0) {
		pthread_mutex_destroy(&decoder->packet_queue_mutex);
		goto fail;
	}
	if (pthread_cond_init(&decoder->packet_queue_not_full, NULL) != 0) {
		pthread_cond_destroy(&decoder->packet_queue_not_empty);
		pthread_mutex_destroy(&decoder->packet_queue_mutex);
		goto fail;
	}
	decoder->packet_queue_initialized = true;
	if (pthread_mutex_init(&decoder->performance_report_mutex, NULL) != 0)
		goto fail;
	decoder->performance_report_mutex_initialized = true;
	if (pthread_mutex_init(&decoder->viewer_snapshot_mutex, NULL) != 0)
		goto fail;
	decoder->viewer_snapshot_mutex_initialized = true;
	if (pthread_mutex_init(&decoder->framebuffer_mutex, NULL) != 0)
		goto fail;
	decoder->framebuffer_mutex_initialized = true;
	atomic_init(&decoder->decode_thread_stop, false);
	atomic_init(&decoder->viewer_thread_stop, false);
	atomic_init(&decoder->perf_bulk_reads, 0u);
	atomic_init(&decoder->perf_bulk_bytes, 0u);
	atomic_init(&decoder->perf_decoded_commands, 0u);
	atomic_init(&decoder->perf_no_damage_commands, 0u);
	atomic_init(&decoder->perf_writereg_commands, 0u);
	atomic_init(&decoder->perf_writereg_redundant_commands, 0u);
	atomic_init(&decoder->perf_writeraw8_commands, 0u);
	atomic_init(&decoder->perf_writerl8_commands, 0u);
	atomic_init(&decoder->perf_writecopy8_commands, 0u);
	atomic_init(&decoder->perf_writerlx8_commands, 0u);
	atomic_init(&decoder->perf_writeraw16_commands, 0u);
	atomic_init(&decoder->perf_writerl16_commands, 0u);
	atomic_init(&decoder->perf_writecopy16_commands, 0u);
	atomic_init(&decoder->perf_writerlx16_commands, 0u);
	atomic_init(&decoder->perf_writerlx16_raw_spans, 0u);
	atomic_init(&decoder->perf_writerlx16_repeat_spans, 0u);
	atomic_init(&decoder->perf_writerlx16_raw_pixels, 0u);
	atomic_init(&decoder->perf_writerlx16_repeat_pixels, 0u);
	atomic_init(&decoder->perf_writerlx16_raw_single_pixel_spans, 0u);
	atomic_init(&decoder->perf_damage_pixels, 0u);
	atomic_init(&decoder->perf_damage_rectangles, 0u);
	atomic_init(&decoder->perf_usb_read_wait_nsec, 0u);
	atomic_init(&decoder->perf_queue_wait_nsec, 0u);
	atomic_init(&decoder->perf_decode_nsec, 0u);
	atomic_init(&decoder->perf_snapshot_copy_nsec, 0u);
	atomic_init(&decoder->perf_sdl_upload_nsec, 0u);
	atomic_init(&decoder->perf_sdl_present_nsec, 0u);
	atomic_init(&decoder->perf_sdl_present_count, 0u);
	udl_decoder_clear_damage(&decoder->viewer_pending_damage);

	udl_sink_init(&decoder->sink,
		      decoder->framebuffer_rgb565,
		      decoder->width,
		      decoder->height,
		      decoder->width);
	udl_transport_init(&decoder->transport, &decoder->sink);
	udl_transport_set_detailed_stats(&decoder->transport, decoder->verbose);
	udl_transport_set_writerlx16_span_stats(&decoder->transport, decoder->verbose);
	if (decoder->framebuffer_xrgb8888) {
		udl_sink_attach_xrgb8888_output(&decoder->sink,
					       decoder->framebuffer_xrgb8888,
					       decoder->framebuffer_xrgb8888_stride_pixels);
	}
	if (udl_decoder_start_worker(decoder) != 0)
		goto fail;

	return 0;

fail:
	udl_decoder_destroy(decoder);
	return -1;
}

static void udl_decoder_destroy(struct udl_decode_runtime *decoder)
{
	if (!decoder)
		return;

	udl_decoder_stop_worker(decoder);
	udl_decoder_stop_viewer(decoder);
	udl_transport_destroy(&decoder->transport);
	udl_sink_destroy(&decoder->sink);
	if (decoder->packet_queue_initialized) {
		pthread_cond_destroy(&decoder->packet_queue_not_full);
		pthread_cond_destroy(&decoder->packet_queue_not_empty);
		pthread_mutex_destroy(&decoder->packet_queue_mutex);
		decoder->packet_queue_initialized = false;
	}
	if (decoder->performance_report_mutex_initialized) {
		pthread_mutex_destroy(&decoder->performance_report_mutex);
		decoder->performance_report_mutex_initialized = false;
	}
	if (decoder->viewer_snapshot_mutex_initialized) {
		pthread_mutex_destroy(&decoder->viewer_snapshot_mutex);
		decoder->viewer_snapshot_mutex_initialized = false;
	}
	if (decoder->framebuffer_mutex_initialized) {
		pthread_mutex_destroy(&decoder->framebuffer_mutex);
		decoder->framebuffer_mutex_initialized = false;
	}
	free(decoder->framebuffer_rgb565);
	if (decoder->framebuffer_xrgb8888_owned)
		free(decoder->framebuffer_xrgb8888);
	free(decoder->viewer_snapshot_xrgb8888);
	displaylink_compositor_surface_destroy(&decoder->viewer_surface);
	decoder->framebuffer_rgb565 = NULL;
	decoder->framebuffer_xrgb8888 = NULL;
	decoder->viewer_snapshot_xrgb8888 = NULL;
	decoder->framebuffer_xrgb8888_owned = false;
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

	udl_decoder_lock(decoder);
	for (row = 0; row < height; ++row) {
		const uint32_t *src = decoder->framebuffer_xrgb8888 +
			((size_t)row * decoder->framebuffer_xrgb8888_stride_pixels);
		uint32_t column;

		for (column = 0; column < width; ++column) {
			const uint32_t pixel = src[column];
			row_buffer[(size_t)column * 3u] = (uint8_t)(pixel >> 16);
			row_buffer[(size_t)column * 3u + 1u] = (uint8_t)(pixel >> 8);
			row_buffer[(size_t)column * 3u + 2u] = (uint8_t)pixel;
		}

		if (fwrite(row_buffer, 1u, (size_t)width * 3u, file) != (size_t)width * 3u) {
			udl_decoder_unlock(decoder);
			perror("write PPM pixels");
			free(row_buffer);
			fclose(file);
			return -1;
		}
	}
	udl_decoder_unlock(decoder);

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
	const struct udl_transport_stats stats = udl_transport_get_stats(&decoder->transport);

	if (!decoder || !decoder->enabled)
		return;

	fprintf(stderr,
		"UDL decode summary: bulk_packets=%llu bulk_bytes=%llu commands=%llu errors=%llu dropped=%llu queue_full_waits=%llu configured=%ux%u signaled=%ux%u depth=%u\n",
		(unsigned long long)decoder->bulk_packets,
		(unsigned long long)decoder->bulk_bytes,
		(unsigned long long)stats.decoded_commands,
		(unsigned long long)stats.decode_errors,
		(unsigned long long)stats.dropped_bytes,
		(unsigned long long)decoder->packet_queue_full_waits,
		decoder->width,
		decoder->height,
		(unsigned int)udl_sink_get_hpixels(&decoder->sink),
		(unsigned int)udl_sink_get_vpixels(&decoder->sink),
		(unsigned int)udl_sink_get_color_depth(&decoder->sink));
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

static int write_single_line_file(const char *path, const char *value)
{
	int fd;
	const size_t length = strlen(value);
	size_t written = 0u;

	fd = open(path, O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return -1;

	while (written < length) {
		ssize_t rc = write(fd, value + written, length - written);

		if (rc < 0) {
			if (errno == EINTR)
				continue;
			close(fd);
			return -1;
		}
		written += (size_t)rc;
	}

	if (close(fd) < 0)
		return -1;

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
	if (found == 0) {
		fprintf(stderr,
			"Unable to auto-detect a UDC under /sys/class/udc because no gadget-capable controllers are currently exposed there.\n");
		fprintf(stderr,
			"On a fresh image this usually means the OTG/device controller is not configured for peripheral mode, the relevant USB gadget/UDC drivers are not loaded, or this board/kernel does not expose a UDC for the chosen port.\n");
		fprintf(stderr,
			"Check: 'ls -la /sys/class/udc', 'lsmod | grep -E \"dwc|udc|gadget|raw_gadget\"', and the live DT for usb@fe800000 / usb2phy@e450 if this is the Rock Pi OTG path.\n");
		fprintf(stderr,
			"If the new image names the controller differently, rerun with explicit '--udc-device <name> --udc-driver <name>' after inspecting /sys/class/udc and the matching uevent file.\n");
		return -1;
	}
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

static int set_udc_soft_connect(const char *udc_device,
				const char *command,
				bool verbose,
				bool ignore_unsupported)
{
	char path[512];
	int saved_errno;

	if (snprintf(path,
		     sizeof(path),
		     "/sys/class/udc/%s/soft_connect",
		     udc_device) >= (int)sizeof(path)) {
		errno = ENAMETOOLONG;
		return -1;
	}

	if (write_single_line_file(path, command) == 0)
		return 0;

	saved_errno = errno;
	if (ignore_unsupported &&
	    (saved_errno == ENOENT || saved_errno == ENOTDIR || saved_errno == EOPNOTSUPP)) {
		if (verbose) {
			fprintf(stderr,
				"UDC soft_connect is unavailable on %s, skipping '%s'\n",
				udc_device,
				command);
		}
		return 0;
	}

	errno = saved_errno;
	if (verbose) {
		fprintf(stderr,
			"Failed to write '%s' to %s\n",
			command,
			path);
		perror("soft_connect");
	}
	return -1;
}

static int read_udc_state(const char *udc_device,
			  char *buffer,
			  size_t capacity)
{
	char path[512];

	if (snprintf(path,
		     sizeof(path),
		     "/sys/class/udc/%s/state",
		     udc_device) >= (int)sizeof(path)) {
		errno = ENAMETOOLONG;
		return -1;
	}

	return read_single_line_file(path, buffer, capacity);
}

static void sleep_microseconds(unsigned int microseconds)
{
	struct timespec delay;

	delay.tv_sec = (time_t)(microseconds / 1000000u);
	delay.tv_nsec = (long)((microseconds % 1000000u) * 1000u);
	while (nanosleep(&delay, &delay) != 0) {
		if (errno != EINTR)
			break;
	}
}

static void force_udc_soft_reconnect(const char *udc_device, bool verbose)
{
	if (verbose) {
		fprintf(stderr,
			"Forcing a UDC soft reconnect on %s so the host sees a fresh attach\n",
			udc_device);
	}

	if (set_udc_soft_connect(udc_device, "disconnect", verbose, true) != 0)
		return;

	sleep_microseconds(UDC_SOFT_RECONNECT_SETTLE_USEC);
	(void)set_udc_soft_connect(udc_device, "connect", verbose, true);
}

static void log_usb_role_switch_states(bool verbose)
{
	DIR *directory;
	struct dirent *entry;
	bool found = false;

	directory = opendir("/sys/class/usb_role");
	if (!directory) {
		if (verbose && errno != ENOENT && errno != ENOTDIR)
			perror("opendir /sys/class/usb_role");
		return;
	}

	while ((entry = readdir(directory)) != NULL) {
		char path[512];
		char value[128];

		if (entry->d_name[0] == '.')
			continue;
		found = true;
		if (snprintf(path,
			     sizeof(path),
			     "/sys/class/usb_role/%s/role",
			     entry->d_name) >= (int)sizeof(path))
			continue;
		if (read_single_line_file(path, value, sizeof(value)) == 0) {
			fprintf(stderr,
				"USB role switch %s currently reports '%s'\n",
				entry->d_name,
				value);
		}
	}

	closedir(directory);
	if (verbose && !found) {
		fprintf(stderr,
			"No USB role switches were exposed under /sys/class/usb_role\n");
	}
}

static void diagnose_udc_attach_state(const char *udc_device,
				      bool startup_soft_reconnect_attempted,
				      bool verbose)
{
	char path[512];
	char state[128];
	char value[128];

	if (read_udc_state(udc_device, state, sizeof(state)) != 0) {
		if (verbose)
			perror("read /sys/class/udc/.../state");
		return;
	}

	if (strcmp(state, "not attached") != 0)
		return;

	fprintf(stderr,
		"UDC %s still reports 'not attached' after startup recovery. If the cable is connected, the host did not observe a fresh attach, soft_connect may be unsupported on this UDC, or the controller/host port still needs a harder reset.\n",
		udc_device);

	if (snprintf(path,
		     sizeof(path),
		     "/sys/class/udc/%s/current_speed",
		     udc_device) < (int)sizeof(path) &&
		read_single_line_file(path, value, sizeof(value)) == 0) {
		fprintf(stderr,
			"UDC %s current_speed=%s\n",
			udc_device,
			value);
	}

	if (snprintf(path,
		     sizeof(path),
		     "/sys/class/udc/%s/soft_connect",
		     udc_device) < (int)sizeof(path)) {
		if (access(path, F_OK) == 0) {
			if (startup_soft_reconnect_attempted) {
				fprintf(stderr,
					"UDC %s exposes soft_connect; the startup disconnect/connect pulse was attempted.\n",
					udc_device);
			} else {
				fprintf(stderr,
					"UDC %s exposes soft_connect, but startup reconnect was disabled so userspace left the idle controller armed for a future host attach.\n",
					udc_device);
			}
		} else {
			fprintf(stderr,
				"UDC %s does not expose soft_connect; userspace cannot force a reattach pulse on this controller.\n",
				udc_device);
		}
	}

	log_usb_role_switch_states(verbose);
}

static void prime_udc_attach_state(const char *udc_device, bool verbose)
{
	char state[128];

	if (read_udc_state(udc_device, state, sizeof(state)) != 0) {
		if (verbose)
			perror("read /sys/class/udc/.../state");
		force_udc_soft_reconnect(udc_device, verbose);
		return;
	}

	if (strcmp(state, "not attached") == 0) {
		fprintf(stderr,
			"UDC %s is armed and idle; leaving soft_connect unchanged so it can wait for a future host attach.\n",
			udc_device);
		return;
	}

	if (verbose) {
		fprintf(stderr,
			"UDC %s is already in state '%s'; forcing a soft reconnect so the current host sees a fresh attach.\n",
			udc_device,
			state);
	}
	force_udc_soft_reconnect(udc_device, verbose);
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

static void build_default_edid(uint8_t edid[128], const char *monitor_name)
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
	write_ascii_monitor_name(&edid[77],
				 monitor_name ? monitor_name : DEFAULT_MONITOR_NAME);

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

static void build_vendor_descriptor(struct raw_runtime *runtime,
				    const struct options *opts)
{
	const uint64_t pixels = (uint64_t)opts->decode_width * (uint64_t)opts->decode_height;
	const uint32_t pixel_limit = pixels > UINT32_MAX ? UINT32_MAX : (uint32_t)pixels;

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
	const bool super_speed = usb_speed_is_super(opts->usb_speed);

	memset(&runtime->device_descriptor, 0, sizeof(runtime->device_descriptor));
	runtime->device_descriptor.bLength = USB_DT_DEVICE_SIZE;
	runtime->device_descriptor.bDescriptorType = USB_DT_DEVICE;
	runtime->device_descriptor.bcdUSB = host_to_le16(super_speed ? 0x0300u : 0x0200u);
	runtime->device_descriptor.bDeviceClass = USB_CLASS_PER_INTERFACE;
	runtime->device_descriptor.bDeviceSubClass = 0u;
	runtime->device_descriptor.bDeviceProtocol = 0u;
	runtime->device_descriptor.bMaxPacketSize0 = super_speed ? 9u : 64u;
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

static void build_bos_descriptor(struct raw_runtime *runtime)
{
	memset(&runtime->bos_descriptor, 0, sizeof(runtime->bos_descriptor));
	if (!usb_speed_is_super(runtime->usb_speed))
		return;

	runtime->bos_descriptor.bos.bLength = USB_DT_BOS_SIZE;
	runtime->bos_descriptor.bos.bDescriptorType = USB_DT_BOS;
	runtime->bos_descriptor.bos.wTotalLength = host_to_le16(sizeof(runtime->bos_descriptor));
	runtime->bos_descriptor.bos.bNumDeviceCaps = 1u;

	runtime->bos_descriptor.ss_cap.bLength = USB_DT_USB_SS_CAP_SIZE;
	runtime->bos_descriptor.ss_cap.bDescriptorType = USB_DT_DEVICE_CAPABILITY;
	runtime->bos_descriptor.ss_cap.bDevCapabilityType = USB_SS_CAP_TYPE;
	runtime->bos_descriptor.ss_cap.bmAttributes = 0u;
	runtime->bos_descriptor.ss_cap.wSpeedSupported = host_to_le16(
		USB_FULL_SPEED_OPERATION |
		USB_HIGH_SPEED_OPERATION |
		USB_5GBPS_OPERATION);
	runtime->bos_descriptor.ss_cap.bFunctionalitySupport = 3u;
	runtime->bos_descriptor.ss_cap.bU1devExitLat = 0u;
	runtime->bos_descriptor.ss_cap.bU2DevExitLat = host_to_le16(0u);
}

static size_t build_string_descriptor(const struct raw_runtime *runtime,
				      uint8_t *buffer,
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
		text = runtime && runtime->manufacturer_string
			? runtime->manufacturer_string
			: DEFAULT_MANUFACTURER_STRING;
		break;
	case 2u:
		text = runtime && runtime->product_string
			? runtime->product_string
			: DEFAULT_PRODUCT_STRING;
		break;
	case 3u:
		text = runtime && runtime->serial_string
			? runtime->serial_string
			: DEFAULT_SERIAL_STRING;
		break;
	default:
		return 0u;
	}

	length = strlen(text);
	if (length > 126u)
		return 0u;
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
				      bool other_speed,
				      bool super_speed)
{
	struct gadget_config_block block;
	struct gadget_ss_config_block ss_block;
	const uint16_t total_length = (uint16_t)(super_speed ? sizeof(ss_block) : sizeof(block));

	if (capacity < total_length)
		return 0u;

	if (super_speed) {
		memset(&ss_block, 0, sizeof(ss_block));
		ss_block.config.bLength = USB_DT_CONFIG_SIZE;
		ss_block.config.bDescriptorType = USB_DT_CONFIG;
		ss_block.config.wTotalLength = host_to_le16(total_length);
		ss_block.config.bNumInterfaces = 1u;
		ss_block.config.bConfigurationValue = 1u;
		ss_block.config.iConfiguration = 0u;
		ss_block.config.bmAttributes = USB_CONFIG_ATT_ONE;
		ss_block.config.bMaxPower = 8u;

		ss_block.interface.bLength = USB_DT_INTERFACE_SIZE;
		ss_block.interface.bDescriptorType = USB_DT_INTERFACE;
		ss_block.interface.bInterfaceNumber = 0u;
		ss_block.interface.bAlternateSetting = 0u;
		ss_block.interface.bNumEndpoints = 1u;
		ss_block.interface.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
		ss_block.interface.bInterfaceSubClass = 0u;
		ss_block.interface.bInterfaceProtocol = 0u;
		ss_block.interface.iInterface = 0u;

		ss_block.bulk_out.bLength = USB_DT_ENDPOINT_SIZE;
		ss_block.bulk_out.bDescriptorType = USB_DT_ENDPOINT;
		ss_block.bulk_out.bEndpointAddress = bulk_out_address;
		ss_block.bulk_out.bmAttributes = USB_ENDPOINT_XFER_BULK;
		ss_block.bulk_out.wMaxPacketSize = host_to_le16(1024u);
		ss_block.bulk_out.bInterval = 0u;

		ss_block.bulk_out_companion.bLength = USB_DT_SS_EP_COMP_SIZE;
		ss_block.bulk_out_companion.bDescriptorType = USB_DT_SS_ENDPOINT_COMP;
		ss_block.bulk_out_companion.bMaxBurst = 0u;
		ss_block.bulk_out_companion.bmAttributes = 0u;
		ss_block.bulk_out_companion.wBytesPerInterval = host_to_le16(0u);

		memcpy(buffer, &ss_block, sizeof(ss_block));
		return sizeof(ss_block);
	}

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
		uint64_t capture_timestamp_nsec = 0u;
		uint64_t read_start_nsec;
		uint64_t read_end_nsec;

		memset(&io, 0, sizeof(io));
		io.inner.ep = handle;
		io.inner.length = sizeof(io.data);

		read_start_nsec = runtime->verbose ? monotonic_nanoseconds() : 0u;
		rc = raw_ep_read(runtime->fd, &io.inner);
		read_end_nsec = runtime->verbose ? monotonic_nanoseconds() : 0u;
		if (runtime->verbose && read_end_nsec >= read_start_nsec)
			udl_decoder_record_counter(&runtime->decoder.perf_usb_read_wait_nsec,
				read_end_nsec - read_start_nsec);
		if (rc < 0) {
			if (errno == EINTR && !stop_requested)
				continue;
			if (!stop_requested && !atomic_load(&runtime->bulk_out_thread_stop) && runtime->verbose)
				perror("ioctl USB_RAW_IOCTL_EP_READ");
			break;
		}
		if (runtime->capture_stream_file) {
			capture_timestamp_nsec = read_end_nsec != 0u ? read_end_nsec : monotonic_nanoseconds();
			if (capture_stream_packet(runtime, io.data, (size_t)rc, capture_timestamp_nsec) != 0) {
				if (!stop_requested)
					perror(runtime->capture_stream_path ? runtime->capture_stream_path : "capture stream");
				stop_requested = 1;
				break;
			}
		}

		if (udl_decoder_submit(&runtime->decoder, io.data, (size_t)rc) != 0)
			break;
		if (runtime->verbose)
			udl_decoder_maybe_log_performance(&runtime->decoder);
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
	descriptor.wMaxPacketSize = host_to_le16(usb_speed_is_super(runtime->usb_speed) ? 1024u : 512u);

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

static void reset_configuration_state(struct raw_runtime *runtime);

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

static void shutdown_raw_gadget(struct raw_runtime *runtime,
				const char *udc_device)
{
	stop_bulk_out_drain_thread(runtime);
	udl_decoder_stop_worker(&runtime->decoder);
	udl_decoder_stop_viewer(&runtime->decoder);
	close_capture_stream(runtime);
	if (runtime->fd >= 0) {
		reset_configuration_state(runtime);
		(void)set_udc_soft_connect(udc_device,
					  "disconnect",
					  runtime->verbose,
					  true);
		sleep_microseconds(UDC_SOFT_RECONNECT_SETTLE_USEC);
		close(runtime->fd);
		runtime->fd = -1;
	}
	udl_decoder_print_summary(&runtime->decoder);
	(void)udl_decoder_dump_image(&runtime->decoder);
	udl_decoder_destroy(&runtime->decoder);
}

static void reset_configuration_state(struct raw_runtime *runtime)
{
	runtime->current_configuration = 0u;
	stop_bulk_out_drain_thread(runtime);
}

static void log_udc_wait_state(struct raw_runtime *runtime, const char *reason)
{
	char state[128];

	if (!runtime->udc_device)
		return;

	if (read_udc_state(runtime->udc_device, state, sizeof(state)) == 0) {
		fprintf(stderr,
			"%s; returning to pre-plug wait state on %s (udc_state=%s)\n",
			reason,
			runtime->udc_device,
			state);
		return;
	}

	fprintf(stderr,
		"%s; returning to pre-plug wait state on %s\n",
		reason,
		runtime->udc_device);
}

static void log_udc_state(struct raw_runtime *runtime, const char *reason)
{
	char state[128];

	if (!runtime->udc_device)
		return;

	if (read_udc_state(runtime->udc_device, state, sizeof(state)) == 0) {
		fprintf(stderr,
			"%s on %s (udc_state=%s)\n",
			reason,
			runtime->udc_device,
			state);
		return;
	}

	fprintf(stderr,
		"%s on %s\n",
		reason,
		runtime->udc_device);
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
		case USB_DT_BOS:
			if (!usb_speed_is_super(runtime->usb_speed))
				return 1;
			memcpy(io->data,
			       &runtime->bos_descriptor,
			       sizeof(runtime->bos_descriptor));
			io->inner.length = sizeof(runtime->bos_descriptor);
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
						  false,
						  usb_speed_is_super(runtime->usb_speed));
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
						  true,
						  false);
			if (response_length == 0u)
				return -1;
			io->inner.length = response_length;
			*action = CONTROL_ACTION_WRITE;
			return 0;
		case USB_DT_STRING:
			response_length = build_string_descriptor(runtime,
							  io->data,
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
	log_udc_state(runtime,
		      "Raw Gadget connect event observed (UDC session active, not proof of host enumeration)");
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
			fprintf(stderr,
				"Raw Gadget event: %s (UDC bound; waiting for host enumeration)\n",
				event_name(event.inner.type));
			if (handle_connect(runtime) != 0)
				return -1;
			break;
		case USB_RAW_EVENT_RESET:
		case USB_RAW_EVENT_DISCONNECT:
			fprintf(stderr, "Raw Gadget event: %s\n", event_name(event.inner.type));
			reset_configuration_state(runtime);
			log_udc_wait_state(runtime,
				event.inner.type == USB_RAW_EVENT_RESET
					? "USB bus reset handled"
					: "Host disconnect handled");
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

int displaylink_session_run(const struct displaylink_session_options *session_opts)
{
	struct options opts;
	struct raw_runtime runtime;
	char udc_driver[UDC_NAME_LENGTH_MAX] = {0};
	char udc_device[UDC_NAME_LENGTH_MAX] = {0};
	int ret = EXIT_FAILURE;

	if (!session_opts)
		return EXIT_FAILURE;
	internal_options_from_public(&opts, session_opts);
	if (validate_options(&opts) != 0)
		return EXIT_FAILURE;
	stop_requested = 0;

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
	runtime.udc_device = udc_device;
	runtime.bulk_out_handle = -1;
	runtime.bulk_out_address = DEFAULT_BULK_OUT_ADDRESS;
	runtime.capture_stream_path = opts.capture_stream_path;
	runtime.manufacturer_string = opts.manufacturer_string;
	runtime.product_string = opts.product_string;
	runtime.serial_string = opts.serial_string;
	runtime.monitor_name = opts.monitor_name;
	runtime.usb_speed = opts.usb_speed;
	runtime.verbose = opts.verbose;
	atomic_init(&runtime.bulk_out_thread_stop, false);
	if (open_capture_stream(&runtime) != 0)
		goto out;
	if (udl_decoder_init(&runtime.decoder, &opts) != 0) {
		fprintf(stderr, "Unable to initialize UDL decode runtime\n");
		goto out;
	}
	if (udl_decoder_start_viewer(&runtime.decoder) != 0) {
		fprintf(stderr, "Unable to start SDL viewer\n");
		goto out;
	}
	build_default_edid(runtime.edid, opts.monitor_name);
	if (opts.edid_path && load_edid_file(opts.edid_path, runtime.edid) != 0)
		goto out;
	build_vendor_descriptor(&runtime, &opts);
	build_device_descriptor(&runtime, &opts);
	build_device_qualifier(&runtime);
	build_bos_descriptor(&runtime);

	runtime.fd = open(opts.raw_device_path, O_RDWR);
	if (runtime.fd < 0) {
		report_raw_device_open_failure(opts.raw_device_path, errno);
		goto out;
	}

	if (raw_init(runtime.fd, opts.usb_speed, udc_driver, udc_device) != 0) {
		perror("ioctl USB_RAW_IOCTL_INIT");
		goto out;
	}
	if (raw_run(runtime.fd) != 0) {
		report_raw_run_failure(udc_driver, udc_device, errno);
		goto out;
	}
	if (opts.startup_soft_reconnect)
		force_udc_soft_reconnect(udc_device, runtime.verbose);
	else
		prime_udc_attach_state(udc_device, runtime.verbose);
	diagnose_udc_attach_state(udc_device,
				      opts.startup_soft_reconnect,
				      runtime.verbose);

	printf("DisplayLink Raw Gadget prepared. raw=%s udc_driver=%s udc_device=%s\n",
	       opts.raw_device_path,
	       udc_driver,
	       udc_device);
	printf("Using %s-speed USB descriptors and endpoint settings.\n",
	       usb_speed_name(opts.usb_speed));
	if (opts.edid_path)
		printf("Using EDID override from %s\n", opts.edid_path);
	if (runtime.capture_stream_path)
		printf("Bulk OUT capture will be written to %s\n", runtime.capture_stream_path);
	printf("Entering Raw Gadget event loop. Press Ctrl+C to stop.\n");
	if (runtime.decoder.enabled) {
		printf("UDL decode is enabled with sink storage %ux%u. Bulk OUT traffic will be decoded as it arrives.\n",
		       runtime.decoder.width,
		       runtime.decoder.height);
		if (runtime.decoder.viewer_enabled)
			printf("A live SDL2 viewer window will be shown at %ux scale.\n",
			       runtime.decoder.window_scale);
		if (runtime.decoder.dump_image_path)
			printf("A decoded snapshot will be written to %s on exit.\n",
			       runtime.decoder.dump_image_path);
	} else {
		printf("UDL decode is disabled; bulk OUT traffic will only be drained.\n");
	}

	if (run_loop(&runtime) != 0)
		goto out;

	ret = EXIT_SUCCESS;

out:
	shutdown_raw_gadget(&runtime, udc_device);
	return ret;
}

#ifndef DISPLAYLINK_SESSION_NO_MAIN
int main(int argc, char **argv)
{
	struct displaylink_session_options opts;

	displaylink_session_options_init(&opts);
	if (displaylink_session_parse_args(argc, argv, &opts) != 0) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	if (displaylink_session_validate_options(&opts) != 0)
		return EXIT_FAILURE;

	install_signal_handlers();
	return displaylink_session_run(&opts);
}
#endif