#ifndef DISPLAYLINK_SESSION_H
#define DISPLAYLINK_SESSION_H

#include <stdbool.h>
#include <stdint.h>

#include <linux/usb/ch9.h>

struct displaylink_output_surface {
	uint32_t *pixels;
	uint32_t width;
	uint32_t height;
	uint32_t stride_pixels;
};

struct displaylink_output_damage {
	bool touched;
	uint32_t x1;
	uint32_t y1;
	uint32_t x2;
	uint32_t y2;
	uint32_t pixel_count;
};

struct displaylink_output_ring {
	struct displaylink_output_surface *surfaces;
	uint32_t surface_count;
};

struct displaylink_output_frame {
	struct displaylink_output_surface surface;
	struct displaylink_output_damage damage;
	uint64_t sequence;
	uint32_t slot_index;
	uint32_t visible_width;
	uint32_t visible_height;
};

struct displaylink_session_options {
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
	struct displaylink_output_ring output_ring;
	bool decode_stream;
	bool show_window;
	bool startup_soft_reconnect;
	bool verbose;
};

struct displaylink_session;

void displaylink_session_options_init(struct displaylink_session_options *opts);
int displaylink_session_parse_args(int argc,
				    char **argv,
				    struct displaylink_session_options *opts);
int displaylink_session_validate_options(const struct displaylink_session_options *opts);
struct displaylink_session *displaylink_session_create(const struct displaylink_session_options *opts);
void displaylink_session_destroy(struct displaylink_session *session);
int displaylink_session_execute(struct displaylink_session *session);
int displaylink_session_start(struct displaylink_session *session);
int displaylink_session_join(struct displaylink_session *session);
bool displaylink_session_is_started(const struct displaylink_session *session);
bool displaylink_session_is_running(const struct displaylink_session *session);
int displaylink_session_get_exit_code(const struct displaylink_session *session);
bool displaylink_session_acquire_latest_output(struct displaylink_session *session,
					       struct displaylink_output_frame *frame);
void displaylink_session_release_output(struct displaylink_session *session,
					 const struct displaylink_output_frame *frame);
int displaylink_session_run(const struct displaylink_session_options *opts);
void displaylink_session_request_stop(struct displaylink_session *session);

#endif