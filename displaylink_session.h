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
	bool decode_stream;
	bool show_window;
	bool startup_soft_reconnect;
	bool verbose;
};

void displaylink_session_options_init(struct displaylink_session_options *opts);
int displaylink_session_parse_args(int argc,
				    char **argv,
				    struct displaylink_session_options *opts);
int displaylink_session_validate_options(const struct displaylink_session_options *opts);
int displaylink_session_run(const struct displaylink_session_options *opts);
void displaylink_session_request_stop(void);

#endif