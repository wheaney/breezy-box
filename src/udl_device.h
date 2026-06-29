#pragma once

#include "usbip.h"
#include "udl_runtime.h"
#include "udl_sink.h"

#include <linux/usb/ch9.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

/* DisplayLink USB identity */
#define DISPLAYLINK_VENDOR_ID  0x17e9
#define DISPLAYLINK_PRODUCT_ID 0x037a

#define USB_LANG_EN_US 0x0409u

/* Endpoint configuration */
#define EP0_BUFFER_SIZE          512u
#define MAX_TRANSFER_BUFFER_SIZE (1024u * 1024u)
#define UDL_BCD_DEVICE           0x0104u
#define UDL_PRIMARY_BULK_OUT_EP  0x01u
#define UDL_INTERRUPT_IN_EP      0x82u
#define UDL_AUX_BULK_OUT_EP      0x0au
#define UDL_INTERRUPT_PACKET_SIZE 8u
#define UDL_INTERRUPT_INTERVAL   4u

/* MS OS descriptor indices */
#define UDL_MS_OS_STRING_INDEX         0xeeu
#define UDL_MS_OS_VENDOR_CODE          0x01u
#define UDL_MS_OS_EXT_COMPAT_INDEX     0x0004u
#define UDL_MS_OS_EXT_PROPERTIES_INDEX 0x0005u

/* UDL bulk/command protocol */
#define UDL_MSG_BULK            0xafu
#define UDL_CMD_OPAQUE_E0       0xe0u
#define UDL_CMD_E0_HEADER_SIZE  9u
#define UDL_CMD_E0_RECORD_SIZE  9u
#define UDL_CMD_E0_RECORD_COUNT 512u
#define UDL_CMD_E0_TOTAL_SIZE   (UDL_CMD_E0_HEADER_SIZE + (UDL_CMD_E0_RECORD_SIZE * UDL_CMD_E0_RECORD_COUNT))
#define UDL_CMD_E0_TRAILER_SIZE 1u

/* Vendor descriptor */
#define UDL_VENDOR_DESCRIPTOR_TYPE     0x5fU
#define UDL_VENDOR_DESCRIPTOR_VERSION  0x0001U
#define UDL_VENDOR_KEY_MAX_PIXELS      0x0200U
#define UDL_VENDOR_DESCRIPTOR_LENGTH   27u
#define UDL_VENDOR_KEY_DEVICE_CAPS     0x0005U
#define UDL_VENDOR_KEY_FIRMWARE_REV    0x0400U
#define UDL_VENDOR_KEY_LINK_MODE       0x0401U
#define UDL_VENDOR_DEFAULT_PIXEL_LIMIT 2080000u

/* Vendor requests */
#define UDL_VENDOR_REQUEST_MEMORY_POKE           0x03U
#define UDL_VENDOR_REQUEST_MEMORY_PEEK           0x04U
#define UDL_VENDOR_REQUEST_SET_KEY               0x12U
#define UDL_VENDOR_REQUEST_WINDOWS_QUERY_05      0x05U
#define UDL_VENDOR_REQUEST_WINDOWS_QUERY_05_REPLY 0x00000f2bU
#define UDL_VENDOR_REQUEST_WINDOWS_PROBE         0x06U
#define UDL_VENDOR_REQUEST_WINDOWS_STATUS        0x13U
#define UDL_VENDOR_REQUEST_UNKNOWN_14            0x14U
#define UDL_VENDOR_REQUEST_EDID                  0x02U

/* Windows poll / status */
#define UDL_WINDOWS_POLL_STATUS_OK 0xf0005000U

/* EDID / control RAM */
#define UDL_EDID_INDEX                       0x00a1U
#define UDL_CONTROL_RAM_SIZE                 65536u
#define UDL_CONTROL_RAM_GRAPHICS_BASE        0xc300u
#define UDL_CONTROL_RAM_GRAPHICS_LIMIT       0xc400u
#define UDL_CONTROL_RAM_STARTUP_STATUS_ADDR  0xc484u
#define UDL_CONTROL_RAM_STARTUP_STATUS_VALUE 0x82u

/* Crypto */
#define UDL_CRYPT_STREAM_PERIOD 4095u
#define UDL_CRYPT_MAP_SIZE      4096u
#define UDL_CRYPT_POLY          0x180fu
#define UDL_CRYPT_LFSR12        0x0829u

/* Framebuffer backing geometry */
#define UDL_MIN_BACKING_16BPP_BYTES (16u * 1024u * 1024u)
#define UDL_MIN_BACKING_WIDTH       4096u

/* Packed gadget descriptor structs */
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
	uint8_t vendor_descriptor[UDL_VENDOR_DESCRIPTOR_LENGTH];
	struct gadget_endpoint_descriptor bulk_out_primary;
	struct gadget_endpoint_descriptor interrupt_in;
	struct gadget_endpoint_descriptor bulk_out_aux;
};

struct __attribute__((packed)) gadget_ss_config_block {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	uint8_t vendor_descriptor[UDL_VENDOR_DESCRIPTOR_LENGTH];
	struct gadget_endpoint_descriptor bulk_out_primary;
	struct gadget_ss_ep_comp_descriptor bulk_out_primary_companion;
	struct gadget_endpoint_descriptor interrupt_in;
	struct gadget_ss_ep_comp_descriptor interrupt_in_companion;
	struct gadget_endpoint_descriptor bulk_out_aux;
	struct gadget_ss_ep_comp_descriptor bulk_out_aux_companion;
};

struct __attribute__((packed)) gadget_bos_block {
	struct usb_bos_descriptor bos;
	struct usb_ss_cap_descriptor ss_cap;
};

/* Per-device options (fed by server config) */
struct options {
	const char *busid;
	const char *device_path;
	const char *edid_path;
	const char *dump_image_path;
	const char *manufacturer_string;
	const char *product_string;
	const char *serial_string;
	const char *monitor_name;
	uint16_t vendor_id;
	uint16_t product_id;
	enum usb_device_speed usb_speed;
	uint32_t decode_width;
	uint32_t decode_height;
	uint32_t refresh_hz;
	uint32_t window_scale;
	bool strict_native_mode;
	bool allow_30hz_fallback;
	bool decode_stream;
	bool show_window;
	bool verbose;
};

struct usbip_cadence_window {
	uint64_t window_start_ns;
	uint64_t control_submit_count;
	uint64_t control_out_bytes;
	uint64_t control_in_reply_bytes;
	uint64_t bulk_out_submit_count;
	uint64_t bulk_out_bytes;
	uint64_t bulk_out_zero_count;
	uint64_t bulk_out_le_256_count;
	uint64_t bulk_out_le_4k_count;
	uint64_t bulk_out_le_16k_count;
	uint64_t bulk_out_le_64k_count;
	uint64_t bulk_out_gt_64k_count;
	uint64_t interrupt_in_submit_count;
	uint64_t interrupt_in_reply_bytes;
	uint64_t unlink_count;
	uint64_t error_submit_count;
};

/* Per-device runtime state */
struct device_runtime {
	struct options opts;
	struct udl_runtime udl;
	struct usb_device_descriptor device_descriptor;
	struct usb_qualifier_descriptor qualifier_descriptor;
	struct gadget_bos_block bos_descriptor;
	struct usbip_usb_device usbip_device;
	struct usbip_usb_interface usbip_interface;
	uint8_t edid[256]; /* base block + optional CEA-861 extension */
	uint8_t vendor_descriptor[UDL_VENDOR_DESCRIPTOR_LENGTH];
	uint8_t control_ram[UDL_CONTROL_RAM_SIZE];
	size_t vendor_descriptor_len;
	uint8_t current_configuration;
	uint8_t last_key[16];
	uint64_t bulk_submit_count;
	uint8_t *bulk_recv_buf;       /* reusable receive buffer, avoids malloc per submit */
	size_t bulk_recv_buf_size;
	struct usbip_cadence_window usbip_cadence;
	pthread_mutex_t import_mutex;
	bool import_mutex_initialized;
	bool imported;
	int import_fd;    /* fd of active import session (-1 when none); protected by import_mutex */
	bool reconfiguring; /* EDID/dimension hot-reload in progress; blocks new imports.
			     * Set by the config watcher, cleared once the reinit (descriptors,
			     * and for dimension changes the udl_runtime + GL texture) is done.
			     * Protected by import_mutex. */
	bool usbip_cadence_logging_enabled;
	bool key_loaded;
	bool vendor_14_seen;
	bool huffman_table_loaded;
	bool windows_blankmode_on;
	bool windows_vidreg_unlock;
	bool windows_display_ready;
	bool first_e0_dumped;
	bool decrypt_enabled;
	uint16_t decrypt_offset;
	/* True when this slot is driven by a gadget transport (FFS/raw) rather than
	 * USB/IP.  Runtime-mutable: flipped under import_mutex by
	 * server_claim_gadget_slot()/server_release_gadget_slot() as a host
	 * connects/disconnects, not fixed for the slot's lifetime. */
	bool is_gadget_device;
	/* True only while a gadget transport has an ACTIVELY CONNECTED host on this
	 * slot (e.g. FFS FUNCTIONFS_ENABLE..DISABLE) — distinct from is_gadget_device,
	 * which just marks "this slot is reserved for the gadget, not USB/IP" and
	 * stays true for the slot's claim duration regardless of host presence.
	 * Set/cleared by the gadget transport itself (ffs_gadget.c's claim/release);
	 * the renderer's connection-status overlay reads this to know whether to
	 * show "no host" vs a live gadget feed.  The OLD /dev/udl_gadget kernel-
	 * module reader path (udl_runtime.gadget_fd) is a different, unrelated
	 * signal — do not conflate the two when checking "is a gadget host live".
	 */
	bool gadget_host_connected;
};

/* Control endpoint result */
enum control_action {
	CONTROL_ACTION_STALL,
	CONTROL_ACTION_ACK,
	CONTROL_ACTION_DATA_IN,
};

struct control_result {
	enum control_action action;
	int status;
	size_t actual_length;
	uint8_t data[EP0_BUFFER_SIZE];
};

/* Function declarations — public surface called from kms_server.c and the renderer */
void fill_usbip_device(struct device_runtime *runtime);
int run_import_session(struct device_runtime *runtime, int fd);
int handle_submit(struct device_runtime *runtime, int fd, struct usbip_header *request);

/*
 * Transport-independent EP0 control dispatch shared by the USB/IP server and the
 * Raw Gadget transport (raw_gadget.c).  Fills *result with the action/status/data
 * for the given setup packet (and OUT payload, if any).  Returns 0 on success
 * (result populated, possibly STALL) or -1 on a fatal handler error.
 */
int udl_device_handle_control(struct device_runtime *runtime,
			      const struct usb_ctrlrequest *setup,
			      const uint8_t *payload,
			      size_t payload_length,
			      struct control_result *result);

/* Logging helpers called from kms_udl_runtime.c via forward declaration */
void log_hex_window(const char *label,
		    const uint8_t *data,
		    size_t length,
		    size_t offset,
		    size_t window_length);
void log_udl_writereg_commands(const uint8_t *data, size_t length);

/* EDID / descriptor builders called from kms_server.c */
void udl_control_ram_seed(struct device_runtime *runtime);
void build_default_edid(uint8_t edid[256],
			const char *monitor_name,
			uint32_t width,
			uint32_t height,
			uint32_t refresh_hz,
			bool strict_native_mode,
			bool allow_30hz_fallback);
int load_edid_file(const char *path, uint8_t edid[256]);
void build_vendor_descriptor(struct device_runtime *runtime);
/* Fill a 0x5f DisplayLink vendor descriptor for the given resolution into buf
 * (UDL_VENDOR_DESCRIPTOR_LENGTH bytes).  Used by the FFS transport to embed a
 * descriptor in its functionfs blob before any slot is claimed. */
void udl_fill_vendor_descriptor(uint8_t buf[UDL_VENDOR_DESCRIPTOR_LENGTH],
				uint32_t width, uint32_t height);
void build_device_descriptor(struct device_runtime *runtime);
void build_device_qualifier(struct device_runtime *runtime);
void build_bos_descriptor(struct device_runtime *runtime);

/* udl_resync_reason_name used in kms_udl_runtime.c */
const char *udl_resync_reason_name(enum udl_transport_resync_reason reason);
