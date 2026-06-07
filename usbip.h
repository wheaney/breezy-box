#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <linux/usb/ch9.h>
#include <sys/types.h>

#define SYSFS_PATH_MAX 256u
#define SYSFS_BUS_ID_SIZE 32u

#define OP_REQUEST (0x80u << 8)
#define OP_REPLY   (0x00u << 8)

#define OP_UNSPEC       0x00u
#define OP_DEVINFO      0x02u
#define OP_IMPORT       0x03u
#define OP_CRYPKEY      0x04u
#define OP_DEVLIST      0x05u
#define OP_EXPORT       0x06u
#define OP_UNEXPORT     0x07u

#define OP_REQ_DEVINFO  (OP_REQUEST | OP_DEVINFO)
#define OP_REP_DEVINFO  (OP_REPLY | OP_DEVINFO)
#define OP_REQ_IMPORT   (OP_REQUEST | OP_IMPORT)
#define OP_REP_IMPORT   (OP_REPLY | OP_IMPORT)
#define OP_REQ_CRYPKEY  (OP_REQUEST | OP_CRYPKEY)
#define OP_REP_CRYPKEY  (OP_REPLY | OP_CRYPKEY)
#define OP_REQ_DEVLIST  (OP_REQUEST | OP_DEVLIST)
#define OP_REP_DEVLIST  (OP_REPLY | OP_DEVLIST)

#define ST_OK 0x00u
#define ST_NA 0x01u
#define ST_DEV_BUSY 0x02u
#define ST_DEV_ERR 0x03u
#define ST_NODEV 0x04u
#define ST_ERROR 0x05u

#define USBIP_CMD_SUBMIT 0x0001u
#define USBIP_CMD_UNLINK 0x0002u
#define USBIP_RET_SUBMIT 0x0003u
#define USBIP_RET_UNLINK 0x0004u

#define USBIP_DIR_OUT 0x00u
#define USBIP_DIR_IN 0x01u

/*
 * DisplayLink traffic over USB/IP is dominated by sustained medium-large OUT
 * payloads (commonly tens of KiB per submit). Give TCP more room to absorb
 * those bursts before the decode thread pushes back on the sender.
 */
#define USBIP_STREAM_RCVBUF_SIZE (2u * 1024u * 1024u)
#define USBIP_STREAM_SNDBUF_SIZE (2u * 1024u * 1024u)

/* Version used on the wire */
#define USBIP_VERSION 0x0111u

struct __attribute__((packed)) usbip_usb_interface {
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t padding;
};

struct __attribute__((packed)) usbip_usb_device {
	char path[SYSFS_PATH_MAX];
	char busid[SYSFS_BUS_ID_SIZE];
	uint32_t busnum;
	uint32_t devnum;
	uint32_t speed;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bConfigurationValue;
	uint8_t bNumConfigurations;
	uint8_t bNumInterfaces;
};

struct __attribute__((packed)) op_common {
	uint16_t version;
	uint16_t code;
	uint32_t status;
};

struct __attribute__((packed)) op_import_request {
	char busid[SYSFS_BUS_ID_SIZE];
};

struct __attribute__((packed)) op_devlist_reply {
	uint32_t ndev;
};

struct __attribute__((packed)) usbip_header_basic {
	uint32_t command;
	uint32_t seqnum;
	uint32_t devid;
	uint32_t direction;
	uint32_t ep;
};

struct __attribute__((packed)) usbip_header_cmd_submit {
	uint32_t transfer_flags;
	int32_t transfer_buffer_length;
	int32_t start_frame;
	int32_t number_of_packets;
	int32_t interval;
	unsigned char setup[8];
};

struct __attribute__((packed)) usbip_header_ret_submit {
	int32_t status;
	int32_t actual_length;
	int32_t start_frame;
	int32_t number_of_packets;
	int32_t error_count;
};

struct __attribute__((packed)) usbip_header_cmd_unlink {
	uint32_t seqnum;
};

struct __attribute__((packed)) usbip_header_ret_unlink {
	int32_t status;
};

struct __attribute__((packed)) usbip_header {
	struct usbip_header_basic base;
	union {
		struct usbip_header_cmd_submit cmd_submit;
		struct usbip_header_ret_submit ret_submit;
		struct usbip_header_cmd_unlink cmd_unlink;
		struct usbip_header_ret_unlink ret_unlink;
	} u;
};

/* Function declarations */
bool usb_speed_is_super(enum usb_device_speed speed);
const char *usb_speed_name(enum usb_device_speed speed);

int set_socket_options(int fd);
ssize_t recv_all(int fd, void *buffer, size_t length);
ssize_t send_all(int fd, const void *buffer, size_t length);
ssize_t send_iov_all(int fd, const struct iovec *iov, int iovcnt);

/*
 * RET_SUBMIT ack batching.  Every USB/IP submit needs a small reply, so a busy
 * DisplayLink stream produces ~one tiny TCP send per bulk OUT, each of which
 * costs a gadget TX completion + NET_TX softirq on the single CPU that services
 * the dwc3 interrupt.  Holding the socket corked while more requests are already
 * buffered coalesces those replies into far fewer segments; we flush before
 * blocking on input, and at a bounded cap so the host's URB pool never starves.
 */
#define USBIP_ACK_BATCH_MAX 16u
bool usbip_ack_batch_enabled(void);
void usbip_set_cork(int fd, int on);
void usbip_flush_cork(int fd);
bool usbip_input_pending(int fd);

uint16_t pack_u16(int pack, uint16_t value);
uint32_t pack_u32(int pack, uint32_t value);
void pack_usbip_usb_device(int pack, struct usbip_usb_device *device);
void pack_op_common(int pack, struct op_common *op);

int send_op_common(int fd, uint16_t code, uint32_t status);
int send_op_common_with_payload(int fd,
				uint16_t code,
				uint32_t status,
				const void *payload,
				size_t payload_length);
int recv_op_common(int fd, struct op_common *op);

void usbip_header_correct_endian(struct usbip_header *header, int pack);
