/*
 * FunctionFS transport — userspace USB DisplayLink device over functionfs.
 *
 * Mirrors the structure of raw_gadget.c (a thin transport that funnels EP0
 * through udl_device_handle_control and bulk OUT into udl_runtime_enqueue_bulk)
 * but speaks the functionfs ep0/epN protocol instead of the Raw Gadget ioctls,
 * so no CONFIG_USB_RAW_GADGET and no per-process UDC ownership are needed.  The
 * descriptor/strings blob and control handling follow the proven ZeroKVM C#
 * DlFunctionFs implementation.
 */
#define _POSIX_C_SOURCE 200809L

#include "ffs_gadget.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <sys/stat.h>

#include <linux/types.h>
#include <linux/usb/ch9.h>
#include <linux/usb/functionfs.h>

#include "udl_device.h"
#include "udl_runtime.h"
#include "server.h"

#define FFS_BULK_OUT_ADDRESS  UDL_PRIMARY_BULK_OUT_EP   /* 0x01 */
#define FFS_INTERRUPT_IN_ADDRESS UDL_INTERRUPT_IN_EP     /* 0x82 */
#define FFS_AUX_BULK_OUT_ADDRESS UDL_AUX_BULK_OUT_EP     /* 0x0a */
#define FFS_EP0_BUFFER_SIZE   512u
#define FFS_BULK_BUFFER_SIZE  (64u * 1024u)
#define FFS_BULK_MAXPACKET_HS 512u
#define FFS_VENDOR_DESC_LEN    UDL_VENDOR_DESCRIPTOR_LENGTH

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define cpu_to_le16(v) ((uint16_t)(v))
#define cpu_to_le32(v) ((uint32_t)(v))
#else
#define cpu_to_le16(v) __builtin_bswap16((uint16_t)(v))
#define cpu_to_le32(v) __builtin_bswap32((uint32_t)(v))
#endif

/* ---------------------------------------------------------------- */
/* Descriptor + strings blobs                                       */
/* ---------------------------------------------------------------- */

/*
 * One speed's descriptor set: interface + endpoint trio (bulk OUT 0x01,
 * interrupt IN 0x82, bulk OUT 0x0a).  The 0x5f vendor descriptor is never
 * embedded here — it is served on-demand only, via the standard
 * GET_DESCRIPTOR(0x5f) SETUP interception below (see write_vendor_descriptor_reply).
 * Embedding it inline in the config descriptor blob was tried and found to
 * break host-side enumeration ("Unrecognized vendor firmware descriptor")
 * even when the kernel's functionfs layer accepted the blob without EINVAL —
 * matches the proven raw_gadget.c and ZeroKVM oracles, neither of which embeds it.
 */
struct __attribute__((packed)) ffs_function_descs {
	struct usb_interface_descriptor intf;
	struct usb_endpoint_descriptor_no_audio bulk_out_primary;
	struct usb_endpoint_descriptor_no_audio interrupt_in;
	struct usb_endpoint_descriptor_no_audio bulk_out_aux;
};

static void fill_ffs_vendor_descriptor(uint8_t desc[FFS_VENDOR_DESC_LEN],
				      uint32_t width,
				      uint32_t height);

static void fill_interface(struct usb_interface_descriptor *intf)
{
	memset(intf, 0, sizeof(*intf));
	intf->bLength = sizeof(*intf);
	intf->bDescriptorType = USB_DT_INTERFACE;
	intf->bInterfaceNumber = 0;
	intf->bAlternateSetting = 0;
	intf->bNumEndpoints = 3;
	intf->bInterfaceClass = USB_CLASS_VENDOR_SPEC;
	intf->bInterfaceSubClass = 0;
	intf->bInterfaceProtocol = 0;
	intf->iInterface = 1;
}

static void fill_bulk_endpoint(struct usb_endpoint_descriptor_no_audio *ep,
			       uint8_t address,
			       uint16_t bulk_maxpacket)
{
	memset(ep, 0, sizeof(*ep));
	ep->bLength = sizeof(*ep);
	ep->bDescriptorType = USB_DT_ENDPOINT;
	ep->bEndpointAddress = address;
	ep->bmAttributes = USB_ENDPOINT_XFER_BULK;
	ep->wMaxPacketSize = cpu_to_le16(bulk_maxpacket);
	ep->bInterval = 0;
}

static void fill_interrupt_in(struct usb_endpoint_descriptor_no_audio *ep)
{
	memset(ep, 0, sizeof(*ep));
	ep->bLength = sizeof(*ep);
	ep->bDescriptorType = USB_DT_ENDPOINT;
	ep->bEndpointAddress = FFS_INTERRUPT_IN_ADDRESS;
	ep->bmAttributes = USB_ENDPOINT_XFER_INT;
	ep->wMaxPacketSize = cpu_to_le16(UDL_INTERRUPT_PACKET_SIZE);
	ep->bInterval = UDL_INTERRUPT_INTERVAL;
}

/*
 * Assemble and write the descriptor blob at full/high speed. Full-speed bulk
 * endpoints are capped at 64 bytes; high-speed uses 512.
 */
static int ffs_write_descriptors(struct ffs_gadget *gadget)
{
	uint8_t blob[sizeof(struct usb_functionfs_descs_head_v2) +
		     2u * sizeof(__le32) +
		     2u * sizeof(struct ffs_function_descs)];
	struct usb_functionfs_descs_head_v2 *head = (void *)blob;
	uint8_t *cursor = blob + sizeof(*head);
	const uint32_t per_speed_count = 4u;
	size_t total;
	ssize_t n;

	/* fs_count, hs_count */
	*(__le32 *)cursor = cpu_to_le32(per_speed_count); cursor += sizeof(__le32);
	*(__le32 *)cursor = cpu_to_le32(per_speed_count); cursor += sizeof(__le32);

	for (int speed = 0; speed < 2; speed++) {
		const uint16_t mps = (speed == 0) ? 64u : FFS_BULK_MAXPACKET_HS;
		struct ffs_function_descs d;

		fill_interface(&d.intf);
		fill_bulk_endpoint(&d.bulk_out_primary, FFS_BULK_OUT_ADDRESS, mps);
		fill_interrupt_in(&d.interrupt_in);
		fill_bulk_endpoint(&d.bulk_out_aux, FFS_AUX_BULK_OUT_ADDRESS, mps);
		memcpy(cursor, &d, sizeof(d));
		cursor += sizeof(d);
	}

	total = (size_t)(cursor - blob);
	head->magic  = cpu_to_le32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2);
	head->length = cpu_to_le32((uint32_t)total);
	head->flags  = cpu_to_le32(FUNCTIONFS_HAS_FS_DESC |
				   FUNCTIONFS_HAS_HS_DESC |
				   FUNCTIONFS_ALL_CTRL_RECIP |
				   FUNCTIONFS_CONFIG0_SETUP);

	n = write(gadget->ep0_fd, blob, total);
	if (n != (ssize_t)total) {
		fprintf(stderr, "ffs-gadget: write descriptors: %s\n",
			n < 0 ? strerror(errno) : "short write");
		return -1;
	}
	(void)dprintf(STDERR_FILENO, "ffs-gadget: wrote descriptors\n");
	syslog(LOG_INFO, "ffs-gadget: wrote descriptors");
	return 0;
}

static int ffs_write_strings(struct ffs_gadget *gadget)
{
	/* One language (0x0409), one string: the interface name (iInterface=1). */
	static const char kInterfaceName[] = "DisplayLink";
	uint8_t blob[sizeof(struct usb_functionfs_strings_head) +
		     sizeof(__le16) + sizeof(kInterfaceName)];
	struct usb_functionfs_strings_head *head = (void *)blob;
	uint8_t *cursor = blob + sizeof(*head);
	ssize_t n;

	head->magic     = cpu_to_le32(FUNCTIONFS_STRINGS_MAGIC);
	head->length    = cpu_to_le32(sizeof(blob));
	head->str_count = cpu_to_le32(1);
	head->lang_count = cpu_to_le32(1);

	*(__le16 *)cursor = cpu_to_le16(0x0409);
	cursor += sizeof(__le16);
	memcpy(cursor, kInterfaceName, sizeof(kInterfaceName)); /* includes NUL */

	n = write(gadget->ep0_fd, blob, sizeof(blob));
	if (n != (ssize_t)sizeof(blob)) {
		fprintf(stderr, "ffs-gadget: write strings: %s\n",
			n < 0 ? strerror(errno) : "short write");
		return -1;
	}
	return 0;
}

/* ---------------------------------------------------------------- */
/* Bulk OUT drain                                                   */
/* ---------------------------------------------------------------- */

static void open_bulk_endpoint(struct ffs_gadget *gadget);
static void close_bulk_endpoint(struct ffs_gadget *gadget);

static void fill_ffs_vendor_descriptor(uint8_t desc[FFS_VENDOR_DESC_LEN],
				      uint32_t width,
				      uint32_t height)
{
	/* Keep FFS and USB/IP byte-identical for 0x5f parsing behavior. */
	udl_fill_vendor_descriptor(desc, width, height);
}

static int write_vendor_descriptor_reply(struct ffs_gadget *gadget,
					 const struct device_runtime *dev,
					 const struct usb_ctrlrequest *setup)
{
	uint8_t desc[FFS_VENDOR_DESC_LEN];
	uint8_t reply[FFS_VENDOR_DESC_LEN];
	uint32_t width = DEFAULT_DISPLAY_WIDTH;
	uint32_t height = DEFAULT_DISPLAY_HEIGHT;
	uint16_t len;

	if (!gadget || !setup)
		return -1;

	if (dev) {
		if (dev->opts.decode_width > 0u)
			width = dev->opts.decode_width;
		if (dev->opts.decode_height > 0u)
			height = dev->opts.decode_height;
	} else if (gadget->server) {
		const uint32_t published_width = atomic_load_explicit(
			&gadget->server->drm_mode_width, memory_order_relaxed);
		const uint32_t published_height = atomic_load_explicit(
			&gadget->server->drm_mode_height, memory_order_relaxed);

		if (published_width > 0u)
			width = published_width;
		if (published_height > 0u)
			height = published_height;
	}

	/*
	 * Serve the same 0x5f geometry descriptor shape as ZeroKVM's working FFS
	 * path: MaxArea + MaxWidth + MaxHeight TLVs.
	 */
	fill_ffs_vendor_descriptor(desc, width, height);
	len = (uint16_t)setup->wLength;
	if (len > sizeof(reply))
		len = sizeof(reply);

	memcpy(reply, desc, len);
	/*
	 * Linux udl_parse_vendor_descriptor() validates the header against the
	 * transfer length returned by usb_get_descriptor():
	 *   desc[0] == len, desc[4] == len - 2.
	 * Keep these two bytes self-consistent with whatever length we actually
	 * return, including short replies.
	 */
	if (len >= 1u)
		reply[0] = (uint8_t)len;
	if (len >= 5u)
		reply[4] = (uint8_t)(len - 2u);

	{
		size_t written = 0u;
		while (written < len) {
			ssize_t n = write(gadget->ep0_fd, reply + written, len - written);
			if (n < 0) {
				const int saved_errno = errno;
				perror("ffs-gadget: ep0 write (vendor descriptor)");
				syslog(LOG_ERR,
				       "ffs-gadget: ep0 write failed for 0x5f descriptor after %u/%u bytes: %s",
				       (unsigned int)written,
				       (unsigned int)len,
				       strerror(saved_errno));
				return -1;
			}
			if (n == 0) {
				fprintf(stderr,
					"ffs-gadget: ep0 write (vendor descriptor) short write at %u/%u\n",
					(unsigned int)written,
					(unsigned int)len);
				syslog(LOG_ERR,
				       "ffs-gadget: ep0 write short write for 0x5f descriptor at %u/%u bytes",
				       (unsigned int)written,
				       (unsigned int)len);
				return -1;
			}
			written += (size_t)n;
		}
	}

	fprintf(stderr,
		"ffs-gadget: GET_DESCRIPTOR 0x5f rt=0x%02x idx=0x%04x req_len=%u sent=%u hdr=[%u,0x%02x,0x%02x,0x%02x,%u]%s\n",
		(unsigned int)setup->bRequestType,
		(unsigned int)(uint16_t)setup->wIndex,
		(unsigned int)(uint16_t)setup->wLength,
		(unsigned int)len,
		(unsigned int)(len >= 1u ? reply[0] : 0u),
		(unsigned int)(len >= 2u ? reply[1] : 0u),
		(unsigned int)(len >= 3u ? reply[2] : 0u),
		(unsigned int)(len >= 4u ? reply[3] : 0u),
		(unsigned int)(len >= 5u ? reply[4] : 0u),
		dev ? "" : " before ENABLE");
	syslog(LOG_INFO,
	       "ffs-gadget: GET_DESCRIPTOR 0x5f rt=0x%02x idx=0x%04x req_len=%u sent=%u hdr=[%u,0x%02x,0x%02x,0x%02x,%u]%s",
	       (unsigned int)setup->bRequestType,
	       (unsigned int)(uint16_t)setup->wIndex,
	       (unsigned int)(uint16_t)setup->wLength,
	       (unsigned int)len,
	       (unsigned int)(len >= 1u ? reply[0] : 0u),
	       (unsigned int)(len >= 2u ? reply[1] : 0u),
	       (unsigned int)(len >= 3u ? reply[2] : 0u),
	       (unsigned int)(len >= 4u ? reply[3] : 0u),
	       (unsigned int)(len >= 5u ? reply[4] : 0u),
	       dev ? "" : " before ENABLE");

	return 0;
}

static bool ffs_stop_requested(const struct ffs_gadget *gadget)
{
	return gadget && atomic_load(&gadget->stop_requested);
}

static int ffs_ep_path(const struct ffs_gadget *gadget, const char *ep,
		       char *out, size_t cap)
{
	int n = snprintf(out, cap, "%s/%s", gadget->mount_path, ep);

	return (n > 0 && (size_t)n < cap) ? 0 : -1;
}

static void *bulk_thread_main(void *arg)
{
	struct ffs_gadget *gadget = arg;
	uint8_t *buf = malloc(FFS_BULK_BUFFER_SIZE);

	if (!buf) {
		fprintf(stderr, "ffs-gadget: bulk buffer alloc failed\n");
		return NULL;
	}

	if (gadget->verbose)
		fprintf(stderr, "ffs-gadget: bulk OUT drain thread started\n");

	while (!ffs_stop_requested(gadget) &&
	       !atomic_load(&gadget->bulk_thread_stop)) {
		ssize_t n;
		struct pollfd pfd;
		struct device_runtime *dev;

		/* Feed only while a host is connected AND a slot is claimed.  The
		 * bulk thread solely owns ep1_fd: open it once enabled+claimed, close
		 * it otherwise.  device is published by the event thread on ENABLE. */
		dev = atomic_load_explicit(&gadget->device, memory_order_acquire);
		if (!atomic_load_explicit(&gadget->enabled, memory_order_acquire) || !dev) {
			close_bulk_endpoint(gadget);
			struct timespec ts = { 0, 20000000L }; /* 20 ms */
			nanosleep(&ts, NULL);
			continue;
		}
		if (gadget->ep1_fd < 0) {
			open_bulk_endpoint(gadget);
			if (gadget->ep1_fd < 0) {
				struct timespec ts = { 0, 20000000L };
				nanosleep(&ts, NULL);
				continue;
			}
		}

		pfd.fd = gadget->ep1_fd;
		pfd.events = POLLIN;
		pfd.revents = 0;
		if (poll(&pfd, 1, 200) <= 0)
			continue;
		if (!(pfd.revents & POLLIN))
			continue;

		n = read(gadget->ep1_fd, buf, FFS_BULK_BUFFER_SIZE);
		if (n > 0) {
			(void)udl_runtime_enqueue_bulk(&dev->udl, buf, (size_t)n);
		} else if (n < 0) {
			if (errno == EINTR || errno == EAGAIN)
				continue;
			/* ESHUTDOWN/EBADF when the host disables the config or we
			 * close the fd; loop will exit on stop or re-enable. */
			if (errno == ESHUTDOWN) {
				struct timespec ts = { 0, 20000000L };
				nanosleep(&ts, NULL);
				continue;
			}
			if (!ffs_stop_requested(gadget) && gadget->verbose)
				perror("ffs-gadget: ep1 read");
			break;
		}
	}

	free(buf);
	if (gadget->verbose)
		fprintf(stderr, "ffs-gadget: bulk OUT drain thread stopped\n");
	return NULL;
}

static void open_bulk_endpoint(struct ffs_gadget *gadget)
{
	char path[320];

	if (gadget->ep1_fd >= 0)
		return;
	if (ffs_ep_path(gadget, "ep1", path, sizeof(path)) != 0)
		return;
	gadget->ep1_fd = open(path, O_RDONLY | O_CLOEXEC);
	if (gadget->ep1_fd < 0) {
		if (gadget->verbose)
			fprintf(stderr, "ffs-gadget: open(%s): %s\n",
				path, strerror(errno));
		return;
	}
	if (gadget->verbose)
		fprintf(stderr, "ffs-gadget: bulk OUT endpoint opened\n");
}

static void close_bulk_endpoint(struct ffs_gadget *gadget)
{
	if (gadget->ep1_fd >= 0) {
		close(gadget->ep1_fd);
		gadget->ep1_fd = -1;
	}
}

/* ---------------------------------------------------------------- */
/* Slot claim / release (on host connect / disconnect)              */
/* ---------------------------------------------------------------- */

/* Claim a display slot for this host connection, if not already holding one.
 * Runs on the event thread (FUNCTIONFS_ENABLE).  Publishes the slot's
 * device_runtime for the bulk thread with release ordering. */
static void ffs_claim_slot(struct ffs_gadget *gadget)
{
	size_t slot = SIZE_MAX;
	bool created = false;

	if (atomic_load_explicit(&gadget->device, memory_order_relaxed))
		return; /* already holding a slot */

	if (server_claim_gadget_slot(gadget->server, &slot, &created) == SIZE_MAX) {
		fprintf(stderr, "ffs-gadget: no display slot available for host\n");
		return;
	}
	gadget->claimed_slot = slot;
	gadget->server->devices[slot].gadget_host_connected = true;
	atomic_store_explicit(&gadget->device, &gadget->server->devices[slot],
			      memory_order_release);
	fprintf(stderr,
		"ffs-gadget: host connected → claimed slot %zu (%s) %ux%u@%u\n",
		slot, created ? "new fallback" : "existing",
		gadget->server->devices[slot].opts.decode_width,
		gadget->server->devices[slot].opts.decode_height,
		gadget->server->devices[slot].opts.refresh_hz);
}

/* Release the claimed slot back to the USB/IP pool (host disconnect).  Clears
 * the published device first so the bulk thread stops feeding it, then releases
 * the slot.  Runs on the event thread. */
static void ffs_release_slot(struct ffs_gadget *gadget)
{
	struct device_runtime *dev =
		atomic_exchange_explicit(&gadget->device, NULL, memory_order_acq_rel);

	if (!dev)
		return;
	dev->gadget_host_connected = false;
	server_release_gadget_slot(gadget->server, gadget->claimed_slot);
	if (gadget->verbose)
		fprintf(stderr, "ffs-gadget: host disconnected → released slot %zu\n",
			gadget->claimed_slot);
}

/* ---------------------------------------------------------------- */
/* EP0 control                                                      */
/* ---------------------------------------------------------------- */

/*
 * Handle one SETUP event.  The data stage (if any) is transferred on ep0 by
 * read()/write() following the SETUP, per the functionfs contract.  Control
 * logic itself is delegated to udl_device_handle_control so the FFS and USB/IP
 * transports stay byte-identical.
 */
static int handle_setup(struct ffs_gadget *gadget,
			const struct usb_ctrlrequest *setup)
{
	struct device_runtime *dev =
		atomic_load_explicit(&gadget->device, memory_order_acquire);
	struct control_result result;
	const uint16_t wValue = (uint16_t)setup->wValue;
	const uint16_t wIndex = (uint16_t)setup->wIndex;
	const uint16_t wLength = (uint16_t)setup->wLength;
	const bool host_to_device = (setup->bRequestType & USB_DIR_IN) == 0u;
	uint8_t payload[FFS_EP0_BUFFER_SIZE];
	const uint8_t *payload_ptr = NULL;
	size_t payload_length = 0u;

	/*
	 * Always trace SETUP packets while chasing host-side descriptor parsing.
	 * This confirms whether GET_DESCRIPTOR(0x5f) reaches FunctionFS userspace
	 * at all, independent of service stdout/stderr routing.
	 */
	fprintf(stderr,
		"ffs-gadget: SETUP rt=0x%02x req=0x%02x val=0x%04x idx=0x%04x len=%u dev=%s\n",
		(unsigned int)setup->bRequestType,
		(unsigned int)setup->bRequest,
		(unsigned int)wValue,
		(unsigned int)wIndex,
		(unsigned int)wLength,
		dev ? "yes" : "no");
	syslog(LOG_INFO,
	       "ffs-gadget: SETUP rt=0x%02x req=0x%02x val=0x%04x idx=0x%04x len=%u dev=%s",
	       (unsigned int)setup->bRequestType,
	       (unsigned int)setup->bRequest,
	       (unsigned int)wValue,
	       (unsigned int)wIndex,
	       (unsigned int)wLength,
	       dev ? "yes" : "no");

	/*
	 * functionfs EP0 completion convention (drivers/usb/gadget/function/f_fs.c,
	 * ffs_ep0_{read,write}):
	 *   - OUT setup: read(ep0) fetches the data stage AND completes the status
	 *     stage (ACK); even a zero-length read ACKs a no-data OUT request.  A
	 *     write(ep0) on an OUT setup is turned into a protocol STALL.
	 *   - IN setup: write(ep0) sends the data stage / ACKs (zero length for a
	 *     no-data request); a read(ep0) on an IN setup is turned into a STALL.
	 *
	 * So we read OUT setups up front — which both hands the payload to the
	 * handler and ACKs the transfer.  Because that read commits us to ACK, OUT
	 * requests cannot be stalled here; that is fine for DisplayLink, whose OUT
	 * requests (SET_KEY, RAM poke, the 0x14 init pulse) are always accepted.
	 */
	if (host_to_device) {
		size_t want = wLength > sizeof(payload) ? sizeof(payload) : wLength;
		ssize_t n = read(gadget->ep0_fd, payload, want);

		if (n < 0) {
			perror("ffs-gadget: ep0 read (out data/status)");
			return -1;
		}
		payload_ptr = payload;
		payload_length = (size_t)n;
	}

	if (!host_to_device &&
	    (setup->bRequestType & (USB_DIR_IN | USB_TYPE_MASK)) ==
		(USB_DIR_IN | USB_TYPE_STANDARD) &&
	    setup->bRequest == USB_REQ_GET_DESCRIPTOR &&
	    (uint8_t)(wValue >> 8) == UDL_VENDOR_DESCRIPTOR_TYPE) {
		fprintf(stderr,
			"ffs-gadget: intercepting GET_DESCRIPTOR(0x5f) val=0x%04x idx=0x%04x len=%u\n",
			(unsigned int)wValue,
			(unsigned int)wIndex,
			(unsigned int)wLength);
		syslog(LOG_INFO,
		       "ffs-gadget: intercepting GET_DESCRIPTOR(0x5f) val=0x%04x idx=0x%04x len=%u",
		       (unsigned int)wValue,
		       (unsigned int)wIndex,
		       (unsigned int)wLength);
		return write_vendor_descriptor_reply(gadget, dev, setup);
	}

	/*
	 * No slot claimed yet (a control request arrived before FUNCTIONFS_ENABLE).
	 * We can't delegate without a device_runtime; complete the transfer minimally
	 * (ACK, never stall) so enumeration isn't broken.  The vendor handshake
	 * (peek/poke/key/EDID) only starts after ENABLE, by which point a slot is
	 * claimed, so this path handles only early standard probes.
	 */
	if (!dev) {
		if (!host_to_device) {
			/* IN: zero-length data acks */
			(void)write(gadget->ep0_fd, payload, 0);
			if (gadget->verbose) {
				fprintf(stderr,
					"ffs-gadget: ACKed pre-claim IN request rt=0x%02x req=0x%02x val=0x%04x idx=0x%04x len=%u with no data\n",
					(unsigned int)setup->bRequestType,
					(unsigned int)setup->bRequest,
					(unsigned int)(uint16_t)setup->wValue,
					(unsigned int)(uint16_t)setup->wIndex,
					(unsigned int)wLength);
			}
		}
		/* OUT was already acked by the read above. */
		return 0;
	}

	if (udl_device_handle_control(dev, setup,
				      payload_ptr, payload_length, &result) != 0)
		return -1;

	/* OUT setups were already completed (ACKed) by the read above. */
	if (host_to_device)
		return 0;

	/* IN setup: complete in the IN direction, or stall via a wrong-way read. */
	switch (result.action) {
	case CONTROL_ACTION_DATA_IN: {
		size_t len = result.actual_length > wLength ? wLength : result.actual_length;

		if (write(gadget->ep0_fd, result.data, len) < 0) {
			perror("ffs-gadget: ep0 write (data in)");
			return -1;
		}
		return 0;
	}
	case CONTROL_ACTION_ACK:
		/* No-data IN request: a zero-length data stage ACKs it. */
		if (write(gadget->ep0_fd, result.data, 0) < 0 && gadget->verbose)
			perror("ffs-gadget: ep0 ack write");
		return 0;
	case CONTROL_ACTION_STALL:
	default:
		(void)read(gadget->ep0_fd, payload, 0); /* read on IN setup → STALL */
		return 0;
	}
}

static int handle_event(struct ffs_gadget *gadget,
			const struct usb_functionfs_event *ev)
{
	switch (ev->type) {
	case FUNCTIONFS_BIND:
		if (gadget->verbose)
			fprintf(stderr, "ffs-gadget: BIND\n");
		break;
	case FUNCTIONFS_UNBIND:
		if (gadget->verbose)
			fprintf(stderr, "ffs-gadget: UNBIND\n");
		atomic_store_explicit(&gadget->enabled, false, memory_order_release);
		ffs_release_slot(gadget);
		break;
	case FUNCTIONFS_ENABLE:
		if (gadget->verbose)
			fprintf(stderr, "ffs-gadget: ENABLE\n");
		/* Host configured the device: claim a slot, then enable feeding.
		 * Publish the slot (release) before enabled so the bulk thread sees a
		 * non-NULL device once it observes enabled. */
		ffs_claim_slot(gadget);
		atomic_store_explicit(&gadget->enabled, true, memory_order_release);
		break;
	case FUNCTIONFS_DISABLE:
		if (gadget->verbose)
			fprintf(stderr, "ffs-gadget: DISABLE\n");
		atomic_store_explicit(&gadget->enabled, false, memory_order_release);
		ffs_release_slot(gadget);
		break;
	case FUNCTIONFS_SETUP:
		return handle_setup(gadget, &ev->u.setup);
	case FUNCTIONFS_SUSPEND:
	case FUNCTIONFS_RESUME:
		break;
	default:
		break;
	}
	return 0;
}

static void *event_thread_main(void *arg)
{
	struct ffs_gadget *gadget = arg;
	struct usb_functionfs_event events[16];

	while (!ffs_stop_requested(gadget)) {
		struct pollfd pfd = { .fd = gadget->ep0_fd, .events = POLLIN };
		ssize_t n;
		size_t count;
		size_t i;

		if (poll(&pfd, 1, 200) <= 0)
			continue;
		if (!(pfd.revents & POLLIN))
			continue;

		n = read(gadget->ep0_fd, events, sizeof(events));
		if (n < 0) {
			if (errno == EINTR || errno == EAGAIN)
				continue;
			if (!ffs_stop_requested(gadget))
				perror("ffs-gadget: ep0 read (events)");
			break;
		}
		count = (size_t)n / sizeof(events[0]);
		for (i = 0; i < count; i++) {
			if (handle_event(gadget, &events[i]) != 0)
				goto done;
		}
	}
done:
	return NULL;
}

/* ---------------------------------------------------------------- */
/* Public API                                                       */
/* ---------------------------------------------------------------- */

bool ffs_gadget_available(const char *mount_path)
{
	const char *base = mount_path ? mount_path : FFS_GADGET_DEFAULT_MOUNT;
	char path[320];
	int n = snprintf(path, sizeof(path), "%s/ep0", base);

	if (n <= 0 || (size_t)n >= sizeof(path))
		return false;
	return access(path, R_OK | W_OK) == 0;
}

int ffs_gadget_start(struct ffs_gadget *gadget,
		     struct server_runtime *server,
		     const char *mount_path,
		     bool verbose)
{
	char ep0_path[320];
	int desc_rc;
	int str_rc;

	if (!gadget || !server)
		return -1;

	memset(gadget, 0, sizeof(*gadget));
	gadget->server = server;
	gadget->ep0_fd = -1;
	gadget->ep1_fd = -1;
	gadget->verbose = verbose;
	gadget->claimed_slot = 0u;
	atomic_init(&gadget->stop_requested, false);
	atomic_init(&gadget->bulk_thread_stop, false);
	atomic_init(&gadget->enabled, false);
	atomic_init(&gadget->device, NULL);
	snprintf(gadget->mount_path, sizeof(gadget->mount_path), "%s",
		 mount_path ? mount_path : FFS_GADGET_DEFAULT_MOUNT);

	if (ffs_ep_path(gadget, "ep0", ep0_path, sizeof(ep0_path)) != 0)
		return -1;
	gadget->ep0_fd = open(ep0_path, O_RDWR | O_CLOEXEC);
	if (gadget->ep0_fd < 0) {
		fprintf(stderr, "ffs-gadget: open(%s): %s\n",
			ep0_path, strerror(errno));
		fprintf(stderr,
			"ffs-gadget: is functionfs mounted? (breezy-gadget.service)\n");
		return -1;
	}

	/* Writing descriptors then strings makes the data endpoints appear; the
	 * root setup script polls for ep1 and then binds the UDC. */
	(void)dprintf(STDERR_FILENO,
		      "ffs-gadget: wrote descriptors preflight (begin)\n");
	syslog(LOG_INFO,
	       "ffs-gadget: wrote descriptors preflight (begin)");
	desc_rc = ffs_write_descriptors(gadget);
	str_rc = (desc_rc == 0) ? ffs_write_strings(gadget) : -1;
	(void)dprintf(STDERR_FILENO,
		      "ffs-gadget: wrote descriptors preflight (end desc_rc=%d str_rc=%d errno=%d)\n",
		      desc_rc, str_rc, errno);
	syslog(LOG_INFO,
	       "ffs-gadget: wrote descriptors preflight (end desc_rc=%d str_rc=%d errno=%d)",
	       desc_rc, str_rc, errno);
	if (desc_rc != 0 || str_rc != 0) {
		close(gadget->ep0_fd);
		gadget->ep0_fd = -1;
		return -1;
	}

	if (pthread_create(&gadget->bulk_thread, NULL, bulk_thread_main, gadget) != 0) {
		perror("pthread_create ffs-gadget bulk");
		close(gadget->ep0_fd);
		gadget->ep0_fd = -1;
		return -1;
	}
	gadget->bulk_thread_created = true;

	if (pthread_create(&gadget->event_thread, NULL, event_thread_main, gadget) != 0) {
		perror("pthread_create ffs-gadget event loop");
		atomic_store(&gadget->stop_requested, true);
		pthread_join(gadget->bulk_thread, NULL);
		gadget->bulk_thread_created = false;
		close(gadget->ep0_fd);
		gadget->ep0_fd = -1;
		return -1;
	}
	gadget->event_thread_created = true;

	fprintf(stderr,
		"ffs-gadget: started on %s, waiting for a host (slot claimed on connect)\n",
		gadget->mount_path);
	syslog(LOG_INFO,
	       "ffs-gadget: started on %s, waiting for a host (slot claimed on connect)",
	       gadget->mount_path);
	return 0;
}

void ffs_gadget_stop(struct ffs_gadget *gadget)
{
	if (!gadget)
		return;

	atomic_store(&gadget->stop_requested, true);
	atomic_store(&gadget->bulk_thread_stop, true);

	if (gadget->event_thread_created) {
		(void)pthread_join(gadget->event_thread, NULL);
		gadget->event_thread_created = false;
	}
	if (gadget->bulk_thread_created) {
		(void)pthread_join(gadget->bulk_thread, NULL);
		gadget->bulk_thread_created = false;
	}
	/* Threads are stopped; if a host was still connected, hand the slot back to
	 * the USB/IP pool (both threads have observed stop and dropped the device). */
	ffs_release_slot(gadget);
	close_bulk_endpoint(gadget);
	if (gadget->ep0_fd >= 0) {
		close(gadget->ep0_fd);
		gadget->ep0_fd = -1;
	}
}
