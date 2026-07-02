/*
 * Raw Gadget transport — userspace USB DisplayLink device on /dev/raw-gadget.
 *
 * Distilled from the standalone displaylink_gadget_raw_gadget.c reference: only
 * the USB machinery survives (UDC bind, EP0 event loop, bulk OUT drain).  All
 * decode/viewer/output-ring logic is gone — EP0 control is delegated to the
 * server's udl_device_handle_control(), and bulk OUT is fed straight into the
 * slot's udl_runtime, exactly like the USB/IP path.
 */
#define _POSIX_C_SOURCE 200809L

#include "raw_gadget.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <linux/usb/ch9.h>
#include <linux/usb/raw_gadget.h>

#include "udl_runtime.h"
#include "server.h"

#define DEFAULT_BULK_OUT_ADDRESS        UDL_PRIMARY_BULK_OUT_EP
#define EP0_IO_BUFFER_SIZE              512u
/* raw_gadget's USB_RAW_IOCTL_EP_READ rejects io->length > PAGE_SIZE (4096).
 * Large DL bulk transfers are simply split across multiple reads. */
#define BULK_OUT_IO_BUFFER_SIZE         4096u
#define UDC_SOFT_RECONNECT_SETTLE_USEC  250000u

/*
 * These mirror the Raw Gadget UAPI's own embedding pattern: a header struct that
 * ends in a flexible array member, immediately followed by inline storage.  GCC's
 * -Wpedantic flags the embedded FAM even though the layout is exactly what the
 * ioctls expect, so quiet it just here.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct usb_raw_control_event {
	struct usb_raw_event inner;
	struct usb_ctrlrequest ctrl;
};

struct usb_raw_control_io {
	struct usb_raw_ep_io inner;
	uint8_t data[EP0_IO_BUFFER_SIZE];
};

struct usb_raw_bulk_io {
	struct usb_raw_ep_io inner;
	uint8_t data[BULK_OUT_IO_BUFFER_SIZE];
};
#pragma GCC diagnostic pop

/* ---------------------------------------------------------------- */
/* Small helpers                                                    */
/* ---------------------------------------------------------------- */

static bool raw_gadget_stop_requested(const struct raw_gadget *gadget)
{
	return gadget && atomic_load(&gadget->stop_requested);
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

static const char *path_basename(const char *path)
{
	const char *slash = strrchr(path, '/');

	return slash ? slash + 1 : path;
}

static int read_single_line_file(const char *path, char *buffer, size_t capacity)
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

static int read_uevent_value(const char *path, const char *key,
			     char *buffer, size_t capacity)
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

/* ---------------------------------------------------------------- */
/* UDC discovery / soft-connect                                     */
/* ---------------------------------------------------------------- */

static int resolve_udc_driver_for_device(const char *udc_device,
					 char *driver_buffer, size_t driver_capacity)
{
	char uevent_path[512];
	char symlink_path[512];
	char driver_target[512];
	ssize_t link_len;

	if (!udc_device || !driver_buffer || driver_capacity == 0u)
		return -1;
	if (snprintf(uevent_path, sizeof(uevent_path),
		     "/sys/class/udc/%s/uevent", udc_device) >= (int)sizeof(uevent_path))
		return -1;
	if (read_uevent_value(uevent_path, "USB_UDC_NAME",
			      driver_buffer, driver_capacity) == 0)
		return 0;

	if (snprintf(symlink_path, sizeof(symlink_path),
		     "/sys/class/udc/%s/device/driver", udc_device) >= (int)sizeof(symlink_path))
		return -1;
	link_len = readlink(symlink_path, driver_target, sizeof(driver_target) - 1u);
	if (link_len < 0)
		return -1;
	driver_target[link_len] = '\0';
	if (snprintf(driver_buffer, driver_capacity, "%s",
		     path_basename(driver_target)) >= (int)driver_capacity)
		return -1;
	return 0;
}

static int auto_detect_udc(char *driver_buffer, size_t driver_capacity,
			   char *device_buffer, size_t device_capacity)
{
	DIR *directory;
	struct dirent *entry;
	int found = 0;

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
				"raw-gadget: multiple UDCs under /sys/class/udc; the gadget transport needs an explicit UDC\n");
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
		fprintf(stderr, "raw-gadget: no UDC found under /sys/class/udc\n");
		return -1;
	}
	if (resolve_udc_driver_for_device(device_buffer, driver_buffer, driver_capacity) != 0) {
		/* Fall back to using the device name as the driver name. */
		if (snprintf(driver_buffer, driver_capacity, "%s", device_buffer) >= (int)driver_capacity)
			return -1;
	}
	return 0;
}

static int set_udc_soft_connect(const char *udc_device, const char *command,
				bool verbose, bool ignore_unsupported)
{
	char path[512];
	int saved_errno;

	if (snprintf(path, sizeof(path),
		     "/sys/class/udc/%s/soft_connect", udc_device) >= (int)sizeof(path)) {
		errno = ENAMETOOLONG;
		return -1;
	}
	if (write_single_line_file(path, command) == 0)
		return 0;
	saved_errno = errno;
	if (ignore_unsupported &&
	    (saved_errno == ENOENT || saved_errno == ENOTDIR || saved_errno == EOPNOTSUPP)) {
		if (verbose)
			fprintf(stderr, "raw-gadget: soft_connect unavailable on %s, skipping '%s'\n",
				udc_device, command);
		return 0;
	}
	errno = saved_errno;
	return -1;
}

/* ---------------------------------------------------------------- */
/* Raw Gadget ioctl wrappers                                        */
/* ---------------------------------------------------------------- */

static int raw_init(int fd, enum usb_device_speed speed,
		    const char *driver_name, const char *device_name)
{
	struct usb_raw_init init;

	memset(&init, 0, sizeof(init));
	if (snprintf((char *)init.driver_name, sizeof(init.driver_name),
		     "%s", driver_name) >= (int)sizeof(init.driver_name))
		return -1;
	if (snprintf((char *)init.device_name, sizeof(init.device_name),
		     "%s", device_name) >= (int)sizeof(init.device_name))
		return -1;
	init.speed = (uint8_t)speed;
	return ioctl(fd, USB_RAW_IOCTL_INIT, &init);
}

static int raw_run(int fd) { return ioctl(fd, USB_RAW_IOCTL_RUN, 0); }

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

static int raw_ep0_stall(int fd) { return ioctl(fd, USB_RAW_IOCTL_EP0_STALL, 0); }

static int raw_ep_enable(int fd, struct usb_endpoint_descriptor *desc)
{
	return ioctl(fd, USB_RAW_IOCTL_EP_ENABLE, desc);
}

static int raw_ep_disable(int fd, int handle)
{
	return ioctl(fd, USB_RAW_IOCTL_EP_DISABLE, handle);
}

static int raw_configure(int fd) { return ioctl(fd, USB_RAW_IOCTL_CONFIGURE, 0); }

static int raw_vbus_draw(int fd, uint32_t power_2ma_units)
{
	return ioctl(fd, USB_RAW_IOCTL_VBUS_DRAW, power_2ma_units);
}

static int raw_eps_info(int fd, struct usb_raw_eps_info *info)
{
	memset(info, 0, sizeof(*info));
	return ioctl(fd, USB_RAW_IOCTL_EPS_INFO, info);
}

/* ---------------------------------------------------------------- */
/* Bulk OUT endpoint                                                */
/* ---------------------------------------------------------------- */

static int choose_bulk_out_address(struct raw_gadget *gadget)
{
	struct usb_raw_eps_info info;
	int count;
	int i;

	count = raw_eps_info(gadget->fd, &info);
	if (count < 0) {
		perror("ioctl USB_RAW_IOCTL_EPS_INFO");
		return -1;
	}
	for (i = 0; i < count; ++i) {
		const struct usb_raw_ep_info *ep = &info.eps[i];
		const uint8_t candidate = (ep->addr == USB_RAW_EP_ADDR_ANY)
			? DEFAULT_BULK_OUT_ADDRESS : (uint8_t)ep->addr;

		if (!ep->caps.type_bulk || !ep->caps.dir_out)
			continue;
		/* The host's udl driver submits render URBs to the primary bulk OUT
		 * address; only that layout is compatible. */
		if (candidate != DEFAULT_BULK_OUT_ADDRESS)
			continue;
		gadget->bulk_out_address = candidate;
		gadget->bulk_out_address_valid = true;
		return 0;
	}
	fprintf(stderr,
		"raw-gadget: no bulk OUT endpoint at address 0x%02x; this UDC layout is incompatible\n",
		DEFAULT_BULK_OUT_ADDRESS);
	return -1;
}

static int enable_bulk_out_endpoint(struct raw_gadget *gadget)
{
	struct usb_endpoint_descriptor descriptor;
	struct device_runtime *dev =
		atomic_load_explicit(&gadget->device, memory_order_acquire);
	int handle;

	/* Always disable first if we think it's enabled: a USB bus reset
	 * (e.g. Windows re-enumerating mid-handshake) invalidates the kernel-
	 * side endpoint state without surfacing a USB_RAW_EVENT_RESET to us,
	 * leaving a stale handle that causes EINVAL on the first EP_READ.
	 * Disabling before re-enabling is idempotent on a fresh session and
	 * restores a clean state after a silent bus reset. */
	if (gadget->bulk_out_handle >= 0) {
		int stale = gadget->bulk_out_handle;
		gadget->bulk_out_handle = -1;
		/* Ignore error — endpoint may already be disabled by a bus reset */
		(void)raw_ep_disable(gadget->fd, stale);
	}
	if (!gadget->bulk_out_address_valid && choose_bulk_out_address(gadget) != 0)
		return -1;

	memset(&descriptor, 0, sizeof(descriptor));
	descriptor.bLength = USB_DT_ENDPOINT_SIZE;
	descriptor.bDescriptorType = USB_DT_ENDPOINT;
	descriptor.bEndpointAddress = gadget->bulk_out_address;
	descriptor.bmAttributes = USB_ENDPOINT_XFER_BULK;
	descriptor.wMaxPacketSize =
		(dev && usb_speed_is_super(dev->opts.usb_speed)) ? 1024u : 512u;

	handle = raw_ep_enable(gadget->fd, &descriptor);
	if (handle < 0) {
		perror("ioctl USB_RAW_IOCTL_EP_ENABLE");
		return -1;
	}
	gadget->bulk_out_handle = handle;
	fprintf(stderr, "raw-gadget: bulk OUT enabled handle=%d address=0x%02x\n",
		gadget->bulk_out_handle, gadget->bulk_out_address);
	return 0;
}

static void disable_bulk_out_endpoint(struct raw_gadget *gadget)
{
	int handle;

	if (gadget->bulk_out_handle < 0)
		return;
	handle = gadget->bulk_out_handle;
	gadget->bulk_out_handle = -1;
	if (raw_ep_disable(gadget->fd, handle) < 0 && gadget->verbose && errno != EINVAL)
		perror("ioctl USB_RAW_IOCTL_EP_DISABLE");
}

static void *bulk_out_drain_thread_main(void *arg)
{
	struct raw_gadget *gadget = arg;
	struct usb_raw_bulk_io io;
	const uint16_t handle = (uint16_t)gadget->bulk_out_handle;

	(void)pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	(void)pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);

	if (gadget->verbose)
		fprintf(stderr, "raw-gadget: bulk OUT drain thread started handle=%u\n",
			(unsigned int)handle);

	while (!raw_gadget_stop_requested(gadget) &&
	       !atomic_load(&gadget->bulk_out_thread_stop)) {
		int rc;

		memset(&io, 0, sizeof(io));
		io.inner.ep = handle;
		io.inner.length = sizeof(io.data);

		rc = raw_ep_read(gadget->fd, &io.inner);
		if (rc < 0) {
			if (errno == EINTR && !raw_gadget_stop_requested(gadget))
				continue;
			if (!raw_gadget_stop_requested(gadget) &&
			    !atomic_load(&gadget->bulk_out_thread_stop) &&
			    gadget->verbose)
				perror("ioctl USB_RAW_IOCTL_EP_READ");
			break;
		}
		if (rc > 0) {
			struct device_runtime *dev =
				atomic_load_explicit(&gadget->device, memory_order_acquire);

			if (dev)
				(void)udl_runtime_enqueue_bulk(&dev->udl, io.data, (size_t)rc);
		}
	}

	if (gadget->verbose)
		fprintf(stderr, "raw-gadget: bulk OUT drain thread stopped\n");
	return NULL;
}

static int start_bulk_out_drain_thread(struct raw_gadget *gadget)
{
	int rc;

	if (gadget->bulk_out_thread_created)
		return 0;
	atomic_store(&gadget->bulk_out_thread_stop, false);
	rc = pthread_create(&gadget->bulk_out_thread, NULL,
			    bulk_out_drain_thread_main, gadget);
	if (rc != 0) {
		errno = rc;
		perror("pthread_create raw-gadget bulk OUT drain");
		return -1;
	}
	gadget->bulk_out_thread_created = true;
	return 0;
}

static void stop_bulk_out_drain_thread(struct raw_gadget *gadget)
{
	if (gadget->bulk_out_thread_created) {
		atomic_store(&gadget->bulk_out_thread_stop, true);
		(void)pthread_cancel(gadget->bulk_out_thread);
		(void)pthread_join(gadget->bulk_out_thread, NULL);
		gadget->bulk_out_thread_created = false;
		atomic_store(&gadget->bulk_out_thread_stop, false);
	}
	disable_bulk_out_endpoint(gadget);
}

static void reset_configuration_state(struct raw_gadget *gadget)
{
	gadget->current_configuration = 0u;
	stop_bulk_out_drain_thread(gadget);
}

/* ---------------------------------------------------------------- */
/* Slot claim / release (on host connect / disconnect)              */
/* ---------------------------------------------------------------- */

/* Claim a display slot for this host connection, if not already holding one.
 * Runs on the event thread (USB_RAW_EVENT_CONNECT).  Publishes the slot's
 * device_runtime for the bulk thread with release ordering. */
static void raw_gadget_claim_slot(struct raw_gadget *gadget)
{
	size_t slot = SIZE_MAX;
	bool created = false;

	if (atomic_load_explicit(&gadget->device, memory_order_relaxed))
		return; /* already holding a slot */

	if (server_claim_gadget_slot(gadget->server, &slot, &created) == SIZE_MAX) {
		fprintf(stderr, "raw-gadget: no display slot available for host\n");
		return;
	}
	gadget->claimed_slot = slot;
	gadget->server->devices[slot].gadget_host_connected = true;
	atomic_store_explicit(&gadget->device, &gadget->server->devices[slot],
			      memory_order_release);
	fprintf(stderr,
		"raw-gadget: host connected → claimed slot %zu (%s) %ux%u@%u\n",
		slot, created ? "new fallback" : "existing",
		gadget->server->devices[slot].opts.decode_width,
		gadget->server->devices[slot].opts.decode_height,
		gadget->server->devices[slot].opts.refresh_hz);
}

/* Release the claimed slot back to the USB/IP pool (host disconnect).  Clears
 * the published device first so the bulk thread stops feeding it, then releases
 * the slot.  Runs on the event thread. */
static void raw_gadget_release_slot(struct raw_gadget *gadget)
{
	struct device_runtime *dev =
		atomic_exchange_explicit(&gadget->device, NULL, memory_order_acq_rel);

	if (!dev)
		return;
	dev->gadget_host_connected = false;
	server_release_gadget_slot(gadget->server, gadget->claimed_slot);
	if (gadget->verbose)
		fprintf(stderr, "raw-gadget: host disconnected → released slot %zu\n",
			gadget->claimed_slot);
}

/* ---------------------------------------------------------------- */
/* EP0 control loop                                                 */
/* ---------------------------------------------------------------- */

/*
 * Translate one EP0 setup event into a raw-gadget transfer.  Standard and
 * vendor requests are dispatched through udl_device_handle_control(); the
 * one exception is SET_CONFIGURATION, which on a real UDC must drive the
 * raw-gadget endpoint enable/configure dance the USB/IP path never needs.
 */
static int handle_control_event(struct raw_gadget *gadget,
				const struct usb_ctrlrequest *setup)
{
	struct device_runtime *dev =
		atomic_load_explicit(&gadget->device, memory_order_acquire);
	struct usb_raw_control_io io;
	struct control_result result;
	const uint16_t wLength = (uint16_t)setup->wLength;
	const bool host_to_device = (setup->bRequestType & USB_DIR_IN) == 0u;
	const uint8_t *payload = NULL;
	size_t payload_length = 0u;

	memset(&io, 0, sizeof(io));
	io.inner.ep = 0u;

	if (gadget->verbose)
		fprintf(stderr,
			"raw-gadget: SETUP rt=0x%02x req=0x%02x val=0x%04x idx=0x%04x len=%u dev=%s\n",
			(unsigned int)setup->bRequestType,
			(unsigned int)setup->bRequest,
			(unsigned int)(uint16_t)setup->wValue,
			(unsigned int)(uint16_t)setup->wIndex,
			(unsigned int)wLength,
			dev ? "yes" : "no");

	/* Collect an OUT data stage up front so vendor handlers (e.g. SET_KEY)
	 * see the payload, mirroring the USB/IP submit which carries it inline. */
	if (host_to_device && wLength > 0u) {
		int rc;

		io.inner.length = wLength > sizeof(io.data) ? sizeof(io.data) : wLength;
		rc = raw_ep0_read(gadget->fd, &io);
		if (rc < 0) {
			perror("ioctl USB_RAW_IOCTL_EP0_READ");
			return -1;
		}
		payload = io.data;
		payload_length = (size_t)rc;
	}

	/*
	 * No slot claimed yet (a control request arrived before
	 * USB_RAW_EVENT_CONNECT claimed one).  Complete the transfer minimally
	 * (ACK, never stall) so enumeration isn't broken; the vendor handshake
	 * only starts once a slot is claimed.
	 */
	if (!dev) {
		struct usb_raw_control_io ack = { .inner = { .ep = 0u, .length = 0u } };

		/*
		 * The transfer's pending flag was latched by direction in the
		 * kernel's gadget_setup(): an IN request with a data stage sets
		 * ep0_in_pending (cleared by EP0_WRITE); everything else sets
		 * ep0_out_pending (cleared by EP0_READ).  Acknowledge with a
		 * zero-length transfer in the MATCHING direction — using the
		 * wrong one fails the kernel's direction check with -EBUSY (the
		 * "pre-claim ack: Device or resource busy" we used to see).  An
		 * OUT request carrying a payload was already drained above, so
		 * only its zero-length status remains (an IN status = EP0_WRITE).
		 */
		if (!host_to_device && wLength > 0u) {
			if (raw_ep0_write(gadget->fd, &ack) < 0 && gadget->verbose)
				perror("ioctl USB_RAW_IOCTL_EP0_WRITE (pre-claim ack)");
		} else {
			if (raw_ep0_read(gadget->fd, &ack) < 0 && gadget->verbose)
				perror("ioctl USB_RAW_IOCTL_EP0_READ (pre-claim ack)");
		}
		return 0;
	}

	/* SET_CONFIGURATION: enable the data endpoints and arm the drain thread. */
	if ((setup->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD &&
	    setup->bRequest == USB_REQ_SET_CONFIGURATION) {
		gadget->current_configuration = (uint8_t)(setup->wValue & 0xffu);
		dev->current_configuration = gadget->current_configuration;
		if (gadget->current_configuration != 0u) {
			if (enable_bulk_out_endpoint(gadget) != 0)
				return -1;
			if (raw_vbus_draw(gadget->fd, 250u) < 0) {
				perror("ioctl USB_RAW_IOCTL_VBUS_DRAW");
				return -1;
			}
			if (raw_configure(gadget->fd) < 0) {
				perror("ioctl USB_RAW_IOCTL_CONFIGURE");
				return -1;
			}
			if (start_bulk_out_drain_thread(gadget) != 0)
				return -1;
		} else {
			stop_bulk_out_drain_thread(gadget);
		}
		struct usb_raw_control_io ack = { .inner = { .ep = 0u, .length = 0u } };
		if (raw_ep0_read(gadget->fd, &ack) < 0) {
			/*
			 * A failed status-stage read here (commonly -EPROTO) means the
			 * host already moved past this transfer's status phase before
			 * we got to it -- e.g. Windows issuing a fast SET_CONFIGURATION
			 * immediately followed by more SETUPs.  This is a race on the
			 * wire, not a fatal gadget error: killing the whole event loop
			 * over it tears down enumeration entirely (Windows then shows
			 * "This device cannot start (Code 10)").  Log and continue; the
			 * next SETUP will be fetched normally on the next loop iteration.
			 */
			if (gadget->verbose)
				perror("ioctl USB_RAW_IOCTL_EP0_READ (set-config ack)");
		}
		return 0;
	}

	if (udl_device_handle_control(dev, setup,
				      payload, payload_length, &result) != 0) {
		/*
		 * The device handler couldn't service this request (unknown or
		 * malformed vendor request).  STALL the control transfer rather
		 * than returning fatally: a STALL cleanly terminates the EP0
		 * transfer and clears the UDC/raw_gadget pending state, so the
		 * NEXT request is serviced normally.  Returning -1 here instead
		 * tears down the whole event loop AND leaves ep0_{in,out}_pending
		 * latched in raw_gadget, after which every subsequent SETUP is
		 * rejected with -EBUSY and enumeration wedges permanently (this
		 * is the cascade that followed Windows' multi-byte RAM poke).
		 */
		if (raw_ep0_stall(gadget->fd) < 0 && gadget->verbose)
			perror("ioctl USB_RAW_IOCTL_EP0_STALL (unhandled request)");
		return 0;
	}

	switch (result.action) {
	case CONTROL_ACTION_STALL:
		if (raw_ep0_stall(gadget->fd) < 0) {
			perror("ioctl USB_RAW_IOCTL_EP0_STALL");
			return -1;
		}
		return 0;
	case CONTROL_ACTION_DATA_IN: {
		size_t len = result.actual_length;

		if (len > sizeof(io.data))
			len = sizeof(io.data);
		if (len > wLength)
			len = wLength;
		memcpy(io.data, result.data, len);
		io.inner.ep = 0u;
		io.inner.length = (unsigned int)len;
		if ((setup->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD &&
		    setup->bRequest == USB_REQ_GET_DESCRIPTOR &&
		    (uint8_t)((uint16_t)setup->wValue >> 8) == UDL_VENDOR_DESCRIPTOR_TYPE) {
			fprintf(stderr,
				"raw-gadget: GET_DESCRIPTOR 0x5f req_len=%u sent=%zu hdr=[%u,0x%02x,0x%02x,0x%02x,%u]\n",
				(unsigned int)wLength, len,
				(unsigned int)(len >= 1u ? io.data[0] : 0u),
				(unsigned int)(len >= 2u ? io.data[1] : 0u),
				(unsigned int)(len >= 3u ? io.data[2] : 0u),
				(unsigned int)(len >= 4u ? io.data[3] : 0u),
				(unsigned int)(len >= 5u ? io.data[4] : 0u));
		}
		if (raw_ep0_write(gadget->fd, &io) < 0) {
			perror("ioctl USB_RAW_IOCTL_EP0_WRITE");
			return -1;
		}
		return 0;
	}
	case CONTROL_ACTION_ACK:
		/* For an IN/no-data status stage (or an already-drained OUT stage)
		 * acknowledge with a zero-length EP0 transfer in the status
		 * direction.  Host-to-device with payload was already read above. */
		if (!host_to_device || wLength == 0u) {
			struct usb_raw_control_io ack = { .inner = { .ep = 0u, .length = 0u } };
			if (raw_ep0_read(gadget->fd, &ack) < 0) {
				/* See the matching comment on the SET_CONFIGURATION ack
				 * above: a raced status-stage read is not fatal. */
				if (gadget->verbose)
					perror("ioctl USB_RAW_IOCTL_EP0_READ (ack)");
			}
		}
		return 0;
	}

	return 0;
}

static int run_event_loop(struct raw_gadget *gadget)
{
	while (!raw_gadget_stop_requested(gadget)) {
		struct usb_raw_control_event event;
		int rc;

		if (gadget->verbose) {
			struct timespec _t0, _t1;
			clock_gettime(CLOCK_MONOTONIC, &_t0);
			rc = raw_event_fetch(gadget->fd, &event);
			clock_gettime(CLOCK_MONOTONIC, &_t1);
			long _ms = (_t1.tv_sec - _t0.tv_sec) * 1000L +
				(_t1.tv_nsec - _t0.tv_nsec) / 1000000L;
			fprintf(stderr,
				"raw-gadget: EVENT_FETCH returned rc=%d type=%d after %ldms wait\n",
				rc, rc >= 0 ? (int)event.inner.type : -1, _ms);
		} else {
			rc = raw_event_fetch(gadget->fd, &event);
		}

		if (rc < 0) {
			if (errno == EINTR)
				continue;
			perror("ioctl USB_RAW_IOCTL_EVENT_FETCH");
			return -1;
		}

		switch ((enum usb_raw_event_type)event.inner.type) {
		case USB_RAW_EVENT_CONNECT:
			reset_configuration_state(gadget);
			if (choose_bulk_out_address(gadget) != 0)
				return -1;
			raw_gadget_claim_slot(gadget);
			fprintf(stderr, "raw-gadget: connect (UDC bound, awaiting enumeration)\n");
			break;
		case USB_RAW_EVENT_RESET:
			/*
			 * A bus reset is part of normal enumeration: the host
			 * resets the bus and then continues probing on the SAME
			 * connection (macOS resets several times before/around
			 * SET_ADDRESS).  Tear down only the configuration/endpoint
			 * state (drain thread, config number) — do NOT release the
			 * slot, or every following SETUP arrives with no device
			 * claimed (dev=no) and enumeration can never complete.
			 * The slot is released on DISCONNECT (cable/host gone).
			 */
			if (gadget->verbose)
				fprintf(stderr, "raw-gadget: reset\n");
			reset_configuration_state(gadget);
			break;
		case USB_RAW_EVENT_DISCONNECT:
			if (gadget->verbose)
				fprintf(stderr, "raw-gadget: disconnect\n");
			reset_configuration_state(gadget);
			raw_gadget_release_slot(gadget);
			break;
		case USB_RAW_EVENT_SUSPEND:
		case USB_RAW_EVENT_RESUME:
			break;
		case USB_RAW_EVENT_CONTROL:
			if (handle_control_event(gadget, &event.ctrl) != 0)
				return -1;
			break;
		default:
			break;
		}
	}
	return 0;
}

/* ---------------------------------------------------------------- */
/* Public API                                                       */
/* ---------------------------------------------------------------- */

static bool any_udc_present(void)
{
	DIR *dir = opendir("/sys/class/udc");
	struct dirent *entry;
	bool found = false;

	if (!dir)
		return false;
	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_name[0] == '.')
			continue;
		found = true;
		break;
	}
	closedir(dir);
	return found;
}

bool raw_gadget_available(const char *raw_device_path)
{
	const char *path = raw_device_path ? raw_device_path : RAW_GADGET_DEFAULT_DEVICE_PATH;

	if (access(path, R_OK | W_OK) != 0)
		return false;
	return any_udc_present();
}

static void *event_thread_main(void *arg)
{
	struct raw_gadget *gadget = arg;

	(void)run_event_loop(gadget);
	return NULL;
}

int raw_gadget_start(struct raw_gadget *gadget,
		     struct server_runtime *server,
		     const char *raw_device_path,
		     bool verbose)
{
	if (!gadget || !server)
		return -1;

	memset(gadget, 0, sizeof(*gadget));
	gadget->server = server;
	gadget->raw_device_path = raw_device_path ? raw_device_path
						  : RAW_GADGET_DEFAULT_DEVICE_PATH;
	gadget->fd = -1;
	gadget->bulk_out_handle = -1;
	gadget->bulk_out_address = DEFAULT_BULK_OUT_ADDRESS;
	gadget->verbose = verbose;
	gadget->claimed_slot = 0u;
	atomic_init(&gadget->stop_requested, false);
	atomic_init(&gadget->bulk_out_thread_stop, false);
	atomic_init(&gadget->device, NULL);

	if (auto_detect_udc(gadget->udc_driver, sizeof(gadget->udc_driver),
			    gadget->udc_device, sizeof(gadget->udc_device)) != 0)
		return -1;

	gadget->fd = open(gadget->raw_device_path, O_RDWR);
	if (gadget->fd < 0) {
		fprintf(stderr, "raw-gadget: open(%s): %s\n",
			gadget->raw_device_path, strerror(errno));
		if (errno == ENOENT)
			fprintf(stderr, "raw-gadget: try 'modprobe raw_gadget' (needs CONFIG_USB_RAW_GADGET)\n");
		return -1;
	}

	/* No slot is claimed yet, so there is no per-slot usb_speed to honor;
	 * USB_SPEED_HIGH is the server-wide default (see server.c) and the UDC
	 * negotiates actual link speed with the host independently of this. */
	if (raw_init(gadget->fd, USB_SPEED_HIGH,
		     gadget->udc_driver, gadget->udc_device) != 0) {
		perror("ioctl USB_RAW_IOCTL_INIT");
		close(gadget->fd);
		gadget->fd = -1;
		return -1;
	}
	if (raw_run(gadget->fd) != 0) {
		perror("ioctl USB_RAW_IOCTL_RUN");
		fprintf(stderr,
			"raw-gadget: could not bind UDC %s (driver %s); is another gadget owning it?\n",
			gadget->udc_device, gadget->udc_driver);
		close(gadget->fd);
		gadget->fd = -1;
		return -1;
	}

	/* Leave an idle UDC armed; pulse a fresh attach if one is already up. */
	{
		char state[64];
		char state_path[512];

		if (snprintf(state_path, sizeof(state_path),
			     "/sys/class/udc/%s/state", gadget->udc_device) < (int)sizeof(state_path) &&
		    read_single_line_file(state_path, state, sizeof(state)) == 0 &&
		    strcmp(state, "not attached") != 0) {
			(void)set_udc_soft_connect(gadget->udc_device, "disconnect", verbose, true);
			sleep_microseconds(UDC_SOFT_RECONNECT_SETTLE_USEC);
			(void)set_udc_soft_connect(gadget->udc_device, "connect", verbose, true);
		}
	}

	if (pthread_create(&gadget->event_thread, NULL, event_thread_main, gadget) != 0) {
		perror("pthread_create raw-gadget event loop");
		reset_configuration_state(gadget);
		close(gadget->fd);
		gadget->fd = -1;
		return -1;
	}
	gadget->event_thread_created = true;

	fprintf(stderr,
		"raw-gadget: started on %s (udc %s/%s), waiting for a host (slot claimed on connect)\n",
		gadget->raw_device_path, gadget->udc_driver, gadget->udc_device);
	return 0;
}

void raw_gadget_stop(struct raw_gadget *gadget)
{
	if (!gadget)
		return;

	atomic_store(&gadget->stop_requested, true);

	if (gadget->fd >= 0) {
		/*
		 * Close the fd FIRST so any blocking ioctl in the event thread
		 * (EVENT_FETCH, EP0_WRITE, EP_READ) returns immediately with an
		 * error.  This lets the threads exit through their normal error
		 * path rather than being cancelled mid-syscall — cancelling
		 * mid-ioctl leaves the UDC EP0 state machine wedged, causing
		 * ECONNRESET on the next session.
		 *
		 * Soft-disconnect is done AFTER the join so the host sees a
		 * clean detach even though the fd is already gone; it writes to
		 * sysfs (not the gadget fd) so it still works.
		 */
		close(gadget->fd);
		gadget->fd = -1;
	}

	/* Threads unblock on fd close; join without cancel. */
	if (gadget->event_thread_created) {
		(void)pthread_join(gadget->event_thread, NULL);
		gadget->event_thread_created = false;
	}
	stop_bulk_out_drain_thread(gadget);
	raw_gadget_release_slot(gadget);

	/* Soft-disconnect after threads have exited so the host sees a clean
	 * detach.  Best-effort — requires root for sysfs write, fails silently
	 * otherwise, but the fd close above already ended the gadget session. */
	(void)set_udc_soft_connect(gadget->udc_device, "disconnect",
				   gadget->verbose, true);
}
