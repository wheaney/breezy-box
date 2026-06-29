#pragma once

#include <stdbool.h>
#include <stdatomic.h>
#include <pthread.h>

#include "udl_device.h"

struct server_runtime;

/*
 * Raw Gadget transport — userspace USB DisplayLink device on /dev/raw-gadget.
 *
 * This is a userspace USB device emulation built on /dev/raw-gadget: it binds a
 * UDC, answers EP0 control traffic (delegated to udl_device_handle_control so it
 * speaks the exact same protocol as the USB/IP path), and drains the bulk OUT
 * endpoint into the slot's udl_runtime via udl_runtime_enqueue_bulk().  From the
 * renderer's point of view a Raw Gadget slot is indistinguishable from a USB/IP
 * slot — it feeds the same decode runtime and is drawn by the same code.
 *
 * Like ffs_gadget.c, it does NOT claim a display slot at start time — it claims
 * the lowest-index free slot dynamically on USB_RAW_EVENT_CONNECT (mirroring
 * FUNCTIONFS_ENABLE) and releases it on RESET/DISCONNECT, via
 * server_claim_gadget_slot()/server_release_gadget_slot().  The claim flags the
 * slot is_gadget_device so the USB/IP server excludes it from devlist/import
 * while the host cable holds it.
 */

#define RAW_GADGET_DEFAULT_DEVICE_PATH "/dev/raw-gadget"

struct raw_gadget {
	struct server_runtime *server;
	_Atomic(struct device_runtime *) device;  /* claimed slot; NULL when none */
	size_t claimed_slot;
	const char *raw_device_path;     /* /dev/raw-gadget */

	char udc_driver[128];
	char udc_device[128];

	int fd;                          /* /dev/raw-gadget; -1 when closed */
	int bulk_out_handle;             /* raw-gadget EP handle; -1 when disabled */
	uint8_t bulk_out_address;        /* chosen bulk OUT endpoint address */
	uint8_t current_configuration;
	bool bulk_out_address_valid;
	bool verbose;

	pthread_t event_thread;
	bool event_thread_created;

	pthread_t bulk_out_thread;
	bool bulk_out_thread_created;
	atomic_bool bulk_out_thread_stop;

	atomic_bool stop_requested;
};

/*
 * Returns true if a Raw Gadget device node is present and a UDC is exposed, i.e.
 * the transport can plausibly be started.  Cheap filesystem probe; no side
 * effects.  Pass NULL for raw_device_path to use the default.
 */
bool raw_gadget_available(const char *raw_device_path);

/*
 * Start the Raw Gadget transport on its own thread.  It comes up slot-less and
 * claims a slot only when a host actually connects (see above).
 * raw_device_path may be NULL for the default.  Returns 0 on success; on
 * failure nothing is left running.
 */
int raw_gadget_start(struct raw_gadget *gadget,
		     struct server_runtime *server,
		     const char *raw_device_path,
		     bool verbose);

/* Signal the transport to stop, join its threads, release any claimed slot,
 * and release the UDC. */
void raw_gadget_stop(struct raw_gadget *gadget);
