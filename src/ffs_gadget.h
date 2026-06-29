#pragma once

#include <stdbool.h>
#include <stdatomic.h>
#include <stddef.h>
#include <pthread.h>

#include "udl_device.h"

struct server_runtime;

/*
 * FunctionFS transport for a single DisplayLink display slot.
 *
 * A privileged setup step (breezy_gadget_setup.sh, run by breezy-gadget.service)
 * builds the configfs gadget skeleton, creates the ffs.<name> function, symlinks
 * it into the config, and mounts functionfs — owned by the app user so the
 * unprivileged renderer can drive it.  This transport then:
 *   - opens ep0 and writes the descriptor + strings blobs (which makes the data
 *     endpoints appear, signalling the root script to bind the UDC);
 *   - opens ep1 (bulk OUT) and drains it into the slot's udl_runtime via
 *     udl_runtime_enqueue_bulk();
 *   - services ep0 control events through udl_device_handle_control(), so it
 *     speaks the exact same DisplayLink protocol as the USB/IP path.
 *
 * From the renderer's point of view a gadget slot is identical to a USB/IP slot:
 * it feeds the same decode runtime and is drawn by the same code.
 *
 * The slot is NOT claimed up front.  The transport comes up slot-less and only
 * claims a display when a host actually connects (FUNCTIONFS_ENABLE), via
 * server_claim_gadget_slot, and releases it back to the USB/IP pool on
 * disconnect (DISABLE/UNBIND).  So a slot is only held while a host is present.
 */

#define FFS_GADGET_DEFAULT_MOUNT "/dev/ffs-dl"

struct ffs_gadget {
	struct server_runtime *server;   /* slot pool; claim happens on host connect */
	char mount_path[256];            /* functionfs mount (contains ep0, ep1, ...) */

	int ep0_fd;                      /* control endpoint; owned by event thread */
	int ep1_fd;                      /* bulk OUT endpoint; owned by bulk thread */
	bool verbose;
	atomic_bool enabled;             /* set on ENABLE, cleared on DISABLE/UNBIND */

	/* Currently claimed slot, published by the event thread for the bulk thread
	 * and read locklessly.  device==NULL means no host / no slot claimed. */
	_Atomic(struct device_runtime *) device;
	size_t claimed_slot;             /* valid while device != NULL */

	pthread_t event_thread;
	bool event_thread_created;

	pthread_t bulk_thread;
	bool bulk_thread_created;
	atomic_bool bulk_thread_stop;

	atomic_bool stop_requested;
};

/*
 * Returns true if a functionfs mount with an ep0 the caller can open is present
 * at mount_path (NULL → default).  Cheap probe; no side effects.
 */
bool ffs_gadget_available(const char *mount_path);

/*
 * Start the FFS transport on its own threads.  No slot is claimed yet — the
 * transport writes its USB descriptors and waits; it claims a slot from
 * `server` only when a host connects.  mount_path may be NULL for the default.
 * Returns 0 on success; on failure nothing is left running.
 */
int ffs_gadget_start(struct ffs_gadget *gadget,
		     struct server_runtime *server,
		     const char *mount_path,
		     bool verbose);

/* Signal the transport to stop, join its threads, and close the endpoints. */
void ffs_gadget_stop(struct ffs_gadget *gadget);
