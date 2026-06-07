/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __F_DISPLAYLINK_H
#define __F_DISPLAYLINK_H

#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/usb/composite.h>

/* DIAGNOSTIC: minimal bulk buffering (was 4 x 64K) to isolate a dwc3 bulk-OUT
 * wedge.  Restore to 4 / (64*1024) once the freeze is understood. */
#define DL_NUM_REQS     1
#define DL_BULK_BUFSIZE (4 * 1024)

struct f_displaylink {
	struct usb_function      func;
	struct usb_ep           *bulk_out;
	struct usb_request      *reqs[DL_NUM_REQS];
	int                      n_reqs;
	bool                     disabled;
	spinlock_t               lock;
	DECLARE_KFIFO_PTR(fifo, u8);
	wait_queue_head_t        read_wait;
	u8                       edid[128];
	u8                       vendor_desc[16];
	u8                       vendor_desc_len;
	u64                      rx_bytes;    /* total bulk bytes pushed into fifo */
	u64                      rx_dropped;  /* bulk bytes dropped (fifo full) */
	u32                      rx_reqs;     /* completed bulk requests */
	struct miscdevice        misc;
};

/*
 * f_dl_alloc - allocate and initialise a DisplayLink function.
 *
 * Registers /dev/udl_gadget, builds the EDID and vendor descriptor, and
 * sets up the usb_function callbacks.  On error returns ERR_PTR.
 */
struct f_displaylink *f_dl_alloc(void);

/*
 * f_dl_free - tear down a DisplayLink function previously returned by
 * f_dl_alloc.
 */
void f_dl_free(struct f_displaylink *dl);

/*
 * f_dl_handle_ctrl - handle DisplayLink device-level control requests.
 *
 * If the request is ours (0x5f vendor descriptor, EDID byte read, or channel
 * select) this fills cdev->req->buf and QUEUES the response on ep0 itself,
 * returning the byte count (>= 0).  If it is not ours it returns -EOPNOTSUPP
 * without queuing, so the caller's return propagates a stall.
 *
 * Both the config setup() and the function setup() callbacks must call this
 * and return its value directly: composite.c's unknown-request path does not
 * auto-queue ep0 (it goes straight to 'done'), so the response has to be
 * queued here.  For the channel-select OUT the queued transfer receives the
 * payload (which is discarded) before the status phase.
 */
int f_dl_handle_ctrl(struct f_displaylink *dl,
		     struct usb_composite_dev *cdev,
		     const struct usb_ctrlrequest *ctrl);

#endif /* __F_DISPLAYLINK_H */
