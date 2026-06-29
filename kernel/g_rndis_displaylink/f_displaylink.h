/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __F_DISPLAYLINK_H
#define __F_DISPLAYLINK_H

#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/usb/composite.h>

/*
 * Bulk-OUT buffering.  Two host facts drive these sizes:
 *
 *  - The host's udl driver keeps up to WRITES_IN_FLIGHT (20) bulk URBs queued,
 *    so we keep several OUT requests armed at all times: if the endpoint ever
 *    runs dry between completions, host URBs pile up unacked and the dwc3 OUT
 *    endpoint can wedge.
 *  - A single host URB is up to MAX_TRANSFER = PAGE_SIZE*16 - BULK_SIZE =
 *    65024 bytes.  Our per-request buffer MUST be >= that, or a full-size OUT
 *    overflows the request and the controller babbles / wedges the endpoint.
 *
 * 64 KiB is a multiple of the 512-byte HS maxpacket (and page-aligned), so it
 * covers the largest host URB and never trips dwc3's unaligned bounce/extra-TRB
 * path on a short final packet.
 */
/*
 * Match the known-good raw-gadget reference EXACTLY: a single in-flight bulk-OUT
 * request of 16 KiB (its BULK_OUT_BUFFER_SIZE).  That userspace gadget drove
 * this same Windows host to a working display on this very controller family,
 * reading one request at a time.  Our earlier 8×64K / 2×64K concurrent arming
 * hard-hangs BOTH dwc3 and sunxi/MUSB on the first bulk-OUT (completion never
 * fires).  A 16 KiB request still receives the host's large (up to 64K) URBs
 * across several maxpacket-aligned completions, terminated by the short packet.
 */
#define DL_NUM_REQS     1
#define DL_BULK_BUFSIZE (16 * 1024)

struct f_displaylink {
	struct usb_function      func;
	struct usb_ep           *bulk_out;
	struct usb_request      *reqs[DL_NUM_REQS];
	int                      n_reqs;
	bool                     disabled;
	/*
	 * Bulk-OUT arming is deferred out of the set_alt callback into a workqueue
	 * (see dl_arm_work).  Arming inside set_alt means the first usb_ep_queue
	 * runs in the UDC's set-alt/setup context — atomic, often with the
	 * controller lock held — which hard-hangs both dwc3 and sunxi on the first
	 * OUT.  The known-good userspace FFS path (ZeroKVM) submits its first AIO
	 * read as a separate, later step after the endpoint is enabled; arm_work
	 * mirrors that by queueing from process context outside the set_alt stack.
	 */
	struct work_struct       arm_work;
	bool                     armed;       /* bulk reqs currently queued */
	spinlock_t               lock;
	DECLARE_KFIFO_PTR(fifo, u8);
	wait_queue_head_t        read_wait;
	u8                       edid[128];
	u8                       vendor_desc[16];
	u8                       vendor_desc_len;
	bool                     key_loaded;  /* Windows SET_KEY (0x12 OUT) seen */
	u8                      *ctrl_ram;    /* 64K control-RAM for Windows peek/poke */
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
