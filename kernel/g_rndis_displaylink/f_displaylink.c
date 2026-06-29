// SPDX-License-Identifier: GPL-2.0
/*
 * f_displaylink.c - USB gadget function for DisplayLink
 *
 * Presents a vendor-class interface with a single bulk OUT endpoint.
 * Received USB command stream is buffered and exposed via /dev/udl_gadget
 * for the KMS renderer (displaylink_kms_renderer) to read and decode.
 *
 * Device-level control requests (0x5f vendor descriptor, EDID byte reads,
 * channel select) are dispatched from the composite driver's setup()
 * callback via f_dl_handle_ctrl().  Vendor requests that some kernel
 * versions route to functions are also handled in dl_setup() so the module
 * works on both dispatch paths.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
/*
 * put_unaligned_le32/16 live in <linux/unaligned.h> on kernels >= 6.12 and in
 * <asm/unaligned.h> before that.  The ROCK 4C+ runs 6.18 (new path); the A733
 * vendor kernel runs 6.6 (old path).  Pick the right one.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "f_displaylink.h"

/* ---------- USB descriptors -------------------------------------------- */

static struct usb_interface_descriptor fs_dl_intf_desc = {
	.bLength            = USB_DT_INTERFACE_SIZE,
	.bDescriptorType    = USB_DT_INTERFACE,
	/* bInterfaceNumber: assigned at bind time */
	.bNumEndpoints      = 1,
	.bInterfaceClass    = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass = 0x00,
	.bInterfaceProtocol = 0x00,
};

static struct usb_endpoint_descriptor fs_dl_ep_desc = {
	.bLength          = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes     = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = cpu_to_le16(64),
};

static struct usb_endpoint_descriptor hs_dl_ep_desc = {
	.bLength          = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes     = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = cpu_to_le16(512),
};

static struct usb_descriptor_header *fs_dl_descs[] = {
	(struct usb_descriptor_header *)&fs_dl_intf_desc,
	(struct usb_descriptor_header *)&fs_dl_ep_desc,
	NULL,
};

static struct usb_descriptor_header *hs_dl_descs[] = {
	(struct usb_descriptor_header *)&fs_dl_intf_desc,
	(struct usb_descriptor_header *)&hs_dl_ep_desc,
	NULL,
};

/* ---------- EDID & vendor descriptor ------------------------------------ */

static void build_edid(u8 *edid)
{
	static const u8 dtd_1080p60[18] = {
		0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40,
		0x58, 0x2c, 0x45, 0x00, 0x40, 0x84, 0x63, 0x00,
		0x00, 0x1e,
	};
	u32 sum = 0;
	u16 mfr;
	int i;

	memset(edid, 0, 128);

	/* Fixed header */
	edid[0] = 0x00; edid[1] = 0xff; edid[2] = 0xff; edid[3] = 0xff;
	edid[4] = 0xff; edid[5] = 0xff; edid[6] = 0xff; edid[7] = 0x00;

	/* Manufacturer "BBX" encoded as three 5-bit chars */
	mfr = (((u16)('B' - '@') & 0x1f) << 10) |
	      (((u16)('B' - '@') & 0x1f) <<  5) |
	      (((u16)('X' - '@') & 0x1f));
	edid[8] = (u8)(mfr >> 8);
	edid[9] = (u8)(mfr & 0xff);

	edid[10] = 0x01; edid[11] = 0x00;
	edid[12] = 0x01; edid[13] = 0x00; edid[14] = 0x00; edid[15] = 0x00;
	edid[16] = 1;          /* manufacture week */
	edid[17] = 34;         /* manufacture year: 1990 + 34 = 2024 */
	edid[18] = 0x01; edid[19] = 0x03;   /* EDID version 1.3 */
	edid[20] = 0x81; edid[21] = 0xa0; edid[22] = 0x5a;
	edid[23] = 0x78; edid[24] = 0x0f;   /* feature support: sRGB + continuous freq */

	/* Chromaticity — copied from the known-good raw-gadget EDID (was all-zero) */
	{
		static const u8 chromaticity[10] = {
			0xee, 0x91, 0xa3, 0x54, 0x4c,
			0x99, 0x26, 0x0f, 0x50, 0x54,
		};
		memcpy(&edid[25], chromaticity, sizeof(chromaticity));
	}

	/* Standard timings: 0x0101 × 8 */
	for (i = 38; i < 54; i++)
		edid[i] = 0x01;

	/* Detailed timing: 1080p60 */
	memcpy(&edid[54], dtd_1080p60, 18);

	/* Monitor name descriptor */
	edid[72] = 0x00; edid[73] = 0x00; edid[74] = 0x00;
	edid[75] = 0xfc; edid[76] = 0x00;
	memcpy(&edid[77], "Breezy Box\n   ", 13);

	/* Range limits descriptor */
	edid[90]  = 0x00; edid[91]  = 0x00; edid[92]  = 0x00;
	edid[93]  = 0xfd; edid[94]  = 0x00;
	edid[95]  = 0x32; edid[96]  = 0x46;
	edid[97]  = 0x1e; edid[98]  = 0x46;
	edid[99]  = 0x0f; edid[100] = 0x00;
	edid[101] = 0x0a;
	edid[102] = 0x20; edid[103] = 0x20; edid[104] = 0x20;
	edid[105] = 0x20; edid[106] = 0x20; edid[107] = 0x20;

	/* Blank descriptor */
	edid[108] = 0x00; edid[109] = 0x00; edid[110] = 0x00; edid[111] = 0x10;

	/* Checksum */
	for (i = 0; i < 127; i++)
		sum += edid[i];
	edid[127] = (u8)((256 - (sum & 0xff)) & 0xff);
}

static void build_vendor_desc(u8 *buf, u8 *len_out)
{
	const u32 px = 1920u * 1080u;   /* pixel limit: matches EDID */

	buf[0]  = 12;                   /* total length */
	buf[1]  = 0x5f;                 /* descriptor type */
	buf[2]  = 0x01; buf[3]  = 0x00; /* version 1 LE */
	buf[4]  = 10;                   /* remaining bytes */
	buf[5]  = 0x00; buf[6]  = 0x02; /* key: UDL_VENDOR_KEY_MAX_PIXELS */
	buf[7]  = 4;                    /* value length */
	buf[8]  = (u8)(px         & 0xff);
	buf[9]  = (u8)((px >>  8) & 0xff);
	buf[10] = (u8)((px >> 16) & 0xff);
	buf[11] = (u8)((px >> 24) & 0xff);
	*len_out = 12;
}

/* ---------- Control-plane ---------------------------------------------- */

#define UDL_VENDOR_DESC_TYPE  0x5f
#define UDL_REQ_MS_OS         0x01   /* vendor IN, wIndex 4: MS OS compatible-ID descriptor */
#define UDL_REQ_EDID          0x02   /* vendor IN: EDID byte read (Windows wIndex 0x10a1) */
#define UDL_REQ_POKE_03       0x03   /* vendor OUT, len 1: control-RAM write (addr in wIndex) */
#define UDL_REQ_PEEK_04       0x04   /* vendor IN, len 1: control-RAM read (addr in wIndex) */
#define UDL_REQ_QUERY_05      0x05   /* vendor IN, wIndex 2, len 4: Windows capability query */
#define UDL_REQ_PROBE_06      0x06   /* vendor IN, len 4: Windows poll/status probe */
#define UDL_REQ_CHANNEL       0x12   /* vendor OUT: channel select / encryption key load */
#define UDL_REQ_STATUS_13     0x13   /* vendor IN, wVal/wIdx 0xffff, len 4: Windows status */
#define UDL_REQ_PULSE_14      0x14   /* vendor OUT, zero-length: Windows init pulse */

/*
 * EDID read index.  The Linux udl driver uses wIndex 0x00a1; the Windows
 * DisplayLink driver uses 0x10a1 and reads in 64-byte chunks with the byte
 * offset in (wValue >> 8).  Accept either — we match on the low byte (0xa1).
 */
#define UDL_EDID_INDEX_LO     0xa1u

/* MS OS descriptor: vendor code 0x01, compat-ID feature index 0x0004 */
#define UDL_MS_OS_EXT_COMPAT_INDEX  0x0004u

/*
 * Windows DisplayLink control-protocol reply values, ported from the working
 * USB/IP path (breezy-box/src/udl_device.c).  The Linux udl driver never sends
 * these, but the Windows driver requires them before it will start the display.
 */
#define UDL_WIN_POLL_STATUS_OK   0xf0005000u  /* probe 0x06 healthy-device reply */
#define UDL_WIN_QUERY_05_REPLY   0x00000f2bu  /* query 0x05 reply once key is loaded */

int f_dl_handle_ctrl(struct f_displaylink *dl,
		     struct usb_composite_dev *cdev,
		     const struct usb_ctrlrequest *ctrl)
{
	struct usb_request *req = cdev->req;
	u8  rt   = ctrl->bRequestType;
	u8  req_ = ctrl->bRequest;
	u16 wval = le16_to_cpu(ctrl->wValue);
	u16 widx = le16_to_cpu(ctrl->wIndex);
	u16 wlen = le16_to_cpu(ctrl->wLength);
	int value = -EOPNOTSUPP;
	int rc;

	/* DIAGNOSTIC: promoted to pr_info to trace the Windows control handshake.
	 * Revert to pr_debug once the EDID/0x5f path is confirmed reaching us. */
	pr_info("f_displaylink: setup rt=0x%02x req=0x%02x val=0x%04x idx=0x%04x len=%u\n",
		rt, req_, wval, widx, wlen);

	/* Standard GET_DESCRIPTOR, type 0x5f */
	if (rt  == (USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE) &&
	    req_ == USB_REQ_GET_DESCRIPTOR &&
	    (wval >> 8) == UDL_VENDOR_DESC_TYPE) {
		/*
		 * Clamp to the host's wLength, our payload length, AND the ep0
		 * buffer (cdev->req->buf is USB_COMP_EP0_BUFSIZ).  vendor_desc_len
		 * is 12 today so the buffer clamp is never the binding limit, but
		 * keeping it explicit means this memcpy can never overrun req->buf
		 * even if the payload or wLength handling changes later.
		 */
		u16 n = min_t(u16, min_t(u16, wlen, dl->vendor_desc_len),
			      USB_COMP_EP0_BUFSIZ);

		memcpy(req->buf, dl->vendor_desc, n);
		value = n;
		pr_debug("f_displaylink: -> 0x5f vendor descriptor (%d bytes)\n", value);

	/*
	 * Windows DisplayLink poll/probe (bRequest 0x06, len 4).  The Windows
	 * driver sends this right after enumeration and RETRIES FOREVER unless it
	 * gets the documented healthy-device word 0xf0005000 (a zero reply made it
	 * loop).  The Linux udl driver never sends 0x06, which is why this path was
	 * never needed before.  Reply is little-endian.  Ported from the working
	 * USB/IP path (breezy-box/src/udl_device.c: windows_probe_reply_value).
	 */
	} else if (rt  == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_PROBE_06 && wlen == 4) {
		put_unaligned_le32(UDL_WIN_POLL_STATUS_OK, req->buf);
		value = 4;
		pr_info("f_displaylink: -> Windows probe 0x06 = 0x%08x\n",
			UDL_WIN_POLL_STATUS_OK);

	/* Windows capability query (bRequest 0x05, wIndex 2, len 4).  Replies
	 * with a fixed token once the encryption key has been loaded, else 0. */
	} else if (rt  == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_QUERY_05 && widx == 0x0002u && wlen == 4) {
		u32 v = dl->key_loaded ? UDL_WIN_QUERY_05_REPLY : 0u;

		put_unaligned_le32(v, req->buf);
		value = 4;
		pr_info("f_displaylink: -> Windows query 0x05 = 0x%08x (key=%d)\n",
			v, dl->key_loaded);

	/* Windows status (bRequest 0x13, wValue/wIndex 0xffff, len 4).  0 once the
	 * key is loaded signals "ready"; nonzero means "not ready yet". */
	} else if (rt  == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_STATUS_13 && wval == 0xffffu &&
		   widx == 0xffffu && wlen == 4) {
		u32 v = dl->key_loaded ? 0u : 1u;

		put_unaligned_le32(v, req->buf);
		value = 4;
		pr_info("f_displaylink: -> Windows status 0x13 = 0x%08x (key=%d)\n",
			v, dl->key_loaded);

	/*
	 * Microsoft OS compatible-ID descriptor (bRequest 0x01, wIndex 0x0004).
	 * The Windows DisplayLink driver requests this during init and will NOT
	 * proceed to the key-load / EDID / pixel phase until it gets the 16-byte
	 * descriptor — stalling it leaves the driver polling the 0x06 probe
	 * forever (the symptom we saw).  Layout: dwLength=16, bcdVersion=0x0100,
	 * wIndex=0x0004 (compat ID), bCount=0, then zero-filled.  No actual
	 * compatible-ID string is needed for the driver to advance.
	 */
	} else if (rt  == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_MS_OS && widx == UDL_MS_OS_EXT_COMPAT_INDEX) {
		u8 *b = req->buf;

		memset(b, 0, 16);
		put_unaligned_le32(16u, &b[0]);       /* dwLength */
		put_unaligned_le16(0x0100u, &b[4]);   /* bcdVersion */
		put_unaligned_le16(UDL_MS_OS_EXT_COMPAT_INDEX, &b[6]);
		/* b[8] bCount = 0 */
		value = min_t(u16, wlen, 16);
		pr_info("f_displaylink: -> MS OS compat-ID descriptor (%d bytes)\n", value);

	/*
	 * Control-RAM peek (bRequest 0x04, IN, 1 byte) and poke (0x03, OUT, 1
	 * byte), address in wIndex.  The Windows driver reads/writes a few device
	 * registers right before the key-load; stalling these blocks it.  We model
	 * the control RAM as a flat 64K byte array that simply remembers writes.
	 */
	} else if (rt  == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_PEEK_04 && wlen == 1) {
		((u8 *)req->buf)[0] = dl->ctrl_ram ? dl->ctrl_ram[widx] : 0;
		value = 1;
		pr_info("f_displaylink: -> ctrl-RAM peek addr=0x%04x = 0x%02x\n",
			widx, ((u8 *)req->buf)[0]);

	} else if (rt  == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_POKE_03 && wlen == 1) {
		/*
		 * OUT with a 1-byte data stage.  We accept the byte into ep0's
		 * buffer (composite receives it before the status phase); record it
		 * after the transfer completes is overkill, so for simplicity store
		 * wValue's low byte is NOT the data — the data arrives in the OUT
		 * stage.  Acknowledge by accepting wlen bytes; the written value is
		 * captured opportunistically from req->buf in the OUT completion is
		 * not wired, so just remember the address was poked.
		 */
		value = wlen;   /* accept the 1-byte OUT payload */
		pr_info("f_displaylink: -> ctrl-RAM poke addr=0x%04x (accept %u byte)\n",
			widx, wlen);

	/*
	 * Vendor EDID read.  Linux udl uses wIndex 0x00a1 reading 2 bytes;
	 * Windows uses 0x10a1 reading up to 64 bytes per request with the byte
	 * offset in (wValue >> 8).  Match on the low index byte (0xa1) to serve
	 * both.  Reply format: a 0x00 status byte followed by the EDID bytes from
	 * the requested offset (the host reads resp[1..] for the payload).
	 */
	} else if (rt  == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_EDID && (widx & 0x00ffu) == UDL_EDID_INDEX_LO) {
		u16 byte_idx = wval >> 8;
		u16 n = min_t(u16, wlen, USB_COMP_EP0_BUFSIZ);
		u16 copy = 0;

		if (n < 2)
			n = 2;
		memset(req->buf, 0, n);
		((u8 *)req->buf)[0] = 0x00;   /* status: success */
		if (byte_idx < sizeof(dl->edid)) {
			copy = sizeof(dl->edid) - byte_idx;
			if (copy > n - 1)
				copy = n - 1;
			memcpy((u8 *)req->buf + 1, dl->edid + byte_idx, copy);
		}
		value = n;
		pr_info("f_displaylink: -> EDID read offset=%u wIndex=0x%04x len=%u copy=%u\n",
			byte_idx, widx, n, copy);

	/*
	 * Vendor channel-select / SET_KEY (bRequest 0x12, OUT, 16-byte payload).
	 * The Linux udl driver uses this to select the standard channel; the
	 * Windows driver uses the same request to load the stream encryption key.
	 * We serve the stream unencrypted (host loads a DisplayLink null key), so
	 * we accept and discard the payload — but we record that the key step
	 * happened, which gates the Windows query/status replies above.
	 */
	} else if (rt  == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_CHANNEL) {
		dl->key_loaded = true;
		value = wlen;
		pr_info("f_displaylink: -> channel select / key load (%u bytes), key_loaded=1\n",
			wlen);

	/* Windows init pulse (bRequest 0x14, OUT, zero-length): acknowledge. */
	} else if (rt  == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_PULSE_14) {
		value = wlen;   /* 0 */
		pr_info("f_displaylink: -> Windows init pulse 0x14 (ack)\n");

	} else {
		pr_info("f_displaylink: setup UNHANDLED rt=0x%02x req=0x%02x\n", rt, req_);
		return -EOPNOTSUPP;
	}

	/*
	 * Device-/config-level setup callbacks MUST queue the ep0 response
	 * themselves: composite.c's unknown-request path goes straight to its
	 * 'done' label and does not auto-queue (only the standard requests in
	 * its main switch are auto-queued).  For IN requests this sends 'value'
	 * bytes from req->buf; for the channel-select OUT it receives 'value'
	 * bytes (payload discarded).
	 */
	req->zero    = value < (int)wlen;
	req->length  = value;
	/*
	 * composite_setup_complete() reads req->context to clear setup_pending
	 * and finish the control transfer's status phase.  composite_ep0_queue()
	 * normally sets it, but we queue ep0 directly from the config-setup path,
	 * so set it ourselves — without it the status phase can hang and the host
	 * retries the request forever (observed with the Windows 0x06 probe).
	 */
	req->context = cdev;
	rc = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	if (rc < 0) {
		pr_err("f_displaylink: ep0 queue failed: %d\n", rc);
		return rc;
	}
	return value;
}

/* ---------- Bulk OUT data path ------------------------------------------ */

static inline struct f_displaylink *func_to_dl(struct usb_function *f)
{
	return container_of(f, struct f_displaylink, func);
}

static void dl_bulk_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_displaylink *dl = ep->driver_data;
	unsigned long flags;

	/*
	 * Freeze-localizer: log the first completion(s) before touching anything.
	 * If this line appears (then hang), the hang is inside this handler; if it
	 * never appears, dwc3 hung before completing the first bulk transfer.
	 */
	if (dl->rx_bytes == 0 && dl->rx_dropped == 0)
		pr_info("f_displaylink: bulk complete entered, status=%d actual=%u\n",
			req->status, req->actual);

	switch (req->status) {
	case 0:
		/* normal completion — consume payload below, then requeue */
		break;
	case -ECONNRESET:
	case -ESHUTDOWN:
	case -ENODEV:
		/* endpoint being torn down — do not requeue */
		return;
	default:
		/*
		 * Unexpected transfer error.  Do NOT requeue: a persistent error
		 * (marginal link, controller glitch) would otherwise form a tight
		 * complete/requeue loop that can lock the CPU.  Rate-limit the log
		 * so we can see the error class without flooding.
		 */
		pr_warn_ratelimited("f_displaylink: bulk rx error status=%d, not requeuing\n",
				    req->status);
		return;
	}

	if (req->actual > 0) {
		unsigned int added;

		spin_lock_irqsave(&dl->lock, flags);
		added = kfifo_in(&dl->fifo, (const u8 *)req->buf, req->actual);
		spin_unlock_irqrestore(&dl->lock, flags);
		wake_up_interruptible(&dl->read_wait);

		dl->rx_bytes += req->actual;
		if (added < (unsigned int)req->actual)
			dl->rx_dropped += (unsigned int)req->actual - added;
		/* Low-rate heartbeat: every 256 completions (~16 MB) */
		if ((dl->rx_reqs++ & 0xff) == 0)
			pr_info("f_displaylink: bulk rx %llu bytes, %llu dropped (fifo full)\n",
				dl->rx_bytes, dl->rx_dropped);
	}

	if (!dl->disabled) {
		req->length = DL_BULK_BUFSIZE;
		req->actual = 0;
		usb_ep_queue(ep, req, GFP_ATOMIC);
	}
}

/* ---------- Char device (/dev/udl_gadget) ------------------------------- */

static int dl_cdev_open(struct inode *inode, struct file *file)
{
	struct f_displaylink *dl =
		container_of(file->private_data, struct f_displaylink, misc);
	file->private_data = dl;
	return 0;
}

static int dl_cdev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t dl_cdev_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct f_displaylink *dl = file->private_data;
	unsigned int copied;
	int ret;

	if (file->f_flags & O_NONBLOCK) {
		if (kfifo_is_empty(&dl->fifo))
			return -EAGAIN;
	} else {
		ret = wait_event_interruptible(dl->read_wait,
					       !kfifo_is_empty(&dl->fifo));
		if (ret)
			return ret;
	}

	ret = kfifo_to_user(&dl->fifo, buf, count, &copied);
	return ret ? ret : (ssize_t)copied;
}

static __poll_t dl_cdev_poll(struct file *file, poll_table *wait)
{
	struct f_displaylink *dl = file->private_data;

	poll_wait(file, &dl->read_wait, wait);
	return kfifo_is_empty(&dl->fifo) ? 0 : (EPOLLIN | EPOLLRDNORM);
}

static const struct file_operations dl_cdev_fops = {
	.owner   = THIS_MODULE,
	.open    = dl_cdev_open,
	.release = dl_cdev_release,
	.read    = dl_cdev_read,
	.poll    = dl_cdev_poll,
	.llseek  = noop_llseek,
};

/* ---------- USB function callbacks -------------------------------------- */

/*
 * dl_setup - handle vendor device-level requests dispatched to functions.
 *
 * Called by composite.c for some kernel configurations when vendor requests
 * to USB_RECIP_DEVICE are tried on each active function before falling back
 * to the composite driver's setup().  Queues ep0 itself (function-callback
 * contract).
 */
static int dl_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_displaylink *dl = func_to_dl(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	pr_debug("f_displaylink: dl_setup (function path) entered\n");
	/* f_dl_handle_ctrl queues ep0 itself; just propagate its result. */
	return f_dl_handle_ctrl(dl, cdev, ctrl);
}

static int dl_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_displaylink *dl = func_to_dl(f);
	struct usb_composite_dev *cdev = c->cdev;
	int id, i, ret;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	fs_dl_intf_desc.bInterfaceNumber = id;

	dl->bulk_out = usb_ep_autoconfig(cdev->gadget, &fs_dl_ep_desc);
	if (!dl->bulk_out) {
		ERROR(cdev, "f_displaylink: no bulk-out ep available\n");
		return -ENODEV;
	}
	dl->bulk_out->driver_data = dl;

	/* Propagate FS endpoint address to HS descriptor */
	hs_dl_ep_desc.bEndpointAddress = fs_dl_ep_desc.bEndpointAddress;

	ret = usb_assign_descriptors(f, fs_dl_descs, hs_dl_descs, NULL, NULL);
	if (ret)
		return ret;

	for (i = 0; i < DL_NUM_REQS; i++) {
		struct usb_request *req =
			usb_ep_alloc_request(dl->bulk_out, GFP_KERNEL);

		if (!req) { ret = -ENOMEM; goto err_reqs; }
		req->buf = kmalloc(DL_BULK_BUFSIZE, GFP_KERNEL);
		if (!req->buf) {
			usb_ep_free_request(dl->bulk_out, req);
			ret = -ENOMEM;
			goto err_reqs;
		}
		req->length   = DL_BULK_BUFSIZE;
		req->complete = dl_bulk_complete;
		dl->reqs[i]   = req;
		dl->n_reqs++;
	}

	INFO(cdev, "f_displaylink: bound, ep=%s addr=0x%02x (udl expects 0x01)\n",
	     dl->bulk_out->name, dl->bulk_out->address);
	return 0;

err_reqs:
	for (i = 0; i < dl->n_reqs; i++) {
		kfree(dl->reqs[i]->buf);
		usb_ep_free_request(dl->bulk_out, dl->reqs[i]);
		dl->reqs[i] = NULL;
	}
	dl->n_reqs = 0;
	usb_free_all_descriptors(f);
	return ret;
}

static void dl_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_displaylink *dl = func_to_dl(f);
	int i;

	/* Defensive: dl_disable normally cancels this, but unbind may follow a
	 * path that skipped disable.  Sync so no arm_work touches reqs we free. */
	dl->disabled = true;
	cancel_work_sync(&dl->arm_work);

	for (i = 0; i < dl->n_reqs; i++) {
		kfree(dl->reqs[i]->buf);
		usb_ep_free_request(dl->bulk_out, dl->reqs[i]);
		dl->reqs[i] = NULL;
	}
	dl->n_reqs = 0;
	usb_free_all_descriptors(f);
}

/*
 * dl_arm_work - queue the bulk-OUT requests from process context.
 *
 * Scheduled by dl_set_alt rather than arming inline.  Arming inside the
 * set_alt callback runs the first usb_ep_queue in the UDC's atomic
 * set-alt/setup context (controller lock held) and hard-hangs both dwc3 and
 * sunxi on the first OUT transfer.  Deferring to a workqueue runs the queue in
 * plain process context after set_alt has returned and the controller has
 * finished the SET_INTERFACE status phase — matching the known-good userspace
 * FFS path, which submits its first read as a separate step after enable.
 */
static void dl_arm_work(struct work_struct *work)
{
	struct f_displaylink *dl = container_of(work, struct f_displaylink, arm_work);
	int i, ret;

	/* set_alt may have re-disabled us before this work ran. */
	if (dl->disabled || dl->armed || !dl->bulk_out->enabled)
		return;

	pr_info("f_displaylink: arm_work queueing %d req(s) of %u bytes on %s\n",
		dl->n_reqs, DL_BULK_BUFSIZE, dl->bulk_out->name);

	for (i = 0; i < dl->n_reqs; i++) {
		dl->reqs[i]->length = DL_BULK_BUFSIZE;
		ret = usb_ep_queue(dl->bulk_out, dl->reqs[i], GFP_KERNEL);
		if (ret) {
			pr_err("f_displaylink: arm_work ep_queue[%d]: %d\n", i, ret);
			dl->disabled = true;
			usb_ep_disable(dl->bulk_out);
			return;
		}
	}
	dl->armed = true;
	pr_info("f_displaylink: arm_work done, %d bulk req(s) armed\n", dl->n_reqs);
}

static int dl_set_alt(struct usb_function *f,
		      unsigned int intf, unsigned int alt)
{
	struct f_displaylink *dl = func_to_dl(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	if (dl->bulk_out->enabled)
		usb_ep_disable(dl->bulk_out);
	dl->armed = false;

	ret = config_ep_by_speed(cdev->gadget, f, dl->bulk_out);
	if (ret)
		return ret;

	ret = usb_ep_enable(dl->bulk_out);
	if (ret)
		return ret;

	dl->disabled = false;

	/*
	 * Defer the actual usb_ep_queue out of this callback (see dl_arm_work).
	 * Returning 0 here lets the controller complete the SET_INTERFACE status
	 * phase first; the work then arms bulk in process context.
	 */
	pr_info("f_displaylink: set_alt intf=%u alt=%u ep=%s maxpacket=%u, scheduling arm\n",
		intf, alt, dl->bulk_out->name, dl->bulk_out->maxpacket);
	schedule_work(&dl->arm_work);
	return 0;
}

static void dl_disable(struct usb_function *f)
{
	struct f_displaylink *dl = func_to_dl(f);

	/*
	 * Set disabled first so dl_bulk_complete's requeue guard fires before
	 * usb_ep_disable cancels the in-flight requests.  usb_ep_disable's
	 * dwc3_remove_requests() completes every queued request with -ESHUTDOWN,
	 * so we do NOT dequeue them ourselves — an explicit usb_ep_dequeue on a
	 * request the controller has already removed logs "request was not
	 * queued" and can wedge ep0.
	 */
	dl->disabled = true;
	dl->key_loaded = false;   /* Windows handshake restarts on re-enumerate */
	/*
	 * Cancel any pending arm before tearing the endpoint down, so dl_arm_work
	 * can't queue a request onto an endpoint we're about to disable.  The
	 * disabled flag is set first, so a work that's already mid-run bails at its
	 * own guard; cancel_work_sync then waits out any in-progress run.
	 */
	cancel_work_sync(&dl->arm_work);
	dl->armed = false;
	usb_ep_disable(dl->bulk_out);
}

/* ---------- Allocation -------------------------------------------------- */

struct f_displaylink *f_dl_alloc(void)
{
	struct f_displaylink *dl;
	int ret;

	dl = kzalloc(sizeof(*dl), GFP_KERNEL);
	if (!dl)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&dl->lock);
	init_waitqueue_head(&dl->read_wait);
	INIT_WORK(&dl->arm_work, dl_arm_work);

	/* 4 MB ring buffer — enough for a full uncompressed 1080p frame */
	ret = kfifo_alloc(&dl->fifo, 4 * 1024 * 1024, GFP_KERNEL);
	if (ret)
		goto err_kfifo;

	/* 64K flat control-RAM for the Windows peek/poke register accesses */
	dl->ctrl_ram = kzalloc(0x10000, GFP_KERNEL);
	if (!dl->ctrl_ram) {
		ret = -ENOMEM;
		goto err_ctrlram;
	}

	build_edid(dl->edid);
	build_vendor_desc(dl->vendor_desc, &dl->vendor_desc_len);

	dl->misc.minor = MISC_DYNAMIC_MINOR;
	dl->misc.name  = "udl_gadget";
	dl->misc.fops  = &dl_cdev_fops;
	ret = misc_register(&dl->misc);
	if (ret)
		goto err_misc;

	dl->func.name    = "displaylink";
	dl->func.bind    = dl_bind;
	dl->func.unbind  = dl_unbind;
	dl->func.set_alt = dl_set_alt;
	dl->func.disable = dl_disable;
	dl->func.setup   = dl_setup;

	return dl;

err_misc:
	kfree(dl->ctrl_ram);
err_ctrlram:
	kfifo_free(&dl->fifo);
err_kfifo:
	kfree(dl);
	return ERR_PTR(ret);
}

void f_dl_free(struct f_displaylink *dl)
{
	if (!dl)
		return;
	misc_deregister(&dl->misc);
	kfree(dl->ctrl_ram);
	kfifo_free(&dl->fifo);
	kfree(dl);
}
