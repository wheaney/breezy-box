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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

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
#define UDL_REQ_EDID          0x02
#define UDL_REQ_CHANNEL       0x12
#define UDL_EDID_INDEX        0x00a1u

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

	pr_debug("f_displaylink: setup rt=0x%02x req=0x%02x val=0x%04x idx=0x%04x len=%u\n",
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

	/* Vendor EDID byte read: returns [0x00, edid[byte_index]] */
	} else if (rt  == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_EDID && widx == UDL_EDID_INDEX) {
		u8  resp[2]  = { 0, 0 };
		u16 byte_idx = wval >> 8;

		if (byte_idx < sizeof(dl->edid))
			resp[1] = dl->edid[byte_idx];
		memcpy(req->buf, resp, sizeof(resp));
		value = min_t(u16, wlen, 2);
		/* Mark only first/last byte so we can see if the full block reads */
		if (byte_idx == 0 || byte_idx == sizeof(dl->edid) - 1)
			pr_info("f_displaylink: EDID read byte %u\n", byte_idx);
		else
			pr_debug("f_displaylink: -> EDID byte %u = 0x%02x\n", byte_idx, resp[1]);

	/* Vendor channel select: OUT, 16-byte key payload we accept and ignore */
	} else if (rt  == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
		   req_ == UDL_REQ_CHANNEL) {
		value = wlen;
		pr_debug("f_displaylink: -> channel select (accept %u bytes)\n", wlen);

	} else {
		pr_debug("f_displaylink: setup UNHANDLED rt=0x%02x req=0x%02x\n", rt, req_);
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
	req->zero   = value < (int)wlen;
	req->length = value;
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

	for (i = 0; i < dl->n_reqs; i++) {
		kfree(dl->reqs[i]->buf);
		usb_ep_free_request(dl->bulk_out, dl->reqs[i]);
		dl->reqs[i] = NULL;
	}
	dl->n_reqs = 0;
	usb_free_all_descriptors(f);
}

static int dl_set_alt(struct usb_function *f,
		      unsigned int intf, unsigned int alt)
{
	struct f_displaylink *dl = func_to_dl(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int i, ret;

	if (dl->bulk_out->enabled)
		usb_ep_disable(dl->bulk_out);

	ret = config_ep_by_speed(cdev->gadget, f, dl->bulk_out);
	if (ret)
		return ret;

	ret = usb_ep_enable(dl->bulk_out);
	if (ret)
		return ret;

	dl->disabled = false;

	pr_info("f_displaylink: set_alt intf=%u alt=%u ep=%s maxpacket=%u, queueing %d req(s) of %u bytes\n",
		intf, alt, dl->bulk_out->name, dl->bulk_out->maxpacket,
		dl->n_reqs, DL_BULK_BUFSIZE);

	for (i = 0; i < dl->n_reqs; i++) {
		dl->reqs[i]->length = DL_BULK_BUFSIZE;
		ret = usb_ep_queue(dl->bulk_out, dl->reqs[i], GFP_ATOMIC);
		if (ret) {
			ERROR(cdev, "f_displaylink: ep_queue[%d]: %d\n", i, ret);
			dl->disabled = true;
			usb_ep_disable(dl->bulk_out);
			return ret;
		}
	}
	pr_info("f_displaylink: set_alt done, %d bulk req(s) armed\n", dl->n_reqs);
	return 0;
}

static void dl_disable(struct usb_function *f)
{
	struct f_displaylink *dl = func_to_dl(f);

	dl->disabled = true;
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

	/* 4 MB ring buffer — enough for a full uncompressed 1080p frame */
	ret = kfifo_alloc(&dl->fifo, 4 * 1024 * 1024, GFP_KERNEL);
	if (ret)
		goto err_kfifo;

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
	kfifo_free(&dl->fifo);
	kfree(dl);
}
