// SPDX-License-Identifier: GPL-2.0
/*
 * g_rndis_displaylink.c - Composite USB gadget: RNDIS + DisplayLink
 *
 * Enumerates as VID=0x17e9 / PID=0x0104 (DisplayLink) with two functions:
 *
 *   RNDIS  - creates usb0; the KMS renderer's USB/IP server listens on this
 *            interface so a host can enumerate additional virtual displays.
 *
 *   DisplayLink - exposes /dev/udl_gadget; the KMS renderer reads the raw
 *                 bulk command stream and renders it through DRM/KMS.
 *
 * Device-level control requests that the composite framework does not handle
 * natively are caught in gdl_setup():
 *   - GET_DESCRIPTOR type 0x5f  → DisplayLink vendor descriptor
 *   - Vendor IN 0x02            → EDID byte read
 *   - Vendor OUT 0x12           → channel select (acknowledged, ignored)
 *
 * Requires usb_f_rndis.ko (kernel CONFIG_USB_F_RNDIS=m).
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb/composite.h>

#include "f_displaylink.h"

MODULE_DESCRIPTION("RNDIS + DisplayLink composite USB gadget");
MODULE_AUTHOR("breezy-box");
MODULE_LICENSE("GPL v2");
MODULE_SOFTDEP("pre: usb_f_rndis");

#define DL_VENDOR_ID   0x17e9
#define DL_PRODUCT_ID  0x0104

/*
 * disable_rndis: when set, present a PURE DisplayLink device — a single
 * vendor-class interface, no RNDIS — matching the known-good DL-only
 * raw-gadget reference.  The Windows DisplayLink driver only promotes the
 * device to an active display for a clean single-interface device; the
 * RNDIS composite makes it health-poll forever without creating a display.
 * Toggle without a rebuild:  modprobe g_rndis_displaylink disable_rndis=1
 */
static bool disable_rndis;
module_param(disable_rndis, bool, 0444);
MODULE_PARM_DESC(disable_rndis, "Present DisplayLink only, no RNDIS (Windows display test)");

/* ---------- USB strings ------------------------------------------------- */

#define STR_MANUFACTURER  0
#define STR_PRODUCT       1
#define STR_SERIAL        2

static struct usb_string gdl_usb_strings[] = {
	[STR_MANUFACTURER].s = "DisplayLink",
	[STR_PRODUCT].s      = "Breezy Box",
	[STR_SERIAL].s       = "BREEZY0002",
	{ }
};

static struct usb_gadget_strings gdl_strtab = {
	.language = 0x0409,   /* en-US */
	.strings  = gdl_usb_strings,
};

static struct usb_gadget_strings *gdl_strtabs[] = {
	&gdl_strtab,
	NULL,
};

/* ---------- Device descriptor ------------------------------------------- */

static struct usb_device_descriptor gdl_device_desc = {
	.bLength            = USB_DT_DEVICE_SIZE,
	.bDescriptorType    = USB_DT_DEVICE,
	.bcdUSB             = cpu_to_le16(0x0200),
	/*
	 * bDeviceClass = 0x00 (per-interface), matching the known-good DL-only
	 * raw-gadget reference (displaylink_gadget_raw_gadget.c) and a real
	 * DisplayLink device.  The Windows DisplayLink driver only promotes the
	 * device to an active display when it sees this class; with 0xEF (Misc/
	 * IAD, needed to pair RNDIS) the driver binds but health-polls forever and
	 * never creates a display.
	 *
	 * TRADE-OFF: without the IAD, Windows cannot pair the RNDIS control+data
	 * interfaces, so RNDIS networking will not bind on Windows in this mode.
	 * This is the deliberate test for whether the device class is what gates
	 * DisplayLink activation.
	 */
	.bDeviceClass       = USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass    = 0x00,
	.bDeviceProtocol    = 0x00,
	.bMaxPacketSize0    = 64,
	.idVendor           = cpu_to_le16(DL_VENDOR_ID),
	.idProduct          = cpu_to_le16(DL_PRODUCT_ID),
	.bcdDevice          = cpu_to_le16(0x0001),
	/* iManufacturer / iProduct / iSerialNumber filled in gdl_bind */
	.bNumConfigurations = 1,
};

/* ---------- Module-level instances -------------------------------------- */

static struct f_displaylink          *g_dl;
static struct usb_function_instance  *g_fi_rndis;
static struct usb_function           *g_f_rndis;

/* ---------- Configuration setup and bind -------------------------------- */

/*
 * gdl_config_setup - handle DisplayLink device-level control requests.
 *
 * composite.c routes here (via usb_configuration.setup) any request that
 * the standard framework does not handle: unknown GET_DESCRIPTOR types
 * (0x5f), vendor IN/OUT to USB_RECIP_DEVICE, etc.  Fill cdev->req->buf and
 * return the byte count; composite.c queues ep0.
 */
static int gdl_config_setup(struct usb_configuration *c,
			    const struct usb_ctrlrequest *ctrl)
{
	int value;

	pr_debug("g_rndis_displaylink: gdl_config_setup (config path) entered\n");
	value = f_dl_handle_ctrl(g_dl, c->cdev, ctrl);
	pr_debug("g_rndis_displaylink: gdl_config_setup -> %d\n", value);
	return value;
}

static int gdl_config_bind(struct usb_configuration *c)
{
	int ret;

	/*
	 * Add DisplayLink FIRST so its bulk-OUT endpoint autoconfigures to the
	 * lowest free OUT endpoint (ep1out / address 0x01).  The host's in-kernel
	 * udl driver hardcodes the bulk pipe to OUT endpoint 1
	 * (usb_sndbulkpipe(udev, 1)) and ignores the descriptor's address, so the
	 * DisplayLink bulk-OUT MUST live at 0x01 or no pixel data is delivered.
	 * If RNDIS is added first it grabs ep1 and DisplayLink lands on ep2out.
	 */
	ret = usb_add_function(c, &g_dl->func);
	if (ret) {
		pr_err("g_rndis_displaylink: usb_add_function(displaylink) = %d\n", ret);
		return ret;
	}

	if (disable_rndis) {
		pr_info("g_rndis_displaylink: DisplayLink-only mode (RNDIS disabled)\n");
		return 0;
	}

	g_f_rndis = usb_get_function(g_fi_rndis);
	if (IS_ERR(g_f_rndis)) {
		ret = PTR_ERR(g_f_rndis);
		g_f_rndis = NULL;
		pr_err("g_rndis_displaylink: usb_get_function(rndis) = %d\n", ret);
		usb_remove_function(c, &g_dl->func);
		return ret;
	}

	ret = usb_add_function(c, g_f_rndis);
	if (ret) {
		pr_err("g_rndis_displaylink: usb_add_function(rndis) = %d\n", ret);
		usb_put_function(g_f_rndis);
		g_f_rndis = NULL;
		usb_remove_function(c, &g_dl->func);
		return ret;
	}

	return 0;
}

static struct usb_configuration gdl_config = {
	.label               = "RNDIS + DisplayLink",
	.bConfigurationValue = 1,
	.bmAttributes        = USB_CONFIG_ATT_ONE,
	.MaxPower            = 500,   /* mA */
	.setup               = gdl_config_setup,
};

/* ---------- Composite driver bind/unbind -------------------------------- */

static int gdl_bind(struct usb_composite_dev *cdev)
{
	int ret;

	ret = usb_string_ids_tab(cdev, gdl_usb_strings);
	if (ret < 0)
		return ret;

	gdl_device_desc.iManufacturer = gdl_usb_strings[STR_MANUFACTURER].id;
	gdl_device_desc.iProduct      = gdl_usb_strings[STR_PRODUCT].id;
	gdl_device_desc.iSerialNumber = gdl_usb_strings[STR_SERIAL].id;

	ret = usb_add_config(cdev, &gdl_config, gdl_config_bind);
	if (ret) {
		pr_err("g_rndis_displaylink: usb_add_config = %d\n", ret);
		return ret;
	}

	pr_info("g_rndis_displaylink: registered (VID=%04x PID=%04x)\n",
		DL_VENDOR_ID, DL_PRODUCT_ID);
	return 0;
}

static int gdl_unbind(struct usb_composite_dev *cdev)
{
	return 0;
}

static struct usb_composite_driver gdl_driver = {
	.name      = "g_rndis_displaylink",
	.dev       = &gdl_device_desc,
	.strings   = gdl_strtabs,
	.max_speed = USB_SPEED_HIGH,
	.bind      = gdl_bind,
	.unbind    = gdl_unbind,
};

/* ---------- Module init/exit -------------------------------------------- */

static int __init gdl_init(void)
{
	int ret;

	g_dl = f_dl_alloc();
	if (IS_ERR(g_dl)) {
		ret = PTR_ERR(g_dl);
		pr_err("g_rndis_displaylink: f_dl_alloc = %d\n", ret);
		return ret;
	}

	if (!disable_rndis) {
		g_fi_rndis = usb_get_function_instance("rndis");
		if (IS_ERR(g_fi_rndis)) {
			ret = PTR_ERR(g_fi_rndis);
			pr_err("g_rndis_displaylink: rndis function unavailable (%d) — "
			       "load usb_f_rndis.ko first\n", ret);
			f_dl_free(g_dl);
			g_dl = NULL;
			return ret;
		}
	} else {
		g_fi_rndis = NULL;
	}

	ret = usb_composite_probe(&gdl_driver);
	if (ret) {
		pr_err("g_rndis_displaylink: usb_composite_probe = %d\n", ret);
		if (!IS_ERR_OR_NULL(g_fi_rndis))
			usb_put_function_instance(g_fi_rndis);
		f_dl_free(g_dl);
		g_dl = NULL;
		return ret;
	}

	return 0;
}

static void __exit gdl_exit(void)
{
	usb_composite_unregister(&gdl_driver);
	/*
	 * usb_composite_unregister calls usb_remove_function → f_rndis_free
	 * for the RNDIS function, so do NOT call usb_put_function here.
	 * Only release the function instance we hold.
	 */
	if (!IS_ERR_OR_NULL(g_fi_rndis))
		usb_put_function_instance(g_fi_rndis);

	f_dl_free(g_dl);
	g_dl = NULL;
}

module_init(gdl_init);
module_exit(gdl_exit);
