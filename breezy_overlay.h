#pragma once

#include "overlay_text.h"

#include <stdbool.h>
#include <stddef.h>

/*
 * breezy_overlay — connection-status overlay for any Breezy Box renderer.
 *
 * Tracks four states and builds the appropriate human-readable message
 * as a GL texture so any GLES2 renderer can draw it:
 *
 *   BREEZY_OVERLAY_NO_GLASSES   no XR glasses detected
 *   BREEZY_OVERLAY_NO_HOST      glasses active but OTG link not enumerated
 *   BREEZY_OVERLAY_NO_CLIENTS   OTG link up but no USB/IP sessions imported
 *   BREEZY_OVERLAY_NORMAL       at least one DisplayLink session active
 *
 * Usage
 * -----
 *   struct breezy_overlay ov = {0};
 *   breezy_overlay_set_addresses(&ov, &link_cfg, gadget_netdev);
 *
 *   // each frame (or each poll cycle):
 *   breezy_overlay_update(&ov, glasses_active, imported_count, device_count);
 *
 *   // render:
 *   if (ov.tex.tex)
 *       // draw ov.tex.tex at the desired position
 *
 *   breezy_overlay_destroy(&ov);
 */

#define BO_ADDR_MAX  64
#define BO_MDNS_MAX  80   /* e.g. "breezy.local" */
#define BO_IFACE_MAX 32

enum breezy_overlay_state {
	BREEZY_OVERLAY_NO_GLASSES,
	BREEZY_OVERLAY_NO_HOST,
	BREEZY_OVERLAY_NO_CLIENTS,
	BREEZY_OVERLAY_NORMAL,
};

struct breezy_overlay {
	/* Addresses populated by breezy_overlay_set_addresses(). */
	char link_mdns[BO_MDNS_MAX];    /* e.g. "breezy.local"     */
	char link_ip[BO_ADDR_MAX];      /* gadget OTG IPv4          */
	char host_ip[BO_ADDR_MAX];      /* host's DHCP address, e.g. "192.168.7.1" */
	char wlan_mdns[BO_MDNS_MAX];    /* e.g. "breezywlan.local"  */
	char wlan_ip[BO_ADDR_MAX];      /* wlan/LAN IPv4, may be empty */
	char otg_iface[BO_IFACE_MAX];   /* e.g. "usb0"              */

	enum breezy_overlay_state state;
	bool host_connected;

	bool arping_result;     /* last arping probe result, persists across calls */

	struct overlay_text tex;        /* GL texture, ready to draw */
};

/*
 * Populate address fields from link_services_config + the OTG netdev name.
 * Must be called before the first breezy_overlay_update().
 * No GL context needed.
 */
struct link_services_config;   /* forward — include link_services.h for the full type */
void breezy_overlay_set_addresses(struct breezy_overlay *ov,
				   const struct link_services_config *link_cfg,
				   const char *otg_netdev);

/*
 * Re-evaluate state and refresh the overlay texture if anything changed.
 * Requires an active GL context.
 *
 *   glasses_active  — true when IMU config is valid and glasses are connected
 *   imported_count  — number of device slots with an active USB/IP import
 *   device_count    — total number of configured device slots
 *
 * Also re-checks whether the host is connected by running ping -c 1 -W 1
 * against host_ip on the OTG interface — carrier/operstate are not reliable
 * on CDC-NCM gadget interfaces across all kernels.
 * Also re-polls the current wlan/LAN address (in case Wi-Fi was assigned
 * after startup) — cheap getifaddrs() scan, done at most once per call.
 */
void breezy_overlay_update(struct breezy_overlay *ov,
			    bool glasses_active,
			    size_t imported_count,
			    size_t device_count);

/* Release the GL texture.  Call with an active GL context. */
void breezy_overlay_destroy(struct breezy_overlay *ov);
