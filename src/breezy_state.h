#pragma once

#include <stdbool.h>
#include <stddef.h>

/*
 * breezy_state — connection-status state machine for the Breezy Box.
 *
 * Tracks six states and builds the appropriate human-readable message string.
 * This struct is pure CPU state — no GL, no EGL.  It survives renderer
 * teardown and reconnect cycles unchanged.
 *
 * The renderer owns the GL texture (struct overlay_text) and calls
 * breezy_state_format_message() to get the string to upload.
 *
 * Usage
 * -----
 *   struct breezy_state bs = {0};
 *   breezy_state_set_addresses(&bs, &link_cfg, gadget_netdev);
 *
 *   // each poll cycle (no GL context required):
 *   bool changed = breezy_state_update(&bs, xr_driver_up, glasses_active,
 *                                       imported_count, device_count, verbose);
 *
 *   // renderer uploads the texture when changed or after reconnect:
 *   if (changed || !overlay_tex.initialized) {
 *       char msg[BS_MSG_MAX];
 *       breezy_state_format_message(&bs, msg, sizeof msg);
 *       overlay_text_update(&overlay_tex, msg);
 *   }
 */

#define BS_ADDR_MAX  64
#define BS_MDNS_MAX  80   /* e.g. "breezy.local" */
#define BS_IFACE_MAX 32
#define BS_MSG_MAX   512

enum breezy_state_mode {
	BREEZY_STATE_XR_DRIVER_DOWN,
	BREEZY_STATE_NO_GLASSES,
	BREEZY_STATE_CALIBRATING,
	BREEZY_STATE_NO_HOST,
	BREEZY_STATE_NO_CLIENTS,
	BREEZY_STATE_NORMAL,
};

struct breezy_state {
	/* OTG/USB gadget link — populated by breezy_state_set_addresses(). */
	char link_mdns[BS_MDNS_MAX];    /* e.g. "breezy.local"     */
	char link_ip[BS_ADDR_MAX];      /* gadget OTG IPv4          */
	char host_ip[BS_ADDR_MAX];      /* host's DHCP address, e.g. "192.168.7.1" */
	char wlan_mdns[BS_MDNS_MAX];    /* e.g. "breezywlan.local"  */
	char wlan_ip[BS_ADDR_MAX];      /* wlan/LAN IPv4, may be empty */
	char otg_iface[BS_IFACE_MAX];   /* e.g. "usb0"              */

	/* Direct wired Ethernet link — populated by breezy_state_set_eth_link(). */
	char eth_iface[BS_IFACE_MAX];   /* e.g. "eth0"              */
	char eth_link_ip[BS_ADDR_MAX];  /* SBC's address on the link */
	char eth_host_ip[BS_ADDR_MAX];  /* host's DHCP address      */

	enum breezy_state_mode mode;
	bool host_connected;    /* host present on any path            */
	bool otg_connected;     /* host confirmed on OTG via arping    */
	bool eth_connected;     /* host confirmed on wired Ethernet via carrier */

	bool arping_result;     /* last OTG arping result, persists across calls */
};

/*
 * Populate OTG address fields from link_services_config + the OTG netdev name.
 * Must be called before the first breezy_state_update().
 * No GL context needed.
 */
struct link_services_config;   /* forward — include link_services.h for the full type */
void breezy_state_set_addresses(struct breezy_state *bs,
				 const struct link_services_config *link_cfg,
				 const char *otg_netdev);

/*
 * Populate direct wired Ethernet address fields.  Call after usb_gadget_setup()
 * once the Ethernet interface name and its CIDR are known.
 * No GL context needed; safe to call multiple times (overwrites previous values).
 */
void breezy_state_set_eth_link(struct breezy_state *bs,
				const char *eth_iface,
				const char *eth_link_ip,
				const char *eth_host_ip);

/*
 * Re-evaluate state and return true if mode or connectivity changed.
 * No GL context needed — pure CPU work (network probing, sysfs reads).
 *
 *   xr_driver_up    — true when the XR driver is producing fresh IMU data
 *   glasses_active  — true when IMU config is valid and glasses are connected
 *   imported_count  — number of device slots with an active USB/IP import
 *   device_count    — total number of configured device slots
 */
bool breezy_state_update(struct breezy_state *bs,
			  bool xr_driver_up,
			  bool glasses_active,
			  size_t imported_count,
			  size_t device_count,
			  bool verbose);

/*
 * Format the current state as a human-readable message string into buf.
 * No GL context needed.
 */
void breezy_state_format_message(const struct breezy_state *bs,
				  char *buf, size_t cap);
