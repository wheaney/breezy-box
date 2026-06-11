#pragma once

#include <stdbool.h>

/*
 * link_services — make the gadget reachable over the USB-ethernet link with no
 * manual host configuration:
 *
 *   - DHCP  (dnsmasq): hands the host an IPv4 address on the link so it can
 *     reach the gadget without the user configuring an IP by hand.
 *   - mDNS  (avahi):   advertises friendly .local names so the host can connect
 *     by name instead of an IP.  Two distinct names are published so the direct
 *     link and any other interface (e.g. Wi-Fi) can be told apart:
 *
 *       link_name (e.g. "breezy")     -> breezy.local, a static record PINNED to
 *                                        the gadget's own link address, so it
 *                                        ALWAYS resolves to the USB link and
 *                                        never to the Wi-Fi address.
 *       wlan_name (e.g. "breezywlan") -> a second static record pinned to the
 *                                        first non-link IPv4 address (prefer a
 *                                        wireless interface when present), so
 *                                        the Wi-Fi/LAN path stays distinct
 *                                        from the direct USB link.
 *
 *     Both names are published as explicit address aliases so the USB-link and
 *     Wi-Fi/LAN paths stay distinguishable by name.
 *
 * Both are the standard system daemons; this module only orchestrates them, so
 * it has no KMS/rendering dependencies and can be reused by any app that brings
 * up the OTG ethernet link (e.g. a future render-only build).  The daemons run
 * as independent processes, so no background thread is required here.
 */

#define LS_IFACE_MAX   32
#define LS_IP_MAX      48
#define LS_LEASE_MAX   16
#define LS_NAME_MAX    64
#define LS_PIDFILE_MAX 80   /* "/run/breezy-dnsmasq-<IFNAMSIZ>.pid\0" */

struct link_services_config {
    char iface[LS_IFACE_MAX];      /* link to serve, e.g. "usb0" */
    char ip_cidr[LS_IP_MAX];       /* if non-empty, assign this address+prefix to iface
                                      via "ip addr add" before starting DHCP, e.g. "192.168.8.2/30";
                                      empty = interface already has its IP (e.g. RNDIS gadget) */
    char host_ip[LS_IP_MAX];       /* IPv4 handed to the host via DHCP, e.g. "192.168.7.1";
                                      empty = skip DHCP (host configures its own address) */
    char lease_time[LS_LEASE_MAX]; /* dnsmasq lease time, e.g. "1h" */
    char link_ip[LS_IP_MAX];       /* the device's OWN address on the link (no prefix),
                                      e.g. "192.168.7.2"; link_name is pinned to it */
    char link_name[LS_NAME_MAX];   /* name pinned to link_ip; "breezy" -> breezy.local
                                      always resolves to the USB link.  Requires link_ip;
                                      empty = skip the pinned link name */
    char wlan_name[LS_NAME_MAX];   /* name pinned to the first non-link IPv4 address
                                      (prefer a wireless iface); "breezywlan" ->
                                      breezywlan.local; empty = skip the non-link name */
    unsigned mtu;                  /* if non-zero, push this interface MTU to the host
                                      via DHCP option 26 so a jumbo link MTU takes
                                      effect on both ends without manual host setup */
};

struct link_services_state {
    bool dhcp_running;
    bool wlan_name_set;
    bool link_name_pinned;
    /* pid-file paths derived from cfg->iface at start time; used by stop() */
    char dnsmasq_pidfile[LS_PIDFILE_MAX];
    char mdns_link_pidfile[LS_PIDFILE_MAX];
    char mdns_wlan_pidfile[LS_PIDFILE_MAX];
};

/* iface=usb0, host_ip=192.168.7.1, lease=1h, link_ip=192.168.7.2,
 * link_name=breezy, wlan_name=breezywlan */
void link_services_config_defaults(struct link_services_config *cfg);

/*
 * Start DHCP + mDNS on cfg->iface.  Best-effort: if dnsmasq or avahi aren't
 * installed it logs an actionable warning and continues (does not fail the
 * caller).  Returns 0 on success, -1 only on invalid arguments.
 */
int link_services_start(const struct link_services_config *cfg,
                        struct link_services_state *state);

/* Stop the DHCP server started by link_services_start.  Idempotent. */
void link_services_stop(struct link_services_state *state);
