#!/usr/bin/env bash
#
# Keep an mDNS name (default: breezywlan.local) pinned to the box's current
# Wi-Fi / LAN address, re-pinning whenever that address changes.
#
# Why this exists as a standalone service instead of living in the renderer:
# the renderer publishes breezywlan.local once, at startup, from
# link_services.c.  At boot the renderer comes up before wlan0 has finished
# DHCP, so detect_non_link_ipv4() finds no address and the publisher is never
# started — and it is never retried, nor re-pinned when DHCP later hands out a
# different lease.  The result is that breezywlan.local "is always advertised
# but never works".  This loop fixes that: it waits for an address, publishes,
# and watches for changes for the life of the box.
#
# avahi-publish-address holds the record only while it runs, so we keep a child
# process alive and replace it when the address changes.
#
# Config via environment (with defaults):
#   WLAN_MDNS_NAME   mDNS short name to publish        (default: breezywlan)
#   WLAN_MDNS_IFACE  preferred interface substring      (default: wlan)
#   WLAN_MDNS_POLL   seconds between address checks      (default: 5)

set -uo pipefail

MDNS_NAME="${WLAN_MDNS_NAME:-breezywlan}"
IFACE_HINT="${WLAN_MDNS_IFACE:-wlan}"
POLL="${WLAN_MDNS_POLL:-5}"
FQDN="${MDNS_NAME}.local"

publisher_pid=""
current_ip=""

cleanup() {
    if [ -n "$publisher_pid" ] && kill -0 "$publisher_pid" 2>/dev/null; then
        kill "$publisher_pid" 2>/dev/null
    fi
    exit 0
}
trap cleanup TERM INT

# Echo the first global IPv4 address on an interface whose name contains
# IFACE_HINT (prefer wireless); fall back to any non-loopback, non-link-local,
# non-direct-link global IPv4.  Excludes the 192.168.77.0/30 direct-Ethernet
# link and 169.254.0.0/16 link-local so we never pin the name to those.
find_wlan_ip() {
    local iface ip
    # Preferred: an interface matching the hint (e.g. wlan0).
    for iface in $(ls /sys/class/net/ 2>/dev/null); do
        case "$iface" in
            *"$IFACE_HINT"*) ;;
            *) continue ;;
        esac
        ip="$(ip -4 -o addr show dev "$iface" scope global 2>/dev/null \
                | awk '{print $4}' | cut -d/ -f1 | head -1)"
        [ -n "$ip" ] && { echo "$ip"; return 0; }
    done
    # Fallback: any global IPv4 that isn't the direct-Ethernet link.
    ip -4 -o addr show scope global 2>/dev/null \
        | awk '{print $2, $4}' \
        | while read -r dev cidr; do
            case "$dev" in lo) continue ;; esac
            local a="${cidr%/*}"
            case "$a" in
                192.168.77.*) continue ;;   # direct-Ethernet link
                169.254.*)    continue ;;   # link-local
            esac
            echo "$a"; return 0
          done
    return 1
}

echo "publish_wlan_mdns: pinning ${FQDN} to the Wi-Fi/LAN address (poll ${POLL}s)"

while true; do
    new_ip="$(find_wlan_ip || true)"

    if [ -n "$new_ip" ] && [ "$new_ip" != "$current_ip" ]; then
        # Address appeared or changed — (re)publish.
        if [ -n "$publisher_pid" ] && kill -0 "$publisher_pid" 2>/dev/null; then
            kill "$publisher_pid" 2>/dev/null
            wait "$publisher_pid" 2>/dev/null
        fi
        avahi-publish-address --no-reverse "$FQDN" "$new_ip" &
        publisher_pid=$!
        current_ip="$new_ip"
        echo "publish_wlan_mdns: ${FQDN} -> ${new_ip}"
    elif [ -z "$new_ip" ] && [ -n "$current_ip" ]; then
        # Address went away — drop the stale record.
        if [ -n "$publisher_pid" ] && kill -0 "$publisher_pid" 2>/dev/null; then
            kill "$publisher_pid" 2>/dev/null
            wait "$publisher_pid" 2>/dev/null
        fi
        publisher_pid=""
        current_ip=""
        echo "publish_wlan_mdns: Wi-Fi/LAN address lost; ${FQDN} unpublished"
    elif [ -n "$publisher_pid" ] && ! kill -0 "$publisher_pid" 2>/dev/null; then
        # Publisher died unexpectedly (avahi restart?) — force a re-publish.
        current_ip=""
        publisher_pid=""
    fi

    sleep "$POLL"
done
