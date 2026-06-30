#!/usr/bin/env bash
# One-time system configuration for breezy-box.
#
# Idempotent — safe to run more than once.  Each section prints what it did
# or "already done" so re-runs are easy to audit.
#
# Run as root (sudo ./setup_system.sh).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
usage() {
    cat <<'EOF'
Usage: sudo ./setup_system.sh [OPTIONS]

Perform one-time system-level setup required by breezy-box:

  • Load USB gadget kernel modules at boot (/etc/modules)
  • Mount configfs at boot (/etc/fstab)
  • Install the usb0 systemd-networkd profile (/etc/systemd/network/)

The changes are also applied immediately where possible so a reboot is not
required for the current session.

Options:
  --app-user USER    User that will run the KMS app (default: the user that
                     invoked sudo).  No user is created.
  --no-wlan          Skip Wi-Fi setup entirely (do not prompt for SSID/password).
  --reset-wlan       Force the Wi-Fi setup to run again even if a connection is
                     already configured (prompts for SSID/password anew).
  -h, --help         Show this help and exit.

Wi-Fi is configured BEFORE any wired-network changes so that, when this script
is run over an SSH session arriving on the Ethernet port, it can bring up
wireless first and then abort safely — reconfiguring the wired link would
otherwise drop that very session.  Run again over Wi-Fi (or a local console)
to finish.
EOF
}

# Default to the user that invoked sudo (the human running this script), not a
# hardcoded name.  Falls back to logname/USER if SUDO_USER is unset.
APP_USER="${SUDO_USER:-$(logname 2>/dev/null || echo "${USER:-root}")}"
NO_WLAN=0
RESET_WLAN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --app-user)
            [[ $# -lt 2 ]] && { echo "error: --app-user requires a value" >&2; exit 1; }
            APP_USER="$2"; shift 2 ;;
        --no-wlan)    NO_WLAN=1; shift ;;
        --reset-wlan) RESET_WLAN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "error: unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

if [[ "$(id -u)" -ne 0 ]]; then
    echo "error: run this script with sudo" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
section() { echo; echo "=== $* ==="; }
done_msg()  { echo "  done: $*"; }
skip_msg()  { echo "  skip: $* (already configured)"; }
warn_msg()  { echo "  warn: $*"; }

# True when NetworkManager is the active network stack.  The wired-link and
# port-redirect sections branch on this: networkd profiles are inert when NM
# owns the interfaces.
nm_active() { systemctl is-active --quiet NetworkManager 2>/dev/null; }

# True when NetworkManager is actually MANAGING the given interface — not merely
# running.  On Armbian/netplan images NM and systemd-networkd are both active, but
# netplan steers the wired port to networkd and NM reports it "unmanaged"; a
# global `nm_active` test would then wrongly write an inert NM keyfile for a
# device NM doesn't own.  Key the wired-link branch on this per-iface check.
nm_manages_iface() {
    local iface="$1" state
    nm_active || return 1
    state="$(nmcli -t -f GENERAL.STATE device show "$iface" 2>/dev/null)"
    # GENERAL.STATE is like "10 (unmanaged)" / "100 (connected)"; anything that
    # contains "unmanaged" (or is empty/absent) means NM is not the owner.
    [[ -n "$state" && "$state" != *unmanaged* ]]
}

# Echo the first physical wired Ethernet interface that is not the OTG gadget
# netdev and not wireless, or nothing if none is found.  Criteria mirror
# detect_wired_eth_iface() in displaylink_kms_renderer.c.  Used by both the
# SSH-over-Ethernet guard and the wired-link section.
detect_wired_eth_iface() {
    local iface
    for iface in $(ls /sys/class/net/ 2>/dev/null | sort); do
        [[ "${#iface}" -ge 16 ]]                  && continue   # IFNAMSIZ safety
        [[ "$iface" == "lo" ]]                    && continue
        [[ "$iface" == "usb0" ]]                  && continue
        [[ -d "/sys/class/net/$iface/wireless" ]] && continue
        [[ ! -e "/sys/class/net/$iface/device" ]] && continue
        echo "$iface"
        return 0
    done
    return 1
}

# ---------------------------------------------------------------------------
section "Required packages (preflight)"

# This script installs NOTHING.  Every package it (and the breezy-box runtime)
# needs must be present up front, or we fail here with the exact apt line to
# run.  Failing fast beats half-configuring the box and then dying mid-network.
#
# Each entry is "command:package" — we check the command and, if missing, add
# the package to the install hint.  A few deps are services with no single
# probe command (dnsmasq's daemon, avahi), so they are checked by package.
MISSING_PKGS=()

# Tools the setup script invokes directly.
require_cmd() {
    local cmd="$1" pkg="$2"
    command -v "$cmd" &>/dev/null || MISSING_PKGS+=("$pkg")
}

require_cmd nmcli      network-manager   # Wi-Fi setup + NM-owned wired link
require_cmd ip         iproute2          # routing / SSH-iface detection
require_cmd curl       curl              # connectivity check + PowerVR blob fetch
require_cmd ping       iputils-ping      # connectivity check fallback
require_cmd openssl    openssl           # self-signed TLS cert for breezy-web
require_cmd nft        nftables          # :80/:443 -> :8081/:8443 redirects
require_cmd patchelf   patchelf          # absolute rpath on the renderer binary
require_cmd setcap     libcap2-bin       # cap_sys_nice for the SCHED_FIFO thread
require_cmd zcat       gzip              # decompress the PowerVR Packages.gz index

# Runtime services with no convenient single-command probe: check the binary the
# breezy runtime spawns.  avahi-publish-address (avahi-utils) + the daemon
# (avahi-daemon) back mDNS; dnsmasq serves DHCP on the direct links.
command -v avahi-publish-address &>/dev/null || MISSING_PKGS+=(avahi-utils avahi-daemon)
command -v dnsmasq               &>/dev/null || MISSING_PKGS+=(dnsmasq)

if [[ "${#MISSING_PKGS[@]}" -gt 0 ]]; then
    # De-dupe while preserving a stable order.
    readarray -t MISSING_PKGS < <(printf '%s\n' "${MISSING_PKGS[@]}" | awk '!seen[$0]++')
    echo "  error: required packages are missing." >&2
    echo >&2
    echo "  Install them first, then re-run this script:" >&2
    echo >&2
    echo "      sudo apt update" >&2
    echo "      sudo apt install -y ${MISSING_PKGS[*]}" >&2
    echo >&2
    exit 1
fi
done_msg "all required packages present"

# ---------------------------------------------------------------------------
section "Kernel modules for USB gadget"

add_module() {
    local mod="$1"
    if ! grep -qxF "$mod" /etc/modules 2>/dev/null; then
        echo "$mod" >> /etc/modules
        done_msg "added '$mod' to /etc/modules"
    else
        skip_msg "$mod"
    fi
    # Also load now so the current session doesn't need a reboot.
    modprobe "$mod" 2>/dev/null && true
}

add_module libcomposite
add_module usb_f_rndis

# ---------------------------------------------------------------------------
section "configfs mount"

FSTAB_LINE="none  /sys/kernel/config  configfs  defaults  0  0"
if ! grep -qF 'configfs' /etc/fstab 2>/dev/null; then
    echo "$FSTAB_LINE" >> /etc/fstab
    done_msg "added configfs entry to /etc/fstab"
else
    skip_msg "configfs already in /etc/fstab"
fi

if ! mountpoint -q /sys/kernel/config 2>/dev/null; then
    mount -t configfs none /sys/kernel/config
    done_msg "mounted configfs now"
else
    skip_msg "configfs already mounted"
fi

# ---------------------------------------------------------------------------
section "Wi-Fi (wireless internet uplink)"

# Configure wireless BEFORE any wired-network changes.  Rationale: this script
# reconfigures the Ethernet port into a direct host link, which drops a regular
# LAN uplink on it.  If the operator is SSH'd in over that wired port, bringing
# up Wi-Fi first gives them a way back in; the SSH-over-Ethernet guard below
# then aborts so the wired reconfig never kills the live session.
#
# Skipped entirely with --no-wlan.  --reset-wlan forces a fresh prompt even when
# a connection already exists.  Currently NetworkManager-only (nmcli); on a
# networkd/wpa_supplicant box we warn and leave Wi-Fi to the operator.

# Echo the first wireless interface name, or nothing.
detect_wlan_iface() {
    local iface
    for iface in $(ls /sys/class/net/ 2>/dev/null | sort); do
        [[ -d "/sys/class/net/$iface/wireless" ]] && { echo "$iface"; return 0; }
    done
    return 1
}

# True when we have a usable default route AND can resolve/reach the internet.
# Used to verify the Wi-Fi actually came up before we proceed to wired changes.
have_internet() {
    # A default route is necessary but not sufficient; confirm reachability.
    ip route show default 2>/dev/null | grep -q . || return 1
    # Prefer a quick TCP/HTTP check (works where ICMP is filtered); fall back to ping.
    if command -v curl &>/dev/null; then
        curl -fsS --max-time 5 -o /dev/null https://1.1.1.1 2>/dev/null && return 0
    fi
    ping -c1 -W3 1.1.1.1 &>/dev/null
}

WLAN_IFACE="$(detect_wlan_iface || true)"

if [[ "$NO_WLAN" -eq 1 ]]; then
    skip_msg "Wi-Fi setup (--no-wlan)"
elif [[ -z "$WLAN_IFACE" ]]; then
    warn_msg "no wireless interface found"
    warn_msg "  ABORTING before any wired-network change so internet/SSH stays intact"
    exit 1
elif ! nm_active; then
    warn_msg "NetworkManager is not active — automated Wi-Fi setup unsupported here"
    warn_msg "  configure wpa_supplicant/networkd manually, or run with --no-wlan to silence this"
else
    # Is a Wi-Fi connection already configured/active?
    WIFI_ACTIVE_SSID="$(nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null \
                          | awk -F: '$1=="yes"{print $2; exit}')"
    HAVE_WIFI_PROFILE="$(nmcli -t -f TYPE connection show 2>/dev/null \
                          | grep -qx '802-11-wireless' && echo 1 || echo 0)"

    if [[ "$RESET_WLAN" -ne 1 && ( -n "$WIFI_ACTIVE_SSID" || "$HAVE_WIFI_PROFILE" -eq 1 ) ]]; then
        skip_msg "Wi-Fi already configured (${WIFI_ACTIVE_SSID:-saved profile}); use --reset-wlan to redo"
    else
        # Prompt for credentials.  Read from the controlling terminal so this
        # works even when the script's stdin is a pipe (curl | sudo bash).
        if [[ ! -t 0 && ! -e /dev/tty ]]; then
            warn_msg "no terminal available to prompt for Wi-Fi credentials — skipping"
            warn_msg "  run interactively, pass credentials via 'nmcli dev wifi connect' first, or use --no-wlan"
        else
            echo "  Enter Wi-Fi credentials for the wireless internet uplink (interface: $WLAN_IFACE)."
            echo "  Leave SSID blank to skip Wi-Fi setup."
            read -r -p "  SSID: " WIFI_SSID < /dev/tty
            if [[ -z "$WIFI_SSID" ]]; then
                skip_msg "Wi-Fi setup (no SSID entered)"
            else
                read -r -s -p "  Password (blank for open network): " WIFI_PSK < /dev/tty
                echo
                # Make sure the radio is on and rescan so the SSID is visible.
                nmcli radio wifi on 2>/dev/null || true
                nmcli dev wifi rescan 2>/dev/null || true
                sleep 2

                if [[ -n "$WIFI_PSK" ]]; then
                    nmcli dev wifi connect "$WIFI_SSID" password "$WIFI_PSK" ifname "$WLAN_IFACE" 2>&1 \
                        | sed 's/^/    /' || true
                else
                    nmcli dev wifi connect "$WIFI_SSID" ifname "$WLAN_IFACE" 2>&1 \
                        | sed 's/^/    /' || true
                fi
                unset WIFI_PSK   # don't leave the secret in the environment

                # Verify: poll for internet for up to ~30s (DHCP + association
                # can take a few seconds).  We REFUSE to proceed to the wired
                # reconfig until Wi-Fi is genuinely up, per the design.
                echo "  Verifying wireless connectivity..."
                wlan_ok=0
                for _ in $(seq 1 15); do
                    if have_internet; then wlan_ok=1; break; fi
                    sleep 2
                done

                if [[ "$wlan_ok" -eq 1 ]]; then
                    done_msg "Wi-Fi connected to '$WIFI_SSID' with internet access"
                else
                    warn_msg "Wi-Fi did NOT come up with internet access after connecting to '$WIFI_SSID'"
                    warn_msg "  check the password/SSID and signal, then re-run with --reset-wlan"
                    warn_msg "  ABORTING before any wired-network change so internet/SSH stays intact"
                    exit 1
                fi
            fi
        fi
    fi
fi

# ---------------------------------------------------------------------------
section "SSH-over-Ethernet safety guard"

# The wired-network sections below reconfigure the Ethernet port into a direct
# host link (static 192.168.77.2/30 + DHCP server / NM shared).  If THIS script
# is being run from an SSH session that arrives over that very wired interface,
# applying those changes would drop the connection mid-run and leave the box
# half-configured.  So: if we can prove the active SSH client is reaching us via
# the wired Ethernet iface, abort here (Wi-Fi is already up by now, so there is
# a way back in).  SSH over Wi-Fi, or a local TTY, proceeds normally.

# Find the interface carrying the SSH session that ultimately invoked this
# script (even under sudo, which strips SSH_CONNECTION from the environment).
# Strategy: walk up the process tree from $$ to find an sshd ancestor, then
# read its TCP peer from /proc/net/tcp (hex little-endian), convert to a dotted
# IP, and resolve that to an interface via "ip route get".
ssh_session_iface() {
    local pid ppid comm client_ip dev

    # Walk up to find the nearest sshd ancestor of this process.
    pid=$$
    while [[ "$pid" -gt 1 ]]; do
        comm="$(cat /proc/"$pid"/comm 2>/dev/null || true)"
        if [[ "$comm" == sshd ]]; then
            break
        fi
        ppid="$(awk '{print $4}' /proc/"$pid"/stat 2>/dev/null || true)"
        [[ -z "$ppid" || "$ppid" == "$pid" ]] && return 1
        pid="$ppid"
    done
    # No sshd ancestor: this is a local console / TTY. Emit an explicit
    # sentinel so the caller can tell "proven local" apart from "had an sshd
    # ancestor but couldn't resolve its iface" (which must fail closed).
    if [[ "$comm" != sshd ]]; then
        echo "__local__"
        return 0
    fi

    # Get the file descriptors for this sshd pid and find the TCP socket inode
    # that is in ESTABLISHED state (tcp_state=01) by cross-referencing /proc/net/tcp.
    local inode sock_inode
    for fd in /proc/"$pid"/fd/*; do
        sock_inode="$(readlink "$fd" 2>/dev/null)" || continue
        [[ "$sock_inode" == socket:* ]] || continue
        inode="${sock_inode//[^0-9]/}"

        # /proc/net/tcp columns: sl local_addr rem_addr state ... inode
        # state=01 = ESTABLISHED; rem_addr is client (hex little-endian).
        client_hex="$(awk -v want="$inode" '$10==want && $4=="01" {print $3}' \
                      /proc/net/tcp /proc/net/tcp6 2>/dev/null | head -n1)"
        [[ -z "$client_hex" ]] && continue

        # Convert hex little-endian IP:PORT → dotted IP.
        # IPv4 (/proc/net/tcp):  AABBCCDD:PPPP  (8 hex digits, little-endian)
        # IPv6 (/proc/net/tcp6): 32 hex digits; for an SSH client this is
        #   normally an IPv4-mapped address whose v4 part is the last 8 digits,
        #   also stored little-endian per 32-bit word — so the tail 8 digits
        #   decode exactly like the v4 case above.
        local hex_ip hex_b1 hex_b2 hex_b3 hex_b4
        hex_ip="${client_hex%%:*}"
        hex_ip="${hex_ip: -8}"          # last 8 hex digits (v4, or v4-mapped tail)
        hex_b4=$(( 16#${hex_ip:0:2} ))
        hex_b3=$(( 16#${hex_ip:2:2} ))
        hex_b2=$(( 16#${hex_ip:4:2} ))
        hex_b1=$(( 16#${hex_ip:6:2} ))
        client_ip="${hex_b1}.${hex_b2}.${hex_b3}.${hex_b4}"
        break
    done

    [[ -z "${client_ip:-}" ]] && return 1

    dev="$(ip route get "$client_ip" 2>/dev/null \
           | grep -oE 'dev [^ ]+' | awk '{print $2}' | head -n1)"
    [[ -n "$dev" ]] && echo "$dev"
}

GUARD_ETH_IFACE="$(detect_wired_eth_iface || true)"
SSH_IFACE="$(ssh_session_iface || true)"

if [[ "$SSH_IFACE" == "__local__" && -z "${SSH_CONNECTION:-}" ]]; then
    skip_msg "not an SSH session (local console) — wired reconfig is safe"
elif [[ -z "$SSH_IFACE" ]]; then
    # We could NOT determine the session's interface. This is the dangerous
    # case: if we're actually on the wired port and guess "safe", the reconfig
    # drops the link mid-run. Fail closed — abort and tell the user how to
    # reconnect. (SSH_CONNECTION, if present, can't tell us the iface either.)
    echo
    echo "  ABORT: could not determine which interface this session is using."
    echo "  The wired-network setup that follows would reconfigure the Ethernet"
    echo "  port into a direct host link (192.168.77.2/30); if this session is"
    echo "  arriving over that port, it would be DROPPED mid-run."
    echo
    echo "  Reconnect over Wi-Fi (or via breezy.local / the OTG USB link), or run"
    echo "  from a local console, and run setup_system.sh again. If you are certain"
    echo "  this session is not on the wired port, set ALLOW_WIRED_RECONFIG=1."
    echo
    [[ "${ALLOW_WIRED_RECONFIG:-}" == "1" ]] \
        && warn_msg "ALLOW_WIRED_RECONFIG=1 — proceeding despite unknown interface" \
        || exit 1
elif [[ -n "$GUARD_ETH_IFACE" && "$SSH_IFACE" == "$GUARD_ETH_IFACE" ]]; then
    echo
    echo "  ABORT: this SSH session is arriving over the wired Ethernet port ($SSH_IFACE)."
    echo "  The wired-network setup that follows would reconfigure $SSH_IFACE into a"
    echo "  direct host link (192.168.77.2/30) and DROP this connection."
    echo
    echo "  Wi-Fi has been configured above. Reconnect over Wi-Fi (or via breezy.local /"
    echo "  the OTG USB link) and run setup_system.sh again to finish the wired setup."
    echo
    exit 1
else
    skip_msg "SSH session is on '$SSH_IFACE' (not the wired port) — wired reconfig is safe"
fi

# ---------------------------------------------------------------------------
section "systemd-networkd profile for usb0"

NETWORK_SRC="$SCRIPT_DIR/network/usb0.network"
NETWORK_DST="/etc/systemd/network/usb0.network"

if [[ ! -f "$NETWORK_SRC" ]]; then
    echo "error: $NETWORK_SRC not found" >&2
    exit 1
fi

if ! cmp -s "$NETWORK_SRC" "$NETWORK_DST" 2>/dev/null; then
    install -m 0644 -D "$NETWORK_SRC" "$NETWORK_DST"
    done_msg "installed $NETWORK_DST"
    systemctl enable --now systemd-networkd 2>/dev/null || true
    networkctl reload 2>/dev/null || systemctl restart systemd-networkd || true
else
    skip_msg "$NETWORK_DST already up to date"
fi

# ---------------------------------------------------------------------------
section "avahi-daemon (mDNS responder)"

# avahi-publish-address is called by the breezy app as an unprivileged user;
# it communicates with the system avahi-daemon over D-Bus, so the daemon must
# be running before the app starts.
if systemctl is-enabled --quiet avahi-daemon 2>/dev/null; then
    skip_msg "avahi-daemon already enabled"
else
    systemctl enable --now avahi-daemon 2>/dev/null && done_msg "enabled and started avahi-daemon" \
        || echo "  warn: could not enable avahi-daemon — install 'avahi-daemon' and 'avahi-utils'"
fi

# ---------------------------------------------------------------------------
section "systemd-networkd profile for direct Ethernet link"

# Detect the first physical wired Ethernet interface that is not the OTG
# gadget netdev and not wireless.  Criteria mirror detect_wired_eth_iface()
# in displaylink_kms_renderer.c.  The interface must be present in sysfs
# at setup time — on most SBCs the built-in Ethernet port always is.
#
# NOTE: this assumes the SBC's Ethernet port is used solely for the direct
# host connection.  Do not run setup_system.sh while it is also serving as
# a regular LAN uplink.
#
# To avoid killing the very session that runs this script, the WLAN-setup and
# SSH-over-Ethernet guard sections above abort before we reach here when the
# operator is connected via the wired port (see those sections).
ETH_IFACE="$(detect_wired_eth_iface || true)"

if [[ -z "$ETH_IFACE" ]]; then
    echo "  skip: no wired Ethernet interface found — plug it in and re-run if needed"
elif nm_manages_iface "$ETH_IFACE"; then
    # NetworkManager owns the wired interface, so a systemd-networkd profile for
    # it is inert (NM keeps the device and applies no static IP / DHCP).  Write
    # an NM keyfile instead: static 192.168.77.2/30 with ipv4.method=shared so
    # NM's own dnsmasq serves the host on the far end.  192.168.77.x is
    # deliberately uncommon so this link doesn't collide with the Wi-Fi uplink.
    ETH_NM_DST="/etc/NetworkManager/system-connections/breezy-${ETH_IFACE}.nmconnection"

    # Drop any leftover networkd profile so the two stacks don't fight over the
    # interface (e.g. from an earlier networkd-based run of this script).
    for old in "/etc/systemd/network/05-breezy-${ETH_IFACE}.network" \
               "/etc/systemd/network/breezy-${ETH_IFACE}.network"; do
        if [[ -f "$old" ]]; then
            rm -f "$old"
            done_msg "removed inert networkd profile $old"
            networkctl reload 2>/dev/null || true
        fi
    done

    ETH_NM_CONTENT="[connection]
id=breezy-${ETH_IFACE}
type=ethernet
interface-name=${ETH_IFACE}
autoconnect=true
autoconnect-priority=10

[ipv4]
method=shared
address1=192.168.77.2/30

[ipv6]
method=ignore"

    CURRENT_CONTENT="$(cat "$ETH_NM_DST" 2>/dev/null || true)"
    if [[ "$CURRENT_CONTENT" != "$ETH_NM_CONTENT" ]]; then
        echo "$ETH_NM_CONTENT" > "$ETH_NM_DST"
        # NM refuses to load keyfiles that are group/world-readable.
        chmod 0600 "$ETH_NM_DST"
        done_msg "installed $ETH_NM_DST (NM shared @192.168.77.2/30)"
        nmcli connection reload 2>/dev/null || true
        # Bring it up if the link has carrier; harmless no-op if not plugged in.
        nmcli connection up "breezy-${ETH_IFACE}" 2>/dev/null \
            && done_msg "activated breezy-${ETH_IFACE}" \
            || warn_msg "breezy-${ETH_IFACE} not active yet (no carrier? plug in the host cable)"
    else
        skip_msg "$ETH_NM_DST already up to date"
    fi
else
    # NM is either not running, or running but does NOT manage this iface (the
    # Armbian/netplan case: NM marks the wired port "unmanaged" and networkd owns
    # it).  Either way, drive it with systemd-networkd's built-in DHCP server.
    # Prefix with 05- so this file sorts before netplan's generated
    # 10-netplan-all-eth-interfaces.network, which otherwise wins and leaves
    # the interface with no static IP and no DHCPServer.
    ETH_NET_DST="/etc/systemd/network/05-breezy-${ETH_IFACE}.network"

    # Drop any inert NM keyfile from an earlier run that took the NM branch (e.g.
    # before nm_manages_iface() existed, when a global nm_active test wrote a
    # keyfile NM never applied).  Leaving it risks the two stacks fighting.
    ETH_NM_OLD="/etc/NetworkManager/system-connections/breezy-${ETH_IFACE}.nmconnection"
    if [[ -f "$ETH_NM_OLD" ]]; then
        rm -f "$ETH_NM_OLD"
        done_msg "removed inert NM keyfile $ETH_NM_OLD"
        nmcli connection reload 2>/dev/null || true
    fi

    # Remove any old unprefixed file left by a previous run of setup_system.sh.
    OLD_ETH_NET_DST="/etc/systemd/network/breezy-${ETH_IFACE}.network"
    if [[ -f "$OLD_ETH_NET_DST" ]]; then
        rm -f "$OLD_ETH_NET_DST"
        done_msg "removed old (unprefixed) $OLD_ETH_NET_DST"
    fi

    # [DHCPServer] replaces dnsmasq: networkd's built-in server serves
    # 192.168.77.1 to the host (PoolOffset=1 from network base 192.168.77.0).
    ETH_NET_CONTENT="# systemd-networkd configuration for the direct Ethernet link.
# Generated by setup_system.sh for interface: ${ETH_IFACE}
# Prefixed 05- to sort before netplan's 10-netplan-all-eth-interfaces.network.

[Match]
Name=${ETH_IFACE}

[Network]
Address=192.168.77.2/30
DHCPServer=yes

[DHCPServer]
PoolOffset=1
PoolSize=1
EmitDNS=no
EmitNTP=no
EmitRouter=no"

    CURRENT_CONTENT="$(cat "$ETH_NET_DST" 2>/dev/null || true)"
    if [[ "$CURRENT_CONTENT" != "$ETH_NET_CONTENT" ]]; then
        echo "$ETH_NET_CONTENT" > "$ETH_NET_DST"
        chmod 0644 "$ETH_NET_DST"
        done_msg "installed $ETH_NET_DST"
        networkctl reload 2>/dev/null || systemctl restart systemd-networkd || true
    else
        skip_msg "$ETH_NET_DST already up to date"
    fi
fi

# ---------------------------------------------------------------------------
section "breezy-gadget system service (disabled: DisplayLink transport is raw_gadget, not FunctionFS)"

# breezy-gadget.service used to set up a FunctionFS configfs gadget and
# permanently bind the box's only UDC to it before the renderer started. The
# renderer now drives DisplayLink via raw_gadget.c (/dev/raw-gadget), which
# binds the UDC itself at runtime and releases it on stop — a UDC can only be
# bound to one gadget driver at a time, so the FFS setup must NOT run, or it
# starves raw_gadget of the UDC (or vice versa, depending on start order).
# Disable+stop it if a previous install left it enabled.
if systemctl is-enabled --quiet breezy-gadget.service 2>/dev/null \
   || systemctl is-active --quiet breezy-gadget.service 2>/dev/null; then
    systemctl disable --now breezy-gadget.service 2>/dev/null \
        && done_msg "disabled breezy-gadget.service (FunctionFS gadget, superseded by raw_gadget)" \
        || echo "  warn: failed to disable breezy-gadget.service"
else
    skip_msg "breezy-gadget.service already disabled"
fi

# /dev/raw-gadget defaults to root:root mode 0600 (CONFIG_USB_RAW_GADGET); the
# renderer runs unprivileged (SupplementaryGroups=video in its unit), so grant
# that group access via udev instead of running the renderer as root.
RAW_GADGET_UDEV_RULE_SRC="$SCRIPT_DIR/udev/99-breezy-raw-gadget.rules"
RAW_GADGET_UDEV_RULE_DST="/etc/udev/rules.d/99-breezy-raw-gadget.rules"
if [[ -f "$RAW_GADGET_UDEV_RULE_SRC" ]]; then
    if ! cmp -s "$RAW_GADGET_UDEV_RULE_SRC" "$RAW_GADGET_UDEV_RULE_DST" 2>/dev/null; then
        install -m 0644 "$RAW_GADGET_UDEV_RULE_SRC" "$RAW_GADGET_UDEV_RULE_DST"
        udevadm control --reload-rules
        udevadm trigger --name-match=raw-gadget 2>/dev/null || true
        done_msg "installed $RAW_GADGET_UDEV_RULE_DST"
    else
        skip_msg "$RAW_GADGET_UDEV_RULE_DST already up to date"
    fi
else
    echo "  warn: $RAW_GADGET_UDEV_RULE_SRC not found, skipping raw-gadget udev rule install"
fi

# ---------------------------------------------------------------------------
section "breezy-gadget-reset system service (OTG PHY reset, transport-agnostic)"

# reset_usb0_otg_stack.sh recovers a wedged dwc3+usb2phy OTG stack (RK3399
# BC1.2 DCP misclassification) at a level below any gadget driver — it's
# useful regardless of whether FFS or raw_gadget owns the UDC, so this stays
# wired up.  Its ExecStartPost used to re-run breezy_gadget_setup.sh (FFS
# configfs rebind); now that raw_gadget owns the UDC at runtime, that
# ExecStartPost is dropped from the installed unit below.
RESET_SCRIPT_SRC="$SCRIPT_DIR/reset_usb0_otg_stack.sh"
RESET_SCRIPT_DST="/usr/local/lib/breezy-box/reset_usb0_otg_stack.sh"
RESET_SERVICE_SRC="$SCRIPT_DIR/systemd/system/breezy-gadget-reset.service"
RESET_SERVICE_DST="/etc/systemd/system/breezy-gadget-reset.service"
UDEV_RULE_SRC="$SCRIPT_DIR/udev/99-breezy-otg-reset.rules"
UDEV_RULE_DST="/etc/udev/rules.d/99-breezy-otg-reset.rules"

if [[ -f "$RESET_SCRIPT_SRC" ]]; then
    mkdir -p "$(dirname "$RESET_SCRIPT_DST")"
    if ! cmp -s "$RESET_SCRIPT_SRC" "$RESET_SCRIPT_DST" 2>/dev/null; then
        install -m 0755 "$RESET_SCRIPT_SRC" "$RESET_SCRIPT_DST"
        done_msg "installed $RESET_SCRIPT_DST"
    else
        skip_msg "$RESET_SCRIPT_DST already up to date"
    fi

    if ! cmp -s "$RESET_SERVICE_SRC" "$RESET_SERVICE_DST" 2>/dev/null; then
        install -m 0644 "$RESET_SERVICE_SRC" "$RESET_SERVICE_DST"
        done_msg "installed $RESET_SERVICE_DST"
    else
        skip_msg "$RESET_SERVICE_DST already up to date"
    fi

    if ! cmp -s "$UDEV_RULE_SRC" "$UDEV_RULE_DST" 2>/dev/null; then
        install -m 0644 "$UDEV_RULE_SRC" "$UDEV_RULE_DST"
        udevadm control --reload-rules
        done_msg "installed $UDEV_RULE_DST"
    else
        skip_msg "$UDEV_RULE_DST already up to date"
    fi

    systemctl daemon-reload
else
    echo "  warn: $RESET_SCRIPT_SRC not found, skipping OTG reset service install"
fi

# ---------------------------------------------------------------------------
section "Realtime scheduling for the render thread (SCHED_FIFO)"

# The renderer boosts its GL thread to SCHED_FIFO (see §4 + breezy-renderer).
# On kernels with CONFIG_RT_GROUP_SCHED=y under cgroup v2 (the A733 BSP),
# realtime is denied to every non-root cgroup and cgroup v2 exposes no knob to
# grant RT bandwidth — so even `chrt -f` as root fails and the file cap is moot.
# Disabling RT *throttling* globally bypasses the per-group enforcement.
#
# GUARDED: applied ONLY when a live probe shows RT is actually blocked.  Boards
# where setcap alone works (e.g. RK3399, CONFIG_RT_GROUP_SCHED=n) keep their RT
# throttle intact.  -1 removes throttling system-wide — acceptable for a
# dedicated appliance with one bounded, CPU-0-pinned RT thread.  The proper fix
# is a kernel built with CONFIG_RT_GROUP_SCHED=n.  See §4.
RT_SYSCTL="/etc/sysctl.d/10-breezy-rt.conf"
if chrt -f 1 true 2>/dev/null; then
    skip_msg "SCHED_FIFO already available — leaving RT throttle intact"
elif [[ "$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null)" == "-1" ]]; then
    skip_msg "RT throttle already disabled (sched_rt_runtime_us=-1)"
else
    echo "kernel.sched_rt_runtime_us = -1" > "$RT_SYSCTL"
    sysctl -w kernel.sched_rt_runtime_us=-1 >/dev/null 2>&1 || true
    if chrt -f 1 true 2>/dev/null; then
        done_msg "RT was blocked (RT_GROUP_SCHED) — disabled RT throttle via $RT_SYSCTL"
    else
        rm -f "$RT_SYSCTL"
        warn_msg "RT still unavailable after sched_rt_runtime_us=-1 — render thread stays at"
        warn_msg "  normal priority; needs a kernel built with CONFIG_RT_GROUP_SCHED=n"
    fi
fi

# ---------------------------------------------------------------------------
section "breezy-renderer kiosk system service (DRM master on tty1)"

# The KMS renderer must take DRM master, which only the active session on seat0
# can do.  We run it as a SYSTEM service bound to tty1 (via PAMName=login +
# TTYPath) so logind seats it on seat0 — a --user service runs in the seatless
# user@.service scope and drmSetMaster() is denied there.  The renderer owns
# tty1; getty@tty1 is MASKED so it can never be (re)spawned there and SIGTERM
# the renderer (which manifests as the overlay flashing, then a black screen).
RENDERER_SVC_SRC="$SCRIPT_DIR/systemd/system/breezy-renderer.service"
RENDERER_SVC_DST="/etc/systemd/system/breezy-renderer.service"

# The VT the renderer owns (matches TTYPath in breezy-renderer.service).
RENDERER_VT="tty1"

# Remove the now-obsolete getty autologin drop-in (renderer owns the VT instead).
OLD_GETTY_DROPIN="/etc/systemd/system/getty@${RENDERER_VT}.service.d/autologin.conf"
if [[ -f "$OLD_GETTY_DROPIN" ]]; then
    rm -f "$OLD_GETTY_DROPIN"
    rmdir "$(dirname "$OLD_GETTY_DROPIN")" 2>/dev/null || true
    done_msg "removed obsolete getty autologin drop-in"
fi

# Mask every console unit that could occupy the renderer's VT.  Rather than
# hardcode getty/kmscon, discover whatever owns the VT: any console daemon
# (getty, kmsconvt, console-getty, a greeter, etc.) grabs the tty and, for the
# KMS ones, DRM master — which makes the renderer's drmSetMaster() fail with
# EPERM (black screen + Restart=always loop).  We mask each so the renderer is
# the sole owner of the VT and the seat.
mask_unit() {
    local unit="$1" why="$2"
    [[ -z "$unit" ]] && return 0
    if [[ "$(systemctl is-enabled "$unit" 2>/dev/null || true)" == "masked" ]]; then
        skip_msg "$unit already masked"
        return 0
    fi
    systemctl stop "$unit" 2>/dev/null || true
    systemctl mask "$unit"
    done_msg "masked $unit ($why)"
}

# 1. Template console instances bound to the renderer's VT, e.g.
#    getty@tty1.service, kmsconvt@tty1.service, serial-getty@tty1.service.
#    Scoped to "@${RENDERER_VT}" so other VTs' gettys (tty2…) are untouched.
#    NOTE: list-units prefixes a "●" status bullet on failed/active units even
#    with --no-legend, so we must pick the field ending in .service, not $1
#    (which would capture the bullet and feed "●" into systemctl mask).
VT_UNITS="$(
    systemctl list-units --all --no-legend "*@${RENDERER_VT}.service" 2>/dev/null \
        | grep -oE '[^[:space:]]+@'"${RENDERER_VT}"'\.service'
)"

# 2. Console template families whose unit FILE exists but may not be loaded yet
#    (logind instantiates them lazily on VT activation).  Mask the VT instance
#    so it can never start there.  list-units in step 1 only sees loaded units,
#    so this catches the not-yet-spawned case.
for tmpl in getty kmsconvt serial-getty; do
    systemctl list-unit-files "${tmpl}@.service" --no-legend 2>/dev/null | grep -q . \
        && VT_UNITS+=$'\n'"${tmpl}@${RENDERER_VT}.service"
done

# 3. Static (non-templated) console units that grab DRM master regardless of
#    VT, masked only if present.  Service units only — never targets.
for static_unit in kmscon.service console-getty.service; do
    systemctl list-unit-files "$static_unit" --no-legend 2>/dev/null | grep -q . \
        && VT_UNITS+=$'\n'"$static_unit"
done

# De-dupe (steps 1 and 2 overlap when a template instance is already loaded).
VT_UNITS="$(printf '%s\n' "$VT_UNITS" | awk 'NF' | sort -u)"

if [[ -z "${VT_UNITS//[$'\n\t ']/}" ]]; then
    skip_msg "no console units found on ${RENDERER_VT} (renderer already sole owner)"
else
    while IFS= read -r unit; do
        [[ -z "$unit" ]] && continue
        mask_unit "$unit" "frees ${RENDERER_VT} + DRM master for the renderer"
    done <<< "$VT_UNITS"
fi

if [[ ! -f "$RENDERER_SVC_SRC" ]]; then
    echo "  warn: $RENDERER_SVC_SRC not found, skipping renderer service install"
else
    # Stop the renderer before patchelf/setcap modify the binary on disk.
    systemctl stop --wait breezy-renderer.service 2>/dev/null || true

    # Substitute the app user into the unit's __APP_USER__ placeholders.
    RENDERER_SVC_TMP="$(mktemp)"
    sed "s/__APP_USER__/$APP_USER/g" "$RENDERER_SVC_SRC" > "$RENDERER_SVC_TMP"

    if ! cmp -s "$RENDERER_SVC_TMP" "$RENDERER_SVC_DST" 2>/dev/null; then
        install -m 0644 "$RENDERER_SVC_TMP" "$RENDERER_SVC_DST"
        systemctl daemon-reload
        done_msg "installed $RENDERER_SVC_DST (kiosk on tty1 as $APP_USER)"
    else
        skip_msg "$RENDERER_SVC_DST already up to date"
    fi
    rm -f "$RENDERER_SVC_TMP"

    # Grant CAP_SYS_NICE on the renderer BINARY (not via systemd
    # AmbientCapabilities, which — combined with User=+PAMName=login — changes
    # PAM credential ordering and costs the renderer its seat/DRM master).  The
    # renderer needs CAP_SYS_NICE to put its GL render thread on SCHED_FIFO:
    # unprivileged RT is refused on this kernel even with LimitRTPRIO set
    # ("SCHED_FIFO boost unavailable: Operation not permitted").  A file
    # capability is independent of the systemd credential path, so it grants the
    # cap without perturbing the seat.  Must be re-applied whenever the binary
    # is recopied — hence it runs here, every setup.
    RENDERER_BIN="/home/$APP_USER/.local/bin/displaylink_kms_renderer"
    RENDERER_DIR="$(dirname "$RENDERER_BIN")"
    if [[ -x "$RENDERER_BIN" ]]; then
        # Absolute rpath BEFORE setcap.  The cap makes the binary AT_SECURE, so
        # ld.so ignores $ORIGIN/LD_LIBRARY_PATH — it then can't find its NativeAOT
        # sibling ZeroKvm.NativeBridge.so, nor (on PowerVR) the PVR Mesa in
        # /usr/local/lib.  An ABSOLUTE rpath covering both fixes it.  patchelf
        # rewrites the file and WIPES the file cap, so this must run before setcap.
        # See docs/hardware/radxa-cubie-a7s.md §4 + §7.
        if [[ ! -e "$RENDERER_DIR/ZeroKvm.NativeBridge.so" ]]; then
            warn_msg "ZeroKvm.NativeBridge.so missing from $RENDERER_DIR — renderer won't start under the cap"
            warn_msg "  copy it from the build tree: .../ZeroKvm.NativeBridge/bin/Release/*/linux-arm64/publish/"
        fi
        WANT_RPATH="$RENDERER_DIR:/usr/local/lib"   # /usr/local/lib is harmless (empty) on non-PVR boards
        # patchelf is guaranteed present by the preflight check.
        if [[ "$(patchelf --print-rpath "$RENDERER_BIN" 2>/dev/null || true)" != "$WANT_RPATH" ]]; then
            patchelf --force-rpath --set-rpath "$WANT_RPATH" "$RENDERER_BIN" \
                && done_msg "set renderer rpath $WANT_RPATH (ZeroKvm + PVR Mesa under cap)" \
                || warn_msg "patchelf --set-rpath failed on $RENDERER_BIN"
        else
            skip_msg "renderer rpath already $WANT_RPATH"
        fi

        # setcap is guaranteed present by the preflight check.  Capture stderr:
        # the common failure is $HOME on a nosuid mount or a filesystem without
        # xattr support, and the reason matters.
        if setcap_err="$(setcap cap_sys_nice+ep "$RENDERER_BIN" 2>&1)"; then
            done_msg "granted cap_sys_nice to $RENDERER_BIN (SCHED_FIFO render thread)"
        else
            warn_msg "setcap on $RENDERER_BIN failed (${setcap_err:-unknown}) — render thread stays at normal priority"
            warn_msg "  if the fs lacks xattr/is nosuid, install the binary to /usr/local/bin instead"
        fi
    else
        warn_msg "renderer binary not found at $RENDERER_BIN — build+copy it, then re-run setup to grant cap_sys_nice"
    fi

    if systemctl is-enabled --quiet breezy-renderer.service 2>/dev/null; then
        skip_msg "breezy-renderer.service already enabled"
    else
        systemctl enable breezy-renderer.service
        done_msg "enabled breezy-renderer.service"
    fi
    systemctl start breezy-renderer.service 2>/dev/null \
        && done_msg "started breezy-renderer.service" \
        || echo "  warn: breezy-renderer.service failed to start — check: journalctl -u breezy-renderer"
fi

# ---------------------------------------------------------------------------
section "Breezy user systemd units"

USER_UNIT_SRC="$SCRIPT_DIR/systemd/user"
USER_UNIT_DST="/etc/systemd/user"
UNITS_CHANGED=0

for unit in breezy.target breezy-xvfb.service \
            breezy-desktop.service breezy-x11vnc.service \
            breezy-novnc.service breezy-web.service; do
    dst="$USER_UNIT_DST/$unit"
    src="$USER_UNIT_SRC/$unit"
    if [[ ! -f "$src" ]]; then
        echo "  warn: $src not found, skipping"
        continue
    fi
    if ! cmp -s "$src" "$dst" 2>/dev/null; then
        install -m 0644 "$src" "$dst"
        done_msg "installed $dst"
        UNITS_CHANGED=1
    else
        skip_msg "$dst already up to date"
    fi
done

systemctl daemon-reload

# Start the headless user services (Xvfb, web, VNC, desktop UI) at boot.
#
# These do NOT need a seat or DRM master — only breezy-renderer (a system
# service on tty1) does.  So we enable LINGER for the app user: systemd then
# starts the user manager at boot and breezy.target (WantedBy=default.target)
# pulls in the headless services, with no tty1 login shell required.
#
# This is the opposite of the old getty-autologin model, where linger had to be
# OFF so breezy.target landed on the tty1 seat for DRM master.  The renderer no
# longer relies on the user session for that, so lingering is now correct.
if id "$APP_USER" &>/dev/null; then
    if loginctl show-user "$APP_USER" -p Linger 2>/dev/null | grep -q 'Linger=yes'; then
        skip_msg "linger already enabled for $APP_USER"
    else
        loginctl enable-linger "$APP_USER"
        done_msg "enabled linger for $APP_USER (headless services start at boot)"
    fi

    # Enable + start breezy.target in the app user's manager.
    USER_CMD="XDG_RUNTIME_DIR=/run/user/$(id -u "$APP_USER") systemctl --user"
    su -l "$APP_USER" -s /bin/bash -c "$USER_CMD daemon-reload" 2>/dev/null || true
    su -l "$APP_USER" -s /bin/bash -c "$USER_CMD enable breezy.target" \
        2>/dev/null && done_msg "enabled breezy.target (user)" || true
    # If the unit files changed, restart the member SERVICES so the new content
    # takes effect (a target restart does not reliably re-pull its Wants= members
    # — it can stop them in the stop phase and leave them dead).  Restarting the
    # services directly reloads their changed definitions and brings them up.
    if [[ "$UNITS_CHANGED" -eq 1 ]] \
        && su -l "$APP_USER" -s /bin/bash -c "$USER_CMD is-active --quiet breezy.target" 2>/dev/null; then
        su -l "$APP_USER" -s /bin/bash -c \
            "$USER_CMD restart breezy-xvfb.service breezy-desktop.service breezy-x11vnc.service breezy-novnc.service breezy-web.service" \
            2>/dev/null && done_msg "restarted breezy services (units changed)" || true
    fi
    # Always (re)start the target so its Wants= pull every member up.  Using
    # 'start' (not 'restart') is deliberate: a target start enqueues the Wants=
    # members without first tearing the target down, so this both covers the
    # first install (target not yet active) and re-pulls anything left stopped.
    su -l "$APP_USER" -s /bin/bash -c "$USER_CMD start breezy.target" \
        2>/dev/null && done_msg "started breezy.target (user)" \
        || echo "  warn: failed to start breezy.target — check: systemctl --user status breezy.target"

    # Clean up the obsolete tty1 launch block from a previous (getty) install.
    PROFILE_DST="/home/$APP_USER/.bash_profile"
    OLD_PROFILE_MARKER="# breezy-box: start breezy.target on tty1"
    if grep -qF "$OLD_PROFILE_MARKER" "$PROFILE_DST" 2>/dev/null; then
        # Delete the marker line and the 3-line if-block that follows it.
        sed -i "/$OLD_PROFILE_MARKER/,/^fi$/d" "$PROFILE_DST"
        done_msg "removed obsolete tty1 launch block from $PROFILE_DST"
    fi
else
    echo "  warn: user $APP_USER not found; user services will be configured when the user exists"
fi

# ---------------------------------------------------------------------------
section "TLS certificate for breezy-web (HTTPS)"

TLS_DIR="/etc/breezy-box/tls"
TLS_CERT="$TLS_DIR/server.crt"
TLS_KEY="$TLS_DIR/server.key"

if [[ -f "$TLS_CERT" && -f "$TLS_KEY" ]]; then
    skip_msg "TLS cert already exists at $TLS_CERT"
else
    if ! command -v openssl &>/dev/null; then
        echo "  warn: openssl not found — install 'openssl' and re-run to enable HTTPS"
    else
        mkdir -p "$TLS_DIR"
        # Use EC P-256 — mongoose's built-in TLS handles ecdsa_secp256r1_sha256
        # cleanly. RSA-2048 triggers a CRT-signing bug in the built-in stack when
        # any prime factor DER-encodes with a leading 0x00 padding byte (reducing
        # the stripped length below the 128-byte minimum check).
        openssl ecparam -name prime256v1 -genkey -noout -out "$TLS_KEY" 2>/dev/null
        openssl req -x509 -new -key "$TLS_KEY" \
            -out "$TLS_CERT" \
            -days 3650 \
            -subj "/CN=breezy.local" \
            -addext "subjectAltName=DNS:breezy.local,DNS:localhost,IP:192.168.77.2,IP:192.168.7.2" \
            2>/dev/null
        chmod 0640 "$TLS_KEY" "$TLS_CERT"
        # The app user (unprivileged) must be able to traverse the directory and
        # read the cert/key, but they must not be world-readable. Group-own both
        # the dir and the files by the app user with 0750/0640.
        chmod 0750 "$TLS_DIR"
        if id "$APP_USER" &>/dev/null; then
            chown "root:$APP_USER" "$TLS_DIR" "$TLS_KEY" "$TLS_CERT"
        fi
        done_msg "generated self-signed cert: $TLS_CERT (10-year, CN=breezy.local)"
    fi
fi

# ---------------------------------------------------------------------------
section "Port 80 -> 8081 and 443 -> 8443 redirects (breezy-web)"

# breezy-web runs unprivileged on 8081 (HTTP) and 8443 (HTTPS);
# redirect incoming :80/:443 via nftables or iptables.  Without this the UI is
# only reachable on :8443 — hitting https://<box> (port 443) gets no answer.
NFT_RULE_MARKER="# breezy-box: redirect ports 80->8081 and 443->8443"

# nftables is guaranteed present by the preflight check; the iptables branch
# below remains as a fallback for boards that already use iptables instead.
if command -v nft &>/dev/null; then
    # Ensure the rules loaded from /etc/nftables.conf survive reboot.
    systemctl enable nftables.service 2>/dev/null || true
    NFT_CONF="/etc/nftables.conf"
    # Remove old single-port rule if present (upgrade path).
    if grep -qF "# breezy-box: redirect port 80 to 8081" "$NFT_CONF" 2>/dev/null; then
        sed -i '/# breezy-box: redirect port 80 to 8081/,/^}/d' "$NFT_CONF"
        done_msg "removed old single-port nftables rule from $NFT_CONF"
    fi
    if grep -qF "$NFT_RULE_MARKER" "$NFT_CONF" 2>/dev/null; then
        skip_msg "nftables redirect rules already in $NFT_CONF"
    else
        cat >> "$NFT_CONF" <<EOF

$NFT_RULE_MARKER
table ip nat {
    chain prerouting {
        type nat hook prerouting priority dstnat;
        tcp dport 80  redirect to :8081
        tcp dport 443 redirect to :8443
    }
}
EOF
        done_msg "added port 80->8081 and 443->8443 redirects to $NFT_CONF"
        nft -f "$NFT_CONF" 2>/dev/null && done_msg "applied nftables rules" \
            || echo "  warn: nft -f failed — rules will apply on next boot"
    fi
elif command -v iptables &>/dev/null; then
    # HTTP redirect
    if iptables -t nat -C PREROUTING -p tcp --dport 80 -j REDIRECT --to-port 8081 &>/dev/null; then
        skip_msg "iptables HTTP redirect rule already active"
    else
        iptables -t nat -A PREROUTING -p tcp --dport 80 -j REDIRECT --to-port 8081
        done_msg "added iptables port 80 -> 8081 redirect"
    fi
    # HTTPS redirect
    if iptables -t nat -C PREROUTING -p tcp --dport 443 -j REDIRECT --to-port 8443 &>/dev/null; then
        skip_msg "iptables HTTPS redirect rule already active"
    else
        iptables -t nat -A PREROUTING -p tcp --dport 443 -j REDIRECT --to-port 8443
        done_msg "added iptables port 443 -> 8443 redirect"
    fi
    if command -v netfilter-persistent &>/dev/null; then
        netfilter-persistent save 2>/dev/null && done_msg "saved iptables rules" || true
    else
        echo "  warn: install 'iptables-persistent' to persist these rules across reboots"
    fi
else
    echo "  warn: neither nft nor iptables found — install nftables and re-run"
fi

# ---------------------------------------------------------------------------
section "Default renderer config (~/.config/breezy-box/config.json)"

# Install the default config only when the user already exists and has no
# config of their own.  Never overwrite an existing file so user edits survive
# re-runs.  The config lives in the app user's XDG_CONFIG_HOME, so we write it
# as that user rather than root.
CONFIG_DEFAULT_SRC="$SCRIPT_DIR/config.default.json"
if [[ ! -f "$CONFIG_DEFAULT_SRC" ]]; then
    warn_msg "config.default.json not found in $SCRIPT_DIR — skipping default config install"
elif ! id "$APP_USER" &>/dev/null; then
    warn_msg "user $APP_USER not found — default config will be installed when the user exists"
else
    APP_HOME="$(getent passwd "$APP_USER" | cut -d: -f6)"
    CONFIG_DST="${APP_HOME}/.config/breezy-box/config.json"
    if [[ -f "$CONFIG_DST" ]]; then
        skip_msg "$CONFIG_DST already exists (not overwriting)"
    else
        install -o "$APP_USER" -g "$APP_USER" -m 0755 -d \
            "$(dirname "$CONFIG_DST")"
        install -o "$APP_USER" -g "$APP_USER" -m 0644 \
            "$CONFIG_DEFAULT_SRC" "$CONFIG_DST"
        done_msg "installed default config: $CONFIG_DST (3× 1080p@30)"
    fi
fi
