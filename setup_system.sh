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
  --app-user USER    User that will run the KMS app (default: breezybox).
                     Used only for informational output; no user is created.
  -h, --help         Show this help and exit.
EOF
}

APP_USER="wayne" #"breezybox"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --app-user)
            [[ $# -lt 2 ]] && { echo "error: --app-user requires a value" >&2; exit 1; }
            APP_USER="$2"; shift 2 ;;
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
ETH_IFACE=""
for iface in $(ls /sys/class/net/ 2>/dev/null | sort); do
    [[ "${#iface}" -ge 16 ]]           && continue   # IFNAMSIZ safety
    [[ "$iface" == "lo" ]]             && continue
    [[ "$iface" == "usb0" ]]           && continue
    [[ -d "/sys/class/net/$iface/wireless" ]] && continue
    [[ ! -e "/sys/class/net/$iface/device" ]] && continue
    ETH_IFACE="$iface"
    break
done

if [[ -z "$ETH_IFACE" ]]; then
    echo "  skip: no wired Ethernet interface found — plug it in and re-run if needed"
else
    # Prefix with 05- so this file sorts before netplan's generated
    # 10-netplan-all-eth-interfaces.network, which otherwise wins and leaves
    # the interface with no static IP and no DHCPServer.
    ETH_NET_DST="/etc/systemd/network/05-breezy-${ETH_IFACE}.network"

    # Remove any old unprefixed file left by a previous run of setup_system.sh.
    OLD_ETH_NET_DST="/etc/systemd/network/breezy-${ETH_IFACE}.network"
    if [[ -f "$OLD_ETH_NET_DST" ]]; then
        rm -f "$OLD_ETH_NET_DST"
        done_msg "removed old (unprefixed) $OLD_ETH_NET_DST"
    fi

    # Generate the networkd profile inline so the interface name is embedded.
    # [DHCPServer] replaces dnsmasq: networkd's built-in server serves
    # 192.168.8.1 to the host (PoolOffset=1 from network base 192.168.8.0).
    # EmitRouter/DNS/NTP=no keeps the host's own routing and resolver intact.
    ETH_NET_CONTENT="# systemd-networkd configuration for the direct Ethernet link.
# Generated by setup_system.sh for interface: ${ETH_IFACE}
# Prefixed 05- to sort before netplan's 10-netplan-all-eth-interfaces.network.

[Match]
Name=${ETH_IFACE}

[Network]
Address=192.168.8.2/30
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
section "Getty autologin for $APP_USER on tty1"

GETTY_DROPIN_DST="/etc/systemd/system/getty@tty1.service.d/autologin.conf"
GETTY_DROPIN_SRC="$SCRIPT_DIR/systemd/system/getty@tty1.service.d/autologin.conf"
GETTY_DROPIN_WANT="$(sed "s/breezy/$APP_USER/g" "$GETTY_DROPIN_SRC")"
GETTY_DROPIN_CURRENT="$(cat "$GETTY_DROPIN_DST" 2>/dev/null || true)"

if [[ "$GETTY_DROPIN_CURRENT" != "$GETTY_DROPIN_WANT" ]]; then
    mkdir -p "$(dirname "$GETTY_DROPIN_DST")"
    echo "$GETTY_DROPIN_WANT" > "$GETTY_DROPIN_DST"
    chmod 0644 "$GETTY_DROPIN_DST"
    systemctl daemon-reload
    done_msg "installed $GETTY_DROPIN_DST (autologin as $APP_USER)"
else
    skip_msg "$GETTY_DROPIN_DST already up to date"
fi

# ---------------------------------------------------------------------------
section "Breezy user systemd units"

USER_UNIT_SRC="$SCRIPT_DIR/systemd/user"
USER_UNIT_DST="/etc/systemd/user"
UNITS_CHANGED=0

for unit in breezy.target breezy-renderer.service breezy-xvfb.service \
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

# Enable and (re-)start breezy.target for the app user.
if id "$APP_USER" &>/dev/null; then
    USER_CMD="XDG_RUNTIME_DIR=/run/user/\$(id -u) systemctl --user"
    # Always re-enable so the WantedBy symlink reflects the current unit file.
    su -l "$APP_USER" -s /bin/bash -c "$USER_CMD enable breezy.target" \
        2>/dev/null && done_msg "enabled breezy.target for $APP_USER" \
        || echo "  warn: could not enable breezy.target as $APP_USER"
    # Restart changed units if the session is already running.
    if [[ "$UNITS_CHANGED" -eq 1 ]]; then
        su -l "$APP_USER" -s /bin/bash -c \
            "$USER_CMD is-active --quiet breezy.target && $USER_CMD restart breezy.target" \
            2>/dev/null && done_msg "restarted breezy.target (units changed)" || true
    fi
else
    echo "  warn: user $APP_USER not found; enable breezy.target manually after creating the user"
fi

# ---------------------------------------------------------------------------
echo
echo "System setup complete."
echo
echo "Next steps if not done yet:"
echo "  1. Create the app user:  useradd -m -s /bin/bash -c 'Breezy Box' $APP_USER"
echo "     Add to groups:        usermod -aG video,render,input $APP_USER"
echo "  2. Enable linger:        loginctl enable-linger $APP_USER"
echo "  3. Fetch vendored deps and build:"
echo "     cd /home/$APP_USER/breezy-box && make deps && make"
echo "     cp displaylink_kms_renderer breezy_web ~/.local/bin/"
echo "  4. Reboot — getty autologin will start the user session and breezy.target."
echo "  5. Rebuild with setcap for gadget configfs access until a dedicated"
echo "     setup service is in place:"
echo "     sudo setcap 'cap_sys_admin,cap_net_admin,cap_sys_module+ep' \\"
echo "         /home/$APP_USER/.local/bin/displaylink_kms_renderer"
