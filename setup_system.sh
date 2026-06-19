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
    # 192.168.77.1 to the host (PoolOffset=1 from network base 192.168.77.0).
    # 192.168.77.x is deliberately uncommon among consumer routers so this wired
    # link does not collide with the SBC's own Wi-Fi/LAN uplink subnet.
    # EmitRouter/DNS/NTP=no keeps the host's own routing and resolver intact.
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
section "breezy-gadget system service (OTG RNDIS setup as root)"

# The gadget setup script runs as root so the renderer can run unprivileged.
GADGET_SCRIPT_SRC="$SCRIPT_DIR/breezy_gadget_setup.sh"
GADGET_SCRIPT_DST="/usr/local/lib/breezy-box/breezy_gadget_setup.sh"
RESET_SCRIPT_SRC="$SCRIPT_DIR/reset_usb0_otg_stack.sh"
RESET_SCRIPT_DST="/usr/local/lib/breezy-box/reset_usb0_otg_stack.sh"
GADGET_SERVICE_SRC="$SCRIPT_DIR/systemd/system/breezy-gadget.service"
GADGET_SERVICE_DST="/etc/systemd/system/breezy-gadget.service"
RESET_SERVICE_SRC="$SCRIPT_DIR/systemd/system/breezy-gadget-reset.service"
RESET_SERVICE_DST="/etc/systemd/system/breezy-gadget-reset.service"
UDEV_RULE_SRC="$SCRIPT_DIR/udev/99-breezy-otg-reset.rules"
UDEV_RULE_DST="/etc/udev/rules.d/99-breezy-otg-reset.rules"

if [[ ! -f "$GADGET_SCRIPT_SRC" ]]; then
    echo "  warn: $GADGET_SCRIPT_SRC not found, skipping gadget service install"
else
    mkdir -p "$(dirname "$GADGET_SCRIPT_DST")"
    if ! cmp -s "$GADGET_SCRIPT_SRC" "$GADGET_SCRIPT_DST" 2>/dev/null; then
        install -m 0755 "$GADGET_SCRIPT_SRC" "$GADGET_SCRIPT_DST"
        done_msg "installed $GADGET_SCRIPT_DST"
    else
        skip_msg "$GADGET_SCRIPT_DST already up to date"
    fi

    if [[ -f "$RESET_SCRIPT_SRC" ]]; then
        if ! cmp -s "$RESET_SCRIPT_SRC" "$RESET_SCRIPT_DST" 2>/dev/null; then
            install -m 0755 "$RESET_SCRIPT_SRC" "$RESET_SCRIPT_DST"
            done_msg "installed $RESET_SCRIPT_DST"
        else
            skip_msg "$RESET_SCRIPT_DST already up to date"
        fi
    else
        echo "  warn: $RESET_SCRIPT_SRC not found"
    fi

    if ! cmp -s "$GADGET_SERVICE_SRC" "$GADGET_SERVICE_DST" 2>/dev/null; then
        install -m 0644 "$GADGET_SERVICE_SRC" "$GADGET_SERVICE_DST"
        done_msg "installed $GADGET_SERVICE_DST"
    else
        skip_msg "$GADGET_SERVICE_DST already up to date"
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
    if systemctl is-enabled --quiet breezy-gadget.service 2>/dev/null; then
        skip_msg "breezy-gadget.service already enabled"
    else
        systemctl enable breezy-gadget.service
        done_msg "enabled breezy-gadget.service"
    fi
    # Start now so the gadget comes up without a reboot.
    systemctl start breezy-gadget.service 2>/dev/null \
        && done_msg "started breezy-gadget.service" \
        || echo "  warn: breezy-gadget.service failed to start — check: journalctl -u breezy-gadget"
fi

# ---------------------------------------------------------------------------
section "Getty autologin for $APP_USER on tty1"

GETTY_DROPIN_DST="/etc/systemd/system/getty@tty1.service.d/autologin.conf"
GETTY_DROPIN_SRC="$SCRIPT_DIR/systemd/system/getty@tty1.service.d/autologin.conf"

# Generate the target content by substituting the placeholder username.
GETTY_DROPIN_TMP="$(mktemp)"
sed "s/breezy/$APP_USER/g" "$GETTY_DROPIN_SRC" > "$GETTY_DROPIN_TMP"

if ! cmp -s "$GETTY_DROPIN_TMP" "$GETTY_DROPIN_DST" 2>/dev/null; then
    mkdir -p "$(dirname "$GETTY_DROPIN_DST")"
    install -m 0644 "$GETTY_DROPIN_TMP" "$GETTY_DROPIN_DST"
    systemctl daemon-reload
    done_msg "installed $GETTY_DROPIN_DST (autologin as $APP_USER)"
else
    skip_msg "$GETTY_DROPIN_DST already up to date"
fi
rm -f "$GETTY_DROPIN_TMP"

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

# Install .bash_profile for the app user to start breezy.target on tty1 login.
# We use .bash_profile (not .profile) so it only fires for bash login shells,
# which is what getty gives us.  The tty check ensures it only runs on tty1
# so SSH and other sessions are unaffected.
#
# NOTE: linger must NOT be enabled for this user.  If it is, systemd starts a
# seat-less user session at boot and breezy.target lands there instead of the
# tty1 session, where it can't take DRM master.  Disable it with:
#   loginctl disable-linger $APP_USER
if id "$APP_USER" &>/dev/null; then
    PROFILE_DST="/home/$APP_USER/.bash_profile"
    PROFILE_MARKER="# breezy-box: start breezy.target on tty1"
    PROFILE_BLOCK="${PROFILE_MARKER}
if [ \"\$(tty)\" = \"/dev/tty1\" ]; then
    systemctl --user start breezy.target
fi"

    if grep -qF "$PROFILE_MARKER" "$PROFILE_DST" 2>/dev/null; then
        skip_msg "$PROFILE_DST already has breezy.target launch block"
    else
        echo "" >> "$PROFILE_DST"
        echo "$PROFILE_BLOCK" >> "$PROFILE_DST"
        chown "$APP_USER:$APP_USER" "$PROFILE_DST"
        done_msg "added breezy.target launch block to $PROFILE_DST"
    fi

    # Restart changed units if the user session is already running on tty1.
    if [[ "$UNITS_CHANGED" -eq 1 ]]; then
        USER_CMD="XDG_RUNTIME_DIR=/run/user/$(id -u "$APP_USER") systemctl --user"
        su -l "$APP_USER" -s /bin/bash -c \
            "$USER_CMD is-active --quiet breezy.target && $USER_CMD restart breezy.target" \
            2>/dev/null && done_msg "restarted breezy.target (units changed)" || true
    fi
else
    echo "  warn: user $APP_USER not found; .bash_profile will be configured when the user exists"
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
            -addext "subjectAltName=DNS:breezy.local,DNS:localhost,IP:192.168.77.2" \
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
# redirect incoming :80/:443 via nftables or iptables.
NFT_RULE_MARKER="# breezy-box: redirect ports 80->8081 and 443->8443"
if command -v nft &>/dev/null; then
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
echo
echo "System setup complete."
echo
echo "Next steps if not done yet:"
echo "  1. Create the app user:  useradd -m -s /bin/bash -c 'Breezy Box' $APP_USER"
echo "     Add to groups:        usermod -aG video,render,input $APP_USER"
echo "  2. Disable linger (if previously enabled — linger breaks DRM master):"
echo "     loginctl disable-linger $APP_USER"
echo "  3. Fetch vendored deps and build (run as $APP_USER):"
echo "     cd /home/$APP_USER/breezy-box && make deps && make"
echo "     cp displaylink_kms_renderer breezy_web ~/.local/bin/"
echo "  4. Trust the self-signed cert on each client:"
echo "     scp root@breezy.local:/etc/breezy-box/tls/server.crt ."
echo "     Then import server.crt into your browser / OS trust store."
echo "     The UI is served at https://breezy.local"
echo "  5. Reboot — getty autologin on tty1 will start the user session and breezy.target."
echo "     breezy-gadget.service runs as root at boot and sets up the OTG gadget;"
echo "     the renderer then runs unprivileged and adopts it."
