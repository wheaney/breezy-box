#!/usr/bin/env bash
# Set up the FunctionFS DisplayLink USB gadget via configfs.
# Run as root (e.g. by breezy-gadget.service) before the renderer starts.
#
# The gadget exposes a single vendor-specific DisplayLink interface backed by
# FunctionFS: the unprivileged renderer (ffs_gadget.c) opens the functionfs
# mount, writes the USB descriptors, and drains the bulk OUT endpoint into its
# decode runtime.  No kernel module and no /dev/raw-gadget are required.
#
# Sequencing note: the UDC can only be bound once the FunctionFS function has
# its descriptors (written by the renderer).  This oneshot therefore only builds
# the configfs skeleton + mounts functionfs, then spawns a DETACHED, PERSISTENT
# poller that loops for as long as the box is up (see bind_poller below) binding
# the UDC whenever the renderer's ep1 is present and unbinding when it isn't —
# so a bare `systemctl restart breezy-renderer.service` (which the OTG port's
# always-on power/host attachment means has no other reattach signal) still
# results in the gadget coming back, without needing breezy-gadget.service
# itself to restart.  This also keeps breezy-gadget.service (Type=oneshot,
# RemainAfterExit) from blocking the renderer it is ordered before.
#
# Idempotent: if the gadget is already bound to a UDC this script exits 0
# (leaving any already-running poller alone).

set -uo pipefail

GADGET_NAME="breezy-displaylink"
GADGET_ROOT="/sys/kernel/config/usb_gadget/${GADGET_NAME}"
FFS_NAME="dl"                         # functions/ffs.${FFS_NAME}
FFS_MOUNT="/dev/ffs-dl"               # must match FFS_GADGET_DEFAULT_MOUNT in ffs_gadget.h
POLLER_PIDFILE="/run/breezy-gadget-bind-poller.pid"

# The app user that runs the renderer; used for the functionfs mount uid/gid so
# the unprivileged renderer can open ep0/ep1.  setup_system.sh substitutes the
# placeholder; otherwise fall back to BREEZY_APP_USER, then the invoking user.
APP_USER="__APP_USER__"
[[ "$APP_USER" == "__APP_USER__" ]] && APP_USER="${BREEZY_APP_USER:-${SUDO_USER:-root}}"

say() { echo "breezy-gadget: $*"; }

# If already bound, nothing to do.
if [[ -f "${GADGET_ROOT}/UDC" ]]; then
    udc="$(cat "${GADGET_ROOT}/UDC" 2>/dev/null || true)"
    udc="${udc%$'\n'}"
    if [[ -n "$udc" ]]; then
        say "gadget already bound to UDC $udc, skipping"
        exit 0
    fi
fi

# Remove any leftover kernel module / RNDIS gadget that would block the UDC.
rmmod g_rndis_displaylink 2>/dev/null || true

mount -t configfs none /sys/kernel/config 2>/dev/null || true
modprobe libcomposite 2>/dev/null || true
modprobe usb_f_fs 2>/dev/null || true

# Detect UDC.
mapfile -t udcs < <(ls /sys/class/udc/ 2>/dev/null)
if [[ ${#udcs[@]} -eq 0 ]]; then
    say "no UDC found" >&2
    exit 1
fi
if [[ ${#udcs[@]} -gt 1 ]]; then
    say "multiple UDCs found: ${udcs[*]}; set udc_name explicitly" >&2
    exit 1
fi
UDC="${udcs[0]}"

teardown() {
    local root="$1"
    [[ -f "${root}/UDC" ]] && printf '\n' > "${root}/UDC" 2>/dev/null || true
    umount "$FFS_MOUNT" 2>/dev/null || true
    rm -f  "${root}/configs/c.1/ffs.${FFS_NAME}"   2>/dev/null || true
    rmdir  "${root}/functions/ffs.${FFS_NAME}"     2>/dev/null || true
    rmdir  "${root}/configs/c.1/strings/0x409"     2>/dev/null || true
    rmdir  "${root}/configs/c.1"                    2>/dev/null || true
    rmdir  "${root}/strings/0x409"                 2>/dev/null || true
    rmdir  "${root}"                               2>/dev/null || true
}

teardown "$GADGET_ROOT"
mkdir -p "$GADGET_ROOT" || { say "mkdir $GADGET_ROOT failed" >&2; exit 1; }

wf() { printf '%s' "$2" > "${GADGET_ROOT}/$1"; }
md() { mkdir -p "${GADGET_ROOT}/$1"; }

# DisplayLink USB identity (vendor-specific device; the per-interface class is
# set in the FunctionFS descriptors the renderer writes).
wf idVendor         "0x17e9"   # DisplayLink
wf idProduct        "0x037a"
wf bcdUSB           "0x0200"
wf bcdDevice        "0x0104"
wf bDeviceClass     "0x00"
wf bDeviceSubClass  "0x00"
wf bDeviceProtocol  "0x00"

md strings/0x409
wf strings/0x409/serialnumber  "BREEZY0001"
wf strings/0x409/manufacturer  "DisplayLink"
wf strings/0x409/product       "DisplayLink Adapter"

md configs/c.1
md configs/c.1/strings/0x409
wf configs/c.1/strings/0x409/configuration "Breezy DisplayLink"
wf configs/c.1/MaxPower "250"

# FunctionFS function instance.
md "functions/ffs.${FFS_NAME}"
ln -sf "${GADGET_ROOT}/functions/ffs.${FFS_NAME}" "${GADGET_ROOT}/configs/c.1/ffs.${FFS_NAME}"

# Mount functionfs so the renderer can open ep0/ep1.  uid/gid hand ownership to
# the app user; without that the unprivileged renderer cannot open the endpoints.
uid="$(id -u "$APP_USER" 2>/dev/null || echo 0)"
gid="$(id -g "$APP_USER" 2>/dev/null || echo 0)"
mkdir -p "$FFS_MOUNT"
if ! mountpoint -q "$FFS_MOUNT"; then
    mount -t functionfs -o "uid=${uid},gid=${gid}" "$FFS_NAME" "$FFS_MOUNT" \
        || { say "mount functionfs at $FFS_MOUNT failed" >&2; exit 1; }
fi
say "functionfs mounted at $FFS_MOUNT (uid=$uid gid=$gid), UDC=$UDC ready to bind"

# Persistent poller: the OTG port is always the power source, so the host is
# already attached before any of this runs and stays attached indefinitely —
# there is no plug-in event to react to, and no service-restart event either
# when only the RENDERER restarts (closing/reopening ep0, which the kernel
# turns into UNBIND/BIND on the function, but nothing else watches for that).
# So this loops for as long as the box is up, tracking ep1's presence as the
# renderer's "I'm alive and wrote descriptors" signal:
#   ep1 appears  -> bind the UDC + pulse a re-enumerate (host sees a fresh device)
#   ep1 vanishes -> unbind (renderer exited/restarted; avoid a half-dead gadget)
#   ep1 reappears -> bind again (this is what makes a bare renderer restart work
#                     without anyone touching breezy-gadget.service)
# Runs detached from this oneshot so it outlives ExecStart returning.
bind_poller() {
    local root="$1" udc="$2" mount="$3"
    local bound=0
    local fail_streak=0
    local backoff=1

    while true; do
        if [[ -e "${mount}/ep1" ]]; then
            if (( ! bound )); then
                if printf '%s' "$udc" > "${root}/UDC" 2>/dev/null; then
                    # Re-enumerate so a host already attached (always true here)
                    # sees a fresh device rather than whatever stale state the
                    # bus was last left in.
                    sleep 0.1
                    printf '\n' > "${root}/UDC" 2>/dev/null || true
                    sleep 0.1
                    printf '%s' "$udc" > "${root}/UDC" 2>/dev/null || true
                    say "DisplayLink FFS gadget bound to UDC $udc"
                    bound=1
                    fail_streak=0
                    backoff=1
                else
                    # A persistently failing bind usually means the UDC is wedged
                    # in a stale "configured" state from a previous instance that
                    # never released it (kernel log: "couldn't find an available
                    # UDC or it's busy") — that needs a manual unbind/reset, this
                    # loop retrying every second can't fix it.  So: log once
                    # immediately, then back off (capped at 60s) and only log
                    # every ~10th retry past that, instead of spamming once/sec
                    # forever (which previously forced killing the whole service
                    # just to silence the journal).
                    (( fail_streak++ ))
                    if (( fail_streak == 1 || fail_streak % 10 == 0 )); then
                        say "UDC bind failed (attempt $fail_streak); if this persists the UDC is likely wedged from a stale bind — check 'cat ${root}/UDC' and the gadget's own state, may need a manual unbind/reset" >&2
                    fi
                    sleep "$backoff"
                    (( backoff < 60 )) && backoff=$(( backoff * 2 < 60 ? backoff * 2 : 60 ))
                    continue
                fi
            fi
        else
            if (( bound )); then
                printf '\n' > "${root}/UDC" 2>/dev/null || true
                say "renderer disconnected from $mount, UDC unbound (waiting for it to come back)"
                bound=0
            fi
            fail_streak=0
            backoff=1
        fi
        sleep 1
    done
}

# Stop any poller left running from a previous invocation of this script
# (e.g. a prior breezy-gadget.service start) before spawning a new one — the
# poller loops forever, so without this guard a second run stacks a second
# infinite loop fighting the first one over the same UDC file.
if [[ -f "$POLLER_PIDFILE" ]]; then
    old_pid="$(cat "$POLLER_PIDFILE" 2>/dev/null || true)"
    if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
        kill "$old_pid" 2>/dev/null || true
        say "stopped previous bind poller (pid $old_pid)"
    fi
    rm -f "$POLLER_PIDFILE"
fi

setsid bash -c "$(declare -f say bind_poller); bind_poller '$GADGET_ROOT' '$UDC' '$FFS_MOUNT'" \
    </dev/null >/dev/null 2>&1 &
echo "$!" > "$POLLER_PIDFILE"
disown 2>/dev/null || true

exit 0
