#!/usr/bin/env bash
# Set up the OTG RNDIS USB gadget via configfs.
# Run as root (e.g. by breezy-gadget.service) before the renderer starts.
# The renderer detects the bound gadget and skips this work entirely.
#
# Idempotent: if the gadget is already bound to a UDC this script exits 0.

set -euo pipefail

GADGET_NAME="breezy-composite"
GADGET_ROOT="/sys/kernel/config/usb_gadget/${GADGET_NAME}"

# If already bound, nothing to do.
if [[ -f "${GADGET_ROOT}/UDC" ]]; then
    udc="$(cat "${GADGET_ROOT}/UDC" 2>/dev/null || true)"
    udc="${udc%$'\n'}"
    if [[ -n "$udc" ]]; then
        echo "breezy-gadget: gadget already bound to UDC $udc, skipping"
        exit 0
    fi
fi

# Remove any leftover kernel module that would block the UDC.
rmmod g_rndis_displaylink 2>/dev/null || true

mount -t configfs none /sys/kernel/config 2>/dev/null || true

# Load modules and verify they are present before touching configfs.
# usb_f_rndis must be loaded BEFORE mkdir functions/rndis.usb0 — otherwise
# configfs creates a plain directory with no function backing, and the kernel
# will accept the UDC bind but fail every USB descriptor request.
modprobe libcomposite
modprobe usb_f_rndis

if ! grep -q '^usb_f_rndis ' /proc/modules 2>/dev/null; then
    echo "breezy-gadget: usb_f_rndis module failed to load" >&2
    exit 1
fi

# Detect UDC.
mapfile -t udcs < <(ls /sys/class/udc/ 2>/dev/null)
if [[ ${#udcs[@]} -eq 0 ]]; then
    echo "breezy-gadget: no UDC found" >&2
    exit 1
fi
if [[ ${#udcs[@]} -gt 1 ]]; then
    echo "breezy-gadget: multiple UDCs found: ${udcs[*]}; set udc_name explicitly" >&2
    exit 1
fi
UDC="${udcs[0]}"

teardown() {
    local root="$1"
    # Unbind first; ignore errors (kernel may reject if not bound).
    printf '\n' > "${root}/UDC" 2>/dev/null || true
    rm -f  "${root}/configs/c.1/rndis.usb0"   2>/dev/null || true
    rm -f  "${root}/os_desc/c.1"               2>/dev/null || true
    rmdir  "${root}/functions/rndis.usb0/os_desc/interface.rndis" 2>/dev/null || true
    rmdir  "${root}/functions/rndis.usb0"      2>/dev/null || true
    rmdir  "${root}/configs/c.1/strings/0x409" 2>/dev/null || true
    rmdir  "${root}/configs/c.1"               2>/dev/null || true
    rmdir  "${root}/strings/0x409"             2>/dev/null || true
    rmdir  "${root}"                           2>/dev/null || true
}

teardown "$GADGET_ROOT"
mkdir -p "$GADGET_ROOT"

wf() { printf '%s' "$2" > "${GADGET_ROOT}/$1"; }
md() { mkdir -p "${GADGET_ROOT}/$1"; }

wf idVendor         "0x1d6b"
wf idProduct        "0x0104"
wf bcdUSB           "0x0200"
wf bcdDevice        "0x0100"
wf bDeviceClass     "0xEF"
wf bDeviceSubClass  "0x02"
wf bDeviceProtocol  "0x01"

md strings/0x409
wf strings/0x409/serialnumber  "BREEZY0001"
wf strings/0x409/manufacturer  "Breezy Box"
wf strings/0x409/product       "Breezy Box OTG RNDIS"

md configs/c.1
md configs/c.1/strings/0x409
wf configs/c.1/strings/0x409/configuration "Breezy RNDIS"
wf configs/c.1/MaxPower "250"

wf os_desc/use           "1"
wf os_desc/b_vendor_code "0xcd"

md functions/rndis.usb0
# Verify the kernel created the function backing (requires usb_f_rndis loaded above).
if [[ ! -f "${GADGET_ROOT}/functions/rndis.usb0/dev_addr" ]]; then
    echo "breezy-gadget: functions/rndis.usb0 has no dev_addr — usb_f_rndis not active?" >&2
    exit 1
fi
wf functions/rndis.usb0/dev_addr  "02:1a:11:00:00:01"
wf functions/rndis.usb0/host_addr "02:1a:11:00:00:02"
wf "functions/rndis.usb0/os_desc/interface.rndis/compatible_id"     "RNDIS"
wf "functions/rndis.usb0/os_desc/interface.rndis/sub_compatible_id" "5162001"

ln -sf "${GADGET_ROOT}/functions/rndis.usb0" "${GADGET_ROOT}/configs/c.1/rndis.usb0"
ln -sf "${GADGET_ROOT}/configs/c.1"          "${GADGET_ROOT}/os_desc/c.1" 2>/dev/null || true

# Bind to UDC to make the gadget live on the bus.
wf UDC "$UDC"

# Toggle unbind/rebind to force a USB disconnect pulse so any host that was
# already connected when this ran sees a fresh enumeration.  Use separate
# writes rather than set -e so a failed unbind doesn't abort before rebind.
sleep 0.05
printf '\n' > "${GADGET_ROOT}/UDC" 2>/dev/null || true
sleep 0.05
wf UDC "$UDC"

echo "breezy-gadget: RNDIS gadget bound to UDC $UDC"
