#!/usr/bin/env bash
#
# load_module.sh - swap the configfs RNDIS gadget for the g_rndis_displaylink
# composite (RNDIS + DisplayLink) kernel module.  Run as root, with the USB
# host UNPLUGGED (UDC state must be "not attached").
#
# SAFETY: never unbinds dwc3 and never touches a live link.  It only clears the
# configfs gadget's UDC (the safe, reversible teardown) while detached.
#
set -uo pipefail

MOD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KO="$MOD_DIR/g_rndis_displaylink.ko"
UDC="$(ls /sys/class/udc 2>/dev/null | head -1)"

say() { printf '\n=== %s ===\n' "$*"; }

if [[ $EUID -ne 0 ]]; then echo "run as root (sudo $0)"; exit 1; fi
[[ -f "$KO" ]] || { echo "missing $KO — build it first (make CC=gcc-13)"; exit 1; }
[[ -n "$UDC" ]] || { echo "no UDC found under /sys/class/udc"; exit 1; }

say "UDC $UDC state"
state="$(cat /sys/class/udc/$UDC/state 2>/dev/null)"
echo "state=$state  function=$(cat /sys/class/udc/$UDC/function 2>/dev/null)"
if [[ "$state" == "configured" ]]; then
    echo "ABORT: UDC is 'configured' (host attached). Unplug the host first —"
    echo "swapping gadgets on a live link can hard-freeze the box."
    exit 1
fi

say "Stopping renderer + configfs gadget"
systemctl stop breezy-renderer.service 2>/dev/null || true
# Unbind the configfs gadget from the UDC (its own ExecStop), then stop the unit.
echo "" > /sys/kernel/config/usb_gadget/breezy-composite/UDC 2>/dev/null || true
systemctl stop breezy-gadget.service 2>/dev/null || true
sleep 1
echo "function now: $(cat /sys/class/udc/$UDC/function 2>/dev/null || echo '(none/unbound)')"

say "Loading modules"
modprobe usb_f_rndis 2>&1 || true
insmod "$KO" 2>&1 || { echo "insmod failed (dmesg tail:)"; dmesg | tail -15; exit 1; }

sleep 1
say "Result"
echo "lsmod:"; lsmod | grep -E 'g_rndis_displaylink|usb_f_rndis|libcomposite'
echo "udl_gadget:"; ls -l /dev/udl_gadget 2>/dev/null || echo "  /dev/udl_gadget MISSING"
echo "UDC function:"; cat /sys/class/udc/$UDC/function 2>/dev/null
echo "UDC state:";    cat /sys/class/udc/$UDC/state 2>/dev/null
say "dmesg (module)"
dmesg | grep -iE 'g_rndis_displaylink|f_displaylink|dwc3' | tail -20
echo
echo "Now plug in the USB host and watch:  sudo dmesg -w | grep -iE 'f_displaylink|udl|dwc3'"
