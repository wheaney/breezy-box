#!/usr/bin/env bash
#
# unload_module.sh - remove the g_rndis_displaylink module and restore the
# normal configfs RNDIS gadget.  Run as root, with the USB host UNPLUGGED.
#
set -uo pipefail
UDC="$(ls /sys/class/udc 2>/dev/null | head -1)"
if [[ $EUID -ne 0 ]]; then echo "run as root (sudo $0)"; exit 1; fi

state="$(cat /sys/class/udc/$UDC/state 2>/dev/null)"
if [[ "$state" == "configured" ]]; then
    echo "ABORT: host attached ($state). Unplug first."
    exit 1
fi

echo "=== Removing module ==="
rmmod g_rndis_displaylink 2>&1 || echo "(was not loaded?)"
sleep 1

echo "=== Restoring configfs RNDIS gadget ==="
systemctl start breezy-gadget.service 2>&1 || true
systemctl start breezy-renderer.service 2>&1 || true
sleep 1
echo "UDC function: $(cat /sys/class/udc/$UDC/function 2>/dev/null)"
echo "UDC state:    $(cat /sys/class/udc/$UDC/state 2>/dev/null)"
