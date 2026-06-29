#!/usr/bin/env bash
#
# dl_only_boot.sh - bring up the DisplayLink-only gadget AT BOOT.
#
# For boards where power and the OTG host arrive on the SAME USB-C port (e.g.
# the Radxa Cubie A7S PD/OTG port): the host is electrically present the moment
# the board powers on, so there is no "after boot, before host-connect" window
# to run load_module.sh.  This script (run by dl-only-gadget.service, ordered
# before the normal breezy-gadget setup) loads g_rndis_displaylink with
# disable_rndis=1 so a pure DisplayLink device is already bound when the host
# enumerates.
#
# Reverts cleanly: `sudo systemctl disable --now dl-only-gadget.service` and the
# normal breezy-gadget.service takes over on the next boot.
#
set -uo pipefail

KO_DIR="/home/wayne/breezy-box/kernel/g_rndis_displaylink"
KO="$KO_DIR/g_rndis_displaylink.ko"
UDC="$(ls /sys/class/udc 2>/dev/null | head -1)"

log() { echo "dl-only-gadget: $*"; }

[[ -n "$UDC" ]] || { log "no UDC found"; exit 1; }
[[ -f "$KO" ]]  || { log "module not built at $KO"; exit 1; }

# Make sure nothing else owns the UDC (the configfs RNDIS gadget must not bind).
echo "" > /sys/kernel/config/usb_gadget/breezy-composite/UDC 2>/dev/null || true

# Dependencies + our module, DisplayLink only.
modprobe libcomposite 2>/dev/null || true
modprobe usb_f_rndis  2>/dev/null || true
if ! lsmod | grep -q '^g_rndis_displaylink'; then
    insmod "$KO" disable_rndis=1 || { log "insmod failed"; dmesg | tail -10; exit 1; }
fi

sleep 1
log "function=$(cat /sys/class/udc/$UDC/function 2>/dev/null) state=$(cat /sys/class/udc/$UDC/state 2>/dev/null)"
log "udl_gadget: $(ls -l /dev/udl_gadget 2>/dev/null || echo MISSING)"
