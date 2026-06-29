#!/usr/bin/env bash
#
# load_module.sh - swap the configfs RNDIS gadget for the g_rndis_displaylink
# composite (RNDIS + DisplayLink) kernel module.  Run as root, with the USB
# host UNPLUGGED (UDC state must be "not attached").
#
# Board/controller-specific quirks live in board_quirks.sh (sourced below) and
# are selected automatically from the UDC driver.  The module itself is
# controller-agnostic.
#
# Usage:
#   sudo ./load_module.sh            # RNDIS + DisplayLink composite
#   sudo ./load_module.sh --dl-only  # DisplayLink only (disable_rndis=1)
#
set -uo pipefail

MOD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KO="$MOD_DIR/g_rndis_displaylink.ko"
UDC="$(ls /sys/class/udc 2>/dev/null | head -1)"
INSMOD_ARGS=""

# shellcheck source=board_quirks.sh
source "$MOD_DIR/board_quirks.sh"

say() { printf '\n=== %s ===\n' "$*"; }

FORCE_ATTACHED=0
for arg in "$@"; do
    case "$arg" in
        --dl-only)        INSMOD_ARGS="disable_rndis=1" ;;
        --force-attached) FORCE_ATTACHED=1 ;;
        *) echo "unknown arg: $arg"; exit 1 ;;
    esac
done

if [[ $EUID -ne 0 ]]; then echo "run as root (sudo $0)"; exit 1; fi
[[ -f "$KO" ]] || { echo "missing $KO — build it first (make)"; exit 1; }
[[ -n "$UDC" ]] || { echo "no UDC found under /sys/class/udc"; exit 1; }

say "Board"
echo "UDC=$UDC driver=$(board_udc_driver) board=$(board_detect)"

say "UDC $UDC state"
state="$(cat /sys/class/udc/$UDC/state 2>/dev/null)"
speed="$(cat /sys/class/udc/$UDC/speed 2>/dev/null || cat /sys/class/udc/$UDC/current_speed 2>/dev/null)"
echo "state=$state speed='${speed:-}' function=$(cat /sys/class/udc/$UDC/function 2>/dev/null)"
if [[ "$state" == "configured" ]]; then
    # A genuine live host reports a negotiated speed; 'configured' with an empty
    # speed is a stale wedge latch (the disconnect was never processed).
    if [[ -n "$speed" && "$speed" != "UNKNOWN" ]]; then
        if [[ "$FORCE_ATTACHED" == "1" ]]; then
            echo "WARNING: host attached (speed=$speed) — proceeding anyway"
            echo "  (--force-attached).  On dwc3/ROCK this can hard-freeze the box;"
            echo "  intended for the sunxi/A733 runtime-swap test with dmesg -w watching."
        else
            echo "ABORT: UDC 'configured' with live speed=$speed (host attached)."
            echo "Unplug the host first, or pass --force-attached to swap on a live"
            echo "link (safe on sunxi/MUSB; risky on dwc3)."
            exit 1
        fi
    else
        echo "NOTE: 'configured' but speed empty — treating as a stale latch."
    fi
fi

say "Stopping renderer + configfs gadget"
systemctl stop breezy-renderer.service 2>/dev/null || true
echo "" > /sys/kernel/config/usb_gadget/breezy-composite/UDC 2>/dev/null || true
systemctl stop breezy-gadget.service 2>/dev/null || true
sleep 1

say "Board-specific pre-load (board_quirks.sh)"
board_pre_load
echo "function now: $(cat /sys/class/udc/$UDC/function 2>/dev/null || echo '(none/unbound)')"

say "Loading module ${INSMOD_ARGS:+($INSMOD_ARGS)}"
modprobe usb_f_rndis 2>&1 || true
# shellcheck disable=SC2086
insmod "$KO" $INSMOD_ARGS 2>&1 || { echo "insmod failed (dmesg tail:)"; dmesg | tail -15; exit 1; }

sleep 1
say "Result"
echo "lsmod:"; lsmod | grep -E 'g_rndis_displaylink|usb_f_rndis|libcomposite'
echo "udl_gadget:"; ls -l /dev/udl_gadget 2>/dev/null || echo "  /dev/udl_gadget MISSING"
echo "UDC function:"; cat /sys/class/udc/$UDC/function 2>/dev/null
echo "UDC state:";    cat /sys/class/udc/$UDC/state 2>/dev/null
say "dmesg (module)"
dmesg | grep -iE 'g_rndis_displaylink|f_displaylink' | tail -12
echo
echo "Now plug in the USB host and watch:"
echo "  sudo dmesg -w | grep -iE 'f_displaylink|probe|peek|poke|key|edid|bulk|channel'"
