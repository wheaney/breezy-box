#!/usr/bin/env bash

set -uo pipefail

usage() {
    cat <<'EOF'
Usage: watch_usb_gadget_attach.sh [--udc DEVICE] [--interval SEC]

Continuously print the SBC-side USB gadget attach state, including the UDC,
the DWC3 debugfs mode/link state, and the matching RK3399 USB2 PHY extcon.
Run this while plugging and unplugging the host cable to see which layer
changes first.

Examples:
  sudo ./watch_usb_gadget_attach.sh
  sudo ./watch_usb_gadget_attach.sh --udc fe800000.usb --interval 0.5
EOF
}

UDC_DEVICE="fe800000.usb"
INTERVAL="1"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --udc)
            if [[ $# -lt 2 ]]; then
                echo "error: --udc requires a value" >&2
                exit 1
            fi
            UDC_DEVICE="$2"
            shift 2
            ;;
        --interval)
            if [[ $# -lt 2 ]]; then
                echo "error: --interval requires a value" >&2
                exit 1
            fi
            INTERVAL="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

read_one() {
    local path="$1"

    if [[ -r "$path" ]]; then
        cat "$path" 2>/dev/null | tr '\n' ' ' | sed 's/[[:space:]]\+/ /g; s/[[:space:]]$//'
    else
        printf 'unavailable'
    fi
}

resolve_otg_extcon() {
    local direct_glob='/sys/devices/platform/ff770000.syscon/ff770000.syscon:usb2phy@e450/extcon/extcon*'
    local path

    for path in $direct_glob; do
        if [[ -d "$path" ]]; then
            printf '%s' "$path"
            return 0
        fi
    done

    for path in /sys/class/extcon/*; do
        [[ -e "$path" ]] || continue
        if [[ "$(readlink -f "$path/device" 2>/dev/null || true)" == *"usb2phy@e450"* ]]; then
            printf '%s' "$path"
            return 0
        fi
    done

    printf '/sys/devices/platform/ff770000.syscon/ff770000.syscon:usb2phy@e450/extcon/extcon?'
}

OTG_EXTCON="$(resolve_otg_extcon)"

while true; do
    clear
    printf 'time: %s\n' "$(date --iso-8601=seconds)"
    printf 'udc: %s\n' "$UDC_DEVICE"
    printf 'udc state: %s\n' "$(read_one "/sys/class/udc/$UDC_DEVICE/state")"
    printf 'udc current_speed: %s\n' "$(read_one "/sys/class/udc/$UDC_DEVICE/current_speed")"
    printf 'udc function: %s\n' "$(read_one "/sys/class/udc/$UDC_DEVICE/function")"
    printf 'dwc3 mode: %s\n' "$(read_one "/sys/kernel/debug/usb/$UDC_DEVICE/mode")"
    printf 'dwc3 link_state: %s\n' "$(read_one "/sys/kernel/debug/usb/$UDC_DEVICE/link_state")"
    printf 'dwc3 soft_connect present: '
    if [[ -e "/sys/class/udc/$UDC_DEVICE/soft_connect" ]]; then
        printf 'yes\n'
    else
        printf 'no\n'
    fi
    printf 'otg extcon: %s\n' "$OTG_EXTCON"
    printf 'otg extcon state: %s\n' "$(read_one "$OTG_EXTCON/state")"
    printf '\nPress Ctrl+C to stop.\n'
    sleep "$INTERVAL"
done