#!/usr/bin/env bash

set -uo pipefail

has_cmd() {
    command -v "$1" >/dev/null 2>&1
}

usage() {
    cat <<'EOF'
Usage: capture_usb_gadget_state.sh [--output-dir DIR] [--udc DEVICE]

Collect SBC-side USB gadget, UDC, role-switch, extcon, and controller state
into a timestamped directory. Run this script with sudo while the gadget is in
the failing state.

Examples:
  sudo ./capture_usb_gadget_state.sh
  sudo ./capture_usb_gadget_state.sh --udc fe800000.usb
  sudo ./capture_usb_gadget_state.sh --output-dir /tmp/udc-capture
EOF
}

run_cmd() {
    local name="$1"
    local outfile="$OUT_DIR/$name.txt"
    local status=0

    shift

    {
        printf '$'
        printf ' %q' "$@"
        printf '\n'
        "$@" || status=$?
        printf '\n[exit=%d]\n' "$status"
    } >"$outfile" 2>&1
}

capture_tree() {
    local name="$1"
    local root="$2"
    local outfile="$OUT_DIR/$name.txt"

    {
        printf '## root\n%s\n\n' "$root"
        if [[ ! -e "$root" ]]; then
            printf 'missing\n'
            exit 0
        fi

        find "$root" -maxdepth 3 \( -type f -o -type l \) | sort | while read -r path; do
            printf '## %s\n' "$path"
            if [[ -L "$path" ]]; then
                readlink "$path" || true
            elif [[ -r "$path" ]]; then
                cat "$path" 2>/dev/null || true
            else
                printf 'unreadable\n'
            fi
            printf '\n\n'
        done
    } >"$outfile" 2>&1
}

run_filtered_dmesg() {
    local name="$1"
    local outfile="$OUT_DIR/$name.txt"
    local status=0

    {
        printf '%s\n' '$ dmesg -T | grep -Ei '\''dwc|dwc2|dwc3|udc|usb|role|typec|extcon|phy|fe800000|rockchip'\''' 
        dmesg -T | \
            grep -Ei 'dwc|dwc2|dwc3|udc|usb|role|typec|extcon|phy|fe800000|rockchip' || status=$?
        printf '\n[exit=%d]\n' "$status"
    } >"$outfile" 2>&1
}

OUT_DIR=""
UDC_DEVICE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output-dir)
            if [[ $# -lt 2 ]]; then
                echo "error: --output-dir requires a value" >&2
                exit 1
            fi
            OUT_DIR="$2"
            shift 2
            ;;
        --udc)
            if [[ $# -lt 2 ]]; then
                echo "error: --udc requires a value" >&2
                exit 1
            fi
            UDC_DEVICE="$2"
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

if [[ "$(id -u)" -ne 0 ]]; then
    echo "error: run this script with sudo" >&2
    exit 1
fi

if [[ -z "$OUT_DIR" ]]; then
    OUT_DIR="usb-gadget-state-$(date +%Y%m%d-%H%M%S)"
fi

if [[ -z "$UDC_DEVICE" ]]; then
    shopt -s nullglob
    udc_entries=(/sys/class/udc/*)
    if [[ ${#udc_entries[@]} -eq 1 ]]; then
        UDC_DEVICE="$(basename "${udc_entries[0]}")"
    fi
fi

mkdir -p "$OUT_DIR"

{
    printf 'timestamp=%s\n' "$(date --iso-8601=seconds)"
    printf 'hostname=%s\n' "$(hostname 2>/dev/null || echo unknown)"
    printf 'udc_device=%s\n' "${UDC_DEVICE:-unknown}"
    printf 'euid=%s\n' "$(id -u)"
} >"$OUT_DIR/metadata.txt"

run_cmd uname uname -a
run_cmd cmdline cat /proc/cmdline
run_cmd lsmod lsmod
run_cmd sys_class_udc ls -la /sys/class/udc
run_cmd sys_class_usb_role ls -la /sys/class/usb_role
run_cmd sys_class_typec ls -la /sys/class/typec
run_cmd sys_class_extcon ls -la /sys/class/extcon
run_cmd sys_class_phy ls -la /sys/class/phy

if [[ -n "$UDC_DEVICE" && -d "/sys/class/udc/$UDC_DEVICE" ]]; then
    capture_tree "udc_${UDC_DEVICE}" "/sys/class/udc/$UDC_DEVICE"
    run_cmd "udc_${UDC_DEVICE}_uevent" cat "/sys/class/udc/$UDC_DEVICE/uevent"
    if [[ -L "/sys/class/udc/$UDC_DEVICE/device/driver" ]]; then
        run_cmd "udc_${UDC_DEVICE}_driver_link" readlink -f "/sys/class/udc/$UDC_DEVICE/device/driver"
    fi
    if [[ -f "/sys/class/udc/$UDC_DEVICE/device/modalias" ]]; then
        run_cmd "udc_${UDC_DEVICE}_modalias" cat "/sys/class/udc/$UDC_DEVICE/device/modalias"
    fi
    if [[ -d "/sys/class/udc/$UDC_DEVICE/device" ]]; then
        capture_tree "udc_${UDC_DEVICE}_device" "/sys/class/udc/$UDC_DEVICE/device"
    fi
fi

for path in /sys/class/usb_role/* /sys/class/typec/* /sys/class/extcon/* /sys/class/phy/*; do
    [[ -e "$path" ]] || continue
    capture_tree "$(basename "$(dirname "$path")")_$(basename "$path")" "$path"
done

for path in /sys/kernel/debug/usb /sys/kernel/debug/phy; do
    [[ -d "$path" ]] || continue
    capture_tree "debug_$(basename "$path")" "$path"
done

if has_cmd dmesg; then
    run_filtered_dmesg kernel_usb_dmesg
fi

if has_cmd journalctl; then
    run_cmd journal_kernel_usb journalctl -k -b --no-pager
fi

printf 'Wrote USB gadget capture to %s\n' "$OUT_DIR"