#!/usr/bin/env bash

set -uo pipefail

has_cmd() {
    command -v "$1" >/dev/null 2>&1
}

usage() {
    cat <<'EOF'
Usage: capture_usb_gadget_state.sh [--output-dir DIR] [--udc DEVICE]

Collect SBC-side USB gadget, UDC, role-switch, extcon, and controller state
into a timestamped directory. The capture also preserves mutable boot files,
overlay/configfs state, and local USB-related system configuration so a bad
post-reboot state can be compared against a future reimage or known-good boot.
Run this script with sudo while the gadget is in the failing state.

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

run_shell() {
    local name="$1"
    local script="$2"

    run_cmd "$name" bash -lc "$script"
}

capture_tree() {
    local name="$1"
    local root="$2"
    local outfile="$OUT_DIR/$name.txt"
    local resolved_root="$root"

    if [[ -L "$root" ]]; then
        resolved_root="$(readlink -f "$root" 2>/dev/null || printf '%s' "$root")"
    fi

    {
        printf '## root\n%s\n\n' "$root"
        printf '## resolved_root\n%s\n\n' "$resolved_root"
        if [[ ! -e "$resolved_root" ]]; then
            printf 'missing\n'
            exit 0
        fi

        find "$resolved_root" -maxdepth 4 \( -type f -o -type l \) | sort | while read -r path; do
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

capture_existing_tree() {
    local name="$1"
    local root="$2"

    if [[ -e "$root" ]]; then
        capture_tree "$name" "$root"
    fi
}

capture_existing_file() {
    local name="$1"
    local path="$2"

    if [[ -e "$path" ]]; then
        run_cmd "$name" cat "$path"
    fi
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
run_cmd mounts mount
run_cmd lsblk lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINT,MODEL
run_cmd sys_class_udc ls -la /sys/class/udc
run_cmd sys_class_usb_role ls -la /sys/class/usb_role
run_cmd sys_class_typec ls -la /sys/class/typec
run_cmd sys_class_extcon ls -la /sys/class/extcon
run_cmd sys_class_phy ls -la /sys/class/phy
run_cmd boot_dir ls -la /boot
run_cmd boot_overlay_user ls -la /boot/overlay-user
run_cmd proc_mtd cat /proc/mtd
run_cmd disk_by_path ls -la /dev/disk/by-path

capture_existing_file boot_armbianEnv /boot/armbianEnv.txt
capture_existing_file boot_cmd /boot/boot.cmd
capture_existing_file boot_scr_strings /boot/boot.scr

if [[ -e /boot/boot.scr ]]; then
    run_cmd boot_scr_strings strings /boot/boot.scr
fi

run_shell boot_dtb_link 'readlink -f /boot/dtb'
run_shell boot_image_link 'readlink -f /boot/Image'
run_shell boot_uinitrd_link 'readlink -f /boot/uInitrd'
run_shell boot_kernel_config 'config="/boot/config-$(uname -r)"; [[ -e "$config" ]] && cat "$config"'
run_shell boot_checksums '
    files=(/boot/armbianEnv.txt /boot/boot.cmd /boot/boot.scr)
    for path in "${files[@]}"; do
        [[ -e "$path" ]] && sha256sum "$path"
    done
    dtb_target="$(readlink -f /boot/dtb 2>/dev/null || true)"
    if [[ -n "$dtb_target" && -e "$dtb_target/rockchip/rk3399-rock-4c-plus.dtb" ]]; then
        sha256sum "$dtb_target/rockchip/rk3399-rock-4c-plus.dtb"
    fi
    if [[ -d /boot/overlay-user ]]; then
        find /boot/overlay-user -maxdepth 1 -type f -name "*.dtbo" -print0 | sort -z | xargs -0r sha256sum
    fi
'

run_shell package_versions '
    if command -v dpkg-query >/dev/null 2>&1; then
        dpkg-query -W -f="${Package} ${Version}\n" "linux-image*" "linux-dtb*" "linux-u-boot*" 2>/dev/null | sort
    fi
'
run_shell apt_history_usb '
    for path in /var/log/apt/history.log /var/log/apt/history.log.* /var/log/dpkg.log /var/log/dpkg.log.*; do
        [[ -e "$path" ]] || continue
        printf "## %s\n" "$path"
        if [[ "$path" == *.gz ]]; then
            zgrep -Ei "linux-image|linux-dtb|linux-u-boot|armbian|raw_gadget|dwc3|usb" "$path" || true
        else
            grep -Ei "linux-image|linux-dtb|linux-u-boot|armbian|raw_gadget|dwc3|usb" "$path" || true
        fi
        printf "\n"
    done
'

run_shell local_usb_config '
    roots=(/etc/systemd/system /etc/udev/rules.d /etc/modprobe.d /etc/modules-load.d /usr/local)
    for root in "${roots[@]}"; do
        [[ -e "$root" ]] || continue
        printf "## %s\n" "$root"
        find "$root" -type f | sort
        printf "\n"
    done
    printf "## usb-related matches\n"
    grep -RinE "usb|udc|gadget|raw_gadget|dwc3|fe800000|extcon|soft_connect|configfs" \
        /etc/systemd/system /etc/udev/rules.d /etc/modprobe.d /etc/modules-load.d /usr/local 2>/dev/null || true
'

capture_existing_tree sys_kernel_config_usb_gadget /sys/kernel/config/usb_gadget
capture_existing_tree sys_kernel_config_dt_overlays /sys/kernel/config/device-tree/overlays
capture_existing_tree proc_device_tree_usb0 /proc/device-tree/usb@fe800000
capture_existing_tree proc_device_tree_usb1 /proc/device-tree/usb@fe900000
capture_existing_tree proc_device_tree_u2phy0 /proc/device-tree/syscon@ff770000/usb2phy@e450
capture_existing_tree proc_device_tree_chosen /proc/device-tree/chosen

run_shell live_dt_usb_summary '
    if command -v fdtdump >/dev/null 2>&1; then
        fdtdump /sys/firmware/fdt 2>/dev/null | grep -A20 -B5 "fe800000\\|fe900000\\|usb2phy@e450\\|dr_mode" || true
    fi
'

run_shell mmc_devices '
    for d in /sys/bus/mmc/devices/mmc*; do
        [[ -d "$d" ]] || continue
        echo "== $d =="
        for f in type name manfid oemid cid csd date serial; do
            [[ -f "$d/$f" ]] && echo "$f: $(cat "$d/$f")"
        done
        readlink -f "$d" || true
        echo
    done
'

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
    run_cmd journal_usb_filtered journalctl -k -b --no-pager -g 'dwc|dwc2|dwc3|udc|usb|role|typec|extcon|phy|fe800000|rockchip|raw_gadget|gadget'
fi

printf 'Wrote USB gadget capture to %s\n' "$OUT_DIR"