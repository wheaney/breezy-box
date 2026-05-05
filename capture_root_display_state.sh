#!/usr/bin/env bash

set -uo pipefail

has_cmd() {
    command -v "$1" >/dev/null 2>&1
}

usage() {
    cat <<'EOF'
Usage: capture_root_display_state.sh [--output-dir DIR] [--driver DRIVER]

Collect root-only DRM debugfs and kernel-side display state into a directory.
Run this script with sudo.

Examples:
  sudo ./capture_root_display_state.sh
  sudo ./capture_root_display_state.sh --driver udl --output-dir /tmp/bbx-root-capture
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

run_filtered_kernel_journal() {
    local name="$1"
    local outfile="$OUT_DIR/$name.txt"
    local status=0

    {
        printf '%s\n' '$ journalctl -k -b --no-pager | grep -Ei '\''drm|displaylink|udl|vga-1|breezy box|crtc|connector|hotplug|card0|card1'\''' 
        journalctl -k -b --no-pager | \
            grep -Ei 'drm|displaylink|udl|vga-1|breezy box|crtc|connector|hotplug|card0|card1' || status=$?
        printf '\n[exit=%d]\n' "$status"
    } >"$outfile" 2>&1
}

capture_debugfs_card() {
    local card_dir="$1"
    local card_name
    local outfile

    card_name="$(basename "$card_dir")"
    outfile="$OUT_DIR/debugfs_${card_name}.txt"

    {
        printf '## path\n%s\n\n' "$card_dir"
        printf '## entries\n'
        ls -la "$card_dir"
        printf '\n'

        for leaf in name clients framebuffer summary state; do
            if [[ -f "$card_dir/$leaf" ]]; then
                printf '## %s\n' "$leaf"
                cat "$card_dir/$leaf"
                printf '\n\n'
            fi
        done
    } >"$outfile" 2>&1
}

OUT_DIR=""
DRIVER="udl"

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
        --driver)
            if [[ $# -lt 2 ]]; then
                echo "error: --driver requires a value" >&2
                exit 1
            fi
            DRIVER="$2"
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
    OUT_DIR="host-display-root-state-$(date +%Y%m%d-%H%M%S)"
fi

mkdir -p "$OUT_DIR"

{
    printf 'timestamp=%s\n' "$(date --iso-8601=seconds)"
    printf 'hostname=%s\n' "$(hostname 2>/dev/null || echo unknown)"
    printf 'driver=%s\n' "$DRIVER"
    printf 'euid=%s\n' "$(id -u)"
} >"$OUT_DIR/metadata.txt"

run_cmd uname uname -a
run_cmd debugfs_mount mount
run_cmd dev_dri ls -l /dev/dri

if [[ -d /dev/dri/by-path ]]; then
    run_cmd dev_dri_by_path ls -l /dev/dri/by-path
fi

if [[ -d /sys/kernel/debug/dri ]]; then
    run_cmd debugfs_dri_listing ls -la /sys/kernel/debug/dri

    shopt -s nullglob
    for card_dir in /sys/kernel/debug/dri/[0-9]*; do
        [[ -d "$card_dir" ]] || continue
        capture_debugfs_card "$card_dir"
    done
fi

if has_cmd journalctl; then
    run_filtered_kernel_journal kernel_display_journal
fi

if has_cmd dmesg; then
    run_cmd dmesg_display dmesg -T
fi

if has_cmd modetest; then
    run_cmd "modetest_${DRIVER}" modetest -M "$DRIVER"
fi

{
    shopt -s nullglob
    for drm_path in /sys/class/drm/card*-*; do
        [[ -d "$drm_path" ]] || continue

        printf '## %s\n' "$drm_path"
        for leaf in status enabled modes dpms; do
            if [[ -f "$drm_path/$leaf" ]]; then
                printf '### %s\n' "$leaf"
                cat "$drm_path/$leaf"
                printf '\n'
            fi
        done
        printf '\n'
    done
} >"$OUT_DIR/drm_sysfs.txt" 2>&1

printf 'Wrote root capture to %s\n' "$OUT_DIR"