#!/usr/bin/env bash

set -uo pipefail

has_cmd() {
    command -v "$1" >/dev/null 2>&1
}

usage() {
    cat <<'EOF'
Usage: capture_host_display_state.sh [--output-dir DIR] [--driver DRIVER]

Collect host-side DRM and GNOME Mutter DisplayConfig state into a directory.
The default DRM driver filter is `udl`.

Examples:
  ./capture_host_display_state.sh
  ./capture_host_display_state.sh --driver udl --output-dir /tmp/bbx-host-capture
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

run_filtered_journal() {
    local name="$1"
    local outfile="$OUT_DIR/$name.txt"
    local status=0

    {
        printf '%s\n' '$ journalctl --user -b --no-pager | grep -Ei '\''mutter|gnome-shell|displayconfig|crtc|displaylink|udl|vga-1|breezy box'\''' 
        journalctl --user -b --no-pager | \
            grep -Ei 'mutter|gnome-shell|displayconfig|crtc|displaylink|udl|vga-1|breezy box' || status=$?
        printf '\n[exit=%d]\n' "$status"
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

if [[ -z "$OUT_DIR" ]]; then
    OUT_DIR="host-display-state-$(date +%Y%m%d-%H%M%S)"
fi

mkdir -p "$OUT_DIR"

{
    printf 'timestamp=%s\n' "$(date --iso-8601=seconds)"
    printf 'hostname=%s\n' "$(hostname 2>/dev/null || echo unknown)"
    printf 'driver=%s\n' "$DRIVER"
} >"$OUT_DIR/metadata.txt"

run_cmd uname uname -a
run_cmd dev_dri ls -l /dev/dri

if [[ -d /dev/dri/by-path ]]; then
    run_cmd dev_dri_by_path ls -l /dev/dri/by-path
fi

if has_cmd lsusb; then
    run_cmd lsusb lsusb
    run_cmd lsusb_tree lsusb -t
fi

if has_cmd modetest; then
    run_cmd "modetest_${DRIVER}" modetest -M "$DRIVER"
fi

if has_cmd gdbus; then
    run_cmd displayconfig_introspect \
        gdbus introspect --session \
        --dest org.gnome.Mutter.DisplayConfig \
        --object-path /org/gnome/Mutter/DisplayConfig
    run_cmd displayconfig_getresources \
        gdbus call --session \
        --dest org.gnome.Mutter.DisplayConfig \
        --object-path /org/gnome/Mutter/DisplayConfig \
        --method org.gnome.Mutter.DisplayConfig.GetResources
    run_cmd displayconfig_getcurrentstate \
        gdbus call --session \
        --dest org.gnome.Mutter.DisplayConfig \
        --object-path /org/gnome/Mutter/DisplayConfig \
        --method org.gnome.Mutter.DisplayConfig.GetCurrentState
fi

if has_cmd journalctl; then
    run_filtered_journal user_display_journal
fi

if has_cmd udevadm; then
    shopt -s nullglob
    for node in /dev/dri/card*; do
        run_cmd "udevadm_$(basename "$node")" udevadm info --name="$node"
    done
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

printf 'Wrote capture to %s\n' "$OUT_DIR"