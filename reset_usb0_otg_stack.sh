#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage: reset_usb0_otg_stack.sh [--udc DEVICE] [--settle SEC] [--dry-run]

Force a deeper recovery of the usb0 OTG stack than the raw-gadget process can do
by itself. The script detects the dwc3 platform device behind the selected UDC,
finds the matching usb2phy OTG PHY provider, unbinds and rebinds them in order,
then reports the post-reset UDC and extcon state.

This recovery path targets the RK3399 dwc3 + usb2phy stack (the BC1.2 DCP
misclassification quirk). On other SoCs the dwc3/usb2phy nodes it relies on may
not exist; in that case it reports what it found and exits cleanly without
touching anything.

When --udc is not given the single bound UDC under /sys/class/udc is used.

This is intended for the bad state where:
  - the UDC still exists
  - the controller remains in device mode but not attached
  - current_speed stays UNKNOWN
  - the usb2phy OTG extcon reports DCP=1 instead of CDP=1

Examples:
  sudo ./reset_usb0_otg_stack.sh
  sudo ./reset_usb0_otg_stack.sh --udc fe800000.usb --settle 2
  sudo ./reset_usb0_otg_stack.sh --dry-run
EOF
}

# Empty default: auto-detect the bound UDC after argument parsing unless the
# caller passes --udc explicitly.  Avoids hardcoding an RK3399 address.
UDC_DEVICE=""
SETTLE_SEC="1"
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --udc)
            [[ $# -ge 2 ]] || { echo "error: --udc requires a value" >&2; exit 1; }
            UDC_DEVICE="$2"
            shift 2
            ;;
        --settle)
            [[ $# -ge 2 ]] || { echo "error: --settle requires a value" >&2; exit 1; }
            SETTLE_SEC="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=1
            shift
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

if [[ $EUID -ne 0 ]]; then
    echo "error: this script must run as root" >&2
    exit 1
fi

# Auto-detect the UDC unless one was given explicitly.  This keeps the script
# board-agnostic: the dwc3/phy/extcon discovery below all keys off the UDC, so
# detecting it here is enough to follow whatever controller the SoC exposes.
if [[ -z "$UDC_DEVICE" ]]; then
    mapfile -t _udcs < <(ls /sys/class/udc/ 2>/dev/null)
    if [[ ${#_udcs[@]} -eq 0 ]]; then
        echo "error: no UDC found under /sys/class/udc; nothing to reset" >&2
        exit 1
    fi
    if [[ ${#_udcs[@]} -gt 1 ]]; then
        echo "error: multiple UDCs found: ${_udcs[*]}; pass --udc to choose one" >&2
        exit 1
    fi
    UDC_DEVICE="${_udcs[0]}"
fi

read_one() {
    local path="$1"

    if [[ -r "$path" ]]; then
        tr '\n' ' ' < "$path" 2>/dev/null | sed 's/[[:space:]]\+/ /g; s/[[:space:]]$//'
    else
        printf 'unavailable'
    fi
}

run_cmd() {
    printf '+ %s\n' "$*"
    if [[ $DRY_RUN -eq 0 ]]; then
        "$@"
    fi
}

write_value() {
    local value="$1"
    local path="$2"

    printf '+ printf %%s %q > %s\n' "$value" "$path"
    if [[ $DRY_RUN -eq 0 ]]; then
        printf '%s' "$value" > "$path"
    fi
}

write_optional_value() {
    local value="$1"
    local path="$2"
    local label="$3"
    local error_text

    printf '+ printf %%s %q > %s\n' "$value" "$path"
    if [[ $DRY_RUN -ne 0 ]]; then
        return 0
    fi

    if error_text=$(printf '%s' "$value" > "$path" 2>&1); then
        return 0
    fi

    printf 'note: skipping %s write %q -> %s (%s)\n' \
        "$label" "$value" "$path" "${error_text:-write failed}" >&2
    return 0
}

device_driver_name() {
    local device_path="$1"
    local driver_path

    driver_path="$(readlink -f "$device_path/driver" 2>/dev/null || true)"
    if [[ -z "$driver_path" ]]; then
        return 1
    fi

    basename "$driver_path"
}

write_driver_action() {
    local action="$1"
    local device_name="$2"
    local driver_name="$3"
    local device_path="$4"
    local control_path="/sys/bus/platform/drivers/$driver_name/$action"
    local error_text
    local current_driver

    printf '+ printf %%s %q > %s\n' "$device_name" "$control_path"
    if [[ $DRY_RUN -ne 0 ]]; then
        return 0
    fi

    if error_text=$(printf '%s' "$device_name" > "$control_path" 2>&1); then
        return 0
    fi

    current_driver="$(device_driver_name "$device_path" || true)"
    if [[ "$action" == bind && "$current_driver" == "$driver_name" ]]; then
        printf 'note: %s is already bound to %s; continuing after bind race (%s)\n' \
            "$device_name" "$driver_name" "${error_text:-write failed}" >&2
        return 0
    fi
    if [[ "$action" == unbind && "$current_driver" != "$driver_name" ]]; then
        printf 'note: %s is no longer bound to %s; continuing after unbind race (%s)\n' \
            "$device_name" "$driver_name" "${error_text:-write failed}" >&2
        return 0
    fi

    printf 'error: %s %s via %s failed (%s)\n' \
        "$action" "$device_name" "$driver_name" "${error_text:-write failed}" >&2
    return 1
}

require_path() {
    local path="$1"
    local label="$2"

    if [[ ! -e "$path" ]]; then
        echo "error: missing $label at $path" >&2
        exit 1
    fi
}

find_otg_phy() {
    local dwc3_device="$1"
    local phy_path
    local candidate

    for candidate in /sys/class/phy/*; do
        [[ -e "$candidate" ]] || continue
        if [[ -e "$candidate/consumer:platform:$dwc3_device" ]] &&
           [[ "$(read_one "$candidate/uevent")" == *"OF_NAME=otg-port"* ]]; then
            phy_path="$(readlink -f "$candidate/device")"
            if [[ -n "$phy_path" ]]; then
                printf '%s' "$phy_path"
                return 0
            fi
        fi
    done

    return 1
}

find_extcon_for_phy() {
    local phy_device="$1"
    local path

    for path in /sys/class/extcon/*; do
        [[ -e "$path" ]] || continue
        if [[ "$(readlink -f "$path/device" 2>/dev/null || true)" == "$phy_device"* ]]; then
            printf '%s' "$path"
            return 0
        fi
    done

    for path in "$phy_device"/extcon/extcon*; do
        [[ -d "$path" ]] || continue
        printf '%s' "$path"
        return 0
    done

    return 1
}

show_state() {
    local header="$1"
    local extcon_path="$2"

    printf '\n[%s]\n' "$header"
    printf 'udc: %s\n' "$UDC_DEVICE"
    printf 'udc state: %s\n' "$(read_one "/sys/class/udc/$UDC_DEVICE/state")"
    printf 'udc current_speed: %s\n' "$(read_one "/sys/class/udc/$UDC_DEVICE/current_speed")"
    printf 'udc function: %s\n' "$(read_one "/sys/class/udc/$UDC_DEVICE/function")"
    printf 'dwc3 mode: %s\n' "$(read_one "/sys/kernel/debug/usb/$UDC_DEVICE/mode")"
    printf 'dwc3 link_state: %s\n' "$(read_one "/sys/kernel/debug/usb/$UDC_DEVICE/link_state")"
    printf 'otg extcon: %s\n' "$extcon_path"
    printf 'otg extcon state: %s\n' "$(read_one "$extcon_path/state")"
}

require_path "/sys/class/udc/$UDC_DEVICE" "UDC"

DWC3_DEVICE_PATH="$(readlink -f "/sys/class/udc/$UDC_DEVICE/device")"
DWC3_DRIVER_PATH="$(readlink -f "/sys/class/udc/$UDC_DEVICE/device/driver")"
require_path "$DWC3_DEVICE_PATH" "DWC3 device"
require_path "$DWC3_DRIVER_PATH" "DWC3 driver"

DWC3_DEVICE="$(basename "$DWC3_DEVICE_PATH")"
DWC3_DRIVER="$(basename "$DWC3_DRIVER_PATH")"

OTG_PHY_DEVICE_PATH="$(find_otg_phy "$DWC3_DEVICE" || true)"
if [[ -z "$OTG_PHY_DEVICE_PATH" ]]; then
    # No usb2phy OTG provider behind this controller.  This is expected on SoCs
    # other than the RK3399 (e.g. Allwinner musb/sunxi), which don't have the
    # dwc3 + usb2phy + extcon topology this recovery path manipulates.  Nothing
    # to reset here, so exit cleanly rather than failing the calling service.
    echo "note: no usb2phy OTG provider behind $DWC3_DEVICE (driver $DWC3_DRIVER);" \
         "this recovery path only applies to the RK3399 dwc3+usb2phy stack — skipping" >&2
    exit 0
fi

OTG_PHY_DEVICE="$(basename "$OTG_PHY_DEVICE_PATH")"
OTG_PHY_DRIVER_PATH="$(readlink -f "$OTG_PHY_DEVICE_PATH/driver")"
require_path "$OTG_PHY_DRIVER_PATH" "OTG PHY driver"
OTG_PHY_DRIVER="$(basename "$OTG_PHY_DRIVER_PATH")"

OTG_EXTCON_PATH="$(find_extcon_for_phy "$OTG_PHY_DEVICE_PATH" || true)"
if [[ -z "$OTG_EXTCON_PATH" ]]; then
    OTG_EXTCON_PATH="$OTG_PHY_DEVICE_PATH/extcon/extcon?"
fi

printf 'Detected recovery path:\n'
printf '  dwc3 device: %s\n' "$DWC3_DEVICE"
printf '  dwc3 driver: %s\n' "$DWC3_DRIVER"
printf '  otg phy device: %s\n' "$OTG_PHY_DEVICE"
printf '  otg phy driver: %s\n' "$OTG_PHY_DRIVER"

show_state before "$OTG_EXTCON_PATH"

if [[ -w "/sys/class/udc/$UDC_DEVICE/soft_connect" ]]; then
    write_optional_value disconnect "/sys/class/udc/$UDC_DEVICE/soft_connect" soft_connect
fi

if [[ -w "$DWC3_DEVICE_PATH/power/control" ]]; then
    write_value on "$DWC3_DEVICE_PATH/power/control"
fi

if [[ -w "$OTG_PHY_DEVICE_PATH/power/control" ]]; then
    write_value on "$OTG_PHY_DEVICE_PATH/power/control"
fi

write_driver_action unbind "$DWC3_DEVICE" "$DWC3_DRIVER" "$DWC3_DEVICE_PATH"
write_driver_action unbind "$OTG_PHY_DEVICE" "$OTG_PHY_DRIVER" "$OTG_PHY_DEVICE_PATH"

run_cmd sleep "$SETTLE_SEC"

write_driver_action bind "$OTG_PHY_DEVICE" "$OTG_PHY_DRIVER" "$OTG_PHY_DEVICE_PATH"
write_driver_action bind "$DWC3_DEVICE" "$DWC3_DRIVER" "$DWC3_DEVICE_PATH"

run_cmd sleep "$SETTLE_SEC"

if [[ -w "/sys/class/udc/$UDC_DEVICE/soft_connect" ]]; then
    write_optional_value connect "/sys/class/udc/$UDC_DEVICE/soft_connect" soft_connect
fi

run_cmd sleep "$SETTLE_SEC"

show_state after "$OTG_EXTCON_PATH"

if [[ $DRY_RUN -eq 0 ]]; then
    printf '\nIf the OTG extcon still reports DCP=1 and current_speed remains UNKNOWN, the controller is still in the bad PHY state and likely needs a lower-level kernel/firmware fix or a stronger board-level power reset.\n'
fi