#!/usr/bin/env bash
#
# board_quirks.sh - board/controller-specific helpers for loading the
# g_rndis_displaylink gadget, sourced by load_module.sh.
#
# The kernel module itself is controller-agnostic (pure USB gadget API).  The
# quirks that differ between boards live HERE, selected by detecting the UDC
# driver, so supporting a new board is just adding a case.
#
# Detected controllers so far:
#   dwc3            - Radxa ROCK 4C+ (RK3399).  Needs the rockchip-usb2phy
#                     DCP-charger-detect workaround and is sensitive to a
#                     udev auto-recovery rule that steals the UDC.
#   sunxi_usb_udc   - Radxa Cubie A7S (Allwinner A733 / sun60iw2).  MUSB-lineage
#                     controller; no DCP/PHY dance needed.
#
# Each board exposes:
#   board_detect            -> echoes a short board id (dwc3|sunxi|unknown)
#   board_pre_load          -> board-specific teardown before insmod (PHY reset,
#                              disabling racing udev rules, etc.); no-op if N/A
#   board_post_unload       -> undo board_pre_load (restore udev rules, etc.)

# --- detection ----------------------------------------------------------------

board_udc() { ls /sys/class/udc 2>/dev/null | head -1; }

board_udc_driver() {
    local udc; udc="$(board_udc)"
    [[ -n "$udc" ]] || return 1
    basename "$(readlink -f "/sys/class/udc/$udc/device/driver" 2>/dev/null)" 2>/dev/null
}

board_detect() {
    case "$(board_udc_driver)" in
        dwc3)          echo dwc3 ;;
        sunxi_usb_udc) echo sunxi ;;
        *)             echo unknown ;;
    esac
}

# --- dwc3 / rockchip quirks ---------------------------------------------------
# A prior direct (hub-less) connection can latch the usb2phy into DCP charger
# mode, forcing full-speed + EPROTO so the host can't read descriptors.  A PHY
# unbind/rebind re-runs charger detection.  EACH rebind also degrades the dwc3
# ep0 state, so only do it when DCP is actually latched.  Separately, a udev
# rule (99-breezy-otg-reset) fires on DCP=1 and rebinds the configfs RNDIS
# gadget on top of us, so move it aside for the duration.

DWC3_OTG_PHY="ff770000.syscon:usb2phy@e450"
DWC3_PHY_DRIVER="/sys/bus/platform/drivers/rockchip-usb2phy"
DWC3_UDEV_RULE="/etc/udev/rules.d/99-breezy-otg-reset.rules"

_dwc3_dcp_latched() {
    [[ "$(cat /sys/class/extcon/extcon1/cable.4/state 2>/dev/null || echo 0)" == "1" ]]
}

board_pre_load_dwc3() {
    if [[ -f "$DWC3_UDEV_RULE" ]]; then
        mv -f "$DWC3_UDEV_RULE" "$DWC3_UDEV_RULE.disabled-by-load_module"
        udevadm control --reload-rules 2>/dev/null || true
        echo "  [dwc3] disabled udev OTG-reset rule"
    fi
    systemctl mask --now breezy-gadget-reset.service 2>/dev/null || true

    if [[ "${FORCE_PHY_RESET:-0}" == "1" ]] || _dwc3_dcp_latched; then
        if [[ -e "$DWC3_PHY_DRIVER/$DWC3_OTG_PHY" ]]; then
            echo "  [dwc3] DCP latched (or forced) - resetting OTG PHY"
            echo "$DWC3_OTG_PHY" > "$DWC3_PHY_DRIVER/unbind" 2>/dev/null && sleep 2
            echo "$DWC3_OTG_PHY" > "$DWC3_PHY_DRIVER/bind"   2>/dev/null && sleep 2
            udevadm settle --timeout=5 2>/dev/null || true
            echo "" > /sys/kernel/config/usb_gadget/breezy-composite/UDC 2>/dev/null || true
        fi
    else
        echo "  [dwc3] DCP clean - skipping PHY rebind (avoids degrading ep0)"
    fi
}

board_post_unload_dwc3() {
    if [[ -f "$DWC3_UDEV_RULE.disabled-by-load_module" ]]; then
        mv -f "$DWC3_UDEV_RULE.disabled-by-load_module" "$DWC3_UDEV_RULE"
        udevadm control --reload-rules 2>/dev/null || true
        echo "  [dwc3] restored udev OTG-reset rule"
    fi
    systemctl unmask breezy-gadget-reset.service 2>/dev/null || true
}

# --- sunxi / allwinner quirks -------------------------------------------------
# The Allwinner sunxi_usb_udc (MUSB lineage) has no rockchip-style DCP charger
# detection, so there is nothing special to do.  Kept as explicit no-ops so the
# board's behaviour is documented rather than implied.

board_pre_load_sunxi()    { echo "  [sunxi] no controller-specific pre-load steps"; }
board_post_unload_sunxi() { :; }

# --- dispatch -----------------------------------------------------------------

board_pre_load() {
    case "$(board_detect)" in
        dwc3)  board_pre_load_dwc3 ;;
        sunxi) board_pre_load_sunxi ;;
        *)     echo "  [board] unknown controller '$(board_udc_driver)' - no quirks applied" ;;
    esac
}

board_post_unload() {
    case "$(board_detect)" in
        dwc3)  board_post_unload_dwc3 ;;
        sunxi) board_post_unload_sunxi ;;
    esac
}
