#!/usr/bin/env bash

set -euo pipefail

usage() {
	cat <<'EOF'
Usage: install_usb0_imod_overlay.sh [--interval UNITS] [--overlay-name NAME]

Install a boot-persistent DT overlay that enables dwc3 interrupt moderation
(IMOD) on the RK3399 fe800000 controller (the OTG/gadget port behind
rockchip_usb2phy).

Why: profiling the CDC-NCM + USB/IP path shows a single core pegged by the
dwc3 interrupt and its NET_RX/NET_TX softirqs -- roughly 2,500-5,500 dwc3
IRQs/s, all on one CPU, scaling with the USB *transfer* count (not IP packet
size, which is why jumbo MTU did nothing). dwc3 IMOD coalesces event-buffer
interrupts so that rate drops, at the cost of a small, bounded completion
latency. This is the one in-tree lever that attacks the IRQ rate itself.

  --interval UNITS   dwc3 `snps,imod-interval`, in units of 250 ns.
                     e.g. 2000 = 500 us  -> caps dwc3 IRQs at ~2,000/s
                          4000 = 1 ms    -> caps at ~1,000/s
                     Default: 2000. Higher = fewer IRQs but more latency.
  --overlay-name NAME  Override the overlay basename.

CAVEAT: whether IMOD engages in gadget mode depends on the dwc3 IP revision.
Verify on the SBC after reboot (see the printed steps) by comparing the dwc3
IRQ rate in /proc/interrupts before and after. If the rate does not drop, this
revision does not honor IMOD and the overlay can be removed.

Examples:
  sudo ./install_usb0_imod_overlay.sh
  sudo ./install_usb0_imod_overlay.sh --interval 4000
EOF
}

OVERLAY_NAME="rk3399-rock-4c-plus-usb0-imod"
INTERVAL="2000"

while [[ $# -gt 0 ]]; do
	case "$1" in
		--interval)
			if [[ $# -lt 2 ]]; then
				echo "error: --interval requires a value" >&2
				exit 1
			fi
			INTERVAL="$2"
			shift 2
			;;
		--overlay-name)
			if [[ $# -lt 2 ]]; then
				echo "error: --overlay-name requires a value" >&2
				exit 1
			fi
			OVERLAY_NAME="$2"
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

if ! [[ "$INTERVAL" =~ ^[0-9]+$ ]] || [[ "$INTERVAL" -lt 1 ]] || [[ "$INTERVAL" -gt 65535 ]]; then
	echo "error: --interval must be an integer in 1..65535 (units of 250 ns)" >&2
	exit 1
fi

if [[ "$(id -u)" -ne 0 ]]; then
	echo "error: run this script with sudo" >&2
	exit 1
fi

if ! command -v dtc >/dev/null 2>&1; then
	echo "error: dtc is required. Install the device-tree-compiler package first." >&2
	exit 1
fi

add_overlay_to_env() {
	local env_file="$1"
	local overlay_name="$2"
	local existing_line
	local existing_value
	local updated_value

	if [[ ! -f "$env_file" ]]; then
		echo "error: missing $env_file" >&2
		return 1
	fi

	existing_line="$(grep -E '^user_overlays=' "$env_file" || true)"
	if [[ -z "$existing_line" ]]; then
		printf '\nuser_overlays=%s\n' "$overlay_name" >>"$env_file"
		return 0
	fi

	existing_value="${existing_line#user_overlays=}"
	if [[ " $existing_value " == *" $overlay_name "* ]]; then
		return 0
	fi

	updated_value="${existing_value} ${overlay_name}"
	sed -i "s|^user_overlays=.*$|user_overlays=${updated_value}|" "$env_file"
}

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

DTS_PATH="$TMP_DIR/${OVERLAY_NAME}.dts"
DTBO_PATH="$TMP_DIR/${OVERLAY_NAME}.dtbo"
BOOT_DTBO_PATH="/boot/overlay-user/${OVERLAY_NAME}.dtbo"

cat >"$DTS_PATH" <<EOF
/dts-v1/;
/plugin/;

/ {
	compatible = "radxa,rock-4c-plus", "rockchip,rk3399";

	fragment@0 {
		target-path = "/usb@fe800000/usb@fe800000";
		__overlay__ {
			snps,imod-interval = <${INTERVAL}>;
		};
	};
};
EOF

mkdir -p /boot/overlay-user
dtc -@ -I dts -O dtb -o "$DTBO_PATH" "$DTS_PATH"
install -m 0644 "$DTBO_PATH" "$BOOT_DTBO_PATH"
add_overlay_to_env /boot/armbianEnv.txt "$OVERLAY_NAME"

echo "Installed $BOOT_DTBO_PATH with snps,imod-interval=${INTERVAL} (= $((INTERVAL / 4)) us)"
echo "Updated /boot/armbianEnv.txt to include user_overlays=${OVERLAY_NAME}"
echo
echo "This overlay coexists with the dr_mode peripheral overlay (same node,"
echo "different property). Reboot the SBC, then verify it took effect:"
echo "  tr '\\0' '\\n' < /proc/device-tree/usb@fe800000/usb@fe800000/snps,imod-interval | xxd"
echo
echo "Measure the effect (run each while wiggling a window on the DisplayLink display):"
echo "  awk '/dwc3/{print \$2}' /proc/interrupts; sleep 1; awk '/dwc3/{print \$2}' /proc/interrupts"
echo "  # the per-second delta on CPU0 should drop versus the pre-IMOD baseline"
echo
echo "To revert: remove ${OVERLAY_NAME} from user_overlays in /boot/armbianEnv.txt,"
echo "delete $BOOT_DTBO_PATH, and reboot."
