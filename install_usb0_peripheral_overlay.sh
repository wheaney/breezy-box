#!/usr/bin/env bash

set -euo pipefail

usage() {
	cat <<'EOF'
Usage: install_usb0_peripheral_overlay.sh [--overlay-name NAME] [--mode MODE]

Install a boot-persistent DT overlay that forces the RK3399 fe800000 DWC3
controller into gadget-capable mode on the Rock Pi 4C+ OTG path.

The default mode is `peripheral`, which is the known-good setting for exposing
fe800000.usb as a UDC on the current Armbian image.

Examples:
  sudo ./install_usb0_peripheral_overlay.sh
  sudo ./install_usb0_peripheral_overlay.sh --mode peripheral
  sudo ./install_usb0_peripheral_overlay.sh --overlay-name rk3399-rock-4c-plus-usb0-peripheral
EOF
}

OVERLAY_NAME="rk3399-rock-4c-plus-usb0-peripheral"
MODE="peripheral"

while [[ $# -gt 0 ]]; do
	case "$1" in
		--overlay-name)
			if [[ $# -lt 2 ]]; then
				echo "error: --overlay-name requires a value" >&2
				exit 1
			fi
			OVERLAY_NAME="$2"
			shift 2
			;;
		--mode)
			if [[ $# -lt 2 ]]; then
				echo "error: --mode requires a value" >&2
				exit 1
			fi
			MODE="$2"
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

if [[ "$MODE" != "peripheral" && "$MODE" != "otg" ]]; then
	echo "error: --mode must be either 'peripheral' or 'otg'" >&2
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
			dr_mode = "${MODE}";
		};
	};
};
EOF

mkdir -p /boot/overlay-user
dtc -@ -I dts -O dtb -o "$DTBO_PATH" "$DTS_PATH"
install -m 0644 "$DTBO_PATH" "$BOOT_DTBO_PATH"
add_overlay_to_env /boot/armbianEnv.txt "$OVERLAY_NAME"

echo "Installed $BOOT_DTBO_PATH with dr_mode=${MODE}"
echo "Updated /boot/armbianEnv.txt to include user_overlays=${OVERLAY_NAME}"
echo "Reboot the SBC, then verify with:"
echo "  tr '\\0' '\\n' < /proc/device-tree/usb@fe800000/usb@fe800000/dr_mode"
echo "  ls -la /sys/class/udc"