#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

USB_INTERFACE=${USB_INTERFACE:-usb0}
USB_ADDRESS=${USB_ADDRESS:-192.168.2.2/24}
BIND_HOST_DEFAULT=${USB_ADDRESS%/*}
BIND_HOST=${BIND_HOST:-$BIND_HOST_DEFAULT}
RENDERER_DEVICE=${RENDERER_DEVICE:-/dev/dri/card0}
RENDERER_DECODER_FRAGMENT=${RENDERER_DECODER_FRAGMENT:-decodebin\ \!\ videoconvert}
PYTHON_BIN=${PYTHON_BIN:-python3}

usage() {
	cat <<EOF
Usage: start_wfd_usb_sink.sh [wfd_mice_sink.py options...]

Bring up the USB network gadget on the SBC, assign ${USB_ADDRESS} to ${USB_INTERFACE},
and launch wfd_mice_sink.py with the current recommended GNOME/Windows defaults.

Environment overrides:
  USB_INTERFACE                Default: ${USB_INTERFACE}
  USB_ADDRESS                  Default: ${USB_ADDRESS}
  BIND_HOST                    Default: ${BIND_HOST}
  RENDERER_DEVICE              Default: ${RENDERER_DEVICE}
  RENDERER_DECODER_FRAGMENT    Default: ${RENDERER_DECODER_FRAGMENT}
  PYTHON_BIN                   Default: ${PYTHON_BIN}

Examples:
  sudo ./start_wfd_usb_sink.sh --verbose
  sudo USB_ADDRESS=192.168.2.20/24 ./start_wfd_usb_sink.sh --service-name 'Breezy Box'
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
	usage
	exit 0
fi

if [[ ${EUID} -ne 0 ]]; then
	echo "This script must run as root so it can configure the USB gadget and network interface." >&2
	exit 1
fi

cd "$SCRIPT_DIR"

./setup_usb_network_gadget.sh
ip link set "$USB_INTERFACE" up
ip addr replace "$USB_ADDRESS" dev "$USB_INTERFACE"

exec "$PYTHON_BIN" ./wfd_mice_sink.py \
	--interface "$USB_INTERFACE" \
	--bind-host "$BIND_HOST" \
	--launch-renderer \
	--renderer-device "$RENDERER_DEVICE" \
	--renderer-decoder-fragment "$RENDERER_DECODER_FRAGMENT" \
	"$@"
