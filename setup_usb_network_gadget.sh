#!/usr/bin/env bash

set -euo pipefail

CONFIGFS_ROOT=/sys/kernel/config/usb_gadget
GADGET_NAME=breezy-box-net
FUNCTION_KIND=ncm
FUNCTION_INSTANCE=usb0
CONFIG_NAME=c.1
LANGUAGE=0x409
SERIAL=BREEZYNET001
MANUFACTURER="Breezy Box"
PRODUCT="Breezy Box USB Network"
ID_VENDOR=0x1d6b
ID_PRODUCT=0x0104
BCD_DEVICE=0x0100
BCD_USB=0x0200
MAX_POWER=250
HOST_MAC=02:00:00:00:00:01
DEVICE_MAC=02:00:00:00:00:02
UDC_DEVICE=
TEARDOWN=0

usage() {
	cat <<'EOF'
Usage: setup_usb_network_gadget.sh [options]

Expose a single USB network gadget over the SBC OTG port.

Options:
	--udc-device <name>       Bind to a specific UDC. Defaults to the first entry in /sys/class/udc.
	--gadget-name <name>      Configfs gadget directory name. Default: breezy-box-net
	--function <ecm|ncm|rndis>
	                         USB networking function type. Default: ncm
	--function-instance <id>  Function instance suffix. Default: usb0
	--serial <value>          USB serial string. Default: BREEZYNET001
	--manufacturer <value>    USB manufacturer string. Default: Breezy Box
	--product <value>         USB product string. Default: Breezy Box USB Network
	--id-vendor <hex>         USB vendor ID. Default: 0x1d6b
	--id-product <hex>        USB product ID. Default: 0x0104
	--host-mac <mac>          Host-visible MAC address. Default: 02:00:00:00:00:01
	--device-mac <mac>        SBC gadget MAC address. Default: 02:00:00:00:00:02
	--teardown                Unbind and remove the gadget instead of creating it
	-h, --help                Show this help message
EOF
}

require_root() {
	if [[ ${EUID} -ne 0 ]]; then
		echo "This script must run as root." >&2
		exit 1
	fi
}

ensure_configfs() {
	modprobe libcomposite >/dev/null 2>&1 || true
	if ! mountpoint -q /sys/kernel/config; then
		mount -t configfs none /sys/kernel/config
	fi
	if [[ ! -d ${CONFIGFS_ROOT} ]]; then
		echo "Configfs USB gadget root is unavailable at ${CONFIGFS_ROOT}." >&2
		exit 1
	fi
}

find_default_udc() {
	local first_udc

	first_udc=$(find /sys/class/udc -mindepth 1 -maxdepth 1 -printf '%f\n' | sort | head -n1 || true)
	if [[ -z ${first_udc} ]]; then
		echo "No gadget-capable UDCs are exposed under /sys/class/udc." >&2
		exit 1
	fi
	printf '%s\n' "${first_udc}"
}

gadget_dir() {
	printf '%s/%s\n' "${CONFIGFS_ROOT}" "${GADGET_NAME}"
}

function_name() {
	printf '%s.%s\n' "${FUNCTION_KIND}" "${FUNCTION_INSTANCE}"
}

cleanup_gadget() {
	local dir function_path config_path

	dir=$(gadget_dir)
	function_path="${dir}/functions/$(function_name)"
	config_path="${dir}/configs/${CONFIG_NAME}"

	if [[ ! -d ${dir} ]]; then
		return
	fi

	if [[ -f ${dir}/UDC ]]; then
		printf '' >"${dir}/UDC" || true
	fi

	if [[ -L ${config_path}/$(function_name) ]]; then
		rm -f "${config_path}/$(function_name)"
	fi

	if [[ -d ${function_path} ]]; then
		rmdir "${function_path}" || true
	fi

	if [[ -d ${config_path}/strings/${LANGUAGE} ]]; then
		rmdir "${config_path}/strings/${LANGUAGE}" || true
	fi

	if [[ -d ${config_path} ]]; then
		rmdir "${config_path}" || true
	fi

	if [[ -d ${dir}/strings/${LANGUAGE} ]]; then
		rmdir "${dir}/strings/${LANGUAGE}" || true
	fi

	rmdir "${dir}" || true
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--udc-device)
			UDC_DEVICE=${2:?missing value for --udc-device}
			shift 2
			;;
		--gadget-name)
			GADGET_NAME=${2:?missing value for --gadget-name}
			shift 2
			;;
		--function)
			FUNCTION_KIND=${2:?missing value for --function}
			shift 2
			;;
		--function-instance)
			FUNCTION_INSTANCE=${2:?missing value for --function-instance}
			shift 2
			;;
		--serial)
			SERIAL=${2:?missing value for --serial}
			shift 2
			;;
		--manufacturer)
			MANUFACTURER=${2:?missing value for --manufacturer}
			shift 2
			;;
		--product)
			PRODUCT=${2:?missing value for --product}
			shift 2
			;;
		--id-vendor)
			ID_VENDOR=${2:?missing value for --id-vendor}
			shift 2
			;;
		--id-product)
			ID_PRODUCT=${2:?missing value for --id-product}
			shift 2
			;;
		--host-mac)
			HOST_MAC=${2:?missing value for --host-mac}
			shift 2
			;;
		--device-mac)
			DEVICE_MAC=${2:?missing value for --device-mac}
			shift 2
			;;
		--teardown)
			TEARDOWN=1
			shift
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			echo "Unknown option: $1" >&2
			usage >&2
			exit 1
			;;
	esac
done

if [[ ${FUNCTION_KIND} != "ecm" && ${FUNCTION_KIND} != "ncm" && ${FUNCTION_KIND} != "rndis" ]]; then
	echo "--function must be one of: ecm, ncm, rndis" >&2
	exit 1
fi

require_root
ensure_configfs

if [[ ${TEARDOWN} -eq 1 ]]; then
	cleanup_gadget
	echo "Removed gadget ${GADGET_NAME}."
	exit 0
fi

if [[ -z ${UDC_DEVICE} ]]; then
	UDC_DEVICE=$(find_default_udc)
fi

DIR=$(gadget_dir)
FUNCTION_NAME=$(function_name)
CONFIG_PATH=${DIR}/configs/${CONFIG_NAME}
FUNCTION_PATH=${DIR}/functions/${FUNCTION_NAME}

mkdir -p "${DIR}"
printf '%s' "${ID_VENDOR}" >"${DIR}/idVendor"
printf '%s' "${ID_PRODUCT}" >"${DIR}/idProduct"
printf '%s' "${BCD_DEVICE}" >"${DIR}/bcdDevice"
printf '%s' "${BCD_USB}" >"${DIR}/bcdUSB"

mkdir -p "${DIR}/strings/${LANGUAGE}"
printf '%s' "${SERIAL}" >"${DIR}/strings/${LANGUAGE}/serialnumber"
printf '%s' "${MANUFACTURER}" >"${DIR}/strings/${LANGUAGE}/manufacturer"
printf '%s' "${PRODUCT}" >"${DIR}/strings/${LANGUAGE}/product"

mkdir -p "${CONFIG_PATH}/strings/${LANGUAGE}"
printf '%s' "Breezy Box network display uplink" >"${CONFIG_PATH}/strings/${LANGUAGE}/configuration"
printf '%s' "${MAX_POWER}" >"${CONFIG_PATH}/MaxPower"

mkdir -p "${FUNCTION_PATH}"
printf '%s' "${HOST_MAC}" >"${FUNCTION_PATH}/host_addr"
printf '%s' "${DEVICE_MAC}" >"${FUNCTION_PATH}/dev_addr"

if [[ ! -L ${CONFIG_PATH}/${FUNCTION_NAME} ]]; then
	ln -s "${FUNCTION_PATH}" "${CONFIG_PATH}/${FUNCTION_NAME}"
fi

printf '' >"${DIR}/UDC"
printf '%s' "${UDC_DEVICE}" >"${DIR}/UDC"

if [[ -f ${FUNCTION_PATH}/ifname ]]; then
	INTERFACE_NAME=$(<"${FUNCTION_PATH}/ifname")
	if [[ -n ${INTERFACE_NAME} ]]; then
		echo "USB network gadget bound on ${UDC_DEVICE}; interface is ${INTERFACE_NAME}."
	else
		echo "USB network gadget bound on ${UDC_DEVICE}."
	fi
else
	echo "USB network gadget bound on ${UDC_DEVICE}."
fi

echo "Next step: assign per-display aliases with network_display_receiver_supervisor.py."