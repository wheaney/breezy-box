#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y build-essential m4 bison flex python3-gi libevdev-dev libdrm-dev libxcb-dri2-0 libgbm-dev dkms libjson-c-dev kmod dnsmasq avahi-daemon avahi-utils libglib2.0-bin gsettings-desktop-schemas libgtk-4-dev gnome-themes-extra libadwaita-1-0 gir1.2-adw-1 nftables xvfb x11vnc novnc network-manager iputils-ping curl patchelf openssl gzip util-linux libssl-dev libelf-dev
wget https://packages.microsoft.com/config/debian/13/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
rm packages-microsoft-prod.deb
sudo apt-get update && sudo apt-get install -y dotnet-sdk-10.0

sudo usermod -aG video,render,input $USER

USER_HOME=$(realpath ~)

if [ ! -f $USER_HOME/breezyUI-x86_64.tar.gz ]; then
  echo "Error: $USER_HOME/breezyUI-x86_64.tar.gz not found"
  exit 1
fi

tar -xf $USER_HOME/breezyUI-x86_64.tar.gz

if [ -z "${XDG_DATA_HOME:-}" ]; then
  XDG_DATA_HOME="$USER_HOME/.local/share"
fi

if [ -z "${XDG_BIN_HOME:-}" ]; then
  XDG_BIN_HOME="$USER_HOME/.local/bin"
fi

mkdir -p "$XDG_BIN_HOME"
mkdir -p "$XDG_DATA_HOME"
cp -r breezy_ui/data/* "$XDG_DATA_HOME"
cp -r breezy_ui/bin/* "$XDG_BIN_HOME"

# update copied files to use the local XDG paths
ESCAPED_XDG_DATA_HOME=$(printf '%s\n' "$XDG_DATA_HOME" | sed -e 's/[\/&]/\\&/g')
sed -i -e "s/\/usr\/local\/share/$ESCAPED_XDG_DATA_HOME/g" "$XDG_BIN_HOME/breezydesktop"
sed -i "/Exec/c\Exec=$XDG_BIN_HOME/breezydesktop" "$XDG_DATA_HOME/applications/com.xronlinux.BreezyDesktop.desktop"

glib-compile-schemas "$XDG_DATA_HOME/glib-2.0/schemas"
gtk-update-icon-cache

rm -rf breezy_ui

sudo usermod -aG video,render,input wayne
wget https://github.com/wheaney/XRLinuxDriver/releases/latest/download/xr_driver_setup

chmod +x ./xr_driver_setup
sudo ./xr_driver_setup

systemctl --user enable xr-driver
systemctl --user start xr-driver

$XDG_BIN_HOME/xr_driver_cli -e -bd

printf "request_features=productivity,productivity_pro\n" >> /dev/shm/xr_driver_control 2>/dev/null || true

# GPU bring-up (Imagination PowerVR BXM, Allwinner A733 only). Mali/Panfrost
# boards (e.g. RK3399) don't have an "img,gpu" DT node and skip this entirely.
if grep -qs 'img,gpu' /sys/bus/platform/devices/*/uevent 2>/dev/null; then
  # Kernel module: img-bxm-dkms ships the source, dkms builds it against the
  # kernel headers (assumed already installed on this image).
  PVR_DKMS_VERSION=$(dpkg-query -W -f='${Version}' img-bxm-dkms)
  sudo dkms install "img-bxm-dkms/$PVR_DKMS_VERSION"
  sudo modprobe pvrsrvkm

  # Userspace: the closed DDK (libVK_IMG.so) + firmware aren't shipped by
  # Armbian (only the kernel module is) — they live in Radxa's a733 test repo,
  # version-locked to the kernel module's DDK (24.2@6603887 / BVNC
  # 36.56.104.183) so they bind with no DDK_VERSION_MISMATCH. Without this the
  # renderer falls back to llvmpipe (software). See docs/hardware/radxa-cubie-a7s.md §7.
  PVR_USERSPACE_LIB="/usr/lib/libVK_IMG.so"
  PVR_REPO="https://radxa-repo.github.io/a733-trixie-test"
  PVR_PKG="xserver-xorg-img-bxm-1.21.1-2.deb"   # the package NAME (Allwinnertech quirk)
  if [ ! -e "$PVR_USERSPACE_LIB" ]; then
    pvr_fn="$(curl -fsSL "$PVR_REPO/dists/a733-trixie-test/main/binary-arm64/Packages.gz" \
                | zcat \
                | awk -v p="Package: $PVR_PKG" '$0==p{f=1} f&&/^Filename:/{print $2; exit}')"
    if [ -z "$pvr_fn" ]; then
      echo "Error: could not resolve $PVR_PKG from $PVR_REPO (no internet?)"
      exit 1
    fi
    curl -fsSL -o /tmp/img-bxm.deb "$PVR_REPO/$pvr_fn"
    sudo dpkg -i /tmp/img-bxm.deb
    sudo ldconfig
    rm -f /tmp/img-bxm.deb
    echo "installed PowerVR userspace + firmware ($PVR_PKG) — reboot (or restart the renderer) so pvrsrvkm loads rgx.fw.* on next GPU bring-up"
  fi
fi

sudo ./disable_nonessential_services.sh

./rebuild.sh