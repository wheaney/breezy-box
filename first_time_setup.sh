#!/usr/bin/env bash
set -euo pipefail

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

./rebuild.sh