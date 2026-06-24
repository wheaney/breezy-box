#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN_DIR="$HOME/.local/bin"

cd "$SCRIPT_DIR"

echo "=== Building ==="
make

echo "=== Stopping services ==="
sudo systemctl stop --wait breezy-renderer.service 2>/dev/null || true
systemctl --user stop breezy.target 2>/dev/null || true

echo "=== Copying binaries ==="
install -m 755 displaylink_kms_renderer "$BIN_DIR/"
install -m 755 breezy_web "$BIN_DIR/"
install -m 755 modules/ZeroKVM/src/ZeroKvm.NativeBridge/bin/Release/net10.0/linux-arm64/publish/ZeroKvm.NativeBridge.so "$BIN_DIR/"

echo "=== Running setup ==="
sudo ./setup_system.sh
