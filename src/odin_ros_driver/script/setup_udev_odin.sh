#!/bin/bash
# setup_udev_odin.sh — One-time setup: allow current user to power-cycle Odin USB
#                       device via sysfs without a password prompt.
#
# Run once with: sudo bash setup_udev_odin.sh
# What it does:
#   1. Installs usb_reset_odin.sh to /usr/local/bin/odin-usb-power-cycle
#   2. Creates a sudoers drop-in so the current (SUDO_USER) can run it without password

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_SCRIPT="$SCRIPT_DIR/usb_reset_odin.sh"
DEST="/usr/local/bin/odin-usb-power-cycle"
SUDOERS_FILE="/etc/sudoers.d/odin-usb-reset"

TARGET_USER="${SUDO_USER:-$(whoami)}"

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Run this script with sudo:  sudo bash $0"
    exit 1
fi

echo "[setup] Installing $DEST ..."
cp "$SRC_SCRIPT" "$DEST"
chmod 755 "$DEST"
chown root:root "$DEST"

echo "[setup] Creating sudoers entry for user '$TARGET_USER' ..."
cat > "$SUDOERS_FILE" <<EOF
# Allow $TARGET_USER to unbind/rebind Odin USB device without password
$TARGET_USER ALL=(root) NOPASSWD: /usr/bin/tee /sys/bus/usb/drivers/usb/unbind
$TARGET_USER ALL=(root) NOPASSWD: /usr/bin/tee /sys/bus/usb/drivers/usb/bind
EOF
chmod 440 "$SUDOERS_FILE"

# Validate sudoers syntax
if visudo -cf "$SUDOERS_FILE"; then
    echo "[setup] Done. '$TARGET_USER' can now run usb_reset_odin.sh without a password."
else
    echo "[setup] ERROR: sudoers syntax check failed — removing broken file."
    rm -f "$SUDOERS_FILE"
    exit 1
fi
