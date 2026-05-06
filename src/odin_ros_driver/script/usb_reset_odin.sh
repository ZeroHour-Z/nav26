#!/bin/bash
# usb_reset_odin.sh — Odin USB hub-level power cycle (equivalent to physical unplug/replug)
#
# After a power loss the Odin device (2207:0019) enters a dirty firmware state.
# Simple device-level reset/unbind is NOT enough — the device firmware needs a
# real power cycle. This script unbinds the PARENT HUB, which cuts VBUS power
# to the port, forcing a full device firmware re-initialization.
#
# Requires root (or sudoers entry created by setup_udev_odin.sh).

set -euo pipefail

VENDOR_ID="2207"
PRODUCT_ID="0019"
SETTLE_TIME="${1:-3}"  # seconds to wait between unbind and rebind

UNBIND_PATH="/sys/bus/usb/drivers/usb/unbind"
BIND_PATH="/sys/bus/usb/drivers/usb/bind"

find_odin_devid() {
    for devpath in /sys/bus/usb/devices/*/idVendor; do
        dir="$(dirname "$devpath")"
        if [ -f "$dir/idVendor" ] && [ -f "$dir/idProduct" ]; then
            vid="$(cat "$dir/idVendor")"
            pid="$(cat "$dir/idProduct")"
            if [ "$vid" = "$VENDOR_ID" ] && [ "$pid" = "$PRODUCT_ID" ]; then
                basename "$dir"
                return 0
            fi
        fi
    done
    return 1
}

# Get the parent hub device ID from a child device ID
# e.g., "2-1.1" -> "2-1", "2-1.1.3" -> "2-1.1"
get_parent_hub() {
    local dev_id="$1"
    # Remove the last .N segment to get parent
    local parent="${dev_id%.*}"
    # If no dot was removed, the parent is a root hub port (e.g., "2-1")
    if [ "$parent" = "$dev_id" ]; then
        echo ""
        return 1
    fi
    echo "$parent"
}

echo "[usb_reset_odin] Looking for Odin device ${VENDOR_ID}:${PRODUCT_ID} ..."

DEV_ID="$(find_odin_devid)" || {
    echo "[usb_reset_odin] Odin device not found on USB bus — skipping reset."
    exit 0
}

echo "[usb_reset_odin] Found device: $DEV_ID"

# Determine what to reset: parent hub if available, otherwise the device itself
PARENT_HUB="$(get_parent_hub "$DEV_ID")" || true
if [ -n "$PARENT_HUB" ] && [ -d "/sys/bus/usb/devices/$PARENT_HUB" ]; then
    RESET_TARGET="$PARENT_HUB"
    echo "[usb_reset_odin] Will reset parent hub: $RESET_TARGET (cuts VBUS power to device port)"
else
    RESET_TARGET="$DEV_ID"
    echo "[usb_reset_odin] No parent hub found, resetting device directly: $RESET_TARGET"
fi

# Unbind (cuts power to downstream ports)
echo "[usb_reset_odin] Unbinding $RESET_TARGET ..."
echo "$RESET_TARGET" | sudo tee "$UNBIND_PATH" > /dev/null

echo "[usb_reset_odin] Waiting ${SETTLE_TIME}s for device firmware to fully power down ..."
sleep "$SETTLE_TIME"

# Rebind (restores power, device firmware re-initializes)
echo "[usb_reset_odin] Rebinding $RESET_TARGET ..."
echo "$RESET_TARGET" | sudo tee "$BIND_PATH" > /dev/null

# Wait for device to re-enumerate
echo "[usb_reset_odin] Waiting for device to re-enumerate ..."
sleep 2

# Verify
if find_odin_devid > /dev/null 2>&1; then
    echo "[usb_reset_odin] Odin device successfully power-cycled and re-enumerated."
else
    echo "[usb_reset_odin] WARNING: Device did not re-enumerate after power cycle!"
    exit 1
fi
