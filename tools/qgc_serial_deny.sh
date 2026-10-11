#!/usr/bin/env bash
set -Eeuo pipefail

APP_ID="${QGC_FLATPAK_ID:-org.mavlink.qgroundcontrol}"

if ! command -v flatpak >/dev/null 2>&1; then
  echo "ERROR: flatpak not found" >&2
  exit 1
fi

echo "Denying ${APP_ID} access to serial/USB devices, keeping GPU/display access..."
flatpak override --user --nodevice=all --device=dri "${APP_ID}"

echo
echo "Current permissions:"
flatpak info --show-permissions "${APP_ID}"

echo
echo "Done. Restart QGroundControl for this to take effect."
echo "Recommended when MAVROS owns /dev/ttyACM0 and QGC connects via UDP."
