#!/usr/bin/env bash
set -Eeuo pipefail

APP_ID="${QGC_FLATPAK_ID:-org.mavlink.qgroundcontrol}"

if ! command -v flatpak >/dev/null 2>&1; then
  echo "ERROR: flatpak not found" >&2
  exit 1
fi

echo "Allowing ${APP_ID} to access serial/USB devices..."
flatpak override --user --device=all "${APP_ID}"

echo
echo "Current permissions:"
flatpak info --show-permissions "${APP_ID}"

echo
echo "Done. Restart QGroundControl for this to take effect."
echo "Note: with serial access enabled, QGC may auto-open /dev/ttyACM0 and conflict with MAVROS."
