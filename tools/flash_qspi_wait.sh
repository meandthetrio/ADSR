#!/usr/bin/env bash
set -euo pipefail

APP_BIN="${1:-build/WaveContV3.bin}"
ADDR="0x90040000"

if ! command -v dfu-util >/dev/null 2>&1; then
	echo "ERROR: dfu-util not found in PATH."
	exit 1
fi

if [[ ! -f "$APP_BIN" ]]; then
	echo "ERROR: Cannot find $APP_BIN"
	echo "Build first (make -j4), or pass the path as an argument."
	exit 1
fi

echo "READY TO FLASH QSPI"
echo "1) Do NOT hold BOOT."
echo "2) Tap RESET once NOW (or double-tap if needed)."
echo "Waiting for DFU device to appear..."

until dfu-util -l 2>/dev/null | grep -q "Found DFU"; do
	sleep 0.15
done

echo "DFU device detected. Flashing $APP_BIN to $ADDR ..."
dfu-util -a 0 -s "${ADDR}:leave" -D "$APP_BIN"
echo "Done."
