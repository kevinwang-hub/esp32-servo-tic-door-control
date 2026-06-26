#!/usr/bin/env bash
# One-command flash for the servo demo.
#   ./flash.sh                 # compile + upload servo_demo
#   ./flash.sh --monitor       # ...then open the serial monitor
#
# Handles the two things that bit us during bring-up:
#   1) frees the serial port if a GUI/monitor is still holding it
#   2) uploads at a reliable 115200 baud with several connect retries
#
# If you still see "No serial data received" / "multiple access on port":
#   - make sure ONLY ONE USB cable is plugged into the board
#     (these dual-USB boards share one CP2102 — two cables fight)
#   - hold the BOOT button while it prints "Connecting...", release after
set -uo pipefail
export PATH="/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin"

cd "$(dirname "$0")"
SKETCH="servo_demo"
FQBN="esp32:esp32:esp32:UploadSpeed=115200"
# Find the first usbserial/usbmodem port without tripping set -e on no-match.
PORT="${PORT:-$(ls /dev/cu.usbserial* /dev/cu.usbmodem* 2>/dev/null | head -1 || true)}"

if [[ -z "${PORT:-}" ]]; then
  echo "No /dev/cu.usbserial* or usbmodem* found. Plug in the board." >&2
  exit 1
fi
echo "Port: $PORT"

# Free the port if something (e.g. the GUI) holds it.
HOLDERS=$(lsof -t "$PORT" 2>/dev/null || true)
if [[ -n "$HOLDERS" ]]; then
  echo "Freeing port (killing: $HOLDERS)"
  kill $HOLDERS 2>/dev/null || true
  sleep 1
fi

echo "Compiling..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH"

ET="$HOME/Library/Arduino15/packages/esp32/tools/esptool_py/5.3.0/esptool"
echo "Uploading (with retries)..."
ok=0
for attempt in 1 2 3 4; do
  echo "--- upload attempt $attempt ---"
  if arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"; then
    ok=1; break
  fi
  echo "retry in 2s... (tip: hold BOOT button now)"
  sleep 2
done

if [[ "$ok" -ne 1 ]]; then
  echo "Upload failed after retries — see BOOT-button note at top of this script." >&2
  exit 2
fi
echo "Upload OK."

if [[ "${1:-}" == "--monitor" ]]; then
  echo "Opening monitor (Ctrl-C to quit)..."
  arduino-cli monitor -p "$PORT" --config baudrate=115200
fi
