#!/bin/bash
# One command to fly. Starts what is not running yet, then the pilot in this terminal.
#   ./fly.sh                 bridge + camera (kept running in the background between flights) + pilot here
#   ./fly.sh hover           same, but the height-only test instead of the pilot
#   ./fly.sh stop            stop the background bridge and camera (the bridge sends STOP and disconnects cleanly)
#   ./fly.sh status          what is running
# Extra words go to the pilot / hover:   ./fly.sh --log demo1     ./fly.sh hover --pid
# Background output: data/logs/bridge.log, data/logs/mono.log
cd "$(dirname "$0")" || exit 1
PY=.venv/bin/python; [ -x "$PY" ] || PY=python3
BRIDGE_ARGS="--motor-cap 1 --lift-first --total-cap 1.6"
MONO_ARGS="--auto-calib --show"
mkdir -p data/logs

running() { pgrep -f "laptop\.$1" >/dev/null; }
bridge_up() { curl -s -m 1 http://127.0.0.1:5008/status | grep -q '"connected": true'; }

case "$1" in
  stop)
    pkill -INT -f "laptop\.control\.ble_gondola"; pkill -INT -f "laptop\.vision\.mono"
    sleep 3; echo "[fly] stopped"; exit 0 ;;
  status)
    running control.ble_gondola && echo "[fly] bridge running ($(bridge_up && echo connected || echo NOT connected yet))" || echo "[fly] bridge not running"
    running vision.mono && echo "[fly] camera running" || echo "[fly] camera not running"
    exit 0 ;;
esac

if ! running control.ble_gondola; then
  echo "[fly] starting the Bluetooth bridge ($BRIDGE_ARGS)"
  nohup $PY -u -m laptop.control.ble_gondola $BRIDGE_ARGS > data/logs/bridge.log 2>&1 < /dev/null &
fi
if ! running vision.mono; then
  echo "[fly] starting the camera ($MONO_ARGS)"
  nohup $PY -u -m laptop.vision.mono $MONO_ARGS > data/logs/mono.log 2>&1 < /dev/null &
fi
printf "[fly] waiting for the box"
for i in $(seq 1 60); do bridge_up && break; printf "."; sleep 1; done; echo
bridge_up || { echo "[fly] the bridge has not connected after 60 s: is the box powered? another laptop connected to it?  tail data/logs/bridge.log"; exit 1; }
echo "[fly] box connected. Hold the balloon at the height you want: it arms by itself 8 s after the camera sees it."

if [ "$1" = "hover" ]; then shift; exec $PY -m laptop.control.hover --auto-arm 8 --log fly "$@"; fi
exec $PY -m laptop.control.pilot --omni-cam room --auto-arm 8 --log fly "$@"
