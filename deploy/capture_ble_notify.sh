#!/bin/sh
set -eu

# Safe Neo 2 experiment: subscribe to GATT notifications, send no DJI payload,
# and retain the controller-side HCI exchange for bit-level inspection.
if [ "$(id -u)" -ne 0 ]; then
    echo "run as root" >&2
    exit 1
fi

PROJECT=/home/operator/neo2-control
DEBUG=/sys/kernel/debug/bluetooth/hci0
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
CAPTURE="$PROJECT/captures/neo2-notify-$STAMP.btsnoop"
RESULT="$PROJECT/captures/neo2-notify-$STAMP.json"
OLD_MIN=$(cat "$DEBUG/conn_min_interval")
OLD_MAX=$(cat "$DEBUG/conn_max_interval")
BTMON_PID=

restore() {
    # Restore max first so the intermediate min/max pair remains valid.
    printf '%s' "$OLD_MAX" > "$DEBUG/conn_max_interval"
    printf '%s' "$OLD_MIN" > "$DEBUG/conn_min_interval"
    if [ -n "$BTMON_PID" ]; then
        kill -INT "$BTMON_PID" 2>/dev/null || true
        wait "$BTMON_PID" 2>/dev/null || true
    fi
    chown operator:operator "$CAPTURE" "$RESULT" 2>/dev/null || true
}
trap restore EXIT INT TERM

mkdir -p "$PROJECT/captures"
btmon --write "$CAPTURE" >/dev/null 2>&1 &
BTMON_PID=$!

# The outer limit protects against a BlueZ disconnect that does not return.
timeout --signal=INT --kill-after=5 45 \
    "$PROJECT/.venv/bin/neo2ctl" ble-notify 02:00:00:00:00:01 \
    --duration 15 --timeout 30 \
    --uuid 0000fff3-0000-1000-8000-00805f9b34fb \
    --uuid 0000fff4-0000-1000-8000-00805f9b34fb \
    --uuid 0000fff5-0000-1000-8000-00805f9b34fb \
    > "$RESULT"

printf '%s\n' "$RESULT"
printf '%s\n' "$CAPTURE"
