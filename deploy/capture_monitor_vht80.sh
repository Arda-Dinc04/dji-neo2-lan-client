#!/bin/sh
set -eu

# Capture the Neo 2 air interface in monitor mode at the width it actually
# uses. The drone's beacon advertises primary channel 149 with HT40+ and a
# VHT 80 MHz operation centred on channel 155. A 20 MHz monitor sees the
# beacons and the control frames but decodes almost no data frame, which is
# what spoiled the 24 July capture: 28 924 frames for only 84 of data.
#
# The drone link belongs to wlan1, so this script borrows the interface and
# gives it back. wlan0, the default route, the firewall and the global
# NetworkManager state are never touched.

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root" >&2
    exit 1
fi

PROJECT=/home/operator/neo2-control
DEVICE=${DEVICE:-wlan1}
PROFILE=${PROFILE:-neo2-lab}
FREQ=${FREQ:-5745}
WIDTH=${WIDTH:-80}
CENTER=${CENTER:-5775}
DURATION=${DURATION:-60}
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
CAPTURE=${CAPTURE:-$PROJECT/private/captures/neo2-air-$STAMP.pcap}
TCPDUMP_PID=

restore() {
    if [ -n "$TCPDUMP_PID" ]; then
        kill -INT "$TCPDUMP_PID" 2>/dev/null || true
        wait "$TCPDUMP_PID" 2>/dev/null || true
    fi
    # Hand the interface back exactly as it was found.
    ip link set "$DEVICE" down 2>/dev/null || true
    iw dev "$DEVICE" set type managed 2>/dev/null || true
    ip link set "$DEVICE" up 2>/dev/null || true
    nmcli device set "$DEVICE" managed yes 2>/dev/null || true
    nmcli connection up "$PROFILE" ifname "$DEVICE" >/dev/null 2>&1 || true
    chown operator:operator "$CAPTURE" 2>/dev/null || true
}
trap restore EXIT INT TERM

mkdir -p "$(dirname "$CAPTURE")"

nmcli device set "$DEVICE" managed no
ip link set "$DEVICE" down
iw dev "$DEVICE" set type monitor
ip link set "$DEVICE" up
iw dev "$DEVICE" set freq "$FREQ" "$WIDTH" "$CENTER"

echo "monitor: $(iw dev "$DEVICE" info | tr -s ' ' | grep -E 'type|channel')"

tcpdump -i "$DEVICE" -s 0 -w "$CAPTURE" >/dev/null 2>&1 &
TCPDUMP_PID=$!
sleep "$DURATION"
kill -INT "$TCPDUMP_PID" 2>/dev/null || true
wait "$TCPDUMP_PID" 2>/dev/null || true
TCPDUMP_PID=

# Frame census: a usable capture is dominated by data frames, not control.
tshark -r "$CAPTURE" -T fields -e wlan.fc.type -e wlan.fc.subtype 2>/dev/null \
    | sort | uniq -c | sort -rn | head -8 \
    | while read -r count type subtype; do
        case "$type" in
            0) label=management ;;
            1) label=control ;;
            2) label=data ;;
            *) label="type$type" ;;
        esac
        printf '  %8s %s/%s %s\n' "$count" "$type" "$subtype" "$label"
    done

printf '%s\n' "$CAPTURE"
