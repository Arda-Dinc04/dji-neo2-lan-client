#!/bin/sh
set -eu

# Decrypt a monitor-mode capture of the Neo 2 air interface.
#
# The pre-shared key is read from the NetworkManager profile that already
# holds it and is passed to tshark through an argument list, never printed and
# never written to a file. The caller only sees frame counts.

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root" >&2
    exit 1
fi

INPUT=${1:?usage: decrypt_air_capture.sh <capture.pcap> [output.pcap]}
OUTPUT=${2:-${INPUT%.pcap}-decrypted.pcap}
PROFILE=${PROFILE:-neo2-lab}

SSID=$(nmcli -t -f 802-11-wireless.ssid connection show "$PROFILE" | cut -d: -f2-)
PSK=$(nmcli --show-secrets -t -f 802-11-wireless-security.psk \
    connection show "$PROFILE" | cut -d: -f2-)
if [ -z "$PSK" ]; then
    echo "no pre-shared key stored in profile $PROFILE" >&2
    exit 1
fi

# A four-way handshake must be present, otherwise no session key can be
# derived and the run would silently produce an empty result.
HANDSHAKE=$(tshark -r "$INPUT" -Y eapol -T fields -e frame.number 2>/dev/null | wc -l)
echo "SSID: $SSID"
echo "EAPOL frames: $HANDSHAKE"
if [ "$HANDSHAKE" -lt 4 ]; then
    echo "warning: fewer than four EAPOL frames, decryption will be partial" >&2
fi

tshark -r "$INPUT" \
    -o wlan.enable_decryption:TRUE \
    -o "uat:80211_keys:\"wpa-pwd\",\"$PSK:$SSID\"" \
    -w "$OUTPUT" -F pcap 2>/dev/null
unset PSK

chown operator:operator "$OUTPUT" 2>/dev/null || true

TOTAL=$(tshark -r "$OUTPUT" -T fields -e frame.number 2>/dev/null | wc -l)
SWUDP=$(tshark -r "$OUTPUT" -Y "udp.port==9003" -T fields -e frame.number 2>/dev/null | wc -l)
echo "frames: $TOTAL"
echo "UDP/9003 datagrams: $SWUDP"
printf '%s\n' "$OUTPUT"
