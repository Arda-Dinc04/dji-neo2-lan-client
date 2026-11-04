from concurrent.futures import ThreadPoolExecutor
import json
import stat

import pytest

from neo2_control.cli import main
from neo2_control.duml import DumlPacket, DumlSequence, neo2_link_device_info
from neo2_control.identity import (
    BleClientIdentity, WlmIdentity, load_or_create_ble_identity, load_or_create_identity,
)


def test_native_projection_preserves_same_identity_across_both_wire_layouts():
    identity = WlmIdentity("0123456789abcdefABCDEF-more-client-data")
    sequence = DumlSequence(current=0xFFFE)
    link = identity.link_report(sequence.next(), physical_port=12345)
    wifi = identity.wifi_report(sequence.next(), physical_port=9003)
    assert identity.device_sn == "0123456789abcdefABC"
    assert link.receiver == 0x07 and wifi.receiver == 0x1B
    assert len(link.payload) == 24 and len(wifi.payload) == 31
    assert link.payload[:4] == bytes.fromhex("01393000")
    assert wifi.payload[:7] == bytes.fromhex("012b2304000000")
    assert link.payload[4:24] == wifi.payload[7:27]
    assert link.payload[23] == 0
    assert wifi.payload[26:] == bytes(5)
    assert (link.seq, wifi.seq) == (0x8000, 0x8001)
    for packet in [link, wifi]:
        assert (packet.sender, packet.cmd_set, packet.cmd_id, packet.cmd_type) == (2, 7, 0x93, 0x40)
        assert DumlPacket.decode(packet.encode()) == packet


def test_short_identification_is_padded_without_extending_private_source():
    identity = WlmIdentity("pc-lab")
    link = identity.link_report(1, physical_port=1)
    wifi = identity.wifi_report(2, physical_port=2)
    assert link.payload[4:] == b"pc-lab" + bytes(14)
    assert wifi.payload[7:] == b"pc-lab" + bytes(18)


@pytest.mark.parametrize("value", ["", "a\0b", "é", "x" * 256])
def test_identification_rejects_values_outside_the_client_profile(value):
    with pytest.raises(ValueError):
        WlmIdentity(value)


@pytest.mark.parametrize("port", [0, -1, 65536])
def test_link_port_must_be_supplied_from_evidence(port):
    with pytest.raises(ValueError):
        neo2_link_device_info(1, physical_port=port, device_id="pc")


def test_concurrent_creators_share_a_complete_stable_private_identity(tmp_path):
    path = tmp_path / "identity.json"
    with ThreadPoolExecutor(max_workers=8) as workers:
        identities = list(workers.map(lambda _: load_or_create_identity(path), range(16)))
    assert len({value.fingerprint for value in identities}) == 1
    assert load_or_create_identity(path) == identities[0]
    assert len(identities[0].device_sn) == 19
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert not list(tmp_path.glob(".neo2-wlm-*"))


@pytest.mark.parametrize("content", ["", "{}", "not JSON", '{"format":"phone-cache","identification_id":"private"}'])
def test_invalid_existing_identity_is_preserved_instead_of_rotating(tmp_path, content):
    path = tmp_path / "identity.json"
    path.write_text(content)
    with pytest.raises(ValueError):
        load_or_create_identity(path)
    assert path.read_text() == content


def test_cli_reuses_identity_and_prints_only_metadata(tmp_path, capsys):
    path = tmp_path / "identity.json"
    assert main(["identity-init", "--file", str(path)]) == 0
    first = json.loads(capsys.readouterr().out)
    assert main(["identity-init", "--file", str(path)]) == 0
    second_output = capsys.readouterr().out
    assert first == json.loads(second_output)
    identity = load_or_create_identity(path)
    assert identity.identification_id not in second_output
    assert identity.identification_id not in repr(identity)
    assert first["identity_sha256"] == identity.fingerprint


def test_ble_identity_is_stable_private_and_distinct_from_wlm(tmp_path):
    path = tmp_path / 'ble.json'
    with ThreadPoolExecutor(max_workers=8) as workers:
        identities = list(workers.map(lambda _: load_or_create_ble_identity(path), range(16)))
    assert all(identity == identities[0] for identity in identities)
    identity = load_or_create_ble_identity(path)
    assert len(identity.identification_id) == 32
    assert identity.identification_id not in repr(identity)
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    with pytest.raises(ValueError, match='format'):
        load_or_create_identity(path)
    assert load_or_create_ble_identity(path) == identity


@pytest.mark.parametrize('value', ['', 'a' * 31, 'a' * 33, 'a' * 31 + '\0', 'é' * 32])
def test_ble_identity_requires_32_non_nul_ascii_bytes(value):
    with pytest.raises(ValueError):
        BleClientIdentity(value)
