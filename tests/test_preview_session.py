import asyncio
import json
from pathlib import Path
from unittest.mock import Mock
import pytest
from neo2_control.session import SwudpSession
from neo2_control.identity import WlmIdentity, BleClientIdentity
from neo2_control.duml import DumlPacket, neo2_heartbeat
from neo2_control.camera_console import CameraConfig, CameraConsole
from neo2_control.ble_bootstrap import _Exchange
from neo2_control.preview_policy import validate_preview_ble


def test_udp_guard_rejects_control_before_transport_allocation():
    identity=WlmIdentity('synthetic-owned-client')
    client=SwudpSession(wlm_identity=identity,preview_only=True)
    client.sender.send=Mock(return_value=b'synthetic-transport')
    heartbeat=neo2_heartbeat(sequence=1).encode()
    assert client._application_datagram(heartbeat,'heartbeat',1,0)==b'synthetic-transport'
    for command in ((3,0xdf),(4,0x14),(2,1),(2,2),(0,7)):
        packet=DumlPacket(2,3,2,0,*command,b'')
        with pytest.raises(ValueError,match='preview_outbound_not_allowed'):
            client._application_datagram(packet.encode(),'heartbeat',2,0)
    client.sender.send.assert_called_once_with(heartbeat)


def test_ble_guard_rejects_before_capture_and_gatt_write():
    identity=BleClientIdentity('synthetic-client'.ljust(32,'x'))
    recorder=Mock();writer=Mock();client=Mock()
    exchange=_Exchange(recorder,Mock(),3,packet_guard=lambda p:validate_preview_ble(p,identity,None))
    with pytest.raises(ValueError,match='preview_outbound_not_allowed'):
        asyncio.run(exchange._write(client,writer,DumlPacket(2,3,1,0,3,0xdf,b'')))
    client.write_gatt_char.assert_not_called()
    recorder.record.assert_not_called()


def test_preview_console_has_no_camera_actions_even_before_connection(tmp_path):
    config=CameraConfig('en0',tmp_path/'wlm',tmp_path/'ble',preview_only=True)
    console=CameraConsole(config)
    for action in ('pitch','photo','record','recenter'):
        with pytest.raises(RuntimeError,match='preview_only_camera_actions_disabled'):console.command(action,{})
    assert not any(console.status()['capabilities'].values())
    assert console.status()['preview_only']


def test_network_identity_failure_blocks_initial_socket_send(tmp_path):
    from neo2_control.session import run_session
    result=run_session(capture_path=tmp_path/'capture.jsonl',host='127.0.0.1',port=54321,
                       duration=.1,network_guard=lambda:False)
    assert result.state=='failed'
    assert result.reason=='camera_network_identity_changed_or_stale'
    assert not any(json.loads(line).get('direction')=='tx' for line in (tmp_path/'capture.jsonl').read_text().splitlines())
