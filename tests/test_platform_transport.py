import socket
from unittest.mock import Mock, patch
import pytest
from neo2_control.platform_transport import normalize_ble_identifier, bind_ipv4_socket, validate_interface_binding
from neo2_control.camera_console import CameraConfig
from neo2_control.device_lease import DeviceLease

UUID = '12345678-1234-1234-1234-123456789abc'


def test_corebluetooth_identifier_reaches_config_and_shared_lease(tmp_path):
    assert normalize_ble_identifier(UUID) == UUID.upper()
    assert normalize_ble_identifier('02:aa:00:00:00:01') == '02:AA:00:00:00:01'
    CameraConfig('en0', tmp_path/'wlm', tmp_path/'ble', ble_address=UUID)
    with DeviceLease('en0', UUID, directory=tmp_path/'locks'):
        with pytest.raises(RuntimeError, match='already_owned'):
            with DeviceLease('other0', UUID.upper(), directory=tmp_path/'locks'):pass
    for invalid in ('', '1234', UUID+' ', None, '/tmp/device'):
        with pytest.raises(ValueError):normalize_ble_identifier(invalid)


def test_darwin_ipv4_binding_uses_interface_index_and_fails_on_missing_interface():
    sock = Mock()
    with patch('neo2_control.platform_transport.sys.platform', 'darwin'), \
         patch('neo2_control.platform_transport.socket.if_nametoindex', return_value=7) as index:
        bind_ipv4_socket(sock, 'en0')
    index.assert_called_once_with('en0')
    sock.setsockopt.assert_called_once_with(socket.IPPROTO_IP, 25, 7)
    with patch('neo2_control.platform_transport.sys.platform', 'darwin'), \
         patch('neo2_control.platform_transport.socket.if_nametoindex', side_effect=OSError('missing')):
        with pytest.raises(OSError):bind_ipv4_socket(Mock(), 'missing0')


def test_linux_binding_remains_null_terminated_device_name():
    sock = Mock()
    with patch('neo2_control.platform_transport.sys.platform', 'linux'), \
         patch.object(socket, 'SO_BINDTODEVICE', 25, create=True):
        bind_ipv4_socket(sock, 'wlan0')
    sock.setsockopt.assert_called_once_with(socket.SOL_SOCKET, 25, b'wlan0\0')
    for invalid in ('en0;echo', '', 'a'*33):
        with pytest.raises(ValueError):validate_interface_binding(invalid)
