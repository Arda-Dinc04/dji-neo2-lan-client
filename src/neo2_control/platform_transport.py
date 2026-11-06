"""Host identifier and IPv4 socket portability; no radio setup or device commands."""
import re
import socket
import sys
from uuid import UUID


def normalize_ble_identifier(value):
    if not isinstance(value, str):
        raise ValueError('invalid_ble_identifier')
    if re.fullmatch(r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}', value):
        return value.upper()
    # CoreBluetooth uses a host-local peripheral UUID rather than a MAC.
    if re.fullmatch(r'[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}', value):
        return str(UUID(value)).upper()
    raise ValueError('invalid_ble_identifier')


def validate_interface_binding(interface):
    if not isinstance(interface, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,32}', interface):
        raise ValueError('invalid network interface name')
    if sys.platform == 'darwin':
        return
    if sys.platform.startswith('linux') and hasattr(socket, 'SO_BINDTODEVICE'):
        return
    raise RuntimeError('interface binding requires Linux SO_BINDTODEVICE or macOS IP_BOUND_IF')


def bind_ipv4_socket(sock, interface):
    validate_interface_binding(interface)
    if sys.platform == 'darwin':
        # Darwin netinet/in.h: IP_BOUND_IF=25, an int interface index.
        # CPython does not expose the constant on the tested Mac build.
        index = socket.if_nametoindex(interface)
        sock.setsockopt(socket.IPPROTO_IP, getattr(socket, 'IP_BOUND_IF', 25), index)
    else:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, interface.encode() + b'\0')
