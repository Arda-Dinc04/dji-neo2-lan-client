"""Private subprocess fixture: fake BLE/NM, real loopback-only UDP owner.

This entry point is not packaged or available through the production CLI.
"""
import asyncio
from functools import partial
from pathlib import Path
import socket
import sys
from types import SimpleNamespace

from neo2_control.ble_bootstrap import WifiCredentials
from neo2_control.control_owner import _receive,run_owner
from neo2_control.device_lease import DeviceLease
from neo2_control.flight_channel import FlightChannel
from neo2_control.flight_connection import FlightConfig,OwnedFlightBackend
from neo2_control.session import run_session


def result(**values):return SimpleNamespace(**values,as_dict=lambda:dict(values))


async def ble(**kwargs):
    try:
        WifiCredentials(bytes.fromhex('020000000002'),b'fixture',b'loopback-only-password').save_new(kwargs['credentials_path'])
        await kwargs['while_connected']()
        return result(state='complete',reason='credentials_saved',connection_cleanup_verified=True)
    finally:kwargs['event_sink']('ble_cleanup_result',cleanup_verified=True)


class Wifi:
    def __init__(self,*args,**kwargs):self.clean=False
    async def __aenter__(self):return self
    async def __aexit__(self,*args):self.clean=True
    def snapshot(self):return {'cleanup_verified':self.clean,'scope':'synthetic_network_manager'}


def channel(**kwargs):
    value=FlightChannel(**kwargs);value.require_loopback();return value


def session(**kwargs):
    kwargs.pop('interface')
    assert kwargs['flight_channel'].snapshot()['loopback_only']
    return run_session(host='127.0.0.1',port=int(sys.argv[4]),**kwargs)


if __name__=='__main__':
    fd=int(sys.argv[1]);bootstrap=socket.socket(fileno=fd);bootstrap.settimeout(3)
    message=_receive(bootstrap)
    assert set(message)=={'kind','mode','connection'} and message['kind']=='configure' and message['mode']=='flight'
    config=FlightConfig(**message['connection']);bootstrap.detach()
    def backend(config):
        return OwnedFlightBackend(config,channel_factory=channel,session_runner=session,ble_runner=ble,
            wifi_factory=Wifi,route_guard=lambda *args,**kw:None,
            lease_factory=partial(DeviceLease,directory=Path(config.capture_directory)/'leases'))
    raise SystemExit(run_owner(fd,sys.argv[2],flight_config=config,backend_factory=backend))
