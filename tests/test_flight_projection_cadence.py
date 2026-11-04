"""Video tick batching must not delay new protocol evidence or final guards."""
from neo2_control.flight_channel import FlightChannel
from neo2_control.session import SwudpSession, OutboundDatagram
from test_wlm_link import IDENTITY
from test_session import acceptance


def test_video_ticks_share_projection_but_inputs_status_and_send_always_refresh(monkeypatch):
    now=[0.]
    channel=FlightChannel(clock=lambda:now[0]);channel.enable_observations(synthetic=True)
    client=SwudpSession(flight_channel=channel,wlm_identity=IDENTITY,
        wlm_wifi_liveview=True,wifi_backend_ready=lambda:True)
    client.start(0.)
    client.receive(acceptance(session=client.transport.session,flags=3),0.)
    calls=[];original=channel.update_observations
    def project(*args,**kwargs):
        calls.append(now[0]);return original(*args,**kwargs)
    monkeypatch.setattr(channel,'update_observations',project)
    for i in range(25):
        now[0]=i/1000;client.tick(now[0])
    assert calls==[0.]
    now[0]=.025;client.tick(now[0]);assert calls==[0.,.025]
    now[0]=.026;client._deliver(3,b'',now[0]);assert calls[-1]==.026
    now[0]=.027;client.snapshot(now[0]);assert calls[-1]==.027
    now[0]=.028
    assert not client.datagram_send_allowed(OutboundDatagram(b'', 'fixture', 'wlm_link_flight',1),now[0])
    assert calls[-1]==.028
    now[0]=.029;client.state='closed';client.tick(now[0]);assert calls[-1]==.029


def test_backend_connection_predicate_retains_route_revocation_without_diagnostic_copy(monkeypatch):
    from neo2_control.control_transport import FlightChannelBackend
    channel=FlightChannel(clock=lambda:100.)
    backend=FlightChannelBackend(channel,synthetic=True)
    channel.bind_transport('127.0.0.1',9003);channel.set_ready(True,100.)
    backend.open(100.)
    def forbidden():raise AssertionError('diagnostic copy in control path')
    monkeypatch.setattr(channel,'snapshot',forbidden)
    assert backend.connected
    channel.set_ready(False,100.);assert not backend.connected
    channel.set_ready(True,100.);assert backend.connected
    channel.revoke();assert not backend.connected
