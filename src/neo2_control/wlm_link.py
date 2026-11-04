"""Persistent owned-PC WLM presence, link-mode follow and capability read.

Logical modes affect the client's selected routes; their names do not assert
that a physical SDR/LTE backend exists. The fixed Wi-Fi WLM tunnel remains
usable while the UAV requests SDR modes. Video switching is explicitly opt-in.
"""
from dataclasses import dataclass, replace
import math
import secrets

from .duml import DumlPacket
from .duml_stream import DumlStreamDecoder
from .identity import WlmIdentity
from .wlm_discovery import WlmBeacon, unpack_forwarded
from .wlm_modes import ModeFollowRequest, WlmModes, mode_follow_reply
from .wlm_presence import own_presence
from .wlm_query import WlmCapabilityRead, capability_query, _peer_hash
from .wlm_wire import wifi_forward
from .wlm_switch import WiFiServiceSwitch
from .camera import CameraChannel
from .flight_channel import FlightChannel
from .flight_observation import FlightRoute
from .host_control import HostControlState, canonical_neo2_id


@dataclass(frozen=True)
class WlmTransmission:
    packet: DumlPacket
    kind: str

    @property
    def sequence(self):
        return unpack_forwarded(self.packet.payload).packet.seq


class WlmWifiLink:
    """One peer, one pending transmission, current identity and local modes.

    Replies reflect applied local state. Only actual socket sends advance the
    shared forwarding counter. A Wi-Fi-only link mask and deterministic ACK
    padding/sequence are explicit PC policies, awaiting physical validation.
    """

    def __init__(self, identity, next_sequence, *, maximum_age=2, event=lambda *a, **k: None,
                 switch_wifi_service=False, prepare_wifi=None, camera_channel=None,
                 initial_forwarding_sequence=1, flight_channel=None):
        if not isinstance(identity, WlmIdentity) or not callable(next_sequence):
            raise ValueError('WLM link requires owned identity and sequence allocator')
        if not math.isfinite(maximum_age) or not 0 < maximum_age <= 5:
            raise ValueError('WLM peer age must be finite and at most5seconds')
        if type(switch_wifi_service) is not bool or (switch_wifi_service and not callable(prepare_wifi)):
            raise ValueError('Wi-Fi service switch requires an explicit backend preparation callback')
        if camera_channel is not None and (not switch_wifi_service or not isinstance(camera_channel,CameraChannel)):
            raise ValueError('camera channel requires the owned Wi-Fi service switch')
        if flight_channel is not None and (not switch_wifi_service or not isinstance(flight_channel,FlightChannel)):
            raise ValueError('flight channel requires the owned Wi-Fi service switch')
        if type(initial_forwarding_sequence) is not int or not 0<=initial_forwarding_sequence<=0xffffffff:
            raise ValueError('initial WLM forwarding sequence must fit uint32')
        self.camera_channel=camera_channel
        self.flight_channel=flight_channel
        self.identity, self.next_sequence, self.maximum_age = identity, next_sequence, maximum_age
        self.event = event
        self.decoder = DumlStreamDecoder()
        self.modes = WlmModes(4,4,4,0,0)
        self.peer_modes = None
        self.peer_id = self.peer_seen_at = None
        self.peer_beacon = None
        self.read = None
        # Camera and flight recovery may encounter an active remote service.
        # Both measured airborne reconnects required a second read; recovery
        # must not silently lose that bounded policy when preview is absent.
        # These are bounded new reads, each with its own correlation sequence.
        self.query_attempt_limit = 3 if camera_channel is not None or flight_channel is not None else 1
        # A captured camera response took2.064s. Keep its bounded response
        # budget within the same current-peer lifetime; ordinary reads retain2s.
        self.query_timeout = min(5 if camera_channel is not None else 2, maximum_age)
        self.queries_sent = 0
        self.query_retry_at = None
        self.nonce = secrets.randbelow(65535)+1
        self.initial_forwarding_sequence = initial_forwarding_sequence
        self.forwarding_sequence = initial_forwarding_sequence
        self.beacon_counter = self.presences_sent = 0
        self.next_presence_at = self.query_after = None
        self.pending = self.follow_reply = None
        self.next_follow_at = 0
        self.follow_accepted = self.follow_replies_sent = self.follow_coalesced = 0
        self.ignored = self.malformed = 0
        self.state = 'awaiting_current_beacon'
        self.peer_stale = False
        self.reason = self.last_time = self.read_observed_state = None
        self.switch_enabled, self.prepare_wifi = switch_wifi_service, prepare_wifi
        self.service_switch = self.switch_observed_state = None
        self.service_generation = None
        self.follow_deferred = self.video_bytes_received = 0

    def _time(self, now):
        if not math.isfinite(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError('WLM link time must be finite and monotonic')
        self.last_time = now
        if self.state not in ('closed','failed') and self.peer_seen_at is not None and (
                now-self.peer_seen_at >= self.maximum_age):
            if (self.flight_channel is None or self.service_switch is None
                    or not self.service_switch.committed):
                self.close(now,'peer_beacon_expired',failed=True)
            elif not self.peer_stale:
                # Discovery pauses do not destroy an already-owned socket and
                # same-generation bilateral commit. Authority still expires
                # at the original deadline; only neutral/LAND may use that
                # transport until a fresh bound beacon returns.
                self.peer_stale=True
                self.state,self.reason='awaiting_current_beacon','peer_beacon_expired'
                self.flight_channel.set_control_fresh(False,now)
                if self.camera_channel is not None:self.camera_channel.set_ready(False,now)
                self.event('wlm_link_peer_stale',application_ready=False)

    def close(self, now, reason='session_closed', *, failed=False):
        if not math.isfinite(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError('WLM link time must be finite and monotonic')
        self.last_time = now
        if self.state not in ('closed','failed'):
            if self.read is not None:self.read.cancel(now,reason)
            if self.service_switch is not None:self.service_switch.cancel(now,reason)
            if self.camera_channel is not None:self.camera_channel.cancel(now,reason)
            if self.flight_channel is not None:self.flight_channel.cancel(now,reason)
            self.pending = self.follow_reply = None
            self.query_retry_at = None
            self.state, self.reason = ('failed' if failed else 'closed'), reason

    def on_beacon(self, beacon, now):
        self._time(now)
        if self.state in ('closed','failed') or not isinstance(beacon,WlmBeacon):return
        metadata = beacon.metadata()
        if (metadata['device_type'],metadata['device_index'],metadata['device_model']) != (0,0,36):return
        if self.flight_channel is not None and (
                not canonical_neo2_id(beacon.device_id)
                or beacon.forwarding_source_hash != _peer_hash(beacon.device_id)
                or beacon.forwarding_destination_hash not in (self.identity.wlm_device_hash,0xffffffff)):
            # Unbound or unrelated discovery cannot renew a flight peer's
            # lifetime, selected services or boot generation.
            self.ignored += 1
            return
        if self.peer_id is not None and beacon.device_id != self.peer_id:
            self.ignored += 1
            return
        if metadata['format_code'] != 5 or not metadata['ability_negotiation_advertised']:
            if self.peer_id is not None:self.close(now,'peer_profile_changed',failed=True)
            return
        if (self.flight_channel is not None and self.peer_beacon is not None
                and beacon.payload[49:51] != self.peer_beacon.payload[49:51]):
            self.close(now,'peer_session_changed',failed=True)
            return
        try:
            peer_modes = WlmModes(*beacon.payload[24:27], *beacon.payload[52:54])
        except ValueError:
            self.malformed += 1
            return
        if self.read is None:
            self.read = WlmCapabilityRead(self.identity,beacon,sequence=self.next_sequence(),
                forwarding_sequence=self.forwarding_sequence,timeout=self.query_timeout)
            self.peer_id = beacon.device_id
            self.state = 'announcing'
        self.peer_modes, self.peer_seen_at, self.peer_beacon = peer_modes, now, beacon
        if self.peer_stale:
            self.peer_stale=False
            self.reason=None
            self.state='capabilities_received'
            self.flight_channel.set_control_fresh(True,now)
            self.event('wlm_link_peer_recovered',application_ready=False)

    def _read_event(self,now):
        if self.read is None:return
        snapshot = self.read.snapshot(now)
        if snapshot['state'] != self.read_observed_state:
            self.read_observed_state = snapshot['state']
            if self.state not in ('closed','failed') and snapshot['state']=='complete':
                self.state='capabilities_received'
            elif self.state not in ('closed','failed') and snapshot['state'] in ('timed_out','response_error'):
                self.state='capability_read_failed'
                if snapshot['state']=='timed_out' and self.queries_sent < self.query_attempt_limit:
                    self.query_retry_at=now+1
            self.event('wlm_link_capability_state',query_state=snapshot['state'],
                       capabilities=snapshot['capabilities'],application_ready=False)

    def _retry_read(self,now):
        if (self.query_retry_at is None or now < self.query_retry_at or self.read is None
                or self.read.state!='timed_out' or self.queries_sent >= self.query_attempt_limit):return
        # _time already enforces the current identical peer's freshness. Never
        # replay the old query or accept its delayed response as this attempt.
        self.read=WlmCapabilityRead(self.identity,self.peer_beacon,sequence=self.next_sequence(),
            forwarding_sequence=self.forwarding_sequence,timeout=self.query_timeout)
        self.query_retry_at=None
        self.query_after=now
        self.read_observed_state=None
        self.state='announcing'
        self._read_event(now)

    def _commit_service(self,modes):
        if self.state in ('closed','failed') or self.prepare_wifi(modes) is not True:
            return False
        self.modes=modes
        return True

    def _switch_event(self,now):
        if self.service_switch is None:return
        snapshot=self.service_switch.snapshot(now)
        if snapshot['state']!=self.switch_observed_state:
            self.switch_observed_state=snapshot['state']
            self.event('wlm_wifi_service_state',switch_state=snapshot['state'],
                       reason=snapshot['reason'],committed=snapshot['committed'],
                       modes=self.modes.metadata(),application_ready=False)

    def _start_switch(self):
        if (not self.switch_enabled or self.service_switch is not None or self.read is None
                or self.read.state!='complete'
                or self.follow_reply is not None):return
        capability=self.read.capabilities[10]
        required=(1<<2)|(1<<3)|(1<<5)|(1<<6)
        if capability.support & required != required or capability.mask & required:return
        self.service_switch=WiFiServiceSwitch(self.identity,self.peer_id,self.modes,self.next_sequence,
                                             prepare=self.prepare_wifi,commit=self._commit_service)
        metadata=self.peer_beacon.metadata()
        self.service_generation=(metadata['device_id_sha256'],self.read.peer_hash,
            self.identity.wlm_device_hash,metadata['native_word_49'])

    def feed(self,data,now):
        self._time(now)
        if not isinstance(data,bytes) or len(data)>1024*1024:
            raise ValueError('WLM link input exceeds bounded byte profile')
        if self.state in ('closed','failed'):return
        if self.read is not None:
            self.read.feed(data,now)
            self._read_event(now)
        for offset in range(0,len(data),1024):
            for outer,_ in self.decoder.feed(data[offset:offset+1024]):
                if (self.flight_channel is not None
                        and (outer.sender & 0x1f,outer.receiver,outer.cmd_set,outer.cmd_id)==(3,2,3,0x2a)
                        and outer.cmd_type in (0x80,0xc0)):
                    # A measured Neo2 LAND ACK arrived directly in this
                    # validated SWUDP peer/session's protocol stream. It has
                    # no WLM hashes to invent. Require our committed bound
                    # generation; the channel still matches the sent request,
                    # reverse endpoints, sequence, version and ACK deadline.
                    route=self.flight_route(now)
                    if (route is not None and route.command_ready
                            and route.host_control.beacon_route_verified
                            and route.host_control.committed_at is not None):
                        self.flight_channel.receive(outer,now)
                        self.event('wlm_link_direct_flight_response_observed',
                                   sequence=outer.seq,application_ready=False)
                    else:self.ignored+=1
                    continue
                if (outer.version,outer.sender,outer.receiver,outer.cmd_set,outer.cmd_id) != (
                        1,0x27,0xee,0x51,1) or outer.cmd_type not in (0,0x20,0x40):
                    self.ignored += 1
                    continue
                try:forwarded=unpack_forwarded(outer.payload)
                except ValueError:
                    self.malformed += 1
                    continue
                inner=forwarded.packet
                if ((self.camera_channel is not None or self.flight_channel is not None) and self.read is not None
                        and forwarded.link_category==2
                        and int.from_bytes(forwarded.attributes[:4],'little')==self.read.peer_hash
                        and int.from_bytes(forwarded.attributes[9:13],'little') in (self.identity.wlm_device_hash,0xffffffff)):
                    if self.camera_channel is not None:self.camera_channel.receive(inner,now)
                    # The first measured TAKEOFF ACK used broadcast
                    # destinationffffffff with this exact current peer hash.
                    # This admission does not renew control/ground authority.
                    if self.flight_channel is not None:self.flight_channel.receive(inner,now)
                if (self.service_switch is not None and forwarded.link_category==2
                        and inner.cmd_id in (6,8)
                        and int.from_bytes(forwarded.attributes[:4],'little')==self.read.peer_hash
                        and int.from_bytes(forwarded.attributes[9:13],'little')==self.identity.wlm_device_hash):
                    self.service_switch.receive(inner,now)
                    self._switch_event(now)
                    continue
                if (forwarded.link_category,inner.sender,inner.receiver,inner.cmd_set,
                        inner.cmd_id,inner.cmd_type) != (2,0xe9,0xee,0x51,0x1c,0x40):
                    continue
                if self.read is None or self.peer_seen_at is None:
                    self.ignored += 1
                    continue
                try:
                    request=ModeFollowRequest.decode(inner.payload)
                    if (request.device_hash,request.device_type,request.device_index) != (
                            self.read.peer_hash,0,0):
                        self.ignored += 1
                        continue
                    modes=request.requested_state(peer_state=self.peer_modes)
                except ValueError:
                    self.malformed += 1
                    continue
                if self.service_switch is not None and self.service_switch.busy:
                    # The SDK acknowledges follows while a switch is busy,
                    # retaining current modes until bilateral completion.
                    request=replace(request,scene=0)
                    self.follow_deferred+=1
                else:
                    self.modes=modes
                    self.follow_accepted += 1
                if self.follow_reply is not None:self.follow_coalesced += 1
                # Applying the mode affects future presence and the controlled
                # route gate. It neither creates nor pretends to own an SDR.
                self.follow_reply=mode_follow_reply(self.identity,request,self.modes,
                                                    result=0,sequence=inner.seq)
                self.event('wlm_link_modes_followed',modes=self.modes.metadata(),
                           application_ready=False)

    def next_packet(self,now):
        self._time(now)
        self._read_event(now)
        if self.state in ('closed','failed') or self.read is None or self.pending is not None:return None
        self._retry_read(now)
        self._start_switch()
        self._switch_event(now)
        committed=(self.service_switch is not None and self.service_switch.committed
                   and self.modes==self.service_switch.target_modes)
        if self.camera_channel is not None:self.camera_channel.set_ready(committed and not self.peer_stale,now)
        if self.flight_channel is not None:
            self.flight_channel.set_ready(bool(committed and self.peer_modes is not None
                and self.peer_modes.command in (4,5,6,7)
                and int.from_bytes(self.peer_beacon.payload[49:51],'little') != 0
                and self.prepare_wifi(self.modes) is True),now)
        if (self.flight_channel is not None and self.flight_channel.landing_pending(now)
                and (flight_packet:=self.flight_channel.next_packet(now,self.next_sequence)) is not None):
            self.pending=WlmTransmission(wifi_forward(flight_packet,self.identity.wlm_device_hash,
                self.forwarding_sequence,destination_hash=self.read.peer_hash),'wlm_link_flight')
        elif self.follow_reply is not None and now>=self.next_follow_at:
            packet=wifi_forward(self.follow_reply,self.identity.wlm_device_hash,self.forwarding_sequence)
            self.follow_reply=None
            self.pending=WlmTransmission(packet,'wlm_link_follow_reply')
        elif self.query_after is not None and now>=self.query_after and self.read.state=='prepared':
            self.read.request=capability_query(self.identity.wlm_device_hash,
                                               self.read.sequence,self.forwarding_sequence)
            self.pending=WlmTransmission(self.read.request,'wlm_link_capability_query')
        elif self.service_switch is not None and (switch:=self.service_switch.next_packet(now)) is not None:
            packet=wifi_forward(switch.packet,self.identity.wlm_device_hash,self.forwarding_sequence,
                                destination_hash=self.read.peer_hash)
            self.pending=WlmTransmission(packet,'wlm_link_switch_'+switch.kind)
        elif (self.flight_channel is not None
                and (flight_packet:=self.flight_channel.next_packet(now,self.next_sequence)) is not None):
            self.pending=WlmTransmission(wifi_forward(flight_packet,self.identity.wlm_device_hash,
                self.forwarding_sequence,destination_hash=self.read.peer_hash),'wlm_link_flight')
        elif self.camera_channel is not None and (camera_packet:=self.camera_channel.next_packet(now,self.next_sequence)) is not None:
            packet=wifi_forward(camera_packet,self.identity.wlm_device_hash,self.forwarding_sequence,
                                destination_hash=self.read.peer_hash)
            self.pending=WlmTransmission(packet,'wlm_link_camera')
        elif self.next_presence_at is None or now>=self.next_presence_at:
            packet=own_presence(self.identity,peer_format=5,nonce=self.nonce,
                counter=self.beacon_counter,forwarding_sequence=self.forwarding_sequence,modes=self.modes)
            self.pending=WlmTransmission(packet,'wlm_link_presence')
        return self.pending

    def sent(self,kind,now):
        self._time(now)
        if self.pending is None or self.pending.kind != kind or self.state in ('closed','failed'):
            raise RuntimeError('WLM link send does not match a pending transmission')
        if kind=='wlm_link_presence':
            self.presences_sent += 1
            self.beacon_counter=(self.beacon_counter+1)&0xffffffff
            self.next_presence_at=now+.5
            if self.presences_sent==3:self.query_after=now+.5
        else:
            self.forwarding_sequence=(self.forwarding_sequence+1)&0xffffffff
            if kind=='wlm_link_follow_reply':
                self.follow_replies_sent += 1
                self.next_follow_at=now+.2
            elif kind.startswith('wlm_link_switch_'):
                self.service_switch.sent(kind.removeprefix('wlm_link_switch_'),now)
            elif kind=='wlm_link_camera':
                self.camera_channel.sent(self.pending.sequence,now)
            elif kind=='wlm_link_flight':
                self.flight_channel.sent(self.pending.sequence,now)
            else:
                self.read.sent(now)
                self.queries_sent += 1
                self.state='reading_capabilities'
        self.pending=None
        self._read_event(now)
        self._switch_event(now)

    def camera_send_allowed(self,sequence,now):
        self._time(now)
        if (self.state in ('closed','failed') or self.camera_channel is None
                or self.pending is None or self.pending.kind!='wlm_link_camera'
                or self.pending.sequence!=sequence):return False
        ready=(self.service_switch is not None and self.service_switch.committed
               and self.modes==self.service_switch.target_modes
               and self.peer_seen_at is not None and 0<=now-self.peer_seen_at<self.maximum_age
               and self.prepare_wifi(self.modes) is True)
        if not ready:self.camera_channel.set_ready(False,now)
        if not self.camera_channel.can_send(sequence,now):
            self.pending=None
            return False
        return True

    def flight_send_allowed(self,sequence,now):
        self._time(now)
        if (self.state in ('closed','failed') or self.flight_channel is None
                or self.pending is None or self.pending.kind!='wlm_link_flight'
                or self.pending.sequence!=sequence):return False
        ready=(self.service_switch is not None and self.service_switch.committed
               and self.modes==self.service_switch.target_modes
               and self.peer_modes is not None and self.peer_modes.command in (4,5,6,7)
               and self.prepare_wifi(self.modes) is True)
        if not ready:self.flight_channel.set_ready(False,now)
        if not self.flight_channel.can_send(sequence,now):
            self.pending=None
            return False
        return True

    def flight_route(self, now):
        """Selected Neo2 peer evidence, independent of a flight authorization."""
        self._time(now)
        if (self.state in ('closed','failed') or self.peer_beacon is None or self.read is None
                or self.peer_seen_at is None):return None
        metadata=self.peer_beacon.metadata()
        if not metadata['native_word_49']:return None
        backend=bool(callable(self.prepare_wifi) and self.prepare_wifi(self.modes) is True)
        ready=bool(self.service_switch is not None and self.service_switch.committed
            and self.modes==self.service_switch.target_modes
            and self.peer_modes is not None and self.peer_modes.command in (4,5,6,7)
            and backend)
        generation=(metadata['device_id_sha256'],self.read.peer_hash,self.identity.wlm_device_hash,
                    metadata['native_word_49'])
        bound=bool(canonical_neo2_id(self.peer_beacon.device_id)
            and self.peer_beacon.device_id == self.peer_id
            and self.peer_beacon.forwarding_source_hash == self.read.peer_hash
            and self.peer_beacon.forwarding_destination_hash in (self.identity.wlm_device_hash,0xffffffff))
        committed=(self.service_switch.completed_at if self.service_switch is not None
                   and self.service_switch.committed and self.service_generation==generation else None)
        state=HostControlState(generation,now,self.peer_seen_at,committed,self.modes,self.peer_modes,
                               backend,bound)
        return FlightRoute(*generation,self.peer_seen_at,ready,host_control=state)

    def video_received(self,size):
        if type(size) is not int or size<0:raise ValueError('invalid delivered video byte count')
        self.video_bytes_received+=size

    def snapshot(self,now):
        self._time(now)
        active=self.peer_seen_at is not None and self.state not in ('closed','failed')
        return {'enabled':True,'state':self.state,'reason':self.reason,
            'modes':self.modes.metadata(),'modes_raw':list(self.modes.encode()),
            'implemented_backends':{'SDR':False,'LTE':False,'WiFi':True},
            'wifi_tunnel_active':active,
            'controlled_wifi_route_available':active and not self.peer_stale and self.modes.command in (4,5,6,7),
            'peer_beacon_stale':self.peer_stale,
            'presence_datagrams_sent':self.presences_sent,'presence_interval_seconds':.5,
            'forwarding_sequence_initial':self.initial_forwarding_sequence,
            'forwarding_sequence_next':self.forwarding_sequence,
            'follow_requests_applied':self.follow_accepted,'follow_replies_sent':self.follow_replies_sent,
            'follow_requests_coalesced':self.follow_coalesced,'follow_reply_pending':self.follow_reply is not None,
            'transmission_pending':self.pending is not None,'ignored_frames':self.ignored,
            'malformed_frames':self.malformed,'discarded_bytes':self.decoder.discarded_bytes,
            'capability_read':{'enabled':True,'state':'awaiting_current_beacon'} if self.read is None else self.read.snapshot(now),
            'capability_queries_sent':self.queries_sent,
            'capability_query_attempt_limit':self.query_attempt_limit,
            'capability_query_retry_in_seconds':(None if self.query_retry_at is None
                else max(0,self.query_retry_at-now)),
            'follow_requests_deferred':self.follow_deferred,
            'service_switch':({'enabled':self.switch_enabled,'state':'awaiting_capabilities'}
                if self.service_switch is None else self.service_switch.snapshot(now)),
            'video_service_requested':self.service_switch is not None and self.service_switch.trigger_sent>0,
            'wifi_service_configured':(self.service_switch is not None and self.service_switch.committed
                                       and self.modes==self.service_switch.target_modes),
            'video_bytes_received':self.video_bytes_received,
            'camera':{'enabled':False} if self.camera_channel is None else {'enabled':True,**self.camera_channel.snapshot()},
            'flight_channel':{'enabled':False} if self.flight_channel is None else {'enabled':True,**self.flight_channel.snapshot()},
            'negotiation_enabled':False,'application_ready':False}
