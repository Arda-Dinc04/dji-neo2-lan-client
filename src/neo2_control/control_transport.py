"""Supervisor adapter for the actual socket owner's typed flight channel.

Request and response bytes remain those allocated and observed by the owner.
Synthetic use latches a loopback constraint before any transport is opened.
Aircraft observations are supplied separately; this adapter invents none.
"""
from .controller import FlightObservation
from .flight import FlightAction
from .flight_channel import FlightChannel


class FlightChannelBackend:
    def __init__(self, channel, observation_provider=None, *, synthetic=False):
        if not isinstance(channel, FlightChannel) or (observation_provider is not None and not callable(observation_provider)):
            raise ValueError('flight backend requires a typed channel and observation provider')
        if type(synthetic) is not bool:
            raise ValueError('flight backend provenance must be boolean')
        if synthetic:
            channel.require_loopback()
        if observation_provider is None:
            channel.enable_observations(synthetic=synthetic)
        self.channel = channel
        self.observation_provider = channel.observation if observation_provider is None else observation_provider
        self._synthetic = synthetic
        self.opened = False

    @property
    def synthetic(self):
        return self._synthetic

    @property
    def connected(self):
        return bool(self.opened and self.channel.transport_connected())

    def open(self, now):
        if self.opened or self.channel.snapshot()['revoked']:
            raise RuntimeError('flight_backend_generation_unavailable')
        self.opened = True

    def close(self, now):
        self.opened = False
        self.channel.revoke()

    def advance(self, now):
        # The UDP owner advances the channel. Receipts carry real timestamps;
        # the supervisor does not manufacture or relabel a response packet.
        return []

    def observation(self):
        sample = self.observation_provider()
        if sample is not None and not isinstance(sample, FlightObservation):
            raise ValueError('flight backend observation must be typed or unknown')
        return sample

    def observation_context(self):
        """One atomic activity/aircraft snapshot for durable uncertainty."""
        with self.channel.lock:
            sample = self.observation()
            context = None
            if sample is not None:
                try:context = self.aircraft_context(self.channel.clock())
                except ValueError:pass
            return sample,context

    def submit_action(self, action, now, *, expires_at, confirmation_key=None, since=None, vertical_limits=None,
                      yaw_limits=None, aircraft_key=None):
        if not self.connected:
            raise RuntimeError('flight_backend_unavailable')
        if vertical_limits is not None and (action is not FlightAction.TAKEOFF or not self.channel.vertical_trial):
            raise ValueError('vertical_trial_limits_require_profile_takeoff')
        if yaw_limits is not None and (action is not FlightAction.TAKEOFF or not self.channel.yaw_trial):
            raise ValueError('yaw_trial_limits_require_profile_takeoff')
        if action is not FlightAction.CONFIRM_LANDING and (confirmation_key is not None or since is not None):
            raise ValueError('landing_confirmation_guard_requires_confirmation_action')
        result = (self.channel.submit_landing(expires_at=expires_at,aircraft_key=aircraft_key)
                  if action is FlightAction.LAND else
                  self.channel.submit_action(action, expires_at=expires_at,
                                             confirmation_key=confirmation_key, since=since,
                                             vertical_limits=vertical_limits,yaw_limits=yaw_limits,aircraft_key=aircraft_key))
        return result['intent_id']

    def submit_control(self, channels, now, *, expires_at):
        if not self.connected:
            raise RuntimeError('flight_backend_unavailable')
        return self.channel.submit_control(channels, expires_at=expires_at)['intent_id']

    def action_receipt(self, intent_id):
        return self.channel.receipt(intent_id)

    def ground_confirmation_key(self, now):
        return self.channel.ground_confirmation_key(now)

    def confirm_ground(self, key, now):
        self.channel.confirm_ground(key,now)

    def invalidate_ground(self):
        self.channel.invalidate_ground()

    def aircraft_context(self, now, *, retained=False):
        if not self.opened:raise ValueError('recovery_route_unavailable')
        return self.channel.aircraft_context(now,retained=retained)

    def landing_route_key(self, now):
        if not self.opened:
            raise ValueError('landing_route_not_ready')
        return self.channel.landing_route_key(now)

    def landing_confirmation_key(self, now, *, since=None):
        if not self.opened:
            raise ValueError('landing_route_not_ready')
        return self.channel.landing_confirmation_key(now,since=since)

    def landing_confirmation_status(self, now):
        value = self.channel.landing_confirmation_status(now)
        if not self.opened:
            value.update(evidence_available=False, reason='landing_backend_not_open')
        return value
