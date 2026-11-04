"""Separate PC observation time from client delivery time; never infer an offset."""
import math

from .operator_client import OperatorError


def finite(value):return type(value) in (int,float) and math.isfinite(value)


def server_now(status):
    value=status.get('now')
    if not finite(value) or value<0:raise OperatorError('program_server_clock_invalid')
    return value


def delivery(status,client_now):
    received=status.get('_client_received_at');rtt=status.get('_client_round_trip_seconds')
    if (not all(finite(v) for v in (received,rtt,client_now)) or not 0<=rtt<2
            or client_now<received):raise OperatorError('program_delivery_invalid')
    return rtt,client_now-received


def motion_stamp(status,client_now):
    """Require fresh PC evidence and account for every elapsed client second.

    The network allowance mirrors the WAN UI, bounded to two seconds. It
    never relaxes the PC's 500 ms check or turns an old receipt into a new one.
    The returned timestamp remains in the PC domain, for sample spacing only.
    """
    rtt,elapsed=delivery(status,client_now)
    sample=status.get('telemetry') or {}
    stamp=sample.get('motion_received_at')
    if stamp is None and sample.get('synthetic') is True:stamp=sample.get('received_at')
    pc_now=server_now(status)
    if not finite(stamp) or not 0<=pc_now-stamp:
        raise OperatorError('program_observation_stale')
    base=pc_now-stamp
    components=sample.get('component_age_seconds')
    if components is not None:
        age=components.get('motion') if isinstance(components,dict) else None
        if not finite(age) or age<0:raise OperatorError('program_observation_stale')
        base=max(base,age)
    if base+rtt+elapsed>min(2.,.5+rtt):raise OperatorError('program_observation_stale')
    return stamp


def land_since(status,reference):
    requested=status.get('land_requested_at');now=server_now(status)
    if not finite(reference) or now<reference:raise OperatorError('program_server_clock_invalid')
    return finite(requested) and reference<=requested<=now
