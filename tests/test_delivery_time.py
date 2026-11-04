"""Conservative observation ages with completely unrelated monotonic epochs."""
import pytest

from neo2_control.delivery_time import land_since,motion_stamp
from neo2_control.operator_client import OperatorError


def status():
    return {'now':100.,'telemetry':{'motion_received_at':99.9,'component_age_seconds':{'motion':.1}},
        '_client_received_at':1_000_000.,'_client_round_trip_seconds':.6}


def test_only_differences_inside_one_clock_domain_are_compared():
    value=status()
    assert motion_stamp(value,1_000_000.1)==99.9
    value['land_requested_at']=99.5
    assert land_since(value,99.)
    assert not land_since(value,99.6)


@pytest.mark.parametrize('change',[
    {'now':99.}, {'now':float('nan')}, {'_client_received_at':1_000_001.},
    {'_client_round_trip_seconds':True},{'_client_round_trip_seconds':2.},
    {'_client_round_trip_seconds':float('inf')},
    {'telemetry':{'motion_received_at':99.9,'component_age_seconds':{'motion':None}}},
    {'telemetry':{'motion_received_at':99.,'component_age_seconds':{'motion':0.}}},
    {'telemetry':{'motion_received_at':99.9,'component_age_seconds':{'motion':1.}}},
    {'telemetry':{'motion_received_at':100.01}},
])
def test_missing_or_inconsistent_evidence_cannot_be_refreshed_by_network_allowance(change):
    value={**status(),**change}
    with pytest.raises(OperatorError):motion_stamp(value,1_000_000.1)


def test_reusing_status_does_not_refresh_it_as_the_client_clock_advances():
    value=status()
    assert motion_stamp(value,1_000_000.1)==99.9
    with pytest.raises(OperatorError,match='observation_stale'):motion_stamp(value,1_000_000.5)


def test_old_landing_from_previous_flight_is_not_a_new_landing_receipt():
    value=status();value['land_requested_at']=95.
    assert not land_since(value,99.)
    value['land_requested_at']=101.
    assert not land_since(value,99.)
