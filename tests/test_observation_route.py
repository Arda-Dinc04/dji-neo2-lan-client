import json
from types import SimpleNamespace

import pytest

from neo2_control import wifi_link


def install_routes(monkeypatch, defaults, selected):
    calls = []
    def command(*args, **kwargs):
        calls.append(args)
        assert args[0] == 'ip' and not kwargs.get('input_text')
        if 'get' in args:
            return SimpleNamespace(stdout=json.dumps(selected))
        return SimpleNamespace(stdout=json.dumps(defaults.get(args[2], [])))
    monkeypatch.setattr(wifi_link, '_run', command)
    return calls


def test_dedicated_route_checks_ipv4_ipv6_defaults_then_fixed_peer_without_mutations(monkeypatch):
    calls = install_routes(monkeypatch, {'-4': [{'dev': 'uplink0'}]},
                           [{'dst': '192.168.2.1', 'dev': 'drone0', 'prefsrc': '192.168.2.100'}])
    wifi_link.require_dedicated_drone_route('drone0')
    assert len(calls) == 3
    assert all('show' in call or 'get' in call for call in calls)


@pytest.mark.parametrize('defaults', [
    {'-4': [{'dev': 'drone0'}]}, {'-6': [{'dev': 'drone0'}]},
    {'-4': [{'nexthops': [{'dev': 'uplink0'}, {'dev': 'drone0'}]}]},
])
def test_any_default_route_on_selected_interface_prevents_a_session(monkeypatch, defaults):
    calls = install_routes(monkeypatch, defaults, [{'dst': '192.168.2.1', 'dev': 'drone0'}])
    with pytest.raises(wifi_link.WifiLinkError, match='carries_default_route'):
        wifi_link.require_dedicated_drone_route('drone0')
    assert not any('get' in call for call in calls)


@pytest.mark.parametrize('routes', [[], {}, [{'dev': 'drone0'}],
    [{'dst': '192.168.2.1', 'dev': 'uplink0'}],
    [{'dst': '192.168.2.1', 'dev': 'drone0', 'gateway': '10.0.0.1'}],
    [{'dst': '192.168.2.1', 'dev': 'drone0', 'type': 'local'}],
    [{'dst': '192.168.2.2', 'dev': 'drone0'}],
])
def test_wrong_missing_indirect_or_local_peer_route_is_rejected(monkeypatch, routes):
    install_routes(monkeypatch, {}, routes)
    with pytest.raises(wifi_link.WifiLinkError, match='not_direct_on_selected_interface'):
        wifi_link.require_dedicated_drone_route('drone0')
