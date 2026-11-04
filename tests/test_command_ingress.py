from dataclasses import FrozenInstanceError,replace

import pytest

from neo2_control.command_ingress import CommandIngress,CommandWindows,MAX_SEQUENCE


EPOCH='a'*48
GENERATION='b'*24


def window_ingress(window):
    return {'ticket':window['ticket'],'mutation_sequence':window['mutation_sequence'],
        'expected_owner_epoch':window['owner_epoch'],
        'expected_connection_generation':window['connection_generation']}


def test_windows_are_authenticated_single_use_and_do_not_queue_authority():
    clock=[100.]
    windows=CommandWindows(clock=lambda:clock[0])
    first=windows.issue(EPOCH,None)
    second=windows.issue(EPOCH,None)
    assert first['mutation_sequence']==1 and second['mutation_sequence']==2
    context=windows.consume(window_ingress(second),
        owner_epoch=EPOCH,connection_generation=None)
    assert context.issued_at==100. and context.expires_at==100.25
    assert CommandIngress.from_dict(context.as_dict())==context
    with pytest.raises(FrozenInstanceError):context.expires_at=200.
    for ticket in (first,second):
        with pytest.raises(ValueError,match='replayed'):
            windows.consume(window_ingress(ticket),
                owner_epoch=EPOCH,connection_generation=None)


@pytest.mark.parametrize('change',[
    lambda v:{**v,'ticket':v['ticket'][:-1]+('f' if v['ticket'][-1]!='f' else 'e')},
    lambda v:{**v,'mutation_sequence':v['mutation_sequence']+1},
    lambda v:{**v,'mutation_sequence':True},
    lambda v:{**v,'ticket':'x'*2049},
    lambda v:{**v,'extra':'untrusted'},
])
def test_tampered_carriers_cannot_modify_a_pc_deadline(change):
    windows=CommandWindows(clock=lambda:10.)
    ticket=windows.issue(EPOCH,GENERATION)
    value=window_ingress(ticket)
    with pytest.raises(ValueError):windows.consume(change(value),owner_epoch=EPOCH,
        connection_generation=GENERATION)
    assert windows.consume(value,owner_epoch=EPOCH,connection_generation=GENERATION).expires_at==10.25


@pytest.mark.parametrize('elapsed',[.25,.251,1.,-1.])
def test_elapsed_transport_never_renews_original_deadline(elapsed):
    clock=[20.];windows=CommandWindows(clock=lambda:clock[0])
    ticket=windows.issue(EPOCH,GENERATION)
    clock[0]+=elapsed
    with pytest.raises(ValueError,match='expired'):
        windows.consume(window_ingress(ticket),
            owner_epoch=EPOCH,connection_generation=GENERATION)


@pytest.mark.parametrize('owner,generation',[(EPOCH,None),('c'*48,GENERATION)])
def test_owner_and_connection_generation_both_bind_even_null(owner,generation):
    windows=CommandWindows(clock=lambda:10.)
    ticket=windows.issue(EPOCH,GENERATION)
    with pytest.raises(ValueError,match='generation_changed'):
        windows.consume(window_ingress(ticket),
            owner_epoch=owner,connection_generation=generation)


def test_windows_do_not_survive_http_application_replacement():
    first=CommandWindows(clock=lambda:10.);second=CommandWindows(clock=lambda:10.)
    ticket=first.issue(EPOCH,None)
    with pytest.raises(ValueError,match='invalid'):
        second.consume(window_ingress(ticket),
            owner_epoch=EPOCH,connection_generation=None)


@pytest.mark.parametrize('changes',[
    {'owner_epoch':'../owner'}, {'connection_generation':'../session'},
    {'issued_at':float('nan')}, {'expires_at':float('inf')},
    {'expires_at':100.251}, {'issued_at':-1.}, {'mutation_sequence':True},
    {'mutation_sequence':MAX_SEQUENCE+1}, {'mutation_sequence':0},
])
def test_private_context_rejects_malformed_values(changes):
    with pytest.raises(ValueError):replace(CommandIngress(EPOCH,GENERATION,1,100.,100.25),**changes)


def test_sequence_exhaustion_requires_a_new_application_and_secret():
    windows=CommandWindows(clock=lambda:10.);windows._issued=MAX_SEQUENCE-1
    assert windows.issue(EPOCH,None)['mutation_sequence']==MAX_SEQUENCE
    with pytest.raises(ValueError,match='exhausted'):windows.issue(EPOCH,None)
