from dataclasses import replace
import pytest
from neo2_control.operator_client import PreviewFrame
from neo2_control.preview_acquisition import ReadOnlyPreviewFeed

CONTEXT=('a'*48,'b'*24)


class ReadOnlyClient:
    """Deliberately provides no command or connection methods."""
    def __init__(self):
        self.now=10.;self.calls=[];self.changed=False;self.delayed=False
        self.frame=PreviewFrame(b'fixture',CONTEXT[1],'frame1',.1,9.95,10.)
    def status(self):
        self.calls.append('status')
        if self.delayed and self.calls.count('status')==2:self.now=10.4
        generation='c'*24 if self.changed and self.calls.count('status')==2 else CONTEXT[1]
        return {'connected':True,'controller_owner':{'epoch':CONTEXT[0]},
            'connection':{'generation':generation,'video':{'live':True}}}
    def read_frame(self):self.calls.append('read_frame');return self.frame


def test_preview_uses_only_read_requests_and_rejects_duplicate_images():
    client=ReadOnlyClient();feed=ReadOnlyPreviewFeed(client,CONTEXT,clock=lambda:client.now)
    result=feed.read();assert result['raw']==b'fixture'
    assert result['received_at']==10.
    assert result['metadata']['preview_availability_age_bound_seconds']==pytest.approx(.15)
    assert feed.read() is None
    assert client.calls==['status','read_frame','status']*2


def test_status_delay_does_not_make_an_old_preview_fresh_again():
    client=ReadOnlyClient();client.delayed=True
    assert ReadOnlyPreviewFeed(client,CONTEXT,clock=lambda:client.now).read() is None


def test_generation_change_while_reading_discards_the_image():
    client=ReadOnlyClient();client.changed=True
    with pytest.raises(ValueError,match='generation_changed'):
        ReadOnlyPreviewFeed(client,CONTEXT,clock=lambda:client.now).read()


def test_frame_from_another_generation_is_not_relabelled():
    client=ReadOnlyClient();client.frame=replace(client.frame,connection_generation='c'*24)
    with pytest.raises(ValueError,match='evidence_invalid'):
        ReadOnlyPreviewFeed(client,CONTEXT,clock=lambda:client.now).read()


def test_measured_altitude_is_carried_without_an_extra_request():
    client=ReadOnlyClient();original=client.status
    def status():
        return {**original(),'now':9.94,'synthetic':False,'telemetry':{
            'synthetic':False,'altitude_m':1.2,'motion_received_at':9.9}}
    client.status=status
    result=ReadOnlyPreviewFeed(client,CONTEXT,clock=lambda:client.now).read()
    assert result['metadata']['altitude_observation']['value_m']==1.2
    assert client.calls==['status','read_frame','status']
