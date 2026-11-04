"""Real B-picture pixels and source clocks survive decoder thread buffering."""
import hashlib,json,subprocess

import pytest

from neo2_control.room_stream import audited_frames
from neo2_control.video_export import _normalizer
from neo2_control.video_lineage import _TrackedFilter,build_video_lineage
from test_video_lineage import encoded


@pytest.fixture
def recording(encoded,tmp_path):
    source,hashes=encoded
    manifest=build_video_lineage(source,tmp_path/'lineage',nominal_fps=6,timeout=30)
    filter=_TrackedFilter();raw=filter.feed(source.read_bytes())+filter.finish()
    canonical=tmp_path/'canonical.hevc'
    result=subprocess.run(_normalizer('prlimit','ffmpeg',6,tmp_path),input=raw,
        capture_output=True,check=True,timeout=20)
    canonical.write_bytes(result.stdout)
    assert hashlib.sha256(result.stdout).hexdigest()==manifest['preparation']['normalization']['sha256']
    pictures=[]
    for item in filter.records:
        if not item['first_slice']:continue
        ordinal=len(pictures)
        pictures.append({'prepared_picture_ordinal':ordinal,'prepared_start':item['source_start'],
            'body_sha256':item['body_sha256'],'last_byte_delivered_monotonic':100.+ordinal/10})
    audit=tmp_path/'audit.json';lineage=tmp_path/'lineage/manifest.json'
    audit.write_text(json.dumps({'prepared_source':manifest['source'],'source':manifest['source'],
        'pictures':pictures}))
    return canonical,lineage,audit,manifest,hashes


def test_rgb_and_packet_clocks_are_identical_across_decoder_thread_modes(recording):
    canonical,lineage,audit,manifest,hashes=recording
    modes=[('SLICE',1),('FRAME',2),('FRAME',4)]
    previous=None
    for mode,threads in modes:
        frames=list(audited_frames(canonical,lineage,audit,stride=1,
            decoder_thread_type=mode,decoder_threads=threads))
        actual=[(f.decoded_index,f.recorded_available_at,hashlib.sha256(f.rgb).hexdigest()) for f in frames]
        expected=[(f['decoded_index'],100.+f['packet']['index']/10,hashes[f['decoded_index']])
            for f in manifest['frames']]
        assert actual==expected
        if previous is not None:assert actual==previous
        previous=actual


def test_backpressure_can_skip_conversion_but_keeps_the_next_frames_identity(recording):
    canonical,lineage,audit,manifest,hashes=recording
    inspected=[]
    def accept(index,available):
        inspected.append((index,available));return index in (2,6)
    frames=list(audited_frames(canonical,lineage,audit,stride=2,accept=accept))
    assert [f.decoded_index for f in frames]==[2,6]
    assert [hashlib.sha256(f.rgb).hexdigest() for f in frames]==[hashes[2],hashes[6]]
    assert [i for i,t in inspected]==[0,2,4,6,8]
    assert [t for i,t in inspected]==[100.+manifest['frames'][i]['packet']['index']/10 for i in (0,2,4,6,8)]
