import json

import pytest

from tools import replay_video_stream
from neo2_control.video_lineage import build_video_lineage
from test_video_lineage import encoded


def test_replay_clock_counts_actual_busy_time_and_compresses_idle(monkeypatch):
    wall = [100.]
    monkeypatch.setattr(replay_video_stream.time, 'monotonic', lambda:wall[0])
    clock = replay_video_stream.ReplayClock(True)
    clock.deliver(10.)
    wall[0] += .2
    assert clock() == pytest.approx(10.2)
    clock.deliver(10.1)  # This input waits for the already busy worker.
    wall[0] += .3
    assert clock() == pytest.approx(10.5)
    clock.deliver(20.)  # Jump over capture idle time, never run backwards.
    assert clock() == pytest.approx(20.)
    clock.idle(.351)
    assert clock() == pytest.approx(20.351)


def test_historical_clock_does_not_claim_to_include_worker_busy_time(monkeypatch):
    wall = [100.]
    monkeypatch.setattr(replay_video_stream.time, 'monotonic', lambda:wall[0])
    clock = replay_video_stream.ReplayClock(False)
    clock.deliver(10.)
    wall[0] += 20.
    assert clock() == 10.
    clock.deliver(10.1)
    assert clock() == 10.1


def test_replay_cli_runs_owned_decoder_process_with_same_b_picture_pixels(encoded, tmp_path, capsys):
    source, _ = encoded
    reference = tmp_path/'offline'
    build_video_lineage(source, reference, timeout=30)
    output = tmp_path/'stream.json'
    args = ['--source',str(source),'--reference',str(reference/'manifest.json'),
        '--output',str(output),'--time-mode','processing','--timeout','30']
    assert replay_video_stream.main(args) == 0
    report = json.loads(output.read_text())
    printed = json.loads(capsys.readouterr().out)
    assert report['summary'] == printed['summary']
    assert report['summary']['decoded_frames'] == 9
    assert report['summary']['decoder_thread_type'] == 'FRAME'
    assert report['latest_after_idle']['retained_frames'] == 0
    assert report['latest_after_idle']['retained_pixel_bytes'] == 0
    assert report['receipt_times'] == 'synthetic_1ms_chunks'
    assert report['context_origin'] == 'offline_replay'
    assert not report['hardware_used'] and not report['flight_authorized']
    assert output.stat().st_mode & 0o777 == 0o600
    original = output.read_bytes()
    with pytest.raises(ValueError, match='output_must_be_new'):
        replay_video_stream.main(args)
    assert output.read_bytes() == original
