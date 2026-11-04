import os
import time

from neo2_control.video import HlsVideo


def test_future_preview_timestamp_cannot_become_fresh_by_clamping_to_zero(tmp_path):
    video=HlsVideo(tmp_path/'hls',preview_jpeg=True)
    video.directory.mkdir(mode=0o700)
    target=video.directory/'preview.jpg';target.write_bytes(b'jpeg-fixture')
    future=time.time()+60;os.utime(target,(future,future))
    snapshot=video.snapshot()
    assert snapshot['preview_available'] is False and snapshot['preview_age_seconds'] is None
