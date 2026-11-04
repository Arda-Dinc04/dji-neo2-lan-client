import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from neo2_control.capture import CaptureRecorder, load_capture
from neo2_control.report import generate_report
from neo2_control.transport import ReliableTransport


class CaptureReportTests(unittest.TestCase):
    def test_deferred_annotations_keep_packet_order_bytes_and_receipt_clocks_and_report_redecodes(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / 'deferred.jsonl'
            raw = ReliableTransport(session=0x1111, base_sequence=0x2220).open_session()
            recorder = CaptureRecorder(capture, decode_annotations=False)
            with patch('neo2_control.capture.decode_datagram', side_effect=AssertionError('live diagnostic decoding')):
                for direction, stamp in (('tx', 123456789), ('rx', 123456790)):
                    event = recorder.record(interface='udp', direction=direction, peer='127.0.0.1:1',
                        raw=raw, timestamp_ns=1792135034000000000, monotonic_ns=stamp)
                    self.assertEqual(event.decoded, {'length': len(raw), 'annotations_deferred': True})
            rows = load_capture(capture)
            self.assertEqual([row['sequence'] for row in rows], [1, 2])
            self.assertEqual([row['direction'] for row in rows], ['tx', 'rx'])
            self.assertEqual([row['monotonic_ns'] for row in rows], [123456789, 123456790])
            self.assertTrue(all(bytes.fromhex(row['raw_hex']) == raw for row in rows))
            self.assertTrue(all(row['timestamp_ns'] == 1792135034000000000 for row in rows))
            page = generate_report(capture, Path(directory) / 'report.html').read_text()
            self.assertIn('dji-reliable-udp', page)
            self.assertIn('link_word', page)
            self.assertNotIn('annotations_deferred', page)
            self.assertTrue(all(row['decoded']['annotations_deferred'] for row in load_capture(capture)))

    def test_invalid_annotation_option_cannot_create_capture_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'absent' / 'capture.jsonl'
            for value in (None, 0, 1, 'false'):
                with self.assertRaises(ValueError):CaptureRecorder(target, decode_annotations=value)
            self.assertFalse(target.parent.exists())

    def test_capture_retains_socket_receipt_clocks_supplied_before_diagnostic_work(self):
        with tempfile.TemporaryDirectory() as directory:
            capture=Path(directory)/'receipt.jsonl'
            event=CaptureRecorder(capture).record(interface='udp',direction='rx',peer='127.0.0.1:1',
                raw=b'fixture',timestamp_ns=1792135034000000000,monotonic_ns=123456789)
            self.assertEqual(event.monotonic_ns,123456789)
            self.assertEqual(event.timestamp_ns,1792135034000000000)
            self.assertEqual(load_capture(capture)[0]['monotonic_ns'],123456789)

    def test_capture_is_lossless_and_report_is_embedded(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "capture.jsonl"
            report = Path(directory) / "report.html"
            raw = ReliableTransport(session=0x1111, base_sequence=0x2220).open_session()
            event = CaptureRecorder(capture).record(
                interface="udp",
                direction="tx",
                peer="192.168.2.1:9003",
                raw=raw,
                note="test",
            )
            self.assertEqual(event.raw_hex, raw.hex())
            loaded = load_capture(capture)
            self.assertEqual(bytes.fromhex(loaded[0]["raw_hex"]), raw)
            output = generate_report(capture, report)
            page = output.read_text(encoding="utf-8")
            self.assertIn("inspection au bit près", page)
            self.assertIn(raw.hex(), page)
            self.assertNotIn("__EVENTS__", page)

    def test_report_redecodes_archived_annotations_from_the_raw_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "stale.jsonl"
            report = Path(directory) / "stale.html"
            raw = ReliableTransport(session=0x1111, base_sequence=0x2220).open_session()
            capture.write_text(
                json.dumps(
                    {
                        "sequence": 1,
                        "timestamp": "2026-07-28T00:00:00+00:00",
                        "timestamp_ns": 0,
                        "monotonic_ns": 0,
                        "interface": "udp",
                        "direction": "tx",
                        "peer": "192.168.2.1:9003",
                        "raw_hex": raw.hex(),
                        "decoded": {"layers": [{"name": "obsolete-reading"}]},
                        "note": "captured by an older decoder",
                    }
                )
                + "\n",
                encoding="utf-8",
                newline="\n",
            )
            page = generate_report(capture, report).read_text(encoding="utf-8")
            self.assertNotIn("obsolete-reading", page)
            self.assertIn("dji-reliable-udp", page)
            self.assertIn(raw.hex(), page)


if __name__ == "__main__":
    unittest.main()
