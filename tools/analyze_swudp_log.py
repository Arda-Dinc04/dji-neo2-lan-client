#!/usr/bin/env python3
"""Extract a privacy-preserving SWUDP session summary from DJI UAVSDK logs.

The public analyzer accepts decoded text logs. Encrypted SDK log decoding is
not distributed with this source snapshot. It deliberately
retains only SWUDP transport metadata; unrelated application records and
device identifiers never enter the generated JSON or HTML report.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable, Iterator
from datetime import datetime
import html
import json
from pathlib import Path
import re
import statistics
from typing import Any

FRAME_MAGIC = b"\x12\x77\x45\x00"


TIMESTAMP_RE = re.compile(
    r"^(?P<timestamp>\d{4}-\d{2}-\d{2} "
    r"\d{2}:\d{2}:\d{2}\.\d{3}) "
)
PORT_RE = re.compile(r"\[WifiServicePort\] Port:(?P<port>\d+)")
LOCAL_PORT_RE = re.compile(r"Udp_Create: Local port: (?P<port>\d+)")
SESSION_RE = re.compile(
    r"SW_Pro_Gnd_Manage_Init: ReqSessionID: (?P<session>\d+)"
)
ENDPOINT_RE = re.compile(
    r"SW_Pro_Gnd_Entry_Start success, .*?"
    r"ip_ = (?P<ip>\d{1,3}(?:\.\d{1,3}){3}), "
    r"port_ = (?P<port>\d+)"
)
MANAGE_COUNT_RE = re.compile(
    r"SW_Pro_Gnd_Manage_Send_Req send (?P<count>\d+)\s*$"
)
BITRATE_RE = re.compile(r"set sw max bitrate = (?P<value>\d+)")
THRESHOLD_RE = re.compile(
    r"set avgTputKbps level th\[(?P<index>\d+)\]="
    r"(?P<value>\d+)"
)
KEY_VALUE_RE = re.compile(r"(?P<key>[A-Za-z]+)=(?P<value>\d+)")
FLOW_COUNT_RE = re.compile(r"(?P<direction>Send|Recv) V1 count: (?P<count>\d+)")
FLOW_ROW_RE = re.compile(
    r"\[SWUDP-I\]\s+"
    r"(?P<index>\d+)\s+"
    r"(?P<sender>0x[0-9a-fA-F]{4})\s+"
    r"(?P<receiver>0x[0-9a-fA-F]{4})\s+"
    r"(?P<cmd_set>0x[0-9a-fA-F]{2})\s+"
    r"(?P<cmd_id>0x[0-9a-fA-F]{2})\s+"
    r"(?P<total>\d+)\s+"
    r"(?P<fail>\d+)\s+"
    r"(?P<frequency_hz>\d+(?:\.\d+)?)\s+"
    r"(?P<last_ms>\d+)\s*$"
)
VIDEO_CALLBACK_RE = re.compile(
    r"VideoObserver ProcessRecvVideoData recv data len = "
    r"(?P<length>\d+), total len = (?P<total>\d+)"
)
VIDEO_RATE_RE = re.compile(
    r"\[SW\]sw_video.*?(?P<recv>\d+(?:\.\d+)?)"
    r"(?P<recv_unit>KB/s|B/s).*?(?P<send>\d+(?:\.\d+)?)"
    r"(?P<send_unit>KB/s|B/s)"
)
XRCE_SUBSCRIPTION_RE = re.compile(
    r"\[XRCE_MGR\]: subscribe Topic: (?P<topic>[^,]+), "
    r"register_id: (?P<register_id>\d+), host_id: "
    r"(?P<host_type>\d+),(?P<host_index>\d+)"
)
STARTUP_MARKERS = (
    ("wifi_device_info_sent_at", "[WlmAssistantLogic] send wifi device info"),
    ("heartbeat_sent_at", "[Heartbeat]Begin Send Heartbeat Pack"),
    (
        "monitor_user_setting_sent_at",
        "[PeriodOpenMCMonitorLogic] SendUserSettingPack",
    ),
    (
        "product_static_capability_request_at",
        "[ProductAbstraction] SendRequest static_cap_type=",
    ),
    (
        "camera_video_module_loaded_at",
        "[Module]: loaded - N3uav3sdk17CameraVideoModuleE",
    ),
    ("first_xrce_subscription_at", "[XRCE_MGR]: subscribe Topic:"),
    ("first_xrce_request_sent_at", "[XRCE_DDS_CLIENT]: Send DDS Pack."),
)
MARKED_EMISSION_FIELDS = frozenset(
    {
        "wifi_device_info_sent_at",
        "heartbeat_sent_at",
        "monitor_user_setting_sent_at",
        "product_static_capability_request_at",
        "first_xrce_request_sent_at",
    }
)


def _parse_timestamp(value: str) -> datetime:
    return datetime.strptime(value, "%Y-%m-%d %H:%M:%S.%f")


def _milliseconds_between(start: str | None, end: str | None) -> int | None:
    if start is None or end is None:
        return None
    return round((_parse_timestamp(end) - _parse_timestamp(start)).total_seconds() * 1000)


def iter_plaintext_lines(path: Path) -> Iterator[str]:
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        yield from stream


def input_is_encrypted(path: Path) -> bool:
    with path.open("rb") as stream:
        return stream.read(4) == FRAME_MAGIC


def analyze_lines(lines: Iterable[str]) -> dict[str, Any]:
    """Return only transport-level observations from one log stream."""

    summary: dict[str, Any] = {
        "schema": "neo2-swudp-log-summary/v3",
        "log": {"first_timestamp": None, "last_timestamp": None},
        "transport": {
            "service_port": None,
            "local_udp_port": None,
            "remote_ip": None,
            "remote_port": None,
            "request_session_id": None,
        },
        "management": {
            "initialized_at": None,
            "request_checkpoints_observed": 0,
            "first_request_checkpoint": None,
            "last_request_checkpoint": None,
            "maximum_request_count": None,
            "failed_send_count": 0,
            "estimated_request_period_ms": None,
        },
        "connection": {
            "link_initialized_at": None,
            "connected_at": None,
            "negotiation_ms": None,
            "maximum_bitrate_kbps": None,
            "throughput_thresholds_kbps": {},
        },
        "video": {
            "demand_connection_changed_at": None,
            "receiver_changed_at": None,
            "first_callback_at": None,
            "first_callback_length": None,
            "first_callback_total": None,
            "first_iframe_request_at": None,
            "first_callback_after_link_ms": None,
            "receiver_before_first_callback_ms": None,
            "first_iframe_after_first_callback_ms": None,
        },
        "startup": {
            "wifi_device_info_sent_at": None,
            "heartbeat_sent_at": None,
            "monitor_user_setting_sent_at": None,
            "product_static_capability_request_at": None,
            "camera_video_module_loaded_at": None,
            "first_xrce_subscription_at": None,
            "first_xrce_request_sent_at": None,
            "timing_from_connection_ms": {},
            "relative_to_first_video_ms": {},
            "pre_video_marked_emissions": [],
        },
        "xrce": {
            "registration_count": 0,
            "unique_topic_count": 0,
            "host_ids": [],
            "first_subscription_after_video_ms": None,
            "first_request_after_video_ms": None,
        },
        "video_samples": [],
        "link_samples": [],
        "flows": {
            "send": {
                "declared_count": None,
                "snapshots_observed": 0,
                "rows": [],
            },
            "recv": {
                "declared_count": None,
                "snapshots_observed": 0,
                "rows": [],
            },
        },
    }

    current_flow_direction: str | None = None
    link_samples: dict[str, dict[str, Any]] = {}
    video_samples: dict[str, dict[str, Any]] = {}
    request_checkpoints: list[dict[str, Any]] = []
    xrce_topics: set[str] = set()
    xrce_host_ids: set[tuple[int, int]] = set()

    for raw_line in lines:
        line = raw_line.rstrip()
        timestamp_match = TIMESTAMP_RE.match(line)
        timestamp = (
            timestamp_match.group("timestamp") if timestamp_match else None
        )
        if timestamp is not None:
            if summary["log"]["first_timestamp"] is None:
                summary["log"]["first_timestamp"] = timestamp
            summary["log"]["last_timestamp"] = timestamp

        if match := PORT_RE.search(line):
            summary["transport"]["service_port"] = int(match.group("port"))
        if match := LOCAL_PORT_RE.search(line):
            summary["transport"]["local_udp_port"] = int(match.group("port"))
        if match := SESSION_RE.search(line):
            summary["transport"]["request_session_id"] = int(
                match.group("session")
            )
            summary["management"]["initialized_at"] = timestamp
        if match := ENDPOINT_RE.search(line):
            summary["transport"]["remote_ip"] = match.group("ip")
            summary["transport"]["remote_port"] = int(match.group("port"))

        if match := MANAGE_COUNT_RE.search(line):
            request_checkpoints.append(
                {"timestamp": timestamp, "count": int(match.group("count"))}
            )
        elif "SW_Pro_Gnd_Manage_Send_Req send failed" in line:
            summary["management"]["failed_send_count"] += 1

        if "SW_Pro_Gnd_Link_Init: Gnd link ReqSessionID:" in line:
            summary["connection"]["link_initialized_at"] = timestamp
        if "wifi datalink connected 1" in line:
            if summary["connection"]["connected_at"] is not None:
                raise ValueError(
                    "multiple connected SWUDP sessions found; analyze one session at a time"
                )
            summary["connection"]["connected_at"] = timestamp
        if match := BITRATE_RE.search(line):
            summary["connection"]["maximum_bitrate_kbps"] = int(
                match.group("value")
            )
        if match := THRESHOLD_RE.search(line):
            summary["connection"]["throughput_thresholds_kbps"][
                match.group("index")
            ] = int(match.group("value"))

        if (
            "ConnectionChanged" in line
            and "[Demand-I]" in line
            and "connection = 1" in line
            and summary["video"]["demand_connection_changed_at"] is None
        ):
            summary["video"]["demand_connection_changed_at"] = timestamp
        if (
            "[Demand-I][SpecialCommandManager] receiver change to" in line
            and summary["video"]["receiver_changed_at"] is None
        ):
            summary["video"]["receiver_changed_at"] = timestamp
        if (
            (match := VIDEO_CALLBACK_RE.search(line))
            and summary["video"]["first_callback_at"] is None
        ):
            summary["video"]["first_callback_at"] = timestamp
            summary["video"]["first_callback_length"] = int(
                match.group("length")
            )
            summary["video"]["first_callback_total"] = int(
                match.group("total")
            )
        if (
            "[Demand-I] Request I Frame" in line
            and summary["video"]["first_iframe_request_at"] is None
        ):
            summary["video"]["first_iframe_request_at"] = timestamp
        for field, marker in STARTUP_MARKERS:
            if (
                marker in line
                and summary["connection"]["connected_at"] is not None
                and timestamp is not None
                and timestamp >= summary["connection"]["connected_at"]
                and summary["startup"][field] is None
            ):
                summary["startup"][field] = timestamp
        if (
            (match := XRCE_SUBSCRIPTION_RE.search(line))
            and summary["connection"]["connected_at"] is not None
            and timestamp is not None
            and timestamp >= summary["connection"]["connected_at"]
        ):
            summary["xrce"]["registration_count"] += 1
            xrce_topics.add(match.group("topic"))
            xrce_host_ids.add(
                (int(match.group("host_type")), int(match.group("host_index")))
            )
        if timestamp is not None and (match := VIDEO_RATE_RE.search(line)):
            recv = float(match.group("recv"))
            send = float(match.group("send"))
            if match.group("recv_unit") == "B/s":
                recv /= 1024.0
            if match.group("send_unit") == "B/s":
                send /= 1024.0
            video_samples[timestamp] = {
                "timestamp": timestamp,
                "receive_kib_s": round(recv, 3),
                "send_kib_s": round(send, 3),
            }

        if timestamp is not None and "<CONN>[T=link_stat]" in line:
            sample = link_samples.setdefault(timestamp, {"timestamp": timestamp})
            for match in KEY_VALUE_RE.finditer(line):
                sample[match.group("key")] = int(match.group("value"))

        if match := FLOW_COUNT_RE.search(line):
            current_flow_direction = match.group("direction").lower()
            flow = summary["flows"][current_flow_direction]
            flow["declared_count"] = int(match.group("count"))
            flow["snapshots_observed"] += 1
            # Each printed table is a complete point-in-time inventory.  Keep
            # only the newest one so long sessions remain compact.
            flow["rows"] = []
            continue

        if current_flow_direction is not None and (
            match := FLOW_ROW_RE.search(line)
        ):
            row: dict[str, Any] = {
                "index": int(match.group("index")),
                "sender": match.group("sender").lower(),
                "receiver": match.group("receiver").lower(),
                "command_set": match.group("cmd_set").lower(),
                "command_id": match.group("cmd_id").lower(),
                "total": int(match.group("total")),
                "fail": int(match.group("fail")),
                "frequency_hz": float(match.group("frequency_hz")),
                "last_ms": int(match.group("last_ms")),
            }
            summary["flows"][current_flow_direction]["rows"].append(row)
        elif current_flow_direction is not None and "[SWUDP-I]" not in line:
            current_flow_direction = None

    periods: list[float] = []
    for previous, current in zip(request_checkpoints, request_checkpoints[1:]):
        count_delta = current["count"] - previous["count"]
        if (
            count_delta > 0
            and previous["timestamp"] is not None
            and current["timestamp"] is not None
        ):
            elapsed_ms = _milliseconds_between(
                previous["timestamp"], current["timestamp"]
            )
            if elapsed_ms is not None:
                periods.append(elapsed_ms / count_delta)
    if periods:
        summary["management"]["estimated_request_period_ms"] = round(
            statistics.median(periods), 1
        )
    if request_checkpoints:
        summary["management"]["request_checkpoints_observed"] = len(
            request_checkpoints
        )
        summary["management"]["first_request_checkpoint"] = (
            request_checkpoints[0]
        )
        summary["management"]["last_request_checkpoint"] = (
            request_checkpoints[-1]
        )
        summary["management"]["maximum_request_count"] = max(
            checkpoint["count"] for checkpoint in request_checkpoints
        )

    summary["connection"]["negotiation_ms"] = _milliseconds_between(
        summary["management"]["initialized_at"],
        summary["connection"]["connected_at"],
    )
    summary["video"]["first_callback_after_link_ms"] = _milliseconds_between(
        summary["connection"]["connected_at"],
        summary["video"]["first_callback_at"],
    )
    summary["video"]["receiver_before_first_callback_ms"] = (
        _milliseconds_between(
            summary["video"]["receiver_changed_at"],
            summary["video"]["first_callback_at"],
        )
    )
    summary["video"]["first_iframe_after_first_callback_ms"] = (
        _milliseconds_between(
            summary["video"]["first_callback_at"],
            summary["video"]["first_iframe_request_at"],
        )
    )
    first_callback = summary["video"]["first_callback_at"]
    connected_at = summary["connection"]["connected_at"]
    event_times = {
        field: summary["startup"][field]
        for field, _ in STARTUP_MARKERS
        if summary["startup"][field] is not None
    }
    summary["startup"]["timing_from_connection_ms"] = {
        field: _milliseconds_between(connected_at, value)
        for field, value in event_times.items()
    }
    summary["startup"]["relative_to_first_video_ms"] = {
        field: _milliseconds_between(first_callback, value)
        for field, value in event_times.items()
    }
    summary["startup"]["pre_video_marked_emissions"] = [
        {
            "field": field,
            "timestamp": value,
            "milliseconds_before_video": -relative,
        }
        for field, value in event_times.items()
        if field in MARKED_EMISSION_FIELDS
        and (relative := _milliseconds_between(first_callback, value)) is not None
        and relative < 0
    ]
    summary["xrce"]["unique_topic_count"] = len(xrce_topics)
    summary["xrce"]["host_ids"] = [
        {"type": host_type, "index": host_index}
        for host_type, host_index in sorted(xrce_host_ids)
    ]
    summary["xrce"]["first_subscription_after_video_ms"] = (
        _milliseconds_between(
            first_callback,
            summary["startup"]["first_xrce_subscription_at"],
        )
    )
    summary["xrce"]["first_request_after_video_ms"] = _milliseconds_between(
        first_callback,
        summary["startup"]["first_xrce_request_sent_at"],
    )
    summary["video_samples"] = list(video_samples.values())
    summary["link_samples"] = list(link_samples.values())
    return summary


def _svg_series(
    samples: list[dict[str, Any]],
    field: str,
    *,
    title: str,
    unit: str,
    color: str,
) -> str:
    points = [
        (sample["timestamp"], float(sample[field]))
        for sample in samples
        if field in sample
    ]
    if not points:
        return f"<p>Aucun échantillon pour {html.escape(title)}.</p>"

    width, height = 900, 250
    left, top, right, bottom = 62, 24, 20, 42
    plot_width = width - left - right
    plot_height = height - top - bottom
    times = [_parse_timestamp(timestamp) for timestamp, _ in points]
    origin = times[0]
    x_values = [(value - origin).total_seconds() for value in times]
    y_values = [value for _, value in points]
    x_max = max(x_values) or 1.0
    y_min = min(y_values)
    y_max = max(y_values)
    if y_min == y_max:
        y_min = 0.0
        y_max = y_max or 1.0

    def x(value: float) -> float:
        return left + value / x_max * plot_width

    def y(value: float) -> float:
        return top + (y_max - value) / (y_max - y_min) * plot_height

    polyline = " ".join(
        f"{x(x_value):.1f},{y(y_value):.1f}"
        for x_value, y_value in zip(x_values, y_values)
    )
    grid = []
    labels = []
    for index in range(5):
        fraction = index / 4
        gy = top + fraction * plot_height
        value = y_max - fraction * (y_max - y_min)
        grid.append(
            f'<line x1="{left}" y1="{gy:.1f}" x2="{width-right}" '
            f'y2="{gy:.1f}" class="grid"/>'
        )
        labels.append(
            f'<text x="{left-8}" y="{gy+4:.1f}" text-anchor="end">'
            f"{value:.0f}</text>"
        )

    return (
        f'<section><h2>{html.escape(title)}</h2>'
        f'<svg viewBox="0 0 {width} {height}" role="img" '
        f'aria-label="{html.escape(title)}">'
        + "".join(grid)
        + "".join(labels)
        + f'<polyline points="{polyline}" fill="none" stroke="{color}" '
        'stroke-width="3" stroke-linejoin="round"/>'
        f'<text x="{left}" y="{height-10}">0 s</text>'
        f'<text x="{width-right}" y="{height-10}" text-anchor="end">'
        f"{x_max:.1f} s</text>"
        f'<text x="14" y="{top}" transform="rotate(90 14 {top})">'
        f"{html.escape(unit)}</text></svg></section>"
    )


def _flow_table(direction: str, flow: dict[str, Any]) -> str:
    rows = sorted(
        flow["rows"],
        key=lambda row: (row["frequency_hz"], row["total"]),
        reverse=True,
    )[:25]
    body = "".join(
        "<tr>"
        f"<td>{row['index']}</td>"
        f"<td><code>{row['sender']}</code></td>"
        f"<td><code>{row['receiver']}</code></td>"
        f"<td><code>{row['command_set']}</code></td>"
        f"<td><code>{row['command_id']}</code></td>"
        f"<td>{row['total']}</td>"
        f"<td>{row['fail']}</td>"
        f"<td>{row['frequency_hz']:.1f}</td>"
        f"<td>{row['last_ms']}</td>"
        "</tr>"
        for row in rows
    )
    declared = flow["declared_count"]
    return (
        f"<section><h2>Flux V1 {html.escape(direction)}</h2>"
        f"<p>{len(flow['rows'])} lignes extraites"
        + (f" / {declared} annoncées." if declared is not None else ".")
        + f" Inventaires observés : {flow['snapshots_observed']}."
        + "</p><div class=\"table-wrap\"><table><thead><tr>"
        "<th>#</th><th>Émetteur</th><th>Récepteur</th>"
        "<th>CmdSet</th><th>CmdId</th><th>Total</th><th>Échecs</th>"
        "<th>Hz</th><th>Dernier ms</th></tr></thead>"
        f"<tbody>{body}</tbody></table></div></section>"
    )


def render_html(summary: dict[str, Any], *, title: str) -> str:
    transport = summary["transport"]
    management = summary["management"]
    connection = summary["connection"]
    video = summary["video"]
    startup = summary["startup"]
    samples = summary["link_samples"]
    video_samples = summary["video_samples"]
    endpoint = "—"
    if transport["remote_ip"] and transport["remote_port"]:
        endpoint = f"{transport['remote_ip']}:{transport['remote_port']}"

    cards = [
        ("Destination", endpoint),
        ("Port UDP local", transport["local_udp_port"] or "—"),
        ("Session", transport["request_session_id"] or "—"),
        ("Période requête", (
            f"{management['estimated_request_period_ms']} ms"
            if management["estimated_request_period_ms"] is not None
            else "—"
        )),
        ("Négociation", (
            f"{connection['negotiation_ms']} ms"
            if connection["negotiation_ms"] is not None
            else "non aboutie"
        )),
        ("Première vidéo", (
            f"+{video['first_callback_after_link_ms']} ms"
            if video["first_callback_after_link_ms"] is not None
            else "non observée"
        )),
        ("Échantillons lien", len(samples)),
    ]
    card_html = "".join(
        f'<div class="card"><span>{html.escape(str(label))}</span>'
        f"<strong>{html.escape(str(value))}</strong></div>"
        for label, value in cards
    )

    timeline_events = [
        ("Initialisation gestion", management["initialized_at"]),
        ("Initialisation lien", connection["link_initialized_at"]),
        ("Datalink connecté", connection["connected_at"]),
        ("Information Wi-Fi applicative émise", startup["wifi_device_info_sent_at"]),
        ("Premier heartbeat V1", startup["heartbeat_sent_at"]),
        ("Réglage moniteur V1", startup["monitor_user_setting_sent_at"]),
        (
            "Requête de capacité statique",
            startup["product_static_capability_request_at"],
        ),
        ("Module vidéo caméra chargé", startup["camera_video_module_loaded_at"]),
        ("Premier abonnement XRCE local", startup["first_xrce_subscription_at"]),
        ("Demand-I connecté", video["demand_connection_changed_at"]),
        ("Destinataire Demand-I changé", video["receiver_changed_at"]),
        ("Premier callback vidéo", video["first_callback_at"]),
        ("Première requête XRCE émise", startup["first_xrce_request_sent_at"]),
        ("Première demande d’I-frame", video["first_iframe_request_at"]),
    ]
    timeline = "".join(
        f"<li><time>{html.escape(value or 'non observé')}</time>"
        f"<span>{html.escape(label)}</span></li>"
        for label, value in timeline_events
    )

    return f"""<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(title)}</title>
<style>
:root {{ color-scheme: dark; --bg:#07111f; --panel:#101f33; --text:#e7eef8;
  --muted:#92a5bd; --accent:#46d6b6; --grid:#29405d; }}
* {{ box-sizing:border-box }} body {{ margin:0; background:var(--bg);
  color:var(--text); font:15px/1.45 system-ui,sans-serif }}
main {{ max-width:1100px; margin:auto; padding:32px 20px 64px }}
h1 {{ margin:0 0 6px }} h2 {{ margin-top:34px }} .muted {{ color:var(--muted) }}
.cards {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(160px,1fr));
  gap:12px; margin:24px 0 }}
.card, section {{ background:var(--panel); border:1px solid #203650;
  border-radius:12px; padding:16px }}
.card span {{ display:block; color:var(--muted); font-size:12px;
  text-transform:uppercase; letter-spacing:.08em }}
.card strong {{ display:block; margin-top:5px; font-size:20px }}
section {{ margin-top:16px }} svg {{ width:100%; height:auto }}
svg text {{ fill:var(--muted); font-size:12px }} .grid {{ stroke:var(--grid) }}
.timeline {{ list-style:none; padding:0; display:grid; gap:10px }}
.timeline li {{ border-left:3px solid var(--accent); padding-left:12px }}
.timeline time {{ display:block; color:var(--muted); font-family:monospace }}
.table-wrap {{ overflow:auto }} table {{ width:100%; border-collapse:collapse }}
th,td {{ padding:7px 9px; border-bottom:1px solid #203650; text-align:right }}
th:nth-child(2),th:nth-child(3),td:nth-child(2),td:nth-child(3) {{
  text-align:left }}
code {{ color:#9ee9d8 }} footer {{ margin-top:28px; color:var(--muted) }}
</style>
</head>
<body><main>
<h1>{html.escape(title)}</h1>
<p class="muted">Synthèse locale limitée aux métadonnées de transport SWUDP.</p>
<div class="cards">{card_html}</div>
<section><h2>Ouverture du lien</h2><ol class="timeline">{timeline}</ol></section>
{_svg_series(samples, "avgTputKbps", title="Débit moyen reçu",
             unit="kbit/s", color="#46d6b6")}
{_svg_series(video_samples, "receive_kib_s", title="Débit vidéo SWUDP reçu",
             unit="Kio/s", color="#7aa7ff")}
{_svg_series(samples, "avgPktLoss", title="Perte moyenne estimée",
             unit="%", color="#ffb45c")}
{_flow_table("émis", summary["flows"]["send"])}
{_flow_table("reçus", summary["flows"]["recv"])}
<footer>Généré par tools/analyze_swudp_log.py — aucune charge applicative,
identité d’appareil ou clé Wi-Fi incluse.</footer>
</main></body></html>
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument(
        "--input-format",
        choices=("auto", "encrypted", "plaintext"),
        default="auto",
    )
    parser.add_argument("--json", dest="json_output", type=Path)
    parser.add_argument("--html", dest="html_output", type=Path)
    parser.add_argument("--title", default="DJI Neo 2 — session SWUDP")
    args = parser.parse_args()

    encrypted = (
        input_is_encrypted(args.input)
        if args.input_format == "auto"
        else args.input_format == "encrypted"
    )
    if encrypted:
        parser.error("encrypted SDK log decoding is not included; supply a decoded plaintext log")
    lines = iter_plaintext_lines(args.input)
    summary = analyze_lines(lines)

    serialized = json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    if args.json_output is not None:
        args.json_output.write_text(serialized, encoding="utf-8")
    if args.html_output is not None:
        args.html_output.write_text(
            render_html(summary, title=args.title), encoding="utf-8"
        )
    if args.json_output is None and args.html_output is None:
        print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
