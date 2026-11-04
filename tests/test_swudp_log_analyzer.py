from __future__ import annotations

import pytest

from tools.analyze_swudp_log import analyze_lines, render_html


def test_public_analyzer_rejects_encrypted_input_before_writing_reports(tmp_path, monkeypatch):
    import sys
    from tools.analyze_swudp_log import FRAME_MAGIC, main

    source = tmp_path / 'sdk.log'
    source.write_bytes(FRAME_MAGIC + b'fixture')
    output = tmp_path / 'report.json'
    monkeypatch.setattr(sys, 'argv', ['analyzer', str(source), '--json', str(output)])
    with pytest.raises(SystemExit) as result:
        main()
    assert result.value.code == 2
    assert not output.exists()


LOG = """\
2026-07-22 15:24:51.354 INFO  [WifiServicePort] Port:9003
2026-07-22 15:24:52.046 INFO  [SWUDP-I] [Udp_Create] Udp_Create: Local port: 53092
2026-07-22 15:24:52.046 INFO  [SWUDP-I] [SW_Pro_Gnd_Manage_Init] SW_Pro_Gnd_Manage_Init: ReqSessionID: 20224
2026-07-22 15:24:52.046 INFO  [WifiServicePort] SW_Pro_Gnd_Entry_Start success, tSWUdpHandle_ fd = 252, ip_ = 192.168.2.1, port_ = 9003
2026-07-22 15:24:54.580 INFO  [SWUDP-I] [SW_Pro_Gnd_Manage_Send_Req] SW_Pro_Gnd_Manage_Send_Req send 5
2026-07-22 15:24:57.092 INFO  [SWUDP-I] [SW_Pro_Gnd_Manage_Send_Req] SW_Pro_Gnd_Manage_Send_Req send 10
2026-07-22 15:25:19.319 INFO  [SWUDP-I] [SW_Gnd_Monitor_Set_Max_Bitrate] set sw max bitrate = 8000
2026-07-22 15:25:19.319 INFO  [SWUDP-I] set avgTputKbps level th[0]=0
2026-07-22 15:25:19.319 INFO  [SWUDP-I] [SW_Pro_Gnd_Link_Init] SW_Pro_Gnd_Link_Init: Gnd link ReqSessionID: 20224
2026-07-22 15:25:19.320 INFO  [Heartbeat]Begin Send Heartbeat Pack
2026-07-22 15:25:19.327 INFO  [WifiServicePort] wifi datalink connected 1
2026-07-22 15:25:20.100 INFO  [WlmAssistantLogic] send wifi device info [redacted]
2026-07-22 15:25:20.130 INFO  [Heartbeat]Begin Send Heartbeat Pack
2026-07-22 15:25:20.130 INFO  [PeriodOpenMCMonitorLogic] SendUserSettingPack
2026-07-22 15:25:20.143 INFO  [ProductAbstraction] SendRequest static_cap_type=4
2026-07-22 15:25:20.165 INFO  [Module]: loaded - N3uav3sdk17CameraVideoModuleE
2026-07-22 15:25:20.167 INFO  [XRCE_MGR]: subscribe Topic: camcap_video_codec, register_id: 279, host_id: 0,0
2026-07-22 15:25:20.180 INFO  [videocore][Demand-I] ConnectionChanged, datalink_id = wlm_1, device_id = 0, connection = 1
2026-07-22 15:25:20.190 INFO  [videocore][Demand-I][SpecialCommandManager] receiver change to (14, 255)
2026-07-22 15:25:20.200 INFO  [SW]sw_video received:102B/s sent:0B/s
2026-07-22 15:25:20.200 INFO  [videolink] VideoObserver ProcessRecvVideoData recv data len = 102, total len = 102, datalink_id = wlm_1
2026-07-22 15:25:20.200 INFO  [XRCE_DDS_CLIENT]: Send DDS Pack. command->2,Topic: camcap_video_codec-0
2026-07-22 15:25:20.422 INFO  [SWUDP-I] <CONN>[T=link_stat]recvTotal=43 recvDiff=26 recvPktCntFiltered=26 calPktLossRate=0 maxPktCnt=26
2026-07-22 15:25:20.422 INFO  [SWUDP-I] <CONN>[T=link_stat]avgPktLoss=0 avgPktLossQuality=100
2026-07-22 15:25:20.422 INFO  [SWUDP-I] <CONN>[T=link_stat]avgTputKbps=492 avgTputKbpsQuality=10
2026-07-22 15:25:21.646 INFO  [videocore][Demand-I] Request I Frame State =1; Feedback State = 2; stream_index: 1
2026-07-22 15:32:42.779 INFO  [SWUDP-I] Send V1 count: 1
2026-07-22 15:32:42.779 INFO  [SWUDP-I] index  sender  receiver cmdSet cmdId total fail  freq(Hz) last(ms)
2026-07-22 15:32:42.779 INFO  [SWUDP-I]     0  0x0200   0x0905    0x01  0x0a 0207  0000    20.7     0107
2026-07-22 15:32:42.780 INFO  [SWUDP-I] Recv V1 count: 1
2026-07-22 15:32:42.780 INFO  [SWUDP-I] index  sender  receiver cmdSet cmdId total fail  freq(Hz) last(ms)
2026-07-22 15:32:42.780 INFO  [SWUDP-I]     0  0x1204   0x0200    0x23  0xb2 0507  0000    50.7     0118
"""


def test_extracts_transport_timing_link_samples_and_flows() -> None:
    summary = analyze_lines(LOG.splitlines())

    assert summary["transport"] == {
        "service_port": 9003,
        "local_udp_port": 53092,
        "remote_ip": "192.168.2.1",
        "remote_port": 9003,
        "request_session_id": 20224,
    }
    assert summary["management"]["estimated_request_period_ms"] == 502.4
    assert summary["connection"]["negotiation_ms"] == 27281
    assert summary["connection"]["maximum_bitrate_kbps"] == 8000
    assert summary["connection"]["throughput_thresholds_kbps"] == {"0": 0}
    assert summary["video"] == {
        "demand_connection_changed_at": "2026-07-22 15:25:20.180",
        "receiver_changed_at": "2026-07-22 15:25:20.190",
        "first_callback_at": "2026-07-22 15:25:20.200",
        "first_callback_length": 102,
        "first_callback_total": 102,
        "first_iframe_request_at": "2026-07-22 15:25:21.646",
        "first_callback_after_link_ms": 873,
        "receiver_before_first_callback_ms": 10,
        "first_iframe_after_first_callback_ms": 1446,
    }
    assert summary["video_samples"] == [
        {
            "timestamp": "2026-07-22 15:25:20.200",
            "receive_kib_s": 0.1,
            "send_kib_s": 0.0,
        }
    ]
    assert summary["startup"] == {
        "wifi_device_info_sent_at": "2026-07-22 15:25:20.100",
        "heartbeat_sent_at": "2026-07-22 15:25:20.130",
        "monitor_user_setting_sent_at": "2026-07-22 15:25:20.130",
        "product_static_capability_request_at": "2026-07-22 15:25:20.143",
        "camera_video_module_loaded_at": "2026-07-22 15:25:20.165",
        "first_xrce_subscription_at": "2026-07-22 15:25:20.167",
        "first_xrce_request_sent_at": "2026-07-22 15:25:20.200",
        "timing_from_connection_ms": {
            "wifi_device_info_sent_at": 773,
            "heartbeat_sent_at": 803,
            "monitor_user_setting_sent_at": 803,
            "product_static_capability_request_at": 816,
            "camera_video_module_loaded_at": 838,
            "first_xrce_subscription_at": 840,
            "first_xrce_request_sent_at": 873,
        },
        "relative_to_first_video_ms": {
            "wifi_device_info_sent_at": -100,
            "heartbeat_sent_at": -70,
            "monitor_user_setting_sent_at": -70,
            "product_static_capability_request_at": -57,
            "camera_video_module_loaded_at": -35,
            "first_xrce_subscription_at": -33,
            "first_xrce_request_sent_at": 0,
        },
        "pre_video_marked_emissions": [
            {
                "field": "wifi_device_info_sent_at",
                "timestamp": "2026-07-22 15:25:20.100",
                "milliseconds_before_video": 100,
            },
            {
                "field": "heartbeat_sent_at",
                "timestamp": "2026-07-22 15:25:20.130",
                "milliseconds_before_video": 70,
            },
            {
                "field": "monitor_user_setting_sent_at",
                "timestamp": "2026-07-22 15:25:20.130",
                "milliseconds_before_video": 70,
            },
            {
                "field": "product_static_capability_request_at",
                "timestamp": "2026-07-22 15:25:20.143",
                "milliseconds_before_video": 57,
            },
        ],
    }
    assert summary["xrce"] == {
        "registration_count": 1,
        "unique_topic_count": 1,
        "host_ids": [{"type": 0, "index": 0}],
        "first_subscription_after_video_ms": -33,
        "first_request_after_video_ms": 0,
    }
    assert summary["link_samples"] == [
        {
            "timestamp": "2026-07-22 15:25:20.422",
            "recvTotal": 43,
            "recvDiff": 26,
            "recvPktCntFiltered": 26,
            "calPktLossRate": 0,
            "maxPktCnt": 26,
            "avgPktLoss": 0,
            "avgPktLossQuality": 100,
            "avgTputKbps": 492,
            "avgTputKbpsQuality": 10,
        }
    ]
    assert summary["flows"]["send"]["rows"][0]["frequency_hz"] == 20.7
    assert summary["flows"]["recv"]["rows"][0]["command_id"] == "0xb2"


def test_reports_official_post_video_xrce_timing_without_sign_ambiguity() -> None:
    lines = """\
2026-07-22 15:25:19.327 INFO  [WifiServicePort] wifi datalink connected 1
2026-07-22 15:25:19.333 INFO  [WlmAssistantLogic] send wifi device info [redacted]
2026-07-22 15:25:20.018 INFO  [videolink] VideoObserver ProcessRecvVideoData recv data len = 102, total len = 102, datalink_id = wlm_1
2026-07-22 15:25:20.071 INFO  [Heartbeat]Begin Send Heartbeat Pack
2026-07-22 15:25:20.071 INFO  [PeriodOpenMCMonitorLogic] SendUserSettingPack
2026-07-22 15:25:20.077 INFO  [ProductAbstraction] SendRequest static_cap_type=4
2026-07-22 15:25:20.112 INFO  [XRCE_MGR]: subscribe Topic: camcap_photo_storage_format, register_id: 277, host_id: 0,0
2026-07-22 15:25:20.113 INFO  [XRCE_MGR]: subscribe Topic: camcap_video_codec, register_id: 279, host_id: 0,0
2026-07-22 15:25:20.188 INFO  [XRCE_DDS_CLIENT]: Send DDS Pack. command->2,Topic: camcap_video_codec-0
"""

    summary = analyze_lines(lines.splitlines())

    assert summary["video"]["first_callback_after_link_ms"] == 691
    assert summary["startup"]["relative_to_first_video_ms"] == {
        "wifi_device_info_sent_at": -685,
        "heartbeat_sent_at": 53,
        "monitor_user_setting_sent_at": 53,
        "product_static_capability_request_at": 59,
        "first_xrce_subscription_at": 94,
        "first_xrce_request_sent_at": 170,
    }
    assert summary["startup"]["pre_video_marked_emissions"] == [
        {
            "field": "wifi_device_info_sent_at",
            "timestamp": "2026-07-22 15:25:19.333",
            "milliseconds_before_video": 685,
        }
    ]
    assert summary["xrce"] == {
        "registration_count": 2,
        "unique_topic_count": 2,
        "host_ids": [{"type": 0, "index": 0}],
        "first_subscription_after_video_ms": 94,
        "first_request_after_video_ms": 170,
    }


def test_rejects_silent_mixing_of_two_connected_sessions() -> None:
    lines = """\
2026-07-22 15:25:19.327 INFO  [WifiServicePort] wifi datalink connected 1
2026-07-22 15:30:00.000 INFO  [WifiServicePort] wifi datalink connected 1
"""

    with pytest.raises(ValueError, match="one session at a time"):
        analyze_lines(lines.splitlines())


def test_html_is_self_contained_and_escapes_title() -> None:
    report = render_html(analyze_lines(LOG.splitlines()), title="<Neo 2>")

    assert "<svg" in report
    assert "Débit moyen reçu" in report
    assert "Débit vidéo SWUDP reçu" in report
    assert "Premier callback vidéo" in report
    assert "Premier heartbeat V1" in report
    assert "&lt;Neo 2&gt;" in report
    assert "<script" not in report
