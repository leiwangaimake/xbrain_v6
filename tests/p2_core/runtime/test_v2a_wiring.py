"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_v2a_wiring.py
Brief: V-2A tests -- audio frame codec + speaker gate + speak parser + chassis_stub log

Description:
Pure-function coverage for V-2A wiring. Live-hardware paths
(arecord, real Zenoh, real HTTP TTS) are exercised by the V-3
integration smoke test, not here.
"""

from __future__ import annotations

import json
import struct
import subprocess
import sys
import time
from pathlib import Path

import pytest

from xbrain.p2_core.audio.audio_io import (
    ASR_RATE_HZ, ASR_SAMPLES_PER_FRAME, AudioFrame,
)
from xbrain.p2_core.runtime.mic_capture import (
    DEFAULT_MIC_TOPIC, MicCaptureConfig, MicCaptureError,
    decode_frame, default_config, encode_frame,
)
from xbrain.p2_core.runtime.speaker_wiring import (
    GATE_TOPIC, SPEAK_ACK_TOPIC, SPEAK_TOPIC,
    GatePayload, SpeakerHwError, parse_speak_payload,
)


pytestmark = pytest.mark.no_device


# -- MIC capture frame codec --

def _sample_frame():
    """Deterministic AudioFrame for round-trip."""
    return AudioFrame(
        rate_hz=ASR_RATE_HZ, channels=1, sample_width=2,
        frame_ms=20,
        samples=list(range(ASR_SAMPLES_PER_FRAME)))


def test_frame_codec_roundtrip():
    f = _sample_frame()
    round = decode_frame(encode_frame(f))
    assert round.rate_hz == f.rate_hz
    assert round.samples == f.samples


def test_frame_codec_wrong_schema_raises():
    bad = json.dumps({"schema": "audio_frame_v0"}).encode()
    with pytest.raises(MicCaptureError, match="unknown audio_frame schema"):
        decode_frame(bad)


def test_default_config_uses_expected_topic():
    cfg = default_config()
    assert cfg.zenoh_topic == DEFAULT_MIC_TOPIC
    assert cfg.arecord_device.startswith("hw:")
    assert cfg.max_queue_frames > 0


def test_mic_config_all_fields_required():
    """MicCaptureConfig has no defaults; caller passes explicit args."""
    cfg = MicCaptureConfig(
        arecord_device="hw:1,0",
        zenoh_topic="rt/audio/mic",
        max_queue_frames=4)
    assert cfg.arecord_device == "hw:1,0"


# -- Speaker gate payload --

def test_gate_payload_bytes_shape():
    p = GatePayload(open=False, reason="tts_playback", mono_ms=1234)
    d = json.loads(p.to_bytes().decode())
    assert d == {"open": False, "reason": "tts_playback", "mono_ms": 1234}


def test_gate_payload_open_true_serialised():
    p = GatePayload(open=True, reason="idle", mono_ms=0)
    d = json.loads(p.to_bytes().decode())
    assert d["open"] is True


# -- cmd/audio/speak parser --

def test_parse_speak_payload_ok():
    raw = json.dumps({"text": "巡逻已启动"}).encode()
    assert parse_speak_payload(raw) == "巡逻已启动"


def test_parse_speak_payload_empty_text_raises():
    raw = json.dumps({"text": "  "}).encode()
    with pytest.raises(SpeakerHwError, match="no text field"):
        parse_speak_payload(raw)


def test_parse_speak_payload_missing_key_raises():
    raw = json.dumps({"other": "field"}).encode()
    with pytest.raises(SpeakerHwError, match="no text field"):
        parse_speak_payload(raw)


def test_topic_constants_match_spec():
    """Named constants MUST match the 11 §2.2 keys."""
    assert SPEAK_TOPIC == "cmd/audio/speak"
    assert SPEAK_ACK_TOPIC == "cmd/audio/speak/ack"
    assert GATE_TOPIC == "rt/audio/gate"


# -- chassis_stub subprocess (starts + accepts + logs) --

_CHASSIS_STUB = Path(__file__).resolve().parents[3] / "scripts" / "dev" / "chassis_stub.py"


def test_chassis_stub_reads_apdu_and_logs(tmp_path):
    """Spawn chassis_stub on a random high port; connect; send a
    16-byte header (last 4 bytes = ASDU len) + ASDU JSON; verify
    the stub prints one summary line to stdout."""
    port = 41531
    proc = subprocess.Popen(
        [sys.executable, str(_CHASSIS_STUB),
         "--port", str(port),
         "--host", "127.0.0.1"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        # Wait until 'listening' appears.
        deadline = time.monotonic() + 3.0
        assert proc.stdout is not None
        line = ""
        while time.monotonic() < deadline:
            line = proc.stdout.readline()
            if "listening" in line:
                break
        assert "listening" in line, "chassis_stub did not print listening"

        import socket
        cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cli.connect(("127.0.0.1", port))

        asdu = json.dumps({
            "PatrolDevice": {"Time": "2026-08-10 18:00:00",
                              "Op": "test"},
        }, ensure_ascii=False).encode("utf-8")
        header = b"\x00" * 12 + struct.pack(">I", len(asdu))
        cli.sendall(header + asdu)
        # Give the stub a moment to process.
        time.sleep(0.25)
        cli.close()

        # Read available lines.
        seen_apdu_line = False
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            ln = proc.stdout.readline()
            if not ln:
                break
            if "APDU" in ln and "PatrolDevice" in ln:
                seen_apdu_line = True
                break
        assert seen_apdu_line, "chassis_stub did not log the APDU"
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
