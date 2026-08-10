"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_v2c_wiring.py
Brief: V-2C tests -- p1 chassis client APDU compose + intent -> APDU mapping

Description:
Focused unit tests. Live Zenoh sub/pub for p3/p5/p1 wiring is
exercised by V-3 smoke.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time

import pytest

from xbrain.p1_motion.runtime.main_wiring import (
    CMD_MOTION_FACTOR_TOPIC, CMD_MOTION_INTENT_TOPIC,
    ChassisClient, ChassisClientConfig, intent_to_apdu,
)
from xbrain.p3_task.runtime.main_wiring import (
    CMD_TASK_TOPIC, STATE_TASK_TOPIC,
)
from xbrain.p5_gateway.runtime.main_wiring import (
    CMD_AUDIO_SPEAK_ACK_TOPIC, STATE_LINK_TOPIC,
    STATE_TASK_TOPIC as P5_STATE_TASK,
)


pytestmark = pytest.mark.no_device


# ---- topic constants match 11 §2.2 ----

def test_p1_topics_match_spec():
    assert CMD_MOTION_INTENT_TOPIC == "cmd/motion/intent"
    assert CMD_MOTION_FACTOR_TOPIC == "cmd/motion/factor"


def test_p3_topics_match_spec():
    assert CMD_TASK_TOPIC == "cmd/task"
    assert STATE_TASK_TOPIC == "state/task"


def test_p5_topics_match_spec():
    assert STATE_LINK_TOPIC == "state/link"
    assert CMD_AUDIO_SPEAK_ACK_TOPIC == "cmd/audio/speak/ack"
    assert P5_STATE_TASK == "state/task"


# ---- p1 intent -> APDU wrapping ----

def test_intent_to_apdu_wraps_patrol_device():
    """13 §2.2 requires ASDU root wrapped in {PatrolDevice: {...}}
    with a Time field."""
    apdu = intent_to_apdu({"intent_id": "B01", "text": "巡逻"})
    assert "PatrolDevice" in apdu
    pd = apdu["PatrolDevice"]
    assert "Time" in pd
    assert pd["Op"] == "IntentForward"
    assert pd["IntentId"] == "B01"
    assert pd["Text"] == "巡逻"


def test_intent_to_apdu_missing_id_still_shapes():
    apdu = intent_to_apdu({})
    assert apdu["PatrolDevice"]["IntentId"] == "?"


def test_apdu_time_field_format():
    """13 §2.2 requires Time as 'YYYY-MM-DD HH:MM:SS' local."""
    apdu = intent_to_apdu({"intent_id": "B01", "text": "x"})
    import re
    assert re.match(r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}$",
                       apdu["PatrolDevice"]["Time"])


# ---- p1 chassis client (live socket via chassis_stub-style server) ----

class _MiniStubServer(threading.Thread):
    """Bind + accept + read one APDU header + JSON body, then close.
    Used purely for round-trip verification."""

    def __init__(self, port: int) -> None:
        super().__init__(daemon=True)
        self._port = port
        self._ready = threading.Event()
        self.received_asdu = None
        self.error = None

    def run(self) -> None:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", self._port))
            s.listen(1)
            self._ready.set()
            client, _ = s.accept()
            header = client.recv(16)
            asdu_len = struct.unpack(">I", header[-4:])[0]
            body = b""
            while len(body) < asdu_len:
                chunk = client.recv(asdu_len - len(body))
                if not chunk:
                    break
                body += chunk
            self.received_asdu = json.loads(body.decode("utf-8"))
            client.close()
            s.close()
        except Exception as exc:      # noqa: BLE001
            self.error = exc


def _pick_port() -> int:
    """Bind :0 then close to get a free port."""
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_chassis_client_sends_and_stub_receives():
    port = _pick_port()
    server = _MiniStubServer(port)
    server.start()
    assert server._ready.wait(timeout=2.0)

    cfg = ChassisClientConfig(host="127.0.0.1", port=port,
                                connect_timeout_s=2.0, retry_delay_s=0.1)
    client = ChassisClient(cfg)
    apdu = intent_to_apdu({"intent_id": "B01", "text": "巡逻"})
    ok = client.send_apdu(apdu)
    assert ok
    server.join(timeout=2.0)
    assert server.received_asdu is not None
    assert server.received_asdu["PatrolDevice"]["IntentId"] == "B01"
    assert client.frames_sent == 1
    client.close()


def test_chassis_client_connect_fail_returns_false():
    """No listener at port -> send returns False; counters advance."""
    port = _pick_port()      # unused; no server bound
    cfg = ChassisClientConfig(host="127.0.0.1", port=port,
                                connect_timeout_s=0.2, retry_delay_s=0.1)
    client = ChassisClient(cfg)
    ok = client.send_apdu(intent_to_apdu({"intent_id": "X"}))
    assert ok is False
    assert client.connect_failures >= 1
    assert client.frames_sent == 0
