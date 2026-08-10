"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: main_wiring.py
Brief: p1_motion voice-loop MVP wiring -- CHS-A client to chassis_stub :30004

Description:
Minimum-viable p1 for the voice-loop smoke test:

  * open RT + GEN sessions
  * subscribe cmd/motion/intent from p4
  * subscribe cmd/motion/factor (would come from p2 arbiter; MVP
    just observes)
  * on each cmd/motion/intent, produce a single-frame cmd_vel and
    forward it to chassis_stub :30004 as a CHS-A APDU frame
    (16-byte header + JSON ASDU)

Real 20 Hz ctrl_loop + rns_avoid + speed_gate + rotation_permit
live in xbrain/p1_motion/{ctrl_loop.py,rns/,gate/,rotation/} and
stay untouched by this MVP. The purpose here is: 'when p4 says
向前 3 米, p1 emits a CHS-A frame the chassis_stub prints.'
"""

from __future__ import annotations

import json
import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional


_logger = logging.getLogger("xbrain.p1.wiring")


CMD_MOTION_INTENT_TOPIC = "cmd/motion/intent"
CMD_MOTION_FACTOR_TOPIC = "cmd/motion/factor"


@dataclass
class ChassisClientConfig:
    """All fields required (CLAUDE.md 3.1)."""
    host: str
    port: int
    connect_timeout_s: float
    retry_delay_s: float


class ChassisClient:
    """Simple CHS-A frame sender to chassis_stub. Auto-reconnect
    on send failure (chassis_stub may not be up when p1 starts)."""

    def __init__(self, cfg: ChassisClientConfig) -> None:
        self._cfg = cfg
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self.frames_sent = 0
        self.connect_attempts = 0
        self.connect_failures = 0

    def _connect(self) -> bool:
        try:
            self.connect_attempts += 1
            s = socket.create_connection(
                (self._cfg.host, self._cfg.port),
                timeout=self._cfg.connect_timeout_s)
            self._sock = s
            _logger.info("p1 chassis connected %s:%d",
                         self._cfg.host, self._cfg.port)
            return True
        except OSError as exc:
            self.connect_failures += 1
            _logger.warning("p1 chassis connect fail: %s", exc)
            self._sock = None
            return False

    def send_apdu(self, asdu_json_dict: dict) -> bool:
        """Send one APDU frame. On failure, drop the socket and
        return False; caller retries on next intent."""
        with self._lock:
            if self._sock is None and not self._connect():
                return False
            asdu = json.dumps(asdu_json_dict,
                                ensure_ascii=False).encode("utf-8")
            header = b"\x00" * 12 + struct.pack(">I", len(asdu))
            try:
                self._sock.sendall(header + asdu)
                self.frames_sent += 1
                return True
            except OSError as exc:
                _logger.warning("p1 chassis send fail: %s", exc)
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
                return False

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None


def intent_to_apdu(intent_payload: dict) -> dict:
    """Compose a CHS-A ASDU JSON dict wrapping the intent. Real
    CHS-A ASDU carries {PatrolDevice: {Time, ...}} per 13 §2.2;
    this MVP wraps the p4 intent envelope inside it."""
    intent_id = intent_payload.get("intent_id", "?")
    text = intent_payload.get("text", "")
    return {
        "PatrolDevice": {
            "Time": time.strftime("%Y-%m-%d %H:%M:%S",
                                   time.localtime()),
            "Op": "IntentForward",
            "IntentId": intent_id,
            "Text": text,
        },
    }


def run_voice_loop_wiring(chassis_cfg: ChassisClientConfig,
                            stop_flag: dict,
                            heartbeat_period_s: float = 5.0) -> int:
    """Block until stop_flag truthy. Returns 0 on clean shutdown."""
    from xbrain.common.runtime.session_ctx import open_planes

    _logger.info("p1 wiring: opening RT + GEN sessions")
    client = ChassisClient(chassis_cfg)
    with open_planes(("rt", "gen")) as (rt, gen):
        _rt = rt   # keep RT session alive for future cmd_vel pub

        def _on_intent(sample) -> None:
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                _logger.warning("p1 malformed cmd/motion/intent")
                return
            apdu = intent_to_apdu(d)
            ok = client.send_apdu(apdu)
            _logger.info("p1 forwarded intent -> chassis (ok=%s intent=%s)",
                         ok, d.get("intent_id"))

        intent_sub = gen.declare_subscriber(
            CMD_MOTION_INTENT_TOPIC, _on_intent)

        # Also subscribe cmd/motion/factor (log only for MVP).
        def _on_factor(sample) -> None:
            _logger.debug("p1 obs cmd/motion/factor (%d bytes)",
                          len(bytes(sample.payload)))
        factor_sub = gen.declare_subscriber(
            CMD_MOTION_FACTOR_TOPIC, _on_factor)

        try:
            last_hb = time.monotonic()
            while not stop_flag.get("stop"):
                now = time.monotonic()
                if now - last_hb >= heartbeat_period_s:
                    _logger.info(
                        "p1 alive; chassis_frames=%d "
                        "chassis_reconnects=%d chassis_fails=%d",
                        client.frames_sent, client.connect_attempts,
                        client.connect_failures)
                    last_hb = now
                time.sleep(0.1)
        finally:
            try:
                intent_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            try:
                factor_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            client.close()
    return 0
