"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptz_wiring.py
Brief: p2_core cmd/ptz subscriber -> ONVIF PTZ driver

Description:
p4 dispatch routes the E-class PTZ intents (E01 move / E05 stop-track / E06
zoom / E07 scan / E08 stop-scan / E09 speed) to cmd/ptz. This is the p2-side
consumer that turns each envelope into an ONVIF ContinuousMove/Stop via the
PtzDriver. GEN plane, same Rust-thread handoff as cmd/payload (the callback
must not block the Zenoh thread, CLAUDE.md 4.2).

Before this, cmd/ptz had NO consumer -- the E-intents published into the
void (2026-08-11 PTZ audit). This closes that gap: the 布控球 now actually
moves on a voice command.

Credentials: the camera host + admin login come from a secrets JSON
(onvif_credentials.json, freeze assertion J validates its 0600 mode). When
the file is absent or malformed, the PTZ path is skipped with a warning
rather than crashing p2 -- a missing camera credential must not take down
the audio/payload loop.
"""
from __future__ import annotations

import json
import logging
import threading
from dataclasses import dataclass
from typing import Optional

from xbrain.p2_core.ptz.ptz_driver import PtzDriver, PtzDriverConfig


_logger = logging.getLogger("xbrain.p2.ptz_wiring")

CMD_PTZ_TOPIC = "cmd/ptz"


@dataclass
class OnvifConfig:
    """Camera endpoint + admin login for the ONVIF PTZ driver."""
    host: str
    user: str
    pwd: str


def load_onvif_config(path: str) -> Optional[OnvifConfig]:
    """Read onvif_credentials.json -> OnvifConfig, or None if absent/bad.

    Returns None (not an exception) when the file is missing or malformed:
    the PTZ device is optional to the voice loop, so a missing credential
    disables PTZ but must not stop audio/payload."""
    try:
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
        return OnvifConfig(host=d["host"], user=d["user"], pwd=d["pwd"])
    except FileNotFoundError:
        _logger.warning("ptz: no onvif credentials at %s; PTZ disabled", path)
        return None
    except (ValueError, KeyError) as exc:
        _logger.warning("ptz: bad onvif credentials at %s (%s); PTZ disabled",
                        path, exc)
        return None


class PtzDomain:
    """Serialises PTZ HTTP/ONVIF calls behind one lock (the ONVIF Session is
    a single keep-alive connection). Any unknown/blocked intent is logged
    and dropped -- never a fake success on the head."""

    def __init__(self, cfg: OnvifConfig) -> None:
        self._driver = PtzDriver(PtzDriverConfig(
            host=cfg.host, user=cfg.user, pwd=cfg.pwd))
        self._lock = threading.Lock()

    def handle_envelope(self, payload_bytes: bytes) -> None:
        """Callback body; runs from a worker thread hand-off, NOT the Rust
        Zenoh thread. Decodes the intent envelope and drives the head."""
        try:
            env = json.loads(payload_bytes.decode("utf-8"))
        except Exception as exc:      # noqa: BLE001
            _logger.warning("cmd/ptz parse fail: %s", exc)
            return
        intent_id = env.get("intent_id", "")
        # Slot values (direction/amount/zoom_dir/level) are merged into the
        # envelope by the orchestrator (16 S8.0.4); pass the whole env as the
        # slot map (extra keys are ignored by the driver).
        with self._lock:
            self._driver.handle(intent_id, env)

    def close(self) -> None:
        with self._lock:
            self._driver.close()

    @property
    def calls_made(self) -> int:
        return self._driver.calls_made
