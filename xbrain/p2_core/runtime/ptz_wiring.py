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
from typing import Callable, Optional

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


# --- SW-12 ptz liveness probe (non-blocking) --------------------------------
#
# Three-state verdict, NOT a bool. The distinction is a hard LOCKOUT-SAFETY
# requirement, not a nicety: the PTZ report S8 records that BOTH cameras lock the
# account after consecutive auth failures ("严禁撞密码; 探针内已硬编码认证一失败
# 立即中止"). So the probe must tell apart:
#   "up"   -- got a non-fault ONVIF response (auth accepted, camera reachable)
#   "down" -- TRANSPORT failure: the HTTP call never completed, so no auth reached
#             the camera. Safe to keep polling; this is the device_offline signal.
#   "auth" -- a SOAP Fault came back (e.g. ter:NotAuthorized, PTZ report S4.3). Auth
#             DID reach the camera and was rejected -> a misconfiguration, and each
#             retry is one more strike toward lockout. The probe STOPS on the first
#             one; it does NOT treat this as device_offline (a config fault is not a
#             down camera).
# Note OnvifSession.call raises OnvifError only on transport failure; on a SOAP
# Fault it RETURNS the fault xml, so both branches must be inspected explicitly.

VERDICT_UP = "up"
VERDICT_DOWN = "down"
VERDICT_AUTH = "auth"


def make_onvif_reachability_check(cfg: "OnvifConfig",
                                  media_path: str = "/onvif/media",
                                  timeout_s: float = 3.0) -> Callable[[], str]:
    """Return a check() -> VERDICT_UP / VERDICT_DOWN / VERDICT_AUTH that does one
    GetProfiles on a DEDICATED session (OnvifSession is not thread-safe -- the probe
    must NOT share the command path's session). A persistent session keeps the TCP
    connection warm between polls."""
    import xbrain.p2_core.ptz.onvif_client as oc

    sess = oc.OnvifSession(cfg.host, cfg.user, cfg.pwd, timeout=timeout_s)
    body = '<GetProfiles xmlns="http://www.onvif.org/ver10/media/wsdl"/>'

    def _check() -> str:
        try:
            xml = sess.call(media_path, body)
        except oc.OnvifError:
            # Transport failure: auth never reached the camera -> safe to retry.
            return VERDICT_DOWN
        if oc.soap_fault(xml) is not None:
            # Auth reached the camera and was rejected -> lockout risk, stop.
            return VERDICT_AUTH
        return VERDICT_UP

    return _check


class PtzLivenessProbe:
    """Poll a three-state reachability check on a background thread and expose the
    latest verdict as .reachable (Optional[bool]), so the p2 heartbeat can feed the
    device bridge WITHOUT blocking on an ONVIF round-trip. The check is injected, so
    the probe logic is testable with no camera.

    .reachable is None until the first poll completes (unknown -> the bridge emits
    nothing), then True on VERDICT_UP and False on VERDICT_DOWN. A configured-but-
    unreachable camera settles to False and the bridge (with its debounce) emits one
    ptz device_offline -- correct: the camera the operator provisioned is down.

    VERDICT_AUTH is special: the loop STOPS and .reachable is set back to None. An
    auth reject is an operator misconfiguration, not a down camera, so it must NOT
    become a device_offline; and it must not be retried, or the camera locks the
    account (PTZ report S8). A p2 restart re-arms the probe once the config is fixed.
    """

    def __init__(self, check_reachable: Callable[[], str],
                 period_s: float = 5.0) -> None:
        self._check = check_reachable
        self._period = period_s
        self._reachable: Optional[bool] = None
        self._auth_blocked = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def reachable(self) -> Optional[bool]:
        return self._reachable

    @property
    def auth_blocked(self) -> bool:
        return self._auth_blocked

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name="p2.ptz-probe", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                verdict = self._check()
            except Exception:      # noqa: BLE001 -- unexpected error = transport down
                verdict = VERDICT_DOWN
            if verdict == VERDICT_UP:
                self._reachable = True
            elif verdict == VERDICT_AUTH:
                # LOCKOUT SAFETY (PTZ report S8): stop on the first auth reject.
                self._auth_blocked = True
                self._reachable = None   # config fault, not a device_offline
                _logger.error(
                    "ptz probe: ONVIF auth rejected; stopping probe to avoid "
                    "account lockout -- fix onvif_credentials.json then restart p2")
                return
            else:
                self._reachable = False   # VERDICT_DOWN (or defensive fallback)
            # Interruptible sleep so stop() returns promptly.
            self._stop.wait(self._period)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
