"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptz_wiring.py
Brief: p2_core cmd/ptz subscriber -> ONVIF PTZ driver

Description:
cmd/ptz carries two shapes and this is the one p2-side consumer of both:
  * the voice E-class intents from p4 dispatch (E01 move / E05 stop-track /
    E06 zoom / E07 scan / E08 stop-scan / E09 speed), recognised by intent_id;
  * the 11 S7.4 PtzCommand (action = jog / nudge / stop / refocus ...), which
    p5 builds from the cloud's xbrain/{rid}/cmd/ptz (Qt contract v0.2, 99 U98)
    and HMI may send directly.
Both end in ptz_controller.PtzController, which owns the head: dead-man,
17 Hz budget, interruptible pulses, domain-5 admission, acks on cmd/ptz/ack
and events on event/warn/ptz. GEN plane; the Zenoh callback only enqueues
(CLAUDE.md 4.2) and ONE worker thread drains the queue IN ORDER -- a thread
per message (the 2026-08-11 shape) could run a stop before the jog it was
meant to end, leaving the head moving until the dead-man fired.

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
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Mapping, Optional

from xbrain.p2_core.ptz.lapi_focus import LapiFocusClient, urllib_http
from xbrain.p2_core.ptz.ptz_controller import PtzController, PtzLimits
from xbrain.p2_core.ptz.ptz_driver import PtzDriver, PtzDriverConfig

_logger = logging.getLogger("xbrain.p2.ptz_wiring")

CMD_PTZ_TOPIC = "cmd/ptz"
#: 11 S2.2 cmd/ptz/ack row (published by p2_core to the requester). Relative,
#: in-robot key: p5 translates it for the cloud (Qt contract v0.2 S7).
CMD_PTZ_ACK_TOPIC = "cmd/ptz/ack"
#: 11 PTZ-D1 / S6.2 category "ptz" (p5 maps it to v2.0 "payload").
EVENT_PTZ_TOPIC = "event/warn/ptz"

#: Resolved p2_core keys the PTZ domain needs; every one is required (a
#: missing one refuses startup with its path, like the broadcast keys).
PTZ_CONFIG_KEYS = ("ptz.hold_max_ms", "ptz.min_pulse_ms", "ptz.max_pulse_ms",
                   "ptz.max_cmd_hz", "ptz.lapi_timeout_s", "ptz.focus_mode",
                   "ptz.shield_move_ptz")
#: Scheduler period: dead-man / pulse-leg resolution. 20 ms is far below the
#: smallest hold (200 ms) and the shortest pulse archive (350 ms).
TICK_S = 0.02
#: Commands buffered between the Zenoh callback and the worker. Renewals are
#: 4 Hz, so 64 is many seconds of backlog; on overflow the frame is dropped
#: and logged (the sender's dead-man covers a lost renewal).
QUEUE_DEPTH = 64

#: Queue marker for "run the F1 / F2 focus session now". An object, not a
#: byte string: no frame a sender can produce compares equal to it.
_FOCUS_SESSION_START = object()

_TITLES = {"PTZ_DEADMAN_STOP": "ptz deadman stop",
           "PTZ_PREEMPTED": "ptz control preempted"}


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


def read_ptz_config(get: Callable[[str], Any], missing: Any) -> Dict[str, Any]:
    """The PTZ_CONFIG_KEYS from the resolved snapshot, or ValueError naming
    every missing / unassigned path (CLAUDE.md 3.6: report, never fill in)."""
    bad = [k for k in PTZ_CONFIG_KEYS if get(k) is missing or get(k) is None]
    if bad:
        raise ValueError("p2_core config (11 S7.4.4 / S7.4.7): missing or "
                         "unassigned: %s (re-run the freeze line)" % ", ".join(bad))
    return {k.split(".", 1)[1]: get(k) for k in PTZ_CONFIG_KEYS}


class PtzDomain:
    """The p2 PTZ domain: one ordered worker, one 50 Hz scheduler, one
    controller, one driver session, one LAPI focus client.

    submit() is the only thing the Zenoh callback may call: it enqueues and
    returns. start() launches the threads and the F1 / F2 focus session;
    close() stops the head, restores focus and drops the connection."""

    def __init__(self, cfg: OnvifConfig, ptz_cfg: Mapping[str, Any],
                 priorities: Mapping[str, int], *,
                 publish: Callable[[str, bytes], None],
                 admission: Optional[Any] = None) -> None:
        """ptz_cfg: read_ptz_config() output. priorities: domain-5 source id ->
        priority (resolved arbiter.domains.ptz.sources.*.priority).
        publish(key, bytes): GEN-plane put for cmd/ptz/ack and event/warn/ptz.
        admission: the live domain-5 gate (domains/ptz_admission, 99 U99 C1),
        handed to the controller unchanged."""
        self._driver = PtzDriver(PtzDriverConfig(
            host=cfg.host, user=cfg.user, pwd=cfg.pwd))
        self._focus = LapiFocusClient(
            cfg.host, http=urllib_http(cfg.host, cfg.user, cfg.pwd),
            timeout_s=float(ptz_cfg["lapi_timeout_s"]),
            focus_mode_auto=int(ptz_cfg["focus_mode"]),
            shield_move_ptz=int(ptz_cfg["shield_move_ptz"]))
        self._publish = publish
        self._evt_boot = "%x" % int(time.monotonic() * 1000)
        self._evt_n = 0
        self.controller = PtzController(
            self._driver, self._focus,
            PtzLimits(hold_max_ms=int(ptz_cfg["hold_max_ms"]),
                      min_pulse_ms=int(ptz_cfg["min_pulse_ms"]),
                      max_pulse_ms=int(ptz_cfg["max_pulse_ms"]),
                      max_cmd_hz=int(ptz_cfg["max_cmd_hz"])),
            priorities,
            now_ms=lambda: int(time.monotonic() * 1000),
            publish_ack=self._publish_ack, publish_event=self._publish_event,
            admission=admission)
        self._q: "queue.Queue[Any]" = queue.Queue(maxsize=QUEUE_DEPTH)
        self._stop = threading.Event()
        self._threads: list = []
        self._voice_n = 0
        self.dropped = 0

    # -- lifecycle -----------------------------------------------------------

    def start(self) -> None:
        """Worker + scheduler threads; the F1 / F2 session runs on the worker
        first so a slow camera never delays p2 startup."""
        self._q.put_nowait(_FOCUS_SESSION_START)
        for name, fn in (("p2.ptz_worker", self._work),
                         ("p2.ptz_tick", self._tick_loop)):
            t = threading.Thread(target=fn, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def close(self) -> None:
        """Stop the head, restore focus (F2 auto), close the session."""
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)
        try:
            self.controller.stop_all("shutdown")
        except Exception:      # noqa: BLE001 -- shutting down anyway
            pass
        self._focus.session_end()
        self._driver.close()

    # -- intake --------------------------------------------------------------

    def submit(self, payload_bytes: bytes) -> None:
        """Zenoh callback body: enqueue only (CLAUDE.md 4.2)."""
        try:
            self._q.put_nowait(payload_bytes)
        except queue.Full:
            self.dropped += 1
            _logger.error("cmd/ptz queue full; frame dropped (dropped=%d)",
                          self.dropped)

    def handle_envelope(self, payload_bytes: bytes) -> None:
        """One cmd/ptz frame: a PtzCommand (has 'action') or a voice intent
        envelope (has 'intent_id'). Never raises (a bad frame from an
        unauthenticated sender must not kill the worker)."""
        try:
            env = json.loads(payload_bytes.decode("utf-8"))
        except Exception as exc:      # noqa: BLE001
            _logger.warning("cmd/ptz parse fail: %s", exc)
            return
        if not isinstance(env, dict):
            _logger.warning("cmd/ptz body is not an object; dropped")
            return
        if "action" in env:
            self.controller.handle_command(env)
            return
        intent_id = env.get("intent_id", "")
        # Slot values (direction/amount/zoom_dir/level) are merged into the
        # envelope by the orchestrator (16 S8.0.4); pass the whole env as the
        # slot map (extra keys are ignored by the planner).
        plan = self._driver.plan(intent_id, env)
        self._voice_n += 1
        cmd_id = "voice-%s-%d" % (intent_id, self._voice_n)
        if plan.kind == "legs":
            verdict = self.controller.handle_legs(plan.legs, "voice_local", cmd_id)
            _logger.info("ptz %s %s -> %s %s", intent_id, plan.note,
                         verdict.get("result"), verdict.get("code"))
        elif plan.kind == "stop":
            self.controller.stop_all(cmd_id)
            _logger.info("ptz %s stop", intent_id)

    # -- threads -------------------------------------------------------------

    def _work(self) -> None:
        """Drain the queue in arrival order (stop after jog stays after jog)."""
        while not self._stop.is_set():
            try:
                item = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is _FOCUS_SESSION_START:
                self._focus.session_start()
                continue
            try:
                self.handle_envelope(item)
            except Exception:      # noqa: BLE001 -- keep the worker alive
                _logger.exception("cmd/ptz handler crashed")

    def _tick_loop(self) -> None:
        """50 Hz scheduler: dead-man, pulse legs, merged velocities."""
        while not self._stop.wait(TICK_S):
            try:
                self.controller.tick()
            except Exception:      # noqa: BLE001 -- keep the scheduler alive
                _logger.exception("ptz tick crashed")

    # -- outputs -------------------------------------------------------------

    def _publish_ack(self, body: Dict[str, Any]) -> None:
        """cmd/ptz/ack (11 S7.7 Ack shape)."""
        try:
            self._publish(CMD_PTZ_ACK_TOPIC,
                          json.dumps(body, ensure_ascii=False).encode("utf-8"))
        except Exception:      # noqa: BLE001 -- the head state is unaffected
            _logger.exception("cmd/ptz/ack publish failed")

    def _publish_event(self, code: str, detail: Dict[str, Any]) -> None:
        """event/warn/ptz with the Qt contract S6 code filled in (p5 derives
        a code from the title only when the producer leaves it empty)."""
        self._evt_n += 1
        body = {"eid": "ptz-%s-%d" % (self._evt_boot, self._evt_n),
                "code": code, "title": _TITLES.get(code, code.lower()),
                "detail": detail, "src": "p2_core",
                # WALL-CLOCK-OK(record): 11 S6.2 event display stamp, never an age.
                "ts": time.time()}
        try:
            self._publish(EVENT_PTZ_TOPIC,
                          json.dumps(body, ensure_ascii=False).encode("utf-8"))
        except Exception:      # noqa: BLE001
            _logger.exception("event/warn/ptz publish failed")

    @property
    def calls_made(self) -> int:
        """Camera commands sent so far (observability)."""
        return self.controller.stats["camera_cmds"]


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
