"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: lapi_focus.py
Brief: LAPI focus control for the PTZ visible camera -- refocus (11 S7.4.7 F4) and the F1 / F2 session settings

Description:
What problem this solves
------------------------
Two focus behaviours of the visible camera are reachable ONLY through the
camera's native LAPI (HTTP / JSON, digest auth), never through ONVIF
(11 S7.4.3 dual-stack table, PTZ-C7 "LAPI wins"):
  * F1 ShieldTrigger.MovePTZ = 1 -- do not refocus while the head moves. The
    factory value 0 makes the lens hunt for focus on every pan / tilt, so the
    picture pumps soft-sharp-soft during a keyboard drive (ptzkey2.py note 3,
    C28.4).
  * F2 FocusMode = 2 (auto) -- and it must STAY auto. ONVIF AutoFocusMode=AUTO
    does not map back to LAPI (F3): a camera left in manual focus is blurred
    on every scene at a different distance, silently.
  * F4 refocus (99 U98, 2026-10-28): FocusMode 2 -> 0 -> dwell -> 2 forces one
    fresh focus search; the end state is auto again, so F4 is a pulse inside
    F2, not a second focus mode. This is ptzkey2.py's 'f' key.
Before this module p2 wrote neither F1 nor F2 (Qt contract v0.2 S9), so a
head driven from the cloud would hunt focus while moving.

What it does
------------
LapiFocusClient reads Channels/0/Image/Focus, changes only the two keys the
write guard authorises (domains/lapi_guard.py: FocusMode,
ShieldTrigger.MovePTZ -- any other key raises before anything is sent), and
PUTs the whole object back (LAPI replaces the object; a partial PUT would
reset the fields left out). Three operations:
  session_start  -- F1 on + F2 auto, remembering the original MovePTZ;
  session_end    -- F2 auto (never leave the camera in manual) + MovePTZ back
                    to the remembered value;
  refocus        -- the F4 pulse; returns (ok, focus_mode_restored) for the
                    ack (11 S7.4.4 refocus_failed row). Whatever fails after
                    FocusMode left auto, one more write puts it back BEFORE
                    the answer goes out.

What it does NOT do (boundaries)
--------------------------------
  * No bitrate / I-frame writes, ever (11 S7.4.7 B0): the guard refuses them
    and this module never names them.
  * No ONVIF focus calls (F3 forbids AutoFocusMode).
  * No retries in a loop: every call is bounded by one short timeout so the
    refocus ack meets the Qt contract's 2 s (contract S9 asks ~0.5 s per
    call; ptzkey2.py's 10 s curl timeout must not be copied).
  * No credential handling: host / user / pwd are injected by the wiring
    from the same secrets file the ONVIF driver uses.

Pitfalls
--------
  * The camera answers HTTP 200 even when it refuses a write; success is
    Response.ResponseCode == 0 in the JSON body, so that is what is checked.
  * Digest auth must be the only auth (no Basic header first): the camera
    counts failed logins and locks the account (PTZ report S8).
"""
from __future__ import annotations  # postponed annotations

import json  # LAPI bodies are JSON
import logging  # English log lines only
import time  # time.sleep is the injectable dwell
import urllib.request  # stdlib digest auth, no requests (CLAUDE.md 4.1)
from typing import Any, Callable, Dict, List, Optional, Tuple  # type hints only

from xbrain.p2_core.domains.lapi_guard import check_write_key  # the 2-key LAPI write allow-list (BIZ-P2-10)

_logger = logging.getLogger("xbrain.p2.lapi_focus")  # one logger name per module

#: LAPI resource of the visible channel's focus settings (C28.4; the same
#: path ptzkey2.py used).
FOCUS_PATH = "/LAPI/V1.0/Channels/0/Image/Focus"  # visible channel, focus object (C28.4)
#: F4 intermediate value: manual focus (11 S7.4.7 F4 "2 -> 0 -> 2").
MANUAL_FOCUS_MODE = 0  # manual: the intermediate step of the pulse
#: F4 dwell between the two writes (11 S7.4.4 refocus row "wait 300 ms").
REFOCUS_DWELL_MS = 300  # contract value, not tuned here

#: (method, url, body or None, timeout_s) -> response bytes; raises on any
#: transport error. Injected so the logic is testable without a camera.
HttpCall = Callable[[str, str, Optional[bytes], float], bytes]  # (method, url, body, timeout) -> raw response


class LapiError(RuntimeError):  # raised for every failed call
    """A LAPI call failed: transport error, bad JSON or ResponseCode != 0."""


def urllib_http(host: str, user: str, pwd: str) -> HttpCall:  # built once by the wiring
    """The production transport: urllib with a digest-only auth handler."""
    mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()  # digest needs a password manager
    mgr.add_password(None, "http://%s/" % host, user, pwd)  # realm None: the camera's realm is not known in advance
    opener = urllib.request.build_opener(urllib.request.HTTPDigestAuthHandler(mgr))  # digest only: no Basic attempt that would count as a failed login

    def call(method: str, url: str, body: Optional[bytes], timeout_s: float) -> bytes:  # the HttpCall the client uses
        """One request; the body is JSON when given."""
        req = urllib.request.Request(url, data=body, method=method)  # one request, method explicit (PUT has a body)
        if body is not None:  # only PUT carries a body
            req.add_header("Content-Type", "application/json")  # LAPI expects JSON
        with opener.open(req, timeout=timeout_s) as resp:  # bounded by the per-call timeout
            return resp.read()  # raw bytes; _parse decides success
    return call  # closure over the opener


class LapiFocusClient:  # one client per camera
    """F1 / F2 session settings and the F4 refocus pulse over LAPI."""

    def __init__(self, host: str, *, http: HttpCall, timeout_s: float,  # host from the secrets file
                 focus_mode_auto: int, shield_move_ptz: int,  # F2 auto value and F1 shield value from config
                 sleep: Callable[[float], None] = time.sleep) -> None:  # injectable for tests (no real 300 ms wait)
        """focus_mode_auto / shield_move_ptz / timeout_s come from the
        resolved config (ptz.focus_mode, ptz.shield_move_ptz,
        ptz.lapi_timeout_s); no defaults here (CLAUDE.md 3.1)."""
        if timeout_s <= 0:  # zero / negative = unbounded wait
            raise ValueError("lapi timeout must be > 0, got %r" % timeout_s)  # refused at construction
        self._url = "http://%s%s" % (host, FOCUS_PATH)  # the one resource this client touches
        self._http = http  # transport
        self._timeout = timeout_s  # per-call bound
        self._auto = focus_mode_auto  # F2 value (2)
        self._shield = shield_move_ptz  # F1 value (1)
        self._sleep = sleep  # dwell implementation
        self._orig_shield: Optional[Any] = None   # MovePTZ before session_start

    # -- primitives ----------------------------------------------------------

    def _get(self) -> Dict[str, Any]:  # read the focus object
        """Read the focus object (Response.Data)."""
        return self._parse(self._http("GET", self._url, None, self._timeout))  # GET has no body

    def _put(self, cfg: Dict[str, Any], changed: List[str]) -> None:  # write the whole object back
        """Write the whole object back after the guard approved every key we
        changed (a refused key raises BEFORE anything is sent)."""
        for key in changed:  # every key we changed
            check_write_key(key)  # raises before sending
        body = json.dumps(cfg).encode("utf-8")  # LAPI replaces the object, so send all of it
        self._parse(self._http("PUT", self._url, body, self._timeout))  # ResponseCode checked in _parse

    @staticmethod  # no instance state needed
    def _parse(raw: bytes) -> Dict[str, Any]:  # shared by GET and PUT
        """Response.ResponseCode must be 0 (HTTP 200 alone proves nothing)."""
        try:  # malformed body -> LapiError
            resp = json.loads(raw.decode("utf-8"))["Response"]  # every LAPI answer is wrapped in Response
        except (ValueError, KeyError, TypeError, AttributeError) as exc:  # not JSON / no Response / wrong type
            raise LapiError("bad LAPI response: %s" % exc) from exc  # chained for the log
        if resp.get("ResponseCode") != 0:  # HTTP 200 alone proves nothing
            raise LapiError("LAPI ResponseCode %r" % resp.get("ResponseCode"))  # the camera refused the request
        data = resp.get("Data")  # the object itself
        return data if isinstance(data, dict) else {}  # empty object for PUT answers without Data

    # -- F1 / F2 session -------------------------------------------------------

    def session_start(self) -> bool:  # F1 + F2 at PTZ domain start
        """F1 MovePTZ = shield, F2 FocusMode = auto. Returns False (logged)
        when the camera cannot be reached -- PTZ keeps working without it, the
        picture just hunts focus while moving."""
        try:  # camera may be unreachable
            cfg = self._get()  # current settings
            shield = cfg.setdefault("ShieldTrigger", {})  # create the sub-object if absent
            self._orig_shield = shield.get("MovePTZ")  # remembered for session_end
            cfg["FocusMode"] = self._auto  # F2: auto focus
            shield["MovePTZ"] = self._shield  # F1: no refocus while moving
            self._put(cfg, ["FocusMode", "ShieldTrigger.MovePTZ"])  # both keys are on the allow-list
        except Exception as exc:          # noqa: BLE001 -- degraded, not fatal
            _logger.warning("ptz focus session start failed: %s", exc)  # degraded, PTZ keeps working
            return False  # caller only logs it
        _logger.info("ptz focus session: FocusMode=%s MovePTZ %s->%s",  # what changed, for the field log
                     self._auto, self._orig_shield, self._shield)  # values in force
        return True  # session set

    def session_end(self) -> bool:  # F2 at p2 shutdown
        """F2 at exit: FocusMode stays auto; MovePTZ back to what it was."""
        try:  # best effort at exit
            cfg = self._get()  # current settings
            cfg["FocusMode"] = self._auto        # never leave it in manual
            changed = ["FocusMode"]  # FocusMode is always written back
            if self._orig_shield is not None:  # known only after a successful start
                cfg.setdefault("ShieldTrigger", {})["MovePTZ"] = self._orig_shield  # restore the factory / previous value
                changed.append("ShieldTrigger.MovePTZ")  # second allowed key
            self._put(cfg, changed)  # one PUT for both
        except Exception as exc:          # noqa: BLE001 -- best effort at exit
            _logger.warning("ptz focus session end failed: %s", exc)  # nothing else to do at exit
            return False  # reported to the caller
        return True  # restored

    # -- F4 refocus ------------------------------------------------------------

    def refocus(self) -> Tuple[bool, bool]:  # the F4 pulse
        """FocusMode auto -> manual -> dwell -> auto. Returns
        (ok, focus_mode_restored). restored is True when the camera is known
        to be back in auto -- also when the failure happened before FocusMode
        was ever touched (it never left auto)."""
        try:  # read first: nothing changed yet
            cfg = self._get()  # current settings
        except Exception as exc:          # noqa: BLE001 -- nothing changed yet
            _logger.warning("ptz refocus: read failed: %s", exc)  # camera unreachable before any write
            return False, True  # never left auto -> restored True
        try:  # first write
            cfg["FocusMode"] = MANUAL_FOCUS_MODE  # manual for one dwell
            self._put(cfg, ["FocusMode"])  # allowed key
        except Exception as exc:          # noqa: BLE001 -- maybe half-written
            _logger.warning("ptz refocus: manual write failed: %s", exc)  # may or may not have landed
            return False, self._restore(cfg)  # put auto back before answering
        self._sleep(REFOCUS_DWELL_MS / 1000.0)  # let the lens start a new search
        try:  # second write
            cfg["FocusMode"] = self._auto  # back to auto (F2 end state)
            self._put(cfg, ["FocusMode"])  # allowed key
        except Exception as exc:          # noqa: BLE001 -- camera may sit in manual
            _logger.warning("ptz refocus: auto write failed: %s", exc)  # camera may sit in manual now
            return False, self._restore(cfg)  # one more auto write, then answer
        return True, True  # refocused, auto in force

    def _restore(self, cfg: Dict[str, Any]) -> bool:  # last attempt to reach the F2 end state
        """One more attempt to put FocusMode back to auto (F2 end state)."""
        try:  # may fail again
            cfg["FocusMode"] = self._auto  # auto
            self._put(cfg, ["FocusMode"])  # allowed key
            return True  # auto is back
        except Exception as exc:          # noqa: BLE001 -- reported in the ack
            _logger.error("ptz refocus: FocusMode NOT restored to auto: %s", exc)  # the camera may stay in manual: logged as error
            return False  # ack reports restored=False
