"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: device_sync.py
Brief: BIZ-P2-12 device-layer mode sync -- POST /mode to payload-service, 409 = OK, timeout/5xx = fail

Description:
What problem this solves
------------------------
payload-service runs its OWN mode machine (idle / func1 / func2 / deter) and
gates every audio session on it: WS /play is admitted only in func1 / func2,
/tts only in idle / func1 (services/payload/core/session.py _PLAY_MODES /
_TTS_MODES). 14 S5.7 makes P2 the only party that switches it, and 14 S5.7.1
maps each system mode onto a device mode (device_map.py).

Until 2026-10-28 nothing called device_map: the device stayed in the `idle`
it boots in, so every cloud broadcast's /play was refused with 403 ("play not
allowed in mode idle", measured on .1.26 2026-10-27 12:33 and 12:39 -- two
whole broadcasts, 40 625 accepted chunks, 0 sent). This module is the missing
caller.

What it does
------------
  * apply(target)    one POST /mode {mode: target}, bounded by T-PAY-1
                     (payload_svc.rest_timeout_ms, 14 S2.3 P-3), and a verdict:
                       200            -> ok (the switch, including the teardown
                                         of the old mode, has COMPLETED: R3)
                       409            -> ok, already there (ML-2; re-entry is
                                         a conflict to the service, a success
                                         to us)
                       timeout / URL  -> fail, reason service_timeout (T-PAY-1
                                         row: E_UNHEALTHY reason=service_timeout)
                       5xx / other    -> fail, reason payload_mode
  * reconcile(target)  GET /status, and apply(target) only if the device is
                     somewhere else. This is how MM-1 ("idle hangs func1") is
                     reached at startup, and how a payload-service restart
                     (which boots back into `idle`) or an exit whose POST
                     failed gets repaired without anyone noticing a refusal.

What it does NOT do
-------------------
  * It does not decide the target. The caller maps the system mode through
    device_map.to_device_mode -- one mapping, one place.
  * It does not roll anything back. A failed apply() on ENTRY is the caller's
    signal to refuse the whole switch (ML-3); a failed apply() on EXIT must not
    block the exit (the speaker has to go quiet), so the caller logs it and
    reconcile() repairs it later.
  * It never retries inside apply(): a retry loop here would turn the 300 ms
    T-PAY-1 bound into an unbounded wait on the P2 main loop.

Pitfalls
--------
  * Treating 409 as failure makes every adjacent func1 -> func1 switch and
    every resend after a P2 restart fail (14 ML-2 names exactly this).
  * urllib raises HTTPError for a 409; the status must be read off the
    exception, not taken as "the call failed".
  * The service's GET /status reports `mode`; a None (service down) is
    "unknown", never "already in target".
"""

from __future__ import annotations

import json
import logging
import urllib.error  # HTTPError carries the status of a non-2xx answer
import urllib.request  # stdlib HTTP: CLAUDE.md 4.1 forbids requests here
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Tuple  # type hints only

_logger = logging.getLogger("xbrain.p2.device_sync")  # English-only log text (CLAUDE.md 2.1)

#: The device-layer modes POST /mode accepts (payload Mode enum minus nothing:
#: `idle` is a legal target for process start/stop only, 14 S5.7.1 last row).
DEVICE_MODES = ("idle", "func1", "func2", "deter")  # payload Mode enum values, verbatim

# A transport is (method, url, body or None, timeout_s) -> (status, body dict).
# Injected so the unit tests can drive every status without a payload-service;
# production uses _urllib_transport below.
Transport = Callable[[str, str, Optional[Dict[str, Any]], float],
                     Tuple[int, Dict[str, Any]]]


@dataclass(frozen=True)  # immutable verdict: callers only read it
class SyncResult:
    """The verdict of one apply().

    ok      True when the device is now in the target mode (200 or 409).
    status  the HTTP status, or 0 when no response arrived at all.
    reason  "" on success; "service_timeout" or "payload_mode" on failure --
            the two detail reasons 14 S2.3 assigns to E_UNHEALTHY.
    """
    ok: bool  # device is in the target mode now
    status: int  # HTTP status, 0 = no answer
    reason: str = ""  # empty on success (not a safety parameter)


def _urllib_transport(method: str, url: str, body: Optional[Dict[str, Any]],
                      timeout_s: float) -> Tuple[int, Dict[str, Any]]:
    """One HTTP exchange with the stdlib (CLAUDE.md 4.1: no requests here).

    Returns (status, decoded body). A 4xx / 5xx comes back as a status, not an
    exception, so the caller can apply ML-2 to a 409. Transport failures
    (refused, timeout) raise OSError / urllib.error.URLError to the caller.
    """
    data = None if body is None else json.dumps(body).encode("utf-8")  # GET has no body
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:  # a non-2xx surfaces as HTTPError
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:  # T-PAY-1 bounds the whole exchange
            return resp.status, _decode(resp.read())  # 2xx: status plus body
    except urllib.error.HTTPError as exc:  # 4xx / 5xx land here
        # urllib turns every non-2xx into an exception; the status is on it.
        return exc.code, _decode(exc.read() or b"")  # the status lives on the exception


def _decode(raw: bytes) -> Dict[str, Any]:
    """Best-effort JSON body; an unparsable body is an empty dict, never a
    crash -- the verdict rests on the status code alone."""
    try:  # a non-2xx surfaces as HTTPError
        out = json.loads(raw.decode("utf-8")) if raw else {}  # empty body = empty dict
    except ValueError:  # non-JSON body (HTML error page)
        return {}  # the status code still decides
    return out if isinstance(out, dict) else {}  # a JSON list is not a body we read


class DeviceModeSync:
    """POST /mode and GET /status against payload-service, one call at a time.

    Owned by the P2 main loop (single-threaded use): apply() is called from the
    mode transition callback, reconcile() from the heartbeat block.
    """

    def __init__(self, base_url: str, timeout_s: float,
                 transport: Optional[Transport] = None) -> None:
        """base_url is payload_svc.base_url; timeout_s is T-PAY-1
        (payload_svc.rest_timeout_ms / 1000). Both come from the resolved
        snapshot -- no default here (CLAUDE.md 3.1 spirit: a missing bound
        must fail loud at startup, not become an invented number)."""
        self._base = base_url.rstrip("/")  # so base + "/mode" has one slash
        self._timeout_s = float(timeout_s)  # T-PAY-1 in seconds
        self._transport = transport or _urllib_transport  # tests inject a fake
        # The device mode as last confirmed by a 200/409 or read from /status.
        # None = unknown (never confirmed yet, or the last call failed).
        self.confirmed: Optional[str] = None  # unknown until the first 200/409

    def apply(self, target: str) -> SyncResult:
        """Switch the device to `target` and report whether it is there now."""
        if target not in DEVICE_MODES:  # a system mode passed by mistake
            # A programming error in the caller's mapping, not a device fault.
            raise ValueError("device mode %r not in %s" % (target, DEVICE_MODES))  # loud: caller bug
        try:
            status, body = self._transport(
                "POST", self._base + "/mode", {"mode": target}, self._timeout_s)
        except (OSError, urllib.error.URLError) as exc:  # refused / timed out: no answer
            # No answer inside T-PAY-1 (or nothing listening): the switch did
            # not provably happen, so the device mode is now unknown.
            self.confirmed = None  # the device mode is now unknown
            _logger.warning("payload POST /mode %s: no response (%s)", target, exc)
            return SyncResult(ok=False, status=0, reason="service_timeout")  # 14 S2.3 T-PAY-1 reason
        if status == 200 or status == 409:  # ML-2: 409 = already there
            # 200: switched (old mode torn down, R3). 409: already there (ML-2).
            self.confirmed = target  # remember what we know
            if status == 200:  # a real switch: worth an INFO line
                _logger.info("payload device mode -> %s (was %s)",
                             target, body.get("previous"))
            return SyncResult(ok=True, status=status)  # device in target mode
        self.confirmed = None
        _logger.warning("payload POST /mode %s refused: HTTP %d %s",
                        target, status, body)
        return SyncResult(ok=False, status=status, reason="payload_mode")  # 5xx / 4xx other than 409

    def set_lights(self, body: Dict[str, Any]) -> SyncResult:
        """POST /lights with a PARTIAL body (fields absent = unchanged, U39).

        Used for the mode-driven light levels of 14 S4.3 / GL-8a: entering B
        sends {redblue: N, strobe: false} in ONE request (ML-4: right after
        the /mode teardown; ML-5: after device_mode in switch_order), leaving B
        sends {redblue: 0}. A failure is reported, never raised -- a lamp that
        did not change is an indicator fault, not a reason to refuse or block
        a mode switch (ML-3 is about /mode only).
        """
        try:
            status, resp = self._transport(
                "POST", self._base + "/lights", dict(body), self._timeout_s)
        except (OSError, urllib.error.URLError) as exc:  # refused / timed out: no answer
            _logger.warning("payload POST /lights %s: no response (%s)", body, exc)
            return SyncResult(ok=False, status=0, reason="service_timeout")  # 14 S2.3 T-PAY-1 reason
        if 200 <= status < 300:  # lights accepted
            return SyncResult(ok=True, status=status)  # device in target mode
        _logger.warning("payload POST /lights %s refused: HTTP %d %s",
                        body, status, resp)
        return SyncResult(ok=False, status=status, reason="payload_lights")  # reported, not raised

    def deter(self, body: Dict[str, Any]) -> SyncResult:
        """POST /deter {on, mode, siren_level, tts_reps} (14 S7.3.2 entry 2).

        payload-service answers 409 when the device is not in `deter` (the
        /mode deter of entry 1 did not stick) -- unlike /mode, 409 here is a
        FAILURE: the deterrent is not running. A re-POST with fresh fields
        restarts the loop (services/payload start_deter), which is how DT-5
        drops the speech (tts_reps 0) while the siren and red/blue go on.
        Reported, never raised; the caller decides whether it refuses D.
        """
        try:
            status, resp = self._transport(
                "POST", self._base + "/deter", dict(body), self._timeout_s)
        except (OSError, urllib.error.URLError) as exc:  # refused / timed out: no answer
            _logger.warning("payload POST /deter %s: no response (%s)", body, exc)
            return SyncResult(ok=False, status=0, reason="service_timeout")  # 14 S2.3 T-PAY-1 reason
        if 200 <= status < 300:  # deterrent armed / disarmed
            return SyncResult(ok=True, status=status)  # done
        _logger.warning("payload POST /deter %s refused: HTTP %d %s",
                        body, status, resp)
        return SyncResult(ok=False, status=status, reason="payload_deter")  # 409 = not in deter mode

    def current(self) -> Optional[str]:
        """The device mode GET /status reports, or None when unknown."""
        try:
            status, body = self._transport(
                "GET", self._base + "/status", None, self._timeout_s)
        except (OSError, urllib.error.URLError):
            return None  # unknown, never assumed
        mode = body.get("mode") if status == 200 else None  # only a 200 body is trusted
        return mode if mode in DEVICE_MODES else None  # out-of-set = unknown (CLAUDE.md 3.5)

    def reconcile(self, target: str) -> Optional[SyncResult]:
        """Bring the device to `target` if it is observably elsewhere.

        Returns None when nothing was sent (already there, or the service did
        not answer /status -- an unknown state is retried next heartbeat, not
        written blind), else the apply() result.
        """
        seen = self.current()  # one GET /status, bounded by T-PAY-1
        if seen is None:
            return None  # service silent: retry next heartbeat, never write blind
        if seen == target:
            self.confirmed = seen  # observed in place: nothing to send
            return None  # already there
        _logger.warning("payload device mode drift: device=%s want=%s -> POST /mode",
                        seen, target)  # a drift is worth a WARNING: someone reset it
        return self.apply(target)  # the one corrective POST
