"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_deter_driver.py
Brief: D mode on the device (14 S7.3.2, 99 U99 C2) -- /deter arm, strobe off, DT-5 cut-off, exit, re-arm; ArbHub B <-> D handover

Description:
Until 2026-10-28 entering D mode sent nothing to payload-service: no /deter,
no sound, while the 5 s reconcile quietly moved the device into a silent
`deter`. These cases drive DeterDriver over the PRODUCTION p2_core.yaml d_mode
block with a recording device double, and ArbHub.handover over the production
arbiter block:
  * the arm body carries the four 14 S7.3.2 fields from the config;
  * a refused /deter is reported (the caller refuses D, ML-3) and arms nothing;
  * DT-5: speech stops at speech_max_repeats x loop_s with tts_reps 0, the
    siren and red/blue stay, a refusal backs off instead of hammering;
  * exit sends /deter off only when asked (towards B the /mode teardown does it);
  * a re-arm after device drift keeps the current speech state;
  * config refusals: deter_texts_supported true, a missing key;
  * B <-> D swaps the speaker source in one step and a TTS cannot slip in.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

from pathlib import Path  # repo-relative config

import pytest  # raises, marks
import yaml  # read the production config

from xbrain.p2_core.d_mode.deter_driver import CUTOFF_RETRY_MS, DeterDriver, DeterPlan  # under test
from xbrain.p2_core.domains.hub import ArbHub  # the handover half
from xbrain.p2_core.mode.device_sync import SyncResult  # the device verdict shape

pytestmark = pytest.mark.no_device  # pure in-process

_ROOT = Path(__file__).resolve().parents[3]  # repo root
_CFG = yaml.safe_load((_ROOT / "configs" / "p2_core.yaml").read_text(
    encoding="utf-8"))  # production p2_core.yaml


def _get(cfg):  # dotted-key reader like the resolved snapshot's
    """Dotted-path getter over a nested dict; None when absent."""
    def get(key):  # one key
        cur = cfg  # walk from the root
        for part in key.split("."):  # each path segment
            if not isinstance(cur, dict) or part not in cur:  # absent
                return None  # the snapshot answers _MISSING; the wiring maps it to None
            cur = cur[part]  # descend
        return cur  # the leaf
    return get  # the reader


_PLAN = DeterPlan.from_config(_get(_CFG))  # production values


class _Dev:  # DeviceModeSync double
    """Records /deter and /lights bodies; answers from a script (default ok)."""

    def __init__(self, *deter_answers):  # answers for successive /deter calls
        self.calls = []  # (path, body)
        self.answers = list(deter_answers)  # SyncResult per /deter call

    def deter(self, body):  # POST /deter
        self.calls.append(("/deter", body))  # record
        return self.answers.pop(0) if self.answers else SyncResult(ok=True, status=200)  # scripted

    def set_lights(self, body):  # POST /lights
        self.calls.append(("/lights", body))  # record
        return SyncResult(ok=True, status=200)  # lights always take it


_REFUSED = SyncResult(ok=False, status=409, reason="payload_deter")  # not in deter mode


def test_the_arm_body_is_the_four_fields_from_the_config():
    """14 S7.3.2 entry 2, values from p2_core.yaml d_mode. MUTATION: send the
    speech_max_repeats as tts_reps -> the body differs -> this fails."""
    d = _CFG["d_mode"]  # production block
    dev = _Dev()  # recording device
    assert DeterDriver(dev, _PLAN).enter(1000).ok  # armed
    assert dev.calls == [  # exactly entry 2 then entry 3
        ("/deter", {"on": True, "mode": d["redblue_mode"],
                    "siren_level": d["siren_level"], "tts_reps": d["tts_reps"]}),
        ("/lights", {"strobe": False})]  # GL-8a


def test_a_refused_deter_arms_nothing():
    """The caller turns not-ok into a refused switch (ML-3); the driver must
    not believe it is in D. MUTATION: set entered before checking ok ->
    running True -> this fails."""
    dev = _Dev(_REFUSED)  # /deter refused
    drv = DeterDriver(dev, _PLAN)  # fresh
    assert drv.enter(1000).ok is False  # reported
    assert drv.running is False  # nothing armed
    assert dev.calls == [("/deter", _PLAN.arm_body(True))]  # no lights call either


def test_dt5_stops_the_speech_once_at_the_cut_off_and_keeps_the_siren():
    """DT-5: max_repeats x loop_s (~203 s) -> one re-arm with tts_reps 0.
    MUTATION: compare against loop_s alone -> fires at 20 s -> this fails."""
    dev = _Dev()  # recording device
    drv = DeterDriver(dev, _PLAN)  # fresh
    drv.enter(0)  # entered at t=0
    cut = _PLAN.cutoff_ms  # production deadline
    assert cut == round(_CFG["d_mode"]["speech_max_repeats"] * _CFG["d_mode"]["loop_s"] * 1000)  # 10 x 20.3 s
    assert drv.tick(cut - 1) is False and drv.speech_on  # one ms early: still speaking
    assert drv.tick(cut) is True and drv.speech_on is False  # cut now
    assert dev.calls[-2] == ("/deter", dict(_PLAN.arm_body(True), tts_reps=0))  # siren + lights only
    assert dev.calls[-1] == ("/lights", {"strobe": False})  # strobe off again after the restart
    n = len(dev.calls)  # nothing more after that
    assert drv.tick(cut + 60000) is False and len(dev.calls) == n  # fires once


def test_dt5_backs_off_after_a_refusal():
    """A refused cut-off is retried CUTOFF_RETRY_MS later, not every 100 ms
    pass. MUTATION: drop the backoff -> a second call at +100 ms -> fails."""
    dev = _Dev(SyncResult(ok=True, status=200), _REFUSED)  # entry ok, first cut refused
    drv = DeterDriver(dev, _PLAN)  # fresh
    drv.enter(0)  # entered
    cut = _PLAN.cutoff_ms  # deadline
    assert drv.tick(cut) is False and drv.speech_on  # refused: still speaking
    n = len(dev.calls)  # calls so far
    assert drv.tick(cut + 100) is False and len(dev.calls) == n  # backed off
    assert drv.tick(cut + CUTOFF_RETRY_MS) is True  # retried and taken


def test_exit_disarms_only_when_asked():
    """Exit to idle sends /deter off before the /mode switch; towards B the
    func2 teardown already stopped it. MUTATION: ignore disarm -> an off is
    sent towards B (a 409 every D -> B) or none towards idle -> fails."""
    dev = _Dev()  # recording device
    drv = DeterDriver(dev, _PLAN)  # fresh
    drv.enter(0)  # in D
    drv.exit(disarm=False)  # D -> B
    assert dev.calls[-1][0] == "/lights" and drv.running is False  # no off sent
    drv.enter(0)  # in D again
    drv.exit(disarm=True)  # D -> idle
    assert dev.calls[-1] == ("/deter", {"on": False})  # siren stopped
    assert drv.tick(10 ** 9) is False  # DT-5 no longer pending after exit


def test_rearm_keeps_the_current_speech_state():
    """After DT-5 a reconcile re-arm must not restart the speech. MUTATION:
    re-arm with arm_body(True) -> tts_reps 3 again -> this fails."""
    dev = _Dev()  # recording device
    drv = DeterDriver(dev, _PLAN)  # fresh
    drv.enter(0)  # in D
    drv.tick(_PLAN.cutoff_ms)  # speech cut
    assert drv.rearm() is True  # device took it
    assert dev.calls[-2] == ("/deter", _PLAN.arm_body(False))  # still siren-only


def test_rearm_outside_d_sends_nothing():
    """Not in D: a reconcile into deter is not ours to arm."""
    dev = _Dev()  # recording device
    assert DeterDriver(dev, _PLAN).rearm() is False and dev.calls == []  # nothing


def test_config_refuses_texts_and_missing_keys():
    """DT-7: payload /deter has no texts[]; a missing value must stop P2
    (CLAUDE.md 3.6). MUTATION: drop the texts check -> no raise -> fails."""
    cfg = yaml.safe_load(yaml.safe_dump(_CFG))  # a deep copy
    cfg["d_mode"]["deter_texts_supported"] = True  # flipped
    with pytest.raises(ValueError, match="deter_texts_supported"):
        DeterPlan.from_config(_get(cfg))
    cfg = yaml.safe_load(yaml.safe_dump(_CFG))  # fresh copy
    del cfg["d_mode"]["loop_s"]  # one key missing
    with pytest.raises(ValueError, match="d_mode.loop_s"):
        DeterPlan.from_config(_get(cfg))


# -- ArbHub.handover (B <-> D, 14 S5.5.3 rule 2) -------------------------------

def test_handover_swaps_the_speaker_source_in_one_step():
    """B -> D: broadcast_b -> alarm_d. Two calls (release, then request)
    would leave the speaker idle in between. MUTATION: request before the
    release -> alarm_d queues behind broadcast_b -> not granted -> fails."""
    hub = ArbHub(_CFG["arbiter"])  # production arbiter config
    rid_b = hub.new_req_id("speaker", "broadcast_b")  # B's attempt
    hub.request("speaker", "broadcast_b", 1000, req_id=rid_b)  # B holds
    rid_d = hub.new_req_id("speaker", "alarm_d")  # D's attempt
    g = hub.handover("speaker", ("broadcast_b", rid_b), "alarm_d", rid_d, 1100)  # B -> D
    assert getattr(g.result, "value", g.result) == "granted"  # at once
    h = hub.holder("speaker")  # the new holder
    assert (h.source_id, h.req_id) == ("alarm_d", rid_d)  # D's
    rid_b2 = hub.new_req_id("speaker", "broadcast_b")  # back: D -> B
    g = hub.handover("speaker", ("alarm_d", rid_d), "broadcast_b", rid_b2, 1200)  # lower source
    assert getattr(g.result, "value", g.result) == "granted"  # still at once
    assert hub.holder("speaker").source_id == "broadcast_b"  # B's again


def test_handover_of_a_queued_old_request_withdraws_it():
    """The old hold may still be queued (behind a TTS that has not acked):
    the swap must withdraw it, not leave a ghost waiter. MUTATION: drop the
    cancel -> broadcast_b stays queued -> this fails."""
    hub = ArbHub(_CFG["arbiter"])  # production arbiter config
    hub.request("speaker", "tts_local", 1000, atomic=True)  # a TTS plays
    rid_b = hub.new_req_id("speaker", "broadcast_b")  # B arrives
    hub.request("speaker", "broadcast_b", 1000, req_id=rid_b)  # queued behind the TTS
    rid_d = hub.new_req_id("speaker", "alarm_d")  # then D
    hub.handover("speaker", ("broadcast_b", rid_b), "alarm_d", rid_d, 1010)  # B -> D before the ack
    waiting = hub.states(1010)["speaker"]["waiting"]  # the queue
    assert [w["source_id"] for w in waiting] == ["alarm_d"]  # no ghost broadcast_b
