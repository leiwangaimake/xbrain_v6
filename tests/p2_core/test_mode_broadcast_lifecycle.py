"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mode_broadcast_lifecycle.py
Brief: B-mode enter/exit through ModeFace -- ML-3 rollback, stream_id check, replay, internal exits, v2.0 state/mode fields

Description:
The 2026-10-28 broadcast gap review (99 U97) found the cloud broadcast mode
half-built. This file pins the ModeFace half of the fix; the device-mode half
lives in tests/p2_core/mode/test_device_sync.py.

  A2  a side effect that fails on the way IN refuses the whole switch (14 ML-3):
      the machine rolls back, the ack is rejected, no state/mode goes out, and
      a resend of the same cmd_id replays the REJECTION (not a success).
  A3  an exit_broadcast carrying a stream_id must name the current session
      (v2.0 S2.5); a mismatch is E_NOT_FOUND and the broadcast keeps going.
  B1  state/mode carries v2.0 S4.3 source / entered_ts / exit_reason, and the
      exit_reason says HOW the broadcast ended (requested / manual_cloud /
      timeout / fault / preempted).
  --  a replayed cmd_id must not re-run the entry side effects (before the fix a
      resent `start` allocated a second stream_id).
  --  exit_alarm must not end a broadcast (it used to map straight to IDLE).

Each case names the mutant it kills (CLAUDE.md 3.3); the mutants were run by
hand with __pycache__ cleared before every run (CLAUDE.md 7.2.1).

The on_transition stub mirrors the real p2 callback's contract: it returns the
new stream_id on entry and echoes the ended one on exit, and raises
TransitionAborted when told the device switch failed -- a stub that returned
None would let the very code under test go unexercised (see the note in
test_mode_wiring.py about the 2026-09-03 stream_id leak).
"""

from __future__ import annotations  # postponed annotations

import json  # frames go in as JSON bytes

import pytest  # fixtures, marks, raises

from xbrain.common.errors import E_NOT_FOUND, E_UNHEALTHY  # closed-set codes (11 S13)
from xbrain.p2_core.mode.state_machine import ModeState, TriggerKind  # machine vocabulary
from xbrain.p2_core.runtime import mode_wiring  # for the _wall_now patch
from xbrain.p2_core.runtime.mode_wiring import ModeFace, TransitionAborted  # under test

pytestmark = pytest.mark.no_device  # pure in-process


class _Rig:  # face + recorder + scriptable callback
    """A ModeFace plus a recording publisher and a scriptable callback."""

    def __init__(self) -> None:  # fresh face in IDLE
        self.sent = []            # (key, body) of every state/mode publish
        self.calls = []           # (from, to) of every side-effect call
        self.fail_entry = False   # make the next entry raise TransitionAborted
        self.n = 0                # stream_id allocator
        self.face = ModeFace(publish=self._pub, on_transition=self._on_tr)  # real ModeFace

    def _pub(self, key, data):  # state/mode recorder
        self.sent.append((key, json.loads(data.decode("utf-8"))))  # decoded body

    def _on_tr(self, from_state, to_state):  # mirrors main_wiring's contract
        self.calls.append((from_state, to_state))  # count side-effect runs
        if to_state == ModeState.BROADCAST:  # entering B
            if self.fail_entry:  # scripted device failure
                # What main_wiring raises when POST /mode func2 times out.
                raise TransitionAborted(E_UNHEALTHY, {  # what main_wiring raises
                    "item": "payload", "reason": "service_timeout"})  # T-PAY-1 reason
            self.n += 1  # next stream_id
            self.cur = "audio-gj001-%04d" % self.n  # v2.0 sample shape
            return {"stream_id": self.cur}  # entry: new id / exit: echo it
        if from_state == ModeState.BROADCAST:  # leaving B
            return {"stream_id": self.cur}  # entry: new id / exit: echo it
        return None  # no extra applied fields

    def send(self, **body):  # one cmd/mode frame
        """One cmd/mode frame; returns the ack."""
        return self.face.handle_frame(json.dumps(body).encode("utf-8"),  # bus bytes
                                      now_mono_ms=1000)  # fixed mono time

    def start(self, cmd_id="c-start", source="cloud"):  # AUDIO_CONTROL start shape
        return self.send(cmd_id=cmd_id, action="set_voice_mode",  # enter B
                         voice_mode="broadcast", source=source)  # with its channel

    def last_state(self):  # latest state/mode
        """The most recent state/mode body published."""
        return self.sent[-1][1]  # body of the last publish


@pytest.fixture  # one rig per case
def rig(monkeypatch):  # pins the wall clock
    # Fixed wall clock for entered_ts (the module reads it in one place).
    monkeypatch.setattr(mode_wiring, "_wall_now", lambda: 1785732030.2)  # deterministic entered_ts
    return _Rig()  # fresh state


# -- A2: ML-3 rollback ------------------------------------------------------

def test_failed_device_switch_refuses_the_whole_entry(rig):
    """POST /mode func2 failed -> rejected E_UNHEALTHY, still IDLE, nothing
    published. MUTATION: swallow TransitionAborted and carry on (the pre-fix
    behaviour) -> ack accepted / state BROADCAST -> this fails."""
    rig.fail_entry = True  # the device switch will fail
    ack = rig.start()  # try to enter B
    assert (ack["result"], ack["code"]) == ("rejected", E_UNHEALTHY)  # ML-3 refusal
    assert ack["detail"]["reason"] == "service_timeout"  # why, from the callback
    assert ack["detail"]["to_mode"] == "broadcast"  # what was refused
    assert rig.face.state == ModeState.IDLE  # not in B
    assert rig.sent == []  # no state/mode for a refused switch


def test_resending_a_rolled_back_cmd_id_replays_the_rejection(rig):
    """The idempotency history must hold the REJECTION after a rollback.
    MUTATION: rollback() leaves the committed result in the history -> the
    resend answers "duplicate" (a success) for a switch that never happened."""
    rig.fail_entry = True  # the device switch will fail
    rig.start(cmd_id="c-x")  # fails
    rig.fail_entry = False  # the device works again
    again = rig.start(cmd_id="c-x")  # same cmd_id resent
    assert again["result"] == "rejected"  # replays the refusal
    assert rig.face.state == ModeState.IDLE  # not in B


def test_a_new_cmd_id_after_a_failure_can_enter(rig):
    """A failure is not sticky: the next start (new cmd_id) enters normally."""
    rig.fail_entry = True  # the device switch will fail
    rig.start(cmd_id="c-1")  # first attempt / first session
    rig.fail_entry = False  # the device works again
    ack = rig.start(cmd_id="c-2")  # a new command
    assert ack["result"] == "accepted"  # accepted
    assert rig.face.state == ModeState.BROADCAST  # still / now in B


# -- replay must not re-run side effects ----------------------------------

def test_replayed_start_does_not_open_a_second_session(rig):
    """Party A resends the same start: answer duplicate with the SAME stream_id,
    call the side effects once. MUTATION: changed computed without
    `not result.is_replay` -> a second call allocates audio-gj001-0002."""
    first = rig.start(cmd_id="c-s")  # original start
    dup = rig.start(cmd_id="c-s")  # Party A resends it
    assert dup["result"] == "duplicate"  # flagged as a replay
    assert len(rig.calls) == 1  # side effects ran once
    assert first["detail"]["applied"]["stream_id"] == "audio-gj001-0001"  # the first id
    assert rig.face.stream_id == "audio-gj001-0001"  # no second id allocated


# -- A3: stream_id on exit --------------------------------------------------

def test_exit_with_a_foreign_stream_id_is_refused_and_broadcast_continues(rig):
    """v2.0 S2.5. MUTATION: drop _check_exit -> the stale exit ends the live
    session -> state IDLE -> this fails."""
    rig.start()  # enter B (audio-gj001-0001)
    ack = rig.send(cmd_id="c-e", action="exit_broadcast",  # an exit with a stream_id
                   stream_id="audio-gj001-0099", source="cloud")  # not the live session
    assert (ack["result"], ack["code"]) == ("rejected", E_NOT_FOUND)  # 11 S7.3 v1.2
    assert ack["detail"]["stream_id"] == "audio-gj001-0099"   # echoed
    assert ack["detail"]["reason"] == "stream_id_mismatch"  # which refusal
    assert rig.face.state == ModeState.BROADCAST  # still / now in B


def test_exit_with_the_right_stream_id_ends_it_and_echoes(rig):
    rig.start()  # enter B (audio-gj001-0001)
    ack = rig.send(cmd_id="c-e", action="exit_broadcast",  # an exit with a stream_id
                   stream_id="audio-gj001-0001", source="cloud")  # the live session
    assert ack["result"] == "accepted"  # accepted
    assert ack["detail"]["applied"]["stream_id"] == "audio-gj001-0001"  # echoed
    assert rig.face.state == ModeState.IDLE  # not in B
    st = rig.last_state()  # latest state/mode
    assert st["stream_id"] is None and st["exit_reason"] == "requested"  # AUDIO_CONTROL exit


def test_late_exit_for_the_session_that_already_ended_is_accepted(rig):
    """The broadcast timed out; Qt then sends its exit for that session.
    Idempotent: accepted, changed=false, id echoed. An id that never existed
    is still E_NOT_FOUND. MUTATION: refuse whenever not in BROADCAST -> the
    late exit is rejected and Qt shows a failure for a session that ended."""
    rig.start()  # enter B (audio-gj001-0001)
    rig.face.exit_internal("timeout", now_mono_ms=2000)  # p2's own idle timeout
    late = rig.send(cmd_id="c-late", action="exit_broadcast",  # Qt's late exit
                    stream_id="audio-gj001-0001", source="cloud")  # for the ended session
    assert late["result"] == "accepted"  # idempotent
    assert late["detail"]["applied"]["changed"] is False  # nothing changed
    assert late["detail"]["applied"]["stream_id"] == "audio-gj001-0001"  # echoed
    bogus = rig.send(cmd_id="c-b", action="exit_broadcast",  # an id never issued
                     stream_id="audio-gj001-0042", source="cloud")  # made up
    assert bogus["code"] == E_NOT_FOUND  # no such session


def test_exit_without_stream_id_is_a_forced_exit(rig):
    """HMI W3 / cloud text C04 carry no stream_id: they stop whatever is on."""
    rig.start()  # enter B (audio-gj001-0001)
    ack = rig.send(cmd_id="c-hmi", action="exit_broadcast", source="hmi")  # W3 button
    assert ack["result"] == "accepted"  # accepted
    assert rig.face.state == ModeState.IDLE  # not in B


def test_exit_alarm_does_not_end_a_broadcast(rig):
    """MUTATION: map exit_alarm to IDLE regardless of the current mode (the
    pre-fix table) -> the broadcast ends -> this fails."""
    rig.start()  # enter B (audio-gj001-0001)
    ack = rig.send(cmd_id="c-ea", action="exit_alarm", source="cloud")  # wrong mode's exit
    assert ack["result"] == "accepted"  # accepted
    assert ack["detail"]["applied"]["changed"] is False  # a no-op
    assert rig.face.state == ModeState.BROADCAST  # still / now in B


# -- B1: v2.0 S4.3 source / entered_ts / exit_reason ------------------------

def test_state_mode_carries_source_and_entered_ts(rig):
    """MUTATION: drop source / entered_ts from the body -> KeyError here."""
    rig.start(source="cloud")  # a cloud start
    st = rig.last_state()  # latest state/mode
    assert st["voice_mode"] == "broadcast"  # in B
    assert st["source"] == "cloud"  # v2.0 S4.3 source
    assert st["entered_ts"] == 1785732030.2  # the pinned wall clock
    assert st["exit_reason"] is None  # B has not ended


@pytest.mark.parametrize("exit_body, reason", [  # who exits -> which reason
    ({"stream_id": "audio-gj001-0001", "source": "cloud"}, "requested"),  # AUDIO_CONTROL
    ({"source": "cloud"}, "manual_cloud"),          # C04 typed in the cloud
    ({"source": "wecom"}, "manual_wecom"),  # WeCom
    ({"source": "hmi"}, "requested"),               # W3 force-exit button
])  # end of table
def test_exit_reason_names_who_ended_it(rig, exit_body, reason):
    """MUTATION: a constant exit_reason -> at least one row fails."""
    rig.start()  # enter B (audio-gj001-0001)
    rig.send(cmd_id="c-x", action="exit_broadcast", **exit_body)  # the exit under test
    assert rig.last_state()["exit_reason"] == reason  # the mapped reason


@pytest.mark.parametrize("reason, trigger", [  # p2-initiated exits
    ("timeout", TriggerKind.TIMEOUT), ("fault", TriggerKind.SAFETY)])  # idle timeout / /play lost
def test_internal_exit_reports_timeout_or_fault(rig, reason, trigger):
    """BCT-4 / v2.0 S4.3: p2's own exits are distinguishable from a person's.
    MUTATION: exit_internal synthesises a plain exit_broadcast frame (the
    pre-fix auto-exit) -> exit_reason "requested" -> this fails."""
    rig.start()  # enter B (audio-gj001-0001)
    rig.face.exit_internal(reason, now_mono_ms=2000, trigger=trigger)  # internal exit
    st = rig.last_state()  # latest state/mode
    assert (st["voice_mode"], st["exit_reason"], st["source"]) == (  # all three fields
        "idle", reason, "system")  # system-initiated


def test_being_switched_away_is_preempted(rig):
    """B -> dialog by a mode command that is not an exit -> preempted."""
    rig.start()  # enter B (audio-gj001-0001)
    rig.send(cmd_id="c-d", action="set_voice_mode", voice_mode="dialog_c",  # switched away
             source="cloud")  # by the cloud
    assert rig.last_state()["exit_reason"] == "preempted"  # not requested


def test_reentering_clears_the_previous_exit_reason(rig):
    rig.start(cmd_id="c-1")  # first attempt / first session
    rig.face.exit_internal("timeout", now_mono_ms=2000)  # p2's own idle timeout
    rig.start(cmd_id="c-2")
    assert rig.last_state()["exit_reason"] is None  # cleared on re-entry


def test_internal_exit_rejects_a_reason_outside_the_closed_set(rig):
    """CLAUDE.md 3.5: out-of-set values raise, never pass through."""
    rig.start()  # enter B (audio-gj001-0001)
    with pytest.raises(ValueError):  # closed set
        rig.face.exit_internal("bored", now_mono_ms=2000)  # not a v2.0 reason
