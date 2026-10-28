"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptz_controller.py
Brief: PtzController -- jog renewal / dead-man / interruptible pulses / stop / budget / admission / refocus

Description:
The controller is the only thing between a cloud keyboard (Qt contract v0.2,
99 U98) and a pan axis with no mechanical limit (11 PTZ-C6), so every rule
of 11 S7.4.4 / S7.4.5 that it implements gets a case here, each naming the
mutant that turns it red (CLAUDE.md 3.3):
  * PTZ-D2 a renewal sends nothing to the camera; PTZ-D1 a missed renewal
    sends exactly one Stop and one PTZ_DEADMAN_STOP event;
  * pulses run in the tick and a stop interrupts them at once (the old
    driver slept through them under the lock);
  * stop is never refused: not for budget, not for a bad axes list, not for
    an unknown source;
  * the 17 Hz budget merges velocity changes last-wins and applies them later
    -- never a queue;
  * domain-5 admission refuses a lower priority and, on a holder change,
    stops the head first and emits PTZ_PREEMPTED (PTZ-D4);
  * refocus runs off the lock, merges a second request, and reports
    focus_mode_restored on failure;
  * deleted fields / actions are refused, never translated.
Time is a fake monotonic clock; camera and focus are recording fakes; the
refocus "thread" runs inline. No device.
"""
from __future__ import annotations  # postponed annotations

import pytest  # test runner

from xbrain.common.errors import (  # closed error set
    E_BUSY,  # admission refusal
    E_CAPABILITY,  # deleted field / action
    E_NOT_IMPLEMENTED,  # action not driven
    E_SCHEMA,  # malformed command
    E_UNHEALTHY,  # camera / LAPI failure
)  # end of error imports
from xbrain.p2_core.ptz.ptz_controller import PtzController, PtzLimits  # units under test

pytestmark = pytest.mark.no_device  # runs without any device


class FakeOps:  # records camera calls
    """Records camera calls; can be told to fail."""

    def __init__(self):  # empty record, healthy camera
        self.calls = []      # ("move", (p, t, z)) / ("stop", (pantilt, zoom))
        self.fail = False    # raise on the next call when True

    def move(self, pan, tilt, zoom):  # same signature as PtzOps.move
        if self.fail:  # simulated camera fault
            raise OSError("camera down")  # what a dead socket raises
        self.calls.append(("move", (pan, tilt, zoom)))  # recorded in call order

    def stop(self, pantilt, zoom):  # same signature as PtzOps.stop
        if self.fail:  # simulated camera fault
            raise OSError("camera down")  # what a dead socket raises
        self.calls.append(("stop", (pantilt, zoom)))  # recorded in call order


class FakeFocus:  # scripted LAPI refocus
    """refocus() result is scripted; counts calls."""

    def __init__(self, result=(True, True)):  # default: refocus succeeds
        self.result = result  # (ok, restored) to return
        self.n = 0  # how many refocus runs

    def refocus(self):  # same signature as FocusOps
        self.n += 1  # count the run
        return self.result  # scripted outcome


class Clock:  # fake monotonic clock
    """Fake monotonic milliseconds."""

    def __init__(self):  # arbitrary non-zero start
        self.t = 10_000  # milliseconds

    def __call__(self):  # now_ms() callable
        return self.t  # current fake time


def make(max_cmd_hz=17, focus=None, spawn=None):  # factory used by every case
    """A controller with recording outputs: c.ops, c.acks, c.events, c.clock."""
    clock = Clock()  # time moves only via advance()
    ops = FakeOps()  # recording camera
    acks, events = [], []  # published acks / events
    c = PtzController(  # controller under test
        ops, focus if focus is not None else FakeFocus(),  # focus fake unless a case brings its own
        PtzLimits(hold_max_ms=2000, min_pulse_ms=100, max_pulse_ms=2800,  # the production limits
                  max_cmd_hz=max_cmd_hz),  # budget varied per case
        {"manual_cloud": 700, "auto_track": 500},  # manual 700 above tracking 500 (resolved config)
        now_ms=clock, publish_ack=acks.append,  # acks collected
        publish_event=lambda code, d: events.append((code, d)),  # events collected as (code, detail)
        spawn=spawn or (lambda fn: fn()))  # refocus runs inline unless a case holds it
    c.ops, c.acks, c.events, c.clock = ops, acks, events, clock  # handles for the asserts
    return c  # ready


def jog(c, cmd_id, pan=0.0, tilt=0.0, zoom=0.0, hold=800, source="cloud"):  # one jog command
    """Send one jog and return its ack."""
    return c.handle_command({"cmd_id": cmd_id, "action": "jog", "source": source,  # the PtzCommand p5 builds
                             "vel": {"pan": pan, "tilt": tilt, "zoom": zoom},  # all three axes explicit
                             "hold_ms": hold})  # dead-man length


def advance(c, ms, step=20):  # scheduler stand-in
    """Move the fake clock forward, ticking like the 50 Hz scheduler."""
    end = c.clock.t + ms  # target time
    while c.clock.t < end:  # tick like the 50 Hz thread
        c.clock.t = min(end, c.clock.t + step)  # never past the target
        c.tick()  # one scheduler step


# -- jog: renewal and dead-man (11 PTZ-D1..D3) --------------------------------

def test_jog_sends_one_move():  # first jog
    """A first jog is exactly one ContinuousMove with all axes."""
    c = make()  # fresh controller, idle head
    a = jog(c, "c-1", pan=-1.0, tilt=1.0)  # diagonal up-left
    assert a["result"] == "accepted" and a["detail"]["renewed"] is False  # accepted, not a renewal
    assert c.ops.calls == [("move", (-1.0, 1.0, 0.0))]  # exactly one ContinuousMove, all axes


def test_renewal_sends_nothing_and_extends_the_deadline():  # PTZ-D2
    """PTZ-D2: same velocity, new cmd_id = renewal: zero camera commands, and
    the head keeps moving past the first deadline. MUTATION: re-send the move
    on renewal -> a second move recorded -> red."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0)  # start
    for i in range(2, 8):            # 6 renewals, 250 ms apart = 1.5 s
        advance(c, 250)  # inside the 800 ms hold
        a = jog(c, "c-%d" % i, pan=1.0)  # new cmd_id each time (PTZ-D3)
        assert a["detail"]["renewed"] is True  # recognised as renewal
    assert c.ops.calls == [("move", (1.0, 0.0, 0.0))]  # still the single first move
    assert c.active and c.events == []  # moving, no dead-man fired


def test_deadman_stops_once_with_event():  # PTZ-D1
    """PTZ-D1: no renewal -> one Stop (all axes) and one PTZ_DEADMAN_STOP.
    MUTATION: drop the dead-man branch -> no stop -> red; fire every tick ->
    two stops -> red."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0, hold=800)  # start with an 800 ms hold
    advance(c, 790)  # just inside the hold
    assert c.ops.calls[-1][0] == "move"          # still inside the hold
    advance(c, 500)  # past it
    stops = [x for x in c.ops.calls if x[0] == "stop"]  # all stops recorded
    assert stops == [("stop", (True, True))]  # exactly one, all axes
    assert [e[0] for e in c.events] == ["PTZ_DEADMAN_STOP"]  # exactly one event
    assert c.events[0][1]["hold_ms"] == 800 and not c.active  # hold reported, head idle


def test_velocity_change_same_source_is_one_move_no_stop():  # same holder changes direction
    """Changing direction from the same holder: one new move, no Stop and no
    preemption event (the holder did not change)."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0)  # right
    jog(c, "c-2", pan=-1.0)  # then left
    assert c.ops.calls == [("move", (1.0, 0.0, 0.0)), ("move", (-1.0, 0.0, 0.0))]  # two moves, no stop between
    assert c.events == []  # no preemption event


# -- admission and handover (11 S7A, PTZ-D4) ----------------------------------

def test_lower_priority_is_refused_with_holder():  # domain 5 refusal
    """auto_track (500) cannot take the head from a cloud jog (700).
    MUTATION: drop the priority comparison -> accepted -> red."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0)  # cloud holds the head
    a = jog(c, "t-1", tilt=0.2, source="auto_track")  # tracker asks
    assert a["result"] == "rejected" and a["code"] == E_BUSY  # refused busy
    assert a["detail"] == {"domain": "ptz", "holder": "manual_cloud",  # arbiter source id, 11 S7A.0
                           "holder_priority": 700, "gen": None}  # no arbiter in this rig -> no gen
    assert len(c.ops.calls) == 1  # nothing sent for the tracker


def test_holder_change_stops_first_and_emits_preempted():  # PTZ-D4
    """PTZ-D4: a different source of equal priority takes over -- Stop BEFORE
    its move, plus PTZ_PREEMPTED. MUTATION: skip the Stop -> the move follows
    the old move directly -> red."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0)  # cloud holds the head
    jog(c, "h-1", tilt=1.0, source="hmi")  # HMI takes it (equal priority)
    assert c.ops.calls == [("move", (1.0, 0.0, 0.0)), ("stop", (True, True)),  # move, stop, then the new move
                           ("move", (0.0, 1.0, 0.0))]  # the HMI velocity
    assert c.events[0][0] == "PTZ_PREEMPTED"  # event emitted
    assert c.events[0][1]["new_holder"] == "hmi"  # names the new holder


# -- pulses run in the tick and are interruptible -------------------------------

def test_nudge_acks_at_start_and_stops_after_pulse():  # nudge timing
    """The ack is out before the pulse ends; the tick stops it after pulse_ms."""
    c = make()  # fresh controller, idle head
    a = c.handle_command({"cmd_id": "c-1", "action": "nudge", "source": "cloud",  # a one-second zoom pulse
                          "vel": {"zoom": 1.0}, "pulse_ms": 1000})  # zoom in
    assert a["result"] == "accepted" and a["detail"]["pulse_ms"] == 1000  # acked at the start
    assert c.ops.calls == [("move", (0.0, 0.0, 1.0))]  # pulse started
    advance(c, 980)  # almost the full pulse
    assert c.ops.calls[-1][0] == "move"  # still moving
    advance(c, 60)  # past the end
    assert c.ops.calls[-1] == ("stop", (True, True)) and not c.active  # stopped, idle


def test_stop_interrupts_a_running_pulse_at_once():  # interruptible pulse
    """A stop during a 2.8 s pulse is executed immediately -- the defect of
    the old sleeping driver. MUTATION: queue the stop behind the pulse ->
    no stop until the pulse ends -> red."""
    c = make()  # fresh controller, idle head
    c.handle_legs([((0.6, 0.0, 0.0), 2800)], "voice_local", "voice-E01-1")  # a long voice pulse
    advance(c, 100)  # early in it
    c.handle_command({"cmd_id": "c-9", "action": "stop", "source": "cloud"})  # cloud stop
    assert c.ops.calls[-1] == ("stop", (True, True)) and not c.active  # stopped at once
    advance(c, 3000)  # the rest of the old pulse window
    assert [x[0] for x in c.ops.calls] == ["move", "stop"]   # nothing resumed


def test_voice_sweep_legs_run_in_order():  # voice sweep
    """Three legs: right, left, right, then a Stop."""
    c = make()  # fresh controller, idle head
    c.handle_legs([((0.6, 0, 0), 1500), ((-0.6, 0, 0), 3000), ((0.6, 0, 0), 1500)],  # right, left (double), right
                  "voice_local", "voice-E07-1")  # voice source
    advance(c, 6100)  # all three legs
    moves = [x[1][0] for x in c.ops.calls if x[0] == "move"]  # pan of each move
    assert moves == [0.6, -0.6, 0.6] and c.ops.calls[-1][0] == "stop"  # order kept, final stop


# -- stop is never refused ----------------------------------------------------

def test_stop_without_motion_still_stops_with_note():  # stop on an idle head
    """Stop is always sent; with nothing moving the ack says so."""
    c = make()  # fresh controller, idle head
    a = c.handle_command({"cmd_id": "c-1", "action": "stop"})  # stop, no motion
    assert a["result"] == "accepted" and a["detail"]["note"] == "no_active_motion"  # accepted with the contract note
    assert c.ops.calls == [("stop", (True, True))]  # the stop still went out


def test_stop_with_bad_axes_is_refused_but_still_stops():  # bad axes
    """MUTATION: refuse before stopping -> no stop recorded -> red."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0)  # head moving
    a = c.handle_command({"cmd_id": "c-2", "action": "stop", "axes": ["roll"]})  # unknown axis
    assert a["result"] == "rejected" and a["code"] == E_SCHEMA  # refused
    assert c.ops.calls[-1] == ("stop", (True, True)) and not c.active  # but stopped first


def test_stop_from_unknown_source_still_stops():  # unknown source
    """The source check must not stand in front of a stop."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0)  # head moving
    a = c.handle_command({"cmd_id": "x", "action": "stop", "source": "martian"})  # sender outside the closed set
    assert a["result"] == "accepted" and c.ops.calls[-1][0] == "stop"  # stopped anyway


def test_zoom_only_stop_keeps_pan_moving():  # partial stop
    """Partial stop: zoom stopped, pan keeps its velocity without a resend.
    MUTATION: clear the whole motion on any stop -> not active -> red."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0, zoom=1.0)  # pan and zoom
    c.handle_command({"cmd_id": "c-2", "action": "stop", "axes": ["zoom"]})  # stop zoom only
    assert c.ops.calls[-1] == ("stop", (False, True))  # only the Zoom flag
    assert c.active  # pan still under the dead-man
    advance(c, 2000)                              # dead-man still guards pan
    assert c.events[-1][0] == "PTZ_DEADMAN_STOP"  # the dead-man ends it later


def test_pan_stop_of_a_diagonal_reissues_tilt():  # diagonal partial stop
    """ONVIF stops pan AND tilt together (one PanTilt flag), so stopping only
    pan of a diagonal must re-send the tilt that should keep going.
    MUTATION: skip the re-issue -> tilt silently stops too -> red."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0, tilt=1.0)  # up-right diagonal
    c.handle_command({"cmd_id": "c-2", "action": "stop", "axes": ["pan"]})  # stop pan only
    assert c.ops.calls[-2:] == [("stop", (True, False)), ("move", (0.0, 1.0, 0.0))]  # PanTilt stop, tilt re-sent
    assert c.active  # still moving


def test_pan_stop_leaves_zoom_running_without_resend():  # zoom unaffected by a pan stop
    """Zoom is a separate ONVIF flag: stopping pan does not touch it, so no
    re-send is needed (one fewer command against the 17 Hz budget)."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0, zoom=0.5)  # pan and zoom
    c.handle_command({"cmd_id": "c-2", "action": "stop", "axes": ["pan"]})  # stop pan only
    assert c.ops.calls[-1] == ("stop", (True, False)) and c.active  # no re-send needed


# -- 17 Hz budget: merge last-wins, never queue --------------------------------

def test_budget_merges_last_wins_and_tick_applies_it():  # 17 Hz budget
    """With a 2/s budget the 3rd and 4th changes are merged; only the newest
    is sent once the window frees. MUTATION: queue every change -> both
    merged velocities sent -> red; send over budget -> 3 moves at once -> red."""
    c = make(max_cmd_hz=2)  # budget of 2 per second
    # hold 2000 ms so the dead-man does not end the jog before the 1 s
    # budget window frees (with 800 it would, correctly, stop the head first).
    jog(c, "c-1", pan=1.0, hold=2000)  # 1st command fits
    jog(c, "c-2", pan=-1.0, hold=2000)  # 2nd command fits
    a3 = jog(c, "c-3", tilt=1.0, hold=2000)  # 3rd merged
    a4 = jog(c, "c-4", tilt=-1.0, hold=2000)  # 4th replaces the 3rd
    assert a3["detail"].get("coalesced") and a4["detail"].get("coalesced")  # both told they were merged
    assert len(c.ops.calls) == 2  # only two reached the camera
    advance(c, 1000)  # the window frees
    assert c.ops.calls[-1] == ("move", (0.0, -1.0, 0.0))  # the newest merge was applied
    assert len([x for x in c.ops.calls if x[0] == "move"]) == 3  # the older merge was never sent


def test_renewal_of_a_merged_velocity_is_a_renewal():  # renewal against a pending merge
    """A renewal is compared with the REQUESTED (merged) velocity, so it does
    not resend while the merge is pending."""
    c = make(max_cmd_hz=1)  # budget of 1
    jog(c, "c-1", pan=1.0)  # fits
    jog(c, "c-2", pan=-1.0)                       # merged
    a = jog(c, "c-3", pan=-1.0)  # same as the merged request
    assert a["detail"]["renewed"] is True  # renewal, no resend


def test_stop_is_not_held_back_by_the_budget():  # stop vs budget
    """MUTATION: budget-gate the stop -> no stop -> red."""
    c = make(max_cmd_hz=1)  # budget of 1
    jog(c, "c-1", pan=1.0)  # uses the budget
    c.handle_command({"cmd_id": "c-2", "action": "stop"})  # stop over budget
    assert c.ops.calls[-1][0] == "stop"  # still sent


# -- refused / clamped shapes ----------------------------------------------------

@pytest.mark.parametrize("field", ["pan_deg", "tilt_deg", "d_pan_deg", "zoom"])  # one case per deleted field
def test_deleted_fields_are_refused_not_translated(field):  # 11 S7.4.4 deleted table
    """11 S7.4.4 deleted fields -> E_CAPABILITY, nothing sent."""
    c = make()  # fresh controller, idle head
    a = c.handle_command({"cmd_id": "c-1", "action": "jog", "source": "cloud",  # otherwise a valid jog
                          "vel": {"pan": 1.0}, "hold_ms": 800, field: 10})  # plus the deleted field
    assert a["code"] == E_CAPABILITY and c.ops.calls == []  # refused, nothing sent


def test_deleted_and_unimplemented_actions():  # deleted / not-driven actions
    """move_abs -> E_CAPABILITY with hint; goto_preset -> E_NOT_IMPLEMENTED."""
    c = make()  # fresh controller, idle head
    a = c.handle_command({"cmd_id": "c-1", "action": "move_abs", "source": "cloud"})  # deleted action
    assert a["code"] == E_CAPABILITY and a["detail"]["hint"]  # refused with a hint
    a = c.handle_command({"cmd_id": "c-2", "action": "goto_preset", "source": "cloud"})  # exists but not driven
    assert a["code"] == E_NOT_IMPLEMENTED and c.ops.calls == []  # refused, nothing sent


def test_jog_shape_errors():  # schema errors
    """No hold_ms (no dead-man), all-zero vel, NaN, unknown source -> E_SCHEMA."""
    c = make()  # fresh controller, idle head
    bad = [{"vel": {"pan": 1.0}},  # no hold_ms
           {"vel": {"pan": 0.0}, "hold_ms": 800},  # all axes zero
           {"vel": {"pan": float("nan")}, "hold_ms": 800},  # NaN velocity
           {"vel": {"pan": 1.0}, "hold_ms": 800, "source": "martian"}]  # unknown source
    for extra in bad:  # each case alone
        cmd = {"cmd_id": "c", "action": "jog", "source": "cloud"}  # valid base
        cmd.update(extra)  # made invalid
        assert c.handle_command(cmd)["code"] == E_SCHEMA  # E_SCHEMA every time
    assert c.ops.calls == []  # nothing sent


def test_clamps_are_reported():  # clamp reporting
    """vel and hold_ms are clamped and named in detail.clamped."""
    c = make()  # fresh controller, idle head
    a = jog(c, "c-1", pan=3.0, hold=50)  # pan above 1, hold below 200
    assert set(a["detail"]["clamped"]) == {"pan", "hold_ms"}  # both named
    assert a["detail"]["hold_ms"] == 200 and c.ops.calls == [("move", (1.0, 0.0, 0.0))]  # clamped values in force


def test_limits_refuse_a_budget_above_17():  # budget ceiling
    """11 S7.4.5: max_cmd_hz must never be configured above 17."""
    with pytest.raises(ValueError):  # construction must fail
        PtzLimits(hold_max_ms=2000, min_pulse_ms=100, max_pulse_ms=2800, max_cmd_hz=18)  # 18 is above the camera ceiling


# -- refocus (11 S7.4.4 / S7.4.7 F4) ---------------------------------------------

def test_refocus_acks_after_it_ran():  # async refocus
    """Async path: handle_command returns None, the ack comes from the worker
    after FocusOps.refocus() ran."""
    focus = FakeFocus()  # succeeds
    c = make(focus=focus)  # controller with that focus
    assert c.handle_command({"cmd_id": "c-1", "action": "refocus", "source": "cloud"}) is None  # no sync ack for a started refocus
    assert focus.n == 1 and c.acks[-1]["result"] == "accepted"  # ran once, acked accepted
    assert c.acks[-1]["detail"]["action"] == "refocus"  # ack names the action


def test_refocus_failure_reports_restored_flag():  # failed refocus
    """MUTATION: report success on failure -> accepted -> red."""
    c = make(focus=FakeFocus(result=(False, True)))  # fails, auto restored
    c.handle_command({"cmd_id": "c-1", "action": "refocus", "source": "cloud"})  # start it
    a = c.acks[-1]  # the worker's ack
    assert a["code"] == E_UNHEALTHY and a["detail"]["reason"] == "refocus_failed"  # E_UNHEALTHY with the contract reason
    assert a["detail"]["focus_mode_restored"] is True  # restored flag passed through


def test_second_refocus_while_running_is_coalesced():  # coalescing
    """MUTATION: run a second refocus -> focus.n == 2 -> red."""
    focus = FakeFocus()  # succeeds when run
    held = []  # worker start captured, not run
    c = make(focus=focus, spawn=held.append)      # the worker has not run yet
    c.handle_command({"cmd_id": "c-1", "action": "refocus", "source": "cloud"})  # first refocus
    a2 = c.handle_command({"cmd_id": "c-2", "action": "refocus", "source": "cloud"})  # second while busy
    assert a2["result"] == "accepted" and a2["detail"]["coalesced"] is True  # merged, answered at once
    held[0]()  # now let the first one run
    assert focus.n == 1  # only one refocus happened


def test_refocus_does_not_touch_motion():  # refocus vs motion
    """A refocus during a jog neither stops nor resends it."""
    c = make()  # fresh controller, idle head
    jog(c, "c-1", pan=1.0)  # head moving
    c.handle_command({"cmd_id": "c-2", "action": "refocus", "source": "cloud"})  # refocus meanwhile
    assert c.ops.calls == [("move", (1.0, 0.0, 0.0))] and c.active  # no extra camera call, still moving


# -- camera faults ----------------------------------------------------------------

def test_camera_error_answers_unhealthy_and_clears_motion():  # camera fault
    """MUTATION: swallow the error and answer accepted -> red."""
    c = make()  # fresh controller, idle head
    c.ops.fail = True  # camera raises
    a = jog(c, "c-1", pan=1.0)  # a jog
    assert a["code"] == E_UNHEALTHY and a["detail"]["item"] == "ptz"  # E_UNHEALTHY item ptz
    assert not c.active  # motion record cleared
