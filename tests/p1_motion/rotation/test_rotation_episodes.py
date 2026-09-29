"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_rotation_episodes.py
Brief: 12 S6A.8 OB-2 rotation events -- rate, edges, evidence, and the publisher

Description:
12 S15 #54 recorded a gap with a very specific shape: the rotation permit
decided correctly on every tick and nobody published the decision, so the field
saw a robot that turned slowly with nothing anywhere saying why. Closing it
introduces the opposite risk in the same breath -- the blind clamp is the
STANDING path since 2026-09-29, so an event per tick is 20 a second on a path
every turn takes, which is the flood EVT-14 exists to prevent and which would
bury the real warnings rather than surface them.

So this file has to hold both directions at once, and neither half is optional:

  too little  a held clamp that reports nothing, or a second turn that is
              silently swallowed (that one is subtle -- see the dedup_window_s
              test, where the swallowing happens in p5 rather than here).
  too much    an event per tick, or an event per boundary flicker of
              spin_like, which is the same flood arriving through a different
              door.

The four rate criteria, and what each would miss on its own:
  1  entering a blind clamp emits exactly ONE event, not 20 a second
  2  holding it for N ticks emits nothing more
  3  leaving it emits nothing, re-arms after the dwell, and the NEXT entry is
     reported again under a new episode id (the module's no-clear semantics --
     see NO_CLEAR_EVENT in episodes.py for why leaving is silent)
  4  a blind clamp and a hard block are DIFFERENT events, because they are
     different machine behaviours and 12 S6A.6 gives them different HMI
     sentences

Criterion 1 is checked twice on purpose: once against hand-built RotationEval
values, and once against what rcg.apply_rotation_permit really returns for a
blind ring. Without the second, every test here could pass against verdict
shapes the permit never actually produces.
"""
from __future__ import annotations

import json
import pathlib

import pytest

from tests.p1_motion.nav.rot_fixture import rot_limits
from xbrain.common.enums import EVENT_CATEGORY, SEVERITY
from xbrain.p1_motion.rotation.episodes import (
    DEDUP_WINDOW_S,
    REARM_PASS_TICKS,
    SEV_FAULT,
    SEV_WARN,
    RotationEpisodeTracker,
)
from xbrain.p1_motion.rotation.rcg import (
    DECISION_LIMIT,
    DECISION_PASS,
    DECISION_REJECT,
    KIND_CLEARANCE_UNCONFIGURED,
    KIND_ROTATION_BLOCKED,
    REASON_OCCUPIED_CELLS,
    REASON_PERMITTED,
    REASON_R_ROBOT_UNCALIBRATED,
    REASON_SECTORS_UNAVAILABLE,
    REASON_UNKNOWN_CELLS,
    RingSample,
    RotationEval,
    apply_rotation_permit,
)

pytestmark = pytest.mark.no_device

# The calibrated body radius 12 S6A.3.3 records (2026-08-04, from the M20S
# manual). A TEST value: configs/ carries no r_robot key and CLAUDE.md iron
# rule 3 forbids landing one to make something run. Same constant and same
# reason as tests/p1_motion/nav/test_nav_tick_rotation.py.
R_ROBOT_CALIBRATED = 0.482


def _pass_eval() -> RotationEval:
    """A tick the permit let through. event_kind None is what marks it."""
    return RotationEval(
        spin_like=False, decision=DECISION_PASS, reason=REASON_PERMITTED,
        wz_in=0.0, wz_out=0.0, wz_limit_radps=None, r_check_m=None,
        occ_cells=0, unknown_cells=0, grid_age_ms=None,
        blocked_sector_deg=(), event_kind=None, detail_item=None)


def _blind_eval(*, wz_in: float = 1.5, reason: str = REASON_UNKNOWN_CELLS,
                unknown: int = 7) -> RotationEval:
    """A blind clamp: the standing path on this machine."""
    return RotationEval(
        spin_like=True, decision=DECISION_LIMIT, reason=reason,
        wz_in=wz_in, wz_out=0.3, wz_limit_radps=0.3, r_check_m=1.482,
        occ_cells=0, unknown_cells=unknown, grid_age_ms=120,
        blocked_sector_deg=((170.0, 190.0),),
        event_kind=KIND_ROTATION_BLOCKED, detail_item=None)


def _blocked_eval() -> RotationEval:
    """A hard block: there really is something in the ring."""
    return RotationEval(
        spin_like=True, decision=DECISION_REJECT, reason=REASON_OCCUPIED_CELLS,
        wz_in=1.5, wz_out=0.0, wz_limit_radps=None, r_check_m=1.482,
        occ_cells=4, unknown_cells=0, grid_age_ms=120,
        blocked_sector_deg=((80.0, 100.0),),
        event_kind=KIND_ROTATION_BLOCKED, detail_item=None)


def _unconfigured_eval() -> RotationEval:
    """RCG-1: no calibrated body radius. A fault, and a retry never helps."""
    return RotationEval(
        spin_like=True, decision=DECISION_REJECT,
        reason=REASON_R_ROBOT_UNCALIBRATED,
        wz_in=1.5, wz_out=0.0, wz_limit_radps=None, r_check_m=None,
        occ_cells=0, unknown_cells=0, grid_age_ms=None,
        blocked_sector_deg=(), event_kind=KIND_CLEARANCE_UNCONFIGURED,
        detail_item=REASON_R_ROBOT_UNCALIBRATED)


def _run(tracker: RotationEpisodeTracker, evals, *, source="nav2_proxy"):
    """Feed a list of RotationEval and collect every event that came out."""
    out = []
    for ev in evals:
        out += tracker.observe(ev, source=source)
    return out


# ---------------------------------------------------------------------------
# Criterion 1 + 2: the rate. This is the core of 12 S15 #54's closure.
# ---------------------------------------------------------------------------

def test_a_held_blind_clamp_emits_exactly_one_event():
    """Sixty ticks of the same verdict -> one event, not sixty.

    Three seconds of a clamped turn at 20 Hz. The number that matters is 1: a
    tracker that emitted per tick would put a thousand events a minute into
    record.db and onto the cloud backfill cursor on a path EVERY turn takes,
    and 12 S14 trap 19 says what happens to a gate that behaves like that in
    the field.

    mutant: drop the `if state == self._state: return []` guard in
    observe() -> 60 events -> red here.
    """
    t = RotationEpisodeTracker()
    evs = _run(t, [_blind_eval() for _ in range(60)])
    assert len(evs) == 1
    e = evs[0]
    assert e.kind == KIND_ROTATION_BLOCKED
    assert e.severity == SEV_WARN
    assert e.blind is True
    assert e.permit.decision == DECISION_LIMIT
    assert e.episode_id == 0


def test_the_reason_may_change_inside_one_episode_without_a_new_event():
    """unknown_cells and sectors_unavailable alternating is NOT a change.

    Both are blind, both clamp, both are kind rotation_blocked -- the machine
    does exactly the same thing. On this robot they alternate whenever the ring
    census sits on its threshold, because evaluate_ring reports whichever blind
    conjunct it reaches first and nothing publishes free_space.sectors, so this
    is the standing case rather than a corner one.

    mutant: put ev.reason into the state triple in observe() -> the alternating
    feed below produces an event per flip -> red here, while every other test
    in this file stays green. That asymmetry is the point: this is the only
    test that pins WHICH fields the edge is keyed on.
    """
    t = RotationEpisodeTracker()
    feed = []
    for i in range(20):
        feed.append(_blind_eval(
            reason=REASON_UNKNOWN_CELLS if i % 2 else REASON_SECTORS_UNAVAILABLE))
    evs = _run(t, feed)
    assert len(evs) == 1
    # The detail carries the reason the edge was taken on, which is the first
    # one seen -- not the one holding twenty ticks later. episodes.py says so
    # in as many words; asserting it keeps the docstring honest.
    assert evs[0].permit.reason == REASON_SECTORS_UNAVAILABLE


# ---------------------------------------------------------------------------
# Criterion 3: leaving, and the dwell that stops a flicker from reopening.
# ---------------------------------------------------------------------------

def test_leaving_emits_nothing_and_the_next_entry_is_reported_again():
    """No clear event, and the NEXT turn is still visible.

    The two halves are one criterion. Silence on the way out is the documented
    semantics (NO_CLEAR_EVENT: 12 S6A.8's kind set has no member for it, cat
    motion is channel normal so 11 S9A.9 E-1's stuck-alarm argument does not
    attach, and the turn ending is already on cmd_vel). But silence is only
    acceptable if re-arming works -- an implementation that never re-armed
    would also emit nothing here and would report the first limited turn after
    boot and nothing ever again.

    mutant: delete `self._state = None` from the re-arm branch -> the second
    burst produces no event -> red on the len(second) assertion.
    """
    t = RotationEpisodeTracker()
    first = _run(t, [_blind_eval() for _ in range(10)])
    assert len(first) == 1
    leaving = _run(t, [_pass_eval() for _ in range(REARM_PASS_TICKS)])
    assert leaving == []                      # no clear, by design
    second = _run(t, [_blind_eval() for _ in range(10)])
    assert len(second) == 1
    # A new turn attempt, so a new episode id. Without this the cloud cannot
    # separate two excursions -- the same property 11 S9A.9 E-2 gives the
    # fence counter.
    assert second[0].episode_id == first[0].episode_id + 1


def test_a_short_gap_does_not_reopen_the_episode():
    """spin_like flicker must not become an event per flicker.

    spin_like is |wz| > wz_eps AND |v|/|wz| < k_rot x r_eff, and a thumb on a
    joystick or an rns_avoid escape with translation in it crosses that
    boundary tick by tick. One PASS tick in the middle of a turn is that, not
    the end of the turn. 12 S5.2 ruled the same failure for this loop and its
    answer is the dwell used here.

    Parametrised up to REARM_PASS_TICKS - 1 so the test follows the constant
    instead of a retyped number; raising the constant cannot silently leave
    this test checking less than it did.

    mutant: re-arm on the first PASS tick -> every gap length below reopens the
    episode and a second event appears -> red.
    """
    for gap in range(1, REARM_PASS_TICKS):
        t = RotationEpisodeTracker()
        feed = ([_blind_eval() for _ in range(5)]
                + [_pass_eval() for _ in range(gap)]
                + [_blind_eval() for _ in range(5)])
        evs = _run(t, feed)
        assert len(evs) == 1, "gap=%d reopened the episode" % gap
        assert evs[0].episode_id == 0
    # The dwell is 12 S5.2's number, not one chosen here, and the loop above
    # would keep passing if somebody quietly shortened it -- it reads the
    # constant. This line is what ties the constant to the document.
    assert REARM_PASS_TICKS == 5


def test_a_verdict_tick_resets_the_dwell_counter():
    """Gaps do not accumulate across the ticks between them.

    Without the reset, four flicker ticks early in a turn plus one flicker tick
    a minute later add up to five and re-arm mid-turn, so the rest of the same
    turn is reported as a second episode. The counter has to mean "consecutive
    since the last verdict", which is only true if a verdict clears it.

    mutant: drop `self._pass_ticks = 0` from the verdict branch of observe()
    -> the feed below re-arms after the single gap and emits twice -> red.
    """
    t = RotationEpisodeTracker()
    feed = ([_blind_eval() for _ in range(5)]
            + [_pass_eval() for _ in range(REARM_PASS_TICKS - 1)]
            + [_blind_eval() for _ in range(5)]
            + [_pass_eval()]
            + [_blind_eval() for _ in range(5)])
    assert len(_run(t, feed)) == 1


def test_the_dwell_is_counted_only_inside_an_open_episode():
    """Hours of straight-line driving must not bank a re-arm.

    mutant: re-arm on the first PASS tick (REARM_PASS_TICKS -> 1) -> the gap of
    one below reopens the episode -> two events -> red.

    *** Measured 2026-09-29, and worth knowing before "strengthening" this:
    moving `self._pass_ticks += 1` above the `if self._state is None` early
    return is an EQUIVALENT mutant, not a survivor to be chased. The counter is
    read only inside an open episode, and every verdict tick zeroes it, so a
    count banked while nothing was open cannot reach a comparison. CLAUDE.md
    7.2.1 says to record that in place rather than invent an assertion for it;
    episodes.py carries the same note beside the branch. What this case does
    pin is the SCENARIO -- a long quiet stretch followed by a flickering turn --
    which the mutant above does kill.
    """
    t = RotationEpisodeTracker()
    _run(t, [_pass_eval() for _ in range(200)])
    feed = ([_blind_eval() for _ in range(3)]
            + [_pass_eval()]
            + [_blind_eval() for _ in range(3)])
    assert len(_run(t, feed)) == 1


# ---------------------------------------------------------------------------
# Criterion 4: a blind clamp and a hard block are two different things.
# ---------------------------------------------------------------------------

def test_a_blind_clamp_and_a_hard_block_are_separate_events():
    """Someone stepping into the ring mid-turn must be reported, not held.

    Both verdicts carry kind rotation_blocked, so an implementation keyed on
    kind alone would emit ONE event for the whole sequence and the operator
    would read "I could not see that way" while the machine was actually
    refusing because a person was 0.5 m off its flank. 12 S6A.3.2 splits
    exactly here and 12 S6A.6 turns the split into two different HMI
    sentences.

    mutant: key the state on ev.event_kind alone -> the second burst produces
    no event -> red.
    """
    t = RotationEpisodeTracker()
    evs = _run(t, [_blind_eval() for _ in range(5)]
               + [_blocked_eval() for _ in range(5)])
    assert len(evs) == 2
    blind, hard = evs
    assert (blind.blind, blind.permit.decision) == (True, DECISION_LIMIT)
    assert (hard.blind, hard.permit.decision) == (False, DECISION_REJECT)
    # Same turn attempt, so the same episode id ties them together. A cloud
    # reader has to be able to see that these two rows are one excursion.
    assert blind.episode_id == hard.episode_id
    # And the wz story must differ, or the two rows say the same thing.
    assert blind.permit.wz_out != 0.0 and hard.permit.wz_out == 0.0


def test_a_fault_kind_is_reported_at_fault_severity():
    """rotation_clearance_unconfigured is a fault, rotation_blocked a warn.

    12 S6A.8's table, and the difference is not cosmetic: the severity is the
    {severity} segment of event/{severity}/motion, so getting it wrong puts the
    row on a key the cloud subscription never matches (11 S2.2.11 V-2).

    mutant: map both kinds to SEV_WARN -> red.
    """
    t = RotationEpisodeTracker()
    evs = _run(t, [_unconfigured_eval() for _ in range(5)])
    assert len(evs) == 1
    assert evs[0].severity == SEV_FAULT
    assert evs[0].permit.detail_item == REASON_R_ROBOT_UNCALIBRATED
    # A transition between the two kinds is a change, not a hold.
    evs2 = _run(t, [_blocked_eval() for _ in range(5)])
    assert len(evs2) == 1 and evs2[0].severity == SEV_WARN


# ---------------------------------------------------------------------------
# Criterion 5: the evidence in the detail.
# ---------------------------------------------------------------------------

def test_detail_carries_every_ob2_field_and_the_evidence():
    """12 S6A.8 OB-2's six mandated fields plus what makes them actionable.

    The mandated six are listed as a literal tuple rather than spelled into
    separate asserts so that deleting one from detail() cannot be compensated
    by a passing test that never mentioned it.

    mutant: drop blocked_sector_deg from detail() -> red. That field is the one
    12 S6A.3.2 asks for by name so the field can see at a glance whether it is
    an obstacle or a coverage hole. mutant: report active_source as "hold"
    (OB-3) -> red. mutant: hard-code blind False -> red here and in the
    real-permit case below.
    """
    t = RotationEpisodeTracker()
    e = _run(t, [_blind_eval()], source="teleop_joystick")[0]
    d = e.detail()
    for field in ("r_check_m", "occ_cells", "unknown_cells",
                  "blocked_sector_deg", "grid_age_ms", "decision"):
        assert field in d, field
    assert d["decision"] == DECISION_LIMIT
    assert d["reason"] == REASON_UNKNOWN_CELLS
    assert d["blind"] is True
    assert d["unknown_cells"] == 7
    assert d["occ_cells"] == 0
    assert d["blocked_sector_deg"] == [[170.0, 190.0]]
    assert d["grid_age_ms"] == 120
    # OB-3: the real arbitration winner, never dressed up as hold.
    assert d["active_source"] == "teleop_joystick"
    # The limit in force, which is NOT recoverable from wz_out when the request
    # was already under it -- the whole reason the field exists.
    assert d["wz_limit_radps"] == 0.3
    assert d["wz_in"] == 1.5 and d["wz_out"] == 0.3
    # JSON-serialisable as it stands: the publisher does no conversion, so a
    # tuple left in here would raise inside the 20 Hz tick.
    json.dumps(d)


def test_a_veto_reports_no_limit_in_force():
    """wz_limit_radps is null when the turn was refused, not zero.

    0.0 would read as "it was allowed to turn at 0 rad/s", which is a different
    statement from "no limit applied, the turn was refused" and is exactly the
    CLAUDE.md 3.1 shape of a zero standing in for an absent value.

    *** This case asserts the TRACKER passes the value through, nothing more.
    It cannot see a change in apply_rotation_permit, because the RotationEval
    it feeds is built here -- measured 2026-09-29: initialising wz_limit to
    limits.wz_blind_radps unconditionally in rcg.py survived this whole file.
    The judge-side assertion that kills that one lives next to the judge, in
    tests/p1_motion/nav/test_nav_tick_rotation.py::
    test_wz_limit_radps_is_reported_only_when_a_clamp_was_applied.

    mutant: return 0.0 instead of None from detail() when wz_limit_radps is
    absent -> red here.
    """
    t = RotationEpisodeTracker()
    d = _run(t, [_blocked_eval()])[0].detail()
    assert d["wz_limit_radps"] is None
    assert d["wz_out"] == 0.0


# ---------------------------------------------------------------------------
# The wire contract: severities, category, and the dedup window.
# ---------------------------------------------------------------------------

def test_the_severities_come_from_the_contract_closed_set():
    """CLAUDE.md 3.5: closed-set values are exported, not typed out.

    Checked against the shared export rather than against the literals this
    module happens to hold, so a rename in 11 S6.1 that reaches sets.yaml is
    caught here instead of becoming a key nothing subscribes to.
    """
    assert SEV_WARN in SEVERITY and SEV_FAULT in SEVERITY
    # The category segment of the key these events go out on.
    assert "motion" in EVENT_CATEGORY


def test_every_kind_the_permit_can_emit_has_a_severity():
    """Meta test, same shape and same reason as
    test_every_behavior_source_has_a_disposal in the judge's own file.

    episodes.py looks the severity up with a subscript so an unmapped kind
    raises rather than picking one. That guard can only fire at run time inside
    a control tick, which is the worst possible place to discover it, so the
    thing that actually holds the mapping complete is this test.

    mutant: add a fourth KIND_ constant to rcg.py without touching
    _SEVERITY_OF_KIND -> red here.
    """
    import xbrain.p1_motion.rotation.episodes as ep
    import xbrain.p1_motion.rotation.rcg as rcg
    kinds = {v for k, v in vars(rcg).items()
             if k.startswith("KIND_") and isinstance(v, str)}
    assert kinds == set(ep._SEVERITY_OF_KIND)


def test_the_event_carries_a_dedup_window_and_it_is_zero():
    """Without a window, p5 swallows every event after the first, for ever.

    record_dao._try_merge takes the latest still-open row with this dedup_key
    and, when neither the event nor that row names a window, merges
    unconditionally. dedup_key is fixed by OB-2 as "rotation:{kind}" -- one key
    for the whole machine life -- so a missing window turns this entire feature
    into one row whose dedup_count grows. It would look implemented and it
    would be invisible.

    mutant: drop dedup_window_s from RotationEvent (or from the published
    body) -> red here and in the publisher test below.
    """
    t = RotationEpisodeTracker()
    e = _run(t, [_blind_eval()])[0]
    assert e.dedup_key == "rotation:%s" % KIND_ROTATION_BLOCKED
    assert e.dedup_window_s == DEDUP_WINDOW_S == 0


# ---------------------------------------------------------------------------
# Against the real permit, so none of the above is a test of fixtures only.
# ---------------------------------------------------------------------------

def test_the_real_permit_on_a_blind_ring_yields_one_event_for_the_whole_turn():
    """rcg.apply_rotation_permit drives the tracker, no hand-built verdicts.

    Everything above builds RotationEval by hand, which is the only way to pin
    the edge rules -- and it is also how a whole file can pass while the permit
    never produces any of those shapes. This runs the real judge over the ring
    this machine actually has (rear unobserved, no free_space.sectors) for
    three seconds of turning, and asserts the same number: one.

    mutant: any change that makes the permit stop reporting event_kind on the
    blind path -> zero events -> red.
    """
    lim = rot_limits()
    r_check = R_ROBOT_CALIBRATED + lim.margin_rot_m
    ring = RingSample(occ_cells=0, unknown_cells=9, total_cells=800,
                      unknown_ratio=0.1, age_ms=100, resolution_m=0.1,
                      sectors_min_m=None,
                      blocked_sector_deg=((170.0, 190.0),))
    t = RotationEpisodeTracker()
    evs = []
    for _ in range(60):
        ev = apply_rotation_permit(
            vx_mps=0.0, vy_mps=0.0, wz_radps=1.5, source="nav2_proxy",
            limits=lim, r_robot_m=R_ROBOT_CALIBRATED, ring=ring)
        assert ev.decision == DECISION_LIMIT      # the standing path
        evs += t.observe(ev, source="nav2_proxy")
    assert len(evs) == 1
    d = evs[0].detail()
    assert d["blind"] is True
    assert d["wz_limit_radps"] == pytest.approx(lim.wz_blind_radps)
    assert d["r_check_m"] == pytest.approx(r_check)


# ---------------------------------------------------------------------------
# The publisher. NavRuntime cannot be constructed under pytest (it needs two
# zenohd routers), so the method is called unbound on a minimal holder -- the
# same convention as test_nav_wiring_ring and test_nav_wiring_estop_epoch.
# ---------------------------------------------------------------------------

class _FakeGen:
    """Captures (key, payload) instead of putting them on a session."""

    def __init__(self) -> None:
        self.puts = []

    def put(self, key, payload):
        self.puts.append((key, json.loads(payload.decode("utf-8"))))


class _Holder:
    """Only the attributes _publish_rotation_event actually touches."""

    def __init__(self) -> None:
        self._gen = _FakeGen()
        self._seq = {"event": 0}
        self._event_boot = "b1"
        self._counts = {"publish_fail": 0}


class _Inp:
    """Stands in for NavInputs: the publisher reads one field off it."""

    def __init__(self, ts_wall_s: float) -> None:
        self.ts_wall_s = ts_wall_s


def _publish(holder, event, ts):
    from xbrain.p1_motion.runtime.nav_wiring import NavRuntime
    NavRuntime._publish_rotation_event(holder, event, _Inp(ts))


def test_the_publisher_puts_the_event_on_the_contract_key():
    """P1-10 event/{severity}/motion, with the OB-2 body.

    mutant: publish on "event/%s/rotation" -> red. motion is the category 11
    S6.2 registers; a rotation category does not exist and the p5 pipeline
    would drop the row at step 1 with E_SCHEMA.
    """
    t = RotationEpisodeTracker()
    e = _run(t, [_blind_eval()])[0]
    h = _Holder()
    _publish(h, e, 1000.0)
    assert len(h._gen.puts) == 1
    key, body = h._gen.puts[0]
    assert key == "event/warn/motion"
    assert body["title"] == KIND_ROTATION_BLOCKED
    assert body["dedup_key"] == "rotation:%s" % KIND_ROTATION_BLOCKED
    assert body["dedup_window_s"] == 0
    assert body["src"] == "p1_motion"
    assert body["detail"]["decision"] == DECISION_LIMIT
    assert body["eid"].startswith("rot-b1-")


def test_the_published_ts_is_a_real_rising_clock():
    """ts 0.0 plus dedup_window_s 0 would swallow every event after the first.

    The p5 merge test is (ts - last_ts) > window. With a constant ts that is
    0 > 0, false, so every later row merges into the first still-open one and
    nothing more reaches the cloud or the HMI. chassis_events.py measured this
    from the other end on 2026-09-27, where the two clocks made the difference
    negative. The fix is the same: one clock, and it must rise.

    mutant: publish "ts": 0.0 like _publish_event does -> red.
    """
    t = RotationEpisodeTracker()
    e = _run(t, [_blind_eval()])[0]
    h = _Holder()
    _publish(h, e, 1000.0)
    _publish(h, e, 1000.5)
    seen = [b["ts"] for _k, b in h._gen.puts]
    assert seen == [1000.0, 1000.5]
    assert all(ts > 0.0 for ts in seen)


def test_a_publish_failure_does_not_escape_into_the_tick():
    """Losing the event beats losing the tick.

    An exception out of here reaches _one_tick, whose guard publishes ZERO for
    that tick (CLAUDE.md 4.4). That trade is wrong on a diagnostic path: the
    permit has already decided and wz is already correct, so a broken session
    would turn an observability feature into a stutter in the control output.

    mutant: remove the try/except in _publish_rotation_event -> red.
    """
    class _Boom:
        def put(self, key, payload):
            raise RuntimeError("session down")

    t = RotationEpisodeTracker()
    e = _run(t, [_blind_eval()])[0]
    h = _Holder()
    h._gen = _Boom()
    _publish(h, e, 1000.0)                     # must not raise
    assert h._counts["publish_fail"] == 1


# ---------------------------------------------------------------------------
# The wiring itself. 12 S15 #54 was not a bug in any of the code above -- all
# of it would have passed -- it was that runtime/nav_wiring.py never read
# NavOutput.rotation. Nothing testable in-process covers that, because
# NavRuntime needs two zenohd routers, so it is asserted on the source text in
# the style of test_nav_wiring_fence_static, which guards the fence half the
# same way.
# ---------------------------------------------------------------------------

_NAV_SRC = (pathlib.Path(__file__).resolve().parents[3]
            / "xbrain" / "p1_motion" / "runtime" / "nav_wiring.py"
            ).read_text(encoding="utf-8")


def test_the_tick_feeds_the_tracker_and_publishes_what_comes_out():
    """The exact defect 12 S15 #54 recorded, asserted directly.

    mutant: delete the `if out.rotation is not None:` block from _one_tick ->
    red. Every other test in this file stays green, which is precisely what
    happened for the whole of 2026-09-29 before this one existed.
    """
    assert "self._rot_episodes.observe(out.rotation" in _NAV_SRC
    assert "self._publish_rotation_event(" in _NAV_SRC
    assert 'self._gen.put("event/%s/motion" % re_.severity' in _NAV_SRC


def test_the_tracker_is_built_once_for_the_process_not_per_tick():
    """A tracker constructed inside the tick has no memory and no edge.

    It would emit on every tick while looking exactly like an edge-triggered
    implementation from the outside -- the tracker's own tests would all still
    pass, because they feed one instance. Placement is the whole difference, so
    placement is what is asserted: the construction must sit in __init__, above
    the tick.

    mutant: move the construction into _one_tick -> red.
    """
    ctor = _NAV_SRC.index("self._rot_episodes = RotationEpisodeTracker()")
    tick = _NAV_SRC.index("def _one_tick(")
    assert ctor < tick
