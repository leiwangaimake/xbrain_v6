"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: episodes.py
Brief: 12 S6A.8 OB-2 rotation events -- edge triggered from the per-tick RotationEval

Description:
The problem this file solves. rotation/rcg.py decides, every tick, whether the
turn this tick asks for may happen and at what wz. 12 S15 #54 measured what
happened to that decision afterwards: nothing. NavOutput.rotation was computed
on every one of the 20 ticks a second and read by no producer, so
event/warn/motion{rotation_blocked} and event/fault/motion{
rotation_clearance_unconfigured} were never published by anybody. Since the
RCG-3 correction of 2026-09-29 made the blind clamp the STANDING path on this
machine, the field symptom of that gap is "tell it to turn around, it turns
very slowly, and there is nothing anywhere saying why" -- which is
indistinguishable from a weak chassis, a derated motor or a high-friction
floor. 12 S6A.8 OB-1 forbids widening gate.limiter and OB-4 records that 11
S3.4's gate block has no field able to answer it either, so the event is the
only exit that exists and this module is the thing that opens it.

Which design sections. Event shape and the detail list: 12 S6A.8 OB-2, verbatim
"detail 带 { r_check_m, occ_cells, unknown_cells, blocked_sector_deg[],
grid_age_ms, decision }". The kinds and their severities: the 12 S6A.8 table
(rotation_blocked = warn, rotation_clearance_unconfigured = fault). The
attribution the detail must NOT fake: OB-3. The hysteresis shape: 12 S5.2.

Edge triggering, which is the load-bearing part. RotationEval is a LEVEL
signal: while a turn is being limited, all 20 ticks of that second carry the
same verdict. Feeding each one to the pipeline is 20 events a second on a path
that is now taken by every turn the robot makes -- the flood EVT-14 exists to
prevent, and the one fence/episodes.py names in its own closing trap. So this
module keeps the verdict it last reported and emits only when that changes.

What counts as a change is a choice, and it is this triple:

    (kind, decision, blind)

  * kind     tells the operator whether this is about the surroundings (warn,
             a retry can help) or about the configuration (fault, it cannot).
  * decision tells him whether the turn was slowed (limit) or refused
             (reject). Those are different machine behaviours.
  * blind    tells him "I could not see that way" from "there is something
             there". 12 S6A.3.2 splits exactly there, and 12 S6A.6 turns the
             same split into two different HMI sentences -- clear the area and
             retry, against rotation is unavailable and retrying will not help.

reason is deliberately NOT in the triple even though it rides in the detail.
On this machine the ring is blind for two reasons at once (the rear is never
observed AND nothing publishes free_space.sectors), and evaluate_ring reports
whichever conjunct it reaches first. A ring whose unknown count sits on its
threshold therefore alternates between unknown_cells_over_max and
sectors_full_circle_unavailable at 20 Hz while the machine's behaviour does not
change at all. Keying on reason would turn that into two events per
oscillation; keying on the triple collapses it, and the detail still carries
the reason the edge was taken on.

Re-arming, and why it needs a dwell. The tracked state clears when the permit
goes back to PASS -- but PASS flickers. spin_like is |wz| > wz_eps AND
|v|/|wz| < k_rot x r_eff, and an operator's thumb on a joystick or an rns_avoid
escape with some translation in it crosses that boundary tick by tick. A
one-tick PASS would end the episode and the next non-PASS tick would open a new
one, i.e. the flood again through a different door. 12 S5.2 already ruled this
exact failure for this exact loop -- rns_avoid chattering at an obstacle
boundary -- and its answer is the shape used here: enter immediately, leave
only after N consecutive ticks that do not qualify. See REARM_PASS_TICKS.

What this module does NOT do, and where that work lives instead.
  * It does not publish. observe() returns values; runtime/nav_wiring.py does
    the Zenoh put. Keeping the I/O out is what lets the whole edge machine be
    tested against a list of RotationEval values with no session, and it is the
    same division chassis_events.py draws for the same reason.
  * It does not read a clock. The dwell is counted in TICKS, not milliseconds,
    because observe() is called exactly once per control tick and the tick
    length is 12 S2.2's structural constant. That also means there is no
    monotonic-clock question here at all (11 CLK-C1): there is no clock.
  * It does not decide the channel. 11 S6.2 fixes motion -> normal and 17 S3.3
    makes the p5 pipeline the authority; the same rule E-1 states for fence.
  * It does not judge anything. Every field it reports comes out of rcg.py. A
    second opinion on the ring here would be 12 S6A.9 ND-3's "judge implemented
    twice", whose failure mode is an entry that permits and an exit that
    refuses.

The looks-right-but-wrong writings, each one a named prohibition.
  * Emitting a "rotation cleared" event on the way out. See NO_CLEAR_EVENT.
  * Emitting a heartbeat while the state persists. See NO_HEARTBEAT.
  * Leaving dedup_window_s off the event. See DEDUP_WINDOW_S -- with a key set
    and no window, record_dao merges every later event of that key into the
    first still-open row for ever, so the second turn of the day and every turn
    after it would silently never reach the cloud or the HMI. An event that is
    published and cannot arrive is worse than no event: it looks done.
  * Counting the dwell while nothing is being reported. The counter is only
    advanced inside an open episode; otherwise it would run up over hours of
    straight-line driving and the number would stop meaning "how long has it
    been quiet since the last verdict".
  * Reporting the arbitration winner as anything but the real one. OB-3
    forbids dressing a zeroed tick up as hold; the source is passed in from
    NavOutput.source unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from xbrain.common.enums import SEVERITY
from xbrain.p1_motion.rotation.rcg import (
    BLIND_REASONS,
    KIND_CLEARANCE_UNCONFIGURED,
    KIND_ROTATION_BLOCKED,
    RotationEval,
)

# ---------------------------------------------------------------------------
# Severities, taken from the shared closed set rather than written out.
#
# CLAUDE.md 3.5 wants closed-set values exported by the shared library, not
# hard-coded strings. parse() returns the value and raises otherwise, so these
# two lines are both the definition and the check: a rename in 11 S6.1 that
# reaches enums/sets.yaml breaks the import of this module immediately, instead
# of producing an event on a key -- event/{severity}/motion -- that no
# subscriber matches and nobody notices is missing (V-2).
# ---------------------------------------------------------------------------
SEV_WARN = SEVERITY.parse("warn")
SEV_FAULT = SEVERITY.parse("fault")

# 12 S6A.8's table, as a mapping rather than an if. The two kinds differ on one
# question -- can a retry ever clear this -- and 12 S6A.7 RC-D5 routes the same
# split to E_BUSY against E_CAPABILITY at the command layer, so the two answers
# have to agree.
#
# A kind that is not in here raises KeyError rather than picking a severity.
# That is the _disposal_for discipline of rcg.py applied to the same problem:
# defaulting to warn would under-report a fault and defaulting to fault would
# put the alarm-shaped severity on an ordinary blocked turn. The case cannot
# arise from data -- both keys come from rcg.py in the same package -- so the
# thing that actually guards it is the meta test that walks rcg's kinds against
# this table, not a run-time branch.
_SEVERITY_OF_KIND = {
    KIND_ROTATION_BLOCKED: SEV_WARN,
    KIND_CLEARANCE_UNCONFIGURED: SEV_FAULT,
}

# ---------------------------------------------------------------------------
# REARM_PASS_TICKS -- how many consecutive PASS ticks end an episode.
#
# 12 S5.2 already fixed this number for this loop. Its rule block reads, in
# translation, activate immediately because that is the safe direction and take
# no hysteresis on it, deactivate only after N consecutive ticks fail the
# condition, N defaulting to 5 ticks = 250 ms. Grep anchor for the section, per
# NUM-4: the heading "5.2" in 12 followed by the word for hysteresis. Its own
# reason is the one that applies here -- a per-tick predicate chattering at its
# boundary must not be allowed to chatter downstream.
# sources/arbiter_p1.py realises the same number as a
# 200 ms dwell compared with a strict >, which at 50 ms ticks deactivates on
# the fifth tick after the last hit.
#
# So this is a number the book already fixed for this loop, not one tuned here.
# Both failure directions are bounded and neither is a safety direction:
#   too large  -> two genuinely separate turns inside a quarter second merge
#                 into one event. A 90 degree turn at the blind clamp takes
#                 about five seconds, so this cannot merge two real turns; it
#                 can only merge one human action with itself.
#   too small  -> one turn is split into several events, i.e. the flood this
#                 exists to prevent, arriving more slowly.
# Neither direction touches wz. The permit has already run and already decided;
# this counter only decides when the NEXT event may be emitted.
# ---------------------------------------------------------------------------
REARM_PASS_TICKS = 5

# ---------------------------------------------------------------------------
# DEDUP_WINDOW_S -- 0, and it must be sent.
#
# 12 S6A.8 OB-2 fixes the key as "rotation:{kind}" and says the dedup follows
# EVT-17's practice. What OB-2 does not say, and what the p5 side makes
# load-bearing: record_dao._try_merge selects the latest still-open row with
# this dedup_key and, when neither the event nor that row carries a window,
# merges unconditionally. A key with no window therefore means the FIRST
# rotation_blocked of the process life creates a row and every later one --
# every later turn, for ever -- folds into it, bumping dedup_count and pushing
# nothing to the cloud or the HMI. That is not deduplication, it is a silent
# drop, and it would leave this whole module looking implemented while the
# field symptom 12 S15 #54 describes stayed exactly as it is.
#
# 0 means "carry the key, coalesce nothing": the merge test is
# (ts - last_ts) > window, so a strictly later event never merges and only an
# event with the identical ts does. It works only because the ts this module's
# events carry comes from one clock and rises -- see the ts note on
# nav_wiring._publish_rotation_event, and chassis_events.DEDUP_WINDOW_S for
# what happened the day two clocks were mixed on one key.
#
# This is not CLAUDE.md 3.1's "0 pretending to be a calibrated value": it is
# not a safety parameter and not a stand-in for a number nobody measured. The
# flood a window would damp is already damped upstream by edge triggering,
# which damps by STATE rather than by a guessed number of seconds.
# ---------------------------------------------------------------------------
DEDUP_WINDOW_S = 0

# ---------------------------------------------------------------------------
# NO_CLEAR_EVENT -- why leaving the state emits nothing.
#
# Three reasons, and the first one is decisive on its own.
#
#  1. There is no value to put in it. 12 S6A.8 defines detail.kind for cat
#     motion as a closed set of three (rotation_blocked,
#     rotation_clearance_unconfigured, rotation_visual_override) and OB-2 fixes
#     decision to { reject, limit, override }. A clear needs a new member of
#     one of those, which is a contract change and therefore a ruling, not an
#     implementation choice (CLAUDE.md 1 and 3.5). Inventing one here would put
#     a value on the wire that no consumer's closed set contains.
#  2. The obligation to pair a raise with a recovery, where this repository
#     states it, is stated of the ALARM channel: 11 S9A.9 E-1 for fence, and
#     the device_offline / device_online rows of 11 S6.2, all with the same
#     reason -- an unpaired raise leaves the cloud stuck in alarm. cat motion
#     is channel normal (11 S6.2), so that obligation does not attach here.
#  3. There is nothing latched to clear. A chassis fault is a state of the
#     machine and a fence breach is a standing fact about where it is;
#     "the ring was not clean while you tried to turn" exists only while a turn
#     is being attempted. The turn ending is already visible at 20 Hz on
#     rt/motion/cmd_vel (wz) and on state/arb/motion (the holder), so a clear
#     event would restate what those already say while costing a closed-set
#     extension.
#
# Reversing this is one branch in observe() plus a kind. It is written down
# here rather than left implicit so that it can be reversed on purpose.
#
# NO_HEARTBEAT -- why a persisting state does not re-announce itself.
#
# The tempting version is "one line every N seconds while the clamp holds".
# N would be a new tunable with no measured basis, which CLAUDE.md 3.7 is
# about, and the thing it would answer is not the thing operations asks. The
# operational question is "is every turn being limited", and that is answered
# by the per-episode events themselves: one per commanded turn, so the rate of
# them IS the answer. Repeats inside one turn add nothing to it.
#
# The case a heartbeat would genuinely cover is an UNBOUNDED episode -- an
# rns_avoid escape that spins on the spot and never leaves, which has no
# command channel and therefore no T-12 style timeout of its own (12 S6A.4.2
# lists it as one of the three sources with no entry check). That case is real
# and is registered in 12 S15 #54 rather than answered with an invented period.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RotationEvent:
    """One 12 S6A.8 OB-2 row, ready for the wiring to envelope and publish.

    It carries the whole RotationEval rather than copies of its fields so the
    detail cannot drift from what the permit actually decided; detail() below
    is the single place the wire shape is written.
    """

    kind: str                # 12 S6A.8 detail.kind
    severity: str            # SEVERITY member, from _SEVERITY_OF_KIND
    dedup_key: str           # OB-2: "rotation:{kind}"
    dedup_window_s: int      # see DEDUP_WINDOW_S
    episode_id: int          # one turn attempt; stable across a kind change
    blind: bool              # reason in BLIND_REASONS -- the 12 S6A.3.2 split
    source: str              # OB-3: the real arbitration winner, never faked
    permit: RotationEval

    def detail(self) -> Dict[str, Any]:
        """The 12 S6A.8 OB-2 detail body.

        OB-2's six mandated fields are r_check_m, occ_cells, unknown_cells,
        blocked_sector_deg, grid_age_ms and decision. The rest are here because
        without them the event cannot be acted on:

          kind / episode_id   which of the three sentences this is, and which
                              turn attempt it belongs to.
          reason              which conjunct of 12 S6A.3.2 failed. The edge is
                              not keyed on it (see the module docstring), so
                              this is the reason AT THE EDGE, not necessarily
                              the one holding a second later.
          blind               "could not see" against "something is there".
                              It is not derivable downstream: both map to kind
                              rotation_blocked, and 12 S6A.6 needs them apart
                              to pick between its two HMI sentences.
          active_source       OB-3, and the axis 12 S6A.4.2 disposes on. Two
                              ticks with identical rings get opposite
                              decisions depending on this, so an event without
                              it cannot be explained at all.
          wz_in / wz_out      what was asked for and what was allowed.
          wz_limit_radps      the clamp in force, or null. NOT inferable from
                              wz_out: a request under the clamp comes out
                              unchanged (see the field's note in rcg.py).
          item                the key or reason naming what is unconfigured,
                              null otherwise. It is what separates "no clamp
                              value configured" from "this source does not get
                              the clamp" -- both are decision reject with
                              wz_out 0 and they send the operator to different
                              places.

        blocked_sector_deg is converted to lists because the wire is JSON and
        a tuple of tuples would serialise to nested arrays anyway; doing it
        here keeps the publisher free of any knowledge of the shape.
        """
        ev = self.permit
        return {
            "kind": self.kind,
            "episode_id": self.episode_id,
            "decision": ev.decision,
            "reason": ev.reason,
            "blind": self.blind,
            "active_source": self.source,
            "wz_in": round(ev.wz_in, 4),
            "wz_out": round(ev.wz_out, 4),
            "wz_limit_radps": (None if ev.wz_limit_radps is None
                               else round(ev.wz_limit_radps, 4)),
            "r_check_m": (None if ev.r_check_m is None
                          else round(ev.r_check_m, 4)),
            "occ_cells": ev.occ_cells,
            "unknown_cells": ev.unknown_cells,
            "blocked_sector_deg": [list(s) for s in ev.blocked_sector_deg],
            "grid_age_ms": ev.grid_age_ms,
            "item": ev.detail_item,
        }


class RotationEpisodeTracker:
    """Per-tick RotationEval in, 12 S6A.8 OB-2 events out.

    One instance per p1 process, driven from the 20 Hz thread only, so no lock.
    The state is in memory: after a restart the next limited turn reports
    itself again with a new episode id, which is the honest answer -- this
    process has never reported it.
    """

    def __init__(self) -> None:
        # The triple last reported, or None when no episode is open.
        self._state: Optional[Tuple[str, str, bool]] = None
        # Consecutive PASS ticks seen INSIDE an open episode. Only meaningful
        # while _state is not None; see the module docstring's prohibition on
        # counting it outside one.
        self._pass_ticks = 0
        # Monotone per process. It is not reset by a kind change, because every
        # verdict of one turn attempt belongs to the same attempt -- the same
        # property 11 S9A.9 E-2 gives the fence episode counter.
        self._episode = 0

    @property
    def episode_id(self) -> int:
        """The current counter. Exposed for the tests and for a status read."""
        return self._episode

    def observe(self, ev: RotationEval, *, source: str) -> List[RotationEvent]:
        """The events this tick -- at most one, usually none.

        ev.event_kind is None exactly when the permit passed (both the
        not-spin_like path and the permitted path set it so), which is why this
        function does not need to look at spin_like or decision to know whether
        there is anything to say.

        mutant: return an event whenever ev.event_kind is not None, dropping
        the state compare -> 20 events a second on the standing path ->
        test_a_held_blind_clamp_emits_exactly_one_event red.
        mutant: clear _state on the first PASS tick instead of after
        REARM_PASS_TICKS -> a one-tick spin_like flicker reopens the episode ->
        test_a_one_tick_gap_does_not_reopen_the_episode red.
        """
        if ev.event_kind is None:
            if self._state is None:
                # Nothing open. Straight-line driving lives here and must cost
                # nothing at all -- no counter, no allocation, no event.
                #
                # *** Measured 2026-09-29: moving the increment below ABOVE
                # this return is an EQUIVALENT mutant, not a hole. The counter
                # is only read inside an open episode and every verdict tick
                # zeroes it, so a count banked out here can never reach a
                # comparison. CLAUDE.md 7.2.1 asks for that to be written down
                # rather than answered with an assertion nothing can fail.
                # The placement stays as it is because it keeps the field's
                # meaning true -- consecutive ticks SINCE THE LAST VERDICT --
                # and the next person to read it should not have to re-derive
                # the equivalence.
                return []
            self._pass_ticks += 1
            if self._pass_ticks >= REARM_PASS_TICKS:
                # The turn is over. Close the episode so the NEXT one is
                # reported, and only then advance the counter, so every event
                # of the attempt that just ended shares one id.
                self._state = None
                self._pass_ticks = 0
                self._episode += 1
            return []

        # A verdict tick. Any PASS run so far was a flicker inside this
        # episode, not the end of it.
        self._pass_ticks = 0
        blind = ev.reason in BLIND_REASONS
        state = (ev.event_kind, ev.decision, blind)
        if state == self._state:
            # Held. This is the 20 Hz case and it must stay silent; the fact
            # that it is still true is already on rt/motion/cmd_vel.
            return []
        self._state = state
        return [RotationEvent(
            kind=ev.event_kind,
            severity=_SEVERITY_OF_KIND[ev.event_kind],
            # OB-2 verbatim. The kind is the whole discriminator on purpose:
            # with DEDUP_WINDOW_S at 0 the key coalesces nothing, so it serves
            # as the p5-side grouping label rather than as a suppressor.
            dedup_key="rotation:%s" % ev.event_kind,
            dedup_window_s=DEDUP_WINDOW_S,
            episode_id=self._episode,
            blind=blind,
            source=source,
            permit=ev)]


__all__ = [
    "SEV_WARN", "SEV_FAULT",
    "REARM_PASS_TICKS", "DEDUP_WINDOW_S",
    "RotationEvent", "RotationEpisodeTracker",
]
