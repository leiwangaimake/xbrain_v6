"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: chassis_events.py
Brief: derive 11 S6.1 Events from the level-triggered ChassisFault snapshot (11 S9.8.4)

Description:
The problem this solves. 11 S9.8.4 puts a ChassisFault on event/fault/chassis:
{"faults":[{code, level, desc, since_ts}], "cleared":[code, ...]}. That is NOT an
Event -- it has no eid / cat / sev / title / detected_at -- and the only consumer,
p5's EventPipeline, rejects it at step 1 with dropped: missing_field:eid. So with
a real chassis attached EVERY fault report would be discarded: nothing in
record.db, nothing on the cloud, HMI all-green while the machine is faulted. No
process performed that conversion; this module is it.

Who derives, and why it is p5. 11 UM-4 is the precedent, verbatim: the event is
derived by P1 from state/robot and published on event/warn/motion, NOT by
quadruped. The producer ships the state snapshot; the CONSUMER derives the event.
p5 is the consumer here (user ruling), and it is also the process that already
owns record.db, the dedupe window and the cloud relay.

Edge triggering, which is the load-bearing part. ChassisFault is a LEVEL-
triggered snapshot at 2 Hz + on change: while a fault persists, every single
report carries the same code again. Feeding each report into the pipeline would
put two events per second per fault into record.db and onto the alarm backfill
cursor -- the event flood EVT-14 exists to prevent. So this module keeps the set
of codes it has already reported and emits:
  * one RAISE event the first time a code appears in faults[]
  * one CLEAR event when that code appears in cleared[]
and nothing at all for the repeats in between. That also absorbs the P1-20 double
path for free (11 S2.2.1 registers TWO subscribers of rt/chassis/fault on
purpose: chassis_relay normally, p1_motion when the relay is dead): the second
copy of one report carries codes that are already in the active set, so it is not
an edge and produces no second Event.

What it does NOT do:
  * it does not publish. observe() returns event dicts; the wiring feeds them to
    the pipeline / HMI ring / cloud relay. Keeping the I/O out is what lets the
    whole edge machine be tested against a list of snapshots with no Zenoh.
  * it does not own the fault code table. CF-1 shape validation is imported from
    common/errors/chassis_faults.py (one regex in the repository); registration
    is an OPEN set (13 QD-6) and is deliberately not consulted here -- an
    unregistered but well-formed code must still raise its event.
  * it does not derive channel. 11 S6.2 fixes chassis -> alarm and the pipeline
    derives it (17 S3.3). Both the raise and the clear therefore ride the SAME
    backfill cursor, which is what 11 S9A.9 E-1 requires of a paired event: a
    recovery on the other cursor leaves the cloud stuck in alarm forever.

Traps that look right and are not:
  1. Giving raise and clear the SAME dedup_key with no window. record_dao merges
     any event whose dedup_key matches a still-open row, so the recovery would be
     swallowed into the raise row and never reach the cloud -- the stuck-alarm
     bug again. See DEDUP_WINDOW_S below for the resolution that still satisfies
     CF-4.
  2. Stamping detected_at with the time WE saw the report. The report is a
     snapshot: a fault that started 40 s ago is re-announced in every report
     since, so "now" would date every fault to the moment p5 happened to look.
     since_ts is the chassis' own occurrence time and is what detected_at means
     (the record.db column comment: evidence time).
  3. Treating a malformed code as "probably chs:". CF-1 forbids it in as many
     words. A malformed entry is rejected and counted; the REPORT is not dropped,
     because the other entries in it are real faults (the same discipline
     read_fault_report implements one layer down).
  4. Stamping Event.ts with since_ts as well. detected_at takes since_ts (rule
     4) and ts does NOT (rule 6): ts is the record_dao merge comparison value
     and since_ts is the CHASSIS wall clock, so a raise dated from it and a
     clear dated from our own clock are two different clocks. With the chassis
     behind us, (ts_clear - ts_raise) came out negative, the merge test
     (ts - last_ts) > window was false for every window, and each real
     recurrence was folded into the old row and vanished -- see DEDUP_WINDOW_S.
  5. Deriving only the four keys 11 S9.8.4's derivation rules name. The wire
     carries five MORE -- details / grouped / resources / source / source_ids
     -- which 13 S7.3 marks upstream with one reason for the whole group:
     field localisation depends on them. Reading past them (this module did
     until 2026-09-27) leaves the fault chain complete and the evidence
     stranded at p5: record.db, the cloud and the HMI get a code and no
     locator, and nothing reports a problem, because a detail that was never
     written and a chassis that named nothing look identical. See
     _evidence_of, which also keeps absent and empty apart for that reason.
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional

from xbrain.common.enums import EVENT_CATEGORY, FAULT_LEVEL, SEVERITY
from xbrain.common.errors.chassis_faults import is_wellformed_fault_code

_logger = logging.getLogger("xbrain.p5.chassis_events")


# 11 S6.2: the category row whose channel is alarm and whose key is literally
# event/fault/chassis. Asserted against the closed set at import so a rename of
# the category (it was chassis_fault until v0.3) fails here rather than at the
# pipeline's step 1, where it would read as "unknown_category" on a live fault.
CHASSIS_CAT = "chassis"
assert CHASSIS_CAT in EVENT_CATEGORY, "11 S6.2 category set lost 'chassis'"

# 11 S4.1 faults[].level (three values, ordered warn < degraded < fatal) ->
# 11 S6.1 Event.sev (four values). The two sets are different sizes, so the map
# is the decision; it is made once, here, and each arm has a reason:
#   warn     -> warn    the vendor says advisory; reporting it as fault would
#                       make every advisory look like a stopped machine.
#   degraded -> fault   13 S7.3 defines degraded as "affects operation" (it is
#                       also where an UNKNOWN vendor severity lands, verbatim
#                       "not warn -- an unknown severity affects operation until
#                       proven otherwise"). SEVERITY has no degraded, and of the
#                       two candidates warn understates a machine that cannot do
#                       its job. 11 S6.2 names fault as the chassis severity.
#   fatal    -> fault   the same cell, unambiguously.
# The level itself is NOT lost: it rides verbatim in detail.level, so the cloud
# and HMI can still separate degraded from fatal.
LEVEL_TO_SEV: Dict[str, str] = {
    "warn": "warn",
    "degraded": "fault",
    "fatal": "fault",
}

# A recovery is not a fault. info is the same choice 11 S6.2 records for the
# device_online half of the device_offline/device_online pair, and like that one
# it still rides channel=alarm (derived from cat), so E-1 holds.
CLEARED_SEV = "info"

# detail.type, the sub-kind a reader branches on. Not in channel_map's override
# table on purpose: both values must keep the category default (alarm), and
# adding them there could only ever move one of them off it.
TYPE_RAISED = "chassis_fault"
TYPE_CLEARED = "chassis_fault_cleared"

# CF-4 wants the dedup_key to be the complete prefixed code, so that chs:0x1007
# and chg:0x1007 -- same number, different vendor space, different fault -- never
# merge. The key alone is not enough though: record_dao merges onto ANY still-open
# row with that key (delivered != -1, no expiry), so with an unbounded window the
# clear would merge into the raise, and a later re-raise into it as well. Both
# would vanish.
#
# 0 means "carry the key, coalesce nothing". The merge test is
# (ts - last_ts) > window, so a strictly later event never merges and an event
# with the IDENTICAL ts does.
#
# *** The window only works because every event on this key carries a ts from
# ONE clock -- the derivation moment on this machine (11 S9.8.4 rule 6). It did
# not: raises were dated from since_ts, the CHASSIS wall clock, while clears
# were dated from ours. Measured 2026-09-27: with the chassis clock behind us,
# raise(ts = chassis) -> clear(ts = ours, larger) -> raise(ts = chassis again,
# SMALLER than last_ts) made (ts - last_ts) negative, so no window could be
# exceeded and the third row was merged into the first and disappeared. A
# sequence of real recurrences persisted as one row whose dedup_count grew.
# One clock plus a monotone reading makes a recurrence strictly later, always.
#
# The duplicate the identical-ts fold would catch (the same report arriving over
# the relay path and the P1-20 path) is not reached here anyway: the second
# copy's code is already in the active set, so it is not an edge and produces no
# event at all. Edge triggering is what absorbs that pair, not the window.
#
# This is not the "0 pretending to be a calibrated value" of CLAUDE.md 3.1: it is
# not a safety parameter and it is not a stand-in for an unknown number. The
# flood that dedup_window_s exists to damp is already prevented upstream by edge
# triggering, which damps by STATE rather than by a guessed number of seconds.
DEDUP_WINDOW_S = 0


class ChassisFaultDeriver:
    """Level-triggered ChassisFault snapshots in, edge-triggered Events out.

    One instance per p5 process. The active-code set is in memory only: after a
    restart the next snapshot re-raises whatever is still faulted (new eids),
    which is the honest answer -- this process has never reported those faults.
    """

    def __init__(self, *, rid: str, eid_gen: Callable[[str, bool], str],
                 src: str = "p5_gateway") -> None:
        # rid fallback for a message whose envelope carries none; the envelope's
        # own rid wins when present (it is the authoritative one, 11 S3.0).
        self._rid = rid
        # eid_gen(code, cleared) -> a globally unique event id. Injected because
        # the boot-token scheme lives in the wiring (see p2_core main_wiring:
        # record.db outlives the process while a bare seq restarts at 0, so two
        # boots would collide on eid and the DAO would degrade the row to JSONL).
        self._eid_gen = eid_gen
        self._src = src
        # code -> since_ts of the occurrence we reported (None when the producer
        # sent none). Membership is the edge test; the value is kept so the CLEAR
        # event can still name when the fault it ends had started.
        self._active: Dict[str, Optional[float]] = {}
        self.stats: Dict[str, int] = {
            "reports": 0,        # snapshots observed
            "raised": 0,         # RAISE events emitted
            "cleared": 0,        # CLEAR events emitted
            "repeat": 0,         # faults[] entries that were not an edge
            "clear_unknown": 0,  # cleared[] codes we never reported
            "bad_code": 0,       # CF-1 malformed -> rejected entry
            "bad_level": 0,      # level outside 11 S4.1 -> rejected entry
            "bad_shape": 0,      # entry that is not an object / has no code
            "no_since_ts": 0,    # entry without a usable since_ts
            "bad_evidence": 0,   # 13 S7.3 evidence field present but mistyped
        }

    @property
    def active_codes(self) -> frozenset:
        """The codes currently reported as faulted. Exposed for the wiring's
        status line and for tests; not part of any wire payload."""
        return frozenset(self._active)

    def observe(self, envelope: Dict[str, Any], *,
                now_wall: float) -> List[Dict[str, Any]]:
        """One event/fault/chassis message -> 0..N record.db event dicts.

        envelope is the decoded 11 S3.0 message (the relay rebuilt it per
        RT-C3.e, so src is the forwarder and data is the byte-identical
        ChassisFault). A bare ChassisFault with no envelope is also accepted:
        the two are told apart by the presence of a data object.

        now_wall is injected (wall-clock seconds) rather than read here, so a
        test can assert exactly which timestamp landed in which field. It is the
        derivation moment on THIS machine and it is what every emitted event
        carries as ts (rule 6) and as created_at; only a raise's detected_at
        comes from the producer's since_ts (rule 4). No monotonic decision is
        made in this module, so CLK-C1 has nothing to say about the wall clock
        here.
        """
        self.stats["reports"] += 1
        body = envelope.get("data")
        if not isinstance(body, dict):
            # No data object -> the message IS the ChassisFault (a bare payload).
            # Accepted because the internal bus still carries bare payloads on
            # some keys; a message that is neither yields no faults and no edges.
            body = envelope
        rid = envelope.get("rid") or self._rid
        out: List[Dict[str, Any]] = []
        out.extend(self._raises(body.get("faults"), rid, now_wall))
        out.extend(self._clears(body.get("cleared"), rid, now_wall))
        return out

    # -- the two halves -------------------------------------------------------

    def _raises(self, faults: Any, rid: str,
                now_wall: float) -> List[Dict[str, Any]]:
        """faults[] -> a RAISE event for each code seen for the first time."""
        out: List[Dict[str, Any]] = []
        if not isinstance(faults, list):
            # An absent faults[] is legal (a report can be all-clear); a present
            # but non-list one is malformed and counted rather than guessed at.
            if faults is not None:
                self.stats["bad_shape"] += 1
            return out
        for entry in faults:
            code = self._code_of(entry, as_object=True)
            if code is None:
                continue
            level = self._level_of(entry)
            if level is None:
                continue
            if code in self._active:
                # Still faulted: the snapshot repeating itself, not a new fault.
                # This is also where the P1-20 second copy of a report lands.
                self.stats["repeat"] += 1
                continue
            since_ts = self._since_of(entry)
            self._active[code] = since_ts
            self.stats["raised"] += 1
            out.append(self._event(
                code=code, cleared=False, rid=rid, level=level,
                desc=self._desc_of(entry), since_ts=since_ts,
                # The 13 S7.3 evidence fields, forwarded whole. Extracted here
                # rather than inside _event so the clear half cannot reach them
                # by accident -- see _clears.
                evidence=self._evidence_of(entry),
                # detected_at is dated from the OCCURRENCE (rule 4). Only when
                # the producer supplied no usable since_ts does it fall back to
                # now -- and detail.since_ts stays null there, so the two are
                # tellable apart. ts is NOT this value: see DEDUP_WINDOW_S.
                detected_ts=since_ts if since_ts is not None else now_wall,
                now_wall=now_wall))
        return out

    def _clears(self, cleared: Any, rid: str,
                now_wall: float) -> List[Dict[str, Any]]:
        """cleared[] -> a CLEAR event for each code that was active."""
        out: List[Dict[str, Any]] = []
        if not isinstance(cleared, list):
            if cleared is not None:
                self.stats["bad_shape"] += 1
            return out
        for entry in cleared:
            code = self._code_of(entry, as_object=False)
            if code is None:
                continue
            if code not in self._active:
                # Clearing something this process never reported: after a p5
                # restart the raise belonged to the previous instance. Emitting a
                # recovery for an event the cloud has no record of would be noise,
                # and the repeat of a cleared[] entry across consecutive snapshots
                # would become a flood. Counted so it is not invisible.
                self.stats["clear_unknown"] += 1
                continue
            since_ts = self._active.pop(code)
            self.stats["cleared"] += 1
            out.append(self._event(
                code=code, cleared=True, rid=rid, level=None,
                # No desc: a cleared[] element is the code and nothing else
                # (CF-1). The text was delivered with the raise, and the
                # recovery names the same code, so nothing is lost.
                desc=None, since_ts=since_ts,
                # No evidence either, and NOT a copy of the raise's. 11 S9.8.4
                # gives cleared[] bare code STRINGS: the chassis says nothing
                # about resources or sources at recovery time, so there is
                # nothing here to forward. Re-emitting what the raise carried
                # would read as "these resources were still implicated when it
                # cleared", which is a claim no message on this key makes; the
                # raise already delivered that evidence under the same
                # dedup_key, so nothing is lost by leaving it out. The recovery
                # detail therefore holds only what a recovery can know: type,
                # code, and the since_ts of the occurrence it ends.
                evidence=None,
                # A clear has no occurrence time of its own -- cleared[] carries
                # only the code (CF-1/CF-3) -- so the recovery is dated NOW, the
                # moment the chassis told us. detail.since_ts still names when the
                # fault it ends had started, so the duration stays recoverable.
                detected_ts=now_wall, now_wall=now_wall))
        return out

    # -- field extraction -----------------------------------------------------

    def _code_of(self, entry: Any, *, as_object: bool) -> Optional[str]:
        """The CF-1 well-formed code of one list element, or None (rejected).

        11 S9.8.4 gives the two lists DIFFERENT element types, and which one is
        expected is passed in rather than sniffed from the value:
          faults[]  -- objects carrying the code in a `code` field;
          cleared[] -- bare code strings. CF-1 puts the regex on "cleared[] 的
                       每一个元素", i.e. the element IS the code, and 13 S7.3
                       CF-3 says the same ("用与发生时逐字相同的带前缀串").

        quadruped shipped cleared[] as objects until 2026-09-27 (one writer
        served both lists) and this method read that form too. The producer is
        fixed (rt_payloads.cc write_cleared_array) and the reader went with it: a
        sniffing reader leaves two wire shapes legal with nothing left to
        decide between them, and the next divergence has nowhere to show up.
        An element of the wrong type is counted as bad_shape, and the rest of
        the report still goes through.
        """
        if as_object:
            if not isinstance(entry, dict):
                self.stats["bad_shape"] += 1
                return None
            raw = entry.get("code")
            if not isinstance(raw, str):
                self.stats["bad_shape"] += 1
                return None
            code = raw
        else:
            if not isinstance(entry, str):
                self.stats["bad_shape"] += 1
                return None
            code = entry
        if not is_wellformed_fault_code(code):
            # CF-1 -> E_SCHEMA for THIS entry. Counted, logged, and the rest of
            # the report continues (13 S6.5 forbid #2: one bad code must not cost
            # the others). No degrade-to-a-space: the prefix is what says which
            # vendor code table the number belongs to.
            self.stats["bad_code"] += 1
            _logger.warning(
                "chassis fault code %r fails CF-1 (11 S9.8.4); entry rejected, "
                "report kept", code)
            return None
        return code

    def _level_of(self, entry: Any) -> Optional[str]:
        """faults[].level, validated against the 11 S4.1 closed set.

        A value outside it is rejected (CLAUDE.md 3.5: no silent pass-through and
        no "interpret the unknown value as something known"), counted, and only
        this entry is lost. Note the direction is the opposite of the code rule
        above: a code is an OPEN set (13 QD-6, the vendor table is incomplete),
        while level is a CLOSED three-value set that quadruped itself derives
        (13 S7.3 severity_to_level), so an unexpected value means the two sides
        disagree about the contract, not that the chassis found a new fault.
        """
        if not isinstance(entry, dict):
            self.stats["bad_shape"] += 1
            return None
        level = entry.get("level")
        if level not in FAULT_LEVEL:
            self.stats["bad_level"] += 1
            _logger.warning(
                "chassis fault level %r not in the 11 S4.1 closed set %s; "
                "entry rejected", level, sorted(FAULT_LEVEL))
            return None
        return level

    def _since_of(self, entry: Any) -> Optional[float]:
        """faults[].since_ts (wall-clock seconds), the fault's own start time.

        Returns None when the producer supplied nothing usable, and says so in
        the counter -- a fault is never dropped over a missing timestamp (the
        code is the load-bearing part), but None is never quietly turned into a
        number that looks measured: the caller dates that event from now and
        detail.since_ts stays null, so a reader can tell the two apart.
        """
        if isinstance(entry, dict):
            raw = entry.get("since_ts")
            if isinstance(raw, (int, float)) and not isinstance(raw, bool):
                return float(raw)
        # null is a shape the producer emits on purpose: quadruped publishes
        # since_ts = null for a fault the chassis sent no Timestamp with, so
        # that this counter -- and not a 1970 detected_at -- is what a reader
        # sees. Until 2026-09-27 quadruped sent since:{sec,nanosec} here and
        # this method converted it; that reader went out with the producer fix
        # (rt_payloads.cc write_fault_array now writes float seconds).
        self.stats["no_since_ts"] += 1
        return None

    def _desc_of(self, entry: Any) -> Optional[str]:
        """faults[].desc, the human-readable line. None when absent.

        One field, no fallbacks. quadruped wrote `name` here until 2026-09-27
        while 13 v1.35 had already corrected the same field one key over
        (rt/chassis/state: "faults 条目第三键按契约示例是 desc"); the producer
        now spells desc on both keys from the same value (CF-5), so reading
        `name` or `details` as alternatives would only keep a second wire
        shape alive with nothing left to produce it.
        """
        if not isinstance(entry, dict):
            return None
        value = entry.get("desc")
        return value if isinstance(value, str) and value else None

    def _evidence_of(self, entry: Any) -> Dict[str, Any]:
        """The five 13 S7.3 evidence fields of one faults[] entry.

        13 S7.3's disposition table marks details / grouped / resources /
        source / source_ids upstream with one reason written against the whole
        group -- "xian chang ding wei kao ta", field localisation depends on
        them. quadruped forwards all five on event/fault/chassis
        (rt_payloads.cc write_fault_array) and 11 S9.8.4 registers them there as
        our extension; until 2026-09-27 this deriver read only the four keys
        the derivation rules name, so the chain was complete and the evidence
        still stopped at p5: record.db, the cloud and the HMI saw a code with
        no locator. A fault chain whose purpose is field localisation that
        drops the localisation is the 3.2 failure mode -- the link is up and
        the payload is not there to tell you.

        ABSENT AND EMPTY ARE DIFFERENT ANSWERS, and this method keeps them so.
        A key the producer did not send is left OUT of detail entirely; an
        empty array the producer DID send lands as []. The first says "the
        chassis was not asked / this producer does not carry the field", the
        second says "the chassis was asked and named nothing". Collapsing them
        (defaulting the absent key to [] here) would let a reader conclude the
        chassis reported no resources when in fact nobody ever looked, and the
        conclusion would be indistinguishable from the true one -- the same
        shape 11 S14.3's source_ids row records for that field's own absence.

        A key that IS present but has the wrong type is dropped and counted.
        The entry itself survives: these five are evidence, not identity, and
        13 S6.5 forbid #2 (one bad member must not cost the others) applies
        with more force here than it does to a malformed code -- the fault is
        real and the operator still needs to hear about it.

        Sizing: no contract anywhere bounds Event.detail. 11 S6.1 gives the
        field no length rule, 17 BL-3 only ESTIMATES an average event at
        backfill.avg_event_bytes for a backlog gauge and explicitly refuses to
        measure the column. So `details` -- free-form vendor text with no
        schema (13 S7.3 says so in as many words) -- is forwarded verbatim and
        NOT truncated: a truncation rule invented here would be a number
        nothing licenses, and a silently clipped diagnostic string is worse
        than a long one. If a bound is ever contracted, it belongs in 11 S6.1
        for every producer at once, not in this one deriver.
        """
        out: Dict[str, Any] = {}
        if not isinstance(entry, dict):
            # Unreachable from _raises (the code reader already rejected a
            # non-object), kept so the method is total on its own input.
            return out
        if "details" in entry:
            value = entry["details"]
            if isinstance(value, str):
                # Empty string included on purpose: the chassis sent the field
                # and it was blank, which is not the same as not sending it.
                out["details"] = value
            else:
                self.stats["bad_evidence"] += 1
        if "grouped" in entry:
            value = entry["grouped"]
            # bool first and bool only: 1 / 0 would satisfy an int test and
            # land a number in a field a reader branches on as a flag.
            if isinstance(value, bool):
                out["grouped"] = value
            else:
                self.stats["bad_evidence"] += 1
        # Three string arrays, same rule for each. source is the MODULE and
        # source_ids the INSTANCE (11 S14.3, 2026-09-27): on a chassis running
        # several instances of one module, source alone cannot say which one
        # faulted, which is the question the field engineer actually has.
        for key in ("resources", "source", "source_ids"):
            if key not in entry:
                continue
            value = entry[key]
            if isinstance(value, list) and all(
                    isinstance(item, str) for item in value):
                # list() and not the reference: detail is handed to the
                # pipeline, record_dao and the cloud relay, and a caller's
                # list aliased into three of them is one mutation away from
                # three disagreeing copies of one report.
                out[key] = list(value)
            else:
                self.stats["bad_evidence"] += 1
        return out

    # -- assembly -------------------------------------------------------------

    def _event(self, *, code: str, cleared: bool, rid: str,
               level: Optional[str], desc: Optional[str],
               since_ts: Optional[float], evidence: Optional[Dict[str, Any]],
               detected_ts: float, now_wall: float) -> Dict[str, Any]:
        """One 11 S6.1 Event in the record.db dict shape the pipeline validates.

        channel is deliberately absent: step 3 derives it from cat and OVERWRITES
        whatever a producer supplied (17 S3.3), so writing one here could only
        ever be a lie that happens to agree.
        """
        sev = CLEARED_SEV if cleared else LEVEL_TO_SEV[level or ""]
        detail: Dict[str, Any] = {
            "type": TYPE_CLEARED if cleared else TYPE_RAISED,
            # The complete prefixed code, never the bare number: the prefix is
            # what makes chs:0x1007 and chg:0x1007 two different faults (CF-4).
            "code": code,
            # null when the producer sent none -- see _since_of. A number here is
            # always one the chassis measured, never the now-fallback.
            "since_ts": since_ts,
        }
        if level is not None:
            # Kept verbatim so the degraded / fatal distinction that LEVEL_TO_SEV
            # collapses is still readable downstream.
            detail["level"] = level
        if desc:
            detail["desc"] = desc
        if evidence:
            # The five 13 S7.3 locator fields, already filtered by
            # _evidence_of: every key in here was present on the wire and well
            # typed, so update() cannot introduce a default. None (the clear
            # half) and {} (a producer that sent none of the five) both leave
            # detail exactly as it was -- absence stays absence.
            detail.update(evidence)
        return {
            "eid": self._eid_gen(code, cleared),
            "rid": rid,
            "cat": CHASSIS_CAT,
            "sev": sev,
            "title": ("chassis fault cleared %s" if cleared
                      else "chassis fault %s") % code,
            "detail": detail,
            "src": self._src,
            # 11 S9.8.4 rule 6: the DERIVATION moment, on this machine's wall
            # clock -- the same clock every other event p5 emits is stamped from
            # (_normalise_event's `now`), and never since_ts. It is also the
            # dedup comparison value (record_dao: ts - last_ts), and that is the
            # whole reason: comparing a chassis-clock raise against an
            # upper-stack clear compares two clocks, and the sign of the
            # difference is then the clock offset rather than the ordering. See
            # DEDUP_WINDOW_S for what that cost. The occurrence time is not
            # lost -- it is detected_at, and detail.since_ts verbatim.
            "ts": now_wall,
            # 11 CLK-A2: ts_sync is copied from the producer or false, never
            # judged here. The derived event carries no sync claim of its own.
            "ts_sync": 0,
            # Evidence time (the record.db column's own words) -- WHEN THE FAULT
            # HAPPENED, from since_ts, not when p5 read the snapshot. A snapshot
            # re-announces a 40 s old fault in every report, so "now" would
            # backdate nothing and mislabel everything. This is the ONE field
            # 11 S9.8.4 rule 4 binds to since_ts; it is a display/audit string
            # and nothing compares it, which is why the chassis clock is safe
            # here and not in ts.
            "detected_at": _fmt_wall(detected_ts),
            # Write time, UTC, ISO. WALL-CLOCK-OK(record): display and audit only.
            "created_at": datetime.fromtimestamp(
                now_wall, timezone.utc).isoformat(),
            # CF-4: the dedup key is the complete prefixed code. See
            # DEDUP_WINDOW_S for why the window is 0 and not absent.
            "dedup_key": code,
            "dedup_window_s": DEDUP_WINDOW_S,
        }

def _fmt_wall(ts: float) -> str:
    """Wall-clock seconds -> the record.db detected_at spelling.

    Same format the p5 wiring's _normalise_event uses for every other event, so
    the column stays sortable as text across producers.
    """
    return datetime.fromtimestamp(ts, timezone.utc).strftime("%Y-%m-%d %H:%M:%S")


# Import-time guard on the two severities this module can produce. A SEVERITY
# rename would otherwise surface as a pipeline drop on a live chassis fault --
# the one message that must never be dropped quietly.
assert CLEARED_SEV in SEVERITY, "11 S6.1 severity set lost 'info'"
assert set(LEVEL_TO_SEV) == set(FAULT_LEVEL), (
    "LEVEL_TO_SEV must cover exactly the 11 S4.1 fault_level set")
assert set(LEVEL_TO_SEV.values()) <= set(SEVERITY), (
    "LEVEL_TO_SEV maps outside the 11 S6.1 severity set")
