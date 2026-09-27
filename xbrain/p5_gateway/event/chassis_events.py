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
# with the IDENTICAL ts does -- which is precisely the duplicate we want folded
# (the same occurrence arriving twice, e.g. over the relay path and the P1-20
# path), because ts is since_ts, the occurrence's own time.
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
            "legacy_shape": 0,   # producer spelling that 11 S9.8.4 does not define
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
        test can assert exactly which timestamp landed in which field. It is used
        ONLY for created_at and for a clear's detected_at; every raise dates
        itself from since_ts. No monotonic decision is made in this module, so
        CLK-C1 has nothing to say about the wall clock here.
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
            code = self._code_of(entry)
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
                # The raise is dated from the occurrence. Only when the producer
                # supplied no usable since_ts does it fall back to now -- and
                # detail.since_ts stays null there, so the two are tellable apart.
                ts=since_ts if since_ts is not None else now_wall,
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
            code = self._code_of(entry)
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
                desc=self._desc_of(entry), since_ts=since_ts,
                # A clear has no occurrence time of its own -- cleared[] carries
                # only the code (CF-1/CF-3) -- so the recovery is dated NOW, the
                # moment the chassis told us. detail.since_ts still names when the
                # fault it ends had started, so the duration stays recoverable.
                ts=now_wall, now_wall=now_wall))
        return out

    # -- field extraction -----------------------------------------------------

    def _code_of(self, entry: Any) -> Optional[str]:
        """The CF-1 well-formed code of one entry, or None (entry rejected).

        11 S9.8.4 gives faults[] entries as objects with a code field and
        cleared[] entries as bare code strings (CF-1: "cleared[] 的每一个元素
        都必须形如 ^(chs|chg):0x[0-9A-Fa-f]{4}$", i.e. the element IS the code).
        Both are accepted here.

        ! quadruped currently writes cleared[] as OBJECTS (rt_payloads.cc
        WriteFaultList is called for both lists), which CF-1 does not allow. The
        object form is read rather than rejected -- refusing it would swallow
        every real recovery, the exact all-green-while-faulted outcome 13 S7.3
        warns about -- but it is counted as legacy_shape and logged, because a
        divergence nobody can see is one nobody fixes.
        """
        if isinstance(entry, str):
            code = entry
        elif isinstance(entry, dict):
            raw = entry.get("code")
            if not isinstance(raw, str):
                self.stats["bad_shape"] += 1
                return None
            code = raw
        else:
            self.stats["bad_shape"] += 1
            return None
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
        (13 S7.3 SeverityToLevel), so an unexpected value means the two sides
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
            # ! quadruped writes since:{sec,nanosec} instead of the since_ts
            # 13 S7.3 requires ("Timestamp{Sec,Nanosec} -> 转 since_ts").
            # Converted rather than ignored, for the same reason as the cleared[]
            # shape above, and counted the same way.
            since = entry.get("since")
            if isinstance(since, dict):
                sec = since.get("sec")
                nsec = since.get("nanosec")
                if isinstance(sec, (int, float)) and not isinstance(sec, bool):
                    self.stats["legacy_shape"] += 1
                    extra = (float(nsec) / 1e9
                             if isinstance(nsec, (int, float))
                             and not isinstance(nsec, bool) else 0.0)
                    return float(sec) + extra
        self.stats["no_since_ts"] += 1
        return None

    def _desc_of(self, entry: Any) -> Optional[str]:
        """faults[].desc, the human-readable line. None when absent.

        ! quadruped writes name + details and no desc on this key, although
        13 v1.35 already corrected the SAME field one key over (rt/chassis/state:
        "faults 条目第三键按契约示例是 desc"). name is read as the fallback
        because 13 S7.3 calls it "未登记码唯一的可读线索" -- losing it would
        leave an unregistered code with nothing but a number.
        """
        if not isinstance(entry, dict):
            return None
        for field in ("desc", "name", "details"):
            value = entry.get(field)
            if isinstance(value, str) and value:
                if field != "desc":
                    self.stats["legacy_shape"] += 1
                return value
        return None

    # -- assembly -------------------------------------------------------------

    def _event(self, *, code: str, cleared: bool, rid: str,
               level: Optional[str], desc: Optional[str],
               since_ts: Optional[float], ts: float,
               now_wall: float) -> Dict[str, Any]:
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
        return {
            "eid": self._eid_gen(code, cleared),
            "rid": rid,
            "cat": CHASSIS_CAT,
            "sev": sev,
            "title": ("chassis fault cleared %s" if cleared
                      else "chassis fault %s") % code,
            "detail": detail,
            "src": self._src,
            # The event's wall-clock stamp: the occurrence time for a raise, the
            # observation time for a clear. It is also the dedup comparison value
            # (record_dao: ts - last_ts), which is why the same occurrence
            # arriving twice folds and a new occurrence does not.
            "ts": ts,
            # 11 CLK-A2: ts_sync is copied from the producer or false, never
            # judged here. The derived event carries no sync claim of its own.
            "ts_sync": 0,
            # Evidence time (the record.db column's own words) -- WHEN THE FAULT
            # HAPPENED, from since_ts, not when p5 read the snapshot. A snapshot
            # re-announces a 40 s old fault in every report, so "now" would
            # backdate nothing and mislabel everything.
            "detected_at": _fmt_wall(ts),
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
