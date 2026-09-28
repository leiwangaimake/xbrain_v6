"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chassis_events.py
Brief: 11 S9.8.4 ChassisFault -> 11 S6.1 Event derivation (edge trigger + CF-1 + CF-4)

Description:
What these pin, and why each one is here rather than being implied by the others:

  * the EDGE, in both directions -- a new code produces exactly one Event, the
    same code repeating in the 2 Hz snapshot produces none, and the code landing
    in cleared[] produces exactly one recovery. The repeat case is the one that
    a "just forward every report" implementation passes every other test with,
    so it is asserted by count, not by "an event came out".
  * detected_at from since_ts, NOT from observation time. A snapshot re-announces
    a fault that started long ago; an implementation that stamps now passes every
    shape assertion and silently backdates nothing, so this needs its own check
    with the two times deliberately far apart.
  * ts from the DERIVATION moment and never from since_ts (11 S9.8.4 rule 6),
    driven through the real RecordDao: with the two clocks mixed a raise ->
    clear -> re-raise sequence lost its third row to the dedup merge whenever
    the chassis clock ran behind ours. No assertion on a ts VALUE can see that,
    so the sequence is persisted and the rows are counted.
  * dedup_key is the COMPLETE prefixed code (CF-4) and the two vendor spaces do
    not merge -- chs:0x1007 and chg:0x1007 must be two independent faults.
  * a CF-1 malformed code is rejected and COUNTED, and the rest of the report
    survives it (13 S6.5 forbid #2: one bad entry must not cost the others).
  * the two lists have DIFFERENT element types and each is required, not
    sniffed: faults[] objects, cleared[] bare code strings. quadruped served
    both with one writer until 2026-09-27 and this module read whatever came;
    producer and reader were fixed together, so both wrong-type directions are
    now asserted -- a sniffing reader leaves two wire shapes legal with nothing
    left to choose between them.
  * the five 13 S7.3 evidence fields (details / grouped / resources / source /
    source_ids) reach detail, and ABSENT stays apart from EMPTY. Both halves
    need their own test: an implementation that defaults the absent keys to
    [] / false / "" passes every "the field is there" assertion while telling
    every reader the chassis was asked and named nothing. The forwarding half
    had no assertion at all until 2026-09-27 and the fields were silently
    dropped -- nothing could go red, because a detail that was never written
    and a chassis with nothing to say look identical.
  * the pipeline accepts what comes out. The derivation's whole purpose is that
    EventPipeline._validate stops dropping these, so one test runs a derived
    event through the real _REQUIRED_FIELDS check instead of restating the list.

Mutations verified red on 2026-09-27 (3.3), each named at its assertion.
"""

import pytest

from xbrain.p5_gateway.event.channel_map import derive_channel
from xbrain.p5_gateway.event.chassis_events import ChassisFaultDeriver
from xbrain.p5_gateway.event.pipeline import _REQUIRED_FIELDS

pytestmark = pytest.mark.no_device


# A fault that started well before any observation time used below, so a
# detected_at taken from "now" cannot accidentally equal the one from since_ts.
SINCE = 1753660812.0
NOW = 1753669999.0


def _deriver():
    """A deriver whose eid is a pure function of (code, cleared) so a test can
    predict it; the real one carries a boot token + seq (record.db outlives the
    process, so a bare seq would collide across restarts)."""
    seq = [0]

    def eid(code, cleared):
        seq[0] += 1
        return "e-%s-%s-%d" % (code, "clr" if cleared else "set", seq[0])

    return ChassisFaultDeriver(rid="m20s", eid_gen=eid)


def _report(faults=(), cleared=()):
    """One 11 S3.0 envelope carrying a ChassisFault, as chassis_relay CR-9
    rebuilds it (src is the forwarder, data is byte-identical)."""
    return {"v": 1, "rid": "m20s", "ts": NOW, "seq": 7,
            "src": "chassis_relay", "ts_sync": False,
            "data": {"faults": list(faults), "cleared": list(cleared)}}


def _fault(code="chs:0x8001", level="fatal", desc="joint over limit",
           since_ts=SINCE):
    return {"code": code, "level": level, "desc": desc, "since_ts": since_ts}


# -- the edge, both directions ------------------------------------------------


def test_a_new_code_raises_exactly_one_event():
    d = _deriver()
    out = d.observe(_report([_fault()]), now_wall=NOW)
    assert len(out) == 1
    ev = out[0]
    assert ev["cat"] == "chassis"
    # fatal -> fault (LEVEL_TO_SEV). mutant: map fatal to warn -> red.
    assert ev["sev"] == "fault"
    assert ev["detail"]["type"] == "chassis_fault"
    assert ev["detail"]["code"] == "chs:0x8001"
    # The level survives the collapse into the 4-value severity set.
    assert ev["detail"]["level"] == "fatal"
    assert ev["detail"]["desc"] == "joint over limit"
    assert ev["src"] == "p5_gateway"
    assert ev["rid"] == "m20s"
    assert d.stats["raised"] == 1


def test_the_same_code_repeating_in_the_snapshot_produces_nothing():
    # ChassisFault is LEVEL triggered: at 2 Hz the same code arrives again and
    # again while the fault persists. mutant: drop the `code in self._active`
    # guard in _raises -> this goes from 0 to 3 events and record.db takes two
    # rows per second per fault.
    d = _deriver()
    assert len(d.observe(_report([_fault()]), now_wall=NOW)) == 1
    assert d.observe(_report([_fault()]), now_wall=NOW + 0.5) == []
    assert d.observe(_report([_fault()]), now_wall=NOW + 1.0) == []
    assert d.stats["raised"] == 1 and d.stats["repeat"] == 2


def test_a_second_copy_of_one_report_is_not_a_second_event():
    # 11 S2.2.1 registers TWO subscribers of rt/chassis/fault on purpose
    # (chassis_relay normally, p1_motion P1-20 when the relay is dead), so p5
    # sees each report twice in normal operation. The edge absorbs it.
    d = _deriver()
    report = _report([_fault()])
    assert len(d.observe(report, now_wall=NOW)) == 1
    assert d.observe(report, now_wall=NOW) == []


def test_cleared_raises_exactly_one_recovery_event():
    d = _deriver()
    d.observe(_report([_fault()]), now_wall=NOW)
    out = d.observe(_report(cleared=["chs:0x8001"]), now_wall=NOW + 60.0)
    assert len(out) == 1
    ev = out[0]
    assert ev["detail"]["type"] == "chassis_fault_cleared"
    # A recovery is not a fault. mutant: emit sev=fault for the clear -> red
    # (and the cloud would show a machine that just recovered as faulted).
    assert ev["sev"] == "info"
    # It still names when the fault it ends had started, so the duration is
    # recoverable from the pair.
    assert ev["detail"]["since_ts"] == SINCE
    # ... but the recovery itself is dated when the chassis said so: cleared[]
    # carries only a code (CF-1/CF-3), there is no clear-time on the wire.
    assert ev["ts"] == NOW + 60.0
    assert d.stats["cleared"] == 1
    # The code left the active set, so the same fault can raise again later.
    assert d.active_codes == frozenset()


def test_clearing_a_code_never_reported_emits_nothing_but_counts():
    # After a p5 restart the raise belonged to the previous instance. A recovery
    # for an event the cloud has no record of is noise, and cleared[] repeating
    # across snapshots would make it a flood.
    d = _deriver()
    assert d.observe(_report(cleared=["chs:0x8001"]), now_wall=NOW) == []
    assert d.stats["clear_unknown"] == 1


def test_a_cleared_code_can_raise_again():
    # The failure this guards: a dedup_key with an open-ended window merges the
    # re-raise into the original row, so the cloud is left believing the fault
    # is still cleared. DEDUP_WINDOW_S = 0 plus the edge reset is what prevents
    # it. mutant: leave the code in _active on clear -> the second raise is 0.
    d = _deriver()
    d.observe(_report([_fault()]), now_wall=NOW)
    d.observe(_report(cleared=["chs:0x8001"]), now_wall=NOW + 60.0)
    again = d.observe(_report([_fault(since_ts=SINCE + 500.0)]),
                      now_wall=NOW + 120.0)
    assert len(again) == 1 and again[0]["detail"]["type"] == "chassis_fault"
    # Two raises of the same code carry DIFFERENT ts, so record_dao's
    # (ts - last_ts) > 0 test refuses to merge them. The ts is the DERIVATION
    # moment (11 S9.8.4 rule 6), not since_ts -- see the three-row test below
    # for what dating it from the chassis clock costs.
    assert again[0]["ts"] == NOW + 120.0


# -- timestamps ---------------------------------------------------------------


def test_detected_at_comes_from_since_ts_not_from_observation_time():
    # The snapshot property: this fault started ~2.5 h before we looked at it.
    # mutant: stamp detected_at from now_wall -> the second assert red.
    d = _deriver()
    ev = d.observe(_report([_fault()]), now_wall=NOW)[0]
    assert ev["detected_at"] == "2025-07-28 00:00:12"
    # ... and ts is the OTHER clock: the derivation moment on this machine
    # (11 S9.8.4 rule 6). The two are deliberately far apart here, so an
    # implementation that stamps either one from the other is red.
    # mutant: ts = since_ts -> red (and see the three-row test below).
    assert ev["ts"] == NOW
    assert ev["ts"] != SINCE
    # created_at is the WRITE time, same clock as ts and the same instant here.
    assert ev["created_at"].startswith("2025-07-28T02:33:19")


def test_a_missing_since_ts_is_not_fabricated_into_detail():
    # QD-6: a real fault is never swallowed over a missing timestamp, but the
    # fallback must not present itself as measured.
    d = _deriver()
    entry = {"code": "chs:0x8001", "level": "warn"}
    ev = d.observe(_report([entry]), now_wall=NOW)[0]
    assert ev["ts"] == NOW                  # dated from now, so the row sorts
    assert ev["detail"]["since_ts"] is None  # ... but detail does not claim it
    assert d.stats["no_since_ts"] == 1


@pytest.mark.asyncio
async def test_a_recurrence_survives_record_dao_when_the_chassis_clock_is_behind(
        tmp_path):
    """raise -> clear -> same code raises again: THREE rows in record.db.

    This is the whole reason 11 S9.8.4 grew rule 6. The deriver's ts is the
    record_dao merge comparison value ((ts - last_ts) > dedup_window_s) and the
    dedup_key is the same prefixed code for all three events, so the three ts
    values have to be strictly increasing or a row is silently absorbed.

    Dating a raise from since_ts broke that, and the clock offset decided the
    outcome rather than the ordering:
      1. raise  ts = since_ts  = CHASSIS clock  (small)
      2. clear  ts = now_wall  = OUR clock      (large) -> inserted, last_ts big
      3. raise  ts = since_ts  = CHASSIS clock  (small) -> ts - last_ts < 0,
         which is <= any window, so it MERGED into row 2 and never existed.
    The operator saw one fault row whose dedup_count went up, i.e. the second
    occurrence of a real chassis fault was not in record.db at all -- and with
    a chassis clock AHEAD of ours the same code would have passed, which is the
    kind of test-passes-on-my-bench that this one exists to remove.

    It is driven through the REAL RecordDao rather than by comparing ts values,
    because the defect is not in a ts value on its own: it is in what the DAO
    does with three of them. An assertion on ts alone cannot see a merge.
    mutant: put back ts = since_ts in _raises -> the row count is 2, not 3.
    """
    import aiosqlite

    from xbrain.p5_gateway.persistence.base import RecordConn
    from xbrain.p5_gateway.persistence.record_dao import RecordDao
    from xbrain.p5_gateway.persistence.schema_record import ALL_RECORD_STATEMENTS

    d = _deriver()
    # The chassis is ~2.5 h BEHIND this machine (SINCE vs NOW): the sign that
    # used to decide whether a recurrence was kept.
    assert SINCE < NOW
    events = []
    events += d.observe(_report([_fault()]), now_wall=NOW)
    events += d.observe(_report(cleared=["chs:0x8001"]), now_wall=NOW + 60.0)
    events += d.observe(_report([_fault(since_ts=SINCE + 500.0)]),
                        now_wall=NOW + 120.0)
    assert len(events) == 3            # the deriver's own edges are unchanged

    async with aiosqlite.connect(":memory:", isolation_level=None) as c:
        for stmt in ALL_RECORD_STATEMENTS:
            await c.execute(stmt)
        # One connection in all three roles: this asserts DAO LOGIC, and
        # :memory: is per-connection so three would be three databases (same
        # arrangement as tests/p5_gateway/persistence/test_record_dao.py).
        dao = RecordDao(RecordConn(role="writer_normal", path=":memory:", conn=c),
                        RecordConn(role="writer_full", path=":memory:", conn=c),
                        RecordConn(role="reader", path=":memory:", conn=c),
                        jsonl_path=str(tmp_path / "degrade.jsonl"))
        results = []
        for ev in events:
            # channel is derived by the pipeline, not by the deriver (17 S3.3);
            # supplied here exactly the way the pipeline supplies it.
            ev = dict(ev, channel=derive_channel(ev["cat"], ev["detail"]))
            results.append(await dao.insert_event(ev))

        # *** the load-bearing assertion. "merged" for any of the three is the
        # bug: a merge writes no row and the occurrence is gone.
        assert [r.status for r in results] == ["inserted"] * 3
        cur = await c.execute("SELECT COUNT(*) FROM events WHERE dedup_key = ?",
                              ("chs:0x8001",))
        assert (await cur.fetchone())[0] == 3
        # And no row absorbed a sibling: three rows each counted once.
        cur = await c.execute(
            "SELECT dedup_count FROM events WHERE dedup_key = ? ORDER BY id",
            ("chs:0x8001",))
        assert [row[0] for row in await cur.fetchall()] == [1, 1, 1]

    # detected_at still comes from the CHASSIS clock (rule 4 is untouched) while
    # ts comes from ours -- the two clocks coexist, they just stop being
    # compared with each other.
    assert events[0]["detected_at"] == "2025-07-28 00:00:12"      # SINCE
    assert events[2]["detail"]["since_ts"] == SINCE + 500.0
    assert [ev["ts"] for ev in events] == [NOW, NOW + 60.0, NOW + 120.0]


# -- CF-1 / CF-4 --------------------------------------------------------------


def test_dedup_key_is_the_complete_prefixed_code():
    d = _deriver()
    ev = d.observe(_report([_fault()]), now_wall=NOW)[0]
    # CF-4 verbatim: dedup_key 用完整带前缀的 code. mutant: strip the prefix
    # (code.split(":")[1]) -> red here and the two spaces would merge below.
    assert ev["dedup_key"] == "chs:0x8001"


def test_the_two_vendor_spaces_are_two_independent_faults():
    # CF-4: same number, different space, must not merge. 0x1007 means
    # "充电桩无电流" in the chg space and is undefined in the chs space.
    d = _deriver()
    out = d.observe(_report([_fault(code="chs:0x1007", level="degraded"),
                             _fault(code="chg:0x1007", level="degraded")]),
                    now_wall=NOW)
    assert len(out) == 2
    assert {e["dedup_key"] for e in out} == {"chs:0x1007", "chg:0x1007"}
    # degraded -> fault: 13 S7.3 defines degraded as affecting operation, and
    # SEVERITY has no degraded of its own. mutant: map degraded to warn -> red.
    assert {e["sev"] for e in out} == {"fault"}


@pytest.mark.parametrize("bad", ["0x8001", "CHS:0x8001", "chs:0x80012",
                                 "xchs:0x8001y", "chs:8001", ""])
def test_a_malformed_code_is_rejected_and_counted(bad):
    # CF-1: no prefix / prefix outside {chs, chg} / mixed case -> E_SCHEMA, and
    # explicitly NOT "assume one of the two spaces".
    d = _deriver()
    assert d.observe(_report([_fault(code=bad)]), now_wall=NOW) == []
    assert d.stats["bad_code"] == 1
    assert d.stats["raised"] == 0


def test_a_malformed_code_does_not_cost_the_other_entries():
    # 13 S6.5 forbid #2 one layer up: a bad entry must not abort the report.
    # mutant: return [] from _raises on the first bad code -> the good fault
    # disappears and HMI shows all-green while the machine is faulted.
    d = _deriver()
    out = d.observe(_report([_fault(code="0x8001"), _fault(code="chs:0x8002")]),
                    now_wall=NOW)
    assert [e["detail"]["code"] for e in out] == ["chs:0x8002"]
    assert d.stats["bad_code"] == 1 and d.stats["raised"] == 1


def test_a_level_outside_the_closed_set_is_rejected_and_counted():
    # CLAUDE.md 3.5: no silent pass-through, no "interpret the unknown value as
    # something known". level is a CLOSED set that quadruped itself derives
    # (13 S7.3), unlike the code, which is open.
    d = _deriver()
    assert d.observe(_report([_fault(level="fail")]), now_wall=NOW) == []
    assert d.stats["bad_level"] == 1


# -- the two element types of 11 S9.8.4, each required ------------------------


def test_cleared_entries_are_code_strings_and_objects_are_rejected():
    # The two lists have DIFFERENT element types: faults[] objects, cleared[]
    # bare code strings (CF-1 puts the regex on "每一个元素"). quadruped served
    # both lists with one writer until 2026-09-27 and p5 read the object form;
    # both sides were fixed together, so the object form is now a divergence and
    # is counted rather than read.
    # mutant: sniff the type again (accept a dict here) -> red.
    d = _deriver()
    d.observe(_report([_fault()]), now_wall=NOW)
    out = d.observe(_report(cleared=[{"code": "chs:0x8001", "level": "fatal"}]),
                    now_wall=NOW + 1.0)
    assert out == []
    assert d.stats["bad_shape"] == 1
    # ...and the fault is still held open, because nothing said it cleared.
    assert d.active_codes == frozenset({"chs:0x8001"})
    # The contract form does clear it, from the same deriver.
    assert len(d.observe(_report(cleared=["chs:0x8001"]), now_wall=NOW + 2)) == 1


def test_faults_entries_must_be_objects():
    # The other direction: a bare string in faults[] has no level and no
    # since_ts, so reading it as a code would raise an event with neither.
    d = _deriver()
    assert d.observe(_report(["chs:0x8001"]), now_wall=NOW) == []
    assert d.stats["bad_shape"] == 1
    # WHICH counter moves is the assertion, not just that the entry is lost. A
    # reader that sniffed the element type would take this malformed string as
    # a code and file it under bad_code -- and bad_code means "the producer
    # built a code wrong", while bad_shape means "the producer put the wrong
    # KIND of thing in this list". Those point at different bugs, and with the
    # sniffing reader the second one can never be reported at all.
    assert d.observe(_report(["0x8001"]), now_wall=NOW) == []
    assert d.stats["bad_shape"] == 2 and d.stats["bad_code"] == 0


def test_since_ts_is_read_as_float_seconds_only():
    # 13 S7.3 verbatim: Timestamp{Sec,Nanosec} -> 转 since_ts, and the producer
    # now does that conversion (rt_payloads.cc write_fault_array). The nested pair
    # is no longer produced and no longer read: an entry carrying it has no
    # usable occurrence time, which is the no_since_ts case.
    # mutant: restore the since:{sec,nanosec} reader -> red.
    d = _deriver()
    entry = {"code": "chs:0x8001", "level": "warn", "name": "batt low",
             "since": {"sec": 1753660812, "nanosec": 500000000}}
    ev = d.observe(_report([entry]), now_wall=NOW)[0]
    assert ev["detail"]["since_ts"] is None
    assert d.stats["no_since_ts"] == 1
    # `name` is not a desc either -- one field, no fallbacks (the producer
    # spells desc on both keys from the same value, CF-5).
    assert "desc" not in ev["detail"]


def test_since_ts_null_is_a_shape_the_producer_emits_on_purpose():
    # quadruped publishes since_ts = null for a fault the chassis sent no
    # Timestamp with. The fault is NOT dropped over it (the code is the
    # load-bearing part) and detail.since_ts stays null, so a reader can tell a
    # measured occurrence time from the fallback to observation time.
    d = _deriver()
    entry = {"code": "chs:0x8001", "level": "fatal", "desc": "no clock",
             "since_ts": None}
    ev = d.observe(_report([entry]), now_wall=NOW)[0]
    assert ev["detail"]["since_ts"] is None
    assert ev["ts"] == NOW
    assert d.stats["no_since_ts"] == 1 and d.stats["raised"] == 1


# -- the point of the whole module -------------------------------------------


def test_a_derived_event_satisfies_the_pipeline_required_fields():
    # This is the defect being fixed: before the derivation the ChassisFault
    # reached EventPipeline._validate with no eid and came back
    # dropped: missing_field:eid, i.e. every chassis fault was discarded.
    d = _deriver()
    for ev in (d.observe(_report([_fault()]), now_wall=NOW)
               + d.observe(_report(cleared=["chs:0x8001"]), now_wall=NOW + 1)):
        missing = [f for f in _REQUIRED_FIELDS if ev.get(f) is None]
        assert missing == [], "derived event is missing %s" % missing


def test_the_wiring_routes_the_chassis_key_into_the_deriver():
    """The derivation is worthless if _on_event never calls it.

    _on_event is a closure inside run_voice_loop_wiring, so this reads p5's real
    source -- a fake callback would just do the right thing and prove nothing
    (3.2 form 1). Two properties, both load-bearing:
      * the route exists and fires BEFORE the generic body, which would put an
        eid-less row in the HMI ring and relay a payload the cloud projector
        rejects for that same missing eid;
      * it returns, so the snapshot does not also fall through.
    MUTATION: delete the two routing lines -> red, and every chassis fault goes
    back to being dropped as missing_field:eid.
    """
    import inspect

    from xbrain.p5_gateway.runtime.main_wiring import run_voice_loop_wiring

    src = inspect.getsource(run_voice_loop_wiring)
    body = src[src.index("def _on_event("):]
    route = body[:body.index("_relay_to_cloud(ev[")]
    assert '_cat == "chassis" and _sev == "fault"' in route
    assert "_on_chassis_fault(key, d)" in route
    # There must be exactly ONE subscriber feeding it: event/** already matches
    # event/fault/chassis, so a dedicated subscriber would double every derived
    # event. MUTATION: add declare_subscriber("event/fault/chassis", ...) -> red.
    assert src.count('declare_subscriber("event/fault/chassis"') == 0
    assert src.count("_on_chassis_fault(key, d)") == 1


def test_both_halves_ride_the_alarm_channel():
    # 11 S9A.9 E-1: a breach and its recovery must share a backfill cursor, or
    # the cloud stays stuck in alarm forever. channel is derived from cat by the
    # pipeline; this asserts the derived detail.type does not accidentally hit a
    # channel_map override that would split the pair.
    d = _deriver()
    raise_ev = d.observe(_report([_fault()]), now_wall=NOW)[0]
    clear_ev = d.observe(_report(cleared=["chs:0x8001"]), now_wall=NOW + 1)[0]
    for ev in (raise_ev, clear_ev):
        assert derive_channel(ev["cat"], ev["detail"]) == "alarm"


# -- the five 13 S7.3 evidence fields -----------------------------------------
#
# quadruped forwards details / grouped / resources / source / source_ids on
# event/fault/chassis (11 S9.8.4 registers them there as our extension) and
# 13 S7.3 marks the whole group upstream with one reason: field localisation
# depends on them. The deriver read past all five until 2026-09-27, so the
# chain was complete and the evidence stopped at p5. These pin the forwarding
# AND the absent/empty distinction, which is the half an implementation gets
# wrong while passing every "the field is there" assertion.


# One faults[] entry as write_fault_array actually writes it: the four derivation
# keys plus the five evidence keys, every one non-empty so a test can tell a
# forwarded value from a default.
EVIDENCE = {
    "details": "motor temp 91C, derating",
    "grouped": True,
    "resources": ["leg_fl", "leg_fr"],
    "source": ["rl_deploy"],
    "source_ids": ["motion_master#0"],
}


def _fault_with(**over):
    """A well-formed faults[] entry carrying the five evidence fields."""
    entry = dict(_fault())
    entry.update(EVIDENCE)
    entry.update(over)
    return entry


def test_all_five_evidence_fields_reach_detail():
    # The point of the whole fault chain is field localisation (13 S7.3
    # verbatim, one reason written against the whole group), and detail is the
    # only thing that reaches record.db, the cloud and the HMI. Asserted field
    # by field rather than by dict equality: a subset assertion would keep
    # passing if four of the five were dropped.
    # MUTATION: delete the `detail.update(evidence)` call in _event -> all five
    # assertions red. Dropping any single key from _evidence_of -> that one red.
    d = _deriver()
    ev = d.observe(_report([_fault_with()]), now_wall=NOW)[0]
    detail = ev["detail"]
    assert detail["details"] == "motor temp 91C, derating"
    assert detail["grouped"] is True
    assert detail["resources"] == ["leg_fl", "leg_fr"]
    assert detail["source"] == ["rl_deploy"]
    assert detail["source_ids"] == ["motion_master#0"]
    # The four derivation keys are untouched by the addition.
    assert detail["code"] == "chs:0x8001"
    assert detail["level"] == "fatal"
    assert d.stats["bad_evidence"] == 0


def test_an_empty_value_the_chassis_sent_is_forwarded_as_empty():
    # "The chassis named no resources" is a real answer and it is NOT the same
    # answer as "this producer does not carry the field". A reader that cannot
    # tell them apart concludes the chassis was asked when nobody looked.
    # MUTATION: make _evidence_of skip falsy values (`if value:` instead of the
    # type test) -> every assertion below red, and the wire's empty arrays
    # become indistinguishable from an absent key.
    d = _deriver()
    ev = d.observe(_report([_fault_with(
        details="", grouped=False, resources=[], source=[],
        source_ids=[])]), now_wall=NOW)[0]
    detail = ev["detail"]
    assert detail["details"] == ""
    assert detail["grouped"] is False
    assert detail["resources"] == []
    assert detail["source"] == []
    assert detail["source_ids"] == []


def test_an_absent_evidence_field_is_absent_from_detail():
    # The other half of the same distinction: nothing is invented. A default of
    # [] / false / "" here would be a measurement the producer never made, and
    # it is exactly the shape 11 S14.3's source_ids row records -- "key missing"
    # and "chassis gave no instance name" are indistinguishable to a consumer,
    # so no assertion anywhere would ever go red over it.
    # MUTATION: default any of the five in _evidence_of (e.g.
    # out["resources"] = entry.get("resources", [])) -> that key red here.
    d = _deriver()
    ev = d.observe(_report([_fault()]), now_wall=NOW)[0]
    for key in ("details", "grouped", "resources", "source", "source_ids"):
        assert key not in ev["detail"], "invented %s out of nothing" % key
    # The event itself is unharmed: evidence is not identity.
    assert ev["detail"]["code"] == "chs:0x8001"
    assert d.stats["raised"] == 1


def test_a_recovery_carries_no_fabricated_evidence():
    # 11 S9.8.4 gives cleared[] bare code STRINGS: at recovery the chassis says
    # nothing about resources or sources, so there is nothing to forward.
    # Copying the raise's evidence onto the clear would read as "these
    # resources were still implicated when it cleared" -- a claim no message on
    # this key makes.
    # MUTATION: remember the raise's evidence and pass it to the clear's
    # _event(...) -> red on the loop below.
    d = _deriver()
    d.observe(_report([_fault_with()]), now_wall=NOW)
    clear_ev = d.observe(_report(cleared=["chs:0x8001"]), now_wall=NOW + 60.0)[0]
    detail = clear_ev["detail"]
    assert detail["type"] == "chassis_fault_cleared"
    for key in ("details", "grouped", "resources", "source", "source_ids"):
        assert key not in detail, "recovery fabricated %s" % key
    # What a recovery CAN know is still there: the code it ends and when that
    # occurrence had started.
    assert detail["code"] == "chs:0x8001"
    assert detail["since_ts"] == SINCE


def test_a_mistyped_evidence_field_is_dropped_counted_and_costs_nothing_else():
    # 13 S6.5 forbid #2 applies with more force to evidence than to a code: the
    # fault is real and the operator still needs to hear about it, so a wrongly
    # typed locator loses itself and nothing more. Passing it through instead
    # would put a string where a consumer branches on an array.
    # MUTATION: pass the value through unchecked -> the two `not in` assertions
    # red. Drop the counter increment -> the stats assertion red.
    d = _deriver()
    ev = d.observe(_report([_fault_with(
        resources="leg_fl", grouped=1)]), now_wall=NOW)[0]
    detail = ev["detail"]
    assert "resources" not in detail
    # 1 is not True: an int test would land a number in a flag field.
    assert "grouped" not in detail
    # The well-typed neighbours in the same entry survive.
    assert detail["source"] == ["rl_deploy"]
    assert detail["source_ids"] == ["motion_master#0"]
    assert d.stats["bad_evidence"] == 2
    assert d.stats["raised"] == 1


def test_an_array_with_a_non_string_member_is_rejected_whole():
    # Half an array is worse than none: a reader counting resources would get a
    # number that is right often enough to be trusted.
    # MUTATION: filter the members instead of rejecting the array -> red.
    d = _deriver()
    ev = d.observe(_report([_fault_with(source_ids=["motion_master#0", 7])]),
                   now_wall=NOW)[0]
    assert "source_ids" not in ev["detail"]
    assert d.stats["bad_evidence"] == 1


def test_the_forwarded_arrays_are_copies_not_the_caller_s_lists():
    # detail is handed on to the pipeline, record_dao and the cloud relay. An
    # aliased list is one mutation away from three disagreeing copies of one
    # report, and the disagreement would surface as a cloud/record.db mismatch
    # nobody could reproduce.
    # MUTATION: out[key] = value (no list() copy) -> red.
    d = _deriver()
    incoming = ["leg_fl"]
    ev = d.observe(_report([_fault_with(resources=incoming)]), now_wall=NOW)[0]
    incoming.append("leg_rr")
    assert ev["detail"]["resources"] == ["leg_fl"]
