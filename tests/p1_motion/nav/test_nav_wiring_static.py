"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_nav_wiring_static.py
Brief: P7.2 wiring presence -- keys subscribed / published, single sink, teardown, config gate

Description:
Same discipline as test_p1_estop's wiring check: the runtime cannot be run
under pytest (it needs two zenohd routers), so the assertions that CAN be
made offline are made on the source text and on the importable pieces:
the three general-plane subscriptions and three publishers exist by their
contract key names, cmd_vel is published from exactly one place (12 S2.2 step
10), every declared subscriber is held in a list and undeclared, __main__
hands the NavConfig (or None, loudly) to the wiring, and unwrap_body accepts
both a bare body and an 11 S3.0 envelope.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

from xbrain.p1_motion.runtime.nav_wiring import (
    CMD_FACTOR_TOPIC,
    CMD_RELMOVE_TOPIC,
    CMD_ROUTE_TOPIC,
    RELMOVE_STATUS_TOPIC,
    STATE_PROGRESS_TOPIC,
    TICK_PERIOD_S,
    unwrap_body,
)

pytestmark = pytest.mark.no_device

_P1 = Path(__file__).resolve().parents[3] / "xbrain" / "p1_motion"
_NAV = (_P1 / "runtime" / "nav_wiring.py").read_text(encoding="utf-8")
_MAIN = (_P1 / "runtime" / "main_wiring.py").read_text(encoding="utf-8")
_ENTRY = (_P1 / "__main__.py").read_text(encoding="utf-8")


def test_contract_keys():
    assert (CMD_ROUTE_TOPIC, CMD_RELMOVE_TOPIC, CMD_FACTOR_TOPIC) == (
        "cmd/motion/route", "cmd/motion/relative_move", "cmd/motion/factor")
    assert (STATE_PROGRESS_TOPIC, RELMOVE_STATUS_TOPIC) == (
        "state/motion/path_progress", "cmd/motion/relative_move/status")
    from xbrain.p1_motion.runtime.nav_wiring import STATE_ARB_TOPIC
    assert STATE_ARB_TOPIC == "state/arb/motion"
    assert 'declare_publisher(STATE_ARB_TOPIC)' in _NAV
    assert '"event/%s/arbitration"' in _NAV
    assert TICK_PERIOD_S == 0.05


def test_subscriptions_are_held_and_undeclared():
    """CLAUDE.md 4.3: every declare_subscriber lands in self._subs; stop()
    undeclares them. mutant: drop one append -> the count red."""
    subs = re.findall(r"self\._subs\.append\(self\._gen\.declare_subscriber\((\w+)", _NAV)
    assert sorted(subs) == ["CMD_FACTOR_TOPIC", "CMD_RELMOVE_TOPIC", "CMD_ROUTE_TOPIC"]
    assert "s.undeclare()" in _NAV


def test_cmd_vel_has_a_single_sink():
    """12 S2.2 step 10: one publisher of rt/motion/cmd_vel, called from the
    CtrlLoop callback only."""
    assert _NAV.count('declare_publisher("xbrain/%s/rt/motion/cmd_vel" % self._rid)') == 1
    assert _NAV.count("self._cmd_pub.put(") == 1
    assert "rt/motion/cmd_vel" not in _MAIN.replace("rt/motion/cmd_vel pub", "")


def test_main_wiring_starts_the_runtime_only_with_rid_and_config():
    assert "nav_rt = NavRuntime(" in _MAIN
    assert "nav_rt.declare()" in _MAIN and "nav_rt.start()" in _MAIN
    assert "nav_rt.stop()" in _MAIN
    assert "teleop_lock = threading.Lock()" in _MAIN
    assert _MAIN.count("with teleop_lock:") == 2
    assert 'gnss_cache["rx"] = time.monotonic()' in _MAIN
    assert 'fix_cache["rx"] = time.monotonic()' in _MAIN


def test_entry_builds_nav_config_from_both_snapshots_and_never_defaults():
    assert 'load_resolved("rns"' in _ENTRY
    assert "build_nav_config(_cfg.tree, _rns.tree)" in _ENTRY
    assert "nav_cfg=nav_cfg" in _ENTRY
    assert "p1 nav loop DISABLED" in _ENTRY


def test_unwrap_body_accepts_bare_and_enveloped():
    body = {"cmd_id": "rm-1", "dx_m": 1.0}
    assert unwrap_body(body) is body
    env = {"v": 1, "rid": "dev", "ts": 1.0, "seq": 3, "src": "p2_core", "data": body}
    assert unwrap_body(env) is body
    assert unwrap_body(None) is None


def test_envelopes_use_the_contract_encoder_not_millisecond_stamps():
    """11 S3.0: ts / mono are seconds. gnss_pose.stamp_envelope stamps
    milliseconds (a pre-existing deviation on state/pose); the P7.2 keys must
    not inherit it. mutant: call stamp_envelope -> red."""
    assert "from xbrain.common.envelope.envelope import Envelope, encode" in _NAV
    assert "stamp_envelope(" not in _NAV          # mentioned in a docstring, never called
    assert _NAV.count("encode(env)") == 1


def test_stats_deque_is_locked_against_the_heartbeat_thread():
    assert "with self._stats_lock:\n                self._periods.append" in _NAV
    assert "with self._stats_lock:\n            p = sorted(self._periods)" in _NAV


def test_no_event_this_process_publishes_is_stamped_with_a_constant_ts():
    """11 S6.1 Event.ts is the producer's time and p5's dedup comparison value.

    Until 2026-09-30 three of p1's four event publishers stamped the literal
    0.0: the arbitration audit, the nav/health/tick-failure rows, and the
    zone_enter/zone_exit rows.

    Claimed no larger than it is: p5's _event_row writes
    `d.get("ts") or data.get("ts") or now.timestamp()` and 0.0 is falsy, so
    today the row lands with p5's ingestion time and the merge arithmetic
    still sees a rising value. What is lost is WHEN the event happened, and
    the thing standing between that and a broken dedup is a fallback that
    fires because 0.0 happens to be falsy. Tighten that `or` to a None check,
    or read ts anywhere else (recorder, replay), and (ts - last_ts) is 0 for
    every event of a key, so no window can ever be exceeded -- which is the
    failure p5 measured from the other end on 2026-09-27
    (chassis_events.DEDUP_WINDOW_S).

    MUTATION: put `"ts": 0.0,` back into any of them -> red.
    """
    for name, text in (("nav_wiring.py", _NAV), ("main_wiring.py", _MAIN)):
        assert '"ts": 0.0' not in text, (
            "%s stamps a constant Event.ts -- see record_dao._try_merge" % name)


def test_every_published_event_carries_both_a_dedup_key_and_a_window():
    """A dedup_key with no dedup_window_s is a silent drop, not a dedup.

    record_dao._try_merge: when the event and the still-open row BOTH carry
    no window, it merges unconditionally. So the field is not optional for a
    producer that sets a key -- p5 has no per-kind window table to fall back
    on (grep dedup_window_s under xbrain/p5_gateway: it only reads what the
    payload carries).

    Every PUBLISHED payload is checked, not a sample: an Event payload is a
    json.dumps({...}) block carrying an eid, which is what distinguishes it
    from the Emit bodies that feed _publish_event (those carry the key as an
    INPUT and get their window stamped by the publisher). So a publisher
    added later cannot quietly be the one without a window.

    MUTATION: delete one "dedup_window_s" line from either file -> red.
    """
    total = 0
    for name, text in (("nav_wiring.py", _NAV), ("main_wiring.py", _MAIN)):
        blocks = [b for b in re.findall(r"json\.dumps\(\{(.*?)\}, ensure_ascii",
                                        text, re.S) if '"eid":' in b]
        assert blocks, "%s composes no event payload any more" % name
        total += len(blocks)
        for block in blocks:
            eid = re.search(r'"eid": "([a-z]+)-', block)
            who = eid.group(1) if eid else block[:40]
            assert '"dedup_key"' in block, (
                "%s: the %s event payload carries no dedup_key" % (name, who))
            assert '"dedup_window_s"' in block, (
                "%s: the %s event payload sets a dedup_key with NO window -- "
                "record_dao merges those unconditionally" % (name, who))
    # nav / fence / rotation / arbitration + main_wiring's zone rows.
    assert total == 5, "the set of p1 event publishers changed (%d found)" % total
