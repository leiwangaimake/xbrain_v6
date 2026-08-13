"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_estop_probe.py
Brief: HMI-W5 estop-path probe state machine tests (17 S6.3)

Description:
Guards the ok/degraded/down verdict of EstopProbe (17 S6.3) with the mutation
each assertion is paired to (CLAUDE.md 3.3 -- an assertion with no red mutant is
not written). The load-bearing one is test_starts_down_before_any_pong: the
whole point of W5 is that with no chassis the button greys, so a mutant that
seeds misses=0 (armed on faith, the fail-silent 3.2 forbids) MUST turn this red.
The others pin the RTT/miss thresholds and the stale-pong guard.

Boundary: pure state machine, no zenoh/clock -- times are plain ints the test
supplies, mirroring how the wiring feeds monotonic ms (11 CLK-C1).
"""

from __future__ import annotations

from xbrain.p5_gateway.hmi.estop_probe import EstopProbe


def _probe():
    # rtt_degrade_ms=200, down_misses=3 -- the configs/p5_gateway.yaml values.
    return EstopProbe(rtt_degrade_ms=200.0, down_misses=3)


def test_starts_down_before_any_pong():
    # W5 core invariant: no pong ever seen -> "down", button greyed.
    # RED MUTANT: seed self._misses = 0 in __init__ -> this reports "ok" with
    # nothing answering (the exact fail-silent the hard-coded "ok" was).
    p = _probe()
    assert p.estop_path() == "down"
    assert p.rtt_ms is None


def test_fresh_fast_pong_is_ok():
    # ping at t=1000, pong at t=1050 -> RTT 50 < 200 -> ok.
    # RED MUTANT: drop the misses=0 reset in on_pong -> stays "down".
    p = _probe()
    p.on_ping_sent(seq=1, mono_ms=1000)
    p.on_pong(seq=1, mono_ms=1050)
    assert p.estop_path() == "ok"
    assert p.rtt_ms == 50.0


def test_slow_pong_is_degraded():
    # RTT 250 >= 200 threshold -> degraded (link alive but too slow).
    # RED MUTANT: use > instead of >= in estop_path -> a pong exactly at 200 would
    # still read ok; here 250 is unambiguous but the boundary test below pins >=.
    p = _probe()
    p.on_ping_sent(seq=1, mono_ms=1000)
    p.on_pong(seq=1, mono_ms=1250)
    assert p.estop_path() == "degraded"


def test_rtt_boundary_is_left_closed():
    # RTT exactly at the threshold counts as degraded (>=), not ok.
    # RED MUTANT: > instead of >= -> 200 reads "ok", masking a link at the edge.
    p = _probe()
    p.on_ping_sent(seq=1, mono_ms=1000)
    p.on_pong(seq=1, mono_ms=1200)
    assert p.estop_path() == "degraded"


def test_recovers_to_ok_after_a_good_pong():
    # A good pong clears an accumulated miss count.
    # RED MUTANT: on_pong not resetting misses -> stuck degraded/down.
    p = _probe()
    p.on_ping_sent(seq=1, mono_ms=1000)   # no pong
    p.on_ping_sent(seq=2, mono_ms=2000)   # miss for seq1
    p.on_pong(seq=2, mono_ms=2040)        # fast pong for seq2
    assert p.estop_path() == "ok"


def test_down_after_down_misses_consecutive_gaps():
    # Three pings with no pong (after one initial good pong) -> down.
    # RED MUTANT: not incrementing misses in on_ping_sent -> never reaches down.
    p = _probe()
    p.on_ping_sent(seq=1, mono_ms=1000)
    p.on_pong(seq=1, mono_ms=1010)        # ok baseline
    assert p.estop_path() == "ok"
    p.on_ping_sent(seq=2, mono_ms=2000)   # awaiting
    p.on_ping_sent(seq=3, mono_ms=3000)   # miss #1 (seq2 never answered)
    p.on_ping_sent(seq=4, mono_ms=4000)   # miss #2
    p.on_ping_sent(seq=5, mono_ms=5000)   # miss #3 -> threshold
    assert p.estop_path() == "down"


def test_stale_pong_for_old_seq_is_ignored():
    # A pong whose seq does not match the outstanding ping must not clear misses.
    # RED MUTANT: drop the seq==_sent_seq guard in on_pong -> a late reply for an
    # old ping revives a dead link (masks a real outage).
    p = _probe()
    p.on_ping_sent(seq=1, mono_ms=1000)
    p.on_ping_sent(seq=2, mono_ms=2000)   # seq1 now stale
    p.on_pong(seq=1, mono_ms=2010)        # late reply for seq1 -> ignored
    assert p.rtt_ms is None               # never recorded an RTT
