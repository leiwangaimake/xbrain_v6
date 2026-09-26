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

from xbrain.p5_gateway.hmi.estop_probe import (EstopProbe, build_ping_data,
                                               pong_seq)


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


# --- 11 S8.5: 关联号在 data 里, 不在信封里 (2026-09-27 收口) -------------
#
# 这几条判据的共同点: 信封 seq 与 data.seq [故意取不同的值].
# 在没有 chassis_relay 的开发机上两者恒相等, 于是任何"读错了地方"的实现
# 都能通过 -- 这正是该缺陷活到 relay 上机那天才暴露的原因. 下面每条都把
# 两个数写成不同的, 读错一个立刻红.

def test_ping_data_carries_the_correlation_seq():
    # RED MUTANT: build_ping_data 把 seq 丢掉(或改成常量) -> 这里红.
    d = build_ping_data(1042, 918273000)
    assert d == {"type": "ping", "seq": 1042, "t_mono_ms": 918273000}


def test_pong_seq_reads_data_not_the_envelope():
    # 部署实况: chassis_relay 按 RT-C3.e 用自己的计数改写了信封 seq(这里 77),
    # p5 真正发出去的号 1042 只在 data 里活着.
    # RED MUTANT: pong_seq 改读顶层 seq -> 返回 77, 这里红.
    pong = {"v": 1, "rid": "dev", "ts": 1789455340.125, "seq": 77,
            "src": "chassis_relay", "ts_sync": False,
            "orig_src": "quadruped",
            "data": {"type": "pong", "seq": 1042, "t_mono_ms": 918273007}}
    assert pong_seq(pong) == 1042


def test_pong_without_data_is_not_correlated():
    # 裸报文(2026-09-27 之前的形态): 没有 data 就没有关联号.
    # *** NO 不回落到顶层 seq. 顶层是最后一跳转发者的计数器, 与 probe_seq
    # 同频同量级, 迟早偶然相等 -> 一次假匹配把 estop_path 跳成 ok, 而按下
    # 急停不会停. 永不匹配(down)是 fail-safe, 偶尔匹配是 fail-silent.
    # RED MUTANT: 加一条 `or payload.get("seq")` 的回落 -> 返回 77, 这里红.
    assert pong_seq({"type": "pong", "seq": 77, "t_mono_ms": 1}) is None


def test_pong_seq_rejects_non_integer_and_bool():
    # bool 是 int 的子类: 不显式排掉, {"seq": true} 会被当成 1 --
    # 而 1 是 p5 开机后发出的第一个 probe_seq, 即一次必然发生的假匹配.
    # RED MUTANT: 去掉 isinstance(seq, bool) 那半 -> True 用例红.
    assert pong_seq({"data": {"seq": True}}) is None
    assert pong_seq({"data": {"seq": "1042"}}) is None
    assert pong_seq({"data": {"seq": 12.5}}) is None
    assert pong_seq({"data": None}) is None
    assert pong_seq({}) is None
    assert pong_seq("not a dict") is None


def test_a_rewritten_envelope_seq_still_matches_end_to_end():
    # 端到端形状: p5 发 1042(data) 而信封 seq 是它自己的 9;
    # relay 改写信封 seq 为 77; quadruped 回显 data.seq; relay 再改写一次.
    # 只要走 data.seq, RTT 就量得出来 -- 这条断言就是整轮收口的判据.
    # RED MUTANT: 任一侧改读信封 seq -> on_pong 收到 77, 匹配不上, rtt_ms
    # 恒 None 且 estop_path 停在 down.
    p = _probe()
    out = build_ping_data(1042, 1000)
    p.on_ping_sent(seq=out["seq"], mono_ms=out["t_mono_ms"])
    pong = {"v": 1, "rid": "dev", "ts": 1.0, "seq": 77, "src": "chassis_relay",
            "ts_sync": False, "data": {"type": "pong", "seq": out["seq"]}}
    got = pong_seq(pong)
    assert got == 1042
    p.on_pong(seq=got, mono_ms=1010)
    assert p.rtt_ms == 10.0
    assert p.estop_path() == "ok"


def test_a_bad_data_seq_never_falls_back_to_the_envelope_seq():
    # 信封完整([顶层 seq 在])而 data.seq 坏掉的那一格.
    #
    # *** 为什么单独写一条: 上面 test_pong_without_data_is_not_correlated 用的
    # 载荷[没有 data], 走的是更早的那个 return; 而 test_pong_seq_rejects_*
    # 用的载荷[没有顶层 seq], 于是一个"data.seq 不可用就读顶层"的实现在那两条
    # 里同样返回 None -- 两条都测不出回落. 2026-09-27 跑变异体实测: 注入该回落
    # 后前面五条全绿, 只有本条能让它红.
    for bad in (True, "1042", 12.5, None, [1042]):
        pong = {"v": 1, "rid": "dev", "ts": 1.0, "seq": 77,
                "src": "chassis_relay", "ts_sync": False,
                "data": {"type": "pong", "seq": bad}}
        assert pong_seq(pong) is None, "fell back to the envelope seq for %r" % (bad,)
