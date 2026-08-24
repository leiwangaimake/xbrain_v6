"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_zones.py
Brief: 报警区入侵 zone_enter/zone_exit + episode/E-1/E-2/E-3 (11 S9A.9, 报警 F2)

Description:
守 ZoneTracker 的跃迁语义: 外->内报 enter, 内->外报 exit, 持续在内不刷屏(E-3),
一次入区过程 enter/exit 共用 episode 且退出后 +1(E-2), 区被删而人在内补 exit
(E-1 成对). 每条配变异体.
"""

from xbrain.p1_motion.fence.fence_set import HeldPolygon
from xbrain.p1_motion.fence.zones import ZoneTracker

# 一个方形报警区(WGS84 lat/lon). 内点约 (34.6975, 135.5055), 外点 (34.70, 135.51).
_SQUARE = HeldPolygon(
    poly_id="f-zone", role="warning", name="gate", hard_enforce=False,
    vertices=((34.697, 135.505), (34.698, 135.505),
              (34.698, 135.506), (34.697, 135.506)))

_IN = (34.6975, 135.5055)      # 区内
_OUT = (34.7000, 135.5100)     # 区外


def test_entering_a_zone_emits_zone_enter():
    """外->内: 一条 zone_enter(alarm), episode 0, dedup key 对."""
    t = ZoneTracker()
    assert t.observe(*_OUT, [_SQUARE]) == []        # 起始在外, 无事件
    evs = t.observe(*_IN, [_SQUARE])
    assert len(evs) == 1
    e = evs[0]
    assert e.kind == "zone_enter" and e.severity == "alarm"
    assert e.poly_id == "f-zone" and e.poly_name == "gate"
    assert e.episode_id == 0 and e.dedup_key == "fence:zone:f-zone:0"


def test_staying_inside_does_not_repeat_enter():
    """*** E-3: 持续在区内不重复发 enter(否则 10 Hz 一次贴边就是事件洪水).

    MUTATION: observe 每拍在区内都发 enter(去掉 not was 判据) -> 第二拍冒出
    事件 -> 这里红.
    """
    t = ZoneTracker()
    t.observe(*_IN, [_SQUARE])                       # 首次进入
    assert t.observe(*_IN, [_SQUARE]) == []          # 仍在内, 不再发
    assert t.observe(*_IN, [_SQUARE]) == []


def test_exit_reuses_the_enter_episode_then_increments():
    """*** E-2: 一次入区 enter/exit 共用 episode; 退出后下次入区 +1.

    MUTATION: 退出后不 +1(删 self._episode[pid] = ep + 1) -> 第二次 enter 仍是
    episode 0 -> 这里断言 episode 1 红.
    """
    t = ZoneTracker()
    en = t.observe(*_IN, [_SQUARE])[0]               # enter ep0
    ex = t.observe(*_OUT, [_SQUARE])[0]              # exit  ep0(共用)
    assert en.episode_id == 0 and ex.episode_id == 0
    assert ex.kind == "zone_exit" and ex.severity == "info"
    assert ex.dedup_key == "fence:zone_exit:f-zone:0"
    # 再入区: episode 必须 +1(否则云端把两次入区混成一次)
    en2 = t.observe(*_IN, [_SQUARE])[0]
    assert en2.episode_id == 1 and en2.dedup_key == "fence:zone:f-zone:1"


def test_zone_removed_while_inside_emits_exit():
    """*** E-1 成对: 报警区被删而机器人还在内, 补一条 zone_exit.

    否则那条 enter 没有配对 exit, 云端永远停在"在区内"告警态.
    MUTATION: 去掉 vanished 补 exit 的那段 -> 删区后无事件 -> 这里红.
    """
    t = ZoneTracker()
    t.observe(*_IN, [_SQUARE])                       # 进入
    evs = t.observe(*_IN, [])                        # active 集里已无该区
    assert len(evs) == 1 and evs[0].kind == "zone_exit"
    assert evs[0].poly_id == "f-zone" and evs[0].episode_id == 0


def test_two_zones_track_independently():
    """两个报警区各自独立 episode/inside -- 进 A 不影响 B."""
    b = HeldPolygon(poly_id="f-b", role="warning", name="depot",
                    hard_enforce=False,
                    vertices=((34.70, 135.51), (34.701, 135.51),
                              (34.701, 135.511), (34.70, 135.511)))
    t = ZoneTracker()
    # _IN 在 f-zone 内, 不在 f-b 内 -> 只 f-zone enter.
    evs = t.observe(*_IN, [_SQUARE, b])
    assert [e.poly_id for e in evs] == ["f-zone"]


def test_outside_all_zones_no_events():
    """一直在所有区外: 恒空."""
    t = ZoneTracker()
    assert t.observe(*_OUT, [_SQUARE]) == []
    assert t.observe(*_OUT, [_SQUARE]) == []


def test_p1_loop_actually_runs_zone_detection():
    """*** 守接线: main_wiring 的 pose 循环真的调 zone_tracker.observe 并发布.

    ZoneTracker 单测再全, 没接进循环也是空转. AST 查 main_wiring: 实例化
    ZoneTracker, 循环里调 .observe, 且把结果交给 _publish_zone_events.

    MUTATION: 删掉循环里 _publish_zone_events(zone_tracker.observe(...)) 那句 ->
    这里红("表对而线没接"防线).
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain" / "p1_motion"
           / "runtime" / "main_wiring.py").read_text(encoding="utf-8")
    called = {getattr(n.func, "attr", "") or getattr(n.func, "id", "")
              for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call)}
    assert "ZoneTracker" in called, "main_wiring 没实例化 ZoneTracker"
    assert "observe" in called, "循环没调 zone_tracker.observe -- 报警区不判入侵"
    assert "_publish_zone_events" in called, (
        "observe 的结果没交给 _publish_zone_events -- 判了不发, 云端收不到告警")
