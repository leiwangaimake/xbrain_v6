"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_projector.py
Brief: 出站状态面的节律判据 -- 该发的发, 不该发的不发 (B-3)

Description:
*** 本文件守的是同一个形状的第三层.
  一层  登记表对, 接线无            -> test_cloud_key_surface_wired
  二层  桥写好了, 启动路径不调它     -> test_cloud_bridge 的 AST 断言
  三层  发布口有了, 没人驱动它       -> 本文件
每一层单独看都完整, 合起来在真机上是"Qt 订阅成功, 永远收不到内容", 而
Zenoh 不报任何错. 三层各自都要有断言, 因为它们互相看不见.

*** 节律断言必须双向, 这是最容易写歪的地方.
只判"到点了会发", 一个[每拍都发]的实现全绿 -- 而它会把 10 Hz 的循环变成
七条 key 各 10 Hz, 在 Q3 上挤掉事件面.
只判"没到点不发", 一个[永远不发]的实现全绿 -- 而那就是本文件要防的病.

Boundaries: 假 bridge, 假单调钟. 形状对不对由 test_state_projection 判.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


class _FakeBridge:
    def __init__(self):
        self.published = []          # [(key, data)]

    def publish_state(self, name, data):
        self.published.append((name, data))

    def keys(self):
        return [k for k, _ in self.published]


class _Clock:
    """假单调钟. NO 不用真钟 -- 一条节律断言若要真的等 5 秒, 它就会被
    改成"等 0.05 秒"然后失去意义."""

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


POSE = {"lat": 31.23, "lon": 121.47, "heading_rad": 1.6,
        "heading_valid": True, "speed_mps": 0.6, "fix_type": "rtk_fixed"}


def _proj(clock=None):
    from xbrain.p5_gateway.runtime.cloud_state import CloudProjector

    bridge = _FakeBridge()
    clock = clock or _Clock()
    return CloudProjector(bridge, now_mono=clock), bridge, clock


def _state(**over):
    st = {"tasks": None, "pose": POSE, "clock": {"ts_sync": True},
          "health": None, "mode": "normal", "geo_cache": None}
    st.update(over)
    return st


# --- 首拍 -------------------------------------------------------------

def test_the_first_tick_publishes_every_key_that_has_a_shape():
    """*** 首拍就要发, 不能等一个周期.

    等的话 Qt 刚连上来的那一秒什么都看不到 -- 而操作员的第一印象就是
    "这机器人没上线". geo/manifest 是唯一的例外(它要等 geo 广播到齐,
    见下一条).
    """
    proj, bridge, _c = _proj()

    sent = proj.tick(_state())

    assert set(sent) == {"state/robot", "state/task", "state/mode",
                         "state/audio", "state/media", "data/file/index"}, sent


def test_the_manifest_waits_briefly_then_goes_out_even_if_empty():
    """*** 开机头 1.5 秒内不发空清单, 之后即便空也要发.

    先发一份空的话, Qt 会以为一个地理对象都没有并把地图画空 -- 而真相
    只是 geo 广播还没到. 反过来, 一直不发也不行: 那台机器人可能真的
    一个对象都没有, 而 Qt 会一直等.

    MUTATION: 把 MANIFEST_FIRST_S 改成 0 -> 前半段红.
    MUTATION: 让 _manifest 在 objects 为空时永远返回 None -> 后半段红.
    """
    proj, bridge, clock = _proj()

    assert "state/geo/manifest" not in proj.tick(_state())
    clock.advance(2.0)
    assert "state/geo/manifest" in proj.tick(_state())
    body = dict(bridge.published)["state/geo/manifest"]
    assert body["full"] is True and body["objects"] == []


# --- 节律 -------------------------------------------------------------

def test_an_unchanged_payload_is_not_republished_before_its_period():
    """*** 没变化且没到点就不发.

    每拍都发的话, 七条 key 各 10 Hz -- 而其中只有 state/robot 该是 10 Hz.
    多出来的量在 Q3 上挤掉事件面, 而事件面是断网补发的依据.

    MUTATION: 让 _due 恒返回 True -> 这里红.
    """
    proj, bridge, clock = _proj()

    st = _state()
    proj.tick(st)
    n_after_first = len(bridge.published)
    clock.advance(0.1)
    sent = proj.tick(st)

    # state/robot 的周期是 0.1s, 到点了; 其余的没到.
    assert sent == ["state/robot"], sent
    assert len(bridge.published) == n_after_first + 1


def test_each_key_honours_its_own_period():
    """v2.0 S2 逐条给的节律. 一条一条对.

    * state/robot 0.1s / state/task,mode,audio 1s / state/media 5s.
    """
    proj, bridge, clock = _proj()

    st = _state()
    proj.tick(st)
    bridge.published.clear()

    clock.advance(1.0)
    sent = set(proj.tick(st))
    assert {"state/robot", "state/task", "state/mode", "state/audio"} <= sent
    assert "state/media" not in sent, "media 的周期是 5 秒, 1 秒就发了"

    clock.advance(4.0)
    assert "state/media" in proj.tick(st)


def test_a_change_publishes_immediately_without_waiting():
    """*** 变化即发. 这是与纯周期发布的关键差别.

    等周期的话, 一次任务状态跃迁最多要等 1 秒才到 Qt -- 而操作员点了
    "停止"之后盯着屏幕, 1 秒的空白足以让他再点一次.

    MUTATION: 把 _due 里的 changed 判断删掉(只留周期) -> 这里红.
    """
    proj, bridge, clock = _proj()

    proj.tick(_state())
    bridge.published.clear()
    clock.advance(0.01)          # 远不到 state/mode 的 1 秒

    sent = proj.tick(_state(mode="broadcast", stream_id="a-1"))

    assert "state/mode" in sent, "模式变了却等周期: %s" % sent


def test_change_detection_compares_serialised_bodies():
    """*** 按序列化结果比, 不按对象比.

    按对象比的话, 一个被原地改过的缓存(list.append 之类)会与上一次比出
    相等, 于是变化发不出去 -- 而它看起来完全正常. 这里用同一个 list 对象
    做验证: 内容变了就必须发.
    """
    proj, bridge, clock = _proj()

    tasks = [{"task_id": "t-1", "task_type": "GOTO_KEYPOINT",
              "state": "queued"}]
    st = _state(tasks=tasks)
    proj.tick(st)
    bridge.published.clear()
    clock.advance(0.01)

    tasks[0]["state"] = "running"          # 原地改, 对象还是同一个
    assert "state/task" in proj.tick(st), "原地改动没被发现"


# --- 健壮性 -----------------------------------------------------------

def test_one_key_raising_does_not_stop_the_others():
    """*** 与 P1 控制循环同一取舍(CLAUDE.md 4.4).

    一条 state/mode 因为闭集越界抛了, 不该让 state/robot 也停发 --
    后者是 Qt 判"机器人还活着"的依据之一.

    MUTATION: 把 tick 里的 try/except 去掉 -> 这里红(整个 tick 抛出).
    """
    proj, bridge, _c = _proj()

    # 一个闭集外的 task state 会让 state/task 的投影抛 ProjectionError.
    sent = proj.tick(_state(tasks=[{"task_id": "t-1", "state": "running",
                                    "task_type": "X"},
                                   {"task_id": "t-2", "state": "weird",
                                    "task_type": "X"}]))

    assert "state/task" not in sent, "闭集外的值被发出去了"
    assert "state/robot" in sent, "一条 key 抛错把其余的也带停了"
    assert proj.errors == 1


def test_a_projection_error_is_counted_not_swallowed_silently():
    """出错要能被看见.

    静默跳过的话, Qt 上那一栏永远空着而日志里什么都没有 -- 联调时会被
    当成"客户端没订阅".
    """
    proj, _b, _c = _proj()

    proj.tick(_state(tasks=[{"task_id": "t", "state": "nope",
                             "task_type": "X"}]))
    assert proj.errors >= 1


# --- 诚实性 -----------------------------------------------------------

def test_robot_state_only_ever_reports_what_has_a_source():
    """*** charging / fault / emergency_stop / offline 都没有机内来源.

    报出来就是编的. 尤其 emergency_stop: estop_probe 给的是[通路健康]
    (ok/degraded/down), 不是[是否已接合] -- 两者不可互推. 一个把通路
    down 当成已急停的实现, 会在探针网络抖动时告诉 Qt 机器人急停了.

    MUTATION: 让 _robot 在 link 缺失时报 offline -> 这里红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import SOURCED_ROBOT_STATES

    proj, bridge, clock = _proj()

    seen = set()
    for st in (_state(), _state(tasks=[{"task_id": "t", "state": "running",
                                        "task_type": "X"}]),
               _state(link=None, pose=None, clock=None)):
        clock.advance(1.0)
        proj.tick(st)
    for name, data in bridge.published:
        if name == "state/robot":
            seen.add(data["robot_state"])

    assert seen <= set(SOURCED_ROBOT_STATES), (
        "报出了没有来源的 robot_state: %s" % sorted(seen - set(
            SOURCED_ROBOT_STATES)))
    assert seen == {"idle", "running"}, (
        "两个值都要出现过, 否则一个恒 idle 的实现也能通过: %s" % seen)


def test_alarm_window_is_false_while_the_clock_is_unsynced():
    """v2.0 S3.5 逐字: 授时未同步时带时间窗的规则不命中.

    报 true 的话 Qt 显示"报警窗生效中", 而实际上规则一条都没命中 --
    操作员以为现场有防护.
    """
    proj, bridge, clock = _proj()

    proj.tick(_state(clock={"ts_sync": False}, alarm_window=True))
    data = dict(bridge.published)["state/robot"]
    assert data["alarm_window_active"] is False

    clock.advance(1.0)
    proj.tick(_state(clock={"ts_sync": True}, alarm_window=True))
    assert dict(bridge.published)["state/robot"]["alarm_window_active"] is True


def test_media_and_file_index_send_empty_arrays_not_nothing():
    """*** 无内容时发空数组, NO 不是不发.

    不发的话 Qt 收不到任何 state/media, 与"后端挂了"不可区分; 而一个
    endpoints: [] 明确说"这台机器上现在没有可用画面".
    """
    proj, bridge, _c = _proj()

    proj.tick(_state())
    body = dict(bridge.published)

    assert body["state/media"]["endpoints"] == []
    assert body["data/file/index"]["files"] == []


def test_snapshot_queue_comes_from_the_full_task_list():
    """*** v2.0 S3.2 的 queue/suspended 只有拿到任务全量才填得出来.

    state/task 广播是 15 S12A 的形状, 只带 active_task 一条 -- 靠它 queue 与
    suspended 恒空. 2026-09-02 实测: 库里有 ready/pending 的任务, 而 Qt 收到的
    快照三个列表全空. 全量来自 P3 的 query/tasks queryable(11 S12.2A), 就是
    HMI 的 /api/tasks 已经在用的那条(平面隔离下 p5 不能直接读 p3 的 task.db).

    变异体: _all_tasks 改回只读 state["tasks"] => 本条红.
    """
    proj, bridge, _c = _proj()

    st = _state()
    # 广播那条只有 active_task; 全量里还有两条排队的.
    st["tasks"] = [{"task_id": "t-run", "state": "running"}]
    st["cloud_tasks"] = [
        {"task_id": "t-run", "task_type": "goto", "state": "running"},
        {"task_id": "t-q1", "task_type": "goto", "state": "ready"},
        {"task_id": "t-s1", "task_type": "patrol", "state": "suspended"},
    ]
    proj.tick(st)

    snap = dict(bridge.published)["state/task"]
    assert snap["current"] and snap["current"]["task_id"] == "t-run"
    assert [t["task_id"] for t in snap["queue"]] == ["t-q1"], (
        "queue 没填出来: %r" % snap["queue"])
    assert [t["task_id"] for t in snap["suspended"]] == ["t-s1"], (
        "suspended 没填出来: %r" % snap["suspended"])
    # task_type 也跟着全量一起有了(此前恒 null -- 广播里根本没这个字段).
    assert snap["current"]["task_type"] == "goto", (
        "task_type 仍为空, v2.0 S3.2 把它列为必填")


def test_it_falls_back_to_the_broadcast_when_the_query_is_empty():
    """*** 全量取不到时回落到广播那条, NO 不整个快照空掉.

    query/tasks 可能因为 P3 忙 / queryable 掉线而无应答. 那时 current 至少还
    能从广播里得出来 -- 比 Qt 看到"一个任务都没有"强, 后者与"真的没任务"
    不可区分.

    变异体: _all_tasks 去掉回落分支 => 本条红.
    """
    proj, bridge, _c = _proj()
    st = _state()
    st["tasks"] = [{"task_id": "t-run", "state": "running"}]
    st["cloud_tasks"] = []          # 查询没结果
    proj.tick(st)

    snap = dict(bridge.published)["state/task"]
    assert snap["current"] and snap["current"]["task_id"] == "t-run", (
        "全量为空时连 current 都丢了")


def test_devices_come_only_from_what_health_actually_reports():
    """*** 只报 health 真的有的那些(v2.0 S4.2: 只发布实际发现的设备).

    *** 2026-09-01 重写: 本条原来喂的是 {"status","label","age_ms"} --
    那三个键在 11 S5.1 里一个都不存在(真名 state / 无 / since_mono). 判据与
    被测实现抄的是同一份想象, 于是两边一起错却一直绿, 而线上 devices 恒空.
    现在喂 11 S5.1 的真实形状: 字段名对不上就红.
    """
    proj, bridge, _c = _proj()

    proj.tick(_state(health={"items": {
        "cam_ptz_vis": {"kind": "device", "level": "degraded",
                        "state": "ok", "since_mono": 100.0},
        "lidar": {"kind": "device", "level": "degraded",
                  "state": "degraded"}}}))

    devs = dict(bridge.published)["state/robot"]["devices"]
    assert {d["id"] for d in devs} == {"cam_ptz_vis", "lidar"}
    assert all(set(d) == {"id", "name", "status", "last_update_ms"}
               for d in devs)
    by_id = {d["id"]: d for d in devs}
    assert by_id["cam_ptz_vis"]["status"] == "online"
    assert by_id["lidar"]["status"] == "degraded"


def test_an_off_set_health_state_refuses_the_whole_key():
    """*** 闭集外的 state 让整条 state/robot 拒发, NO 不兜底成 unknown.

    *** 2026-09-01 推翻本条原立论. 原文写"映到 unknown, 不猜一个" -- 但
    CLAUDE.md 3.5 与 v2.0 S1.3 两侧都逐字禁止"未知值降级解释", 而兜底成
    unknown 正是那个降级. 真正的区别在: unknown 是 11 S5.1 闭集里的一个
    [有来源的]值(设备无生产者时 p2 就报它), 拿它去接一个[闭集外]的值, Qt
    就再也分不清"p2 说不知道"和"p2 说了我们不认识的话".

    NOTE 代价要写明: 一个坏的 health item 会让整条 state/robot 停发, 而它同时
    载着 battery/gps/motion/clock. 这个爆炸半径是有意接受的 -- p2 是
    health/summary 的唯一发布者, 出现闭集外的值只可能是 p2 坏了或契约变了,
    两种都该立刻响, 而不是让 Qt 拿着一份掺了假值的快照继续跑.

    变异体: to_v2_device_status 改成 .get(value, "unknown") => state/robot
    重新出现在 published 里, 本条红.
    """
    proj, bridge, _c = _proj()

    proj.tick(_state(health={"items": {
        "x": {"kind": "device", "state": "weird"}}}))

    assert "state/robot" not in dict(bridge.published), (
        "闭集外的 health state 被放行了: %r"
        % dict(bridge.published).get("state/robot", {}).get("devices"))
    assert proj.errors >= 1, "拒发了却没记 errors, 那这次拒绝无人可见"


# --- 主循环真的在驱动 -------------------------------------------------

def test_main_wiring_actually_ticks_the_projector():
    """*** 第三层的守门断言.

    投影器写好了而主循环不调它 = 上面每一条都全绿, 真机上 Qt 一条状态
    都收不到. 用 AST 查 main_wiring 里对 .tick( 的调用, NO 不 grep --
    本文件与模块头的注释里都写着 tick, grep 会命中注释.

    MUTATION: 注释掉主循环里的 cloud_projector.tick(hmi_state) -> 这里红.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p5_gateway" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    ticks = [n for n in ast.walk(ast.parse(src))
             if isinstance(n, ast.Call)
             and getattr(n.func, "attr", "") == "tick"
             and getattr(getattr(n.func, "value", None), "id", "")
             == "cloud_projector"]
    assert len(ticks) == 1, (
        "main_wiring 里对 cloud_projector.tick 的调用有 %d 处 -- "
        "出站面没有人驱动" % len(ticks))
