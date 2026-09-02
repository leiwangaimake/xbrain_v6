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


def test_manifest_has_a_keepalive_period_not_change_only():
    """*** manifest 必须周期重发, NO 不能只在变化时发.

    v2.0 S4.5 逐字: "session 建立后 2 秒内必须发布一份 full=true 的全量清单".
    我方拿不到 session 建立信号(Qt 的订阅是 Zenoh 内部的事, 不产生回调), 原
    实现用"网关启动"近似, 结果 manifest 一辈子只在 p5 开机那一瞬发一次 --
    而 Zenoh 不给后加入的订阅者补发历史消息, Qt 只要不是恰好在那一秒连着就
    永远收不到. 2026-09-02 甲方实测反馈"没收到 state/geo/manifest".

    连锁后果: v2.0 S2.1 要求 recorded_path_id "必须存在于当前 manifest",
    拿不到清单 = 一条导航任务都发不出来.

    周期必须 <= 2.0 s: 那是 v2.0 承诺的上界本身, 取更长就已经违约.

    变异体: 改回 None 或改成 5.0 => 本条红.
    """
    from xbrain.p5_gateway.runtime.cloud_wiring import OUTBOUND_PERIODS

    period = OUTBOUND_PERIODS["state/geo/manifest"]
    assert period is not None, (
        "manifest 仍是[只在变化时发] -- 后连的 Qt 永远收不到")
    assert period <= 2.0, (
        "周期 %.1fs 超过 v2.0 S4.5 承诺的 2 秒上界" % period)


def test_manifest_reads_the_real_geocache_field_names():
    """*** 用 GeoCache 的真实键名喂, NO 不用想象的形状.

    _manifest_objects 原来读 snap["keypoints"] / snap["fences"] / obj["id"],
    而 GeoCache 存的是 snap["waypoints"] / (没有 fences) / obj["geo_id"].
    后果: 航点与围栏一条都不进 manifest; 路径能取到但 id 是空串.

    变异体: 把 waypoints 改回 keypoints => 本条红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import _manifest_objects

    snap = {
        "waypoints": [{"geo_id": "w-east_gate", "name": "东门岗", "rev": 1,
                       "geom": {"lat": 31.2, "lon": 121.4}}],
        "routes": [{"geo_id": "r-oil_area", "name": "油库线", "rev": 2}],
        "fences": [{"geo_id": "f-alarm", "name": "报警区", "rev": 1}],
    }
    out = {o["geo_id"]: o for o in _manifest_objects(snap)}
    assert set(out) == {"w-east_gate", "r-oil_area", "f-alarm"}, (
        "三类没有都进来: %s" % sorted(out))
    assert out["r-oil_area"]["type"] == "recorded_path"
    assert out["w-east_gate"]["type"] == "waypoint"
    assert out["f-alarm"]["type"] == "alarm_region"
    assert out["w-east_gate"]["latitude"] == 31.2, "geom 里的坐标没取出来"


def test_a_missing_geo_id_is_skipped_not_synthesised():
    """*** 取不到 id 就跳过, NO 不能"补个前缀"合成一个.

    原写法是 gid if gid.startswith("r-") else "r-" + gid -- 它把[取不到值]
    与[值没有前缀]合成同一个动作, 于是空串被悄悄变成 "r-" 发了出去.
    甲方 2026-09-02 收到的正是这批 "r-", 反馈"geo_id 名称不对, 格式应为
    r-[a-z0-9_]{1,40}" -- 完全准确, 而拒绝理由指向格式, 没人会想到是我方
    组装时丢了值.

    一个不完整的清单比一个掺了假 id 的清单诚实: 前者 Qt 少看到一条, 后者
    Qt 会拿着 "r-" 去下发.

    变异体: 去掉正则校验直接 append => 本条红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import _manifest_objects

    out = _manifest_objects({
        "routes": [{"geo_id": "", "name": "丢了id的路径", "rev": 1},
                   {"name": "根本没有id字段", "rev": 1},
                   {"geo_id": "r-good_one", "name": "正常的", "rev": 1}],
    })
    ids = [o["geo_id"] for o in out]
    assert ids == ["r-good_one"], (
        "合成了假 id 或漏掉了正常的: %r" % ids)
    assert "r-" not in [i for i in ids if len(i) <= 2], "又造出了裸前缀"


def test_manifest_rejects_ids_that_violate_the_v2_regex():
    """*** 连字符在主体里是非法的, 必须挡住.

    v2.0 S2.1: r-[a-z0-9_]{1,40} -- 类型前缀后那一位是连字符, 主体只允许
    下划线. 一个 r-oil-area 在机内 cmd/geo 会被收下(机内规则宽松), 到云端
    就被网关 field_validate 拒掉. manifest 发出去之前先挡, Qt 才不会拿到
    一个它自己也用不了的 id.

    变异体: 把正则放宽成 [a-z0-9_-] => 本条红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import _manifest_objects
    out = _manifest_objects({"routes": [
        {"geo_id": "r-oil-area", "name": "带连字符", "rev": 1},
        {"geo_id": "r-oil_area", "name": "合规", "rev": 1}]})
    assert [o["geo_id"] for o in out] == ["r-oil_area"]


def test_snapshot_lists_come_from_the_task_state_broadcast():
    """*** v2.0 S3.2 的 current/queue/suspended 从 11 S4.4 TaskState 取.

    契约把 state/task 定成云端的任务态通道(S2.2.2 消费者一栏逐字列了云端),
    而且[只有它带 route_id 与 started_ts] -- v2.0 S3.2 的两个必填字段.
    query/tasks 装的是 TaskCard(17 S6.8.4 的 HMI 面板形状), 填不出这两个.

    本条早先断言的是反过来的优先级: 那时 P3 的 state/task 还是占位形状
    (只有 active_task 一条), 广播里 queue 与 suspended 恒空, 只能拿 TaskCard
    凑全量. P3 改发真正的 TaskState 之后那个理由没有了.

    变异体: _all_tasks 改回优先 cloud_tasks => route_id 变 null, 本条红.
    """
    proj, bridge, _c = _proj()

    st = _state()
    # 广播 = TaskState 扁平化后的三段, 带 v2.0 要的字段.
    st["tasks"] = [
        {"task_id": "t-run", "type": "goto", "state": "running",
         "route_id": "r-oil_area", "started_ts": 1788339000.0,
         "progress": None},
        {"task_id": "t-q1", "type": "goto", "state": "ready"},
        {"task_id": "t-s1", "type": "patrol", "state": "suspended"},
    ]
    # TaskCard 那份仍在(HMI 面板用), 但不该被云端快照选中.
    st["cloud_tasks"] = [
        {"task_id": "t-card", "task_type": "goto", "state": "running"},
    ]
    proj.tick(st)

    snap = dict(bridge.published)["state/task"]
    assert snap["current"] and snap["current"]["task_id"] == "t-run"
    assert [t["task_id"] for t in snap["queue"]] == ["t-q1"], (
        "queue 没填出来: %r" % snap["queue"])
    assert [t["task_id"] for t in snap["suspended"]] == ["t-s1"], (
        "suspended 没填出来: %r" % snap["suspended"])
    # 这两个字段是本次修复的靶心: TaskCard 那条路填不出来.
    assert snap["current"]["route_id"] == "r-oil_area"
    assert snap["current"]["started_ts"] == 1788339000.0


def test_the_task_card_source_is_still_the_fallback():
    """广播还没到(p3 刚起 / 丢了一拍)时不能让整个快照空掉: 回落到
    query/tasks 的 TaskCard, 少两个字段好过一条任务都看不到.

    变异体: 删掉 _all_tasks 的回落分支 => 本条红.
    """
    proj, bridge, _c = _proj()
    st = _state()
    st["tasks"] = []                      # 广播还没来
    st["cloud_tasks"] = [
        {"task_id": "t-card", "task_type": "goto", "state": "running"}]
    proj.tick(st)
    snap = dict(bridge.published)["state/task"]
    assert snap["current"] and snap["current"]["task_id"] == "t-card"
    # TaskCard 带不出 route_id -- 诚实地是 null, 不是编一个.
    assert snap["current"]["route_id"] is None

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
