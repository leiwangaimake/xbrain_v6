"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_alarm_chain_e2e.py
Brief: 报警链跨进程形状端到端 -- p3 FenceSet -> p1 -> zone/state -> p5 (报警 E/D)

Description:
单测各自在进程内验逻辑, 但[跨进程的形状对不对]是单测看不见的: p3 build_fence_set
算的 crc32 p1 自算比对过不过? p1 build_fence_runtime_state 发的 state/fence, p5 的
D 读 active.rev 读得到吗? 这些接缝只有把[真实组件]用[真实消息形状]串起来才验得到 --
本文件就干这个(不起真 Zenoh/真进程, 那是真机 e2e 的事; 这里保证形状能接上, 免得
把可以在本机发现的错留到起栈那天).

覆盖两条接缝:
  E: p3.build_fence_set(warning) --crc32-> p1.compile_fence_set --hold-> p1.ZoneTracker
     (pose 在区内) -> zone_enter 事件.
  D: p5.CloudBridge 收 SET_ALARM_CONFIG -> fan-out -> 承接 accepted -> 登记待确认;
     p1.build_fence_runtime_state(rev 前进) 当 state/fence 喂给 p5 -> state/task done.
"""

import json

import pytest

pytestmark = pytest.mark.no_device

RID = "gj-001"

# 一个营区 allow + 一个报警区 warning(WGS84). build_fence_set 要求恰 1 allow.
_RING_ALLOW = [[34.60, 135.40], [34.80, 135.40], [34.80, 135.60],
               [34.60, 135.60]]
_RING_ZONE = [[34.697, 135.505], [34.698, 135.505], [34.698, 135.506],
              [34.697, 135.506]]


def _fence_rows(rev=1):
    """FencesDAO.list_active() 的行形状: (fence_id, name, role, kind, geom_json,
    hard_enforce, rev). 一个 allow(营区) + 一个 warning(报警区)."""
    return [
        ("f-camp", "camp", "allow", "polygon",
         json.dumps({"points": _RING_ALLOW}), 1, rev),
        ("f-zone", "gate", "warning", "polygon",
         json.dumps({"points": _RING_ZONE}), 0, rev),
    ]


def test_e_p3_fenceset_crc32_is_accepted_by_p1_and_zone_fires():
    """*** E 接缝: p3 的 FenceSet(真 crc32) 过 p1 自算比对, 报警区入侵报 zone_enter.

    这条把 F0(共享 crc32) + F1(p1 持有) + F2(zone) 用[p3 真的会发的那个 FenceSet]
    串起来 -- 单测里 p1 的 fixture 是自己造的 crc32, 这里是 p3 build_fence_set 造的.
    crc32 若两侧算法有一丝不同, p1.compile_fence_set 会抛, 这条立刻红.
    """
    from xbrain.p1_motion.fence.fence_set import compile_fence_set
    from xbrain.p1_motion.fence.zones import ZoneTracker
    from xbrain.p3_task.fence.fence_set import build_fence_set

    # p3 侧: 从 fence.db 行整形成 FenceSet(带 p3 算的 crc32).
    wire = build_fence_set(_fence_rows(rev=5), fence_set_id="fs-active", rev=5)

    # p1 侧: 自算 crc32 比对 + 持有. 不抛 = 两侧 crc32 逐字节一致(F0 单一真源).
    held = compile_fence_set(wire)
    assert held.rev == 5
    assert [p.poly_id for p in held.warning_polygons()] == ["f-zone"]

    # 机器人 pose 落进报警区 -> zone_enter(E). 区内点 (34.6975, 135.5055).
    tracker = ZoneTracker()
    assert tracker.observe(34.70, 135.51, held.warning_polygons()) == []  # 区外
    evs = tracker.observe(34.6975, 135.5055, held.warning_polygons())
    assert len(evs) == 1 and evs[0].kind == "zone_enter"
    assert evs[0].poly_id == "f-zone"


def test_d_p1_state_fence_advance_resolves_p5_alarm_terminal():
    """*** D 接缝: p1 的 state/fence(rev 前进) 让 p5 发 SET_ALARM_CONFIG done 终态.

    把 F3(p1 build_fence_runtime_state) + D(p5 读 active.rev) 用[p1 真的会发的那个
    state/fence 信封]串起来. p1 的 FenceRuntimeState 若字段/嵌套与 p5 的 _on_state_
    fence 读法对不上(active.rev 读不到), 终态发不出, 这条红.
    """
    from xbrain.p1_motion.fence.fence_set import (FenceSetHolder,
                                                  build_fence_runtime_state)
    from xbrain.p3_task.fence.fence_set import build_fence_set
    from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge

    # --- p5 侧: 收 SET_ALARM_CONFIG, fan-out, 承接 accepted, 登记待确认 ---
    puts = []

    class _Pub:
        def __init__(self, k):
            self._k = k

        def put(self, p):
            puts.append((self._k, p))

    class _Session:
        def __init__(self):
            self.subs = {}

        def declare_subscriber(self, k, cb):
            self.subs[k] = cb
            return object()

        def declare_publisher(self, k):
            return _Pub(k)

    class _Sample:
        def __init__(self, k, body):
            self.key_expr = k
            self.payload = json.dumps(body, ensure_ascii=False).encode("utf-8")

    session = _Session()
    mono = {"t": 1000.0}
    bridge = CloudBridge(session, RID, now_mono=lambda: mono["t"],
                         now_wall=lambda: 1785732000.0)
    bridge.wire()

    holder = FenceSetHolder()

    def _feed_state_fence(rev, applied):
        holder.accept(build_fence_set(_fence_rows(rev=rev),
                                      fence_set_id="fs-active", rev=rev))
        st = build_fence_runtime_state(holder.active, now_mono_s=applied,
                                       applied_mono_s=applied)
        session.subs["state/fence"](_Sample("state/fence", {"v": 1, "data": st}))

    # *** 稳态: 报警配置下发[前], p1 已在 1 Hz 广播 state/fence(rev=5). p5 据此
    # 记 rev0=5 -- 这样才测的是[rev 前进]那条路(而非"受理时无围栏视图"那条).
    _feed_state_fence(5, applied=0.5)

    region = {"id": "f-zone", "op": "upsert", "base_rev": 0, "name": "gate",
              "type": "alarm_region", "enabled": True, "applies_to": ["person"],
              "vertices": [{"latitude": 34.697, "longitude": 135.505},
                           {"latitude": 34.698, "longitude": 135.505},
                           {"latitude": 34.698, "longitude": 135.506}]}
    # src 必须是 SRC_QT("qt_hmi") -- is_cloud_frame 靠它区分[云端发来的]与网关
    # 自己重建发给 p3 的那条(防回环). 用别的值会被当成非云端帧静默忽略.
    alarm = {"v": 1, "rid": RID, "ts": 1785732000.5, "seq": 1, "src": "qt_hmi",
             "data": {"msg_id": "m-1", "task_id": "t-1",
                      "task_type": "SET_ALARM_CONFIG",
                      "payload": {"alarm_level": 1, "siren_level": 70,
                                  "duration_sec": 5, "cooldown_sec": 2.0,
                                  "alarm_window": {"start": "22:00",
                                                   "end": "05:00"},
                                  "rules": [], "regions": [region]}}}
    # 云端下发 -> fan-out 一条 cmd/geo fence upsert.
    session.subs["xbrain/%s/cmd/task" % RID](
        _Sample("xbrain/%s/cmd/task" % RID, alarm))
    # p3 承接: 单 enabled=true 区域 fan-out 出[两条](upsert + 激活, 审计 #3),
    # 子 cmd_id c-m-1:0 / c-m-1:1, 两条都回 accepted 才聚合成 accepted.
    for _cid in ("c-m-1:0", "c-m-1:1"):
        session.subs["cmd/geo/ack"](_Sample("cmd/geo/ack", {
            "schema": "task_ack_v1", "cmd_id": _cid, "result": "accepted",
            "code": "OK"}))

    def _task_results():
        return [json.loads(p.decode("utf-8"))["data"]
                for k, p in puts if k.endswith("/state/task")
                and json.loads(p.decode("utf-8"))["data"].get(
                    "message_type") == "result"]

    assert _task_results() == []                    # 受理了, 但还没确认生效(rev0=5)

    # 同版 state/fence 再来一帧(rev 还是 5) -> 没前进, 不发终态.
    _feed_state_fence(5, applied=0.6)
    assert _task_results() == []

    # 报警配置写入使围栏集换版 -> rev=6, p1 发新 state/fence -> 生效确认.
    _feed_state_fence(6, applied=2.0)

    res = _task_results()
    assert len(res) == 1 and res[0]["state"] == "done", (
        "p1 的 state/fence.active.rev 前进后, p5 没发 SET_ALARM_CONFIG done 终态 "
        "-- F3 与 D 的 state/fence 形状没接上")
    assert res[0]["task_type"] == "SET_ALARM_CONFIG"


def test_set_state_region_carries_target_in_obj_state():
    """*** 审计 #2: set_state 报警区的目标态必须在 obj.state, p3 才读得到.

    apply_set_state 从 (cmd.obj or {}).get("state") 读(geo_write); 放顶层会被
    parse_geo_command 丢弃 -> p3 回 "set_state needs obj.state". 本条把
    _region_to_fence 的 set_state 输出喂 p3 解析器, 断言 cmd.obj.state 拿得到.

    MUTATION: _region_to_fence set_state 放顶层 state(不放 obj) -> cmd.obj 为
    None -> 这里红.
    """
    from xbrain.p3_task.ingest.geo_command import parse_geo_command
    from xbrain.p5_gateway.inbound.task_router import _region_to_fence

    region = {"id": "f-zone", "op": "set_state", "base_rev": 3, "enabled": False}
    # _region_to_fence 现返回列表(审计 #3), set_state 仍是单条, 取 [0].
    parsed = parse_geo_command(_region_to_fence("c-1", region)[0])
    assert parsed.action == "set_state" and parsed.type == "fence"
    assert (parsed.obj or {}).get("state") == "disabled", (
        "set_state 目标态没落在 obj.state -- p3 apply_set_state 读不到(审计 #2)")


def test_enabled_region_fans_out_upsert_plus_activate():
    """*** 审计 #3: enabled=true 报警区 -> upsert(建 draft) + set_state->active.

    没有激活那条, p3 的 upsert 把新 fence 留 draft(不进 list_active)-> 不广播 ->
    p1 不持有 -> E 不报 zone_enter, 且 active.rev 不前进 -> D 恒 timeout. 云端激活
    无需 L2(11 S7.9.5 cloud 列  无 L2), 用 force(upsert 后 rev 预测不了).

    MUTATION: _region_to_fence 对 enabled=true 只回 upsert(不追激活) -> len 1 -> 红.
    """
    from xbrain.p3_task.ingest.geo_command import parse_geo_command
    from xbrain.p5_gateway.inbound.task_router import _region_to_fence

    region = {"id": "f-zone", "op": "upsert", "base_rev": 0, "name": "gate",
              "type": "alarm_region", "enabled": True, "applies_to": ["person"],
              "vertices": [{"latitude": 34.697, "longitude": 135.505},
                           {"latitude": 34.698, "longitude": 135.505},
                           {"latitude": 34.698, "longitude": 135.506}]}
    cmds = _region_to_fence("c-1", region)
    assert len(cmds) == 2, "enabled=true 没产出 upsert+激活两条(审计 #3)"
    up, act = cmds
    assert up["action"] == "upsert" and up["obj"]["geom"]["role"] == "warning"
    # 激活那条 p3 认得(set_state, obj.state=active), 且 force(云端无 L2, rev 预测不了).
    parsed = parse_geo_command(act)
    assert parsed.action == "set_state" and parsed.type == "fence"
    assert (parsed.obj or {}).get("state") == "active"
    assert act["force"] is True

    # enabled=false 只 upsert(留 draft, "存了不启用").
    off = _region_to_fence("c-1", dict(region, enabled=False))
    assert len(off) == 1 and off[0]["action"] == "upsert"
