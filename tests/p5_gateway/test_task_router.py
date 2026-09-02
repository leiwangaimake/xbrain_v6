"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_router.py
Brief: A-2 task_type 五值拆分 + 禁止项显式拒绝 + 云端不绕过仲裁

Description:
甲方把五种任务全发到一条 cmd/task, 用 task_type 区分; 我方机内按域分 key.
本文件守那次拆分, 重点在三处:

*** 一 云端是[输入源之一], 不是特权通道.
本机还有本地 MIC(voice)与将来的微信(wecom). 三者共用同一批机内 key 与同一套
仲裁, 区别只在 origin/source 字段(11 S7.9.5 的四值闭集). 所以云端接入不新建
通路 -- 而"不新建"这件事要有断言守着, 否则最省事的写法就是给云端开一条直达
执行层的近路, 那会让"云端下发时本地喊话是否该被打断"有两套答案.

*** 二 禁止项必须显式拒绝并分类.
v2.0 分了两种: "本期禁止的能力"(MANUAL_VELOCITY, dog_to_pc)与"已不存在的
旧名称"(RETURN_HOME 等十个). 两者回的 detail.code 不同 --
前者 E_CHANNEL_DENIED(这个能力不对云端开放), 后者 E_NOT_IMPLEMENTED(这个
名字已经没有了). 混为一谈会让 Qt 开发者以为改个名字就能用.

*** 三 origin 恒 cloud, NO 不从报文取.
11 CH-1: 通道即权限. 让发起方自称 origin 等于没有边界 -- 一条云端报文
声称自己是 voice, 就能拿到语音通道的权限.

Boundaries: 只测拆分与拒绝. 不测业务字段合法性(waypoints 经纬度之类) --
那由机内既有解析器做, 而那些解析器正是语音链路也在用的.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


def _data(task_type, payload, msg_id="msg-1", task_id="task-1"):
    return {"msg_id": msg_id, "task_id": task_id,
            "task_type": task_type, "payload": payload}


#: 会在机内[创建任务行]的 task_type 及其最小合法 payload.
#: 只列 create 类: STOP_TASK 是对既有任务的状态迁移, SET_ALARM_CONFIG 走
#: cmd/geo, AUDIO_CONTROL 走 cmd/audio/speak -- 三者都不产生 tasks 行, 也就
#: 不经过 15 S12 的闭集检查. 把它们混进来会让判据看起来覆盖更广而实际在测
#: 无关的东西.
_CREATE_TASK_SAMPLES = {
    "GOTO_KEYPOINT": {
        "coordinate_system": "WGS84",
        "recorded_path_id": "r-north",
        "waypoints": [{"id": "w-1", "name": "x", "latitude": 31.2,
                       "longitude": 121.4, "altitude": 8.4,
                       "arrival_radius_m": 3.0}],
    },
}


def _reject(task_type, payload):
    from xbrain.p5_gateway.inbound.cloud_inbound import InboundReject
    from xbrain.p5_gateway.inbound.task_router import route
    try:
        route(_data(task_type, payload))
    except InboundReject as exc:
        return exc.fields
    return None


# --- 一 五值各自落到对的机内 key --------------------------------------

def test_all_five_open_types_route_somewhere():
    """*** 五个开放类型逐个必须有落点.

    只测 GOTO_KEYPOINT 的话, 一个只认它的实现能通过 -- 而 Qt 联调时会把
    五个都发一遍.
    """
    from xbrain.p5_gateway.inbound.task_router import (KEY_AUDIO, KEY_ESTOP,
                                                       KEY_GEO, KEY_TASK,
                                                       OPEN_TASK_TYPES, route)

    payloads = {
        "GOTO_KEYPOINT": {"coordinate_system": "WGS84",
                          "recorded_path_id": "r-a",
                          "waypoints": [{"id": "w-1", "name": "x",
                                         "latitude": 31.2, "longitude": 121.4,
                                         "altitude": 8.4,
                                         "arrival_radius_m": 3.0}]},
        "STOP_TASK": {"target_task_id": "task-x", "action": "cancel"},
        "ESTOP": {"action": "stop"},
        # SET_ALARM_CONFIG fan-out: 带一个 region 才产出命令(空 regions -> 0
        # 条, 那条路径批B 才有 cmd/config). 这里给一个合法 alarm_region.
        "SET_ALARM_CONFIG": {"alarm_level": 1, "siren_level": 70,
                             "duration_sec": 5, "cooldown_sec": 2.0,
                             "alarm_window": {"start": "22:00", "end": "05:00"},
                             "rules": [],
                             "regions": [{"id": "f-x", "op": "upsert",
                                          "base_rev": 0, "name": "z",
                                          "type": "alarm_region",
                                          "enabled": True,
                                          "applies_to": ["person"],
                                          "vertices": [
                                              {"latitude": 31.0,
                                               "longitude": 121.0},
                                              {"latitude": 31.1,
                                               "longitude": 121.0},
                                              {"latitude": 31.1,
                                               "longitude": 121.1}]}]},
        "AUDIO_CONTROL": {"mode": "pc_to_dog", "action": "start"},
    }
    assert set(payloads) == set(OPEN_TASK_TYPES), (
        "本用例的样例集与 OPEN_TASK_TYPES 不一致 -- 新增类型时要同步")
    want = {"GOTO_KEYPOINT": KEY_TASK, "STOP_TASK": KEY_TASK,
            "ESTOP": KEY_ESTOP, "SET_ALARM_CONFIG": KEY_GEO,
            "AUDIO_CONTROL": KEY_AUDIO}
    for task_type, payload in payloads.items():
        # route 现在返回 [(key, payload), ...]; 每类的第一条落点即验收目标.
        cmds = route(_data(task_type, payload))
        assert cmds, "%s 没有产出任何机内命令" % task_type
        key = cmds[0][0]
        assert key == want[task_type], (
            "%s 落到了 %s, 应该是 %s" % (task_type, key, want[task_type]))


def test_estop_goes_to_its_own_key_not_the_task_queue():
    """*** v2.0 S3.3 逐字: 后端必须单独订阅 cmd/estop.

    "不得经普通任务 FIFO, 限流或降级处理". 把急停排在普通任务后面, 意味着
    前面有一条正在处理的任务时急停要等 -- 而急停的全部意义就是不等.
    """
    from xbrain.p5_gateway.inbound.task_router import KEY_ESTOP, KEY_TASK, route

    (key, _body), = route(_data("ESTOP", {"action": "stop"}))
    assert key == KEY_ESTOP and key != KEY_TASK


def test_goto_becomes_the_internal_submit_shape():
    """GOTO_KEYPOINT -> 11 S7.2 的 {action:"submit", task:{...}}.

    这是 p3_task 既有解析器认的形状 -- 语音下发的任务也是这个形状.
    两条输入走同一套判定, 围栏/路径版本这些才不会有两个答案.
    """
    from xbrain.p5_gateway.inbound.task_router import route

    (_key, body), = route(_data("GOTO_KEYPOINT", {
        "coordinate_system": "WGS84", "recorded_path_id": "r-north",
        "waypoints": [{"id": "w-1", "name": "x", "latitude": 31.2,
                       "longitude": 121.4, "altitude": 8.4,
                       "arrival_radius_m": 3.0}]}))
    assert body["action"] == "submit"
    assert body["task"]["task_id"] == "task-1"
    # *** 2026-09-02 订正: 本行原写 body["task"]["recorded_path_id"], 与本条
    # docstring 自己的理由("p3_task 既有解析器认的形状")直接矛盾 -- p3 读的是
    # 11 S7.2 的 route_id, recorded_path_id 是 v2.0 那侧的 wire 名, 它一个字
    # 都不认. 判据抄的是被测代码自己, 于是两边一起错却一直绿, 而线上甲方发的
    # 路径引用静默丢失(库里 route_geo_id=NULL).
    assert body["task"]["route_id"] == "r-north"
    assert "recorded_path_id" not in body["task"]


def test_stop_task_requires_an_explicit_target():
    """*** v2.0 S3.2 逐字禁止"省略即当前任务".

    11 S7.2 给的第一条理由是要害: 队列是活的 -- 操作员看到"A 在跑"到命令
    抵达之间 A 可能已结束而 B 开始, 简写会暂停错的那条, 而日志里看不出
    发生过这件事.
    """
    from xbrain.p5_gateway.outbound.error_map import CODE_INVALID_FIELD

    fields = _reject("STOP_TASK", {"action": "cancel"})
    assert fields and fields["error_code"] == CODE_INVALID_FIELD
    assert fields["detail"]["field"] == "target_task_id"


def test_stop_task_action_is_a_closed_set():
    """pause|resume|cancel 三值. stop / abort 之类要拒."""
    for bad in ("stop", "abort", "kill", None):
        assert _reject("STOP_TASK",
                       {"target_task_id": "t", "action": bad}), bad


# --- 二 禁止项分类拒绝 ------------------------------------------------

def test_manual_velocity_is_channel_denied():
    """*** v2.0 S3.6: 云端遥控本期未立项, 后端收到必须拒绝.

    这是客户联调时[会故意测]的一条 -- 他们要确认我方不会执行速度.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.outbound.error_map import CODE_TASK_UNSUPPORTED

    fields = _reject("MANUAL_VELOCITY", {"vx": 1.0})
    assert fields["error_code"] == CODE_TASK_UNSUPPORTED
    assert fields["detail"]["code"] == errors.E_CHANNEL_DENIED


def test_retired_names_are_not_implemented_not_channel_denied():
    """*** 两类禁止项的 detail.code 不同, 不得混为一谈.

    "本期禁止的能力"与"已不存在的旧名称"对 Qt 开发者的含义完全不同:
    前者意味着"以后可能开", 后者意味着"这个名字没有了, 去看新契约".
    混报会让人以为改个名字就能用.
    """
    from xbrain.common import errors
    from xbrain.p5_gateway.inbound.task_router import RETIRED_TASK_TYPES

    assert RETIRED_TASK_TYPES, "旧名称清单是空的"
    for name in RETIRED_TASK_TYPES:
        fields = _reject(name, {})
        assert fields, name
        assert fields["detail"]["code"] == "E_TASK_UNSUPPORTED", name
        # 报错里要给出当前允许的清单 -- 否则 Qt 开发者要去翻文档.
        assert fields["detail"]["allowed"], name
        # *** reason 必须点名"已不是本期协议能力", 而不只是"不支持".
        #
        # 第一版只查 detail.code, 而删掉 RETIRED 分支后这些名字会落到
        # 下一个"不支持的任务类型"分支 -- detail.code 同样是
        # E_NOT_IMPLEMENTED, 于是变异体不红. 两条分支的区别全在 reason:
        # "已不是本期协议能力"(去看新契约)与"不支持"(可能拼错了),
        # 对 Qt 开发者的指向完全不同.
        assert "no longer part of this phase protocol" in fields["reason"], (
            "%s 的拒绝理由没有点明它是退役名称: %r" % (name, fields["reason"]))


def test_dog_to_pc_is_refused():
    """*** v2.0 S3.5 逐字: 不得静默启用机上麦克风上行.

    静默启用是个隐私问题, 而它不产生任何可见现象 -- 没有断言的话谁也
    不会发现.
    """
    from xbrain.common import errors

    fields = _reject("AUDIO_CONTROL", {"mode": "dog_to_pc", "action": "start"})
    assert fields["detail"]["code"] == errors.E_CHANNEL_DENIED


def test_keep_in_region_is_refused_with_the_verbatim_reason():
    """*** keep_in 一律拒, 理由逐字.

    v2.0 S3.4: "regions[] 只允许 alarm_region, 禁止 keep_in". 评审 R10.5
    给了兜底措辞"营区边界不经此通道配置".
    * 这不是形式主义: keep_in 是安全围栏, 用报警配置通道改它意味着一条
    改报警的命令能改掉机器人的活动边界.
    """
    fields = _reject("SET_ALARM_CONFIG", {
        "alarm_level": 1,
        "regions": [{"id": "f-camp", "type": "keep_in", "op": "upsert"}]})
    assert fields, "keep_in 被放行了"
    assert "camp keep-in boundary is not configured through this channel" in fields["reason"]
    assert fields["detail"]["region_id"] == "f-camp"


def test_an_unknown_task_type_is_refused():
    """闭集外的值必抛(CLAUDE.md 3.5), NO 不静默映射到某个现有能力."""
    assert _reject("SOMETHING_NEW", {})
    assert _reject(None, {})


# --- 三 origin 恒 cloud, 不绕过仲裁 -----------------------------------

def test_origin_is_always_cloud_never_taken_from_the_frame():
    """*** 11 CH-1: 通道即权限.

    让发起方自称 origin 等于没有边界 -- 一条云端报文声称自己是 voice,
    就能拿到语音通道的权限.

    MUTATION: 改成 payload.get("source", CLOUD_ORIGIN) -> 这里红.
    """
    from xbrain.p5_gateway.inbound.task_router import CLOUD_ORIGIN, route

    # 报文里塞一个假的 source/origin, 结果必须仍是 cloud.
    (_k, body), = route(_data("GOTO_KEYPOINT", {
        "coordinate_system": "WGS84", "recorded_path_id": "r-a",
        "waypoints": [{"id": "w-1", "name": "x", "latitude": 31.2,
                       "longitude": 121.4, "altitude": 8.4,
                       "arrival_radius_m": 3.0}],
        "source": "voice", "origin": "voice"}))
    assert body["source"] == CLOUD_ORIGIN, "origin 被报文里的值覆盖了"

    # SET_ALARM_CONFIG fan-out: 每条 fence 命令的 origin 也必须恒 cloud, 不被
    # 报文里的 "hmi" 覆盖.
    cmds2 = route(_data("SET_ALARM_CONFIG",
                        {"alarm_level": 1, "siren_level": 70,
                         "duration_sec": 5, "cooldown_sec": 2.0,
                         "alarm_window": {"start": "22:00", "end": "05:00"},
                         "rules": [],
                         "regions": [{"id": "f-x", "op": "upsert",
                                      "base_rev": 0, "name": "z",
                                      "type": "alarm_region", "enabled": True,
                                      "applies_to": ["person"],
                                      "vertices": [
                                          {"latitude": 31.0, "longitude": 121.0},
                                          {"latitude": 31.1, "longitude": 121.0},
                                          {"latitude": 31.1,
                                           "longitude": 121.1}]}],
                         "origin": "hmi"}))
    assert cmds2[0][1]["origin"] == CLOUD_ORIGIN


def test_cloud_origin_is_in_the_shared_closed_set():
    """*** 云端复用既有的四值闭集, NO 不新建一套.

    11 S7.9.5 的 geo_origin 闭集是 cloud|wecom|hmi|voice -- 那正是本机的
    四种输入形式. 云端接入只是往既有通路里多一个 origin, 互斥与优先级由
    既有仲裁器管, 与语音链路完全一致.

    如果这里用了一个闭集外的新值, 机内的通道权限判定会拒绝它(闭集外必抛),
    而那种失败会在联调时表现为"云端指令全部被拒且原因难懂".
    """
    from xbrain.p3_task.ingest.geo_command import GEO_ORIGIN
    from xbrain.p5_gateway.inbound.task_router import CLOUD_ORIGIN

    assert CLOUD_ORIGIN in GEO_ORIGIN, (
        "云端 origin %r 不在 11 S7.9.5 的闭集里: %s"
        % (CLOUD_ORIGIN, sorted(GEO_ORIGIN)))
    # 四种输入形式都在同一个闭集里 -- 这是"云端不是特权通道"的结构保证.
    for other in ("voice", "hmi", "wecom"):
        assert other in GEO_ORIGIN, other


def test_goto_uses_the_s7_2_field_names_not_the_v2_wire_names():
    """*** 机内 task 的字段名必须是 11 S7.2 的, NO 不是 v2.0 的 wire 名.

    S7.2 的 task 结构逐字是 {task_id, type, priority, route_id, params,
    resume_policy, not_before_ts}. p3 的 task_row_from_command 按这些名字取:
      route_geo_id <- body["route_id"]
      mission_json <- body["params"]
    本函数原来发 recorded_path_id 与平铺的 waypoints/coordinate_system,
    两个名字 p3 都不认 -> 路径引用与全部航点[静默丢失], 而 ack 照常 accepted,
    任务照常入库, 从外面看一切正常.

    2026-09-02 联调实测: 甲方发 r-night + 2 个航点, 库里 route_geo_id=NULL,
    mission_json={"source":"cloud","params":{}}.

    连带代价(task_row.py 头注): route_geo_id 为空时, geo_refs(11 S7.9.4 删除
    确认的影响集)不得不同时按名字匹配 -- 纯 id 匹配会对一条正被三个任务引用
    的路径回答"没有任何引用".

    变异体: 把 route_id 改回 recorded_path_id => 本条红.
    """
    from xbrain.p5_gateway.inbound.task_router import route

    (_key, body), = route(_data("GOTO_KEYPOINT",
                                _CREATE_TASK_SAMPLES["GOTO_KEYPOINT"]))
    task = body["task"]
    assert task.get("route_id") == "r-north", (
        "route_id 没带上(p3 读的是这个名字): %r" % task.get("route_id"))
    assert "recorded_path_id" not in task, (
        "还在用 v2.0 的 wire 名 -- p3 不认")
    params = task.get("params") or {}
    assert params.get("waypoints"), "航点没进 params, mission_json 会是空的"
    assert params["waypoints"][0]["id"] == "w-1"
    assert params.get("coordinate_system") == "WGS84"


def test_the_payload_survives_into_the_task_row():
    """*** 端到端: 网关发的形状必须能被 p3 的入库函数取出来.

    上一条只测网关的输出形状; 这一条把它真的喂给 task_row_from_command,
    确认 route_geo_id 与 mission_json 落到位. 两个模块各自"对"而合起来错,
    正是这个缺陷的形状 -- 单测各测一半就都是绿的.

    变异体: 网关改回 recorded_path_id => route_geo_id 变空, 本条红.
    """
    from dataclasses import replace as _replace
    from xbrain.p3_task.ingest.task_row import task_row_from_command
    from xbrain.p5_gateway.inbound.task_router import route
    from xbrain.p3_task.ingest.task_command import parse_task_command

    (_key, body), = route(_data("GOTO_KEYPOINT",
                                _CREATE_TASK_SAMPLES["GOTO_KEYPOINT"]))
    cmd = parse_task_command(body)
    row = task_row_from_command(cmd, submit_seq=1, now_mono_ms=1000,
                                created_at="2026-09-02T00:00:00Z")
    assert row.route_geo_id == "r-north", (
        "route_geo_id 没落库: %r" % row.route_geo_id)
    import json as _json
    mission = _json.loads(row.mission_json)
    assert mission["params"].get("waypoints"), (
        "航点没进 mission_json: %r" % mission)


def test_every_emitted_task_type_is_in_p3_closed_set():
    """*** 网关发出的 task.type 必须是 15 S12 闭集的成员.

    这条守的是 2026-09-01 联调前实测抓到的缺陷: _goto 发的是 "goto_keypoint",
    而 15 S12 的 TASK_TYPES 是 patrol|goto|charge|return_home|standby|teach|
    follow. task_row_from_command 查不中即抛, 网关翻成 error_code 3001 /
    E_INTERNAL -- 也就是[每一条云端导航指令都被拒].

    *** 为什么原有的断言全部漏掉它.
    test_goto_becomes_the_internal_submit_shape 与 test_cloud_bridge 的
    test_a_goto_lands_on_the_internal_key_in_the_s7_2_shape 都只把本函数的
    输出与一个写死的字面量比对 -- 而那个字面量抄的正是被测代码自己. 两侧各
    自自洽, 中间那条缝没有断言(CLAUDE.md 3.2 形态6: 扫描面不声明).
    同一个文件里的 test_cloud_origin_is_in_the_shared_closed_set 对 origin
    轴做了正确的跨边界检查; 漏的是 task.type 轴.

    *** 写成遍历而不是只钉 goto: 将来任何新增的 task_type 路由都自动进这条
    判据. 只断言 goto 的话, 下一个 create-task 类型会重演同一个缺陷.

    变异体: 把 _goto 的 "goto" 改回 "goto_keypoint" => 本条必红.
    """
    from xbrain.p3_task.persistence.schema_task import TASK_TYPES
    from xbrain.p5_gateway.inbound.task_router import route

    emitted = {}
    for task_type, payload in _CREATE_TASK_SAMPLES.items():
        for _key, body in route(_data(task_type, payload)):
            if body.get("action") == "submit" and "task" in body:
                emitted[task_type] = body["task"].get("type")

    assert emitted, (
        "一个 submit 都没抽到 -- 判据瞎了(CLAUDE.md 3.2 形态6). "
        "_CREATE_TASK_SAMPLES 的 payload 可能已与 route 的校验脱节")

    for wire_name, internal_type in sorted(emitted.items()):
        assert internal_type in TASK_TYPES, (
            "v2.0 的 %s 落到机内 task.type=%r, 不在 15 S12 闭集 %s 里. "
            "p3 的 task_row_from_command 会抛, 云端收到 E_INTERNAL"
            % (wire_name, internal_type, sorted(TASK_TYPES)))


def test_goto_lands_the_same_task_type_as_the_voice_path():
    """*** 云端与语音是同一个业务动作, 必须落同一个 task_type.

    上一条只保证[在闭集里]; 在闭集里而选错值(比如落成 patrol)同样是缺陷,
    且症状更隐蔽 -- 任务会被受理并入库, 只是语义错了, 调度优先级与终态解释
    都跟着错, 而没有任何东西会报错.

    锚点是语音侧的 B01: goto_waypoint -> "goto"
    (p4_agent/runtime/task_request.py 的 _TASK_CREATE_INTENTS). 从那张表读,
    NO 不在这里再写一遍字面量 -- 写死的话两处会各自漂移, 而这条断言的全部
    价值就是不让它们漂.

    变异体: 把 _goto 改成任何别的闭集成员(patrol/follow) => 本条必红,
    而上一条仍绿. 两条判据合起来才既管[合法]又管[正确].
    """
    from xbrain.p4_agent.runtime.task_request import _TASK_CREATE_INTENTS
    from xbrain.p5_gateway.inbound.task_router import route

    voice_goto = _TASK_CREATE_INTENTS["goto_waypoint"]

    (_key, body), = route(_data("GOTO_KEYPOINT",
                                _CREATE_TASK_SAMPLES["GOTO_KEYPOINT"]))
    assert body["task"]["type"] == voice_goto, (
        "云端 GOTO_KEYPOINT 落 %r, 语音 goto_waypoint 落 %r -- "
        "同一个动作在机内有了两套语义"
        % (body["task"]["type"], voice_goto))


def test_routing_targets_are_the_existing_internal_keys():
    """*** 拆分的落点必须是[机内既有 key], NO 不为云端新开一条.

    给云端开专用 key 的写法很省事, 代价是执行层出现两条并行通路 --
    于是仲裁只看得见其中一条, 而"云端下发时本地喊话是否该被打断"就有了
    两套答案.

    这里查落点确实是机内模块正在订阅的那几条.
    """
    from xbrain.p5_gateway.inbound.task_router import (KEY_AUDIO, KEY_ESTOP,
                                                       KEY_GEO, KEY_TASK)

    # 这四条都是 11 S2.2 登记的机内 key, 且语音/HMI 链路也在用.
    assert KEY_TASK == "cmd/task"
    assert KEY_ESTOP == "cmd/estop"
    assert KEY_GEO == "cmd/geo"
    assert KEY_AUDIO == "cmd/audio/speak"
    # 没有任何一条带 cloud 前缀或后缀 -- 那会是"专用通路"的形状.
    for key in (KEY_TASK, KEY_ESTOP, KEY_GEO, KEY_AUDIO):
        assert "cloud" not in key, "%s 看起来是给云端专开的通路" % key


# --- 音频 stream_id 的两条方向 ---------------------------------------

def test_start_must_not_carry_a_stream_id():
    """v2.0 S2.5: stream_id 由后端在 start 的 ack 里分配."""
    assert _reject("AUDIO_CONTROL",
                   {"mode": "pc_to_dog", "action": "start",
                    "stream_id": "audio-1"})


def test_exit_must_carry_the_original_stream_id():
    """退出必须带原 ID; 后端不得为退出请求分配新 ID.

    一个给退出也发新 ID 的实现, 会让 Qt 无法确认自己退的是不是刚才那一路.
    """
    assert _reject("AUDIO_CONTROL",
                   {"mode": "pc_to_dog", "action": "exit_broadcast"})
    from xbrain.p5_gateway.inbound.task_router import route
    (_k, body), = route(_data("AUDIO_CONTROL",
                              {"mode": "pc_to_dog", "action": "exit_broadcast",
                               "stream_id": "audio-gj001-0001"}))
    assert body["stream_id"] == "audio-gj001-0001"
