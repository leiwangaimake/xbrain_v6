"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_router.py
Brief: v2.0 task_type 五值 -> 机内 key 与契约形状 (A-2)

Description:
甲方把五种任务全发到一条 cmd/task 上, 用 data.task_type 区分. 我方机内则是
按域分 key 的(任务 / 急停 / 地理 / 音频各一条). 本模块做那次拆分.

  v2.0 task_type      机内 key              机内形状
  ------------------  --------------------  ---------------------------
  GOTO_KEYPOINT       cmd/task              11 S7.2 {action:"submit", task}
  STOP_TASK           cmd/task              11 S7.2 {action:pause|resume|cancel}
  ESTOP               cmd/estop             {type:"estop", action:"stop"}
  SET_ALARM_CONFIG    cmd/geo               11 S7.9 {action:"upsert", origin}
  AUDIO_CONTROL       cmd/audio/speak       进入/退出喊话
  MANUAL_VELOCITY     -- 本期禁止, 回 1006 --

*** 云端是[输入源之一], 不是特权通道.
本机还有本地 MIC(voice) 与将来的微信(wecom). 三者共用同一批机内 key 与同一套
仲裁, 区别只在 origin/source 字段:
  cloud  云端 Qt
  voice  本地 MIC
  wecom  微信(未实现)
  hmi    本机 HMI
这四个值就是 11 S7.9.5 的 geo_origin 闭集, 已在 common/enums 里. 所以云端
接入[不新建一套通路], 只是往既有通路里多一个 origin -- 互斥与优先级由既有的
仲裁器管, 与语音链路完全一致.
=> NO 绝不给云端开一条绕过仲裁的近路: 那会让"云端下发时本地喊话是否该被打断"
这件事有两套答案.

*** 禁止项必须[显式拒绝]并说明, 不能静默丢.
v2.0 S2.6 逐字列了本期不得发送的能力, 并要求回 1006 + E_CHANNEL_DENIED 或
E_TASK_UNSUPPORTED. 静默丢在 Qt 那边是"点了没反应".

Boundaries: 只做 task_type -> (key, 机内 payload) 的映射. 不校验业务字段
(waypoints 的经纬度合不合法之类) -- 那由各机内模块的既有解析器做, 而那些
解析器正是语音链路也在用的, 复用它们才能保证两条输入走同一套判定.
"""

from __future__ import annotations

from typing import Any, Dict, List, Tuple

from ...common import errors
from ..outbound.error_map import (V2_DETAIL_TASK_UNSUPPORTED,
                                  build_error_fields)
from .cloud_inbound import InboundReject
from .field_validate import check_ids, validate_alarm, validate_goto

#: 云端来的一律 origin/source = cloud. NO 不从报文里取 --
#: origin 是授权边界(11 CH-1 通道即权限), 让发起方自称等于没有边界.
CLOUD_ORIGIN = "cloud"

#: 机内 key.
KEY_TASK = "cmd/task"
KEY_ESTOP = "cmd/estop"
KEY_GEO = "cmd/geo"
KEY_AUDIO = "cmd/audio/speak"

#: v2.0 S3 的六个 task_type. 前五个本期开放, MANUAL_VELOCITY 本期禁止.
OPEN_TASK_TYPES = ("GOTO_KEYPOINT", "STOP_TASK", "ESTOP",
                   "SET_ALARM_CONFIG", "AUDIO_CONTROL")

#: v2.0 S3 逐字列出的[旧名称], 收到必须以"不支持的任务类型"拒绝,
#: NO 不能静默映射到某个现有能力.
RETIRED_TASK_TYPES = (
    "INSPECTION_ROUTE", "FOLLOW_RECORDED_PATH", "PAUSE_TASK", "RESUME_TASK",
    "RETURN_HOME", "SET_GEOFENCE", "SET_KEYPOINTS", "SET_RECORDED_PATHS",
    "START_RECORDING", "STOP_RECORDING",
)

#: 本期明令禁止的能力(v2.0 S2.6 / S3.6). 与"旧名称"分开报:
#: 前者是"这个能力本期不对云端开放", 后者是"这个名字已经不存在".
FORBIDDEN_TASK_TYPES = ("MANUAL_VELOCITY",)

#: v2.0 S2.2: STOP_TASK 的三个动作, 与机内 TASK_ACTION 同名.
_STOP_ACTIONS = ("pause", "resume", "cancel")


def route(data: Dict[str, Any]) -> List[Tuple[str, Dict[str, Any]]]:
    """云端 data -> [(机内 key, 机内 payload), ...]. 拒绝时抛 InboundReject.

    data 是已过信封校验的 v2.0 data 对象(msg_id/task_id/task_type/payload).

    *** 返回[列表], 因为一条云端命令未必对应一条机内命令.
    GOTO/STOP/ESTOP/AUDIO 都是一对一(列表里一条), 但 SET_ALARM_CONFIG 是一条
    云端命令 fan-out 成 N 条机内命令(每个 alarm_region 一条 cmd/geo fence
    upsert; 批B 起再加 cmd/config 的规则/声光). 网关收齐这 N 条的机内 ack 后
    聚合成一条 v2.0 终态(见 cloud_wiring 的 _fanout). 早先本函数返回单条
    tuple, 而报警配置本质是[跨多落点的配置事务], 单条装不下.
    """
    task_type = data.get("task_type")
    payload = data.get("payload")

    if task_type in FORBIDDEN_TASK_TYPES:
        # v2.0 S3.6 逐字: 后端收到历史 MANUAL_VELOCITY 或云端 cmd/teleop
        # 应拒绝并回 E_CHANNEL_DENIED, 不能执行速度.
        raise InboundReject(build_error_fields(
            errors.E_CHANNEL_DENIED,
            "cloud continuous teleop is not enabled this phase",
            {"task_type": task_type}))
    if task_type in RETIRED_TASK_TYPES:
        # v2.0 S2.6: 未支持[任务类型] -> detail.code "E_TASK_UNSUPPORTED"
        # (审计 C-1). 我方原生码是 E_NOT_IMPLEMENTED(-> 1006), 但发给 Qt 的
        # detail.code 用 v2.0 逐字点名的值.
        raise InboundReject(build_error_fields(
            errors.E_NOT_IMPLEMENTED,
            "task type is no longer part of this phase protocol: %s" % task_type,
            {"task_type": task_type, "allowed": list(OPEN_TASK_TYPES)},
            detail_code=V2_DETAIL_TASK_UNSUPPORTED))
    if task_type not in OPEN_TASK_TYPES:
        raise InboundReject(build_error_fields(
            errors.E_NOT_IMPLEMENTED,
            "unsupported task type: %r" % (task_type,),
            {"task_type": task_type, "allowed": list(OPEN_TASK_TYPES)},
            detail_code=V2_DETAIL_TASK_UNSUPPORTED))
    if not isinstance(payload, dict):
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "payload is not an object",
            {"field": "payload",
             "got_type": type(payload).__name__}))

    # C-2(审计): msg_id/task_id 必填 + v2.0 S1.2 ID 正则. check_ids 抛 1002
    # (缺失)或 1003(格式). 早于拆分 -- 一个格式非法的 id 不该被带进机内.
    check_ids(data)
    cmd_id = data["msg_id"]
    task_id = data["task_id"]

    if task_type == "GOTO_KEYPOINT":
        return [(KEY_TASK, _goto(cmd_id, task_id, payload))]
    if task_type == "STOP_TASK":
        return [(KEY_TASK, _stop(cmd_id, payload))]
    if task_type == "ESTOP":
        return [(KEY_ESTOP, _estop(cmd_id, payload))]
    if task_type == "SET_ALARM_CONFIG":
        # fan-out: 一条报警配置 -> N 条 cmd/geo fence(warning) upsert.
        # 批A 只投影 regions(几何); rules/声光是批B/C 的 cmd/config.
        return [(KEY_GEO, c) for c in _alarm(cmd_id, payload)]
    return [(KEY_AUDIO, _audio(cmd_id, payload))]


def _goto(cmd_id: str, task_id: str, payload: Dict) -> Dict[str, Any]:
    """GOTO_KEYPOINT -> 11 S7.2 的 {action:"submit", task:{...}}.

    *** 字段级校验(coordinate_system/arrival_radius_m/waypoint id)在
    validate_goto -- 那些是 v2.0 协议字段, 机内不认, 网关是唯一能挡住非法
    值的地方(审计 B-2). 但 waypoint 是否在围栏内 / 路径版本对不对这类[需要
    geo.db 知识]的判定仍留给 p3 的既有解析器 -- 复用它, 云端与语音两条输入
    才走同一套业务判定.
    """
    validate_goto(payload)
    return {
        "cmd_id": cmd_id,
        "action": "submit",
        "task": {
            "task_id": task_id,
            # *** "goto", NO 不是 "goto_keypoint".
            # 权威闭集是 15 S12 的 TASK_TYPES(patrol|goto|charge|return_home|
            # standby|teach|follow), 由 tasks 表的 DDL CHECK 强制. 云端的
            # GOTO_KEYPOINT 是 v2.0 的 wire 名, 落到机内必须换成闭集里的值.
            # 对照点: 语音链路的 B01 goto_waypoint 意图映射的也是 "goto"
            # (p4_agent/runtime/task_request.py 的 _TASK_CREATE_INTENTS).
            # 两条输入是同一个业务动作, 落不同 type 就等于机内有两套任务语义.
            #
            # 原写 "goto_keypoint" 的后果不是报错难懂, 而是[每一条云端导航
            # 指令都被拒]: task_row_from_command 查闭集不中即抛, 网关把它
            # 翻成 error_code 3001 / E_INTERNAL 回给 Qt. 2026-09-01 用
            # scripts/dev/cloud_probe.py 对真实栈发帧才暴露 -- 单测只把
            # 本函数的输出与自己比对, 从未与 15 S12 的闭集对撞.
            "type": "goto",
            # 云端的计划级路径 ID; p3 解析时换成实际 route_id/route_rev
            # 并在回报里带上(v2.0 S3.1 要求).
            "recorded_path_id": payload.get("recorded_path_id"),
            "coordinate_system": payload.get("coordinate_system"),
            "waypoints": payload.get("waypoints"),
        },
        "source": CLOUD_ORIGIN,
        "reason": "",
    }


def _stop(cmd_id: str, payload: Dict) -> Dict[str, Any]:
    """STOP_TASK -> 11 S7.2 的 control 动作.

    *** target_task_id 必填, 不接受"省略即当前任务".
    v2.0 S3.2 逐字禁止. 11 S7.2 给的四条理由里第一条是要害: 队列是活的 --
    操作员看到"A 在跑"到命令抵达之间 A 可能已结束而 B 开始, 简写会暂停
    错的那条, 且日志里看不出发生过这件事.
    """
    action = payload.get("action")
    if action not in _STOP_ACTIONS:
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA,
            "STOP_TASK action must be pause|resume|cancel",
            {"field": "action", "got": action}))
    target = payload.get("target_task_id")
    if not target:
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA,
            "target_task_id is required; omission does not imply the current task",
            {"field": "target_task_id"}))
    return {
        "cmd_id": cmd_id,
        "action": action,
        "task_id": target,
        "source": CLOUD_ORIGIN,
        # v2.0 S2.2: reason 是自由文本, 原样进事件与审计.
        # 客户答复 4.4 逐字确认我方不做枚举校验, 只做自由文本落库.
        "reason": payload.get("reason") or "",
    }


def _estop(cmd_id: str, payload: Dict) -> Dict[str, Any]:
    """ESTOP -> cmd/estop.

    *** 走[独立 key], 不进普通任务队列.
    v2.0 S3.3 逐字: "后端必须单独订阅 cmd/estop, 不得经普通任务 FIFO,
    限流或降级处理". 那条 key 在我方是 Q0 最高优先级.
    """
    action = payload.get("action")
    if action != "stop":
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "ESTOP action must be stop",
            {"field": "action", "got": action}))
    return {
        "cmd_id": cmd_id,
        "type": "estop",
        "action": "stop",
        "source": CLOUD_ORIGIN,
        "reason": payload.get("reason") or "",
    }


def _alarm(cmd_id: str, payload: Dict) -> List[Dict[str, Any]]:
    """SET_ALARM_CONFIG -> N 条 cmd/geo fence(warning) upsert (fan-out).

    *** 报警配置不是一处 alarm_config, 是跨多落点的配置事务(方案 v0.1).
    alarm_region 在我方就是 fence 的 warning role(11 S9A.2, 旧名 zone); 规则
    (person/vehicle_in_region)与声光(alarm_level/siren_level)走 cmd/config 的
    suspicion_rules(批B/C, 11 R4.5/R6.1). 早先本函数把全部塞进一条畸形
    cmd/geo({alarm_config, regions}), p3 的 geo 解析器认不出(缺 type/geo_id/
    obj)直接抛 E_SCHEMA -- 报警配置到 p3 就断了. 本函数改为字段级拆解: 每个
    region 投影成一条 p3 认识的 fence upsert.

    *** 批A 只投影 regions(几何). rules/声光在批B/C 追加为 cmd/config 命令.
    => 本批过后, 只带 regions 的报警配置能落库; 只改 rules/声光(regions 为空)
    的暂时 fan-out 出 0 条 -- 那条路径由批B 补(批B 起至少有一条 cmd/config).

    *** keep_in 一律拒, 且理由要逐字可读.
    v2.0 S3.4 逐字: "regions[] 只允许 alarm_region, 禁止 keep_in. 营区 keep-in
    必须走独立安全围栏接口". keep_in 是[安全围栏], 用报警配置通道改它意味着
    一条改报警的命令能改掉机器人的活动边界 -- 评审 R10.5 措辞回 3001.
    """
    regions = payload.get("regions") or []
    if not isinstance(regions, list):
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "regions is not an array", {"field": "regions"}))
    for idx, region in enumerate(regions):
        if not isinstance(region, dict):
            raise InboundReject(build_error_fields(
                errors.E_SCHEMA, "regions[%d] is not an object" % idx,
                {"field": "regions[%d]" % idx}))
        if region.get("type") == "keep_in":
            raise InboundReject(build_error_fields(
                errors.E_CONFIG_INVALID,
                "camp keep-in boundary is not configured through this channel",
                {"field": "regions[%d].type" % idx,
                 "region_id": region.get("id")}))
    # B-3(审计): 标量范围 + rules + regions 结构校验. keep_in 已在上面拒过
    # (它是安全边界, 拒绝理由要逐字), 这里补其余 v2.0 S2.4 字段.
    validate_alarm(payload)
    # 每个 region 展开成 1~2 条 fence 命令(见 _region_to_fence: enabled=true 的
    # upsert 要 upsert + 激活两条). cmd_id 是占位 -- 网关 fan-out 时给每条子命令
    # 打唯一子 cmd_id(见 cloud_wiring._handle_task). flatten 成一维.
    return [c for region in regions for c in _region_to_fence(cmd_id, region)]


def _region_to_fence(cmd_id: str, region: Dict) -> List[Dict[str, Any]]:
    """v2.0 alarm_region -> 11 S7.8 的 fence(role=warning) 命令[列表](1~2 条).

    alarm_region 与 fence warning 是同一个东西(11 S9A.2, warning 旧名 zone;
    zone_enter/zone_exit 逐字"role=warning, 纯点在多边形内, FE-1"). 增量语义
    op=upsert|delete|set_state 映到 p3 geo 命令的 action; role 恒 warning.

    *** enabled=true 的 upsert 要[两条]: upsert + set_state->active(审计 #3).
    p3 的 upsert 恒把新 fence 建成 draft(apply_upsert 用 _INITIAL_STATE=draft),
    draft 不进 list_active -> 不广播 -> p1 不持有 -> 既不报 zone_enter(E), 也不
    让 state/fence.active.rev 前进(D 恒 timeout). 所以要跟一条 set_state->active
    把它激活. 云端激活[无需 L2](11 S7.9.5 cloud 列是  无 L2; L2 只在 hmi/
    voice 列 -- 一度误读为 cloud 也要 L2, 复核订正). enabled=false 只 upsert(留
    draft, 即"存了不启用").

    *** 激活用 force=true: upsert 后的 rev 由 p3 定, 网关预测不了(identical
    content 时 p3 还不 bump rev), base_rev 对不上会 conflict. 本网关刚建这条
    fence, 无并发写者, force 只跳并发检查 -- NO 不跳 <=5-active/1-allow 配额触发
    (那是 DB 触发, 激活超额仍会被挡, 安全的).

    * vertices 是 v2.0 [{latitude, longitude}], p3 geom.outer 收 [[lat,lon]]
    (geo_object._latlon 两种都认). applies_to(区域对哪类目标生效)不落 fence 几何
    -- 它是[规则]维度, 随 rules 进 suspicion_rules(批B, 卡 L2 挂起).
    """
    op = region.get("op")
    geo_id = region.get("id")
    base_rev = region.get("base_rev")
    if op == "delete":
        return [{"cmd_id": cmd_id, "action": "delete", "type": "fence",
                 "geo_id": geo_id, "origin": CLOUD_ORIGIN, "base_rev": base_rev}]
    if op == "set_state":
        # v2.0 enabled(bool) -> p3 fence state(active|disabled). 停用 = 规则不再
        # 命中它, 但几何还在(不是删除). 目标态走 obj.state(审计 #2: 信封无 state
        # 成员, apply_set_state 从 (cmd.obj or {}).get("state") 读; 放顶层被丢).
        # ->active 走 force(理由同下), ->disabled 是安全方向留 base_rev 走正常并发.
        if region.get("enabled"):
            return [_activate_fence(cmd_id, geo_id)]
        return [{"cmd_id": cmd_id, "action": "set_state", "type": "fence",
                 "geo_id": geo_id, "origin": CLOUD_ORIGIN, "base_rev": base_rev,
                 "obj": {"state": "disabled"}}]
    # op == "upsert"(field_validate 已把 op 限在 upsert|delete|set_state).
    verts = [[v["latitude"], v["longitude"]]
             for v in (region.get("vertices") or [])]
    cmds = [{"cmd_id": cmd_id, "action": "upsert", "type": "fence",
             "geo_id": geo_id, "origin": CLOUD_ORIGIN, "base_rev": base_rev,
             "obj": {"name": region.get("name"),
                     "geom": {"role": "warning", "outer": verts}}}]
    if region.get("enabled"):
        cmds.append(_activate_fence(cmd_id, geo_id))
    return cmds


def _activate_fence(cmd_id: str, geo_id: str) -> Dict[str, Any]:
    """set_state fence->active(force). 云端激活无需 L2(11 S7.9.5); force 因紧跟
    upsert 时 rev 预测不了, 见 _region_to_fence 头注."""
    return {"cmd_id": cmd_id, "action": "set_state", "type": "fence",
            "geo_id": geo_id, "origin": CLOUD_ORIGIN, "force": True,
            "obj": {"state": "active"}}


def _audio(cmd_id: str, payload: Dict) -> Dict[str, Any]:
    """AUDIO_CONTROL -> cmd/audio/speak.

    *** dog_to_pc 本期禁止, 收到必须拒绝.
    v2.0 S3.5 逐字: "后端收到必须拒绝, 不得静默启用机上麦克风上行".
    静默启用的后果是隐私问题, 而它不会产生任何可见现象.

    *** start 不得带 stream_id, exit_broadcast 必须带.
    v2.0 S2.5: stream_id 由后端在 start 的 ack 里分配; 退出时必须回显同一个,
    后端不得为退出请求分配新 ID. 一个给退出也发新 ID 的实现, 会让 Qt 无法
    确认自己退的是不是刚才那一路.
    """
    mode = payload.get("mode")
    if mode != "pc_to_dog":
        raise InboundReject(build_error_fields(
            errors.E_CHANNEL_DENIED,
            "only pc_to_dog is enabled this phase; dog_to_pc must not enable onboard mic uplink",
            {"field": "mode", "got": mode}))
    action = payload.get("action")
    if action not in ("start", "exit_broadcast"):
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "AUDIO_CONTROL action must be start|exit_broadcast",
            {"field": "action", "got": action}))
    stream_id = payload.get("stream_id")
    if action == "start" and stream_id:
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "start must not carry stream_id; the backend assigns it",
            {"field": "stream_id"}))
    if action == "exit_broadcast" and not stream_id:
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "exit_broadcast must carry the stream_id to exit",
            {"field": "stream_id"}))
    return {
        "cmd_id": cmd_id,
        "action": action,
        "mode": mode,
        "stream_id": stream_id,
        "source": CLOUD_ORIGIN,
    }
