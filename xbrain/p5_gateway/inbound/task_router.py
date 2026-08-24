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

from typing import Any, Dict, Tuple

from ...common import errors
from ..outbound.error_map import build_error_fields
from .cloud_inbound import InboundReject

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
#: 🚫 不能静默映射到某个现有能力.
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


def route(data: Dict[str, Any]) -> Tuple[str, Dict[str, Any]]:
    """云端 data -> (机内 key, 机内 payload). 拒绝时抛 InboundReject.

    data 是已过信封校验的 v2.0 data 对象(msg_id/task_id/task_type/payload).
    """
    task_type = data.get("task_type")
    payload = data.get("payload")

    if task_type in FORBIDDEN_TASK_TYPES:
        # v2.0 S3.6 逐字: 后端收到历史 MANUAL_VELOCITY 或云端 cmd/teleop
        # 应拒绝并回 E_CHANNEL_DENIED, 不能执行速度.
        raise InboundReject(build_error_fields(
            errors.E_CHANNEL_DENIED,
            "云端连续遥控本期未立项, 不开放",
            {"task_type": task_type}))
    if task_type in RETIRED_TASK_TYPES:
        raise InboundReject(build_error_fields(
            errors.E_NOT_IMPLEMENTED,
            "该任务类型已不是本期协议能力: %s" % task_type,
            {"task_type": task_type, "allowed": list(OPEN_TASK_TYPES)}))
    if task_type not in OPEN_TASK_TYPES:
        raise InboundReject(build_error_fields(
            errors.E_NOT_IMPLEMENTED,
            "不支持的任务类型: %r" % (task_type,),
            {"task_type": task_type, "allowed": list(OPEN_TASK_TYPES)}))
    if not isinstance(payload, dict):
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "payload 不是对象",
            {"field": "payload",
             "got_type": type(payload).__name__}))

    cmd_id = data.get("msg_id")
    task_id = data.get("task_id")
    if not cmd_id or not task_id:
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "data 缺少 msg_id 或 task_id",
            {"field": "msg_id" if not cmd_id else "task_id"}))

    if task_type == "GOTO_KEYPOINT":
        return KEY_TASK, _goto(cmd_id, task_id, payload)
    if task_type == "STOP_TASK":
        return KEY_TASK, _stop(cmd_id, payload)
    if task_type == "ESTOP":
        return KEY_ESTOP, _estop(cmd_id, payload)
    if task_type == "SET_ALARM_CONFIG":
        return KEY_GEO, _alarm(cmd_id, payload)
    return KEY_AUDIO, _audio(cmd_id, payload)


def _goto(cmd_id: str, task_id: str, payload: Dict) -> Dict[str, Any]:
    """GOTO_KEYPOINT -> 11 S7.2 的 {action:"submit", task:{...}}.

    * 只做形状转换, 不校验 waypoints 内容 -- 那由 p3_task 的既有解析器做,
    而那个解析器正是语音下发的任务也在走的. 复用它, 两条输入才会得到
    完全一致的判定(包括围栏, 路径版本这些).
    """
    return {
        "cmd_id": cmd_id,
        "action": "submit",
        "task": {
            "task_id": task_id,
            "type": "goto_keypoint",
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
            "STOP_TASK 的 action 必须是 pause|resume|cancel",
            {"field": "action", "got": action}))
    target = payload.get("target_task_id")
    if not target:
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA,
            "target_task_id 必填, 不接受省略即当前任务",
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
            errors.E_SCHEMA, "ESTOP 的 action 固定为 stop",
            {"field": "action", "got": action}))
    return {
        "cmd_id": cmd_id,
        "type": "estop",
        "action": "stop",
        "source": CLOUD_ORIGIN,
        "reason": payload.get("reason") or "",
    }


def _alarm(cmd_id: str, payload: Dict) -> Dict[str, Any]:
    """SET_ALARM_CONFIG -> cmd/geo 的区域增量更新.

    *** keep_in 一律拒, 且理由要逐字可读.
    v2.0 S3.4 逐字: "regions[] 只允许 alarm_region, 禁止 keep_in.
    营区 keep-in 必须走独立安全围栏接口". 评审 R10.5 给了兜底措辞:
    回 3001 且 reason 逐字写"营区边界不经此通道配置".
    * 这条不是形式主义: keep_in 是[安全围栏], 用报警配置通道改它意味着
    一条改报警的命令能改掉机器人的活动边界.
    """
    regions = payload.get("regions") or []
    if not isinstance(regions, list):
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "regions 不是数组", {"field": "regions"}))
    for idx, region in enumerate(regions):
        if not isinstance(region, dict):
            raise InboundReject(build_error_fields(
                errors.E_SCHEMA, "regions[%d] 不是对象" % idx,
                {"field": "regions[%d]" % idx}))
        if region.get("type") == "keep_in":
            raise InboundReject(build_error_fields(
                errors.E_CONFIG_INVALID,
                "营区边界不经此通道配置",
                {"field": "regions[%d].type" % idx,
                 "region_id": region.get("id")}))
    return {
        "cmd_id": cmd_id,
        "action": "upsert",
        # origin 是授权边界(CH-1 通道即权限), 恒 cloud, 不从报文取.
        "origin": CLOUD_ORIGIN,
        "alarm_config": {
            "alarm_level": payload.get("alarm_level"),
            "siren_level": payload.get("siren_level"),
            "duration_sec": payload.get("duration_sec"),
            "cooldown_sec": payload.get("cooldown_sec"),
            "alarm_window": payload.get("alarm_window"),
            "rules": payload.get("rules"),
        },
        "regions": regions,
    }


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
            "本期只开放 pc_to_dog; dog_to_pc 不启用机上麦克风上行",
            {"field": "mode", "got": mode}))
    action = payload.get("action")
    if action not in ("start", "exit_broadcast"):
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "AUDIO_CONTROL 的 action 必须是 start|exit_broadcast",
            {"field": "action", "got": action}))
    stream_id = payload.get("stream_id")
    if action == "start" and stream_id:
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "start 不得携带 stream_id, 它由后端分配",
            {"field": "stream_id"}))
    if action == "exit_broadcast" and not stream_id:
        raise InboundReject(build_error_fields(
            errors.E_SCHEMA, "exit_broadcast 必须携带要退出的 stream_id",
            {"field": "stream_id"}))
    return {
        "cmd_id": cmd_id,
        "action": action,
        "mode": mode,
        "stream_id": stream_id,
        "source": CLOUD_ORIGIN,
    }
