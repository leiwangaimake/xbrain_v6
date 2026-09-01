"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: state_projection.py
Brief: 机内状态 -> v2.0 云端 state/* 的纯投影函数 (B-2)

Description:
网关为了服务 HMI 已经把机内总线上的东西缓存了一份(state/task, state/mode,
state/pose, state/clock, health/summary, geo objects). 云端面不另起一套采集,
只是同一份缓存的第二个消费者 -- 两个消费者读同一份数据, HMI 上看到的和 Qt
上看到的就不会打架. 本文件是那层换算, 全部是纯函数, 不碰 Zenoh 也不读钟.

*** 无源的字段一律 null, NO 绝不编一个像样的数.
这是 CLAUDE.md 3.1 投到线上的形式. 具体到本项目: state/power 今天没有任何
发布者(要 chassis_relay 加真底盘), 于是 battery 无源. 三种做法里:
  soc: 0    -> Qt 显示电量耗尽, 操作员会中止出勤
  soc: 100  -> Qt 显示满电, 操作员会派它出长任务, 半路没电
  soc: null -> Qt 显示"未接入", 联调当天一眼看出是哪一侧没做完
前两种都是 fail-silent, 而且是[看起来正常]的那种. 第三种难看但真实.

* 这条与 HMI 侧的做法是同一条: hmi/data_readers.py 的 pose_group 在无源时
返回一个每个值都是 None 的壳加 available: false, 注释写着"NOT a zeroed
pose". 云端面沿用它, 两个面才会同时诚实.

*** 已知无源清单(写在代码里而不是文档里, 因为它会变):
  battery   state/power 无发布者
  storage   无采集点
  motion    有 speed_mps, 无 gait / angular
  devices   health/summary 的 kind==device 项, 不含 cap
接线补上以后, 对应的 UNSOURCED_* 常量要跟着删, 而 tests 里有一条断言盯着
这份清单与真实产出一致 -- 免得源接上了而投影还在发 null.

Boundaries: 只做形状换算与闭集校验. 不发布(那是 runtime/cloud_wiring.py),
不读钟(ts 与 seq 由信封层填).
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

from .cloud_envelope import normalise_progress

# --- v2.0 闭集 --------------------------------------------------------
#
# 闭集外的值必须抛, NO 不静默透传也不"未知值降级解释"(CLAUDE.md 3.5).
# v2.0 S1.3 从对面重复了同一条: "不得把未知枚举降级解释为某个已知值".

ROBOT_STATES = ("offline", "idle", "running", "charging", "fault",
                "emergency_stop")
TASK_STATES = ("idle", "queued", "running", "paused", "completed", "failed",
               "cancelled")

#: 机内 12 值(15 S3.2 / 11 S4.4) -> v2.0 S3.2 的 7 值.
#:
#: *** 这张表原先不存在, 而两侧只有 3 个名字碰巧相同.
#: 投影原来拿机内值直接对 TASK_STATES 求值, 不中就抛, 于是 pending/ready/done
#: 这些[每条任务必经]的状态一律被拒: 任务在 ack 之后就从 Qt 的 state/task 里
#: 消失(current 恒 null), 而 15 S3.2 的 12 值里有 9 个不在 v2.0 那 7 值内.
#: 2026-09-01 联调预演实测: 一条 GOTO 落库后, 该 ERROR 以 10 Hz 刷了三万条.
#:
#: *** 分档依据是 15 S3.2 的状态机图, 不是名字相近.
#: - pending/scheduled/blocked/ready 都在图上 running 之前, 且都能[不经人工]
#:   自动往前走(定时到 / 依赖解除 / 校验通过 / 被调度选中) => queued.
#:   blocked 归 queued 而不是 paused: 图上它"解除"后自动回 ready, 与
#:   suspended 需要 resume 是两回事; 报成 paused 会让操作员去按恢复.
#: - suspended 是 15 S3.2 的"活跃"集成员且可 resume(passive 人工/急停/低电,
#:   yielding 抢占) => paused, 正好落 v2.0 快照的 suspended 桶.
#: - done => completed(只是换名).
#:
#: *** 三个"终态但非干净成败"的值统一归 failed, 这是一个判断, 记在这里.
#: needs_review(15 S3.2: 挂起超时或恢复校验失败, 终态需人工处置) /
#: interrupted(本期无产生者, 但历史库有行) / wait_for_power_off(关机流程里
#: [已进终态但尚未上行完毕]的任务)三者都是终态, 都不是干净的成功, 也都不是
#: 操作员主动取消.
#: - NO 不映 paused: paused 在 v2.0 语义里可恢复, 而这三个是终态, 报 paused
#:   会让 Qt 显示一个永远恢复不了的暂停任务.
#: - wait_for_power_off 的原终态(done/failed/cancelled)从这个值本身读不出来,
#:   所以只能选一个方向. 选 failed 而不是 completed: 报一个没核实过的成功,
#:   操作员不会再去看; 报 failed 他会去看. 失效方向取"会被发现"的那个.
#: - *** 这三个终态今天都进不了 v2.0 快照的任何一个桶(分桶只取
#:   running/queued/paused), 所以本行今天只决定[校验过不过]. 权威终态另由
#:   message_type=result 发出(v2.0 S4.1: 必须由任务权威模块产生, 网关不猜).
#:   将来若分桶扩到终态, 这里的方向已经是安全的那个.
INTERNAL_TO_V2_TASK_STATE = {
    "pending":            "queued",
    "scheduled":          "queued",
    "blocked":            "queued",
    "ready":              "queued",
    "running":            "running",
    "suspended":          "paused",
    "done":               "completed",
    "failed":             "failed",
    "cancelled":          "cancelled",
    "needs_review":       "failed",
    "interrupted":        "failed",
    "wait_for_power_off": "failed",
}


def to_v2_task_state(value: Any) -> str:
    """机内 task.state -> v2.0 S3.2 的值. 表外的值抛.

    *** 抛而不是兜底, 保留原实现选对了的那一半.
    原实现的取舍是对的(越界必抛, CLAUDE.md 3.5) -- 它只是没写映射表.
    一个"表外就返回 failed"的兜底会让新增的机内状态静默变成失败, 而 11 S4.4
    的枚举一旦扩容, 我们要的是在这里响, 不是在 Qt 上看到一批莫名其妙的失败.
    完备性由 tests/p5_gateway/test_state_projection.py 的双向差集判据守.
    """
    mapped = INTERNAL_TO_V2_TASK_STATE.get(value)
    if mapped is None:
        raise ValueError(
            "task.state=%r not in the 15 S3.2 internal closed set %s"
            % (value, sorted(INTERNAL_TO_V2_TASK_STATE)))
    return mapped
#: 机内 HealthSummary items[].state (11 S5.1 五值) -> v2.0 S4.2 devices[].status.
#:
#: *** 这张表原先不存在, 而两侧一个名字都不重合.
#: _devices_from_health 原来找 item["status"] -- 那个键在 11 S5.1 里根本没有
#: (真名是 state), 于是每一项都落进 "unknown" 兜底. 加上订阅的 key 也是错的
#: (health/factor, 见 main_wiring), devices 实际是恒空数组.
#:
#: *** warn -> degraded 是一个判断, 记在这里.
#: v2.0 没有 warn 档. 映 degraded 而不是 online: warn 在 11 S5.1A 是"仅记录"
#: 级, 报 online 会让操作员完全看不见它; 报 degraded 是显得比实际差, 而这个
#: 方向会被发现, 反过来不会. 与 17 S10.2 那条"绑窄立即发现 / 绑宽永不发现"
#: 的不对称取舍同一理由.
#:
#: *** v2.0 的 offline 永远不会被产生 -- 机内 five 值里没有对应态.
#: 一个设备没有生产者时 11 S5.1 报 unknown(不是 offline), 设备报错时报 fail.
#: 映射非满射是事实不是缺口; 写在这里免得下一个人以为漏了一行.
HEALTH_STATE_TO_V2_STATUS = {
    "ok":       "online",
    "warn":     "degraded",
    "degraded": "degraded",
    "fail":     "fault",
    "unknown":  "unknown",
}


def to_v2_device_status(value: Any) -> str:
    """HealthSummary items[].state -> v2.0 devices[].status. 表外的值抛.

    与 to_v2_task_state 同一取舍(CLAUDE.md 3.5 越界必抛). 兜底成 unknown 会
    让 11 S5.1 将来扩容的新 state 静默变成"未知设备", 而我们要的是在这里响.
    """
    mapped = HEALTH_STATE_TO_V2_STATUS.get(value)
    if mapped is None:
        # ProjectionError 而不是裸 ValueError: cloud_state 的 tick 只对它走
        # "refused"分支(记 ERROR 并跳过这条 key), 裸 ValueError 会落进
        # "crashed"分支, 日志措辞会指向一个不存在的实现 bug.
        raise ProjectionError(
            "health item state=%r not in the 11 S5.1 closed set %s"
            % (value, sorted(HEALTH_STATE_TO_V2_STATUS)))
    return mapped


GPS_FIXES = ("none", "gps", "dgps", "rtk_float", "rtk_fixed")
DEVICE_STATUSES = ("online", "degraded", "offline", "fault", "unknown")
VOICE_MODES = ("normal", "broadcast", "alarm")
MODE_SOURCES = ("cloud", "local", "autonomy", "system")
SPEAKER_STATES = ("idle", "buffering", "playing", "fault")
MIC_STATES = ("disabled", "idle", "recording", "fault")
GEO_TYPES = ("waypoint", "recorded_path", "alarm_region")

#: v2.0 S4.3 exit_reason 闭集(8 值). null 另外允许(表示"没有退出").
EXIT_REASONS = ("requested", "timeout", "preempted", "fault", "target_lost",
                "target_left_fence", "manual_cloud", "manual_wecom")

#: v2.0 S5.1 事件 sev / state 闭集.
EVENT_SEV = ("info", "warn", "error", "fatal")
EVENT_STATES = ("active", "cleared", "acknowledged", "occurred")

#: 今天没有任何机内发布者的段. 见模块头.
UNSOURCED_ROBOT_SECTIONS = ("battery", "storage")


class ProjectionError(ValueError):
    """投影出了一个闭集外的值. 发出去就是让 Qt 收到它字典里没有的枚举."""


def _closed(value: Any, closed: Sequence[str], field: str) -> str:
    if value not in closed:
        raise ProjectionError(
            "%s=%r not in the v2.0 closed set %s" % (field, value, closed))
    return value


def _closed_or_none(value: Any, closed: Sequence[str], field: str):
    """闭集校验, 但 None 放行(nullable 字段). exit_reason 这类"没有值"是
    合法的, 只有[给了一个闭集外的非 null 值]才是错的. """
    if value is None:
        return None
    return _closed(value, closed, field)


def event_payload(internal: Dict[str, Any], *, sev: str,
                  category: str) -> Dict[str, Any]:
    """机内事件 -> v2.0 S5.1 event data(审计 D-2).

    *** 为什么要重整, 不能直接透传机内事件.
    机内事件的字段名与 v2.0 不一样(severity/event_id vs sev/eid), 而且
    [sev/category 根本不在机内 data 里 -- 它们在 key 上]. 直接透传的话
    Qt 按 S5 找 data.sev 会找不到, 且 S5 逐字"不发布 event_id/severity 别名".
    本函数把机内事件规整成 S5.1 的 14 字段, 字段名归一, 缺的补 null/空数组.

    *** sev/category 从 key 取(权威), 并校验闭集.
    S5 逐字"event key 的 severity/category 与 data.sev/category 逐字一致".
    key 是网关按 (sev, cat) 建的, 所以 key 就是权威源 -- data.sev/category
    直接用它, 二者天然一致, NO 不从机内 data 里另取(那可能与 key 不符).
    """
    _closed(sev, EVENT_SEV, "sev")
    state = internal.get("state") or "occurred"     # S5.4 任务事件默认 occurred
    _closed(state, EVENT_STATES, "state")
    eid = internal.get("eid") or internal.get("event_id")
    if not eid:
        # eid 是可靠事件的幂等 ID(S5.1 必填), 缺了 Qt 没法去重/补发关联.
        raise ProjectionError("event missing eid (v2.0 S5.1)")
    return {
        "eid": eid,
        "sev": sev,
        "category": category,
        "state": state,
        # source 归一: 机内可能用 src. 缺则网关.
        "source": internal.get("source") or internal.get("src") or "p5_gateway",
        "code": internal.get("code") or "",
        "title": internal.get("title") or "",
        "message": internal.get("message") or "",
        # 以下 nullable, 不适用为 null(S5.1).
        "task_id": internal.get("task_id"),
        "operator": internal.get("operator"),
        "result": internal.get("result"),
        "detail": dict(internal.get("detail") or {}),
        # 无引用为空数组(S5.1), NO 不是 null.
        "media": list(internal.get("media") or ()),
        "file_refs": list(internal.get("file_refs") or ()),
    }


# --- state/robot ------------------------------------------------------

def robot_payload(*, robot_state: str, task_state: str,
                  pose: Optional[Dict[str, Any]],
                  clock: Optional[Dict[str, Any]],
                  devices: Optional[Sequence[Dict[str, Any]]],
                  alarm_window_active: bool,
                  motion_speed_mps: Optional[float] = None
                  ) -> Dict[str, Any]:
    """v2.0 S4.2 的八段.

    *** battery 与 storage 恒为 null, 且这是有意的.
    它们今天没有任何机内发布者. 见模块头对三种做法的比较 -- 编一个数会让
    操作员据此做出错误决定, 而 null 只是难看.

    * alarm_window_active 必须[反映实际生效状态], 不是配置里写没写.
    v2.0 S3.5 逐字: 授时未同步时带时间窗的规则不命中. 所以调用方在
    clock.ts_sync 为 false 时必须传 False 进来 -- 本函数不替它判断, 因为
    "哪些规则带时间窗"是 P2 的知识, 不是投影层的.
    """
    return {
        "robot_state": _closed(robot_state, ROBOT_STATES, "robot_state"),
        "task_state": _closed(task_state, TASK_STATES, "task_state"),
        "gps": _gps(pose),
        "battery": None,        # UNSOURCED: state/power 无发布者
        "motion": _motion(pose, motion_speed_mps),
        "devices": _devices(devices),
        "storage": None,        # UNSOURCED: 无采集点
        "clock": _clock(clock),
        "alarm_window_active": bool(alarm_window_active),
    }


def _gps(pose: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """无 pose -> null 整段.

    NO 不返回一个 fix="none" 且经纬度为 0 的壳: (0, 0) 是几内亚湾上的一个
    真实坐标, Qt 会把机器人画在那里. 这个具体的坑本项目在 HMI 时区推导上
    踩过(见 hmi/geo_timezone.py: (0,0) 映到 Etc/GMT 必须挡掉).
    """
    if not pose:
        return None
    fix = pose.get("fix_type")
    return {
        "fix": _closed(fix, GPS_FIXES, "gps.fix") if fix else "none",
        "latitude": pose.get("lat"),
        "longitude": pose.get("lon"),
        "altitude_m": pose.get("alt"),
        "heading_deg": _heading_deg(pose),
        "speed_mps": pose.get("speed_mps"),
        "accuracy_m": pose.get("cov_h_m"),
    }


def _heading_deg(pose: Dict[str, Any]) -> Optional[float]:
    """机内是弧度(heading_rad), v2.0 要度.

    *** heading_valid 为假时返回 null, NO 不返回换算后的数.
    一个无效航向换算成度仍然是个像模像样的角度, Qt 会照着画箭头 -- 而机器
    人实际朝哪没人知道. 这是"能力不足时不假装有保证"(CLAUDE.md 3.2)在一个
    具体字段上的样子.
    """
    import math

    if not pose.get("heading_valid"):
        return None
    rad = pose.get("heading_rad")
    if rad is None:
        return None
    return math.degrees(float(rad)) % 360.0


def _motion(pose: Optional[Dict[str, Any]],
            speed_mps: Optional[float]) -> Dict[str, Any]:
    """gait 与 angular 无源 -> null; linear 走 pose 的 speed_mps."""
    linear = speed_mps
    if linear is None and pose:
        linear = pose.get("speed_mps")
    return {
        "gait": None,                  # UNSOURCED: 底盘步态未上报
        "linear_speed_mps": linear,
        "angular_speed_radps": None,   # UNSOURCED
    }


def _devices(devices: Optional[Sequence[Dict[str, Any]]]
             ) -> List[Dict[str, Any]]:
    """v2.0 S4.2 逐字: 后端只发布[实际发现的]设备.

    * 于是无源时返回空数组而不是 null -- 空数组说"一个都没发现", 语义准确;
    null 说"这段我没实现", 而 devices 这段是实现了的, 只是今天一个都没有.
    两者对 Qt 的意思不同, 别混.
    """
    out: List[Dict[str, Any]] = []
    for dev in devices or ():
        out.append({
            "id": dev["id"],
            "name": dev.get("name", dev["id"]),
            "status": _closed(dev.get("status", "unknown"), DEVICE_STATUSES,
                              "devices[].status"),
            "last_update_ms": dev.get("last_update_ms"),
        })
    return out


def _clock(clock: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """ts_sync 缺省一律 false(11 F-2 逐字).

    *** 只有字面 true 才算同步.
    rtk_driver 是唯一有权判定 ClockStatus.sync 的进程(CLK-A1); 消费者不得
    把一个字符串 "true" 或数字 1 升格成同步. 缺省 false 是 fail-safe 方向:
    未同步只是让带时间窗的规则不命中, 而误判已同步会让它们在错误的时刻命中.
    """
    if not clock:
        return {"ts_sync": False, "source": None, "age_ms": None}
    return {
        "ts_sync": clock.get("ts_sync") is True,
        "source": clock.get("source"),
        "age_ms": clock.get("age_ms"),
    }


# --- state/mode -------------------------------------------------------

def mode_payload(*, voice_mode: str, source: str,
                 stream_id: Optional[str] = None,
                 entered_ts: Optional[float] = None,
                 exit_reason: Optional[str] = None) -> Dict[str, Any]:
    """v2.0 S4.3 五字段, 全部必填(值可为 null).

    broadcast 时 stream_id 必填 -- 没有它 Qt 没法把 state/audio 的帧对上
    是哪一次喊话, 两条 key 就各说各话.
    """
    mode = _closed(voice_mode, VOICE_MODES, "voice_mode")
    if mode == "broadcast" and not stream_id:
        raise ProjectionError(
            "voice_mode=broadcast requires a stream_id (v2.0 S4.3)")
    return {
        "voice_mode": mode,
        "source": _closed(source, MODE_SOURCES, "source"),
        "stream_id": stream_id,
        "entered_ts": entered_ts,
        # exit_reason 闭集(v2.0 S4.3, 审计 D-1). null 允许(没有退出), 非 null
        # 必须在 8 值里 -- 我方发一个闭集外的值 Qt 会当未知枚举(S1.3 禁降级).
        "exit_reason": _closed_or_none(exit_reason, EXIT_REASONS, "exit_reason"),
    }


# --- state/audio ------------------------------------------------------

def audio_payload(*, speaker_state: str, microphone_state: str,
                  stream_id: Optional[str] = None,
                  speaker_holder: Optional[str] = None,
                  speaker_holder_type: Optional[str] = None,
                  last_frame_age_ms: Optional[int] = None,
                  exit_reason: Optional[str] = None) -> Dict[str, Any]:
    """v2.0 S4.4.

    *** playing / recording 由状态推导, NO 不单独传.
    与 ack 的 accepted 同一个道理(见 outbound/task_ack.py): 两个字段表达
    同一件事时它们迟早会不一致, 而一条 {speaker_state:"fault",
    playing:true} 会让 Qt 的按钮亮着而喇叭是哑的.
    """
    spk = _closed(speaker_state, SPEAKER_STATES, "speaker_state")
    mic = _closed(microphone_state, MIC_STATES, "microphone_state")
    return {
        "stream_id": stream_id,
        "playing": spk == "playing",
        "recording": mic == "recording",
        "speaker_state": spk,
        "microphone_state": mic,
        "speaker_holder": speaker_holder,
        "speaker_holder_type": speaker_holder_type,
        "last_frame_age_ms": last_frame_age_ms,
        # 同 state/mode 的 exit_reason 闭集(D-1).
        "exit_reason": _closed_or_none(exit_reason, EXIT_REASONS, "exit_reason"),
    }


# --- state/geo/manifest -----------------------------------------------

_GEO_PREFIX = {"waypoint": "w-", "recorded_path": "r-", "alarm_region": "f-"}


def geo_manifest_payload(*, manifest_rev: int,
                         objects: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """v2.0 S4.5. full 本期固定 true -- 消息永远是全量快照.

    *** geo_id 前缀按 type 强制(w- / r- / f-).
    v2.0 逐字规定了前缀, 而前缀是 Qt 用来分辨类型的第二条线索. 一条 type
    与前缀不一致的记录会让两条线索打架, Qt 按哪条走都可能是错的 -- 所以
    在这里就抛, 不发出去.
    """
    out = []
    for obj in objects:
        typ = _closed(obj.get("type"), GEO_TYPES, "objects[].type")
        geo_id = obj.get("geo_id") or ""
        want = _GEO_PREFIX[typ]
        if not geo_id.startswith(want):
            raise ProjectionError(
                "geo_id %r does not carry the %r prefix required for type %s"
                % (geo_id, want, typ))
        item = {"geo_id": geo_id, "type": typ,
                "name": obj.get("name", ""), "rev": int(obj.get("rev", 0))}
        if typ == "waypoint":
            # v2.0 逐字: waypoint 必须带 WGS84 坐标. 缺了就抛 -- 一个没有
            # 坐标的路点在地图上没法画, 而静默发出去只会让 Qt 那边空一格.
            for field in ("latitude", "longitude"):
                if obj.get(field) is None:
                    raise ProjectionError(
                        "waypoint %s is missing %s (v2.0 S4.5)"
                        % (geo_id, field))
            item["latitude"] = obj["latitude"]
            item["longitude"] = obj["longitude"]
            item["altitude"] = obj.get("altitude")
        elif typ == "alarm_region":
            if obj.get("enabled") is None:
                raise ProjectionError(
                    "alarm_region %s is missing enabled (v2.0 S4.5)" % geo_id)
            item["enabled"] = bool(obj["enabled"])
        out.append(item)
    return {"manifest_rev": int(manifest_rev), "full": True, "objects": out}


# --- state/task -------------------------------------------------------

def task_item(task: Dict[str, Any]) -> Dict[str, Any]:
    """v2.0 S3.2 的任务项. 十个字段全部必填(值可为 null).

    *** progress_percent 未知必须是 null, 禁止填 0(v2.0 S3.2 逐字).
    填 0 的话进度条停在最左边, 与"刚开始"完全一样 -- 操作员会以为任务卡住
    并去中止它. 这一条与 CLAUDE.md 3.1 那条"0.0 冒充已赋值"是同一个失效
    模式的两个面.
    """
    return {
        "task_id": task.get("task_id"),
        "task_type": task.get("task_type"),
        "state": _closed(task.get("state"), TASK_STATES, "task.state"),
        "current_waypoint_id": task.get("current_waypoint_id"),
        "completed_count": int(task.get("completed_count", 0)),
        "total_count": int(task.get("total_count", 0)),
        "progress_percent": normalise_progress(task.get("progress_percent")),
        "route_id": task.get("route_id"),
        "route_rev": task.get("route_rev"),
        "started_ts": task.get("started_ts"),
        "message": task.get("message", ""),
    }
