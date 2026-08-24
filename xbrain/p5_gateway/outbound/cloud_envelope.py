"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cloud_envelope.py
Brief: 云端 v2.0 六字段信封 + state/link 三值 + state/task 双形态

Description:
甲方 v2.0 S1.1 冻结了跨主机信封: v / rid / ts / seq / src / data 六字段,
其中 ts 是 float64 Unix 秒(UTC). 机内信封是另一套(S3.0 九字段, 带 mono),
两者[不能混]:

  * 机内消息带 mono 与 boot, 用于单调钟判定;
  * v2.0 S1.1 逐字"跨主机消息不得携带 mono, boot".

所以出网关这一步要重建信封而不是透传. 这也正是 11 S4.6 e 条逐字要求的
"重新封装而非透传".

*** seq 的语义比看上去细.
v2.0 S1.1: "按[发布进程 + rid + 完整 key]分别递增; 发布进程启动时从 1 开始,
同一进程内的短线重连不重置". 三点各有理由:
  * 按 key 分别递增 -> Qt 能对每条流独立判缺口, 而不是被别的流的丢包干扰;
  * 短线重连不重置 -> 重连不是新进程, 重置会让 Qt 误以为对端重启了;
  * 只有进程重启才回到 1 -> Qt 在新连接周期重建水位.
一个"全局一个计数器"的实现在前两点上都错, 而它在单流测试里看不出来.

*** state/link 的 level -> state 是[有损]映射, 这里只做无争议的三段.
11 S4.6 用 level 0..3(L0 正常 / L1 degraded / L2 down / L3 返航触发),
v2.0 只有三值 up|degraded|down. L0/L1/L2 一一对应, 而 L3 映到哪个值
[需要裁决] -- L3 时机器人仍在动(正在返航), 映成 down 会让 Qt 显示离线,
映成 degraded 又丢掉"已触发返航"这个信息.
=> 本模块对 L3 显式抛, NO 不擅自选一个. 见 NEXT 的裁决项.

Boundaries: 只做信封与形状转换. 不判断内容对错, 不发布 -- 发布在 runtime.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

#: v2.0 S1.1: 后端 -> Qt 固定 p5_gateway. 内部真实来源放业务字段 data.source.
SRC_GATEWAY = "p5_gateway"

#: v2.0 S1.1: Qt -> 后端固定 qt_hmi. 入站校验用它区分"外部报文"与"机内报文".
SRC_QT = "qt_hmi"

#: 信封版本. v2.0 固定 1.
ENVELOPE_V = 1

#: 11 S4.6 的 level -> v2.0 S4.1 的 state. L3 有意缺席, 见模块头注.
_LEVEL_TO_STATE = {0: "up", 1: "degraded", 2: "down"}


class SeqCounter:
    """按 (rid, key) 分别递增的序号发生器.

    *** 为什么不是一个全局计数器.
    v2.0 S1.1 逐字"按发布进程 + rid + 完整 key 分别递增". 全局计数器会让
    Qt 在 state/robot(10 Hz)与 state/link(1 Hz)之间看到巨大的号差 ->
    它按缺口诊断的逻辑会误报丢包, 而实际一条都没丢.

    进程内短线重连不重置: 重连不是新进程. 只有进程重启才从 1 开始 --
    那时 Qt 在新连接周期重建水位(v2.0 S9.4).
    """

    def __init__(self):
        self._next: Dict[tuple, int] = {}

    def next(self, rid: str, key: str) -> int:
        slot = (rid, key)
        # 从 1 开始: v2.0 逐字. 0 会被 Qt 当成"未初始化".
        value = self._next.get(slot, 1)
        self._next[slot] = value + 1
        return value


def build_envelope(rid: str, key: str, data: Dict[str, Any], *,
                   ts: float, seq: int, src: str = SRC_GATEWAY
                   ) -> Dict[str, Any]:
    """组装 v2.0 六字段信封.

    ts 由调用方给: 本模块不读钟. 理由是可测性 -- 一个自己读钟的构建器
    没法在测试里断言它填了什么(CLAUDE.md 3.4 另有要求: 时长判定用单调钟,
    而这里的 ts 是给人看的墙钟, 两者不冲突).
    """
    if not isinstance(data, dict):
        raise TypeError("data must be a dict; got %r" % type(data).__name__)
    return {
        "v": ENVELOPE_V,
        "rid": rid,
        "ts": float(ts),
        "seq": int(seq),
        "src": src,
        "data": data,
    }


class UnmappedLinkLevel(ValueError):
    """link level 没有 v2.0 落点(今天只有 L3).

    抛而不是挑一个: L3 是"返航已触发", 机器人仍在动. 映成 down 会让 Qt
    显示离线并可能触发操作员的应急流程; 映成 degraded 则丢掉返航这件事.
    两种都是错的, 而选哪个是[双方要在联调纪要里定]的事.
    """


def link_state_word(level: int) -> str:
    """11 的 level -> v2.0 的 state 三值.

    MUTATION 提示: 给 L3 加一个默认落点 -> tests 里那条 raises 变红.
    """
    if level in _LEVEL_TO_STATE:
        return _LEVEL_TO_STATE[level]
    raise UnmappedLinkLevel(
        "link level %r has no v2.0 state word. L3 (return-to-base triggered) "
        "is deliberately unmapped: the robot is still moving, so neither "
        "'down' nor 'degraded' is right. Needs a joint decision." % level)


def link_payload(snapshot, estop_path: str) -> Dict[str, Any]:
    """LinkSnapshot -> v2.0 S4.1 的四字段.

    v2.0 只要四个字段, 而我方 snapshot 有九个. 多发的字段 Qt 会保留但不解释
    (S1.3"接收方可以保留未知扩展字段"), 不过多发没有好处: 它让报文变大,
    且会让下一个读契约的人以为那些字段是约定的一部分.
    => 只发四个.
    """
    return {
        "state": link_state_word(snapshot.level),
        "cloud_link": bool(snapshot.cloud_link),
        "disconnected_s": float(snapshot.disconnected_s),
        "estop_path": estop_path,
    }


#: v2.0 S3.2/S3.3: 同一条 state/task 用 message_type 区分两种形状.
#: 这是 R12.4 的变通 -- 甲方把独立的 task/result key 合并进 state/task,
#: 我方 2026-08-08 答复里接受了. 理由(逐字): 终态仍由任务权威模块产生,
#: 不由网关猜测; 有 duration_sec/distance_m/ended_ts 权威值; 少一条订阅.
MSG_SNAPSHOT = "snapshot"
MSG_RESULT = "result"

MESSAGE_TYPES = (MSG_SNAPSHOT, MSG_RESULT)


def task_snapshot(msg_id: str, current: Optional[Dict],
                  queue: list, suspended: list) -> Dict[str, Any]:
    """state/task 的 snapshot 形态(v2.0 S3.2).

    current 为 None 表示当前没有任务 -- 那是一个确定的答案, 与"还没算出来"
    不同, 所以 NO 不用 {} 代替.
    """
    return {
        "msg_id": msg_id,
        "message_type": MSG_SNAPSHOT,
        "current": current,
        "queue": list(queue),
        "suspended": list(suspended),
    }


def task_result(msg_id: str, task_id: str, task_type: str, state: str,
                result_code: int, reason: str, summary: Dict,
                detail: Dict = None) -> Dict[str, Any]:
    """state/task 的 result 形态(v2.0 S3.3).

    *** state 三值闭集, 越界即抛.
    v2.0 S3.3 逐字 "done | failed | cancelled". 一个把 'completed' 也放行的
    实现会让 Qt 收到一个它不认识的终态 -- 而 S1.3 逐字禁止"把未知枚举降级
    解释为某个已知值", 所以 Qt 那边只会显示不出来, 不会报错.
    """
    if state not in ("done", "failed", "cancelled"):
        raise ValueError(
            "task result state %r not in v2.0 S3.3 closed set "
            "(done|failed|cancelled)" % state)
    return {
        "msg_id": msg_id,
        "message_type": MSG_RESULT,
        "task_id": task_id,
        "task_type": task_type,
        "state": state,
        "result_code": int(result_code),
        "reason": reason,
        "summary": dict(summary),
        "detail": dict(detail or {}),
    }


def normalise_progress(percent) -> Optional[float]:
    """进度未知一律 None, NO 不填 0.

    v2.0 S3.2 逐字: "progress_percent 允许为 null, 表示路径/总里程仍在计算,
    禁止填 0 冒充". 理由很实: 0 与"卡在起点"在界面上完全一样, 而操作员据此
    做的判断相反 -- 一个会等, 一个会去现场看.
    """
    if percent is None:
        return None
    value = float(percent)
    if not 0.0 <= value <= 100.0:
        raise ValueError("progress_percent %r outside 0..100" % percent)
    return value
