"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_ack.py
Brief: cmd/task/ack 八字段 + rid+msg_id 幂等去重 (A-3)

Description:
v2.0 S3.1 规定每条 cmd/task 都要在 2 秒内回一条 ack, 八个字段一个不能少:
msg_id / ref_msg_id / task_id / task_type / result / accepted / error_code /
reason / detail. 结构拒绝, 业务拒绝和 duplicate 都算.

*** duplicate 不是"又执行了一次", 是"上次那条已经处理过".
v2.0 S4.1 逐字: "duplicate 表示原请求已经处理, 不能再次创建任务, 重置进度
或重复执行". 这一条决定了幂等要做在[执行之前]:
  收到 -> 查 (rid, msg_id) 见过没 -> 见过就直接回 duplicate, NO 不再往下走
一个"先执行再查重"的实现会让重发的 GOTO_KEYPOINT 创建第二条任务, 而 Qt
那边看到的 ack 是 duplicate -- 它以为什么都没发生.

*** 去重窗口不少于 60 秒, 且不得靠条数上限提前淘汰.
v2.0 S1.2 逐字点了这两件. 后者是要害: 一个"只保留最近 100 条"的实现在
高频下发时会把 30 秒前的 ID 挤掉, 于是一条 40 秒后的重发被当成新请求 --
而它满足"窗口不少于 60 秒"这句话的字面.

*** 时间一律用单调钟(CLAUDE.md 3.4 / CLK-C1).
墙钟在 NTP 阶跃时会往回跳, 那会让窗口忽然变长或变短; 变短的方向尤其糟 --
它让重复执行成为可能.

Boundaries: 只构造 ack 与判重. 不发布, 不判断业务能不能执行.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional, Tuple

#: v2.0 S3.1 的 result 三值闭集.
RESULT_ACCEPTED = "accepted"
RESULT_REJECTED = "rejected"
RESULT_DUPLICATE = "duplicate"

RESULTS = (RESULT_ACCEPTED, RESULT_REJECTED, RESULT_DUPLICATE)

#: v2.0 S1.2: 统一去重窗口不少于 60 秒.
DEDUP_WINDOW_S = 60.0


class AckShapeError(ValueError):
    """ack 形状不合 v2.0 S3.1."""


def build_ack(*, msg_id: str, ref_msg_id: str, task_id: str, task_type: str,
              result: str, error_code: int = 0, reason: str = "",
              detail: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """组装 v2.0 S3.1 的 ack data 对象.

    *** accepted 由 result 推导, NO 不让调用方单独传.
    两个字段表达同一件事时, 它们迟早会不一致 -- 而一条
    {result:"rejected", accepted:true} 的 ack 会让 Qt 显示成功而机器人
    什么都没做. v2.0 S3.1 逐字给了对应关系: accepted/duplicate 为 true,
    rejected 为 false.
    """
    if result not in RESULTS:
        raise AckShapeError(
            "ack result %r not in v2.0 S3.1 closed set %s" % (result, RESULTS))
    accepted = result in (RESULT_ACCEPTED, RESULT_DUPLICATE)
    if accepted and error_code != 0:
        # 受理却带非零码, 是两处判断打架的信号. Qt 那边会按 accepted 走
        # 成功分支, 而 error_code 里的原因永远没人看.
        raise AckShapeError(
            "result=%s implies error_code 0; got %d" % (result, error_code))
    if not accepted and error_code == 0:
        # 拒绝必须带非零码(v2.0 S10 逐字).
        raise AckShapeError("result=rejected requires a non-zero error_code")
    if not accepted and not reason:
        raise AckShapeError("result=rejected requires a human-readable reason")
    return {
        "msg_id": msg_id,
        "ref_msg_id": ref_msg_id,
        "task_id": task_id,
        # v2.0 S3.1 逐字: 原请求任务类型; 音频 ack 也不得省略.
        "task_type": task_type,
        "result": result,
        "accepted": accepted,
        "error_code": int(error_code),
        "reason": reason,
        "detail": dict(detail or {}),
    }


class DedupWindow:
    """(rid, msg_id) 幂等窗口.

    *** 只按时间淘汰, NO 不设条数上限.
    v2.0 S1.2 逐字"不得仅依赖条数上限提前淘汰窗口内的 ID". 一个"只保留
    最近 100 条"的实现在高频下发时会把 30 秒前的 ID 挤掉, 于是一条 40 秒后
    的重发被当成新请求 -- 而它满足"窗口不少于 60 秒"这句话的字面.

    * 内存增长: 窗口是有界的(60 秒), 所以条目数由[这 60 秒内的下发频率]
    决定, 不会无限涨. 每次 seen() 顺手清理过期项, 不另起清理线程 --
    一个只在定时器里清理的实现, 在定时器挂掉时会静默地无限增长.
    """

    def __init__(self, window_s: float = DEDUP_WINDOW_S, clock=None):
        if window_s < DEDUP_WINDOW_S:
            # 收窄窗口会让重复执行成为可能, 而那正是甲方点名要防的.
            raise ValueError(
                "dedup window %.1fs is below the v2.0 S1.2 minimum %.1fs"
                % (window_s, DEDUP_WINDOW_S))
        self._window = float(window_s)
        # 单调钟(CLAUDE.md 3.4): 墙钟阶跃会让窗口忽然变短, 而变短的方向
        # 让重复执行成为可能.
        self._clock = clock or time.monotonic
        self._seen: Dict[Tuple[str, str], float] = {}

    def seen(self, rid: str, msg_id: str) -> bool:
        """这条 (rid, msg_id) 在窗口内出现过吗. 顺便登记本次."""
        now = self._clock()
        self._evict(now)
        key = (rid, msg_id)
        if key in self._seen:
            # * 命中时[不刷新]时间戳: 刷新会让一条被反复重发的消息永远
            # 留在窗口里, 窗口就不再是"最近 60 秒"而是"最后一次重发起 60 秒".
            return True
        self._seen[key] = now
        return False

    def _evict(self, now: float) -> None:
        cutoff = now - self._window
        stale = [k for k, t in self._seen.items() if t < cutoff]
        for key in stale:
            del self._seen[key]

    def size(self) -> int:
        """窗口内条目数. 只为可观测, 不参与判定."""
        return len(self._seen)


def duplicate_ack(*, msg_id: str, ref_msg_id: str, task_id: str,
                  task_type: str) -> Dict[str, Any]:
    """幂等命中时的 ack.

    accepted=true 且 error_code=0 -- 因为原请求确实被受理过. Qt 据此
    知道"这条已经在处理了, 不用再发".
    """
    return build_ack(msg_id=msg_id, ref_msg_id=ref_msg_id, task_id=task_id,
                     task_type=task_type, result=RESULT_DUPLICATE,
                     error_code=0, reason="",
                     detail={"note": "original request already processed"})
