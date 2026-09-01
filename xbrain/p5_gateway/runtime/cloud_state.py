"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cloud_state.py
Brief: 云端出站状态面的节律驱动 -- 变化即发 + 各自的保活频率 (B-3)

Description:
B-2 建好了 publish_state 与七条 publisher, 但[没有人调它]. 这是本轮反复
撞见的同一个形状第三层: 登记表对而接线无(B-1 修) -> 桥写好而启动路径不调
(B-1 补断言) -> 发布口有了而没人驱动(本文件). 每一层单独看都完整, 合起来
在真机上是"Qt 订阅成功, 永远收不到内容", 而 Zenoh 不报任何错.

*** 节律来自 v2.0 S2 逐条, NO 不由实现者顺手定.
  state/robot          固定 10 Hz
  state/task           变化即发 + 至少 1 Hz
  state/mode           1 Hz + 变化即发
  state/audio          1 Hz + 变化即发
  state/media          变化即发 + 每 5 s 全量保活
  state/geo/manifest   变化即发; session 建立后 2 s 内一份全量
  data/file/index      连接/变化时发完整索引
发慢了 Qt 判超时(它对 state/link 是连续 3 秒未收到即离线); 发快了在 Q3 上
挤掉别的东西.

*** "变化即发"按[序列化后的 data]比对, 不按对象比对.
按对象比的话, 一个内部可变的缓存(比如 list 被原地改)会与上一次比出相等,
于是变化发不出去 -- 而它看起来完全正常. 序列化一次的成本远小于一条状态
变化被吞掉的代价.

*** 单调钟(CLAUDE.md 3.4 / CLK-C1).
节律是时长判定. 墙钟在 NTP 阶跃时往回跳会让"该发了没"判错 -- 往前跳一小时
会让每条 key 在同一拍全部触发, 往后跳会让它们静默一小时.

! 已知无源, 写在这里而不是文档里(它会变):
  robot_state 只能取 idle / running -- charging / fault / emergency_stop /
  offline 都没有机内来源(充电态在 p3, 急停接合态没有发布者; estop_probe
  给的是[通路健康]不是[是否已接合], 两者不可互推).
  state/media 与 data/file/index 今天无内容, 发空集合而不是不发 --
  见 _media / _file_index 各自的说明.

Boundaries: 只决定"这一拍发哪几条"并调 publish_state. 形状换算在
outbound/state_projection.py, 信封与 seq 在 outbound/cloud_envelope.py.
"""

from __future__ import annotations

import json
import logging
import time
import uuid
from typing import Any, Callable, Dict, List, Optional

from ..outbound.cloud_envelope import UnmappedLinkLevel
from ..outbound.task_result import TaskResultTracker, build_result
from ..outbound.state_projection import (ProjectionError, audio_payload,
                                         geo_manifest_payload, mode_payload,
                                         robot_payload, task_item,
                                         to_v2_device_status,
                                         to_v2_task_state)

_logger = logging.getLogger(__name__)

#: robot_state 今天能诚实给出的两个值. 见模块头的无源清单.
SOURCED_ROBOT_STATES = ("idle", "running")

#: 每条消息都不同的字段. 变化比对时必须摘掉, 见 _due 的说明.
VOLATILE_FIELDS = ("msg_id",)

#: session 建立后多久必须发一份全量 manifest(v2.0 S2 逐字 2 秒).
#: 取 1.5 是为了留出发布与传输的余量 -- 卡在 2.0 上发, 到 Qt 那里就超了.
MANIFEST_FIRST_S = 1.5


class CloudProjector:
    """把网关已有的 hmi_state 缓存按各自节律投到云端面.

    * 复用 hmi_state 而不另起采集: 两个消费者读同一份数据, HMI 上看到的
    与 Qt 上看到的就不会打架. 各自采集的话, 两个面会在不同时刻取样, 联调
    时"HMI 显示 running 而 Qt 显示 idle"这种问题查起来极费时间.
    """

    def __init__(self, bridge, *,
                 now_mono: Optional[Callable[[], float]] = None) -> None:
        self._bridge = bridge
        self._now = now_mono or time.monotonic
        self._last_sent: Dict[str, float] = {}
        self._last_body: Dict[str, str] = {}
        self._start = self._now()
        self._manifest_sent = False
        # 终态跃迁跟踪. snapshot 按节律发, result 事件驱动 -- 两条走同一条
        # key(v2.0 R12.4 不另设 task/result), 靠 message_type 区分.
        self._results = TaskResultTracker()
        # 回调线程 append, 循环线程取走. 见 observe_task.
        self._pending_results: List[Dict[str, Any]] = []
        #: 只为可观测: 投影抛错的次数. 不参与判定.
        self.errors = 0

    # --- 每拍一次 ------------------------------------------------------

    def tick(self, state: Dict[str, Any]) -> List[str]:
        """决定这一拍发哪几条, 发掉, 返回 key 名列表.

        *** 单条 key 的投影抛错不得中断其余的.
        一条 state/mode 因为闭集越界抛了, 不该让 state/robot 也停发 --
        后者是 Qt 判"机器人还活着"的依据之一. 与 P1 控制循环那条同一取舍
        (CLAUDE.md 4.4: 循环内注入 raise -> 本拍零速 + 落 fault + 下一拍
        循环仍在跑).
        """
        sent: List[str] = []
        # *** result 先于 snapshot 发.
        # 反过来的话, 一个刚完成的任务会先从 snapshot 的 current 里消失
        # (它不再 running), 然后才收到 result -- Qt 中间那一瞬看到的是
        # "没有任务在跑, 也没有结果", 而操作员正盯着屏幕等结果.
        for data in self._drain_results(state):
            self._bridge.publish_state("state/task", data)
            sent.append("state/task:result")
        for name, build in (
                ("state/robot", self._robot),
                ("state/task", self._task),
                ("state/mode", self._mode),
                ("state/audio", self._audio),
                ("state/media", self._media),
                ("state/geo/manifest", self._manifest),
                ("data/file/index", self._file_index),
                ("state/link", self._link)):
            try:
                data = build(state)
            except (ProjectionError, UnmappedLinkLevel) as exc:
                # 闭集越界之类. 记下来但不发 -- 发出去就是让 Qt 收到它
                # 字典里没有的枚举, 而 v2.0 S1.3 禁止它降级解释.
                self.errors += 1
                _logger.error("p5 cloud projection %s refused: %s", name, exc)
                continue
            except Exception:                    # noqa: BLE001
                self.errors += 1
                _logger.exception("p5 cloud projection %s crashed", name)
                continue
            if data is None:
                continue
            if self._due(name, data):
                self._bridge.publish_state(name, data)
                sent.append(name)
        return sent

    def _due(self, name: str, data: Dict[str, Any]) -> bool:
        """该发了吗. 变化即发, 否则等周期.

        * 比对的是序列化后的字符串, 见模块头.

        *** 比对前要摘掉每次都不同的字段(VOLATILE_FIELDS).
        state/task 的 data 里有一个 msg_id, 按 v2.0 S3.2 它每条消息一个新
        值. 直接拿整个 data 去比, [每一拍都会比出"变了"] -- 于是变化检测
        对这条 key 完全失效, 它以 10 Hz 发, 而它的节律是 1 Hz.
        这个缺陷是断言抓出来的: 一条"没变化就不发"的用例红了, 而实现看起来
        毫无问题 -- 谁也不会想到一个 ID 字段能把节律控制整个绕过去.
        """
        from .cloud_wiring import OUTBOUND_PERIODS

        stable = {k: v for k, v in data.items() if k not in VOLATILE_FIELDS}
        body = json.dumps(stable, ensure_ascii=False, sort_keys=True)
        changed = self._last_body.get(name) != body
        now = self._now()
        period = OUTBOUND_PERIODS[name]
        elapsed = now - self._last_sent.get(name, float("-inf"))
        if not changed and (period is None or elapsed < period):
            return False
        self._last_body[name] = body
        self._last_sent[name] = now
        return True

    # --- 各条 key 的投影 ------------------------------------------------

    def _robot(self, state: Dict[str, Any]) -> Dict[str, Any]:
        tasks = state.get("tasks") or []
        running = any(t.get("state") == "running" for t in tasks
                      if isinstance(t, dict))
        clock = state.get("clock")
        ts_sync = bool(clock and clock.get("ts_sync") is True)
        return robot_payload(
            # 只有 idle / running 有来源. 见模块头 -- charging 与
            # emergency_stop 报出来就是编的.
            robot_state="running" if running else "idle",
            task_state="running" if running else "idle",
            pose=state.get("pose"),
            clock=clock,
            # CLK-C1: 单调钟从这里传进去, 投影函数自己不取时间
            # (无设备单测才能喂一个固定的 now).
            devices=_devices_from_health(state.get("health"),
                                          time.monotonic()),
            # v2.0 S3.5: 授时未同步时带时间窗的规则不命中, 所以这里必须
            # 反映[实际生效]而不是配置里写没写. 没有窗配置来源时为 False.
            alarm_window_active=ts_sync and bool(state.get("alarm_window")),
        )

    def _task(self, state: Dict[str, Any]) -> Dict[str, Any]:
        """v2.0 S3.2 全机快照. result 形态由任务终结时另行发布(R12.4)."""
        tasks = [t for t in (state.get("tasks") or []) if isinstance(t, dict)]
        # *** 每一条都要过闭集, NO 不只挑三个桶里的.
        # 原先的写法按 running / queued / paused 三个桶分类, 于是一条
        # state 是闭集外值的任务[哪个桶都进不去, 就此消失] -- 快照里没有它,
        # Qt 上看不到它, 而它可能正在跑. 静默丢弃比抛错糟: 抛错至少有日志.
        # 这条同样是断言抓出来的(CLAUDE.md 3.5 越界必抛).
        #
        # *** 先映射再分桶. 上面三个桶名是 v2.0 的词, 而 tasks 里装的是机内
        # 12 值(15 S3.2) -- 两侧只有 running/failed/cancelled 三个名字碰巧
        # 相同. 不映射的话 pending/ready/done 这些每条任务必经的状态既过不了
        # 闭集校验, 也一个桶都进不去.
        tasks = [dict(t, state=to_v2_task_state(t.get("state")))
                 for t in tasks]
        for task in tasks:
            _closed_task_state(task.get("state"))
        current = next((t for t in tasks if t.get("state") == "running"), None)
        return {
            "msg_id": uuid.uuid4().hex,
            "message_type": "snapshot",
            "current": task_item(current) if current else None,
            "queue": [task_item(t) for t in tasks
                      if t.get("state") == "queued"],
            "suspended": [task_item(t) for t in tasks
                          if t.get("state") == "paused"],
        }

    def observe_task(self, task: Any) -> None:
        """从 state/task 的 Zenoh 回调调用, 每次广播一次.

        *** 这是终态判定成立的前提, 不是优化.
        跃迁规则(task_result.observe)要求看到 [非终态 -> 终态] 那一次跃迁.
        只在 10 Hz 的 tick 上采样的话, 一个 50 ms 内 running -> completed
        的快任务只会被看到 completed 一次, 于是永远发不出 result.

        * RUST 线程(CLAUDE.md 4.2): 只做纯 CPU 判定与 list.append, NO 不
        发布, 不 await. 发布在 tick 里, 由循环线程做.
        """
        if not isinstance(task, dict):
            return
        try:
            data = self._result_for(task)
        except Exception:                        # noqa: BLE001
            self.errors += 1
            _logger.exception("p5 cloud task result build failed")
            return
        if data is not None:
            # list.append 在 GIL 下是原子的, 与本进程其它跨线程缓存同一
            # 做法(见 main_wiring 的 hmi_state 注释).
            self._pending_results.append(data)

    def _result_for(self, task: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """一条任务观察 -> 一条 result(或 None). 跃迁判定在 tracker 里."""
        hit = self._results.observe(task.get("task_id"), task.get("state"))
        if hit is None:
            return None
        task_id, result_state = hit
        failed = result_state != "done"
        return build_result(
            task_id=task_id,
            task_type=task.get("task_type") or "",
            state=result_state,
            # 失败码与原因必须来自任务记录. 没有的话给一个通用码而不是 0 --
            # 0 是"成功", 一条 state=failed 而 code=0 的 result 会让 Qt 走
            # 成功分支.
            result_code=0 if not failed else int(
                task.get("result_code") or 2001),
            reason="" if not failed else (
                task.get("reason") or "task ended without a recorded reason"),
            completed_count=int(task.get("completed_count") or 0),
            total_count=int(task.get("total_count") or 0),
            distance_m=task.get("distance_m"),
            duration_sec=float(task.get("duration_sec") or 0.0),
            started_ts=task.get("started_ts"),
            ended_ts=task.get("ended_ts"),
            route_id=task.get("route_id"),
            route_rev=task.get("route_rev"),
            detail=task.get("result_detail"))

    def _drain_results(self, state: Dict[str, Any]) -> List[Dict[str, Any]]:
        """这一拍有哪些任务刚跃迁到终态.

        NO 不走 _due 的节律判定 -- result 是[事件], 每一条都要发. 走节律的
        话, 两个任务在同一秒内先后结束, 第二条会被"没到点"吃掉, 而它永远
        不会再来一次(跃迁只发生一次).
        """
        # 回调那边攒下的先取走. 用 [:] 一次性换出而不是逐条 pop --
        # 回调随时可能 append, 逐条 pop 会与它交错.
        out: List[Dict[str, Any]] = self._pending_results[:]
        del self._pending_results[:len(out)]
        # 兜底: 快照里若有回调没见过的跃迁(比如回调那一瞬掉了一条), 这里
        # 补上. tracker 的 _done 保证不会因此重复.
        for task in (state.get("tasks") or []):
            if not isinstance(task, dict):
                continue
            data = self._result_for(task)
            if data is not None:
                out.append(data)
        return out

    def _mode(self, state: Dict[str, Any]) -> Dict[str, Any]:
        mode = state.get("mode")
        # 机内 state/mode 用的词与 v2.0 一致(normal/broadcast/alarm);
        # 缺源时取 normal 而不是抛 -- 一条发不出去的 state/mode 会让 Qt
        # 的喊话按钮永远处于未知态, 而 normal 是它的默认显示.
        return mode_payload(
            voice_mode=mode if mode in ("normal", "broadcast", "alarm")
            else "normal",
            source="system",
            stream_id=state.get("stream_id"),
            entered_ts=state.get("mode_entered_ts"),
            exit_reason=state.get("mode_exit_reason"))

    def _audio(self, state: Dict[str, Any]) -> Dict[str, Any]:
        """喇叭/麦克风状态.

        * speaker_holder 反映半双工门控的持有者. 本项目 AEC 结构上不可能
        (TTS 在 GZH-2 设备内合成, 上装侧拿不到播出波形), 所以喇叭与麦克风
        互斥是靠门控实现的 -- microphone_state 在放音时是 disabled 而不是
        idle, 这个差别对 Qt 的按钮显示有意义.
        """
        playing = bool(state.get("speaking"))
        return audio_payload(
            speaker_state="playing" if playing else "idle",
            microphone_state="disabled" if playing else "idle",
            stream_id=state.get("stream_id"),
            speaker_holder=state.get("speaker_holder"),
            speaker_holder_type=state.get("speaker_holder_type"),
            last_frame_age_ms=state.get("last_frame_age_ms"))

    def _media(self, state: Dict[str, Any]) -> Dict[str, Any]:
        """动态 RTSP 端点.

        *** 无端点时发空数组, NO 不是不发.
        不发的话 Qt 收不到任何 state/media, 与"后端挂了"不可区分; 而一个
        endpoints: [] 明确说"这台机器上现在没有可用画面". v2.0 要求每 5 s
        全量保活, 保活的前提是有东西可发.
        """
        return {"endpoints": list(state.get("media_endpoints") or ())}

    def _manifest(self, state: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """geo 全量清单.

        v2.0 S2: session 建立后 2 s 内发一份全量, 之后变化即发. 这里用
        [网关启动]近似 session 建立 -- 云端 session 的建立时刻我方没有
        权威信号(Qt 订阅是 Zenoh 内部的事, 不产生回调).
        * 这个近似是保守的: 网关先起, Qt 后连, Qt 连上时清单已经在发了.
        """
        cache = state.get("geo_cache")
        objects = []
        if cache is not None and hasattr(cache, "snapshot"):
            snap = cache.snapshot(int(self._now() * 1000.0))
            objects = _manifest_objects(snap)
        if not objects and not self._manifest_sent:
            if self._now() - self._start < MANIFEST_FIRST_S:
                # 还在开机头 1.5 秒内, geo 可能还没广播过来. 等一等再发,
                # 免得先发一份空清单让 Qt 以为一个对象都没有.
                return None
        self._manifest_sent = True
        return geo_manifest_payload(
            manifest_rev=int(state.get("geo_manifest_rev", 0)),
            objects=objects)

    def _link(self, state: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """v2.0 S4.1 的四字段.

        *** 机内 state/link 与云端 state/link 形状完全不同.
        机内那条(11 S4.6)带 level / gw_start_mono / link_epoch / thresholds
        一大堆, 是给 P1 判返航和给 HMI 点亮 ESTOP 用的; v2.0 只要四个字段.
        直接把机内那条转出去, Qt 会因为缺 state / cloud_link 而整条拒收
        (S1.3 必填字段缺失是拒绝条件) -- 而拒收在 Zenoh 侧不产生任何回音,
        表现就是"Qt 判机器人离线", 且它每 3 秒判一次, 永远离线.

        * link 缺源时返回 None(不发) 而不是发一个 down.
        发 down 的话 Qt 显示离线 -- 而"网关刚起来还没算出链路状态"与
        "链路真的断了"是两件事. 不发, Qt 按它自己的 3 秒规则判离线, 结论
        一样但没有我方编造的成分.
        """
        from ..outbound.cloud_envelope import UnmappedLinkLevel, link_state_word

        link = state.get("link")
        if not link:
            return None
        try:
            word = link_state_word(link.get("level"))
        except UnmappedLinkLevel:
            # E-2 裁决(2026-08-24, 用户)后 L0..L3 都有落点(L3->degraded);
            # 到这里说明 level 越界(<0 或 >3), 那是上游 link_state 的缺陷.
            # 抛而不猜(3.5 越界必抛); tick 的兜底记 error, 这拍不发 state/link
            # (Qt 按 3 秒规则判离线, 好过发一个与真实链路无关的猜测值).
            raise
        return {
            "state": word,
            "cloud_link": word != "down",
            "disconnected_s": float(link.get("disconnected_s") or 0.0),
            "estop_path": link.get("estop_path") or "down",
        }

    def _file_index(self, state: Dict[str, Any]) -> Dict[str, Any]:
        """可下载文件索引.

        今天无内容(文件面未建). 与 media 同一取舍: 发空数组说"没有可下载
        的文件", 不发则与后端挂了不可区分.
        """
        return {"files": list(state.get("file_index") or ())}


# --- 小工具 -----------------------------------------------------------

def _closed_task_state(value: Any) -> str:
    """任务 state 必须在 v2.0 S3.2 闭集内, 否则抛.

    调用点在 _task 的开头, 对[每一条]任务求值 -- 包括那些不会进任何桶的.
    见 _task 里的说明.
    """
    from ..outbound.state_projection import TASK_STATES

    if value not in TASK_STATES:
        raise ProjectionError(
            "task.state=%r not in the v2.0 S3.2 closed set %s"
            % (value, TASK_STATES))
    return value



def _devices_from_health(health: Optional[Dict[str, Any]],
                         now_mono: Optional[float] = None
                         ) -> List[Dict[str, Any]]:
    """health/summary -> v2.0 的 devices[].

    *** 只发[实际发现的]设备(v2.0 S4.2 逐字), 且只发 kind=="device".
    11 S5.1 的 items 混着两类: device(物理设备在不在) 与 cap(一项能力是否
    成立 -- 时钟同步/算力余量/磁盘/电量). v2.0 要的是设备清单, 把 cap 混进去
    会让 Qt 的设备面板出现 clock/compute 这种点不开也修不了的"设备".

    *** 本函数 2026-09-01 重写. 原版对着一个想象的形状写, 三个字段全错:
    找 item["status"](真名 state, 且值域完全不同) / item["label"](11 S5.1 没有
    这个键) / item["age_ms"](真名 since_mono, 单位是单调钟秒不是毫秒).
    加上订阅的 key 也是错的(见 main_wiring 的 HEALTH_SUMMARY_TOPIC 注释),
    于是 devices 恒空. 而 v2.0 S4.2 的"后端只发布实际发现的设备"让这个空
    数组看起来完全合规 -- 空的原因是"没收到", 不是"没发现", 两者在报文上
    不可区分. 联调时甲方会以为机器人一个设备都没接.

    *** name 用 id 本身, NO 不在这里建中文名表.
    11 S5.1A 的十九项规范表只有 项/kind/level/是否计入 speed_factor/失败后果,
    没有人类可读名这一列 -- 全库没有这个真源. 在网关新建一张 key->中文名的
    表就是造第二份真源, 它会在闭集扩容时漂移(CLAUDE.md 3.7 那条"人抄的数
    会过期"). 报 id 是诚实的; 要中文名应由 11 S5.1 加 label 字段, 那是改契约.

    *** last_update_ms 无来源时为 null, NO 不填 0.
    since_mono 在 11 S5.1 是可选字段. 填 0 表示"刚刚更新", 与真的刚更新完全
    一样 -- 与 CLAUDE.md 3.1 那条"0.0 冒充已赋值"是同一个失效模式.
    now_mono 由调用方传入(CLK-C1: 时间从外面传, 便于无设备单测).
    """
    items = (health or {}).get("items") or {}
    out = []
    for name, item in sorted(items.items()):
        if not isinstance(item, dict):
            continue
        if item.get("kind") != "device":
            continue
        age_ms = None
        since = item.get("since_mono")
        if now_mono is not None and isinstance(since, (int, float)):
            # 单调钟差; 负值(时钟回拨不可能, 但报文可能是坏的)夹到 0.
            age_ms = int(max(0.0, now_mono - float(since)) * 1000.0)
        out.append({"id": name,
                    "name": name,
                    "status": to_v2_device_status(item.get("state")),
                    "last_update_ms": age_ms})
    return out


def _manifest_objects(snap: Any) -> List[Dict[str, Any]]:
    """GeoCache 快照 -> v2.0 objects[].

    只搬 v2.0 认的三类. 机内 geo 还有 dock 等对象, 它们不在客户契约里 --
    发过去 Qt 会因为 type 越界而拒收整条消息(v2.0 S1.3 把枚举越界列为
    拒绝条件), 于是[一个多余的对象会让整份清单发不出去].
    """
    out: List[Dict[str, Any]] = []
    if not isinstance(snap, dict):
        return out
    for kp in snap.get("keypoints") or ():
        gid = kp.get("id") or ""
        out.append({"geo_id": gid if gid.startswith("w-") else "w-" + gid,
                    "type": "waypoint", "name": kp.get("name", ""),
                    "rev": kp.get("rev", 0),
                    "latitude": kp.get("lat"), "longitude": kp.get("lon"),
                    "altitude": kp.get("alt")})
    for rt in snap.get("routes") or ():
        gid = rt.get("id") or ""
        out.append({"geo_id": gid if gid.startswith("r-") else "r-" + gid,
                    "type": "recorded_path", "name": rt.get("name", ""),
                    "rev": rt.get("rev", 0)})
    for fc in snap.get("fences") or ():
        gid = fc.get("id") or ""
        out.append({"geo_id": gid if gid.startswith("f-") else "f-" + gid,
                    "type": "alarm_region", "name": fc.get("name", ""),
                    "rev": fc.get("rev", 0),
                    "enabled": bool(fc.get("enabled", True))})
    return out


__all__ = ["CloudProjector", "SOURCED_ROBOT_STATES",
           "MANIFEST_FIRST_S", "VOLATILE_FIELDS"]
