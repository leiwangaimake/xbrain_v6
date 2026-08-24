"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cloud_wiring.py
Brief: 云端 Qt 面的真实接线 -- 五条入站订阅 + 三条 ack 发布 (B-1)

Description:
本文件是 2026-08-23 那次核实的直接产物. 当时查出 outbound/key_surface.py
里躺着一张完整且正确的云端 key 登记表, 而 P5 的真实接线里[一条云端 key 都
没有] -- 入站 0/5, 出站 1/12. 一张对的表让每个读它的人都以为面已经建好了.
本文件把表变成接线.

*** 云端 key 与机内 key 是[两条不同的 Zenoh key], 这是整个设计的支点.
  云端  xbrain/{rid}/cmd/task    <- Qt 发, 只有网关订
  机内  cmd/task                 <- 网关重建后发, p3_task 订(它今天就订这条)
两者只差一个 xbrain/{rid}/ 前缀, 但在 Zenoh 眼里毫无关系. 这一层落差正是
v2.0 S7.3 要求的"校验后重建"的落脚点: Qt 的报文进不了机内总线, 除非网关
先验过信封, 核过 rid, 拆过 task_type.

* 由此得到一条重要性质: [语音/文本链路一个字都不用动]. p3_task 今天订
cmd/task, 明天还订 cmd/task, 收到的报文形状(11 S7.2 的 {action, task,
cmd_id})也完全一样 -- 因为网关重建出来的就是那个形状. 云端只是往同一个
机内入口多接了一根管子, 与本地 MIC / HMI / 将来的微信并列, 共用下游同一套
仲裁与互斥. 这是用户 2026-08-24 明令的"不要影响其他语音, 文本任务指令的
实际功能连线和功能执行还有互斥".

*** origin 恒为 "cloud", NO 不从报文里取.
CH-1"通道即权限": 授权边界是[报文从哪条 key 进来的], 不是报文自己声称的
身份. 一个从 cmd/task 云端 key 进来却自称 origin="voice" 的报文, 如果被
信了, 就绕过了云端通道该有的全部限制. task_router.CLOUD_ORIGIN 是唯一写者.

*** 回环防护: is_cloud_frame 按 src 区分.
用户裁决 E-1 选了"网关多订 cmd/task". 虽然前缀差异已经让机内那条与云端那条
不是同一 key, src 判别仍然保留 -- 因为它防的是另一件事: 若将来有人把两条
key 合一(或加了 xbrain/**/cmd/task 这样的通配订阅), 网关会开始收到自己刚
重建的那条并再重建一次, 无限回环, 而每一圈都是合法报文, 没有任何东西会报错.
这类缺陷在单条报文的测试里完全看不出来.

Boundaries: 只做[信封校验 -> 拆分 -> 转机内 key -> 回 ack]. NO 不判断任务
业务上能不能执行(那是 p3_task 的活, 它的答复走机内 cmd/task/ack), 也不做
状态投影(出站 state/* 在 B-2).
"""

from __future__ import annotations

import json
import logging
import time
import uuid
from typing import Any, Callable, Dict, List, Optional, Tuple

from ..inbound.cloud_inbound import (InboundReject, SRC_QT, frame_ids,
                                     is_cloud_frame, parse_frame, rid_from_key)
from ..inbound.task_router import CLOUD_ORIGIN, route
from ..outbound.cloud_envelope import SeqCounter, build_envelope
from ..outbound.task_ack import (DedupWindow, RESULT_ACCEPTED, RESULT_REJECTED,
                                 build_ack, duplicate_ack)

_logger = logging.getLogger(__name__)


# --- 云端 key 模板 -----------------------------------------------------
#
# 一律带 xbrain/%s/ 前缀(rid). 与 p1_motion 订 rt/gnss 用的同一种写法 --
# 那里也是 "xbrain/%s/rt/gnss/heading" % rid. 用模板常量而不是就地拼串,
# 是为了让 tests/p5_gateway/test_cloud_key_surface_wired.py 的 AST 提取器
# 能从真实接线里读出 key: 它解析 declare_subscriber 的第一个实参.

CLOUD_CMD_TASK = "xbrain/%s/cmd/task"
CLOUD_CMD_ESTOP = "xbrain/%s/cmd/estop"
CLOUD_CMD_MEDIA_SESSION = "xbrain/%s/cmd/media/session"
CLOUD_CMD_FILE_ACK = "xbrain/%s/cmd/file/ack"
CLOUD_AUDIO_BROADCAST = "xbrain/%s/audio/broadcast"

CLOUD_CMD_TASK_ACK = "xbrain/%s/cmd/task/ack"
CLOUD_CMD_ESTOP_ACK = "xbrain/%s/cmd/estop/ack"
CLOUD_CMD_MEDIA_SESSION_ACK = "xbrain/%s/cmd/media/session/ack"

# 出站状态面(B-2). 由 CloudProjector 按各自节律发布.
CLOUD_STATE_TASK = "xbrain/%s/state/task"
CLOUD_STATE_ROBOT = "xbrain/%s/state/robot"
CLOUD_STATE_MODE = "xbrain/%s/state/mode"
CLOUD_STATE_AUDIO = "xbrain/%s/state/audio"
CLOUD_STATE_MEDIA = "xbrain/%s/state/media"
CLOUD_STATE_GEO_MANIFEST = "xbrain/%s/state/geo/manifest"
CLOUD_DATA_FILE_INDEX = "xbrain/%s/data/file/index"
CLOUD_STATE_LINK = "xbrain/%s/state/link"

#: 事件面. v2.0 的 key 是 xbrain/{rid}/event/{severity}/{category}, 而机内
#: 生产者(p2_core / p3_task / p5 自己)一律发在相对 event/{sev}/{cat} 上.
#: 两者不是同一条 key, 所以[Qt 今天一条事件都收不到].
#:
#: *** 这一条与 17 S3.5.0"P5 不是实时中继"看似冲突, 裁决是转发, 理由有二:
#: (1) 用户 2026-08-24 明令: 契约与这三份客户文档冲突时以客户文档为准,
#:     而 v2.0 把 key 定死带 rid 前缀;
#: (2) 更实质的一条 -- 生产者发的是[裸事件体], 没有 v2.0 的六字段信封
#:     (rid / seq / src 都没有). 即便把生产者的 key 改成绝对形, 信封仍然
#:     缺, Qt 会按 S1.3 的"必填字段缺失"整条拒收. 所以无论如何都要有一个
#:     加信封的点, 那个点只能是网关.
#: 改生产者的 key 会动到 p2_core / p3_task 的既有接线与 HMI 的事件流,
#: 而转发是纯增量.
CLOUD_EVENT = "xbrain/%s/event/%s/%s"

#: 出站状态面九条(不含 state/link 与 event/**, 那两条另有发布者).
#: v2.0 S2 给的节律, 单位秒. state/robot 固定 10 Hz 是其中最快的一条.
OUTBOUND_PERIODS = {
    "state/robot": 0.1,           # v2.0 S4.2 逐字"固定 10 Hz"
    "state/task": 1.0,            # 变化即发 + 至少 1 Hz
    "state/mode": 1.0,            # 1 Hz + 变化即发
    "state/audio": 1.0,           # 1 Hz + 变化即发
    "state/media": 5.0,           # 每 5 s 全量保活
    "state/geo/manifest": None,   # 变化即发; session 建立后 2 s 内一份全量
    "data/file/index": None,      # 可靠面, 连接/变化时发
    "state/link": 1.0,            # v2.0 S4.1 逐字"1 Hz + 变化即发"
}

#: 入站五条. 顺序即 v2.0 S2 表的顺序.
INBOUND_TEMPLATES = (CLOUD_CMD_TASK, CLOUD_CMD_ESTOP, CLOUD_CMD_MEDIA_SESSION,
                     CLOUD_CMD_FILE_ACK, CLOUD_AUDIO_BROADCAST)


def relative_key(template: str) -> str:
    """把 "xbrain/%s/cmd/task" 还原成 "cmd/task".

    登记表与契约表都用相对名, 接线里用绝对模板; 这个函数是两者之间唯一的
    换算点, 免得两处各写一份而在某次改名时错开.
    """
    head = "xbrain/%s/"
    if not template.startswith(head):
        raise ValueError("not a cloud key template: %r" % template)
    return template[len(head):]


class CloudBridge:
    """云端入站桥. 持订阅与发布句柄, 逐条报文走"验 -> 拆 -> 转 -> 答".

    *** 句柄必须被本对象接住(CLAUDE.md 4.3).
    declare_subscriber 的返回值一旦被 GC, Rust 端订阅会[悄悄注销] -- 没有
    异常, 没有日志, 只是从此收不到报文. 与"客户还没连上来"完全不可区分.
    """

    def __init__(self, session: Any, rid: str, *,
                 internal_put: Optional[Callable[[str, bytes], None]] = None,
                 dedup: Optional[DedupWindow] = None) -> None:
        if not rid:
            # 没有 rid 就构不出合法 key. 宁可不建桥也不建一条 "xbrain//cmd/task"
            # -- 后者会订到一个谁都不发的 key 上, 表现为"客户端连上了但没反应".
            raise ValueError("cloud bridge needs a rid")
        self._session = session
        self._rid = rid
        self._subs: List[Any] = []          # 强引用容器, 见类文档
        self._pubs: Dict[str, Any] = {}
        self._seq = SeqCounter()
        self._dedup = dedup or DedupWindow()
        self._internal_put = internal_put or self._default_internal_put
        self._internal_pubs: Dict[str, Any] = {}
        # 事件 publisher 按 (sev, cat) 缓存. 见 publish_event.
        self._event_pubs: Dict[str, Any] = {}
        #: 只为可观测: 各类报文的处理计数. 不参与任何判定.
        self.stats: Dict[str, int] = {"accepted": 0, "rejected": 0,
                                      "duplicate": 0, "ignored": 0}

    # --- 接线 ---------------------------------------------------------

    def wire(self) -> None:
        """声明五条入站订阅与三条 ack 发布."""
        rid = self._rid
        self._subs.append(self._session.declare_subscriber(
            CLOUD_CMD_TASK % rid, self._on_cloud_task))
        self._subs.append(self._session.declare_subscriber(
            CLOUD_CMD_ESTOP % rid, self._on_cloud_estop))
        self._subs.append(self._session.declare_subscriber(
            CLOUD_CMD_MEDIA_SESSION % rid, self._on_cloud_media_session))
        self._subs.append(self._session.declare_subscriber(
            CLOUD_CMD_FILE_ACK % rid, self._on_cloud_file_ack))
        self._subs.append(self._session.declare_subscriber(
            CLOUD_AUDIO_BROADCAST % rid, self._on_cloud_audio_broadcast))

        self._pubs["cmd/task/ack"] = self._session.declare_publisher(
            CLOUD_CMD_TASK_ACK % rid)
        self._pubs["cmd/estop/ack"] = self._session.declare_publisher(
            CLOUD_CMD_ESTOP_ACK % rid)
        self._pubs["cmd/media/session/ack"] = self._session.declare_publisher(
            CLOUD_CMD_MEDIA_SESSION_ACK % rid)
        # 出站状态面(B-2). declare 与 put 分开: Qt 一订阅就该看到有发布者,
        # 哪怕第一帧还没到 -- 一条没有发布者的 key 在 Zenoh 上与"网络不通"
        # 不可区分, 而客户会去查他们自己的网络.
        #
        # * 逐条写开, NO 不用循环 -- 循环把 key 藏进了运行期.
        # 守这面的判据(tests/.../test_cloud_key_surface_wired.py)是静态
        # AST 提取: 它读 declare_publisher 的第一个实参. 写成
        # `for _, tpl in (...): declare_publisher(tpl % rid)` 的话, 实参是
        # 一个循环变量, 提取器看不见任何 key -- 于是"这条 key 接了没有"
        # 变成一个只能靠起栈才能回答的问题. 一段让静态检查失明的代码,
        # 省下的几行不值.
        self._pubs["state/task"] = self._session.declare_publisher(
            CLOUD_STATE_TASK % rid)
        self._pubs["state/robot"] = self._session.declare_publisher(
            CLOUD_STATE_ROBOT % rid)
        self._pubs["state/mode"] = self._session.declare_publisher(
            CLOUD_STATE_MODE % rid)
        self._pubs["state/audio"] = self._session.declare_publisher(
            CLOUD_STATE_AUDIO % rid)
        self._pubs["state/media"] = self._session.declare_publisher(
            CLOUD_STATE_MEDIA % rid)
        self._pubs["state/geo/manifest"] = self._session.declare_publisher(
            CLOUD_STATE_GEO_MANIFEST % rid)
        self._pubs["data/file/index"] = self._session.declare_publisher(
            CLOUD_DATA_FILE_INDEX % rid)
        self._pubs["state/link"] = self._session.declare_publisher(
            CLOUD_STATE_LINK % rid)
        _logger.info("p5 cloud bridge wired: rid=%s, %d subs, %d pubs",
                     rid, len(self._subs), len(self._pubs))

    def alive(self) -> int:
        """活着的订阅数. 启动自检用 -- 见 CLAUDE.md 4.3."""
        return len(self._subs)

    # --- 入站: cmd/task ------------------------------------------------

    def _on_cloud_task(self, sample: Any) -> None:
        """RUST 线程. 全程无 await 无 asyncio(CLAUDE.md 4.2).

        这里做的是纯 CPU 的解码与判定, 加两次 Zenoh put. NO 不碰事件循环,
        不进队列 -- 一条 Qt 指令走队列意味着它要等下一个 tick, 而 v2.0 S3.1
        给的 ack 预算是 2 秒, 队列会把预算花在等待上.
        """
        try:
            self._handle_task(sample)
        except Exception:                       # noqa: BLE001
            # *** 兜底必须在这里, 不在更里层.
            # Zenoh 的回调若抛出, 异常在 Rust 侧被吞掉, 表现是"这条报文没了"
            # 而不是任何报错. 后续报文照收 -- 于是缺陷以"偶发丢指令"的形式
            # 出现, 极难定位.
            _logger.exception("p5 cloud cmd/task handler crashed")

    def _handle_task(self, sample: Any) -> None:
        raw, key = _sample_parts(sample)
        rid = rid_from_key(key)
        if rid is None or rid != self._rid:
            # 订的是精确 key, 正常到不了这里; 到了说明有人用通配订阅或改了
            # key 结构. 这时候没有可信的 rid, 连 ack 都发不出去 -- 只能记日志.
            _logger.warning("p5 cloud cmd/task on unexpected key %r", key)
            self.stats["ignored"] += 1
            return

        try:
            body = parse_frame(raw, rid)
        except InboundReject as exc:
            self._reject_task(raw, exc.fields)
            return

        if not is_cloud_frame(body):
            # 网关自己重建的报文. 见模块头"回环防护".
            self.stats["ignored"] += 1
            return

        msg_id, task_id, task_type = frame_ids(body)
        if msg_id and self._dedup.seen(rid, msg_id):
            self._publish_ack("cmd/task/ack", duplicate_ack(
                msg_id=_new_msg_id(), ref_msg_id=msg_id,
                task_id=task_id or "", task_type=task_type or ""))
            self.stats["duplicate"] += 1
            return

        try:
            internal_key, payload = route(body["data"])
        except InboundReject as exc:
            self._reject_task(raw, exc.fields, msg_id=msg_id,
                              task_id=task_id, task_type=task_type)
            return

        # *** 先转发再回 ack.
        # 反过来的话, 一次转发失败会留下一条"已受理"的 ack 而机器人什么都
        # 没做 -- Qt 那边看到 accepted 就不会重发了.
        self._internal_put(internal_key, json.dumps(
            payload, ensure_ascii=False).encode("utf-8"))
        self._publish_ack("cmd/task/ack", build_ack(
            msg_id=_new_msg_id(), ref_msg_id=msg_id or "",
            task_id=task_id or "", task_type=task_type or "",
            result=RESULT_ACCEPTED))
        self.stats["accepted"] += 1
        _logger.info("p5 cloud task %s -> %s (origin=%s)",
                     task_type, internal_key, CLOUD_ORIGIN)

    def _reject_task(self, raw: bytes, fields: Dict[str, Any], *,
                     msg_id: Optional[str] = None,
                     task_id: Optional[str] = None,
                     task_type: Optional[str] = None) -> None:
        """把一条拒绝填成 v2.0 S3.1 的八字段 ack.

        *** 拒绝也必须回 ack, 且必须尽量带上 ref_msg_id.
        v2.0 S7.3 逐字禁止静默丢弃. 但报文可能坏在 JSON 层, 那时连 msg_id
        都读不出来 -- 于是这里再做一次[尽力而为]的浅解析: 能读出 msg_id 就
        带上, 读不出就留空. 留空的 ack 仍然有用(Qt 至少知道有东西被拒了),
        NO 不因为读不出 id 就不发.
        """
        if msg_id is None:
            msg_id, task_id, task_type = _best_effort_ids(raw)
        self._publish_ack("cmd/task/ack", build_ack(
            msg_id=_new_msg_id(), ref_msg_id=msg_id or "",
            task_id=task_id or "", task_type=task_type or "",
            result=RESULT_REJECTED,
            error_code=fields["error_code"], reason=fields["reason"],
            detail=fields.get("detail")))
        self.stats["rejected"] += 1

    # --- 入站: cmd/estop ----------------------------------------------

    def _on_cloud_estop(self, sample: Any) -> None:
        """急停. NO 不走任务管线, 不查去重窗口.

        *** 急停不做幂等抑制 -- 这是与 cmd/task 的关键差别.
        重复的 GOTO_KEYPOINT 会创建第二条任务, 所以要拦; 重复的急停只是
        再停一次, 无害. 反过来, 一条被去重窗口吃掉的急停就是[没停] --
        用户按了两次而第二次没有生效. 两个方向的代价不对称, 所以这里选
        "宁可多停一次".
        """
        try:
            raw, key = _sample_parts(sample)
            rid = rid_from_key(key)
            if rid != self._rid:
                self.stats["ignored"] += 1
                return
            try:
                body = parse_frame(raw, rid)
            except InboundReject as exc:
                msg_id, _, _ = _best_effort_ids(raw)
                self._publish_ack("cmd/estop/ack", build_ack(
                    msg_id=_new_msg_id(), ref_msg_id=msg_id or "",
                    task_id="", task_type="ESTOP", result=RESULT_REJECTED,
                    error_code=exc.fields["error_code"],
                    reason=exc.fields["reason"],
                    detail=exc.fields.get("detail")))
                self.stats["rejected"] += 1
                return
            if not is_cloud_frame(body):
                self.stats["ignored"] += 1
                return
            msg_id, task_id, _ = frame_ids(body)
            data = body["data"]
            # 机内 cmd/estop 的形状与 HMI 按钮发的完全一致(见 main_wiring
            # _estop_sender), 于是下游 quadruped 侧不需要分辨来源.
            action = data.get("payload", {}).get("action", "stop")
            self._internal_put("cmd/estop", json.dumps(
                {"type": "estop", "action": action,
                 "origin": CLOUD_ORIGIN}, ensure_ascii=False).encode("utf-8"))
            self._publish_ack("cmd/estop/ack", build_ack(
                msg_id=_new_msg_id(), ref_msg_id=msg_id or "",
                task_id=task_id or "", task_type="ESTOP",
                result=RESULT_ACCEPTED))
            self.stats["accepted"] += 1
            _logger.warning("p5 cloud ESTOP %s -> cmd/estop", action)
        except Exception:                       # noqa: BLE001
            _logger.exception("p5 cloud cmd/estop handler crashed")

    # --- 入站: 三条尚无下游的 key --------------------------------------

    def _on_cloud_media_session(self, sample: Any) -> None:
        """媒体会话请求. 下游端点尚未建(B 组 state/media), 这里先如实拒绝.

        *** 为什么是"拒绝"而不是"先接住不答".
        不答就是静默丢弃(v2.0 S7.3 明禁): Qt 点了拉流没反应, 操作员会重试,
        重试同样没反应. 回一条 E_NOT_IMPLEMENTED 的拒绝, 联调当天一眼就能
        看出是哪一侧没做完. NO 不回一个假的 accepted -- 那会让 Qt 去连一个
        不存在的端点, 表现成网络故障.
        """
        self._reject_unimplemented(sample, "cmd/media/session/ack",
                                   "media session endpoints not implemented")

    def _on_cloud_file_ack(self, sample: Any) -> None:
        """Qt 对 data/file/index 的确认. 索引发布尚未建, 收到只记日志.

        NO 这条不回 ack -- 它自己就是一条 ack. 回一条 ack 的 ack 会在两侧
        之间形成来回.
        """
        try:
            raw, key = _sample_parts(sample)
            _logger.info("p5 cloud file ack on %s: %d bytes (no index "
                         "publisher yet)", key, len(raw))
            self.stats["ignored"] += 1
        except Exception:                       # noqa: BLE001
            _logger.exception("p5 cloud cmd/file/ack handler crashed")

    def _on_cloud_audio_broadcast(self, sample: Any) -> None:
        """云端喊话 PCM 帧. 机内 TTS 链路在, 云端 PCM 入站未接.

        NO 这条同样不回 ack: v2.0 里 audio/broadcast 是连续帧流, 逐帧回 ack
        会把 ack 的量做到与音频帧一样多. 它的答复走 state/audio(B-2).
        """
        try:
            raw, _key = _sample_parts(sample)
            self._audio_bytes = getattr(self, "_audio_bytes", 0) + len(raw)
            self.stats["ignored"] += 1
        except Exception:                       # noqa: BLE001
            _logger.exception("p5 cloud audio/broadcast handler crashed")

    def _reject_unimplemented(self, sample: Any, ack_name: str,
                              reason: str) -> None:
        from ...common import errors
        from ..outbound.error_map import to_qt_code

        try:
            raw, _key = _sample_parts(sample)
            msg_id, task_id, task_type = _best_effort_ids(raw)
            self._publish_ack(ack_name, build_ack(
                msg_id=_new_msg_id(), ref_msg_id=msg_id or "",
                task_id=task_id or "", task_type=task_type or "",
                result=RESULT_REJECTED,
                error_code=to_qt_code(errors.E_NOT_IMPLEMENTED),
                reason=reason,
                detail={"code": errors.E_NOT_IMPLEMENTED}))
            self.stats["rejected"] += 1
        except Exception:                       # noqa: BLE001
            _logger.exception("p5 cloud %s reject failed", ack_name)

    # --- 出站 ---------------------------------------------------------

    def _publish_ack(self, name: str, data: Dict[str, Any]) -> None:
        """把 ack data 包进六字段信封发出去."""
        pub = self._pubs.get(name)
        if pub is None:
            _logger.error("p5 cloud ack %s has no publisher", name)
            return
        # ts 是 v2.0 S1.1 的 float64 Unix 秒 -- 墙钟, 给 Qt 做跨机对齐与
        # 日志用. CLAUDE.md 3.4 禁的是拿墙钟做[超时/周期/年龄]判定, 这里
        # 一个都不是: 本进程内的时长判定(去重窗口)走的是 DedupWindow 的
        # 单调钟.
        body = build_envelope(self._rid, name, data,
                              ts=time.time(),   # WALL-CLOCK-OK(align)
                              seq=self._seq.next(self._rid, name))
        pub.put(json.dumps(body, ensure_ascii=False).encode("utf-8"))

    def publish_state(self, name: str, data: Dict[str, Any]) -> None:
        """把一段已投影好的 data 发到对应云端 key.

        *** 投影本身在 outbound/state_projection.py, 这里只负责发.
        分开是因为投影是纯函数(可在本机逐字段断言), 而发布要 Zenoh. 混在
        一起的话, "字段对不对"就只能在真机上验 -- 而真机验证的每一轮要
        几分钟, 于是没人会去验边界情况.
        """
        pub = self._pubs.get(name)
        if pub is None:
            raise KeyError("no cloud publisher for %r" % name)
        body = build_envelope(self._rid, name, data,
                              ts=time.time(),   # WALL-CLOCK-OK(align)
                              seq=self._seq.next(self._rid, name))
        pub.put(json.dumps(body, ensure_ascii=False).encode("utf-8"))

    def publish_event(self, severity: str, category: str,
                      data: Dict[str, Any]) -> None:
        """把一条机内事件转到云端 event key 上.

        *** 每条 (sev, cat) 组合一个 publisher, 按需建并缓存.
        事件 key 带两段通配, 逐条声明不现实(九类 x 四级). 缓存是为了不在
        每条事件上做一次 declare -- 声明有成本, 而告警风暴时事件是成批的.

        * seq 仍按 key 分区: Qt 对可靠面按业务 ID(eid)去重, 但 seq 的连续性
        是它判丢包的依据, 混在一起会让每条 key 看起来一直在丢.
        """
        key = "event/%s/%s" % (severity, category)
        pub = self._event_pubs.get(key)
        if pub is None:
            pub = self._session.declare_publisher(
                CLOUD_EVENT % (self._rid, severity, category))
            self._event_pubs[key] = pub
        body = build_envelope(self._rid, key, data,
                              ts=time.time(),   # WALL-CLOCK-OK(align)
                              seq=self._seq.next(self._rid, key))
        pub.put(json.dumps(body, ensure_ascii=False).encode("utf-8"))

    def _default_internal_put(self, key: str, payload: bytes) -> None:
        """往机内相对 key 发. publisher 按 key 缓存 -- 每条报文新建一个
        publisher 会在 Zenoh 侧反复做声明/注销, 而声明是有成本的."""
        pub = self._internal_pubs.get(key)
        if pub is None:
            pub = self._session.declare_publisher(key)
            self._internal_pubs[key] = pub
        pub.put(payload)


# --- 小工具 -----------------------------------------------------------

def _sample_parts(sample: Any) -> Tuple[bytes, str]:
    """从 Zenoh sample 取 (payload, key). 两种属性名都试 -- zenoh-python
    在 0.x 与 1.x 之间改过 key_expr 的取法."""
    raw = bytes(sample.payload)
    key = getattr(sample, "key_expr", None)
    return raw, str(key) if key is not None else ""


def _new_msg_id() -> str:
    """ack 自己的 msg_id. v2.0 S1.1: 每条报文一个新 id, 与 ref_msg_id 不同."""
    return uuid.uuid4().hex


def _best_effort_ids(raw: bytes) -> Tuple[Optional[str], Optional[str],
                                          Optional[str]]:
    """报文可能坏在任何一层; 尽力读出三个 id, 读不出返回 None.

    NO 不让这里抛 -- 它的调用点全部在"已经要发拒绝了"的路径上, 再抛一次
    就等于把拒绝也弄丢了, 回到静默丢弃.
    """
    try:
        body = json.loads(raw.decode("utf-8"))
        if not isinstance(body, dict):
            return None, None, None
        data = body.get("data")
        if not isinstance(data, dict):
            return None, None, None
        return (_str_or_none(data.get("msg_id")),
                _str_or_none(data.get("task_id")),
                _str_or_none(data.get("task_type")))
    except Exception:                           # noqa: BLE001
        return None, None, None


def _str_or_none(value: Any) -> Optional[str]:
    return value if isinstance(value, str) else None


def maybe_wire(session: Any, rid: str) -> Optional[CloudBridge]:
    """rid 存在才建桥, 否则返回 None.

    与 p1_motion 的 GNSS 桥同一取舍: XBRAIN_ROBOT_ID 未设时跳过而不是拼一个
    "xbrain//cmd/task" 这样的非法 key. 后者会订到一个谁都不发的 key 上,
    表现为"客户端连上了但完全没反应" -- 与网络不通不可区分.
    """
    if not rid:
        _logger.warning("p5 cloud bridge skipped: XBRAIN_ROBOT_ID unset")
        return None
    bridge = CloudBridge(session, rid)
    bridge.wire()
    return bridge


__all__ = ["CloudBridge", "maybe_wire", "relative_key", "INBOUND_TEMPLATES",
           "CLOUD_CMD_TASK", "CLOUD_CMD_ESTOP", "CLOUD_CMD_MEDIA_SESSION",
           "CLOUD_CMD_FILE_ACK", "CLOUD_AUDIO_BROADCAST",
           "CLOUD_CMD_TASK_ACK", "CLOUD_CMD_ESTOP_ACK",
           "CLOUD_CMD_MEDIA_SESSION_ACK", "SRC_QT", "OUTBOUND_PERIODS"]
