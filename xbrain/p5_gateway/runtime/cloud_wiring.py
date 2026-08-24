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

from ...common import errors
from ..inbound.cloud_inbound import (InboundReject, SRC_QT, frame_ids,
                                     is_cloud_frame, parse_frame, rid_from_key)
from ..inbound.task_router import (CLOUD_ORIGIN, KEY_AUDIO, KEY_GEO,
                                    KEY_TASK, route)
from ..outbound.ack_translate import translate_ack
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

#: 云端发起的机内命令的 cmd_id 前缀(审计 A-1). 像 HMI 的 "h-", 网关用它在
#: 机内 ack 上认出"这条是回应云端的", 与 HMI/语音发起的分开.
CLOUD_CMD_PREFIX = "c-"

#: pending ack 的超时(v2.0 S1.4: 2 秒内必回 ack). p3 正常 ms 级回, 这个超时
#: 只兜 p3 挂了的情形 -- 到点还没收到机内 ack 就回一条 rejected+timeout.
PENDING_ACK_TIMEOUT_S = 2.0

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
                 dedup: Optional[DedupWindow] = None,
                 now_mono: Optional[Callable[[], float]] = None) -> None:
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
        # AUDIO_CONTROL start 的 stream_id 由网关分配(审计 B-1). 按 rid 计数,
        # 从 1 起. 见 _handle_audio.
        self._audio_seq = 0
        # 拒绝审计事件的 eid 源(审计 E-1). v2.0 S10: 每次任务拒绝必须产生
        # 一条可靠 event/{sev}/task. boot token 让 eid 跨网关重启不撞
        # (record.db 持久化, 重启后 seq 从 0 但 boot 不同).
        self._reject_boot = uuid.uuid4().hex[:6]
        self._reject_seq = 0
        # A-1 承接: 转发给 p3 的 cmd/task/geo 登记在这, 等 p3 的机内 ack 回来
        # 翻译转发到云端. cmd_id("c-"+msg_id) -> (msg_id, task_id, task_type,
        # mono_ms). 见 _handle_task / _on_internal_ack / tick.
        self._pending: Dict[str, tuple] = {}
        self._now_mono = now_mono or time.monotonic
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
        # A-1: 承接 p3 的机内业务 ack. GOTO/STOP 走 cmd/task -> cmd/task/ack;
        # ALARM 走 cmd/geo -> cmd/geo/ack. 两条都是[相对]机内 key(不带 rid
        # 前缀), p3 在本机发. 收到后按 "c-" 前缀认出云端的, 翻译成 v2.0 回云端.
        self._subs.append(self._session.declare_subscriber(
            "cmd/task/ack", self._on_internal_ack))
        self._subs.append(self._session.declare_subscriber(
            "cmd/geo/ack", self._on_internal_ack))
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

        # *** AUDIO_CONTROL 的 ack 由网关自造 + 分配 stream_id(审计 B-1).
        # 它走 cmd/audio/speak(p2 speaker), 而 p2 speaker 是语音 TTS 通道,
        # NO 不产生 v2.0 的 stream_id -- 那是云端喊话协议的字段, 只有网关
        # (云端翻译点)能给. start ack 必须带新分配的 stream_id(v2.0 S2.5/
        # S3.1), 否则 Qt 拿不到它就发不了 audio/broadcast 帧(S8).
        if internal_key == KEY_AUDIO:
            self._handle_audio(msg_id, task_id, task_type, payload)
            return

        # *** A-1: GOTO/STOP/ALARM 不回乐观 accepted, 而是[登记 pending +
        # 转发], 等 p3 的机内 ack 回来翻译成 v2.0 回云端.
        # 审计头号发现: 立即回 accepted 会掩盖 p3 的业务拒绝(E_NOT_FOUND /
        # E_OUT_OF_FENCE / E_BUSY / duplicate), 而 v2.0 S1.4 逐字要求 ack
        # "包括结构拒绝, 业务拒绝和 duplicate".
        # 机内 payload 的 cmd_id 打上 "c-" 前缀, p3 回 ack 时复用它, 网关
        # 据此在机内 cmd/task/ack 上认出云端发起的那些.
        internal_cmd_id = CLOUD_CMD_PREFIX + (msg_id or "")
        payload["cmd_id"] = internal_cmd_id
        self._pending[internal_cmd_id] = (
            msg_id or "", task_id or "", task_type or "", self._now_mono())
        self._internal_put(internal_key, json.dumps(
            payload, ensure_ascii=False).encode("utf-8"))
        _logger.info("p5 cloud task %s -> %s (origin=%s, pending ack)",
                     task_type, internal_key, CLOUD_ORIGIN)

    def _on_internal_ack(self, sample: Any) -> None:
        """机内 cmd/task/ack 或 cmd/geo/ack 到达: 若是云端发起的, 翻译回云端.

        RUST 线程(CLAUDE.md 4.2): 纯解码 + 查表 + 一次 put. NO 不 await.

        *** 靠 "c-" 前缀 + pending 表[双重]认云端的.
        这条机内 ack 也载着 HMI(h-)与语音发起的答复. 前缀先粗筛, pending 表
        再精确匹配 -- 一条 "c-" 前缀但不在 pending 里的(比如网关重启后 p3
        迟到的 ack)不会被误发, 因为 pending 里没有它.
        """
        try:
            raw, _key = _sample_parts(sample)
            body = json.loads(raw.decode("utf-8"))
            if not isinstance(body, dict):
                return
            cmd_id = body.get("cmd_id")
            if not isinstance(cmd_id, str):
                return
            # *** pending 表是唯一且充分的门, NO 不另加前缀检查.
            # 机内 cmd/task/ack 也载着 HMI(h-)与语音发起的答复, 但只有网关
            # 自己转发过的("c-"+msg_id)才在 pending 里 -- 查不到就丢弃. 这
            # 一条 O(1) 查找同时完成了[是不是云端的]与[是否已超时清理]两件事:
            #   HMI/语音的 cmd_id      -> 不在 pending -> 丢
            #   网关重启后的迟到 ack   -> 不在 pending -> 丢(请求早已没人等)
            # "c-" 前缀仍在转发时打上, 那是给 main_wiring 的 HMI 侧
            # (_on_uplink_ack 查 "h-")区分用的, 不是本处的过滤依据.
            entry = self._pending.pop(cmd_id, None)
            if entry is None:
                return
            msg_id, task_id, task_type, _mono = entry
            v2_ack = translate_ack(
                body, ref_msg_id=msg_id, task_id=task_id, task_type=task_type,
                new_msg_id=_new_msg_id())
            self._publish_ack("cmd/task/ack", v2_ack)
            self.stats["accepted" if v2_ack["accepted"] else "rejected"] += 1
            # E-1: 承接的 p3 业务拒绝也要有审计 event(p3 只在状态迁移时发
            # event, 被拒任务不进状态机 -> p3 没发, 网关补).
            if not v2_ack["accepted"] and v2_ack["result"] == "rejected":
                self._emit_task_reject_event(
                    ref_msg_id=msg_id, task_id=task_id, task_type=task_type,
                    error_code=v2_ack["error_code"],
                    detail_code=v2_ack["detail"].get("code", ""),
                    reason=v2_ack["reason"])
        except Exception:                        # noqa: BLE001
            _logger.exception("p5 cloud internal-ack handler crashed")

    def tick(self, now_mono_ms: int = None) -> None:
        """由网关主循环周期调用: 清理超时的 pending, 回一条 timeout ack.

        v2.0 S1.4: 2 秒内必回 ack. p3 正常 ms 级回, 到点还没回说明 p3 挂了或
        极慢 -- 回一条 rejected(2002 忙/超时)让 Qt 不至于无限等; NO 不静默
        丢弃, 那样 Qt 会一直等到 3 秒判离线.
        """
        from ..outbound.error_map import build_error_fields

        now = self._now_mono()
        expired = [cid for cid, (_m, _t, _tt, mono) in self._pending.items()
                   if now - mono >= PENDING_ACK_TIMEOUT_S]
        for cid in expired:
            msg_id, task_id, task_type, _mono = self._pending.pop(cid)
            fields = build_error_fields(
                errors.E_TIMEOUT,
                "backend did not answer within %.0fs" % PENDING_ACK_TIMEOUT_S)
            self._publish_ack("cmd/task/ack", build_ack(
                msg_id=_new_msg_id(), ref_msg_id=msg_id, task_id=task_id,
                task_type=task_type, result=RESULT_REJECTED,
                error_code=fields["error_code"], reason=fields["reason"],
                detail=fields["detail"]))
            self.stats["rejected"] += 1
            _logger.warning("p5 cloud task %s timed out (no p3 ack in %.0fs)",
                            task_type, PENDING_ACK_TIMEOUT_S)

    def pending_count(self) -> int:
        """在途 pending 数. 只为可观测/测试."""
        return len(self._pending)

    def _alloc_stream_id(self) -> str:
        """分配一个新 broadcast stream_id(v2.0 S2.5: 后端在 start 分配).

        格式 audio-{rid}-{seq:04d}, 匹配 v2.0 S1.2 ID 正则. 按 rid 计数从 1
        起 -- 同一台车的会话号连续, 便于联调时对上是第几次喊话.
        """
        self._audio_seq += 1
        return "audio-%s-%04d" % (self._rid, self._audio_seq)

    def _handle_audio(self, msg_id: Optional[str], task_id: Optional[str],
                      task_type: Optional[str], payload: Dict[str, Any]) -> None:
        """AUDIO_CONTROL 的转发 + 自造 ack(带 stream_id, 审计 B-1).

        start          分配 stream_id -> ack.detail.stream_id -> 转发带该 id
        exit_broadcast 回显请求里的 stream_id -> ack.detail.stream_id(同一个)
                       (v2.0 S3.1: 退出 ack 回显原 id, NO 不分配新 id)
        """
        action = payload.get("action")
        if action == "start":
            stream_id = self._alloc_stream_id()
            payload["stream_id"] = stream_id     # 转发给 p2 时带上
        else:                                    # exit_broadcast: 回显原 id
            stream_id = payload.get("stream_id")
        # 先转发再回 ack(同 _handle_task 的顺序理由: 转发失败不留下假 accepted).
        self._internal_put(KEY_AUDIO, json.dumps(
            payload, ensure_ascii=False).encode("utf-8"))
        self._publish_ack("cmd/task/ack", build_ack(
            msg_id=_new_msg_id(), ref_msg_id=msg_id or "",
            task_id=task_id or "", task_type=task_type or "AUDIO_CONTROL",
            result=RESULT_ACCEPTED, detail={"stream_id": stream_id}))
        self.stats["accepted"] += 1
        _logger.info("p5 cloud audio %s stream_id=%s", action, stream_id)

    def _emit_task_reject_event(self, *, ref_msg_id: Optional[str],
                                task_id: Optional[str],
                                task_type: Optional[str],
                                error_code: int, detail_code: str,
                                reason: str) -> None:
        """一次云端 cmd/task 拒绝 -> 一条可靠 event/warn/task(审计 E-1).

        *** v2.0 S10 逐字: 每次任务拒绝必须产生一条可靠 event/{sev}/task,
        保证断网后可审计. 网关自己产生的拒绝(结构/路由/字段)p3 根本看不到,
        p3 的业务拒绝也只在[状态迁移]时发 event, 而被拒的任务不进状态机 --
        两处都不发, 于是[没有任何拒绝有审计]. 网关是云端唯一的审计点, 在这里
        补上: 拒绝是可恢复的(操作员改了重发), 所以 sev=warn.

        source=p5_gateway: 这条 event 是网关观测到云端拒绝而产生的; 与 p3 的
        任务事件(source=p3_task)不同源, 不同 eid, 不重复.
        """
        self._reject_seq += 1
        try:
            self.publish_event("warn", "task", {
                "eid": "task-reject-%s-%d" % (self._reject_boot,
                                              self._reject_seq),
                "state": "occurred",
                "source": "p5_gateway",
                "code": detail_code or errors.E_SCHEMA,
                "title": "cloud task rejected",
                "message": reason or "",
                "task_id": task_id or None,
                "detail": {"task_type": task_type, "ref_msg_id": ref_msg_id,
                           "error_code": error_code},
            })
        except Exception:                        # noqa: BLE001
            # 审计 event 发失败不能连累 ack -- ack 已经发出去了, 这是尽力而为.
            _logger.exception("p5 cloud reject-event emit failed")

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
        # E-1: 网关自己产生的拒绝也要有审计 event(p3 看不到这条拒绝).
        _det = fields.get("detail") or {}
        self._emit_task_reject_event(
            ref_msg_id=msg_id, task_id=task_id, task_type=task_type,
            error_code=fields["error_code"], detail_code=_det.get("code", ""),
            reason=fields["reason"])

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
        # D-2: 机内事件 -> v2.0 S5.1 字段集(字段名归一, sev/category 从 key
        # 取, 补齐缺失). 直接透传机内 data 的话 Qt 找 data.sev 会找不到
        # (机内 sev/category 在 key 上不在 data 里).
        from ..outbound.state_projection import event_payload
        v2_data = event_payload(data, sev=severity, category=category)
        body = build_envelope(self._rid, key, v2_data,
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
