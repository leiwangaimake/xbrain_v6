"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_bridge.py
Brief: 云端桥的行为判据 -- 一条 Qt 报文真的落到机内 key 并回出 ack

Description:
test_cloud_key_surface_wired.py 判的是[有没有 declare], 那是一切的前提;
本文件判的是 declare 之后那段 -- 报文进来以后到底去了哪.

*** 两件事分开判, 是因为它们各自都能在对方全绿时坏掉:
  接线在, 路由错  -> Qt 发的 GOTO_KEYPOINT 被网关收下了, 回了 accepted,
                     而机内 cmd/task 上什么都没出现. 机器人不动, ack 说好了.
  路由对, 接线无  -> 就是 2026-08-23 查出的那个状态: 所有转换函数都能单测
                     通过, 而总线上一条云端 key 都没有.

*** 本文件的假 session 记录[每一次 put 的 key 与字节], NO 不记调用次数.
只数次数的话, 一个把 payload 发到错误 key 上的实现照样通过 -- 而那正是
上一轮批 14-16 抓到的"只测构建器看不见总线"那类缺陷的同一个形状.

Boundaries: 不起真 Zenoh. 真机验证靠 ORIN 起栈, 那是另一回事; 这里要保证的是
逻辑上"报文到了正确的 key", 免得把可以在本机发现的错留到联调当天.
"""
from __future__ import annotations

import json

import pytest

pytestmark = pytest.mark.no_device


class _FakePub:
    def __init__(self, key, log):
        self._key = key
        self._log = log

    def put(self, payload):
        self._log.append((self._key, payload))


class _FakeSession:
    """记录声明与发布. 订阅句柄故意做成一个普通对象 -- 用例要能断言桥
    确实把它接住了(CLAUDE.md 4.3)."""

    def __init__(self):
        self.subs = {}
        self.pubs = []
        self.puts = []

    def declare_subscriber(self, key, cb):
        self.subs[key] = cb
        return object()

    def declare_publisher(self, key):
        self.pubs.append(key)
        return _FakePub(key, self.puts)


class _Sample:
    def __init__(self, key, body):
        self.key_expr = key
        self.payload = (body if isinstance(body, bytes)
                        else json.dumps(body, ensure_ascii=False)
                        .encode("utf-8"))


RID = "gj-001"


def _bridge():
    from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge

    session = _FakeSession()
    bridge = CloudBridge(session, RID)
    bridge.wire()
    return bridge, session


#: 一个合法的 v2.0 alarm_region(过 field_validate 的 _validate_regions).
#: 报警测试复用它 -- op=upsert, f- 前缀 id, >=3 顶点.
_ALARM_REGION = {"id": "f-x", "op": "upsert", "base_rev": 0, "name": "zone",
                 "type": "alarm_region", "enabled": True,
                 "applies_to": ["person"],
                 "vertices": [{"latitude": 34.697, "longitude": 135.505},
                              {"latitude": 34.698, "longitude": 135.505},
                              {"latitude": 34.698, "longitude": 135.506}]}


def _alarm_body(regions, msg_id="m-1"):
    """一份合法 v2.0 SET_ALARM_CONFIG 报文, regions 由调用方给."""
    return _qt_task(data={
        "msg_id": msg_id, "task_type": "SET_ALARM_CONFIG",
        "payload": {"alarm_level": 1, "siren_level": 70, "duration_sec": 5,
                    "cooldown_sec": 2.0,
                    "alarm_window": {"start": "22:00", "end": "05:00"},
                    "rules": [], "regions": regions}})


def _qt_task(**over):
    # 合法 v2.0 GOTO payload (审计 B-2 后 route 校验字段级约束: WGS84 大写,
    # r-/w- 前缀, arrival_radius_m 0.5..10.0). fixture 必须过校验才能测拆分.
    data = {"msg_id": "m-1", "task_id": "t-1", "task_type": "GOTO_KEYPOINT",
            "payload": {"recorded_path_id": "r-route",
                        "coordinate_system": "WGS84",
                        "waypoints": [{"id": "w-gate", "name": "gate",
                                       "latitude": 34.697, "longitude": 135.505,
                                       "altitude": 8.4,
                                       "arrival_radius_m": 3.0}]}}
    data.update(over.pop("data", {}))
    body = {"v": 1, "rid": RID, "ts": 1785732000.5, "seq": 1,
            "src": "qt_hmi", "data": data}
    body.update(over)
    return body


def _feed(session, key_suffix, body):
    """把一条报文喂给对应回调, 就像 Zenoh 那样."""
    key = "xbrain/%s/%s" % (RID, key_suffix)
    session.subs[key](_Sample(key, body))


def _puts_to(session, key_suffix):
    key = "xbrain/%s/%s" % (RID, key_suffix)
    return [json.loads(p.decode("utf-8")) for k, p in session.puts if k == key]


def _internal_puts(session, key):
    return [json.loads(p.decode("utf-8")) for k, p in session.puts if k == key]


def _feed_internal_ack(session, ack_key, cmd_id, result, code="OK",
                       message="", detail=None):
    """模拟 p3 在机内 ack_key(cmd/task/ack 或 cmd/geo/ack)上回一条业务 ack.

    这是 A-1 承接链路的另一半: 桥转发命令后, p3 处理并在机内 ack key 上回
    结果, 桥订阅它, 翻译成 v2.0 发到云端. ack_key 是相对 key(p3 在本机发).
    """
    body = {"schema": "task_ack_v1", "cmd_id": cmd_id,
            "result": result, "code": code}
    if message:
        body["message"] = message
    if detail is not None:
        body["detail"] = detail
    session.subs[ack_key](_Sample(ack_key, body))


# --- 接线本身 ---------------------------------------------------------

def test_five_inbound_and_three_ack_keys_are_declared():
    """基线. 没有它, 下面每条"报文没到"的断言都可能只是喂错了 key."""
    _bridge_, session = _bridge()

    # 5 条云端入站 + 2 条机内 ack(承接 p3 业务 ack, A-1) + 1 条机内 state/fence
    # (D: 确认 SET_ALARM_CONFIG 生效, 追 active.rev). 后三条是相对 key.
    assert sorted(session.subs) == sorted([
        "xbrain/gj-001/audio/broadcast",
        "xbrain/gj-001/cmd/estop",
        "xbrain/gj-001/cmd/file/ack",
        "xbrain/gj-001/cmd/media/session",
        "xbrain/gj-001/cmd/task",
        "cmd/task/ack", "cmd/geo/ack", "state/fence"])
    # 三条 ack + 八条出站状态面.
    # * state/link 起初以为"main_wiring 里已有发布者", 那是看错了: 那条
    # 发的是机内相对 key, 而 Qt 订的是带 rid 前缀的. 两条 key 都要有,
    # 形状还不一样(机内 11 S4.6 一大堆字段, 云端 v2.0 S4.1 只四个).
    # event/** 不在这里 -- 它按 (sev, cat) 组合按需建, 见 publish_event.
    assert sorted(session.pubs) == sorted([
        "xbrain/gj-001/cmd/estop/ack",
        "xbrain/gj-001/cmd/media/session/ack",
        "xbrain/gj-001/cmd/task/ack",
        "xbrain/gj-001/data/file/index",
        "xbrain/gj-001/state/audio",
        "xbrain/gj-001/state/geo/manifest",
        "xbrain/gj-001/state/media",
        "xbrain/gj-001/state/mode",
        "xbrain/gj-001/state/robot",
        "xbrain/gj-001/state/task",
        "xbrain/gj-001/state/link"])


def test_subscriber_handles_are_held():
    """*** zenoh-python 头号陷阱(CLAUDE.md 4.3).

    declare_subscriber 的返回值被 GC 后, Rust 端订阅悄悄注销 -- 没有异常,
    没有日志, 从此收不到报文. 与"客户还没连上来"完全不可区分.

    MUTATION: 把 wire() 里的 self._subs.append(...) 换成裸调用 -> 这里红.
    """
    bridge, _session = _bridge()

    assert bridge.alive() == 8, (
        "桥只接住了 %d 个订阅句柄, 其余会被 GC 掉(5 云端入站 + 2 机内 ack + "
        "1 state/fence)" % bridge.alive())


# --- 一条合法任务走完全程 ---------------------------------------------

def test_a_goto_lands_on_the_internal_key_in_the_s7_2_shape():
    """*** 本文件最重要的一条.

    Qt 的 GOTO_KEYPOINT 必须以 11 S7.2 的形状出现在[机内相对 key] cmd/task
    上 -- 也就是 p3_task 今天就在订的那一条, 语音下发走的也是它.
    形状一致是"不影响语音链路"的全部依据: p3 收到的东西与今天完全一样.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())

    internal = _internal_puts(session, "cmd/task")
    assert len(internal) == 1, "机内 cmd/task 上没有出现报文: %s" % session.puts
    p = internal[0]
    assert p["action"] == "submit"
    # A-1: 转发的 cmd_id 打 "c-" 前缀, 网关据此在机内 ack 上认出云端发起的.
    assert p["cmd_id"] == "c-m-1"        # "c-" + 云端 msg_id

    assert p["task"]["task_id"] == "t-1"
    assert p["task"]["type"] == "goto_keypoint"
    assert p["source"] == "cloud", (
        "source 不是 cloud -- 下游没法按来源做互斥与权限判定")


def test_the_internal_key_carries_no_rid_prefix():
    """*** 机内 key 必须是相对的.

    发成 xbrain/gj-001/cmd/task 的话, p3_task 一条也收不到 -- 它订的是裸
    cmd/task. 而 Zenoh 不会因此报任何错: 发布成功, 无人接收, 表现为
    "云端下发没反应", 与网络不通不可区分.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())

    internal_keys = [k for k, _ in session.puts if not k.startswith("xbrain/")]
    assert internal_keys == ["cmd/task"], (
        "机内发布用了带前缀的 key: %s" % [k for k, _ in session.puts])


def test_a_forwarded_task_gets_no_cloud_ack_until_p3_answers():
    """*** A-1 核心: 转发后[不]立即回 accepted, 等 p3 的机内 ack.

    审计头号发现: 立即回乐观 accepted 会掩盖 p3 的业务拒绝. 改成 pending --
    转发后云端还没有 ack, 桥有一条 pending 在等.

    MUTATION: 恢复转发后立即 publish_ack(accepted) -> 这里红(pending 期就有
    云端 ack 了).
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())

    assert not _puts_to(session, "cmd/task/ack"), (
        "转发后立即回了云端 ack -- 那会掩盖 p3 的业务判断")
    assert _b.pending_count() == 1, "转发的任务没有登记 pending"


def test_p3_accepted_ack_is_translated_to_the_cloud():
    """*** 承接: p3 回 accepted, 桥翻译成 v2.0 八字段发云端(S3.1)."""
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())
    # p3 处理后在机内 cmd/task/ack 回 accepted(cmd_id 复用 "c-m-1").
    _feed_internal_ack(session, "cmd/task/ack", "c-m-1", "accepted")

    acks = _puts_to(session, "cmd/task/ack")
    assert len(acks) == 1
    env = acks[0]
    assert env["src"] == "p5_gateway" and env["rid"] == RID
    d = env["data"]
    assert d["result"] == "accepted" and d["accepted"] is True
    assert d["ref_msg_id"] == "m-1" and d["task_id"] == "t-1"
    assert d["task_type"] == "GOTO_KEYPOINT"
    assert d["error_code"] == 0
    assert d["msg_id"] != d["ref_msg_id"]
    assert _b.pending_count() == 0, "翻译后 pending 没有清掉"


def test_p3_business_reject_reaches_the_cloud():
    """*** A-1 要害: p3 的业务拒绝(围栏外)必须回到云端(S1.4/S3.1).

    审计前这条永远回不去(网关乐观 accepted). 现在 p3 回 rejected+E_OUT_OF_FENCE,
    桥翻译成 v2.0 rejected+2006 发云端.

    MUTATION: _on_internal_ack 不发云端 ack -> 这里红.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())
    _feed_internal_ack(session, "cmd/task/ack", "c-m-1", "rejected",
                       code="E_OUT_OF_FENCE", message="第 4 个关键点位于围栏外")

    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "rejected" and d["accepted"] is False
    assert d["error_code"] == 2006             # E_OUT_OF_FENCE -> 2006
    assert d["detail"]["code"] == "E_OUT_OF_FENCE"
    assert d["reason"]                          # 失败必须有可读 reason


def test_p3_duplicate_reaches_the_cloud():
    """p3 的 duplicate(重复的 STOP)也要回到云端(S1.4)."""
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())
    _feed_internal_ack(session, "cmd/task/ack", "c-m-1", "duplicate")

    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "duplicate" and d["accepted"] is True
    assert d["error_code"] == 0


def test_an_internal_ack_for_a_non_cloud_command_is_ignored():
    """*** HMI/语音发起的机内 ack 不转发到云端.

    机内 cmd/task/ack 也载着 HMI(h-)与语音的答复. 桥只处理[自己转发过]的
    (在 pending 里的); HMI 的 cmd_id 不在 pending -> 丢弃.

    MUTATION: _on_internal_ack 无条件 publish(不查 pending) -> 这里红.
    """
    _b, session = _bridge()

    _feed_internal_ack(session, "cmd/task/ack", "h-hmi-1", "accepted")

    assert not _puts_to(session, "cmd/task/ack"), (
        "HMI 发起的 ack 被当成云端的转发了")


def test_audio_forwards_before_its_self_ack():
    """*** 顺序有意义(AUDIO 仍是网关自造 ack, B-1).

    GOTO/STOP/ALARM 不再自造 ack(A-1 改承接), 但 AUDIO 的 stream_id 由网关
    分配, ack 仍自造 -- 且必须先转发再回 ack: 先回 ack 再转发的话, 一次转发
    失败会留下一条 accepted 而 p2 什么都没收到.

    MUTATION: 把 _handle_audio 里的 _internal_put 与 _publish_ack 对调 ->
    这里红.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _audio("start"))

    keys = [k for k, _ in session.puts]
    assert keys.index("cmd/audio/speak") < keys.index(
        "xbrain/gj-001/cmd/task/ack"), "ack 先于转发发出: %s" % keys


# --- 拒绝路径 ---------------------------------------------------------

def test_broken_json_still_gets_an_ack():
    """v2.0 S7.3 明禁静默丢弃.

    丢掉的表现是 Qt 点了没反应 -- 操作员会重试, 重试同样被丢, 于是他认为
    机器人死了. 而日志里什么都没有(报文坏在 JSON 层, 连 msg_id 都读不出).
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", b'{"v": 1, "rid": ')

    acks = _puts_to(session, "cmd/task/ack")
    assert len(acks) == 1, "坏报文被静默丢弃了"
    d = acks[0]["data"]
    assert d["result"] == "rejected" and d["error_code"] != 0
    assert d["reason"]


def test_a_reject_still_carries_ref_msg_id_when_it_can_be_read():
    """*** 尽力而为地带上 ref_msg_id.

    信封坏了但 data.msg_id 还读得出来时(比如少了 seq 字段), ack 必须带上
    它 -- 否则 Qt 收到一条不知道在答复什么的拒绝, 只能整批重发.
    """
    _b, session = _bridge()

    body = _qt_task()
    del body["seq"]
    _feed(session, "cmd/task", body)

    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "rejected"
    assert d["ref_msg_id"] == "m-1", (
        "信封坏了就连能读出来的 msg_id 也不带了: %s" % d)


def test_a_retired_task_type_is_rejected_not_forwarded():
    """已下线的任务类型必须拦在网关, NO 不往机内转.

    转下去的话 p3_task 会用它自己的错误码拒绝, 而那个码没有 v2.0 落点 --
    Qt 收到一个它字典里没有的整数.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task",
          _qt_task(data={"task_type": "START_PATROL"}))

    assert not _internal_puts(session, "cmd/task"), "下线类型被转进了机内总线"
    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "rejected"
    assert d["detail"]["code"] == "E_TASK_UNSUPPORTED"


def test_manual_velocity_is_refused_as_a_channel_violation():
    """v2.0 S3.6: 云端连续遥控本期不开放, 回 E_CHANNEL_DENIED.

    与"没实现"分开报是有原因的: 没实现意味着以后会有, 而这条是[通道上
    不许], 客户据此就不该再等我们补.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task",
          _qt_task(data={"task_type": "MANUAL_VELOCITY"}))

    assert not _internal_puts(session, "cmd/task")
    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["detail"]["code"] == "E_CHANNEL_DENIED"


# --- 回环 -------------------------------------------------------------

def test_the_gateways_own_frames_are_ignored():
    """*** 回环防护.

    网关重建的报文若被自己再收一次并再重建, 就是无限回环 -- 而每一圈都是
    合法报文, 没有任何东西会报错, 只是总线被打满.

    MUTATION: 删掉 _handle_task 里的 is_cloud_frame 判断 -> 这里红.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task(src="p5_gateway"))

    assert not session.puts, (
        "网关处理了自己发出的报文 -- 回环: %s" % [k for k, _ in session.puts])


def test_an_hmi_frame_on_the_cloud_key_is_ignored_too():
    """只有 src=qt_hmi 才算云端报文.

    机内其它发布者不该借云端 key 绕过各自的权限判定 -- CH-1 的通道即权限
    是按 key 分的, 而 src 是这条 key 上的第二道闸.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task(src="hmi"))

    assert not session.puts


# --- 幂等 -------------------------------------------------------------

def test_a_repeat_is_acked_duplicate_and_forwarded_only_once():
    """*** 幂等的真判据是[机内只出现一次], 不是[ack 说 duplicate].

    只查 ack 的话, 一个"先转发再查重"的实现照样通过: 它会创建第二条任务,
    而 Qt 收到的 ack 是 duplicate -- Qt 以为什么都没发生.

    MUTATION: 把去重判断挪到 _internal_put 之后 -> 这里红.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())      # 第一次: 转发 + pending
    _feed(session, "cmd/task", _qt_task())      # 第二次: dedup 命中

    assert len(_internal_puts(session, "cmd/task")) == 1, (
        "重发的任务被转了两次 -- 会创建第二条任务")
    # 第一次转发不回 ack(等 p3, A-1); 第二次 dedup 直接回 duplicate.
    acks = _puts_to(session, "cmd/task/ack")
    assert [a["data"]["result"] for a in acks] == ["duplicate"], (
        "第一次转发就回了 ack, 或 dedup 没生效: %s"
        % [a["data"]["result"] for a in acks])


def test_a_different_msg_id_is_not_deduped():
    """反向. 没有这条, 一个"永远判重复"的实现能让上一条通过 -- 而那个
    实现会让第二条真指令永远发不出去."""
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())
    _feed(session, "cmd/task",
          _qt_task(data={"msg_id": "m-2", "task_id": "t-2"}))

    assert len(_internal_puts(session, "cmd/task")) == 2


def test_estop_is_never_deduped():
    """*** 急停不做幂等抑制, 与 cmd/task 相反.

    两个方向的代价不对称: 重复的 GOTO 会创建第二条任务(有害), 重复的急停
    只是再停一次(无害); 而一条被窗口吃掉的急停就是[没停] -- 用户按了两次
    而第二次没有生效.

    MUTATION: 给 _on_cloud_estop 加上 dedup 判断 -> 这里红.
    """
    _b, session = _bridge()

    estop = {"v": 1, "rid": RID, "ts": 1785732000.5, "seq": 1, "src": "qt_hmi",
             "data": {"msg_id": "e-1", "task_id": "", "task_type": "ESTOP",
                      "payload": {"action": "stop"}}}
    _feed(session, "cmd/estop", estop)
    _feed(session, "cmd/estop", estop)

    assert len(_internal_puts(session, "cmd/estop")) == 2, (
        "第二次急停被去重窗口吃掉了")


def test_estop_reaches_the_internal_key_in_the_hmi_shape():
    """机内 cmd/estop 的形状必须与 HMI 按钮发的一致.

    不一致的话下游要按来源分辨两种形状 -- 而急停链路上多一个分支就是多一处
    可能走错的地方(CRL-1 只搬运不判断).
    """
    _b, session = _bridge()

    _feed(session, "cmd/estop",
          {"v": 1, "rid": RID, "ts": 1.0, "seq": 1, "src": "qt_hmi",
           "data": {"msg_id": "e-1", "task_id": "", "task_type": "ESTOP",
                    "payload": {"action": "stop"}}})

    p = _internal_puts(session, "cmd/estop")[0]
    assert p["type"] == "estop" and p["action"] == "stop"
    assert p["origin"] == "cloud"
    d = _puts_to(session, "cmd/estop/ack")[0]["data"]
    assert d["result"] == "accepted" and d["task_type"] == "ESTOP"


# --- 尚未建成的下游 ---------------------------------------------------

def test_media_session_is_refused_honestly():
    """*** 如实拒绝, NO 不回假的 accepted.

    回 accepted 会让 Qt 去连一个不存在的端点, 表现成网络故障 -- 客户会去
    查他们自己的网络. 一条 E_NOT_IMPLEMENTED 让联调当天一眼看出是哪一侧
    没做完.
    """
    _b, session = _bridge()

    _feed(session, "cmd/media/session",
          {"v": 1, "rid": RID, "ts": 1.0, "seq": 1, "src": "qt_hmi",
           "data": {"msg_id": "s-1", "task_type": "MEDIA_SESSION",
                    "payload": {"action": "start"}}})

    d = _puts_to(session, "cmd/media/session/ack")[0]["data"]
    assert d["result"] == "rejected"
    assert d["detail"]["code"] == "E_NOT_IMPLEMENTED"
    assert d["ref_msg_id"] == "s-1"


def test_file_ack_and_audio_frames_produce_no_ack():
    """*** 这两条不回 ack, 且这是有意的.

    cmd/file/ack 本身就是一条 ack -- 回一条 ack 的 ack 会在两侧之间形成
    来回. audio/broadcast 是连续帧流 -- 逐帧回 ack 会把 ack 的量做到与
    音频帧一样多, 在 Q3 上挤掉别的东西.
    """
    _b, session = _bridge()

    _feed(session, "cmd/file/ack",
          {"v": 1, "rid": RID, "ts": 1.0, "seq": 1, "src": "qt_hmi",
           "data": {"msg_id": "f-1"}})
    _feed(session, "audio/broadcast", b"\x00\x01\x02\x03")

    assert not session.puts, "这两条 key 回了东西: %s" % session.puts


# --- 健壮性 -----------------------------------------------------------

def test_a_crashing_callback_does_not_escape():
    """*** Zenoh 回调抛出的异常在 Rust 侧被吞掉.

    表现是"这条报文没了"而不是任何报错, 后续报文照收 -- 于是缺陷以
    "偶发丢指令"的形式出现, 极难定位. 所以每个回调自己兜底.
    """
    from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge

    session = _FakeSession()
    bridge = CloudBridge(session, RID)
    bridge.wire()

    class _Exploding:
        key_expr = "xbrain/gj-001/cmd/task"

        @property
        def payload(self):
            raise RuntimeError("payload read blew up")

    # 不抛即通过. 抛了的话真机上就是静默丢报文.
    session.subs["xbrain/gj-001/cmd/task"](_Exploding())


def test_a_bridge_without_a_rid_refuses_to_build():
    """rid 缺失时宁可不建桥.

    建了的话 key 是 "xbrain//cmd/task" -- 订到一个谁都不发的 key 上,
    表现为"客户端连上了但完全没反应", 与网络不通不可区分.
    """
    from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge, maybe_wire

    with pytest.raises(ValueError):
        CloudBridge(_FakeSession(), "")
    assert maybe_wire(_FakeSession(), "") is None


# --- 与本地链路的互不干扰 ---------------------------------------------

def test_cloud_uses_only_existing_internal_keys():
    """*** 云端不得自造机内 key.

    自造一条 cloud/task 之类的 key, 就等于给云端指令开了一条绕过既有仲裁
    与互斥的旁路 -- 本地 MIC / HMI / 将来的微信都在既有那几条上排队, 而
    云端从旁边插进去. 用户 2026-08-24 明令四种输入形式共用一套执行链路.
    """
    from xbrain.p5_gateway.inbound.task_router import (KEY_AUDIO, KEY_ESTOP,
                                                       KEY_GEO, KEY_TASK)

    # 这四条都是 11 S2.2 既有的机内 key, 语音与 HMI 今天就在用.
    assert {KEY_TASK, KEY_ESTOP, KEY_GEO, KEY_AUDIO} == {
        "cmd/task", "cmd/estop", "cmd/geo", "cmd/audio/speak"}


def test_a_frame_claiming_another_origin_is_still_stamped_cloud():
    """*** CH-1 通道即权限, 直击.

    上一版只断言正常报文的 source 是 cloud -- 那测不到"从报文里取 origin"
    这个实现改动, 因为正常报文里根本没有 origin 字段, 兜底值恰好也是 cloud.
    变异体实测没红才发现. 要判它, 必须[真的发起那次攻击].

    一个自称 origin="voice" 的云端报文若被信了, 就绕过了云端通道该有的全部
    限制 -- 而语音通道在本机是有物理前提的(人在现场按了唤醒词), 云端没有.

    MUTATION: 把 _goto 的 source 改成 payload.get("origin", CLOUD_ORIGIN)
    -> 这里红.
    """
    _b, session = _bridge()

    body = _qt_task()
    body["data"]["payload"]["origin"] = "voice"     # 伪装
    body["data"]["origin"] = "voice"                # 换个位置再试
    body["origin"] = "voice"                        # 信封层也试
    _feed(session, "cmd/task", body)

    p = _internal_puts(session, "cmd/task")[0]
    assert p["source"] == "cloud", (
        "报文自称的 origin 被采信了: source=%r -- 通道即权限被绕过"
        % p["source"])


def test_cloud_origin_is_in_the_existing_closed_set():
    """origin 必须是 11 S7.9.5 闭集里的值.

    闭集是 cloud|wecom|hmi|voice -- 四种输入形式各一个. 云端用 "cloud",
    NO 不新造一个 "qt" 之类的值: 下游按 origin 做权限与互斥判定, 一个闭集
    外的值会被 CLAUDE.md 3.5 的越界必抛拦下, 表现为云端指令全部失败.
    """
    from xbrain.p5_gateway.inbound.task_router import CLOUD_ORIGIN

    assert CLOUD_ORIGIN == "cloud"


# --- 桥必须被启动路径真的调用 -----------------------------------------

def test_main_wiring_actually_builds_the_bridge():
    """*** 同一个陷阱升了一层, 所以单列一条.

    2026-08-23 查出的是"登记表对, 接线无". 现在接线写好了, 下一个同形的
    坑是[桥写好了, 启动路径不调它] -- 上面 23 条会全绿, 而真机上云端一条
    报文都收不到. 用例全绿 + 真机没反应, 正是最难查的组合.

    判据用 AST: 找 main_wiring 里对 maybe_wire 的调用. NO 不 grep 字符串 --
    模块头的注释里就写着 cloud_wiring.py, grep 会命中注释, 于是判据在删掉
    调用后仍然绿(CLAUDE.md 3.2 判据自伤的近亲: 判据扫到了不该扫的东西).

    MUTATION: 注释掉 main_wiring 里那行 maybe_wire(...) -> 这里红.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p5_gateway" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    calls = [n for n in ast.walk(ast.parse(src))
             if isinstance(n, ast.Call)
             and getattr(n.func, "id", "") == "maybe_wire"]
    assert len(calls) == 1, (
        "main_wiring 里对 maybe_wire 的调用有 %d 处 -- 桥没有被启动路径调用, "
        "或者被调了两次(两座桥会各订一份, 每条报文处理两遍)" % len(calls))


def test_the_cloud_face_does_not_subscribe_a_command_key_p3_owns():
    """*** 云端面不得抢机内[命令]key 的订阅.

    网关若订了裸 cmd/task/cmd/geo(p3 订阅的命令 key), 它会收到 p4_agent 发的
    语音任务并当成云端报文再处理一遍 -- 语音任务被执行两次.

    * A-1 后桥[确实]订了两条机内相对 key: cmd/task/ack 与 cmd/geo/ack. 但那是
    [承接 p3 发布的 ack], 不是抢 p3 的命令订阅 -- p3 发布 ack, 桥消费 ack,
    方向相反, 不会重复执行任何命令. 所以判据精确到: 桥订的相对 key 只能是
    [ack key], 命令 key(cmd/task, cmd/geo, cmd/audio/speak, cmd/estop 裸形)
    一条都不能有.
    """
    _b, session = _bridge()

    relative = [k for k in session.subs if not k.startswith("xbrain/")]
    # 命令 key = cmd/* 且非 ack. state/fence 是[状态]key(p1 发, 桥消费, 不会重复
    # 执行命令), 不算命令 key -- D 订它确认 SET_ALARM_CONFIG 生效.
    command_keys = [k for k in relative
                    if k.startswith("cmd/") and not k.endswith("/ack")]
    assert not command_keys, (
        "云端桥订了机内命令 key: %s -- 语音任务会被处理两遍" % command_keys)
    # 订的相对 key: 两条 ack(承接)+ 一条 state/fence(D 确认生效), 不多不少.
    assert sorted(relative) == ["cmd/geo/ack", "cmd/task/ack", "state/fence"], (
        relative)


# --- 出站状态面 -------------------------------------------------------

def test_publish_state_wraps_the_payload_in_the_six_field_envelope():
    """投影产出的 data 必须被包进 v2.0 S1.1 的六字段信封.

    裸发 data 的话 Qt 拿不到 rid/seq/ts, 也就没法做去重与乱序判定 --
    而它对可靠面(event/data)是按业务 ID 去重的, 对状态面靠 seq.
    """
    bridge, session = _bridge()

    bridge.publish_state("state/mode", {"voice_mode": "normal"})

    env = _puts_to(session, "state/mode")[0]
    assert set(env) == {"v", "rid", "ts", "seq", "src", "data"}
    assert env["src"] == "p5_gateway" and env["rid"] == RID
    assert env["data"]["voice_mode"] == "normal"


def test_seq_is_per_key_not_global():
    """*** seq 按 key 独立递增.

    全局一个计数器的话, 10 Hz 的 state/robot 会把 1 Hz 的 state/mode 的
    seq 顶到很大且跳跃 -- 而 Qt 用 seq 判乱序与丢包, 看到的就是"这条 key
    一直在丢包". v2.0 S1.1 把 seq 定义在 key 维度上.

    MUTATION: 把 SeqCounter 的 slot 从 (rid, key) 改成 rid -> 这里红.
    """
    bridge, session = _bridge()

    for _ in range(3):
        bridge.publish_state("state/robot", {"robot_state": "idle"})
    bridge.publish_state("state/mode", {"voice_mode": "normal"})

    assert [e["seq"] for e in _puts_to(session, "state/robot")] == [1, 2, 3]
    assert [e["seq"] for e in _puts_to(session, "state/mode")] == [1], (
        "state/mode 的 seq 被 state/robot 顶走了")


def test_publishing_to_an_unknown_key_raises():
    """NO 不静默吞掉一个发不出去的状态.

    吞掉的表现是 Qt 上那一栏永远空着, 而日志里什么都没有 -- 联调时会被
    当成"客户端没订阅".
    """
    bridge, _session = _bridge()

    with pytest.raises(KeyError):
        bridge.publish_state("state/nonexistent", {})


def test_every_outbound_key_has_a_declared_period_or_an_explicit_none():
    """*** 每条出站 key 的节律必须写下来.

    没写的那条会以某个人当时顺手的频率发 -- 而 v2.0 S2 对每条都给了节律
    (state/robot 固定 10 Hz, state/media 每 5 s 保活). 发慢了 Qt 判超时,
    发快了挤占带宽. None 表示[事件驱动], 是一个明确的选择而不是遗漏.
    """
    from xbrain.p5_gateway.runtime.cloud_wiring import OUTBOUND_PERIODS

    _b, session = _bridge()
    state_pubs = {k.split("/", 2)[2] for k in session.pubs
                  if "/ack" not in k}

    assert state_pubs == set(OUTBOUND_PERIODS), (
        "接线里的出站状态 key %s 与节律表 %s 对不上"
        % (sorted(state_pubs), sorted(OUTBOUND_PERIODS)))
    assert OUTBOUND_PERIODS["state/robot"] == 0.1, "v2.0 S4.2 逐字固定 10 Hz"
    assert OUTBOUND_PERIODS["state/media"] == 5.0, "v2.0 S2 每 5 s 保活"


# --- 事件转发 ---------------------------------------------------------

def test_an_event_goes_out_on_the_prefixed_cloud_key():
    """*** 机内事件发在相对 event/{sev}/{cat} 上, Qt 订的是带前缀的那条.

    2026-08-24 查出: 全系统每一条事件(p2_core / p3_task / p5 自己)都只发
    在相对 key 上 -- [Qt 一条事件都收不到], 而 Zenoh 不报错. 与 state/link
    同一个病, 而事件面是断网补发与审计的依据, 丢了更要命.
    """
    bridge, session = _bridge()

    bridge.publish_event("error", "alarm", {"eid": "e-1", "title": "x"})

    keys = [k for k, _ in session.puts]
    assert keys == ["xbrain/gj-001/event/error/alarm"], keys
    env = json.loads(session.puts[0][1].decode("utf-8"))
    assert set(env) == {"v", "rid", "ts", "seq", "src", "data"}
    assert env["data"]["eid"] == "e-1"


def test_event_publishers_are_cached_per_severity_and_category():
    """按需建并缓存.

    每条事件建一个 publisher 的话, 告警风暴时会做成百次 declare -- 而
    声明在 Zenoh 侧要走一遍会话协商. 缓存后同一组合只声明一次.
    """
    bridge, session = _bridge()

    for _ in range(5):
        bridge.publish_event("error", "alarm", {"eid": "e"})
    bridge.publish_event("warn", "comm", {"eid": "e2"})

    event_pubs = [k for k in session.pubs if "/event/" in k]
    assert sorted(event_pubs) == ["xbrain/gj-001/event/error/alarm",
                                  "xbrain/gj-001/event/warn/comm"], event_pubs


def test_event_seq_is_per_category_not_shared():
    """seq 按 key 分区. Qt 对可靠面按 eid 去重, 但 seq 的连续性是它判丢包
    的依据 -- 混在一起会让每条 key 看起来一直在丢."""
    bridge, session = _bridge()

    bridge.publish_event("error", "alarm", {"eid": "a"})
    bridge.publish_event("warn", "comm", {"eid": "b"})
    bridge.publish_event("error", "alarm", {"eid": "c"})

    seqs = {}
    for k, p in session.puts:
        seqs.setdefault(k, []).append(json.loads(p.decode("utf-8"))["seq"])
    assert seqs["xbrain/gj-001/event/error/alarm"] == [1, 2]
    assert seqs["xbrain/gj-001/event/warn/comm"] == [1]


def test_main_wiring_relays_events_to_the_cloud():
    """*** 转发口有了, 事件回调要真的调它.

    第三层的守门断言, 与 cloud_projector.tick 那条同形.

    MUTATION: 注释掉 _on_event 里的 cloud_bridge.publish_event(...) -> 红.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p5_gateway" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    calls = [n for n in ast.walk(ast.parse(src))
             if isinstance(n, ast.Call)
             and getattr(n.func, "attr", "") == "publish_event"]
    assert len(calls) == 1, (
        "main_wiring 里对 publish_event 的调用有 %d 处 -- 事件不会到云端"
        % len(calls))


# --- B-1: 音频 stream_id 分配 -----------------------------------------

def _audio(action, stream_id=None, msg_id="a-1"):
    data = {"msg_id": msg_id, "task_id": "ta-1", "task_type": "AUDIO_CONTROL",
            "payload": {"mode": "pc_to_dog", "action": action}}
    if stream_id is not None:
        data["payload"]["stream_id"] = stream_id
    return {"v": 1, "rid": RID, "ts": 1.0, "seq": 1, "src": "qt_hmi",
            "data": data}


def test_audio_start_ack_carries_a_new_stream_id():
    """*** v2.0 S2.5/S3.1: start ack 必须带后端新分配的 stream_id.

    Qt 拿这个 stream_id 才能发 audio/broadcast 帧(S8). 没有它喊话发不出去.

    MUTATION: _handle_audio 里 start 不分配 stream_id -> 这里红.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _audio("start"))

    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "accepted"
    sid = d["detail"]["stream_id"]
    assert sid and sid.startswith("audio-"), "start ack 没带分配的 stream_id"
    # 机内转发也带上同一个 stream_id(p2 要按它标记这一路喊话).
    fwd = _internal_puts(session, "cmd/audio/speak")[0]
    assert fwd["stream_id"] == sid


def test_audio_start_never_carries_a_client_stream_id():
    """start 携带 stream_id 是非法的(网关分配, 不接受客户端给).

    这条在 task_router 层拒(start must not carry stream_id). 验证它到不了
    _handle_audio -- 一个 rejected ack, 不是 accepted.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _audio("start", stream_id="client-forced"))

    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "rejected"


def test_audio_exit_echoes_the_original_stream_id():
    """*** v2.0 S3.1: exit_broadcast ack 回显请求里的 stream_id, NO 不分配新的.

    分配新 id 会让 Qt 无法确认自己退的是不是刚才那一路.

    MUTATION: _handle_audio 的 exit 分支也调 _alloc_stream_id -> 这里红.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _audio("exit_broadcast", stream_id="audio-x-0007"))

    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "accepted"
    assert d["detail"]["stream_id"] == "audio-x-0007", "退出没回显原 stream_id"


def test_two_starts_get_distinct_stream_ids():
    """每次 start 分配不同的 stream_id -- 两路喊话不能撞号."""
    _b, session = _bridge()

    _feed(session, "cmd/task", _audio("start", msg_id="a-1"))
    _feed(session, "cmd/task", _audio("start", msg_id="a-2"))

    acks = _puts_to(session, "cmd/task/ack")
    s1 = acks[0]["data"]["detail"]["stream_id"]
    s2 = acks[1]["data"]["detail"]["stream_id"]
    assert s1 != s2, "两次 start 分配了同一个 stream_id"


def test_audio_stream_id_matches_the_v2_id_regex():
    """分配的 stream_id 必须匹配 v2.0 S1.2 ID 正则(audio/broadcast 帧要用它)."""
    import re

    _b, session = _bridge()
    _feed(session, "cmd/task", _audio("start"))
    sid = _puts_to(session, "cmd/task/ack")[0]["data"]["detail"]["stream_id"]
    assert re.match(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$", sid), sid


# --- A-1: pending 超时 tick --------------------------------------------

def _bridge_clock():
    """带假单调钟的桥, 用于测 pending 超时."""
    from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge

    session = _FakeSession()
    clock = {"t": 1000.0}
    bridge = CloudBridge(session, RID, now_mono=lambda: clock["t"])
    bridge.wire()
    return bridge, session, clock


def test_pending_not_expired_before_the_timeout():
    """*** tick 在超时前不清 pending, 不回 timeout ack.

    MUTATION: PENDING_ACK_TIMEOUT_S 改成 0 -> 这里红(立刻超时).
    """
    bridge, session, clock = _bridge_clock()

    _feed(session, "cmd/task", _qt_task())
    clock["t"] += 1.9                            # < 2.0s
    bridge.tick()

    assert bridge.pending_count() == 1, "还没到点就清了 pending"
    assert not _puts_to(session, "cmd/task/ack")


def test_pending_expires_and_gets_a_timeout_reject():
    """*** p3 2 秒没回, tick 清 pending 并回 rejected+timeout(v2.0 S1.4).

    NO 不静默丢弃 -- 那样 Qt 会等到 3 秒判离线.

    MUTATION: tick 里不 publish_ack -> 这里红(超时后云端没有 ack).
    """
    bridge, session, clock = _bridge_clock()

    _feed(session, "cmd/task", _qt_task())
    clock["t"] += 2.1                            # > 2.0s
    bridge.tick()

    assert bridge.pending_count() == 0, "超时后 pending 没清"
    acks = _puts_to(session, "cmd/task/ack")
    assert len(acks) == 1
    d = acks[0]["data"]
    assert d["result"] == "rejected" and d["error_code"] != 0
    assert d["ref_msg_id"] == "m-1" and d["reason"]


def test_a_p3_ack_before_timeout_cancels_the_pending():
    """p3 在超时前回了 ack, 后续 tick 不该再发 timeout ack."""
    bridge, session, clock = _bridge_clock()

    _feed(session, "cmd/task", _qt_task())
    _feed_internal_ack(session, "cmd/task/ack", "c-m-1", "accepted")
    assert bridge.pending_count() == 0
    clock["t"] += 5.0
    bridge.tick()

    # 只有 p3 的 accepted, 没有第二条 timeout ack.
    acks = _puts_to(session, "cmd/task/ack")
    assert [a["data"]["result"] for a in acks] == ["accepted"]


def test_alarm_business_ack_comes_back_on_cmd_geo_ack():
    """*** ALARM fan-out 成 cmd/geo fence upsert, p3 的 ack 在 cmd/geo/ack 上
    回来, 聚合翻译后转发. 单区域(N=1)是退化: 子 cmd_id 无 :i 后缀, 仍是 c-m-1.

    审计前 ALARM 的业务结果(版本冲突 E_GEO_CONFLICT)回不到云端.
    """
    bridge, session, _clock = _bridge_clock()

    alarm = _qt_task(data={"task_type": "SET_ALARM_CONFIG",
                           "payload": {"alarm_level": 1, "siren_level": 70,
                                       "duration_sec": 5, "cooldown_sec": 2.0,
                                       "alarm_window": {"start": "22:00",
                                                        "end": "05:00"},
                                       "rules": [],
                                       "regions": [_ALARM_REGION]}})
    _feed(session, "cmd/task", alarm)
    # 单区域 fan-out -> 一条 cmd/geo fence upsert, 子 cmd_id = c-m-1(N=1 无后缀).
    assert _internal_puts(session, "cmd/geo")[0]["type"] == "fence"
    # p3 在 cmd/geo/ack 回版本冲突.
    _feed_internal_ack(session, "cmd/geo/ack", "c-m-1", "rejected",
                       code="E_GEO_CONFLICT", message="qu yu ban ben chong tu")

    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "rejected"
    assert d["detail"]["code"] == "E_GEO_CONFLICT"


def test_alarm_fanout_two_regions_wait_for_both_then_aggregate():
    """*** 批A fan-out+聚合: 两个 alarm_region -> 2 条 cmd/geo fence upsert;
    网关必须[等两条机内 ack 都收齐]才回一条 v2.0 ack(配置是事务).

    子 cmd_id 带 :i 后缀(N>1), 全成 -> 整体 accepted.

    MUTATION: aggregate 在收齐前就回(去掉 len(acks)<expected 的等待) -> 第一
    条子 ack 到就回终态, 下面"只收一条不该回"的断言红.
    """
    bridge, session, _clock = _bridge_clock()
    r0 = dict(_ALARM_REGION, id="f-a")
    r1 = dict(_ALARM_REGION, id="f-b")
    _feed(session, "cmd/task", _alarm_body([r0, r1]))

    # 两条 fence 命令出去, 子 cmd_id 带 :i 后缀.
    geo = _internal_puts(session, "cmd/geo")
    assert [g["cmd_id"] for g in geo] == ["c-m-1:0", "c-m-1:1"], (
        "fan-out 没产出两条带 :i 后缀的子命令: %s" % [g["cmd_id"] for g in geo])
    assert [g["geo_id"] for g in geo] == ["f-a", "f-b"]

    # 只回一条子 ack -> 还没收齐, 不能回 v2.0 终态.
    _feed_internal_ack(session, "cmd/geo/ack", "c-m-1:0", "accepted")
    assert not _puts_to(session, "cmd/task/ack"), (
        "只收到一条子 ack 网关就回了终态 -- 配置事务被拆成了半截")

    # 第二条到 -> 收齐 -> 聚合成一条 accepted.
    _feed_internal_ack(session, "cmd/geo/ack", "c-m-1:1", "accepted")
    acks = _puts_to(session, "cmd/task/ack")
    assert len(acks) == 1, "收齐后应恰好回一条 v2.0 ack, 实际 %d" % len(acks)
    assert acks[0]["data"]["result"] == "accepted"


def test_alarm_fanout_any_region_rejected_fails_the_whole():
    """*** 一票否决: 两区域里一条被拒 -> 整体 rejected(带那条的 code).

    配置事务: 一半区域写进去一半没写对操作员更难处理, 宁可整体失败让他重发.

    MUTATION: aggregate 改成"全部收齐即 accepted"(忽略失败) -> 这里 result
    仍是 accepted, 红.
    """
    bridge, session, _clock = _bridge_clock()
    r0 = dict(_ALARM_REGION, id="f-a")
    r1 = dict(_ALARM_REGION, id="f-b")
    _feed(session, "cmd/task", _alarm_body([r0, r1]))

    _feed_internal_ack(session, "cmd/geo/ack", "c-m-1:0", "accepted")
    _feed_internal_ack(session, "cmd/geo/ack", "c-m-1:1", "rejected",
                       code="E_GEO_CONFLICT", message="ban ben chong tu")

    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "rejected", "一条区域被拒, 整体却没判失败"
    assert d["detail"]["code"] == "E_GEO_CONFLICT"


def test_main_wiring_ticks_the_cloud_bridge_for_pending_timeout():
    """*** 守接线: pending 超时清理靠 main_wiring 主循环调 cloud_bridge.tick.

    tick 写好了而主循环不调它, pending 会永远积着, 超时 ack 永远不发 --
    p3 挂掉时云端无限等. AST 查 main_wiring 里对 cloud_bridge.tick 的调用.

    MUTATION: 注释掉主循环里的 cloud_bridge.tick() -> 这里红.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p5_gateway" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    ticks = [n for n in ast.walk(ast.parse(src))
             if isinstance(n, ast.Call)
             and getattr(n.func, "attr", "") == "tick"
             and getattr(getattr(n.func, "value", None), "id", "")
             == "cloud_bridge"]
    assert len(ticks) == 1, (
        "main_wiring 里对 cloud_bridge.tick 的调用有 %d 处 -- pending 超时"
        "没有人清" % len(ticks))


# --- E-1: 拒绝产生可靠审计 event (审计第三轮) --------------------------

def _events_on(session, sev, cat):
    key = "xbrain/%s/event/%s/%s" % (RID, sev, cat)
    return [json.loads(p.decode("utf-8")) for k, p in session.puts if k == key]


def test_a_structural_reject_produces_a_task_event():
    """*** v2.0 S10: 每次任务拒绝必须产生一条可靠 event/warn/task.

    网关自己产生的拒绝(字段非法)p3 看不到 -- 不发 event 就断网后无审计.

    MUTATION: _reject_task 里删掉 _emit_task_reject_event 调用 -> 这里红.
    """
    _b, session = _bridge()

    # coordinate_system 非法 -> route/field_validate 拒(网关侧结构拒绝).
    bad = _qt_task()
    bad["data"]["payload"]["coordinate_system"] = "GCJ02"
    _feed(session, "cmd/task", bad)

    # 回了 rejected ack.
    d = _puts_to(session, "cmd/task/ack")[0]["data"]
    assert d["result"] == "rejected"
    # 且产生了一条 event/warn/task.
    evts = _events_on(session, "warn", "task")
    assert len(evts) == 1, "结构拒绝没有产生审计 event"
    ev = evts[0]["data"]
    assert ev["sev"] == "warn" and ev["category"] == "task"
    assert ev["eid"].startswith("task-reject-")
    assert ev["source"] == "p5_gateway"


def test_a_p3_business_reject_produces_a_task_event():
    """*** 承接的 p3 业务拒绝(围栏外)也要有审计 event(E-1).

    p3 只在状态迁移时发 event, 被拒任务不进状态机 -> p3 没发, 网关补.

    MUTATION: _on_internal_ack 里删掉 reject event 分支 -> 这里红.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())
    _feed_internal_ack(session, "cmd/task/ack", "c-m-1", "rejected",
                       code="E_OUT_OF_FENCE", message="围栏外")

    evts = _events_on(session, "warn", "task")
    assert len(evts) == 1, "p3 业务拒绝没有产生审计 event"
    assert evts[0]["data"]["detail"]["error_code"] == 2006


def test_an_accepted_task_produces_no_reject_event():
    """反向: 受理的任务不产生拒绝 event(否则每条成功任务也刷一条 warn)."""
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())
    _feed_internal_ack(session, "cmd/task/ack", "c-m-1", "accepted")

    assert not _events_on(session, "warn", "task"), (
        "受理的任务也产生了拒绝 event")


def test_reject_events_get_distinct_eids():
    """多次拒绝的 event eid 不撞(可靠去重靠 eid)."""
    _b, session = _bridge()

    for mid in ("x1", "x2"):
        bad = _qt_task(data={"msg_id": mid, "task_id": "t",
                             "task_type": "NOPE_TYPE", "payload": {}})
        _feed(session, "cmd/task", bad)

    evts = _events_on(session, "warn", "task")
    eids = [e["data"]["eid"] for e in evts]
    assert len(eids) == 2 and len(set(eids)) == 2, "拒绝 event 的 eid 撞了"


# --- E-2: 协议错误 -> system, 任务拒绝 -> task (审计第三轮) ------------

def test_a_protocol_error_produces_a_system_event_not_task():
    """*** v2.0 S9.1/S2.6: 协议错误(信封坏/rid 不符)记 event/warn/system.

    协议层拒绝与任务层拒绝是两类, Qt 按 category 分流. 信封坏归到 task
    category 会让 Qt 把一条"报文格式错"当成"某任务被拒".

    MUTATION: parse_frame 拒绝路径不传 event_category=system -> 这里红.
    """
    _b, session = _bridge()

    # 坏 JSON -> parse_frame 协议层拒绝.
    _feed(session, "cmd/task", b'{"v": 1, "rid": ')

    # 回了 rejected ack.
    assert _puts_to(session, "cmd/task/ack")[0]["data"]["result"] == "rejected"
    # 事件在 system, 不在 task.
    assert _events_on(session, "warn", "system"), "协议错误没进 system category"
    assert not _events_on(session, "warn", "task"), "协议错误跑到了 task category"


def test_a_field_reject_stays_in_task_category():
    """任务层拒绝(字段非法)仍归 event/warn/task."""
    _b, session = _bridge()

    bad = _qt_task()
    bad["data"]["payload"]["coordinate_system"] = "GCJ02"
    _feed(session, "cmd/task", bad)

    assert _events_on(session, "warn", "task"), "字段拒绝没进 task category"
    assert not _events_on(session, "warn", "system"), "字段拒绝跑到了 system"


def test_a_rid_mismatch_is_a_system_event():
    """rid 不一致(§9.1 协议错误) -> event/warn/system."""
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task(rid="gj-999"))  # rid 与 key 不符

    assert _events_on(session, "warn", "system"), "rid 不符没进 system category"


# --- D: SET_ALARM_CONFIG 终态(state/fence.active.rev 确认) v2.0 S3.4 --------

def _alarm_bridge():
    """带单调钟+墙钟双注入的桥(D 用: 6s 判定单调, summary 时间戳墙钟)."""
    from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge
    session = _FakeSession()
    mono = {"t": 1000.0}
    wall = {"t": 1785732000.0}
    b = CloudBridge(session, RID, now_mono=lambda: mono["t"],
                    now_wall=lambda: wall["t"])
    b.wire()
    return b, session, mono, wall


def _feed_state_fence(session, rev):
    """喂一帧 P1 的 state/fence(相对 key), active.rev=rev."""
    env = {"v": 1, "data": {"active": {"rev": rev}} if rev is not None
           else {"active": None}}
    session.subs["state/fence"](_Sample("state/fence", env))


def _accept_one_alarm(session, msg_id="m-1"):
    """下发单区域报警配置并喂 p3 accepted ack -> 聚合 accepted -> 登记待确认终态."""
    _feed(session, "cmd/task", _alarm_body([_ALARM_REGION], msg_id=msg_id))
    _feed_internal_ack(session, "cmd/geo/ack", "c-" + msg_id, "accepted")


def _task_results(session):
    """state/task 上 message_type=result 的那些 data."""
    return [p["data"] for p in _puts_to(session, "state/task")
            if p["data"].get("message_type") == "result"]


def test_alarm_done_when_fence_active_rev_advances():
    """*** D: 受理后 state/fence.active.rev 越过受理时的值 -> done 终态.

    MUTATION: _register_alarm_terminal 不登记(受理即完事) -> rev 推进也不发终态
    -> 这里无 result -> 红.
    """
    _b, session, _mono, _wall = _alarm_bridge()
    _feed_state_fence(session, 3)                   # 受理前已有 active.rev=3
    _accept_one_alarm(session)                      # rev0=3, 待确认
    assert _task_results(session) == []             # 还没推进, 无终态
    _feed_state_fence(session, 4)                   # 围栏换新版 -> 生效
    res = _task_results(session)
    assert len(res) == 1 and res[0]["state"] == "done"
    assert res[0]["task_type"] == "SET_ALARM_CONFIG"
    assert res[0]["result_code"] == 0


def test_alarm_done_when_no_prior_fence_view_then_first_rev():
    """rev0=None(受理时还没 state/fence): 收到第一帧 active.rev 即算生效 -> done."""
    _b, session, _mono, _wall = _alarm_bridge()
    _accept_one_alarm(session)                      # rev0=None
    assert _task_results(session) == []
    _feed_state_fence(session, 7)                   # 从"无视图"到"有" -> done
    assert [r["state"] for r in _task_results(session)] == ["done"]


def test_alarm_failed_on_6s_timeout():
    """*** D: 6s 内 active.rev 没推进 -> failed 终态("生效未确认"), 非零 code.

    MUTATION: tick 不清超时报警 -> 无 failed 终态 -> 红.
    """
    _b, session, mono, _wall = _alarm_bridge()
    _feed_state_fence(session, 3)
    _accept_one_alarm(session)                      # rev0=3
    mono["t"] += 6.1                                # 越过 6s deadline
    _b.tick()
    res = _task_results(session)
    assert len(res) == 1 and res[0]["state"] == "failed"
    assert res[0]["result_code"] != 0 and res[0]["reason"]


def test_no_advance_no_terminal_before_timeout():
    """rev 没越过 rev0(同值再来): 不发终态(E-3 式不误报生效)."""
    _b, session, _mono, _wall = _alarm_bridge()
    _feed_state_fence(session, 3)
    _accept_one_alarm(session)
    _feed_state_fence(session, 3)                   # 同版, 没换
    assert _task_results(session) == []


def test_goto_does_not_register_an_alarm_terminal():
    """*** 只有 SET_ALARM_CONFIG 登记待确认终态; GOTO 不受影响.

    MUTATION: 登记条件去掉 task_type 判据 -> GOTO 也登记 -> rev 推进后冒出一条
    GOTO 的终态 -> 红.
    """
    _b, session, _mono, _wall = _alarm_bridge()
    _feed(session, "cmd/task", _qt_task())          # GOTO
    _feed_internal_ack(session, "cmd/task/ack", "c-m-1", "accepted")
    _feed_state_fence(session, 5)
    assert _task_results(session) == [], "GOTO 不该产生 state/fence 驱动的终态"


def test_rejected_alarm_does_not_wait_for_effect():
    """被拒的报警配置不登记待确认 -- 没受理就没有生效可等."""
    _b, session, _mono, _wall = _alarm_bridge()
    _feed(session, "cmd/task", _alarm_body([_ALARM_REGION]))
    _feed_internal_ack(session, "cmd/geo/ack", "c-m-1", "rejected",
                       code="E_GEO_CONFLICT", message="ban ben chong tu")
    _feed_state_fence(session, 9)
    assert _task_results(session) == []
