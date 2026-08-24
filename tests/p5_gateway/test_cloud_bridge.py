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


def _qt_task(**over):
    data = {"msg_id": "m-1", "task_id": "t-1", "task_type": "GOTO_KEYPOINT",
            "payload": {"recorded_path_id": "p-9",
                        "coordinate_system": "wgs84",
                        "waypoints": [{"lat": 34.697, "lon": 135.505}]}}
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


# --- 接线本身 ---------------------------------------------------------

def test_five_inbound_and_three_ack_keys_are_declared():
    """基线. 没有它, 下面每条"报文没到"的断言都可能只是喂错了 key."""
    _bridge_, session = _bridge()

    assert sorted(session.subs) == sorted([
        "xbrain/gj-001/audio/broadcast",
        "xbrain/gj-001/cmd/estop",
        "xbrain/gj-001/cmd/file/ack",
        "xbrain/gj-001/cmd/media/session",
        "xbrain/gj-001/cmd/task"])
    # 三条 ack + 七条出站状态面. state/link 与 event/** 不在这里 --
    # 它们在 main_wiring 里另有发布者(接手前就存在), 桥不去抢.
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
        "xbrain/gj-001/state/task"])


def test_subscriber_handles_are_held():
    """*** zenoh-python 头号陷阱(CLAUDE.md 4.3).

    declare_subscriber 的返回值被 GC 后, Rust 端订阅悄悄注销 -- 没有异常,
    没有日志, 从此收不到报文. 与"客户还没连上来"完全不可区分.

    MUTATION: 把 wire() 里的 self._subs.append(...) 换成裸调用 -> 这里红.
    """
    bridge, _session = _bridge()

    assert bridge.alive() == 5, (
        "桥只接住了 %d 个订阅句柄, 其余会被 GC 掉" % bridge.alive())


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
    assert p["cmd_id"] == "m-1"          # 幂等键 = 云端 msg_id (11 S2.3)
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


def test_an_accepted_task_gets_an_ack_on_the_cloud_key():
    """v2.0 S3.1: 每条 cmd/task 都要回 ack, 八字段齐全."""
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())

    acks = _puts_to(session, "cmd/task/ack")
    assert len(acks) == 1
    env = acks[0]
    assert env["src"] == "p5_gateway" and env["rid"] == RID
    d = env["data"]
    assert d["result"] == "accepted" and d["accepted"] is True
    assert d["ref_msg_id"] == "m-1" and d["task_id"] == "t-1"
    assert d["task_type"] == "GOTO_KEYPOINT"
    assert d["error_code"] == 0
    assert d["msg_id"] != d["ref_msg_id"], (
        "ack 的 msg_id 与被答复的那条相同 -- Qt 没法区分请求与答复")


def test_forwarding_happens_before_the_accepted_ack():
    """*** 顺序有意义, 不是风格问题.

    先回 ack 再转发的话, 一次转发失败会留下一条"已受理"的 ack 而机器人
    什么都没做 -- Qt 看到 accepted 就不会重发, 这条指令就此消失.

    MUTATION: 把 _handle_task 里的 put 与 _publish_ack 两行对调 -> 这里红.
    """
    _b, session = _bridge()

    _feed(session, "cmd/task", _qt_task())

    keys = [k for k, _ in session.puts]
    assert keys.index("cmd/task") < keys.index("xbrain/gj-001/cmd/task/ack"), (
        "ack 先于转发发出: %s" % keys)


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
    assert d["detail"]["code"] == "E_NOT_IMPLEMENTED"


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

    _feed(session, "cmd/task", _qt_task())
    _feed(session, "cmd/task", _qt_task())

    assert len(_internal_puts(session, "cmd/task")) == 1, (
        "重发的任务被转了两次 -- 会创建第二条任务")
    acks = _puts_to(session, "cmd/task/ack")
    assert [a["data"]["result"] for a in acks] == ["accepted", "duplicate"]


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


def test_the_cloud_face_declares_no_relative_key_that_p3_owns():
    """*** 云端面不得抢机内 key 的订阅.

    网关若也订了裸 cmd/task, 它会收到 p4_agent 发的语音任务并当成云端报文
    再处理一遍 -- 语音任务被执行两次. 这是"不影响语音链路"最容易破的一处.

    判据: 桥声明的订阅 key 全部带 xbrain/ 前缀, 一条相对 key 都没有.
    """
    _b, session = _bridge()

    relative = [k for k in session.subs if not k.startswith("xbrain/")]
    assert not relative, (
        "云端桥订了机内相对 key: %s -- 语音任务会被处理两遍" % relative)


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
