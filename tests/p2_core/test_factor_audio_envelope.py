"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_factor_audio_envelope.py
Brief: cmd/motion/factor 与 state/audio 必须带 11 S3.0 信封, 消费侧同批解包

Description:
S3.0 逐字"所有 Zenoh JSON 载荷共用此外层结构". 这两条 key 在 2026-09-28 之前
发的是[裸报文] -- 顶层直接是 11 S3.6 HealthFactor / 11 S8.10 AudioState 的
字段, 没有 v/rid/ts/mono/boot/seq/src/ts_sync. 与 health/summary(同日修) .
p5 的 state/link 与 event/{sev}/comm(2026-09-27 修)是同一个缺陷.

本文件分三半, 每一半都能在另外两半全绿时坏掉:
  * 发布侧 -- p2 的两个 put 真的走 stamp_body, 且两条 key 的信封 seq 各自
    递增(不是共用一个计数器);
  * 消费侧 cmd/motion/factor -- p1 nav_wiring 的 unwrap_body 两种形态都吃.
    它[本来就能解], 所以本批不改 p1; 但"不必改"必须有判据, 否则下一个人
    简化 _on_factor 时会把兼容拆掉, 而拆掉的现象是 P1 的 HealthFactorSlot
    永远停在 "never" = 零速, p2 那边每秒照发;
  * 消费侧 state/audio -- p5 的 _on_state_audio 本批新加解包. 这一半不做
    就是回归: 2026-09-27 修 event/{sev}/comm 时只改发布侧, 消费侧仍读顶层,
    引入过一次线上回归(5d6981a).

*** 为什么 state/audio 漏解包比 health/summary 更坏, 单列一条判据:
cloud_state._audio 读 a.get("speaker") 取不到 -> holder 为 None ->
speaking = (holder != "none") 判成 True -> Qt 上"喇叭一直在响". 一个 health
漏解包至少空成一张空设备表(看得出不对), 而音频这条会变成一个[看起来在工作]
的假状态 -- 正是 CLAUDE.md 3.2 形态一.
"""
from __future__ import annotations

import inspect
import json

import pytest

from xbrain.common.envelope import decode
from xbrain.p2_core.runtime.main_wiring import run_voice_loop_wiring, stamp_body

pytestmark = pytest.mark.no_device


#: 11 S3.6 HealthFactor 的一份真实体(build_health_factor 的产物形状).
_FACTOR = {
    "speed_factor": 0.5,
    "allow_motion": True,
    "max_profile": "obstacle_avoid",
    "reason": "battery_unknown",
    "detail_ref": "health/summary",
}

#: 11 S8.10 AudioState 里 p2 真正持有的那部分(build_audio_state 的产物形状).
_AUDIO = {
    "speaker": {"holder": "none", "since_mono": None},
    "mic": {"open": True, "gate_reason": "idle"},
    "devices": {"mic": "ok"},
    "voice_mode": None,
    "ts_mono": 1454352.7,
}


def _decode(raw: bytes) -> dict:
    return json.loads(raw.decode("utf-8"))


# -- 发布侧 -------------------------------------------------------------------


def test_the_factor_body_is_the_data_not_the_top_level():
    """裸报文与信封的区别就在这一条.

    MUTATION: 发布点改回 json.dumps(factor_body) -> 顶层出现 speed_factor,
    data 消失, 本条红.
    """
    env = _decode(stamp_body(_FACTOR, rid="m20s", boot="abc12345", seq=7,
                             ts_sync=False))
    assert "speed_factor" not in env
    assert decode(env).data == _FACTOR
    assert env["src"] == "p2_core"


def test_the_audio_body_is_the_data_not_the_top_level():
    env = _decode(stamp_body(_AUDIO, rid="m20s", boot="abc12345", seq=9,
                             ts_sync=True))
    assert "speaker" not in env
    assert decode(env).data == _AUDIO
    assert env["ts_sync"] is True


def test_both_keys_are_published_through_the_stamper():
    """纯函数写对而调用处还在 json.dumps(裸 dict), 是"两半都对不上"的典型
    形状 -- 所以读 p2 的真实接线源码.

    MUTATION: 任一处改回 json.dumps(<body>) -> 本条红.
    """
    src = inspect.getsource(run_voice_loop_wiring)
    assert "factor_pub.put(stamp_body(" in src
    assert "audio_state_pub.put(stamp_body(" in src
    # 裸形态不得残留: 旧写法与新写法并存意味着有一条路径还在发裸报文.
    assert "factor_pub.put(json.dumps(" not in src
    assert "audio_state_pub.put(json.dumps(" not in src


def test_each_key_carries_its_own_envelope_seq():
    """S3.0 的 seq 按 key 各自递增, 而这三条 key 的节律不同(state/audio 是
    "变更即报", 最快 10 Hz; 另两条 1 Hz).

    共用一个计数器的现象不是报错: 1 Hz 那两条在消费方看来每帧跳十几号,
    按 U18 判成"中间丢了十几帧", 于是补发游标一直在追一段不存在的空洞.
    MUTATION: 三处都传同一个计数器 -> 本条红.
    """
    src = inspect.getsource(run_voice_loop_wiring)
    assert "seq=_next_seq(CMD_MOTION_FACTOR_TOPIC)" in src
    assert "seq=_next_seq(STATE_AUDIO_TOPIC)" in src
    assert "seq=_next_seq(HEALTH_SUMMARY_TOPIC)" in src


def test_next_seq_counts_per_key_and_starts_at_one():
    """上一条只说接线传了什么; 这一条跑真的计数器.

    源码断言与行为断言各管一半: 只读源码的话, 一个恒返回 0 的 _next_seq
    照样通过.
    """
    src = inspect.getsource(run_voice_loop_wiring)
    # 闭包取不到, 所以在同一段源码语义下重建它 -- 重建体与接线体由上一条
    # 的 "_env_seq[key] = n" 判据(在 test_health_summary_envelope.py 里)
    # 钉住同源.
    assert "_env_seq[key] = n" in src

    seq: dict = {}

    def _next(key: str) -> int:
        n = seq.get(key, 0) + 1
        seq[key] = n
        return n

    assert _next("a") == 1
    assert _next("b") == 1        # 各自从 1 起, NO 不共用
    assert _next("a") == 2


# -- 消费侧 cmd/motion/factor -------------------------------------------------


def test_p1_nav_unwraps_the_factor_envelope_and_still_eats_the_bare_form():
    """P1-4 的真实消费点. unwrap_body 认信封靠 v/src/data 三个键同时在.

    桩发布者(scripts/sil/zenoh_world.py 与 scripts/dev/pose_stub.py --grant)
    仍发裸报文, 所以两种形态必须都能走 -- 各写一条代码路径必然分叉.
    """
    from xbrain.p1_motion.runtime.nav_wiring import unwrap_body

    enveloped = _decode(stamp_body(_FACTOR, rid="m20s", boot="b", seq=1,
                                   ts_sync=False))
    assert unwrap_body(enveloped) == _FACTOR
    assert unwrap_body(_FACTOR) is _FACTOR
    # 只有 data 而没有 v/src 的不是信封(例如某个发布者恰好有个叫 data 的
    # 业务字段) -- 误拆会把整条体丢掉.
    assert unwrap_body({"data": {"x": 1}}) == {"data": {"x": 1}}


def test_p1_on_factor_goes_through_unwrap_body():
    """"不必改 p1"必须有判据.

    MUTATION: 把 _on_factor 里的 unwrap_body(doc) 换成 doc -> 本条红, 且
    线上 HealthFactorSlot 会对每一帧抛 HealthFactorError 并停在 "never".
    """
    from xbrain.p1_motion.runtime import nav_wiring

    src = inspect.getsource(nav_wiring.NavRuntime._on_factor)
    assert "unwrap_body(doc)" in src


def test_the_health_factor_slot_rejects_a_frame_that_was_not_unwrapped():
    """上一条的后果, 跑出来.

    信封顶层没有 speed_factor / allow_motion / max_profile, 所以
    HealthFactorSlot.on_message 会抛 -- 帧被丢, 槽位停在 never = 零速.
    这条是"漏解包"的可观测证据, 不是重复断言.
    """
    from xbrain.p1_motion.nav.health_factor import (HealthFactorError,
                                                    HealthFactorSlot)

    enveloped = _decode(stamp_body(_FACTOR, rid="m20s", boot="b", seq=1,
                                   ts_sync=False))
    slot = HealthFactorSlot(degrade_after_ms=3000, dead_after_ms=10000)
    with pytest.raises(HealthFactorError):
        slot.on_message(enveloped, 1000)
    assert slot.view(1000).state == "never"
    # 解包之后同一帧是好的 -- 否则上面那条会退化成"这个体本来就不合法".
    slot.on_message(enveloped["data"], 1000)
    assert slot.view(1000).state == "ok"


# -- 消费侧 state/audio -------------------------------------------------------


def test_p5_on_state_audio_reads_the_body_out_of_the_envelope():
    """本批新加的那一半. 只改发布侧而这里不改, 就是 5d6981a 那次回归.

    MUTATION: 删掉解包的两行 -> 本条红.
    """
    from xbrain.p5_gateway.runtime.main_wiring import (
        run_voice_loop_wiring as p5_wiring)

    src = inspect.getsource(p5_wiring)
    block = src[src.index("def _on_state_audio("):]
    block = block[:block.index("def _on_state_pose(")]
    assert 'inner = d.get("data")' in block
    assert 'hmi_state["audio"] = d' in block


def test_an_unwrapped_audio_envelope_reads_as_a_speaker_that_never_stops():
    """为什么这条 key 的解包不是装饰 -- 把漏解包的后果跑出来.

    _audio 读 a.get("speaker") 取不到 -> holder 为 None ->
    speaking = (holder != "none") 为 True -> speaker_state = playing.
    一条 1 Hz 稳定发出的 "playing" 与真的在喊话完全不可区分, 而机器人
    是静默的(CLAUDE.md 3.2 形态一: 一条永远绿的读数).
    """
    from xbrain.p5_gateway.runtime.cloud_state import CloudProjector

    class _Bridge:
        def publish_state(self, name, data):    # pragma: no cover - 不发
            raise AssertionError("projection test must not publish")

    now = [100.0]
    proj = CloudProjector(_Bridge(), now_mono=lambda: now[0])
    stamp_ms = now[0] * 1000.0

    body_ok = proj._audio({"audio": _AUDIO, "audio_updated_ms": stamp_ms})
    assert body_ok["speaker_state"] == "idle"
    assert body_ok["playing"] is False

    enveloped = _decode(stamp_body(_AUDIO, rid="m20s", boot="b", seq=1,
                                   ts_sync=False))
    body_bad = proj._audio({"audio": enveloped, "audio_updated_ms": stamp_ms})
    assert body_bad["speaker_state"] == "playing"
    assert body_bad["playing"] is True
    # ...而麦克风同时被报成 disabled(半双工), 于是界面上"正在喊话"与
    # "麦克风被关"两格一起说谎.
    assert body_bad["microphone_state"] == "disabled"


def test_p5_on_state_audio_still_accepts_the_bare_form():
    """桩发布者与旧版本 p2 都不带信封.

    用一个最小替身跑真实的解包判据 -- 源码断言说不出"裸形态还能用".
    """
    def unwrap(d):
        if isinstance(d, dict):
            inner = d.get("data")
            if isinstance(inner, dict):
                d = inner
        return d

    assert unwrap(_AUDIO) is _AUDIO
    enveloped = _decode(stamp_body(_AUDIO, rid="m20s", boot="b", seq=1,
                                   ts_sync=False))
    assert unwrap(enveloped) == _AUDIO
    # data 不是 dict 时不得吞掉整条报文.
    assert unwrap({"speaker": {}, "data": "oops"})["speaker"] == {}
