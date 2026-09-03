"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_audio_projection.py
Brief: CloudProjector._audio -- 11 S8.10 -> v2.0 S4.4 的映射

Description:
守云端音频状态的四件事: (1) 没收到过 p2 的快照就[不发], 不是报 idle;
(2) 收到过但太旧报 fault, 不是继续报最后一帧; (3) 说话/静音/设备故障各自
落到 v2.0 的哪个闭集值; (4) 属于云端喊话链路的两个字段(stream_id /
last_frame_age_ms)在链路接通前必须为 null.

为什么值得单独立测: 2026-09-03 之前 _audio 读的是 hmi_state 根上五个
[没有任何人写]的扁平键(speaking / stream_id / speaker_holder /
speaker_holder_type / last_frame_age_ms). 五个 .get() 全落 None, 于是
speaker_state 恒 "idle" 且 microphone_state 恒 "idle" -- 一条 1 Hz 稳定
发出的 idle, 与真的空闲在报文上完全不可区分. 甲方联调时看到的是一对
永远不变的常量, 而系统"看起来完全正常".

本文件不测 p2 那侧的构造(见 tests/p2_core/test_audio_state_build.py).
"""

import pytest

from xbrain.p5_gateway.runtime.cloud_state import (AUDIO_STALE_MS,

                                                   CloudProjector)

# INF-TS-1 三档 marker. 纯函数 / 静态检查, 不碰任何硬件 -> no_device.
pytestmark = pytest.mark.no_device


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class _Bridge:
    def __init__(self):
        self.published = []

    def publish_state(self, name, data):
        self.published.append((name, data))


def _proj():
    clock = _Clock()
    return CloudProjector(_Bridge(), now_mono=clock), clock


def _audio_snap(holder="none", mic_open=False, devices=None):
    """11 S8.10 形状的一份 p2 快照."""
    return {"speaker": {"holder": holder},
            "mic": {"open": mic_open, "gate_reason": "idle"},
            "devices": devices if devices is not None else {"mic": "ok"},
            "voice_mode": None, "ts_mono": 0.0}


def _state(clock, **over):
    st = {"audio": _audio_snap(), "audio_updated_ms": clock.t * 1000.0}
    st.update(over)
    return st


# --- 没有数据 ---------------------------------------------------------

def test_no_snapshot_publishes_nothing_rather_than_a_healthy_idle():
    """*** 没收到过 p2 的 state/audio 就不发这一轮.

    报 idle 是拿"我不知道"冒充"一切正常". Qt 分得清"没有 state/audio"与
    "网关挂了" -- 后者由 heartbeat / state/link 覆盖.

    MUTATION: 把 return None 改成 return audio_payload(idle, idle) -> 红.
    """
    proj, clock = _proj()
    assert proj._audio({"audio": None}) is None
    assert proj._audio({}) is None
    # 非 dict 也不能当成有数据(一条坏报文不该被解释成空闲).
    assert proj._audio({"audio": "idle"}) is None


# --- 陈旧 -------------------------------------------------------------

def test_a_stale_snapshot_becomes_fault_not_the_last_known_value():
    """p2 停发之后不能继续把最后一帧当现状.

    p2 的下限是 1 Hz, 连续 5 拍没来已经不是抖动. 这时喇叭能不能用是未知
    的, 而操作员按"喊话"不会有任何反应 -- 界面停在冻结的正常态最坏.

    MUTATION: 把 age_ms > AUDIO_STALE_MS 改成 >= 一个极大值 -> 红.
    """
    proj, clock = _proj()
    st = _state(clock)
    # 刚到, 不陈旧.
    assert proj._audio(st)["speaker_state"] == "idle"
    # 越过门限.
    clock.advance(AUDIO_STALE_MS / 1000.0 + 0.1)
    out = proj._audio(st)
    assert out["speaker_state"] == "fault"
    assert out["microphone_state"] == "fault"


def test_the_stale_gate_tolerates_one_missed_beat():
    """*** 与上一条配对: 门限不能收到 1 拍.

    收到 1 拍等于要求零抖动, 判据会频繁误报; 而一条频繁误报的判据最后
    一定被人放宽成永远不报(CLAUDE.md 3.2 形态二).
    """
    proj, clock = _proj()
    st = _state(clock)
    clock.advance(2.0)          # 漏了一拍多
    assert proj._audio(st)["speaker_state"] == "idle"


# --- 正常映射 ---------------------------------------------------------

def test_idle_maps_to_idle_idle():
    proj, clock = _proj()
    out = proj._audio(_state(clock))
    assert (out["speaker_state"], out["microphone_state"]) == ("idle", "idle")
    assert out["playing"] is False
    assert out["speaker_holder"] is None


def test_speaking_maps_to_playing_and_gates_the_mic_to_disabled():
    """半双工: 放音时麦克风是 disabled 而不是 idle.

    AEC 在本项目结构上不可能(TTS 在 GZH-2 设备内合成), 互斥全靠门控 --
    这个差别对 Qt 的按钮显示有意义.
    """
    proj, clock = _proj()
    out = proj._audio(_state(clock, audio=_audio_snap(holder="alarm_d")))
    assert out["speaker_state"] == "playing"
    assert out["playing"] is True
    assert out["microphone_state"] == "disabled"
    assert out["speaker_holder"] == "alarm_d"


def test_speaking_with_an_unknown_source_still_reads_as_playing():
    """holder 为 None(发布方没填 source) 时仍然是"在响".

    *** 这是最容易踩反的一脚: 把 None 当空闲, 于是一次没填 source 的
    喊话在界面上表现为"喇叭没响", 而喇叭正在响.

    MUTATION: speaking 改成 bool(holder) 或 holder not in (None,"none")
    -> 这里红.
    """
    proj, clock = _proj()
    out = proj._audio(_state(clock, audio=_audio_snap(holder=None)))
    assert out["speaker_state"] == "playing"
    assert out["microphone_state"] == "disabled"


def test_recording_wins_over_idle_when_the_mic_is_open():
    proj, clock = _proj()
    out = proj._audio(_state(clock, audio=_audio_snap(mic_open=True)))
    assert out["microphone_state"] == "recording"
    assert out["recording"] is True


# --- 设备故障 ---------------------------------------------------------

def test_speaker_fail_overrides_playing():
    """devices.speaker=fail 是 FATAL 级, 压过"在放音"."""
    proj, clock = _proj()
    st = _state(clock, audio=_audio_snap(
        holder="alarm_d", devices={"mic": "ok", "speaker": "fail"}))
    assert proj._audio(st)["speaker_state"] == "fault"


def test_a_missing_speaker_key_is_not_a_fault():
    """*** 与上一条配对. 问不到 payload-service 时 devices.speaker 整键不在.

    当成 fault 会在 payload 服务没起的机器上凭空造一条 FATAL 告警 --
    而它现在正是这个状态(GZH-2 硬件送去做线缆, 不在线).

    MUTATION: 把判据写成 dev.get("speaker") != "ok" -> 这里红.
    """
    proj, clock = _proj()
    st = _state(clock, audio=_audio_snap(devices={"mic": "ok"}))
    assert proj._audio(st)["speaker_state"] == "idle"


def test_mic_absent_and_fail_both_read_as_fault():
    """MIC_STATES 没有"没装麦"这个值; disabled 专指被门控关掉.

    拿 disabled 表示"没有麦"会让操作员以为放完音就能录.
    """
    proj, clock = _proj()
    for dev in ("fail", "absent"):
        st = _state(clock, audio=_audio_snap(devices={"mic": dev}))
        assert proj._audio(st)["microphone_state"] == "fault", dev


# --- 尚未接通的链路 ---------------------------------------------------

def test_broadcast_only_fields_stay_null_until_that_chain_lands():
    """stream_id 与 last_frame_age_ms 属云端喊话链路(audio/broadcast).

    那条链路还没接(p5 收到 PCM 直接丢弃), 现在填任何值都是谎报系统有
    这个能力(CLAUDE.md 9.3). 链路接上时[本条要跟着改], 改不动说明链路
    没真接上.
    """
    proj, clock = _proj()
    out = proj._audio(_state(clock, audio=_audio_snap(holder="broadcast_b")))
    assert out["stream_id"] is None
    assert out["last_frame_age_ms"] is None


def test_speaker_holder_type_is_null_because_v2_gives_no_closed_set():
    """v2.0 S4.4 只给了一个样例值 "cloud", 没有闭集.

    在网关按 holder 前缀自建 cloud/local/wecom/alarm 映射表就是造第二份
    真源. 猜错比报 null 更坏: Qt 若按这个字段切界面, 自造的值会让它切错.
    待甲方给出闭集后再填, 届时本条改成正向断言.
    """
    proj, clock = _proj()
    out = proj._audio(_state(clock, audio=_audio_snap(holder="tts_cloud")))
    assert out["speaker_holder_type"] is None
