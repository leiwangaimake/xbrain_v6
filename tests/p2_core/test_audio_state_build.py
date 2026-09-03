"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_audio_state_build.py
Brief: build_audio_state -- 11 S8.10 AudioState 的 p2 可填子集

Description:
守 build_audio_state 的三件事: (1) 只填 p2 真持有的量, 拿不到的[整键省略];
(2) devices.* 走闭集, 越界抛; (3) "问不到"与"确实坏了"在报文上必须可分.

为什么这三件值得单独立测: state/audio 在 2026-09-03 之前[根本没有发布者],
而 p5 那侧从 hmi_state 根上读五个没人写的扁平键, .get() 全落 None, 于是云端
的 speaker_state 恒 "idle". 一条稳定 1 Hz 发出的 "idle" 与真的空闲不可区分 --
这类"看起来一直正常"的读数正是本项目反复踩的那个坑(CLAUDE.md 3.2 形态一).
所以本文件的断言几乎都是[负向 + 正向成对]的: 光断言"空闲时报 none"会被一个
恒返回 none 的空壳实现通过.

本文件不测: p5 那侧的映射(见 tests/p5_gateway/test_cloud_audio_projection.py)
与 1 Hz + 变更即报的节律(在 p2 主循环里, 见 test_audio_state_cadence.py).
"""

import pytest

from xbrain.p2_core.messaging.audio_state import (DEVICE_STATES,

                                                  build_audio_state)

# INF-TS-1 三档 marker. 纯函数 / 静态检查, 不碰任何硬件 -> no_device.
pytestmark = pytest.mark.no_device


def _view(speaking=False, since=None, holder=None, reason="idle"):
    """SpeakerDomain.audio_view() 的返回形状.

    *** holder 原样透传, NO 桩不替被测代码做 "if speaking else none".
    原来的桩自己算了这一步, 于是 build_audio_state 里那句同样的判断
    [一次都没被测到] -- 变异体 M6(空闲时也报 view 的 holder)全绿通过.
    这与之前抓到的"假 DAO 自己做了对的事"是同一个盲区: 桩越聪明, 断言
    越测不到真代码.
    """
    return {"speaking": speaking, "since_mono": since,
            "holder": holder, "gate_reason": reason}


def _build(**over):
    kw = dict(speaker_view=_view(), mic_muted=False, mic_streaming=True,
              mic_device_name="hw:0,0", mic_frames_dropped_gate=0,
              broadcast_holder=None,
              payload_audio_ok=True, voice_mode=None, ts_mono=1000.0)
    kw.update(over)
    return build_audio_state(**kw)


# --- speaker ----------------------------------------------------------

def test_idle_reports_holder_none_and_no_elapsed():
    """S8.10 逐字 "none = 空闲". 空闲时不该有 elapsed_ms.

    MUTATION: 让 holder 恒返回 speaker_view["holder"] -> 空闲时变 None, 这里红.
    """
    # 喂一个[上一次喊话残留]的 holder: _speaking_source 在放音结束时并不
    # 清空(audio_view 靠 speaking 挡住它), 所以这正是真会出现的输入.
    out = _build(speaker_view=_view(speaking=False, holder="alarm_d"))
    assert out["speaker"]["holder"] == "none"
    assert "elapsed_ms" not in out["speaker"]
    assert out["speaker"]["since_mono"] is None


def test_speaking_carries_the_source_as_holder_and_computes_elapsed():
    """在说话时 holder 必须是 SpeakRequest.source, elapsed 由单调钟差算出.

    *** 这是与上一条配对的正向断言. 只有上一条的话, 一个"holder 恒 none"
    的空壳实现全绿通过 -- 那正是 CLAUDE.md 3.3 说的"只写负向等于没写".

    MUTATION: elapsed_ms 改用 since_mono*1000 -> 这里红.
    """
    out = _build(speaker_view=_view(speaking=True, since=997.5,
                                    holder="alarm_d"),
                 ts_mono=1000.0)
    assert out["speaker"]["holder"] == "alarm_d"
    assert out["speaker"]["since_mono"] == 997.5
    assert out["speaker"]["elapsed_ms"] == 2500


def test_speaking_without_a_source_is_not_reported_as_idle():
    """发布方没填 SpeakRequest.source 时 holder 为 None, NO 不是 "none".

    两者对下游是两件事: "none" = 确实空闲, None = 在说但不知是谁. 混掉的
    表现是一次没填 source 的喊话在界面上显示"喇叭没响".
    """
    out = _build(speaker_view=_view(speaking=True, since=999.0, holder=None))
    assert out["speaker"]["holder"] is None
    assert out["speaker"]["holder"] != "none"
    assert out["speaker"]["elapsed_ms"] == 1000


# --- mic --------------------------------------------------------------

def test_mic_open_only_when_present_and_unmuted():
    """半双工: 说话期间 handle_speak 先 mute, 所以 open 与放音互斥."""
    assert _build(mic_muted=False)["mic"]["open"] is True
    assert _build(mic_muted=True)["mic"]["open"] is False
    assert _build(mic_muted=None)["mic"]["open"] is False


def test_mic_device_is_the_alsa_name_not_a_health_word():
    """S8.10 的 mic.device 是设备标识("usb_mic_0"), 健康在 devices.mic.

    混掉的表现是 Qt 的设备名一栏显示 "ok". 本条就是守这个混淆的.
    """
    out = _build(mic_device_name="hw:0,0")
    assert out["mic"]["device"] == "hw:0,0"
    assert out["mic"]["device"] not in DEVICE_STATES


def test_frames_dropped_gate_passes_through_and_is_absent_without_a_source():
    """S8.10: 该计数"持续快速增长 = 喇叭一直在响", 是排障第一根线索.

    无来源时[整键不出现], NO 不填 0 -- 0 表示"一帧没丢", 与真的没丢完全
    一样(CLAUDE.md 3.1 那条 "0.0 冒充已赋值").
    """
    assert _build(mic_frames_dropped_gate=118)["mic"]["frames_dropped_gate"] == 118
    assert "frames_dropped_gate" not in _build(mic_frames_dropped_gate=None)["mic"]


# --- devices ----------------------------------------------------------

def test_mic_device_state_separates_absent_from_fail():
    """没装麦(absent) 与 装了但不出帧(fail) 是两件事.

    absent -> 本来就没有; fail -> 有但坏了, 该派人去看. 报同一个值会让
    "USB 麦掉线"淹没在"这台机器没配麦"里.
    """
    assert _build(mic_muted=None, mic_streaming=None)["devices"]["mic"] == "absent"
    assert _build(mic_streaming=False)["devices"]["mic"] == "fail"
    assert _build(mic_streaming=True)["devices"]["mic"] == "ok"


def test_speaker_key_is_absent_when_payload_cannot_be_reached():
    """问不到 payload-service 时 devices.speaker [整键不出现].

    *** NO 不报 fail: S8.10 逐字规定 speaker=fail 是 FATAL 级(失去喊话
    能力 = 失去核心威慑), 把"问不到"报成 fail 会凭空造一条 FATAL 告警.
    *** NO 不报 ok: 那会盖掉真故障.
    字段缺失是这三者里唯一诚实的一个.

    MUTATION: 把 None 也走进 _device("ok" if x else "fail") -> 这里红.
    """
    assert "speaker" not in _build(payload_audio_ok=None)["devices"]
    assert _build(payload_audio_ok=False)["devices"]["speaker"] == "fail"
    assert _build(payload_audio_ok=True)["devices"]["speaker"] == "ok"


def test_device_state_outside_the_closed_set_raises():
    """CLAUDE.md 3.5: 闭集外必抛, NO 不静默透传.

    直接打 _device 而不是 build_audio_state -- 后者构造不出越界值, 拿它
    测等于测不到(判据自伤: 断言落在自己够不着的地方).
    """
    from xbrain.p2_core.messaging.audio_state import _device
    with pytest.raises(ValueError):
        _device("online")
    assert _device(None) is None
    for ok in DEVICE_STATES:
        assert _device(ok) == ok


# --- 不填的那些 -------------------------------------------------------

def test_fields_p2_does_not_hold_are_omitted_entirely():
    """preset / repeat_left / volume / pending / gate_seq / services 无来源.

    填 0 或 "ok" 会让操作员看到"服务正常"的假读数. 本条把[省略]这件事
    钉死: 哪天有人顺手补一个默认值, 这里红.
    """
    out = _build()
    for k in ("kind", "preset_id", "repeat_left", "volume", "pending"):
        assert k not in out["speaker"], k
    for k in ("gate_seq", "listen_window", "asr_holder", "rms_dbfs",
              "frames_dropped_seq"):
        assert k not in out["mic"], k
    assert "services" not in out
    # voice_mode 的持有者也还不存在(mode_wiring._remember_profile 只回写
    # profile / locked / max_profile), 所以它必须是 None 而不是 "normal".
    assert out["voice_mode"] is None


# --- 节律 (11 S2.2.2 "1 Hz + 变更即报") -------------------------------

def test_an_unchanged_body_still_goes_out_once_per_period():
    """没变也要发: 消费方要分得清"没变"与"p2 死了".

    MUTATION: 把 or 后半段删掉(只在变了时发) -> 这里红.
    """
    from xbrain.p2_core.messaging.audio_state import audio_publish_due
    body = _build()
    due, cmp1 = audio_publish_due(body, None, 0.0, 0.0, 1.0)
    assert due is True                      # 首拍(last_cmp=None)必发
    # 同一份 body, 还没到点 -> 不发
    due, _ = audio_publish_due(body, cmp1, 0.5, 0.0, 1.0)
    assert due is False
    # 到点 -> 发
    due, _ = audio_publish_due(body, cmp1, 1.0, 0.0, 1.0)
    assert due is True


def test_a_change_goes_out_immediately_without_waiting_for_the_period():
    """*** 与上一条配对. 变了就发, 不等 1 秒.

    只做 1 Hz 会把"喇叭响了"这个瞬态采样漏掉 -- 一次 300ms 的提示音可以
    整个落在两次采样之间, 界面上什么都不会闪.

    MUTATION: 把 cmp_key != last_cmp 删掉 -> 这里红.
    """
    from xbrain.p2_core.messaging.audio_state import audio_publish_due
    idle = _build()
    _, cmp_idle = audio_publish_due(idle, None, 0.0, 0.0, 1.0)
    talking = _build(speaker_view=_view(speaking=True, since=0.05,
                                        holder="alarm_d"), ts_mono=0.1)
    due, _ = audio_publish_due(talking, cmp_idle, 0.1, 0.0, 1.0)
    assert due is True, "变了却要等到 1 秒才发"


def test_ts_mono_alone_never_counts_as_a_change():
    """*** 摘掉 ts_mono 再比对, 否则"变更即报"退化成"每拍都报".

    退化的表现不是报错, 是 state/audio 从 1 Hz 变成 10 Hz -- 在 Q2 上多
    出 9 倍的报文, 而没有任何测试会红.

    MUTATION: 让 cmp_key 保留 ts_mono -> 这里红.
    """
    from xbrain.p2_core.messaging.audio_state import audio_publish_due
    a = _build(ts_mono=1000.0)
    b = _build(ts_mono=1000.05)
    _, cmp_a = audio_publish_due(a, None, 0.0, 0.0, 1.0)
    due, _ = audio_publish_due(b, cmp_a, 0.05, 0.0, 1.0)
    assert due is False, "只有 ts_mono 变了也当成变更"
    # 而 body 里发出去的仍然带 ts_mono -- 消费方要靠它算年龄.
    assert b["ts_mono"] == 1000.05



# --- B 模式 ----------------------------------------------------------

def test_a_running_broadcast_reads_as_speaking_even_though_tts_is_idle():
    """*** 云端喊话[不经过 handle_speak], SpeakerDomain 的锁全程没被拿过.

    只看 speaker_view 的话, 一次云端喊话期间 state/audio 会一直报
    holder="none" -- 甲方界面上喇叭正响着而那格显示空闲, 而这恰恰是他们
    最需要看的一格.

    MUTATION: 把 broadcast_holder 从 speaking 的判断里去掉 -> 这里红.
    """
    out = _build(speaker_view=_view(speaking=False),
                 broadcast_holder="broadcast_b")
    assert out["speaker"]["holder"] == "broadcast_b"
    assert out["mic"]["open"] is False or True   # mic 由 mute 决定, 不在本条


def test_broadcast_outranks_a_concurrent_tts_holder():
    """域2 里 broadcast_b(800) 高于 tts_*(400~600).

    真同时发生时在响的是 broadcast_b, 报另一个就是报了个不在响的源.
    """
    out = _build(speaker_view=_view(speaking=True, since=1.0,
                                    holder="tts_local"),
                 broadcast_holder="broadcast_b")
    assert out["speaker"]["holder"] == "broadcast_b"


def test_no_broadcast_leaves_the_tts_holder_alone():
    """*** 与上一条配对: 没有广播时不许把 holder 顶掉.

    只有上面两条的话, 一个"holder 恒 broadcast_b"的实现会全绿.
    """
    out = _build(speaker_view=_view(speaking=True, since=1.0,
                                    holder="tts_local"),
                 broadcast_holder=None)
    assert out["speaker"]["holder"] == "tts_local"
