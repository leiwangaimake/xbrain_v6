"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: audio_state.py
Brief: BIZ-P2-0 -- state/audio and rt/audio/gate coherence rule

Description:
BIZ-P2-0 assertion #4: 'inject mic=fail -> state/audio.mic becomes
device_fault, and rt/audio/gate.reason=device_fault same-tick'.
The variant that makes this rule non-trivial: 'only change gate,
not state/audio -> variant fires red'.

The physical failure mode this catches: the mic device dropped off
the bus. P2's audio_io publishes state/audio.mic=device_fault; the
half-duplex publisher (owner of rt/audio/gate) must simultaneously
publish gate=closed reason=device_fault. If only ONE side moves, a
downstream consumer (P4) sees inconsistent state -- gate says
'closed for device fault' but state/audio still says 'ok', so
health/factor never downgrades and the operator sees a green fleet
while the robot is deaf.

This module owns the ATOMIC pair. Callers submit an audio-status
change through `apply_mic_status(new_status)`, and the module emits
BOTH messages via the injected publishers in one call. Skipping the
gate half OR the state half is only possible by NOT calling this
function, and the p2_publisher whitelist gate prevents an ad-hoc
call site from publishing either key without going through here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, FrozenSet, Optional, Tuple


# Closed set for state/audio.mic per 11 S8.9.1 (mic status).
# ok            = capturing, frames flowing
# muted         = intentionally gated (half-duplex, mode = broadcast, ...)
# device_fault  = mic hardware / driver failure (evdev EBUSY / disconnect)
# not_configured = deploy did not set up an audio source
_MIC_STATUSES: FrozenSet[str] = frozenset({
    "ok", "muted", "device_fault", "not_configured",
})

# Closed set for rt/audio/gate.reason per 11 S8.9.2 AsrGate.
# When gate is closed, reason names WHY. The seven-value set from
# 14 S4.1.3 GS-1..GS-3 + BIZ-P2-4 spec:
_GATE_REASONS: FrozenSet[str] = frozenset({
    "speaker_active", "tail_hold", "b_mode", "device_fault",
    "not_configured", "hes", "unknown",
})


@dataclass(frozen=True)
class AudioStateSnapshot:
    """The pair of values state/audio and rt/audio/gate publish together.

    The very existence of this dataclass makes it structurally
    impossible to publish only ONE of the two -- callers construct a
    snapshot and hand it off; the emitter publishes both."""
    mic_status: str            # state/audio.mic (closed set)
    mic_open: bool             # rt/audio/gate.mic_open
    gate_reason: str           # rt/audio/gate.reason (closed set)

    def __post_init__(self) -> None:
        if self.mic_status not in _MIC_STATUSES:
            raise ValueError(
                "mic_status %r not in closed set %s"
                % (self.mic_status, sorted(_MIC_STATUSES)))
        if self.gate_reason not in _GATE_REASONS:
            raise ValueError(
                "gate_reason %r not in closed set %s"
                % (self.gate_reason, sorted(_GATE_REASONS)))
        # Coherence: if mic_status = device_fault, gate reason must
        # ALSO be device_fault (BIZ-P2-0 assertion #4). Any other pair
        # here is a construction defect.
        if self.mic_status == "device_fault" \
                and self.gate_reason != "device_fault":
            raise ValueError(
                "mic_status=device_fault requires gate_reason=device_fault "
                "(BIZ-P2-0 assertion #4); got gate_reason=%r"
                % self.gate_reason)
        if self.mic_status == "device_fault" and self.mic_open:
            raise ValueError(
                "mic_status=device_fault requires mic_open=False; "
                "cannot claim open microphone under a device fault")
        if self.mic_status == "not_configured" \
                and self.gate_reason not in ("not_configured", "unknown"):
            raise ValueError(
                "mic_status=not_configured requires gate_reason "
                "not_configured or unknown; got %r" % self.gate_reason)


def publish_snapshot(
    snap: AudioStateSnapshot,
    publish_state_audio: Callable[[Dict[str, Any]], None],
    publish_gate: Callable[[Dict[str, Any]], None],
) -> None:
    """Publish state/audio + rt/audio/gate atomically from a snapshot.

    'Atomically' here means 'in one function call and in this order';
    it does not mean anything about Zenoh network semantics (Zenoh
    provides no cross-key transaction). If publish_state_audio raises
    the gate publish still runs -- the caller decides whether to
    treat that as complete failure or as best-effort; either way both
    sides SAW the intent to publish, which is the check the assertion
    variant #4 tests.

    Order note: publish state/audio FIRST so a subscriber that reads
    both keys in one query sees state/audio matching the gate change
    it just observed -- not the reverse."""
    publish_state_audio({
        "mic": snap.mic_status,
    })
    publish_gate({
        "mic_open": snap.mic_open,
        "reason": snap.gate_reason,
    })


#: 11 S8.10 devices.* 闭集(逐字 "ok | degraded | fail | absent").
DEVICE_STATES: FrozenSet[str] = frozenset({"ok", "degraded", "fail", "absent"})


def _device(value: Optional[str]) -> Optional[str]:
    """闭集卫. 越界必抛(CLAUDE.md 3.5), None 透传表示"无来源"."""
    if value is None:
        return None
    if value not in DEVICE_STATES:
        raise ValueError(
            "devices.* must be in %r, got %r" % (sorted(DEVICE_STATES), value))
    return value


#: 11 S8.10 AudioState 里 p2 [当前真正持有]的那部分.
#:
#: *** 只填拿得到的, 其余整键省略 -- NO 不填占位值.
#: S8.10 的完整结构还有 speaker.kind / preset_id / repeat_left / volume /
#: pending(WAIT_ATOMIC 等待队列), mic.gate_seq / listen_window / asr_holder /
#: rms_dbfs / frames_dropped_seq, 以及 services.asr|tts 的 state/p95/breaker/
#: queue. 这些量的持有者不是 p2: preset 与等待队列属预置话术子系统(未建),
#: asr/tts 的服务指标属 p4 的 ai_client, gate_seq 属 BIZ-P2-0 的门控序号
#: (SpeakerDomain 现在只记 open/reason, 不记序号).
#: 填 0 或 "ok" 会让操作员看到一个"服务正常"的假读数 -- 与 CLAUDE.md 3.1 的
#: "0.0 冒充已赋值"同一个失效模式, 只是换成了字符串.
#: 拿不到的整键不出现在报文里, 消费方按"字段缺失 = 该子系统未建"处理.
def build_audio_state(*, speaker_view: Dict[str, Any],
                      mic_muted: Optional[bool],
                      mic_streaming: Optional[bool],
                      mic_device_name: Optional[str],
                      mic_frames_dropped_gate: Optional[int],
                      broadcast_holder: Optional[str],
                      broadcast_since_mono: Optional[float],
                      payload_audio_ok: Optional[bool],
                      voice_mode: Optional[str],
                      ts_mono: float) -> Dict[str, Any]:
    """live 输入 -> 11 S8.10 AudioState 的 p2 可填子集.

    speaker_view 来自 SpeakerDomain.audio_view() 的一次性快照 -- 分开读会让
    speaker 与 gate 在同一条报文里不自洽(BIZ-P2-0 断言四).

    mic_muted / mic_streaming 为 None 表示[没有 MIC 发布线程](未配麦克风),
    与 False(有麦但静音 / 有麦但没出帧)是三件不同的事:
      None  -> devices.mic = absent
      False -> devices.mic = fail    (线程在, 但没出帧)
      True  -> devices.mic = ok

    *** mic_streaming 必须来自[真的出帧了]这个观测, NO 不能用线程 is_alive().
    MicCaptureThread 有重生循环, 线程永不退出 => is_alive() 恒 True(见
    mic_capture.py 那段注释). 拿它当存活信号就是一条永远绿的读数
    (CLAUDE.md 3.2 形态一) -- 拔掉 USB 麦, 这条报文照样说 mic ok.

    payload_audio_ok 为 None 表示[问不到] payload-service(它可能没起, 或
    GZH-2 硬件不在线). 这时 devices.speaker [整键不出现], NO 不报 fail:
    S8.10 逐字规定 speaker=fail 是 FATAL 级(失去喊话能力 = 失去核心威慑),
    把"问不到"报成 fail 会凭空造一条 FATAL 告警; 报 ok 则会盖掉真故障.
    字段缺失是这三者里唯一诚实的一个.
    """
    # B 模式的云端音频[不经过 handle_speak]: PCM 从 audio/broadcast 直接进
    # WS /play, SpeakerDomain 的锁全程没被拿过. 只看 speaker_view 的话,
    # 一次云端喊话期间 state/audio 会一直报 holder="none" -- 甲方界面上
    # 喇叭正响着而那格显示空闲.
    # broadcast_holder 非 None 时优先: 域2 里 broadcast_b(800) 高于所有
    # tts_*(400~600), 真同时发生时也是它在响.
    speaking = bool(speaker_view.get("speaking")) or broadcast_holder is not None
    since_mono = speaker_view.get("since_mono")
    speaker: Dict[str, Any] = {
        # S8.10 逐字 "none = 空闲". 说话中而发布方没填 source 时为 None.
        "holder": (broadcast_holder if broadcast_holder is not None
                   else (speaker_view.get("holder") if speaking else "none")),
        # 广播时用广播会话的起点: SpeakerDomain 的锁全程没被拿过, 它那个
        # since_mono 是上一次 TTS 的(或 None). 报错的起点会让操作员看到
        # "已经喊了 3 小时".
        "since_mono": (broadcast_since_mono if broadcast_holder is not None
                       else (since_mono if speaking else None)),
    }
    # elapsed 必须与上面那个 since_mono [同源], NO 不能再读一次局部变量.
    # 分开读的话广播期间 elapsed 会按上一次 TTS 的起点算, 与同一条报文里
    # 的 since_mono 自相矛盾.
    _since = speaker["since_mono"]
    if speaking and isinstance(_since, (int, float)):
        # 单调钟之差, 单位 ms(CLK-C1: ts_mono 由调用方传入, 本函数不读钟,
        # 无设备单测才能喂一个固定的 now).
        speaker["elapsed_ms"] = int(max(0.0, ts_mono - _since) * 1000.0)

    mic_present = mic_muted is not None
    mic: Dict[str, Any] = {
        # open = 有麦 且 未静音. 半双工下说话期间必然静音(handle_speak 先
        # mute 再请求 TTS), 所以 speaker.holder != none 与 mic.open 天然互斥.
        "open": bool(mic_present and not mic_muted),
        "gate_reason": speaker_view.get("gate_reason", "idle"),
    }
    if mic_device_name:
        mic["device"] = mic_device_name
    if mic_frames_dropped_gate is not None:
        # S8.10: "持续快速增长 = 喇叭一直在响", 是现场排障的第一根线索.
        mic["frames_dropped_gate"] = int(mic_frames_dropped_gate)

    devices: Dict[str, Any] = {}
    if not mic_present:
        devices["mic"] = _device("absent")
    else:
        devices["mic"] = _device("ok" if mic_streaming else "fail")
    if payload_audio_ok is not None:
        devices["speaker"] = _device("ok" if payload_audio_ok else "fail")

    return {
        "speaker": speaker,
        "mic": mic,
        "devices": devices,
        "voice_mode": voice_mode,
        "ts_mono": ts_mono,
    }


#: 参与报文但不参与"变了没"比对的键.
VOLATILE_KEY = "ts_mono"


#: state/audio 的节律判据(11 S2.2.2 "1 Hz + 变更即报").
#:
#: 从 p2 主循环里提出来的 -- 循环里那几行[已经写错过一次]: 把求值整个包在
#: "now - last_sent >= period" 里, 于是"变更即报"被降频成 1 Hz, 而里面那条
#: keepalive 分支永远进不去(CLAUDE.md 3.2 形态一: 一条不执行的分支, 测试
#: 照样绿). 提成纯函数才测得到.
def audio_publish_due(body: Dict[str, Any],
                      last_cmp: Optional[Dict[str, Any]],
                      now_mono: float,
                      last_sent_mono: float,
                      period_s: float) -> Tuple[bool, Dict[str, Any]]:
    """要不要发这一条? 返回 (要发, 供下次比对的键).

    period_s 是[下限]不是节拍: 变了就立刻发(上限由调用方的循环频率定),
    没变也至少每 period_s 发一次.

    *** 比对前摘掉 ts_mono.
    它每拍都不同, 带上的话"变更即报"退化成"每拍都报"(p5 的 _due 为同一个
    原因摘掉 msg_id). 摘的是[比对键], 发出去的报文仍然带 ts_mono -- 消费方
    要靠它算年龄.
    """
    cmp_key = {k: v for k, v in body.items() if k != VOLATILE_KEY}
    due = (cmp_key != last_cmp) or (now_mono - last_sent_mono >= period_s)
    return due, cmp_key
