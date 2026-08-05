"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: config.py
Brief: Centralized, environment-overridable configuration for the AI_runtime test process.

Description:
  AI_runtime is the orchestrator of 功能1 近场AI对话: it is the sole audio client of
  payload-service (invariant R2), the only caller of asr-service and llama-server, and the
  owner of the mode state machine. That makes it the one process that has to know where all
  three services live and how the turn is timed, so every one of those numbers is collected
  here instead of being spelled out at its point of use.

  Why one frozen object rather than module-level constants: the turn loop, the VAD and the
  three clients all read the same settings, and a run has to be reproducible from the
  values it started with. A frozen dataclass means the configuration a run reports in its
  first log line is provably the configuration it used to the end.

  Why every field is environment-overridable: this process is a TEST harness. Retuning a
  VAD threshold or pointing it at a service on another port is something done repeatedly
  during a bring-up session, and editing source between runs would make the runs
  incomparable. from_env() is the only place environment parsing happens.

  A bad override is a hard error, never a silent fallback: a mistyped AI_VAD_THRESHOLD that
  quietly reverted to the default would surface as "the robot does not hear me", which is
  an expensive thing to debug on a device compared with refusing to start.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

# Words accepted for boolean overrides. Anything else raises rather than reading as false,
# so a typo cannot silently disable a feature the operator believed was on.
_TRUE_WORDS = frozenset({"1", "true", "yes", "on"})
_FALSE_WORDS = frozenset({"0", "false", "no", "off"})

# This process lives at tests/ai_runtime/, so the repository root is two levels up.
# Resolved at import time so the derived model path is absolute no matter which directory
# the operator launched from, matching how AsrConfig locates its own model tree.
_AI_RUNTIME_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_AI_RUNTIME_DIR))

# Environment keys. Named constants rather than inline strings so the key a value is read
# from and the key documented for the operator cannot drift apart.
_ENV_PAYLOAD_URL = "AI_PAYLOAD_URL"
_ENV_ASR_URL = "AI_ASR_URL"
_ENV_LLM_URL = "AI_LLM_URL"
_ENV_LLM_MODEL = "AI_LLM_MODEL"
_ENV_LLM_MAX_TOKENS = "AI_LLM_MAX_TOKENS"
_ENV_LLM_TEMPERATURE = "AI_LLM_TEMPERATURE"
_ENV_LLM_SYSTEM_PROMPT = "AI_LLM_SYSTEM_PROMPT"
_ENV_HTTP_TIMEOUT_S = "AI_HTTP_TIMEOUT_S"
_ENV_MIC_SOURCE = "AI_MIC_SOURCE"
_ENV_MIC_ALSA_DEVICE = "AI_MIC_ALSA_DEVICE"
_ENV_MIC_OPEN_TIMEOUT_S = "AI_MIC_OPEN_TIMEOUT_S"
_ENV_VAD_BACKEND = "AI_VAD_BACKEND"
_ENV_VAD_SILERO_MODEL = "AI_VAD_SILERO_MODEL"
_ENV_VAD_SILERO_THRESHOLD = "AI_VAD_SILERO_THRESHOLD"
_ENV_VAD_THRESHOLD = "AI_VAD_THRESHOLD"
_ENV_VAD_START_MS = "AI_VAD_START_MS"
_ENV_VAD_STOP_MS = "AI_VAD_STOP_MS"
_ENV_VAD_PREROLL_MS = "AI_VAD_PREROLL_MS"
_ENV_VAD_MIN_UTTERANCE_MS = "AI_VAD_MIN_UTTERANCE_MS"
_ENV_VAD_MAX_UTTERANCE_MS = "AI_VAD_MAX_UTTERANCE_MS"
_ENV_TTS_VOICE = "AI_TTS_VOICE"
_ENV_TTS_GATE_MARGIN_MS = "AI_TTS_GATE_MARGIN_MS"
_ENV_REPLY_MAX_CHARS = "AI_REPLY_MAX_CHARS"
_ENV_INTERCOM_HOST = "AI_INTERCOM_HOST"
_ENV_INTERCOM_PORT = "AI_INTERCOM_PORT"
_ENV_INTERCOM_TURNAROUND_MS = "AI_INTERCOM_TURNAROUND_MS"


class AiRuntimeConfigError(ValueError):
    """Raised when an environment override cannot be parsed into its typed field.

    House rule bans bare Exception. A dedicated type separates "the operator mistyped a
    setting" from every runtime fault this process can hit -- a service being down, the
    device link dropping, a decode failing -- which need entirely different remedies and
    so must not share an exception type. ValueError is the base because every case here is
    literally a bad value.
    """


def _env_str(name: str, default: str) -> str:
    """Read a string environment override, or return default if unset.

    Args:
        name: the environment variable name to read.
        default: the value to use when the variable is absent.

    Returns:
        The variable's value, or default when it is not set.

    An empty string is returned as-is rather than treated as unset, because "" is a
    meaningful value for the system prompt field (the plan leaves that prompt empty for
    now) and collapsing it into the default would make it impossible to ask for.
    """
    raw = os.environ.get(name)
    return default if raw is None else raw


def _env_int(name: str, default: int) -> int:
    """Read an integer environment override, or return default if unset.

    Args:
        name: the environment variable name to read.
        default: the value to use when the variable is absent.

    Returns:
        The parsed integer, or default when the variable is not set.

    Raises:
        AiRuntimeConfigError: when the variable is set but is not a valid integer.

    int()'s own message does not name the offending variable, which is the first thing an
    operator needs, so the failure is re-raised with the key included.
    """
    raw = os.environ.get(name)
    # Absent variable is the normal case: fall back to the documented default.
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        # Chain the original so the traceback keeps the underlying parse failure.
        raise AiRuntimeConfigError(f"env {name}={raw!r} is not an integer") from exc


def _env_float(name: str, default: float) -> float:
    """Read a float environment override, or return default if unset.

    Args:
        name: the environment variable name to read.
        default: the value to use when the variable is absent.

    Returns:
        The parsed float, or default when the variable is not set.

    Raises:
        AiRuntimeConfigError: when the variable is set but is not a valid float.
    """
    raw = os.environ.get(name)
    # Absent variable is the normal case: fall back to the documented default.
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError as exc:
        # Chain the original so both the key name and the parse cause survive.
        raise AiRuntimeConfigError(f"env {name}={raw!r} is not a float") from exc


@dataclass(frozen=True)
class AiRuntimeConfig:
    """Immutable runtime configuration for the AI_runtime orchestrator.

    Frozen for the same reason as PayloadConfig and AsrConfig: one instance is shared by
    the turn loop, the VAD and all three service clients, several of which run on worker
    threads, and a mutation from any of them would be a bug that only shows up as an
    unreproducible run.

    The defaults describe the standard on-Orin deployment, where all four processes are on
    the same box and talk over loopback. Every field can be overridden through the
    environment, which is how a bring-up session retunes without editing source.
    """

    # -- service endpoints ---------------------------------------------------
    # payload-service owns the device sockets (R1); AI_runtime reaches it only over this
    # base URL, never the device's own 8519/8529 ports. Loopback because the plan puts all
    # four processes on the Orin, and the audio plane must not cross a network hop.
    payload_url: str = "http://127.0.0.1:18080"
    # asr-service; the port it binds by default (AsrConfig.port_asr).
    asr_url: str = "http://127.0.0.1:18081"
    # llama-server. 18082 comes from the port allocation in 99 U52 -- HMI 8080, payload
    # 18080, asr 18081, llama 18082 -- and NOT from llm_server.sh's historical default of
    # 8080, which is the port U52 assigns to the HMI. Pointing the LLM client at 8080 would
    # therefore aim it at a different service the moment both are deployed, and the symptom
    # would be a puzzling protocol error rather than a connection refusal.
    llm_url: str = "http://127.0.0.1:18082"

    # -- LLM request shape ---------------------------------------------------
    # llama-server serves whatever model was loaded on its command line and ignores this
    # field, but the OpenAI schema requires it and it lands in the request log, so a
    # descriptive value is more useful than an empty one.
    llm_model: str = "local"
    # Reply length cap. A 功能1 answer is spoken aloud by the device, and speech is far
    # slower than reading, so a long reply is a long dead air rather than a better answer.
    llm_max_tokens: int = 256
    # Low but non-zero: robot command acknowledgements should be near-deterministic, while
    # exactly 0.0 makes repeated identical questions produce identical wording, which reads
    # as a recording rather than a reply.
    llm_temperature: float = 0.3
    # ★ Constrains FORM ONLY -- how long the reply may be and what characters may appear in
    # it -- and says nothing about who the robot is or how it should behave.
    #
    # That split is deliberate. The original value here was empty, on the reasoning that
    # 16's prompt design has not landed and an invented prompt would have to be un-learned.
    # That reasoning holds for persona and behaviour, and this prompt still leaves both
    # alone: naming the robot, or telling it what it may agree to do, is a product decision
    # that belongs in 16 §6's mission layer, not in a test harness.
    #
    # It does NOT hold for the two facts below, because no future design will want either
    # of them to be otherwise:
    #
    #   Length. reply_max_chars already truncates at 120, so the bound exists either way.
    #   With nothing telling the model, it wrote 500 characters and got guillotined
    #   mid-sentence -- observed live, twice, one reply ending at "2. ". Stating the same
    #   limit the code enforces converts a hard cut into a complete short answer, and turns
    #   reply_max_chars back into the backstop it was written to be.
    #
    #   Spoken form. This text is not displayed, it is handed to the device's TTS and read
    #   aloud verbatim. The model's default markdown -- "1. **信息查询**：" -- is spoken as
    #   punctuation and asterisks. That is a plumbing fact about the output path, not a
    #   style preference.
    #
    # ⚠️ This is the CONVERSATIONAL path, which is a different call from 16 §6.3.1's system
    # layer: that one drives the intent parser and asks for a JSON envelope, so the two
    # prompts do not compete. Note also that 00 CMD-40 and CMD-50 say V6 should not ship
    # free-form LLM chat at all -- answers come from templates over real state, and small
    # talk from a whitelist. This prompt makes the loop pleasant to TEST with; it is not a
    # design for what the product says.
    llm_system_prompt: str = (
        "你是场区巡检机器人的语音助手。你的回答会被直接朗读出来。\n"
        "1 只用纯中文口语，不要用星号、井号、编号列表、表情或任何标记符号。\n"
        "2 每次回答不超过 40 个字，一到两句话说完。\n"
        "3 说不清或做不到就直接简短说明，不要展开解释。"
    )

    # Ceiling on how much of an HTTP response this process will wait for. Sized well above
    # the measured decode time (RTF 0.04 on this box) so a normal slow turn is not cut
    # short, but finite so a hung service ends the turn instead of the whole session.
    http_timeout_s: float = 30.0

    # -- microphone source ---------------------------------------------------
    # Where the 20 ms frames come from: "local" (a microphone on the Orin, read through
    # ALSA) or "device" (the payload's own mic, streamed over payload-service's WS /mic).
    #
    # local is the default because device does not currently work for dialogue. Issue A1:
    # the payload's [40] uplink is decimated to 0.235x realtime -- measured, 2.85 s
    # delivered for 12.10 s recorded, with only 0.10 s of tail after the stop, so the
    # missing audio is never sent rather than sent late. ASR cannot read speech with three
    # quarters of it absent, and the frame carries no rate parameter to turn that off.
    #
    # device is kept, not deleted, because it is the intended long-term path: it is the
    # only one that hears from where the robot is standing, and the vendor has been asked
    # about the rate. Selecting it needs no other change -- the two sources present the
    # same frames.
    mic_source: str = "local"
    # ALSA device opened when mic_source is local. /etc/asound.conf on the Orin points
    # default at the USB card; when the mic array replaces the dongle, that file is the
    # thing to update, not this default.
    #
    # "default" rather than a hw: name because the plug layer must be in the path. It is
    # what converts the current USB dongle's 48 kHz capture -- the only rate it offers --
    # down to the 16 kHz this pipeline requires, and it does so well: the default
    # converter measured -98.2 dB alias rejection for 48k->16k on this box.
    #
    # Overriding this with a bare hw: name is the trap. A hw: device does no conversion,
    # and arecord asked for a rate the hardware cannot do does not fail -- it warns on
    # stderr and proceeds at the hardware's rate. The stream would then be 48 kHz audio
    # read as 16 kHz: every utterance three times too long, every pitch a third of what it
    # was, and ASR transcribing nonsense with nothing anywhere reporting an error.
    mic_alsa_device: str = "default"
    # How long to wait for the first frame when opening the local source. Deliberately far
    # shorter than http_timeout_s: this covers "is the microphone plugged in and free",
    # which is answered in milliseconds when it is, so waiting longer only delays the
    # report. An unplugged mic should stop a bring-up run in seconds.
    mic_open_timeout_s: float = 5.0

    # -- VAD (utterance segmentation) ----------------------------------------
    # Which per-frame speech test the segmenter uses: "silero" or "energy".
    #
    # silero is the default because the failure that matters is not missed speech, it is
    # NON-speech getting through. The zipformer behind asr-service is a pure Chinese model
    # with no blank output -- handed a segment of fan noise it does not return nothing, it
    # returns a plausible Chinese word, and the turn loop then answers a question nobody
    # asked. Energy cannot tell a loud noise from a voice, because loudness is the only
    # thing it measures; silero was trained to, so it rejects the noise before ASR ever
    # sees it.
    #
    # energy is retained because it is the only backend that needs no model file and no
    # inference, which is what lets the segmentation state machine be tested exhaustively
    # from synthetic PCM, and what keeps a bring-up run possible on a box where the onnx
    # model has not been deployed yet.
    vad_backend: str = "silero"
    # Where silero_vad.onnx lives: directly under services/asr/, NOT inside one of the
    # model-<export>/ directories. It used to sit inside the zipformer export, which made
    # it look like part of that download -- so swapping the ASR model moved a file that has
    # nothing to do with ASR weights and broke this default. VAD is a service-level asset
    # shared across every ASR family, so it lives at the service root and survives any
    # model change. Deriving it from the repository root keeps it correct regardless of cwd.
    vad_silero_model: str = os.path.join(
        _REPO_ROOT, "services", "asr", "silero_vad.onnx"
    )
    # Speech probability above which silero calls a window speech, 0.0 to 1.0. 0.5 is the
    # model's own balanced operating point. Raising it rejects more borderline audio at the
    # cost of clipping quiet talkers; the near-field 功能1 mic sits centimetres from the
    # mouth, so there is little reason to lower it.
    vad_silero_threshold: float = 0.5
    # Mean absolute sample amplitude, in int16 counts, above which a 20 ms frame counts as
    # speech. Used only by the "energy" backend. 40 is calibrated for the PAYLOAD mic
    # (mic_source=device), whose noise floor measured about 2 counts against speech at
    # about 950 -- well clear of both.
    #
    # It is WRONG for the USB microphone the local source currently reads: that one's idle
    # floor measured 52 to 127 counts, mean 67, so 40 sits below the noise and the energy
    # backend would report speech continuously. Raise AI_VAD_THRESHOLD before pairing
    # energy with mic_source=local. The default is left here rather than retuned because a
    # correct value needs the speech level measured on that microphone too, and the
    # default backend is silero, which judges by content and is unaffected either way.
    vad_threshold: float = 40.0
    # Consecutive speech needed to declare an utterance started. Long enough that a door
    # slam or a single click cannot open a turn, short enough not to clip a first syllable.
    vad_start_ms: int = 120
    # Trailing silence that ends an utterance.
    #
    # ★★ This is the single largest term in the voice loop AND the only one that does no
    # work -- it is time spent proving the speaker stopped. At 500 ms it was ~31% of a
    # measured 1620 ms audio-in-to-audio-out loop, larger than ASR (360) or the whole LLM
    # reply (312). Lowering it is worth more than any other tuning available.
    #
    # ★ 300, measured. The floor under this value is the longest silence a speaker leaves
    # IN THE MIDDLE of a command -- go below that and the endpointer fires early, splitting
    # one utterance into two. Run over the 59 real human recordings with THIS detector
    # (silero, reset per file), the longest intra-utterance pause was:
    #
    #     p50 0 ms   p90 0 ms   p95 0 ms   max 0 ms      (59/59 spoken in one breath)
    #     thresholds 250/300/400/500/750 ms all split 0/59
    #
    # So 500 had no evidence under it and 300 costs nothing on this corpus, while returning
    # 200 ms of pure latency.
    #
    # ⚠️★★ What that corpus does NOT establish, stated plainly because the number looks
    # safer than it is:
    #   - It is ONE speaker reading prompts. Field speech has hesitations (前进...呃...三米)
    #     that read-aloud speech does not.
    #   - 99 Q-U53-3 has this parameter open and marked 阻塞验收, with V5's measured 750 ms
    #     on record and the instruction 「现场若误切句则调回 750 ms」.
    #   - ★★★ Truncation is NOT uniformly safe. 「停止前进」 clipped to 「停止」 is harmless;
    #     「前进三米」 clipped to 「前进」 is an UNBOUNDED forward command. Mid-command
    #     truncation must therefore be an explicit check item in the field acoustic test,
    #     not something inferred from an aggregate split rate.
    #
    # Set from 500 on authorization 2026-08-03 with the measurement above.
    vad_stop_ms: int = 300
    # Audio kept from BEFORE the start trigger. Without it the opening of every utterance --
    # exactly the consonant that distinguishes 前进 from 前近 -- would be missing from what
    # ASR receives, because those frames were consumed proving that speech had begun.
    #
    # ★ CALIBRATED 2026-08-03. The planned 300 ms was far too short, and the symptom was
    # unmistakable once looked for: a live 功能1 session heard 你好 as 好 and 你能做什么 as
    # 能做什么, and replaying the 59 human recordings through this very segmenter reproduced
    # it on 43 of them -- 退出广播 -> 出广播, 重启系统 -> 启系统, 不对 -> 对 -- while the
    # same clips handed to ASR whole transcribed correctly. The loss was never in the
    # recognizer; it was that the segment began after the first syllable.
    #
    # Measured agreement between "segment" and "whole clip", sweeping only this knob:
    #     300 ms  27%      500 ms  95%      800 ms  98%
    #     400 ms  88%      600 ms  95%     1000 ms  97%
    #
    # ★★ Why 800 and not the smallest value that works: the failure is asymmetric in the
    # same way 16 §4.1 describes for the estop path. An over-long preroll makes the
    # recognizer occasionally append a syllable (急停 -> 急停嗯), which costs nothing --
    # the bypass matcher does a SUBSTRING search, so 急停 is still found. A short preroll
    # removes the first character, and 急停 with its first character gone is not an estop
    # at all. Extra context is recoverable; missing onset is not.
    #
    # Cost is negligible and bounded by construction: 800 ms of 16 kHz mono s16le is 25.6 kB
    # held in a fixed-length ring, and the extra audio costs about 34 ms of decode at the
    # measured RTF of 0.043. It adds NO response latency -- the preroll is audio already
    # captured, so the trigger fires at the same instant either way.
    #
    # ⚠️ The knob is not monotone: 1000 ms scored slightly worse than 800, because leading
    # silence is itself something these models will invent text from. Do not raise it
    # further without re-running the sweep.
    vad_preroll_ms: int = 800
    # Utterances shorter than this are discarded unrecognized. A burst that clears the
    # threshold for a tenth of a second is a noise transient, and sending it to ASR costs a
    # decode and risks a hallucinated command.
    vad_min_utterance_ms: int = 300
    # Hard stop for a single utterance, so continuous noise (or a stuck-open mic) cannot
    # grow one segment without bound and starve the turn loop forever.
    vad_max_utterance_ms: int = 15000

    # -- speak phase ---------------------------------------------------------
    # Device TTS voice: 0 male, 1 female, matching the [31] frame's first payload byte and
    # payload-service's POST /tts voice field.
    tts_voice: int = 0
    # Added to the est_ms that POST /tts returns before the mic gate reopens. The device
    # emits no TTS-done event, so the end of speech can only be estimated; this margin
    # covers the estimate being short. It is deliberately generous because the two failure
    # modes are not symmetric: reopening early makes the robot transcribe its own voice and
    # answer itself, while reopening late only costs a moment of extra silence. The plan
    # requires this to be calibrated against the real device (section 7).
    tts_gate_margin_ms: int = 700
    # Upper bound on the reply text handed to the device's TTS. The [31] frame carries the
    # whole string and the device speaks all of it, so an over-long reply is an utterance
    # that cannot be interrupted; truncating here bounds the worst case.
    reply_max_chars: int = 120

    # -- 功能2 intercom ------------------------------------------------------
    # Where the intercom server listens for office-client (plan section 5.5). This is the
    # ONE listener in this system that must accept an off-box connection: 远场对讲 means
    # the operator is in the office and the robot is not, so a loopback bind would make the
    # feature impossible. That is why it differs from payload-service and asr-service,
    # which pin themselves to the loopback precisely because nothing off-box should reach
    # them. Override to a specific interface address to narrow it.
    #
    # There is no authentication on this socket. It carries live microphone audio in both
    # directions and can make the robot's loudspeaker say anything, so it belongs on a
    # trusted lab network only -- treat exposing it as equivalent to leaving the payload's
    # speaker unattended.
    intercom_host: str = "0.0.0.0"
    # 18082 continues the block: 18080 payload-service, 18081 asr-service.
    intercom_port: int = 18082
    # How long the robot's microphone stays gated after the operator releases PTT.
    #
    # Releasing closes the /play socket, which makes payload-service flush and send [11],
    # but the sound already leaving the loudspeaker keeps reaching the microphone a few
    # centimetres away. Forwarding immediately would send the operator the decaying tail of
    # their own voice. This is the 功能2 counterpart of tts_gate_margin_ms and is asymmetric
    # in the same way: too long costs a moment of missed reply, too short means the operator
    # hears themselves, so the default errs long.
    #
    # 300 ms is a starting point, NOT a measurement -- it has to be calibrated against the
    # real device, and against however far apart the microphone and speaker end up mounted.
    intercom_turnaround_ms: int = 300

    @classmethod
    def from_env(cls) -> "AiRuntimeConfig":
        """Build a configuration from the environment, falling back to the defaults.

        Returns:
            An AiRuntimeConfig whose every field is either the documented default or the
            parsed environment override.

        Raises:
            AiRuntimeConfigError: if any override is set but cannot be parsed.

        This is the ONLY place the environment is read. Keeping it in one classmethod is
        what lets a test construct a config directly, with no environment involved, and
        still exercise the same object the real process runs on.
        """
        return cls(
            payload_url=_env_str(_ENV_PAYLOAD_URL, cls.payload_url),
            asr_url=_env_str(_ENV_ASR_URL, cls.asr_url),
            llm_url=_env_str(_ENV_LLM_URL, cls.llm_url),
            llm_model=_env_str(_ENV_LLM_MODEL, cls.llm_model),
            llm_max_tokens=_env_int(_ENV_LLM_MAX_TOKENS, cls.llm_max_tokens),
            llm_temperature=_env_float(_ENV_LLM_TEMPERATURE, cls.llm_temperature),
            llm_system_prompt=_env_str(_ENV_LLM_SYSTEM_PROMPT, cls.llm_system_prompt),
            http_timeout_s=_env_float(_ENV_HTTP_TIMEOUT_S, cls.http_timeout_s),
            mic_source=_env_str(_ENV_MIC_SOURCE, cls.mic_source),
            mic_alsa_device=_env_str(_ENV_MIC_ALSA_DEVICE, cls.mic_alsa_device),
            mic_open_timeout_s=_env_float(_ENV_MIC_OPEN_TIMEOUT_S, cls.mic_open_timeout_s),
            vad_backend=_env_str(_ENV_VAD_BACKEND, cls.vad_backend),
            vad_silero_model=_env_str(_ENV_VAD_SILERO_MODEL, cls.vad_silero_model),
            vad_silero_threshold=_env_float(
                _ENV_VAD_SILERO_THRESHOLD, cls.vad_silero_threshold
            ),
            vad_threshold=_env_float(_ENV_VAD_THRESHOLD, cls.vad_threshold),
            vad_start_ms=_env_int(_ENV_VAD_START_MS, cls.vad_start_ms),
            vad_stop_ms=_env_int(_ENV_VAD_STOP_MS, cls.vad_stop_ms),
            vad_preroll_ms=_env_int(_ENV_VAD_PREROLL_MS, cls.vad_preroll_ms),
            vad_min_utterance_ms=_env_int(_ENV_VAD_MIN_UTTERANCE_MS, cls.vad_min_utterance_ms),
            vad_max_utterance_ms=_env_int(_ENV_VAD_MAX_UTTERANCE_MS, cls.vad_max_utterance_ms),
            tts_voice=_env_int(_ENV_TTS_VOICE, cls.tts_voice),
            tts_gate_margin_ms=_env_int(_ENV_TTS_GATE_MARGIN_MS, cls.tts_gate_margin_ms),
            reply_max_chars=_env_int(_ENV_REPLY_MAX_CHARS, cls.reply_max_chars),
            intercom_host=_env_str(_ENV_INTERCOM_HOST, cls.intercom_host),
            intercom_port=_env_int(_ENV_INTERCOM_PORT, cls.intercom_port),
            intercom_turnaround_ms=_env_int(
                _ENV_INTERCOM_TURNAROUND_MS, cls.intercom_turnaround_ms
            ),
        )
