"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: config.py
Brief: Centralized, environment-overridable configuration for payload-service.

Description:
  payload-service is the sole owner of the GZH-2 device TCP links (8519 audio,
  8529 lights) per the test-system development plan section 10. This module is the
  one place configuration lives, so that the rest of the service never reaches into
  os.environ directly and every tunable has exactly one authoritative default.

  Design rationale:
    - A single frozen dataclass (PayloadConfig) is passed by value to every
      component, so there is no global mutable state and no import-time side
      effect; a test can construct a config with any values it likes.
    - from_env() is the ONLY boundary that reads the process environment. Keeping
      that translation in one function means the "which env var maps to which
      field" knowledge is not scattered across the codebase.
    - Every field is overridable via environment so the identical code image runs
      on the Orin (production single-box deploy) and on a developer bench, with no
      source edits between the two -- only the environment differs.

  Scope discipline (house V1/V2 rule): only the fields the milestones so far reached
  actually consume are defined here. Milestone M3 adds the three TTS timing-estimate
  knobs (development plan sections 7 and 10) because POST /tts now needs them to return
  est_ms; milestone M4 adds deter_tts_text because the 功能3 deter loop needs a line to
  speak and POST /deter carries no text field. The ASR base URL and LLM base URL from
  section 10 are still intentionally absent -- the functional-mode milestone that first
  dials those services will add them -- so this file never carries a speculative, unused
  config switch.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

# Environment variable names are declared once here as module constants rather than
# repeated as string literals at each lookup site. The reason is failure isolation:
# if a name is ever misspelled, the typo exists in exactly ONE place, so the field
# default and the env lookup can never silently disagree (which would make an
# override look like it "does nothing" for a caller who set the correctly spelled
# variable). The values match the keys documented in development plan section 10.
_ENV_DEVICE_HOST = "DEVICE_HOST"
_ENV_PORT_AUDIO = "PORT_AUDIO"
_ENV_PORT_LIGHTS = "PORT_LIGHTS"
_ENV_PORT_PAYLOAD = "PORT_PAYLOAD"
_ENV_SOCKET_TIMEOUT_S = "SOCKET_TIMEOUT_S"
_ENV_CLOSE_FLUSH_MS = "CLOSE_FLUSH_MS"
# TTS playback-time estimate knobs (development plan section 10). The env names are the
# exact keys the plan documents -- BASE/PER_CHAR/TAIL -- kept verbatim so an operator's
# section-10 override lands on the right field.
_ENV_TTS_EST_BASE_MS = "TTS_EST_BASE_MS"
_ENV_TTS_EST_PER_CHAR_MS = "PER_CHAR_MS"
_ENV_TTS_EST_TAIL_MS = "TAIL_MS"
# The deterrent line 功能3 (deter mode) speaks on a loop. POST /deter carries no text
# field (development plan section 5.1), so the utterance is configuration, not request
# data -- one operator-overridable default that the deter loop reads at start.
_ENV_DETER_TTS_TEXT = "DETER_TTS_TEXT"
_ENV_DETER_TTS_PAUSE_MS = "DETER_TTS_PAUSE_MS"


class ConfigError(ValueError):
    """Raised when an environment override cannot be parsed into its typed field.

    Why a dedicated exception (house rule bans bare Exception): startup config
    errors are operator errors (a bad env value), categorically different from a
    runtime device fault, and callers/log scrapers benefit from being able to tell
    them apart. Subclassing ValueError keeps it semantically a "bad value" while
    still being catchable as the specific ConfigError type.

    Why fail loud at startup instead of falling back to the default: a mistyped
    PORT_* or timeout that silently reverted to a default would resurface much
    later as a confusing "connection refused" deep inside device_link, far from the
    real cause. Refusing to start points the operator straight at the bad variable.
    """


def _env_int(name: str, default: int) -> int:
    """Read an integer environment override, or return default if unset.

    Args:
        name: the environment variable name to read.
        default: the value to use when the variable is absent.

    Returns:
        The parsed integer, or default when the variable is not set.

    Raises:
        ConfigError: when the variable is set but not a valid integer.

    The raw int() ValueError is re-raised as a ConfigError that names the offending
    variable, because int()'s own message ("invalid literal for int() ...") does
    not say WHICH configuration key was wrong, which is the first thing an operator
    needs to know.
    """
    raw = os.environ.get(name)
    # Absent variable is the normal case: fall back to the documented default.
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        # Chain the original error (from exc) so the traceback still shows the
        # underlying int() failure while the top-level message names the key.
        raise ConfigError(f"env {name}={raw!r} is not an integer") from exc


def _env_float(name: str, default: float) -> float:
    """Read a float environment override, or return default if unset.

    Args:
        name: the environment variable name to read.
        default: the value to use when the variable is absent.

    Returns:
        The parsed float, or default when the variable is not set.

    Raises:
        ConfigError: when the variable is set but not a valid float.

    This is a near-duplicate of _env_int but kept separate on purpose: sharing one
    generic parser would force a less precise error message ("not a number"),
    whereas the operator is better served by being told the field wanted a float
    specifically. The tiny duplication is cheaper than that lost precision.
    """
    raw = os.environ.get(name)
    # Absent variable is the normal case: fall back to the documented default.
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError as exc:
        # Chain the original error so both the key name and the parse cause survive.
        raise ConfigError(f"env {name}={raw!r} is not a float") from exc


@dataclass(frozen=True)
class PayloadConfig:
    """Immutable runtime configuration for payload-service.

    Frozen for two reasons: (1) it is shared across request handlers and background
    threads, and an accidental mutation from any of them would be a data race that
    is painful to reproduce; (2) configuration is conceptually a startup snapshot,
    so if the environment ever needs re-reading the correct action is to build a
    fresh instance via from_env(), not to poke fields on the live one.

    The class-level assignments below double as the authoritative defaults: because
    the dataclass stores them as class attributes, from_env() can read them back as
    cls.<field> to supply the fallback, so the defaults are written exactly once.
    """

    # Device address plus the two raw device TCP ports. 8519 carries Opus audio
    # (record uplink and hail/TTS downlink); 8529 carries the 8D-framed lights and
    # status protocol. The IP and both ports are field-verified on real hardware.
    device_host: str = "192.168.144.38"
    port_audio: int = 8519
    port_lights: int = 8529

    # Local REST/WS listen port for this service. The loopback bind itself is
    # enforced in app.py (not here), but this port number must match the base URL
    # that AI_runtime's payload client dials, so it is centralized as config.
    port_payload: int = 18080

    # Connect timeout (seconds) for the two device sockets. Deliberately modest so a
    # powered-off or unplugged device surfaces as a quick failure that the reconnect
    # loop can retry, rather than a multi-minute hang that looks like a wedge.
    socket_timeout_s: float = 5.0

    # Grace period (milliseconds) that device_link waits before closing a device
    # socket. The device pushes 0x25 status about once a second, so a socket almost always
    # holds unread receive data; on Linux, close() with unread rx triggers a TCP
    # RST, and on real hardware that RST was observed to drop the device's final
    # frame (leaving the searchlight stuck on). device_link drains rx and waits this
    # long so the teardown is a clean FIN instead. See development plan section 9.
    close_flush_ms: int = 200

    # TTS playback-time estimate parameters (development plan sections 7 and 10). The
    # device gives NO "TTS finished" event, so POST /tts must RETURN an estimate that
    # AI_runtime uses to time its mic gate. The estimate is
    #   est_ms = max(base, char_count * per_char) + tail
    # where char_count is len(text) in characters (not UTF-8 bytes -- speaking rate
    # tracks characters, so a two-character Chinese word counts as two). base is a floor
    # for very short utterances, per_char is the per-character speaking cost, and tail is
    # fixed slack for the device's start/stop latency.
    #
    # * CALIBRATED 2026-08-03 against the real device, replacing the planned 800/180/500.
    # Method: record continuously on the Orin's USB microphone while firing POST /tts, then
    # locate the playback in the capture by energy. Four utterances, 2 to 26 characters:
    #
    #   spoken rate           232 - 240 ms/char   (the planned 180 was ~25% low)
    #   first audio           357 - 447 ms after the request returns  (M-PAY-2, was unknown)
    #   old estimate covered  80 - 83% of what the gate actually needs, on every sentence
    #
    # ** Why the first-audio delay belongs in `tail`: the mic gate starts when the REQUEST
    # is sent, but the device is silent for roughly 400 ms after that while it synthesizes.
    # The old tail (500 ms) was sized for "start/stop latency" alone, so the estimate ran
    # out about 1.2 s before the longest utterance finished -- the gate would reopen the
    # microphone while the robot was still speaking, and the robot would record its own
    # tail. 16 §3.0.4.1 calls that the failure to avoid, and notes AEC cannot back it up
    # here because the upstack never holds the played waveform.
    #
    # * Direction of error is deliberate: 250 ms/char is above the measured 240, and the
    # tail covers the worst observed first-audio delay plus margin, so every measured case
    # is covered with 15-25% to spare. Over-estimating costs response latency; under-
    # estimating costs the half-duplex property itself, which is why 16 §3.0.4.1 permits
    # adjusting this value upward only.
    tts_est_base_ms: int = 800
    tts_est_per_char_ms: int = 250
    tts_est_tail_ms: int = 900

    # Deterrent utterance for 功能3 (deter mode), spoken on a loop in the male voice
    # (development plan section 1: 红蓝双闪 + 警笛 + 男声TTS). POST /deter has no text
    # field, so this config value IS the line -- 99 Q-C29-1 makes DETER_TTS_TEXT the single
    # source of truth precisely so the wording can change without a code change. The wording
    # below is the operator's (甲方) own, which is what Q-C29-1 was waiting on; note it says
    # 管控 rather than the 管制 of the earlier probe tool and 警戒 of the first draft.
    #
    # * "|" separates SPOKEN SEGMENTS, not words. Each segment is sent as its own [31] and
    # the device speaks them one after another, so the gap between segments is a real pause
    # under our control (deter_tts_pause_ms) rather than something we hope the device's TTS
    # infers from punctuation -- we have no evidence it treats punctuation as timing at all.
    # Within a segment, ASCII spaces stand in for punctuation (the house rule bans Chinese
    # punctuation in source). A line with no "|" is simply one segment, as before.
    deter_tts_text: str = "警告|你已进入管控区域 请立即离开"

    # * The audible beat after each mid-sentence segment -- the pause after 警告. Because
    # the device's start latency cancels between two consecutive segments (see
    # estimate_segment_gap_ms), this value IS very nearly the silence the operator hears,
    # not a margin on top of one. Tuned by ear on the real unit.
    deter_tts_pause_ms: int = 500

    @classmethod
    def from_env(cls) -> "PayloadConfig":
        """Build a PayloadConfig from process environment overrides.

        Returns:
            A fully populated, immutable PayloadConfig. Any field whose environment
            variable is unset takes the class default.

        Raises:
            ConfigError: if any set variable fails to parse as its field type.

        This is the single boundary between the untyped environment and the typed
        config object. Every field is overridable so the same code runs unchanged on
        the Orin and on a bench; the string field is read directly while the numeric
        fields go through the validating helpers above.
        """
        return cls(
            # String field: no parsing needed, so read straight from the environment.
            device_host=os.environ.get(_ENV_DEVICE_HOST, cls.device_host),
            # Numeric fields: validated via the helpers so a bad value fails loudly.
            port_audio=_env_int(_ENV_PORT_AUDIO, cls.port_audio),
            port_lights=_env_int(_ENV_PORT_LIGHTS, cls.port_lights),
            port_payload=_env_int(_ENV_PORT_PAYLOAD, cls.port_payload),
            socket_timeout_s=_env_float(_ENV_SOCKET_TIMEOUT_S, cls.socket_timeout_s),
            close_flush_ms=_env_int(_ENV_CLOSE_FLUSH_MS, cls.close_flush_ms),
            # TTS estimate knobs: integer milliseconds, validated like the other ints.
            tts_est_base_ms=_env_int(_ENV_TTS_EST_BASE_MS, cls.tts_est_base_ms),
            tts_est_per_char_ms=_env_int(_ENV_TTS_EST_PER_CHAR_MS, cls.tts_est_per_char_ms),
            tts_est_tail_ms=_env_int(_ENV_TTS_EST_TAIL_MS, cls.tts_est_tail_ms),
            # String field like device_host: read straight from the environment, no parse.
            deter_tts_text=os.environ.get(_ENV_DETER_TTS_TEXT, cls.deter_tts_text),
            deter_tts_pause_ms=_env_int(_ENV_DETER_TTS_PAUSE_MS, cls.deter_tts_pause_ms),
        )

    def estimate_tts_ms(self, text: str) -> int:
        """Estimate how long the device will take to speak text, in milliseconds.

        Args:
            text: the utterance that POST /tts will send to the device.

        Returns:
            The estimated playback duration in milliseconds:
                max(base, char_count * per_char) + tail
            using this config's three knobs.

        The estimate lives here, next to the knobs it reads, so the formula and its
        parameters cannot drift apart and so it can be unit-tested without standing up
        the REST layer. It exists because the device emits NO "TTS finished" event
        (development plan section 9): AI_runtime cannot listen for completion, so it
        instead waits this estimate (plus its own margin) before reopening the mic gate,
        which is why POST /tts returns the value.

        char_count is len(text) -- the number of characters, not UTF-8 bytes -- because
        speaking time tracks characters spoken: a two-character Chinese word takes about
        as long as two characters regardless of its six-byte UTF-8 encoding. The max()
        applies base as a floor so a very short utterance still reserves a sane minimum,
        and tail adds fixed slack for the device's own start/stop latency.
        """
        char_count = len(text)
        return max(self.tts_est_base_ms, char_count * self.tts_est_per_char_ms) + self.tts_est_tail_ms

    def estimate_segment_gap_ms(self, text: str) -> int:
        """Estimate when a MID-SENTENCE segment has finished, plus the deliberate pause.

        Args:
            text: one spoken segment (a "|"-separated piece of deter_tts_text).

        Returns:
            Milliseconds to wait after sending this segment before sending the next:
                char_count * per_char + tail + deter_tts_pause_ms

        * Why this is NOT estimate_tts_ms. That method exists to gate the MICROPHONE: it
        must never reopen early, so it floors the estimate at tts_est_base_ms and a caller
        that waits too long merely loses a moment of listening. Here the same slack is
        audible -- it becomes dead air in the middle of a sentence. On a two-character
        segment like 警告 the 800 ms floor dominates the ~500 ms of actual speech and
        stretched the pause to a measured 1.66 s, which reads as two unrelated sentences
        rather than one warning with a beat.

        tail is dropped too, and that is not a shortcut -- it CANCELS. The device stays
        silent for a start latency after each request, so this segment is heard from
        start+0 to start+speech, and the next is heard from W+start onward. The audible
        silence is (W + start) - (start + speech) = W - speech: the latency appears on both
        sides and falls out. What remains is the speech itself plus the beat we want, which
        is exactly what the operator hears and therefore the only thing worth tuning.

        Keeping tail instead made the measured pause after 警告 1.66 s. char_count *
        per_char is itself conservative (250 ms/char against a measured 232-240), so a
        deter_tts_pause_ms of zero would still not send the next segment early; the pause
        is headroom on top of headroom.
        """
        return len(text) * self.tts_est_per_char_ms + self.deter_tts_pause_ms
