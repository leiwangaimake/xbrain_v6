"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: config.py
Brief: Centralized, environment-overridable configuration for asr-service.

Description:
  asr-service is a pure inference service: it turns audio into text and knows nothing
  about the GZH-2 device (development plan sections 4 and 5.3). This module is the one
  place its configuration lives, so no other module reads os.environ and every tunable
  has exactly one authoritative default.

  Design rationale (deliberately the same shape as services/payload/config.py):
    - A single frozen dataclass (AsrConfig) is passed by value to every component, so
      there is no global mutable state and no import-time side effect; a test can build
      a config with any values it likes and never touch the process environment.
    - from_env() is the ONLY boundary that reads the environment, so the "which env var
      maps to which field" knowledge is not scattered across the codebase.
    - Every field is overridable, so the identical code image runs on the Orin and on a
      developer bench with only the environment differing.

  Why the model and hotword paths default to locations DERIVED from this file rather
  than absolute strings: services/asr/ ships as one self-contained tree (the model
  directories and hotwords.txt sit beside this module), so deriving the defaults from
  __file__ keeps the whole service relocatable -- exactly the property
  services/llm/llm_server.sh gets from its SCRIPT_DIR. A hardcoded /opt/xbrain_v6/...
  default would silently break the moment the tree is copied elsewhere, and would make a
  test fixture impossible to point away from the real multi-hundred-megabyte model.

  Why a model FAMILY is configuration and not a constant: choosing an ASR model is an
  empirical question, and the measurement that settles it has not finished -- the current
  default was picked on synthesized speech, and it has to be re-run against real recordings
  from the deployed USB microphone. A family field means that re-run, and any A/B it
  provokes, is an environment variable rather than a patch. The accepted names and the
  capabilities behind them live in core/families.py.

  Scope discipline (house V1/V2 rule): only fields the current milestone consumes exist
  here. There is no streaming-endpoint knob, because the WS /v1/audio/stream upgrade is
  explicitly deferred in plan section 5.3, and no ASR_BASE_URL, because that key belongs
  to the CLIENT side (AI_runtime dialing this service), not to the server itself.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

from .core import families

# Environment variable names, declared once as module constants rather than repeated as
# string literals at each lookup site. If a name is ever misspelled the typo then exists
# in exactly ONE place, so a field default and its env lookup can never silently
# disagree -- which would make an operator's correctly spelled override look inert.
_ENV_PORT_ASR = "PORT_ASR"
_ENV_MODEL_FAMILY = "ASR_MODEL_FAMILY"
_ENV_MODEL_DIR = "ASR_MODEL_DIR"
_ENV_HOTWORDS_FILE = "ASR_HOTWORDS_FILE"
_ENV_HOTWORDS_SCORE = "ASR_HOTWORDS_SCORE"
_ENV_NUM_THREADS = "ASR_NUM_THREADS"
_ENV_INT8 = "ASR_INT8"
_ENV_MIN_AUDIO_MS = "ASR_MIN_AUDIO_MS"
_ENV_REQUIRE_SAMPLE_RATE = "ASR_REQUIRE_SAMPLE_RATE"

# Directory holding this module, used to derive the defaults below. Computed once at
# import so the paths are absolute regardless of the caller's working directory.
_SERVICE_DIR = os.path.dirname(os.path.abspath(__file__))

# Where each family's weights live when ASR_MODEL_DIR is not set. A family and a directory
# are not independent -- a paraformer directory holds no joiner and a transducer directory
# holds no am.mvn -- so pairing them here means the ONE default an operator gets is a pair
# that actually loads, and overriding the family alone cannot silently point a new factory
# at the old files.
#
# Both directories are named model-<export> rather than one of them being a bare `model/`:
# the reported model identity is derived from this basename (recognizer.py), so a directory
# that names its export makes /status say which weights are loaded, and a flat `model/`
# could only report the family. Symmetry also removes the question of which family "owns"
# the unqualified name.
# ★ Paths resolve through 11 §11A.4.1's layout: <root>/<name>/<ver>/ with a `current`
# symlink beside the version directories. Pointing at `current` rather than at `1.0.0` is
# the whole mechanism -- §11A.4.1 defines rollback as 「改软链 + 重启单元」, and a config
# that named the version directly would make that flip a code change.
#
# ⚠️ The ROOT here is services/asr/models/, not the contract's /opt/xbrain/models/.
# Reconciling those two roots is DEC-15 (仓库布局), whose decider is the main session; the
# structure below is the contract's, and moving it to a decided root is a mv plus this line.
_MODEL_ROOT = os.path.join(_SERVICE_DIR, "models")
_FAMILY_MODEL_DIRS = {
    "transducer": os.path.join(_MODEL_ROOT, "zipformer-multi-zh-hans", "current"),
    "paraformer": os.path.join(_MODEL_ROOT, "paraformer-zh-2023-09", "current"),
}

# Default family. Measured 2026-08-03 against eight alternatives on one corpus synthesized
# from the `18` command set: paraformer-zh-2023-09 reached 53.1% exact / 22.7% CER where the
# previously deployed multi-zh-hans transducer reached 40.1% / 30.5%, at RTF 0.050 against a
# requirement of RTF < 0.5. The transducer is the only biasable family, and biasing was
# measured to be worth about one point once its vocabulary gap is closed -- an order of
# magnitude less than the accuracy difference, which is why accuracy wins the default.
_DEFAULT_FAMILY = "paraformer"

# Strings accepted as boolean true/false for ASR_INT8. Spelled out as sets (rather than
# "value != '0'") so a typo like ASR_INT8=flase is REJECTED instead of being read as
# true -- a silent misread here would swap the model precision and quietly change RTF.
_TRUE_WORDS = frozenset({"1", "true", "yes", "on"})
_FALSE_WORDS = frozenset({"0", "false", "no", "off"})


class AsrConfigError(ValueError):
    """Raised when an environment override cannot be parsed into its typed field.

    House rule bans bare Exception. A dedicated type also separates operator errors (a
    bad env value at startup) from runtime inference faults, which callers and log
    scrapers need to tell apart. It subclasses ValueError because every case is a bad
    value; failing loudly at startup beats silently reverting to a default, which would
    resurface much later as an unexplained slowdown or a missing hotword effect.
    """


def _env_int(name: str, default: int) -> int:
    """Read an integer environment override, or return default if unset.

    Args:
        name: the environment variable name to read.
        default: the value to use when the variable is absent.

    Returns:
        The parsed integer, or default when the variable is not set.

    Raises:
        AsrConfigError: when the variable is set but is not a valid integer.

    int()'s own message does not say WHICH key was wrong, which is the first thing an
    operator needs, so it is re-raised with the offending variable named.
    """
    raw = os.environ.get(name)
    # Absent variable is the normal case: fall back to the documented default.
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        # Chain the original error so the traceback keeps the underlying parse failure.
        raise AsrConfigError(f"env {name}={raw!r} is not an integer") from exc


def _env_float(name: str, default: float) -> float:
    """Read a float environment override, or return default if unset.

    Args:
        name: the environment variable name to read.
        default: the value to use when the variable is absent.

    Returns:
        The parsed float, or default when the variable is not set.

    Raises:
        AsrConfigError: when the variable is set but is not a valid float.
    """
    raw = os.environ.get(name)
    # Absent variable is the normal case: fall back to the documented default.
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError as exc:
        # Chain the original error so both the key name and the parse cause survive.
        raise AsrConfigError(f"env {name}={raw!r} is not a float") from exc


def _env_bool(name: str, default: bool) -> bool:
    """Read a boolean environment override, or return default if unset.

    Args:
        name: the environment variable name to read.
        default: the value to use when the variable is absent.

    Returns:
        The parsed boolean, or default when the variable is not set.

    Raises:
        AsrConfigError: when the variable is set to anything outside the accepted words.

    Case and surrounding whitespace are ignored so "True" and " on " both work, but an
    unrecognised word is an error rather than a silent false -- see _TRUE_WORDS.
    """
    raw = os.environ.get(name)
    # Absent variable is the normal case: fall back to the documented default.
    if raw is None:
        return default
    word = raw.strip().lower()
    if word in _TRUE_WORDS:
        return True
    if word in _FALSE_WORDS:
        return False
    raise AsrConfigError(f"env {name}={raw!r} is not a boolean (use 1/0, true/false, yes/no, on/off)")


def _env_choice(name: str, default: str, allowed: tuple) -> str:
    """Read an environment override constrained to a closed set, or return default.

    Args:
        name: the environment variable name to read.
        default: the value to use when the variable is absent.
        allowed: every accepted value, listed in the error message when one misses.

    Returns:
        The chosen value, guaranteed to be a member of allowed.

    Raises:
        AsrConfigError: when the variable is set to anything outside allowed.

    An unrecognised value is an error rather than a silent fall back to default, for the
    same reason ASR_INT8=flase is rejected above: quietly loading a different architecture
    than the operator asked for produces an accuracy change with no visible cause.
    """
    raw = os.environ.get(name)
    # Absent variable is the normal case: fall back to the documented default.
    if raw is None:
        return default
    word = raw.strip()
    if word not in allowed:
        raise AsrConfigError(
            f"env {name}={raw!r} is not one of {', '.join(allowed)}"
        )
    return word


@dataclass(frozen=True)
class AsrConfig:
    """Immutable runtime configuration for asr-service.

    Frozen for the same two reasons as PayloadConfig: it is shared across request
    handlers and the decode worker thread, where an accidental mutation would be a hard
    to reproduce data race; and configuration is conceptually a startup snapshot, so
    re-reading the environment means building a fresh instance, never poking fields on
    the live one.

    The class-level assignments below double as the authoritative defaults: the
    dataclass stores them as class attributes, so from_env() reads them back as
    cls.<field> for its fallbacks and each default is written exactly once.
    """

    # Local REST listen port for this service. The loopback bind itself is enforced in
    # app.py (not here); this number must match the ASR_BASE_URL that AI_runtime's
    # asr_client dials (development plan section 10), so it is centralized as config.
    # 18081 sits next to payload-service's 18080 and clear of llama-server's 8080.
    port_asr: int = 18081

    # Which model architecture to load; see core/families.py for the accepted names and for
    # what each family can do. It is configuration rather than a constant because choosing
    # a model is a measurement, and the next measurement (against real recorded speech
    # rather than synthesized audio) has to be able to move it without a code change.
    model_family: str = _DEFAULT_FAMILY

    # Directory holding the model export for model_family: the network ONNX file(s) plus
    # tokens.txt. Derived from this file's location so the service tree stays relocatable;
    # recognizer.py turns this directory, the family and use_int8 into concrete file paths
    # and checks that every one of them exists before building anything.
    model_dir: str = _FAMILY_MODEL_DIRS[_DEFAULT_FAMILY]

    # Hotword vocabulary file (contextual biasing). It ships beside this module and is
    # hand-maintained, so it is data, not code -- see core/hotwords.py for the format
    # and for why the file the engine finally reads is a normalized temporary copy.
    hotwords_file: str = os.path.join(_SERVICE_DIR, "hotwords.txt")

    # How hard to bias decoding toward the hotwords. Larger values make a listed phrase
    # more likely to win over an acoustically similar unlisted one, at the cost of more
    # false hits on audio that never contained it. 2.0 is sherpa-onnx's own documented
    # starting point and is the value the bench script measures against; treat it as a
    # field-tunable knob rather than a constant, since the right level depends on how
    # noisy the near-field pickup turns out to be on the boat.
    hotwords_score: float = 2.0

    # ONNX Runtime intra-op threads per decode.
    #
    # ★★★ 1, not 4, and the reason is counter-intuitive enough to be worth the paragraph:
    # 11 AIR-P3 pins this service to CPUAffinity=7, a SINGLE core shared with P4/P5/HMI.
    # Four threads timesharing one core do not merely serialize -- they contend. Measured
    # on the Orin, decode of the same audio pinned to core 7:
    #
    #     threads=4  p50 616 ms  rtf 0.452      threads=2  326 ms  0.237
    #     threads=1  p50 166 ms  rtf 0.119   <- 3.7x FASTER than 4
    #
    # 8.5x slower than unpinned is worse than the 4x pure serialization would predict, and
    # the gap closes entirely at one thread, which is what identifies it as contention.
    #
    # ★ Why it matters beyond tidiness: at threads=4 the decode of a 3 s utterance costs
    # 1356 ms against the 350 ms that 00 §4.10.6 ⑧ budgets for this stage -- 3.9x over, and
    # enough on its own to miss CMD-60 (「说完最后一个字到底盘产生位移 P95 <= 2.0 s」) before
    # the chassis has moved at all. At threads=1 it costs 357 ms, essentially the budget.
    # rtf 0.452 also sits against the AS requirement of rtf < 0.5 with nothing to spare,
    # on a core that 10 §3.2 gives to four other consumers.
    #
    # ⚠️ The old comment here said 4 「leaves headroom」 for the neighbours and that raising
    # it is 「the first knob to try if RTF is at risk」. Both are backwards under AIR-P3:
    # more threads take MORE of the shared core and make RTF worse. Raising this is only
    # ever correct if the affinity pin is removed, which would itself violate AIR-P3.
    #
    # ★ 11 §14.2 F-23 lists 线程数 in the NOT-frozen column, so this value is a tuning
    # decision (PR + notification), not a freeze-gate item. Changed on authorization
    # 2026-08-03 with the measurement above.
    num_threads: int = 1

    # Use the int8-quantised model files when true. int8 is markedly faster than fp32 on
    # the Orin's CPU at a negligible accuracy cost for this model, and RTF < 0.5 is a
    # hard requirement for 功能1 (a slow transcription stalls the whole dialogue turn),
    # so the fast path is the DEFAULT and fp32 is the opt-in used to A/B accuracy.
    use_int8: bool = True

    # Shortest upload the service will hand to the engine, in milliseconds. This is an
    # ENGINE CAPABILITY FLOOR, not a speech-length policy: below roughly 100 ms the
    # zipformer's frontend convolution has fewer frames than its kernel needs and the
    # decode raises, which the REST layer could only report as a 503 -- an operational
    # fault code for what is really an unusable input, telling the caller to retry bytes
    # that can never succeed. Rejecting it at the boundary makes it a 400 instead.
    #
    # ★ Deliberately NOT raised to a "minimum utterance" value. 100 ms is well under any
    # real command -- "停" measures about 190 ms -- so this gate cannot swallow a genuine
    # one-syllable estop. Where the VAD should cut is a separate decision that belongs to
    # P2's audio_io and to field calibration, not to a floor that exists because a
    # convolution needs frames.
    min_audio_ms: int = 100

    # Sample rate the upload must declare, or 0 to accept any. 11 AS-2 fixes this interface
    # at 16 kHz and the whole audio plane produces exactly that, so a different rate is a
    # misconfigured capture chain rather than a caller variant -- and one that would
    # otherwise be silently resampled by the engine and show up only as unexplained
    # recognition errors (16 §3.0.2). Overridable to 0 for a bench replaying archived audio
    # at its original rate, where the check protects nothing.
    require_sample_rate: int = 16000

    @classmethod
    def from_env(cls) -> "AsrConfig":
        """Build an AsrConfig from process environment overrides.

        Returns:
            A fully populated, immutable AsrConfig. Any field whose environment
            variable is unset takes the class default.

        Raises:
            AsrConfigError: if any variable that IS set fails to parse as its field type.

        This is the single boundary between the untyped environment and the typed config
        object. The two path fields are read directly (any string is a legal path --
        whether it EXISTS is checked by recognizer.py, which is where a missing model is
        actionable), while the numeric and boolean fields go through the validating
        helpers above so a bad value fails at startup rather than mid-request.

        model_dir follows model_family when it is not overridden explicitly, so setting
        ASR_MODEL_FAMILY alone yields a directory that matches it. Setting ASR_MODEL_DIR
        alone likewise still works: it wins outright, which is what an operator pointing
        at a bench copy of the same family expects.
        """
        # Resolved first: the family decides which directory the default points at.
        family = _env_choice(_ENV_MODEL_FAMILY, cls.model_family, families.FAMILY_NAMES)
        model_dir = os.environ.get(_ENV_MODEL_DIR)
        if model_dir is None:
            # ★ No silent fallback. core/families.py can LOAD more architectures than this
            # tree ships weights for, and the failure that matters is specific: the file
            # stems overlap between families -- both paraformer and sense_voice look for
            # "model*.int8.onnx" plus tokens.txt -- so falling back to another family's
            # directory would not raise "file missing", it would BUILD A SENSE_VOICE
            # RECOGNIZER ON PARAFORMER WEIGHTS and serve confident nonsense. Refusing here
            # is the same rule _env_choice applies to an unknown family name, for the same
            # reason: quietly loading something other than what was asked for surfaces only
            # as an accuracy mystery.
            try:
                model_dir = _FAMILY_MODEL_DIRS[family]
            except KeyError as exc:
                raise AsrConfigError(
                    f"no model directory ships for the {family!r} family; set "
                    f"{_ENV_MODEL_DIR} to point at its export "
                    f"(families with a default: {', '.join(sorted(_FAMILY_MODEL_DIRS))})"
                ) from exc
        return cls(
            port_asr=_env_int(_ENV_PORT_ASR, cls.port_asr),
            model_family=family,
            # Resolved above: either the explicit override or this family's own export.
            model_dir=model_dir,
            # Remaining path field: no parsing needed, so read straight from the environment.
            hotwords_file=os.environ.get(_ENV_HOTWORDS_FILE, cls.hotwords_file),
            hotwords_score=_env_float(_ENV_HOTWORDS_SCORE, cls.hotwords_score),
            num_threads=_env_int(_ENV_NUM_THREADS, cls.num_threads),
            use_int8=_env_bool(_ENV_INT8, cls.use_int8),
            min_audio_ms=_env_int(_ENV_MIN_AUDIO_MS, cls.min_audio_ms),
            require_sample_rate=_env_int(_ENV_REQUIRE_SAMPLE_RATE,
                                        cls.require_sample_rate),
        )
