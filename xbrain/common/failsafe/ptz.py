"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptz.py
Brief: PTZ fail-safes for the two uncalibrated blocks -- homing (T-PTZ-1) and speed (T-PTZ-3)

Description:
The problem this solves. Two PTZ capabilities cannot be honestly offered today.
(1) 11 T-PTZ-1: whether a taught preset actually returns the head to a known
orientation is unverifiable by machine -- position read-back is a永远假值
(180, 0), so only a human eye can confirm it, and until someone does,
ptz.preset_effective is null. (2) 18 T-PTZ-3: the speed_pct -> degree/second
calibration curve (omega) is unmeasured, so "turn N degrees" cannot be timed and
"set speed to 60%" cannot be validated. This module is the fail-loud branch for
both, so that E02/E03/E04 (homing) and E09/E10/G33 (speed/relative-turn/zoom-
query) refuse or degrade in exactly one place instead of each caller improvising.

Which design sections.
  * 11 S7.4.8 -- preset_effective is null pending T-PTZ-1 and 拒绝启动 (10 S5.4.2
    R-3); when it is false, goto_preset -> rejected + E_CAPABILITY,
    detail.reason = "preset_ineffective" (11 S7.4 表, 逐字).
  * 21 S1 T-PTZ-1 row -- E02/E03/E04 一律 rejected + E_CAPABILITY; 不得写成布尔
    冒充已标定; 不得用 accepted 冒充"已到位".
  * 18-B S2 -- E09 speed levels are the closed set {slow, normal, fast, up, down};
    闭集档位而非 0-100 because voice cannot say a percentage AND T-PTZ-3 is open.
  * 18-B S3 / S8 Q-5 -- E10 直接 rejected + 引导改用 E01 三档 (依据 CLAUDE.md 3.1:
    未标定一律 null, 绝不写猜值冒充已赋值; 母本 7.2 禁令 不得偷偷折成某个档位).
  * 21 S1 T-PTZ-3 row + 18 S9.7 -- G33 只答档位, 不答角度或倍率.

What this file does NOT do. It does not compose the TTS the operator hears
("本机云台不支持归位") -- 11 S8.13.5 makes error phrasing the gateway's job, so
this layer emits a reason token and lets the gateway phrase it. It does not set
preset_effective to any value: the guard REFUSES boot while it is null, forcing a
human decision, and never substitutes true/false to get past startup. And it does
not lift either block; the 云深处 writeup / PTZ hardware measurement would only
ever make a capability AVAILABLE, which 21 S1 puts explicitly out of scope.

The looks-right-but-wrong traps, each a real prohibition:
  * accepted-means-arrived. goto_preset returning accepted proves only that the
    command was received; on this hardware "命令被接受" and "转对了" are永远不可
    区分 (21 S1). So the homing branch returns REJECTED, never accepted, and the
    mutant that swaps it is what the test catches.
  * a degree/second value for the gears. Writing omega for slow/normal/fast --
    in a doc OR in code -- fabricates the very curve T-PTZ-3 has not measured
    (CLAUDE.md 3.1 fail-silent: a guessed omega makes the operator think 30 deg
    happened when 8 or 90 did). The gear carries a NAME only; attaching a number
    is the mutant.
  * a boolean fallback for preset_effective. Defaulting null to false to "let it
    boot" is the fail-silent shape 10 S5.4.2 R-3 forbids -- it hides the fact that
    nobody has made the T-PTZ-1 call. The guard raises on null, it does not
    coerce.
"""

from dataclasses import dataclass

# MISSING distinguishes "no config layer mentioned this key" from an explicit
# null. Both must refuse boot for preset_effective, but they are different
# operator errors (key deleted vs key left null), so the guard treats them
# together yet the sentinel keeps them from being confused with a real False.
from ..config import MISSING
# Codes and the base error come from the shared library, never as literals
# (CLAUDE.md 3.5). E_CONFIG_INVALID is the startup-refusal code; E_CAPABILITY is
# the "本体能力不支持" reject; ClosedSetViolation raises E_SCHEMA for an out-of-set
# gear, which is 闭集外必抛 (11 S13.6).
from ..errors import E_CAPABILITY, E_CONFIG_INVALID
from ..errors.exceptions import ClosedSetViolation, XbrainError
from .outcome import STATUS_REJECTED, FailSafeResult

# The config key the boot guard reports. Written out so the raised message names
# the exact path an operator has to go set, the same contract 10 S5.4.2 assertion
# A follows for the safety namespaces.
PRESET_EFFECTIVE_KEY = "ptz.preset_effective"

# The homing intents that route to this block, 18-B S1:
#   E02 ptz_home / E03 ptz_preset / E04 ptz_track
# E03 is the sole dependency of the other two (11 S7.4 表), so all three fall
# together when preset_effective is false. Held as data + cited, not a magic list.
PTZ_PRESET_INTENTS = frozenset({"E02", "E03", "E04"})

# The E09 speed-level closed set, 18-B S2. normal is a real member -- the default
# gear ("恢复正常转速", 开机初值), which 99 U76(4) confirms is a normal LEVEL, not
# a default-value key. Kept here rather than in enums/sets.yaml because it is a
# voice-layer set with no C++ consumer, so it needs no cross-language export; a
# metatest binds it to 18-B S2 by symmetric difference exactly as sets.yaml does.
# No count is written here or anywhere (CLAUDE.md 3.7).
PTZ_SPEED_LEVELS = frozenset({"slow", "normal", "fast", "up", "down"})

# The detail.reason for a homing reject, 11 S7.4 表 (逐字 detail{reason:
# "preset_ineffective"}). A named constant so the three homing intents cannot
# drift into three spellings of the same cause.
REASON_PRESET_INEFFECTIVE = "preset_ineffective"

# The detail.reason for an E10 reject. E_CAPABILITY's detail is "unspecified" in
# codes.yaml, so this token is descriptive, not a closed-set member; it ties the
# reject to its true cause -- the speed curve (T-PTZ-3) is unmeasured -- so the
# gateway can phrase it and a log reader can see why. Not a guessed VALUE: it
# names the missing calibration, it does not supply one.
REASON_PTZ_SPEED_UNCALIBRATED = "ptz_speed_uncalibrated"

# The intent E10 steers the operator toward, 18-B S3 / S8 Q-5 ("引导改用 E01 三
# 档"). E01 ptz_move is the timed-pulse relative move (pulse_ms 350/1000/2800,
# 18-B S1) that works WITHOUT an omega curve. An intent id, not a sentence.
GUIDE_USE_E01 = "E01"


def require_preset_effective(value: object) -> bool:
    """Boot guard: preset_effective must be a decided bool, or the stack refuses.

    Returns the bool when the value is a real True/False. Raises E_CONFIG_INVALID,
    naming the key, when the value is null or the key is absent -- that is the
    T-PTZ-1-undecided state, and 10 S5.4.2 R-3 makes it a startup refusal so that
    nobody deploys without a human having made the call. It also raises on a non-
    bool, because a string or number in this slot is a corrupt config, not a
    decision.

    The mutant this exists to catch: coercing null to False (a boolean fallback
    "冒充已标定"). Under that mutation a null boots as "homing unavailable" and the
    forcing function is gone -- exactly the fail-silent 10 S5.4.2 R-3 forbids.
    """
    # is MISSING / is None BEFORE the bool check. isinstance(None, bool) is False,
    # so None would reach the non-bool branch anyway, but null and absent deserve
    # the startup message that names the T-PTZ-1 decision, not the generic corrupt
    # message -- they are the expected pre-calibration state, not a typo.
    if value is MISSING or value is None:
        raise XbrainError(
            E_CONFIG_INVALID,
            f"{PRESET_EFFECTIVE_KEY} is null or absent -- T-PTZ-1 undecided; set "
            f"it true or false, never leave it to a default (10 S5.4.2 R-3)",
            detail={"key": PRESET_EFFECTIVE_KEY},
        )
    # isinstance rather than "in (True, False)" so a 0/1 int -- which == False/True
    # -- is also rejected. The contract field is a bool; an int here is a config
    # that lost its type, and treating 1 as True would launder that.
    if not isinstance(value, bool):
        raise XbrainError(
            E_CONFIG_INVALID,
            f"{PRESET_EFFECTIVE_KEY} must be a bool, got {type(value).__name__}",
            detail={"key": PRESET_EFFECTIVE_KEY},
        )
    # Returns the plain bool: the caller needs the decision to pick a route
    # (true -> normal goto_preset, false -> ptz_preset_failsafe), and a heavier
    # return type would only be unwrapped again at every call site. Note true is
    # returned too -- this guard proves a decision WAS made, it does not itself
    # decide that homing works; that verdict is still human-observed (T-PTZ-1).
    return value


def ptz_preset_failsafe(intent: str) -> FailSafeResult:
    """E02/E03/E04 -> rejected + E_CAPABILITY while preset homing is unavailable.

    This is the branch taken when preset_effective is false (or, today, before
    T-PTZ-1 is decided). It ALWAYS rejects -- 21 S1 逐字 "一律 rejected" -- and it
    NEVER returns accepted, because accepted on this hardware is indistinguishable
    from "转对了" and would be a fake guarantee. The reason token lets the gateway
    answer "本机云台不支持归位" (11 S8.13.5 owns that wording, not this layer).
    """
    # Unconditional reject, and that is the whole design: the caller routes here
    # ONLY when preset homing is unavailable, so there is no "maybe" branch. It
    # deliberately does NOT take preset_effective as an argument -- a true value
    # would mean the caller should have gone to the normal goto_preset path, and
    # taking the flag here would invite this function to grow an accept branch,
    # which is precisely the accepted-冒充-已到位 shape 21 S1 forbids. intent is
    # taken only to validate the routing, never to vary the verdict.
    if intent not in PTZ_PRESET_INTENTS:
        # A non-homing intent routed here is a caller bug. Reject-by-default would
        # hide it; raise instead so the mis-route surfaces at the seam.
        raise ValueError(
            f"ptz_preset_failsafe got {intent!r}, not a homing intent "
            f"{sorted(PTZ_PRESET_INTENTS)} (18-B S1 E02/E03/E04)"
        )
    return FailSafeResult(
        status=STATUS_REJECTED,
        code=E_CAPABILITY,
        detail={"reason": REASON_PRESET_INEFFECTIVE},
    )


def ptz_move_deg_failsafe() -> FailSafeResult:
    """E10 ptz_move_deg -> rejected + E_CAPABILITY, guiding to E01's three gears.

    omega is null (T-PTZ-3), so "turn N degrees" cannot be timed. The only honest
    answer is to reject and point the operator at E01, the relative timed-pulse
    move that needs no omega. It carries NO angle and NO pulse_ms: echoing a
    guessed motion is the fail-silent path 18-B S3 forbids (拿猜的 omega 先跑), and
    18-B hard-constraint 3 forbids writing an estimated angle as a position. The
    mutant this catches is returning accepted with a computed pulse -- running on
    a guessed omega.
    """
    # No parameters at all. The direction / angle_deg the operator said are
    # discarded upstream on purpose: accepting them here would tempt a later hand
    # to "just compute a pulse from them", and there is no omega to compute with.
    # E_CAPABILITY, not E_BUSY: E_BUSY's client behaviour is backoff-retry, and a
    # retry against a curve that was never measured never clears -- the same
    # transient-vs-persistent split 12 S6A.7 RC-D5 draws for the rotation codes.
    return FailSafeResult(
        status=STATUS_REJECTED,
        code=E_CAPABILITY,
        detail={"reason": REASON_PTZ_SPEED_UNCALIBRATED},
        guidance=GUIDE_USE_E01,
    )


def parse_ptz_speed_level(level: str) -> str:
    """Return level if it is in the E09 closed set, else raise (11 S13.6).

    Out-of-set raises ClosedSetViolation (-> E_SCHEMA), never a silent pass-
    through and never a "nearest gear" degrade -- the same closed-set discipline
    the shared enums enforce. This is the single membership gate for E09.
    """
    # A named function rather than an inline `level in PTZ_SPEED_LEVELS` at each
    # call site: one membership gate is one place to raise, so an out-of-set gear
    # cannot slip past a caller that forgot to check. Same shape the shared enums
    # use -- every parse routes through a single ClosedSet.parse.
    if level not in PTZ_SPEED_LEVELS:
        raise ClosedSetViolation("ptz_speed_level", level)
    return level


@dataclass(frozen=True)
class PtzSpeedGear:
    """An E09 speed command -- a gear NAME and nothing physical.

    There is deliberately no omega / deg_per_s field. 18-B S2 and 21 S1 forbid a
    degree/second value for the gears because T-PTZ-3 has not measured the curve;
    the wire form is a label only. to_wire() is the assertion surface: it must
    stay a single-key mapping, and the mutant that adds a numeric speed is caught
    there.
    """

    level: str

    def to_wire(self) -> dict:
        """The E09 payload: exactly {"level": <gear>}.

        A fresh dict per call. The key set is the load-bearing invariant -- any
        second key (deg_per_s, omega, ...) is the "给三档写度每秒当量" mutant.
        """
        return {"level": self.level}


def resolve_ptz_speed_gear(level: str) -> PtzSpeedGear:
    """Validate an E09 level and return the gear, carrying only its name."""
    # parse first, construct second: an out-of-set level raises before any gear
    # object exists, so a PtzSpeedGear is never built around an invalid level and
    # no downstream code has to re-validate what it holds.
    return PtzSpeedGear(level=parse_ptz_speed_level(level))


@dataclass(frozen=True)
class PtzZoomAnswer:
    """A G33 query answer -- a zoom gear (档位) and nothing numeric.

    G33 answers "已放大到第 N 档" (18 S9.7); it must NOT fold MaxZoom (3300,
    uncalibrated per T-PTZ-4) into "33 倍", and per 21 S1 must not answer an angle
    or a magnification. The gear label is supplied by the caller from ptz state;
    this type's job is to carry it and refuse to attach a number. to_wire() is the
    assertion surface, like PtzSpeedGear.
    """

    gear: str

    def to_wire(self) -> dict:
        """The G33 answer payload: exactly {"gear": <label>}.

        Any second key (magnification, zoom_ratio, angle_deg) is the "答倍率"
        mutant this guards.
        """
        return {"gear": self.gear}


def query_ptz_zoom_gear(gear: str) -> PtzZoomAnswer:
    """G33 -> a gear-only answer.

    gear is the current zoom 档位 the caller read from ptz state (a label, not a
    ratio -- ptz.zoom is published null while MaxZoom is uncalibrated, 18 S9.7). A
    non-string gear is a caller bug: refusing here keeps a raw ratio from being
    smuggled in as the "gear".
    """
    # G33 sits in the 18 T-PTZ-3 block (21 S1) even though the magnification it
    # must NOT emit is gated by a different debt (T-PTZ-4, MaxZoom calibration).
    # The rule "answer a gear, never a number" holds under both debts, so this
    # branch does not care which one is open -- it just never lets a number pass.
    if not isinstance(gear, str):
        raise ValueError(
            f"query_ptz_zoom_gear expects a gear label string, got "
            f"{type(gear).__name__} -- a raw magnification must not be passed as "
            f"a gear (18 S9.7, T-PTZ-4)"
        )
    # The gear label is carried through unchanged and unvalidated against a fixed
    # set: the zoom档位 are "第 N 档,共三档" (an index, not a named vocabulary like
    # the E09 speed gears), so the caller owns which labels exist. This layer's one
    # job is to refuse a magnification, which the isinstance check above enforces.
    return PtzZoomAnswer(gear=gear)


__all__ = [
    "PRESET_EFFECTIVE_KEY",
    "PTZ_PRESET_INTENTS",
    "PTZ_SPEED_LEVELS",
    "REASON_PRESET_INEFFECTIVE",
    "REASON_PTZ_SPEED_UNCALIBRATED",
    "GUIDE_USE_E01",
    "require_preset_effective",
    "ptz_preset_failsafe",
    "ptz_move_deg_failsafe",
    "parse_ptz_speed_level",
    "PtzSpeedGear",
    "resolve_ptz_speed_gear",
    "PtzZoomAnswer",
    "query_ptz_zoom_gear",
]
