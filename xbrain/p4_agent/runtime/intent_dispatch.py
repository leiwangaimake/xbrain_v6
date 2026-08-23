"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: intent_dispatch.py
Brief: p4_agent intent -> outbound cmd/* key dispatcher

Description:
Once the tier-1 classifier or LLM produces an intent id (e.g. B09,
D01, R2, T02), this module maps it to ONE of the five outbound
key families P4 publishes on the GEN plane:

  cmd/audio/speak       -- chat reply, prompt, D-mode broadcast
  cmd/task              -- patrol, goto, charge, standby, teach
  cmd/motion/intent     -- ad-hoc motion (turn, backup, spin)
  cmd/ptz               -- PTZ pan/tilt/zoom/preset
  cmd/payload           -- lights, siren, alarm level, gimbal payload

The mapping is a data table (INTENT_TO_KEY) so a spec change adds
a row rather than an if/elif branch. Unknown intent id ->
UnknownIntentDispatch (never silent).

The dispatcher does NOT decide WHAT to say / do; it just decides
WHICH KEY. The intent_id + args -> payload composer is a separate
concern per key family. For the voice-loop MVP, each dispatch
carries the raw ASR text under `text` field so downstream
subscribers can pretty-print in the smoke test.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


class UnknownIntentDispatch(Exception):
    pass


# Key families (11 S2.2). Table lookup by intent-id prefix + specific id.
CMD_AUDIO_SPEAK = "cmd/audio/speak"
CMD_TASK = "cmd/task"
CMD_MOTION_INTENT = "cmd/motion/intent"
CMD_PTZ = "cmd/ptz"
CMD_PAYLOAD = "cmd/payload"
# 11 S12A.1 splits the F class across two keys: F01-F10 open / drive / end a
# RECORDING SESSION on cmd/teach, F11-F15 are CRUD on an existing object and go
# to cmd/geo. Neither is cmd/task -- see the note on the F prefix below.
CMD_MODE = "cmd/mode"
CMD_TEACH = "cmd/teach"
CMD_GEO = "cmd/geo"


# Prefix-first mapping. Exact intent ids that need a special
# destination override the prefix rule via INTENT_TO_KEY.
_PREFIX_MAP = {
    "A": CMD_MOTION_INTENT,   # A01-A12 ad-hoc motion (forward/turn/etc)
    "B": CMD_TASK,             # B01-B09 tasks (patrol, charge, standby, estop)
    # C is the MODE class (18 C01-C08). The six whose 18 effect column reads
    # "P2 -> ..." are overridden to cmd/mode below; this prefix entry stays
    # CMD_TASK as the backstop for the two that are NOT mode commands:
    #   C06 standby  -- 18: "P3 挂起任务 + P1 hold", effect is P3/P1 not P2;
    #   C08 query_mode_switch_ok -- 18: a QUERY ("can we switch to broadcast?").
    # *** The old comment here read "C01-C07 (hold, slow, cancel, look, spin)",
    # which describes a different intent set entirely -- this table was written
    # against an assumed C class, and every C intent went to cmd/task where P3
    # skipped it for having no top-level action. Eight dead voice commands.
    "C": CMD_TASK,
    "D": CMD_AUDIO_SPEAK,      # D01-D13 broadcast / chitchat / speak_stop
    "E": CMD_PTZ,              # PTZ family
    # F is the RECORDING class (18 F01-F15), not follow -- follow is B11. Every
    # F id is overridden below, so this prefix entry is only a backstop; it
    # points at cmd/teach because that is where ten of the fifteen go and a new
    # F id is far more likely to be a session action than a task.
    "F": CMD_TEACH,            # F01-F15 recording (per-id overrides below)
    "G": CMD_AUDIO_SPEAK,      # G01-G24 queries -> speak the reply
    "H": CMD_TASK,             # H01-H08 system (bit / report / sleep / reboot)
    "I": CMD_AUDIO_SPEAK,      # I01-I05 session ack (confirm/deny/repeat)
    "J": CMD_AUDIO_SPEAK,      # J01/J02 chitchat -> speak canned reply
    "R": CMD_PAYLOAD,          # R-* payload/lights/siren
    "T": CMD_TASK,             # T-* task lifecycle
}


# Overrides for specific ids where the prefix rule is wrong.
INTENT_TO_KEY: Dict[str, str] = {
    "B09": CMD_TASK,            # estop is a task-family event (queued)
    "D12": CMD_TASK,            # speak_stop is a TASK-side action + audio ack
    # 2026-08-11 V-STROBE-1: D-prefix defaults to CMD_AUDIO_SPEAK but
    # the actual lamp / siren D-intents drive HARDWARE via
    # payload-service. Route them to cmd/payload; p2 PayloadDomain
    # subscribes and calls the HTTP endpoints.
    "D01": CMD_PAYLOAD,          # light_on
    "D02": CMD_PAYLOAD,          # light_off
    "D03": CMD_PAYLOAD,          # light_auto
    "D04": CMD_PAYLOAD,          # siren_on
    "D05": CMD_PAYLOAD,          # siren_off
    "D06": CMD_PAYLOAD,          # strobe_on (red-blue warning lamp)
    "D07": CMD_PAYLOAD,          # strobe_off
    "D10": CMD_PAYLOAD,          # set_volume (8519 audio; p2 -> POST /volume)
    "D11": CMD_PAYLOAD,          # chassis_light
    "D17": CMD_PAYLOAD,          # set_light_bright (18-A)
    "D18": CMD_PAYLOAD,          # set_strobe_mode (18-A)
}


# 11 S12A.1, transcribed per id. F01-F10 are session actions on cmd/teach;
# F11-F15 act on a stored object and go to cmd/geo. Written out rather than
# derived from a number range so adding F16 is a decision somebody makes, not
# something a comparison operator makes for them.
INTENT_TO_KEY.update({
    "F01": CMD_TEACH, "F02": CMD_TEACH, "F03": CMD_TEACH, "F04": CMD_TEACH,
    "F05": CMD_TEACH, "F06": CMD_TEACH, "F07": CMD_TEACH, "F08": CMD_TEACH,
    "F09": CMD_TEACH, "F10": CMD_TEACH,
    "F11": CMD_GEO, "F12": CMD_GEO, "F13": CMD_GEO, "F14": CMD_GEO,
    "F15": CMD_GEO,
})

# 18 C class -> cmd/mode. Per-id, NOT by prefix: only the six whose 18 effect
# column reads "P2 -> ..." are mode commands.
#
# C06 standby and C08 query_mode_switch_ok are deliberately ABSENT -- see the
# "C" entry in _PREFIX_MAP. Adding them here is the tempting one-line change
# that would make a question ("can we switch to broadcast?") issue a mode
# switch, which is the opposite of what the operator asked.
INTENT_TO_KEY.update({
    "C01": CMD_MODE, "C02": CMD_MODE, "C03": CMD_MODE,
    "C04": CMD_MODE, "C05": CMD_MODE, "C07": CMD_MODE,
})


@dataclass(frozen=True)
class DispatchResult:
    intent_id: str
    key: str
    payload: dict


def choose_key(intent_id: str) -> str:
    """Return the target key for an intent id. Raises on unknown."""
    if intent_id in INTENT_TO_KEY:
        return INTENT_TO_KEY[intent_id]
    if not intent_id:
        raise UnknownIntentDispatch("empty intent_id")
    prefix = intent_id[0]
    if prefix not in _PREFIX_MAP:
        raise UnknownIntentDispatch(
            f"intent {intent_id!r}: prefix {prefix!r} has no key mapping")
    return _PREFIX_MAP[prefix]


def build_payload(intent_id: str, text: str,
                    extra: dict = None) -> dict:
    """Compose the outbound payload. Every key family carries at
    least intent_id + text + timestamp fields so the smoke-test
    subscriber can print a legible line."""
    import time
    p = {
        "schema": "p4_intent_v1",
        "intent_id": intent_id,
        "text": text,
        "mono_ms": int(time.monotonic() * 1000),
    }
    if extra:
        p.update(extra)
    return p


def dispatch(intent_id: str, text: str,
              extra: dict = None) -> DispatchResult:
    """One-shot: pick key + build payload."""
    key = choose_key(intent_id)
    return DispatchResult(intent_id=intent_id, key=key,
                            payload=build_payload(intent_id, text, extra))
