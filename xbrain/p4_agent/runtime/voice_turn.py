"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: voice_turn.py
Brief: p4 builds xbrain/{rid}/state/voice_turn -- the per-turn audit record (11 S8.7.6)

Description:
11 S2.2 registers p4_agent as the publisher of xbrain/{rid}/state/voice_turn,
"one interaction's audit record", and until 2026-10-26 nothing published it.
Three things depend on it and were missing without it:

  * the answer, for anyone not standing next to the robot. Replies are
    spoken by the GZH-2 module's own TTS (p2 POST /tts, 11 S7.5A.5): the
    main board never holds the audio, so a cloud operator, the HMI and the
    commx debugging client cannot hear it. reply_text is how they read it,
    and with require_tts_reply=false it is the ONLY place the answer goes
    (11 S8.7.5 "answer only on state/voice_turn").
  * what ASR heard. asr_raw and asr_text are both mandatory (11 S8.7.6:
    "both must be reported and the HMI must show both") -- with only the
    corrected text nobody can tell "ASR misheard" from "post-processing
    rewrote it wrong", and 16 S3 cannot be tuned in the field.
  * the stitch between "what the model said" and "what the robot did".
    For a motion intent the record reuses the turn_id, intent and slots of
    the cmd/motion/intent frame that went out, so the two keys can be
    compared byte for byte (11 MI-1 / 16 EV-1).

Two field rulings by the user (2026-10-26, see 11 S8.7.6 implementation note):
  * an OVERHEARD turn (16 S5.2.1: not addressed to the robot) is still
    recorded -- verdict "unknown", reply_kind "none", nothing spoken -- and
    the extra field `layer` ("overheard" vs "unknown") separates it from a
    turn the LLM could not place, which also ends as verdict "unknown".
    Recording is not replying: 16 S5.2.1's "completely silent" is about
    speech, lamps and dialog state, and none of those happen.
  * reply_kind "stream" means "spoken by on-line synthesis", which on this
    robot is the device TTS fed with text -- not a PCM stream on the board.

Boundaries: building the record only. Publishing is the caller's (the mic
turn handler and the cloud runner both append it to the frames they publish);
storing corrections in the asr_correction table is p5's (11 S2.2 subscriber
column). asr_conf and the asr/llm/tts latencies are not filled: the ASR client
returns text only, and the TTS runs in a device that reports no timing.
"""
from __future__ import annotations

import json  # the record goes out as one JSON object
import time  # ts_mono (CLK-C1)
import uuid  # turn ids for turns that sent no motion frame
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

from xbrain.p4_agent.runtime.intent_dispatch import CMD_AUDIO_SPEAK

#: 11 S2.2 key template (%s = robot id). Robot-prefixed: the HMI and the
#: cloud subscribe it directly, p5 stores its corrections.
VOICE_TURN_KEY_FMT = "xbrain/%s/state/voice_turn"  # 11 S2.2, Q2 event

#: channel -> session slot. 11 S8.7.6 has four channels and three sessions;
#: hmi shares the local slot (16 S11.1 implementation note, 2026-10-25).
SESSION_OF_CHANNEL = {"mic_local": "local", "cloud": "cloud",
                      "wecom": "wecom", "hmi": "local"}

#: Confirmation levels as the registry carries them (18 S13.1 via
#: registry/intents.py CONFIRM_LEVELS). 11 S8.7.6's example comment lists
#: "L0 | L1 | L2 | L3"; 18 splits L1 into L1a / L1b and that comment is
#: corrected in the same batch (iron rule 1).
AUTH_LEVELS = ("L0", "L1a", "L1b", "L2", "L3")  # 18 S13.1

#: 16 S4 bypass actions; anything else is a bug upstream and is refused.
BYPASS_ACTIONS = ("estop", "prone", "stand")  # 16 S4 the three bypass words

#: TurnDecision.kind -> 11 S8.7.6 verdict. Every kind the orchestrator can
#: return is listed; an unlisted kind raises instead of being guessed, so a
#: new branch in the orchestrator cannot silently land as "accepted".
VERDICT_OF_KIND = {
    "dispatch": "accepted",          # an action went out
    "bypass": "accepted",            # Tier1 stop / prone / stand went out
    "reply": "accepted",             # answered (query, chitchat, a re-ask)
    "await_confirm": "pending_confirm",   # L2: waiting for the operator
    "await_approval": "pending_confirm",  # H08-type: waiting for approval
    "denied": "rejected",            # refused with a spoken reason
    "bypass_suppressed": "rejected",  # U45: estop held while recording
    "confirm_denied": "rejected",    # the operator said no
    "confirm_timeout": "rejected",   # nobody answered the confirm
    "overheard": "unknown",          # user ruling 2026-10-26, see header
}


class VoiceTurnError(ValueError):
    """A record field is outside its closed set (a bug upstream)."""


def new_turn_id() -> str:
    """A fresh "vt-" id, the same shape the motion-intent builder uses."""
    return "vt-" + uuid.uuid4().hex[:12]  # 12 hex: unique per turn


def _motion_data(decision: Any) -> Optional[Mapping[str, Any]]:
    """The data block of the cmd/motion/intent frame this turn sent, if any.

    The orchestrator puts the full MotionIntent envelope under the dispatch
    payload's "motion_intent"; its "data" carries turn_id / intent / slots.
    None for every turn that sent no motion frame.
    """
    dr = getattr(decision, "dispatch_result", None)  # None when no action went out
    payload = getattr(dr, "payload", None)  # the dispatch envelope
    if not isinstance(payload, dict):  # no dispatch: no motion frame
        return None  # no dispatch at all
    mi = payload.get("motion_intent")  # the MotionIntent envelope
    data = mi.get("data") if isinstance(mi, dict) else None
    return data if isinstance(data, dict) else None  # None: not a motion turn


def _corrections(rows: Iterable[Sequence[Any]]) -> list:
    """16 S3 corrections -> 11 S8.7.6 corrections[] entries.

    The post-processor reports (layer, from, to, n): for L1 an exact
    dictionary hit, so score 1.0 (as in the 11 example); for L2 n is the
    pinyin edit distance, which is not a similarity score -- it is reported
    as "distance" and score stays null rather than being invented.
    """
    out = []  # in the post-processor's order
    for layer, src, dst, n in rows:  # the 16 S3 tuple shape
        entry = {"layer": layer, "from": src, "to": dst}  # what changed into what
        if layer == "L1":
            entry["score"] = 1.0  # exact dictionary replacement
        else:  # L2: pinyin distance
            entry["score"] = None  # no similarity score exists for L2
            entry["distance"] = n  # pinyin edit distance, as computed
        out.append(entry)  # kept in order
    return out  # may be empty: nothing rewritten


def build_voice_turn(*, channel: str, asr_raw: str, asr_text: str,
                     decision: Any,  # TurnDecision, or None if gated
                     published_keys: Iterable[str],  # what actually went out
                     corrections: Iterable[Sequence[Any]] = (),
                     latency_ms: Mapping[str, int],  # what this caller measured
                     reject_code: Optional[str] = None,  # a 11 S13 code, else None
                     now_mono: Optional[float] = None) -> Dict[str, Any]:
    """One state/voice_turn record (11 S8.7.6).

    channel: mic_local | cloud | wecom | hmi. asr_raw / asr_text: the ASR
    output before and after 16 S3 (typed text: both are the text). decision:
    the TurnDecision, or None when the turn never reached the orchestrator
    (text-channel admission refused it). published_keys: the keys the turn
    actually published, which decides reply_kind. latency_ms: what this
    caller measured ({} when nothing) -- required, not defaulted, because the
    p4 safety-default scan (CLAUDE.md 3.1) reads any *_ms default as a limit. Raises VoiceTurnError on a
    closed-set violation (11 S13.6: out-of-set values are refused, never
    passed through).
    """
    if channel not in SESSION_OF_CHANNEL:
        raise VoiceTurnError("channel %r" % (channel,))
    keys = set(published_keys)  # set: order does not matter here
    if decision is None:
        # Refused at the channel gate before any classification.
        verdict, intent, level, layer, bypass, reply = (
            "rejected", None, None, None, None, None)
        slots: Dict[str, Any] = {}  # no intent, no slots
    else:
        if decision.kind not in VERDICT_OF_KIND:
            raise VoiceTurnError("decision kind %r" % (decision.kind,))
        verdict = VERDICT_OF_KIND[decision.kind]  # closed-set mapped
        intent = decision.intent_name  # the registry name, e.g. turn_around
        level = decision.level  # registry level (L0..L3)
        layer = decision.layer  # which classifier layer decided
        bypass = decision.bypass_action  # estop / prone / stand / None
        reply = decision.tts_text or decision.reply_text  # spoken text if any
        slots = {}  # filled below for motion turns
        if level is not None and level not in AUTH_LEVELS:
            raise VoiceTurnError("auth level %r" % (level,))
        if bypass is not None and bypass not in BYPASS_ACTIONS:
            raise VoiceTurnError("bypass %r" % (bypass,))
    data = _motion_data(decision)  # None for non-motion turns
    if data is not None:
        # MI-1 / EV-1: the very turn_id, intent and slots of the frame that
        # went out, not a second rendering of them.
        turn_id = data["turn_id"]  # the stitch with the frame
        intent = data["intent"]
        slots = data["slots"]
    else:
        turn_id = new_turn_id()  # no frame to stitch to
    record: Dict[str, Any] = {
        "turn_id": turn_id,
        "session": SESSION_OF_CHANNEL[channel],  # 16 S11.1 slot
        "channel": channel,  # as given, checked above
        "asr_raw": asr_raw,  # mandatory, before 16 S3
        "asr_text": asr_text,  # mandatory, after 16 S3
        "corrections": _corrections(corrections),  # 11 S8.7.6 entry shape
        "bypass": bypass,  # null unless a bypass fired
        "intent": intent,
        "slots": slots,  # {} when none
        "auth_level": level,  # null for gated turns
        "verdict": verdict,  # accepted / rejected / ...
        "reject_code": reject_code,  # e.g. E_CHANNEL_DENIED
        "reply_text": reply,  # the answer, spoken or not
        # "stream" = spoken by on-line synthesis (the device TTS here); a
        # reply that was suppressed (require_tts_reply=false) or never made
        # is "none" even when reply_text is filled.
        "reply_kind": "stream" if CMD_AUDIO_SPEAK in keys else "none",
        # Extra field (user ruling 2026-10-26): which classifier layer
        # decided, so verdict "unknown" can be told apart -- "overheard"
        # (not addressed) vs "unknown" (the LLM could not place it).
        "layer": layer,
        "ts_mono": time.monotonic() if now_mono is None else now_mono,
    }
    if latency_ms:  # omitted when nothing measured
        record["latency_ms"] = dict(latency_ms)  # only what was measured
    return record  # ready for encode()


def encode(record: Mapping[str, Any]) -> bytes:
    """Compact UTF-8 JSON, Chinese kept readable (the HMI shows it as is)."""
    return json.dumps(record, ensure_ascii=False,  # readable Chinese
                      separators=(",", ":")).encode("utf-8")


def voice_turn_pair(key: Optional[str], record: Mapping[str, Any]
                    ) -> Tuple[Tuple[str, bytes], ...]:
    """(key, bytes) to append to a turn's publishes; () when no key is set.

    No key means no robot id was known at startup; the wiring logged that
    once, and every turn then simply publishes no audit record rather than
    one on xbrain/None/....
    """
    if not key:  # no robot id at startup
        return ()  # publish nothing
    return ((key, encode(record)),)  # one pair, appended last
