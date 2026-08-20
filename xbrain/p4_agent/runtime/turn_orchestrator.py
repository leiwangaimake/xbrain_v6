"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: turn_orchestrator.py
Brief: GWY-P4-38 (32.F) -- one P4 turn: six-step chain + confirm + envelope

Description:
16 S5.2: the authoritative order for turning one ASR utterance into an
action. This orchestrator strings the already-built P4 modules into that
order, REPLACING the V-2B naive_classify MVP (turn_loop.py). The steps:

  1 safety-bypass (16 S4)   estop / prone / stand match BEFORE any
    classification. A bypass hit goes straight to Tier1 and NEVER reaches
    the classifier -- the whole point of a bypass is that it cannot be
    reasoned about. Recording suppresses VOICE estop (U45).
  2-6 priority chain (16 S5.2) via classifier.classify_text:
    2 long-phrase exact, 3 session-state (confirm response), 4 large-class,
    5 overheard (silent, 16 S5.2.1), 6 unknown -> LLM.
  route (registry): fastpath -> dispatch DIRECTLY (no LLM, no prompt
    assemble); llm / unknown -> tier-2 (GPU-gated grammar-constrained
    classify, GWY-P4-37).
  auth gate: L0/L1a/L1b dispatch now; L2 opens a confirm and WAITS for
    I01/I02 (the confirm response arrives as the NEXT turn's session-state
    match) -- it is NOT dispatched until confirmed; L3 opens a cloud
    approval. CL-2: estop_path==down upgrades an L1b to L2 (one more gate
    when both non-voice estops are unhealthy).
  envelope + validation: a dispatched intent is wrapped in an
    IntentEnvelope (EV-1..7) before it leaves P4.
  reply: chitchat/out_of_scope go through the preset responder (never the
    LLM, never an echo of the user's words).

What this does NOT own (kept in their sub-tasks, injected here): the LLM
call + grammar + prompt assembly (tier2_fn, GWY-P4-37/12/10); live state
for G-query data (GWY-P4-39); DB record/schedule (GWY-P4-40). The
orchestrator decides WHAT happens; those provide the moving parts.

Traps this guards (each has a mutation test):
  * a bypass that went through the classify chain -- estop would then be
    subject to overheard/LLM latency, which is the one thing it must not be
  * an L2 intent dispatched on the same turn it was heard -- the confirm
    would be cosmetic
  * a fastpath intent that touched the LLM -- burns the GPU slot and adds
    latency to a path defined as not needing it
"""
from __future__ import annotations

import re
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

from xbrain.p4_agent.classifier.keyword_matcher import (
    KeywordMatcher, classify_text,
)
from xbrain.p4_agent.classifier.large_class import (
    has_ptz_subject, resolve_large_class, resolve_ptz,
)
from xbrain.p4_agent.envelope.intent_envelope import IntentEnvelope
from xbrain.p4_agent.registry.intents import IntentEntry, IntentRegistry
from xbrain.p4_agent.runtime.intent_dispatch import (
    DispatchResult, dispatch,
)
from xbrain.p4_agent.runtime.geo_request import (
    GeoRequestError, is_geo_intent, manifest_from_state, to_geo_command,
)
from xbrain.p4_agent.runtime.task_request import (
    is_task_create_intent, to_task_request,
)
from xbrain.p4_agent.runtime.teach_request import (
    TeachRequestError, is_teach_intent, session_id_from_state,
    to_teach_command,
)
from xbrain.p4_agent.safety_bypass import matcher as bypass_matcher
from xbrain.p4_agent.safety_bypass import recording_gate
from xbrain.p4_agent.session.chitchat import ChitchatResponder, ChitchatState
from xbrain.p4_agent.session.state_machines import L2ConfirmState, L2Slot


# Spoken wording for the F-class build failures. Hot-tunable phrasing, ASCII
# punctuation per CLAUDE.md 2.2; the keys are the English reasons the builders
# raise. An unmapped reason falls back to a generic line rather than speaking
# the English text at the operator (the 2026-08-11 ORIN test heard an intent
# name read aloud, which is the same defect).
_F_REASON_CN = {
    "no recording session is open": "现在没有正在录制的会话",
    "save needs a name": "要保存的话请说个名字",
    "a keypoint needs a name": "这个点要起个名字",
    "the object catalogue is not available yet": "对象列表还没加载好, 稍后再说一次",
}

# Reply-family intent NAMES routed to the preset responder (never LLM,
# never echo). J01/J02/I05 plus the out_of_scope sentinel (16 S11.5).
_CHITCHAT_NAMES = frozenset({"greeting", "identity", "help", "out_of_scope"})

# Short Chinese labels for spoken confirm/ack feedback (hot-tunable
# wording; ASCII punct). Keyed by 18 id. Only the common action/payload
# intents need one; anything absent falls back to a generic ack. This is
# feedback wording, NOT a data source -- the authoritative restate with
# slot values is GWY-P4-35 (render_restate), wired separately.
_CN_LABELS = {
    "A04": "原地待命", "A05": "站起来", "A06": "趴下", "A11": "转身",
    "B02": "开始巡逻", "B07": "取消任务", "B08": "返航", "B09": "回充电桩",
    "D01": "打开照明灯", "D02": "关闭照明灯", "D04": "打开警笛",
    "D06": "打开爆闪灯", "D07": "关闭爆闪灯",
    "E01": "转动云台", "E05": "停止跟踪", "E06": "变焦", "E07": "云台扫描",
    "E08": "停止扫描", "E09": "调整云台转速", "H07": "重启系统",
}

# PTZ intents that are capability-BLOCKED on this head (18-B): E02 home /
# E03 preset / E04 track sit behind T-PTZ-1 (absolute positioning is a
# no-op on the PELCO-D head), E10 degree-move behind T-PTZ-3. They are
# rejected with an E_CAPABILITY spoken reply BEFORE dispatch -- never sent
# to cmd/ptz (where they would be a no-op that looks like success).
_PTZ_BLOCKED = {
    "E02": "云台归位暂时用不了,可以说向左或向右转一点",
    "E03": "云台预置位暂时用不了,可以说向左或向右转一点",
    "E04": "云台自动跟踪暂时用不了",
    "E10": "云台按角度转暂不支持,可以说向左转一点或向右转一点",
}

# A degree/angle specifier (30 du / 九十度 / 一百八十度). A PTZ move carrying
# an explicit angle is E10 ptz_move_deg -- NOT a bounded jog (E01). The head
# has no absolute positioning (T-PTZ-1) and the pulse->degree map is
# uncalibrated (T-PTZ-4), so E10 is capability-rejected. Without this guard a
# degree move keyword-matches E01 ("云台左转") and silently jogs a fixed pulse,
# so 30/90/180 du all turn the same small amount and the operator is misled
# that the angle took effect (18-B R-6: never execute a clamped/ignored angle
# silently). The number may be Arabic or Chinese numerals.
_DEGREE_RE = re.compile(r"[0-9零一二三四五六七八九十百千两]+\s*度")

# Scan cues. A ptz_move (E01) whose text ALSO says 环视/一圈/一周/扫 is really
# a scan (E07). Layer-2 keyword match cannot decide this: for "平台向左环视一周"
# the E01 keyword "平台向左" (4) TIES the E07 "环视一周" (4), and E01 (earlier in
# the registry) wins the tie -> a full-circle command executes as a single
# small move. Overriding E01->E07 when a scan cue is present fixes every
# prefix (云台/平台/镜头) uniformly, without piling more keyword strings.
_SCAN_CUES = ("环视", "一圈", "一周", "扫")


def _has_ptz_degree(text: str) -> bool:
    return bool(_DEGREE_RE.search(text or ""))


def _has_scan_cue(text: str) -> bool:
    return any(c in (text or "") for c in _SCAN_CUES)


def refine_ptz_intent(intent_id: str, text: str) -> str:
    """Post-classify PTZ correction (18-B). Three fixes a flat keyword match
    cannot make:

      * PTZ-prefix reclaim: a 云台-prefixed command that a chassis keyword
        stole (A13 '慢一点' <- '云台旋转速度慢一点', matched at layer 2 before
        the large-class router could run) is reclaimed to the PTZ family, per
        the operator's prefix rule (云台 prefix -> PTZ). Only when resolve_ptz
        finds a concrete E-intent; otherwise the original stands (so a
        non-action like '云台坏了' is not forced into a move).
      * degree move -> E10 (rejected): an explicit angle is unsupported.
      * scan phrased as a move -> E07.
    """
    # Reclaim first: a non-PTZ classification with a PTZ subject present is the
    # prefix-rule violation; resolve_ptz gives the right E-intent (possibly
    # E01, which the degree/scan step below then refines further).
    if intent_id[:1] != "E" and has_ptz_subject(text):
        reclaimed = resolve_ptz(text)
        if reclaimed is not None:
            intent_id = reclaimed
    if intent_id == "E01":
        # Degree move first: an angle makes it E10 (rejected), even if it also
        # said a scan word (a degree'd scan is not a thing we support).
        if _has_ptz_degree(text):
            return "E10"
        if _has_scan_cue(text):
            return "E07"
    return intent_id

# latency_class per route (EV-7 consistency, mirrored from the envelope).
_LATENCY_BY_ROUTE = {
    "fastpath": "fastpath",
    "fastpath_then_llm": "fastpath",
    "llm": "llm",
    "bypass": "bypass",
}


def _payload_slots(intent_id: str, text: str) -> Dict[str, Any]:
    """Fill the closed-set slot for a device intent from the ASR text
    (16 S8.0.4). Returns an extra dict merged into the dispatch payload so
    the consumer (p2) gets the requested value. Empty when a slot is absent
    (e.g. D18 '换一种' -> no mode -> p2 cycles). Covers the payload
    level/mode/volume intents (D10/D17/D18) and the PTZ move/zoom/speed
    intents (E01/E06/E09)."""
    from xbrain.p4_agent.slots.payload_slots import (
        parse_light_level, parse_strobe_mode, parse_volume,
    )
    from xbrain.p4_agent.slots.ptz_slots import (
        parse_ptz_amount, parse_ptz_direction, parse_ptz_speed_level,
        parse_scan, parse_zoom_direction,
    )
    if intent_id == "D17":                       # set_light_bright
        level = parse_light_level(text)
        return {"level": level} if level else {}
    if intent_id == "D18":                       # set_strobe_mode
        mode = parse_strobe_mode(text)
        return {"mode": mode} if mode is not None else {}
    if intent_id == "D10":                        # set_volume
        vol = parse_volume(text)
        return {"volume": vol} if vol else {}
    if intent_id == "E01":                        # ptz_move
        d = parse_ptz_direction(text)
        out: Dict[str, Any] = {"amount": parse_ptz_amount(text)}
        if d:
            out["direction"] = d
        return out
    if intent_id == "E06":                        # ptz_zoom
        z = parse_zoom_direction(text)
        out = {"amount": parse_ptz_amount(text)}
        if z:
            out["zoom_dir"] = z
        return out
    if intent_id == "E07":                        # ptz_scan (sweep/orbit)
        return parse_scan(text)
    if intent_id == "E09":                        # set_ptz_speed
        level = parse_ptz_speed_level(text)
        return {"level": level} if level else {}
    return {}


@dataclass
class PendingConfirm:
    """An L2 intent awaiting I01/I02. Holds enough to dispatch on confirm."""
    entry: IntentEntry
    text: str
    slot: L2Slot


@dataclass
class OrchestratorSession:
    """Per-operator turn state. One instance lives across a dialog."""
    recording: recording_gate.RecordingState = field(
        default_factory=recording_gate.RecordingState)
    chitchat: ChitchatState = field(default_factory=ChitchatState)
    pending_confirm: Optional[PendingConfirm] = None
    # CL-2 (16): when both non-voice estop paths are unhealthy the link
    # publisher sets estop_path=down; an L1b then upgrades to L2.
    estop_path: str = "up"


@dataclass
class TurnDecision:
    """The outcome of one turn. `kind` names the branch taken."""
    kind: str
    intent_id: Optional[str] = None
    intent_name: Optional[str] = None
    route: Optional[str] = None
    auth: Optional[str] = None
    level: Optional[str] = None
    layer: Optional[str] = None
    bypass_action: Optional[str] = None
    dispatch_result: Optional[DispatchResult] = None
    envelope: Optional[IntentEnvelope] = None
    reply_text: Optional[str] = None
    tts_text: Optional[str] = None
    # Observability for the mutation tests: did this turn touch the LLM /
    # assemble a prompt? A fastpath turn must show both false.
    llm_used: bool = False
    prompt_assembled: bool = False


@dataclass(frozen=True)
class Tier2Classification:
    """What the tier-2 LLM classify returns: the picked intent NAME plus the
    slots the model extracted (e.g. speak_custom's `text`, a route `name`).
    The slots are what the fastpath could not fill -- carrying them is why
    tier-2 returns this instead of a bare name (the free-text slot would be
    lost otherwise)."""
    name: str
    slots: dict


# tier2_fn(text, session, now_mono_ms) -> Tier2Classification or None. None
# means no confident mission / the GPU gate denied the call / the LLM failed
# (the caller declines the turn). Injected so the orchestrator stays testable
# without a live LLM; main_wiring binds it to llm_tier2_fn.build_tier2_fn.
# Calling it is what marks a turn as LLM-using.
Tier2Fn = Callable[[str, "OrchestratorSession", int],
                   Optional[Tier2Classification]]


class TurnOrchestrator:
    """Runs 16 S5.2 for one utterance. Stateless across turns except for
    what the caller carries in OrchestratorSession."""

    def __init__(
        self,
        registry: IntentRegistry,
        *,
        chitchat: ChitchatResponder,
        tier2_fn: Tier2Fn,
        l2_timeout_ms: int,
        matcher: Optional[KeywordMatcher] = None,
        query_fn: Optional[Callable[[IntentEntry], Optional[str]]] = None,
        source: str = "voice",
        geo_state_fn: Optional[Callable[[], Dict[str, Any]]] = None,
    ) -> None:
        self._registry = registry
        self._matcher = matcher or KeywordMatcher(registry)
        self._chitchat = chitchat
        self._tier2 = tier2_fn
        self._l2_timeout_ms = l2_timeout_ms
        # The channel this orchestrator serves ('voice' for the mic loop, or a
        # text channel). Recorded into a task-create request's `source` so P3
        # (and telemetry) can tell a spoken task from a typed one.
        self._source = source
        # GWY-P4-39/41: for a G-class query, query_fn returns the answer
        # rendered from LIVE state (or an 'unknown' reply when stale). When
        # it is None or returns None, the query falls through to a normal
        # dispatch (the data source for that G id is not wired yet).
        self._query_fn = query_fn
        # 11 S12A.1: the F class needs two pieces of LIVE state that only exist
        # outside this class -- the open recording session (state/teach) and the
        # object catalogue (state/geo/manifest). Injected as one getter for the
        # same reason query_fn is: the orchestrator stays a pure decision
        # function and the wiring owns the caches. None means neither is wired,
        # and every F intent then answers with the reason rather than sending a
        # command built on a guess.
        self._geo_state_fn = geo_state_fn

    # -- public entry ----------------------------------------------------

    def handle_turn(self, text: str, session: OrchestratorSession,
                    now_mono_ms: int) -> TurnDecision:
        """Process one ASR utterance end-to-end. See module docstring for
        the ordering (16 S5.2)."""
        text = (text or "").strip()

        # STEP 1: safety-bypass BEFORE classification (16 S4). Match raw
        # then normalized; a hit here never reaches the classifier.
        hit = (bypass_matcher.match_raw(text)
               or bypass_matcher.match_normalized(text))
        if hit is not None:
            supp = recording_gate.evaluate(session.recording, hit.action)
            if supp is not None:
                # U45: voice estop suppressed while recording; still logged
                # + advise the handle. NOT dispatched as a stop.
                return TurnDecision(
                    kind="bypass_suppressed",
                    bypass_action=hit.action,
                    tts_text=supp.tts_advice)
            # Straight to Tier1 (bypass route). No classify, no envelope
            # gating -- that is the safety guarantee.
            return TurnDecision(
                kind="bypass", bypass_action=hit.action, route="bypass")

        # STEP 3 (early): if a confirm is open, THIS turn is its response.
        if session.pending_confirm is not None:
            return self._resolve_pending_confirm(text, session, now_mono_ms)

        if not text:
            return TurnDecision(kind="overheard", layer="overheard")

        # STEPS 2-6: the priority chain. large_class_fn supplies layer 4
        # (16 S5.2): the deterministic device-family router that rescues
        # keyword MISSES for PTZ (E) and payload (D) so a slightly-reworded
        # command ("云台朝左", "把照明灯搞亮点") still resolves. It runs only
        # when layer 2 missed, so exact keywords are never overridden.
        result = classify_text(text, self._registry, self._matcher,
                               large_class_fn=resolve_large_class)
        if result.layer == "overheard":
            # 16 S5.2.1: not addressed to the robot -> completely silent.
            return TurnDecision(kind="overheard", layer="overheard")
        if result.fires_llm:
            # Layer 6: nothing matched but it was directed -> tier-2 LLM.
            return self._run_tier2(text, session, now_mono_ms)

        # Layers 2/3/4 matched an intent id. Apply the PTZ post-classify
        # correction (18-B): a degree move -> E10 (rejected), a scan phrased
        # as a move -> E07. A flat keyword match cannot make either call.
        intent_id = refine_ptz_intent(result.intent, text)
        entry = self._registry.by_id(intent_id)
        return self._route_entry(entry, text, session, now_mono_ms,
                                 layer=result.layer)

    # -- routing ---------------------------------------------------------

    def _route_entry(self, entry: IntentEntry, text: str,
                     session: OrchestratorSession, now_mono_ms: int,
                     layer: str,
                     llm_slots: Optional[dict] = None) -> TurnDecision:
        """Apply route (fastpath vs LLM) and auth (confirm gate). llm_slots
        (tier-2 only) are the slots the LLM extracted; they are merged OVER the
        fastpath slots so a free-text slot (speak_custom's text) survives."""
        # Reply-family intents (chitchat / help) answer from a preset and
        # never dispatch an action. Handled before the auth gate: they are
        # all L0 and produce speech, not motion.
        if entry.name in _CHITCHAT_NAMES:
            return self._reply_chitchat(entry, session, layer)

        # Capability-blocked PTZ intents (18-B T-PTZ-1/3): reject with a
        # spoken reason and do NOT dispatch -- they are a no-op on the head,
        # and a silent dispatch would look like success.
        blocked = _PTZ_BLOCKED.get(entry.id)
        if blocked is not None:
            return TurnDecision(
                kind="denied", intent_id=entry.id, intent_name=entry.name,
                route=entry.route, auth=entry.auth, layer=layer,
                tts_text=blocked)

        # G-class query: answer from LIVE state (GWY-P4-39). query_fn
        # returns the rendered answer (or an 'unknown' reply when the state
        # is stale); None means the data source for this G id is not wired,
        # so fall through to a normal dispatch.
        if entry.id[:1] == "G" and self._query_fn is not None:
            answer = self._query_fn(entry)
            if answer is not None:
                return TurnDecision(
                    kind="reply", intent_id=entry.id, intent_name=entry.name,
                    route=entry.route, auth=entry.auth, layer=layer,
                    reply_text=answer, tts_text=answer)

        # Effective auth with CL-2 upgrade (L1b -> L2 when estop down).
        eff_auth = self._effective_auth(entry.auth, session)

        if eff_auth == "L2":
            # Open a confirm and WAIT. Do NOT dispatch this turn.
            slot = L2Slot(timeout_millis=self._l2_timeout_ms)
            slot.request(now_mono_ms=now_mono_ms)
            session.pending_confirm = PendingConfirm(
                entry=entry, text=text, slot=slot)
            return TurnDecision(
                kind="await_confirm", intent_id=entry.id,
                intent_name=entry.name, route=entry.route, auth=eff_auth,
                layer=layer, tts_text=self._confirm_prompt(entry))
        if eff_auth == "L3":
            # Cloud approval flow (GWY-P4-38 hand-off to P5). Not dispatched
            # until the cloud confirm_token arrives.
            return TurnDecision(
                kind="await_approval", intent_id=entry.id,
                intent_name=entry.name, route=entry.route, auth=eff_auth,
                layer=layer, tts_text=self._approval_prompt(entry))

        # L0 / L1a / L1b -> dispatch now. Pass through any tier-2 LLM slots.
        return self._dispatch_entry(entry, text, session, now_mono_ms,
                                    layer, eff_auth, llm_slots=llm_slots)

    def _dispatch_entry(self, entry: IntentEntry, text: str,
                        session: OrchestratorSession, now_mono_ms: int,
                        layer: str, eff_auth: str,
                        llm_slots: Optional[dict] = None) -> TurnDecision:
        """Route the FAST leg. fastpath / fastpath_then_llm dispatch
        directly (no LLM, no prompt assemble). A pure-llm matched intent
        still needs slot fill via tier-2. llm_slots (from a layer-6 tier-2
        classification) are merged over the fastpath slots below."""
        llm_used = llm_slots is not None       # layer-6 already ran the LLM
        # A route=='llm' matched intent needs the LLM to fill its slots -- BUT
        # only when a layer-6 classification did not already supply them (else
        # this would be a wasteful second LLM call on the same utterance).
        if entry.route == "llm" and llm_slots is None:
            classified = self._tier2(text, session, now_mono_ms)
            llm_used = True
            if classified is None:
                return TurnDecision(kind="denied", intent_id=entry.id,
                                    intent_name=entry.name, route=entry.route,
                                    auth=eff_auth, layer=layer, llm_used=True,
                                    prompt_assembled=True)
            llm_slots = classified.slots
        # Fill fastpath closed-set slots from the text (16 S8.0.4) for the
        # payload level/mode/volume intents, so p2 gets the requested value
        # (D17 level / D18 mode / D10 volume), not just the intent id.
        extra = _payload_slots(entry.id, text)
        # Merge the tier-2 LLM slots OVER the fastpath ones: the LLM fills the
        # slots the fastpath regex cannot (free text, names), and the fastpath
        # fills the closed-set enums. LLM values win a key collision.
        if llm_slots:
            extra = {**(extra or {}), **llm_slots}
        # PB4: a task-CREATE intent (goto/patrol/charge/teach/follow) carries a
        # cmd/task REQUEST P3's ingest records into task.db -- {task_type,
        # intent, id, slots, source}. Without it the cmd/task frame is just the
        # p4_intent_v1 {intent_id, text} P3 cannot turn into a task row. Control
        # intents (pause/cancel/stop_follow) are NOT creates: to_task_request
        # returns None and the frame stays as-is (they act on an existing task).
        if is_task_create_intent(entry.name):
            # Pass the turn's text so it lands in tasks.command_text (15 S9.5A.4
            # / 17 S6.8.4 field 3): party-A requires the raw command stored for
            # incident traceability. `text` here is what the turn acted on (ASR
            # transcript post normalisation, or the typed text), same value the
            # dispatch below uses -- so what is stored is what was executed.
            treq = to_task_request(
                entry.name, self._registry,
                slots=dict(extra), source=self._source, text=text)
            if treq is not None:
                extra = {**(extra or {}), "task_request": treq}
        # 11 S12A.1: the F class. F01-F10 build a TeachCommand for cmd/teach,
        # F11-F15 a GeoCommand for cmd/geo. A build failure is spoken back
        # ("there is no recording in progress", "no route named X") rather than
        # dispatched -- sending a command that cannot succeed costs the operator
        # a round trip and tells them nothing they can act on.
        if is_teach_intent(entry.name) or is_geo_intent(entry.name):
            extra, failure = self._build_f_class(entry, extra)
            if failure is not None:
                return TurnDecision(kind="reply", intent_id=entry.id,
                                    intent_name=entry.name, route=entry.route,
                                    auth=eff_auth, level=eff_auth, layer=layer,
                                    reply_text=failure, tts_text=failure,
                                    llm_used=llm_used,
                                    prompt_assembled=llm_used)
        # Build the envelope (EV-1..7) and dispatch.
        env = self._build_envelope(entry, eff_auth, slots=dict(extra))
        result = dispatch(entry.id, text, extra or None)
        return TurnDecision(
            kind="dispatch", intent_id=entry.id, intent_name=entry.name,
            route=entry.route, auth=eff_auth, level=eff_auth, layer=layer,
            dispatch_result=result, envelope=env,
            tts_text=self._dispatch_ack(entry),
            llm_used=llm_used, prompt_assembled=llm_used)

    def _run_tier2(self, text: str, session: OrchestratorSession,
                   now_mono_ms: int) -> TurnDecision:
        """Layer-6 unknown: tier-2 LLM classify. Marks LLM used + prompt
        assembled (both happen inside tier2_fn)."""
        result = self._tier2(text, session, now_mono_ms)
        if result is None:
            # No mission / gate denied / LLM failed: decline (caller may TTS).
            return TurnDecision(kind="denied", layer="unknown",
                                llm_used=True, prompt_assembled=True)
        classified = result.name
        if classified in _CHITCHAT_NAMES:
            # The LLM classified it as out_of_scope / chitchat.
            reply = self._chitchat.respond(classified, session.chitchat)
            return TurnDecision(
                kind="reply", intent_name=classified, layer="unknown",
                reply_text=reply, tts_text=reply, route="llm",
                llm_used=True, prompt_assembled=True)
        # A real intent the LLM picked: route it like a matched intent, passing
        # the LLM-extracted slots (the free-text ones the fastpath cannot fill).
        entry = self._registry.by_name(classified)
        decision = self._route_entry(entry, text, session, now_mono_ms,
                                     layer="unknown", llm_slots=result.slots)
        # Preserve the fact the LLM was used to reach this classification.
        decision.llm_used = True
        decision.prompt_assembled = True
        return decision

    # -- confirm resolution ---------------------------------------------

    def _resolve_pending_confirm(self, text: str,
                                 session: OrchestratorSession,
                                 now_mono_ms: int) -> TurnDecision:
        """A confirm is open; interpret this turn as the I01/I02 response
        (16 S5.2 layer 3 session-state)."""
        pc = session.pending_confirm
        # Timeout first: a stale confirm must not accept a late 'yes'.
        pc.slot.tick(now_mono_ms=now_mono_ms)
        if pc.slot.state == L2ConfirmState.TIMED_OUT:
            session.pending_confirm = None
            return TurnDecision(kind="confirm_timeout", intent_id=pc.entry.id,
                                intent_name=pc.entry.name,
                                tts_text=self._timeout_prompt(pc.entry))
        # Classify the response; I01 confirm / I02 deny are session-state
        # intents supplied here (they are excluded from the layer-2 index).
        response = self._classify_confirm_response(text)
        if response == "I02":       # deny
            session.pending_confirm = None
            return TurnDecision(kind="confirm_denied", intent_id=pc.entry.id,
                                intent_name=pc.entry.name,
                                tts_text=self._denied_prompt(pc.entry))
        if response != "I01":       # neither yes nor no -> keep waiting
            return TurnDecision(kind="await_confirm", intent_id=pc.entry.id,
                                intent_name=pc.entry.name,
                                tts_text=self._confirm_prompt(pc.entry))
        # Confirmed: dispatch the held intent NOW.
        pc.slot.confirm()
        entry, held_text = pc.entry, pc.text
        session.pending_confirm = None
        env = self._build_envelope(entry, "L2", slots={})
        result = dispatch(entry.id, held_text)
        return TurnDecision(
            kind="dispatch", intent_id=entry.id, intent_name=entry.name,
            route=entry.route, auth="L2", level="L2", layer="session_state",
            dispatch_result=result, envelope=env,
            tts_text=self._dispatch_ack(entry))

    def _classify_confirm_response(self, text: str) -> Optional[str]:
        """Map a confirm-window utterance to I01 (confirm) / I02 (deny) /
        None. Uses the registry keywords for I01/I02 (which are excluded
        from the layer-2 index, so they are matched here, at layer 3)."""
        i01 = self._registry.by_id("I01")
        i02 = self._registry.by_id("I02")
        # Deny checked with the same longest-first spirit; here a simple
        # substring is enough because the window is a yes/no context.
        for kw in i02.keywords:
            if kw and kw in text:
                return "I02"
        for kw in i01.keywords:
            if kw and kw in text:
                return "I01"
        return None

    # -- replies ---------------------------------------------------------

    def _reply_chitchat(self, entry: IntentEntry,
                        session: OrchestratorSession,
                        layer: str) -> TurnDecision:
        reply = self._chitchat.respond(entry.name, session.chitchat)
        return TurnDecision(
            kind="reply", intent_id=entry.id, intent_name=entry.name,
            route=entry.route, auth=entry.auth, layer=layer,
            reply_text=reply, tts_text=reply)

    # -- envelope + helpers ---------------------------------------------

    def _build_f_class(self, entry: IntentEntry,
                       extra: Optional[Dict[str, Any]]):
        """(slots with the command attached, None) or (slots, spoken reason).

        Returned as a pair rather than stashed on self: an orchestrator that
        carried per-turn state on the instance would leak one turn into the
        next, and these turns are exactly the ones where that matters (a failed
        save followed by a successful one).

        The reason is Chinese because it is spoken straight back to the
        operator; the codes behind it (E_TEACH_STATE, E_NOT_FOUND) would only
        reach them as "internal error".
        """
        state = self._geo_state_fn() if self._geo_state_fn else {}
        merged = dict(extra or {})
        cmd_id = uuid.uuid4().hex
        try:
            if is_teach_intent(entry.name):
                cmd = to_teach_command(
                    entry.name, slots=merged, cmd_id=cmd_id,
                    source=self._source,
                    session_id=session_id_from_state(state.get("state/teach")))
                if cmd is not None:
                    merged["teach_command"] = cmd
            else:
                cmd = to_geo_command(
                    entry.name, slots=merged, cmd_id=cmd_id,
                    manifest=manifest_from_state(
                        state.get("state/geo/manifest")),
                    origin="voice" if self._source == "voice" else "voice")
                if cmd is not None:
                    merged["geo_command"] = cmd
        except TeachRequestError as exc:
            return dict(extra or {}), _F_REASON_CN.get(
                str(exc), "这个录制指令现在没法执行")
        except GeoRequestError as exc:
            return dict(extra or {}), _F_REASON_CN.get(
                str(exc), "没找到你说的那个对象")
        return merged, None

    def _build_envelope(self, entry: IntentEntry, level: str,
                        slots: Dict[str, Any]) -> IntentEnvelope:
        """Wrap a dispatched intent (EV-1..7 checked in __post_init__)."""
        return IntentEnvelope(
            id=entry.id,
            intent=entry.name,
            route=entry.route,
            auth=entry.auth,
            level=level,
            slots=slots,
            cmd_id=uuid.uuid4().hex,
            latency_class=_LATENCY_BY_ROUTE[entry.route],
        )

    def _effective_auth(self, auth: str, session: OrchestratorSession) -> str:
        """CL-2: estop_path==down upgrades L1b -> L2 (one extra gate)."""
        if auth == "L1b" and session.estop_path == "down":
            return "L2"
        return auth

    # Prompt wording (hot phrasing; ASCII punct, Chinese words). A Chinese
    # label per intent gives the operator something meaningful to confirm;
    # falls back to a generic prompt. NEVER speaks the English intent name
    # (the 2026-08-11 ORIN test heard '确认要执行cancel_task吗').
    def _confirm_prompt(self, entry: IntentEntry) -> str:
        label = _CN_LABELS.get(entry.id)
        if label:
            return "确认" + label + "吗,请说是或否"
        return "确认执行这个操作吗,请说是或否"

    def _approval_prompt(self, entry: IntentEntry) -> str:
        return "该操作需要云端审批,已上报等待批准"

    def _denied_prompt(self, entry: IntentEntry) -> str:
        return "好的,已取消"

    def _timeout_prompt(self, entry: IntentEntry) -> str:
        return "没有收到确认,已取消"

    def _dispatch_ack(self, entry: IntentEntry) -> str:
        """Brief spoken acknowledgment for a dispatched action, so a motion
        command is not silent. A Chinese label when known, else 'received'.
        (Full L1a/L1b restate with slot values is GWY-P4-35 wiring; this is
        the minimal feedback the 2026-08-11 ORIN test showed was missing.)"""
        label = _CN_LABELS.get(entry.id)
        return ("好的," + label) if label else "收到"
