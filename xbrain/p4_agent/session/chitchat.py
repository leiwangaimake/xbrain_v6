"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: chitchat.py
Brief: GWY-P4-36 (32.D) -- zero-LLM chitchat / out_of_scope preset responder

Description:
16 S11.5 / 18 S12 / CMD-50..54: chitchat replies are PRESETS, never LLM
free-form. This module turns a whitelist intent (J01 greeting, J02
identity, I05 help) or the out_of_scope fallback into a preset reply from
configs/chitchat.yaml, and tracks the per-session consecutive out_of_scope
count so the capability overview (help.reply) is announced after
consecutive_threshold hits (18 S12.5).

Why this is a separate module from the LLM tier:
* 18 S12.1 reason 2: a 3B model hallucinates when it free-generates; reason
  3: a reply occupies the speaker [domain 2] and reveals the robot's
  position. So chitchat has ZERO LLM by construction -- this module never
  imports or calls an LLM client. That is the enforcement of
  allow_llm_freeform=false, not a runtime branch that could be flipped.
* It also never echoes the user's ASR text (CMD-30 spirit): a greeting
  returns a preset liveness reply, not a repeat of what was heard.

Boundary (what this does NOT do): it does not classify (that is the
priority chain), does not dispatch actions, and does not decide whether a
phrase is chitchat vs overheard (16 S5.2.1). It is only the reply
generator for intents the caller has ALREADY classified as chitchat /
out_of_scope.

allow_llm_freeform is the Q-18-2 deferred switch (18 S12.5), default NOT
allowed. Constructing with it true raises rather than silently enabling an
unimplemented free-form path -- a fail-safe, not a fail-open stub.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Mapping, Optional


class ChitchatPresetError(RuntimeError):
    """chitchat.yaml is missing a required preset key."""


# The presets a real closed-set intent routes to (configs/intents.yaml):
# greeting J01, identity J02, help I05, plus the out_of_scope fallback.
# thanks/farewell/weather are NOT here -- 18 S12.3 merged them into
# out_of_scope (see chitchat.yaml divergence note).
_REQUIRED_TOP_KEYS = ("greeting", "identity", "help", "out_of_scope")


@dataclass
class ChitchatState:
    """Per-session chitchat state. Only the consecutive out_of_scope run
    matters (18 S12.5): any successful interaction breaks the streak."""

    consecutive_out_of_scope: int = 0

    def reset_out_of_scope(self) -> None:
        self.consecutive_out_of_scope = 0


class ChitchatResponder:
    """Preset reply generator. Built once from the loaded chitchat.yaml.

    respond() returns a preset STRING and never touches the LLM. The
    out_of_scope path mutates the passed-in ChitchatState so the count is
    per-session, not shared across operators."""

    __slots__ = ("_p", "_threshold")

    def __init__(self, presets: Mapping[str, Any],
                 *, allow_llm_freeform: bool = False) -> None:
        if allow_llm_freeform:
            # Q-18-2 default is NOT allowed (18 S12.5). Free-form is not
            # implemented; enabling it must be a deliberate design change,
            # not a silent stub -- raise rather than fail open.
            raise ChitchatPresetError(
                "allow_llm_freeform is not supported (Q-18-2 deferred; "
                "chitchat is preset-only, CMD-50..54)")
        missing = [k for k in _REQUIRED_TOP_KEYS if k not in presets]
        if missing:
            raise ChitchatPresetError(
                "chitchat.yaml missing preset key(s) %s" % missing)
        oos = presets["out_of_scope"]
        if "consecutive_threshold" not in oos:
            raise ChitchatPresetError(
                "chitchat.yaml out_of_scope missing consecutive_threshold")
        # Threshold is READ FROM CONFIG, never hardcoded (18 S12.5): a
        # hardcoded 3 would drift the moment the preset file changed.
        self._threshold = int(oos["consecutive_threshold"])
        self._p = presets

    def _greeting(self, time_of_day: Optional[str]) -> str:
        g = self._p["greeting"]
        # Deterministic pick: index 0. The liveness signal (18 S12.1) does
        # not need randomness, and a deterministic reply keeps tests and
        # field logs reproducible. time_of_day picks a variant when given.
        if time_of_day:
            variants = g.get("time_variant", {}).get(time_of_day)
            if variants:
                return variants[0]
        return g["default"][0]

    def respond(self, intent_name: str, state: ChitchatState,
                *, time_of_day: Optional[str] = None) -> str:
        """Return the preset reply for a chitchat / out_of_scope intent.

        Zero LLM, never echoes user text. Raises for an intent that is not
        a chitchat-family intent (the caller must not route action intents
        here). The out_of_scope path advances state.consecutive_out_of_scope
        and, on reaching the configured threshold, returns the capability
        overview (help.reply) and resets the run."""
        if intent_name == "greeting":
            state.reset_out_of_scope()
            return self._greeting(time_of_day)
        if intent_name == "identity":
            state.reset_out_of_scope()
            return self._p["identity"]["reply"]
        if intent_name == "help":
            state.reset_out_of_scope()
            return self._p["help"]["reply"]
        if intent_name == "out_of_scope":
            state.consecutive_out_of_scope += 1
            if state.consecutive_out_of_scope >= self._threshold:
                # 18 S12.5: consecutive out_of_scope means the command set
                # has a gap; announce the capability overview and restart
                # the run so it fires again only after another full streak.
                state.reset_out_of_scope()
                return self._p["help"]["reply"]
            return self._p["out_of_scope"]["reply"]
        raise ChitchatPresetError(
            "intent %r is not a chitchat-family intent (greeting/identity/"
            "help/out_of_scope only)" % intent_name)
