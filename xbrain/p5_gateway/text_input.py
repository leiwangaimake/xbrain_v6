"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: text_input.py
Brief: GWY-P4-42 (32.J) -- P5 builds the cmd/voice_text message

Description:
11 S8.7.5 / 17 P5: text commands from the HMI (P5's own backend), WeChat
(E channel) and cloud text (T channel) enter the robot on cmd/voice_text.
P5 is the gateway; this builds the cmd/voice_text envelope P5 publishes for
P4 to consume. The message is TEXT -- it never carries audio and never
touches ASR (11 S9178).

Boundary: this only SHAPES the message. Classification, the channel gate
(H03f cloud-only), and the reply all happen on the P4 side
(runtime/text_channel.py). Keeping the shape here and the policy there
avoids two copies of the channel rules.
"""
from __future__ import annotations

from typing import Any, Dict, Optional


# 11 S8.7.5 channel closed set.
_CHANNELS = frozenset({"cloud", "wecom", "hmi"})


class TextInputError(RuntimeError):
    """A text-input field is missing or outside the closed set."""


def build_voice_text_msg(
    channel: str,
    text: str,
    cmd_id: str,
    issued_ts: float,
    *,
    require_tts_reply: bool = True,
    slots: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Build the cmd/voice_text message (11 S8.7.5).

    channel MUST be in {cloud, wecom, hmi}; text MUST be non-empty. slots
    carries any fields the HMI parsed structurally (e.g. force_step for
    H03), so the P4 channel gate can apply the H03f rule without re-parsing.
    issued_ts is the WALL-clock issue time (an audit timestamp, not a
    timeout -- 11 uses issued_ts for the ledger, CLK-C1 governs deadlines
    which are computed on the monotonic clock downstream).
    """
    if channel not in _CHANNELS:
        raise TextInputError(
            "channel %r not in %s" % (channel, sorted(_CHANNELS)))
    if not text or not text.strip():
        raise TextInputError("text must be non-empty")
    msg: Dict[str, Any] = {
        "cmd_id": cmd_id,
        "channel": channel,
        "text": text,
        "require_tts_reply": require_tts_reply,
        "issued_ts": issued_ts,
    }
    if slots:
        msg["slots"] = dict(slots)
    return msg
