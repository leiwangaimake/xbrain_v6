"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: turn_labels.py
Brief: one turn's entry (channel + modality) -> the provenance value each 11 command contract asks for

Description:
Every command P4 builds records where it came from, and each contract spells
that differently:

  key          field            11 closed set                         read by
  cmd/task     source           (example "hmi"; P3's explicit table   P3: tasks.source and the
                                accepts voice/text/hmi/cloud/wecom)   scheduler priority (15 S4.2)
  cmd/mode     source           voice_local|cloud|hmi|wecom|...       S7.3 (audit, arbitration)
  cmd/system   origin           voice|cloud|wecom|hmi                 S7.15.4 permission matrix
  cmd/geo      origin           voice|cloud|wecom|hmi                 S7.9.5 write matrix
  cmd/teach    issuer.channel   local_voice|cloud_voice|cloud_text|   S12A.4 audit and the
                                wecom|hmi                             non_local_issuer warning
  cmd/motion/  channel          mic_local|cloud|wecom|hmi             MO-2 (built in
  intent                                                              motion_intent_request)

Until 2026-10-26 all of them came from ONE constructor constant, source=
"voice", while a single orchestrator served the microphone, cloud text, cloud
voice and the HMI. So a task sent from the cloud was queued as on-site
(priority 40, not 80), cmd/mode carried "voice" (not even in its closed set),
the system permission matrix judged every command as local voice, and the geo
origin was the literal expression `"voice" if ... else "voice"`.

This module is the single mapping. The orchestrator passes the turn's
channel (mic_local | cloud | wecom | hmi) and modality (voice | text) and asks
for the value per contract; every function raises on an out-of-set input
(11 S13.6), and the builders re-check their own closed set on what they
receive.

Impossible combinations are refused, not repaired: the local microphone is
always voice and the HMI is always text; asking for "mic_local + text" means
a caller bug.

Boundaries: labels only. What a consumer does with them -- P3's priority,
P2's permission checks -- stays with the consumer.
"""
from __future__ import annotations

#: The four entries a turn can come in on (11 S8.7.6 channel / MO-2 set).
TURN_CHANNELS = frozenset({"mic_local", "cloud", "wecom", "hmi"})
#: How the turn was given: spoken (ASR ran) or typed.
MODALITIES = frozenset({"voice", "text"})

#: cmd/task source values P4 sends. P3's ingest table maps them onto the
#: five-value tasks.source (voice/hmi -> local, cloud, wecom).
TASK_SOURCES = frozenset({"voice", "cloud", "wecom", "hmi"})
#: 11 S7.3 ModeCommand.source (the example comment's closed set). P4 only
#: produces the first four; auto_track / bit belong to other publishers.
MODE_SOURCES = frozenset({"voice_local", "cloud", "hmi", "wecom",
                          "auto_track", "bit"})
#: 11 S7.15 SystemCommand.origin and S7.9.5 GeoCommand.origin.
ORIGINS = frozenset({"voice", "cloud", "wecom", "hmi"})
#: 11 S12A.4 TeachCommand issuer.channel.
TEACH_ISSUER_CHANNELS = frozenset({"local_voice", "cloud_voice",
                                   "cloud_text", "wecom", "hmi"})

#: channel -> the "voice / cloud / wecom / hmi" spelling shared by cmd/task
#: source and the cmd/system / cmd/geo origin.
_PLAIN = {"mic_local": "voice", "cloud": "cloud", "wecom": "wecom",
          "hmi": "hmi"}  # one table, three contracts
#: channel -> ModeCommand.source (S7.3 spells the microphone voice_local).
_MODE = {"mic_local": "voice_local", "cloud": "cloud", "wecom": "wecom",
         "hmi": "hmi"}  # only the mic differs


def check_turn(channel: str, modality: str) -> None:
    """Raise ValueError unless (channel, modality) is a possible turn.

    The local microphone only hears speech and the HMI only sends text;
    cloud and wecom carry both (cmd/voice_text and audio/voice_in).
    """
    if channel not in TURN_CHANNELS:
        raise ValueError("channel %r not in %s"
                         % (channel, sorted(TURN_CHANNELS)))
    if modality not in MODALITIES:
        raise ValueError("modality %r not in %s"
                         % (modality, sorted(MODALITIES)))
    if (channel, modality) in (("mic_local", "text"), ("hmi", "voice")):
        raise ValueError("impossible turn: %s via %s" % (modality, channel))


def task_source(channel: str) -> str:
    """cmd/task source (TaskCommand, task control) for the turn's channel."""
    check_turn(channel, "text" if channel == "hmi" else "voice")
    return _PLAIN[channel]  # voice / cloud / wecom / hmi


def mode_source(channel: str) -> str:
    """cmd/mode ModeCommand.source (11 S7.3) for the turn's channel."""
    check_turn(channel, "text" if channel == "hmi" else "voice")
    return _MODE[channel]  # voice_local / cloud / wecom / hmi


def origin(channel: str) -> str:
    """cmd/system and cmd/geo origin (11 S7.15 / S7.9.5) for the channel.

    One function for both keys: the two contracts spell it identically, and
    two copies of one table are how they would drift apart.
    """
    check_turn(channel, "text" if channel == "hmi" else "voice")
    return _PLAIN[channel]  # voice / cloud / wecom / hmi


def teach_issuer_channel(channel: str, modality: str) -> str:
    """cmd/teach issuer.channel (11 S12A.4): the only label that also
    records the modality, so a cloud turn is cloud_voice or cloud_text."""
    check_turn(channel, modality)
    if channel == "mic_local":
        return "local_voice"  # the microphone: always spoken
    if channel == "cloud":
        return "cloud_voice" if modality == "voice" else "cloud_text"
    return channel  # wecom / hmi: S12A.4 has one value each
