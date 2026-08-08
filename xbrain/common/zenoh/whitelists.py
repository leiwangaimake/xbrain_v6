"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: whitelists.py
Brief: INF-ZN-7 committed cross-plane whitelists -- generated snapshot,
       drift-gated by scripts/doccheck/whitelist_gen.py --check

Description:
This module is a GENERATED SNAPSHOT of the five cross-plane process whitelists
that 11 S1.1.6 (hand tables) + WL-G1 (S2.2 generation) define. The runtime
imports these constants; scripts/doccheck/whitelist_gen.py --check re-derives
them from the doc and refuses commit on any delta, which is the drift gate
S1.1.6 asks for.

Two write paths, one truth:
  * perception / p1_motion / chassis_relay: HAND tables in S1.1.6 (RT-C3.b's
    literal "逐条列出" -- their surface is small and closed, so a hand table
    reads cleanly). Every entry below is inline-annotated with its S1.1.6
    row code (PC-1, P1-N, CR-N) so a review can find the truth row in one grep.
  * p2_core / p4_agent: GENERATED from S2.2 by WL-G1. 25-30 keys per process,
    changing every round, so a hand-copied table would silently lag ("抄本
    必然滞后" -- S1.1.6 note). No row codes on these -- their truth is the
    S2.2 pub/sub columns for that process name.

Rules the runtime and CI both rely on:
  * PUB/SUB sets are frozen (frozenset) so no consumer can amend a whitelist
    in place at run time; a rogue "add one key just this once" would defeat
    the whole registration surface.
  * Regenerate with: python scripts/doccheck/whitelist_gen.py --emit
    Diff the JSON, update this file, commit -- the two must move together
    or --check refuses. Never hand-edit one process's set to "fix a runtime
    surprise": the runtime surprise means the doc has a real change, and
    the doc is what should be edited.
  * WL-G1 (p2_core / p4_agent) is generated from S2.2 rather than
    hand-copied, so it inherits whatever S2.2 grew. WL-G3 wildcards are
    refused at the generator, so no wildcard can enter these sets.
"""

# --------------------------------------------------------------------------
# 1) perception -- hand table (11 S1.1.6 ①). 2 pub, 0 sub.
# perception publishes its own detections + its own events; it never
# subscribes on the general plane (RT-C3 keeps its RT inputs RT-side).
# --------------------------------------------------------------------------

PERCEPTION_PUB = frozenset({
    "state/targets",                              # PC-1: PerceptionFrame w/ mask
    "event/{severity}/perception",                # PC-2: own-produced events only
})
# Not a subscriber on the general plane. Left explicit rather than absent so a
# grep for PERCEPTION_SUB finds the deliberate emptiness, not a missing name.
PERCEPTION_SUB = frozenset()


# --------------------------------------------------------------------------
# 2) p1_motion -- hand table (11 S1.1.6 ②). P1-1..P1-23. Direction column
# governs membership: P1-2 (state/robot) is "--", pointing readers to
# chassis_relay CR-4 rather than declaring anything itself, and MUST NOT
# count. Every PUB row here is a S2.2.2/S2.2.3 declaration P1 owns; every
# SUB row is a command or RT-forward-source P1 consumes.
# --------------------------------------------------------------------------

P1_MOTION_PUB = frozenset({
    "state/pose",                                 # P1-1: 10 Hz fused pose
    "cmd/motion/relative_move/status",            # P1-6: relative_move progress
    "event/{severity}/motion",                    # P1-10: own motion events
    "state/motion/path_progress",                 # P1-12: patrol progress (U07a)
    "state/clock",                                # P1-13: RT clock mirror to GEN
    "state/fence",                                # P1-14: fence runtime state
    "cmd/fence/ack",                              # P1-16: ack of fence commands
    "state/teleop",                               # P1-17: teleop mux state
    "cmd/config/ack",                             # P1-18: config apply ack
    "state/arb/{domain}",                         # P1-22: S7A.8 pub-only for motion
    "event/{severity}/arbitration",               # P1-23: motion arb audit events
})
P1_MOTION_SUB = frozenset({
    "cmd/motion/behavior",                        # P1-3: behavior switch
    "cmd/motion/factor",                          # P1-4: speed factor (1 Hz)
    "cmd/motion/relative_move",                   # P1-5: one-shot relative move
    "cmd/chassis/mode",                           # P1-7: mode forward (via P1)
    "cmd/chassis/light",                          # P1-8: light forward (via P1)
    "cmd/config",                                 # P1-9: hot-updates (P1 filters)
    "cmd/motion/route",                           # P1-11: route push (patrol path)
    "cmd/fence",                                  # P1-15: fence table push
    "cmd/teleop",                                 # P1-19: teleop input mux'd
    "rt/chassis/fault",                           # P1-20: chassis_relay-dead path
    "cmd/estop",                                  # P1-21: soft-estop, one of 4 subs
})


# --------------------------------------------------------------------------
# 3) chassis_relay -- hand table (11 S1.1.6 ③). 12 forwarding rows, no own
# business data. Direction column is GEN->RT (relay SUBS the general-plane
# key it forwards inward) or RT->GEN (relay PUBS the general-plane key it
# forwards outward). All rows are safety- or state-critical; two carry the
# ESTOP path (CR-1 in, CR-10 out) and MUST reach the wire whichever way
# the process partition around them changes.
# --------------------------------------------------------------------------

CHASSIS_RELAY_PUB = frozenset({
    "rt/safety/probe/pong",                       # CR-3: probe response (RT->GEN)
    "rt/chassis/state",                           # CR-4: 10 Hz chassis state
    "rt/chassis/power",                           # CR-5: power / battery
    "rt/chassis/basic",                           # CR-6: basic 2 Hz
    "rt/chassis/motion",                          # CR-7: motion 10 Hz
    "rt/chassis/device",                          # CR-8: device 2 Hz
    "rt/chassis/fault",                           # CR-9: fault events
    "rt/safety/estop/ack",                        # CR-10: estop ack (safety key)
    "rt/chassis/ctrl/ack",                        # CR-12: enable/lock ack (R-2)
})
CHASSIS_RELAY_SUB = frozenset({
    "cmd/estop",                                  # CR-1: soft estop in (Q0)
    "probe/estop/ping",                           # CR-2: probe ping
    "cmd/chassis/ctrl",                           # CR-11: R-2 enable/lock (Q0)
})


# --------------------------------------------------------------------------
# 4) p2_core -- WL-G1 GENERATED from S2.2 (11 S1.1.6 ④). ~50 keys total.
# Every entry appears because a S2.2.n row named "p2_core" in the publisher
# or subscriber cell; regenerate to update. WL-G3 forbids wildcards in the
# generated sets, so a wildcard here means the generator or the doc is bad.
# --------------------------------------------------------------------------

P2_CORE_PUB = frozenset({
    "cmd/arb/{domain}/req",                       # arb req (as consumer)
    "cmd/audio/speak",                            # speak to payload (self-consume)
    "cmd/audio/speak/ack",                        # ack of own speak
    "cmd/chassis/light",                          # mode-driver LIGHT out
    "cmd/chassis/mode",                           # mode-driver MODE out
    "cmd/config/ack",                             # config ack (its scope)
    "cmd/fence/ack",                              # ack for fence updates
    "cmd/mode/ack",                               # mode-command ack
    "cmd/motion/behavior",                        # mode-driver behavior out
    "cmd/motion/factor",                          # 1 Hz speed factor
    "cmd/motion/intent/ack",                      # intent-command ack
    "cmd/motion/relative_move",                   # bypass A01 (voice estop)
    "cmd/payload/ack",                            # payload-command ack
    "cmd/ptz/ack",                                # ptz-command ack (domain 5)
    "cmd/system/ack",                             # system-command ack
    "health/bit",                                 # BIT results (self-published)
    "health/summary",                             # overall health digest
    "rt/audio/gate",                              # AsrGate authority (RT)
    "rt/audio/lease",                             # audio domain lease
    "rt/audio/mic",                               # local USB MIC 50 Hz (RT)
    "state/arb/{domain}",                         # arb state for owned domains
    "state/audio",                                # aggregate audio state
    "state/mode",                                 # mode SM output
})
P2_CORE_SUB = frozenset({
    "audio/broadcast",                            # broadcast audio in
    "audio/voice_in",                             # voice-in (loopback for own)
    "cmd/arb/{domain}/req",                       # arb requests inbound
    "cmd/audio/speak",                            # speak commands routed
    "cmd/audio/speak/ack",                        # ack observed for accounting
    "cmd/estop",                                  # soft estop (one of 4 subs)
    "cmd/fence",                                  # fence updates for mode gates
    "cmd/mode",                                   # mode switch commands
    "cmd/motion/intent",                          # motion intents (voice)
    "cmd/motion/relative_move/status",            # observe P1's relative_move
    "cmd/payload",                                # payload commands
    "cmd/ptz",                                    # ptz commands (arbiter feeds)
    "cmd/system",                                 # system commands (bit / etc)
    "health/ai",                                  # AI service health
    "health/bit",                                 # BIT retry / redisplay
    "rt/audio/play",                              # playback audio (RT sub)
    "state/chassis_basic",                        # chassis basic state
    "state/chassis_device",                       # device state (fan/temp)
    "state/chassis_motion",                       # chassis motion state
    "state/clock",                                # clock sync state
    "state/fence",                                # fence runtime state
    "state/link",                                 # link (cloud/hmi) state
    "state/pose",                                 # pose for mode gates
    "state/power",                                # power for mode gates
    "state/robot",                                # robot state (hes lock)
    "state/targets",                              # perception targets (D-mode)
})


# --------------------------------------------------------------------------
# 5) p4_agent -- WL-G1 GENERATED from S2.2 (11 S1.1.6 ⑤). ~42 keys total.
# p4_agent is the intent pipeline; publishers are the commands it EMITS
# from voice / text, subscribers are the state feeds it needs to answer
# queries and route commands. WL-G3 wildcards are forbidden here too.
# --------------------------------------------------------------------------

P4_AGENT_PUB = frozenset({
    "cmd/approval",                               # L3 approval issue
    "cmd/arb/{domain}/req",                       # domain 6 (gpu) req
    "cmd/audio/speak",                            # voice reply / TTS
    "cmd/chassis/ctrl",                           # enable / unlock (R-2)
    "cmd/config/ack",                             # config ack (its scope)
    "cmd/estop",                                  # A01 bypass emits estop
    "cmd/fence/ack",                              # fence-command ack
    "cmd/geo",                                    # geo objects (voice)
    "cmd/mode",                                   # mode switch (voice)
    "cmd/motion/intent",                          # motion intents (voice)
    "cmd/payload",                                # payload commands (voice)
    "cmd/ptz",                                    # ptz commands (voice)
    "cmd/system",                                 # system commands (voice)
    "cmd/task",                                   # B-class task submit
    "cmd/teach",                                  # teach commands
    "health/ai",                                  # AI service health emit
    "rt/audio/play",                              # playback (RT publish)
    "state/arb/{domain}",                         # arb state for gpu domain
    "state/voice_turn",                           # turn-by-turn voice state
})
P4_AGENT_SUB = frozenset({
    "audio/broadcast",                            # broadcast audio in
    "audio/voice_in",                             # voice in from mic
    "cmd/audio/speak/ack",                        # speak completion feedback
    "cmd/chassis/ctrl/ack",                       # enable / unlock ack
    "cmd/estop/ack",                              # estop ack observation
    "cmd/fence",                                  # fence commands (for context)
    "cmd/motion/intent/ack",                      # intent ack observation
    "cmd/motion/relative_move/status",            # relative_move progress
    "cmd/system/ack",                             # system ack
    "cmd/voice_text",                             # text-mode voice input
    "rt/audio/gate",                              # gate for voice input
    "rt/audio/lease",                             # domain lease observed
    "rt/audio/mic",                               # mic for ASR
    "state/approval",                             # L3 approval queue
    "state/arbitration",                          # 7-domain arb summary
    "state/audio",                                # audio state (queries)
    "state/fence",                                # fence runtime state
    "state/geo/manifest",                         # geo manifest (queries)
    "state/link",                                 # link state
    "state/mode",                                 # mode state (queries)
    "state/pose",                                 # pose (position queries)
    "state/task",                                 # task state (queries)
    "state/teach",                                # teach state
})


# --------------------------------------------------------------------------
# Aggregate map, so a caller can look up by process name without importing
# nine individual constants. Kept AFTER the individual definitions so those
# stay the visible truth (the map is just a convenience index).
# --------------------------------------------------------------------------

WHITELISTS = {
    "perception":    {"pub": PERCEPTION_PUB,    "sub": PERCEPTION_SUB},
    "p1_motion":     {"pub": P1_MOTION_PUB,     "sub": P1_MOTION_SUB},
    "chassis_relay": {"pub": CHASSIS_RELAY_PUB, "sub": CHASSIS_RELAY_SUB},
    "p2_core":       {"pub": P2_CORE_PUB,       "sub": P2_CORE_SUB},
    "p4_agent":      {"pub": P4_AGENT_PUB,      "sub": P4_AGENT_SUB},
}
