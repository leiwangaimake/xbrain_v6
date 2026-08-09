"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: BIZ-P2-0 -- p2_core messaging layer (Zenoh session + registries)

Description:
Owns the Zenoh session, publisher / subscriber declaration registry,
cross-thread posting discipline, and startup publisher-face selfcheck
for p2_core. Also owns the state/audio + rt/audio/gate coherence
rule (device_fault must land on both keys same-tick or neither).

Sub-modules:
  * whitelist_gate.py  -- P2_CORE_PUB / P2_CORE_SUB subset checks at boot
  * p2_publisher.py    -- thread-checked pub wrapper around a session
  * p2_subscriber.py   -- SubscriberRegistry-backed sub wrapper
  * audio_state.py     -- state/audio + rt/audio/gate coherence

Design decision (BIZ-P2-0 doc #1 hard rule): every declare_subscriber
call in xbrain/p2_core/ MUST park its return value in a long-lived
container (self.x = / list.append / SubscriberRegistry.declare). A
bare `_ = sub` or a function-local `sub = ...` returns to garbage
collection and the Rust side silently unsubscribes. This module's
p2_subscriber.py enforces that by making SubscriberRegistry the ONLY
supported declare path in P2.
"""
