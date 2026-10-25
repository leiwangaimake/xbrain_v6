"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_outbound_keys.py
Brief: p4 has a cached publisher for every key an intent can be routed to

Description:
p4's main_wiring declares one GEN publisher per entry of OUTBOUND_KEYS at
startup, and _publish DROPS (with a warning) any key that has none. Until
2026-10-25 that tuple was a hand copy holding five of the nine keys
intent_dispatch routes to: every C-class mode command (cmd/mode), F-class
recording / geo command (cmd/teach, cmd/geo) and H-class system command
(cmd/system) was classified, confirmed, answered aloud -- and never sent.
The same shape of bug dropped cmd/estop on 2026-08-11.

What these cases pin:
  * every intent in the real registry (configs/intents.yaml) routes to a key
    p4 can publish -- the question asked of the tables p4 actually loads,
    not of a list typed into the test;
  * the four keys that were dropped, and cmd/estop (published by the bypass
    outside the routing tables), are each present by name.

Mutants this file kills (run by hand, see the commit):
  * the old five-key hand copy restored           -> registry case
  * per-id overrides left out of dispatch_keys()  -> registry case (cmd/geo,
    cmd/system and cmd/mode exist only as overrides)
  * CMD_ESTOP left out of OUTBOUND_KEYS           -> named-key case

Boundaries: membership only. Whether a subscriber exists for each key is
the contract table's question (11 S2.2), and declaring the publishers needs
a live Zenoh session, which the stack test covers.
"""
from __future__ import annotations

import pytest
import yaml

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.intent_dispatch import choose_key
from xbrain.p4_agent.runtime.main_wiring import OUTBOUND_KEYS

# Pure table reads; no robot, no Zenoh session.
pytestmark = pytest.mark.no_device

#: The registry p4 loads in production (the same file __main__ reads).
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"


def test_every_registered_intent_routes_to_a_publishable_key():
    """No intent in the real registry may be routed into the drop path.

    The failure message lists every offender with its key, so a red run
    says which class went dark rather than only that something did.
    """
    # Load exactly what p4 loads, so a new registry row is covered at once.
    with open(_INTENTS, encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))
    # Every (id, key) pair whose key has no cached publisher.
    dropped = sorted((e.id, choose_key(e.id)) for e in reg.entries
                     if choose_key(e.id) not in OUTBOUND_KEYS)
    assert dropped == []


@pytest.mark.parametrize("key", [
    "cmd/mode",     # C class: alarm / broadcast mode switches (p2)
    "cmd/teach",    # F01-F10: recording session (p3)
    "cmd/geo",      # F11-F15: stored-object CRUD (p3)
    "cmd/system",   # H class: system actions
    "cmd/estop",    # safety bypass, published outside the routing tables
])
def test_each_previously_dropped_key_has_a_publisher(key):
    """Named by hand on purpose: these are the keys that were lost once.

    The registry case alone would stay green if the registry stopped using
    one of them; this case keeps the publisher even then, because the
    bypass (estop) never goes through the registry at all.
    """
    # Membership in the tuple main_wiring declares publishers from.
    assert key in OUTBOUND_KEYS
