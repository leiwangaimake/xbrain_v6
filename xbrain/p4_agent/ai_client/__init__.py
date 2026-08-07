"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The AI-service gateway (16 S14) -- the ONLY place business code reaches
       ai_asr / llama-server, with the S8.13.5 error map and the S9.3 breaker

Description:
16 S14 / CLAUDE.md 4.1: business modules must NOT import requests and call the AI
services directly; they go through ai_client. This package is that boundary. It
exposes the error map (11 S8.13.5, the gateway's one job of turning HTTP status
into a closed-set code) and the circuit breaker (16 S9.3), which are pure and
tested here; the async httpx transcribe/chat callers that use them are added
alongside (they need the live services or a mock transport to exercise).

This first landing is the gateway CORE -- map + breaker -- because it is the part
that is pure logic and fully testable without a running service, and it is where
the two failure modes the rest of the system depends on are decided: which code a
failure becomes, and when to stop hammering a dead service.
"""

from xbrain.p4_agent.ai_client.breaker import BreakerState, CircuitBreaker
from xbrain.p4_agent.ai_client.errors_map import (
    AS7_TIMEOUT_S, MappedError, map_status, map_transport_error,
)

__all__ = [
    "map_status", "map_transport_error", "MappedError", "AS7_TIMEOUT_S",
    "CircuitBreaker", "BreakerState",
]
