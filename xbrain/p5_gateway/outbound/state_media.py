"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: state_media.py
Brief: CHK-1-42 state/media publish (0.1 Hz + change-triggered, credential-free)

Description:
17 §7 state/media reports the reachability of the on-robot media
endpoints (visible-light camera, IR camera, RGBD). Discipline:

  * publish on ANY reachability change + 0.1 Hz heartbeat floor
    (missing the "change-triggered" half means an operator wouldn't
    see 'reachable=false' for up to 10 seconds after a cable pull)
  * every endpoint entry has exactly FOUR fields:
      name, kind ('rgb'/'ir'/'rgbd'), reachable, last_ok_mono_ms
    (monotonic clock; wall clock would fail clock_scan.py)
  * NEVER embed credential fields (password / passwd / secret /
    credential) in the payload; a static scan reports any such
    key in the produced dict
  * semantic scope: our-side reachability only. NO probing of the
    Qt-side cloud endpoints -- that is a Qt-side concern
  * QoS profile: Q2_state
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


HEARTBEAT_PERIOD_MS = 10_000       # 0.1 Hz
ENDPOINT_KINDS = frozenset({"rgb", "ir", "rgbd"})
FORBIDDEN_CREDENTIAL_KEYS = frozenset({
    "password", "passwd", "secret", "credential",
    "token", "api_key",
})


class StateMediaCredentialLeak(Exception):
    """Payload constructor tried to include a credential field."""


@dataclass(frozen=True)
class Endpoint:
    name: str
    kind: str
    reachable: bool
    last_ok_mono_ms: int

    def __post_init__(self) -> None:
        if self.kind not in ENDPOINT_KINDS:
            raise ValueError(
                f"Endpoint kind {self.kind!r} not in {sorted(ENDPOINT_KINDS)}")


def build_payload(endpoints: List[Endpoint]) -> dict:
    """Assemble the state/media payload. Refuses to add any field
    whose name is in FORBIDDEN_CREDENTIAL_KEYS."""
    items = []
    for ep in endpoints:
        items.append({
            "name": ep.name,
            "kind": ep.kind,
            "reachable": ep.reachable,
            "last_ok_mono_ms": ep.last_ok_mono_ms,
        })
    payload = {"endpoints": items}
    scan_credential_keys(payload)
    return payload


def scan_credential_keys(obj) -> None:
    """Recursive credential-name check. Payload construction MUST
    NEVER include any FORBIDDEN key."""
    if isinstance(obj, dict):
        for k in obj:
            if k in FORBIDDEN_CREDENTIAL_KEYS:
                raise StateMediaCredentialLeak(
                    f"state/media payload contains forbidden "
                    f"credential key {k!r}")
        for v in obj.values():
            scan_credential_keys(v)
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            scan_credential_keys(v)


@dataclass
class StateMediaPublisher:
    """change-triggered + 0.1 Hz heartbeat scheduler."""
    last_snapshot: Optional[dict] = None
    last_publish_mono_ms: int = 0

    def observe(self, endpoints: List[Endpoint],
                  now_mono_ms: int) -> Optional[dict]:
        """Return the payload to publish this tick, or None."""
        payload = build_payload(endpoints)
        should_publish = False
        if payload != self.last_snapshot:
            should_publish = True    # change-triggered
        elif now_mono_ms - self.last_publish_mono_ms >= HEARTBEAT_PERIOD_MS:
            should_publish = True    # heartbeat
        if not should_publish:
            return None
        self.last_snapshot = payload
        self.last_publish_mono_ms = now_mono_ms
        return payload


REQUIRED_QOS_PROFILE = "Q2_state"


def assert_qos_profile(profile: str) -> None:
    """CHK-1-42 (v): must be Q2_state; other profiles fail."""
    if profile != REQUIRED_QOS_PROFILE:
        raise ValueError(
            f"state/media QoS profile must be {REQUIRED_QOS_PROFILE!r}, "
            f"got {profile!r}")
