"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cross_plane_compliance.py
Brief: INF-ZN-9 cross-plane forwarding compliance (RT-C3.b/c/e + CRL-1..6 + WL-G2)

Description:
The whitelist that governs cross-plane forwarding (GEN <-> RT via
chassis_relay) is a compile-time constant. INF-ZN-9 checks:

  * unlisted keys are NOT forwarded either way (GEN->RT or RT->GEN)
  * forwarded envelopes have seq/ts/src REBUILT on the target
    plane; orig_ts + orig_src fields preserve the source's values
    (byte-level comparison, not just payload)
  * each whitelisted entry has a SINGLE direction; the reverse
    direction MUST NOT forward
  * WL-G2: audio/broadcast can never appear in the rt/audio/*
    forwarding list (audio does not cross into the RT plane)
  * CRL-3: chassis_relay's whitelist is a compile-time constant
    array, NEVER read from a yaml/config file
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple


# WL-G2: audio/broadcast MUST NOT be in any forwarding entry.
FORBIDDEN_KEYS = frozenset({
    "audio/broadcast",
})


@dataclass(frozen=True)
class ForwardingEntry:
    """One whitelisted cross-plane forwarding rule."""
    src_key: str
    dst_key: str
    direction: str        # 'gen_to_rt' or 'rt_to_gen'

    def __post_init__(self) -> None:
        if self.direction not in ("gen_to_rt", "rt_to_gen"):
            raise ValueError(
                f"direction must be gen_to_rt/rt_to_gen, got "
                f"{self.direction!r}")


class WhitelistViolation(Exception):
    pass


def assert_no_forbidden_keys(entries: Iterable[ForwardingEntry]) -> None:
    """WL-G2 gate: audio/broadcast must never be forwarded."""
    for e in entries:
        if e.src_key in FORBIDDEN_KEYS or e.dst_key in FORBIDDEN_KEYS:
            raise WhitelistViolation(
                f"WL-G2 violation: forwarding entry "
                f"({e.src_key!r} -> {e.dst_key!r}) references "
                f"forbidden key(s)")


def assert_unique_direction(entries: Iterable[ForwardingEntry]) -> None:
    """Each src_key appears in ONE direction only. Reverse
    direction for the same key is a WhitelistViolation."""
    seen: dict = {}
    for e in entries:
        key = e.src_key
        if key in seen and seen[key] != e.direction:
            raise WhitelistViolation(
                f"key {key!r} has both {seen[key]!r} and "
                f"{e.direction!r} directions; a whitelist entry may "
                f"forward only ONE way")
        seen[key] = e.direction


@dataclass(frozen=True)
class RelayEnvelope:
    """The envelope shape chassis_relay produces on the far-plane
    output. `orig_ts` + `orig_src` MUST be the ORIGINAL source
    values; `seq` + `ts` + `src` MUST be the RELAY's own values."""
    seq: int
    ts_mono_ms: int
    src: str          # relay's identifier
    orig_ts_mono_ms: int
    orig_src: str
    payload: bytes


def check_envelope_rebuilt(env: RelayEnvelope,
                             producer_src: str,
                             producer_ts_mono_ms: int) -> None:
    """Verify seq/ts/src are the RELAY's; orig_* are the ORIGINAL's.
    A relay that forwards the envelope verbatim (without rebuilding
    the outer trio) fails this."""
    if env.src == producer_src:
        raise WhitelistViolation(
            f"relay envelope.src ({env.src!r}) equals producer "
            f"src ({producer_src!r}); envelope was NOT rebuilt")
    if env.ts_mono_ms == producer_ts_mono_ms:
        raise WhitelistViolation(
            f"relay envelope.ts equals producer ts; envelope was "
            f"NOT rebuilt")
    if env.orig_src != producer_src:
        raise WhitelistViolation(
            f"relay envelope.orig_src ({env.orig_src!r}) does not "
            f"preserve producer src ({producer_src!r})")
    if env.orig_ts_mono_ms != producer_ts_mono_ms:
        raise WhitelistViolation(
            f"relay envelope.orig_ts_mono_ms does not preserve "
            f"producer ts")


class CrlYamlReadForbidden(Exception):
    """CRL-3 gate: chassis_relay must not read a yaml/config file."""


def assert_relay_source_uses_compile_time_constant(
        source_text: str) -> None:
    """CRL-3: whitelist is a compile-time constant array. Any
    yaml / json / config read from chassis_relay code is a
    violation."""
    forbidden_markers = (
        "yaml.load", "yaml.safe_load", "json.load", "open(",
        "read_yaml", ".yaml\"", ".yml\"",
    )
    hits = [m for m in forbidden_markers if m in source_text]
    if hits:
        raise CrlYamlReadForbidden(
            f"chassis_relay source contains config-read markers "
            f"{hits}; CRL-3 requires the whitelist to be a compile-"
            f"time constant array")
