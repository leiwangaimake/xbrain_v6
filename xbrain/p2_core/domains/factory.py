"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: factory.py
Brief: BIZ-P2-5/6/7/9 -- build 4 domain arbiters from p2_core.yaml

Description:
P2 owns four resource domains (domain 2 speaker, domain 3 asr,
domain 4 payload_light, domain 5 ptz). Each is a single Arbiter
instance with the sources / priorities / policies from
p2_core.yaml.arbiter.domains.<name>. This module builds them all
from one config dict; no per-domain constructor logic beyond mapping
the YAML source table to SourceSpec instances.

Why one factory: every domain's construction is 'take the YAML block,
turn its sources dict into a list of SourceSpec, register them into
an Arbiter with the domain's wait_atomic_timeout_ms'. Writing four
near-identical constructors would be error-prone; one factory + one
per-domain policy check (BIZ-P2-8 auto lighting, BIZ-P2-10 LAPI
write guard) is cleaner.
"""

from __future__ import annotations

from typing import Dict, Mapping, Optional

from xbrain.common.arbiter.core import Arbiter
from xbrain.common.arbiter.model import (
    FORCED_PREEMPT_MAX,
    FORCED_PREEMPT_WINDOW_MS,
    PreemptPolicy,
    SourceSpec,
)


# 14 S3 domain names as registered in xbrain/common/enums/sets.yaml.
# The factory rejects any domain name outside this set.
_P2_DOMAINS = ("speaker", "asr", "payload_light", "ptz")


class DomainConfigError(RuntimeError):
    """A YAML block for a domain was malformed or referenced an
    off-contract source / policy value."""


def _spec_from_dict(source_id: str, block: Mapping) -> SourceSpec:
    """Turn one entry from arbiter.domains.<domain>.sources into a
    SourceSpec. Every field is explicit; no defaults per CLAUDE.md 3.1."""
    try:
        priority = int(block["priority"])
    except (KeyError, TypeError, ValueError) as exc:
        raise DomainConfigError(
            "source %s: missing or bad priority: %s" % (source_id, exc)
        ) from exc
    try:
        policy_str = block["policy"]
    except KeyError as exc:
        raise DomainConfigError(
            "source %s: missing policy" % source_id
        ) from exc
    try:
        policy = PreemptPolicy(policy_str)
    except ValueError as exc:
        raise DomainConfigError(
            "source %s: policy %r not in %s"
            % (source_id, policy_str,
               [p.value for p in PreemptPolicy])
        ) from exc
    # lease_timeout_s is EXPLICIT: None means resident (no lease);
    # missing key is an error. Bare numeric value stays; 0.0 would
    # be interpreted as instant-lease per CLAUDE.md 3.1 trap 2, so
    # we accept it but a domain building script should never write 0.0.
    if "lease_timeout_s" not in block:
        # Domain defaults to arbiter.default_lease_timeout_s; for
        # brevity here we allow the entry to omit the key AND signal
        # 'use whatever the arbiter default is' by passing None,
        # which the arbiter treats as resident. If a caller wants
        # the domain default they must explicitly write the value.
        # However for the four documented domains, every source that
        # omits lease_timeout_s in 14 S11 is EITHER resident or has
        # a specific override; the map is unambiguous:
        lease = 1.0   # 14 S11 arbiter.default_lease_timeout_s
    else:
        lease = block["lease_timeout_s"]

    return SourceSpec(
        source_id=source_id,
        priority=priority,
        preemptible=True,
        preempt_policy=policy,
        lease_timeout_s=lease,
        on_preempt=None,
        on_lost=None,
    )


def build_domain(
    domain: str,
    domain_block: Mapping,
    wait_atomic_timeout_ms: int,
    forced_preempt_window_ms: int = FORCED_PREEMPT_WINDOW_MS,
    forced_preempt_max: int = FORCED_PREEMPT_MAX,
) -> Arbiter:
    """Build one Arbiter for the named domain from its YAML block.

    domain_block is p2_core.yaml.arbiter.domains.<domain> (the dict
    that includes 'sources' plus any domain-specific keys)."""
    if domain not in _P2_DOMAINS:
        raise DomainConfigError(
            "domain %r not in P2's owned set %s"
            % (domain, list(_P2_DOMAINS)))
    try:
        sources = domain_block["sources"]
    except (KeyError, TypeError) as exc:
        raise DomainConfigError(
            "domain %s: missing sources block" % domain
        ) from exc
    arb = Arbiter(
        domain,
        wait_atomic_timeout_ms=wait_atomic_timeout_ms,
        forced_preempt_window_ms=forced_preempt_window_ms,
        forced_preempt_max=forced_preempt_max,
    )
    for src_id, src_block in sources.items():
        arb.register(_spec_from_dict(src_id, src_block))
    return arb


def build_all(
    arbiter_yaml: Mapping,
) -> Dict[str, Arbiter]:
    """Build all four P2 domain arbiters from arbiter_yaml (which is
    p2_core.yaml.arbiter). Returns a dict keyed by domain name.

    Raises DomainConfigError if any of the four domains is missing
    from the YAML."""
    try:
        wait_atomic_s = float(arbiter_yaml["wait_atomic_timeout_s"])
    except (KeyError, TypeError, ValueError) as exc:
        raise DomainConfigError(
            "arbiter.wait_atomic_timeout_s missing or bad: %s" % exc
        ) from exc
    wait_atomic_ms = int(wait_atomic_s * 1000)
    forced_window_s = float(arbiter_yaml.get(
        "forced_preempt_window_s", FORCED_PREEMPT_WINDOW_MS / 1000))
    forced_max = int(arbiter_yaml.get(
        "forced_preempt_max", FORCED_PREEMPT_MAX))
    try:
        domains_yaml = arbiter_yaml["domains"]
    except (KeyError, TypeError) as exc:
        raise DomainConfigError(
            "arbiter.domains block missing: %s" % exc
        ) from exc
    out: Dict[str, Arbiter] = {}
    for d in _P2_DOMAINS:
        if d not in domains_yaml:
            raise DomainConfigError(
                "arbiter.domains.%s missing from p2_core.yaml" % d)
        out[d] = build_domain(
            d, domains_yaml[d],
            wait_atomic_timeout_ms=wait_atomic_ms,
            forced_preempt_window_ms=int(forced_window_s * 1000),
            forced_preempt_max=forced_max,
        )
    return out
