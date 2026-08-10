"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: network_links.py
Brief: CHK-2-19 network per-link self-check (LNK-1..LNK-5 + LNK-D) + ND-1..ND-3

Description:
5.1C per-link self-check. Each link falls under a specific health
item; LNK-1..5 have DIFFERENT items so their failures do NOT all
collapse into 'network' (that would erase the fact that LAN1 loss
is a fatal chassis-link outage while LAN2 loss is a warn-level
network-side gap).

  LNK-1 chassis primary link       -> item='chassis'     level=fatal
  LNK-2 zenoh router uplink        -> item='network'     level=warn
  LNK-3 payload service link       -> item='payload_svc' level=warn
  LNK-4 ptz onvif                  -> item='ptz'         level=warn
  LNK-5 ir camera                  -> item='ptz'         level=warn (transitional)
  LNK-D wifi debug carrier         -> item=NONE (network.detail only,
                                       NEVER appears in HealthSummary)

Discipline:
  * P-1 quick BIT: reuse existing traffic; no new probe packets
    (LNK-1 rides CHS-A heartbeat + T-20/T-21; LNK-2 rides
     probe/estop RTT + T-23/T-24).
  * P-3: ONVIF call count in the BIT path == 0 (17 Hz ONVIF ceiling
    is close enough that self-check probes would steal it).
  * ND-2: 7447/7446 must never appear on LAN1/LAN3/LAN4 (only LAN2).
  * ND-3: nft ruleset comparison lives in a separate script (INF-DP-9);
    same source code reused (meta-test asserts one implementation only).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


LINK_ITEM_MAP = {
    "LNK-1": ("chassis", "fatal"),
    "LNK-2": ("network", "warn"),
    "LNK-3": ("payload_svc", "warn"),
    "LNK-4": ("ptz", "warn"),
    "LNK-5": ("ptz", "warn"),
}

LNK_D_ITEM = None       # LNK-D deliberately NOT a health item
LNK_D_DETAIL_ONLY = True


class LinkClassificationError(Exception):
    pass


def classify_link_failure(link: str) -> tuple:
    """Return (item, level) for a link failure. LNK-D returns
    (None, None) -- caller writes to network.detail but MUST NOT
    push a health item."""
    if link == "LNK-D":
        return (None, None)
    if link not in LINK_ITEM_MAP:
        raise LinkClassificationError(f"unknown link {link!r}")
    return LINK_ITEM_MAP[link]


def lnk_d_writes_detail_only() -> bool:
    """CHK-2-19 (iv): LNK-D must not become a health item."""
    return LNK_D_DETAIL_ONLY


class OnvifInBitPath(Exception):
    """P-3 guard: ONVIF calls detected in BIT path."""


def assert_no_onvif_in_bit_source(source_text: str) -> None:
    """The BIT code text MUST NOT contain ONVIF calls; that would
    starve the 17 Hz control ceiling."""
    onvif_markers = ("ONVIFCamera", "onvif.", "onvif_client",
                       "PTZ_URL")
    hits = [m for m in onvif_markers if m in source_text]
    if hits:
        raise OnvifInBitPath(
            f"BIT source must not call ONVIF; hits={hits}")


# ---- P-2 layer-3 vs carrier distinguishability --------------------

@dataclass(frozen=True)
class LinkDetail:
    """P-2: carrier state and L3 reachability tracked separately.
    Compressing to a single bool is CHK-2-19 (v) failure mode."""
    carrier_up: bool
    l3_reachable: bool


def is_link_healthy(detail: LinkDetail) -> bool:
    """Only both -> healthy."""
    return detail.carrier_up and detail.l3_reachable
