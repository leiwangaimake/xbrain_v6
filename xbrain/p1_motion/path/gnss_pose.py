"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: gnss_pose.py
Brief: RT-plane rt/gnss/heading + rt/clock/status -> GEN state/pose + state/clock

Description:
p1_motion is the ONE cross-plane bridge for GNSS (11 S1.1.6 whitelist: p1 订
rt/gnss/*, 发 state/pose). rtk_driver publishes GnssHeading on the RT plane; the
general-plane consumers (P4 queries, P5/HMI) only ever read state/pose +
state/clock. This module is the pure, testable core of that bridge: it maps a
received GnssHeading data dict into the state/pose data dict, and mirrors a
ClockStatus into state/clock (11 S2.2.12 P1-13 -- P1 mirrors, never re-judges;
CLK-A2).

Boundary and current gaps:
  * assemble_pose maps only the HEADING half of state/pose. The fix half
    (fix_type / lat / lon / cov_h_m / i_fix) comes from rt/gnss/fix, which
    rtk_driver does not publish yet -- those stay None here, NOT defaulted to a
    plausible value (a fake fix_type would over-trust position, NAV-02 / 3.1).
  * mirror_clock is fail-SAFE: no ClockStatus (rtk_driver clock not wired yet, or
    stale) -> sync=false / source=none (CLK-A3), never fail-open.

The two functions are pure (dict in, dict out) so the mutation tests of 3.3 can
drive them without a Zenoh session; the wiring that subscribes/publishes lives in
runtime/main_wiring.py.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional


# state/pose keeps the flat shape the HMI reader (p5 data_readers.pose_group) and
# the P4 query layer already read: heading_* / speed / fix_* at the top of `data`.
def assemble_pose(gnss_heading: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Map a received GnssHeading `data` dict into the state/pose `data` dict.

    None (no heading received yet) yields the safe L3 pose: heading invalid, every
    field None -- the same shape a real L3 GnssHeading produces, so a consumer
    cannot tell 'not wired yet' from 'no heading', and both mean do-not-trust."""
    gh = gnss_heading or {}
    return {
        # Heading half -- straight from GnssHeading (11 S3.3). heading_valid is
        # the SOLE downstream criterion (H-1); it is never synthesised here.
        "heading_rad": gh.get("heading_rad"),
        "heading_valid": bool(gh.get("heading_valid", False)),
        "heading_source": gh.get("source"),          # dual_antenna|cog|none
        "heading_level": gh.get("level"),            # 1|2|3
        "speed_mps": gh.get("speed_mps"),
        "cov_rad": gh.get("cov_rad"),                # null at L3 (NAV-02)
        "i_heading": gh.get("i_heading"),
        "yaw_capable": bool(gh.get("yaw_capable", False)),
        # Fix half -- from rt/gnss/fix, NOT published by rtk_driver yet. Left None
        # rather than defaulted; a fabricated fix_type would let a consumer act on
        # a position that was never measured (3.1 fail-silent).
        "fix_type": None,
        "lat": None,
        "lon": None,
        "alt": None,
        "cov_h_m": None,
        "i_fix": None,
    }


def mirror_clock(clock_status: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Mirror a received ClockStatus `data` into the state/clock `data`. P1 copies
    sync/source verbatim and NEVER re-judges (CLK-A1 gives that power to
    rtk_driver alone; CLK-A2 says copy). Missing/None -> fail-safe sync=false."""
    cs = clock_status
    if not cs:
        return {"sync": False, "source": "none"}     # CLK-A3 fail-safe
    return {
        "sync": bool(cs.get("sync", False)),
        "source": cs.get("source", "none"),
    }


def stamp_envelope(data: Dict[str, Any], *, rid: str, boot: str, seq: int,
                   src: str, ts_sync: bool) -> Dict[str, Any]:
    """Wrap `data` in the 11 S3.0 envelope. ts is wall ms (align/log only); mono is
    steady ms (CLK-C1). ts_sync is the copied ClockStatus.sync (CLK-A2), never a
    local judgement."""
    return {
        "v": 1,
        "rid": rid,
        "ts": int(time.time() * 1000.0),          # WALL-CLOCK-OK(align/log)
        "mono": int(time.monotonic() * 1000.0),   # CLK-C1 monotonic
        "boot": boot,
        "seq": seq,
        "src": src,
        "ts_sync": ts_sync,
        "data": data,
    }
