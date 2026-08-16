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


# i_fix by fix_type (11 S3.2.1 / S4.5): the speed-gate quality factor. rtk_fixed
# full trust, rtk_float 0.4, everything else 0 (no autonomous motion). Derived
# HERE from fix_type, not carried in GnssFix.
_I_FIX_BY_TYPE = {
    "rtk_fixed": 1.0,
    "rtk_float": 0.4,
    "dgps": 0.0,
    "single": 0.0,
    "no_fix": 0.0,
}


# state/pose keeps the flat shape the HMI reader (p5 data_readers.pose_group) and
# the P4 query layer already read: heading_* / speed / fix_* at the top of `data`.
def assemble_pose(gnss_heading: Optional[Dict[str, Any]],
                  gnss_fix: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Merge a GnssHeading `data` and a GnssFix `data` into the state/pose `data`.

    Either None yields the safe shell for that half: no heading -> invalid/None,
    no fix -> fix_type None + position None -- the same shape a real L3/no_fix
    solution produces, so a consumer cannot tell 'not wired' from 'no fix', and
    both mean do-not-trust. i_fix is derived from fix_type here (11 S3.2.1)."""
    gh = gnss_heading or {}
    gf = gnss_fix or {}
    fix_type = gf.get("fix_type")
    return {
        # Heading half -- straight from GnssHeading (11 S3.3). heading_valid is
        # the SOLE downstream criterion (H-1); it is never synthesised here.
        "heading_rad": gh.get("heading_rad"),
        "heading_valid": bool(gh.get("heading_valid", False)),
        "heading_source": gh.get("source"),          # dual_antenna|cog|none
        "heading_level": gh.get("level"),            # 1|2|3
        # baseline_valid distinguishes the dual-antenna INT (true = fixed/integer
        # solve) from FLOAT (false) heading -- it varies within source=dual_antenna
        # (11 S3.3 L1 gate). Passed through so the HMI can show 双天线INT vs
        # 双天线FLOAT; None when the source carried no baseline (cog/none).
        "baseline_valid": gh.get("baseline_valid"),
        "speed_mps": gh.get("speed_mps"),
        "cov_rad": gh.get("cov_rad"),                # null at L3 (NAV-02)
        "i_heading": gh.get("i_heading"),
        "yaw_capable": bool(gh.get("yaw_capable", False)),
        # Fix half -- from GnssFix (11 S3.2). lat/lon/cov are None (not 0) when the
        # module had no position (NAV-02); fix_type is always present. i_fix is
        # derived from fix_type; num_satellites feeds G44 (was the 18-C S7 gap).
        "fix_type": fix_type,
        "lat": gf.get("lat"),
        "lon": gf.get("lon"),
        "alt": gf.get("alt"),
        "cov_h_m": gf.get("cov_h_m"),
        "i_fix": _I_FIX_BY_TYPE.get(fix_type) if fix_type else None,
        "num_satellites": gf.get("sats"),
    }


def mirror_clock(clock_status: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Mirror a received ClockStatus `data` into the state/clock `data`. P1 copies
    the fields VERBATIM and NEVER re-judges (CLK-A1 gives that power to
    rtk_driver alone; CLK-A2 says copy). Missing/None -> fail-safe sync=false.

    Carries the (mono_ref, utc_ref) anchor through, not just sync/source: 11 S3.11
    marks both required on state/clock and states 'any consumer can convert mono
    to UTC' from the pair. G24 query_time (18 S9.5) is exactly that consumer -- it
    reconstructs the current UTC as utc_ref + (now_mono - mono_ref), so the answer
    rides the rtk_driver's synced wall baseline via a MONOTONIC delta (CLK-C1),
    never a local wall-clock read. Absent anchor -> the fields stay None and the
    consumer falls back to 'unsynced' rather than fabricating a time."""
    cs = clock_status
    if not cs:
        return {"sync": False, "source": "none"}     # CLK-A3 fail-safe
    return {
        "sync": bool(cs.get("sync", False)),
        "source": cs.get("source", "none"),
        # (mono, wall) anchor for mono->UTC reconstruction (11 S3.11). Copied as
        # received; None when the source did not carry them (older publisher).
        "mono_ref": cs.get("mono_ref"),
        "utc_ref": cs.get("utc_ref"),
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
