"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Fail-safe branches for the three permanently-unavailable capability blocks (INF-DB-3)

Description:
What this package is. INF-DB-3 lands the fail-loud branches for three blocks of
functionality that are unavailable today and stay that way until external facts
arrive: V-33 (side/rear LiDAR coverage), 11 T-PTZ-1 (PTZ homing), and 18 T-PTZ-3
(PTZ speed calibration). These are NOT defects -- they are the normal result of a
fail-safe taking effect (21 S1). The package's job is to make each block refuse
or degrade honestly, in one place, so that the "make it available" temptation has
nowhere to hide: none of these branches may be lifted here (不得为了让功能可用
而豁免), because the cloud writeup and hardware measurements that would lift them
are, by definition, not in this repo.

How it is laid out.
  * outcome.py -- the one FailSafeResult type all three branches return, plus the
    ack-status and confirmation-level constants they speak in.
  * rotation.py -- V-33: the sweep-ring model and the E_BUSY block on every
    in-place spin (18 A09~A12, C07), and the static L1b for lateral moves.
  * ptz.py -- 11 T-PTZ-1 (homing preset) and 18 T-PTZ-3 (speed / relative turn /
    zoom query); they share the PTZ subsystem and the accepted-is-not-arrival trap.

Why this file has no code. It follows the same rule as its parent
xbrain/common/__init__.py: subpackages are imported explicitly by the module that
needs one (from xbrain.common.failsafe.rotation import rotation_failsafe). A flat
re-export here would be a second, drift-prone copy of every submodule's public
list, and it would earn no clarity -- callers already say which block they mean.

Where it sits and what it depends on. xbrain/common/, imported by the gateway /
P4 / P2 code that routes an intent to its fail-safe. It depends only on the other
common/ layers (errors for the E_* codes, config for the MISSING sentinel), holds
no ROS, no I/O, and reads no config -- it takes resolved values as arguments, so
no safety default lives here (CLAUDE.md 3.1).

Relationship to INF-DB-4. INF-DB-4 adds a general capability_guard() to
common/errors/ and covers the chassis open-set fault codes (99 U68). This package
is the SPECIFIC three-block fail-safe and depends only on INF-CM-4 (the E_BUSY /
E_CAPABILITY rows), not on INF-DB-4 -- so it constructs its own results rather
than calling a guard that does not exist yet.
"""
