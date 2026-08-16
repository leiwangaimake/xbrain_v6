"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: data_readers.py
Brief: Project P5 in-process runtime state into the 17 S6.8 HMI snapshot

Description:
The problem this solves. The HMI page needs one JSON snapshot per refresh
covering 17 S6.8's groups: geo (fences/routes/waypoints), pose, quality/mode,
plan/progress, events. This module is the read side -- it turns whatever P5
already holds in-process into that snapshot, and returns null/empty for the
parts whose source is not yet built.

Which section this follows: 17 S6.8 (the frozen A..F data set) and 17 S6.10.4
(data source + today's availability under the hardware gate).

The one architectural rule that shapes every reader here: P5 does NOT read
P3's task.db/geo.db/fence.db directly. P3 is their single writer (15 S9), and
the planes are isolated. P5 receives this data by SUBSCRIPTION and caches it:
fence geometry from cmd/fence (17 S6.9, P5 is the geometry consumer), task
state from state/task, pose from state/pose, and events from P5's OWN record.db
(P5 is that DB's writer). So every reader below takes a P5 runtime object (a
cache / last-seen state / events source), never a P3 DB handle.

What it does NOT do, and why. It does NOT fabricate pose. 17 S6.10.4: the pose/
GPS/ENU/heading/speed/RTK/precision fields all depend on perception (design not
written, GATED-DESIGN) + rtk_driver (not built, GATED-HW) + quadruped (awaiting
chassis). Until those exist, pose_group() returns a null pose with
`available: false`, and the frontend renders a "no fix" state. Drawing a robot
arrow at (0,0) or an RTK "Float" badge from a constant would be exactly the
fail-silent 3.1/3.2 forbids -- a reviewer (and the operator) could not tell the
map position was fake.

Traps already hit / to avoid:
  * progress_percent MUST be None when route_total_m is unknown, never 0 or 100
    (outbound/projection.py already encodes this; do not re-derive a fake %).
  * an absent fence cache is NOT an empty fence set: empty means "subscribed,
    nothing staged", absent/degraded means "we do not know" -> the /api/fences
    contract returns 503 E_DEGRADED, not 200 [] (17 S6.9 P5F-2). This module
    reports availability so the route layer can pick the right status code.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence


def geo_group(
    fences: Optional[Sequence[Dict[str, Any]]],
    routes: Optional[Sequence[Dict[str, Any]]],
    waypoints: Optional[Sequence[Dict[str, Any]]],
    enu_origin: Optional[Dict[str, float]],
) -> Dict[str, Any]:
    """17 S6.8 group A: map geometry (fences, recorded routes, waypoints).

    Each argument is what P5 has cached from its subscriptions, or None when the
    source has not reported yet. None is preserved as `available: false` so the
    map can grey the layer instead of drawing an empty world as if it were real.
    Fences keep name + polygon vertices + role (U5 naming; role in {keep_in,
    zone}); routes keep name + points (U5 naming); waypoints keep name + geom +
    recorded flag (U6 marker state). enu_origin anchors the ENU metre grid.
    """
    return {
        # available=False means "not known" (grey the layer); an empty list with
        # available=True means "known to be empty" -- the two must not collapse.
        "fences": _layer(fences),
        "routes": _layer(routes),
        "waypoints": _layer(waypoints),
        "enu_origin": enu_origin,     # None until localisation origin is set
    }


def _layer(items: Optional[Sequence[Dict[str, Any]]]) -> Dict[str, Any]:
    """Wrap a cached geometry list into {available, items}.

    available is False exactly when the cache is absent (None). A present-but-
    empty list is available=True with items=[]: "subscribed, nothing staged".
    """
    if items is None:
        return {"available": False, "items": []}
    return {"available": True, "items": list(items)}


def pose_group(pose: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """17 S6.8 group B/C: pose + localisation quality.

    `pose` is the last state/pose payload, or None. Until perception + rtk_driver
    + chassis exist (17 S6.10.4, GATED), pose is None and this returns an
    explicit no-fix shell -- NOT a zeroed pose. The frontend must read
    `available` and render "no fix" (hide the robot arrow, blank the GPS/ENU/RTK
    readouts) rather than plot (0,0). Fields mirror 17 S6.8: lat/lon/alt,
    heading_rad + heading_valid, speed_mps, fix_type, cov_h_m, yaw_capable.
    """
    if not pose:
        # No source -> no fix. Every value is None so the UI cannot mistake a
        # default for a reading (3.1/3.2 fail-silent guard).
        return {
            "available": False,
            "lat": None, "lon": None, "alt": None,
            "heading_rad": None, "heading_valid": False,
            "heading_source": None, "heading_level": None,
            "speed_mps": None, "fix_type": None, "cov_h_m": None,
            "num_satellites": None,
            "yaw_capable": False,
        }
    # Pass through only the fields 17 S6.8 defines; a stray key from the source
    # is dropped so the UI contract cannot quietly grow.
    return {
        "available": True,
        "lat": pose.get("lat"), "lon": pose.get("lon"), "alt": pose.get("alt"),
        "heading_rad": pose.get("heading_rad"),
        "heading_valid": bool(pose.get("heading_valid", False)),
        # RTK heading status (18-C G45/G46): source + degradation level, so the UI
        # can show 双天线(L1)/航迹(L2)/无(L3) instead of only the raw angle.
        "heading_source": pose.get("heading_source"),
        "heading_level": pose.get("heading_level"),
        "speed_mps": pose.get("speed_mps"),
        "fix_type": pose.get("fix_type"),
        "cov_h_m": pose.get("cov_h_m"),
        "num_satellites": pose.get("num_satellites"),   # 18-C G44 / RTK panel
        "yaw_capable": bool(pose.get("yaw_capable", False)),
    }


def clock_group(clock: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """state/clock -> HMI clock group (18-C G47). sync judgement belongs to
    rtk_driver (CLK-A1); the HMI only DISPLAYS it. No source (not wired / stale)
    -> sync False, source none -- fail-safe, never a fabricated 'synced'."""
    if not clock:
        return {"available": False, "sync": False, "source": "none"}
    return {
        "available": True,
        "sync": bool(clock.get("sync", False)),
        "source": clock.get("source", "none"),
    }


def plan_group(tasks: Optional[Sequence[Dict[str, Any]]]) -> Dict[str, Any]:
    """17 S6.8 group D: the current-plan panel.

    `tasks` is P5's cache of state/task (or None). Each task keeps the fields the
    panel shows: task_id, plan name, state badge, dispatch time, ordered target
    list with per-point done flags, progress (done/total), result summary.
    Progress and per-point status depend on P1 real path execution + route
    expansion (17 S6.10.4, EX-1 not done): where the task carries no total_steps,
    progress stays {done, total: None} and the panel shows the badge without a
    fraction -- never a fabricated 2/3.
    """
    if tasks is None:
        return {"available": False, "plans": []}
    return {"available": True, "plans": [_plan(t) for t in tasks]}


def _plan(task: Dict[str, Any]) -> Dict[str, Any]:
    """One current-plan card from a cached task record.

    total may be None (route not expanded yet); done is 0 when unknown. The
    frontend renders "done / total" only when total is not None, else just the
    state badge -- the 17 S6.10.4 rule against a fabricated fraction.
    """
    total = task.get("total_steps")
    done = task.get("current_step") or 0
    return {
        "task_id": task.get("task_id"),
        "name": task.get("name"),              # None until plan-name projected
        "state": task.get("state"),            # closed set, 11 S4.4
        "dispatch_ts": task.get("created_at"), # approx dispatch (17 S6.10.2 note)
        "targets": task.get("targets") or [],  # ordered, [] until route expanded
        "progress": {"done": done, "total": total},
        "result": task.get("result"),          # None until alarm-count wired
    }


def status_group(
    mode: Optional[str],
    link: Optional[Dict[str, Any]],
    health: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """17 S6.8 group C/E: footer status + link.

    mode (P2 usage_mode, available), link (state/link -- carries estop_path, the
    field the ESTOP button greys itself by, 17 S6.3/NAV-64), health (P2 summary;
    the rtk item has no real input until rtk_driver, so its value may be null
    while its structure is present). None inputs are surfaced as null, not
    defaulted.
    """
    estop_path = (link or {}).get("estop_path")
    return {
        "mode": mode,                          # from P2, available
        # estop_path drives the ESTOP button enable/disable in the frontend:
        # anything other than "ok" -> button greyed + "estop unavailable" (NAV-64).
        "estop_path": estop_path,
        "link_latency_ms": (link or {}).get("latency_ms"),
        "health": health,                      # structure present, rtk value gated
    }


def events_group(events: Optional[Sequence[Dict[str, Any]]]) -> Dict[str, Any]:
    """17 S6.8 group F: recent events for the map dots + event stream.

    `events` come from P5's own record.db (P5 is the writer, so this IS a real
    source today). Each event keeps eid/title/sev/cat/ts and pos (lat/lon) --
    but pos is only real once pose exists to stamp it (17 S6.10.4); a null pos
    means "event known, location not". The map plots a dot only when pos is
    non-null, so a located-nowhere event shows in the stream but not on the map.
    """
    if events is None:
        return {"available": False, "items": []}
    return {"available": True, "items": [_event(e) for e in events]}


def _event(ev: Dict[str, Any]) -> Dict[str, Any]:
    """One event row projected for the HMI (map dot + stream line)."""
    pos = ev.get("pos")
    return {
        "eid": ev.get("eid"),
        "title": ev.get("title"),
        "sev": ev.get("sev"),                  # severity -> colour (S6.6)
        "cat": ev.get("cat"),
        "ts": ev.get("ts"),
        # pos None -> stream only, no map dot (do not plot at 0,0).
        "pos": pos if (pos and pos.get("lat") is not None) else None,
    }


def rest_list_endpoint(value: Optional[Sequence[Any]],
                        key: str) -> Dict[str, Any]:
    """W8: body for the 17 S6.5 read-only LIST endpoints (/api/routes,
    /api/docks, /api/approval/pending).

    available reflects whether the source is actually wired: None -> false + [],
    NEVER a 200 empty set presented as authoritative (the same P5F-2 lesson the
    fence endpoint enforces -- an empty list a client cannot tell from "no data"
    is the fail-silent 3.2 forbids). routes/docks are gated on geo.db (P5 does
    not read P3 dbs, 15 four-DB model), approval on the L3 queue (no feed yet).
    """
    return {"available": value is not None,
            key: list(value) if value else []}


def rest_object_endpoint(value: Optional[Any], key: str) -> Dict[str, Any]:
    """W8: body for the 17 S6.5 read-only OBJECT endpoints (/api/health,
    /api/bit, /api/metrics).

    Passthrough of the authoritative upstream payload: P5 RELAYS P2's health/*
    (health/factor, health/bit) unchanged -- it does not recompute a second
    truth (G-2 same-source; 17 S3.8 warns a re-reported metric is a second
    source). available:false + null until the topic is subscribed; /api/metrics
    stays gated until a telemetry aggregator is wired (NEXT.md).
    """
    return {"available": value is not None, key: value}


def build_snapshot(
    *,
    fences: Optional[Sequence[Dict[str, Any]]] = None,
    routes: Optional[Sequence[Dict[str, Any]]] = None,
    waypoints: Optional[Sequence[Dict[str, Any]]] = None,
    enu_origin: Optional[Dict[str, float]] = None,
    pose: Optional[Dict[str, Any]] = None,
    tasks: Optional[Sequence[Dict[str, Any]]] = None,
    mode: Optional[str] = None,
    link: Optional[Dict[str, Any]] = None,
    health: Optional[Dict[str, Any]] = None,
    events: Optional[Sequence[Dict[str, Any]]] = None,
    clock: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Assemble the full 17 S6.8 HMI snapshot from P5 runtime state.

    Every argument defaults to None so a partially-wired backend (today's
    reality under the hardware gate) still produces a valid snapshot -- the
    absent groups report `available: false` and the frontend greys them. This is
    the single method the web server calls per push/GET; keeping the assembly
    here (not in the route handler) means the same projection is unit-tested
    without a running server.
    """
    return {
        "geo": geo_group(fences, routes, waypoints, enu_origin),
        "pose": pose_group(pose),
        "plan": plan_group(tasks),
        "status": status_group(mode, link, health),
        "events": events_group(events),
        "clock": clock_group(clock),
    }


_MISSING = object()   # sentinel: a group present in curr but absent in prev


def snapshot_delta(prev: Dict[str, Any],
                   curr: Dict[str, Any]) -> Dict[str, Any]:
    """W6: the changed top-level groups between two snapshots (17 S6.2 state_delta).

    Returns {group_key: new_value} for each of the build_snapshot groups (geo /
    pose / plan / status / events) whose value differs from prev, and {} when
    nothing changed. The WS sends {} as a keepalive, never a full resend -- an
    all-quiet tick is the common case and its delta is empty (that IS the
    bandwidth win over pushing the whole snapshot every tick).

    Group-level, NOT deep field-level: the frontend merges a changed group
    wholesale (currentSnap[k] = delta[k]), so a single changed status replaces
    the whole status group. Cheap to compute, trivially correct to merge. The
    _MISSING sentinel makes a group that is present in curr but absent in prev
    count as changed (a value of None is a real value, not 'absent').
    """
    return {k: v for k, v in curr.items() if v != prev.get(k, _MISSING)}
