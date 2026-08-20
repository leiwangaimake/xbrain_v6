"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_snapshot_delta.py
Brief: HMI-W6 state_delta group diff tests (17 S6.2)

Description:
Guards snapshot_delta: the WS pushes only the changed top-level groups each tick
(17 S6.2 state_delta) instead of the whole snapshot, and a quiet tick yields {}
(the keepalive / bandwidth win). Each claim is paired with its red mutant (3.3).

The load-bearing case is test_quiet_tick_is_empty_delta: a mutant that inverts
the != (or returns the whole snapshot regardless) makes an all-quiet tick resend
everything -- exactly the full-per-tick behaviour W6 removes -> red. The round-
trip case proves the frontend's group-level merge (currentSnap[k] = delta[k])
reconstructs the new snapshot, so no field is silently dropped.
"""

from __future__ import annotations

from xbrain.p5_gateway.hmi.data_readers import build_snapshot, snapshot_delta


def _snap(mode=None, tasks=None, events=None):
    # Build real snapshots so the diff runs over the actual group shapes.
    return build_snapshot(mode=mode, tasks=tasks, events=events)


def test_quiet_tick_is_empty_delta():
    # Nothing changed between two identical snapshots -> {} (keepalive only).
    # RED MUTANT: `==` instead of `!=` (or return curr wholesale) -> every group
    # comes back and the WS resends the full snapshot each tick.
    a = _snap(mode="patrol")
    b = _snap(mode="patrol")
    assert snapshot_delta(a, b) == {}


def test_only_the_changed_group_is_sent():
    # mode flips -> only the `status` group changes; geo/pose/plan/events do not.
    # RED MUTANT: return all keys -> geo/plan/events ride along needlessly.
    a = _snap(mode="patrol")
    b = _snap(mode="idle")
    delta = snapshot_delta(a, b)
    assert set(delta.keys()) == {"status"}
    assert delta["status"]["mode"] == "idle"


def test_group_absent_in_prev_counts_as_changed():
    # A group present in curr but absent in prev must be included (the _MISSING
    # sentinel), so a first diff against {} yields every key -- the S6.8 groups
    # plus the top-level `timezone` (17 S6.10.2 v1.3).
    # RED MUTANT: drop the sentinel and default to None -> a group whose value
    # were None would be dropped; here the empty-prev case must send them all.
    #
    # Compared against build_snapshot's own keys rather than a transcribed list:
    # a list here has to be edited every time a group is added (teach, 2026-08-20
    # S12A.5), and the edit is indistinguishable from accepting a group that was
    # dropped by accident.
    b = _snap(mode="patrol")
    delta = snapshot_delta({}, b)
    assert set(delta.keys()) == set(b.keys())
    assert {"geo", "pose", "plan", "status", "events", "clock",
            "timezone"} <= set(delta.keys())


def test_delta_merge_round_trips_to_the_new_snapshot():
    # Applying the delta onto prev (the frontend's group-level merge) must equal
    # curr -- no field silently lost. RED MUTANT: a shallow/partial group value
    # in the delta would make the merged result differ from curr.
    a = _snap(mode="patrol", tasks=[{"task_id": "t1", "state": "running"}])
    b = _snap(mode="idle", tasks=[{"task_id": "t1", "state": "completed"}],
              events=[{"eid": "e1", "title": "x", "sev": "info"}])
    merged = {**a, **snapshot_delta(a, b)}
    assert merged == b
