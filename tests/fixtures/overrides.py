"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: overrides.py
Brief: CHK-0-56 null-key overrides for the fixture config set (L1/L2 only)

Description:
Real /opt/xbrain_v6/configs/ deliberately leaves 60+ leaf values as
null (CLAUDE.md 3.1 "保持这个形态, 不要为了让它跑起来而填数"). The
fixture cannot fill them in the real tree without violating that
rule, so it materialises a COPY of the tree at test-run and applies
this override map to the copy.

Two invariants this file MUST hold, enforced by
tests/fixtures_meta/test_fixture_integrity.py:

  * every dotted key here targets a leaf that is null in the current
    real /opt/xbrain_v6/configs/ (superset that ignores non-null keys
    would silently mask a real value; sub-set would leak nulls into
    the fixture)
  * NO key here begins with 'common.safety.' -- the safety layer
    (10 S5.4.6 ENV-2) is symlinked from real configs at fixture-build
    time; overriding a safety value would break the same-source
    guarantee (CHK-0-56 criterion iv)

Values are the minimum needed to satisfy freeze assertions A + G
(other assertions land as stubs in the current tree per CFG-FZ-1).
Numeric values are consistent with the 12/13/14 spec ranges so
assertion G's SP-1/SP-2/SP-5/SP-11/AS-7 rows pass.
"""

from __future__ import annotations

from typing import Any, Dict


NULL_OVERRIDES: Dict[str, Any] = {
    # ---- common.audio -----------------------------------------------
    "common.audio.bypass_keywords": ["停止", "急停", "紧急停止"],

    # ---- common.calib -----------------------------------------------
    # calib.* is authoritative at L4b (calib/{robot_id}.yaml written
    # by fixture conftest). At L1 we install empty-dict placeholders
    # for frames + gate so assertion A does not report a null leaf
    # (empty dict is 'declared but content deferred to L4b').
    "common.calib.calib_rev": "unspecified",
    "common.calib.frames": {},
    "common.calib.gate": {},
    "common.calib.lat_err_ref_m": 0.05,

    # ---- common.cmdset ---------------------------------------------
    "common.cmdset.intents_file": "/opt/xbrain_v6/configs/intents.yaml",
    "common.cmdset.missions_dir": "/opt/xbrain_v6/configs/missions",
    "common.cmdset.query_templates": "/opt/xbrain_v6/configs/query_templates.yaml",
    "common.cmdset.version": "v1.0-fixture",

    # ---- common.db --------------------------------------------------
    "common.db.fence_db": "/opt/xbrain_v6/data/fence.db",
    "common.db.geo_db": "/opt/xbrain_v6/data/geo.db",
    "common.db.record_db": "/opt/xbrain_v6/data/record.db",
    "common.db.task_db": "/opt/xbrain_v6/data/task.db",
    "common.db.pragma.busy_timeout_ms": 5000,
    "common.db.pragma.journal_mode": "WAL",
    "common.db.pragma.synchronous": "FULL",

    # ---- common.fence ----------------------------------------------
    "common.fence.margin_by_fix.dgps": 1.5,
    "common.fence.margin_by_fix.rtk_fixed": 0.5,
    "common.fence.margin_by_fix.rtk_float": 1.0,
    "common.fence.margin_by_fix.single": 3.0,
    "common.fence.predict_dt_s": 0.4,
    "common.fence.soft_margin_min_m": 0.5,

    # ---- common.geo ------------------------------------------------
    # enu_origin: MUST live at L4 (sites layer only, per FV-ORG-3).
    # Fixture conftest.py writes sites/lab.yaml with the three
    # sub-keys; the L1 shape we install is a 3-null dict so that
    # assertion A's _A_NULL_EXCEPTIONS (which names the .lat/.lon/.alt
    # LEAVES not the parent) actually skips them.
    "common.geo.enu_origin": {"lat": None, "lon": None, "alt": None},
    #
    # ---- common.motion.profiles ------------------------------------
    "common.motion.profiles.obstacle_avoid.max_mps": 0.5,
    "common.motion.profiles.obstacle_avoid.require_sense_m": 0.3,
    "common.motion.profiles.obstacle_avoid.sensors": ["lidar", "rgbd"],
    "common.motion.profiles.obstacle_avoid.up_enter_mps": 0.4,
    "common.motion.profiles.patrol.max_mps": 1.5,
    "common.motion.profiles.patrol.require_sense_m": 1.0,
    "common.motion.profiles.patrol.sensors": ["lidar", "rgbd", "odom"],
    "common.motion.profiles.patrol.up_enter_mps": 1.2,

    # ---- common.priority -------------------------------------------
    "common.priority.task.auto": 30,
    "common.priority.task.charge": 80,
    "common.priority.task.cloud": 60,
    "common.priority.task.local": 40,
    "common.priority.task.wecom": 50,

    # ---- common.qos (shape mirrors tests/boot/freeze/test_f_qos_and_port.py
    #      _build_green_qos which is the canonical passable example) ----
    "common.qos.profiles": {
        "Q0_safety": {"congestion_control": "drop", "priority": "real_time",
                       "reliability": "reliable", "express": True,
                       "handler": {"kind": "ring", "depth": 8}},
        "Q1_rt":     {"congestion_control": "drop", "priority": "real_time",
                       "reliability": "best_effort", "express": True,
                       "handler": {"kind": "ring", "depth": 1}},
        "Q2_state":  {"congestion_control": "drop", "priority": "data_high",
                       "reliability": "reliable", "express": False,
                       "handler": {"kind": "ring", "depth": 4}},
        "Q3_cmd":    {"congestion_control": "block", "priority": "data",
                       "reliability": "reliable", "express": False,
                       "handler": {"kind": "fifo", "depth": 256}},
        "Q4_stream": {"congestion_control": "drop",
                       "priority": "interactive_high",
                       "reliability": "best_effort", "express": False,
                       "handler": {"kind": "ring", "depth": 10}},
    },
    "common.qos.bindings": [
        {"match": "xbrain/*/cmd/estop", "profile": "Q0_safety"},
        {"match": "xbrain/*/probe/estop/**", "profile": "Q0_safety"},
        {"match": "xbrain/*/rt/**", "profile": "Q1_rt"},
        {"match": "xbrain/*/**", "profile": "Q3_cmd"},   # fallback
    ],

    # ---- common.recording ------------------------------------------
    "common.recording.fence_close_tol_m": 1.0,
    "common.recording.max_fences": 5,
    "common.recording.min_dist_m": 0.5,
    "common.recording.sample_hz": 1.0,
    "common.recording.session_timeout_s": 300.0,

    # ---- common.retention ------------------------------------------
    "common.retention.command_log_days": 180,
    "common.retention.event_days": 90,
    "common.retention.task_days": 30,

    # ---- common.robot_id / common.site_id --------------------------
    "common.robot_id": "lab_robot",
    "common.site_id": "lab",

    # ---- common.spec (SP-1/SP-2/SP-11 gates in assertion G) --------
    "common.spec.max_vx_mps": 2.0,       # SP-2 upper bound for profile.max_mps
    "common.spec.max_vy_mps": 0.5,
    "common.spec.max_wz_radps": 1.2,
    "common.spec.max_accel_mps2": 1.5,
    "common.spec.max_decel_mps2": 2.5,   # SP-5 right-side: >= safety.brake.a_mps2
    "common.spec.odom.yaw_drift_dps": 0.1,
    "common.spec.terrain.max_gap_m": 0.2,
    "common.spec.terrain.max_slope_deg": 30.0,
    "common.spec.terrain.max_step_height_m": 0.15,

    # ---- common.zenoh (CFG-BT-* deploy) ---------------------------
    "common.zenoh.gen_connect": "tcp/127.0.0.1:7447",
    "common.zenoh.gen_listen": ["tcp/127.0.0.1:7447"],
    "common.zenoh.rt_endpoint": "tcp/127.0.0.1:7449",
}


SAFETY_KEY_PREFIX = "common.safety."


def assert_no_safety_overrides() -> None:
    """CHK-0-56 (iv): NULL_OVERRIDES may not touch common.safety.*.
    Called from the meta test; raises with the offending key on any
    violation."""
    for k in NULL_OVERRIDES:
        if k.startswith(SAFETY_KEY_PREFIX):
            raise AssertionError(
                "CHK-0-56 (iv) violation: NULL_OVERRIDES touches "
                "safety key %r; safety layer must remain same-source "
                "with real configs/safety/ (10 S5.4.6 ENV-2)" % k)


def set_by_path(tree: Dict[str, Any], dotted: str, value: Any) -> None:
    """Set a leaf value in a nested dict by dotted path. Creates
    intermediate dicts as needed."""
    parts = dotted.split(".")
    cur = tree
    for p in parts[:-1]:
        nxt = cur.get(p)
        if not isinstance(nxt, dict):
            nxt = {}
            cur[p] = nxt
        cur = nxt
    cur[parts[-1]] = value


def apply_overrides(tree: Dict[str, Any],
                     extras: Dict[str, Any] = None) -> None:
    """Apply NULL_OVERRIDES to `tree` in place. `extras` is an
    optional per-test override map (used by mutation tests to inject
    bogus values)."""
    for k, v in NULL_OVERRIDES.items():
        set_by_path(tree, k, v)
    if extras:
        for k, v in extras.items():
            set_by_path(tree, k, v)


# ---- Per-proc L6 null fills -----------------------------------------
# 2026-08-10 (V-P4-NULLS): p4_agent.yaml declares 12 undecided keys as
# null per CLAUDE.md 3.1 -- freeze assertion A rejects them, and so
# does load_p4_config at process start. For dev voice-loop testing the
# fixture needs to substitute a non-null placeholder for each so
# freeze can complete AND the P4 process boots. Values here are the
# same 'fixture-not-a-*' + 0.987654 markers used in
# tests/p4_agent/test_config_loader.py::FIXTURE_FILL; a real deploy
# still needs Q-P4-2 / Q-P4-3 / D-AI-1..3 decisions before it can
# ship these values. The keys live in P4's own namespace, so they
# route to p4_agent.yaml (NOT common.yaml) per 10 S5.4.3 (L6 owns
# its own top-level namespace).
P4_L6_FILL: Dict[str, Any] = {
    "grammar.max_enum_items": 7,
    "gateway.llm.endpoint": "http://not-a-real-host.invalid/v1/chat/completions",
    "gateway.asr.model": "fixture-not-a-model",
    "gateway.tts.base_url": "http://not-a-real-host.invalid",
    "gateway.tts.path": "/fixture",
    "gateway.tts.model": "fixture-not-a-model",
    "gateway.tts.voice": "fixture-not-a-voice",
    "gateway.tts.speed": 0.987654,
    "gateway.tts.response_format": "fixture",
    "gateway.tts.stream": False,
    "gateway.tts.timeout_s": 5.0,
    "gateway.gpu_token.throttle_speed_mps": 0.987654,
}
