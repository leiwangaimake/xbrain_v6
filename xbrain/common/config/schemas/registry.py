"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: registry.py
Brief: CFG-10 schema assets, one per config file, plus the file-keyed lookup

Description:
CFG-FZ-17 requires a schema for each of the 19 config files under the config
root. This module holds those 19 assets and validate_config(rel_path, tree),
which the freeze oneshot calls once per parsed file BEFORE it builds the overlay
and BEFORE assertion A. The engine (type / required / range) lives in spec.py;
this file is only the data and the lookup.

Where the schemas come from, and the line this module will not cross:

  * Types and required-ness for a file that ALREADY carries keys on disk
    (common.yaml, models/m20s.yaml, safety/brake.yaml, p4_agent.yaml) are read
    off that skeleton and cross-checked against the key table in 10 S5.4.5 and
    the per-volume config sections. This is derivation, not invention.
  * safety/clock.yaml is empty on disk, but 11 S1.5.5 gives a compact, settled
    yaml block for it (a safety file with seven keys). Those keys are declared
    here with required=False, so the empty file passes today and a wrong TYPE on
    a future fill is still caught. This is the one empty file that earns a typed
    schema, because its whole key set sits in one readable block.
  * The other 14 files are empty skeletons whose key sets are deferred to a whole
    design volume and, in several cases, still pending a ruling (p1_motion's
    teleop.cloud, for one). Their schema is intentionally EMPTY: a registry entry
    with an authority citation and nothing to enforce yet. Transcribing a full
    schema ahead of the config would invent structure, and the first correct fill
    that spelled a key differently than my guess would be rejected -- a false
    gate, the exact "pretend a guarantee exists" failure CLAUDE.md 3.2 forbids.
    An empty schema accepts the eventual keys and is the designated place to add
    types when the file is filled.

What this module does NOT encode, on purpose:

  * No safety RANGES. SP-1..SP-11 and S-1..S-6 are assertion G / CFG-11 (10
    S5.4.6 lists CFG-10 and CFG-11 as two layers, "不得合并成一层"; the authority
    table is 12 S12.1). t_lat_s >= 0.4, a_mps2 <= spec.max_decel_mps2 and the
    rest are checked there, not here. The FieldSpec range mechanism is for pure
    per-key domain bounds; none of the 19 files has one that is doc-stated and
    not already a safety assertion (closed_set_threshold, the only candidate, is
    an uncalibrated placeholder per 16 Q-P4-9), so the production schemas carry
    no ranges. The mechanism is exercised by its own test.
  * No namespace rules (check_namespace, layers.py) and no null/missing checks
    (assertions A and M). This layer is types and structure, nothing else.
"""

from typing import Any, Dict, Tuple

from .spec import (Schema, SchemaError, anything, boolean, integer, listof,
                   mapping, num, text, validate_tree)

# ---------------------------------------------------------------------------
# 1. common.yaml  (L1)  -- authority: 10 S5.4.5 "共享参数唯一定义处对照表"
#
# The key POSITIONS here are the whole left column of the 10 S5.4.5 table (10
# S5.4.4 assertion M: "清单 = S5.4.5 对照表的全部左列"), which is why they are
# required=True: this file is the L1 declaration of every common.* position, so
# each must be present even while its value is still null. Types follow the
# unit/semantics of each leaf: *_mps / *_m / *_s / *_hz / *_days -> number or
# integer; *_db / *_dir / *_file / endpoint / id -> string; profiles / pragma /
# frames / gate / qos.profiles / enu_origin -> mapping; gen_listen / bindings /
# sensors -> list; allow_* -> boolean. Safety leaves (spec.*, safety.*) are typed
# but NOT range-checked -- that is assertion G.
# ---------------------------------------------------------------------------
_COMMON: Dict[str, Any] = {
    # identity
    "common.robot_id": text(),                       # 11 S2.1, fills Zenoh {rid}
    "common.site_id": text(),                        # sites/{site_id}.yaml name
    # zenoh endpoints (11 S1.1.2 / S1.1.4 / NET-C9)
    "common.zenoh.rt_endpoint": text(),
    "common.zenoh.gen_listen": listof(),             # per-port list, not a scalar
    "common.zenoh.gen_connect": text(),
    # qos (11 S2.4.7); a locked safety namespace
    "common.qos.profiles": mapping(),                # profile definitions object
    "common.qos.bindings": listof(),                 # ordered, first-match table
    # spec (11 S9.6). Typed number; range is SP-1, which is assertion G.
    "common.spec.max_vx_mps": num(),                 # m/s hard limit
    "common.spec.max_vy_mps": num(),                 # m/s strafe limit
    "common.spec.max_wz_radps": num(),               # rad/s yaw limit
    "common.spec.max_accel_mps2": num(),             # m/s^2 accel limit
    "common.spec.max_decel_mps2": num(),             # m/s^2 decel limit
    # motion.profiles (11 S9.6.1); the two-gear table lives HERE (L1)
    "common.motion.profiles.obstacle_avoid.max_mps": num(),          # gear cap
    "common.motion.profiles.obstacle_avoid.up_enter_mps": num(),     # up-shift gate
    "common.motion.profiles.obstacle_avoid.require_sense_m": num(),  # brake-derived
    "common.motion.profiles.obstacle_avoid.sensors": listof(),       # admit list
    "common.motion.profiles.patrol.max_mps": num(),                  # gear cap
    "common.motion.profiles.patrol.up_enter_mps": num(),             # up-shift gate
    "common.motion.profiles.patrol.require_sense_m": num(),          # brake-derived
    "common.motion.profiles.patrol.sensors": listof(),               # admit list
    # safety constants (positions declared here; values in safety/*.yaml)
    "common.safety.t_lat_s": num(),                  # s latency; SP-5 range is G
    "common.safety.brake.a_mps2": num(),             # m/s^2 brake decel
    "common.safety.brake.k": num(),                  # brake safety factor
    "common.safety.clock.unsynced_max_speed_mps": num(),    # m/s unsynced cap
    "common.safety.clock.allow_unsynced_motion": boolean(),  # lab bypass flag
    # fence (11 S9A.7 / S9A.12)
    "common.fence.margin_by_fix.rtk_fixed": num(),   # m inset, best fix
    "common.fence.margin_by_fix.rtk_float": num(),   # m inset, float fix
    "common.fence.margin_by_fix.dgps": num(),        # m inset, dgps fix
    "common.fence.margin_by_fix.single": num(),      # m inset, single fix
    "common.fence.soft_margin_min_m": num(),         # m soft-band floor
    "common.fence.predict_dt_s": num(),              # s prediction step
    # geo origin (11 S7.8.4); filled form is {lat,lon,alt}, i.e. a mapping
    "common.geo.enu_origin": mapping(),              # coordinate object
    # calib (11 S10.1.1 / S10.4); frames/gate are objects, filled per-robot at L4b
    "common.calib.frames": mapping(),                # extrinsic frames object
    "common.calib.gate": mapping(),                  # calib gate thresholds
    "common.calib.lat_err_ref_m": num(),             # m lateral error ref
    # calib_rev appears only beside config_rev in 11 (a revision marker); whether
    # it is a counter or a label is not fixed, so ANY rather than a guessed token.
    "common.calib.calib_rev": anything(),
    # databases (15 S9.0 U46): four db paths + shared pragma
    "common.db.task_db": text(),                     # task.db path
    "common.db.record_db": text(),                   # record.db path
    "common.db.fence_db": text(),                    # fence.db path
    "common.db.geo_db": text(),                      # geo.db path
    "common.db.pragma.journal_mode": text(),         # e.g. "WAL"
    # synchronous is SQLite's enum, spelled either "FULL" or 0..3 depending on the
    # writer; 15 S9.1 calls the pragma set open-ended, so ANY not a guessed token.
    "common.db.pragma.synchronous": anything(),
    "common.db.pragma.busy_timeout_ms": integer(),   # ms busy timeout
    # retention (15 S9.11): day counts
    "common.retention.task_days": integer(),         # days; assertion C ordering
    "common.retention.event_days": integer(),        # days
    "common.retention.command_log_days": integer(),  # days
    # recording (99 U42/U44)
    "common.recording.min_dist_m": num(),            # m dedup distance
    "common.recording.session_timeout_s": num(),     # s session cap
    "common.recording.sample_hz": num(),             # Hz sample rate
    "common.recording.max_fences": integer(),        # count cap
    "common.recording.fence_close_tol_m": num(),     # m close tolerance
    # command set (16 S12.4 / 18 S16): paths + version label
    "common.cmdset.version": text(),                 # cmdset semver label
    "common.cmdset.intents_file": text(),            # intents.yaml path
    "common.cmdset.missions_dir": text(),            # missions dir path
    "common.cmdset.query_templates": text(),         # query templates path
    # bypass words (18 A01): three groups; the filled shape (mapping vs flat list)
    # is not fixed in 11/18, so ANY -- present is checked, shape is deferred.
    "common.audio.bypass_keywords": anything(),
    # task priorities (15 S4.2): integer ranks, five sources
    "common.priority.task.cloud": integer(),         # rank, cloud source
    "common.priority.task.wecom": integer(),         # rank, wecom source
    "common.priority.task.local": integer(),         # rank, local source
    "common.priority.task.auto": integer(),          # rank, auto source
    "common.priority.task.charge": integer(),        # rank, charge source
}

# ---------------------------------------------------------------------------
# 2. models/m20s.yaml  (L2)  -- authority: 11 S9.6 / S9.6.1.1
#
# This file DOES carry values on disk, so required=True and types are read off
# the skeleton: holonomic is a capability bool, the five spec limits are numbers
# (null today), terrain/odom are numbers, trust_by_gait and gait_limits are the
# populated tables. gait_limits' key SET (== the gait closed set) is SP-9, an
# assertion-G check, so it is NOT enforced here -- only the leaf types are.
# ---------------------------------------------------------------------------
_M20S: Dict[str, Any] = {
    "common.spec.holonomic": boolean(),                    # strafe capability flag
    "common.spec.max_vx_mps": num(),                       # m/s; SP-1 range is G
    "common.spec.max_vy_mps": num(),                       # m/s strafe; null today
    "common.spec.max_wz_radps": num(),                     # rad/s yaw; null today
    "common.spec.max_accel_mps2": num(),                   # m/s^2; null today
    "common.spec.max_decel_mps2": num(),                   # m/s^2; SP-5 rhs
    "common.spec.terrain.max_slope_deg": num(),            # deg; null today
    "common.spec.terrain.max_step_height_m": num(),        # m; null today
    "common.spec.terrain.max_gap_m": num(),                # m; null today
    "common.spec.odom.yaw_drift_dps": num(),               # deg/s; null (M-09)
    "common.spec.odom.trust_by_gait.flat": num(),          # trust weight, filled
    "common.spec.odom.trust_by_gait.stair": num(),         # trust weight, filled
    "common.spec.gait_limits.basic.max_vx_mps": num(),     # per-gait cap (S9.6.1.1)
    "common.spec.gait_limits.platform.max_vx_mps": num(),  # per-gait cap
    "common.spec.gait_limits.flat.max_vx_mps": num(),      # per-gait cap
    "common.spec.gait_limits.stair_agile.max_vx_mps": num(),     # per-gait cap
    "common.spec.gait_limits.stair_standard.max_vx_mps": num(),  # per-gait cap
}

# ---------------------------------------------------------------------------
# 3. safety/brake.yaml  (L3)  -- authority: 10 S5.4.5 / 11 S9.6.2
#
# Three keys, all present with real values on disk. This is the file the
# CFG-FZ-17 mutation (1) targets: t_lat_s written as the string "0.4" must go
# red HERE, on the number type, before assertion G ever tries "0.4" >= 0.4. The
# >= 0.4 bound itself is SP-5 (assertion G), deliberately absent here.
# ---------------------------------------------------------------------------
_BRAKE: Dict[str, Any] = {
    "common.safety.t_lat_s": num(),
    "common.safety.brake.a_mps2": num(),
    "common.safety.brake.k": num(),
}

# ---------------------------------------------------------------------------
# 4. safety/clock.yaml  (L3)  -- authority: 11 S1.5.5 config block
#
# Empty on disk (values pending calibration, one still pending an RTC-battery
# confirmation). required=False so the empty file passes today; the types are the
# one place a wrong fill (sync_timeout_s: "5") is caught. Keys and types are read
# straight off the 11 S1.5.5 yaml block, not guessed. The unsynced-speed CAP and
# the meaning of allow_unsynced_motion are safety semantics, not schema range.
# ---------------------------------------------------------------------------
_CLOCK: Dict[str, Any] = {
    "common.safety.clock.sync_timeout_s": num(required=False),
    "common.safety.clock.offset_threshold_ms": num(required=False),
    "common.safety.clock.ref_max_age_s": num(required=False),
    "common.safety.clock.unsynced_max_speed_mps": num(required=False),
    "common.safety.clock.rtc_trusted": boolean(required=False),
    "common.safety.clock.step_notify_ms": num(required=False),
    "common.safety.clock.allow_unsynced_motion": boolean(required=False),
}

# ---------------------------------------------------------------------------
# 5. p4_agent.yaml  (L6)  -- authority: 16 S14
#
# A fully-populated L6 file. Types are read off the on-disk values; keys that are
# null today (endpoint, model, tts.*, max_enum_items, throttle_speed_mps) get the
# type their context fixes (a url/name is a string, a speed is a number), which
# declares a type without inventing a VALUE.
#
# Two required=False groups, each for a documented reason:
#   * gateway.tts.* -- 16 Q-P4-32 says the whole segment's existence is undecided
#     and its correct closed form may be DELETION. Marking it required would turn
#     that legitimate future deletion into a false failure.
#   * grammar.max_enum_items -- 16 S14 forbids inventing even a bound for it; it
#     is a lone null the reader must fill, so presence is not forced here (its
#     absence is assertion A's to report, not a structural schema fault).
# ---------------------------------------------------------------------------
_P4: Dict[str, Any] = {
    # asr_post (16 S3.1): three-stage post-processing
    "asr_post.dict_file": text(),                    # L1 dict path (hot-reload)
    "asr_post.pinyin_distance_threshold": integer(),  # L2 edit-distance gate
    "asr_post.ignore_tone": boolean(),               # L2 tone-insensitive flag
    "asr_post.closed_set_threshold": num(),          # L3 gate; Q-P4-9 placeholder
    "asr_post.log_corrections": boolean(),           # keep correction trail
    # bypass (16 S4): three word lists + a flag
    "bypass.match_before_postprocess": boolean(),    # match estop pre-correction
    "bypass.estop": listof(),                        # estop word list
    "bypass.prone": listof(),                        # prone word list
    "bypass.stand": listof(),                        # stand word list
    # intent routing (16 S5)
    # 2026-08-10 CFG-FZ-18-b: intent.keyword_rules removed. The path is
    # provided by common.cmdset.intents_file (canonical name from 10
    # S5.4.5 cmdset row) and consumed via ${common.cmdset.intents_file}
    # -- see configs/p4_agent.yaml top-of-intent-block comment. Zero
    # consumers grep in xbrain/p4_agent for the old key; safe to drop.
    "intent.use_small_model": boolean(),             # enable tier-2 classifier
    "intent.fallback_to_llm": boolean(),             # fall through to llm
    # prompt (16 S6): integer token budgets + files + history
    "prompt.budget_tokens.system": integer(),        # tokens, system layer
    "prompt.budget_tokens.mission": integer(),       # tokens, mission layer
    "prompt.budget_tokens.context": integer(),       # tokens, context layer
    "prompt.budget_tokens.history": integer(),       # tokens, history layer
    "prompt.budget_tokens.total": integer(),         # tokens, total budget
    "prompt.system_file": text(),                    # system prompt path
    "prompt.mission_dir": text(),                    # mission prompt dir
    "prompt.history.enable_on": listof(),            # closed 3-value set (16 S6.3.5)
    "prompt.history.line_templates.clarify": text(),         # clarify template
    "prompt.history.line_templates.recording": text(),       # recording template
    "prompt.history.line_templates.pending_confirm": text(),  # confirm template
    "prompt.history.pinned": boolean(),              # never trimmed (H-3)
    "prompt.cache_prompt": boolean(),                # llama.cpp prompt cache
    # grammar (16 S7)
    "grammar.enable": boolean(),                     # GBNF constrained decode
    "grammar.regenerate_on_data_change": boolean(),  # rebuild grammar on change
    "grammar.max_enum_items": integer(required=False),   # lone null, see header
    "grammar.fallback_on_overflow": text(),          # overflow strategy name
    # gateway.llm (16 S9)
    "gateway.llm.endpoint": text(),                  # url; null today (Q-P4-3)
    "gateway.llm.timeout_s": num(),                  # s llm timeout (8s, 16 S9.3)
    "gateway.llm.n_predict_max": integer(),          # max decode tokens
    "gateway.llm.circuit_breaker.fail_threshold": integer(),  # trips after N fails
    "gateway.llm.circuit_breaker.open_s": integer(),          # s breaker open
    # gateway.asr (16 S9)
    "gateway.asr.base_url": text(),                  # asr service base url
    "gateway.asr.path": text(),                      # transcription endpoint path
    "gateway.asr.model": text(),                     # null today (D-AI-1/2)
    "gateway.asr.language": text(),                  # asr language tag
    "gateway.asr.response_format": text(),           # verbose_json (AS-4)
    "gateway.asr.hotwords_via_prompt": boolean(),    # V-A3 hotword carrier
    "gateway.asr.timeout_s": num(),                  # AS-7 upper bound is assertion G
    # gateway.tts (16 S9); whole segment undecided (Q-P4-32) -> required=False
    "gateway.tts.base_url": text(required=False),    # tts base url (segment TBD)
    "gateway.tts.path": text(required=False),        # tts endpoint path
    "gateway.tts.model": text(required=False),       # tts model name
    "gateway.tts.voice": text(required=False),       # tts voice name
    "gateway.tts.speed": num(required=False),        # tts speed factor
    "gateway.tts.response_format": text(required=False),  # tts response format
    "gateway.tts.stream": boolean(required=False),   # tts streaming flag
    "gateway.tts.timeout_s": num(required=False),    # s; AS-7 bound is assertion G
    # gateway.gpu_token (16 S9); throttle_speed_mps bound is SP-11 (assertion G)
    "gateway.gpu_token.enable": boolean(),           # token gating on
    "gateway.gpu_token.count": integer(),            # concurrent token count
    "gateway.gpu_token.acquire_timeout_s": num(),    # s acquire timeout
    "gateway.gpu_token.throttle_speed_mps": num(),   # null today (Q-P4-13)
    # auth / level registration (16 S8.3A)
    "auth.table_file": text(),                       # level table path
    "auth.assert_row_count": integer(),              # expected row count (CS-A3)
    "auth.assert_intent_count": integer(),           # expected intents (CS-A2)
    "auth.assert_level_hist.L0": integer(),          # level histogram, L0
    "auth.assert_level_hist.L1a": integer(),         # level histogram, L1a
    "auth.assert_level_hist.L1b": integer(),         # level histogram, L1b
    "auth.assert_level_hist.L2": integer(),          # level histogram, L2
    "auth.assert_level_hist.L3": integer(),          # level histogram, L3
    "auth.cmdset_18_export": text(),                 # 18-series export path
    "auth.scope_default_h01": text(),                # H01 scope default
    "auth.force_step_default_h03": boolean(),        # H03 force-step default
    "auth.h03f_channels": listof(),                  # channels allowed for force
    "auth.which_shared_production": boolean(),       # shared production flag
    "auth.nonvoice_estop.use_state_link": boolean(),   # criterion 2 source
    "auth.nonvoice_estop.use_state_teleop": boolean(),  # criterion 1 source
    "auth.nonvoice_estop.teleop_rest_poll_s": integer(),   # 0 is a real value here
    "auth.nonvoice_estop.teleop_stale_s": num(),     # s staleness cutoff
    "auth.nonvoice_estop.unknown_is_unhealthy": boolean(),  # unknown = unhealthy
    "auth.a06_backward_static_l1b": boolean(),       # A06 static-L1b default
    "auth.l1b_wait.on_gate_reopen": boolean(),       # wait on mic reopen
    "auth.l1b_wait.fallback_ms_extra": integer(),    # ms tail fallback
    "auth.l1b_wait.on_speak_rejected": text(),       # escalate-on-reject action
    "auth.tts_est.default_s_per_sentence": num(),    # s per sentence estimate
    "auth.tts_est.max_duration_factor": num(),       # est x factor
    "auth.tts_est.max_duration_floor_ms": integer(),  # ms floor
    # session (16 S11.1)
    "session.idle_clear_s": integer(),               # s idle clear
    "session.slots": listof(),                       # session slot names
    # recording routing (16 S11.3): a single P4-side routing decision
    "recording.reject_navigation_while_recording": boolean(),  # CMD-16 routing
    # confirm (16 S11.4)
    "confirm.timeout_s": num(),                      # s confirm timeout
    "confirm.max_reask": integer(),                  # max re-ask count
    "confirm.require_impact_summary": boolean(),     # CMD-31 impact summary
    # chitchat (16 S11.5)
    "chitchat.file": text(),                         # chitchat whitelist path
    "chitchat.allow_llm_freeform": boolean(),        # CMD-55 freeform flag
    "chitchat.out_of_scope_consecutive_threshold": integer(),  # oos streak limit
    # cmdset (16 S12.4): version labels + paths
    "cmdset.version": text(),                        # cmdset semver label
    "cmdset.min_agent_version": text(),              # min agent version gate
    "cmdset.intents_file": text(),                   # intents.yaml path
    "cmdset.missions_dir": text(),                   # missions dir path
    "cmdset.query_templates": text(),                # query templates path
    "cmdset.restate_templates": text(),              # restate templates path
    "cmdset.conflict_check_on_load": boolean(),      # keyword conflict check
}

# ---------------------------------------------------------------------------
# 6..19. The deferred files.
#
# Empty on disk; key set deferred to the cited volume and, in places, still
# pending a ruling. Schema is intentionally empty (no fields): it accepts the
# current empty file AND the eventual keys, and is the place to add types when
# the file is transcribed. NOT invented ahead of the config -- see the module
# header. Each entry still carries its authority so the next author knows the
# source, and so the coverage self-test can prove all 19 files have an entry.
# ---------------------------------------------------------------------------
_DEFERRED: Dict[str, str] = {
    # content files (data tables), authority per file header
    "asr_dict.yaml": "16 S12.3 (hot-reloadable ASR correction dictionary)",
    "chitchat.yaml": "16 S11.5 / 18 S17 (chitchat whitelist)",
    "intents.yaml": "16 S14 / 18 S16 (128-intent keyword table)",
    "ptz_presets.yaml": "18-B (PTZ preset table)",
    "query_templates.yaml": "16 S12.4 / 18 (query render templates)",
    "restate_templates.yaml": "16 S12.4 / 18 (restatement templates)",
    "speech_presets.yaml": "11 S8.8 (BIT speech preset manifest)",
    "suspicion_rules.yaml": "suspicion rule table (P4 pipeline)",
    "nav2/behavior_only.yaml": "13 / Nav2 behavior_server (spin/backup/wait only)",
    # L6 process-private files, authority per file header
    "p1_motion.yaml": "12 S12 (P1 private config)",
    "p2_core.yaml": "14 S11 (P2 private config, incl. bit segment)",
    "p3_task.yaml": "15 S12 (P3 private config)",
    "p5_gateway.yaml": "17 S10 (P5 private config)",
    "quadruped.yaml": "13 S8.1 / S8.2 (quadruped private config)",
}


def _build_registry() -> Dict[str, Schema]:
    """Assemble the 19 Schema assets keyed by config-relative path.

    Built once at import so a duplicate or a typo'd path fails loudly here rather
    than on first validate_config call in the field. The four typed files are
    spelled out; the deferred ones are generated with empty field maps so their
    authority string is the single source and cannot drift from the citation.
    """
    reg: Dict[str, Schema] = {
        "common.yaml": Schema("common.yaml", "10 S5.4.5", _COMMON),
        "models/m20s.yaml": Schema("models/m20s.yaml", "11 S9.6 / S9.6.1.1", _M20S),
        "safety/brake.yaml": Schema("safety/brake.yaml", "10 S5.4.5 / 11 S9.6.2", _BRAKE),
        "safety/clock.yaml": Schema("safety/clock.yaml", "11 S1.5.5", _CLOCK),
        "p4_agent.yaml": Schema("p4_agent.yaml", "16 S14", _P4),
    }
    # Empty-field schemas for the deferred files. A fresh dict per entry, never a
    # shared one, so a future edit that adds a field to one cannot leak into the
    # others through aliasing.
    for rel, authority in _DEFERRED.items():
        # Guard against a path appearing in both maps -- that would silently make
        # one definition win and is exactly the kind of drift this build catches.
        if rel in reg:
            raise ValueError(f"config path {rel!r} is defined twice in registry")
        reg[rel] = Schema(rel, authority, {})
    return reg


#: rel_path -> Schema for all 19 files. The keys ARE the fixed config-file set
#: (sites/ and calib/ are per-instance templates and are not among the 19, per
#: the CFG-FZ-17 triage).
SCHEMAS: Dict[str, Schema] = _build_registry()

#: The 19 relative paths, sorted for a stable order. Consumers (and the coverage
#: self-test) read this rather than re-deriving the set.
CONFIG_FILES: Tuple[str, ...] = tuple(sorted(SCHEMAS))


def validate_config(rel_path: str, tree: Any) -> None:
    """Validate one parsed config file by its config-relative path.

    rel_path is the file's path under the config root (e.g. "safety/brake.yaml"),
    NOT an absolute path: this package never names the config source, which is
    why no_config_source_read.py finds nothing to flag here. The freeze service
    resolves the root, reads and parses the file, then hands (rel_path, tree) in.

    A path with no schema RAISES rather than passing. A silent pass would mean a
    newly added config file ships with no schema and no one notices until a bad
    value reaches a process -- the same silent-shrink failure the error-code
    loader guards against. The coverage self-test keeps SCHEMAS and the on-disk
    file set in step so this raise does not fire in normal operation.
    """
    schema = SCHEMAS.get(rel_path)
    if schema is None:
        raise SchemaError(
            f"no CFG-10 schema registered for config file {rel_path!r}; "
            f"every file under the config root must have one (CFG-FZ-17)",
            path=rel_path, expected="a registered schema", actual="none")
    validate_tree(schema, tree)
