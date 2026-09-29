"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_unwired_modules.py
Brief: meta door -- a module nothing in production even names is registered, not silent

Description:
This file solves one problem: a module can sit in xbrain/ with a full
green test suite and NO production caller, and nothing says so.

The repository has already paid for this twice, both recorded in
docs/NEXT.md:
  * route/push.py -- 97 lines, 6 green tests, zero callers, and not one
    of its fields matched 11 S3.5A RouteGeometry. It stayed that way for
    months while the question it was supposed to answer ("can P3 push a
    route?") had the answer "no".
  * the r_eff max() test in the old rns/ tree -- a green test pinning a
    shape that 12 v0.7 had already declared void, i.e. a test that forced
    the WRONG implementation (docs/rns-legacy-audit.md S3).
Both are the same failure: a unit test imports the function directly and
asserts on it, so the test is green whether or not anything calls it.
CLAUDE.md S3.2 form 1 -- "is there an implementation that does nothing
and still passes?" For a module with no callers the answer is always yes,
because "does nothing" is exactly what it already does.

WHAT THIS DOOR DOES
  It computes the set of xbrain/ modules that are not NAMED anywhere else
  in the production trees, and compares it against the frozen registry
  below in BOTH directions:
    * a module becomes unwired and is not in the registry  -> red
    * a registry entry becomes wired (or is deleted)       -> red
  The second direction is the half that makes the list shrink instead of
  rot: you cannot wire something and leave a stale entry behind.

WHY "NAMED" AND NOT "IMPORTED"
  An import-based checker has to model relative imports, imports made
  inside a function body (this repo does that on purpose in the runtime
  wirings to keep startup cheap), and importlib. Getting any of those
  wrong produces a FALSE POSITIVE on a module that really is wired --
  and a door that is red for a wired module is a door somebody relaxes
  to "contains is enough" within the week (CLAUDE.md S3.2 form 2).
  A name-mention check errs the other way: a module mentioned only in a
  neighbouring comment counts as wired and is NOT reported. So this door
  under-reports by construction. What it does report is the hard core:
  modules that no other production file names at all. That asymmetry is
  deliberate; do not "tighten" it into an import graph without also
  proving the false-positive rate is zero.

WHAT THIS DOOR DOES NOT DO
  It does not say a listed module is wrong, dead, or should be deleted.
  Startup self-checks, static-scan helpers and lint bodies legitimately
  have no in-tree caller. It says only: nothing production-side names it,
  therefore no test of it can tell you whether the system uses it. The
  disposition per entry is the note in the registry, and each note is
  supposed to become either a wiring commit or a deletion commit.

SCAN SURFACE (CLAUDE.md S3.2 form 6 -- an undeclared count is unreadable)
  candidates: xbrain/**/*.py, excluding __init__.py and _legacy/ trees
  mentions:   xbrain/**/*.py + scripts/**/*.py
  NOT in the surface: tests/ -- which is where this file lives, so the
  door cannot match its own text (CLAUDE.md S3.2 form 3, self-harm).
  The surface is printed in every failure message; a number produced by
  an unstated surface is not a finding.

RED PROOF (CLAUDE.md S3.3 -- an assertion that never went red is unwritten)
  Drop a new .py under xbrain/ that nothing names: this goes red naming
  it. Delete any registry entry: this goes red the other way. Both were
  run before this file was committed.
"""

import os
import re

import pytest

pytestmark = pytest.mark.no_device

# Repository root, derived from this file so the door works from any cwd.
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# --- scan surface, declared as data so the failure message can print it ---

# Where candidate modules are looked for. Only xbrain/ -- C++ lives in
# common/ and ros2_ws/ and has its own discipline gate (tests/ci).
_CANDIDATE_ROOTS = ("xbrain",)

# Where a mention counts. scripts/ is included because several xbrain
# modules are driven by a scripts/ entry point rather than by another
# xbrain module, and that is wiring, not orphanhood.
_MENTION_ROOTS = ("xbrain", "scripts")

# Directory names excluded from candidacy. _legacy is the tombstone tree
# created by PM1.5 (docs/rns-legacy-audit.md S2): those files are kept
# ON PURPOSE with headstone headers pointing at what replaced them, so
# reporting them would be noise that trains people to ignore this door.
_EXCLUDED_DIR_PARTS = ("__pycache__", "_legacy")


# --- the frozen registry -------------------------------------------------
#
# Every entry is DEBT, not an exemption. The note says why it is here and
# who closes it. Four tags, closed set:
#
#   ruled-dead  a ruling to delete it already exists; only execution is
#               missing. Closing move = delete the file and its tests.
#   no-surface  it guards or serves an interface that does not exist in
#               production, so its own tests are green by construction.
#               Closing move = build the surface, or delete the module.
#   registered  already carried in docs/NEXT.md with a stated blocker;
#               listed here only so the set is complete.
#   untriaged   the reconciliation did not get to it. This tag is the one
#               that must go to zero; it is not a resting place. It
#               REACHED zero on 2026-09-28 -- if you find yourself adding
#               it back, the entry you are about to write is the thing to
#               think about, not the tag.
#
# Do NOT add an entry to make this test green. Adding one is a claim
# that the module has no production caller -- which is the defect, not
# the fix.
_KNOWN_UNWIRED = {
    # ---- ruled-dead ----
    # Routes cmd/motion/behavior{goto} to path_follow(300). 12 v0.8
    # (#20-1/#20-9) REMOVED path_follow from the behavior source closed
    # set; sources/arbiter_p1.py has no PATH_FOLLOW member any more. So
    # this returns a source that cannot exist, and its tests pin it.
    # Same shape as the r_eff max() test: green, and wrong to follow.
    "xbrain/p1_motion/route/behavior_goto.py": "ruled-dead: 12 v0.8 deleted path_follow",

    # ---- no-surface ----
    # Guards the PTZ LAPI WRITE key set. There is no LAPI client in
    # production -- p2_core/ptz/ speaks ONVIF only. A guard with no write
    # site to guard cannot fail, so BIZ-P2-10 is not actually met.
    "xbrain/p2_core/domains/lapi_guard.py": "no-surface: no LAPI client in production, ONVIF only",

    # ---- registered elsewhere ----
    # NEXT S5 SW-23: its regex extracts 0 rows from 18 today, so the
    # CS-A1/CS-A2 two-way diff is empty-vs-empty and always passes.
    "xbrain/p4_agent/registry/cmdset_extractor.py": "registered: NEXT S5 SW-23",
    # NEXT S5 SW-21 UNIT class: implementation exists, zero consumers.
    "xbrain/p2_core/health/restrict_matrix.py": "registered: NEXT S5 SW-21 UNIT class",

    # ---- the STARTUP GATE tranche, triaged 2026-09-28 ----
    # Five modules named themselves a startup gate ("refuse startup",
    # "refuse to boot", "every one refuses process start if it fails")
    # and not one was called. A gate nobody invokes and a gate that
    # always passes are indistinguishable from outside -- CLAUDE.md S3.2
    # form 1 at process scale. Outcome: refuse_to_boot.py was WIRED into
    # the freeze entry point; p4 startup_assertions.py lost its duplicate
    # half and had its unique half wired; the two zenoh ones are
    # no-surface above; this last one waits on a ruling.
    #
    # CFG-BT-14. Two separate reasons it cannot just be wired.
    # (1) No consumer exists. The classifier answers "what CLASS is this
    #     startup failure", and the things that would act on the answer --
    #     the observation window (CFG-BT-4) and P1's allow_motion gate
    #     (CFG-BT-11) -- are Phase 2 and not built. Nothing in the tree
    #     calls classify(); boot_fail.py (which IS wired, from
    #     p5_gateway/__main__.py) records failures without classifying.
    # (2) Three of its 29 rows are a decision it was not entitled to
    #     make. 10 S3.3.6 gives row 27 (RTK not fixed) the class column
    #     verbatim "不属失败" and rows 28/29 (perception / RNS internal
    #     boot failure) verbatim "指针" -- neither is one of R/B/D/T.
    #     _CLASSIFIER_TABLE assigns CLASS_D to all three, and its own
    #     comment says "# Non-failure: RTK not fixed" one line above
    #     doing it. requires_hmi_marker(CLASS_D) is True, so a consumer
    #     would raise a persistent HMI degraded marker on every cold
    #     start before RTK fix -- for a condition the doc says is not a
    #     failure. The meta-test only diffs the ID set against the doc,
    #     never the class column, so nothing can go red on this.
    #     docs/PHASE0_待裁决清单.md CFG-BT-14 (3) already registers the
    #     question verbatim ("这三行收不收, 收进来给什么类, 十四册没写").
    #     Ruling options: (a) drop the three rows from the table and make
    #     classify() raise for them; (b) give 27 its own non-failure
    #     class and 28/29 a pointer class, i.e. 10 S3.3.6 gains classes;
    #     (c) keep D and write the justification into 10 S3.3.6.
    #     CLAUDE.md S9.1: no verified fact decides this, so it is a
    #     ruling, not a doc defect -- NOT fixed here.
    "xbrain/boot/failure_class.py": "registered: NEXT S8.12 / PHASE0 CFG-BT-14 -- no consumer until CFG-BT-4+CFG-BT-11, and rows 27/28/29 carry an unruled class",
    # refuse_to_boot.py left this registry on 2026-09-28: it is now called
    # from xbrain/boot/freeze/__main__.py, which catches the XbrainError an
    # assertion runner raises and renders the 10 S5.4.5 three-section
    # listing instead of letting a traceback out.
    # INF-ZN-5 wants each process to hand selfcheck() the list of
    # Declarations it built. No process builds one: every runtime wiring
    # calls session.declare_publisher(key) directly, with no QoS profile
    # object anywhere (p1/p2/p4/p5 runtime/*_wiring.py), and
    # load_qos_table has exactly ONE consumer in the tree -- freeze
    # assertion F -- so the A-2..A-7 anti-patterns are checked against the
    # QoS TABLE at freeze time and there is nothing per-process to check.
    # Closing move = give the declare sites a profile (a 11 S2.4.7 data
    # question, not a code one), then call this at the end of bring-up.
    "xbrain/common/zenoh/startup_selfcheck.py": "no-surface: no process builds a Declaration list; every declare_publisher call passes a bare key",

    # INF-ZN-9 checks cross-plane FORWARDING. The only forwarder is
    # chassis_relay, which is C++, and CRL-3 makes its whitelist a
    # compile-time constant array -- scripts/ci/cxx_discipline_audit.py is
    # what audits it. No Python process forwards between planes, so this
    # module has nothing to be given. Closing move = delete it, or move
    # the audit here if the C++ one is ever retired.
    "xbrain/common/zenoh/cross_plane_compliance.py": "no-surface: the only cross-plane forwarder is C++ chassis_relay (CRL-3 compile-time array), audited by scripts/ci/cxx_discipline_audit.py",

    # CHK-2-51 audits that every scan-class script exports SCAN_SURFACE.
    # `grep -rn SCAN_SURFACE scripts/` returns ZERO: not one lint, ci or
    # doccheck script declares one, so audit_scan_scripts() runs over an
    # empty set and cannot fail (CLAUDE.md S3.2 form 1). Closing move =
    # make the scan scripts declare SCAN_SURFACE, or delete this.
    "xbrain/common/checks/scan_surface.py": "no-surface: zero scripts/ files export SCAN_SURFACE, so the audit runs over an empty set",

    # CFG-BT-18 FSC-LOCK refuses a ConfigCommand that touches a locked
    # safety key. cmd/config has NO subscriber in any of the five P
    # processes; even the CHK-1-58 admission dispatcher under
    # common/config/hotreload/ is itself uncalled (it only escapes this
    # registry because its package __init__ names it). So 11 S7.6 is
    # unimplemented at the wire, and this is a guard on a channel nobody
    # reads. Same blocker as p1_motion/config/hot_update.py below.
    "xbrain/common/config/locked_keys.py": "no-surface: cmd/config has no subscriber in any P process (11 S7.6 unimplemented at the wire)",

    # MOT-CM-1 PRC-69/70 permissive cls parser (off-set cls -> unknown +
    # one report). Nothing parses a perception cls string through it:
    # perception_src/three_keys.py and rns/grid.py both take cls as an
    # opaque value and rns/grid.py does its own _is_dynamic_class() test.
    # Closing move = route the three_keys intake through this parser so
    # an off-set class is reported once instead of flowing on silently.
    "xbrain/common/enums/cls_permissive.py": "no-surface: nothing routes a perception cls through it; rns/grid.py classifies with its own predicate",

    # MOT-PM-26 mirrors cmd/config + forwarded keys into P1. Same blocker
    # as common/config/locked_keys.py: no cmd/config subscriber exists.
    "xbrain/p1_motion/config/hot_update.py": "no-surface: same missing cmd/config subscriber as common/config/locked_keys.py",

    # MOT-PM-32 RT-C3 plane predicates. p1 opens both sessions inline in
    # runtime/main_wiring.py and never calls a plane selfcheck; the
    # predicates need the same per-declaration inventory
    # common/zenoh/startup_selfcheck.py needs and nobody builds.
    "xbrain/p1_motion/config/zenoh_planes.py": "no-surface: p1 declares on both planes inline, no declaration inventory to check",

    # CHK-1-45 R2.3-b. NOT the same rule as the wired one: nav/host_gate.
    # apply_gate clips vx < 0 to -gate.v_max_free, which is v_lin_free*h*i
    # and can exceed 0.5 m/s, while R2.3-b is an ABSOLUTE rear cap because
    # there is no rear perception (11 S15.6 D-33). So this is a real gap.
    #   2026-09-30: the module also USED to define NEGATIVE_VX_CAP_LIMITER =
    #   "negative_vx_cap", commented "closed-set enum value", while 11
    #   S9.6.5 / enums/sets.yaml gate_limiter carries no such member --
    #   wiring it would have put an out-of-set gate.limiter on the wire
    #   (CLAUDE.md S3.5). Removed: apply() now reports only WHETHER it
    #   clipped.
    # What still blocks wiring is the attribution itself, and that is a
    # ruling, not a code change: 12 S6A.8 OB-1 forbids adding a member, and
    # no existing member fits (S9.6.5's argmax runs over unsigned magnitude
    # caps; this one exists only when vx < 0, and step 4's downstream list
    # was narrowed in v0.7 to exactly brake and fence). Same shape as OB-4
    # for wz: a new gate field, or an event. 11's call.
    "xbrain/p1_motion/gate/negative_vx.py": "registered: NEXT S8.12 -- real gap, but R2.3-b has no gate.limiter member and OB-1 bars adding one; needs a ruling on the attribution before it can be wired",

    # MOT-PM-16 PS-1..PS-6. p1 runtime/main_wiring.py builds and publishes
    # state/pose inline in the GNSS bridge (via path/gnss_pose.py); the
    # MotionSnapshot -> pose/cmd_vel byte-identity shape here has no call
    # site. Closing move = make the bridge go through it, or delete.
    "xbrain/p1_motion/path/pose_assembly.py": "no-surface: main_wiring publishes state/pose inline through path/gnss_pose.py",

    # MOT-PM-30 TR-1..TR-3 recording lock. Recording (teach) lives in P3
    # (p3_task/teach/session.py + runtime.py); p1 holds no recording
    # state at all, so there is no p1-side owner for this lock.
    "xbrain/p1_motion/path/recording_lock.py": "no-surface: teach recording state lives in p3_task/teach/, p1 holds none",

    # CHK-1-12 profile switch SM. The 20 Hz path is nav/nav_tick.py and it
    # has no profile-switch stage; ctrl_loop's 12 S2.2 ten-step list has
    # none either. Nothing publishes or consumes a profile transition.
    "xbrain/p1_motion/profile/switch_sm.py": "no-surface: neither nav_tick nor ctrl_loop has a profile-switch stage",

    # CHK-1-20 is the UNIQUE BYPASS of the rotation permit. 2026-09-29: the
    # permit itself IS running now (nav_tick step 6b -> rotation/rcg.py), so
    # the old reason here -- "the gate never fires" -- no longer holds and has
    # been replaced rather than left standing.
    #
    # The bypass stays unwired on purpose, and the reason is 12 S6A.6's own
    # fourth condition: the release must land an event naming the confirming
    # command's origin and cmd_id. Step 6b sees a velocity, not a command, so
    # it has no identity to name; 12 S6A.6 says that without that record the
    # switch is a silent safety-release channel. rcg.py therefore refuses to
    # construct when allow_visual_override is true, which is the same verdict
    # reached loudly. Closing move = wire the permit's ENTRY call site
    # (12 S4.6.4 RC-1), where a cmd_id exists, and route the bypass there.
    "xbrain/p1_motion/rotation/visual_override.py": "no-surface: 12 S6A.6 needs the confirming cmd_id in the release event and step 6b has none; the entry call site (12 S4.6.4 RC-1) is unwired",

    # MOT-PM-21. Two problems. (1) Superseded: teleop/state.py is the
    # WIRED arbiter (main_wiring.py:266 TeleopTracker) and implements
    # T-1..T-5 with the 11 S12A.9.7 closed set gamepad / keyboard_local /
    # keyboard_hmi / virtual_stick. This module holds the same four names
    # but only parses and ages frames; it ranks nothing, so wiring it
    # would put a SECOND freshness table beside the arbiter's.
    #   2026-09-30: its TeleopSource used to be keyboard / joystick / hmi
    #   / cloud -- not one of them in the closed set -- with 500 ms for
    #   hmi and 1000 ms for cloud, where 11 S12A.9.6 and 11 S13
    #   E_TELEOP_STALE both give 200 ms local / 400 ms HMI. Its tests
    #   pinned all of it. Corrected in the same commit as this note; the
    #   registration below stands on the duplication, not on the names.
    # (2) The TL-1/TL-2 half (parse estop before normalize) has no other
    # implementation, but its input rt/teleop/input has no subscriber and
    # 11 S12A.9.4's 2026-08-05 F3 finding leaves WHO relays the gamepad
    # estop undecided between three candidates. Deleting the file would
    # delete TL-1/TL-2 with it, so this waits on that ruling.
    "xbrain/p1_motion/teleop/four_source.py": "registered: NEXT S8.12 -- arbiter half superseded by teleop/state.py, this one would be a second freshness table; TL-1/TL-2 half blocked on the 11 S12A F3 relay ruling",

    # BIZ-P2-8 decides auto lighting from three sources (photocell /
    # image brightness / almanac). None of the three exists anywhere in
    # the tree: `grep -rn "photocell|almanac"` over xbrain/ hits only this
    # file. runtime/payload_wiring.py serves explicit D-class light
    # intents and nothing else. Closing move = build a source, or delete.
    "xbrain/p2_core/domains/lighting_auto.py": "no-surface: none of the three A/B/C light sources (photocell / image / almanac) exists",

    # CHK-1-09 health for cam_ptz_ir. That item is not a row in
    # p2_core/health/items.py, so the aggregate has no such item to drive
    # and nothing can report it unhealthy.
    "xbrain/p2_core/health/aux/ir_camera.py": "no-surface: cam_ptz_ir is not a row in p2_core/health/items.py",

    # BIZ-P2-12 system mode -> device mode, i.e. what to POST to
    # payload-service /mode. There is no /mode client: payload_client/
    # holds only errors_map.py, and no module in the tree POSTs /mode
    # (the wired payload clients are p4_agent/ai_client/lights_client.py
    # and tts_client.py, /lights and /tts). mode/state_machine.py says
    # verbatim that it does NOT own the acquire/release calls and points
    # here, so the pointer is correct and the target is unreachable.
    "xbrain/p2_core/mode/device_map.py": "no-surface: no payload-service /mode client exists (payload_client/ is errors_map only)",

    # CHK-2-08 PAY-29 needs ptz.t_drift_s. configs/p2_core.yaml's ptz
    # block has no such key, and the wired PtzDriver has no timer of any
    # kind. Closing move = the key is a 14 S11 addition, which is a
    # config-contract change, not a wiring change.
    "xbrain/p2_core/ptz/drift_home.py": "no-surface: ptz.t_drift_s does not exist in configs/p2_core.yaml's ptz block",

    # CHK-1-33 PTZ-D1 long-pulse deadman. The wired PtzDriver does fixed
    # _pulse(pulse_ms) jogs (ContinuousMove, sleep, Stop); the cmd/ptz
    # envelope carries no hold_ms field for a renewal to renew.
    "xbrain/p2_core/ptz/hold_ms_renewal.py": "no-surface: cmd/ptz carries no hold_ms; PtzDriver does fixed-length pulses",

    # BIZ-P3-12 S7.3 resume MECHANICS (snapshot, start index, direction
    # consistency). P3 has the suspend half wired (lifecycle/
    # estop_suspend.py) and the ES-3 "human command unfreezes" predicate,
    # and state/machine.py has the suspended->ready transition, but
    # NOTHING computes where a resumed route restarts -- schedule/
    # driver.py just applies the transition. Real gap; wiring it changes
    # what a resumed patrol does on the ground, so it needs a bench run.
    "xbrain/p3_task/route/suspend_resume.py": "registered: NEXT S8.12 -- real gap (nothing computes the resume start index); wiring changes on-ground behaviour, needs a bench run",

    # GWY-P4-04 writes the asr_correction table (16 S3.5). No schema or
    # migration in the tree creates that table.
    "xbrain/p4_agent/asr_post/correction_log.py": "no-surface: the asr_correction table exists in no schema or migration",

    # GWY-P4-03 L1/L2/L3 ASR post-processing. The wired voice loop
    # (runtime/turn_loop.py) goes transcribe_utterance -> classify with
    # NO post-processing step between them, so raw ASR text reaches the
    # classifier. Real gap, and wiring it changes what the robot hears;
    # needs a bench run against real ASR output before it goes in.
    "xbrain/p4_agent/asr_post/three_layer.py": "registered: NEXT S8.12 -- turn_loop feeds raw ASR text straight to the classifier; needs a bench run",

    # GWY-P4-02b half-duplex observation. p4 subscribes rt/audio/mic
    # (runtime/main_wiring.py:189) but NOT rt/audio/gate, although p2
    # publishes it (runtime/speaker_wiring.py) and p4_agent/__main__.py's
    # own docstring lists "subscribe rt/audio/gate" under what it does NOT
    # do yet. failure/handlers.py GATE-1 has no input for the same
    # reason. Wireable, and the highest-value item in this list -- but its
    # fail-safe is "gate silent > 1 s => assume closed => drop ASR", so
    # turning it on can make the mic loop deaf wherever p2's speaker
    # wiring is not up. Needs a bench run, not a desk decision.
    "xbrain/p4_agent/audio_rx/gate_observer.py": "registered: NEXT S8.12 -- p4 never subscribes rt/audio/gate; wiring it can deafen the mic loop, needs a bench run",

    # GWY-P4-25 PS-1..PS-6 pose ring + still_1s_ok. turn_orchestrator
    # builds the IntentEnvelope inline (envelope/intent_envelope.py) and
    # never touches a pose ring; nothing in the tree names PoseRing.
    "xbrain/p4_agent/envelope/pose_snap.py": "no-surface: turn_orchestrator builds the envelope inline and holds no pose history",

    # CHK-1-31 RJ-1 vs RJ-2 spoken refusal scripts. The DECISION half is
    # already wired on the P2 side (runtime/motion_intent_wiring.py G-6
    # refuses a yaw intent when pose.yaw_capable is false); what has no
    # caller is the P4-side wording split. Closing move = have the
    # orchestrator render the refusal from the ack, or delete.
    "xbrain/p4_agent/failsafe/rotation_reject.py": "no-surface: the refusal DECISION is wired in p2 motion_intent_wiring (G-6); the P4 wording split has no render site",

    # CHK-2-35. Its bidirectional_diff_vs_yaml() is a META-CHECK against
    # configs/intents.yaml keywords[], and that file exists -- but the
    # test that calls it builds its fake yaml FROM D_EXPANSION_TABLE, so
    # the "synced" assertion is empty by construction and the real file is
    # never compared (CLAUDE.md S3.2 form 7). Two defects, one entry: it
    # has no production caller (a meta-check belongs in scripts/, not
    # xbrain/, CLAUDE.md S0.2) AND its only assertion cannot fail.
    "xbrain/p4_agent/intents_expand/d01_d10.py": "no-surface: meta-check with no production caller, and its test derives the oracle from the table it checks",

    # GWY-P4-28 PL-1..PL-6 D-class light/volume routing. The wired path is
    # p2_core/runtime/payload_wiring.py -> ai_client/lights_client.py,
    # which maps the D intents to /lights itself; nothing names
    # route_brightness / route_volume / route_lights_on.
    "xbrain/p4_agent/registry/d_class.py": "no-surface: payload_wiring maps the D intents to /lights directly, no projection layer",

    # GWY-P4-31 the seven 18-B rulings (E09 session tier, cumulative drift
    # warn, forbidden words in a restate). Its consumer would be the
    # restate/PTZ turn path; nothing names resolve_e09_tier or
    # CumulativeDrift, and the restate engine it would guard is itself
    # unwired (see templates/restate_engine.py below).
    "xbrain/p4_agent/registry/rulings_18b.py": "no-surface: its consumer is the restate path, which is itself unwired",

    # GWY-P4-30 deterministic time-expression normalisation. Nothing names
    # TimeExpr / parse_delay / parse_at_local. The slots that would carry
    # a time expression (p4_agent/slots/) do not call it.
    "xbrain/p4_agent/registry/time_expr.py": "no-surface: no slot extractor calls it; nothing names TimeExpr / parse_delay",

    # GWY-P4-27 projects a tools table from intents.yaml and caps it at 5.
    # No tools table is built anywhere; the AI-36 cap that IS enforced
    # lives in registry/missions.py (load time) and gbnf/generator.py
    # (grammar build), both by their own counting.
    "xbrain/p4_agent/registry/tools_projection.py": "no-surface: no tools table is built; the AI-36 cap is enforced by missions.py and gbnf/generator.py",

    # GWY-P4-17 CL-1/CL-3 + LB-1/LB-2. turn_orchestrator implements CL-2
    # (estop_path down upgrades L1b to L2) and the L0/L1a/L1b dispatch
    # INLINE, and CL-1 / CL-3 / LB-1 / LB-2 appear nowhere else in the
    # tree. So this is half duplicate, half unique -- and replacing the
    # orchestrator's inline auth gate with it changes which commands ask
    # for confirmation. That is a live-behaviour change on the
    # confirmation path; it needs a bench run.
    "xbrain/p4_agent/session/level_routing.py": "registered: NEXT S8.12 -- CL-2 + dispatch are inline in turn_orchestrator, CL-1/CL-3/LB-1/LB-2 nowhere; changing the auth gate needs a bench run",

    # GWY-P4-16 L1a/L1b restate rendering. turn_orchestrator says verbatim
    # at two places that the authoritative restate with slot values is
    # "GWY-P4-35 (render_restate), wired separately" and that what it does
    # is feedback wording only -- i.e. the runtime KNOWS this is missing
    # and names the item. Wiring it changes what the robot says out loud
    # on every L1a/L1b command; needs a bench run.
    "xbrain/p4_agent/templates/restate_engine.py": "registered: NEXT S8.12 -- turn_orchestrator names GWY-P4-35 as 'wired separately' and it never was; changes spoken output, needs a bench run",

    # GWY-P5-19/20/21 A-F group validator + the UI rotation redaction.
    # hmi/data_readers.py is the WIRED 17 S6.8 projection and validates
    # nothing; web_server.py / ws_protocol.py never name validate_group or
    # redact_ptz_angle_for_ui. Closing move = have the snapshot path
    # validate through it, or delete.
    "xbrain/p5_gateway/hmi/data_sets.py": "no-surface: hmi/data_readers.py is the wired 17 S6.8 projection and no path validates the groups",

    # GWY-P4-42 (32.J): P5 builds the cmd/voice_text envelope. P5 has no
    # text-command intake at all -- neither hmi/ws_protocol.py nor
    # inbound/cloud_inbound.py carries an utterance -- so nothing ever
    # publishes that key. The CONSUMER side is wired
    # (p4_agent/runtime/text_channel.py), which is why this is a producer
    # gap and not a dead module: build the intake, or delete.
    "xbrain/p5_gateway/text_input.py": "no-surface: P5 has no text-command intake; nothing can publish cmd/voice_text (the p4 consumer IS wired)",
}

# The tag vocabulary is closed on purpose: a free-text note field drifts
# into "TODO" within a month, and then the registry says nothing.
_LEGAL_TAGS = ("ruled-dead", "no-surface", "registered", "untriaged")


def _walk_py(root):
    """Yield every .py under root, skipping excluded directory parts."""
    base = os.path.join(_REPO, root)
    for dirpath, dirnames, filenames in os.walk(base):
        # Prune in place so os.walk does not descend into excluded trees.
        dirnames[:] = [d for d in dirnames if d not in _EXCLUDED_DIR_PARTS]
        if any(part in _EXCLUDED_DIR_PARTS for part in dirpath.split(os.sep)):
            continue
        for name in filenames:
            if name.endswith(".py"):
                yield os.path.join(dirpath, name)


def _rel(path):
    """Repo-relative path with forward slashes, so registry keys are stable."""
    return os.path.relpath(path, _REPO).replace(os.sep, "/")


def _read(path):
    """Read a source file; unreadable bytes must not take the door down."""
    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        return handle.read()


def _collect_unwired():
    """Return the set of repo-relative candidate modules that no other
    production file names.

    A candidate is any xbrain/ module file other than __init__.py -- the
    package files are naming plumbing, not units of behaviour, and they
    are reached by importing the package, so they are never 'unwired'.
    """
    candidates = {}
    for root in _CANDIDATE_ROOTS:
        for path in _walk_py(root):
            if os.path.basename(path) == "__init__.py":
                continue
            candidates[path] = os.path.basename(path)[:-3]

    corpus = []
    for root in _MENTION_ROOTS:
        for path in _walk_py(root):
            corpus.append((path, _read(path)))

    unwired = set()
    for path, stem in candidates.items():
        # Word boundaries so 'progress' does not satisfy 'progress_sink'
        # and vice versa; the stem is what both import forms spell.
        pattern = re.compile(r"\b" + re.escape(stem) + r"\b")
        if not any(pattern.search(text) for other, text in corpus if other != path):
            unwired.add(_rel(path))
    return unwired


def _surface_note():
    """The sentence every failure message carries. A count without its
    scan surface is not something a reader can act on."""
    return (
        "scan surface: candidates=%s (excluding __init__.py and %s), "
        "mentions=%s, tests/ deliberately NOT scanned"
        % (list(_CANDIDATE_ROOTS), list(_EXCLUDED_DIR_PARTS), list(_MENTION_ROOTS))
    )


def test_no_new_unwired_module_appears():
    """A module that nothing production-side names must be registered.

    This is the direction that catches a route/push.py being born: the
    file lands, its own tests are green, and this door names it the same
    day.
    """
    unwired = _collect_unwired()
    fresh = sorted(unwired - set(_KNOWN_UNWIRED))
    assert not fresh, (
        "these xbrain/ modules are named by NO other production file, so no "
        "test of them can tell you whether the system uses them:\n  %s\n"
        "Wire it, delete it, or add it to _KNOWN_UNWIRED WITH a reason "
        "(adding an entry is admitting the defect, not fixing it).\n%s"
        % ("\n  ".join(fresh), _surface_note())
    )


def test_registry_has_no_stale_entry():
    """The shrink direction: an entry that got wired, or a file that was
    deleted, must leave the registry in the same commit.

    Without this half the registry only grows, and a growing list of
    'known unwired' reads exactly like a clean report.
    """
    unwired = _collect_unwired()
    stale = sorted(set(_KNOWN_UNWIRED) - unwired)
    resolved = [p for p in stale if os.path.exists(os.path.join(_REPO, p))]
    gone = [p for p in stale if not os.path.exists(os.path.join(_REPO, p))]
    assert not stale, (
        "_KNOWN_UNWIRED is stale. Now wired (remove the entry): %s. "
        "No longer on disk (remove the entry): %s.\n%s"
        % (resolved, gone, _surface_note())
    )


def test_every_registry_entry_carries_a_legal_tag():
    """A note field with no vocabulary becomes 'TODO' and stops meaning
    anything. Each entry must open with one of the four tags, and the
    tag must be followed by a reason -- 'untriaged' is the only tag
    allowed to stand alone, and it is the one that must reach zero."""
    bad = []
    for path, note in sorted(_KNOWN_UNWIRED.items()):
        tag = note.split(":", 1)[0].strip()
        if tag not in _LEGAL_TAGS:
            bad.append((path, note, "tag %r not in %s" % (tag, list(_LEGAL_TAGS))))
            continue
        if tag != "untriaged" and ":" not in note:
            bad.append((path, note, "tag %r requires a reason after ':'" % tag))
    assert not bad, "registry entries with an unusable note: %s" % bad
