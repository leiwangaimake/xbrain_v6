"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_consumer_contract.py
Brief: W-11 consumer contract -- each simulated scenario -> the RNS verdict

Description:
The RNS side's half of the interface stage (perception-rns-reply-20260911
Q1.4 stage 1): the seven sample sequences under tests/perception/samples/ are
fed through the PRODUCTION path (perception_src.three_keys parsers, then
RnsSource.compute with a real tick context) and the verdict the contract
prescribes is asserted per scenario. Two guards on the samples themselves:
disk == generator (nobody hand-edits a sample into something the contract
never said) and every sample parses strictly (a sample the intake would
reject is not a sample).

Verdicts and their clauses:
  normal          drives (vx > 0.5 * v_nom), no failure, no rejection
  all_unknown     vx <= unk_g_min * v_nom      (20 S8.1A + RNS-I-2)
  no_seg          0 < vx <= no_seg_speed_cap    (20 S3.1.11)
  extrinsic_uncal failure extrinsic_uncalibrated, mission refused (11 S3.1B.4)
  tf_stale        vx == 0, WAIT_DYNAMIC past the static dwell (20 S3.1.5 raw refused)
  clock_reset     perception_epoch_reset audited, then drives (11 S3.1B.5 v2.1)
  dropout         objects_lost audited, 0 < vx <= no_seg_speed_cap (T-52)
  semantic_only   fuse_bin UNKNOWN before / BLOCKED at the S-only 2.5 m edge (11 v2.2)
  ground_withdrawn fit fallback withdrawn (19 S3.2A v1.6): vx <= unk_g_min * v_nom, no
                  failure, both reasons parsed, the p1 host gate does NOT veto (fresh != dead)
Each verdict is also the mutant list: the B1/B2 source mutants (T-52 cap off,
extrinsic flag ignored, raw accepted) redden the matching scenario here too.
"""
from __future__ import annotations

import copy
import importlib.util
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import pytest
import yaml

from xbrain.p1_motion.perception_src.three_keys import KEYS, parse_payload
from xbrain.p1_motion.rns.inputs import PerceptionSnapshot
from xbrain.p1_motion.rns.route import Mission
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind, NavFailReason, NavState, Origin

pytestmark = pytest.mark.no_device

ROOT = Path(__file__).resolve().parents[2]
SAMPLES = ROOT / "tests" / "perception" / "samples"
SIM = ROOT / "scripts" / "dev" / "perception_sim.py"
_REAL_CFG = yaml.safe_load((ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))
R_EFF = 0.5
V_NOM = 1.0


def _sim():
    """Import the generator by path (scripts/ is not a package)."""
    spec = importlib.util.spec_from_file_location("perception_sim", SIM)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@dataclass
class Ctx:
    pose_xy: Optional[Tuple[float, float]] = (0.0, 0.0)
    yaw_rad: Optional[float] = 0.0
    v_nom_mps: Optional[float] = V_NOM
    wz_max_rps: Optional[float] = 1.0
    perception: Optional[PerceptionSnapshot] = None
    now_mono_ms: Optional[int] = None
    holonomic: bool = False


def _cfg():
    return copy.deepcopy(_REAL_CFG)


def _mission():
    pts = [(i * 0.5, 0.0) for i in range(41)]
    return Mission(MissionKind.PATH, Origin.ROUTE, pts,
                   search_window=8, arrival_radius_m=1.0, max_deviation_m=10.0)


def _load(name: str) -> dict:
    return json.loads((SAMPLES / name / "sequence.json").read_text(encoding="utf-8"))


def _snapshot(tick: dict) -> PerceptionSnapshot:
    """The production decode path: wire JSON -> envelope -> body -> DTO."""
    dtos = {}
    for key in KEYS:
        w = tick.get(key)
        dtos[key] = None if w is None else parse_payload(
            key, json.dumps(w).encode("utf-8"))
    return PerceptionSnapshot(profile=dtos["profile"], objects=dtos["objects"],
                              status=dtos["status"])


def _run(name: str):
    """Feed every tick of the scenario; return (source, last output)."""
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    out = None
    for tick in _load(name)["ticks"]:
        out = s.compute(Ctx(perception=_snapshot(tick), now_mono_ms=tick["now_mono_ms"]))
    return s, out


# ── the samples themselves ───────────────────────────────────────────────────
def test_samples_on_disk_match_generator():
    # disk == generator: a hand-edited sample would silently change what the
    # contract "says". mutant: edit any sample file -> reddens.
    sim = _sim()
    expected = sim.render(sim.build_scenarios("dev"))
    for name, text in expected.items():
        assert (SAMPLES / name / "sequence.json").read_text(encoding="utf-8") == text, name
    index = json.loads((SAMPLES / "index.json").read_text(encoding="utf-8"))
    assert set(index) == set(expected) == set(sim.SCENARIOS)


def test_every_sample_parses_strictly():
    # a sample the production intake would reject is not a sample.
    for name in _sim().SCENARIOS:
        for tick in _load(name)["ticks"]:
            _snapshot(tick)


# ── verdicts ─────────────────────────────────────────────────────────────────
def test_normal_drives_without_failure():
    s, out = _run("normal")
    assert out is not None and out.vx.value > 0.5 * V_NOM
    assert s.take_failure() is None and s.nav_state() == NavState.FOLLOW
    kinds = [r.kind for r in s.audit.drain()]
    assert not any(k.startswith("perception_") for k in kinds)


def test_all_unknown_is_capped_by_unknown_ratio():
    # 20 S8.1A: forward sector all UNKNOWN -> g_down saturates at unk_g_min;
    # RNS-I-2 (invalid ratio 0.85) caps as well. Never full speed.
    s, out = _run("all_unknown")
    cap = _cfg()["rns"]["speed"]["unk_g_min"] * V_NOM
    assert out is not None and out.vx.value <= cap + 1e-9
    assert s.take_failure() is None


def test_no_seg_limits_speed_to_no_seg_cap():
    s, out = _run("no_seg")
    cap = _cfg()["rns"]["perception"]["no_seg_speed_cap_mps"]
    assert out is not None and 0.0 < out.vx.value <= cap + 1e-9


def test_extrinsic_uncalibrated_refuses_the_mission():
    s, out = _run("extrinsic_uncal")
    f = s.take_failure()
    assert f is not None and f.reason is NavFailReason.EXTRINSIC_UNCALIBRATED
    assert s.nav_state() == NavState.IDLE
    assert out is not None and out.vx.value == 0.0


def test_tf_stale_raw_velocity_keeps_the_car_dynamic():
    # a raw-trusting consumer would let the zero-velocity car go static at
    # 2 s and drive on; the refusal (A-FUS-7) keeps it a stop. mutant: return
    # the raw value in usable_velocity -> drives -> reddens.
    s, out = _run("tf_stale")
    assert out is not None and out.vx.value == 0.0
    assert s.nav_state() == NavState.WAIT_DYNAMIC


def test_clock_reset_is_an_epoch_reset_not_a_drop():
    s, out = _run("clock_reset")
    kinds = [r.kind for r in s.audit.drain()]
    assert "perception_epoch_reset" in kinds
    assert "perception_out_of_order" not in kinds
    assert out is not None and out.vx.value > 0.0
    assert s.take_failure() is None


def test_semantic_only_edge_blocks_without_granting_free():
    # 11 S3.1B.1 v2.2 (Q3 ruling): a bin with no depth but a semantic block at
    # 2.5 m is BLOCKED there and UNKNOWN before it -- S never grants FREE. The
    # assertion is on the accepted profile through fuse_bin (the tick's own
    # consumption rule, 20 S3.1.2), not on the memory grid: a 0.25 m cell
    # merges ~11 bins at 2.5 m, so a neighbouring open bin's FREE sample can
    # land in the same cell (20 #20-27, out of this contract's scope).
    # mutant: fuse_bin returning FREE for d_free None (RNS-I-1 broken) ->
    # the 1.5 m query reads FREE -> reddens.
    from xbrain.p1_motion.rns.grid import fuse_bin
    from xbrain.p1_motion.rns.types import Cell
    s, out = _run("semantic_only")
    prof = s._acc["profile"]                      # the last ACCEPTED profile
    i = 90                                        # dead ahead (angle 0)
    assert prof.d_free[i] is None and prof.d_block[i] == 2.5 and prof.src[i] == 0b0100
    assert fuse_bin(prof.d_free[i], prof.d_block[i], prof.src[i], 1.5) == Cell.UNKNOWN
    assert fuse_bin(prof.d_free[i], prof.d_block[i], prof.src[i], 2.5) == Cell.BLOCKED
    assert s.take_failure() is None and out is not None


def test_dropout_of_objects_caps_speed_and_is_audited():
    s, out = _run("dropout")
    cap = _cfg()["rns"]["perception"]["no_seg_speed_cap_mps"]
    assert out is not None and 0.0 < out.vx.value <= cap + 1e-9
    assert "objects_lost" in [r.kind for r in s.audit.drain()]
    assert s.take_failure() is None


def test_ground_withdrawn_is_unknown_not_a_stop():
    """19 S3.2A v1.6 / r5: a withdrawal frame is UNKNOWN everywhere but the
    independent S block. RNS: capped by the UNKNOWN share, no failure, the S
    edge still BLOCKED. Host (p1 host_gate): fresh + no forward bin known is
    NOT the dead veto -- the ceiling stays profile/spec. mutant: veto on
    f_free None in host_gate -> red; mutant: drop the S block from the
    scenario's fuse -> red."""
    from xbrain.p1_motion.nav.health_factor import HealthView
    from xbrain.p1_motion.nav.host_gate import compute_gate, forward_d_free
    s, out = _run("ground_withdrawn")
    cap = _cfg()["rns"]["speed"]["unk_g_min"] * V_NOM
    assert out is not None and out.vx.value <= cap + 1e-9
    assert s.take_failure() is None
    last = _load("ground_withdrawn")["ticks"][-1]
    snap = _snapshot(last)
    assert set(snap.status.degraded_reasons) >= {"ground_fit_fallback", "ground_free_withdrawn"}
    assert all(v is None for v in snap.profile.d_free)
    assert snap.profile.src[90] & 0b0100 and snap.profile.d_block[90] == 2.5
    assert forward_d_free(snap.profile.d_free) is None
    g = compute_gate(v_nom_mps=2.0, spec_max_vx_mps=2.0,
                     f_free_mps=forward_d_free(snap.profile.d_free),
                     health=HealthView(1.0, True, "patrol", "ok", 100), i_fix=1.0,
                     i_heading=1.0, heading_valid=True, estop=False, perception_dead=False)
    assert not g.veto and g.v_max_fwd == 2.0
