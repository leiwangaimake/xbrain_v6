"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: host_gate.py
Brief: p1 output-layer speed gate around the RNS candidate (12 S2.2 steps 3/6, 11 S9.6.2)

Description:
RNS emits a CANDIDATE (20 RNS-M-3); the host owns the velocity ceiling. This
module is the host's ceiling for the P7.2 loop: the SIL host gate (sil_server:
f(d_free) over the forward sector clamps a positive vx) plus the terms the real
process has and the simulator does not.

Formula (11 S9.6.2 / 12 S2.2 line 3, the four-in-min, two-multiplied form):
    v_lin = min(profile.max_mps, f(d_free), spec.max_vx_mps)     [m/s terms]
    v_max = v_lin * h_factor * i_factor                            [factors]
    i_factor = i_fix * i_heading (11 S3.2.1 / S3.3.1)
g(targets) is absent on purpose: 11 S3.1B replaced rt/perception/targets with
the objects list RNS consumes itself (20 S5), so near persons stop the robot
INSIDE the candidate (WAIT_DYNAMIC), not in a second host term.

Vetoes (11 S9.6.5 rows 1-5 and 8; all axes zero, limiter = the row):
    estop       soft-estop latched (P1-21)                  -> "estop"
    health      allow_motion false / factor never or dead   -> "health"
    rtk         no usable fix (i_fix absent or 0)           -> "rtk"
    heading     heading_valid false (RL-1 autonomous veto)  -> "heading"
    free_space  perception DEAD or never received           -> "free_space"
                (12 S3.3 perception row: f() -> 0, output zero)
The free_space veto is the safety gate P7.2 adds over the pre-wiring behaviour:
RnsSource._run_follow degrades to the bare follow spine when ctx.perception is
None, which is right for a replay host but must never move a real robot.

Which axes f gates. Exactly as the SIL host: f(d_free) is a FORWARD clearance,
so it caps vx > 0 only. Reverse vx and lateral vy keep the ceiling without the
f term (profile / spec x h x i) -- gating the body-shield vy or a backup with
the front clearance would pin the robot against the very wall it is easing off,
and would diverge from the three SIL batteries that validated v2.0. wz is not
gated here (rotation permit RCG-1..4 is a separate stage, not chained in this
phase; registered in NEXT.md).

Attribution (11 S9.6.5 concurrency rule): every term's cut delta is measured;
limiter = the largest delta, ties by the closed-set order (which IS the
priority order 1..13); limiter_all = all deltas > 0.05 m/s in descending delta,
then the terms tied at the min with delta 0 in closed-set order (U54 addition).
"none" only when nothing clipped and h == i == 1.0 (11 S9.6.5 last row).

Trap: computing "which term limited" as argmin of the raw terms. With profile
2.0 / f 2.0 / spec 2.0 and h 0.6 the min is a three-way tie and the real cut is
health -- the 11 S3.4 example. Delta-based attribution gets that right; argmin
does not.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from xbrain.common.enums import GATE_LIMITER
from xbrain.p1_motion.gate.speed_gate import f_speed_gate
from xbrain.p1_motion.nav.health_factor import HealthView

#: forward sector for f(d_free): bins within this many of centre (SIL host:
#: |i - 90| <= 20 on 181 bins over +/-45 deg, i.e. about +/-10 deg).
FWD_HALF_BINS = 20
#: 11 S3.4 limiter_all threshold (delta > 0.05 m/s).
LIMITER_ALL_DELTA_MPS = 0.05
#: 11 S9.6.5 order == GATE_LIMITER order; index() is the tie-break rank.
_ORDER: List[str] = list(GATE_LIMITER)
#: the two factor limiters (11 S9.6.5 rows 3-5): h -> health; i -> rtk/heading.
_FACTOR_NAMES = ("health", "rtk", "heading")


def forward_d_free(d_free: Sequence[Optional[float]]) -> Optional[float]:
    """min d_free over the forward sector; None entries (unknown bins) are
    skipped; None when no forward bin is known -- the caller treats that as
    dead (nothing in front is measured, nothing in front is safe)."""
    n = len(d_free)
    c = n // 2
    vals = [d for i, d in enumerate(d_free)
            if abs(i - c) <= FWD_HALF_BINS and d is not None]
    return min(vals) if vals else None


@dataclass(frozen=True)
class GateResult:
    """The ceiling for this tick plus what cut it."""
    v_max_fwd: float                  # ceiling for vx > 0 (includes f)
    v_max_free: float                 # ceiling for vx < 0 and |vy| (no f term)
    veto: bool
    veto_limiter: Optional[str]
    h_factor: float
    i_factor: float
    cuts: Tuple[Tuple[str, float], ...]   # (limiter, delta_mps), delta > 0
    tied_min: Tuple[str, ...]             # m/s terms tied at the minimum


def _rank(name: str) -> int:
    return _ORDER.index(name)


def _veto(limiter: str) -> GateResult:
    return GateResult(0.0, 0.0, True, limiter, 0.0, 0.0, (), ())


def compute_gate(*, v_nom_mps: float, spec_max_vx_mps: float,
                 f_free_mps: Optional[float], health: HealthView,
                 i_fix: Optional[float], i_heading: Optional[float],
                 heading_valid: bool, estop: bool,
                 perception_dead: bool,
                 profile_downgraded: bool = False) -> GateResult:
    """The ceiling. Vetoes are checked in 11 S9.6.5 priority order and the
    first hit names the limiter; otherwise the four-in-min / two-multiplied
    formula with per-term deltas for attribution.
    mutant: drop the perception_dead veto -> a robot with no perception frame
    ever drives on the bare follow spine -> test_perception_dead_is_a_veto
    red."""
    if estop:
        return _veto("estop")
    if not health.allow_motion:
        return _veto("health")
    if i_fix is None or i_fix <= 0.0:
        return _veto("rtk")
    if not heading_valid or i_heading is None or i_heading <= 0.0:
        return _veto("heading")
    if perception_dead or f_free_mps is None:
        return _veto("free_space")
    h = float(health.speed_factor)
    i = float(i_fix) * float(i_heading)
    f = f_speed_gate(f_free_mps)
    terms: Dict[str, float] = {"profile": float(v_nom_mps),
                               "free_space": f,
                               "spec": float(spec_max_vx_mps)}
    v_lin = min(terms.values())
    v_lin_free = min(terms["profile"], terms["spec"])
    # delta of the min term(s): what the next-larger term would have allowed.
    above = sorted(v for v in terms.values() if v > v_lin)
    second = above[0] if above else v_lin
    tied = tuple(sorted((k for k, v in terms.items() if v == v_lin), key=_rank))
    # 11 S3.6 max_profile: a health-imposed tier cap is attributed to health
    # (11 S9.6.5 row 3 "max_profile 导致降档"), not to profile.
    if profile_downgraded:
        tied = tuple("health" if k == "profile" else k for k in tied)
        tied = tuple(sorted(tied, key=_rank))
    cuts: List[Tuple[str, float]] = []
    term_delta = second - v_lin
    if term_delta > 0.0 and len(tied) == 1:
        cuts.append((tied[0], term_delta))
    # factor deltas, applied in the 11 S9.6.2 order (h then i).
    d_h = v_lin * (1.0 - h)
    if d_h > 0.0:
        cuts.append(("health", d_h))
    d_i = v_lin * h * (1.0 - i)
    if d_i > 0.0:
        # the smaller factor dominates; equal -> rtk (priority order).
        cuts.append(("rtk" if float(i_fix) <= float(i_heading) else "heading",
                     d_i))
    cuts.sort(key=lambda kv: (-kv[1], _rank(kv[0])))
    return GateResult(v_max_fwd=v_lin * h * i, v_max_free=v_lin_free * h * i,
                      veto=False, veto_limiter=None, h_factor=h, i_factor=i,
                      cuts=tuple(cuts), tied_min=tied)


def attribute(gate: GateResult, raw_vx: float) -> Tuple[str, Tuple[str, ...]]:
    """(limiter, limiter_all) for the 11 S3.4 gate block. "none" iff no veto,
    |raw_vx| within the ceiling and h == i == 1.0 (11 S9.6.5 last row).
    mutant: report the tied-min term instead of the largest delta -> the
    11 S3.4 worked example attributes to free_space instead of health ->
    test_attribution_matches_11_s34_example red."""
    if gate.veto:
        return (gate.veto_limiter or "none", (gate.veto_limiter,) if gate.veto_limiter else ())
    ceiling = gate.v_max_fwd if raw_vx >= 0.0 else gate.v_max_free
    clipped = abs(raw_vx) > ceiling + 1e-9
    if not clipped and gate.h_factor == 1.0 and gate.i_factor == 1.0:
        return ("none", ())
    if gate.cuts:
        limiter = gate.cuts[0][0]
    else:
        limiter = gate.tied_min[0] if gate.tied_min else "none"
    all_: List[str] = [k for k, d in gate.cuts if d > LIMITER_ALL_DELTA_MPS]
    for k in gate.tied_min:
        if k not in all_:
            all_.append(k)
    return (limiter, tuple(all_))


def apply_gate(gate: GateResult, vx: float, vy: float, wz: float,
               holonomic: bool, wz_max_radps: Optional[float] = None) -> Tuple[float, float, float]:
    """Clip the candidate. Veto -> all zero. vx > 0 by the forward ceiling,
    vx < 0 and |vy| by the f-less ceiling; wz only by the chassis limit
    spec.max_wz_radps when given (RNS already clamps to ctx.wz_max, this is
    the host's own belt -- the rotation PERMIT is a separate stage, see the
    module note). A non-holonomic chassis never gets vy (12 S4.7.3 TS-4)."""
    if gate.veto:
        return (0.0, 0.0, 0.0)
    if vx > 0.0:
        vx = min(vx, gate.v_max_fwd)
    elif vx < 0.0:
        vx = max(vx, -gate.v_max_free)
    if not holonomic:
        vy = 0.0
    else:
        vy = max(-gate.v_max_free, min(vy, gate.v_max_free))
    if wz_max_radps is not None:
        wz = max(-wz_max_radps, min(wz, wz_max_radps))
    return (vx, vy, wz)
