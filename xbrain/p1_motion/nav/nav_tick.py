"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: nav_tick.py
Brief: the 20 Hz navigation tick -- RNS candidate through the P1 arbiter and host gate (12 S2.2)

Description:
One call per tick, pure (no Zenoh, no clock reads): NavInputs (the tick snapshot
the wiring assembled from its latest-value slots, 12 S2.2 step 1 / RTC-6) in,
NavOutput (the cmd_vel body fields plus attribution) out. Steps, in 12 S2.2
order:
  1  snapshot          NavInputs IS the snapshot; nothing here re-reads a slot
  2  freshness         profile age -> Freshness via CAM_THRESH (12 S3.3);
                       no profile ever -> FAILED
  3  speed gate        host_gate.compute_gate (f, h, i, spec, profile)
  4  behaviour sources rns_avoid.compute(ctx) when it holds the slot
  5  arbitration       P1Arbiter: rns_avoid (900) vs hold (100); TR-RNS-1 and
                       estop are SUSPENSION edges on the source, not priorities
  6  gate clip         host_gate.apply_gate (vx > 0 by f; vx < 0 / vy without f)
  9  output            NavOutput; the wiring serialises 11 S3.4 and publishes

Suspension (20 RNS-M-7 / 12 S4.2c.5 TR-RNS-1): estop latched OR a local teleop
input active -> on_preempted once (edge), mission kept, is_active False so hold
wins and the output is zero; both clear -> on_release once -> RNS re-evaluates
from FOLLOW (never replays the pre-suspend intent). The edge is tracked HERE so
the wiring cannot call on_preempted twenty times a second.

Why hold is always noted: 12 S4.2 "hold 永远存在且永远活跃" -- with no mission
the holder is hold and the tick still emits a (zero) cmd_vel, which is what
chassis Tier 1 needs to stay out of timeout_lock.

What it does NOT do: no publishing, no mission entry (route / relmove intake +
the rns_avoid adapter), no path_progress (progress.py), no rotation permit /
fence clip / jerk limiter (12 S2.2 6b / 7 / 8 -- not chained this phase,
NEXT.md).

Trap: passing ctx.perception = None to RNS when the profile is merely OLD. RNS
ages the profile itself (T-50 slow / T-51 zero) and its memory grid needs the
last accepted frame; the host's FAILED veto sits on top of the candidate, it
does not withhold the snapshot.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Tuple

from xbrain.p1_motion.freshness.degradation import CAM_THRESH, Freshness, classify
from xbrain.p1_motion.nav.health_factor import HealthView
from xbrain.p1_motion.nav.host_gate import (apply_gate, attribute, compute_gate,
                                            forward_d_free)
from xbrain.p1_motion.rns.inputs import PerceptionSnapshot
from xbrain.p1_motion.sources.arbiter_p1 import BehaviorSource, P1Arbiter
from xbrain.p1_motion.sources.rns_avoid import RnsAvoidSource


class NavTickConfigError(ValueError):
    """A loop parameter missing or non-positive (CLAUDE.md 3.1: refuse)."""


@dataclass(frozen=True)
class NavInputs:
    """The tick snapshot. i_fix None = no usable fix; i_heading None = no
    heading; perception is the latest-wins view (profile None = never)."""
    now_mono_ms: int
    pose_xy: Optional[Tuple[float, float]]
    yaw_rad: Optional[float]
    heading_valid: bool
    i_fix: Optional[float]
    i_heading: Optional[float]
    perception: PerceptionSnapshot
    health: HealthView
    estop: bool
    teleop_active: bool


@dataclass(frozen=True)
class NavOutput:
    """What goes on the wire (11 S3.4 body + gate block) and why."""
    vx: float
    vy: float
    wz: float
    raw_vx: float
    raw_vy: float
    raw_wz: float
    v_max: float
    limiter: str
    limiter_all: Tuple[str, ...]
    source: str
    h_factor: float
    i_factor: float
    freshness: str
    suspended: bool
    nav_state: str


class RnsCtx:
    """The ctx RnsSource.compute reads (attribute names are the contract the
    SIL / batteries and tests/p1_motion/rns share)."""

    def __init__(self, inp: NavInputs, v_nom: float, wz_max: float,
                 holonomic: bool) -> None:
        self.pose_xy = inp.pose_xy
        self.yaw_rad = inp.yaw_rad
        self.v_nom_mps = v_nom
        self.wz_max_rps = wz_max
        self.perception = inp.perception
        self.now_mono_ms = inp.now_mono_ms
        self.holonomic = holonomic


def _positive(name: str, v: Any) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0.0:
        raise NavTickConfigError("%s must be a positive number, got %r" % (name, v))
    return float(v)


class NavTick:
    """Per-tick orchestrator. One instance per process, ticked at 20 Hz."""

    def __init__(self, source: RnsAvoidSource, arbiter: P1Arbiter, *,
                 v_nom_mps: Any, wz_max_rps: Any, spec_max_vx_mps: Any,
                 holonomic: bool) -> None:
        self._src = source
        self._arb = arbiter
        self._v_nom = _positive("v_nom_mps", v_nom_mps)
        self._wz_max = _positive("spec.max_wz_radps", wz_max_rps)
        self._spec_vx = _positive("spec.max_vx_mps", spec_max_vx_mps)
        self._holo = bool(holonomic)
        self._suspended = False

    @property
    def suspended(self) -> bool:
        return self._suspended

    def _freshness(self, snap: PerceptionSnapshot, now: int) -> Freshness:
        """12 S3.3 perception row on the profile's own capture time (11 S3.1B
        TIME-2): never received == FAILED."""
        if snap is None or snap.profile is None:
            return Freshness.FAILED
        return classify(now - snap.profile.t_capture_mono_ms, CAM_THRESH)

    def _suspension_edge(self, inp: NavInputs, ctx: RnsCtx) -> None:
        """estop or teleop -> preempt once; both clear -> release once.
        mutant: call on_release every tick instead of on the edge -> RNS
        re-enters FOLLOW while estop is still latched -> test_estop_suspends
        red (the candidate reappears under estop)."""
        want = bool(inp.estop or inp.teleop_active)
        if want and not self._suspended:
            self._src.on_preempted(ctx)
            self._suspended = True
        elif not want and self._suspended:
            self._src.on_release(ctx)
            self._suspended = False

    def run(self, inp: NavInputs) -> NavOutput:
        """One tick (12 S2.2). Always returns an output; zero when nothing may
        drive. The RNS candidate is computed only when rns_avoid holds the
        slot, so a suspended / mission-less source is never asked."""
        now = inp.now_mono_ms
        ctx = RnsCtx(inp, self._v_nom, self._wz_max, self._holo)
        fresh = self._freshness(inp.perception, now)
        self._suspension_edge(inp, ctx)
        # arbitration (12 S5.1 A-1): note the sources that want the slot, hold
        # always; deactivation hysteresis is the arbiter's dwell.
        if self._src.is_active(ctx):
            self._arb.note(BehaviorSource.RNS_AVOID, now)
        self._arb.note(BehaviorSource.HOLD, now)
        self._arb.tick(now)
        holder = self._arb.holder()
        raw = (0.0, 0.0, 0.0)
        source = BehaviorSource.HOLD.value
        if holder is BehaviorSource.RNS_AVOID:
            cand = self._src.compute(ctx)
            if cand is not None:
                # Mps is a unit type, not a float subclass (CFG-CM-18):
                # read .value; float(Mps) is a TypeError by design.
                raw = (float(cand.vx.value), float(cand.vy.value), float(cand.wz))
                source = BehaviorSource.RNS_AVOID.value
        profile = inp.perception.profile if inp.perception is not None else None
        f_free = forward_d_free(profile.d_free) if profile is not None else None
        gate = compute_gate(
            v_nom_mps=self._v_nom, spec_max_vx_mps=self._spec_vx,
            f_free_mps=f_free, health=inp.health, i_fix=inp.i_fix,
            i_heading=inp.i_heading, heading_valid=inp.heading_valid,
            estop=inp.estop, perception_dead=(fresh is Freshness.FAILED))
        vx, vy, wz = apply_gate(gate, raw[0], raw[1], raw[2], self._holo)
        limiter, limiter_all = attribute(gate, raw[0])
        return NavOutput(
            vx=vx, vy=vy, wz=wz, raw_vx=raw[0], raw_vy=raw[1], raw_wz=raw[2],
            v_max=gate.v_max_fwd, limiter=limiter, limiter_all=limiter_all,
            source=source, h_factor=gate.h_factor, i_factor=gate.i_factor,
            freshness=fresh.value, suspended=self._suspended,
            nav_state=self._src.nav_state().value)
