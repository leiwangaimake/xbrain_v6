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
  6  gate clip         host_gate.apply_gate (vx > 0 by f; vx < 0 / vy without f);
                       f carries the 12 S6.2 band hysteresis (BandHysteresis)
  6b rotation permit   rcg.apply_rotation_permit on the GATED velocity, only
                       when the tick is spin_like (12 S6A.4.1). The one gate in
                       V6 that constrains wz at all -- 12 S6A.1 shows the other
                       six layers are each a no-op on it. Veto or clamp by
                       source (12 S6A.4.2); never touches limiter (OB-1)
  7  fence clip        fence/clip.evaluate on the post-6b vector (12 S7)
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
the rns_avoid adapter), no path_progress (progress.py), no jerk limiter
(12 S2.2 step 8 -- not chained this phase). Step 7 fence clip IS chained since
2026-09-12 (fence/clip.py) and step 6b the rotation permit since 2026-09-29
(rotation/rcg.py); NEXT.md carries what is left.

About step 6b's data. The permit's ring read (RingSample) arrives on NavInputs.
It used to be None always -- 12 S6A.3.1 RC-D2 named rt/lidar/grid the one
primary source and the machine has no LiDAR -- and RC-D2 was corrected on
2026-09-29 to name the RNS memory grid, which runtime/nav_wiring assembles into
a sample. None still means "the ring cannot be answered at all", and the permit
still refuses on it; what changed is that None is no longer the standing state.

Three outcomes now reach the tick, not two (12 S6A.3.2): permitted leaves wz
alone, a hard refusal applies 12 S6A.4.2's per-source table, and a BLIND
refusal -- the ring is answerable but that direction was not observed -- clamps
wz to wz_blind_radps for every source. The blind branch is the standing state
on this machine: the RGBD covers the forward half, so the rear of the ring is
unobserved on every tick. What still refuses outright here is RCG-1, because
configs/rns.yaml carries no inflation.r_robot_m (12 S15 #10).

Trap: passing ctx.perception = None to RNS when the profile is merely OLD. RNS
ages the profile itself (T-50 slow / T-51 zero) and its memory grid needs the
last accepted frame; the host's FAILED veto sits on top of the candidate, it
does not withhold the snapshot.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Tuple

from xbrain.p1_motion.ctrl_loop import CtrlState
from xbrain.p1_motion.fence.clip import (
    CompiledFence,
    FenceConstants,
    FenceEval,
    evaluate,
    inset_for_fix,
    speed_cap_mps,
)
from xbrain.p1_motion.freshness.degradation import CAM_THRESH, Freshness, classify
from xbrain.p1_motion.gate.speed_gate import BandHysteresis, SpeedGateError
from xbrain.p1_motion.nav.health_factor import HealthView
from xbrain.p1_motion.nav.host_gate import (
    apply_gate,
    attribute,
    compute_gate,
    fence_attribution,
    forward_d_free,
)
from xbrain.p1_motion.rns.inputs import PerceptionSnapshot
from xbrain.p1_motion.rotation.rcg import (
    RingSample,
    RotationEval,
    RotationLimits,
    apply_rotation_permit,
)
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
    # wall-clock seconds for the report envelopes' ts (align / log ONLY, never
    # a judgement input -- CLK-C1); the wiring reads it once per tick.
    ts_wall_s: float = 0.0    # WALL-CLOCK-OK(align/log)
    fix_type: Optional[str] = None            # 11 S3.2.1 GnssFix.fix_type (fence inset)
    heading_source: Optional[str] = None      # 11 S3.3 source of a VALID heading (cog -> L2 speed)
    fence: Optional[CompiledFence] = None     # the active FenceSet, compiled (12 S7)
    # 12 S2.2 step 6b input: this tick's read of the sweep annulus. None means
    # the RC-D2 primary source produced nothing, which is a REFUSE and never a
    # skip -- and, since 2026-09-29, never the blind clamp either: "no data"
    # and "the data says I could not see" are different answers. It is a tick
    # input rather than a NavTick field because it changes every tick, exactly
    # like perception: the wiring assembles it, the tick only reads it.
    ring: Optional[RingSample] = None
    # 11 S3.5 P1-3 hold from P2 (99 U90 R-8 (a) + U99): B / D mode is on, so
    # the AUTONOMOUS motion stops -- the RNS source is suspended exactly like
    # estop / local teleop suspend it. Manual intervention (handset, HMI,
    # cloud teleop) is not a NavTick source and keeps working: the user ruled
    # "suppress autonomous, keep manual" (12 S4.2: manual outranks autonomy).
    mode_hold: bool = False
    # 99 U134: the teleop candidate this tick -- (vx, vy, wz) in body units,
    # already scaled by the voice-set speeds (teleop/velocity.py) -- and the
    # P1 source it drives as. None = no teleop input drives (or no speed is
    # known yet). Until 2026-11-05 teleop only suspended RNS and NOTHING drove
    # the robot under any teleop input.
    teleop_cmd: Optional[Tuple[float, float, float]] = None
    teleop_source: Optional[BehaviorSource] = None
    # 99 U150 (user ruling 2026-11-06): the cloud keyboard (keyboard_cloud,
    # the recording-time driving tool) drives UNCONDITIONALLY -- only the
    # estop and the chassis limits apply; positioning, heading, perception,
    # health, the fence and the rotation permit do not touch it. True only
    # while that device is the one driving (nav_wiring._teleop_drive).
    teleop_unconditional: bool = False
    # 99 U186: what the cloud keyboard is recording -- "route" | "fence" | None
    # (state/teach session.kind while a recording runs). A ROUTE recording is
    # stopped at the effective boundary of the activity area / a forbid zone
    # (user 2026-11-13: "录路径拦停, 没有减速段"); a fence recording is not.
    recording_kind: Optional[str] = None


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
    profile: str = "patrol"       # effective tier (11 S3.4 gate.profile)
    fence: Optional[FenceEval] = None   # 12 S2.2 step 7 result (state/fence + events)
    # 12 S2.2 step 6b result. Always present -- a non-spin tick reports
    # decision "pass" with spin_like False -- so the wiring never has to guess
    # whether the permit ran. limiter / limiter_all above are deliberately
    # untouched by it: 12 S6A.8 OB-1 forbids widening that closed set, since
    # every value in it describes a LINEAR cap and 11 S9.6.5's argmax over the
    # reduction Delta is undefined for wz. OB-2 routes this out as an event.
    rotation: Optional[RotationEval] = None
    # 99 U195: RNS's own zero reason this tick (no_way_through / contact_fuse)
    # or None -- logged by the wiring; the limiter above names free_space.
    rns_hold: Optional[str] = None


class RnsCtx:
    """The ctx RnsSource.compute reads (attribute names are the contract the
    SIL / batteries and tests/p1_motion/rns share)."""

    def __init__(self, inp: NavInputs, v_nom: float, wz_max: float,
                 holonomic: bool, fence: Optional[CompiledFence] = None,
                 fence_inset_m: Optional[float] = None) -> None:
        self.pose_xy = inp.pose_xy
        self.yaw_rad = inp.yaw_rad
        self.v_nom_mps = v_nom
        self.wz_max_rps = wz_max
        self.perception = inp.perception
        self.now_mono_ms = inp.now_mono_ms
        self.holonomic = holonomic
        # The compiled fence and this tick's inset, for the CANDIDATE-level
        # gate (clip.candidate_fence_hazard). Both default to None and RNS
        # reads them with getattr(..., None), so a ctx that does not carry
        # them -- the SIL bench builds its own -- simply has no fence gate
        # and behaves exactly as before (NEXT I-1).
        #
        # *** Why RNS gets the fence at all, when 12 S7 already enforces it
        # on the final velocity: without it RNS keeps proposing a subgoal
        # through the boundary, the clip removes the outward component, and
        # the robot sits on the fence (NEXT R-4b measured ~70 s of that).
        # Handing it the SAME geometry the clip uses is the whole point --
        # a second inside/outside notion would recreate the stall.
        self.fence = fence
        self.fence_inset_m = fence_inset_m


def _positive(name: str, v: Any) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0.0:
        raise NavTickConfigError("%s must be a positive number, got %r" % (name, v))
    return float(v)


#: 99 U142: the fixes that run like rtk_fixed but under the degraded-fix cap.
DEGRADED_FIXES = ("rtk_float", "dgps")


def quality_caps(fix_type: Optional[str], heading_source: Optional[str],
                 fix_cap_mps: float, cog_fixed_mps: float, cog_degraded_mps: float,
                 bridge_fixed_mps: float, float_cap_mps: float, cog_float_mps: float
                 ) -> Tuple[Optional[float], Optional[float]]:
    """(fix_cap_mps, cog_drive_mps) for compute_gate this tick (99 U142).

    The user's rulings of 2026-11-06, as one decision table:
      * heading_source "cog" (an L2 heading that is VALID -- L2-blind never
        reaches here, the pose drops its source) -> drive at a fixed speed:
        cog_fixed_mps (1.0) on rtk_fixed, cog_degraded_mps (0.6) on rtk_float
        / dgps. Both are above the COG validity threshold (0.5): the faster
        one gives COG a wide margin and a better heading (error ~ atan(dv /
        v)) where the position is good; on a float / dgps position the
        slowest speed that still keeps COG alive. The 0.5 fix cap does not
        apply on top of either -- it would hold the robot at the threshold;
      * rtk_float / dgps -> capped at fix_cap_mps, otherwise like rtk_fixed;
      * rtk_fixed with an L1 heading -> neither term: the ONLY case that runs
        at the navigation top speed (2.0 since 99 U153);
      * climb / bridge (99 U151 / U152) -> bridge_fixed_mps on rtk_fixed (99
        U178), cog_degraded_mps on float / dgps.
    99 U178 (user ruling 2026-11-13): an L2 (COG) heading on rtk_fixed runs
    at 2.0 like the dual antenna (cog_fixed_mps 1.0 -> 2.0), but the IMU
    bridge and the climb stay at 1.0 -- they are the rescue path while the
    GNSS heading is gone, and the odometry yaw they lean on has not been
    measured at 2.0. Until 2026-11-13 they shared cog_fixed_mps, so raising
    COG would have raised them too; bridge_fixed_mps is their own value.
    99 U180 (user ruling 2026-11-13): rtk_float got its own pair -- the
    position cap float_cap_mps and the COG speed cog_float_mps, both 1.0 --
    while dgps keeps fix_cap_mps / cog_degraded_mps (0.5 / 0.6): the COG
    speed bypasses the position cap, so raising only the COG speed would
    have run a COG heading on float at 1.0 while the better dual-antenna
    heading on the same fix stayed at 0.5. dgps stays slow because its
    ~1.5 m position error eats the fence inset. On float the climb / bridge
    take cog_float_mps but never more than bridge_fixed_mps (the rescue
    path is not faster on a worse fix than on rtk_fixed).
    single / no_fix never get here as a moving case: their i_fix is absent and
    the gate vetoes on rtk (11 S3.2.1, unchanged).
    """
    if heading_source == "cog":  # L2: a fixed COG speed, by fix quality
        if fix_type == "rtk_float":
            return None, cog_float_mps  # 99 U180
        return None, (cog_degraded_mps if fix_type in DEGRADED_FIXES else cog_fixed_mps)
    if heading_source in ("climb", "bridge"):
        # 99 U151: the heading-deadlock climb runs at the navigation top speed
        # (no extra term on rtk_fixed); on float / dgps the 0.5 fix cap would
        # pin it AT the COG threshold, so it gets the L2 degraded COG speed.
        # 99 U152: the IMU-bridged heading is driven the same way -- it is not
        # an L2 heading (no fixed COG speed), and a slow bridge would keep COG
        # from coming back.
        # 99 U153 (user ruling 2026-11-07): the navigation top speed went to
        # 2.0 for the dual antenna on rtk_fixed ONLY, so on rtk_fixed a climb /
        # bridge no longer takes the top speed: it is held at the COG fixed
        # speed (1.0), which still keeps COG alive for the hand-back.
        # 99 U178: its own value (bridge_fixed_mps, 1.0) now that COG runs at
        # 2.0 -- ~~cog_fixed_mps~~ shared until 2026-11-13.
        if fix_type == "rtk_float":
            return None, min(cog_float_mps, bridge_fixed_mps)  # 99 U180
        return None, (cog_degraded_mps if fix_type in DEGRADED_FIXES else bridge_fixed_mps)
    if fix_type == "rtk_float":
        return float_cap_mps, None  # 99 U180: float has its own cap
    if fix_type in DEGRADED_FIXES:
        return fix_cap_mps, None  # dgps: capped
    return None, None  # rtk_fixed + L1: nothing added


class NavTick:
    """Per-tick orchestrator. One instance per process, ticked at 20 Hz."""

    def __init__(self, source: RnsAvoidSource, arbiter: P1Arbiter, *,
                 v_nom_mps: Any, wz_max_rps: Any, spec_max_vx_mps: Any,
                 holonomic: bool, speed_up_hold_ms: Any, d_up_margin_m: Any,
                 rot_limits: RotationLimits, r_robot_m: Optional[float],
                 v_obstacle_avoid_mps: Any = None,
                 fence_consts: Optional[FenceConstants] = None,
                 degraded_fix_cap_mps: Any, cog_drive_fixed_mps: Any,
                 cog_drive_degraded_mps: Any, bridge_drive_fixed_mps: Any,
                 float_fix_cap_mps: Any, cog_drive_float_mps: Any) -> None:
        self._src = source
        self._arb = arbiter
        # 12 S2.2 step 6b constants. REQUIRED, with no None-means-skip shape --
        # unlike fence_consts above, which tolerates None for the unit-test
        # spine. 12 S6A.7 RC-D7 refuses to give the rotation permit any off
        # switch, reasoning that one enabled:false is a remote channel for
        # releasing rotation protection; a constructor that silently ran
        # without the permit when the block was absent would be that switch
        # spelled differently, so a missing block is a construction failure.
        if not isinstance(rot_limits, RotationLimits):
            raise NavTickConfigError(
                "rot_limits must be a RotationLimits (12 S12 rotation_clearance); "
                "there is no no-permit mode (12 S6A.7 RC-D7), got %r" % (rot_limits,))
        self._rot = rot_limits
        # The TRUE r_robot, and None when it is not configured anywhere. 12 S12
        # keeps its single definition in the RNS geometry section and forbids a
        # P1-private second copy, so this arrives from nav_cfg rather than from
        # a key of our own. None and 0.0 mean the same thing to RCG-1 -- "not
        # known, so do not turn" -- and neither is ever replaced by the
        # trigger-side fallback (12 S6A.4.1 iron rule 1).
        self._r_robot = r_robot_m
        # 12 S6.2 / S6.7 band hysteresis on the f term (U54: rise to 2.0 needs
        # 3.5 m for 3 s, drop is instant). Both numbers come from the resolved
        # snapshot (speed_gate.hysteresis.*); a null leaf refuses here.
        try:
            self._hyst = BandHysteresis(speed_up_hold_ms, d_up_margin_m)
        except SpeedGateError as exc:
            raise NavTickConfigError(str(exc)) from exc
        # 12 S2.2 step 7 runs only when the resolved fence constants exist
        # (nav_cfg builds them, refusing on any null leaf); None is the
        # unit-test stack, never a production shape (main_wiring always
        # passes cfg.fence).
        self._fence = fence_consts
        self._v_nom = _positive("v_nom_mps", v_nom_mps)
        # 11 S3.6 max_profile == obstacle_avoid caps the nominal at that tier's
        # max_mps (U54: 0.5). None = the caller has no tier table (tests of the
        # spine); then a downgrade request is honoured by the h factor alone.
        self._v_oa = (None if v_obstacle_avoid_mps is None
                      else _positive("v_obstacle_avoid_mps", v_obstacle_avoid_mps))
        self._wz_max = _positive("spec.max_wz_radps", wz_max_rps)
        self._spec_vx = _positive("spec.max_vx_mps", spec_max_vx_mps)
        # 99 U142: REQUIRED, no default -- a missing cap would let a float /
        # dgps fix run at full speed (CLAUDE.md 3.1); nav_cfg reads both keys.
        self._fix_cap = _positive("nav.degraded_fix_cap_mps", degraded_fix_cap_mps)
        self._cog_fixed = _positive("nav.cog_drive_fixed_mps", cog_drive_fixed_mps)
        self._cog_degraded = _positive("nav.cog_drive_degraded_mps", cog_drive_degraded_mps)
        # 99 U178: the climb / IMU-bridge speed on rtk_fixed, required like the rest
        self._bridge_fixed = _positive("nav.bridge_drive_fixed_mps", bridge_drive_fixed_mps)
        # 99 U180: the rtk_float pair; the two degraded_* values above are dgps only now
        self._float_cap = _positive("nav.float_fix_cap_mps", float_fix_cap_mps)
        self._cog_float = _positive("nav.cog_drive_float_mps", cog_drive_float_mps)
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
        """estop, teleop or a P2 mode hold -> preempt once; all clear ->
        release once (the mission is kept, as for estop / teleop).
        mutant: call on_release every tick instead of on the edge -> RNS
        re-enters FOLLOW while estop is still latched -> test_estop_suspends
        red (the candidate reappears under estop)."""
        want = bool(inp.estop or inp.teleop_active or inp.mode_hold)
        if want and not self._suspended:
            self._src.on_preempted(ctx)
            self._suspended = True
        elif not want and self._suspended:
            self._src.on_release(ctx)
            self._suspended = False

    def _unconditional_teleop(self, inp: NavInputs, raw: Tuple[float, float, float],
                              source: str, fresh: Any) -> NavOutput:
        """99 U150: see _unconditional_teleop_impl."""
        return _unconditional_teleop_impl(self, inp, raw, source, fresh)

    def run(self, inp: NavInputs) -> NavOutput:
        """One tick (12 S2.2). Always returns an output; zero when nothing may
        drive. The RNS candidate is computed only when rns_avoid holds the
        slot, so a suspended / mission-less source is never asked."""
        now = inp.now_mono_ms
        # 11 S3.6 max_profile: obstacle_avoid tier requested -> the mission
        # nominal (and the RNS ctx nominal) drop to that tier's max_mps.
        v_nom = self._v_nom
        downgraded = False
        if inp.health.max_profile == "obstacle_avoid" and self._v_oa is not None \
                and self._v_oa < v_nom:
            v_nom = self._v_oa
            downgraded = True
        # ONE inset_for_fix call site per tick, read by two consumers: the
        # candidate gate (through the ctx, below) and the zone speed cap
        # (before compute_gate). The ctx carries the INSET rather than the
        # fix_type so RNS never has to know the 11 S3.2.1 table -- one owner
        # for that mapping, which is also why this is not computed twice.
        # None when the process has no fence constants at all (the unit-test
        # spine), which makes both consumers inert.
        fence_inset = (inset_for_fix(inp.fix_type, self._fence)
                       if self._fence is not None else None)
        # *** `if self._fence is not None` 在 fence= 这一侧是[等价变异点],
        # 2026-10-08 实测确认: 去掉它直接传 inp.fence, 没有任何测试会红, 而且
        # 行为真的不变 -- 没有围栏常量时 fence_inset 已经是 None, 而
        # candidate_fence_hazard 对 inset 为 None 一律返回 False. 它是第二道
        # 带子(两个字段必须同生同死), 所以单独改一侧杀不掉.
        # NO 不为它硬写一条断言(7.2.1: 等价变异在代码里注明, 不硬写判据) --
        # 真正该守的是"两者同生同死", 那由上面 fence_inset 的同一个条件保证.
        ctx = RnsCtx(inp, v_nom, self._wz_max, self._holo,
                     fence=inp.fence if self._fence is not None else None,
                     fence_inset_m=fence_inset)
        fresh = self._freshness(inp.perception, now)
        self._suspension_edge(inp, ctx)
        # arbitration (12 S5.1 A-1): note the sources that want the slot, hold
        # always; deactivation hysteresis is the arbiter's dwell.
        if self._src.is_active(ctx):
            self._arb.note(BehaviorSource.RNS_AVOID, now)
        if inp.teleop_cmd is not None and inp.teleop_source is not None:
            # 99 U134: the teleop source wants the slot (RNS is suspended by
            # the same input, TR-RNS-1, so it is not competing this tick)
            self._arb.note(inp.teleop_source, now)
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
        elif holder in _TELEOP_SOURCES:
            # 99 U134: the operator's keys, through the SAME gate / rotation
            # permit / fence below (11 S12A.9.2). A holder kept only by the
            # arbiter's dwell after the input stopped drives zero.
            if inp.teleop_cmd is not None and inp.teleop_source is holder:
                raw = tuple(float(v) for v in inp.teleop_cmd)  # type: ignore[assignment]
            source = holder.value
            if inp.teleop_unconditional and inp.teleop_source is holder:
                return self._unconditional_teleop(inp, raw, source, fresh)
        profile = inp.perception.profile if inp.perception is not None else None
        f_free = forward_d_free(profile.d_free) if profile is not None else None
        # the f term with its rise hysteresis (12 S6.2 table): the clearance
        # feeds the band memory every tick, the gate takes the band's speed.
        f_mps = self._hyst.update(f_free, now)
        # 11 S9A.2 block B-2 sends the in-zone ceiling "直接进 S9.6.1 的
        # min()", and S9.6.1 IS this gate -- so the zone cap is computed HERE,
        # before step 6, not inside the step-7 clip. Two consequences worth
        # stating because both are deliberate:
        #   * it applies on a heading_lost tick. Being inside a polygon is a
        #     question about POSITION; the clip degrades to warn_only without
        #     heading only because the body->ENU rotation it needs is
        #     undefined, and a scalar ceiling needs no rotation.
        #   * it is None whenever there is no fence, no speed_limit zone, no
        #     position, or no inset -- so a system with no fence data gets
        #     exactly the pre-zone gate (NEXT I-1: no fence constrains
        #     nothing). speed_cap_mps is where each of those is decided.
        zone_cap = speed_cap_mps(inp.fence, inp.pose_xy, fence_inset)
        fix_cap, cog_drive = quality_caps(inp.fix_type, inp.heading_source, self._fix_cap,
                                          self._cog_fixed, self._cog_degraded,
                                          self._bridge_fixed, self._float_cap,
                                          self._cog_float)  # 99 U142 / U178 / U180
        gate = compute_gate(
            v_nom_mps=v_nom, spec_max_vx_mps=self._spec_vx,
            free_space_mps=f_mps, health=inp.health, i_fix=inp.i_fix,
            i_heading=inp.i_heading, heading_valid=inp.heading_valid,
            estop=inp.estop, perception_dead=(fresh is Freshness.FAILED),
            profile_downgraded=downgraded, fence_zone_mps=zone_cap,
            fix_cap_mps=fix_cap, cog_drive_mps=cog_drive)
        vx, vy, wz = apply_gate(gate, raw[0], raw[1], raw[2], self._holo,
                                wz_max_radps=self._wz_max)
        limiter, limiter_all = attribute(gate, raw[0])
        # 99 U195 (user 2026-11-13): when RNS itself zeroed the candidate (no
        # way through / contact fuse) the gate saw raw 0 and attributed
        # nothing, so a 4.5 min stall read "follow, limiter none". It is the
        # blocked view in front: free_space, an existing limiter value.
        rns_hold = (self._src.hold_reason()
                    if source == BehaviorSource.RNS_AVOID.value else None)
        if rns_hold is not None and limiter == "none":
            limiter, limiter_all = "free_space", ("free_space",)
        # 12 S2.2 step 6b, between the speed gate and the fence exactly as the
        # step list orders it. It reads the GATED vx / vy: at this point in the
        # chain a tick whose vx the gate has already zeroed IS a spin, and
        # judging the pre-gate candidate instead would wave that case through.
        # The winning source name is passed verbatim -- 12 S6A.4.2's disposal
        # differs per source and its rns_avoid row demands the test be written
        # out rather than defaulted.
        rot = apply_rotation_permit(
            vx_mps=vx, vy_mps=vy, wz_radps=wz, source=source,
            limits=self._rot, r_robot_m=self._r_robot, ring=inp.ring)
        wz = rot.wz_out
        v_max = gate.v_max_fwd
        fev: Optional[FenceEval] = None
        if self._fence is not None:
            # 12 S2.2 step 7 / 12 S7: the fence acts on the GATED candidate
            # (after step 6), with the gate's h * i on the soft cap (11 S9A.6
            # (5) multiplies the fence term by them too). It runs on a veto
            # tick as well -- the vector is zero then, but state/fence still
            # wants this tick's geometry (inside / soft / outside).
            fev = evaluate(inp.fence, self._fence, xy=inp.pose_xy,
                           yaw_rad=inp.yaw_rad, vx=vx, vy=vy,
                           fix_type=inp.fix_type, heading_valid=inp.heading_valid,
                           hi_factor=gate.h_factor * gate.i_factor)
            vx, vy = fev.vx, fev.vy
            if fev.clipped:
                limiter, limiter_all = fence_attribution(limiter, limiter_all,
                                                         gate, fev.cut_mps)
            # 11 S3.4 gate.v_max: the scalar form of the fence term when the
            # candidate approaches the boundary (the collinear case of the
            # half-space cap, see clip.py).
            if fev.outward and fev.v_fence_mps is not None and not gate.veto:
                v_max = min(v_max, fev.v_fence_mps * gate.h_factor * gate.i_factor)
        return NavOutput(
            vx=vx, vy=vy, wz=wz, raw_vx=raw[0], raw_vy=raw[1], raw_wz=raw[2],
            v_max=v_max, limiter=limiter, limiter_all=limiter_all,
            source=source, h_factor=gate.h_factor, i_factor=gate.i_factor,
            freshness=fresh.value, suspended=self._suspended,
            nav_state=self._src.nav_state().value,
            profile="obstacle_avoid" if downgraded else "patrol",
            fence=fev, rotation=rot, rns_hold=rns_hold)


def _clamp(v: float, lim: float) -> float:
    """v limited to +/- lim."""
    return max(-lim, min(v, lim))


def _unconditional_teleop_impl(tick: "NavTick", inp: "NavInputs", raw: Tuple[float, float, float],
                               source: str, fresh: Any) -> "NavOutput":
    """99 U150: the cloud keyboard's velocity -- estop zeroes it, the chassis
    limits (spec max vx / max wz, vy only on a holonomic body) bound it, and
    nothing else is applied.

    *** Why: the user ruled (2026-11-06) that manual keyboard control, used
    while recording routes and fences, is the operator's alone. Through the
    shared gate a dual-antenna heading dropout during an in-place turn (COG
    is undefined at zero speed) vetoed the whole command and the 2 s recover
    hold kept it at zero: turns stuttered and "lagged" while forward, back
    and strafe (which keep COG) were fine (bench 2026-11-06: wz zeroed on
    23 % / 65 % of the ticks of a left / right turn, limiter heading).
    The fence is still EVALUATED (state/fence shows inside / outside) but its
    clip is not applied; no rotation verdict is made.
    """
    if inp.estop:
        vx = vy = wz = 0.0  # the estop always wins
        limiter = "estop"
    else:
        vx = _clamp(raw[0], tick._spec_vx)  # chassis forward / back limit
        vy = _clamp(raw[1], tick._spec_vx) if tick._holo else 0.0  # strafe only if holonomic
        wz = _clamp(raw[2], tick._wz_max)  # chassis turn limit
        limiter = "none"
    fev = None
    if tick._fence is not None:  # geometry for state/fence; the clip is NOT applied
        fev = evaluate(inp.fence, tick._fence, xy=inp.pose_xy, yaw_rad=inp.yaw_rad,
                       vx=vx, vy=vy, fix_type=inp.fix_type,
                       heading_valid=inp.heading_valid, hi_factor=1.0)
        if (inp.recording_kind == "route" and not inp.estop and fev is not None
                and any(p.hard_now for p in fev.per_poly)):  # the estop's label wins
            # 99 U186: a route recording stops AT the boundary -- the clip's
            # result is taken only once a boundary's zero cap is active (at
            # or past the effective line, now or predicted), so there is no
            # slow-down band before it; inside the area the keyboard stays
            # unconditional (U150). The clip keeps the inward / sliding part,
            # so the operator can drive back (12 S7.2).
            vx, vy = fev.vx, fev.vy
            limiter = "fence"
    return NavOutput(
        vx=vx, vy=vy, wz=wz, raw_vx=raw[0], raw_vy=raw[1], raw_wz=raw[2],
        v_max=tick._spec_vx, limiter=limiter, limiter_all=[limiter] if limiter != "none" else [],
        source=source, h_factor=1.0, i_factor=1.0, freshness=fresh.value,
        suspended=tick._suspended, nav_state=tick._src.nav_state().value,
        profile="patrol", fence=fev, rotation=None)


#: 99 U134: the teleop sources a NavTick can hand the slot to.
_TELEOP_SOURCES = frozenset({BehaviorSource.TELEOP_KEYBOARD,
                             BehaviorSource.TELEOP_JOYSTICK})


def ctrl_state_for(inp: NavInputs, out: NavOutput, *, health_ever_ok: bool,
                   inputs_ever_ready: bool) -> CtrlState:
    """The 12 S11 P1 state word for this tick, derived from the same facts the
    gate used -- the CtrlLoop then zeroes every non-ACTIVE state itself, a
    second belt under the NavTick vetoes:
      WAIT_GRANT  P2's factor never granted yet (11 S3.6 'never')
      WAIT_INPUT  granted, but pose / heading / perception not yet ready
      SAFE_STOP   was ready once, now allow_motion false or an input dead
                  (12 S11: '任何状态 -> SAFE_STOP', back to READY on recovery)
      READY       granted + inputs ok, no behaviour source active (hold)
      ACTIVE      rns_avoid or a teleop source (99 U134) holds the slot
    mutant: report ACTIVE while the health veto is on -> the state word says
    driving for a zero-velocity health stop -> test_ctrl_state_words red."""
    if inp.teleop_unconditional and out.source in {s.value for s in _TELEOP_SOURCES}:
        # 99 U150: the cloud keyboard drives whatever the inputs say; the
        # estop still zeroes it (NavTick) and the CtrlLoop zeroes on estop.
        return CtrlState.ACTIVE
    inputs_ok = (inp.pose_xy is not None and inp.heading_valid
                 and out.freshness != Freshness.FAILED.value
                 and inp.i_fix is not None and inp.i_fix > 0.0)
    if not inp.health.allow_motion:
        return CtrlState.SAFE_STOP if health_ever_ok else CtrlState.WAIT_GRANT
    if not inputs_ok:
        return CtrlState.SAFE_STOP if inputs_ever_ready else CtrlState.WAIT_INPUT
    if out.source == BehaviorSource.RNS_AVOID.value:
        return CtrlState.ACTIVE
    if out.source in {s.value for s in _TELEOP_SOURCES}:
        # 99 U134: a teleop holder drives too -- READY would have the CtrlLoop
        # zero every teleop velocity (12 S11: only ACTIVE passes through)
        return CtrlState.ACTIVE
    return CtrlState.READY
