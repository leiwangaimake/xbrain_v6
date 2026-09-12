#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: perception_sim.py
Brief: W-11 -- the three perception keys' simulated message set (19 S15 W-11)

Description:
The interface-stage counterpart the RNS side owes the perception implementer
(perception-rns-reply-20260911 Q1.2): seven scenario sequences of the three
keys xbrain/{rid}/rt/perception/{profile,objects,status} in the exact wire
form (11 S3.0 envelope around the 11 S3.1B body), each with the RNS verdict
the consumer-contract test pins (tests/perception/test_consumer_contract.py).

Modes:
  --write [DIR]   regenerate tests/perception/samples/<scenario>/sequence.json
                  (deterministic; the test asserts disk == generator so nobody
                  hand-edits a sample into something the contract never said)
  --publish       publish one scenario (or all, looped) on the RT plane so the
                  producer / receiver tooling (perception_rx_audit.py) can be
                  exercised without a camera. Body timestamps are REBASED to
                  this host's CLOCK_MONOTONIC so RNS ageing is meaningful.
  (no mode)       print the scenario index (name -> expected verdict).

Scenarios (11 S3.1B / 20 S3.1 semantics the verdict comes from):
  normal          open ground + a box at 3 m, ~20 deg left; a static cone to the
                  right -> RNS drives at nominal speed, no failure
  all_unknown     every bin d_free/d_block null (strong reflection), invalid
                  ratio 0.85 -> UNKNOWN cap (S8.1A) + RNS-I-2 cap: slow
  no_seg          t_seg null, bit0 clear everywhere -> geometry-only FREE ->
                  no_seg_speed_cap (S3.1.11)
  extrinsic_uncal all three messages extrinsic_calibrated=false ->
                  extrinsic_uncalibrated failure, mission refused (11 S3.1B.4)
  tf_stale        TF unavailable: a 'static' car 1 m ahead with
                  velocity_frame=raw for 2.25 s -> raw refused (S3.1.5) ->
                  never judged static -> WAIT (a raw-trusting consumer drives)
  clock_reset     third tick's timestamps sit > 1 s BEHIND the earlier ones
                  (machine restart) -> epoch reset audited, memory cleared,
                  frame accepted (11 S3.1B.5 v2.1)
  dropout         objects 700 ms old while profile/status are fresh -> T-52
                  channel lost: objects_lost audited, speed capped
  semantic_only   centre bins have NO depth (all null) but a semantic
                  footprint 2.5 m ahead was injected: d_block 2.5, src 0b0100
                  (11 S3.1B.1 v2.2) -> BLOCKED at 2.5 m, UNKNOWN before it,
                  never FREE

What this is NOT: a physics simulator (that is scripts/sil), and not a
producer-side reference implementation -- the bodies are hand-built from the
contract's examples, so a field the contract does not define is absent here.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

ROOT = Path(__file__).resolve().parents[2]
SAMPLES_DIR = ROOT / "tests" / "perception" / "samples"

# 11 S3.1B.1 example sector: +-45 deg, 0.5 deg, 181 bins (the producer will
# replace these with the measured FOV -- Q-2 -- the shape is what matters here)
N_BINS = 181
ANGLE_MIN = -0.785398
ANGLE_STEP = 0.008727
RANGE_MAX = 6.0
BLIND_NEAR = 0.59
Z_PASS = 0.75
T0 = 100000                 # monotonic ms base of the static samples
RT_ENDPOINT = "tcp/127.0.0.1:7449"
SCENARIOS = ("normal", "all_unknown", "no_seg", "extrinsic_uncal",
             "tf_stale", "clock_reset", "dropout", "semantic_only",
             "ground_withdrawn")
# src bits (11 S3.1B.1 v2.1): bit0 T, bit1 G, bit2 S, bit3 NEG
SRC_T = 1
SRC_G = 2
SRC_S = 4


# ── body builders (11 S3.1B.1 / .2 / .3) ─────────────────────────────────────
def profile_body(now: int, *, kind: str = "open", extrinsic: bool = True,
                 pose_used: Optional[dict] = None) -> Dict[str, Any]:
    """One ProfileMsg body. kind: open (box at 3 m in bins 130..134, ~+20 deg),
    unknown (every bin null: nothing observed), noseg (open, no T evidence),
    semantic_only (open, but bins 85..95 have no depth and a 2.5 m S block),
    ground_withdrawn (19 S3.2A v1.6: fit failed with no error basis -> every
    bin d_free/d_block/h_block null, geometry HAS samples (bit1) but backs
    nothing, t_seg present yet bit0 = 0; one independent S block survives in
    bins 85..95 at 2.5 m)."""
    seg = kind != "noseg"
    d_free: List[Optional[float]] = []
    d_block: List[Optional[float]] = []
    h_block: List[Optional[float]] = []
    src: List[int] = []
    conf: List[int] = []
    for i in range(N_BINS):
        if kind == "unknown":
            # RNS-I-1: unobserved is null, never 0 nor range_max; src 0 says
            # geometry saw NOTHING in this bin (11 S3.1B.1 v2.1 bit1)
            d_free.append(None); d_block.append(None); h_block.append(None)
            src.append(0); conf.append(0)
            continue
        if kind == "ground_withdrawn":
            # 19 S3.2A v1.6 withdrawal: samples exist (bit1) but back no FREE
            # (bit0 = 0) and no plane-dependent block; the S block is the one
            # piece of evidence that does not depend on the failed plane.
            d_free.append(None); h_block.append(None)
            if 85 <= i <= 95:
                d_block.append(2.5); src.append(SRC_G | SRC_S)
            else:
                d_block.append(None); src.append(SRC_G)
            conf.append(0)
            continue
        if kind == "semantic_only" and 85 <= i <= 95:
            # 11 S3.1B.1 v2.2 (Q3 ruling): no geometry at all in this bin,
            # but a semantic footprint injected a 2.5 m block: S only gives
            # BLOCKED, never FREE -> d_free stays null, h_block null, bit1 0
            d_free.append(None); d_block.append(2.5); h_block.append(None)
            src.append(SRC_S); conf.append(0)
            continue
        if 130 <= i <= 134:
            # a 0.4 m box at 3.0 m: far-cell-end stop puts d_free at 2.75
            d_free.append(2.75); d_block.append(3.0); h_block.append(0.4)
        else:
            d_free.append(RANGE_MAX); d_block.append(None); h_block.append(None)
        src.append((SRC_T if seg else 0) | SRC_G)
        conf.append(220)
    return {
        "schema": "perception_profile_v1",
        "t_capture_mono_ms": now - 20,          # exposure midpoint (TIME-1)
        "t_publish_mono_ms": now - 5,
        "frame": "base_link",
        "extrinsic_calibrated": extrinsic,
        "pose_used": pose_used,
        "t_seg_mono_ms": (now - 30) if seg else None,   # TIME-3 / S3.1.11
        "z_pass_m": Z_PASS,
        "angle_min_rad": ANGLE_MIN, "angle_step_rad": ANGLE_STEP,
        "n_bins": N_BINS, "range_max_m": RANGE_MAX, "blind_near_m": BLIND_NEAR,
        "d_free": d_free, "d_block": d_block, "h_block": h_block,
        "src": src, "conf": conf,
        "terrain": [0] * N_BINS,                # v1: unknown (19 PD-13)
        "slope_deg": [None] * N_BINS,           # v1: null is legal (11 v2.1)
    }


def obj(track_id: int, class_name: str, cx: float, cy: float, *,
        r_near: float, velocity_xy=(0.0, 0.0), velocity_frame: str = "ego_removed",
        velocity_status: str = "static", confidence: float = 0.92,
        semantic_status: str = "confirmed", now: int = T0) -> Dict[str, Any]:
    """One tracked object: a small triangular ground hull around (cx, cy) in
    base_link (11 S3.1B.2: 3..12 CCW points, heights as an interval)."""
    hull = [[cx - 0.3, cy - 0.3], [cx + 0.3, cy - 0.3], [cx, cy + 0.3]]
    return {
        "track_id": track_id, "class_name": class_name, "class_id": 0,
        "confidence": confidence, "semantic_status": semantic_status,
        "footprint_xy": hull, "z_min": 0.02, "z_max": 1.7, "r_near": r_near,
        "velocity_xy": list(velocity_xy), "velocity_frame": velocity_frame,
        "velocity_valid": velocity_frame == "ego_removed",
        "velocity_status": velocity_status, "stable_frames": 40,
        "first_seen_mono_ms": now - 2000, "last_seen_mono_ms": now - 20,
        "depth_quality": "good",
    }


def objects_body(now: int, objects: List[Dict[str, Any]], *,
                 extrinsic: bool = True, t_capture: Optional[int] = None) -> Dict[str, Any]:
    return {
        "schema": "perception_objects_v1",
        "t_capture_mono_ms": (now - 20) if t_capture is None else t_capture,
        "t_publish_mono_ms": (now - 5) if t_capture is None else t_capture + 15,
        "frame": "base_link",
        "extrinsic_calibrated": extrinsic,
        "objects": objects,
    }


def status_body(now: int, *, extrinsic: bool = True, invalid_ratio: float = 0.05,
                seg_available: bool = True, reasons=(), gap_max: float = 44.0,
                t_publish: Optional[int] = None) -> Dict[str, Any]:
    return {
        "schema": "perception_status_v1",
        "t_publish_mono_ms": (now - 5) if t_publish is None else t_publish,
        "fps_depth": 29.6, "fps_infer": 21.3,
        "latency_ms_p50": 118.0, "latency_ms_p99": 196.0,
        "infer_gap_ms_p99": 47.0,
        "infer_gap_ms_max": gap_max,                # 11 v2.1 tier-1 reading
        "invalid_pixel_ratio": invalid_ratio,
        "ground_seg_level": 2,
        "extrinsic_calibrated": extrinsic,
        "traversable_seg_available": seg_available,
        "degraded_reasons": list(reasons),
    }


def wire(rid: str, seq: int, body: Dict[str, Any]) -> Dict[str, Any]:
    """The 11 S3.0 envelope around a body. The static samples omit mono/boot
    (a legal envelope shape, CLK-C4); --publish stamps them live."""
    return {"v": 1, "rid": rid, "ts": 0.0, "seq": seq, "src": "perception",
            "ts_sync": False, "data": body}


# ── the seven scenarios ──────────────────────────────────────────────────────
def build_scenarios(rid: str = "dev") -> Dict[str, Dict[str, Any]]:
    """name -> {"expect": human verdict, "ticks": [{now_mono_ms, profile,
    objects, status}, ...]}. Deterministic (no clock reads)."""
    out: Dict[str, Dict[str, Any]] = {}

    def tick(now: int, prof, objs, stat, seq: int) -> Dict[str, Any]:
        return {"now_mono_ms": now,
                "profile": None if prof is None else wire(rid, seq, prof),
                "objects": None if objs is None else wire(rid, seq, objs),
                "status": None if stat is None else wire(rid, seq, stat)}

    cone = obj(7, "traffic_cone", 2.6, -1.5, r_near=2.9)
    out["normal"] = {
        "expect": "drives at nominal speed; no failure; cone off-corridor ignored",
        "ticks": [tick(T0 + k * 50, profile_body(T0 + k * 50),
                       objects_body(T0 + k * 50, [cone]),
                       status_body(T0 + k * 50), k + 1) for k in range(3)]}
    out["all_unknown"] = {
        "expect": "UNKNOWN cap (S8.1A) + invalid-ratio cap (RNS-I-2): vx <= unk_g_min * v_nom",
        "ticks": [tick(T0 + k * 50, profile_body(T0 + k * 50, kind="unknown"),
                       objects_body(T0 + k * 50, []),
                       status_body(T0 + k * 50, invalid_ratio=0.85,
                                   reasons=("roi_fallback",)), k + 1)
                  for k in range(2)]}
    out["no_seg"] = {
        "expect": "geometry-only FREE: 0 < vx <= no_seg_speed_cap_mps (S3.1.11)",
        "ticks": [tick(T0 + k * 50, profile_body(T0 + k * 50, kind="noseg"),
                       objects_body(T0 + k * 50, []),
                       status_body(T0 + k * 50, seg_available=False,
                                   reasons=("no_traversable_seg",)), k + 1)
                  for k in range(2)]}
    out["extrinsic_uncal"] = {
        "expect": "failure extrinsic_uncalibrated; mission refused (11 S3.1B.4)",
        "ticks": [tick(T0, profile_body(T0, extrinsic=False),
                       objects_body(T0, [], extrinsic=False),
                       status_body(T0, extrinsic=False), 1)]}
    # a car reporting ZERO raw velocity for 2.25 s: with ego motion not removed
    # that zero means nothing (the robot may be the one moving), so the raw
    # refusal keeps it dynamic -> stop. A consumer that trusted raw would let it
    # pass the static criterion at 2 s and drive on -- the discriminating case.
    parked = obj(12, "car", 1.0, 0.0, r_near=1.0, velocity_xy=(0.0, 0.0),
                 velocity_frame="raw", velocity_status="static")
    out["tf_stale"] = {
        "expect": "raw velocity refused (S3.1.5): 'static' raw car stays dynamic past the dwell -> WAIT_DYNAMIC, vx == 0",
        "ticks": [tick(T0 + k * 250, profile_body(T0 + k * 250, pose_used=None),
                       objects_body(T0 + k * 250, [parked]),
                       status_body(T0 + k * 250), k + 1) for k in range(10)]}
    out["clock_reset"] = {
        "expect": "third tick > 1 s behind -> perception_epoch_reset audited, memory cleared, drives",
        "ticks": [tick(T0, profile_body(T0), objects_body(T0, []), status_body(T0), 1),
                  tick(T0 + 50, profile_body(T0 + 50), objects_body(T0 + 50, []),
                       status_body(T0 + 50), 2),
                  tick(3000, profile_body(3000), objects_body(3000, []),
                       status_body(3000), 1)]}
    out["dropout"] = {
        "expect": "objects 700 ms old (> T-52): objects_lost audited, 0 < vx <= no_seg_speed_cap_mps",
        "ticks": [tick(T0 + k * 50, profile_body(T0 + k * 50),
                       objects_body(T0 + k * 50, [], t_capture=T0 + k * 50 - 700),
                       status_body(T0 + k * 50, reasons=("infer_gap",), gap_max=700.0),
                       k + 1) for k in range(2)]}
    barrier = obj(21, "barrier", 2.5, 0.0, r_near=2.5)
    out["ground_withdrawn"] = {
        "expect": "fit fallback withdrawn (19 S3.2A v1.6): all bins UNKNOWN except the S block; "
                  "vx <= unk_g_min * v_nom, no failure, host gate not a veto (fresh)",
        "ticks": [tick(T0 + k * 50, profile_body(T0 + k * 50, kind="ground_withdrawn"),
                       objects_body(T0 + k * 50, [barrier]),
                       status_body(T0 + k * 50, reasons=("ground_fit_fallback",
                                                         "ground_free_withdrawn")),
                       k + 1) for k in range(2)]}
    out["semantic_only"] = {
        "expect": "S-only block: grid UNKNOWN at 1.5 m ahead (never FREE), BLOCKED at 2.5 m; no failure",
        "ticks": [tick(T0 + k * 50, profile_body(T0 + k * 50, kind="semantic_only"),
                       objects_body(T0 + k * 50, [barrier]),
                       status_body(T0 + k * 50), k + 1) for k in range(2)]}
    return out


def render(scenarios: Dict[str, Dict[str, Any]]) -> Dict[str, str]:
    """name -> canonical JSON text (sorted keys, 2-space indent, trailing
    newline) so the on-disk sample set is byte-comparable to the generator."""
    return {name: json.dumps({"scenario": name, **sc}, indent=2, sort_keys=True,
                             ensure_ascii=True) + "\n"
            for name, sc in scenarios.items()}


def write_samples(root: Path, rid: str = "dev") -> List[Path]:
    written = []
    texts = render(build_scenarios(rid))
    for name, text in texts.items():
        d = root / name
        d.mkdir(parents=True, exist_ok=True)
        p = d / "sequence.json"
        p.write_text(text, encoding="utf-8")
        written.append(p)
    index = {name: sc["expect"] for name, sc in build_scenarios(rid).items()}
    ip = root / "index.json"
    ip.write_text(json.dumps(index, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
                  encoding="utf-8")
    written.append(ip)
    return written


# ── --publish: put the sequence on the RT plane with live timestamps ─────────
def _rebase(body: Dict[str, Any], delta: int) -> Dict[str, Any]:
    """Shift every *_mono_ms timestamp of a body by delta so the sample's
    relative timing is kept but the absolute base is this host's clock."""
    out = dict(body)
    for k in ("t_capture_mono_ms", "t_publish_mono_ms", "t_seg_mono_ms"):
        if isinstance(out.get(k), int):
            out[k] = out[k] + delta
    if "objects" in out and isinstance(out["objects"], list):
        objs = []
        for o in out["objects"]:
            o2 = dict(o)
            for k in ("first_seen_mono_ms", "last_seen_mono_ms"):
                if isinstance(o2.get(k), int):
                    o2[k] = o2[k] + delta
            objs.append(o2)
        out["objects"] = objs
    return out


def publish(scenario_names, rid: str, loop: bool) -> int:
    import zenoh  # lazy: --write / index need no zenoh install
    from xbrain.common.envelope import read_local_boot_id
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["%s"]' % RT_ENDPOINT)
    session = zenoh.open(conf)
    pubs = {k: session.declare_publisher("xbrain/%s/rt/perception/%s" % (rid, k))
            for k in ("profile", "objects", "status")}
    try:
        boot = read_local_boot_id()
    except Exception:      # noqa: BLE001 -- no boot_id file (non-Linux dev box)
        boot = None
    scenarios = build_scenarios(rid)
    seq = {k: 0 for k in pubs}
    try:
        while True:
            for name in scenario_names:
                ticks = scenarios[name]["ticks"]
                base_now = int(time.monotonic() * 1000)
                t_first = ticks[0]["now_mono_ms"]
                for t in ticks:
                    # keep the sample's relative timing; rebase to this clock
                    delta = base_now - t_first
                    target = t["now_mono_ms"] + delta
                    while int(time.monotonic() * 1000) < target:
                        time.sleep(0.002)
                    for key, pub in pubs.items():
                        w = t.get(key)
                        if w is None:
                            continue
                        seq[key] += 1
                        env = dict(w)
                        env["seq"] = seq[key]
                        env["ts"] = time.time()
                        env["mono"] = time.monotonic()
                        if boot:
                            env["boot"] = boot
                        env["data"] = _rebase(w["data"], delta)
                        pub.put(json.dumps(env, ensure_ascii=True).encode("utf-8"))
                print("published %s (%d ticks)" % (name, len(ticks)), file=sys.stderr)
            if not loop:
                return 0
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0
    finally:
        session.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="perception_sim.py",
        description="W-11: simulated three-key perception messages (11 S3.1B)")
    ap.add_argument("--rid", default="dev", help="robot id in the key (default dev)")
    ap.add_argument("--write", nargs="?", const=str(SAMPLES_DIR), default=None,
                    metavar="DIR", help="write the sample set (default dir: %s)"
                    % SAMPLES_DIR)
    ap.add_argument("--publish", nargs="?", const="all", default=None,
                    metavar="SCENARIO", help="publish one scenario (or 'all') on the RT plane")
    ap.add_argument("--loop", action="store_true", help="with --publish: repeat until Ctrl-C")
    args = ap.parse_args(argv)
    if args.write is not None:
        for p in write_samples(Path(args.write), args.rid):
            print(p)
        return 0
    if args.publish is not None:
        names = list(SCENARIOS) if args.publish == "all" else [args.publish]
        for n in names:
            if n not in SCENARIOS:
                print("unknown scenario %r; choose from %s" % (n, ", ".join(SCENARIOS)),
                      file=sys.stderr)
                return 2
        return publish(names, args.rid, args.loop)
    for name, sc in build_scenarios(args.rid).items():
        print("%-16s %s" % (name, sc["expect"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
