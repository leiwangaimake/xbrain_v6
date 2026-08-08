"""CFG-FZ-9 assertion H tests: five variants + baseline."""

import math
from typing import Any, Dict

import pytest

from xbrain.boot.freeze.assertions.h_calib import run
from xbrain.common.errors.exceptions import XbrainError


# Green scaffold: uses the S10.4.4 example values so recompute matches.
def _green_calib() -> Dict[str, Any]:
    return {"common": {"calib": {
        "schema": "xbrain.calib/1",
        "robot_id": "xb-001",
        "calib_rev": "2026-08-08+gtest",
        "d_ref_m": 10.0,
        "gate": {"warn_m": 0.17, "reject_m": 0.35},
        "lat_err_ref_m": None,  # filled in by _finalize
        "frames": {
            "cam_rgbd": {
                "parent": "base_link",
                "xyz": [0.121, 0.004, 0.318],
                "rpy": [0.0012, -0.0087, 0.0031],
                "accuracy": {
                    "method": "target_board",
                    "rmse_reproj_px": 0.42,
                    "sigma_trans_m": [0.004, 0.004, 0.007],
                    "sigma_rot_rad": [0.0021, 0.0024, 0.0019],
                    "n_samples": 184,
                },
            },
        },
    }}}


def _finalize(tree):
    """Set lat_err_ref_m to the recomputed value so the scaffold is self-consistent."""
    frames = tree["common"]["calib"]["frames"]
    best = 0.0
    for f in frames.values():
        acc = f.get("accuracy")
        if not acc:
            continue
        rx, ry, rz = acc["sigma_rot_rad"]
        tx, ty, _ = acc["sigma_trans_m"]
        v = 10.0 * math.sin(max(rx, ry, rz)) + math.hypot(tx, ty)
        best = max(best, v)
    tree["common"]["calib"]["lat_err_ref_m"] = best
    return tree


def _ctx(tree=None, rid="xb-001"):
    return {"config_root": "/tmp",
            "calib_raw": _finalize(tree or _green_calib()),
            "common_robot_id": rid}


def test_green_scaffold_passes():
    result = run(_ctx())
    assert result["status"] == "pass"
    assert result["assertion"] == "H"
    assert result["checks_run"] == 5
    assert result["frames_checked"] == 1
    assert result["outcome"] == "pass"


# Variant 1: robot_id mismatch
def test_variant_1_robot_id_mismatch_h1_red():
    tree = _green_calib()
    tree["common"]["calib"]["robot_id"] = "xb-999"
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, rid="xb-001"))
    assert ei.value.detail["kind"] == "calib_robot_id_mismatch"


# Variant 2: unregistered frame key
def test_variant_2_unregistered_frame_h2_red():
    tree = _green_calib()
    tree["common"]["calib"]["frames"]["foo"] = {
        "parent": "base_link", "xyz": [0, 0, 0], "rpy": [0, 0, 0],
        "accuracy": {"method": "manual_survey",
                     "sigma_trans_m": [0.01, 0.01, 0.01],
                     "sigma_rot_rad": [0.001, 0.001, 0.001],
                     "n_samples": 3},
    }
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree))
    assert ei.value.detail["kind"] == "unregistered_frame_key"
    assert ei.value.detail["value"] == "foo"


def test_h2_widen_via_extra_ids():
    tree = _green_calib()
    tree["common"]["calib"]["frames"]["custom_thing"] = {
        "parent": "base_link", "xyz": [0, 0, 0], "rpy": [0, 0, 0],
        "accuracy": {"method": "manual_survey",
                     "sigma_trans_m": [0.001, 0.001, 0.001],
                     "sigma_rot_rad": [0.0001, 0.0001, 0.0001],
                     "n_samples": 3},
    }
    ctx = _ctx(tree)
    ctx["extra_frame_ids"] = ["custom_thing"]
    result = run(ctx)
    assert result["status"] == "pass"


# Variant 3: accuracy block missing
def test_variant_3_no_accuracy_h3_red():
    tree = _green_calib()
    del tree["common"]["calib"]["frames"]["cam_rgbd"]["accuracy"]
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree))
    assert ei.value.detail["kind"] == "accuracy_incomplete"
    assert ei.value.detail["reason"] == "block_missing"


def test_h3_partial_accuracy_missing_field():
    tree = _green_calib()
    del tree["common"]["calib"]["frames"]["cam_rgbd"]["accuracy"]["n_samples"]
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree))
    assert ei.value.detail["kind"] == "accuracy_incomplete"
    assert ei.value.detail["missing"] == "n_samples"


# Variant 4: lat_err_ref_m tampered
def test_variant_4_tampered_lat_err_h4_red():
    tree = _finalize(_green_calib())
    tree["common"]["calib"]["lat_err_ref_m"] = 0.001  # deliberately wrong
    ctx = {"config_root": "/tmp", "calib_raw": tree,
           "common_robot_id": "xb-001"}
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "lat_err_recompute_mismatch"


def test_h4_within_tolerance_passes():
    tree = _finalize(_green_calib())
    # nudge by 1e-8 -- well below 1e-6 tolerance
    tree["common"]["calib"]["lat_err_ref_m"] += 1e-8
    ctx = {"config_root": "/tmp", "calib_raw": tree,
           "common_robot_id": "xb-001"}
    result = run(ctx)
    assert result["status"] == "pass"


# Variant 5: lat_err > reject
def test_variant_5_lat_err_over_reject_h5_red():
    tree = _green_calib()
    # Bloat sigma_rot to push lat_err above 0.35.
    tree["common"]["calib"]["frames"]["cam_rgbd"]["accuracy"][
        "sigma_rot_rad"] = [0.05, 0.05, 0.05]
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree))
    assert ei.value.detail["kind"] == "lat_err_over_reject"


def test_h5_degrade_band_returns_signal():
    tree = _green_calib()
    # Push into degrade band: > warn 0.17, <= reject 0.35.
    # sigma_rot_rad ~ 0.02 gives d*sin(0.02) = 10*0.02 = 0.2 + tiny trans.
    tree["common"]["calib"]["frames"]["cam_rgbd"]["accuracy"][
        "sigma_rot_rad"] = [0.02, 0.02, 0.02]
    result = run(_ctx(tree))
    assert result["outcome"] == "degrade"
    assert 0.17 < result["lat_err_ref_m"] <= 0.35


def test_h5_default_thresholds_when_gate_absent():
    tree = _green_calib()
    del tree["common"]["calib"]["gate"]
    tree["common"]["calib"]["frames"]["cam_rgbd"]["accuracy"][
        "sigma_rot_rad"] = [0.05, 0.05, 0.05]
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree))
    assert ei.value.detail["kind"] == "lat_err_over_reject"


def test_multi_frame_max_wins():
    tree = _green_calib()
    tree["common"]["calib"]["frames"]["rslidar"] = {
        "parent": "base_link", "xyz": [0.305, 0, 0.142],
        "rpy": [0, 0, 0],
        "accuracy": {
            "method": "manual_survey",
            "sigma_trans_m": [0.02, 0.02, 0.01],
            "sigma_rot_rad": [0.03, 0.03, 0.03],
            "n_samples": 3,
        },
    }
    tree = _finalize(tree)
    result = run({"config_root": "/tmp", "calib_raw": tree,
                  "common_robot_id": "xb-001"})
    # Bigger sigma from rslidar dominates cam_rgbd.
    assert result["lat_err_ref_m"] > 0.2


def test_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
