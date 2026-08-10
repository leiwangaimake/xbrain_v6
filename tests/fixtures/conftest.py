"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: conftest.py
Brief: CHK-0-56 pytest fixture `resolved_configs` -- materialise a filled config tree

Description:
Runtime-builds a full L0-L6 tree under tmp_path:

  <tmp>/configs/
    common.yaml            <- prod copy + NULL_OVERRIDES applied
    models/m20s.yaml       <- prod copy + spec-related overrides
    safety/                <- SYMLINK to /opt/xbrain_v6/configs/safety
                              (ENV-2: L3 never follows XBRAIN_CONFIG_DIR
                               in production; here we intentionally point
                               to the real files so tests exercise the
                               SAME source of truth -- CHK-0-56 (iv))
    ...                    <- every other yaml under configs/ copied 1:1
  <tmp>/resolved/          <- tmpfs stand-in for /run/xbrain/resolved

Then runs xbrain.boot.freeze.pipeline.run_freeze() against that tree
and yields the ResolvedConfigs handle (paths + manifest dict + any
runner error).

The fixture takes an optional `mutation_extras` param via pytest's
indirect parametrize so mutation tests can inject bogus values into
the copy without touching prod. Similarly, `mutation_safety_copy=True`
replaces the safety symlink with a mutable copy so a mutation test
can flip a safety key to trigger assertion G.
"""

from __future__ import annotations

import os
import shutil
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

import pytest
import yaml

from tests.fixtures.overrides import apply_overrides


REAL_CONFIG_ROOT = "/opt/xbrain_v6/configs"


# Assertions the fixture MAY fail because the failure reflects a
# real prod-side defect that CHK-0-56 is not chartered to fix.
# Each entry is documented with the specific prod contradiction it
# surfaces. When the underlying defect is fixed by its owning
# ticket, delete the entry AND the fixture will start requiring it
# green (which is the design: the fixture is a canary).
KNOWN_FAILING_ASSERTIONS = {
    # B     was here until CFG-FZ-18-b (2026-08-10). intent.keyword_rules
    #       is now off the schema + off configs/p4_agent.yaml.
    # S22   was here between CFG-FZ-18-b and C-FZ-S22-LOOSEN (2026-08-10).
    #       Loosened to match 10 S5.4.3 line 2803-2804 which only forbids
    #       common.* top-level at L6, a rule already owned by assertion B.
    # materialise was here between C-FZ-S22-LOOSEN and V-CALIB-FIXTURE.
    #       Resolved by giving the CHK-0-56 fixture a full H-3 compatible
    #       ptz_base accuracy block (see _write_l4b_lab_robot_calib) so
    #       H-3 + H-4 both pass and materialise can expand the
    #       ${common.calib.frames.ptz_base.h_camera_m} ref in
    #       p2_core.yaml. Fixture values are synthetic (1 cm trans, ~0.057
    #       deg rot); real deployment still needs on-bench calibration.
    #
    # Empty dict = no KNOWN_FAILING. Any freeze failure below this line
    # surfaces as a real regression, not a documented gap.
}


@dataclass
class ResolvedConfigs:
    config_root: str
    resolved_root: str
    manifest: Optional[dict] = None
    freeze_error: Optional[Exception] = None
    per_process: Dict[str, str] = field(default_factory=dict)


def _copy_configs(dst: Path,
                    include_safety_as_copy: bool = False) -> None:
    """Copy every file/dir under REAL_CONFIG_ROOT into dst, except:
      * safety/  -> symlink to the real dir (unless caller asks for a
                    mutable copy)
      * sites/_skeleton.yaml, calib/_skeleton.yaml -- skipped
        (they are template placeholders per the file headers)
    """
    dst.mkdir(parents=True, exist_ok=True)
    src = Path(REAL_CONFIG_ROOT)
    for item in src.iterdir():
        if item.name == "safety":
            if include_safety_as_copy:
                shutil.copytree(item, dst / "safety")
            else:
                (dst / "safety").symlink_to(item.resolve())
        elif item.is_dir():
            shutil.copytree(item, dst / item.name)
        else:
            shutil.copy2(item, dst / item.name)
    # Prune templates so the L4/L4b loader (if wired) never trips on
    # underscore-prefixed placeholders.
    for tmpl in ((dst / "sites/_skeleton.yaml"),
                    (dst / "calib/_skeleton.yaml")):
        if tmpl.exists():
            tmpl.unlink()


def _rewrite_yaml_with_overrides(dst: Path,
                                    mutation_extras: Optional[Dict[str, Any]] = None) -> None:
    """Fill nulls in the copied config tree.

    Layer namespace rules (10 S5.4.3):
      * L1 common.yaml   -- may write any common.*
      * L2 models/*.yaml -- may write common.spec.*, common.motion.*
      * L3 safety/*.yaml -- may write common.safety.* (NOT touched here;
        symlinked to prod per CHK-0-56 iv)

    So we route each override to the layer whose namespace covers it:
    everything except common.spec.* / common.motion.* goes into
    common.yaml; spec + motion values BOTH go into common.yaml AND
    models/m20s.yaml so either layer alone satisfies the check (models
    is the authoritative layer at deploy; common carries the placeholder)."""
    from tests.fixtures.overrides import NULL_OVERRIDES

    def _route(dotted: str) -> tuple:
        if dotted.startswith("common.spec.") or dotted.startswith("common.motion."):
            return ("common.yaml", "models/m20s.yaml")
        return ("common.yaml",)

    per_file: Dict[str, Dict[str, Any]] = {}
    for k, v in NULL_OVERRIDES.items():
        for rel in _route(k):
            per_file.setdefault(rel, {})[k] = v
    if mutation_extras:
        for k, v in mutation_extras.items():
            for rel in _route(k):
                per_file.setdefault(rel, {})[k] = v

    for rel, subset in per_file.items():
        path = dst / rel
        if not path.exists():
            continue
        with open(path, encoding="utf-8") as fh:
            tree = yaml.safe_load(fh) or {}
        # apply_overrides ignores the extras kwarg semantics here; we
        # pass an empty NULL_OVERRIDES-equivalent via set_by_path
        # directly to control WHICH keys land in THIS file.
        from tests.fixtures.overrides import set_by_path
        for k, v in subset.items():
            set_by_path(tree, k, v)
        with open(path, "w", encoding="utf-8") as fh:
            yaml.safe_dump(tree, fh, allow_unicode=True, sort_keys=True)


def _mutate_safety_file(safety_dir: Path,
                          mutations: Dict[str, Any]) -> None:
    """Only used by mutation test (a). Directly writes into a safety
    yaml copy. Requires `include_safety_as_copy=True` at copy time."""
    if not mutations:
        return
    for rel, patch in mutations.items():
        path = safety_dir / rel
        with open(path, encoding="utf-8") as fh:
            tree = yaml.safe_load(fh) or {}
        # Deep-merge patch into tree.
        def _merge(a: dict, b: dict) -> None:
            for k, v in b.items():
                if isinstance(v, dict) and isinstance(a.get(k), dict):
                    _merge(a[k], v)
                else:
                    a[k] = v
        _merge(tree, patch)
        with open(path, "w", encoding="utf-8") as fh:
            yaml.safe_dump(tree, fh, allow_unicode=True, sort_keys=True)


def _write_l4_lab_site(dst: Path) -> None:
    """Write sites/lab.yaml matching common.site_id in the overrides.
    Namespace allowed at L4: common.geo.*, common.site.*, common.retention.*"""
    site = {
        "common": {
            "geo": {
                "enu_origin": {"lat": 31.2304, "lon": 121.4737, "alt": 4.0},
            },
        },
    }
    (dst / "sites").mkdir(parents=True, exist_ok=True)
    with open(dst / "sites" / "lab.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(site, fh, allow_unicode=True, sort_keys=True)


def _write_l4b_lab_robot_calib(dst: Path) -> None:
    """Write calib/lab_robot.yaml matching common.robot_id.
    L4b writes only common.calib.* per 10 S5.4.3.

    ptz_base frame is filled with a fixture-synthetic accuracy block
    complete enough to satisfy H-3 (schema) + H-4 (recompute) so
    freeze can reach the materialise assertion. Values are NOT real
    calibration -- they exist so:
      1. p2_core.yaml's ${common.calib.frames.ptz_base.h_camera_m}
         resolves
      2. H's mutation tests can still detect real defects when the
         fixture is derived-from (each H mutation flips ONE field
         and expects the resulting fixture to red)
    Every number here is derivable arithmetic (see the sigma_rot/
    sigma_trans below and the lat_err_ref_m recompute alongside),
    not a bench measurement -- so no operator will mistake this for
    a real calibration.
    """
    import math
    sigma_trans_m = [0.01, 0.01, 0.01]           # 1 cm translation
    sigma_rot_rad = [0.001, 0.001, 0.001]        # ~0.057 deg rotation
    d_ref = 10.0                                  # H-4 formula d_ref
    sigma_rot_max = max(sigma_rot_rad)
    sigma_trans_lat = math.hypot(sigma_trans_m[0], sigma_trans_m[1])
    # S10.1.1 formula verbatim: lat_err_at_d = d * sin(sigma_rot_max)
    # + sigma_trans_lat. Recomputed here so H-4's file-vs-formula
    # comparison matches to within 1e-6 m.
    lat_err_ref_m = d_ref * math.sin(sigma_rot_max) + sigma_trans_lat

    calib = {
        "common": {
            "calib": {
                "calib_rev": 1,
                # H-1 identity check: calib.robot_id == common.robot_id
                "robot_id": "lab_robot",
                "frames": {
                    # ptz_base -- H-2 whitelisted (S10.4.4 example).
                    "ptz_base": {
                        # Accuracy block nested under 'accuracy:' per
                        # h_calib.py's _accuracy_reader shape.
                        "accuracy": {
                            "method": "fixture-synthetic",
                            "sigma_trans_m": sigma_trans_m,
                            "sigma_rot_rad": sigma_rot_rad,
                            "n_samples": 100,
                        },
                        # xyz + rpy extrinsics.
                        "xyz": [0.0, 0.0, 0.42],
                        "rpy": [0.0, 0.0, 0.0],
                        # Consumed by p2_core.yaml ptz.h_camera_m ref.
                        "h_camera_m": 0.42,
                    },
                },
                # H-5 gate thresholds (M-24 placeholders per h_calib.py).
                "gate": {
                    "warn_m": 0.17,
                    "reject_m": 0.35,
                },
                # H-4 recompute target -- must match the formula above.
                "lat_err_ref_m": lat_err_ref_m,
            },
        },
    }
    (dst / "calib").mkdir(parents=True, exist_ok=True)
    with open(dst / "calib" / "lab_robot.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(calib, fh, allow_unicode=True, sort_keys=True)


def _build_and_freeze(tmp_path: Path,
                        mutation_extras: Optional[Dict[str, Any]] = None,
                        safety_copy: bool = False,
                        safety_mutations: Optional[Dict[str, Any]] = None
                        ) -> ResolvedConfigs:
    from xbrain.boot.freeze.pipeline import run_freeze

    cfg_root = tmp_path / "configs"
    resolved_root = tmp_path / "resolved"
    resolved_root.mkdir(parents=True, exist_ok=True)
    _copy_configs(cfg_root, include_safety_as_copy=safety_copy)
    _rewrite_yaml_with_overrides(cfg_root, mutation_extras=mutation_extras)
    _write_l4_lab_site(cfg_root)
    _write_l4b_lab_robot_calib(cfg_root)
    if safety_copy and safety_mutations:
        _mutate_safety_file(cfg_root / "safety", safety_mutations)

    handle = ResolvedConfigs(
        config_root=str(cfg_root),
        resolved_root=str(resolved_root),
    )
    try:
        handle.manifest = run_freeze(
            boot_id="fixture-boot-id",
            config_root=str(cfg_root),
            config_root_overridden=True,
            common_digest="fixture-digest",
            config_rev="fixture-rev",
            resolved_root=str(resolved_root),
        )
    except Exception as e:
        handle.freeze_error = e
        # If the failure is a documented pre-existing prod defect
        # (KNOWN_FAILING_ASSERTIONS), synthesise a stand-in manifest
        # so downstream tests can still inspect it. The freeze_error
        # is preserved for reporting.
        handle.manifest = _synthesise_manifest_after_expected_failure(
            e, cfg_root=str(cfg_root))
    return handle


def _synthesise_manifest_after_expected_failure(exc: Exception,
                                                  cfg_root: str) -> Optional[dict]:
    """When freeze aborts on a KNOWN_FAILING assertion, return a
    manifest-shaped dict with that assertion marked 'known_fail' and
    the reason. Returns None when the failure is not documented, so
    the test still red-lights an unexpected regression."""
    msg = str(exc)
    for name, reason in KNOWN_FAILING_ASSERTIONS.items():
        if ("assertion %s failed" % name) in msg or msg.startswith(
                "E_CONFIG_INVALID: assertion %s" % name):
            return {
                "schema": "xbrain-manifest-v1",
                "boot_id": "fixture-boot-id",
                "config_root": cfg_root,
                "config_root_overridden": True,
                "common_digest": "fixture-digest",
                "config_rev": "fixture-rev",
                "assertions": {
                    name: {"status": "known_fail",
                             "assertion": name,
                             "message": msg,
                             "reason": reason},
                },
            }
    return None


@pytest.fixture
def resolved_configs(tmp_path: Path) -> ResolvedConfigs:
    """Default happy-path fixture: prod configs + NULL_OVERRIDES,
    safety symlinked to real /opt/xbrain_v6/configs/safety, freeze
    runs, manifest returned."""
    return _build_and_freeze(tmp_path)


@pytest.fixture
def resolved_configs_factory(tmp_path: Path):
    """Advanced fixture returning a callable so mutation tests can
    parameterise. Usage:

        cfgs = resolved_configs_factory(mutation_extras={...})
    """
    def _make(mutation_extras: Optional[Dict[str, Any]] = None,
                safety_copy: bool = False,
                safety_mutations: Optional[Dict[str, Any]] = None
                ) -> ResolvedConfigs:
        # A per-call subdir so multiple mutations in one test don't
        # collide.
        subdir = tmp_path / f"case-{int(time.monotonic_ns()) & 0xffff:04x}"
        subdir.mkdir(parents=True, exist_ok=True)
        return _build_and_freeze(
            subdir, mutation_extras=mutation_extras,
            safety_copy=safety_copy,
            safety_mutations=safety_mutations)
    return _make
