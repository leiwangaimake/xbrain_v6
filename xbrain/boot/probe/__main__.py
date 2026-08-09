"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __main__.py
Brief: xbrain.boot.probe entry point (Stage 0 oneshot)

Description:
Wire-up. Reads a probe config YAML, runs the four platform checks
plus GATE-6, and exits with 0 on pass or non-zero on any failure.
Non-zero exit propagates through systemd's Requires= chain and
prevents Stage 0z / 0c / 1 / 2 / 3 from starting.

Config lookup order (frozen -- no fallbacks per CLAUDE.md 3.1):
  * XBRAIN_PROBE_CONFIG environment variable (test override)
  * /opt/xbrain_v6/configs/probe/thresholds.yaml (deploy default)

The config's job is to specify the numeric thresholds; there is no
"default" for any of them. A missing key crashes the probe (which
maps to non-zero exit, which refuses to release Stage 0).

The one exit-code contract worth spelling out here: sys.exit(1) for
any failure. The E_* code and detail dict go to stderr as a JSON line
per failure. Systemd captures stderr to the journal; the gateway can
then translate the journal line into an event/fault (per 11 S8.13.5
"error mapping is the gateway's single implementation point").
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import List

import yaml

from xbrain.boot.probe import checks, net_profile
from xbrain.common.errors import E_CONFIG_INVALID, E_STORAGE_CORRUPT
from xbrain.common.errors.exceptions import XbrainError


_DEFAULT_CONFIG_PATH = "/opt/xbrain_v6/configs/probe/thresholds.yaml"
_DEFAULT_HW_PROFILE_PATH = "/etc/xbrain/hw_profile"


def _emit(code: str, detail: dict) -> None:
    """Print a JSON line to stderr. Systemd captures it into the
    journal; the gateway parses it back per 11 S8.13.5. One line per
    failure, never batched, so a probe that fires two failures both
    show up rather than one masking the other."""
    line = json.dumps(
        {"code": code, "detail": detail},
        ensure_ascii=False, sort_keys=True)
    print(line, file=sys.stderr, flush=True)


def _load_config(path: str) -> dict:
    """Read probe thresholds. Every required key must be present and
    non-null -- CLAUDE.md 3.1 forbids defaulting safety parameters."""
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise ValueError("probe config top-level must be a mapping")
    required = ("disk", "memory", "temperature", "databases")
    for k in required:
        if doc.get(k) is None:
            raise ValueError(
                "probe config missing required key %r "
                "(unresolved per CLAUDE.md 3.1)" % k)
    return doc


def run(config_path: str = _DEFAULT_CONFIG_PATH,
        hw_profile_path: str = _DEFAULT_HW_PROFILE_PATH,
        iface_reader=None) -> int:
    """Run all Stage 0 checks; return process exit code.

    iface_reader is an injection point for tests -- read_actual() uses
    it in place of the real `ip addr` invocation."""
    failures: List[tuple] = []   # (code, detail)

    # --- Load config (E_CONFIG_INVALID if malformed) ------------
    try:
        cfg = _load_config(config_path)
    except (FileNotFoundError, ValueError, yaml.YAMLError) as exc:
        _emit(E_CONFIG_INVALID, {
            "kind": "probe_config_invalid",
            "config_path": config_path,
            "reason": str(exc),
        })
        return 1

    # --- Disk ---------------------------------------------------
    disk_cfg = cfg["disk"]
    for entry in disk_cfg:
        path = entry["path"]
        thresh = entry["threshold_pct"]
        f = checks.check_disk(path, thresh)
        if f is not None:
            failures.append((E_CONFIG_INVALID, f))

    # --- Memory -------------------------------------------------
    mem_cfg = cfg["memory"]
    f = checks.check_memory(mem_cfg["min_free_kb"])
    if f is not None:
        failures.append((E_CONFIG_INVALID, f))

    # --- Temperature --------------------------------------------
    temp_cfg = cfg["temperature"]
    f = checks.check_temperature(
        temp_cfg["sensors"], temp_cfg["max_temp_c"])
    if f is not None:
        failures.append((E_CONFIG_INVALID, f))

    # --- Databases (E_STORAGE_CORRUPT on corruption) ------------
    for db in cfg["databases"]:
        f = checks.check_db_schema(db["path"], db["expected_version"])
        if f is None:
            continue
        if f["kind"] == "db_corrupt":
            failures.append((E_STORAGE_CORRUPT, f))
        else:
            # Missing / schema mismatch are config-invalid, not
            # storage-corrupt (the file is fine, just wrong version
            # or absent).
            failures.append((E_CONFIG_INVALID, f))

    # --- GATE-6 network profile ---------------------------------
    try:
        profile = net_profile.load_profile(hw_profile_path)
    except (FileNotFoundError, ValueError, yaml.YAMLError) as exc:
        _emit(E_CONFIG_INVALID, {
            "kind": "hw_profile_missing",
            "hw_profile_path": hw_profile_path,
            "reason": str(exc),
        })
        return 1

    # NET-C1 profile-internal check first: if the profile itself has
    # overlapping segments there is no point comparing to actuals.
    overlaps = net_profile.find_network_overlaps(profile)
    for o in overlaps:
        failures.append((E_CONFIG_INVALID, o))

    if not overlaps:
        if iface_reader is None:
            actual = net_profile.read_actual(profile)
        else:
            actual = net_profile.read_actual(profile, iface_reader=iface_reader)
        for d in net_profile.diff_profile(profile, actual):
            failures.append((E_CONFIG_INVALID, d))

    # --- Emit and exit ------------------------------------------
    for code, detail in failures:
        _emit(code, detail)
    return 0 if not failures else 1


def main() -> None:
    cfg_path = os.environ.get("XBRAIN_PROBE_CONFIG", _DEFAULT_CONFIG_PATH)
    hw_path = os.environ.get("XBRAIN_HW_PROFILE", _DEFAULT_HW_PROFILE_PATH)
    sys.exit(run(cfg_path, hw_path))


if __name__ == "__main__":
    main()
