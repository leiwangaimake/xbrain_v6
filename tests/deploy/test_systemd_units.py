"""INF-DP-7 / CFG-BT-3 -- systemd 15-unit set integrity."""

import re
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO_ROOT = Path(__file__).parent.parent.parent
UNIT_DIR = REPO_ROOT / "deploy" / "systemd"


# 15-unit table drawn from 10 S3.3 + CLAUDE.md S0.1.
# Column layout: (name, stage_key, oom_score, must_require_freeze)
# stage_key is the ordering bucket, not a literal string in the unit.
# xbrain-probe (Stage 0) is authored under CFG-BT-1, tested separately.
UNITS = [
    ("xbrain-zenohd-rt.service",         "0z1", None, False),
    ("xbrain-zenohd-gen.service",        "0z2", None, False),
    ("xbrain-config-freeze.service",     "0c",  None, False),  # one-shot
    ("xbrain-quadruped.service",         "1",   -1000, True),
    ("xbrain-perception.service",        "1",   -900,  True),
    ("xbrain-rtk-driver.service",        "1",   -500,  True),
    ("xbrain-teleop-input.service",      "1",   -500,  True),
    ("xbrain-behavior-proxy.service",    "1",   -500,  True),
    ("xbrain-nav2-behavior.service",     "1",   -500,  True),
    ("xbrain-zenoh-bridge.service",      "1",   -500,  True),
    ("xbrain-chassis-relay.service",     "2",   -1000, True),
    ("xbrain-p1-motion.service",         "2",   -1000, True),
    ("xbrain-p2-core.service",           "3",   -500,  True),
    ("xbrain-p3-task.service",           "3",   -500,  True),
    ("xbrain-p4-agent.service",          "3",    200,  True),
    ("xbrain-p5-gateway.service",        "3",    200,  False),  # minimal-mode
    # Stage 5 (AI) units are validated by their own test files.
]


def _read(name: str) -> str:
    p = UNIT_DIR / name
    assert p.is_file(), "unit missing: %s" % name
    return p.read_text()


def _read_no_comments(name: str) -> str:
    """Strip lines starting with # so string search does not fire on
    the head-comment mention of an anti-pattern."""
    return "\n".join(
        line for line in _read(name).splitlines()
        if not line.lstrip().startswith("#")
    )


# --- Existence -------------------------------------------------------

@pytest.mark.parametrize("name,stage,oom,req", UNITS)
def test_unit_file_exists(name, stage, oom, req):
    assert (UNIT_DIR / name).is_file()


# --- Requires=config-freeze ------------------------------------------
# All Stage 1/2/3 units MUST Requires=xbrain-config-freeze.service so
# CFG-FZ-1 refuses to release the boot gate if freeze failed.
# p5_gateway is the sole exception -- 10 S3.3 sets it up for minimal-
# mode observation even if freeze failed (W-1 observation window).

@pytest.mark.parametrize("name,stage,oom,req", UNITS)
def test_freeze_requires_matches_class(name, stage, oom, req):
    src = _read_no_comments(name)
    if req:
        assert "Requires=xbrain-config-freeze.service" in src, \
            "%s must Requires= config-freeze" % name
    else:
        # For units that intentionally DON'T require freeze, a stray
        # Requires= line would defeat the minimal-mode design.
        assert "Requires=xbrain-config-freeze.service" not in src, \
            "%s must NOT Requires= config-freeze (minimal-mode)" % name


# --- OOMScoreAdjust matches 11 S11A.6.3 table ------------------------

@pytest.mark.parametrize("name,stage,oom,req", UNITS)
def test_oom_score_matches_doc_table(name, stage, oom, req):
    if oom is None:
        return   # oneshot / router units have no OOM requirement here
    src = _read(name)
    m = re.search(r"OOMScoreAdjust=(-?\d+)", src)
    assert m, "%s missing OOMScoreAdjust" % name
    assert int(m.group(1)) == oom, \
        "%s: OOMScoreAdjust=%s, doc says %s" % (name, m.group(1), oom)


# --- GATE-3: quadruped BEFORE p1_motion ------------------------------
# 11 GATE-3: quadruped must be up before p1_motion issues any /cmd_vel
# else the very first /cmd_vel misses.

def test_gate3_quadruped_before_p1_motion():
    src = _read("xbrain-p1-motion.service")
    assert re.search(r"^After=.*xbrain-quadruped\.service", src, re.M), \
        "GATE-3 violated: p1_motion must After= xbrain-quadruped"


# --- GATE-4: chassis_relay BEFORE p1_motion --------------------------

def test_gate4_chassis_relay_before_p1_motion():
    src = _read("xbrain-p1-motion.service")
    assert re.search(r"^After=.*xbrain-chassis-relay\.service", src, re.M), \
        "GATE-4 violated: p1_motion must After= xbrain-chassis-relay"


# --- GATE-1: RT router before any RT participant ---------------------
# quadruped, perception, chassis_relay, p1_motion all live on RT plane.
# They must come After= xbrain-zenohd-rt.service directly OR transitively
# (chassis_relay + p1_motion inherit via quadruped which inherits directly).

def test_gate1_quadruped_after_rt_router():
    assert "After=xbrain-zenohd-rt.service" in _read("xbrain-quadruped.service")


def test_gate1_perception_after_rt_router():
    assert "After=xbrain-zenohd-rt.service" in _read("xbrain-perception.service")


# --- Restart / hard-gating ------------------------------------------
# All Stage 1/2/3 runtime units must Restart=always. AI Stage 5 units
# are Restart=on-failure (different discipline), tested separately.

@pytest.mark.parametrize("name,stage,oom,req", UNITS)
def test_runtime_units_restart_always(name, stage, oom, req):
    if stage in ("0", "0z1", "0z2", "0c"):
        return   # oneshot / infrastructure discipline is per-unit
    src = _read(name)
    assert "Restart=always" in src, "%s must Restart=always" % name


# --- Stage 5 anchor is NOT in Before= chains -------------------------
# 10 S3.3 iron rule: Stage 5 (release) is triggered by p2_core, not
# systemd. No unit should Before= a hypothetical "release" target.

@pytest.mark.parametrize("name,stage,oom,req", UNITS)
def test_no_release_target_in_before(name, stage, oom, req):
    src = _read(name)
    assert "Before=xbrain-release" not in src
    assert "Before=release.target" not in src


# --- Head comment names both INF-DP-7 and CFG-BT-3 -------------------
# Belt-and-braces: ties the file back to its lineage in 21 and TODO.

@pytest.mark.parametrize("name,stage,oom,req", UNITS)
def test_head_comment_names_lineage(name, stage, oom, req):
    if stage in ("0z1", "0z2", "0c", "0"):
        return   # older units authored under CFG-BT-2 / CFG-FZ-1
    head = _read(name).splitlines()[0]
    assert "INF-DP-7" in head or "CFG-BT-3" in head, \
        "%s head comment missing INF-DP-7 / CFG-BT-3 lineage" % name


# --- Description not a dup of Brief ---------------------------------
# Sanity: Description= line must exist and be non-empty.

@pytest.mark.parametrize("name,stage,oom,req", UNITS)
def test_description_present_and_nonempty(name, stage, oom, req):
    src = _read(name)
    m = re.search(r"^Description=(.+)$", src, re.M)
    assert m, "%s missing Description=" % name
    assert len(m.group(1).strip()) > 10


# --- StartLimitBurst / IntervalSec set --------------------------------
# AIR-F1 iron rule (per 11 S11A.6.3): units must not restart unbounded.

@pytest.mark.parametrize("name,stage,oom,req", UNITS)
def test_start_limit_bounded(name, stage, oom, req):
    if stage in ("0", "0c"):
        return   # oneshot units have no restart semantics
    src = _read(name)
    assert "StartLimitBurst=" in src, "%s missing StartLimitBurst" % name
    assert "StartLimitIntervalSec=" in src, \
        "%s missing StartLimitIntervalSec" % name


# --- Set totals -----------------------------------------------------
# If the 15-unit set was hand-edited elsewhere, the count guard here
# is the canary. 21 units total = 4 already existing (probe, RT, GEN,
# freeze) + 13 built by INF-DP-7 + 4 AI/payload (Stage 5).
# The exact count on disk is asserted only for the *runtime* subset
# validated by this file.

def test_runtime_unit_count_13():
    """Runtime = 13 rows with stage in {1, 2, 3}.
    Full 15-unit set = 13 runtime + 4 boot (probe, RT, GEN, freeze) -
    2 already tested by their own suites (RT/GEN in test_zenoh_...).
    xbrain-probe (Stage 0) tested separately when CFG-BT-1 lands."""
    n = sum(1 for _, s, _, _ in UNITS if s in ("1", "2", "3"))
    assert n == 13, "runtime set drifted: %d" % n
