"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_health_all.py
Brief: BIZ-P2-18/20 health items vs the contract + the restricted-function matrix

Description:
*** Brief 由占位串改写(2026-08-23). 原值是按路径自动生成的
"health tests -- health all" -- 既没说清本文件测什么, 也无法据以索引任务号, 于是 P2 是唯一
无法自动提取证据映射的子系统(CLAUDE.md 2.5 要求 Brief 一行说清).
BIZ-P2-18/19/20/21 -- health items + factor + restrict matrix + three-stops.
"""


import pytest

from xbrain.p2_core.health.factor import (
    FactorConfig, FactorOutput, compute_factor, factor_for,
)
from xbrain.p2_core.health.items import (
    BIT_ONLY_ITEMS, HEALTH_ITEMS, ITEMS, HealthLevel, HealthState,
    ITEM_LEVELS, is_fatal, level_of,
)
from xbrain.p2_core.health.restrict_matrix import (
    check_asr_local_admission, check_new_task_admission,
    check_ptz_command, check_time_window_rules_active,
)
from xbrain.p2_core.three_stops import (
    ForceStrobeState, StopEvent, StopReason,
    apply_rearm, apply_stop,
)


pytestmark = pytest.mark.no_device


# --- items ---

def _contract_items():
    """The 11 S5.1A table, read from the contract.

    Extracted rather than transcribed a second time here: a test that restates
    the table is comparing one transcription against another, and a shared
    mistake reads as agreement. Columns: name, kind, level, counts toward
    speed_factor, drives allow_motion.
    """
    import os
    import re
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    with open(os.path.join(root, "docs", "11-接口契约.md"),
              encoding="utf-8") as fh:
        lines = fh.read().split("\n")
    start = next(i for i, line in enumerate(lines)
                 if line.startswith("| 项 | `kind` |"))
    rows = {}
    for line in lines[start + 1:]:
        if not line.startswith("|"):
            break
        cells = [c.strip() for c in line.split("|")[1:-1]]
        if not cells or set(cells[0]) <= set("-: "):
            continue
        name = re.search(r"`([a-z_]+)`", cells[0])
        level = re.search(r"(fatal|degraded|warn)", cells[2])
        if not name or not level:
            continue
        rows[name.group(1)] = (
            "device" if "device" in cells[1] else "cap",
            level.group(1),
            # The role columns are ticked for yes and carry 否 or a dash for
            # no; the tick is the only positive marker, so it is what is read.
            "\u2705" in cells[3], "\u2705" in cells[4])
    return rows


def test_health_items_match_the_contract():
    """*** The check whose ABSENCE let the table drift.

    Until 2026-08-20 this file asserted only that a handful of names were
    PRESENT. A presence check passes on any superset, so it stayed green while
    the code held nineteen items of which seven were invented (estop,
    payload_svc, state_link, config_freeze, persistence, ptz_home, ai_svc) and
    seven contract items were missing -- including compute and battery, two of
    the five whose failure must forbid motion. The aggregate could not refuse
    motion for an overheating computer: the item did not exist to fail.

    MUTATION: drop any row from ITEMS, or add one -- this reports it by name.
    """
    contract = _contract_items()
    assert set(contract) == set(ITEMS), (
        "only in contract: %s; only in code: %s"
        % (sorted(set(contract) - set(ITEMS)),
           sorted(set(ITEMS) - set(contract))))


@pytest.mark.parametrize("name", sorted(ITEMS))
def test_each_item_row_matches_the_contract(name):
    """Per-item, all four columns. One case per row so a wrong level names the
    item instead of failing a whole-table comparison.

    MUTATION: flip counts_in_speed_factor on rtk (the one that looks like an
    oversight -- rtk is fatal, why would it not count?) and this reddens. It
    must not count: 12 S6.6 multiplies h_factor by i_factor afterwards, so
    counting rtk here applies its restriction twice.
    """
    kind, level, counts, drives = _contract_items()[name]
    item = ITEMS[name]
    assert item.kind.value == kind
    assert item.level.value == level
    assert item.counts_in_speed_factor is counts
    assert item.drives_allow_motion is drives


def test_bit_only_items_stay_out_of_the_health_summary():
    """11 S5.1A: a BIT-only item is the result of a one-shot boot action, not a
    monitored quantity. In HealthSummary it would be a field that never updates
    -- the operator reads ptz_home: ok without knowing it is three days old."""
    assert not (set(BIT_ONLY_ITEMS) & HEALTH_ITEMS)
    assert "ptz_home" in BIT_ONLY_ITEMS


def test_is_fatal_returns_true_for_fatal_items():
    assert is_fatal("chassis")
    assert is_fatal("cam_rgbd")
    assert is_fatal("clock")        # fatal per S5.1A (was degraded in code)
    assert not is_fatal("mic")
    assert not is_fatal("ptz")


def test_level_of_raises_on_unknown_item():
    with pytest.raises(KeyError):
        level_of("this_is_not_an_item")


# --- factor: fatal-fail -> allow_motion=False ---

def _cfg():
    return FactorConfig(fatal_degraded=0.3, degraded_fail=0.5,
                        degraded_degraded=0.7, unknown=0.5)


def test_fatal_fail_forces_allow_motion_false():
    """Fatal item in FAIL -> allow_motion=False, factor=0, profile=none."""
    states = {"chassis": HealthState.FAIL}
    out = compute_factor(states, _cfg())
    assert out.allow_motion is False
    assert out.speed_factor == 0.0
    assert out.max_profile == "none"


def test_all_ok_gives_factor_one():
    states = {k: HealthState.OK for k in HEALTH_ITEMS}
    out = compute_factor(states, _cfg())
    assert out.allow_motion is True
    assert out.speed_factor == 1.0
    assert out.max_profile == "patrol"


def test_degraded_lidar_multiplies_factor():
    """lidar is DEGRADED level; state DEGRADED -> 0.7 factor."""
    states = {k: HealthState.OK for k in HEALTH_ITEMS}
    states["lidar"] = HealthState.DEGRADED
    out = compute_factor(states, _cfg())
    assert out.speed_factor == pytest.approx(0.7)


def test_speed_factor_is_min_not_product():
    """*** 14 S8.2 step 2 says min, and gives the reason: a product compounds
    several mild degradations into an unreasonable value.

    Two degraded items at 0.7 and 0.3: min is 0.3, a product would be 0.21.
    MUTATION: multiply instead -- this reddens, and on the robot the speed
    becomes a number nobody standing next to it can explain.
    """
    states = {k: HealthState.OK for k in HEALTH_ITEMS}
    states["lidar"] = HealthState.DEGRADED       # degraded level -> 0.7
    states["chassis"] = HealthState.DEGRADED     # fatal level    -> 0.3
    out = compute_factor(states, _cfg())
    assert out.speed_factor == pytest.approx(0.3)


#: 14 S8.2 step 1, the whole (state x level) table. Transcribed as data so the
#: cases below drive every cell rather than the four that happen to occur.
_FACTOR_TABLE = [
    # (level, state, expected factor)
    ("fatal", HealthState.OK, 1.0), ("fatal", HealthState.WARN, 1.0),
    ("fatal", HealthState.DEGRADED, 0.3), ("fatal", HealthState.UNKNOWN, 0.5),
    ("degraded", HealthState.OK, 1.0), ("degraded", HealthState.WARN, 1.0),
    ("degraded", HealthState.DEGRADED, 0.7),
    ("degraded", HealthState.FAIL, 0.5),
    ("degraded", HealthState.UNKNOWN, 0.5),
    # The warn row: 1.0 in EVERY state, unknown included (answer 1).
    ("warn", HealthState.OK, 1.0), ("warn", HealthState.WARN, 1.0),
    ("warn", HealthState.DEGRADED, 1.0), ("warn", HealthState.FAIL, 1.0),
    ("warn", HealthState.UNKNOWN, 1.0),
]

#: One item of each level to drive the table with.
_LEVEL_SAMPLE = {"fatal": "chassis", "degraded": "lidar", "warn": "dla"}


@pytest.mark.parametrize("level,state,expected", _FACTOR_TABLE)
def test_factor_table_cell(level, state, expected):
    """*** Driven through factor_for, NOT through compute_factor.

    The warn row is the reason. Every warn-level item in the current closed set
    (dla, payload_light, network) is also an item that does not count toward
    speed_factor -- so a case that reached the warn rule through the aggregate
    would stay green with the rule deleted, and the first time a warn item ever
    did count, every startup would drop to half speed with nothing on site to
    explain it. That is what 14 S8.2 answer 1 exists to prevent, so it is
    asserted where it is observable.

    MUTATION: remove the warn short-circuit in factor_for -- the four warn rows
    with a non-ok state go red here and stay green through the aggregate.
    """
    assert factor_for(_LEVEL_SAMPLE[level], state, _cfg()) == \
        pytest.approx(expected)


def test_fatal_but_warn_state_does_not_slow_down():
    """14 S8.2 answer 2: warn means off-nominal but inside tolerance (BIT-20),
    and inside tolerance is not a reason to slow down."""
    states = {k: HealthState.OK for k in HEALTH_ITEMS}
    states["chassis"] = HealthState.WARN
    assert compute_factor(states, _cfg()).speed_factor == pytest.approx(1.0)


def test_rtk_and_heading_do_not_enter_either_aggregate():
    """*** HL-2. They are fatal, yet count toward neither speed_factor nor
    allow_motion: their constraint is the i_fix / i_heading hard caps of
    11 S3.2.1, and 12 S6.6 multiplies h_factor by i_factor afterwards, so
    counting them here applies the same restriction twice.

    MUTATION: mark either as counting -- speed drops on an RTK loss twice
    over, and marking them as driving allow_motion zero-speeds the teleop
    escape, which U34 forbids by name.
    """
    states = {k: HealthState.OK for k in HEALTH_ITEMS}
    states["rtk"] = HealthState.FAIL
    states["heading"] = HealthState.FAIL
    out = compute_factor(states, _cfg())
    assert out.allow_motion is True
    assert out.speed_factor == pytest.approx(1.0)


def test_unreported_item_is_unknown_not_absent():
    """A partial map must not read as a healthy one: skipping an item nobody
    reported would make the aggregate IMPROVE when a health source dies.
    MUTATION: iterate item_states instead of the closed set -- an empty map
    then yields speed_factor 1.0 and a happy robot with no monitors at all."""
    out = compute_factor({}, _cfg())
    assert out.speed_factor < 1.0
    # cam_rgbd unknown -> no profile is admissible -> no motion (S8.3).
    assert out.allow_motion is False and out.max_profile == "none"


def test_unknown_camera_forbids_motion():
    """*** 14 S8.3: if not even obstacle_avoid is admissible, allow_motion is
    false. The previous code returned allow_motion=True with max_profile=none
    -- "you may move, at no profile".

    This is the case an ORIN with no camera lands in, and false is the right
    answer there: without the obstacle-avoidance sensor the robot may not be
    driven, including under teleop while recording.
    """
    states = {k: HealthState.OK for k in HEALTH_ITEMS}
    states["cam_rgbd"] = HealthState.UNKNOWN
    out = compute_factor(states, _cfg())
    assert out.allow_motion is False and out.max_profile == "none"


def test_cam_rgbd_not_ok_reduces_profile_to_none():
    """max_profile requires cam_rgbd OK (per p2_core.yaml.health)."""
    states = {"cam_rgbd": HealthState.FAIL}
    out = compute_factor(states, _cfg())
    # cam_rgbd is FATAL + FAIL -> allow_motion False first.
    assert out.allow_motion is False


def test_cam_rgbd_degraded_still_allows_patrol():
    states = {"cam_rgbd": HealthState.DEGRADED}
    out = compute_factor(states, _cfg())
    # DEGRADED cam_rgbd not FAIL, so allow_motion True, profile patrol
    # (both admissible profiles require it, degraded still counts).
    assert out.allow_motion is True


# --- restrict matrix ---

def test_rtk_degraded_refuses_new_tasks_fm1():
    """FM-1: rtk=degraded -> new tasks refused with E_DEGRADED."""
    d = check_new_task_admission({"rtk": HealthState.DEGRADED})
    assert not d.allowed
    assert d.code == "E_DEGRADED"
    assert d.detail_item == "rtk"


def test_rtk_ok_admits_new_tasks():
    d = check_new_task_admission({"rtk": HealthState.OK})
    assert d.allowed


def test_mic_fail_blocks_asr_local():
    """FM-3: mic=fail blocks asr_local (does NOT auto-switch mode)."""
    d = check_asr_local_admission({"mic": HealthState.FAIL})
    assert not d.allowed
    assert d.code == "E_UNHEALTHY"
    assert d.detail_item == "mic"


def test_ptz_fail_returns_unhealthy_not_capability():
    """FM-2: ptz=fail returns E_UNHEALTHY (device broke), NOT
    E_CAPABILITY (which would say the robot has no PTZ)."""
    d = check_ptz_command({"ptz": HealthState.FAIL})
    assert not d.allowed
    assert d.code == "E_UNHEALTHY"
    # Explicitly NOT E_CAPABILITY.
    assert d.code != "E_CAPABILITY"


def test_clock_fail_disables_time_window_rules():
    """RE-3a via restrict matrix."""
    assert check_time_window_rules_active({"clock": HealthState.FAIL}) is False
    assert check_time_window_rules_active({"clock": HealthState.OK}) is True


# --- three_stops single-branch handler ---

class _FakeArb:
    def __init__(self):
        self.calls = []

    def arb_suspend(self, reason, cmd_id, now_mono_ms):
        self.calls.append(("suspend", reason, cmd_id, now_mono_ms))

    def arb_rearm(self, now_mono_ms):
        self.calls.append(("rearm", now_mono_ms))


def _run_stop(reason: StopReason):
    arb = _FakeArb()
    strobe = ForceStrobeState()
    events = []
    apply_stop(
        StopEvent(reason=reason, cmd_id="c1", now_mono_ms=0),
        domain1_arbiter=arb,
        strobe_state=strobe,
        emit_event=lambda e: events.append(e),
    )
    return arb, strobe, events


def test_all_three_stops_use_same_handler_branch():
    """BIZ-P2-21: three-stop branch count == 1. Verified by asserting
    each of the three reasons produces the SAME shape of side-effects,
    differing only in event.detail.reason."""
    for reason in StopReason:
        arb, strobe, events = _run_stop(reason)
        # Same arbiter call: arb_suspend with reason=<value>, cmd_id=c1.
        assert arb.calls == [("suspend", reason.value, "c1", 0)]
        # Same strobe force ON.
        assert strobe.active is True
        # Same event kind; detail.reason differs.
        assert events == [{
            "kind": "estop",
            "detail": {"reason": reason.value, "cmd_id": "c1"},
        }]


def test_rearm_clears_force_strobe_and_calls_arbiter():
    arb = _FakeArb()
    strobe = ForceStrobeState(active=True)
    events = []
    apply_rearm(
        cmd_id="new_cmd", now_mono_ms=100,
        domain1_arbiter=arb, strobe_state=strobe,
        emit_event=lambda e: events.append(e),
    )
    assert arb.calls == [("rearm", 100)]
    assert strobe.active is False
    assert events[0]["kind"] == "estop_rearm"
