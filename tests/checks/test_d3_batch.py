"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_d3_batch.py
Brief: D-3 batch CHK-2-08/35/51 + INF-QD-2 + INF-ZN-9 tests

Description:
PTZ drift auto-home three-triggers + speed tiers; D01/D10 few-shot
expansion + D10 mute-vs-restore; scan-surface meta-gate (SCAN
_SURFACE + self-exclusion + docs-never-included); constraint-id
triple addresser; cross-plane forwarding compliance (WL-G2 +
envelope rebuild + CRL-3).
"""

from __future__ import annotations

import pytest

from xbrain.common.checks.scan_surface import (
    SCAN_SURFACE as META_SCAN_SURFACE,
    ScanSurfaceViolation, ScriptSurface,
    audit_scan_scripts, check_docs_never_included,
    check_self_excluded, load_script_surface,
)
from xbrain.common.docref.addresser import (
    BOOK_ID_RE, ConstraintId, ConstraintIdShapeError,
    constraint_cover_diff, require_triple,
)
from xbrain.common.zenoh.cross_plane_compliance import (
    CrlYamlReadForbidden, FORBIDDEN_KEYS, ForwardingEntry,
    RelayEnvelope, WhitelistViolation,
    assert_no_forbidden_keys, assert_relay_source_uses_compile_time_constant,
    assert_unique_direction, check_envelope_rebuilt,
)
from xbrain.p2_core.ptz.drift_home import (
    DriftHomeConfig, DriftHomeConfigError, DriftHomeState,
    DriftHomeTrigger, PtzSpeedTierConfigError, PtzSpeedTiers,
    check_t_drift_trigger, note_home_fired,
)
from xbrain.p4_agent.intents_expand.d01_d10 import (
    D10ClassificationError, D10_MUTE_LEVEL,
    D_EXPANSION_TABLE, bidirectional_diff_vs_yaml,
    classify_d10, resolve_d10_level,
)


pytestmark = pytest.mark.no_device


# ---------- CHK-2-08 PTZ drift-home ----------

def test_drift_config_zero_refused():
    with pytest.raises(DriftHomeConfigError, match="no auto-home"):
        DriftHomeConfig(t_drift_s=0.0)


def test_drift_config_negative_refused():
    with pytest.raises(DriftHomeConfigError):
        DriftHomeConfig(t_drift_s=-1.0)


def test_drift_state_starts_at_zero():
    s = DriftHomeState()
    assert s.accumulated_mono_ms == 0


def test_drift_accumulation_monotonic():
    """T-2 discipline: accumulation uses MONOTONIC clock."""
    s = DriftHomeState()
    s.on_tracking_tick(now_mono_ms=1000)
    s.on_tracking_tick(now_mono_ms=1100)
    s.on_tracking_tick(now_mono_ms=1250)
    # First tick sets baseline; deltas are 100 + 150 = 250 ms.
    assert s.accumulated_mono_ms == 250


def test_drift_t_trigger_at_threshold():
    cfg = DriftHomeConfig(t_drift_s=1.0)   # 1s
    s = DriftHomeState(accumulated_mono_ms=1000)
    assert check_t_drift_trigger(s, cfg) is True


def test_drift_t_trigger_below_threshold():
    cfg = DriftHomeConfig(t_drift_s=1.0)
    s = DriftHomeState(accumulated_mono_ms=999)
    assert check_t_drift_trigger(s, cfg) is False


def test_drift_note_home_resets_accumulator():
    s = DriftHomeState(accumulated_mono_ms=5000)
    note_home_fired(s)
    assert s.accumulated_mono_ms == 0 and s.homed_times == 1


def test_drift_trigger_enum_three_values():
    """T-1 mode_exit, T-2 t_drift, T-3 operator_zero -- three
    distinct triggers named by enum so a future removal is caught."""
    values = {t.value for t in DriftHomeTrigger}
    assert values == {"mode_exit", "t_drift_accumulated",
                        "operator_zero"}


def test_ptz_speed_tiers_zero_refused():
    with pytest.raises(PtzSpeedTierConfigError):
        PtzSpeedTiers(speed_coarse=0.0, speed_fine=0.0)


def test_ptz_speed_tiers_equal_refused():
    """Variant C guard: collapsing coarse == fine loses operator's
    fine-tune ability."""
    with pytest.raises(PtzSpeedTierConfigError, match="collapsed"):
        PtzSpeedTiers(speed_coarse=1.0, speed_fine=1.0)


def test_ptz_speed_tiers_ordering_enforced():
    """fine must be < coarse (tier semantic)."""
    with pytest.raises(PtzSpeedTierConfigError, match="ordering"):
        PtzSpeedTiers(speed_coarse=0.5, speed_fine=1.0)


def test_ptz_speed_tiers_valid_ok():
    t = PtzSpeedTiers(speed_coarse=1.0, speed_fine=0.3)
    assert t.speed_coarse == 1.0 and t.speed_fine == 0.3


# ---------- CHK-2-35 D01/D06/D10 few-shot ----------

def test_d10_mute_level_is_zero():
    """D10 hard branch: '静音' -> level == 0."""
    assert D10_MUTE_LEVEL == 0
    kind = classify_d10("静音")
    assert resolve_d10_level(kind, resolved_restore_level=25) == 0


def test_d10_bie_chusheng_also_mute():
    kind = classify_d10("别出声")
    assert kind == "mute"
    assert resolve_d10_level(kind, resolved_restore_level=25) == 0


def test_d10_restore_uses_resolved_value():
    """Variant c guard: no code default; the tier value comes from
    resolved products."""
    kind = classify_d10("音量恢复正常")
    assert kind == "restore"
    assert resolve_d10_level(kind, resolved_restore_level=25) == 25


def test_d10_restore_non_int_refused():
    """resolved product must supply an integer level."""
    with pytest.raises(D10ClassificationError):
        resolve_d10_level("restore", resolved_restore_level=None)


def test_d10_mute_level_folded_to_1_would_variant_b_red():
    """Regression guard: if someone changes D10_MUTE_LEVEL to 1,
    the test would notice."""
    assert D10_MUTE_LEVEL != 1


def test_d10_unknown_returns_kind_unknown():
    """Utterances outside D10's set don't classify; caller falls
    through to D06 relative adjust."""
    assert classify_d10("音量调大") == "unknown"


def test_expansion_yaml_diff_empty_when_synced():
    """Meta-check: bidirectional diff empty when yaml matches
    D_EXPANSION_TABLE exactly."""
    fake_yaml = {k: list(v) for k, v in D_EXPANSION_TABLE.items()}
    assert bidirectional_diff_vs_yaml(fake_yaml) == {}


def test_expansion_yaml_diff_reddens_when_yaml_misses_entry():
    """Variant a guard: yaml missing D02 while D_EXPANSION_TABLE
    has D02 -> diff reports 'expansion_only'."""
    fake_yaml = {k: list(v) for k, v in D_EXPANSION_TABLE.items()}
    fake_yaml["D02"] = []       # yaml stripped
    d = bidirectional_diff_vs_yaml(fake_yaml)
    assert "D02" in d
    assert len(d["D02"]["expansion_only"]) > 0


def test_expansion_covers_five_intents():
    assert set(D_EXPANSION_TABLE) == {"D01", "D02", "D06", "D07", "D10"}


# ---------- CHK-2-51 scan-surface meta-gate ----------

def test_meta_scan_surface_has_required_keys():
    for k in ("include", "exclude", "extensions"):
        assert k in META_SCAN_SURFACE


def test_meta_scan_surface_excludes_self():
    """This module MUST exclude itself from its SCAN_SURFACE
    (form 3 self-injury guard)."""
    assert any("scan_surface.py" in x for x in META_SCAN_SURFACE["exclude"])


def test_meta_scan_surface_excludes_docs():
    assert "docs" in META_SCAN_SURFACE["exclude"]


def test_load_script_surface_missing_raises():
    with pytest.raises(ScanSurfaceViolation, match="does not export"):
        load_script_surface("scripts/lint/foo.py", {})


def test_load_script_surface_wrong_type_raises():
    with pytest.raises(ScanSurfaceViolation, match="must be a dict"):
        load_script_surface("scripts/lint/foo.py",
                              {"SCAN_SURFACE": "a string"})


def test_load_script_surface_missing_key_raises():
    with pytest.raises(ScanSurfaceViolation, match="missing key"):
        load_script_surface("scripts/lint/foo.py",
                              {"SCAN_SURFACE": {"include": ()}})


def test_check_self_excluded_ok_when_listed():
    s = ScriptSurface(path="scripts/lint/foo.py",
                        include=("xbrain",),
                        exclude=("scripts/lint/foo.py",),
                        extensions=(".py",))
    check_self_excluded(s)


def test_check_self_excluded_via_parent_dir():
    """Excluding the whole scripts/lint/ dir also counts."""
    s = ScriptSurface(path="scripts/lint/foo.py",
                        include=("xbrain",),
                        exclude=("scripts/lint/",),
                        extensions=(".py",))
    check_self_excluded(s)


def test_check_self_excluded_missing_reddens():
    """Variant b guard: removing self-exclusion is detected."""
    s = ScriptSurface(path="scripts/lint/foo.py",
                        include=("xbrain",),
                        exclude=("some/other/path",),
                        extensions=(".py",))
    with pytest.raises(ScanSurfaceViolation, match="ITSELF"):
        check_self_excluded(s)


def test_check_docs_never_included_ok():
    s = ScriptSurface(path="scripts/lint/foo.py",
                        include=("xbrain", "tests"),
                        exclude=(), extensions=(".py",))
    check_docs_never_included(s)


def test_check_docs_included_reddens():
    """Variant c guard."""
    s = ScriptSurface(path="scripts/lint/foo.py",
                        include=("docs/",),
                        exclude=(), extensions=(".py",))
    with pytest.raises(ScanSurfaceViolation, match="docs/"):
        check_docs_never_included(s)


def test_audit_multiple_scripts_first_fail_raises():
    scripts = [
        ScriptSurface(path="scripts/lint/good.py",
                        include=("xbrain",),
                        exclude=("scripts/lint/good.py",),
                        extensions=(".py",)),
        ScriptSurface(path="scripts/lint/bad.py",
                        include=("docs/",),
                        exclude=("scripts/lint/bad.py",),
                        extensions=(".py",)),
    ]
    with pytest.raises(ScanSurfaceViolation, match="docs/"):
        audit_scan_scripts(scripts)


# ---------- INF-QD-2 constraint ID triple ----------

def test_book_id_regex_covers_normal_book():
    assert BOOK_ID_RE.match("11")
    assert BOOK_ID_RE.match("18-A")
    assert BOOK_ID_RE.match("99")


def test_book_id_regex_rejects_bare_number():
    assert not BOOK_ID_RE.match("100")   # three-digit


def test_constraint_id_bare_string_refused():
    """Variant guard: bare id_local ('S-1') MUST be refused."""
    with pytest.raises(ConstraintIdShapeError, match="bare id"):
        require_triple("S-1")


def test_constraint_id_triple_ok():
    cid = require_triple(("12", "§12.1", "S-1"))
    assert cid.book == "12"
    assert cid.section == "§12.1"
    assert cid.id_local == "S-1"


def test_constraint_id_wrong_arity_refused():
    with pytest.raises(ConstraintIdShapeError, match="elements"):
        require_triple(("12", "S-1"))


def test_constraint_id_missing_section_refused():
    with pytest.raises(ConstraintIdShapeError, match="section"):
        ConstraintId(book="11", section="", id_local="S-1")


def test_constraint_id_missing_id_local_refused():
    with pytest.raises(ConstraintIdShapeError, match="id_local"):
        ConstraintId(book="11", section="§14.6", id_local="")


def test_constraint_id_bad_book_refused():
    with pytest.raises(ConstraintIdShapeError, match="book"):
        ConstraintId(book="not_a_book", section="§1", id_local="S-1")


def test_constraint_id_as_string():
    cid = ConstraintId(book="11", section="§14.6", id_local="S-1")
    assert cid.as_string() == "11 §14.6 S-1"


def test_constraint_cover_diff_matching_sets_empty():
    a = ConstraintId("11", "§14.6", "S-1")
    b = ConstraintId("11", "§14.6", "S-2")
    d = constraint_cover_diff([a, b], [a, b])
    assert d == {"spec_only": (), "impl_only": ()}


def test_constraint_cover_diff_reports_both_sides():
    a = ConstraintId("11", "§14.6", "S-1")
    b = ConstraintId("11", "§14.6", "S-2")
    c = ConstraintId("11", "§14.6", "S-3")
    d = constraint_cover_diff(spec_ids=[a, b], impl_ids=[b, c])
    assert d["spec_only"] == ("11 §14.6 S-1",)
    assert d["impl_only"] == ("11 §14.6 S-3",)


# ---------- INF-ZN-9 cross-plane compliance ----------

def test_forbidden_keys_matches_wl_g2():
    """WL-G2 spec: audio/broadcast forbidden."""
    assert "audio/broadcast" in FORBIDDEN_KEYS


def test_no_forbidden_keys_clean_ok():
    entries = [ForwardingEntry(src_key="cmd/estop", dst_key="rt/estop",
                                  direction="gen_to_rt")]
    assert_no_forbidden_keys(entries)


def test_no_forbidden_keys_audio_hit_raises():
    entries = [ForwardingEntry(src_key="audio/broadcast",
                                  dst_key="rt/audio/play",
                                  direction="gen_to_rt")]
    with pytest.raises(WhitelistViolation, match="WL-G2"):
        assert_no_forbidden_keys(entries)


def test_unique_direction_ok_when_single_dir():
    entries = [
        ForwardingEntry(src_key="cmd/estop", dst_key="rt/estop",
                          direction="gen_to_rt"),
        ForwardingEntry(src_key="state/health", dst_key="rt/health",
                          direction="rt_to_gen"),
    ]
    assert_unique_direction(entries)


def test_unique_direction_reversed_key_raises():
    entries = [
        ForwardingEntry(src_key="cmd/estop", dst_key="rt/estop",
                          direction="gen_to_rt"),
        ForwardingEntry(src_key="cmd/estop", dst_key="gen/estop",
                          direction="rt_to_gen"),
    ]
    with pytest.raises(WhitelistViolation, match="both"):
        assert_unique_direction(entries)


def test_envelope_rebuild_ok():
    env = RelayEnvelope(
        seq=1000, ts_mono_ms=5000, src="chassis_relay",
        orig_ts_mono_ms=4900, orig_src="p2_core",
        payload=b"test")
    check_envelope_rebuilt(env, producer_src="p2_core",
                            producer_ts_mono_ms=4900)


def test_envelope_not_rebuilt_src_reddens():
    """Variant guard: relay forwarding verbatim (src unchanged)."""
    env = RelayEnvelope(
        seq=1000, ts_mono_ms=5000, src="p2_core",   # NOT rebuilt
        orig_ts_mono_ms=4900, orig_src="p2_core",
        payload=b"test")
    with pytest.raises(WhitelistViolation, match="NOT rebuilt"):
        check_envelope_rebuilt(env, producer_src="p2_core",
                                producer_ts_mono_ms=4900)


def test_envelope_orig_src_lost_reddens():
    env = RelayEnvelope(
        seq=1000, ts_mono_ms=5000, src="chassis_relay",
        orig_ts_mono_ms=4900, orig_src="",   # LOST
        payload=b"test")
    with pytest.raises(WhitelistViolation, match="does not preserve"):
        check_envelope_rebuilt(env, producer_src="p2_core",
                                producer_ts_mono_ms=4900)


def test_relay_source_yaml_read_refused():
    """CRL-3 guard: chassis_relay MUST NOT read yaml/config."""
    with pytest.raises(CrlYamlReadForbidden, match="config-read"):
        assert_relay_source_uses_compile_time_constant(
            "whitelist = yaml.safe_load(open('whitelist.yaml'))")


def test_relay_source_json_read_refused():
    with pytest.raises(CrlYamlReadForbidden):
        assert_relay_source_uses_compile_time_constant(
            "config = json.load(open('rules.json'))")


def test_relay_source_compile_time_constant_ok():
    """Clean source with a static constant array passes."""
    src = ("const WHITELIST = [\n"
           "  {'src':'cmd/estop','dst':'rt/estop'},\n"
           "];")
    assert_relay_source_uses_compile_time_constant(src)
