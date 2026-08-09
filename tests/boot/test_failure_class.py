"""CFG-BT-14 startup failure classifier tests."""

import pytest

from xbrain.boot.failure_class import (
    CLASSES, CLASS_B, CLASS_D, CLASS_R, CLASS_T,
    all_ids, all_rows, classify,
    is_reject, requires_hmi_marker, requires_upgrade,
)


def test_all_29_ids_present():
    """Doc §3.3.6 lists 29 rows (1..29 + sub-rows 3b/7b/7c/7d/7e/7f/7g/7h/7i)."""
    ids = set(all_ids())
    # Numbered rows 1..29
    for i in range(1, 30):
        assert str(i) in ids, "missing item %d" % i
    # Sub-rows added in 2026-08-05 S28 + earlier
    for sub in ("3b", "7b", "7c", "7d", "7e", "7f", "7g", "7h", "7i"):
        assert sub in ids, "missing sub-row %s" % sub


def test_classify_returns_row():
    row = classify("1")
    assert row.cls == CLASS_R
    assert row.ecode == "E_STORAGE_CORRUPT"
    assert "S3.3.6" in row.ref


def test_classify_unknown_id_raises_key_error():
    with pytest.raises(KeyError, match="not registered"):
        classify("999")


def test_every_row_has_valid_class():
    for row in all_rows():
        assert row.cls in CLASSES, "%s cls %r not in CLASSES" % (row.id, row.cls)


def test_every_r_row_has_ecode():
    """R rows MUST carry an ecode -- reject with no code is a defect."""
    for row in all_rows():
        if row.cls == CLASS_R:
            assert row.ecode is not None, "R row %s has no ecode" % row.id


def test_every_t_row_has_upgrade_target():
    """BOOT-I3: T MUST have an upgrade path; else infinite retry.
    Doc allows T upgrade to R, B, or D (10 S3.3.6 shows T->R, T->B,
    T->D each in different rows)."""
    for row in all_rows():
        if row.cls == CLASS_T:
            assert row.upgrade_to in (CLASS_R, CLASS_B, CLASS_D), \
                "T row %s upgrade_to=%r invalid" % (row.id, row.upgrade_to)


def test_non_t_rows_have_no_upgrade():
    for row in all_rows():
        if row.cls != CLASS_T:
            assert row.upgrade_to is None, \
                "%s row %s should not have upgrade_to" % (row.cls, row.id)


def test_is_reject_true_only_for_r():
    assert is_reject(CLASS_R) is True
    for cls in (CLASS_B, CLASS_D, CLASS_T):
        assert is_reject(cls) is False


def test_requires_upgrade_true_only_for_t():
    assert requires_upgrade(CLASS_T) is True
    for cls in (CLASS_R, CLASS_B, CLASS_D):
        assert requires_upgrade(cls) is False


def test_requires_hmi_marker_true_only_for_d():
    assert requires_hmi_marker(CLASS_D) is True
    for cls in (CLASS_R, CLASS_B, CLASS_T):
        assert requires_hmi_marker(cls) is False


# Named CFG-BT-14 variants
def test_variant_1_r_row_missing_ecode_would_fail():
    """CFG-BT-14 variant ①: R that doesn't forbid motion = defect.
    Encoded here as 'R must have ecode' -- an R without an ecode
    signals no downstream can distinguish it from silent-fail."""
    for row in all_rows():
        if row.cls == CLASS_R:
            assert row.ecode is not None


def test_variant_2_t_infinite_retry_forbidden():
    """CFG-BT-14 variant ②: T with no upper bound = red.
    Encoded here: every T MUST have upgrade_to set."""
    ts = [r for r in all_rows() if r.cls == CLASS_T]
    assert ts, "expected at least one T-class row"
    for row in ts:
        assert row.upgrade_to is not None


def test_variant_3_d_persistent_marker_flag():
    """CFG-BT-14 variant ③: D without warn-event + HMI marker = red.
    Encoded here: requires_hmi_marker distinguishes D from others."""
    d_rows = [r for r in all_rows() if r.cls == CLASS_D]
    assert d_rows
    for row in d_rows:
        assert requires_hmi_marker(row.cls) is True


def test_all_ecodes_in_group_l_set():
    """Every ecode carried by the table is in the ALL_CODES set."""
    from xbrain.common.errors import ALL_CODES
    known = set(ALL_CODES)
    for row in all_rows():
        if row.ecode is not None:
            assert row.ecode in known, \
                "%s ecode %s not in ALL_CODES" % (row.id, row.ecode)


def test_class_distribution_reasonable():
    """Sanity: R dominates (safety-first), plus D + T present."""
    from collections import Counter
    counts = Counter(r.cls for r in all_rows())
    assert counts[CLASS_R] >= 15
    assert counts[CLASS_D] >= 5
    assert counts[CLASS_T] >= 2


def test_ids_are_unique():
    ids = [r.id for r in all_rows()]
    assert len(ids) == len(set(ids))
