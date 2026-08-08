"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: _alias_table.py
Brief: §5.4.5 alias blacklist + reverse-entry guard list

Description:
Assertion B (no-duplicates) rejects private keys whose NAME matches
any entry in the 10 S5.4.5 alias table. That table lists names an L6
file might use for a quantity that is already shared in common.*; if
either name lands in a process config, the two names drift apart the
first time one gets updated.

BLACKLIST contents (one row per name, cited row from §5.4.5):
  common.fence.soft_margin_min_m       -> margin_soft_m / soft_margin_m
                                          / fence_margin_m
  common.recording.min_dist_m          -> dedup_min_dist_m /
                                          point_min_dist_m /
                                          sample_min_dist_m
  common.recording.session_timeout_s   -> max_duration_s
  common.db.record_db                  -> db_path / event.db_path /
                                          events_db
  common.geo.enu_origin                -> origin / enu_ref /
                                          datum_origin
  common.motion.profiles               -> profiles / profile_table /
                                          speed_profiles
  common.cmdset.intents_file           -> keyword_rules / intents_path

REVERSE_ENTRIES: names that LOOK like they should be on the blacklist
but must NOT be. These are names that legitimately co-exist with a
common.* key at the SAME value but come from a DIFFERENT source (§5.4.5
末段 "反向条目"). A common trap: g_person_dist_m equals patrol.require_
sense_m but the two are NOT aliases -- they measure different things
that happen to match today.

The meta-test in test_b_no_duplicates asserts BLACKLIST is disjoint
from REVERSE_ENTRIES. Variant 4 (CFG-FZ-4 verbatim): 'put g_person_
dist_m on the blacklist' -> meta-test must go red.
"""

from typing import FrozenSet

# One-row-per-name; leaves the citation as a comment so a future editor
# of 10 S5.4.5 knows which section obligated the row. Alphabetical.
BLACKLIST: FrozenSet[str] = frozenset({
    # 10 S5.4.5 recording row: three names for min_dist_m.
    "dedup_min_dist_m",
    "point_min_dist_m",
    "sample_min_dist_m",
    # 10 S5.4.5 recording row: session_timeout_s alias.
    "max_duration_s",
    # 10 S5.4.5 fence row: three names for soft_margin_min_m.
    "margin_soft_m",
    "soft_margin_m",
    "fence_margin_m",
    # 10 S5.4.5 db row: db_path / events_db (event.db_path is a dotted
    # path, matched separately by check_l6's dotted-name arm).
    "db_path",
    "events_db",
    # 10 S5.4.5 geo row: three names for enu_origin.
    "origin",
    "enu_ref",
    "datum_origin",
    # 10 S5.4.5 motion.profiles row: three names for the profile table.
    "profiles",
    "profile_table",
    "speed_profiles",
    # 10 S5.4.5 cmdset row: two names for intents_file.
    "keyword_rules",
    "intents_path",
})

# NAMES that look like aliases but are legitimately separate.
# Adding a name here documents "we deliberately did not add this to
# BLACKLIST"; the meta-test uses this to catch a future editor who
# adds one of these names to BLACKLIST by mistake.
REVERSE_ENTRIES: FrozenSet[str] = frozenset({
    # 10 S5.4.5 末表: g_person_dist_m equals patrol.require_sense_m
    # by value today, but different physical meaning -- keep separate.
    # Variant 4 of CFG-FZ-4 tries to add this to BLACKLIST; meta-test
    # catches that mistake.
    "g_person_dist_m",
})
