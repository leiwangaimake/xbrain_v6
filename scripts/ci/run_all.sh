#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: run_all.sh
# Brief: INF-CI-5 -- CI master runner (calls every check in checks.yaml)
#
# Description:
# Runs every check registered in scripts/ci/checks.yaml, one at a time.
# Any failure sets FINAL_RC to 1 but the runner keeps going (so a run
# shows every failure at once, not just the first). No `|| true` --
# INF-CI-5 variant ① uses that pattern as the mutation.
#
# Emits a timestamped JSON report to $XBRAIN_CI_REPORT (default:
# /tmp/xbrain_ci_report.json). The report contains per-check name +
# path + rc + duration_ms so a downstream dashboard can trend.
#
# The registry file is the source of truth; if this script and the
# registry drift, tests/common/test_ci_registry.py fails on the next
# run (bidirectional diff).
#

set -euo pipefail

# Repo root derived from this script's own location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

REPORT_PATH="${XBRAIN_CI_REPORT:-/tmp/xbrain_ci_report.json}"
FINAL_RC=0

# Timestamp for the report. Uses date -u so a report from any host is
# unambiguous.
STARTED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

# Report accumulator: bash string, converted to JSON at end.
declare -a REPORT_ROWS=()

# One check invocation. Args: name, path, [extra args...].
run_check() {
    local name="$1"; shift
    local path="$1"; shift
    local start_s end_s ms rc

    printf '==> [%s] %s\n' "$name" "$path"
    start_s=$(date +%s%N)
    if [[ "$path" == "pytest" ]]; then
        # Special path: run pytest under the repo root with its args.
        # Set +e locally so we do NOT let pytest's non-zero abort the
        # runner via set -e; we track rc explicitly and propagate at
        # the end. This is the ONLY place we relax set -e.
        set +e
        (cd "$REPO_ROOT" && python3 -m pytest "$@" --tb=no -q)
        rc=$?
        set -e
    else
        set +e
        (cd "$REPO_ROOT" && python3 "$path" "$@")
        rc=$?
        set -e
    fi
    end_s=$(date +%s%N)
    ms=$(( (end_s - start_s) / 1000000 ))
    printf '    -> rc=%d duration=%dms\n' "$rc" "$ms"

    if [[ $rc -ne 0 ]]; then
        FINAL_RC=1
    fi
    REPORT_ROWS+=("{\"name\":\"$name\",\"path\":\"$path\",\"rc\":$rc,\"duration_ms\":$ms}")
}

# --- Checks ---------------------------------------------------------
# Order matches scripts/ci/checks.yaml. Adding a check means adding it
# BOTH here AND in checks.yaml -- the meta test catches drift.

run_check static_rules       scripts/ci/static_rules.py
run_check layout_gate        scripts/ci/layout_gate.py
run_check header_lint        scripts/lint/header_lint.py
run_check comment_ratio      scripts/lint/comment_ratio.py
run_check no_chinese_in_log  scripts/lint/no_chinese_in_log.py
run_check map1_scan          scripts/doccheck/map1_scan.py
run_check null_guard         scripts/ci/null_guard.py
run_check pytest_common      pytest tests/common
run_check pytest_boot_freeze pytest tests/boot/freeze
run_check pytest_configs     pytest tests/configs

# --- Report ---------------------------------------------------------
ROWS_JOINED=$(IFS=,; echo "${REPORT_ROWS[*]}")
FINISHED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
cat > "$REPORT_PATH" <<JSON
{"started_at":"$STARTED_AT","finished_at":"$FINISHED_AT","final_rc":$FINAL_RC,"checks":[$ROWS_JOINED]}
JSON
printf 'report written to %s\n' "$REPORT_PATH"

printf 'final_rc=%d\n' "$FINAL_RC"
exit "$FINAL_RC"
