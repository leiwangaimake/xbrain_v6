#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: run_sec.sh
# Brief: CFG-BT-21 / INF-DB-5 -- SEC-1..SEC-12 delivery security check
#
# Description:
# Runs the twelve SEC checks in order and produces a JSON report at
# {out_dir}/sec-{robot_id}-{ts}.json. Exit code non-zero iff any
# blocking (BLOCKING) check failed. WARNING-level checks (SEC-9)
# never block; they surface in the report only.
#
# Two-level severity is intentional. SEC-9 warns that the charge
# manager slipped into a state that could preempt /NAV_CMD, but the
# state is transient and cleared on next boot -- blocking delivery on
# it would produce false-positive rejects. Every other SEC is a hard
# gate. The report distinguishes them so an operator does not think
# "SEC-9 warn" means "everything passed".
#
# Off-device stubs. SEC-1/3/4/9/10 need real hardware (PTZ ball,
# cloud-side scan host, RTSP source, chassis, calibration rig). When
# invoked on a bench without those, they emit SKIPPED with reason
# "requires device"; SKIPPED is NEITHER pass nor fail -- it's a
# distinct third state that must also be logged. A test that treats
# SKIPPED as pass is 3.2 form 3 self-catches waiting to happen.
#
# Each check is a separate script under scripts/sec/checks/ so they
# can be maintained, tested, and run individually.
#
# Usage:
#   bash scripts/sec/run_sec.sh
#   XBRAIN_SEC_OUT_DIR=/tmp/sec bash scripts/sec/run_sec.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CHECKS_DIR="$SCRIPT_DIR/checks"

OUT_DIR="${XBRAIN_SEC_OUT_DIR:-$REPO_ROOT/data/sec}"
mkdir -p "$OUT_DIR"

# Robot id + timestamp (wall clock intentional for filename).
ROBOT_ID="${XBRAIN_ROBOT_ID:-$(hostname)}"
TS="$(date +%s)"
REPORT="$OUT_DIR/sec-$ROBOT_ID-$TS.json"

# Aggregate results into a JSON array via a Python helper -- shell
# builders drift on quoting the moment a check outputs unicode.
python3 "$SCRIPT_DIR/aggregate.py" \
    --checks-dir "$CHECKS_DIR" \
    --out "$REPORT" \
    "$@"

status=$?
echo "wrote $REPORT (aggregate exit=$status)"
exit $status
