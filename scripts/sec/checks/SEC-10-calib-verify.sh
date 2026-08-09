#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-10-calib-verify.sh
# Brief: SEC checklist item SEC-10 -- calib_verify --rid must pass
#
# Description:
# Runs external calibration verifier against this robot rid;
# requires the rig + real chassis.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

_emit() {
    # $1=status  $2=message  $3=severity (default BLOCKING)
    local status="$1"
    local msg="$2"
    local sev="${3:-BLOCKING}"
    python3 - "$status" "$msg" "$sev" << 'PYEOF'
import json, sys
print(json.dumps({
    "id": "SEC-10",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

if [[ -z "${XBRAIN_ROBOT_ID:-}" ]]; then
    _emit SKIPPED "no XBRAIN_ROBOT_ID; calib_verify cannot run"
    exit 0
fi
if ! command -v calib_verify >/dev/null 2>&1; then
    _emit SKIPPED "calib_verify binary not present"
    exit 0
fi
_emit FAIL "SEC-10 not yet implemented (need calib_verify wrapper)"
exit 0
