#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-08-v5-bridge-deleted.sh
# Brief: SEC checklist item SEC-08 -- V5 legacy zenoh_bridge.json5 must be absent
#
# Description:
# GATE-5 verbatim. Runs anywhere the repo is present.

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
    "id": "SEC-08",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

legacy="$REPO_ROOT/ros2_ws/bridge/config/zenoh_bridge.json5"
if [[ -f "$legacy" ]]; then
    _emit FAIL "V5 zenoh_bridge.json5 found at $legacy (GATE-5 violation)"
else
    _emit PASS "no V5 legacy zenoh_bridge.json5"
fi
exit 0
