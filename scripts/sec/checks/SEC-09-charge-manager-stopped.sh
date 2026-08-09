#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-09-charge-manager-stopped.sh
# Brief: SEC checklist item SEC-09 -- charge_manager must be stopped (WARNING only)
#
# Description:
# charge_manager can preempt /NAV_CMD if running. WARNING-level
# because the state clears on next boot; not blocking.

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
    "id": "SEC-09",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

if ! command -v systemctl >/dev/null 2>&1; then
    _emit SKIPPED "systemctl not present" WARNING
    exit 0
fi
if systemctl is-active --quiet charge_manager 2>/dev/null; then
    _emit FAIL "charge_manager is active (may preempt /NAV_CMD)" WARNING
else
    _emit PASS "charge_manager not active" WARNING
fi
exit 0
