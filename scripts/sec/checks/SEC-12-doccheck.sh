#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-12-doccheck.sh
# Brief: SEC checklist item SEC-12 -- doccheck: 3 items pass
#
# Description:
# Runs scripts/doccheck/ suite. Missing runner = SKIPPED.

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
    "id": "SEC-12",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

dc="$REPO_ROOT/scripts/doccheck"
if [[ ! -d "$dc" ]]; then
    _emit SKIPPED "scripts/doccheck not present"
    exit 0
fi
if [[ -x "$dc/run_all.sh" ]]; then
    if bash "$dc/run_all.sh" >/dev/null 2>&1; then
        _emit PASS "doccheck run_all.sh green"
    else
        _emit FAIL "doccheck run_all.sh non-zero"
    fi
else
    _emit SKIPPED "doccheck run_all.sh not present"
fi
exit 0
