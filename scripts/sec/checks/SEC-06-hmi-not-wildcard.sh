#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-06-hmi-not-wildcard.sh
# Brief: SEC checklist item SEC-06 -- HMI listener must not bind 0.0.0.0
#
# Description:
# ss -ltnp lookup for p5_gateway HMI port. Requires ss + running
# p5_gateway service.

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
    "id": "SEC-06",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

if ! command -v ss >/dev/null 2>&1; then
    _emit SKIPPED "ss binary not present"
    exit 0
fi
if ! ss -ltnp 2>/dev/null | grep -q p5_gateway; then
    _emit SKIPPED "p5_gateway not running; nothing to check"
    exit 0
fi
if ss -ltnp 2>/dev/null | grep p5_gateway | grep -q "0\.0\.0\.0:"; then
    _emit FAIL "p5_gateway HMI bound to 0.0.0.0 (SEC-06 violation)"
else
    _emit PASS "p5_gateway HMI not on 0.0.0.0"
fi
exit 0
