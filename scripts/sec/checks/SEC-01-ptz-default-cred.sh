#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-01-ptz-default-cred.sh
# Brief: SEC checklist item SEC-01 -- PTZ ball default password must FAIL to login
#
# Description:
# Requires a live PTZ ball on the LAN. Sends a login attempt
# with the factory-default password; PASS iff the login is
# rejected. On a bench without the ball, SKIPPED.

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
    "id": "SEC-01",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

if [[ -z "${XBRAIN_PTZ_IP:-}" ]]; then
    _emit SKIPPED "no XBRAIN_PTZ_IP set; PTZ ball not on this bench"
    exit 0
fi
_emit FAIL "SEC-01 not yet implemented on-device (probe stub); requires PTZ"
exit 0
