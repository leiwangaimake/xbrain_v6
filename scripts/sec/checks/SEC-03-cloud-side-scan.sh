#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-03-cloud-side-scan.sh
# Brief: SEC checklist item SEC-03 -- device management ports must be unreachable from cloud
#
# Description:
# Runs an nmap-style probe from a cloud-side scanner box against
# the deployed robot. Requires a scanner host + inbound path.

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
    "id": "SEC-03",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

if [[ -z "${XBRAIN_CLOUD_SCAN_HOST:-}" ]]; then
    _emit SKIPPED "no XBRAIN_CLOUD_SCAN_HOST; cloud-side scan not runnable"
    exit 0
fi
_emit FAIL "SEC-03 not yet implemented (needs cloud-side runner)"
exit 0
