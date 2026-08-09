#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-04-rtsp-no-cred.sh
# Brief: SEC checklist item SEC-04 -- RTSP pull without credentials must 401/403
#
# Description:
# ffprobe against RTSP URL with no credentials; PASS iff 401/403.
# Requires an RTSP source live on the LAN.

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
    "id": "SEC-04",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

if [[ -z "${XBRAIN_RTSP_URL:-}" ]]; then
    _emit SKIPPED "no XBRAIN_RTSP_URL; RTSP source not present"
    exit 0
fi
_emit FAIL "SEC-04 not yet implemented (needs ffprobe wrapper)"
exit 0
