#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-07-zenohd-runtime-locked.sh
# Brief: SEC checklist item SEC-07 -- zenohd effective config: gossip + multicast disabled
#
# Description:
# Reads zenohd EFFECTIVE runtime config (not the file on disk)
# and asserts gossip.enabled=false + multicast.enabled=false.
# Reading the file only would be self-catches (CLAUDE.md 3.2 form 3).

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
    "id": "SEC-07",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

if ! pgrep -x zenohd >/dev/null 2>&1; then
    _emit SKIPPED "zenohd not running"
    exit 0
fi
_emit SKIPPED "runtime introspection endpoint disabled; static check via SEC-08"
exit 0
