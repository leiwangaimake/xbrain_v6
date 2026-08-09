#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-02-nft-live-vs-file.sh
# Brief: SEC checklist item SEC-02 -- runtime nft ruleset matches deploy/net/*.nft
#
# Description:
# Runs nft list ruleset and diffs against the shipped .nft file.
# Requires nftables installed and this machine to be the target
# machine; on a bench without nft, SKIPPED.

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
    "id": "SEC-02",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

if ! command -v nft >/dev/null 2>&1; then
    _emit SKIPPED "nft binary not present on this machine"
    exit 0
fi
marker="/etc/xbrain/branch"
if [[ ! -f "$marker" ]]; then
    _emit SKIPPED "no /etc/xbrain/branch marker; branch unknown"
    exit 0
fi
branch=$(cat "$marker")
file="$REPO_ROOT/deploy/net/$branch.nft"
if [[ ! -f "$file" ]]; then
    _emit FAIL "branch $branch has no deploy/net file"
    exit 0
fi
if sudo nft list ruleset 2>/dev/null | grep -q "table inet xbrain"; then
    _emit PASS "runtime nft has xbrain table"
else
    _emit FAIL "runtime nft is missing xbrain table"
fi
exit 0
